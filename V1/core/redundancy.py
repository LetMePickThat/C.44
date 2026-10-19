#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
REDUNDANCY -- does an earlier watch sector already meet the need of a later one?

Sector A meets sector B's need in a direction when, with the waveform plan the optimiser chooses for A (same code
path as run(): core.optimizer.sector_plan), A's search in that direction:
  - guarantees at least the same detection range on B's target (RCS, Swerling, migration at v_close_max) as B's own
    spec, initial search only (one detection, the model's Pd). A target closing at v can cross the detection range
    just after a look and is seen one revisit later, so a search reaching R with revisit RR guarantees R - v RR:
        R_AB - v_B rr_A >= R_B - v_B rr_B   (1 % of R_B tolerance, as the optimiser's "short of range")
    i.e. a range margin can make up for a slower revisit, and a faster revisit for a shorter reach. In the scan bin of
    the direction, R_AB = solve(A's waveform, B's target) x (A's power scale)^(1/4): A only transmits the power that
    just reaches A's own range ("no excess energy"), and range goes as P^(1/4);
  - keeps B's range unambiguous (A's PRI, same rule as the optimiser, unless B allows ambiguity) and resolves at
    least as finely (B's res_max_m).
v_close_min_mps plays no part: the model evaluates migration at v_close_max only.

List order is seniority: sector j is only tested against sectors 0..j-1. The part of B whose need is met is the union
over those sectors; B is fully redundant above COVER_FULL of its scanned solid angle. A partial overlap comes with a
trim proposal -- B's box minus the boxes of the earlier sectors that meet its need over their whole overlap with B --
when those shapes are boxes (the difference involving a cap is neither a box nor a cap, so no proposal then).
[D] Where A is short of its OWN range (high scan angles, flagged by the optimiser for A), A meeting B's need is
judged on the rest of the overlap: that shortfall is A's to fix. The proposal then reports the share of the removed
part where A is short, since B's need is not met there until A is fixed.

No Qt dependency: gui.prep_tab.RedundancyWorker runs find_redundancies() on a background thread.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import geometry as geo
from . import mission as md
from . import optimizer as mo
from . import radar_model as rm

COVER_FULL = 0.995      # [D] fraction of B's scanned solid angle above which B is fully redundant
COVER_MIN = 0.01        # [D] partial overlaps below this fraction are not reported
RANGE_TOL = 0.01        # [D] same 1 % tolerance as the optimiser's "short of range"
MIN_PIECE_DEG = 1.0     # [D] trimmed boxes thinner than this (phi or eps) are dropped

# every sector field the optimiser's plan, the need test or the shape depends on
PLAN_FIELDS = geo.SECTOR_GEOMETRY_FIELDS + (
    "rcs_m2", "swerling", "range_km", "rr_target_s", "v_close_max_mps", "res_max_m", "allow_ambiguous",
    "max_wf_per_sector", "short_tol", "freq_ghz_candidates")

_PLANS: Dict[str, Optional[dict]] = {}
_PLAN_CACHE_MAX = 256


@dataclass
class Finding:
    index: int                          # index of the redundant sector (B) in the mission
    sid: str
    by: List[str]                       # earlier sectors meeting part of B's need, mission order
    fraction: float                     # fraction of B's scanned solid angle whose need is already met
    full: bool
    pieces: Optional[List[dict]]        # trim proposal (sectors replacing B), None if none can be made
    note: str = ""                      # why there is no trim proposal
    caveat: str = ""                    # earlier sectors short of their own range in the part a trim removes
    detail: str = ""                    # per covering sector: guaranteed detection range vs B's need


def mission_signature(mission: dict) -> str:
    """Everything find_redundancies() depends on: a result is current while the signature is unchanged."""
    g = mission.get("global", {})
    return json.dumps({"g": [g.get("eps_min_deg"), g.get("eps_max_deg"), g.get("face_elec_mw")],
                       "s": [[s.get("id")] + [s.get(f) for f in PLAN_FIELDS] for s in mission.get("sectors", [])]},
                      default=str)


def sector_signature(sec: dict) -> str:
    return json.dumps([sec.get("id")] + [sec.get(f) for f in PLAN_FIELDS], default=str)


def _plan(sec: dict, eps_min: float, eps_max: float, p_avg_max_w: float) -> Optional[dict]:
    key = json.dumps([sec.get(f) for f in PLAN_FIELDS] + [eps_min, eps_max, p_avg_max_w], default=str)
    if key not in _PLANS:
        if len(_PLANS) >= _PLAN_CACHE_MAX:
            _PLANS.clear()
        _PLANS[key] = mo.sector_plan(sec, eps_min, eps_max, p_avg_max_w)
    return _PLANS[key]


def guaranteed_range_km(range_km: float, rr_s: float, v_mps: float) -> float:
    """Detection range guaranteed by a search reaching range_km with revisit rr_s on a target closing at v_mps."""
    return range_km - v_mps * rr_s / 1e3


def need_met(grid: geo.SphereGrid, sec_a: dict, plan: Optional[dict], sec_b: dict,
             cells: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Optional[float]]:
    """Subsets of `cells` (directions inside both sectors) where sector A, as planned, (1) meets sector B's need and
    (2) falls short of its own range; (3) A's worst reach on B's target where it meets the need (km, None if nowhere)."""
    if plan is None:
        return np.zeros_like(cells), np.zeros_like(cells), None
    v_b = sec_b["v_close_max_mps"]
    tgt_b = rm.Target("need", sec_b["rcs_m2"], int(sec_b.get("swerling", 1)), v_b)
    g_need = (guaranteed_range_km(sec_b["range_km"], sec_b["rr_target_s"], v_b)
              - RANGE_TOL * sec_b["range_km"] + v_b * sec_a["rr_target_s"] / 1e3)   # = reach A needs
    res_b = sec_b.get("res_max_m", mo.RES_MAX_M_DEFAULT)
    f = plan["freq_ghz"]
    nb = len(plan["wf"])
    theta = mo._theta_centres(f, nb)
    ok = np.zeros(nb, dtype=bool)
    reach = np.full(nb, np.inf)
    for i, w in enumerate(plan["wf"]):
        if w is None:
            continue
        if mo.C / (2 * w.sig_bw_mhz * 1e6) > res_b + 1e-9:
            continue
        if w.n > 1 and not sec_b.get("allow_ambiguous") and mo.C * w.pri_s / 2e3 < sec_b["range_km"]:
            continue
        r = mo.range_at(w, f, float(theta[i]), tgt_b)
        if r is not None:
            reach[i] = r * plan["power_scale"][i] ** 0.25
            ok[i] = reach[i] >= g_need
    bins = np.minimum((grid.theta / mo.THETA_BIN_DEG).astype(int), nb - 1)
    met = cells & ok[bins]
    worst = float(reach[np.unique(bins[met])].min()) if met.any() else None
    return met, cells & np.asarray(plan["short"])[bins], worst


def find_redundancies(mission: dict, grid: geo.SphereGrid) -> List[Finding]:
    g = mission.get("global", {})
    eps_min, eps_max = g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0)
    p_avg_max_w = rm.face_avg_rf_w(g.get("face_elec_mw", rm.FACE_ELEC_POWER_MW))
    sectors = mission.get("sectors", [])
    scanned = grid.scanned(eps_min, eps_max)
    masks: List[Optional[np.ndarray]] = []
    for i, sec in enumerate(sectors):
        masks.append(grid.sector_mask(sec) & scanned if not md.validate_sector(sec, i) else None)
    ids = [s.get("id") for s in sectors]
    out = []
    for j, sb in enumerate(sectors):
        mb = masks[j]
        if mb is None or not mb.any():
            continue
        om_b = float((grid.d_omega * mb).sum())
        covered = np.zeros_like(mb)
        by, clean, short_notes, details = [], [], [], []
        v_b = sb["v_close_max_mps"]
        g_b = guaranteed_range_km(sb["range_km"], sb["rr_target_s"], v_b)
        for i in range(j):
            sa, ma = sectors[i], masks[i]
            if ma is None:
                continue
            inter = ma & mb
            if not inter.any():
                continue
            met, short, worst = need_met(grid, sa, _plan(sa, eps_min, eps_max, p_avg_max_w), sb, inter)
            om_met = float((grid.d_omega * met).sum())
            if om_met <= 0.0:
                continue
            by.append(sa["id"])
            details.append(f"{sa['id']} detects {sb['id']}'s target from {worst:.0f} km (worst scan angle), revisit "
                           f"{sa['rr_target_s']:g} s: {guaranteed_range_km(worst, sa['rr_target_s'], v_b):.0f} km "
                           f"guaranteed at {v_b:.0f} m/s, vs {g_b:.0f} km for {sb['id']}'s own spec "
                           f"({sb['range_km']:.0f} km, revisit {sb['rr_target_s']:g} s)")
            covered |= met
            om_short = float((grid.d_omega * short).sum())
            if om_met >= COVER_FULL * (float((grid.d_omega * inter).sum()) - om_short):
                clean.append(sa)
                if om_short > 0.0 and sa.get("shape", "box") == "box":
                    short_notes.append(f"{sa['id']} is short of its own range ({sa['range_km']:.0f} km) on "
                                       f"{100 * om_short / om_b:.0f} % of {sb['id']}'s volume (high scan angles): "
                                       f"{sb['id']}'s need is only met there once {sa['id']} is fixed")
        frac = float((grid.d_omega * covered).sum()) / om_b
        if frac < COVER_MIN:
            continue
        full = frac >= COVER_FULL
        pieces, note = (None, "") if full else trim_proposal(sb, clean, grid, scanned, ids)
        out.append(Finding(index=j, sid=sb["id"], by=by, fraction=frac, full=full, pieces=pieces, note=note,
                           caveat="; ".join(short_notes) if pieces else "", detail="\n".join(details)))
    return out


# ----------------------------------------------------------------------------- trim proposal (box algebra)
def _phi_intervals(sec: dict) -> List[Tuple[float, float]]:
    lo, hi = sec["phi_min_deg"], sec["phi_max_deg"]
    if hi - lo >= 360.0:
        return [(0.0, 360.0)]
    p1, p2 = lo % 360.0, hi % 360.0
    return [(p1, p2)] if p1 <= p2 else [(p1, 360.0), (0.0, p2)]


def _subtract(intervals: List[Tuple[float, float]], cuts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    out = []
    for a, b in intervals:
        segs = [(a, b)]
        for c, d in cuts:
            nxt = []
            for x, y in segs:
                if d <= x or c >= y:
                    nxt.append((x, y))
                    continue
                if c > x:
                    nxt.append((x, c))
                if d < y:
                    nxt.append((d, y))
            segs = nxt
        out.extend(segs)
    return out


def _phi_boxes(intervals: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """(phi_min, phi_max) box spans, joining an interval ending at 360 with one starting at 0 into one wrapping span."""
    iv = sorted(intervals)
    if iv == [(0.0, 360.0)]:
        return iv
    if len(iv) > 1 and iv[0][0] == 0.0 and iv[-1][1] == 360.0:
        iv = [(iv[-1][0], iv[0][1])] + iv[1:-1]
    return iv


def trim_proposal(sb: dict, clean: List[dict], grid: geo.SphereGrid, scanned: np.ndarray,
                  existing_ids: List[str]) -> Tuple[Optional[List[dict]], str]:
    """B's box minus the boxes of the earlier sectors meeting B's need over their whole overlap with B. The first
    piece keeps B's id; the others get B's id with a _2, _3... suffix."""
    if sb.get("shape", "box") != "box":
        return None, f"{sb['id']} is a cap; an automatic trim needs box shapes"
    boxes = [a for a in clean if a.get("shape", "box") == "box"]
    if not boxes:
        return None, ("no earlier box sector meets the need over its whole overlap with "
                      f"{sb['id']} (coverage depends on the scan angle, or the covering sector is a cap)")
    e_lo, e_hi = sb["eps_min_deg"], sb["eps_max_deg"]
    edges = sorted({e_lo, e_hi} | {min(max(e, e_lo), e_hi) for a in boxes for e in (a["eps_min_deg"], a["eps_max_deg"])})
    bands: List[Tuple[float, float, Tuple]] = []
    for e0, e1 in zip(edges[:-1], edges[1:]):
        cuts = [iv for a in boxes if a["eps_min_deg"] <= e0 and a["eps_max_deg"] >= e1 for iv in _phi_intervals(a)]
        phi = tuple(_phi_boxes(_subtract(_phi_intervals(sb), cuts)))
        if bands and bands[-1][2] == phi and bands[-1][1] == e0:
            bands[-1] = (bands[-1][0], e1, phi)
        else:
            bands.append((e0, e1, phi))
    pieces = []
    for e0, e1, phi in bands:
        if e1 - e0 < MIN_PIECE_DEG:
            continue
        for p0, p1 in phi:
            if (p1 - p0) % 360.0 < MIN_PIECE_DEG and not (p0 == 0.0 and p1 == 360.0):
                continue
            piece = dict(sb, phi_min_deg=round(p0, 6), phi_max_deg=round(p1, 6),
                         eps_min_deg=round(e0, 6), eps_max_deg=round(e1, 6))
            if (grid.sector_mask(piece) & scanned).any():
                pieces.append(piece)
    if not pieces:
        return None, "nothing left to scan once the covered part is removed"
    taken = set(existing_ids) - {sb["id"]}
    n = 2
    for k, piece in enumerate(pieces):
        if k == 0:
            piece["id"] = sb["id"]
            continue
        while f"{sb['id']}_{n}" in taken:
            n += 1
        piece["id"] = f"{sb['id']}_{n}"
        taken.add(piece["id"])
    return pieces, ""
