#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
REDUNDANCY -- does another watch sector already meet the need of this one?

Sector A meets sector B's need in a direction when, with the waveform plan the optimiser chooses for A (same code
path as run(): core.optimizer.sector_plan), A's search in that direction:
  - guarantees at least the same detection range on B's target (RCS, Swerling, migration at v_close_max) as B's own
    spec, initial search only (one detection, the model's Pd). A target closing at v can cross the detection range
    just after a look and is seen one revisit later, so a search reaching R with revisit RR guarantees R - v RR:
        R_AB - v_B rr_A >= R_B - v_B rr_B   (1 % of R_B tolerance, as the optimiser's "short of range")
    i.e. a range margin can make up for a slower revisit, and a faster revisit for a shorter reach. In the scan bin of
    the direction, R_AB = solve(A's waveform, B's target) x (A's power scale)^(1/4): A only transmits the power that
    just reaches A's own range ("no excess energy"), and range goes as P^(1/4); A's echoes are processed with B's RX
    weighting (B's rx_sll_db: the digital RX can apply both to the same dwell);
  - keeps B's range unambiguous (A's PRI, same rule as the optimiser, unless B allows ambiguity) and resolves at
    least as finely (B's res_max_m).
v_close_min_mps plays no part: the model evaluates migration at v_close_max only.

Every sector is tested against every OTHER sector that geometrically overlaps it, in both directions, regardless of
mission list order -- a sector built first is not assumed "senior": a sector calibrated for 5000 km necessarily also
sees at 2000 km, so it can make an earlier, shorter-range sector redundant just as well as the other way round (the
two directions are independent tests, need_met(A covers B) and need_met(B covers A), and either, both or neither can
hold -- e.g. the long-range one here needs no fast revisit, so no v_close penalty stands in its way, while the
short-range one is simply too slow/short to reciprocate: see REASON_RANGE). For each sector B, the part of its need
met is the union over every OTHER sector A that covers some of it; B is fully redundant above COVER_FULL of its
scanned solid angle. A partial overlap comes with a trim proposal -- B's box minus the boxes of the covering sectors
that meet its need over their whole overlap with B -- when those shapes are boxes (the difference involving a cap is
neither a box nor a cap, so no proposal then). A mutual case (A covers B AND B covers A over the same volume, e.g.
near-identical sectors) yields two independent Findings, one per sector, each proposing to trim/drop the other --
not a conflict: applying either one is a valid fix, and the other's Finding is simply recomputed away on the next
check once the mission has changed.
[D] Where A is short of its OWN range (high scan angles, flagged by the optimiser for A), A meeting B's need is
judged on the rest of the overlap: that shortfall is A's to fix. The proposal then reports the share of the removed
part where A is short, since B's need is not met there until A is fixed.

find_redundancies() also returns pair_notes: one line per tested ordered (A, B) direction (regardless of whether B
ends up counting as redundant), naming the dominant REASON_* this module's physics test found in the part of the
overlap A does not cover for B -- e.g. a long-range/slow-revisit watch sector and a short-range/fast-closing
self-defence one can geometrically overlap and never be redundant either way (REASON_RANGE never closes in the
slow-revisit direction, and the short-range side simply cannot reach the other's need). Since both directions of a
geometric overlap are tested, a pair can carry up to two notes, one per direction -- gui.prep_tab looks up both.
core.optimizer.overlaps() flags every geometric overlap on its own, independently of this module and without
planning any waveform; gui.prep_tab uses pair_notes to turn that generic "budgeted independently, check for a
duplicated requirement if not" into the actual reason(s), instead of running a second, duplicate physics check.

No Qt dependency: gui.prep_tab.RedundancyWorker runs find_redundancies() on a background thread.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import frames
from . import geometry as geo
from . import mission as md
from . import optimizer as mo
from . import radar_model as rm
from .params import P

# Values from radar_system.toml [tool.redundancy].
COVER_FULL = P.get("tool.redundancy.cover_full")        # fraction of B's scanned solid angle above which B is fully redundant
COVER_MIN = P.get("tool.redundancy.cover_min")          # partial overlaps below this fraction are not reported
RANGE_TOL = P.get("tool.redundancy.range_tol")          # same tolerance as the optimiser's "short of range"
MIN_PIECE_DEG = P.get("tool.redundancy.min_piece_deg")  # trimmed boxes thinner than this (phi or eps) are dropped

# Why a cell of an A/B overlap is NOT met by A's plan (REASON_NONE where it is). A geometric overlap
# (core.optimizer.overlaps, reported independently of this module) is not necessarily a duplicated requirement --
# e.g. a long-range/slow-revisit watch sector overlapping a short-range/fast-closing self-defence one are
# physically different missions, since REASON_RANGE's revisit-compensation term will not close for them. The
# codes let find_redundancies() name the dominant reason instead of leaving the overlap unexplained.
REASON_NONE = 0
REASON_NO_WAVEFORM = 1
REASON_RESOLUTION = 2
REASON_AMBIGUITY = 3
REASON_RANGE = 4
REASON_BLIND = 5
REASON_TEXT = {
    REASON_BLIND: "{a}'s pulses leave a blind range beyond {b}'s shortest range of interest",
    REASON_NO_WAVEFORM: "{a} has no waveform planned in that direction (outside its own reach)",
    REASON_RESOLUTION: "{a}'s range resolution is coarser than {b} requires",
    REASON_AMBIGUITY: "{a}'s waveform would leave {b}'s range ambiguous, which {b} disallows",
    REASON_RANGE: "{a}'s guaranteed range (after its own revisit rate) falls short of {b}'s requirement",
}

# every sector field the optimiser's plan, the need test or the shape depends on
PLAN_FIELDS = geo.SECTOR_GEOMETRY_FIELDS + frames.SECTOR_FRAME_FIELDS + (
    "range_km", "rr_target_s", "v_close_max_mps", "res_max_m", "range_min_km", "allow_ambiguous",
    "max_wf_per_sector", "short_tol", "freq_ghz_candidates", "spectrum_exclusions_ghz", "rx_sll_db") + rm.SECTOR_TARGET_KEYS

_PLANS: Dict[str, Optional[dict]] = {}
_PLAN_CACHE_MAX = P.get("tool.redundancy.plan_cache_max")


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
    return json.dumps({"g": [g.get("eps_min_deg"), g.get("eps_max_deg"), g.get("face_elec_mw"), g.get("sky"), g.get("nav"),
                             g.get("spectrum_exclusions_ghz")],
                       "s": [[s.get("id")] + [s.get(f) for f in PLAN_FIELDS] for s in mission.get("sectors", [])]},
                      default=str)


def sector_signature(sec: dict) -> str:
    return json.dumps([sec.get("id")] + [sec.get(f) for f in PLAN_FIELDS], default=str)


def _plan(sec: dict, eps_min: float, eps_max: float, p_avg_max_w: float, sky: Optional[dict] = None) -> Optional[dict]:
    key = json.dumps([sec.get(f) for f in PLAN_FIELDS] + [eps_min, eps_max, p_avg_max_w, sky], default=str)
    if key not in _PLANS:
        if len(_PLANS) >= _PLAN_CACHE_MAX:
            _PLANS.clear()
        _PLANS[key] = mo.sector_plan(sec, eps_min, eps_max, p_avg_max_w, sky)
    return _PLANS[key]


def guaranteed_range_km(range_km: float, rr_s: float, v_mps: float) -> float:
    """Detection range guaranteed by a search reaching range_km with revisit rr_s on a target closing at v_mps. Since
    step 2.3 the search criterion is the cumulative probability reached AT range_km over the looks spaced v x RR
    (the revisit is inside the criterion), so the guaranteed range is range_km itself (it was range_km - v RR)."""
    return range_km


def need_met(grid: geo.SphereGrid, sec_a: dict, plan: Optional[dict], sec_b: dict,
             cells: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Optional[float], np.ndarray]:
    """Subsets of `cells` (directions inside both sectors) where sector A, as planned, (1) meets sector B's need and
    (2) falls short of its own range; (3) A's worst reach on B's target where it meets the need (km, None if
    nowhere); (4) per cell, which REASON_* code explains why it is not met (REASON_NONE where it is)."""
    if plan is None:
        z = np.zeros_like(cells)
        return z, z, None, np.where(cells, REASON_NO_WAVEFORM, REASON_NONE).astype(np.int8)
    v_b = sec_b["v_close_max_mps"]
    tgt_b = mo.sector_target(sec_b)                      # B's target: class + sector values (step 2.4)
    g_need = sec_b["range_km"] * (1.0 - RANGE_TOL)          # A's cumulative reach on B's need (step 2.3)
    cum = (float(v_b), float(sec_a["rr_target_s"]), float(sec_b["range_km"]))   # A's revisit, B's target and range
    res_b = sec_b.get("res_max_m", mo.RES_MAX_M_DEFAULT)
    f = plan["freq_ghz"]
    nb = len(plan["wf"])
    theta = mo._theta_centres(f, nb)
    ok = np.zeros(nb, dtype=bool)
    reach = np.full(nb, np.inf)
    reason = np.full(nb, REASON_NO_WAVEFORM, dtype=np.int8)
    for i, w in enumerate(plan["wf"]):
        if w is None:
            continue
        if mo.resolution_m(sec_b, w.sig_bw_mhz) > res_b + 1e-9:       # physical resolution, B's sidelobes (2.1)
            reason[i] = REASON_RESOLUTION
            continue
        if (w.n > 1 and not sec_b.get("allow_ambiguous")
                and w.pri_s < rm.unambiguous_pri_s(rm.BAND_S, sec_b["range_km"], w.tau_s) - 1e-12):
            reason[i] = REASON_AMBIGUITY
            continue
        if rm.blind_range_km(rm.BAND_S, w.tx_block_s) > mo.range_min_km(sec_b) + 1e-9:
            reason[i] = REASON_BLIND
            continue
        r = mo.range_at(w, f, float(theta[i]), tgt_b, mo.rx_sll_db(sec_b), mo.velocity_span_mps(sec_b),
                        plan["pfa"][i], cum,        # A's echoes and false-alarm setting, B's RX weighting and need
                        spacing=plan.get("beam_spacing_hpbw"))           # A's lattice (step 3.1, P5)
        if r is not None:
            reach[i] = r * plan["pulse_scale"][i] ** 0.25
            ok[i] = reach[i] >= g_need
        reason[i] = REASON_NONE if ok[i] else REASON_RANGE
    bins = np.minimum((grid.theta / mo.THETA_BIN_DEG).astype(int), nb - 1)
    met = cells & ok[bins]
    worst = float(reach[np.unique(bins[met])].min()) if met.any() else None
    return met, cells & np.asarray(plan["short"])[bins], worst, np.where(cells, reason[bins], REASON_NONE)


def _dominant_reason(grid: geo.SphereGrid, reason_cells: np.ndarray, unmet: np.ndarray) -> int:
    """The REASON_* code covering the most solid angle of `unmet` (REASON_RANGE if nothing is set, e.g. an
    empty mask -- the most common case, so it is the least surprising fallback)."""
    weights = {code: float((grid.d_omega * unmet * (reason_cells == code)).sum())
              for code in (REASON_RESOLUTION, REASON_AMBIGUITY, REASON_BLIND, REASON_RANGE, REASON_NO_WAVEFORM)}
    weights = {c: w for c, w in weights.items() if w > 0.0}
    return max(weights, key=weights.get) if weights else REASON_RANGE


def _pair_note(sa_id: str, sb_id: str, om_met: float, om_inter: float, reason_cells: np.ndarray,
               unmet: np.ndarray, grid: geo.SphereGrid) -> str:
    """One line on why an A/B geometric overlap (core.optimizer.overlaps, reported independently of this module)
    is, or is not, a duplicated requirement -- the dominant REASON_* over the part still unmet, so an operator
    does not have to guess between 'checked and fine' and 'never checked'."""
    frac = om_met / om_inter if om_inter > 0.0 else 0.0
    if frac >= COVER_FULL:
        return f"{sa_id} meets {sb_id}'s need over their whole shared volume (see the redundancy alert below)"
    reason = REASON_TEXT[_dominant_reason(grid, reason_cells, unmet)].format(a=sa_id, b=sb_id)
    if frac >= COVER_MIN:
        return f"{sa_id} covers only {100 * frac:.0f} % of the shared volume for {sb_id}'s need -- {reason} elsewhere"
    return f"{reason} -- not a duplicated requirement"


def find_redundancies(mission: dict, grid: geo.SphereGrid) -> Tuple[List[Finding], Dict[Tuple[str, str], str]]:
    """Step 3.0: computed on the mission turned into the ship frame (core/frames.py); nothing if its frames are invalid."""
    try:
        mission, _ = frames.resolve(mission)
    except ValueError:
        return [], {}
    mission = mo.with_spectrum(mission)
    g = mission.get("global", {})
    eps_min, eps_max = g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0)
    p_avg_max_w = rm.face_elec_avail_w(g.get("face_elec_mw", rm.FACE_ELEC_POWER_MW))
    sectors = mission.get("sectors", [])
    scanned = grid.scanned(eps_min, eps_max)
    masks: List[Optional[np.ndarray]] = []
    for i, sec in enumerate(sectors):
        masks.append(grid.sector_mask(sec) & scanned if not md.validate_sector(sec, i) else None)
    ids = [s.get("id") for s in sectors]
    out = []
    pair_notes: Dict[Tuple[str, str], str] = {}
    for j, sb in enumerate(sectors):
        mb = masks[j]
        if mb is None or not mb.any():
            continue
        om_b = float((grid.d_omega * mb).sum())
        covered = np.zeros_like(mb)
        by, clean, short_notes, details = [], [], [], []
        v_b = sb["v_close_max_mps"]
        g_b = guaranteed_range_km(sb["range_km"], sb["rr_target_s"], v_b)
        for i, sa in enumerate(sectors):
            if i == j:
                continue
            ma = masks[i]
            if ma is None:
                continue
            inter = ma & mb
            if not inter.any():
                continue
            met, short, worst, reason_cells = need_met(grid, sa, _plan(sa, eps_min, eps_max, p_avg_max_w, g.get("sky")),
                                                       sb, inter)
            om_met = float((grid.d_omega * met).sum())
            om_inter = float((grid.d_omega * inter).sum())
            pair_notes[(sa["id"], sb["id"])] = _pair_note(sa["id"], sb["id"], om_met, om_inter, reason_cells,
                                                           inter & ~met, grid)
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
    return out, pair_notes


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
    boxes = [a for a in clean if a.get("shape", "box") == "box" and frames.frame_key(a) == frames.frame_key(sb)]
    if not boxes:
        return None, ("no earlier box sector in the same frame meets the need over its whole overlap with "
                      f"{sb['id']} (coverage depends on the scan angle, or the covering sector is a cap or in "
                      f"another frame)")
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
                pieces.append(frames.unresolved(piece))
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
