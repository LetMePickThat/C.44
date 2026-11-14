#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COST_ESTIMATE -- fast, approximate per-sector radar budget for real-time feedback while the operator
is editing the mission preparation (sliders / form fields). NOT the authoritative result: the exact
numbers come from core.optimizer.run() (full waveform search), called on "Run calculation".

Backed by cost_table.json (built once by tools/gen_cost_table.py): for S band, 2.8 GHz, a grid of
(range_km, rcs_m2, v_close_mps, scan_theta_deg) -> cheapest ELECTRICAL energy (RF / PAE, step 1.3) per beam position that just reaches
that range (or None if unreachable even with the best waveform at full power).

Interpolation strategy (all in log-energy space, since energy spans orders of magnitude):
  - range       : continuous, never re-snapped (that would make the gauges jump while typing) -- log-log
                  between the two bracketing FEASIBLE tabulated ranges when the request falls inside the
                  tabulated span (the energy(R) curve is only piecewise ~R^4 across catalogue-waveform
                  changeovers), else the model's own E ~ R^4 law anchored on the one available neighbour.
  - RCS, speed  : bilinear (RCS log-spaced, speed linear) between the 4 bracketing grid points --
                  nearest-neighbour snapping was shown to under/over-shoot range capability by ~25 %+
                  near grid half-decades (e.g. RCS 3000 snapped to 1000 m^2).
  - scan angle  : linear between the 2 bracketing tabulated angles, evaluated per raster cell.

Energy is the quantity this module estimates well (validated against core.optimizer.run() on the
3 example sectors and mission_veille500: within -1 % / +4 % of the v2 optimiser, which also picks the cheapest waveform per scan angle). Time-on-air
is deliberately NOT estimated the same way: the table picks, independently at each (range, RCS, speed,
angle) cell, whichever (N, tau, Btx) combo is cheapest in ENERGY, but many combos tie on energy while
differing wildly in N*tau (time-on-air is under-determined there); the optimiser, in contrast,
commits to a single shared waveform across a whole face/sector. Cross-checking against the reference
run showed this makes a naive per-cell time estimate off by up to 27x (e.g. sector S03). Only a
mathematically honest LOWER bound (average power / peak power -- you cannot transmit in less time than
energy/peak-power) is exposed here; the exact time-on-air only comes from core.optimizer.run().

Active tracking (AT): a sector's range brackets (target count + precision) used to have zero effect on these
gauges, which could show a face as comfortably under budget while the AT load alone later over-subscribed it in
the full calculation. _estimate_at_faces() now adds one representative Ku look per face per bracket (position-
weighted mean scan angle, target count split like the search positions) via core.tracking.ku_look() -- same
waveform search as the exact budget, just without its per-scan-angle-bin grouping. Still not authoritative.
"""
from __future__ import annotations

import dataclasses
import json
import threading
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import params
from . import cost_profile  # noqa: F401  (its [tool.cost_profile] values are read at import)
from . import geometry as geo
from . import radar_model as rm
from . import tracking as trk

DEFAULT_TABLE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cost_table.json")


@dataclass
class CostTable:
    R: List[float]
    RCS: List[float]
    V: List[float]
    TH: List[float]
    ppk: float
    pavg_ref_w: float          # electrical power for transmission the table was built at (step 1.3)
    t: Dict[str, Optional[list]]
    reach: Optional[Dict[str, list]] = None   # per cell: [max range km at full power, that waveform's energy J]
    params_fingerprint: Optional[str] = None  # fingerprint of radar_system.toml the table was built from
    rx_sll_db: float = rm.RX_SLL_DEFAULT_DB   # RX weighting the table was built with (older tables: the default)

    @property
    def is_current(self) -> bool:
        """False when radar_system.toml changed since the table was built (rerun tools/gen_cost_table.py)."""
        return self.params_fingerprint == params.fingerprint(params.P)

    @staticmethod
    def _key(r_km: float, rcs_m2: float, v_mps: float, th_deg: float) -> str:
        # gen_cost_table.py builds keys from the raw RCS list (mixed int/float literals, e.g. 0.1 but
        # 1000 not 1000.0) -- reproduce that exact string, or whole-valued RCS never matches.
        rcs_key = int(rcs_m2) if float(rcs_m2).is_integer() else rcs_m2
        return f"{int(r_km)}|{rcs_key}|{int(v_mps)}|{int(th_deg)}"

    def entry(self, r_km: float, rcs_m2: float, v_mps: float, th_deg: float) -> Optional[list]:
        return self.t.get(self._key(r_km, rcs_m2, v_mps, th_deg))

    def reach_entry(self, r_km: float, rcs_m2: float, v_mps: float, th_deg: float) -> Optional[list]:
        return self.reach.get(self._key(r_km, rcs_m2, v_mps, th_deg)) if self.reach else None


def load_table(path: str = DEFAULT_TABLE_PATH) -> Optional[CostTable]:
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as fh:
        d = json.load(fh)
    return CostTable(R=d["R"], RCS=d["RCS"], V=d["V"], TH=d["TH"], ppk=d["ppk"], pavg_ref_w=d["pavg"], t=d["t"],
                     reach=d.get("reach"), params_fingerprint=d.get("params_fingerprint"),
                     rx_sll_db=d.get("rx_sll_db", rm.RX_SLL_DEFAULT_DB))


def _bracket(value: float, grid: List[float], log: bool = False) -> Tuple[int, int, float]:
    """Index of the two grid points bracketing `value` (clamped at the ends) and the interpolation
    fraction in [0, 1] (0 = exactly on the low point, 1 = exactly on the high point)."""
    arr = np.array(grid, dtype=float)
    g = np.log(np.maximum(arr, 1e-9)) if log else arr
    v = math.log(max(value, 1e-9)) if log else value
    if v <= g[0]:
        return 0, 0, 0.0
    if v >= g[-1]:
        n = len(arr) - 1
        return n, n, 0.0
    i = int(np.searchsorted(g, v)) - 1
    i = max(0, min(i, len(arr) - 2))
    span = g[i + 1] - g[i]
    frac = (v - g[i]) / span if span != 0 else 0.0
    return i, i + 1, frac


@dataclass
class PositionEnergyEstimate:
    energy_j: Optional[float]
    feasible: bool
    note: str = ""


def estimate_energy_per_position(table: CostTable, r_want_km: float, rcs_m2: float, v_mps: float,
                                  theta_deg: float) -> PositionEnergyEstimate:
    """Energy needed for one beam-position dwell to just reach r_want_km, at the given RCS / closing
    speed / scan angle, with RCS and speed snapped to the nearest tabulated value (used as a building
    block for the bilinear interpolation below -- call with exact grid values). Range is interpolated
    log-log between the two bracketing FEASIBLE tabulated ranges when r_want_km falls inside the
    tabulated span (the true energy(R) curve is only piecewise ~R^4: a different catalogue waveform can
    win on either side of a tabulated row, so anchoring on a single neighbour and extrapolating with the
    model's R^4 law overshoots in between -- cross-checked at up to ~30% per cell, worst around 700-4000 km).
    Outside the tabulated span (below the shortest feasible row, or above when no 'reach' data caps it),
    there is only one neighbour to anchor on, so the R^4 law is still used there."""
    feasible_rs = [(r, table.entry(r, rcs_m2, v_mps, theta_deg)) for r in table.R]
    feasible_rs = [(r, e) for r, e in feasible_rs if e is not None]
    # True reach of the cell (tables from gen_cost_table >= v2): at the row just above the request, so its PRI
    # constraint is at least as strict as the request's
    upper = [r for r in table.R if r >= r_want_km] or [max(table.R)]
    reach = table.reach_entry(upper[0], rcs_m2, v_mps, theta_deg)
    if reach is not None and reach[1] is not None and r_want_km > reach[0]:
        # Unreachable even with the longest waveform: cost it at that waveform's full-power energy (what the exact
        # optimiser spends on a position short of range), not an arbitrary figure.
        return PositionEnergyEstimate(reach[1], False,
                                       f"range {r_want_km:.0f} km beyond capability ({reach[0]:.0f} km at full power)")
    if not feasible_rs:
        return PositionEnergyEstimate(None, False, "no waveform covers this table cell")
    rs = sorted(r for r, _ in feasible_rs)
    e_at = {r: e[0] for r, e in feasible_rs}
    r_max = rs[-1]
    if r_want_km > r_max and reach is None:
        # Old table without reach data: the longest tabulated reach falls short, and extrapolating further with
        # the same R^4 law would understate the true cost (waveform saturates well before this).
        return PositionEnergyEstimate(None, False,
                                       f"range {r_want_km:.0f} km beyond estimated capability (tabulated max {r_max:.0f} km)")
    r_lo = max((r for r in rs if r <= r_want_km), default=None)
    r_hi = min((r for r in rs if r >= r_want_km), default=None)
    if r_lo is not None and r_hi is not None and r_lo != r_hi:
        t = (math.log(r_want_km) - math.log(r_lo)) / (math.log(r_hi) - math.log(r_lo))
        e = math.exp(math.log(e_at[r_lo]) + t * (math.log(e_at[r_hi]) - math.log(e_at[r_lo])))
    else:
        r_ref = r_lo if r_lo is not None else r_hi
        e = e_at[r_ref] * (r_want_km / r_ref) ** 4
    return PositionEnergyEstimate(e, True)


def energy_profile_by_theta(table: CostTable, r_want_km: float, rcs_m2: float,
                             v_mps: float) -> Tuple[np.ndarray, np.ndarray]:
    """log(energy per position) at each tabulated scan angle, for the exact (continuous) RCS and speed
    requested -- bilinear interpolation (log-RCS x linear-speed) across the 4 bracketing grid corners.
    Returns (log_energy[len(TH)], feasible[len(TH)])."""
    i_r0, i_r1, fr = _bracket(rcs_m2, table.RCS, log=True)
    i_v0, i_v1, fv = _bracket(v_mps, table.V, log=False)
    rcs0, rcs1 = table.RCS[i_r0], table.RCS[i_r1]
    v0, v1 = table.V[i_v0], table.V[i_v1]
    # bilinear weights of (rcs0,v0) (rcs0,v1) (rcs1,v0) (rcs1,v1), matching the corners loop order below
    weights = ((1 - fr) * (1 - fv), (1 - fr) * fv, fr * (1 - fv), fr * fv)
    log_e, feas = [], []
    for th in table.TH:
        corners, corner_ok = [], []
        for rcs_c in (rcs0, rcs1):
            for v_c in (v0, v1):
                est = estimate_energy_per_position(table, r_want_km, rcs_c, v_c, th)
                corners.append(math.log(est.energy_j) if est.energy_j else None)   # short corners: full-power energy
                corner_ok.append(est.feasible)
        vals = [c for c in corners if c is not None]
        if not vals:
            log_e.append(math.log(table.ppk))   # no information at all (old table): pessimistic placeholder
            feas.append(False)
            continue
        c00, c01, c10, c11 = (c if c is not None else max(vals) for c in corners)  # (rcs0,v0)(rcs0,v1)(rcs1,v0)(rcs1,v1)
        e_v0 = c00 * (1 - fr) + c10 * fr
        e_v1 = c01 * (1 - fr) + c11 * fr
        log_e.append(e_v0 * (1 - fv) + e_v1 * fv)
        # A corner at zero weight (the request sits exactly on the opposite grid node, e.g. the default RCS/speed)
        # must not veto feasibility: only the corners actually blended in above can.
        feas.append(all(ok for ok, wgt in zip(corner_ok, weights) if wgt > 1e-9))
    return np.array(log_e), np.array(feas)


@dataclass
class FaceBudgetEstimate:
    positions: float
    energy_j_per_refresh: float
    energy_fraction: float     # TOTAL (search + AT), since both share the face's electrical power
    avg_power_kw: float        # TOTAL (search + AT)
    time_fraction_min: float   # search only -- ~LOWER bound (RF avg power / peak power, RF <= elec x pae_sat)
    unreachable_pct: float     # search positions short of range, in %
    at_avg_power_kw: float = 0.0        # active-tracking share of avg_power_kw, broken out for display
    at_ku_time_fraction: float = 0.0    # AT Ku duty cycle (seconds of Ku TX per second), not a floor -- see below
    at_unreachable_tracks: float = 0.0  # AT tracks (target-count units) short of their bracket's range


def _sector_target(sec: dict) -> rm.Target:
    """The sector's target (radar_model.target_for_sector) for live editing: an incomplete sector falls back to its
    own RCS / Swerling / speed with the default class."""
    s = {"v_close_max_mps": 1000.0, **sec}
    if s.get("target_type") is None:
        s.setdefault("rcs_m2", 1.0)
    try:
        return rm.target_for_sector(s)
    except (KeyError, ValueError, TypeError):
        return rm.Target(str(sec.get("id", "?")), float(sec.get("rcs_m2", 1.0) or 1.0), int(sec.get("swerling", 1) or 1),
                         float(sec.get("v_close_max_mps", 1000.0) or 1000.0))


def _estimate_at_faces(sec: dict, grid: geo.SphereGrid, face_sel: Dict[int, Tuple[np.ndarray, np.ndarray]],
                        npos_all: float, p_avg_max_w: float) -> Dict[int, Tuple[float, float, float]]:
    """Approximate per-face active-tracking (AT) power and Ku duty cycle of one sector's range brackets: one
    representative Ku look per face per bracket (at the face's position-weighted mean scan angle), with the
    bracket's target count split across faces in proportion to search positions -- the same convention
    core.tracking.sector_budget uses per scan-angle bin ('share = gr["pos"] / p_all * n_targets'), just without
    its finer per-bin grouping. Fast enough for live editing (core.tracking.ku_look() is itself cached); not the
    authoritative number (see module docstring) -- the exact AT budget only comes from core.tracking.budget()."""
    brackets = sec.get("tracking", [])
    if not brackets or npos_all <= 0:
        return {}
    tgt = _sector_target(sec)
    ku_lim = rm.BAND_KU.scan_limit_deg(trk.KU_REF_GHZ)   # the fast estimate: Ku core-band centre (step 3.2)
    out: Dict[int, Tuple[float, float, float]] = {}
    for k, (sel, w_sel) in face_sel.items():
        npos_k = float(w_sel.sum())
        theta_k = float(np.average(grid.theta[sel], weights=w_sel))
        power_w, ku_time, unreachable = 0.0, 0.0, 0.0
        for b in brackets:
            n_tr_k = b.get("n_targets", 0.0) * npos_k / npos_all
            if n_tr_k <= 0:
                continue
            rr = b.get("rr_s") or 1.0
            if theta_k > ku_lim:
                unreachable += n_tr_k
                continue
            pack = rm.TrackingPack(b["angle_zone_deg"] / trk.ANGLE_ZONE_SIGMAS, b["vr_acc_mps"], b.get("range_acc_m"))
            # the former track look (acceleration known, no pointing law, no initiation nor loss): only while the
            # volume's instant profile, which prices the brackets by the step 3.3 model, is not ready (5.5, Q3)
            look = trk.ku_look(theta_k, dataclasses.replace(tgt, accel_max_mps2=0.0), b["r_max_km"], p_avg_max_w, pack,
                               sec.get("rx_sll_db"))
            if look is None:
                unreachable += n_tr_k
                continue
            power_w += n_tr_k * look["energy_J"] / rr
            ku_time += n_tr_k * look["time_s"] / rr
        out[k] = (power_w, ku_time, unreachable)
    return out


def useful_range_km(sec: dict, surfaces, d: np.ndarray) -> np.ndarray:
    """The range a direction needs (user, 10 November 2026, the exact budget's rule, step 3.1 P4 Q9): the sector's
    required range, brought back to d - guard where the pointing meets a surface at d (<= 0: no volume there)."""
    from . import environment as env
    from . import optimizer as mo
    d = np.asarray(d, float).reshape(-1, 3)
    r = np.full(len(d), float(sec.get("range_km", 0.0)))
    if surfaces:
        guard = mo.surface_guard_km(sec)
        for s_ in surfaces:
            r = np.minimum(r, env.surface_distance_km(s_, d) - guard)
    return r


def _cell_dirs(grid: geo.SphereGrid) -> np.ndarray:
    p, e = np.radians(grid.phi), np.radians(grid.eps)
    return np.stack([np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)], axis=-1)


def _sky_surfaces(sky: Optional[dict]):
    from . import environment as env
    try:
        return env.surfaces(sky)
    except (KeyError, TypeError, ValueError):
        return ()


def estimate_sector_faces(table: CostTable, sec: dict, grid: geo.SphereGrid, eps_min: float, eps_max: float,
                           p_avg_max_w: float, f_ghz: float = geo.VIZ_FREQ_GHZ,
                           sky: Optional[dict] = None) -> Dict[int, FaceBudgetEstimate]:
    """Approximate per-face energy budget for one sector: bins the sector's scanned directions by face
    ownership, then for each raster cell interpolates energy continuously in scan angle (per cell) and
    in RCS/speed (once per sector, both fixed at the sector level) -- no solve() calls, table lookups only."""
    mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
    if not mask.any():
        return {}
    cell = rm.beam_cell_area_uv(f_ghz)
    weight = grid.w * grid.d_omega / cell  # beam positions per raster cell (same convention as the optimiser)
    rr = sec.get("rr_target_s", 1.0) or 1.0
    # Another RX weighting than the table's only changes eta_illum_rx: the SNR scales as eta / R^4, so the energy of
    # the sector's range is the table's at the equivalent range R (eta_table / eta)^(1/4)
    eta_ratio = (rm.rx_beam("S", table.rx_sll_db).eta / rm.rx_beam("S", sec.get("rx_sll_db")).eta)
    r_want = sec.get("range_km", 0.0) * eta_ratio ** 0.25
    rcs = _sector_target(sec).rcs_at(f_ghz)          # S frontal RCS of the sector's target (class + values, 2.4)
    v = sec.get("v_close_max_mps", 1000.0)
    th_grid = np.array(table.TH, dtype=float)
    log_e_at_th, feas_at_th = energy_profile_by_theta(table, r_want, rcs, v)

    # the ground (sky: the mission sky in the ship frame): a pointing meeting a surface costed at d - guard (E ~ R^4),
    # none where that is <= 0 -- the exact budget's rule (step 3.1, P4 Q9; user, 10 November 2026)
    surf = _sky_surfaces(sky)
    fac = np.ones(grid.phi.shape)
    if surf:
        r_req = float(sec.get("range_km", 0.0)) or 1.0
        r_eff = useful_range_km(sec, surf, _cell_dirs(grid).reshape(-1, 3)).reshape(grid.phi.shape)
        fac = np.clip(r_eff / r_req, 0.0, 1.0) ** 4
        mask = mask & (r_eff > 0.0)
        if not mask.any():
            return {}
    face_sel: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    npos_all = 0.0
    for k in range(4):
        sel = mask & (grid.owner == k)
        if sel.any():
            face_sel[k] = (sel, weight[sel])
            npos_all += float(weight[sel].sum())
    at_by_face = _estimate_at_faces(sec, grid, face_sel, npos_all, p_avg_max_w)

    out: Dict[int, FaceBudgetEstimate] = {}
    for k, (sel, w_sel) in face_sel.items():
        npos_total = float(w_sel.sum())
        theta_sel = np.clip(grid.theta[sel], th_grid[0], th_grid[-1])
        e_cell = np.exp(np.interp(theta_sel, th_grid, log_e_at_th)) * fac[sel]
        e_tot = float((w_sel * e_cell).sum())
        search_power_w = e_tot / rr
        nearest_idx = np.argmin(np.abs(theta_sel[:, None] - th_grid[None, :]), axis=1)
        short_w = float(w_sel[~feas_at_th[nearest_idx]].sum())
        at_power_w, at_ku_time, at_unreach = at_by_face.get(k, (0.0, 0.0, 0.0))
        total_power_w = search_power_w + at_power_w
        out[k] = FaceBudgetEstimate(
            positions=npos_total,
            energy_j_per_refresh=e_tot,
            energy_fraction=total_power_w / p_avg_max_w,
            avg_power_kw=total_power_w / 1e3,
            time_fraction_min=search_power_w * rm.BAND_S.pae_sat / table.ppk,   # RF <= electrical x pae_sat
            unreachable_pct=100.0 * short_w / npos_total if npos_total > 0 else 0.0,
            at_avg_power_kw=at_power_w / 1e3,
            at_ku_time_fraction=at_ku_time,
            at_unreachable_tracks=at_unreach,
        )
    return out


# ----------------------------------------------------------------------------- range gauge (user, 10 November 2026)
_TABLE_CACHE: Dict[str, Optional[CostTable]] = {}


def cached_table(path: str = DEFAULT_TABLE_PATH) -> Optional[CostTable]:
    """The cost table, read once."""
    if path not in _TABLE_CACHE:
        _TABLE_CACHE[path] = load_table(path)
    return _TABLE_CACHE[path]


def _face_reach(grid: geo.SphereGrid) -> Tuple[np.ndarray, np.ndarray]:
    """(reach (4, n_phi, n_eps): the face's scan-limit cone contains the direction, theta (4, ...): its scan angle
    there, deg) of every raster cell, for every face (B1: a direction in the overlap of two cones can be served by
    either); cached on the grid."""
    cached = grid.__dict__.get("_face_reach")
    if cached is not None:
        return cached
    from . import optimizer as mo
    d = _cell_dirs(grid)
    sin_lim = math.sin(math.radians(rm.SECTOR_RADIUS_DEG))
    reach = np.zeros((4,) + grid.phi.shape, bool)
    theta = np.full((4,) + grid.phi.shape, 90.0)
    for k, pk in enumerate(mo.FACE_PHI):
        n, eu, ev = (np.asarray(x) for x in mo.face_frame(pk))
        w, u, v = d @ n, d @ eu, d @ ev
        reach[k] = (w > 0) & (np.sqrt(u * u + v * v) <= sin_lim)
        theta[k] = np.degrees(np.arccos(np.clip(w, 0.0, 1.0)))
    grid.__dict__["_face_reach"] = (reach, theta)
    return reach, theta


def _cap_w(g: dict) -> List[float]:
    """Each face's energy limit (W drawn): its feeder, or the mission's optional cap (face_elec_mw, a number for all,
    a list of 4; 0 / absent = none)."""
    cap = g.get("face_elec_mw")
    caps = ([float(c) for c in cap] if isinstance(cap, (list, tuple)) and len(cap) == 4 else
            [float(cap)] * 4 if isinstance(cap, (int, float)) and cap else [math.inf] * 4)
    return [min(rm.FACE_FEEDER_MAX_MW, c if c > 0 else math.inf) * 1e6 for c in caps]


def _instant_rows(inst: dict, sec: dict, grid: geo.SphereGrid, eps_min: float, eps_max: float, covered: np.ndarray,
                  reach: np.ndarray, theta: np.ndarray, mask: np.ndarray, r_eff: Optional[np.ndarray],
                  rr: float) -> Optional[Tuple[np.ndarray, dict]]:
    """Demand rows [cells of mask, face, (W drawn, S time, Ku time)] of a search volume from its instant profile
    (core/cost_profile.get_instant): positions per cell at the plan's frequency and lattice spacing; energy and TX
    time per position of the plan's 1-deg bin at the cell's scan angle on each face (a bin the plan leaves empty: its
    nearest populated bin through the scan factor, E ~ h^-4); per bin, the plan's positions over the volume's own
    (owner faces, whole volume) take out what the sky blinds. None: no populated bin."""
    e_b, t_b = np.asarray(inst["e"], float), np.asarray(inst["t"], float)
    popd = np.asarray(inst["populated"], bool)
    nb, bd = len(e_b), float(inst["bin_deg"])
    idx = np.flatnonzero(popd)
    if not len(idx) or not mask.any():
        return None
    h = np.asarray(inst["h"], float) if inst.get("h") else np.ones(nb)
    near = idx[np.argmin(np.abs(idx[None, :] - np.arange(nb)[:, None]), axis=1)]
    k_sc = (h[near] / np.maximum(h, 1e-12)) ** 4
    fe = np.where(popd, e_b, e_b[near] * k_sc)
    ft = np.where(popd, t_b, t_b[near] * k_sc)
    w_i = grid.w * grid.d_omega / rm.beam_cell_area_uv(float(inst["f"]), float(inst["spacing"]))
    whole = grid.sector_mask({k_: v_ for k_, v_ in sec.items() if k_ != "_relief"}) & grid.scanned(eps_min, eps_max) \
        & covered
    if r_eff is not None:
        whole = whole & (r_eff > 0.0)
    own = np.clip(grid.owner, 0, 3)
    th_own = np.take_along_axis(theta, own[None], axis=0)[0]
    b_own = np.clip((th_own / bd).astype(int), 0, nb - 1)
    est_b = np.bincount(b_own[whole], weights=w_i[whole], minlength=nb)[:nb]
    plan_b = np.asarray(inst["pos"], float).sum(axis=0)
    with np.errstate(invalid="ignore", divide="ignore"):
        keep_b = np.where(est_b > 0.0, np.clip(plan_b / est_b, 0.0, 1.5), 1.0)
    base = w_i[mask] * keep_b[b_own[mask]] / rr
    rows = np.full((int(mask.sum()), 4, 3), np.nan)
    bins = np.zeros((int(mask.sum()), 4), int)
    for k in range(4):
        ok = reach[k][mask]
        b = np.clip((theta[k][mask][ok] / bd).astype(int), 0, nb - 1)
        bins[ok, k] = b
        rows[ok, k, 0] = fe[b] * base[ok] / rm.CHAIN_EFF
        rows[ok, k, 1] = ft[b] * base[ok]
        rows[ok, k, 2] = 0.0
    # c: per face, the memory traffic (bit/s) and processing power (W) at the revisit asked, each share on its cheapest
    # face (as the recalibration); the bins the plan leaves empty take their nearest populated bin's waveform
    other = {"mem_bps": [0.0] * 4, "proc_w": [0.0] * 4}
    if inst.get("mem_bits") is not None:
        mb, pj = np.asarray(inst["mem_bits"], float)[near], np.asarray(inst["proc_J"], float)[near]
        e = np.where(np.isfinite(rows[:, :, 0]), rows[:, :, 0], np.inf)
        kc = np.argmin(e, axis=1)
        okc = np.isfinite(e[np.arange(len(rows)), kc])
        bc = bins[np.arange(len(rows)), kc]
        for k in range(4):
            sel = okc & (kc == k)
            other["mem_bps"][k] = float((base[sel] * mb[bc[sel]]).sum())
            other["proc_w"][k] = float((base[sel] * pj[bc[sel]]).sum())
    return rows, other


def allocation_demands(table: CostTable, sectors_ship: List[dict], grid: geo.SphereGrid, eps_min: float,
                       eps_max: float, sky: Optional[dict] = None, g: Optional[dict] = None,
                       f_ghz: float = geo.VIZ_FREQ_GHZ, calib: Optional[dict] = None,
                       profiles: Optional[str] = None, estimating: Optional[List[str]] = None,
                       reach_out: Optional[dict] = None, instant_out: Optional[List[str]] = None,
                       other_out: Optional[dict] = None, blind_out: Optional[dict] = None,
                       tracks_out: Optional[List[str]] = None):
    """The final allocation rule's demands and budgets from the fast estimate (pass R2, user 10 November 2026): the
    always-on of each face (tier 0); each search volume (P1 if self-protection, else P4_n) split in its raster cells,
    each costed on every face whose scan-limit cone contains it (its own scan angle, the ground rule, B1), energy
    drawn from the ship bus (through CHAIN_EFF) and S time (lower bound, energy x pae_sat / peak power: C1, not
    recalibrated, M1); the track brackets (P3 by class) on their owner faces (energy, Ku time; by the step 3.3 model
    from the instant profile, 5.5; tracks_out: the volumes so priced); the spare shortens
    the search revisits with realloc_mode PRIO_SEARCH, down to rr_min_s (D1, R2.2). The coverage hole is left out
    (the gauge shows it). calib (pass R3d, core/calibration.lookup): {sector id: {"e": [4], "t": [4]}}, the ratios
    exact / estimate per face of the last exact run, applied to the volume's shares on each face (M1); under
    ALWAYS_ON_KEY, each face's always-on measured by that run (else always_on_static_w).
    profiles (option (c), core/cost_profile): None = the table only; "background" = each volume priced by its own
    profile (the optimiser's laws: cumulative criterion, its candidates and Pfa), the table meanwhile while it is
    computed in the background (its id appended to estimating); "wait" = the profile computed here. With profiles,
    each search volume is priced by its INSTANT profile when there is one (b2-b4, user 12 November 2026: the exact
    optimiser's plan at the instant, its lattice spacing, sky noise, blinded directions and surface clutter; the true
    S time per position; no extra ground factor, the plan holds it), at calib[sid]["f"] (the last exact run's
    frequency) or the table's; its id appended to instant_out.
    -> (demands, budgets)."""
    from . import allocation as al
    from . import margin
    g = g or {}
    p_avail = rm.face_elec_avail_w(rm.FACE_ELEC_POWER_MW)
    on = (calib or {}).get(ALWAYS_ON_KEY)
    demands = []
    # d (user, 12 November 2026): every face spends cal_duration / cal_interval of its S and Ku transmit time on the
    # calibration (step 3.4 E1), as the exact budget counts it
    cal_s, cal_ku = (float(rm.array_errors(b)["cal_time_frac"]) for b in ("S", "Ku"))
    for k in range(4):
        r = np.full((1, 4, 3), np.nan)
        r[0, k] = (float(on[k]) if on else always_on_static_w(), cal_s, cal_ku)
        demands.append(al.Demand(f"always-on {geo.FACE_NAMES[k]}", (0, 0), "other", r))
    reach, theta = _face_reach(grid)
    covered = reach.any(axis=0)
    weight = grid.w * grid.d_omega / rm.beam_cell_area_uv(f_ghz)
    surf = _sky_surfaces(sky)
    th_grid = np.array(table.TH, dtype=float)
    mode = g.get("realloc_mode") or margin.MODE_DEFAULT
    def add_tracks(sec: dict, sid: str, msk: np.ndarray, inst: Optional[dict] = None) -> None:
        """The volume's track brackets (P3) on the faces of its directions msk (user, 11 November 2026: the WHOLE
        volume for a relieved one: its targets appear there whoever detects them). inst: the volume's instant
        profile; its "tracks" (5.5, renvoi 3.3: the step 3.3 model, at the peak) price the brackets when there is one
        per bracket, else the former track look (_estimate_at_faces) stands in meanwhile (Q3)."""
        brackets = sec.get("tracking") or []
        if brackets:
            t_rows = []
            tr = (inst or {}).get("tracks")
            if tr is not None and len(tr) == len(brackets):
                for b_ in tr:
                    for k in range(4):
                        w_t = float(b_["e"][k]) * p_avail / rm.CHAIN_EFF
                        if w_t > 0 or float(b_["tk"][k]) > 0:
                            r = np.full((4, 3), np.nan)
                            r[k] = (w_t, 0.0, float(b_["tk"][k]))
                            t_rows.append(r)
                if tracks_out is not None:
                    tracks_out.append(sid)
            else:
                face_sel = {}
                for k in range(4):
                    sel = msk & (grid.owner == k)
                    if sel.any():
                        face_sel[k] = (sel, weight[sel])
                npos = float(sum(w_.sum() for _s, w_ in face_sel.values()))
                at = _estimate_at_faces(sec, grid, face_sel, npos, p_avail)
                for k, (pw, ku, _un) in at.items():
                    if pw > 0 or ku > 0:
                        r = np.full((4, 3), np.nan)
                        r[k] = (pw / rm.CHAIN_EFF, 0.0, ku)
                        t_rows.append(r)
            if t_rows:
                rank = al.CLASS_RANK.get(sec.get("target_type") or "", len(al.CLASS_RANK))
                demands.append(al.Demand(f"tracks {sid}", (3, rank), "track", np.array(t_rows),
                                         rr_s=float(brackets[0].get("rr_s") or 1.0),
                                         n_targets=float(sum(b_.get("n_targets", 0.0) for b_ in brackets)),
                                         sector=sid))

    for i, sec in enumerate(sectors_ship):
        sid = str(sec.get("id", f"#{i + 1}"))
        try:
            mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max) & covered
            mask_all = None                            # (a relieved volume: its whole volume, for its tracks)
            if sec.get("_relief"):
                mask_all = grid.sector_mask({k_: v_ for k_, v_ in sec.items() if k_ != "_relief"})                     & grid.scanned(eps_min, eps_max) & covered
            fac = np.ones(grid.phi.shape)
            if surf:
                r_req = float(sec.get("range_km", 0.0)) or 1.0
                r_eff = useful_range_km(sec, surf, _cell_dirs(grid).reshape(-1, 3)).reshape(grid.phi.shape)
                fac = np.clip(r_eff / r_req, 0.0, 1.0) ** 4
                mask = mask & (r_eff > 0.0)
                if mask_all is not None:
                    mask_all = mask_all & (r_eff > 0.0)
            if not mask.any():                         # (relieved in full: no search of its own, its tracks kept)
                if mask_all is not None and mask_all.any():
                    inst_t = None
                    if profiles and sec.get("tracking"):   # (Q4: its instant profile on the whole volume, for them)
                        from . import cost_profile
                        f_i = float(((calib or {}).get(sid) or {}).get("f") or table_freq_ghz())
                        inst_t, fresh = cost_profile.get_instant({k_: v_ for k_, v_ in sec.items() if k_ != "_relief"},
                                                                 sky, eps_min, eps_max, f_i, wait=profiles == "wait")
                        if not fresh and estimating is not None and sid not in estimating:
                            estimating.append(sid)
                        if inst_t is not None and inst_t.get("status") != "OK":
                            inst_t = None
                    add_tracks(sec, sid, mask_all, inst_t)
                    if profiles:                       # ... and the check of its confirmation (its targets, its
                        from . import cost_profile     # range), not of its reach (the covering volume's)
                        prof = cost_profile.get(sec, float(weight[mask_all].sum()), th_grid.tolist(),
                                                float(table_freq_ghz()), wait=profiles == "wait")
                        if prof is None and estimating is not None:
                            estimating.append(sid)
                        elif prof is not None and reach_out is not None:
                            reach_out[sid] = {"confirm_ok_spacings": prof.get("confirm_ok_spacings"),
                                              "confirm_max_km": prof.get("confirm_max_km")}
                continue
            rr = float(sec.get("rr_target_s", 1.0) or 1.0)
            eta_ratio = rm.rx_beam("S", table.rx_sll_db).eta / rm.rx_beam("S", sec.get("rx_sll_db")).eta
            r_want = float(sec.get("range_km", 0.0)) * eta_ratio ** 0.25
            if r_want <= 0.0:
                continue
            prof = None
            if profiles:                              # option (c): the volume's own cost profile
                from . import cost_profile
                prof = cost_profile.get(sec, float(weight[mask].sum()), th_grid.tolist(), float(table_freq_ghz()),
                                        wait=profiles == "wait")
                if prof is None and estimating is not None:
                    estimating.append(sid)
            reach_km = None
            if prof is not None:
                log_e = np.asarray(prof["log_e"], float)
                if prof.get("r_reach_km"):                 # (user, 11 November 2026: a range out of reach was shown
                    reach_km = np.asarray(prof["r_reach_km"], float)   # held: its cost at full power was "served")
                    if reach_out is not None:
                        reach_out[sid] = {"th": th_grid.tolist(), "r_reach_km": reach_km.tolist(),
                                          "r_want_km": float(sec.get("range_km", 0.0)),
                                          "confirm_ok_spacings": prof.get("confirm_ok_spacings"),
                                          "confirm_max_km": prof.get("confirm_max_km")}
            else:
                log_e, _feas = energy_profile_by_theta(table, r_want, _sector_target(sec).rcs_at(f_ghz),
                                                       sec.get("v_close_max_mps", 1000.0))
            inst = None
            if profiles:                              # b2-b4 (user, 12 November 2026): the exact plan at the instant
                from . import cost_profile
                f_i = float(((calib or {}).get(sid) or {}).get("f") or table_freq_ghz())
                sec_i = {k_: v_ for k_, v_ in sec.items() if k_ != "_relief"}
                inst, fresh = cost_profile.get_instant(sec_i, sky, eps_min, eps_max, f_i, wait=profiles == "wait")
                if not fresh and estimating is not None and sid not in estimating:
                    estimating.append(sid)
                if inst is not None and inst.get("status") != "OK":
                    inst = None
                if inst is not None and blind_out is not None and inst.get("blind_km"):   # (5.5: the played view's
                    blind_out[sid] = {"bin_deg": float(inst.get("bin_deg", 1.0)),             # inner surface)
                                      "blind_km": list(inst["blind_km"])}
            rows = None
            if inst is not None:
                got = _instant_rows(inst, sec, grid, eps_min, eps_max, covered, reach, theta, mask,
                                    r_eff if surf else None, rr)
                if got is not None:
                    rows, oth = got
                    if instant_out is not None:
                        instant_out.append(sid)
                    if other_out is not None:
                        other_out[sid] = oth
                    cf = inst.get("confirm")
                    if cf:                             # d: the confirmation of its false alarms (P2, never shed)
                        c_rows = []
                        for k in range(4):
                            w_c = float(cf["e"][k]) * p_avail / rm.CHAIN_EFF
                            if w_c > 0 or cf["ts"][k] > 0 or cf["tk"][k] > 0:
                                r_c = np.full((4, 3), np.nan)
                                r_c[k] = (w_c, float(cf["ts"][k]), float(cf["tk"][k]))
                                c_rows.append(r_c)
                        if c_rows:
                            demands.append(al.Demand(f"confirmation {sid}", (2, 0), "other", np.array(c_rows),
                                                     sector=sid))
            if rows is None:
                rows = np.full((int(mask.sum()), 4, 3), np.nan)
                base = (weight * fac)[mask] / rr
                for k in range(4):
                    ok = reach[k][mask]
                    e = np.exp(np.interp(np.clip(theta[k][mask][ok], th_grid[0], th_grid[-1]), th_grid, log_e)) * base[ok]
                    rows[ok, k, 0] = e / rm.CHAIN_EFF
                    rows[ok, k, 1] = e * rm.BAND_S.pae_sat / table.ppk
                    rows[ok, k, 2] = 0.0
            c = (calib or {}).get(sid)
            if c is not None and c.get("e") is not None:   # pass R3d (M1): recalibrated by the exact computation
                rows[:, :, 0] *= np.asarray(c["e"], float)[None, :]
                rows[:, :, 1] *= np.asarray(c["t"], float)[None, :]
            tier = (1, 0) if sec.get("self_protection") else (4, int(sec.get("search_priority", 1) or 1))
            h = rr / max(float(sec.get("rr_min_s") or margin.RR_MIN_SEARCH_S), 1e-9) if mode == "PRIO_SEARCH" else 1.0
            demands.append(al.Demand(f"search {sid}", tier, "search", rows, rr_s=rr, h_max=max(1.0, h), sector=sid))
            add_tracks(sec, sid, mask_all if mask_all is not None else mask, inst)
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            continue                                   # (a sector mid-edit: left out)
    budgets = al.Budgets(face_w=_cap_w(g), ceiling=float(g.get("load_ceiling") or al.LOAD_CEILING))
    return demands, budgets


def _reuse_limits(demands: list, other: Dict[str, dict], ceiling: float):
    """c: (g per search volume, why, load per constraint at the revisits asked, constraints) of the memory bandwidth
    and processing of every face against the load ceiling, by margin._fill (the exact reuse's water-filling) from
    each volume's load at its revisit asked (other: allocation_demands' other_out); None without such loads."""
    from . import compute, margin
    ids = [d.sector for d in demands if d.kind == "search" and d.sector in other]
    if not ids:
        return None
    caps = {"memory bandwidth": ("mem_bps", compute.MEMORY_BW_BPS),
            "compute": ("proc_w", compute.CEILING_W * (1.0 - compute.STATIC_FRAC))}
    cons: Dict[str, dict] = {}
    for k in range(4):
        for name, (key, cap) in caps.items():
            if cap <= 0:
                continue
            a = {s: float(other[s][key][k]) / cap for s in ids if float(other[s][key][k]) > 0.0}
            if a:
                cons[f"F{k + 1} {name}"] = {"fixed": 0.0, "a": a, "face": k}
    if not cons:
        return None
    nominal = {n: sum(c["a"].values()) for n, c in cons.items()}
    g_max = {d.sector: float(d.h_max) for d in demands if d.kind == "search" and d.sector in ids}
    g, why = margin._fill(cons, ids, g_max, ceiling, nominal)
    return g, why, nominal, cons


_LIVE_CACHE: Dict[str, dict] = {}


def _key(obj) -> str:
    return json.dumps(obj, sort_keys=True, default=lambda o: np.asarray(o, float).round(9).tolist()
                      if isinstance(o, (np.ndarray, np.generic)) else str(o))


ALWAYS_ON_KEY = "__always_on__"         # calib: each face's always-on (W drawn) measured by the last exact run


def always_on_static_w() -> float:
    """The always-on of a face drawn from the bus before any exact run (user, 10 November 2026, option (c)): the
    receive chains, converters and control, and the processing at its static share (compute.static_frac x
    power.dbf_kw), not at its ceiling: what the exact computation counts for a lightly loaded processor (step 4.4);
    after an exact run, the always-on it measured (core/calibration.always_on)."""
    ceiling = float(params.P.get("power.dbf_kw")) * 1e3
    return (rm.non_tx_power_w() - ceiling * (1.0 - float(params.P.get("compute.static_frac")))) / rm.CHAIN_EFF


def table_freq_ghz() -> float:
    """The frequency the table and the cost profiles are computed at (tool.cost_table.freq_ghz)."""
    return float(params.P.get("tool.cost_table.freq_ghz"))


def live_allocation(table: CostTable, sectors_ship: List[dict], grid: geo.SphereGrid, eps_min: float, eps_max: float,
                    sky: Optional[dict] = None, g: Optional[dict] = None, calib: Optional[dict] = None,
                    profiles: Optional[str] = None, relief: Optional[str] = None) -> dict:
    """_live_allocation() (below), and with relief (user, 11 November 2026, A and B: "background" or "wait", as
    profiles; None: none) the directions of a volume whose need another one already meets taken out of its demand
    (core/redundancy.relief, judged with the shares the rule serves without it) and the rule again. Adds "relief"
    {id: {"share", "by"}} and "relief_pending" (True: still computed, the volumes budgeted in full meanwhile)."""
    raw = _live_allocation(table, sectors_ship, grid, eps_min, eps_max, sky, g, calib, profiles)
    from . import redundancy as _red
    if not relief or not _red.RELIEF_ON:
        return raw
    if raw.get("estimating"):                          # (the shares served still priced by the table: wait for them)
        return dict(raw, relief={}, relief_pending=True, served={})
    from . import redundancy
    served = {}
    for sid, rec in raw.get("by_sector", {}).items():
        sec = next((s for s in sectors_ship if str(s.get("id")) == sid), {})
        served[sid] = (float(rec.get("rho", 1.0)), float(rec.get("rr_tenable_s", sec.get("rr_target_s", 1.0))))
    try:
        pk = redundancy.relief_cached(sectors_ship, grid, eps_min, eps_max, sky, g, served, wait=relief == "wait")
    except Exception:  # noqa: BLE001 -- a relief the model refuses: none (budgeted in full)
        pk = {}
    if pk is None:
        return dict(raw, relief={}, relief_pending=True, served=served)
    rl = pk.get("relief") or {}
    # the cover and the redundancy check from the same pass (user, 12 November 2026): "X % covered, Y % taken out"
    extra = {"cover": dict(pk.get("cover") or {}),
             "redundancy": {"findings": dict(pk.get("findings") or {}), "notes": dict(pk.get("notes") or {})}}
    if not rl:
        return dict(raw, relief={}, relief_pending=False, served=served, **extra)
    secs = [dict(s, _relief=rl[str(s.get("id"))]["code"]) if str(s.get("id")) in rl else s for s in sectors_ship]
    out = dict(_live_allocation(table, secs, grid, eps_min, eps_max, sky, g, calib, profiles))
    out["relief"] = {sid: {"share": v["share"], "by": list(v["by"]), "code": v["code"]} for sid, v in rl.items()}
    out["relief_pending"] = False
    out["served"] = served
    out.update(extra)
    ach = {k: dict(v) for k, v in (out.get("achieved") or {}).items()}
    for sid, v in out["relief"].items():
        v = {"share": v["share"], "by": v["by"]}
        if sid in ach:
            ach[sid]["relief"] = v
        else:                                          # (relieved in full: nothing of its own left to search)
            sec = next((s for s in sectors_ship if str(s.get("id")) == sid), {})
            ach[sid] = {"worst_km": None, "ground_km": None, "rr_s": float(sec.get("rr_target_s", 0.0) or 0.0),
                        "limit": None, "ok": True, "confirm": True, "relief": v}
    out["achieved"] = ach
    return out


def _live_allocation(table: CostTable, sectors_ship: List[dict], grid: geo.SphereGrid, eps_min: float,
                     eps_max: float, sky: Optional[dict] = None, g: Optional[dict] = None,
                     calib: Optional[dict] = None, profiles: Optional[str] = None) -> dict:
    """The final allocation rule on the fast estimate at one instant (the live load and the range gauge share it: one
    computation per set of volumes, instant and mission settings). The rule's result (allocation.solve) plus
    "by_sector" {id: its search record}, "ref_w" (the face reference, power.face_elec_mw: 100 % of the energy bars)
    and "time_lower_bound" (True: some volume's time is the fast estimate's lower bound, not recalibrated by an exact
    run), "not_recalibrated" (those volumes; pass R3d, calib as allocation_demands), "estimating" (the volumes still
    priced by the table while their cost profile is computed; profiles as allocation_demands). A result with volumes
    estimating is not kept in the cache (the next call takes the profiles that are ready)."""
    from . import allocation as al
    gk = {k: (g or {}).get(k) for k in ("face_elec_mw", "realloc_mode", "load_ceiling")}
    key = _key([sectors_ship, eps_min, eps_max, sky, gk, id(grid), calib, profiles])
    hit = _LIVE_CACHE.get(key)
    if hit is not None:
        return hit
    estimating: List[str] = []
    reach: Dict[str, dict] = {}
    instant: List[str] = []
    other: Dict[str, dict] = {}
    blind: Dict[str, dict] = {}
    tracked: List[str] = []
    demands, budgets = allocation_demands(table, sectors_ship, grid, eps_min, eps_max, sky, gk, calib=calib,
                                          profiles=profiles, estimating=estimating, reach_out=reach,
                                          instant_out=instant, other_out=other, blind_out=blind, tracks_out=tracked)
    # c (user, 12 November 2026): the resources the rule does not hold but the exact margin reuse does (memory
    # bandwidth, processing), linear in the pointing rate: the same water-filling (margin._fill) caps each search
    # volume's revisit gain before the rule; above 100 % at the revisits asked the exact verdict is NOK
    lim = _reuse_limits(demands, other, budgets.ceiling)
    if lim is not None:
        import dataclasses as _dcs
        g_r = lim[0]
        demands = [_dcs.replace(d, h_max=max(1.0, min(d.h_max, g_r[d.sector])))
                   if d.kind == "search" and d.sector in g_r else d for d in demands]
    # just need (user, 12 November 2026): the rule with no target present (the tracks of n_targets are the peak, not
    # a load: the spare goes to realloc_mode), then the peak apart (every track present, no spare: the P4 search
    # shed, never P1). The confirmation of the false alarms (nominal view) comes with each volume's instant profile
    # (d, 12 November 2026); the true detections of new_targets_per_rr at the peak stay the exact calculation's.
    nominal = [d for d in demands if d.kind != "track"]
    out = al.solve(nominal, budgets)
    if len(nominal) < len(demands):
        import dataclasses as _dcs
        peak_rep = al.solve([_dcs.replace(d, h_max=1.0) for d in demands], budgets)
        out["peak"] = al.peak_summary(peak_rep)
        out["peak"]["report"] = {k_: peak_rep[k_] for k_ in ("demands", "faces", "ship", "binding", "status")}
        # (5.5 point A: the diagnosis per constraint reads the peak loads too)
        out["flags"].extend(out["peak"]["flags"])
        if out["peak"]["status"] != "ABSORBED":
            out["status"] = "NOT_TENABLE"
    if lim is not None:                               # c: the loads the rule does not hold, at the revisits served
        g_r, why_r, nom, cons = lim
        served = {d["sector"]: float(d.get("level", 1.0)) for d in out["demands"] if d.get("kind") == "search"}
        loads = {n: sum(a * max(1.0, served.get(s, 1.0)) for s, a in c["a"].items()) for n, c in cons.items()}
        out["other_loads"] = {"asked": nom, "served": loads,
                              "reuse_limited_by": {s: why_r[s] for s in why_r},
                              "shares": {n: dict(c["a"]) for n, c in cons.items()}}   # (5.5 A: who weighs)
        for n, v in sorted(nom.items()):
            if v > 1.0 + 1e-9:
                out["flags"].append(f"{n} at {100 * v:.0f} % at the revisits asked (exact budget's rule): "
                                    f"mission not tenable")
                out["status"] = "NOT_TENABLE"
            elif v > budgets.ceiling + 1e-9:
                out["flags"].append(f"{n} at {100 * v:.0f} %, above the load ceiling {100 * budgets.ceiling:.0f} %")
    out["reach"] = reach                              # per volume: the range reachable at full pulse per scan angle
    out["blind"] = blind                              # per volume: the blind range of each scan-angle bin (5.5)
    out["out_of_reach"] = {}                           # per volume: the share of its directions out of reach
    for sid, rc in reach.items():
        sec = next((s for s in sectors_ship if str(s.get("id")) == sid), None)
        if sec is None:
            continue
        try:
            th_cells = _face_reach(grid)[1]
            mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
            th_min = np.where(np.isfinite(th_cells), th_cells, np.inf).min(axis=0)[mask]
            r_cell = np.interp(np.clip(th_min, rc["th"][0], rc["th"][-1]), rc["th"], rc["r_reach_km"])
            w = (grid.w * grid.d_omega)[mask]
            short = r_cell < rc["r_want_km"] * (1.0 - 1e-6)
            if short.any():
                out["out_of_reach"][sid] = {"share": float(w[short].sum() / max(w.sum(), 1e-12)),
                                            "max_km": float(np.max(rc["r_reach_km"])),
                                            "worst_km": float(np.min(r_cell))}
        except (KeyError, ValueError, IndexError):
            continue
    out["no_confirmation"], out["confirmation_depends"] = [], []
    n_sp = len(params.P.get("search_grid.beam_spacing_options_hpbw"))
    for sid, rc in reach.items():                      # the confirmation of the detections (P2: never shed)
        okl = rc.get("confirm_ok_spacings")
        if okl is None:
            continue
        if not okl:
            out["no_confirmation"].append(sid)
            cm = rc.get("confirm_max_km")
            out.setdefault("confirm_max_km", {})[sid] = cm
            out["flags"].append(f"{sid}: its detections cannot be confirmed (no admissible confirmation mode at any "
                                f"beam spacing)" + (f"; confirmation possible up to ~{cm:.0f} km" if cm else "")
                                + ": mission not tenable")
        elif len(okl) < n_sp:
            out["confirmation_depends"].append(sid)
            rng = f"{min(okl):g}" if min(okl) == max(okl) else f"{min(okl):g}-{max(okl):g}"
            out["flags"].append(f"{sid}: confirmation only with a search beam spacing of {rng} HPBW: the exact "
                                f"calculation says which the optimiser keeps")
    if out["no_confirmation"]:
        out["status"] = "NOT_TENABLE"
    if out["out_of_reach"]:
        if out["status"] == "OK":
            out["status"] = "DEGRADED"
        for sid, o in out["out_of_reach"].items():
            out["flags"].append(f"{sid}: out of reach on {100 * o['share']:.0f} % of the volume, whatever the "
                                f"energy (at most ~{o['max_km']:.0f} km at full pulse)")
    asked = np.zeros(4)                                # what every demand asks at full service, per face (each share
    for d in demands:                                  # on its cheapest face): shown beside what is served when the
        rows = np.asarray(d.rows, float).reshape(-1, 4, 3)   # rule cannot serve it all (user, 11 November 2026)
        if not len(rows):
            continue
        e = np.where(np.isfinite(rows[:, :, 0]), rows[:, :, 0], np.inf)
        k = np.argmin(e, axis=1)
        ok = np.isfinite(e[np.arange(len(rows)), k])
        np.add.at(asked, k[ok], e[np.arange(len(rows)), k][ok])
    out["asked_w"] = [float(x) for x in asked]
    out["by_sector"] = {r["sector"]: r for r in out["demands"] if r["kind"] == "search" and r["sector"] is not None}
    out["ref_w"] = rm.FACE_ELEC_POWER_MW * 1e6
    out["not_recalibrated"] = sorted(s for s in out["by_sector"]
                                     if ((calib or {}).get(s) or {}).get("e") is None)
    out["instant_priced"] = sorted(instant)            # (b2-b4: priced by the exact plan at the instant, true S time)
    out["tracks_priced"] = sorted(set(tracked))       # (5.5, renvoi 3.3: tracks by the step 3.3 model)
    out["time_lower_bound"] = bool([s for s in out["not_recalibrated"] if s not in instant])
    out["estimating"] = estimating
    out["achieved"] = {}                               # per volume (user, 11 November 2026: the 3D labels' 2nd line)
    for sid, rec in out["by_sector"].items():
        sec = next((s for s in sectors_ship if str(s.get("id")) == sid), None)
        if sec is None:
            continue
        r_want = float(sec.get("range_km", 0.0) or 0.0)
        # out of reach, the profile prices the full pulse, i.e. the range reachable: the allocation's share rho then
        # applies to that range, not to the one asked (user, 11 November 2026: S01's label did not move with its
        # priority, min(rho R, reach) staying at the reach while rho >= 0.8)
        rho = float(rec.get("rho", 1.0))
        o = out["out_of_reach"].get(sid)
        reach_w = min(r_want, float(o.get("worst_km", r_want))) if o is not None else r_want
        worst = rho * reach_w
        limit = "reach" if reach_w < r_want * (1.0 - 1e-6) else ("power" if rho < 1.0 - 1e-6 else None)
        rr_asked = float(sec.get("rr_target_s", 0.0) or 0.0)
        rr = float(rec.get("rr_tenable_s", rr_asked))
        # the terrain mask (user, 11 November 2026): where the ground cuts the volume the range needed is the ground's
        # (step 3.1 P4 Q9, already in the costs); the label says it, and the volume is held when the range held covers
        # the range NEEDED in every direction
        ground = None
        need_max = r_want
        try:
            surf = _sky_surfaces(sky)
            if surf:
                cells = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
                need = useful_range_km(sec, surf, _cell_dirs(grid)[cells].reshape(-1, 3))
                need = need[need > 0.0]
                cut = need < r_want * (1.0 - 1e-6)
                if len(need) and cut.any():
                    ground = (float(need[cut].min()), float(need[cut].max()), float(cut.mean()))
                    need_max = float(need.max())
        except (KeyError, TypeError, ValueError):
            pass
        sky_part = ground is None or ground[2] < 1.0 - 1e-9
        out["achieved"][sid] = {"worst_km": worst if sky_part else None, "ground_km": ground, "rr_s": rr,
                                "limit": limit if sky_part else ("power" if rho < 1.0 - 1e-6 else None),
                                "ok": rho * r_want >= need_max * (1.0 - 1e-6)
                                and worst >= (need_max if not sky_part else r_want) * (1.0 - 1e-6)
                                and rr <= rr_asked * (1.0 + 1e-9),           # (range and revisit only)
                                "confirm": sid not in out["no_confirmation"],   # (said on a line of its own)
                                "confirm_max_km": (out.get("confirm_max_km") or {}).get(sid)}
    if estimating:
        return out
    if len(_LIVE_CACHE) > 8:
        _LIVE_CACHE.clear()
    _LIVE_CACHE[key] = out
    return out


# ----------------------------------------------------------------------------- the fast estimate off the window's thread
# (user, 12 November 2026: the window froze ~2-4 s at each edit of a large mission). One worker process, kept, computes
# live_allocation (its cost profiles and reliefs in its own background); the views get at once the result of their
# inputs when known, else the last one they were given (marked "stale"), and are redrawn when a new result differs.
ASYNC_RESUBMIT_S = 3.0                   # a result still estimating (profiles, relief): asked again this often
_A_LOCK = threading.RLock()     # (re-entrant: a future already done runs _a_done at once in the thread that adds its
                                # callback, under this lock: a plain Lock deadlocked the window, 12 November 2026)
_A: Dict[str, dict] = {"jobs": {}, "cache": {}, "args": {}, "last": {}, "tag_key": {}, "digest": {}}
_A_POOL = None
_A_GRIDS: Dict[float, geo.SphereGrid] = {}


def _live_task(sectors_ship, res_deg, eps_min, eps_max, sky, g, calib, relief):
    """Worker side: the fast estimate as live_allocation computes it (profiles and relief in the worker's own
    background)."""
    grid = _A_GRIDS.get(res_deg)
    if grid is None:
        grid = _A_GRIDS[res_deg] = geo.SphereGrid(res_deg=res_deg)
    table = cached_table()
    if table is None:
        return None
    return live_allocation(table, sectors_ship, grid, eps_min, eps_max, sky, g, calib=calib, profiles="background",
                           relief=relief)


def _digest(res: Optional[dict]) -> str:
    if res is None:
        return ""
    return _key({k: res.get(k) for k in ("status", "estimating", "relief_pending", "faces", "by_sector", "achieved",
                                         "relief", "cover", "out_of_reach", "no_confirmation")})


def estimate_pool():
    """The estimate's worker process (the live estimate, its profiles and reliefs in its threads, the redundancy check:
    one set of caches, E2)."""
    global _A_POOL
    if _A_POOL is None:
        import atexit
        from concurrent.futures import ProcessPoolExecutor
        from .optimizer import _mark_pool_worker
        _A_POOL = ProcessPoolExecutor(max_workers=1, initializer=_mark_pool_worker)
        atexit.register(lambda: _A_POOL.shutdown(wait=False, cancel_futures=True))
    return _A_POOL


def _a_submit(key: str) -> None:
    """(under _A_LOCK) the inputs of key sent to the worker."""
    from . import background
    fut = estimate_pool().submit(_live_task, *_A["args"][key])
    _A["jobs"][key] = fut
    background.ESTIMATE.start(key)
    fut.add_done_callback(lambda f_, k_=key: _a_done(k_, f_))


def _a_done(key: str, fut) -> None:
    from . import background
    try:
        res = fut.result()
    except Exception:  # noqa: BLE001 -- a mission mid-edit the rule refuses: no result
        res = None
    with _A_LOCK:
        _A["jobs"].pop(key, None)
        background.ESTIMATE.done(key)
        # the relief runs in a process of the worker (its tracker unseen from the window): counted here from the
        # results, so that the badge shows it for as long as it runs (user, 12 November 2026: grey "ESTIMATE" for
        # the 7 min of a relief)
        if res is not None and res.get("relief_pending") and key in _A["tag_key"].values():
            background.RELIEF.start(key)
        elif res is not None:
            background.RELIEF.done(key)
        if res is not None:
            if len(_A["cache"]) > 32:
                for k_ in list(_A["cache"])[:16]:
                    if k_ not in _A["tag_key"].values():
                        _A["cache"].pop(k_, None)
            _A["cache"][key] = res
            d = _digest(res)
            if d != _A["digest"].get(key):         # (redrawn only when it changed)
                _A["digest"][key] = d
                background.bump()
        live = key in _A["tag_key"].values()
        again = live and res is not None and bool(res.get("estimating") or res.get("relief_pending"))
    if again:
        t = threading.Timer(ASYNC_RESUBMIT_S, _a_resubmit, args=(key,))
        t.daemon = True
        t.start()


def _a_resubmit(key: str) -> None:
    from . import background
    if background.estimate_paused():               # (asked again at the resume: background.bump)
        return
    with _A_LOCK:
        if key in _A["tag_key"].values() and key not in _A["jobs"]:
            try:
                _a_submit(key)
            except RuntimeError:                   # (the worker shut down: the tool is closing)
                _A["jobs"].pop(key, None)


def async_pending() -> int:
    """Estimates being computed, and the live ones still estimating (asked again): the views keep polling."""
    with _A_LOCK:
        n = len(_A["jobs"])
        for key in set(_A["tag_key"].values()):
            r = _A["cache"].get(key)
            if key not in _A["jobs"] and r is not None and (r.get("estimating") or r.get("relief_pending")):
                n += 1
        return n


# (user, 13 November 2026) the live estimate HELD after an edit of the volumes: every edit, even a small one, started
# a whole computation (profiles, relief, allocation); held, live_allocation_async starts nothing and gives the last
# result "obsolete" (a result already known for the same inputs, e.g. an edit undone, is still given); released by
# the operator (the badge "OBSOLETE ESTIMATE - RERUN" clicked)
_HOLD = {"on": False}


def hold(on: bool) -> None:
    _HOLD["on"] = bool(on)


def held() -> bool:
    return bool(_HOLD["on"])


def live_allocation_async(table: CostTable, sectors_ship: List[dict], grid: geo.SphereGrid, eps_min: float,
                          eps_max: float, sky: Optional[dict] = None, g: Optional[dict] = None,
                          calib: Optional[dict] = None, relief: Optional[str] = "background",
                          tag: str = "view") -> Optional[dict]:
    """live_allocation (profiles "background") computed in the worker process: the result of these inputs when known,
    else the last one given to `tag` with "stale": True (None the first time); the views poll core.background and
    are redrawn when a result differs (core.cost_profile.generation)."""
    gk = {k: (g or {}).get(k) for k in ("face_elec_mw", "realloc_mode", "load_ceiling", "spectrum_exclusions_ghz")}
    res_deg = float(getattr(grid, "res_deg", 1.0))
    key = _key([sectors_ship, eps_min, eps_max, sky, gk, res_deg, calib, relief])
    with _A_LOCK:
        old = _A["tag_key"].get(tag)
        _A["tag_key"][tag] = key
        if old is not None and old != key and old not in _A["tag_key"].values():
            from . import background
            background.RELIEF.cancel(old)            # (its relief no longer awaited: off the badge)
        hit = _A["cache"].get(key)
        if hit is None and _HOLD["on"]:              # (held: nothing started, the last result said obsolete)
            last = _A["last"].get(tag)
            return None if last is None else dict(last, stale=True, obsolete=True)
        from . import background
        if hit is None and background.estimate_paused():   # (an exact calculation running: nothing started)
            last = _A["last"].get(tag)
            return None if last is None else dict(last, stale=True)
        if hit is None and key not in _A["jobs"]:
            _A["args"][key] = ([dict(s) for s in sectors_ship], res_deg, eps_min, eps_max, sky, dict(g or {}),
                               dict(calib or {}), relief)
            try:
                _a_submit(key)
            except Exception:  # noqa: BLE001 -- no worker process (frozen install): computed here
                _A["jobs"].pop(key, None)
                hit = None
                sync = True
            else:
                sync = False
        else:
            sync = False
        if hit is not None:
            _A["last"][tag] = hit
            return hit
        last = _A["last"].get(tag)
    if sync:
        out = live_allocation(table, sectors_ship, grid, eps_min, eps_max, sky, g, calib=calib,
                              profiles="background", relief=relief)
        with _A_LOCK:
            _A["last"][tag] = out
        return out
    return None if last is None else dict(last, stale=True)
