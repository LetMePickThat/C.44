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

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import params
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
            look = trk.ku_look(theta_k, tgt, b["r_max_km"], p_avg_max_w, pack, sec.get("rx_sll_db"))
            if look is None:
                unreachable += n_tr_k
                continue
            power_w += n_tr_k * look["energy_J"] / rr
            ku_time += n_tr_k * look["time_s"] / rr
        out[k] = (power_w, ku_time, unreachable)
    return out


def estimate_sector_faces(table: CostTable, sec: dict, grid: geo.SphereGrid, eps_min: float, eps_max: float,
                           p_avg_max_w: float, f_ghz: float = geo.VIZ_FREQ_GHZ) -> Dict[int, FaceBudgetEstimate]:
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
        e_cell = np.exp(np.interp(theta_sel, th_grid, log_e_at_th))
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
