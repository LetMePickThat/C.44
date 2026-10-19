#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COST_ESTIMATE -- fast, approximate per-sector radar budget for real-time feedback while the operator
is editing the mission preparation (sliders / form fields). NOT the authoritative result: the exact
numbers come from core.optimizer.run() (full waveform search), called on "Run calculation".

Backed by cost_table.json (built once by tools/gen_cost_table.py): for S band, 2.8 GHz, a grid of
(range_km, rcs_m2, v_close_mps, scan_theta_deg) -> cheapest energy per beam position that just reaches
that range (or None if unreachable even with the best waveform at full power).

Interpolation strategy (all in log-energy space, since energy spans orders of magnitude):
  - range       : continuous via the model's own E ~ R^4 law, anchored on the nearest FEASIBLE
                  tabulated range (never re-snapped -- that would make the gauges jump while typing).
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
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import geometry as geo
from . import radar_model as rm

DEFAULT_TABLE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cost_table.json")


@dataclass
class CostTable:
    R: List[float]
    RCS: List[float]
    V: List[float]
    TH: List[float]
    ppk: float
    pavg_ref_w: float
    t: Dict[str, Optional[list]]
    reach: Optional[Dict[str, list]] = None   # per cell: [max range km at full power, that waveform's energy J]

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
                     reach=d.get("reach"))


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
    block for the bilinear interpolation below -- call with exact grid values). Range is extrapolated
    from the nearest FEASIBLE tabulated range via the model's E ~ R^4 law."""
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
    rs = [r for r, _ in feasible_rs]
    r_max = max(rs)
    if r_want_km > r_max and reach is None:
        # Old table without reach data: the longest tabulated reach falls short, and extrapolating further with
        # the same R^4 law would understate the true cost (waveform saturates well before this).
        return PositionEnergyEstimate(None, False,
                                       f"range {r_want_km:.0f} km beyond estimated capability (tabulated max {r_max:.0f} km)")
    r_ref = min(rs, key=lambda r: abs(math.log(r) - math.log(max(r_want_km, 1e-6))))
    e_ref = dict(feasible_rs)[r_ref][0]
    e = e_ref * (r_want_km / r_ref) ** 4
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
        feas.append(all(corner_ok))
    return np.array(log_e), np.array(feas)


@dataclass
class FaceBudgetEstimate:
    positions: float
    energy_j_per_refresh: float
    energy_fraction: float
    avg_power_kw: float
    time_fraction_min: float   # LOWER bound only (avg power / peak power) -- see module docstring
    unreachable_pct: float


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
    r_want = sec.get("range_km", 0.0)
    rcs = sec.get("rcs_m2", 1.0)
    v = sec.get("v_close_max_mps", 1000.0)
    th_grid = np.array(table.TH, dtype=float)
    log_e_at_th, feas_at_th = energy_profile_by_theta(table, r_want, rcs, v)
    out: Dict[int, FaceBudgetEstimate] = {}
    for k in range(4):
        sel = mask & (grid.owner == k)
        if not sel.any():
            continue
        w_sel = weight[sel]
        npos_total = float(w_sel.sum())
        theta_sel = np.clip(grid.theta[sel], th_grid[0], th_grid[-1])
        e_cell = np.exp(np.interp(theta_sel, th_grid, log_e_at_th))
        e_tot = float((w_sel * e_cell).sum())
        avg_power_w = e_tot / rr
        nearest_idx = np.argmin(np.abs(theta_sel[:, None] - th_grid[None, :]), axis=1)
        short_w = float(w_sel[~feas_at_th[nearest_idx]].sum())
        out[k] = FaceBudgetEstimate(
            positions=npos_total,
            energy_j_per_refresh=e_tot,
            energy_fraction=avg_power_w / p_avg_max_w,
            avg_power_kw=avg_power_w / 1e3,
            time_fraction_min=avg_power_w / table.ppk,
            unreachable_pct=100.0 * short_w / npos_total if npos_total > 0 else 0.0,
        )
    return out
