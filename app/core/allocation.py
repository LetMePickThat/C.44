#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
POWER ALLOCATION -- the ship's electrical power shared by the four faces (roadmap step 5.2, pass P1, validated by the
user on 25 October 2026).

Need of a face = what it draws from the ship bus: its always-on consumption (receive chains, converters, control, the
processing as computed by step 4.4, not its ceiling) and pumps, plus the transmissions of every task, all through the
power supply and distribution (CHAIN_EFF). The tasks are sorted in priority tiers (the scheduler's priorities, step 4.3,
and the target class for the tracks):
  0  always-on and calibration          1  search of the self-protection volumes (P1)
  2  confirmations and initiations      3  track updates, by class: KKV > SHIP > DECOY > DEBRIS > other
  3' Ku burn-through (allowed)          4  search of the other volumes, P4_1, P4_2, ...
  5  margin reuse (pass P4): the part of a search drawn by its revisit shortened below the one asked (core/margin.py),
     or (pass P7, realloc_mode PRIO_TRACK) the part of a track bracket drawn by its faster cadence; the first given
     back
Until model V6.37.0 these needs were shared pro rata (allocate(): water-filling of the ship total, the tiers in
order on a shortfall, the last one in part with a tenable revisit); since pass R3 (user, 10 November 2026) every
allocation is the final rule below.

The FINAL rule (passes R1 and R2, user 10 November 2026) is solve() at the end of this module: range first, moves
between faces in the overlaps, energy and S / Ku time together under the load ceiling, spare to the revisits, no
under-use. The fast estimate uses it (core/cost_estimate.live_allocation), and since pass R3 (model V6.37.0) the
exact computation (allocate_rule(), at the end); the pro-rata sharing above (allocate()) was removed then.
"""
from __future__ import annotations

import dataclasses as _dc
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import sparse as _sp
from scipy.optimize import linprog as _linprog

try:                                       # HiGHS called directly (10 November 2026): scipy 1.15's linprog reads the
    import highspy as _hc                  # whole basis back once per column after the solve (quadratic: 15 s for
    _Highs = _hc.Highs                     # 0.2 s of solve at 7800 columns); the official package if installed,
except ImportError:                        # else the copy scipy ships (same API), else linprog
    try:
        from scipy.optimize._highspy import _core as _hc
        _Highs = _hc._Highs
    except ImportError:
        _hc = _Highs = None

from . import radar_model as rm
from .params import P

SHIP_W = float(P.get("power.ship_elec_total_mw")) * 1e6
CLASS_RANK = {"KKV": 0, "SHIP": 1, "DECOY": 2, "DEBRIS": 3}         # the rest (GENERIC, ST, ...) after
TIER_NAMES = {(0, 0): "always-on", (1, 0): "P1 self-protection search", (2, 0): "P2 confirmation / initiation",
              (3, 9): "P3' burn-through", (5, 0): "P5 margin reuse"}


def tier_name(t: Tuple[int, int]) -> str:
    if t in TIER_NAMES:
        return TIER_NAMES[t]
    if t[0] == 3:
        return "P3 tracks " + next((c for c, r in CLASS_RANK.items() if r == t[1]), "other")
    return f"P4_{t[1]} search"


def _caps(res: dict) -> List[float]:
    cap_in = res.get("global", {}).get("face_elec_cap_mw")
    caps = ([float(cap_in)] * 4 if isinstance(cap_in, (int, float)) and cap_in else
            [float(c) for c in cap_in] if isinstance(cap_in, list) else [math.inf] * 4)
    return [min(rm.FACE_FEEDER_MAX_MW, c) * 1e6 for c in caps]


def items(mission: dict, res: dict) -> List[dict]:
    """Every need of the faces: (label, sector, kind, tier, W drawn per face, S and Ku transmit time per face as
    shares of the face's time: "ts", "tk", pass R3)."""
    secs = {s["id"]: s for s in mission.get("sectors", [])}
    p_ref = float(res["global"]["face_elec_avail_kW"]) * 1e3
    k_w = p_ref / rm.CHAIN_EFF
    out = []
    comp = res.get("compute", {}).get("faces", {})
    trk = res.get("tracking", {})
    tf = trk.get("faces", {})
    always = [0.0] * 4
    cal_s, cal_ku = rm.array_errors("S")["cal_time_frac"], rm.array_errors("Ku")["cal_time_frac"]   # step 3.4 (E1)
    for k in range(4):
        f = tf.get(str(k), {})
        non_tx = comp.get(str(k), {}).get("non_tx_kW_actual")
        non_tx = (rm.non_tx_power_w() / 1e3) if non_tx is None else non_tx
        always[k] = (non_tx + f.get("pumps_kW", 0.0)) * 1e3 / rm.CHAIN_EFF
    out.append({"label": "always-on", "sector": None, "kind": "always", "tier": (0, 0), "w": always,
                "ts": [cal_s] * 4, "tk": [cal_ku] * 4})
    for r in res.get("sectors", []):
        if r.get("status") != "OK":
            continue
        sec = secs.get(r["id"], {})
        tier = (1, 0) if sec.get("self_protection") else (4, int(sec.get("search_priority", 1)))
        fc = r.get("faces", {})
        w = [float(fc.get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
        ts = [float(fc.get(str(k), {}).get("time_fraction", 0.0)) for k in range(4)]
        g = float(r.get("_reuse_g", 1.0))                    # step 5.2 (P4): the revisit asked in its own tier
        pairs = [(int(key.split(">")[0]), int(key.split(">")[1]), float(v.get("fe_own_reach", 0.0)) * k_w,
                  float(v.get("ft_own_reach", 0.0)), float(v.get("fe_other", 0.0)) * k_w, float(v.get("ft_other", 0.0)))
                 for key, v in (r.get("face_overlap") or {}).items()
                 if float(v.get("sr_reach", 0.0)) > 0 and not r.get("_moved")]     # R3c: a move is not taken back
        out.append({"label": f"search {r['id']}", "sector": r["id"], "kind": "search", "tier": tier,
                    "w": [v / g for v in w], "ts": [v / g for v in ts], "tk": [0.0] * 4,
                    "pairs": [(k, j) + tuple(x / g for x in c) for k, j, *c in pairs],
                    "rr_s": float(r["rr_target_s"]) * g})
        if g > 1.0:
            out.append({"label": f"margin reuse {r['id']}", "sector": r["id"], "kind": "reuse", "tier": (5, 0),
                        "w": [v - v / g for v in w], "ts": [v - v / g for v in ts], "tk": [0.0] * 4,
                        "pairs": [(k, j) + tuple(x - x / g for x in c) for k, j, *c in pairs]})
    for t in trk.get("sectors", []):
        sec = secs.get(t["id"], {})
        cf = (t.get("confirmation") or {}).get("faces", {})
        w = [float(cf.get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
        if any(w):
            out.append({"label": f"confirmation {t['id']}", "sector": t["id"], "kind": "confirm", "tier": (2, 0),
                        "w": w, "ts": [float(cf.get(str(k), {}).get("s_time_fraction", 0.0)) for k in range(4)],
                        "tk": [float(cf.get(str(k), {}).get("ku_time_fraction", 0.0)) for k in range(4)]})
        rank = CLASS_RANK.get(sec.get("target_type") or "", len(CLASS_RANK))
        brs = sec.get("tracking", [])
        for i, a in enumerate(t.get("at", [])):
            af = a.get("faces", {})
            w = [float(af.get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
            tk = [float(af.get(str(k), {}).get("ku_time_fraction", 0.0)) for k in range(4)]
            b = brs[i] if i < len(brs) else {}
            if b.get("_reuse_h", 1.0) > 1.0:                 # step 5.2 (P7): the cadence asked in its own tier
                w0 = [min(v, float(e) * k_w) for v, e in zip(w, b["_e_nominal"])]
                tk0 = [t_ * v0 / v if v > 0 else t_ for t_, v, v0 in zip(tk, w, w0)]   # time goes as the energy
                out.append({"label": f"margin reuse {t['id']}/{i}", "sector": t["id"], "kind": "reuse",
                            "tier": (5, 0), "w": [v - v0 for v, v0 in zip(w, w0)], "ts": [0.0] * 4,
                            "tk": [t_ - t0 for t_, t0 in zip(tk, tk0)]})
                w, tk = w0, tk0
            if any(w):
                out.append({"label": f"tracks {t['id']}/{i}", "sector": t["id"], "kind": "track", "tier": (3, rank),
                            "w": w, "ts": [0.0] * 4, "tk": tk, "n_targets": float(a.get("n_targets", 0.0)),
                            "rr_s": float(b.get("rr_s", 0.0) or 0.0)})
        bt = t.get("ku_burn_through") or {}
        w = [float(bt.get("faces", {}).get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
        if any(w):
            out.append({"label": f"burn-through {t['id']}", "sector": t["id"], "kind": "burn_through",
                        "tier": (3, 9), "w": w, "ts": [0.0] * 4,
                        "tk": [float(bt.get("faces", {}).get(str(k), {}).get("ku_time_fraction", 0.0))
                               for k in range(4)]})
    return out


# ============================================================================= final allocation rule (pass R1)
# Framed with the user on 10 November 2026 ("the radar manages its resources by itself to stick as close as possible to
# what is asked, without under-use"). One rule per instant, the same for the live load, the range gauge, the exact
# optimiser and, later, the flight:
#   - resources: the ship's pool (power.ship_elec_total_mw), each face's feeder and operator cap (energy), each face's
#     S and Ku transmit time; every one under the load ceiling (hard, C2); heat, compute, memory and beams stay checks
#     of the verdict (C1);
#   - a demand (a search volume, a track bracket, a confirmation, the always-on...) is split in shares, each served by
#     one or several faces (the faces that scan it: a share in the overlap of the 75 deg cones may be moved to the
#     neighbouring face, B1, with no threshold, B2), its cost at full service known per face;
#   - the tiers are served in order (realloc_mode keeps that order, D2); within a tier, the service level m (the cost
#     multiplier) is the same for every demand as long as they compete for a resource, a demand rising above it only
#     with what the others cannot use (progressive filling: max-min, no under-use; A4);
#   - a search volume at level m < 1 loses range first, uniformly over its directions (A1, A3): rho = m^(1/4) down to
#     rho_min (A2, tool.global_opt.rho_min), then its revisit (RR / (m / rho_min^4)); a track bracket or another task
#     serves the share m of its load (tracks tenable n m); P0 to P2 not served in full: the mission is not tenable;
#   - the spare, once every tier is served, shortens the revisits (cadences) of the demands allowed to (h_max > 1: the
#     margin reuse, realloc_mode), by a common factor up to each one's floor (D1);
#   - the shares are finally served at the least cost (sum of the loads), every level fixed: a direction goes to the
#     face that scans it best unless a resource prevents it.
# The energy and the time of a share are taken proportional to its level (C1: the fast estimate's time is a lower bound
# proportional to the energy; the exact optimiser confirms, pass R3). Each step is a linear programme: with y_rk the
# level-weighted share of row r served by face k, sum_k y_rk = m(d(r)), the resources sum_r y_rk c_rkj <= cap_kj.
RHO_MIN = float(P.get("tool.global_opt.rho_min"))
LOAD_CEILING = float(P.get("tool.global_opt.load_ceiling"))
RATIO_BIN_DB = float(P.get("tool.global_opt.alloc_ratio_bin_db"))
RESOURCES = ("energy", "S time", "Ku time")
_TOL = 1e-7


@_dc.dataclass
class Demand:
    """One demand of the rule. rows (n, 4, 3): per share, per face, its cost at full service (energy W drawn from the
    ship bus, S time fraction, Ku time fraction); NaN where the face cannot serve the share."""
    label: str
    tier: Tuple[int, int]
    kind: str                         # "search" (range then revisit), "track" (tracks tenable), "other"
    rows: np.ndarray
    rr_s: float = 0.0                 # search: the revisit asked; track: the update period asked
    n_targets: float = 0.0            # track: the tracks asked
    h_max: float = 1.0                # spare: the cost may grow up to h_max (revisit / h), 1 = never
    sector: Optional[str] = None


@_dc.dataclass
class Budgets:
    ship_w: float = SHIP_W
    face_w: Sequence[float] = (rm.FACE_FEEDER_MAX_MW * 1e6,) * 4    # feeder, or the operator's cap
    time_max: float = 1.0             # transmit time of a face and band (share of the time)
    ceiling: float = LOAD_CEILING


def search_effect(m: float, rr_s: float, rho_min: float = RHO_MIN) -> Tuple[float, float]:
    """(range held / range asked, revisit tenable) of a search volume served at level m (A1, A2, D1)."""
    if m >= 1.0:
        return 1.0, rr_s / m
    if m >= rho_min ** 4:
        return m ** 0.25, rr_s
    return rho_min, (rr_s * rho_min ** 4 / m) if m > 0 else math.inf


def _merge(rows: np.ndarray, w: np.ndarray) -> Tuple[np.ndarray, float]:
    """The shares of a demand, those a single face can serve summed per face (exact: no choice), those several faces
    can serve summed when their physical cost ratios between the faces agree within RATIO_BIN_DB; the shares no face
    can serve dropped (their share returned)."""
    rows = np.asarray(rows, float).reshape(-1, 4, 3)
    ok = np.all(np.isfinite(rows), axis=2)                     # (n, 4): face k can serve share r
    n_ok = ok.sum(axis=1)
    out = []
    multi = rows[n_ok > 1]
    if len(multi):
        c = np.where(ok[n_ok > 1], np.nan_to_num(multi) @ w, np.nan)          # physical cost per face
        ref = np.nanmin(c, axis=1)
        db = 10.0 * np.log10(np.maximum(c, 1e-300) / np.maximum(ref, 1e-300)[:, None])
        key = np.where(np.isfinite(db), np.floor(db / RATIO_BIN_DB), -1).astype(np.int64)
        key = np.concatenate([ok[n_ok > 1].astype(np.int64), np.minimum(key, 10 ** 6)], axis=1)
        _u, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.reshape(-1)
        merged = np.zeros((inv.max() + 1, 4, 3))
        np.add.at(merged, inv, np.nan_to_num(multi))
        mask = np.zeros((inv.max() + 1, 4), bool)
        mask[inv] = ok[n_ok > 1]
        merged[~mask] = np.nan
        out.append(merged)
    for k in range(4):
        one = (n_ok == 1) & ok[:, k]
        if one.any():
            r = np.full((1, 4, 3), np.nan)
            r[0, k] = rows[one, k].sum(axis=0)
            out.append(r)
    return (np.concatenate(out, axis=0) if out else np.zeros((0, 4, 3))), float((n_ok == 0).sum()) / max(len(rows), 1)


class _Programme:
    """The linear programme of the rule over every demand (variables y_rk, then the common level t)."""

    def __init__(self, demands: List[Demand], b: Budgets):
        self.lost = []
        rows, row_d = [], []
        w = np.array([1.0 / b.ship_w, 1.0 / b.time_max, 1.0 / b.time_max])   # the same reference for every face
        for d, dm in enumerate(demands):
            r, lost = _merge(dm.rows, w)
            self.lost.append(lost)
            rows.append(r)
            row_d += [d] * len(r)
        rows = np.concatenate(rows, axis=0) if rows else np.zeros((0, 4, 3))
        self.row_d = np.array(row_d, int)
        rr, kk = np.nonzero(np.all(np.isfinite(rows), axis=2))
        self.var_row, self.var_face, self.cost = rr, kk, rows[rr, kk]          # cost (nv, 3)
        nv, nr = len(rr), len(rows)
        self.nv, self.nr = nv, nr
        cols = np.arange(nv)
        ub_r = np.concatenate([3 * kk + j for j in range(3)] + [np.full(nv, 12)])
        ub_c = np.concatenate([cols] * 4)
        ub_v = np.concatenate([self.cost[:, j] for j in range(3)] + [self.cost[:, 0]])
        self.A_ub = _sp.csr_matrix((ub_v, (ub_r, ub_c)), shape=(13, nv + 1))
        caps = []
        for k in range(4):
            caps += [min(float(b.face_w[k]), b.ship_w), b.time_max, b.time_max]
        self.caps = np.array(caps + [b.ship_w], float)
        self.b_ub = self.caps * b.ceiling
        self.A_y = _sp.csr_matrix((np.ones(nv), (rr, cols)), shape=(nr, nv))
        self.c_cost = self.cost @ w                                           # physical cost (least cost, B2)

    def solve(self, level: np.ndarray, free: Sequence[int], lo: float, hi: float, least_cost: bool = False):
        """max t (or the least cost, t fixed), the demands in free at level t in [lo, hi], the others at level[d]."""
        fr = np.isin(self.row_d, list(free)) if len(free) else np.zeros(self.nr, bool)
        t_col = _sp.csr_matrix(-fr.astype(float)[:, None])
        A_eq = _sp.hstack([self.A_y, t_col], format="csr")
        b_eq = np.where(fr, 0.0, level[self.row_d]) if self.nr else np.zeros(0)
        c = np.zeros(self.nv + 1)
        if least_cost:
            c[:-1] = self.c_cost
        else:
            c[-1] = -1.0
        t_lo, t_hi = (lo, hi) if len(free) else (0.0, 0.0)
        return _lp(c, self.A_ub, self.b_ub, A_eq, b_eq, self.nv, t_lo, t_hi)

    def can_rise(self, x: np.ndarray, d: int) -> bool:
        """Sufficient test (no programme): every share of d has a face with room on every resource the share uses
        (and the ship has room if it uses energy), at the solution x."""
        use = self.A_ub @ x
        tight = use >= self.b_ub * (1.0 - 1e-6) - 1e-12
        rows = np.nonzero(self.row_d == d)[0]
        if not len(rows):
            return False
        v = np.isin(self.var_row, rows)
        c = self.cost[v]
        room = ~((c[:, 0] > 0) & tight[3 * self.var_face[v]]) & ~((c[:, 1] > 0) & tight[3 * self.var_face[v] + 1])             & ~((c[:, 2] > 0) & tight[3 * self.var_face[v] + 2]) & ~((c[:, 0] > 0) & tight[12])
        has = np.zeros(self.nr, bool)
        has[self.var_row[v][room]] = True
        return bool(has[rows].all())


class _Result:
    def __init__(self, x: np.ndarray):
        self.x = x


def _lp(c, A_ub, b_ub, A_eq, b_eq, nv: int, t_lo: float, t_hi: float) -> Optional[_Result]:
    """min c x, A_ub x <= b_ub, A_eq x = b_eq, y >= 0 (nv columns), t_lo <= t <= t_hi (the last column); None when
    no optimum."""
    lb = np.zeros(nv + 1)
    lb[-1] = t_lo
    if _Highs is None:
        res = _linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq if A_eq.shape[0] else None,
                       b_eq=b_eq if A_eq.shape[0] else None,
                       bounds=[(0, None)] * nv + [(t_lo, t_hi)], method="highs")
        return _Result(res.x) if res.status == 0 else None
    inf = _hc.kHighsInf
    A = _sp.vstack([A_ub, A_eq], format="csc")
    lp = _hc.HighsLp()
    lp.num_col_, lp.num_row_ = A.shape[1], A.shape[0]
    lp.col_cost_ = np.asarray(c, float)
    lp.col_lower_ = lb
    ub = np.full(nv + 1, inf)
    ub[-1] = t_hi
    lp.col_upper_ = ub
    lp.row_lower_ = np.concatenate([np.full(A_ub.shape[0], -inf), b_eq])
    lp.row_upper_ = np.concatenate([np.asarray(b_ub, float), b_eq])
    lp.a_matrix_.format_ = _hc.MatrixFormat.kColwise
    lp.a_matrix_.start_ = A.indptr
    lp.a_matrix_.index_ = A.indices
    lp.a_matrix_.value_ = A.data
    h = _Highs()
    h.setOptionValue("output_flag", False)
    h.passModel(lp)
    h.run()
    if h.getModelStatus() != _hc.HighsModelStatus.kOptimal:
        return None
    return _Result(np.array(h.getSolution().col_value))


def _fill(prog: _Programme, level: np.ndarray, members: List[int], lo: float, hi: Dict[int, float]) -> None:
    """Progressive filling of members from lo: a common level as high as the resources allow, the demands that cannot
    rise fixed there, the others rising on (level updated in place)."""
    free = [d for d in members if hi[d] > lo + _TOL]
    for d in members:
        if d not in free:
            level[d] = min(hi[d], max(level[d], lo))
    while free:
        top = min(hi[d] for d in free)
        res = prog.solve(level, free, lo, top)
        if res is None:                                       # (numerical: the floor itself not held)
            for d in free:
                level[d] = lo
            return
        t = float(res.x[-1])
        done = [d for d in free if hi[d] <= t + _TOL]         # at their own ceiling
        if not done:
            for d in free:                                    # the demands that cannot rise above t
                level[d] = t
            for d in free:
                if prog.can_rise(res.x, d):
                    continue
                others = [e for e in free if e != d]
                for e in others:
                    level[e] = t * (1.0 - 1e-9)
                r = prog.solve(level, [d], t, hi[d])
                if r is None or float(r.x[-1]) <= t * (1.0 + 1e-6) + _TOL:
                    done.append(d)
            if not done:
                done = list(free)
        for d in done:                                        # (just below t: the next programmes stay feasible)
            level[d] = t * (1.0 - 1e-9)
        free = [d for d in free if d not in done]
        for d in free:
            level[d] = t * (1.0 - 1e-9)
        lo = t * (1.0 - 1e-9)


def solve(demands: List[Demand], budgets: Optional[Budgets] = None, rho_min: float = RHO_MIN) -> dict:
    """The final allocation rule (pass R1) at one instant: every demand's level, its effect (range held, revisit or
    tracks tenable) and its split over the faces; the faces' and the ship's loads; the binding resources; the flags."""
    b = budgets or Budgets()
    prog = _Programme(demands, b)
    n = len(demands)
    level = np.zeros(n)
    for t in sorted({dm.tier for dm in demands}):
        members = [d for d in range(n) if demands[d].tier == t]
        _fill(prog, level, members, 0.0, {d: 1.0 for d in members})
    spare = [d for d in range(n) if demands[d].h_max > 1.0 and level[d] >= 1.0 - 1e-6]
    if spare:
        _fill(prog, level, spare, min(level[d] for d in spare), {d: float(demands[d].h_max) for d in spare})
    res = prog.solve(level * (1.0 - 1e-9), [], 0.0, 0.0, least_cost=True)
    out = _report(demands, b, prog, level, res.x[:-1] if res is not None else np.zeros(prog.nv), rho_min)
    if res is None:
        out["status"] = "FAILED"
        out["flags"].insert(0, "allocation programme failed: no split over the faces found")
    return out


def _report(demands, b, prog, level, y, rho_min) -> dict:
    n = len(demands)
    use = np.zeros((4, 3))
    per = np.zeros((n, 4, 3))
    moved_e = np.zeros((n, 3))                                # served by a face other than the cheapest
    moved_f = np.zeros((4, 3))                                # (the same, per serving face)
    d_of = prog.row_d[prog.var_row]
    np.add.at(use, (prog.var_face,), y[:, None] * prog.cost)
    np.add.at(per, (d_of, prog.var_face), y[:, None] * prog.cost)
    if prog.nv:
        c_row = np.full((prog.nr, 4), np.inf)
        c_row[prog.var_row, prog.var_face] = prog.c_cost
        moved = prog.c_cost > c_row.min(axis=1)[prog.var_row] * (1.0 + 1e-6)   # (ties: not moved)
        np.add.at(moved_e, d_of[moved], y[moved, None] * prog.cost[moved])
        np.add.at(moved_f, prog.var_face[moved], y[moved, None] * prog.cost[moved])
    flags: List[str] = []
    out_d = []
    for d, dm in enumerate(demands):
        m = float(min(level[d], dm.h_max))
        for r in (1.0, dm.h_max):                             # (the levels are kept just below t)
            m = r if abs(m - r) < 1e-6 * r else m
        tot = per[d].sum(axis=0)
        j = int(np.argmax(tot / np.maximum(prog.caps[:3], 1e-30))) if tot.any() else 0   # its main resource
        rec = {"label": dm.label, "sector": dm.sector, "tier": tier_name(dm.tier), "kind": dm.kind, "level": m,
               "unserved_share": prog.lost[d],
               "moved_share": float(moved_e[d, j] / tot[j]) if tot[j] > 0 else 0.0,
               "faces": {str(k): {"share": float(per[d, k, j] / tot[j]) if tot[j] > 0 else 0.0,
                                  **{RESOURCES[i]: float(per[d, k, i]) for i in range(3)}}
                         for k in range(4) if per[d, k].any()}}
        if dm.kind == "search":
            rho, rr = search_effect(m, dm.rr_s, rho_min)
            rec.update(rho=rho, rr_tenable_s=rr)
            if m < 1.0:
                flags.append(f"{dm.label}: range held {100 * rho:.0f} % of the range asked"
                             + (f", revisit {rr:.3g} s (asked {dm.rr_s:g} s)" if rr > dm.rr_s * (1 + 1e-9) else ""))
        elif dm.kind == "track":
            rec.update(tracks_tenable=dm.n_targets * min(m, 1.0),
                       period_tenable_s=dm.rr_s / m if m > 1.0 else dm.rr_s)
            if m < 1.0:
                flags.append(f"{dm.label}: {dm.n_targets * m:.3g} tracks tenable of {dm.n_targets:g}")
        elif m < 1.0:
            flags.append(f"{dm.label}: only {100 * m:.0f} % served")
        if dm.tier[0] <= 2 and m < 1.0 - 1e-6:
            flags.append(f"mission not tenable: {dm.label} ({tier_name(dm.tier)}) is never shed")
        if prog.lost[d] > 0:
            flags.append(f"{dm.label}: {100 * prog.lost[d]:.1f} % of its shares scanned by no face")
        out_d.append(rec)
    caps = prog.caps.reshape(-1)
    faces = {}
    binding = []
    for k in range(4):
        f = {}
        for j, name in enumerate(RESOURCES):
            cap = caps[3 * k + j]
            f[name] = float(use[k, j])
            f[name + " load"] = float(use[k, j] / cap) if cap > 0 else 0.0
            if cap > 0 and use[k, j] >= cap * b.ceiling * (1.0 - 1e-5):
                binding.append(f"F{k + 1} {name}")
        f["energy cap"] = float(caps[3 * k])
        f["moved"] = {RESOURCES[j]: float(moved_f[k, j]) for j in range(3)}   # serving shares of a neighbour
        faces[str(k)] = f
    ship_use = float(use[:, 0].sum())
    if ship_use >= b.ship_w * b.ceiling * (1.0 - 1e-5):
        binding.append("ship energy")
    status = ("NOT_TENABLE" if any(f.startswith("mission not tenable") for f in flags)
              else "DEGRADED" if any(r["level"] < 1.0 - 1e-6 for r in out_d) else "OK")
    return {"status": status, "demands": out_d, "faces": faces,
            "ship": {"energy": ship_use, "energy load": ship_use / b.ship_w, "cap": b.ship_w},
            "ceiling": b.ceiling, "binding": binding, "flags": flags}


# ============================================================================= the rule in the exact computation (R3)
# Pass R3a (user, 10 November 2026): the exact optimiser's power allocation is the final rule, in place of the
# pro-rata sharing (allocate(), removed in pass R3e).
# The demands are the exact plans' needs (items(): energy drawn, S time, Ku time per face, the optimiser's real times,
# not the fast estimate's lower bound); a search share in the overlap of two scan cones may be served by either face
# (pass R3b, _search_rows), the rule only reporting the move until pass R3c re-plans it on its new face; the spare stays with the margin reuse (core/margin.py, decision 2 of R3). The verdict rows (user, 10
# November 2026): a face's energy load = the draw its plan asks at full service over its own limit (feeder, or the
# operator's cap face_elec_mw), the ship's = the total asked over the ship's pool; what the rule actually serves is in
# "partial" (level, range held rho, revisit or tracks tenable).
_KIND = {"search": "search", "track": "track"}


def _search_rows(it: dict) -> List[np.ndarray]:
    """Pass R3b (B1, decision 1 of R3, user 10 November 2026): the shares of a search volume. The part of face k's
    load that a neighbour j also reaches under its scan limit (optimizer.face_overlap, plan kept: the plan's own cost
    where it has the scan bin, else its nearest waveform scaled by range_at) is a share either face may serve, at its
    cost on k (fe_own_reach, ft_own_reach) or on j (fe_other, ft_other); the rest of k's load stays on k. A direction
    two neighbours reach is in both pairs: the pairs of a face are scaled down so that they never exceed its load."""
    rows = []
    for k in range(4):
        own = np.array([float(it["w"][k]), float(it["ts"][k]), float(it["tk"][k])])
        pairs = [p_ for p_ in it.get("pairs", []) if p_[0] == k]
        if not own.any():
            continue
        take = np.array([[p_[2], p_[3], 0.0] for p_ in pairs]).reshape(-1, 3)
        tot = take.sum(axis=0)
        f = float(min(1.0, *(own[i] / tot[i] for i in range(3) if tot[i] > 0))) if tot.any() else 1.0
        for p_, c in zip(pairs, take):
            r = np.full((4, 3), np.nan)
            r[k] = c * f
            r[p_[1]] = (p_[4] * f, p_[5] * f, 0.0)
            if np.nansum(r[k]) > 0:
                rows.append(r)
        rest = np.maximum(own - tot * f, 0.0)
        if rest.any():
            r = np.full((4, 3), np.nan)
            r[k] = rest
            rows.append(r)
    return rows


def exact_demands(mission: dict, res: dict) -> Tuple[List[Demand], Budgets, List[dict]]:
    """(demands, budgets, the items they come from) of an assembled exact result: a search volume's shares in the
    overlaps of the scan cones may move to the neighbouring face (pass R3b), the other needs stay on their face."""
    its = items(mission, res)
    demands = []
    for it in its:
        if it.get("pairs"):
            rows = _search_rows(it)
        else:
            rows = []
            for k in range(4):
                c = (float(it["w"][k]), float(it.get("ts", [0.0] * 4)[k]), float(it.get("tk", [0.0] * 4)[k]))
                if any(v > 0 for v in c):
                    r = np.full((4, 3), np.nan)
                    r[k] = c
                    rows.append(r)
        demands.append(Demand(it["label"], it["tier"], _KIND.get(it["kind"], "other"),
                              np.array(rows) if rows else np.zeros((0, 4, 3)), rr_s=float(it.get("rr_s", 0.0)),
                              n_targets=float(it.get("n_targets", 0.0)), sector=it["sector"]))
    ceiling = float(mission.get("global", {}).get("load_ceiling", LOAD_CEILING))
    return demands, Budgets(ship_w=SHIP_W, face_w=_caps(res), time_max=1.0, ceiling=ceiling), its


def allocate_rule(mission: dict, res: dict) -> dict:
    """The final rule on an assembled exact result (pass R3a), in the keys the verdict, the margin reuse and the
    report read (those of the former pro-rata allocate()), the rule's own report under "rule"."""
    demands, b, its = exact_demands(mission, res)
    rep = solve(demands, b)
    need = [sum(it["w"][k] for it in its) for k in range(4)]
    caps = [min(float(b.face_w[k]), b.ship_w) for k in range(4)]
    tiers_out = []
    for t in sorted({it["tier"] for it in its}):
        ds = [(it, rec) for it, rec in zip(its, rep["demands"]) if it["tier"] == t]
        w = sum(sum(it["w"]) for it, _ in ds)
        served = sum(sum(it["w"]) * min(1.0, rec["level"]) for it, rec in ds)
        tiers_out.append({"tier": tier_name(t), "need_MW": w / 1e6, "served_MW": served / 1e6,
                          "fraction": served / w if w > 0 else 1.0})
    partial = []
    by_id = {r["id"]: r for r in res.get("sectors", [])}
    for it, rec in zip(its, rep["demands"]):
        r = by_id.get(rec["sector"]) if rec["kind"] == "search" else None
        planned = r is not None and "_rho" in r and it["kind"] == "search"      # pass R3c: re-planned at rho x R
        if rec["level"] >= 1.0 - 1e-6 and not planned:
            continue
        p_ = {"label": rec["label"], "tier": rec["tier"], "served_fraction": float(rec["level"])}
        if planned:                             # against what was asked: the plan's own cut, then the rule's level,
            f = float(r["_rho"]) ** 4 * float(r["_rr_asked_s"]) / float(r["_rr_tenable_s"])
            m_tot = float(rec["level"]) * f     # composed as ONE level on the request (the range never below rho_min,
            rho, rr = search_effect(m_tot, float(r["_rr_asked_s"]))   # the revisit giving way beyond: N3 10 Nov)
            p_.update(served_fraction=m_tot, rho=rho, rr_tenable_s=rr, re_planned=True)
        elif rec["kind"] == "search":
            p_.update(rho=rec["rho"], rr_tenable_s=rec["rr_tenable_s"])
        elif rec["kind"] == "track":
            p_["tracks_tenable"] = rec["tracks_tenable"]
        partial.append(p_)
    faces = {str(k): {"need_MW": need[k] / 1e6, "cap_MW": caps[k] / 1e6,
                      "capped": caps[k] < b.ship_w * (1.0 - 1e-9),          # an operator cap below the ship's pool
                      "alloc_MW": rep["faces"][str(k)]["energy"] / 1e6,    # what the rule serves (W drawn)
                      "always_on_MW": its[0]["w"][k] / 1e6, "load": need[k] / caps[k] if caps[k] > 0 else math.inf}
             for k in range(4)}
    return {"ship_MW": b.ship_w / 1e6, "need_MW": sum(need) / 1e6, "drawn_MW": rep["ship"]["energy"] / 1e6,
            "status": "OK" if rep["status"] == "OK" else "SHORT", "faces": faces, "tiers": tiers_out,
            "partial": partial, "flags": list(rep["flags"]), "rule": rep}


_EXTENSIVE = ("positions", "energy_J_per_refresh", "dissipation_kW", "dissipation_kW_m2", "avg_power_kW")


def moved_results(res: dict) -> Tuple[List[dict], List[dict]]:
    """Pass R3c (user, 10 November 2026: moves at plan kept): the sector results with the moves of the rule applied,
    and the moves. A search volume whose shares the rule serves on a neighbouring face gets its face totals from the
    rule's split (energy and S time per face at full service: the served ones over the level), the other extensive
    figures (positions, energy per refresh, dissipation, mean power) leaving the faces that lose load in proportion
    of the energy they lose and going to the faces that gain in proportion of the energy they gain; a face that had
    none of the volume gets the scan limit as its largest scan angle. Marked "_moved" (no move option any more: the
    rule does not take it back), the plan's own totals under "_faces_planned". The exact re-planning of the moved
    directions is the flight's (step 6.5)."""
    rule = (res.get("power_allocation") or {}).get("rule") or {}
    p_ref = float(res["global"]["face_elec_avail_kW"]) * 1e3
    k_w = p_ref / rm.CHAIN_EFF
    by_id = {r["id"]: r for r in res.get("sectors", [])}
    moved: Dict[str, dict] = {}
    moves = []
    for rec in rule.get("demands", []):
        r = by_id.get(rec.get("sector"))
        m = float(rec.get("level", 0.0))
        if rec.get("kind") != "search" or r is None or r.get("_moved") or r.get("status") != "OK" or m <= 0:
            continue
        g = float(r.get("_reuse_g", 1.0))
        old = r.get("faces", {})
        e_old = {int(k): float(v.get("energy_fraction", 0.0)) for k, v in old.items()}
        e_new = {int(k): float(v.get("energy", 0.0)) / m * g / k_w for k, v in rec.get("faces", {}).items()}
        t_new = {int(k): float(v.get("S time", 0.0)) / m * g for k, v in rec.get("faces", {}).items()}
        tot = sum(e_old.values())
        if tot <= 0 or all(abs(e_new.get(k, 0.0) - e_old.get(k, 0.0)) <= 1e-6 * tot for k in range(4)):
            continue
        lose = {k: max(0.0, 1.0 - e_new.get(k, 0.0) / e) for k, e in e_old.items() if e > 0}
        gain = {k: max(0.0, e_new.get(k, 0.0) - e_old.get(k, 0.0)) for k in range(4)}
        g_tot = sum(gain.values())
        faces = {}
        for k in range(4):
            if e_new.get(k, 0.0) <= 0 and t_new.get(k, 0.0) <= 0:
                continue
            f = dict(old.get(str(k), {}))
            for key in _EXTENSIVE:
                kept = float(f.get(key, 0.0)) * (1.0 - lose.get(k, 0.0))
                given = sum(float(old[str(i)].get(key, 0.0)) * lose[i] for i in lose)
                f[key] = kept + (given * gain[k] / g_tot if g_tot > 0 else 0.0)
            f["energy_fraction"], f["time_fraction"] = e_new.get(k, 0.0), t_new.get(k, 0.0)
            if "theta_max_deg" not in f:
                f_ghz = float(r.get("frequency", {}).get("freq_ghz", 0.0))
                f["theta_max_deg"] = float(rm.band_for_frequency(f_ghz).scan_limit_deg(f_ghz)) if f_ghz else None
            faces[str(k)] = f
        moved[r["id"]] = dict(r, faces=faces, _moved=True, _faces_planned=old)
        moves.append({"sector": r["id"], "moved_share": float(rec.get("moved_share", 0.0)),
                      "energy_fraction": {str(k): (e_old.get(k, 0.0), e_new.get(k, 0.0)) for k in range(4)
                                          if e_old.get(k, 0.0) or e_new.get(k, 0.0)}})
    return [moved.get(r["id"], r) for r in res.get("sectors", [])], moves
