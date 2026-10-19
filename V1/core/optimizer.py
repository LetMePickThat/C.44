#!/usr/bin/env python3
"""
OPTIMIZER (core/optimizer.py, ex mission_optimizer.py) V1.0 -- pre-mission waveform / budget optimiser for the S-band surveillance (search) function.

Input  : mission file (JSON or CSV) = global parameters + any number of watch sectors.
Output : per sector -> suggested WF, achieved range, RR estimate, radar budget (time, energy, average power, per face);
         mission totals per face; infeasibility flags with the missing power.

Uses core/radar_model.py (radar equation, losses, lattice, face geometry). Ship frame: Z = nose axis (+Z radar faces),
phi = azimuth about Z from +X toward +Y, eps = elevation from XY plane toward +Z. Faces at phi = 0/90/180/270, tilt 20 deg.

Design rules (flagged [D] = decision, change on request) -- optimiser v2 (AUDIT/SPEC_optimiseur_v2.md,
with the audit corrections):
  - [D] One waveform per BIN_DEG scan-angle bin (at most max_wf_per_sector distinct waveforms per sector, neighbouring
        bins merged by dynamic programming), each reaching the requested range in every 1-deg bin it covers; power is
        then scaled down per 1-deg bin to just reach the range ("no excess energy").
  - [D] Objective = min max(energy fraction, time-on-air fraction) over the faces, via an alpha-swept scalarisation.
        Ranking: extra unscanned fraction caused by the frequency, then positions short of range (1 % tolerance),
        then the objective.
  - Range vs scan angle: in the model R(theta) = R(0) h(f, theta) with h independent of the waveform (checked at
    run time), so each candidate costs one solve() per frequency.
  - [D] Unambiguous range imposed on PRI (PRI >= 2 R / c) unless sector says "allow_ambiguous": true.
  - [D] Migration evaluated at v_close_max_mps; Pd/Pfa/loss budget are the model's locked values.
  - [D] Sectors are independent; overlapping sectors are budgeted twice (flagged). A sector spanning several
        faces is budgeted per face (each face has its own 2 MW).
  - [D] RR is stretched proportionally when a face is over-subscribed:
        RR_est = RR_target x max(1, sum of energy fractions, sum of time fractions).
  - Tracking margin per face: the only S/Ku coupling is the face's electrical power, so the power left after search,
    (1 - sum of energy fractions) x P_avg, is reported with the equivalent Ku duty cycle (Ku peak 9.19 MW).

Performance:
  - solve() range results are memoised per process (_range_cached), so repeated runs (e.g. re-running from the GUI
    after editing one sector) and sectors sharing a threat (same RCS / Swerling / speed / range) reuse them.
  - Sectors are optimised in parallel worker PROCESSES (not threads: the search is pure-Python CPU work, and threads
    would serialise on the GIL) once the mission has PARALLEL_MIN_SECTORS sectors or more. The pool is persistent
    and lazily started, so its start-up cost (~1-2 s on Windows) is paid once per session, and each worker keeps
    its own solve() cache and sphere raster between runs.
"""
import csv
import functools
import json
import math
import os
import sys
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from concurrent.futures.process import BrokenProcessPool
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import radar_model as rm

FACE_PHI = (0.0, 90.0, 180.0, 270.0)
D_RES_DEG = 0.2                 # raster step (deg) on the (phi, eps) sphere
FREQ_CANDIDATES = (2.7, 2.8, 2.9)
TAU_CAND = (20e-6, 50e-6, 100e-6, 200e-6, 400e-6, 600e-6, 800e-6, 937.5e-6)
N_CAND = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32)
B_CAND = (1, 2, 4, 8)
SIGBW_CAND = (10.0, 20.0, 30.0, 50.0, 75.0)
RES_MAX_M_DEFAULT = 2.0         # [D] range resolution requested if the sector gives none (-> 75 MHz)
THETA_BIN_DEG = 1.0
C = rm.C_MPS
SOLVE_CACHE_SIZE = 1 << 18      # memoised solve() ranges per process (~50 MB when full)
PARALLEL_MIN_SECTORS = 6        # [D] below this, a sequential run (~0.2 s/sector) beats process start-up
MAX_WORKERS = max(1, (os.cpu_count() or 2) // 2)   # ~physical cores; each worker holds a ~65 MB sphere raster
BIN_DEG = 5                     # [D] scan-angle bins (in 1-deg bins) sharing one waveform
MAX_WF_PER_SECTOR = 4           # [D] default cap on distinct waveforms per sector (sector key "max_wf_per_sector")
SHORT_TOL = 0.01                # [D] positions-short-of-range fractions within 1 % rank equal (sector key "short_tol")
ALPHAS = np.linspace(0.0, 1.0, 21)   # energy weight of the scalarised energy/time cost
SHORT_PENALTY = 1e6             # DP penalty per fraction of positions short of range (ranks range before cost)
TRACK_MARGIN_WARN = 0.10        # [D] warn when a face keeps less than this fraction of its power for tracking


# ----------------------------------------------------------------------------- geometry
class Sphere:
    """Raster of the sphere with, for each direction, its owner face (responsibility), w = cos(theta), theta."""

    def __init__(self, eps_min: float, eps_max: float):
        n_phi, n_eps = int(360 / D_RES_DEG), int(180 / D_RES_DEG)
        phi = (np.arange(n_phi) + 0.5) * D_RES_DEG                     # 0..360
        eps = -90.0 + (np.arange(n_eps) + 0.5) * D_RES_DEG
        self.phi, self.eps = np.meshgrid(phi, eps, indexing="ij")
        self.d_omega = np.radians(D_RES_DEG) ** 2 * np.cos(np.radians(self.eps))
        p, e = np.radians(self.phi), np.radians(self.eps)
        dx, dy, dz = np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)
        t = math.radians(rm.SECTOR_TILT_DEG)
        sin_lim = math.sin(math.radians(rm.SECTOR_RADIUS_DEG))
        self.owner = -np.ones(self.phi.shape, dtype=np.int8)
        self.w = np.zeros(self.phi.shape)
        best_w = np.full(self.phi.shape, -1.0)
        for k, pk in enumerate(FACE_PHI):
            c, s = math.cos(math.radians(pk)), math.sin(math.radians(pk))
            n = (math.cos(t) * c, math.cos(t) * s, math.sin(t))
            eu = (-s, c, 0.0)
            ev = (-math.sin(t) * c, -math.sin(t) * s, math.cos(t))
            w = dx * n[0] + dy * n[1] + dz * n[2]
            u = dx * eu[0] + dy * eu[1]
            v = dx * ev[0] + dy * ev[1] + dz * ev[2]
            ok = (w > 0) & (np.abs(u) <= w * math.cos(t) - v * math.sin(t) + 1e-12) & \
                 (np.sqrt(u * u + v * v) <= sin_lim) & (w > best_w)
            self.owner[ok] = k
            self.w[ok] = w[ok]
            best_w[ok] = w[ok]
        self.theta = np.degrees(np.arccos(np.clip(self.w, 0.0, 1.0)))
        self.scanned = (self.owner >= 0) & (self.eps >= eps_min) & (self.eps <= eps_max)

    def mask(self, sec: dict) -> np.ndarray:
        shape = sec.get("shape", "box")
        if shape == "cap":
            p0, e0, rad = math.radians(sec["phi0_deg"]), math.radians(sec["eps0_deg"]), sec["radius_deg"]
            p, e = np.radians(self.phi), np.radians(self.eps)
            cosang = np.sin(e) * math.sin(e0) + np.cos(e) * math.cos(e0) * np.cos(p - p0)
            return np.degrees(np.arccos(np.clip(cosang, -1, 1))) <= rad
        p1, p2 = sec["phi_min_deg"] % 360.0, sec["phi_max_deg"] % 360.0
        if sec["phi_max_deg"] - sec["phi_min_deg"] >= 360.0:
            mphi = np.ones(self.phi.shape, bool)
        elif p1 <= p2:
            mphi = (self.phi >= p1) & (self.phi <= p2)
        else:
            mphi = (self.phi >= p1) | (self.phi <= p2)
        return mphi & (self.eps >= sec["eps_min_deg"]) & (self.eps <= sec["eps_max_deg"])


def positions_table(sph: Sphere, m: np.ndarray, f_ghz: float) -> Tuple[Dict[int, np.ndarray], float]:
    """Beam positions per face and per 1-deg scan bin inside mask m. Also the unscanned sector fraction (solid angle)."""
    cell = rm.beam_cell_area_uv(f_ghz)
    lim = rm.band_for_frequency(f_ghz).scan_limit_deg(f_ghz)
    nb = int(math.ceil(rm.SECTOR_RADIUS_DEG / THETA_BIN_DEG)) + 1
    out = {k: np.zeros(nb) for k in range(4)}
    tot_om = float((sph.d_omega * m).sum())
    sel = m & sph.scanned & (sph.theta <= lim)
    for k in range(4):
        s = sel & (sph.owner == k)
        if not s.any():
            continue
        wgt = sph.w[s] * sph.d_omega[s] / cell
        idx = np.minimum((sph.theta[s] / THETA_BIN_DEG).astype(int), nb - 1)
        np.add.at(out[k], idx, wgt)
    scanned_om = float((sph.d_omega * sel).sum())
    return out, (1.0 - scanned_om / tot_om) if tot_om > 0 else 1.0


# ----------------------------------------------------------------------------- optimiser
def make_wf(name, tau, n, pri, bw, b, integ):
    return rm.Waveform(name=name, role="SEARCH", band="S", tau_s=tau, n=n, pri_s=pri, sig_bw_mhz=bw,
                       b_tx=b, integration=integ)


@functools.lru_cache(maxsize=SOLVE_CACHE_SIZE)
def _range_cached(w, f: float, theta: float, rcs_m2: float, swerling: int, v_close_mps: float) -> Optional[float]:
    """Keyed on everything the range depends on (Waveform is a frozen dataclass; the target name is irrelevant).
    The model's module constants are assumed fixed for the life of the process."""
    try:
        tgt = rm.Target("cache", rcs_m2, swerling, v_close_mps)
        return rm.solve(w, f, theta, tgt, rm.FACE_ELEC_POWER_MW).range_km
    except ValueError:
        return None


def range_at(w, f, theta, tgt):
    return _range_cached(w, f, theta, tgt.rcs_m2, tgt.swerling, tgt.v_close_mps)


def candidate_waveforms(sec: dict) -> List:
    """Search-space enumeration (same grid and PRI rules as v1)."""
    r_want = sec["range_km"]
    res_max = sec.get("res_max_m", RES_MAX_M_DEFAULT)
    bws = [b for b in SIGBW_CAND if C / (2 * b * 1e6) <= res_max + 1e-9] or [max(SIGBW_CAND)]
    pri_unamb = 2 * r_want * 1e3 / C
    out = []
    for integ in ("COHERENT", "NONCOHERENT"):
        for b in B_CAND:
            for bw in bws:
                for n in N_CAND:
                    for tau in TAU_CAND:
                        pri = max(tau, pri_unamb) if not sec.get("allow_ambiguous") else max(tau, 1e-3)
                        if n == 1:
                            pri = max(tau, 1e-3)
                        try:
                            out.append(make_wf("c", tau, n, pri, bw, b, integ))
                        except ValueError:
                            continue
    return out


def _theta_centres(f: float, nb: int) -> np.ndarray:
    """Evaluation angle of each 1-deg scan bin: its centre, clamped to the scan limit at f."""
    lim = rm.band_for_frequency(f).scan_limit_deg(f)
    return np.minimum((np.arange(nb) + 0.5) * THETA_BIN_DEG, min(lim, rm.SECTOR_RADIUS_DEG))


@functools.lru_cache(maxsize=64)
def _scan_factor(f: float, nb: int) -> Optional[np.ndarray]:
    """h(f, theta_i) = R(theta_i) / R(0) at the 1-deg bin centres. In the model the scan angle only enters the range
    through the array gain and the scan loss, so h does not depend on the waveform, and one solve() per candidate (at
    theta = 0) gives its range in every bin. Checked here on two dissimilar waveforms; None if a model change ever
    breaks the property (callers then solve() every bin)."""
    th = _theta_centres(f, nb)
    tgt = rm.Target("ref", 1.0, 1, 5000.0)
    hs = []
    for w in (make_wf("ref1", 50e-6, 1, 1e-3, 75.0, 1, "NONCOHERENT"), make_wf("ref2", 400e-6, 12, 4e-3, 10.0, 4, "COHERENT")):
        r0 = rm.solve(w, f, 0.0, tgt).range_km
        hs.append(np.array([rm.solve(w, f, float(t), tgt).range_km / r0 for t in th]))
    return hs[0] if np.allclose(hs[0], hs[1], rtol=1e-9, atol=0.0) else None


def _range_matrix(cands: List, f: float, tgt, nb: int) -> np.ndarray:
    """ranges[c, i] (km, full peak power) of candidate c in 1-deg bin i; NaN where the waveform is not playable."""
    r = np.full((len(cands), nb), np.nan)
    h = _scan_factor(f, nb)
    th = _theta_centres(f, nb)
    for j, w in enumerate(cands):
        if h is not None:
            r0 = range_at(w, f, 0.0, tgt)
            if r0 is not None:
                r[j] = r0 * h
        else:
            for i, t in enumerate(th):
                v = range_at(w, f, float(t), tgt)
                if v is not None:
                    r[j, i] = v
    return r


class _Plan:
    """Waveform assignment for one sector at one frequency: candidate index per 1-deg bin (-1 = no position)."""

    def __init__(self, choice, P, r, e_pos, t_pos, r_want, rr, p_avg_max_w):
        self.choice = choice
        idx = np.flatnonzero(choice >= 0)
        rc = r[choice[idx], idx]                                   # range of the chosen WF in each populated bin
        s = np.minimum(1.0, (r_want / rc) ** 4)                    # power scale: "no excess energy"
        self.e_bin = np.zeros(len(choice))                         # J per position
        self.t_bin = np.zeros(len(choice))                         # s of TX time per position
        self.r_bin = np.full(len(choice), np.nan)
        self.s_bin = np.ones(len(choice))
        self.e_bin[idx] = e_pos[choice[idx]] * s
        self.t_bin[idx] = t_pos[choice[idx]]
        self.r_bin[idx] = rc
        self.s_bin[idx] = s
        self.fe = (P * self.e_bin).sum(axis=1) / (p_avg_max_w * rr)    # per face
        self.ft = (P * self.t_bin).sum(axis=1) / rr
        p_tot = P.sum(axis=0)
        self.short_pos = float(p_tot[idx][rc < r_want].sum())
        self.short_frac = self.short_pos / float(p_tot.sum())
        self.objective = float(max(self.fe.max(), self.ft.max()))


def _plan_waveforms(P, r, e_pos, t_pos, r_want, rr, p_avg_max_w, max_wf, short_tol) -> Optional[_Plan]:
    """Best waveform plan at one frequency. Scan bins are grouped by BIN_DEG; each group gets one waveform, and
    neighbouring groups may share one so that at most max_wf distinct waveforms are used (dynamic programming over
    contiguous segments). Within a segment, a candidate is admissible if it reaches r_want in every populated bin;
    with none admissible, the candidate with the fewest positions short of range is used at full power. The
    energy/time trade-off is scalarised (alpha E + (1 - alpha) T) and alpha is swept to minimise max(E, T)."""
    p_tot = P.sum(axis=0)
    nb = len(p_tot)
    groups = [np.arange(lo, min(lo + BIN_DEG, nb)) for lo in range(0, nb, BIN_DEG)]
    groups = [g[p_tot[g] > 0] for g in groups]
    groups = [g for g in groups if len(g)]
    if not groups:
        return None
    n_g = len(groups)
    playable = ~np.isnan(r)
    with np.errstate(invalid="ignore", divide="ignore"):
        s = np.where(playable, np.minimum(1.0, (r_want / r) ** 4), 1.0)
        reach = playable & (r >= r_want)
    e_cost = e_pos[:, None] * s * p_tot[None, :] / (p_avg_max_w * rr)     # energy fraction contributed per bin
    t_cost = t_pos[:, None] * p_tot[None, :] / rr                          # time fraction contributed per bin
    short_p = np.where(reach, 0.0, p_tot[None, :])
    # per-segment aggregates, segment = groups a..b
    seg = {}
    for a in range(n_g):
        bins = np.concatenate(groups[a:a + 1])
        e_s, t_s = e_cost[:, bins].sum(1), t_cost[:, bins].sum(1)
        adm, play, sh = reach[:, bins].all(1), playable[:, bins].all(1), short_p[:, bins].sum(1)
        seg[a, a] = (e_s, t_s, adm, play, sh)
        for b in range(a + 1, n_g):
            bins = groups[b]
            e_s, t_s = e_s + e_cost[:, bins].sum(1), t_s + t_cost[:, bins].sum(1)
            adm, play = adm & reach[:, bins].all(1), play & playable[:, bins].all(1)
            sh = sh + short_p[:, bins].sum(1)
            seg[a, b] = (e_s, t_s, adm, play, sh)
    p_all = float(p_tot.sum())
    best = None
    for alpha in ALPHAS:
        pick = {}
        for (a, b), (e_s, t_s, adm, play, sh) in seg.items():
            cost = alpha * e_s + (1.0 - alpha) * t_s
            if adm.any():
                c = int(np.argmin(np.where(adm, cost, np.inf)))
                pick[a, b] = (c, float(cost[c]))
            elif play.any():
                c = int(np.lexsort((cost, np.where(play, sh, np.inf)))[0])
                pick[a, b] = (c, float(cost[c]) + SHORT_PENALTY * float(sh[c]) / p_all)
        # dp[m][j]: best cost covering groups 0..j-1 with m segments
        inf = (math.inf, None)
        dp = [[inf] * (n_g + 1) for _ in range(max_wf + 1)]
        dp[0][0] = (0.0, None)
        for m in range(1, max_wf + 1):
            for j in range(1, n_g + 1):
                for a in range(j):
                    prev, p = dp[m - 1][a], pick.get((a, j - 1))
                    if p is None or prev[0] == math.inf:
                        continue
                    cost = prev[0] + p[1]
                    if cost < dp[m][j][0]:
                        dp[m][j] = (cost, (a, p[0]))
        m_best = min(range(1, max_wf + 1), key=lambda m: dp[m][n_g][0])
        if dp[m_best][n_g][0] == math.inf:
            continue
        choice = -np.ones(nb, dtype=int)
        j, m = n_g, m_best
        while j > 0:
            a, c = dp[m][j][1]
            for g in groups[a:j]:
                choice[g] = c
            j, m = a, m - 1
        plan = _Plan(choice, P, r, e_pos, t_pos, r_want, rr, p_avg_max_w)
        key = (round(plan.short_frac / short_tol), plan.objective)
        if best is None or key < best[0]:
            best = (key, plan)
    return best[1] if best else None


def _wf_dict(name: str, w, f: float, v_max: float) -> dict:
    return {"name": name, "band": "S", "freq_ghz": f, "integration": w.integration,
            "tau_us": round(w.tau_s * 1e6, 1), "n_pulses": w.n, "pri_ms": round(w.pri_s * 1e3, 3),
            "sig_bw_mhz": w.sig_bw_mhz, "b_tx": w.b_tx,
            "dwell_ms": round(w.dwell_s * 1e3, 1), "duty_pct": round(100 * w.duty, 3),
            "unamb_range_km": round(C * w.pri_s / 2e3), "doppler_max_kHz": round(2 * v_max / rm.lambda_m(f) / 1e3, 1)}


def _best_plan(sec: dict, sph: Sphere, m: np.ndarray, p_avg_max_w: float):
    """Frequency and waveform plan chosen for a sector: (f, P, unscanned, plan, cands), or a status dict
    (NO_POSITIONS / NO_WAVEFORM) when there is none."""
    tgt = rm.Target(sec["id"], sec["rcs_m2"], int(sec.get("swerling", 1)), sec["v_close_max_mps"])
    r_want, rr = sec["range_km"], sec["rr_target_s"]
    max_wf = max(1, int(sec.get("max_wf_per_sector", MAX_WF_PER_SECTOR)))
    short_tol = sec.get("short_tol", SHORT_TOL)
    cands = candidate_waveforms(sec)
    e_pos = np.array([rm.BAND_S.p_pk_total_w * w.n * w.tau_s / w.b_tx for w in cands])   # J per position, full power
    t_pos = np.array([w.n * w.tau_s / w.b_tx for w in cands])                            # s of TX time per position
    per_f = []
    for f in sec.get("freq_ghz_candidates", FREQ_CANDIDATES):
        pos, unscanned = positions_table(sph, m, f)
        P = np.array([pos[k] for k in range(4)])
        if P.sum() > 0 and cands:
            per_f.append((f, P, unscanned, _range_matrix(cands, f, tgt, P.shape[1])))
    if not per_f:
        return {"id": sec["id"], "status": "NO_POSITIONS", "note": "empty sector or fully unscanned"}
    min_uns = min(x[2] for x in per_f)
    best = None
    for f, P, unscanned, r in per_f:
        plan = _plan_waveforms(P, r, e_pos, t_pos, r_want, rr, p_avg_max_w, max_wf, short_tol)
        if plan is None:
            continue
        # [D] a frequency that leaves more of the sector unscanned (e.g. > 2.85 GHz, 70 deg scan limit) never wins
        key = (round(unscanned - min_uns, 3), round(plan.short_frac / short_tol), plan.objective)
        if best is None or key < best[0]:
            best = (key, f, P, unscanned, plan)
    if best is None:
        return {"id": sec["id"], "status": "NO_WAVEFORM", "note": "no playable waveform in the search space"}
    _, f, P, unscanned, plan = best
    return f, P, unscanned, plan, cands


def sector_plan(sec: dict, eps_min: float, eps_max: float, p_avg_max_w: float) -> Optional[dict]:
    """The waveform plan run() would choose for this sector (same code path), per 1-deg scan bin: "wf" (Waveform or
    None where the sector has no position), "power_scale" (power fraction actually transmitted, "no excess energy"),
    "short" (the sector's own range is not reached there, even at full power), and "freq_ghz". None if the sector
    has no plan. Used to test whether this sector also meets another one's need."""
    rm.SECTOR_EPS_MIN_DEG, rm.SECTOR_EPS_MAX_DEG = eps_min, eps_max
    sph = _sphere(eps_min, eps_max)
    best = _best_plan(sec, sph, sph.mask(sec), p_avg_max_w)
    if isinstance(best, dict):
        return None
    f, _, _, plan, cands = best
    return {"freq_ghz": f, "wf": [cands[c] if c >= 0 else None for c in plan.choice],
            "power_scale": [float(s) for s in plan.s_bin],
            "short": [bool(r < sec["range_km"]) for r in plan.r_bin]}   # NaN (no position) compares False


def optimise_sector(sec: dict, sph: Sphere, m: np.ndarray, p_avg_max_w: float) -> dict:
    best = _best_plan(sec, sph, m, p_avg_max_w)
    if isinstance(best, dict):
        return best
    f, P, unscanned, plan, cands = best
    r_want, rr = sec["range_km"], sec["rr_target_s"]
    v_max = sec["v_close_max_mps"]
    p_tot = P.sum(axis=0)
    populated = np.flatnonzero(p_tot > 0)

    # distinct waveforms, in increasing scan angle, and their scan-angle span
    order = []
    for i in populated:
        if plan.choice[i] not in order:
            order.append(int(plan.choice[i]))
    wf_set = []
    for n, c in enumerate(order, 1):
        bins = populated[plan.choice[populated] == c]
        d = _wf_dict(f"WF-{sec['id']}" + (f"-{n}" if len(order) > 1 else ""), cands[c], f, v_max)
        d.update({"theta_lo_deg": float(bins.min() * THETA_BIN_DEG), "theta_hi_deg": float((bins.max() + 1) * THETA_BIN_DEG),
                  "positions": round(float(p_tot[bins].sum()), 1)})
        wf_set.append(d)
    by_name = {c: d for c, d in zip(order, wf_set)}
    wf_by_bin = []
    for lo in range(0, len(p_tot), BIN_DEG):
        bins = [i for i in range(lo, min(lo + BIN_DEG, len(p_tot))) if p_tot[i] > 0]
        if not bins:
            continue
        worst = bins[-1]
        wf_by_bin.append({"theta_lo_deg": float(lo * THETA_BIN_DEG), "theta_hi_deg": float((lo + BIN_DEG) * THETA_BIN_DEG),
                          "positions": round(float(p_tot[bins].sum()), 1), "wf": {k: v for k, v in by_name[int(plan.choice[worst])].items()
                                 if k not in ("theta_lo_deg", "theta_hi_deg", "positions")},
                          "range_km_at_bin_max": round(float(plan.r_bin[worst])),
                          "power_scale": round(float(plan.s_bin[worst]), 3)})

    faces = {}
    for k in range(4):
        if P[k].sum() <= 0:
            continue
        e_tot = float(plan.fe[k]) * p_avg_max_w * rr
        faces[k] = {"positions": round(float(P[k].sum()), 1), "energy_J_per_refresh": e_tot,
                    "energy_fraction": float(plan.fe[k]), "time_fraction": float(plan.ft[k]),
                    "avg_power_kW": e_tot / rr / 1e3,
                    "theta_max_deg": float(np.flatnonzero(P[k] > 0).max() + 1) * THETA_BIN_DEG}
    load = max(max(fc["energy_fraction"], fc["time_fraction"]) for fc in faces.values())
    f_e_max = max(fc["energy_fraction"] for fc in faces.values())
    f_t_max = max(fc["time_fraction"] for fc in faces.values())
    r_worst, r_best = float(plan.r_bin[populated[-1]]), float(plan.r_bin[populated[0]])
    main = max(wf_set, key=lambda d: d["positions"])
    out = {
        "id": sec["id"], "status": "OK",
        "wf": {k: v for k, v in main.items() if k not in ("theta_lo_deg", "theta_hi_deg", "positions")},
        "wf_set": wf_set,
        "wf_by_bin": wf_by_bin,
        "range_km": {"requested": r_want, "at_best_scan": round(r_best), "at_worst_scan": round(r_worst),
                     "positions_short_of_range_pct": round(100 * plan.short_frac, 1)},
        "unscanned_pct": round(100 * unscanned, 1),
        "positions_total": round(float(p_tot.sum()), 1),
        "faces": {str(k): v for k, v in faces.items()},
        "rr_target_s": rr,
        "rr_alone_s": rr * load,                               # RR if this sector had the face to itself (energy and time)
        "time_to_impact_at_detection_s": (r_want * 1e3 / v_max - rr) if v_max > 0 else None,
    }
    out["flags"] = []
    if plan.short_pos > 0:
        out["flags"].append(f"range {r_want:.0f} km not reached on {100 * plan.short_frac:.1f} % of positions at full power "
                            f"(worst scan {r_worst:.0f} km)")
    if f_e_max > 1.0:
        out["flags"].append(f"RR target {rr:.0f} s needs {100 * f_e_max:.0f} % of a face's average power "
                            f"(missing {1e-3 * p_avg_max_w * (f_e_max - 1):.0f} kW); best RR alone = {rr * load:.0f} s")
    if f_t_max > 1.0:
        out["flags"].append(f"RR target {rr:.0f} s needs {100 * f_t_max:.0f} % time on air; best RR alone = {rr * load:.0f} s")
    if unscanned > 0.005:
        out["flags"].append(f"{100 * unscanned:.1f} % of the sector is outside the scanned zone (scan limit / eps window)")
    return out


# ----------------------------------------------------------------------------- parallel execution
_SPHERES: Dict[Tuple[float, float], Sphere] = {}
_EXECUTOR: Optional[ProcessPoolExecutor] = None


def _sphere(eps_min: float, eps_max: float) -> Sphere:
    """One sphere raster per process, rebuilt only when the eps window changes."""
    key = (eps_min, eps_max)
    if key not in _SPHERES:
        _SPHERES.clear()
        _SPHERES[key] = Sphere(eps_min, eps_max)
    return _SPHERES[key]


def _optimise_task(sec: dict, eps_min: float, eps_max: float, p_avg_max: float) -> dict:
    """Worker-process entry point (module level so it can be pickled)."""
    rm.SECTOR_EPS_MIN_DEG, rm.SECTOR_EPS_MAX_DEG = eps_min, eps_max
    sph = _sphere(eps_min, eps_max)
    return optimise_sector(sec, sph, sph.mask(sec), p_avg_max)


def _executor() -> ProcessPoolExecutor:
    global _EXECUTOR
    if _EXECUTOR is None:
        _EXECUTOR = ProcessPoolExecutor(max_workers=MAX_WORKERS)   # workers are spawned on demand
    return _EXECUTOR


def _run_sequential(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb) -> Tuple[List[dict], bool]:
    results = []
    for i, sec in enumerate(sectors):
        if cancel_cb is not None and cancel_cb():
            return results, True
        results.append(_optimise_task(sec, eps_min, eps_max, p_avg_max))
        if progress_cb is not None:
            progress_cb(i + 1, len(sectors), sec["id"])
    return results, False


def _run_parallel(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb, idle=None) -> Tuple[List[dict], bool]:
    """Sectors spread over the worker pool. Cancel is polled every 0.2 s: queued sectors are dropped at once,
    sectors already running finish in the background and their results are discarded. idle() is a short
    main-process job run while waiting (returns False when it has nothing left to do)."""
    global _EXECUTOR
    out: List[Optional[dict]] = [None] * len(sectors)
    try:
        ex = _executor()
        index = {ex.submit(_optimise_task, sec, eps_min, eps_max, p_avg_max): i for i, sec in enumerate(sectors)}
        pending, n_done = set(index), 0
        while pending:
            if cancel_cb is not None and cancel_cb():
                for fut in pending:
                    fut.cancel()
                return [r for r in out if r is not None], True
            busy = idle is not None and idle()
            done, pending = wait(pending, timeout=0.0 if busy else 0.2, return_when=FIRST_COMPLETED)
            for fut in done:
                i = index[fut]
                out[i] = fut.result()
                n_done += 1
                if progress_cb is not None:
                    progress_cb(n_done, len(sectors), sectors[i]["id"])
    except BrokenProcessPool:
        _EXECUTOR = None   # a worker died (e.g. out of memory): start a fresh pool on the next run
        raise
    return out, False


def _mask_box(m: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
    """(phi_lo, phi_hi, eps_lo, eps_hi) index bounding box of a mask, None if empty."""
    rows = np.flatnonzero(m.any(axis=1))
    if rows.size == 0:
        return None
    cols = np.flatnonzero(m.any(axis=0))
    return int(rows[0]), int(rows[-1]) + 1, int(cols[0]), int(cols[-1]) + 1


def _overlap_warnings(sph: Sphere, masks: Dict[str, np.ndarray]) -> List[str]:
    """Pairwise sector overlaps. Each pair is evaluated only on the intersection of the two bounding boxes, so
    disjoint sectors cost nothing and the n^2 pair loop stays cheap as the sector count grows."""
    boxes = {sid: _mask_box(m) for sid, m in masks.items()}
    ids, warn = list(masks), []
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            ba, bb = boxes[a], boxes[b]
            if ba is None or bb is None:
                continue
            r0, r1, c0, c1 = max(ba[0], bb[0]), min(ba[1], bb[1]), max(ba[2], bb[2]), min(ba[3], bb[3])
            if r0 >= r1 or c0 >= c1:
                continue
            sl = (slice(r0, r1), slice(c0, c1))
            ov = float((sph.d_omega[sl] * (masks[a][sl] & masks[b][sl])).sum())
            if ov > 1e-4:
                warn.append(f"sectors {a} / {b} overlap ({ov:.3f} sr): budgeted twice")
    return warn


def run(mission: dict, progress_cb=None, cancel_cb=None, workers: Optional[int] = None) -> dict:
    """progress_cb(done, total, sector_id) is called after each sector (optional, e.g. for a UI
    progress bar). cancel_cb() is polled between sectors (every 0.2 s in parallel mode); if it returns True,
    run() stops early and returns the partial result with "cancelled": True (sectors not computed are absent).
    workers: None = automatic (parallel from PARALLEL_MIN_SECTORS sectors), 1 = sequential, > 1 = parallel."""
    g = mission.get("global", {})
    eps_min, eps_max = g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0)
    rm.SECTOR_EPS_MIN_DEG, rm.SECTOR_EPS_MAX_DEG = eps_min, eps_max
    elec = g.get("face_elec_mw", rm.FACE_ELEC_POWER_MW)
    p_avg_max = elec * 1e6 * rm.CHAIN_EFF * rm.PAE
    sectors = mission["sectors"]
    parallel = (len(sectors) >= PARALLEL_MIN_SECTORS) if workers is None else workers > 1
    masks: Dict[str, np.ndarray] = {}
    to_mask = iter(sectors)

    def build_next_mask() -> bool:   # overlap-check masks, built while the workers run
        sec = next(to_mask, None)
        if sec is None:
            return False
        masks[sec["id"]] = _sphere(eps_min, eps_max).mask(sec)
        return True

    if parallel and len(sectors) > 1:
        results, cancelled = _run_parallel(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb,
                                           idle=build_next_mask)
    else:
        results, cancelled = _run_sequential(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb)
    sph = _sphere(eps_min, eps_max)
    done_ids = {r["id"] for r in results}
    for sec in sectors:   # masks not built yet (sequential run, or workers faster than the main process)
        if sec["id"] in done_ids and sec["id"] not in masks:
            masks[sec["id"]] = sph.mask(sec)
    masks = {sec["id"]: masks[sec["id"]] for sec in sectors if sec["id"] in done_ids}   # mission order
    # mission totals per face
    tot = {k: {"energy_fraction": 0.0, "time_fraction": 0.0, "avg_power_kW": 0.0, "positions": 0.0} for k in range(4)}
    for r in results:
        for k, fc in r.get("faces", {}).items():
            for key in tot[int(k)]:
                tot[int(k)][key] += fc["positions" if key == "positions" else key]
    dil = {k: max(1.0, v["energy_fraction"], v["time_fraction"]) for k, v in tot.items()}
    for r in results:
        if r["status"] != "OK":
            continue
        r["rr_est_mission_s"] = r["rr_target_s"] * max(dil[int(k)] for k in r["faces"])
    # overlaps (only among sectors actually computed, in case of an early cancellation)
    warn = _overlap_warnings(sph, masks)
    # tracking margin: power left after search (the S and Ku arrays only share the face's electrical power)
    for k, v in tot.items():
        p_avail_w = max(0.0, 1.0 - v["energy_fraction"]) * p_avg_max
        v["power_margin"] = 1.0 - v["energy_fraction"]
        v["p_available_kW"] = p_avail_w / 1e3
        v["ku_duty_pct"] = 100.0 * p_avail_w / rm.BAND_KU.p_pk_total_w
        v["time_margin_s"] = 1.0 - v["time_fraction"]      # S-band TX time only; Ku is a separate array
        if v["positions"] <= 0:
            continue
        if v["energy_fraction"] > 1.0 or v["time_fraction"] > 1.0:
            warn.append(f"face F{k + 1} over-subscribed: energy {100 * v['energy_fraction']:.0f} %, "
                        f"time on air {100 * v['time_fraction']:.0f} % (RR stretched x{dil[k]:.2f})")
        elif v["power_margin"] < TRACK_MARGIN_WARN:
            warn.append(f"face F{k + 1} tracking margin < {100 * TRACK_MARGIN_WARN:.0f} % "
                        f"({v['p_available_kW']:.0f} kW left for Ku)")
    return {"global": {"eps_min_deg": eps_min, "eps_max_deg": eps_max, "face_elec_mw": elec,
                       "face_avg_rf_kW": p_avg_max / 1e3, "objective": "max(energy,time)",
                       "bin_deg": BIN_DEG, "max_wf_per_sector": MAX_WF_PER_SECTOR},
            "cancelled": cancelled,
            "sectors": results,
            "faces_total": {str(k): {kk: round(vv, 4) for kk, vv in v.items()} for k, v in tot.items()},
            "warnings": warn}


# ----------------------------------------------------------------------------- I/O
def load_mission(path: str) -> dict:
    if path.lower().endswith(".csv"):
        secs = []
        with open(path, newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(r for r in fh if not r.lstrip().startswith("#")):
                s = {}
                for k, v in row.items():
                    if v is None or v == "":
                        continue
                    s[k] = v if k in ("id", "shape") else (v.lower() == "true" if k == "allow_ambiguous" else float(v))
                secs.append(s)
        return {"global": {}, "sectors": secs}
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def print_report(res: dict) -> None:
    g = res["global"]
    print("=" * 100)
    print(f" MISSION PRE-OPTIMISATION | eps window [{g['eps_min_deg']:.0f}, {g['eps_max_deg']:.0f}] deg | "
          f"per face: {g['face_elec_mw']} MW elec -> {g['face_avg_rf_kW']:.0f} kW avg RF")
    print("=" * 100)
    for r in res["sectors"]:
        if r["status"] != "OK":
            print(f"[{r['id']}] {r['status']}: {r.get('note','')}")
            continue
        print(f"[{r['id']}] {len(r['wf_set'])} waveform(s)")
        for w in r["wf_set"]:
            print(f"     {w['name']}  theta {w['theta_lo_deg']:.0f}-{w['theta_hi_deg']:.0f} deg ({w['positions']:.0f} pos)  "
                  f"{w['freq_ghz']} GHz {w['integration']}  tau {w['tau_us']} us x N {w['n_pulses']} "
                  f"@ PRI {w['pri_ms']} ms  B {w['sig_bw_mhz']:.0f} MHz  Btx {w['b_tx']}  dwell {w['dwell_ms']} ms  "
                  f"(Dopp. max {w['doppler_max_kHz']} kHz)")
        rg = r["range_km"]
        print(f"     Range: wanted {rg['requested']:.0f} km | worst scan {rg['at_worst_scan']} km | best scan {rg['at_best_scan']} km"
              f" | positions: {r['positions_total']:.0f}")
        for k, fc in r["faces"].items():
            print(f"     face {k}: {fc['positions']:.0f} pos, theta<={fc['theta_max_deg']:.0f} deg | energy {100*fc['energy_fraction']:.1f} % "
                  f"| avg {fc['avg_power_kW']:.0f} kW | time on air {100*fc['time_fraction']:.2f} %")
        tti = r["time_to_impact_at_detection_s"]
        print(f"     RR: target {r['rr_target_s']:.0f} s | alone {r['rr_alone_s']:.0f} s | in mission {r['rr_est_mission_s']:.0f} s"
              f" | time to impact after detection (vmax) {'n/a' if tti is None else f'{tti:.0f} s'}")
        for fl in r["flags"]:
            print("     !!", fl)
    print("-" * 100)
    for k, v in res["faces_total"].items():
        if v["positions"] > 0:
            load = max(v["energy_fraction"], v["time_fraction"])
            st = "OK" if load <= 1 else f"OVER-SUBSCRIBED x{load:.2f}"
            print(f" face {k}: {v['positions']:.0f} pos | energy {100*v['energy_fraction']:.1f} % | avg {v['avg_power_kW']:.0f} kW "
                  f"| time on air {100*v['time_fraction']:.1f} % -> {st} | tracking margin {v['p_available_kW']:.0f} kW "
                  f"(Ku duty {v['ku_duty_pct']:.1f} %)")
    for wmsg in res["warnings"]:
        print(" WARNING:", wmsg)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python -m core.optimizer mission.json|mission.csv [report.json]")
        sys.exit(1)
    result = run(load_mission(sys.argv[1]))
    print_report(result)
    if len(sys.argv) > 2:
        with open(sys.argv[2], "w", encoding="utf-8") as fh:
            json.dump(result, fh, indent=2)
        print("report ->", sys.argv[2])
