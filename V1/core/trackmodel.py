#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TRACK MODEL -- the physics of an active track (roadmap step 3.3, blocks A-F validated by the user on 25 October 2026),
used by core/tracking.py for the active-tracking (AT) brackets of a sector.

  A  Steady-state Kalman filter per axis, 3 states (position, velocity, acceleration), Singer manoeuvre model
     (sigma_m^2 = a_max^2 / 3 (1 + 4 p_max - p0) plus a process-noise floor, time constant of the class), discretised
     exactly (Van Loan) and solved by the discrete Riccati equation. Three decoupled axes: range (range and Doppler
     measurements) and two cross-range axes (R x angle). Requirements: the angular zone (sigma = zone / 4) on the
     PREDICTED angle at the next update, the range and radial-velocity accuracies on the FILTERED track. Each is
     turned into the largest measurement error that meets it (APPROXIMATION, prudent: the range-position and the
     radial-velocity requirements are each met by a filter using only its own measurement; the joint filter is
     better), then into the accuracy pack of the Ku track look (radar_model.tracking_snr_req), the measurement floors
     taken out in quadrature (E4-E6).
  B  The predicted angle error points the look (pointing law, radar_model.position_losses); velocity and acceleration
     banks over +/- k sigma of the predicted values; a Markov chain over the consecutive misses (the look of each
     coasted period keeps its energy, the pointing error grows) gives the probability of losing the track.
  C  Initiation: the covariance from the confirmation hand-over, iterated at the update period until every
     requirement is within init_conv_frac of its steady value: number of looks and mean pointing error.
  F  Discrimination of two classes (KKV / decoy): four independent indices (depolarisation ratio through the two
     polarisation channels, fluctuation statistic, RCS level including the seeker optics, manoeuvre seen by the
     filter), each a Gaussian test whose error falls with the number of looks and the SNR; d' combined in quadrature,
     P(correct) = Phi(d' / 2). Faraday rotation uncertainty adds a systematic error to the depolarisation index unless
     the TX polarisation is switched (full scattering matrix, polar_switch_loss_db).
Test missions are not requirements; the values here are model choices of the user (tracking.*, tool.tracking.*).
"""
from __future__ import annotations

import functools
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.linalg import expm, solve_discrete_are

from . import radar_model as rm
from . import repoint
from .params import P

_T, _TT = "tracking.", "tool.tracking."
P_MAX = P.get(_T + "p_max")
P0 = P.get(_T + "p0")
FLOOR_MPS2 = P.get(_T + "process_noise_floor_mps2")
PD_UPDATE = P.get(_T + "pd_update")
N_MISS_LOST = int(P.get(_T + "n_miss_lost"))
VEL_BANK_SIGMAS = P.get(_T + "vel_bank_sigmas")
ACCEL_BANK_SIGMAS = P.get(_T + "accel_bank_sigmas")
INIT_VEL_SIGMA_FRAC = P.get(_T + "init_vel_sigma_frac")
INIT_CONV_FRAC = P.get(_T + "init_conv_frac")
ANGLE_FLOOR_RAD = P.get(_T + "angle_floor_mrad") * 1e-3
RANGE_FLOOR_M = P.get(_T + "range_floor_m")
VR_FLOOR_MPS = P.get(_T + "vr_floor_mps")
P_DISCRIMINATION = P.get(_T + "p_discrimination")
DECEPTION_VERIFY_RATE = P.get(_T + "deception_verify_rate")
TRACK_FRAME_K = P.get(_T + "track_frame_k")
POLAR_SWITCH_LOSS_DB = P.get(_T + "polar_switch_loss_db")
FARADAY_UNCERT_FRAC = P.get(_T + "faraday_uncert_frac")
FLOOR_TAU_S = P.get(_TT + "floor_tau_s")
N_RANGES = int(P.get(_TT + "n_ranges"))
RR_ALERT_ITER = int(P.get(_TT + "rr_alert_iter"))
DISC_N_MAX = int(P.get(_TT + "disc_n_max"))
SEPARABILITY_N_LOOKS = int(P.get(_TT + "separability_n_looks"))
GATE_CELLS = P.get(_TT + "gate_cells")
FALSE_TRACK_FLAG_PER_S = P.get(_TT + "false_track_flag_per_s")
for _k, _v in (("p_max", P_MAX), ("p0", P0)):
    if not 0.0 <= _v <= 1.0:
        raise ValueError(f"{P.path}: tracking.{_k} must lie in [0, 1]")
if N_MISS_LOST < 1 or not 0.0 < PD_UPDATE < 1.0:
    raise ValueError(f"{P.path}: tracking.n_miss_lost >= 1 and 0 < tracking.pd_update < 1")


# ----------------------------------------------------------------------------- A: Singer model, Riccati
def manoeuvre(tgt: "rm.Target") -> Tuple[float, float]:
    """(sigma_m m/s2, time constant s) of the target's random acceleration per axis (A3-A6): Singer intensity of the
    class a_max on every axis, plus the floor in quadrature; a class without manoeuvre keeps the floor only, with the
    tool's floor time constant."""
    sm2 = tgt.accel_max_mps2 ** 2 / 3.0 * (1.0 + 4.0 * P_MAX - P0)
    tau = tgt.maneuver_tau_s if (tgt.maneuver_tau_s > 0.0 and tgt.accel_max_mps2 > 0.0) else FLOOR_TAU_S
    return math.sqrt(sm2 + FLOOR_MPS2 ** 2), float(tau)


@functools.lru_cache(maxsize=4096)
def singer(t_s: float, sigma_m: float, tau_s: float) -> Tuple[np.ndarray, np.ndarray]:
    """(F, Q) of the Singer model over one period t_s, exact (Van Loan): d/dt [p v a] = [v a -a / tau] + w, w white of
    density 2 sigma_m^2 / tau on the acceleration."""
    alpha = 1.0 / tau_s
    a = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, -alpha]])
    gqg = np.zeros((3, 3))
    gqg[2, 2] = 2.0 * alpha * sigma_m ** 2
    m = np.zeros((6, 6))
    m[:3, :3], m[:3, 3:], m[3:, 3:] = -a, gqg, a.T
    e = expm(m * t_s)
    f = e[3:, 3:].T
    q = f @ e[:3, 3:]
    q = 0.5 * (q + q.T)
    f.setflags(write=False)
    q.setflags(write=False)
    return f, q


def steady(f: np.ndarray, q: np.ndarray, h: np.ndarray, r: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(predicted, filtered) steady-state covariances of the Kalman filter (discrete Riccati equation)."""
    try:
        p = solve_discrete_are(f.T, h.T, q + 1e-30 * np.eye(len(q)), r)
    except (ValueError, np.linalg.LinAlgError):            # ill-conditioned: plain Riccati iteration
        p = q.copy()
        for _ in range(20000):
            pf = p - p @ h.T @ np.linalg.solve(h @ p @ h.T + r, h @ p)
            nxt = f @ pf @ f.T + q
            if np.allclose(nxt, p, rtol=1e-10, atol=0.0):
                p = nxt
                break
            p = nxt
    s = h @ p @ h.T + r
    pf = p - p @ h.T @ np.linalg.solve(s, h @ p)
    return p, pf


_H_POS = np.array([[1.0, 0.0, 0.0]])
_H_POS_VEL = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
_H_VEL2 = np.array([[1.0, 0.0]])


def _bisect_max(ok, lo: float, hi: float, iters: int = 60) -> float:
    """Largest x in [lo, hi] (log scale) with ok(x) True, ok increasing toward lo; lo if even lo fails (returns 0)."""
    if not ok(lo):
        return 0.0
    if ok(hi):
        return hi
    a, b = math.log(lo), math.log(hi)
    for _ in range(iters):
        m = 0.5 * (a + b)
        a, b = (m, b) if ok(math.exp(m)) else (a, m)
    return math.exp(a)


@functools.lru_cache(maxsize=8192)
def allowed_measurement(t_s: float, sigma_m: float, tau_s: float, r_m: float, zone_sigma_rad: float,
                        range_acc_m: Optional[float], vr_acc_mps: float) -> Dict[str, float]:
    """Largest measurement errors meeting the bracket's requirements at period t_s and range r_m (A7, decoupled and
    prudent, see the module docstring): 'theta' rad (predicted angle <= zone sigma), 'r' m (filtered range), 'vr' m/s
    (filtered radial velocity); 0 = not met even with a perfect measurement."""
    f, q = singer(t_s, sigma_m, tau_s)
    lim = zone_sigma_rad * r_m

    def cross_ok(s):
        p, _ = steady(f, q, _H_POS, np.array([[s * s]]))
        return math.sqrt(p[0, 0]) <= lim

    out = {"theta": _bisect_max(cross_ok, 1e-3 * lim, 1e3 * lim) / r_m}
    if range_acc_m is not None:
        def range_ok(s):
            _, pf = steady(f, q, _H_POS, np.array([[s * s]]))
            return math.sqrt(pf[0, 0]) <= range_acc_m
        out["r"] = _bisect_max(range_ok, 1e-3 * range_acc_m, 1e3 * range_acc_m)
    else:
        out["r"] = math.inf
    f2, q2 = f[1:, 1:], q[1:, 1:]

    def vel_ok(s):
        _, pf = steady(f2, q2, _H_VEL2, np.array([[s * s]]))
        return math.sqrt(pf[0, 0]) <= vr_acc_mps
    out["vr"] = _bisect_max(vel_ok, 1e-3 * vr_acc_mps, 1e3 * vr_acc_mps)
    return out


def _less_floor(allowed: float, floor: float) -> float:
    return math.sqrt(allowed * allowed - floor * floor) if allowed > floor else 0.0


class TrackState:
    """The steady track of one bracket at one range: the accuracy pack of the update look and what points it."""

    def __init__(self, tgt: "rm.Target", t_s: float, r_km: float, zone_deg: float, range_acc_m: Optional[float],
                 vr_acc_mps: float, f_ku: float, theta_deg: float, pd_update: Optional[float] = None,
                 nav_axis_deg: Optional[float] = None):
        # step 5.1: Pd of an update (bracket key pd_update) and the navigation's pointing error (mission acceleration)
        self.pd_update = PD_UPDATE if pd_update is None else float(pd_update)
        nav_deg = repoint.axis_sigma_deg() if nav_axis_deg is None else float(nav_axis_deg)
        self.t_s, self.r_m = float(t_s), float(r_km) * 1e3
        self.sigma_m, self.tau_s = manoeuvre(tgt)
        zone_sigma = math.radians(zone_deg / rm_zone_sigmas())
        al = allowed_measurement(round(self.t_s, 6), self.sigma_m, self.tau_s, round(self.r_m, 0),
                                 zone_sigma, range_acc_m, vr_acc_mps)
        self.allowed = al
        # noise part each look must reach (floors taken out, E4-E6); 0 = requirement out of reach at any SNR
        self.noise_theta = _less_floor(al["theta"], ANGLE_FLOOR_RAD)
        self.noise_r = math.inf if range_acc_m is None else _less_floor(al["r"], RANGE_FLOOR_M)
        self.noise_vr = _less_floor(al["vr"], VR_FLOOR_MPS)
        self.feasible = self.noise_theta > 0.0 and self.noise_r > 0.0 and self.noise_vr > 0.0
        f, q = singer(round(self.t_s, 6), self.sigma_m, self.tau_s)
        self.f, self.q = f, q
        # the steady filter with the measurements at their allowed (worst) values: what points the next look
        s_c = max(al["theta"], 1e-12) * self.r_m
        self.p_cross, self.pf_cross = steady(f, q, _H_POS, np.array([[s_c * s_c]]))
        s_r = al["r"] if math.isfinite(al["r"]) else 1e6
        self.p_range, self.pf_range = steady(f, q, _H_POS_VEL, np.diag([s_r * s_r, max(al["vr"], 1e-9) ** 2]))
        hp = math.radians(rm.hpbw_deg(f_ku, theta_deg))
        self.hpbw_rad = hp
        # step 4.5: the navigation's per-axis pointing error adds to the filter's (C3)
        self.pointing_hpbw = math.hypot(math.sqrt(self.p_cross[0, 0]) / self.r_m, math.radians(nav_deg)) / hp
        self.v_span_mps = 2.0 * VEL_BANK_SIGMAS * math.sqrt(self.p_range[1, 1])
        # never wider than the class's own acceleration (E1), never below the floor
        self.a_bank_mps2 = min(ACCEL_BANK_SIGMAS * math.sqrt(max(self.p_cross[2, 2], self.p_range[2, 2])),
                               max(tgt.accel_max_mps2, FLOOR_MPS2))
        self.zone_sigma_rad, self.range_acc_m, self.vr_acc_mps = zone_sigma, range_acc_m, vr_acc_mps

    def pack(self) -> "rm.TrackingPack":
        return rm.TrackingPack(math.degrees(self.noise_theta), self.noise_vr,
                               None if not math.isfinite(self.noise_r) else self.noise_r)

    def coast_pointing(self, k: int) -> float:
        """Pointing error (TX beamwidths) of a look after k missed updates (B6)."""
        p = self.p_cross
        for _ in range(k):
            p = self.f @ p @ self.f.T + self.q
        return math.sqrt(p[0, 0]) / self.r_m / self.hpbw_rad


def rm_zone_sigmas() -> float:
    return P.get(_T + "angle_zone_sigmas")


def track_target(tgt: "rm.Target", st: TrackState) -> "rm.Target":
    """The target as a track look sees it: its acceleration bank over +/- k sigma of the predicted acceleration (E1)."""
    import dataclasses
    return dataclasses.replace(tgt, accel_max_mps2=st.a_bank_mps2)


# ----------------------------------------------------------------------------- B: miss and loss chain
def loss_per_update(pds: List[float]) -> Tuple[float, float]:
    """(probability that a given update ends the track, mean looks per period = 1) of the renewal chain over the
    consecutive misses: state k = k misses so far, hit -> 0, miss -> k + 1, the N-th miss loses the track (a new
    track then starts in state 0). pds[k] = Pd of the look in state k (B6-B7)."""
    n = len(pds)
    # stationary distribution of the chain where 'lost' restarts at state 0
    pi = np.zeros(n)
    pi[0] = 1.0
    for k in range(1, n):
        pi[k] = pi[k - 1] * (1.0 - pds[k - 1])
    pi /= pi.sum()
    return float(pi[-1] * (1.0 - pds[-1])), 1.0


# ----------------------------------------------------------------------------- C: initiation
def initiation(st: TrackState, tgt: "rm.Target", sigma_theta_conf_rad: float, sigma_r_conf_m: float,
               sigma_vr_conf_mps: float, zone_deg: float, range_acc_m: Optional[float], vr_acc_mps: float,
               n_max: int = 200) -> Tuple[int, float]:
    """(looks until every requirement of the bracket is held within INIT_CONV_FRAC, mean pointing error of those looks
    in TX beamwidths) from the confirmation hand-over (C1-C3; the criterion is the operator's accuracy x (1 + frac):
    against the steady value, a target with a tiny process noise would need hundreds of looks to settle). The
    measurements are the better of their allowed values and the hand-over accuracies."""
    f, q = st.f, st.q
    sa0 = tgt.accel_max_mps2 / math.sqrt(3.0) if tgt.accel_max_mps2 > 0 else FLOOR_MPS2
    vt0 = INIT_VEL_SIGMA_FRAC * tgt.v_close_mps
    pc = np.diag([(sigma_theta_conf_rad * st.r_m) ** 2, vt0 ** 2, sa0 ** 2])
    pr = np.diag([sigma_r_conf_m ** 2, sigma_vr_conf_mps ** 2, sa0 ** 2])
    # initiation looks measure at least as well as the confirmation hand-over (a steady track with little process
    # noise may accept far worse single measurements: they would not settle a new track)
    rc = np.array([[(min(max(st.allowed["theta"], 1e-12), sigma_theta_conf_rad) * st.r_m) ** 2]])
    s_r = min(st.allowed["r"] if math.isfinite(st.allowed["r"]) else 1e6, sigma_r_conf_m)
    rr = np.diag([s_r * s_r, min(max(st.allowed["vr"], 1e-9), sigma_vr_conf_mps) ** 2])
    zone_lim = st.zone_sigma_rad * st.r_m * (1.0 + INIT_CONV_FRAC)
    r_lim = (math.inf if range_acc_m is None else range_acc_m) * (1.0 + INIT_CONV_FRAC)
    v_lim = vr_acc_mps * (1.0 + INIT_CONV_FRAC)
    pts = []
    for n in range(1, n_max + 1):
        pc = f @ pc @ f.T + q
        pr = f @ pr @ f.T + q
        pts.append(math.sqrt(pc[0, 0]) / st.r_m / st.hpbw_rad)
        pc = pc - pc @ _H_POS.T @ np.linalg.solve(_H_POS @ pc @ _H_POS.T + rc, _H_POS @ pc)
        pr = pr - pr @ _H_POS_VEL.T @ np.linalg.solve(_H_POS_VEL @ pr @ _H_POS_VEL.T + rr, _H_POS_VEL @ pr)
        nxt = f @ pc @ f.T + q
        if (math.sqrt(nxt[0, 0]) <= zone_lim and math.sqrt(pr[0, 0]) <= r_lim and math.sqrt(pr[1, 1]) <= v_lim):
            return n, float(np.mean(pts))
    return n_max, float(np.mean(pts))


# ----------------------------------------------------------------------------- F: discrimination
def _phi(x: float) -> float:
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def _inv_k(tgt: "rm.Target", f_ghz: float) -> float:
    k = rm.look_fluctuation(tgt, f_ghz, tgt.swerling, 1, 0.0).k
    return 0.0 if math.isinf(k) else 1.0 / k


def indices(a: "rm.Target", b: "rm.Target", f_ghz: float, snr_lin: float, n: int, faraday_rad: float,
            switched: bool, sigma_a_filt: float) -> Dict[str, float]:
    """d' of each discrimination index between classes a and b after n looks at SNR snr_lin (F1, F4, F5)."""
    out: Dict[str, float] = {}
    snr = snr_lin * (rm.db_to_lin(-POLAR_SWITCH_LOSS_DB) if switched else 1.0)
    n = max(int(n), 1)
    # depolarisation ratio through the two channels
    ra, rb = a.depol_lin(f_ghz), b.depol_lin(f_ghz)
    rho = math.sqrt(ra * rb)
    var = (1.0 + rho) / snr + (1.0 + rho) / (snr * rho)
    if switched:
        var *= 0.5                                          # the full matrix: both TX polarisations measured
    sys_ = 0.0 if switched else (math.sin(FARADAY_UNCERT_FRAC * faraday_rad) ** 2) / rho
    # step 5.1 (B3): the spread from one object to another is systematic (more looks do not reduce it)
    k_db = (math.log(10.0) / 10.0) ** 2
    obj_d = k_db * 0.5 * (a.depol_spread_db ** 2 + b.depol_spread_db ** 2)
    out["depolarisation"] = abs(math.log(ra / rb)) / math.sqrt(var / n + sys_ ** 2 + obj_d)
    # fluctuation statistic: 1 / k of the look powers
    ia, ib = _inv_k(a, f_ghz), _inv_k(b, f_ghz)
    sa = (ia + 1.0 / snr) * math.sqrt(2.0 / n)
    sb = (ib + 1.0 / snr) * math.sqrt(2.0 / n)
    out["fluctuation"] = abs(ia - ib) / math.sqrt(0.5 * (sa * sa + sb * sb)) if (sa + sb) > 0 else 0.0
    # RCS level (body + seeker optics)
    la, lb = math.log(a.rcs_at(f_ghz)), math.log(b.rcs_at(f_ghz))
    # step 5.1 (B): every presentation is possible (aspect spread of the table, none for a class seen frontally) and
    # objects of a class differ (rcs_spread_db): both systematic
    obj_r = k_db * 0.5 * (a.rcs_spread_db ** 2 + b.rcs_spread_db ** 2 + a.aspect_spread_db() ** 2
                          + b.aspect_spread_db() ** 2)
    s_l = math.sqrt((0.5 * (ia + ib) + 1.0 / snr) / n + obj_r)
    out["rcs"] = abs(la - lb) / s_l if s_l > 0 else 0.0
    # manoeuvre seen by the filter (RMS Singer acceleration of each class against the filtered acceleration error)
    ma = math.sqrt(a.accel_max_mps2 ** 2 / 3.0 * (1.0 + 4.0 * P_MAX - P0))
    mb = math.sqrt(b.accel_max_mps2 ** 2 / 3.0 * (1.0 + 4.0 * P_MAX - P0))
    out["manoeuvre"] = abs(ma - mb) / max(sigma_a_filt, 1e-9)
    return out


def p_correct(d: Dict[str, float]) -> float:
    return _phi(math.sqrt(sum(v * v for v in d.values())) / 2.0)


def discrimination(a: "rm.Target", b: "rm.Target", f_ghz: float, snr_lin: float, faraday_rad: float,
                   sigma_a_filt: float, p_target: Optional[float] = None) -> Dict[str, object]:
    """Looks needed to reach P_DISCRIMINATION between a and b, without and with the TX polarisation switching (F5);
    the switching is kept when it needs fewer looks."""
    res = {}
    p_t = P_DISCRIMINATION if p_target is None else float(p_target)      # step 5.1: bracket key p_discrimination
    for sw in (False, True):
        n_need = None
        for n in range(1, DISC_N_MAX + 1):
            if p_correct(indices(a, b, f_ghz, snr_lin, n, faraday_rad, sw, sigma_a_filt)) >= p_t:
                n_need = n
                break
        res[sw] = (n_need, p_correct(indices(a, b, f_ghz, snr_lin, DISC_N_MAX, faraday_rad, sw, sigma_a_filt)))
    use = res[True][0] is not None and (res[False][0] is None or res[True][0] < res[False][0])
    n_need, p_max = res[use]
    d = indices(a, b, f_ghz, snr_lin, n_need or DISC_N_MAX, faraday_rad, use, sigma_a_filt)
    return {"looks": n_need, "p_at_max_looks": p_max, "polarisation_switching": use,
            "dominant_index": max(d, key=d.get), "d_prime": {k: round(v, 3) for k, v in d.items()}}


def separability(f_ghz: float, snr_lin: float) -> Dict[str, Dict[str, float]]:
    """P(correct) between every pair of library classes after SEPARABILITY_N_LOOKS looks at snr_lin, no Faraday, no
    manoeuvre index (F3, indicator only)."""
    names = [n for n in rm.TARGETS if n != "ST"]
    out: Dict[str, Dict[str, float]] = {}
    for i, x in enumerate(names):
        for y in names[i + 1:]:
            d = indices(rm.TARGETS[x], rm.TARGETS[y], f_ghz, snr_lin, SEPARABILITY_N_LOOKS, 0.0, False, math.inf)
            d.pop("manoeuvre")
            out.setdefault(x, {})[y] = round(p_correct(d), 4)
    return out
