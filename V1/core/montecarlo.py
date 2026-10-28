#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MONTE-CARLO -- the mission plan played by random draws, against the analytic model of the Pmiss (roadmap step 5.4,
blocks A-H validated by the user on 27 October 2026). On demand only (A2): `python -m core.montecarlo <mission>`; the
plan never changes (A3). The scene is the mission's navigation snapshot; the dense scene (masking, polluted CFAR) is
phase 6 and the reflections on the structure are out of the model (user, 27 October 2026).

Pass M1: the test targets of the search (blocks B and C) and the cumulative Pd of each search sector.
  - Plan: the waveform, Pfa, pulse scale and full-pulse range of each 1-deg scan bin, as the optimiser kept it
    (optimizer.run(keep_plans=True)); the SNR of a look is READ from the plan, not solved again (B3): mean SNR before
    the position losses at the requirement range = the required SNR of the bin (cumulative criterion, positions law)
    + 40 log10(reach / R), reach = full-pulse range x pulse_scale^(1/4) (sky, jammers and eclipse inside it, as the
    plan has them). Noise-limited scaling (R / r)^4 between the looks; a sector with a self-protection jammer is flagged
    (its burn-through goes as 1 / r^2).
  - Test target (C1-C6): one per draw, scan bin drawn in proportion to the bin's positions (masked and blinded ones are
    not in the plan), closing at v_close_max (or uniform between v_close_min and v_close_max, v_draw), looks at
    R_bin (1 + (k + phi) q), q = v RR / R_bin, phase phi uniform (the criterion of step 2.3); values of the sector, else
    of its class; frontal RCS. Every look whose mean Pd stays above pd_look_floor counts, without a time bound (as the
    analytic criterion; user, 27 October 2026).
  - Position of the target (B4), DRAWN instead of averaged: a point uniform in the TX lattice cell, FIXED for the life
    of the target, seen through the frame-to-frame lattice shifts of step 4.3 bis (cycle of tool.scanorder.n_shifts
    shifts (a + b) j / n): TX offset = the point reduced into the shifted TX cell, RX offset = that offset reduced into
    the RX lattice of the pointing; Doppler straddle and velocity-filter offset fixed per target (constant velocity);
    range straddle drawn per look (the target crosses many range cells between two revisits). A partition (sub-aperture)
    beam draws its TX loss per look from the analytic samples (flagged). The analytic model averages every position
    loss independently look by look: the difference is what this pass measures.
  - Fluctuation (B5): target power over a look Gamma(k) with the look's shape (Swerling case, class family and T0,
    agility, polarisation diversity); between looks the same draw if RR < T0, else a new one.
  - Decision (B6): the statistic Z = sum of the n noncoherent samples (noncentral chi-square, unit complex noise per
    sample) drawn exactly against the threshold of the bin's Pfa (homogeneous noise; the CFAR loss stays in the plan).
  - Statistics (G1-G3): seed per sector, batches of `batch` targets up to n_max, stop from n_min when the Wilson
    half-width at ci_level is below ci_tol.
  - State (A3, per sector, on the positions within range): NOK when the upper bound of the interval is below the
    sector's Pd; ALERTE when the analytic Pd is above the upper bound (model optimistic); else OK.
Pass M2 (user, 27 October 2026, M2-1 to M2-9):
  - Eclipse (M2-6): the plan's mean eclipse loss is given back to the SNR and each echo is lost whole with the share
    of the common S slots of the final scheduler.
  - Confirmation (M2-4): for every detected test target in range, the angle error per axis is drawn from the signal
    SNR of the look that detected (bias + K q / sqrt(SNR) x N(0, 1), Ku beamwidths), the looks of the chosen mode
    (KU1+7, KU7, S2 with its pointed S touch) decided exactly at the drawn gain and power (one draw for looks within
    T0); the Ku looks keep the margin of the plan (detection at the design SNR, as step 3.2).
  - Timelines (M2-1, M2-2, M2-5, M2-7): an event file per face and band with the tasks of scheduler._classes and its
    rule (priority, deadline, no preemption, recharge pauses); the confirmations come from the drawn arrivals: false
    alarms per search dwell (Poisson of the analytic mean, M2-3) and true detections; n_rep timelines per face, counted
    window of 10 longest revisits (at most horizon_max_s) after warmup_revisits. Latency of a true detection = waits
    + services of its confirmation tasks + round trips + processing (step 4.3); per face and band the mean waits
    against Cobham, late updates and pauses against the synthetic timeline.
  - States (M2-8): NOK when the requirement is not met beyond the interval (P confirmation below confirm_p_min,
    latency above its requirement); ALERTE when the analytic value is optimistic. The false alarms per revisit and the
    mean waits per class are information (user, 27 October 2026, after the trials: Poisson of the analytic mean, and
    waits of microseconds).
Pass M3 (user, 27 October 2026, T1-T11; option A: the Kalman filter runs, D2 (b)):
  - per active-tracking bracket, tracks_per_bracket tracks of updates_per_track updates, each at one of the
    bracket's ranges (the budget's update look there, tracking.KEEP_MC_OBJECTS); truth = Singer manoeuvre on three
    axes, filter = the same Singer Kalman running from the hand-over (angle error of the confirming Ku look drawn in
    M2, else the analytic one); each update a drawn look (pointing error = prediction - truth + navigation, real Ku
    TX / RX patterns or the 7-beam cluster, straddles, fluctuation renewed every T0, Ku eclipse), measurement errors
    from the drawn SNR (floors in quadrature); loss after n_miss_lost consecutive misses, then a new initiation;
    initiation done on the criterion of trackmodel.initiation. Range fixed for the life of a track (T8).
  - Initiation AS FAST AS POSSIBLE (user, 27 October 2026, after the first trial: at the bracket period the first
    look pointed 9.5 Ku beamwidths off and 98.5 % of the initiations were lost): each interval the longest keeping the
    predicted pointing error under init_pointing_max_hpbw, between the round trip + processing and the period, with
    the budget's look sized for that pointing.
  - Two populations per bracket: tracks started in the filter's steady state (loss per update and RMS accuracies of
    established tracks, against the analytic chain, p_loss_max and the bracket's requirements) and tracks born from
    the hand-over (share established, looks and time to establish against the analytic initiation); NOK / ALERTE as
    T10.
To come: M4 verdict and load_ceiling; M5 tests at N3, snapshots, docs.
"""
from __future__ import annotations

import dataclasses
import functools
import heapq
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import detection
from . import optimizer as mo
from . import radar_model as rm
from . import scheduler as sch
from . import tracking as trk
from .params import P

_T = "tool.montecarlo."
SEED = int(P.get(_T + "seed"))
N_MIN = int(P.get(_T + "n_min"))
N_MAX = int(P.get(_T + "n_max"))
CI_TOL = float(P.get(_T + "ci_tol"))
CI_LEVEL = float(P.get(_T + "ci_level"))
HORIZON_MAX_S = float(P.get(_T + "horizon_max_s"))          # scheduler timeline of a draw (M2)
WARMUP_REVISITS = int(P.get(_T + "warmup_revisits"))        # revisits played before the counted time (M2)
N_REP = int(P.get(_T + "n_rep"))                            # drawn timelines per face (M2-2)
TRACKS_PER_BRACKET = int(P.get(_T + "tracks_per_bracket"))  # M3 (T7)
UPDATES_PER_TRACK = int(P.get(_T + "updates_per_track"))    # M3 (T7)
INIT_POINTING_MAX_HPBW = float(trk.tm.INIT_POINTING_MAX_HPBW)  # fast initiation, a radar rule of 3.3 (tracking.*)
HORIZON_REVISITS = 10                                       # M2-2: counted window = 10 longest revisits of the face
V_DRAW = str(P.get(_T + "v_draw"))
BATCH = int(P.get(_T + "batch"))
PD_LOOK_FLOOR = float(P.get(_T + "pd_look_floor"))
N_SHIFTS = int(P.get("tool.scanorder.n_shifts"))
K_MAX_LOOKS = 20000                       # safety bound on the looks of one test target (flagged if it cuts)


# ----------------------------------------------------------------------------- statistics
def _z(level: float) -> float:
    """Two-sided normal quantile of a confidence level (bisection on erf)."""
    lo, hi, p = 0.0, 10.0, 0.5 + level / 2.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if 0.5 * (1.0 + math.erf(mid / math.sqrt(2.0))) < p else (lo, mid)
    return 0.5 * (lo + hi)


def wilson(k: int, n: int, level: float = CI_LEVEL) -> Tuple[float, float]:
    """Wilson interval of a proportion k / n (G3)."""
    if n <= 0:
        return 0.0, 1.0
    z = _z(level)
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2.0 * n)) / d
    h = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


# ----------------------------------------------------------------------------- lattices
def _basis(d: float) -> np.ndarray:
    """Basis (rows a, b) of the triangular lattice of step d, as radar_model._hex_cell_points."""
    return np.array([[d, 0.0], [d / 2.0, d * math.sqrt(3.0) / 2.0]])


def reduce_hex(p: np.ndarray, d: float) -> np.ndarray:
    """Offset of points p (N, 2) to their nearest site of the triangular lattice of step d (the hexagonal cell)."""
    b = _basis(d)
    c = p @ np.linalg.inv(b)
    f = np.floor(c)
    best, best_d = None, None
    for i in (0.0, 1.0):
        for j in (0.0, 1.0):
            site = (f + np.array([i, j])) @ b
            off = p - site
            dist = (off * off).sum(axis=-1)
            if best is None:
                best, best_d = off, dist
            else:
                sel = dist < best_d
                best = np.where(sel[..., None], off, best)
                best_d = np.where(sel, dist, best_d)
    return best


# ----------------------------------------------------------------------------- one scan bin of the plan
@dataclass
class BinModel:
    """What a look of one scan bin needs (B3-B6). Losses in dB; arrays of equally likely samples."""
    n: int                                  # noncoherent samples (coherent blocks)
    shape: float                            # Gamma shape of the target power over a look (inf: steady)
    threshold: float                        # of the statistic, from the bin's Pfa
    x_req_db: float                         # analytic required mean SNR at R_bin before the position losses
    x_avail_db: float                       # SNR the plan gives at R_bin before the position losses
    r_bin_km: float                         # requirement range of the bin
    short: bool                             # the plan does not reach the sector's range there
    pos_l: Tuple[float, ...]                # analytic positions law (losses, weights)
    pos_w: Tuple[float, ...]
    pfa: float
    tx_cell: float = 0.0                    # TX lattice step (lambda / D); 0: no lattice loss
    tx_beam: object = None
    rx_cell: float = 0.0
    rx_beam: object = None
    tx_samples: Optional[np.ndarray] = None  # partition beam: TX loss drawn per look from these
    range_samples: np.ndarray = field(default_factory=lambda: np.zeros(1))
    doppler_samples: np.ndarray = field(default_factory=lambda: np.zeros(1))
    velocity_samples: np.ndarray = field(default_factory=lambda: np.zeros(1))
    notes: Tuple[str, ...] = ()

    def curve(self) -> np.ndarray:
        return detection.mean_curve(self.n, detection.Fluct(self.shape), self.pfa, self.pos_l, self.pos_w)


def analytic_pcum(curve: np.ndarray, q: float, x_db: float, floor: float = rm.CUM_PD_FLOOR,
                  echo_loss: float = 0.0) -> float:
    """Cumulative Pd of the analytic model at mean SNR x_db at the requirement range (detection._required_snr_db),
    each echo lost whole with probability echo_loss (the eclipse the plan holds)."""
    if q <= 0.0:
        return float(np.interp(x_db, detection.SNR_GRID_DB, curve)) * (1.0 - echo_loss)
    sh = detection._look_shifts(round(q, 9))
    pk = np.interp(x_db - sh, detection.SNR_GRID_DB, curve, left=float(curve[0]), right=float(curve[-1]))
    pk = np.where(pk < floor, 0.0, pk) * (1.0 - echo_loss)
    return float(np.mean(1.0 - np.prod(1.0 - pk, axis=1)))


def bin_model(w, f: float, theta: float, tgt, sec: dict, pfa: float, spacing: Optional[float], reach_km: float,
              short: bool) -> BinModel:
    """The look of a planned waveform in one scan bin (same solve as the optimiser: optimizer.parts_at path)."""
    r_req = float(sec["range_km"])
    r_bin = r_req if (short or reach_km >= r_req * (1.0 - 1e-9)) else reach_km   # surface-limited bin: its distance
    cum = (float(sec["v_close_max_mps"]), float(sec["rr_target_s"]), r_bin) + mo.search_criterion(sec)[3:]
    wb = w if w.tx_mode != "superposition" else dataclasses.replace(w, b_tx=1, tx_mode="single")   # parts_at twin
    b = rm.solve_base(wb, f, theta, tgt, mo.rx_sll_db(sec), mo.velocity_span_mps(sec))
    k = rm.solve_look(b, pfa=pfa, cum=cum, grid_spacing_hpbw=spacing)
    x_req = k.snr_db + k.positions_db
    notes = []
    band = b.band.name
    pat = rm.PATTERNS[band]
    sp = rm._spacing(spacing)
    tx_cell = tx_beam = rx_cell = rx_beam = tx_samples = None
    if rm.ENABLE_GRID_LOSS:
        sub = rm.tx_subaperture(w)
        if sub is None:
            tx_cell, tx_beam = sp * pat.tx.hpbw, pat.tx
        else:
            tx_samples = rm.subaperture_grid_losses_db(band, sub, spacing)
            notes.append("partition beam: TX lattice loss drawn per look")
        rx_beam = rm.rx_beam(band, b.sll_key)
        rx_cell = rm.RX_BEAM_SPACING_HPBW * rx_beam.hpbw
    # the same samples as radar_model._fixed_position_law / position_losses (step 2.3), kept apart to be drawn
    u = (np.arange(401) + 0.5) / 401 - 0.5
    rng_s = -10.0 * np.log10(rm._range_response(b.psl)(u / rm.RANGE_GRID_OVERSAMPLING))
    dop_s = np.zeros(1)
    if b.ppb > 1:
        resp = (np.sinc(u / rm.DOPPLER_ZERO_PADDING) ** 2 if w.doppler_window_db is None
                else rm._range_response(w.doppler_window_db)(u / rm.DOPPLER_ZERO_PADDING))
        dop_s = -10.0 * np.log10(resp)
    vel_s = np.zeros(1)
    if b.bank_half > 0.0:
        vel_s = np.array([rm.bank_mismatch_loss_db(dv, w.tau_s, w.sig_bw_mhz, b.t_span, w.n, b.n_blocks == 1)
                          for dv in (np.arange(41) + 0.5) / 41 * b.bank_half])
    reach = reach_km if reach_km > 0 else r_bin
    return BinModel(n=b.n_blocks, shape=detection.shape_of(b.fl, b.n_blocks), threshold=detection.threshold(b.n_blocks, pfa),
                    x_req_db=x_req, x_avail_db=x_req + 40.0 * math.log10(reach / r_bin), r_bin_km=r_bin, short=short,
                    pos_l=tuple(k.pos_l), pos_w=tuple(k.pos_w), pfa=pfa, tx_cell=tx_cell or 0.0, tx_beam=tx_beam,
                    rx_cell=rx_cell or 0.0, rx_beam=rx_beam, tx_samples=tx_samples, range_samples=rng_s,
                    doppler_samples=dop_s, velocity_samples=vel_s, notes=tuple(notes))


# ----------------------------------------------------------------------------- the draws
# ----------------------------------------------------------------------------- truth draws (robustness, 28 Oct 2026)
@functools.lru_cache(maxsize=64)
def _aspect_table(tgt: "rm.Target", f_ghz: float) -> np.ndarray:
    """RCS (dB) of the class seen at 0..180 deg from its nose (1-deg grid): the class's table and seeker optics through
    the frontal presentation at that offset (radar_model.Target.rcs_at), the class as it is (Target.physical)."""
    return np.array([10.0 * math.log10(max(dataclasses.replace(tgt.physical(), aspect_policy="frontal",
                                                               aspect_offset_deg=float(a)).rcs_at(f_ghz), 1e-30))
                     for a in range(181)])


def draw_truth(tgt: "rm.Target", f_s: float, f_ku: float, n: int, rng: np.random.Generator, cfg: dict,
               v_min: float, v_max: float) -> Dict[str, np.ndarray]:
    """The true targets behind n draws (robustness test, user 28 October 2026): the plan is sized on the class means,
    each true target departs from them. RCS: class spread (1 sigma rcs_spread_db, truncated at rcs_trunc_sigmas) and
    aspect (cos uniform over the sphere; a class seen frontally: within frontal_cone_deg of the nose), against the RCS
    the plan holds; closing speed uniform over the sector's span; rotation (coherence time) log-uniform over
    1 / rot_factor .. rot_factor; the other Swerling case of the class with probability fluct_alt_p (look shapes x 2
    or / 2, a steady class stays steady); manoeuvre: acceleration uniform 0 .. a_max of the class (process-noise
    scale of the truth). Offsets in dB at S and Ku."""
    sig = float(tgt.rcs_spread_db)
    tr = float(cfg["rcs_trunc_sigmas"])
    sp = np.clip(rng.standard_normal(n), -tr, tr) * sig
    frontal = tgt.policy() == "frontal"
    c_min = math.cos(math.radians(float(cfg["frontal_cone_deg"]))) if frontal else -1.0
    asp = np.degrees(np.arccos(rng.uniform(c_min, 1.0, n)))
    ref_s, ref_k = 10.0 * math.log10(tgt.rcs_at(f_s)), 10.0 * math.log10(tgt.rcs_at(f_ku))
    tab_s, tab_k = _aspect_table(tgt, f_s), _aspect_table(tgt, f_ku)
    grid = np.arange(181.0)
    off_s = sp + np.interp(asp, grid, tab_s) - ref_s
    off_k = sp + np.interp(asp, grid, tab_k) - ref_k
    rot = float(cfg["rot_factor"])
    rot_f = np.exp(rng.uniform(-math.log(rot), math.log(rot), n))
    fl = tgt.fluct_s.upper().replace(" ", "")
    alt = 2.0 if fl in ("SW1", "SW2", "SW1/2") else (0.5 if fl in ("SW3", "SW4", "SW3/4") else 1.0)
    shape_f = np.where(rng.random(n) < float(cfg["fluct_alt_p"]), alt, 1.0)
    sm, _ = trk.tm.manoeuvre(tgt)
    sm_t = np.array([trk.tm.manoeuvre(dataclasses.replace(tgt, accel_max_mps2=float(a)))[0]
                     for a in rng.uniform(0.0, 1.0, n) * tgt.accel_max_mps2])
    return {"off_s_db": off_s, "off_ku_db": off_k, "aspect_deg": asp, "spread_db": sp,
            "v_mps": rng.uniform(v_min, v_max, n), "t0_factor": 1.0 / rot_f, "shape_factor": shape_f,
            "accel_scale": sm_t / sm if sm > 0 else np.ones(n)}


def draw_targets(bm: BinModel, rng: np.random.Generator, n: int, rr_s: float, v_mps: np.ndarray,
                 t0_s: float, correlated: bool = True, eclipse_rho: float = 0.0, with_snr: bool = False,
                 truth: Optional[Dict[str, np.ndarray]] = None):
    """(detected (bool, n), horizon cut) for n test targets of one bin: any look at R_bin (1 + (k + phi) q) detects;
    cut = looks that could still detect were left out by the K_MAX_LOOKS safety bound. Every look whose mean Pd stays
    above pd_look_floor is drawn, without a time bound, as the analytic criterion counts them (user, 27 October 2026:
    horizon_max_s cut the worst-range window of slow far targets; it bounds the scheduler timeline of M2 only).
    correlated False: every position loss drawn anew at each look (the analytic assumption, for the tests).
    eclipse_rho (M2-6): each echo is lost whole with this probability (the share of the common transmit slots).
    with_snr: also the signal SNR (linear) of the first look that detects (NaN when none), for the confirmation.
    truth (robustness test): the true targets' RCS offsets at S (dB), coherence-time and look-shape factors."""
    q = v_mps * rr_s / (bm.r_bin_km * 1e3)                           # (n,)
    curve = bm.curve()
    # looks that can still detect: mean Pd above the floor (at the smallest q, the most looks)
    q_min = float(q.min())
    k_cap = K_MAX_LOOKS
    k_need = 1
    if q_min > 0:
        k_try = np.arange(1, K_MAX_LOOKS + 2)
        x_top = bm.x_avail_db + (float(truth["off_s_db"].max()) if truth is not None else 0.0)   # strongest target
        pk = np.interp(x_top - 40.0 * np.log10(1.0 + (k_try - 1) * q_min), detection.SNR_GRID_DB, curve,
                       left=float(curve[0]), right=float(curve[-1]))
        alive = np.flatnonzero(pk >= PD_LOOK_FLOOR)
        k_need = int(alive.max()) + 1 if len(alive) else 1
    K = max(1, min(k_need, k_cap))
    phi = rng.random(n)
    kk = np.arange(K)[None, :] + phi[:, None]                           # (n, K)
    snr_db = bm.x_avail_db - 40.0 * np.log10(1.0 + kk * q[:, None])
    if truth is not None:
        snr_db = snr_db + truth["off_s_db"][:, None]
    loss = np.zeros((n, K))
    # angular offsets: a point of the TX cell fixed per target, seen through the frame-to-frame lattice shifts
    if bm.tx_cell > 0.0 or bm.rx_cell > 0.0:
        basis = _basis(bm.tx_cell) if bm.tx_cell > 0.0 else _basis(1.0)
        if correlated:
            p0 = rng.random((n, 2)) @ basis                              # uniform over one cell (modulo the lattice)
            j0 = rng.integers(0, N_SHIFTS, n)
            j = (j0[:, None] + np.arange(K)[None, :]) % N_SHIFTS
            pts = p0[:, None, :] - (j[..., None] / N_SHIFTS) * (basis[0] + basis[1])[None, None, :]
        else:
            pts = rng.random((n, K, 2)) @ basis
        d_tx = reduce_hex(pts, bm.tx_cell) if bm.tx_cell > 0.0 else pts
        if bm.tx_cell > 0.0:
            loss += rm._beam_loss_db(bm.tx_beam, np.hypot(d_tx[..., 0], d_tx[..., 1]))
        if bm.rx_cell > 0.0:
            d_rx = reduce_hex(d_tx, bm.rx_cell)
            loss += rm._beam_loss_db(bm.rx_beam, np.hypot(d_rx[..., 0], d_rx[..., 1]))
    if bm.tx_samples is not None:
        loss += bm.tx_samples[rng.integers(0, len(bm.tx_samples), (n, K))]
    loss += bm.range_samples[rng.integers(0, len(bm.range_samples), (n, K))]
    shape_nk = (n, 1) if correlated else (n, K)
    loss += bm.doppler_samples[rng.integers(0, len(bm.doppler_samples), shape_nk)]
    loss += bm.velocity_samples[rng.integers(0, len(bm.velocity_samples), shape_nk)]
    lam = 10.0 ** ((snr_db - loss) / 10.0)                              # mean total SNR of each look
    if math.isfinite(bm.shape) and bm.shape <= 1e4 and truth is not None:
        k_t = bm.shape * truth["shape_factor"][:, None]                  # (n, 1)
        g = rng.gamma(np.broadcast_to(k_t, (n, K)), 1.0 / np.broadcast_to(k_t, (n, K)))
        same = correlated & (rr_s < t0_s * truth["t0_factor"])            # per true target
        lam = lam * np.where(same[:, None], g[:, :1], g)
    elif math.isfinite(bm.shape) and bm.shape <= 1e4:                  # as detection.pd_curve (_STEADY_SHAPE)
        same = correlated and rr_s < t0_s
        g = rng.gamma(bm.shape, 1.0 / bm.shape, (n, 1) if same else (n, K))
        lam = lam * g
    z = 0.5 * rng.noncentral_chisquare(2 * bm.n, np.maximum(2.0 * lam, 1e-300))
    hit = z > bm.threshold
    if eclipse_rho > 0.0:
        hit &= rng.random((n, K)) >= eclipse_rho
    det = hit.any(axis=1)
    if not with_snr:
        return det, k_need > k_cap
    first = np.argmax(hit, axis=1)
    snr = np.where(det, lam[np.arange(n), first], np.nan)
    return det, k_need > k_cap, snr


# ----------------------------------------------------------------------------- confirmation (M2-4)
def _look_power(rng: np.random.Generator, shape: float, size, same: bool, n_looks: int) -> np.ndarray:
    """Target power factors (mean 1) of n_looks single-sample looks of `size` targets: Gamma(shape), one draw for all
    the looks of a target when they fall within its coherence time (same), else one per look."""
    if not (math.isfinite(shape) and shape <= 1e4):
        return np.ones((size, n_looks))
    g = rng.gamma(shape, 1.0 / shape, (size, 1) if same else (size, n_looks))
    return np.broadcast_to(g, (size, n_looks))


def _detect1(rng: np.random.Generator, snr: np.ndarray, t1: float) -> np.ndarray:
    """Exact decision of single-sample looks of mean total SNR snr (power fluctuation already in it)."""
    return 0.5 * rng.noncentral_chisquare(2, np.maximum(2.0 * snr, 1e-300)) > t1


@dataclass
class ConfirmSpec:
    """The confirmation of a sector as the plan chose it (step 3.2): mode, margins, the statistics of its cluster;
    draw() plays it for the S detections given by their signal SNR."""
    sid: str
    mode: str
    margin_db: object                       # Ku margin, or [touch, Ku] for S2
    success_analytic: float
    p_min: float
    st: "trk.ConfirmStats"
    t0_s_s: float
    t0_ku_s: float
    trip_s: float
    n_freqs: int = 1                        # K1: independent Ku frequencies the looks hop over
    attempts: int = 1                       # K3: confirmations of one detection at most
    ku_gain: object = 1.0                   # true targets' Ku RCS against the plan's (robustness test)

    def _ku17(self, rng, ex, ey, m_db: float, cluster_only: bool, attempt: int = 0) -> np.ndarray:
        st, n = self.st, len(ex)
        sx, sy = trk.attempt_shift(attempt, rm.BEAM_SPACING_HPBW)          # K11: the cluster shifted per attempt
        snr0 = st.snr_ku * rm.db_to_lin(m_db) * self.ku_gain             # ku_gain: true targets' Ku RCS (robustness)
        t1 = detection.threshold(1, rm.DETECT_PFA)
        shape = detection.shape_of(st.fl_ku, 1)
        looks = len(st.cl.beams) + 1
        # K1: the looks hop over n_freqs frequencies: one target power per frequency (shared within T0)
        nf = max(1, min(self.n_freqs, looks))
        gf = _look_power(rng, shape, n, False, nf) if nf > 1 else _look_power(rng, shape, n,
                                                                               self.trip_s < self.t0_ku_s, looks)
        g = gf[:, np.arange(looks) % nf] if nf > 1 else gf
        ok = np.zeros(n, dtype=bool)
        snr_hit = np.full(n, np.nan)                    # signal SNR of the Ku look that confirmed (M3 hand-over)
        if not cluster_only:
            x = snr0 * trk._tx_gain(ex, ey) * g[:, 0]
            ok = _detect1(rng, x, t1)
            snr_hit = np.where(ok, x, snr_hit)
        for j, (cx, cy, ku, kv) in enumerate(st.cl.beams):
            x = snr0 * st.cl.eirp * trk._tx_gain(ex - sx - cx, ey - sy - cy, ku, kv) * g[:, j + 1]
            hit = _detect1(rng, x, t1)
            snr_hit = np.where(hit & ~ok, x, snr_hit)
            ok |= hit
        self.last_snr_ku = snr_hit
        return ok

    def draw(self, snr_det: np.ndarray, rng: np.random.Generator, ku_off_db: Optional[np.ndarray] = None) -> dict:
        """Success of the confirmation of each detection (signal SNR snr_det of the S look that detected): angle error
        per axis bias + K q / sqrt(SNR) x N(0, 1) in Ku beamwidths, the mode's looks decided exactly (single sample,
        Pfa of the Ku looks, fluctuation of the class; one power draw for looks within T0). ku_off_db: the true
        targets' Ku RCS against the plan's (robustness test)."""
        st, n = self.st, len(snr_det)
        self.ku_gain = 1.0 if ku_off_db is None else 10.0 ** (np.asarray(ku_off_db) / 10.0)
        sig = trk.K_ML_S * st.q / np.sqrt(np.maximum(snr_det, 1e-12))
        ex = st.bias_ku + sig * rng.standard_normal(n)
        ey = st.bias_ku + sig * rng.standard_normal(n)
        if self.mode in ("KU1+7", "KU7"):
            ok = self._ku17(rng, ex, ey, float(self.margin_db), self.mode == "KU7")
            snr_h = self.last_snr_ku.copy()
            for a_ in range(1, self.attempts):           # K3: again on other frequencies, the same angle error
                if ok.all():
                    break
                ok2 = self._ku17(rng, ex, ey, float(self.margin_db), self.mode == "KU7", a_)
                snr_h = np.where(~ok & ok2, self.last_snr_ku, snr_h)
                ok = ok | ok2
            self.last_snr_ku = snr_h                     # the SNR of the attempt that confirmed, not the last one
        else:                                   # S2: pointed S touch, then KU1+7 on the combined estimate
            mt, mk = (float(x) for x in self.margin_db)
            t_snr = st.touch_snr0 * rm.db_to_lin(mt)
            t1 = detection.threshold(1, rm.DETECT_PFA)
            g = _look_power(rng, detection.shape_of(st.fl_s, 1), n, False, 1)[:, 0]
            touched = _detect1(rng, t_snr * trk._tx_gain(ex / st.q, ey / st.q) * g, t1)
            sig2 = trk.K_ML_S * st.q / np.sqrt(np.maximum(snr_det, 1e-12) + t_snr)
            ex2 = st.bias_ku + sig2 * rng.standard_normal(n)
            ey2 = st.bias_ku + sig2 * rng.standard_normal(n)
            ok = touched & self._ku17(rng, ex2, ey2, mk, False)
            snr_h = self.last_snr_ku.copy()
            for a_ in range(1, self.attempts):           # K3: the Ku part again, the same combined estimate
                ok2 = touched & self._ku17(rng, ex2, ey2, mk, False, a_)
                snr_h = np.where(~ok & ok2, self.last_snr_ku, snr_h)
                ok = ok | ok2
            self.last_snr_ku = snr_h
        self.handover_snr = self.last_snr_ku[ok]           # M3: hand-over accuracy of the confirmed detections
        self.last_ok = ok
        k = int(ok.sum())
        lo, hi = wilson(k, n)
        p = k / n
        if hi < self.p_min:
            state, why = "NOK", f"P confirmation {p:.3f} (upper bound {hi:.3f}) below confirm_p_min {self.p_min:g}"
        elif self.success_analytic > hi:
            state, why = "ALERTE", (f"analytic P confirmation {self.success_analytic:.3f} above the upper bound "
                                    f"{hi:.3f} (model optimistic)")
        else:
            state, why = "OK", ""
        return {"mode": self.mode, "detections": n, "p_mc": p, "ci": [lo, hi], "p_analytic": self.success_analytic,
                "p_min": self.p_min, "state": state, "why": why}


def confirm_spec(sec: dict, plan: dict, t: dict, g: dict) -> Optional[ConfirmSpec]:
    """The sector's confirmation from the tracking budget of the run (None: no admissible mode)."""
    mode = t.get("confirm_mode")
    c = t.get("confirmation") or {}
    if not mode or not c.get("admissible", True) or c.get("margin_db") is None:
        return None
    s = trk._with_mission_settings(sec, g)
    tgt = mo.sector_target(s)
    f_s, f_ku = float(plan["freq_ghz"]), float(t["ku_freq_ghz"])
    cl = next((x for x in trk.cluster_variants() if x.name == c.get("cluster_tx")), None)
    if cl is None:
        return None
    p_min = float(s.get("confirm_p_min", trk.CONFIRM_P_MIN))
    st = trk.ConfirmStats(rm.look_fluctuation(tgt, f_s, tgt.swerling, 1, 0.0),
                          rm.look_fluctuation(tgt, f_ku, tgt.swerling, 1, 0.0), f_s, f_ku,
                          plan.get("beam_spacing_hpbw"), cl, p_min)
    at = t.get("at") or []
    n_f = int(at[0].get("init_freqs") or 1) if at else trk.init_hop_freqs({}, f_ku, min(trk.KU_BW), tgt.length_m,
                                                                        trk.exclusions_ku(s))
    return ConfirmSpec(sid=sec["id"], mode=mode, margin_db=c["margin_db"], success_analytic=float(c["success"]),
                       p_min=p_min, st=st, t0_s_s=tgt.t0_s(f_s), t0_ku_s=tgt.t0_s(f_ku),
                       trip_s=2.0 * float(sec["range_km"]) * 1e3 / rm.C_MPS, n_freqs=max(1, n_f),
                       attempts=int(c.get("attempts", 1)))


# ----------------------------------------------------------------------------- one sector
def sector_mc(sec: dict, plan: dict, index: int, seed: int = SEED, correlated: bool = True,
              eclipse: Tuple[float, float] = (0.0, 0.0), confirm: Optional["ConfirmSpec"] = None,
              truth_cfg: Optional[dict] = None, f_ku: Optional[float] = None) -> dict:
    """Cumulative Pd of the test targets of one search sector, by draws, against the analytic model (M1); eclipse =
    (probability of a whole echo loss the plan holds, share of the common S slots): each echo is lost whole with that
    share (M2-6; since the return on 3.2 / 5.2 the plan holds the same law); confirm: the confirmation of every detected
    test target in range is drawn (M2-4). truth_cfg (robustness test): every test target is a true target drawn
    around the class (draw_truth), its parameters kept per target in "truth" for the sensitivity of the report."""
    t_start = time.perf_counter()
    f = float(plan["freq_ghz"])
    nb = len(plan["wf"])
    theta = mo._theta_centres(f, nb)
    tgt = mo.sector_target(sec)
    weights = np.array([float(plan["positions"][i]) if plan["wf"][i] is not None else 0.0 for i in range(nb)])
    if weights.sum() <= 0.0:
        return {"id": sec["id"], "status": "NO_PLAN"}
    models: Dict[int, BinModel] = {}
    notes = set()
    for i in np.flatnonzero(weights > 0):
        reach = float(plan["range_km"][i]) * float(plan["pulse_scale"][i]) ** 0.25
        # the criterion the plan was sized on, its eclipse included (the SNR is read from the plan through it)
        sec_p = dict(sec, _eclipse_rho=eclipse[0]) if eclipse[0] > 0.0 else sec
        models[int(i)] = bin_model(plan["wf"][i], f, float(theta[i]), tgt, sec_p, float(plan["pfa"][i]),
                                   plan.get("beam_spacing_hpbw"), reach, bool(plan["short"][i]))
        notes.update(models[int(i)].notes)
    if mo.env.spj_density_w_hz(sec, f) is not None:
        notes.add("self-protection jammer: looks scaled as noise-limited (burn-through goes as 1/r^2)")
    rr = float(sec["rr_target_s"])
    vmax = float(sec["v_close_max_mps"])
    vmin = float(sec.get("v_close_min_mps", vmax))
    t0 = tgt.t0_s(f)
    pd_req = float(sec["pd"]) if sec.get("pd") is not None else rm.DETECT_PD
    # analytic expectation, weighted over the bins (the plan's law at the plan's SNR)
    p_w = weights / weights.sum()
    an = {}
    for i, bm in models.items():
        an[i] = analytic_pcum(bm.curve(), vmax * rr / (bm.r_bin_km * 1e3), bm.x_avail_db, echo_loss=eclipse[0])
    if eclipse[0] > 0.0 or eclipse[1] > 0.0:
        notes.add(f"eclipse drawn whole per echo ({100 * eclipse[1]:.2f} % of the time), the plan holds "
                  f"{100 * eclipse[0]:.2f} %")
    snr_det: List[np.ndarray] = []
    in_range = np.array([0.0 if (i not in models or models[i].short) else weights[i] for i in range(nb)])
    pd_an = float(sum(p_w[i] * an[i] for i in models))
    pd_an_in = (float(sum(in_range[i] * an[i] for i in models) / in_range.sum()) if in_range.sum() > 0 else None)
    rng = np.random.default_rng([int(seed), int(index)])
    bins = np.array(sorted(models))
    probs = weights[bins] / weights[bins].sum()
    hits = tries = hits_in = tries_in = 0
    rec: Dict[str, List[np.ndarray]] = {}                # robustness: per true target in range, its draw and outcome
    ku_det: List[np.ndarray] = []
    while tries < N_MAX:
        nb_draw = min(BATCH, N_MAX - tries)
        pick = bins[rng.choice(len(bins), nb_draw, p=probs)]
        for i in np.unique(pick):
            m = int((pick == i).sum())
            tr = None
            if truth_cfg is not None:
                tr = draw_truth(tgt, f, float(f_ku or trk.KU_REF_GHZ), m, rng, truth_cfg, vmin, vmax)
                v = tr["v_mps"]
            else:
                v = (np.full(m, vmax) if V_DRAW != "uniform" else rng.uniform(vmin, vmax, m))
            det, cut, snr = draw_targets(models[int(i)], rng, m, rr, v, t0, correlated, eclipse[1], True, tr)
            if cut:
                notes.add(f"looks beyond {K_MAX_LOOKS} left out (pessimistic)")
            hits += int(det.sum())
            tries += m
            if not models[int(i)].short:
                hits_in += int(det.sum())
                tries_in += m
                snr_det.append(snr[det])
                if tr is not None:
                    for key_, val in tr.items():
                        rec.setdefault(key_, []).append(val)
                    rec.setdefault("detected", []).append(det)
                    ku_det.append(tr["off_ku_db"][det])
        lo, hi = wilson(hits_in, tries_in) if tries_in else wilson(hits, tries)
        if tries >= N_MIN and (hi - lo) / 2.0 < CI_TOL:
            break
    ci = wilson(hits, tries)
    ci_in = wilson(hits_in, tries_in) if tries_in else (None, None)
    pd_mc_in = hits_in / tries_in if tries_in else None
    if pd_mc_in is None:
        state, why = "NOK", "no position within range"
    elif ci_in[1] < pd_req:
        state, why = "NOK", f"Pd {pd_mc_in:.3f} (upper bound {ci_in[1]:.3f}) below the required {pd_req:g}"
    elif pd_an_in is not None and pd_an_in > ci_in[1]:
        state, why = "ALERTE", f"analytic Pd {pd_an_in:.3f} above the upper bound {ci_in[1]:.3f} (model optimistic)"
    else:
        state, why = "OK", ""
    row_conf = None
    if confirm is not None and snr_det:
        row_conf = confirm.draw(np.concatenate(snr_det), np.random.default_rng([int(seed), int(index), 1]),
                                np.concatenate(ku_det) if ku_det else None)
        if rec:
            ok = np.zeros(sum(len(x) for x in rec["detected"]), dtype=bool)
            ok[np.flatnonzero(np.concatenate(rec["detected"]))] = confirm.last_ok
            rec["confirmed"] = [ok]
    return {"id": sec["id"], "status": "OK", "pd_required": pd_req, "targets": tries, "targets_in_range": tries_in,
            "pd_mc": hits / tries, "ci": list(ci), "pd_mc_in_range": pd_mc_in, "ci_in_range": list(ci_in),
            "pd_analytic": pd_an, "pd_analytic_in_range": pd_an_in,
            "short_share": float(1.0 - in_range.sum() / weights.sum()), "state": state, "why": why,
            "v_draw": V_DRAW, "correlated": correlated, "notes": sorted(notes), "confirmation": row_conf,
            "truth": {k: np.concatenate(v).tolist() for k, v in rec.items()} if rec else None,
            "runtime_s": time.perf_counter() - t_start}


# ----------------------------------------------------------------------------- the drawn timeline (M2-1, M2-2, M2-3, M2-5, M2-7)
def poisson_ci(count: int, level: float = CI_LEVEL) -> Tuple[float, float]:
    """Interval of a Poisson mean from one count (Byar's approximation)."""
    z = _z(level)
    lo = 0.0 if count == 0 else count * (1.0 - 1.0 / (9.0 * count) - z / (3.0 * math.sqrt(count))) ** 3
    c1 = count + 1
    hi = c1 * (1.0 - 1.0 / (9.0 * c1) + z / (3.0 * math.sqrt(c1))) ** 3
    return max(0.0, lo), hi


def quantile_ci(x: np.ndarray, p: float, level: float = CI_LEVEL):
    """(quantile, lower, upper) of the p quantile of a sample, from the binomial law of the order statistics."""
    n = len(x)
    if n == 0:
        return None, None, None
    xs = np.sort(x)
    h = _z(level) * math.sqrt(n * p * (1.0 - p))
    i_lo, i_hi = int(max(0, math.floor(n * p - h) - 1)), int(min(n - 1, math.ceil(n * p + h)))
    return float(np.quantile(xs, p)), float(xs[i_lo]), float(xs[i_hi])


def _greedy(tasks: List[tuple], lim: float, ratio: float) -> Tuple[List[float], int]:
    """Start delay of every task (same order as `tasks`) and the recharge pauses: the rule of scheduler._timeline,
    greedy by priority then deadline, no preemption, a pause of busy x ratio after lim of contiguous transmission.
    A task: (release, prio, deadline, duration)."""
    order = sorted(range(len(tasks)), key=lambda i: tasks[i][0])
    delay = [0.0] * len(tasks)
    ready: List[tuple] = []
    i, t, busy_run, pauses = 0, 0.0, 0.0, 0
    while i < len(order) or ready:
        while i < len(order) and tasks[order[i]][0] <= t:
            rel, prio, dl, dur = tasks[order[i]]
            heapq.heappush(ready, (prio, dl, rel, dur, order[i]))
            i += 1
        if not ready:
            t = tasks[order[i]][0]
            busy_run = 0.0
            continue
        prio, dl, rel, dur, idx = heapq.heappop(ready)
        if busy_run + dur > lim and busy_run > 0:
            t += busy_run * ratio
            pauses += 1
            busy_run = 0.0
        delay[idx] = t - rel
        t += dur
        busy_run += dur
    return delay, pauses


def _arrivals(rng: np.random.Generator, k: int, classes, mission: dict, out: dict,
              t_end: float) -> Dict[str, List[Tuple[float, bool]]]:
    """Confirmation arrivals of face k per sector over [0, t_end] as (time, true): false alarms drawn per search dwell
    (Poisson of the analytic mean per dwell, M2-3), true detections Poisson at new_targets_per_rr (+ objects_per_rr)
    x the face's share per revisit."""
    secs = {s["id"]: s for s in mission["sectors"]}
    tsec = {s["id"]: s for s in out.get("tracking", {}).get("sectors", [])}
    arr: Dict[str, List[Tuple[float, bool]]] = {}
    for c in classes["S"]:
        if c.kind != "search" or c.sector not in tsec:
            continue
        t, sec = tsec[c.sector], secs[c.sector]
        rr = float(sec["rr_target_s"])
        fa = float(t["false_alarms_per_rr_face"][k])
        n_dw = max(1, len(c.dwells) if c.dwells else int(round(c.count)))
        step = rr / n_dw
        times = np.arange(rng.random() * step, t_end, step)
        counts = rng.poisson(fa / n_dw, len(times)) if fa > 0 else np.zeros(len(times), dtype=int)
        lst = [(float(x), False) for x, m in zip(times, counts) for _ in range(int(m))]
        n_new = float(sec.get("new_targets_per_rr", 1.0)) + float(sec.get("objects_per_rr", 0.0))
        rate = n_new * float(t["p_share"][k]) / rr
        if rate > 0:
            lst += [(float(x), True) for x in rng.uniform(0.0, t_end, rng.poisson(rate * t_end))]
        arr[c.sector] = sorted(lst)
    return arr


def face_timeline(mission: dict, out: dict, k: int, rep: int, seed: int, scans) -> dict:
    """One drawn timeline of face k, both bands: the tasks of scheduler._classes (calibration, search dwells in scan
    order, periodic updates, Poisson initiations and burn-through), the confirmations from the drawn arrivals (after
    the processing of the detection). Counted window: after WARMUP_REVISITS longest revisits of the face, for
    HORIZON_REVISITS of them, at most horizon_max_s. Returns per band the waits per class, late updates, pauses; per
    sector the false alarms counted and, per true detection, its (wait, service, counted) per band."""
    rng = np.random.default_rng([int(seed), 1000 + k, int(rep)])
    classes = sch._classes(k, mission, out, scans)
    r_max = max([c.period for c in classes["S"] if c.kind == "search" and c.period] + [1.0])
    warm = WARMUP_REVISITS * r_max
    horizon = min(HORIZON_MAX_S, HORIZON_REVISITS * r_max)
    t_end = warm + horizon
    arr = _arrivals(rng, k, classes, mission, out, t_end)
    res = {"S": {}, "Ku": {}, "confirm": {}, "window_s": horizon,
           "fa": {sid: sum(1 for x, true in lst if not true and x >= warm) for sid, lst in arr.items()}}
    proc = {sid: sch.compute.processing_s(out, sid) for sid in arr}
    for b in ("S", "Ku"):
        tasks, meta = [], []

        def add(rel, prio, dl, dur, lab, kind, tag=None):
            tasks.append((float(rel), prio, float(dl), float(dur)))
            meta.append((lab, kind, tag))
        for c in classes[b]:
            if c.s <= 0 or c.rho <= 0:
                continue
            if c.kind == "cal":
                add(0.0, c.prio, c.period * sch.CAL_MAX_DELAY_FRAC, c.s, c.label, c.kind)
            elif c.kind == "search" and c.dwells:
                step = c.period / len(c.dwells)
                for j, t in enumerate(np.arange(rng.random() * step, t_end, step)):
                    add(t, c.prio, t + c.period, c.dwells[j % len(c.dwells)], c.label, c.kind)
            elif c.kind in ("search", "update"):
                if c.count <= 0:
                    continue
                step = c.period / c.count
                for t in np.arange(rng.random() * step, t_end, step):
                    add(t, c.prio, t + (c.period if c.kind == "search" else sch.UPDATE_JITTER_FRAC * c.period),
                        c.s, c.label, c.kind)
            elif c.kind == "confirm":
                for n, (t, true) in enumerate(arr.get(c.sector, [])):
                    rel = t + proc.get(c.sector, 0.0)
                    add(rel, c.prio, rel + 1e9, c.s, c.label, c.kind, (c.sector, n, true, t >= warm))
            elif c.rate > 0:
                for t in rng.uniform(0.0, t_end, rng.poisson(c.rate * t_end)):
                    add(t, c.prio, t + 1e9, c.s, c.label, c.kind)
        if not tasks:
            continue
        lim, ratio = sch._contiguous(k, b, out.get("tracking", {}))
        delay, pauses = _greedy(tasks, lim, ratio)
        waits: Dict[str, List[float]] = {}
        late = n_upd = 0
        for (rel, prio, dl, dur), (lab, kind, tag), w in zip(tasks, meta, delay):
            if tag is not None and tag[2]:
                res["confirm"].setdefault(tag[0], {}).setdefault(tag[1], {})[b] = (w, dur, tag[3])
            if rel < warm:
                continue
            waits.setdefault(lab, []).append(w)
            if kind == "update":
                n_upd += 1
                late += (rel + w > dl)
        res[b] = {"mean_wait_s": {lab: float(np.mean(v)) for lab, v in waits.items()},
                  "max_delay_s": {lab: float(np.max(v)) for lab, v in waits.items()},
                  "late_updates": late, "updates": n_upd, "pauses": pauses}
    return res


def mission_scans(mission: dict, out: dict) -> dict:
    """The scan of every planned search sector (step 4.3 bis), as the drawn timelines play them."""
    from . import scanorder as so
    secs = {s["id"]: s for s in mission["sectors"]}
    g = out.get("global", {})
    sph = mo._sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    sky = mission.get("global", {}).get("sky")
    return {r["id"]: so.sector_scan(secs[r["id"]], r, sph, sky) for r in out.get("sectors", [])
            if r.get("status") == "OK" and r["id"] in secs}


def timelines(mission: dict, out: dict, seed: int = SEED, progress_cb=None,
              runs: Optional[Dict[Tuple[int, int], dict]] = None) -> dict:
    """N_REP drawn timelines of every face: per sector the false alarms per revisit and the confirmation latency of the
    true detections (95 % quantile and its interval), per face and band the waits, late updates and pauses, against
    the analytic scheduler of step 4.3 (M2-5, M2-7, M2-8). runs: the timelines drawn already (face, rep), in parallel."""
    secs = {s["id"]: s for s in mission["sectors"]}
    order = [s["id"] for s in mission["sectors"]]
    scans = mission_scans(mission, out) if runs is None else None
    tsec = {s["id"]: s for s in out.get("tracking", {}).get("sectors", [])}
    ssec = {s["id"]: s for s in out.get("scheduler", {}).get("sectors", [])}
    fa_by: Dict[str, Dict[int, List[float]]] = {}          # per sector and face: [false alarms, revisits counted]
    lat: Dict[str, List[float]] = {}
    faces = {}
    for k in range(4):
        per = {"S": {}, "Ku": {}}
        upd = {"S": [0, 0, 0], "Ku": [0, 0, 0]}
        for rep in range(N_REP):
            r = face_timeline(mission, out, k, rep, seed, scans) if runs is None else runs[(k, rep)]
            for sid, n in r["fa"].items():
                e = fa_by.setdefault(sid, {}).setdefault(k, [0, 0.0])
                e[0] += n
                e[1] += r["window_s"] / float(secs[sid]["rr_target_s"])
            for sid, by in r["confirm"].items():
                t = tsec.get(sid, {})
                trips = float((t.get("confirmation") or {}).get("round_trips", 1) or 1)
                rtt = 2.0 * float(secs[sid]["range_km"]) * 1e3 / rm.C_MPS
                proc = sch.compute.processing_s(out, sid)
                for parts in by.values():
                    if all(x[2] for x in parts.values()):
                        lat.setdefault(sid, []).append(sum(w + s for w, s, _ in parts.values()) + trips * rtt + proc)
            for b in ("S", "Ku"):
                rb = r.get(b) or {}
                for lab, v in (rb.get("mean_wait_s") or {}).items():
                    e = per[b].setdefault(lab, {"mean": [], "max": 0.0})
                    e["mean"].append(v)
                    e["max"] = max(e["max"], rb["max_delay_s"][lab])
                for j, key in enumerate(("late_updates", "updates", "pauses")):
                    upd[b][j] += rb.get(key, 0)
        if progress_cb:
            progress_cb(k + 1, 4, f"F{k + 1}")
        an = out.get("scheduler", {}).get("faces", {}).get(str(k)) or {}
        fk = {}
        for b in ("S", "Ku"):
            late, n_upd, pauses = upd[b]
            rows = []
            for lab, v in sorted(per[b].items()):
                m = np.array(v["mean"])
                mu = float(m.mean())
                h = (_z(CI_LEVEL) * float(m.std(ddof=1)) / math.sqrt(len(m))) if len(m) > 1 else math.inf
                a = (an.get(b, {}).get("mean_wait_s") or {}).get(lab)
                # information only (user, 27 October 2026): the states are on the latency and the late updates
                rows.append({"class": lab, "mean_wait_mc_s": mu, "ci": [mu - h, mu + h], "mean_wait_analytic_s": a,
                             "max_delay_mc_s": v["max"], "state": "INFO", "why": ""})
            late_an = (an.get(b, {}).get("timeline") or {}).get("late_update_frac")
            frac = late / n_upd if n_upd else 0.0
            bad = frac > 0.0 and not late_an
            fk[b] = {"classes": rows, "late_update_frac_mc": frac, "late_update_frac_analytic": late_an,
                     "updates": n_upd, "recharge_pauses": pauses, "state": "ALERTE" if bad else "OK",
                     "why": f"{100 * frac:.2f} % of the track updates late (none in the analytic timeline)" if bad else ""}
        faces[str(k)] = fk
    sectors = []
    for sid in sorted(set(fa_by) | set(lat), key=order.index):
        sec = secs[sid]
        req_fa = mo.fa_per_rr(sec)
        by = fa_by.get(sid, {})
        c = int(sum(v[0] for v in by.values()))
        fa_mc = sum(v[0] / v[1] for v in by.values() if v[1] > 0) if by else None
        # interval: Poisson on the total count over the effective exposure (faces counted over different windows)
        u = (c / fa_mc) if (fa_mc and c) else min((v[1] for v in by.values()), default=0.0)
        lo, hi = poisson_ci(c)
        an_fa = float(tsec.get(sid, {}).get("false_alarms_per_rr", 0.0) or 0.0)
        ci_fa = [lo / u, hi / u] if u else [None, None]
        # information only (user, 27 October 2026): drawn as Poisson of the analytic mean (M2-3), the count cannot
        # disagree with the model; the analytic figure above fa_per_rr is an open point (Pfa rounding)
        st_fa, why_fa = "INFO", (f"analytic {an_fa:.3g} above the {req_fa:g} tolerated" if an_fa > req_fa * (1 + 1e-9)
                                 else "")
        x = np.array(lat.get(sid, []))
        q, qlo, qhi = quantile_ci(x, 0.95)
        req = sch.latency_requirement_s(sec)
        a_lat = ssec.get(sid, {}).get("latency_q95_s")
        if q is None:
            st_l, why_l = "OK", "no true detection drawn"
        elif qlo > req:
            st_l, why_l = "NOK", f"latency {q:.3g} s (95 %, lower bound {qlo:.3g}) above the {req:.3g} s required"
        elif a_lat is not None and a_lat < qlo:
            st_l, why_l = "ALERTE", f"analytic latency {a_lat:.3g} s below the interval (model optimistic)"
        else:
            st_l, why_l = "OK", ""
        sectors.append({"id": sid, "fa_per_rr_mc": fa_mc, "fa_ci": ci_fa, "fa_per_rr_analytic": an_fa,
                        "fa_per_rr_required": req_fa, "fa_state": st_fa, "fa_why": why_fa,
                        "latency_q95_mc_s": q, "latency_ci": [qlo, qhi], "latency_q95_analytic_s": a_lat,
                        "latency_required_s": req, "detections": int(len(x)), "latency_state": st_l,
                        "latency_why": why_l})
    return {"n_rep": N_REP, "faces": faces, "sectors": sectors}


# ----------------------------------------------------------------------------- tracks (M3: T1-T11)
def _hex_rings(rings: int) -> np.ndarray:
    """Offsets (unit spacing) of the sites of a hexagonal lattice, ring by ring from the centre (lever D)."""
    pts = [(0.0, 0.0)]
    a, b = np.array([1.0, 0.0]), np.array([0.5, math.sqrt(3.0) / 2.0])
    for r in range(1, rings + 1):
        ring = []
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if max(abs(i), abs(j), abs(i + j)) == r:
                    ring.append(tuple(i * a + j * b))
        ring.sort(key=lambda p: math.atan2(p[1], p[0]))
        pts.extend(ring)
    return np.array(pts)


_HEX37 = _hex_rings(3)
@dataclass
class TrackLook:
    """The update look of one bracket at one range, as the track budget chose it (step 3.3), for the draws: its
    detection law (n, shape, threshold), its SNR before the position losses, the samples of its range / Doppler /
    velocity-filter losses, the TX and RX beams, the cluster offsets (cluster mode), the mean SNR it delivers at the
    steady pointing (reference of the measurement accuracies)."""
    n: int
    shape: float
    threshold: float
    x_db: float
    range_samples: np.ndarray
    doppler_samples: np.ndarray
    velocity_samples: np.ndarray
    tx_beam: object
    rx_beam: object
    hpbw_lamd: float                        # Ku TX beamwidth in lambda / D (offsets in beamwidths -> beam loss)
    cluster: Optional[np.ndarray]           # (7, 2) beam centres in TX beamwidths, None: one pointed look
    rx_cell_samples: Optional[np.ndarray]   # cluster: RX lattice loss samples (digital RX lattice of the search)
    snr_ref: float = 1.0


def track_look(upd: dict, st, mc: dict, pointing_hpbw: Optional[float] = None) -> TrackLook:
    """TrackLook of an update chosen by tracking._track_update (same parts as tracking._coast_pd); pointing_hpbw: the
    pointing law the look was sized on (None = the steady one of st), reference of its accuracies."""
    look = upd["look"]
    sp, j = look["space"], look["index"]
    d = sp.det[j]
    p0 = sp.det_parts[j]
    margin = 40.0 * math.log10(p0[0] / sp.rn[j]) if sp.rn[j] > 0 and p0[0] > sp.rn[j] else 0.0
    tgt_t = dataclasses.replace(trk.tm.track_target(mc["tgt"], st), name="ku")
    b = rm.solve_base(d, mc["f_ku"], 0.0, tgt_t, mc["sll"], round(st.v_span_mps, 1))
    u = (np.arange(401) + 0.5) / 401 - 0.5
    rng_s = -10.0 * np.log10(rm._range_response(b.psl)(u / rm.RANGE_GRID_OVERSAMPLING))
    dop_s = np.zeros(1)
    if b.ppb > 1:
        resp = (np.sinc(u / rm.DOPPLER_ZERO_PADDING) ** 2 if d.doppler_window_db is None
                else rm._range_response(d.doppler_window_db)(u / rm.DOPPLER_ZERO_PADDING))
        dop_s = -10.0 * np.log10(resp)
    vel_s = np.zeros(1)
    if b.bank_half > 0.0:
        vel_s = np.array([rm.bank_mismatch_loss_db(dv, d.tau_s, d.sig_bw_mhz, b.t_span, d.n, b.n_blocks == 1)
                          for dv in (np.arange(41) + 0.5) / 41 * b.bank_half])
    pat = rm.PATTERNS["Ku"]
    rxb = rm.rx_beam("Ku", mc["sll"])
    cluster = rx_cells = None
    if upd["mode"] == "cluster":
        s_ = rm.BEAM_SPACING_HPBW
        cluster = np.array([[0.0, 0.0]] + [[s_ * math.cos(a), s_ * math.sin(a)] for a in np.radians(np.arange(0, 360, 60))])
        rx = rm._hex_cell_points(rm.RX_BEAM_SPACING_HPBW * rxb.hpbw)
        rx_cells = rm._beam_loss_db(rxb, np.hypot(rx[:, 0], rx[:, 1]))
    tl = TrackLook(n=b.n_blocks, shape=detection.shape_of(b.fl, b.n_blocks),
                   threshold=detection.threshold(b.n_blocks, rm.DETECT_PFA),
                   x_db=p0[2] + margin + look.get("snr_gain_db", 0.0),     # kappa approximation: energy x kappa
                   range_samples=rng_s, doppler_samples=dop_s, velocity_samples=vel_s, tx_beam=pat.tx, rx_beam=rxb,
                   hpbw_lamd=pat.tx.hpbw, cluster=cluster, rx_cell_samples=rx_cells)
    # reference SNR of the accuracies: the mean signal SNR the look gives at the steady pointing law (no fluctuation)
    rng = np.random.default_rng(0)
    e = (st.pointing_hpbw if pointing_hpbw is None else pointing_hpbw) * rng.standard_normal((4000, 2))
    tl.snr_ref = float(np.mean(10.0 ** ((tl.x_db - _track_loss(tl, e, rng)) / 10.0)))
    return tl


def _track_loss(tl: TrackLook, e: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Position losses (dB) of looks pointed with errors e (..., 2) in TX beamwidths; a cluster: its best beam."""
    n = e.shape[:-1]
    draw = lambda a: a[rng.integers(0, len(a), n)]                       # noqa: E731
    base = draw(tl.range_samples) + draw(tl.doppler_samples) + draw(tl.velocity_samples)
    if tl.cluster is None:
        r = np.hypot(e[..., 0], e[..., 1]) * tl.hpbw_lamd
        return base + rm._beam_loss_db(tl.tx_beam, r) + rm._beam_loss_db(tl.rx_beam, r)
    d = e[..., None, :] - tl.cluster                                      # (..., 7, 2)
    r = np.hypot(d[..., 0], d[..., 1]) * tl.hpbw_lamd
    return base + rm._beam_loss_db(tl.tx_beam, r).min(axis=-1) + draw(tl.rx_cell_samples)


def accuracy_kappa(tl: TrackLook, pointing_hpbw: float, n: int = 4000) -> float:
    """kappa = E[snr_ref / SNR | detection] of a look (return on 3.3 from step 5.4): the measurement variance of the
    looks that detect, against its value at the look's mean SNR snr_ref, with the pointing error (Gaussian per axis,
    pointing_hpbw), the position losses and the target fluctuation (Gamma of the look's shape) drawn as in
    simulate_bracket; each draw weighed by its exact probability of detection. Fixed seed: the analytic budget is
    deterministic."""
    from scipy.stats import ncx2
    rng = np.random.default_rng(12345)
    e = max(float(pointing_hpbw), 0.0) * rng.standard_normal((n, 2))
    lin = 10.0 ** ((tl.x_db - _track_loss(tl, e, rng)) / 10.0)
    k = float(tl.shape)
    g = rng.gamma(k, 1.0 / k, n) if math.isfinite(k) and k <= 1e4 else np.ones(n)
    lam = np.maximum(lin * g, 1e-300)
    w = ncx2.sf(2.0 * tl.threshold, 2 * tl.n, 2.0 * lam)
    if w.sum() <= 0.0:
        return 1.0
    return max(1.0, float((w * tl.snr_ref / lam).sum() / w.sum()))


def miss_chain_pds(tl: TrackLook, st, nav_rad: float, n_lost: int, t0_s: float, n: int = 4000) -> List[float]:
    """Pd of the look in each state of the miss chain (k consecutive misses so far), CONDITIONED on those misses
    (return on 3.3 from step 5.4): the predicted cross-range errors of the looks are drawn jointly, the first from the
    steady prediction st.p_cross, each next one coasted from it (F x + w over the period), so a large filter error
    gives misses in a row as in the draws; navigation per look; target power the same within T0 (period below T0),
    else a new one. pds[k] = 1 - E[prod_{j<=k} miss_j] / E[prod_{j<k} miss_j]. Fixed seed (deterministic)."""
    from scipy.stats import ncx2
    rng = np.random.default_rng(54321)
    f, q = st.f, st.q
    lp = np.linalg.cholesky(st.p_cross + 1e-30 * np.eye(3))
    lq = np.linalg.cholesky(q + 1e-30 * np.eye(3))
    x = [np.einsum("ij,naj->nai", lp, rng.standard_normal((n, 2, 3)))]          # (n, axes, 3)
    for _ in range(n_lost - 1):
        x.append(np.einsum("ij,naj->nai", f, x[-1]) + np.einsum("ij,naj->nai", lq, rng.standard_normal((n, 2, 3))))
    k = float(tl.shape)
    fl = math.isfinite(k) and k <= 1e4
    same = math.isfinite(t0_s) and st.t_s < t0_s
    g = rng.gamma(k, 1.0 / k, n) if fl else np.ones(n)
    miss = np.ones(n)
    prods = [1.0]
    for xk in x:
        e = (xk[:, :, 0] / st.r_m + nav_rad * rng.standard_normal((n, 2))) / st.hpbw_rad
        if fl and not same:
            g = rng.gamma(k, 1.0 / k, n)
        lam = np.maximum(10.0 ** ((tl.x_db - _track_loss(tl, e, rng)) / 10.0) * g, 1e-300)
        miss = miss * (1.0 - ncx2.sf(2.0 * tl.threshold, 2 * tl.n, 2.0 * lam))
        prods.append(float(miss.mean()))
    return [1.0 - (prods[i + 1] / prods[i] if prods[i] > 0 else 1.0) for i in range(n_lost)]


def _sqrtm(q: np.ndarray) -> np.ndarray:
    w, v = np.linalg.eigh(0.5 * (q + q.T))
    return v * np.sqrt(np.maximum(w, 0.0))


def _kf_update(x, p, z, r_diag, h):
    """Batch Kalman update: x (n, 3), p (n, 3, 3), z (n, m), r_diag (n, m), h (m, 3); rows where r_diag is NaN skip."""
    s = np.einsum("ij,njk,lk->nil", h, p, h) + np.einsum("ni,ij->nij", np.nan_to_num(r_diag, nan=1.0), np.eye(len(h)))
    k = np.einsum("nij,kj->nik", p, h) @ np.linalg.inv(s)
    y = z - np.einsum("ij,nj->ni", h, x)
    upd = ~np.isnan(r_diag[:, 0])
    x_new = x + np.einsum("nij,nj->ni", k, y)
    p_new = p - k @ np.einsum("ij,njk->nik", h, p)
    return np.where(upd[:, None], x_new, x), np.where(upd[:, None, None], p_new, p)


def simulate_bracket(states: list, mc: dict, n_tracks: int, n_upd: int, rng: np.random.Generator,
                     rho_ku: float, handover_snr: Optional[np.ndarray], steady_start: bool = False,
                     snr_scaled: bool = True, refs: Optional[dict] = None, truth_cfg: Optional[dict] = None) -> dict:
    """Tracks of one bracket drawn with the Singer Kalman filter running (T1-T9), each at one of the bracket's ranges:
    truth on three axes, each look decided at the real pointing error with the drawn SNR, measurements from that SNR,
    loss after n_miss_lost consecutive misses. steady_start False: tracks born from the confirmation hand-over and
    initiated AS FAST AS POSSIBLE (user, 27 October 2026): each initiation interval is the longest keeping the
    predicted pointing error below init_pointing_max_hpbw, between the round trip + processing and the bracket period,
    with the initiation look (the budget's update look sized for that pointing); a lost track is initiated again.
    steady_start True: tracks started in the steady state of the filter (filtered covariance), restarted so after a
    loss: the loss per update and the accuracies of established tracks, apart from the initiation.
    snr_scaled False: measurement errors at their design values whatever the drawn SNR (diagnostic).
    Return on 3.3 (H10): the initiation looks hop over the bracket's independent Ku frequencies (one target power per
    frequency, renewed every T0), or wait T0 after a miss when only one is free; refs (A b): the accuracies the filter
    really holds in steady state per range (from the steady-start population), the initiation ending within
    init_conv_frac of them. truth_cfg (robustness test): each track follows a true target drawn around the class
    (draw_truth: Ku RCS, look shapes, coherence time, manoeuvre of the truth), kept per track in "truth"."""
    tm = trk.tm
    nav_deg = mc["st_kw"].get("nav_axis_deg")
    nav_rad = math.radians(trk.repoint.axis_sigma_deg() if nav_deg is None else float(nav_deg))
    h1 = np.array([[1.0, 0.0, 0.0]])
    h2 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    pick = rng.integers(0, len(states), n_tracks)
    tot = {"updates": 0, "losses": 0, "init_done": [], "init_time": [], "init_lost": 0, "inits": 0,
           "err_angle2": 0.0, "err_r2": 0.0, "err_vr2": 0.0, "n_err": 0, "n_hit": 0, "dt_min_s": None,
           "hits": 0, "point2": 0.0, "i_looks": 0, "i_hits": 0, "i_point2": 0.0, "i_dt": [], "i_fail": [0, 0, 0],
           "i_mode": None, "refs": {}}
    req = (None, None, None)
    for si in np.unique(pick):
        rk, st, upd = states[int(si)]
        n = int((pick == si).sum())
        tl_upd = track_look(upd, st, mc)
        tl_init = tl_upd
        st_init = st
        nf_state = None
        tot["i_mode"] = "update look (no initiation look found)"
        if not steady_start and mc.get("gr") is not None:
            iu, nf_state = trk.init_look(mc["gr"], mc["tgt"], st, rk, mc["ctx"], mc.get("r") or {})
            if iu is not None:
                tl_init = track_look(iu, st, mc, max(tm.INIT_POINTING_MAX_HPBW, st.pointing_hpbw))
                st_init = iu.get("state", st)            # the initiation look's noise parts (its kappa)
                tot["i_mode"] = f"{iu['mode']} look, SNR {tl_init.x_db:.2f} dB (update look {tl_upd.x_db:.2f} dB)"
        tl_res = None                                    # lever D: one beam of a local re-search after a miss
        if not steady_start and mc.get("gr") is not None:
            rs = trk._research_look(mc["gr"], mc["tgt"], trk.research_state(st), rk, mc["ctx"])
            if rs is not None:
                tl_res = track_look({"look": rs, "mode": "single"}, st, mc)
        n_fail_init = 1 + tm.RESEARCH_ATTEMPTS
        r_m, hp, T = st.r_m, st.hpbw_rad, st.t_s
        tgt = mc["tgt"]
        t0 = tgt.t0_s(mc["f_ku"])
        # time grid of the intervals: the round trip + processing up to the period (the period is the last entry)
        dt_min = min(T, 2.0 * r_m / rm.C_MPS + mc.get("proc_s", 0.0))
        tot["dt_min_s"] = dt_min
        extra = [t0] if (math.isfinite(t0) and dt_min < t0 < T) else []
        dts = np.unique(np.concatenate([np.geomspace(dt_min, T, 48), [T], extra])) if dt_min < T else np.array([T])
        n_f = max(1, min(int(nf_state or mc.get("init_freqs", 1) or 1), 8))   # H1: independent Ku frequencies
        k_t0 = min(int(np.searchsorted(dts, t0 - 1e-12)) if math.isfinite(t0) else len(dts) - 1, len(dts) - 1)
        fq = [tm.singer(float(d), st.sigma_m, st.tau_s) for d in dts]
        fs = np.array([x[0] for x in fq])
        qs = np.array([x[1] for x in fq])
        sqs = np.array([_sqrtm(q) for q in qs])
        k_t = len(dts) - 1
        sa0 = tgt.accel_max_mps2 / math.sqrt(3.0) if tgt.accel_max_mps2 > 0 else tm.FLOOR_MPS2
        vt0 = tm.init_vt_sigma(tgt)                                  # lever C
        s_r0 = st.allowed["r"] if math.isfinite(st.allowed["r"]) else 1e3
        s_v0 = st.allowed["vr"]
        f_t, q_t = fs[k_t], qs[k_t]
        zone_lim = st.zone_sigma_rad * r_m * (1.0 + tm.INIT_CONV_FRAC)
        r_lim = (math.inf if st.range_acc_m is None else st.range_acc_m) * (1.0 + tm.INIT_CONV_FRAC)
        v_lim = st.vr_acc_mps * (1.0 + tm.INIT_CONV_FRAC)
        if refs and int(si) in refs:                     # A (b): the accuracies the filter really holds, when worse
            ref = refs[int(si)]                          # than required (never stricter than the requirement)
            zone_lim = max(zone_lim, ref[0] * (1.0 + tm.INIT_CONV_FRAC))
            r_lim = max(r_lim, ref[1] * (1.0 + tm.INIT_CONV_FRAC)) if math.isfinite(r_lim) else r_lim
            v_lim = max(v_lim, ref[2] * (1.0 + tm.INIT_CONV_FRAC))
        acc_ref = np.zeros(4)                            # steady start: sums of the accuracies the filter holds
        req = (st.zone_sigma_rad, st.range_acc_m, st.vr_acc_mps)
        noise_r = st.noise_r if math.isfinite(st.noise_r) else s_r0
        noise_r_init = st_init.noise_r if math.isfinite(st_init.noise_r) else s_r0

        def priors(m):
            """Covariances of new tracks (cross, cross, range): hand-over, or the filter's steady state."""
            if steady_start:
                pc = np.broadcast_to(st.pf_cross, (m, 3, 3)).copy()
                pr = np.broadcast_to(st.pf_range, (m, 3, 3)).copy()
                return pc, pc.copy(), pr
            if handover_snr is not None and len(handover_snr):
                snr = handover_snr[rng.integers(0, len(handover_snr), m)]
                sth = trk.K_ML_KU * math.radians(rm.hpbw_deg(mc["f_ku"])) / np.sqrt(np.maximum(snr, 1e-12))
            else:
                sth = np.full(m, mc["hand_theta_rad"])
            pc = np.zeros((m, 3, 3))
            pc[:, 0, 0], pc[:, 1, 1], pc[:, 2, 2] = (sth * r_m) ** 2, vt0 ** 2, sa0 ** 2
            pr = np.zeros((m, 3, 3))
            pr[:, 0, 0], pr[:, 1, 1], pr[:, 2, 2] = s_r0 ** 2, s_v0 ** 2, sa0 ** 2
            return pc, pc.copy(), pr

        truth = [np.zeros((n, 3)) for _ in range(3)]
        est = [np.zeros((n, 3)) for _ in range(3)]
        cov = [np.zeros((n, 3, 3)) for _ in range(3)]
        misses = np.zeros(n, dtype=int)
        in_init = np.zeros(n, dtype=bool)
        init_looks = np.zeros(n, dtype=int)
        init_t = np.zeros(n)
        g_f = np.ones((n, n_f))                          # target power per Ku frequency (H1-H2)
        t_f = np.full((n, n_f), np.inf)
        ci = np.zeros(n, dtype=int)                      # frequency of the next initiation look
        tr = draw_truth(tgt, mc["f_ku"], mc["f_ku"], n, rng, truth_cfg, 0.0, 0.0) if truth_cfg is not None else None
        gain_t = 10.0 ** (tr["off_ku_db"] / 10.0) if tr is not None else np.ones(n)
        shp_t = tr["shape_factor"] if tr is not None else np.ones(n)
        t0_t = t0 * (tr["t0_factor"] if tr is not None else np.ones(n))
        acc_t = tr["accel_scale"] if tr is not None else np.ones(n)
        loss_t, upd_t = np.zeros(n), np.zeros(n)          # per track: losses and updates (steady), for the report

        def start(idx):
            """(Re)start the tracks idx: truth drawn from the prior around an estimate at 0."""
            m = len(idx)
            for ax, p0 in enumerate(priors(m)):
                truth[ax][idx] = np.einsum("nij,nj->ni", np.linalg.cholesky(p0 + 1e-30 * np.eye(3)),
                                           rng.standard_normal((m, 3)))
                est[ax][idx] = 0.0
                cov[ax][idx] = p0
            misses[idx] = 0
            in_init[idx] = not steady_start
            init_looks[idx] = 0
            init_t[idx] = 0.0
            t_f[idx] = np.inf
            ci[idx] = 0
            if not steady_start:
                tot["inits"] += m
        start(np.arange(n))
        for u in range(n_upd):
            # interval of each track: the period, or (initiation) the longest keeping the predicted pointing error low
            k = np.full(n, k_t)
            if in_init.any() and len(dts) > 1:
                p00 = np.maximum(np.einsum("ka,nab,kb->nk", fs[:, 0, :], cov[0], fs[:, 0, :]),
                                 np.einsum("ka,nab,kb->nk", fs[:, 0, :], cov[1], fs[:, 0, :])) + qs[None, :, 0, 0]
                ok = np.sqrt(p00 / r_m ** 2 + nav_rad ** 2) / hp <= INIT_POINTING_MAX_HPBW
                k_init = np.where(ok.any(axis=1), (len(dts) - 1) - np.argmax(ok[:, ::-1], axis=1), 0)
                if n_f == 1:                             # H3: one frequency, wait T0 after a miss
                    k_init = np.where(misses > 0, np.maximum(k_init, k_t0), k_init)
                k = np.where(in_init, k_init, k_t)
            dt = dts[k]
            for kk in np.unique(k):
                sel = k == kk
                for ax in range(3):
                    truth[ax][sel] = (truth[ax][sel] @ fs[kk].T
                                      + (rng.standard_normal((int(sel.sum()), 3)) @ sqs[kk].T) * acc_t[sel, None])
                    est[ax][sel] = est[ax][sel] @ fs[kk].T
                    cov[ax][sel] = fs[kk] @ cov[ax][sel] @ fs[kk].T + qs[kk]
            e = np.stack([(truth[a][:, 0] - est[a][:, 0]) / r_m + nav_rad * rng.standard_normal(n) for a in (0, 1)], -1) / hp
            steady = ~in_init
            tot["err_angle2"] += float((((truth[0][:, 0] - est[0][:, 0]) / r_m) ** 2)[steady].sum())
            # target power per frequency: a new draw once T0 has passed since its last one; initiation looks hop
            cur = np.where(in_init, ci, 0)
            rows = np.arange(n)
            # each look draws with its OWN shape (a frequency-agile initiation train averages several independent
            # samples: far fewer deep fades than one pulse; fixed after the pd_init sweep, 27 October 2026)
            shp = np.where(in_init, tl_init.shape, tl_upd.shape) * shp_t
            fin = np.isfinite(shp) & (shp <= 1e4)
            if fin.any():
                t_f += dt[:, None]
                new = ~np.isfinite(t_f[rows, cur]) | (t_f[rows, cur] >= np.where(np.isfinite(t0_t), t0_t, np.inf))
                k_ = np.where(fin, shp, 1.0)
                draw = np.where(fin, rng.gamma(k_, 1.0 / k_), 1.0)
                g_f[rows, cur] = np.where(new, draw, g_f[rows, cur])
                t_f[rows, cur] = np.where(new, 0.0, t_f[rows, cur])
            g = g_f[rows, cur]
            ci = np.where(in_init, (ci + 1) % n_f, ci)
            lam = np.empty(n)
            hit = np.empty(n, dtype=bool)
            res_sel = in_init & (misses > 0) if tl_res is not None else np.zeros(n, dtype=bool)
            for tl, sel in ((tl_upd, steady), (tl_init, in_init & ~res_sel)):
                if not sel.any():
                    continue
                lam[sel] = 10.0 ** ((tl.x_db - _track_loss(tl, e[sel], rng)) / 10.0) * g[sel] * gain_t[sel]
                hit[sel] = 0.5 * rng.noncentral_chisquare(2 * tl.n, np.maximum(2.0 * lam[sel], 1e-300)) > tl.threshold
            if res_sel.any():
                # lever D: rings of beams around the prediction covering RESEARCH_SIGMAS of its error, the beam that
                # detects measures; one target power for the cluster
                idx = np.flatnonzero(res_sel)
                sg = np.sqrt(np.maximum(cov[0][idx, 0, 0], cov[1][idx, 0, 0]) / r_m ** 2 + nav_rad ** 2) / hp
                nb = np.array([tm.research_cluster(float(x), rm.BEAM_SPACING_HPBW)[0] for x in sg])
                # the lattice shifted by (a + b) j / 3 from one re-search to the next (as the search frames, step 4.3
                # bis): a target on a cell vertex (the worst point, s / sqrt(3)) is on a beam axis at the next one
                off = _HEX37 * rm.BEAM_SPACING_HPBW
                shift = np.array([trk.attempt_shift(int(a_) - 1, rm.BEAM_SPACING_HPBW) for a_ in misses[idx]])
                d = e[idx][:, None, :] - (off[None, :, :] + shift[:, None, :])
                # a shifted re-search keeps one more beam on the estimate (the shifted lattice leaves it on a vertex)
                d = np.concatenate([d, e[idx][:, None, :]], axis=1)
                keep_c = np.any(shift != 0.0, axis=1)
                lb = 10.0 ** ((tl_res.x_db - _track_loss(tl_res, d, rng)) / 10.0) * (g * gain_t)[idx][:, None]
                hb = 0.5 * rng.noncentral_chisquare(2 * tl_res.n, np.maximum(2.0 * lb, 1e-300)) > tl_res.threshold
                hb[:, :-1] &= np.arange(len(off))[None, :] < nb[:, None]
                hb[:, -1] &= keep_c
                lam[idx] = np.where(hb, lb, 0.0).max(axis=1)
                hit[idx] = hb.any(axis=1)
                lam[idx] = np.where(hit[idx], lam[idx], lb[:, 0])
                tot["i_research"] = tot.get("i_research", 0) + len(idx)
                tot["i_research_beams"] = tot.get("i_research_beams", 0) + int(nb.sum())
            if rho_ku > 0.0:
                hit &= rng.random(n) >= rho_ku
            tot["hits"] += int((hit & steady).sum())
            tot["i_looks"] += int(in_init.sum())
            tot["i_hits"] += int((hit & in_init).sum())
            tot["i_point2"] += float((e ** 2).sum(axis=1)[in_init].sum() / 2.0)
            if in_init.any():
                tot["i_dt"].extend(dt[in_init][:50].tolist())
            tot["point2"] += float((e ** 2).sum(axis=1)[steady].sum() / 2.0)
            ref = np.where(in_init, tl_init.snr_ref, tl_upd.snr_ref)
            if tl_res is not None:
                ref = np.where(res_sel, tl_res.snr_ref, ref)
            scale = np.sqrt(ref / np.maximum(lam, 1e-12)) if snr_scaled else np.ones(n)
            # noise parts at the reference SNR: the update look's, or the initiation look's (each at its own kappa)
            n_th = np.where(in_init & ~res_sel, st_init.noise_theta, st.noise_theta)
            n_rr = np.where(in_init & ~res_sel, noise_r_init, noise_r)
            n_vv = np.where(in_init & ~res_sel, st_init.noise_vr, st.noise_vr)
            s_th = np.sqrt((n_th * scale) ** 2 + tm.ANGLE_FLOOR_RAD ** 2) * r_m
            s_rr = np.sqrt((n_rr * scale) ** 2 + tm.RANGE_FLOOR_M ** 2)
            s_vv = np.sqrt((n_vv * scale) ** 2 + tm.VR_FLOOR_MPS ** 2)
            for a in (0, 1):
                rd = np.where(hit, s_th ** 2, np.nan)[:, None]
                z = truth[a][:, :1] + s_th[:, None] * rng.standard_normal((n, 1))
                est[a], cov[a] = _kf_update(est[a], cov[a], z, rd, h1)
            rd = np.where(hit[:, None], np.stack([s_rr ** 2, s_vv ** 2], -1), np.nan)
            z = truth[2][:, :2] + np.stack([s_rr, s_vv], -1) * rng.standard_normal((n, 2))
            est[2], cov[2] = _kf_update(est[2], cov[2], z, rd, h2)
            # range and radial velocity: the track just after a successful update (user, 28 October 2026)
            upd_ok = steady & hit
            tot["err_r2"] += float(((truth[2][:, 0] - est[2][:, 0]) ** 2)[upd_ok].sum())
            tot["err_vr2"] += float(((truth[2][:, 1] - est[2][:, 1]) ** 2)[upd_ok].sum())
            tot["n_hit"] += int(upd_ok.sum())
            tot["n_err"] += int(steady.sum())
            tot["updates"] += int(steady.sum())
            misses = np.where(hit, 0, misses + 1)
            init_looks += in_init
            init_t += np.where(in_init, dt, 0.0)
            # initiation done: the criterion of trackmodel.initiation at the period, on the filter's own covariance
            nxt = np.einsum("ij,njk,lk->nil", f_t, cov[0], f_t) + q_t
            if steady_start and u >= 20:
                acc_ref += [float(np.sqrt(nxt[:, 0, 0]).sum()), float(np.sqrt(cov[2][:, 0, 0]).sum()),
                            float(np.sqrt(cov[2][:, 1, 1]).sum()), n]
            done = in_init & (np.sqrt(nxt[:, 0, 0]) <= zone_lim) & (np.sqrt(cov[2][:, 0, 0]) <= r_lim) \
                & (np.sqrt(cov[2][:, 1, 1]) <= v_lim)
            tot["init_done"].extend(init_looks[done].tolist())
            still = in_init & ~done
            tot["i_fail"][0] += int((still & (np.sqrt(nxt[:, 0, 0]) > zone_lim)).sum())
            tot["i_fail"][1] += int((still & (np.sqrt(cov[2][:, 0, 0]) > r_lim)).sum())
            tot["i_fail"][2] += int((still & (np.sqrt(cov[2][:, 1, 1]) > v_lim)).sum())
            tot["init_time"].extend(init_t[done].tolist())
            in_init &= ~done
            lost = misses >= np.where(in_init, n_fail_init, tm.N_MISS_LOST)   # lever D in the initiation
            upd_t += steady
            loss_t += lost & ~in_init
            if lost.any():
                tot["losses"] += int((lost & ~in_init).sum())
                tot["init_lost"] += int((lost & in_init).sum())
                li = lost & in_init                      # diagnostics of the initiations lost
                if li.any():
                    sg_l = np.sqrt(np.maximum(cov[0][li, 0, 0], cov[1][li, 0, 0]) / r_m ** 2 + nav_rad ** 2) / hp
                    err_l = np.hypot(e[li, 0], e[li, 1])
                    tot.setdefault("lost_diag", []).extend(zip(sg_l.tolist(), err_l.tolist(), init_looks[li].tolist(),
                                                              init_t[li].tolist()))
                start(np.flatnonzero(lost))
        if steady_start and acc_ref[3] > 0:
            tot["refs"][int(si)] = tuple(acc_ref[:3] / acc_ref[3])
        if tr is not None:
            for key_, val in list(tr.items()) + [("losses", loss_t), ("updates", upd_t)]:
                tot.setdefault("truth", {}).setdefault(key_, []).extend(np.asarray(val).tolist())
    ne, nh = max(tot["n_err"], 1), max(tot["n_hit"], 1)
    return {**tot, "pd_update_mc": tot["hits"] / ne, "pointing_rms_hpbw": math.sqrt(tot["point2"] / ne),
            "rms_angle_rad": math.sqrt(tot["err_angle2"] / ne), "rms_r_m": math.sqrt(tot["err_r2"] / nh),
            "rms_vr_mps": math.sqrt(tot["err_vr2"] / nh), "req_zone_rad": req[0], "req_r_m": req[1],
            "req_vr_mps": req[2]}


def bracket_objects(mission: dict, out: dict, sid: str) -> List[Optional[dict]]:
    """The track budget of one sector at its chosen Ku frequency, recomputed in this process with the track objects
    kept (tracking.KEEP_MC_OBJECTS): the "_mc" entry of each bracket."""
    g = out["global"]
    secs = {s["id"]: trk._with_mission_settings(s, mission.get("global", {})) for s in mission["sectors"]}
    r = next(x for x in out["sectors"] if x["id"] == sid)
    t = next(x for x in out["tracking"]["sectors"] if x["id"] == sid)
    sec = secs[sid]
    sph = mo._sphere(g["eps_min_deg"], g["eps_max_deg"])
    sec_s = dict(sec, _grid_spacing=r.get("beam_spacing_hpbw", rm.BEAM_SPACING_HPBW))
    pos, _, _, _ = mo.search_positions(sec_s, sph, sph.mask(sec), r["wf"]["freq_ghz"], mission.get("global", {}).get("sky"))
    pm = np.array([pos[k] for k in range(4)])
    trk.KEEP_MC_OBJECTS = True
    try:
        sb = trk._ku_freq_task(sec, r, pm, g["face_elec_avail_kW"] * 1e3, g["eps_min_deg"], g["eps_max_deg"],
                               mission.get("global", {}).get("sky"), float(t["ku_freq_ghz"]))
    finally:
        trk.KEEP_MC_OBJECTS = False
    return [b.get("_mc") for b in sb.get("at", [])]


def tracks(mission: dict, out: dict, handover: Dict[str, np.ndarray], seed: int = SEED,
           truth_cfg: Optional[dict] = None) -> List[dict]:
    """M3 for every bracket of active tracking of the mission (T9-T10); truth_cfg: true targets (robustness)."""
    rows = []
    for n, t in enumerate(out.get("tracking", {}).get("sectors", [])):
        if t.get("at"):
            rows += sector_tracks(mission, out, n, handover.get(t["id"]), seed, truth_cfg)
    return rows


def sector_tracks(mission: dict, out: dict, n: int, ho: Optional[np.ndarray], seed: int = SEED,
                  truth_cfg: Optional[dict] = None) -> List[dict]:
    """M3 for the brackets of the n-th tracking sector (ho: its drawn hand-over SNRs, None = analytic)."""
    rho_ku = float((out.get("scheduler") or {}).get("slots", {}).get("Ku", 0.0))
    rows = []
    t = out["tracking"]["sectors"][n]
    if t.get("at"):
        objs = bracket_objects(mission, out, t["id"])
        for bi, (at, mc) in enumerate(zip(t["at"], objs)):
            row = {"id": f"{t['id']}/{bi}", "r_min_km": at.get("r_min_km"), "r_max_km": at.get("r_max_km"),
                   "period_s": at.get("rr_s")}
            if not mc or not mc["states"]:
                rows.append({**row, "status": "NO_LOOK"})
                continue
            t_start = time.perf_counter()
            mc = dict(mc, proc_s=sch.compute.processing_s(out, t["id"]))
            rng = np.random.default_rng([int(seed), 2000 + n, bi])
            res = simulate_bracket(mc["states"], mc, TRACKS_PER_BRACKET, UPDATES_PER_TRACK, rng, rho_ku, ho, True,
                                   truth_cfg=truth_cfg)
            ini = simulate_bracket(mc["states"], mc, TRACKS_PER_BRACKET, UPDATES_PER_TRACK, rng, rho_ku, ho, False,
                                   refs=res["refs"], truth_cfg=truth_cfg)
            lo, hi = wilson(res["losses"], res["updates"])
            p_mc = res["losses"] / res["updates"] if res["updates"] else None
            p_an = float(at.get("p_loss_per_update", 0.0))
            looks = np.array(ini["init_done"])
            dur = np.array(ini["init_time"])
            i_mean = float(looks.mean()) if len(looks) else None
            i_q95 = float(np.quantile(looks, 0.95)) if len(looks) else None
            i_an = at.get("init_looks")
            n_ini = ini["inits"]
            ok_ini = len(looks)
            acc = [("angle", res["rms_angle_rad"], res["req_zone_rad"])]
            if res["req_r_m"] is not None:
                acc.append(("range", res["rms_r_m"], res["req_r_m"]))
            acc.append(("radial velocity", res["rms_vr_mps"], res["req_vr_mps"]))
            state, why = "OK", []
            p_max = float(P.get("tracking.p_loss_max"))
            if p_mc is not None and lo > p_max:
                state = "NOK"
                why.append(f"loss per update {p_mc:.2e} (lower bound {lo:.2e}) above p_loss_max {p_max:g}")
            bad = [f"{nm} RMS {v:.3g} above {req_:.3g}" for nm, v, req_ in acc if req_ and v > req_ * (1.0 + CI_TOL)]
            if bad:
                state = "NOK"
                why.extend(bad)
            if state == "OK" and p_mc is not None and p_an < lo:
                state = "ALERTE"
                why.append(f"analytic loss {p_an:.2e} below the interval (model optimistic)")
            if state == "OK" and i_an is not None and i_mean is not None and i_an < i_mean * (1.0 - CI_TOL):
                state = "ALERTE"
                why.append(f"analytic initiation {i_an:.3g} look(s) below the drawn mean {i_mean:.1f}")
            # initiation losses per attempt against the analytic chain (return on 3.3, H5)
            ilo, ihi = wilson(ini["init_lost"], ok_ini + ini["init_lost"])
            il_an = at.get("init_loss_prob")
            if state == "OK" and il_an is not None and il_an < ilo:
                state = "ALERTE"
                why.append(f"analytic initiation loss {il_an:.2e} below the interval [{ilo:.2e}, {ihi:.2e}]")
            rows.append({**row, "status": "OK", "tracks": TRACKS_PER_BRACKET, "updates": res["updates"],
                         "pd_update_mc": res["pd_update_mc"], "pointing_rms_hpbw": res["pointing_rms_hpbw"],
                         "pd_update_design": float(at.get("pd_update") or trk.tm.PD_UPDATE),
                         "pointing_sigma_analytic_hpbw": at.get("pointing_sigma_hpbw"),
                         "p_loss_mc": p_mc, "ci": [lo, hi], "p_loss_analytic": p_an, "p_loss_max": p_max,
                         "rms": {nm: v for nm, v, _ in acc}, "required": {nm: req_ for nm, _, req_ in acc},
                         "inits": n_ini, "init_done": ok_ini, "init_lost": ini["init_lost"],
                         "init_success": ok_ini / max(ok_ini + ini["init_lost"], 1),
                         "init_looks_mc_mean": i_mean, "init_looks_mc_q95": i_q95, "init_looks_analytic": i_an,
                         "init_time_mean_s": float(dur.mean()) if len(dur) else None,
                         "init_time_q95_s": float(np.quantile(dur, 0.95)) if len(dur) else None,
                         "init_dt_min_s": ini["dt_min_s"], "init_loss_ci": [ilo, ihi],
                         "init_loss_analytic": at.get("init_loss_prob"), "init_time_analytic_s": at.get("init_time_s"),
                         "init_freqs": at.get("init_freqs"), "init_mode": at.get("init_mode"),
                         "handover": "drawn (M2)" if ho is not None and len(ho) else "analytic",
                         "state": state, "why": "; ".join(why), "truth": res.get("truth"),
                         "runtime_s": time.perf_counter() - t_start})
    return rows


# ----------------------------------------------------------------------------- mission
def track_chain(mission: dict, out: dict, rows: List[dict], trk_rows: List[dict]) -> List[dict]:
    """Requirement of the user (27 October 2026): the chain first detection > confirmation > established track fails at
    most tracking.p_track_fail_max. Per sector with tracks: drawn confirmation success (M2) x drawn initiation
    success of the bracket where the tracks start (M3), the interval from the two Wilson intervals; against the
    analytic chain."""
    secs = {s["id"]: s for s in mission["sectors"]}
    tsec = {t["id"]: t for t in out.get("tracking", {}).get("sectors", [])}
    by_id = {r["id"]: r for r in trk_rows}
    lim = float(trk.tm.P_TRACK_FAIL_MAX)
    res = []
    for r in rows:
        c = r.get("confirmation")
        sid = r["id"]
        t = tsec.get(sid)
        if not c or not t or not t.get("at"):
            continue
        sec = secs[sid]
        bi = trk._start_bracket(sec.get("tracking", []), float(sec["range_km"]))
        bi = 0 if bi is None else bi
        b = by_id.get(f"{sid}/{bi}")
        if not b or b.get("status") != "OK":
            continue
        n_i = b["init_done"] + b["init_lost"]
        p_i = b["init_done"] / n_i if n_i else 0.0
        ci_i = wilson(b["init_done"], n_i)
        p_c, ci_c = c["p_mc"], c["ci"]
        fail = 1.0 - p_c * p_i
        lo, hi = 1.0 - ci_c[1] * ci_i[1], 1.0 - ci_c[0] * ci_i[0]
        at = t["at"][bi]
        an = (1.0 - float(c["p_analytic"]) * (1.0 - float(at["init_loss_prob"]))
              if at.get("init_loss_prob") is not None else None)
        if lo > lim:
            state, why = "NOK", f"chain failure {fail:.2e} (lower bound {lo:.2e}) above {lim:g}"
        elif an is not None and an < lo:
            state, why = "ALERTE", f"analytic chain failure {an:.2e} below the interval (model optimistic)"
        else:
            state, why = "OK", ""
        res.append({"id": sid, "bracket": bi, "confirmation_mc": p_c, "initiation_mc": p_i, "failure_mc": fail,
                    "ci": [lo, hi], "failure_analytic": an, "required": lim, "state": state, "why": why})
    return res


def plan_eclipse_rho(out: dict) -> Dict[str, float]:
    """Probability of a whole echo loss by the eclipse each sector's kept plan holds: the global optimiser's last
    coupling of the sector (step 5.2; return on 3.2 / 5.2 from step 5.4)."""
    ecl: Dict[str, float] = {}
    for h in (out.get("global_opt") or {}).get("history", []):
        for sid, k in (h.get("couplings") or {}).items():
            ecl[sid] = float(k.get("_eclipse_rho", 0.0))
    return ecl


def sector_task(mission: dict, out: dict, n: int, seed: int = SEED,
                truth_cfg: Optional[dict] = None) -> Tuple[dict, Optional[np.ndarray]]:
    """(row, drawn hand-over SNRs or None) of the n-th search sector: M1, M2-6 and its confirmation M2-4."""
    sec = mission["sectors"][n]
    plan = next((r.get("_plan") for r in out.get("sectors", []) if r["id"] == sec["id"]), None)
    if not plan:
        return {"id": sec["id"], "status": "NO_PLAN"}, None
    t = next((x for x in out.get("tracking", {}).get("sectors", []) if x["id"] == sec["id"]), None)
    spec = confirm_spec(sec, plan, t, mission.get("global", {})) if t is not None else None
    f_ku = float(t["ku_freq_ghz"]) if t is not None and t.get("ku_freq_ghz") else None
    rho = float((out.get("scheduler") or {}).get("slots", {}).get("S", 0.0))
    row = sector_mc(sec, plan, n, seed, eclipse=(plan_eclipse_rho(out).get(sec["id"], 0.0), rho), confirm=spec,
                    truth_cfg=truth_cfg, f_ku=f_ku)
    return row, getattr(spec, "handover_snr", None) if spec is not None else None


# parallel draws (decision 4, user 28 October 2026): sectors, face timelines and tracking sectors in worker processes,
# each element with its own seed as in the sequential run, so the results are identical; the mission, the plans and the
# module sizes (which tools/robustness.py changes) are handed once to every worker
_W: Dict[str, object] = {}
_SIZES = ("N_MIN", "N_MAX", "CI_TOL", "N_REP", "TRACKS_PER_BRACKET", "UPDATES_PER_TRACK")


def _worker_init(mission: dict, out: dict, sizes: dict) -> None:
    mo._mark_pool_worker()
    globals().update(sizes)
    _W.clear()
    _W.update(mission=mission, out=out)


def _w_sector(n: int, seed: int, truth_cfg: Optional[dict]):
    return sector_task(_W["mission"], _W["out"], n, seed, truth_cfg)


def _w_face(k: int, rep: int, seed: int) -> dict:
    if "scans" not in _W:
        _W["scans"] = mission_scans(_W["mission"], _W["out"])
    return face_timeline(_W["mission"], _W["out"], k, rep, seed, _W["scans"])


def _w_tracks(n: int, ho: Optional[np.ndarray], seed: int, truth_cfg: Optional[dict]) -> List[dict]:
    return sector_tracks(_W["mission"], _W["out"], n, ho, seed, truth_cfg)


def _gather(ex, jobs: list, progress_cb=None, label=None) -> list:
    """The results of jobs [(fn, args)] in their order; progress as they complete."""
    from concurrent.futures import as_completed
    futs = {ex.submit(fn, *args): i for i, (fn, args) in enumerate(jobs)}
    res: list = [None] * len(jobs)
    for d, f in enumerate(as_completed(futs), 1):
        res[futs[f]] = f.result()
        if progress_cb:
            progress_cb(d, len(jobs), label(futs[f]) if label else str(futs[f]))
    return res


def run(mission: dict, out: Optional[dict] = None, seed: int = SEED, progress_cb=None,
        truth_cfg: Optional[dict] = None, workers: Optional[int] = None) -> dict:
    """Monte-Carlo of a mission: per search sector the cumulative Pd of the test targets with the eclipse drawn (M1,
    M2-6) and their confirmation (M2-4); the drawn timelines of the faces (false alarms, latencies, scheduler, M2).
    out: a run kept with its plans (optimizer.run(keep_plans=True)); None = computed here. workers: None = the
    optimiser's max_workers, 1 = sequential (same results)."""
    t_start = time.perf_counter()
    if out is None or not any(r.get("_plan") for r in out.get("sectors", [])):
        out = mo.run(mission, keep_plans=True)
    t_plan = time.perf_counter() - t_start
    ids = [s["id"] for s in mission["sectors"]]
    tsecs = out.get("tracking", {}).get("sectors", [])
    rho = float((out.get("scheduler") or {}).get("slots", {}).get("S", 0.0))
    # automatic: sequential inside a worker or under the parallel test runner (their workers already fill the machine)
    auto = 1 if (mo._IN_POOL_WORKER or os.environ.get("PYTEST_XDIST_WORKER")) else mo.MAX_WORKERS
    n_w = auto if workers is None else int(workers)
    ex = None
    if n_w > 1:
        from concurrent.futures import ProcessPoolExecutor
        ex = ProcessPoolExecutor(max_workers=n_w, initializer=_worker_init,
                                 initargs=(mission, out, {k: globals()[k] for k in _SIZES}))
    try:
        if ex is None:
            got = []
            for n in range(len(ids)):
                got.append(sector_task(mission, out, n, seed, truth_cfg))
                if progress_cb:
                    progress_cb(n + 1, len(ids), ids[n])
        else:
            got = _gather(ex, [(_w_sector, (n, seed, truth_cfg)) for n in range(len(ids))], progress_cb,
                          lambda i: ids[i])
        rows = [r for r, _ in got]
        handover = {ids[n]: h for n, (_, h) in enumerate(got) if h is not None}
        t_sec = time.perf_counter() - t_start - t_plan
        t0_ = time.perf_counter()
        if ex is None:
            tl = timelines(mission, out, seed, progress_cb)
        else:
            keys = [(k, rep) for k in range(4) for rep in range(N_REP)]
            res = _gather(ex, [(_w_face, (k, rep, seed)) for k, rep in keys], progress_cb,
                          lambda i: f"F{keys[i][0] + 1}/{keys[i][1]}")
            tl = timelines(mission, out, seed, None, runs=dict(zip(keys, res)))
        t_line = time.perf_counter() - t0_
        t_tl = time.perf_counter()
        if ex is None:
            trk_rows = tracks(mission, out, handover, seed, truth_cfg)
        else:
            idx = [n for n, t in enumerate(tsecs) if t.get("at")]
            trk_rows = [r for rs in _gather(ex, [(_w_tracks, (n, handover.get(tsecs[n]["id"]), seed, truth_cfg))
                                                 for n in idx], progress_cb, lambda i: tsecs[idx[i]]["id"])
                        for r in rs]
        t_trk = time.perf_counter() - t_tl
    finally:
        if ex is not None:
            ex.shutdown(wait=True)
    chain = track_chain(mission, out, rows, trk_rows)
    states = [r.get("state") for r in rows if r.get("status") == "OK"]
    states += [(r.get("confirmation") or {}).get("state") for r in rows]
    states += [r[x] for r in tl["sectors"] for x in ("fa_state", "latency_state")]
    states += [fb["state"] for fk in tl["faces"].values() for fb in fk.values()]
    states += [c["state"] for fk in tl["faces"].values() for fb in fk.values() for c in fb["classes"]]
    states += [r.get("state") for r in trk_rows]
    states += [r.get("state") for r in chain]
    state = "NOK" if "NOK" in states else ("ALERTE" if "ALERTE" in states else "OK")
    return {"pass": "M3", "seed": int(seed), "ci_level": CI_LEVEL, "state": state, "sectors": rows,
            "eclipse_share_s": rho, "timelines": tl, "tracks": trk_rows, "chain": chain,
            "stage_s": {"plan": t_plan, "search_confirmation": t_sec, "timelines": t_line, "tracks": t_trk},
            "runtime_s": time.perf_counter() - t_start}


def report_lines(mc: dict) -> List[str]:
    """Text report of a Monte-Carlo run (H3)."""
    lines = [f"MONTE-CARLO (step 5.4, pass {mc['pass']}, seed {mc['seed']}, intervals {mc['ci_level']:.0%}): "
             f"{mc['state']} ({mc['runtime_s']:.1f} s)",
             f"  {'sector':<14}{'targets':>8}{'Pd req':>8}{'analytic':>10}{'Monte-Carlo':>13}{'interval':>17}  state"]
    for r in mc["sectors"]:
        if r.get("status") != "OK":
            lines.append(f"  {r['id']:<14}  no plan")
            continue
        a = r["pd_analytic_in_range"]
        m = r["pd_mc_in_range"]
        ci = r["ci_in_range"]
        lines.append(f"  {r['id']:<14}{r['targets']:>8}{r['pd_required']:>8.3f}"
                     f"{(f'{a:.3f}' if a is not None else '-'):>10}{(f'{m:.3f}' if m is not None else '-'):>13}"
                     f"{(f'[{ci[0]:.3f}, {ci[1]:.3f}]' if ci[0] is not None else '-'):>17}  {r['state']}"
                     + (f" -- {r['why']}" if r["why"] else ""))
        if r["short_share"] > 0:
            lines.append(f"  {'':<14}positions short of range {r['short_share']:.1%} (all positions: Pd "
                         f"{r['pd_mc']:.3f}, analytic {r['pd_analytic']:.3f})")
        for note in r["notes"]:
            lines.append(f"  {'':<14}note: {note}")
        c = r.get("confirmation")
        if c:
            lines.append(f"  {'':<14}confirmation {c['mode']}: {c['detections']} detections, P {c['p_mc']:.3f} "
                         f"[{c['ci'][0]:.3f}, {c['ci'][1]:.3f}], analytic {c['p_analytic']:.3f}, min {c['p_min']:g}  "
                         f"{c['state']}" + (f" -- {c['why']}" if c["why"] else ""))
    tl = mc.get("timelines")
    if tl:
        lines.append(f"  Timelines ({tl['n_rep']} per face): false alarms per revisit and confirmation latency (95 %)")
        for r in tl["sectors"]:
            f_ = r["fa_per_rr_mc"]
            lines.append(f"  {r['id']:<14}FA/RR {f_:.3g} [{r['fa_ci'][0]:.3g}, {r['fa_ci'][1]:.3g}] analytic "
                         f"{r['fa_per_rr_analytic']:.3g} max {r['fa_per_rr_required']:g} (information"
                         + (f": {r['fa_why']})" if r["fa_why"] else ")")
                         if f_ is not None else f"  {r['id']:<14}FA/RR -")
            if r["latency_q95_mc_s"] is not None:
                a = r["latency_q95_analytic_s"]
                lines.append(f"  {'':<14}latency {r['latency_q95_mc_s']:.3g} s [{r['latency_ci'][0]:.3g}, "
                             f"{r['latency_ci'][1]:.3g}] ({r['detections']} detections) analytic "
                             f"{(f'{a:.3g}' if a is not None else '-')} s required {r['latency_required_s']:.3g} s "
                             f"{r['latency_state']}" + (f" -- {r['latency_why']}" if r["latency_why"] else ""))
        for k, fk in tl["faces"].items():
            for b, fb in fk.items():
                bad = [c for c in fb["classes"] if c["state"] not in ("OK", "INFO")]
                if fb["updates"] or bad or fb["recharge_pauses"] or fb["state"] != "OK":
                    lines.append(f"  F{int(k) + 1} {b}: late updates {100 * fb['late_update_frac_mc']:.2f} % of "
                                 f"{fb['updates']}, recharge pauses {fb['recharge_pauses']}  {fb['state']}"
                                 + (f" -- {fb['why']}" if fb["why"] else ""))
                for c in bad:
                    lines.append(f"  F{int(k) + 1} {b}: {c['class']}: {c['why']}")
    if mc.get("tracks"):
        lines.append("  Tracks (Kalman drawn): loss per update, initiation looks, RMS accuracies")
        for r in mc["tracks"]:
            if r.get("status") != "OK":
                lines.append(f"  {r['id']:<14}no update look")
                continue
            p_ = r["p_loss_mc"] if r["p_loss_mc"] is not None else math.nan
            im = r["init_looks_mc_mean"]
            lines.append(f"  {r['id']:<14}steady: loss {p_:.2e} [{r['ci'][0]:.2e}, {r['ci'][1]:.2e}] over "
                         f"{r['updates']} updates, analytic {r['p_loss_analytic']:.2e}, max {r['p_loss_max']:g}  {r['state']}"
                         + (f" -- {r['why']}" if r["why"] else ""))
            it = r["init_time_mean_s"]
            iq = r["init_looks_mc_q95"]
            lines.append(f"  {'':<14}initiation: {100 * r['init_success']:.1f} % established ({r['init_lost']} lost), "
                         f"{(f'{im:.1f}' if im is not None else '-')} looks (95 %: "
                         f"{(f'{iq:.0f}' if iq is not None else '-')}), "
                         f"{(f'{it:.3g} s' if it is not None else '-')}; analytic {r['init_looks_analytic']:.3g} "
                         f"look(s), {r['init_time_analytic_s']:.3g} s, loss {r['init_loss_analytic']:.2e}; "
                         f"{r['init_freqs']} Ku frequencies ({r['init_mode']}), hand-over {r['handover']}"
                         if r["init_looks_analytic"] is not None else
                         f"{(f'{it:.3g} s' if it is not None else '-')}; hand-over {r['handover']}")
            lines.append(f"  {'':<14}RMS " + ", ".join(f"{k} {v:.3g} (required {r['required'][k]:.3g})"
                                                     for k, v in r["rms"].items()))
    if mc.get("chain"):
        lines.append("  Chain first detection > confirmation > established track (failure, requirement "
                     f"{mc['chain'][0]['required']:g})")
        for r in mc["chain"]:
            a = r["failure_analytic"]
            lines.append(f"  {r['id']:<14}failure {r['failure_mc']:.2e} [{r['ci'][0]:.2e}, {r['ci'][1]:.2e}] "
                         f"(confirmation {r['confirmation_mc']:.4f} x initiation {r['initiation_mc']:.4f}, bracket "
                         f"{r['bracket']}), analytic {(f'{a:.2e}' if a is not None else '-')}  {r['state']}"
                         + (f" -- {r['why']}" if r["why"] else ""))
    return lines


def _clean(x):
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_clean(v) for v in x]
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return x


def main(argv: List[str]) -> int:
    if not argv:
        print("usage: python -m core.montecarlo <mission.json|.csv> [result.json]")
        return 2
    mission = mo.load_mission(argv[0])
    mc = run(mission, progress_cb=lambda d, n, s: print(f"  {d}/{n} {s}", flush=True))
    print("\n".join(report_lines(mc)))
    if len(argv) > 1:
        with open(argv[1], "w", encoding="utf-8") as fh:
            json.dump(_clean(mc), fh, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
