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
To come: M3 tracks; M4 verdict and load_ceiling; M5 tests at N3, snapshots, docs.
"""
from __future__ import annotations

import dataclasses
import heapq
import json
import math
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


def analytic_pcum(curve: np.ndarray, q: float, x_db: float, floor: float = rm.CUM_PD_FLOOR) -> float:
    """Cumulative Pd of the analytic model at mean SNR x_db at the requirement range (detection._required_snr_db)."""
    if q <= 0.0:
        return float(np.interp(x_db, detection.SNR_GRID_DB, curve))
    sh = detection._look_shifts(round(q, 9))
    pk = np.interp(x_db - sh, detection.SNR_GRID_DB, curve, left=float(curve[0]), right=float(curve[-1]))
    pk = np.where(pk < floor, 0.0, pk)
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
def draw_targets(bm: BinModel, rng: np.random.Generator, n: int, rr_s: float, v_mps: np.ndarray,
                 t0_s: float, correlated: bool = True, eclipse_rho: float = 0.0, with_snr: bool = False):
    """(detected (bool, n), horizon cut) for n test targets of one bin: any look at R_bin (1 + (k + phi) q) detects;
    cut = looks that could still detect were left out by the K_MAX_LOOKS safety bound. Every look whose mean Pd stays
    above pd_look_floor is drawn, without a time bound, as the analytic criterion counts them (user, 27 October 2026:
    horizon_max_s cut the worst-range window of slow far targets; it bounds the scheduler timeline of M2 only).
    correlated False: every position loss drawn anew at each look (the analytic assumption, for the tests).
    eclipse_rho (M2-6): each echo is lost whole with this probability (the share of the common transmit slots).
    with_snr: also the signal SNR (linear) of the first look that detects (NaN when none), for the confirmation."""
    q = v_mps * rr_s / (bm.r_bin_km * 1e3)                           # (n,)
    curve = bm.curve()
    # looks that can still detect: mean Pd above the floor (at the smallest q, the most looks)
    q_min = float(q.min())
    k_cap = K_MAX_LOOKS
    k_need = 1
    if q_min > 0:
        k_try = np.arange(1, K_MAX_LOOKS + 2)
        pk = np.interp(bm.x_avail_db - 40.0 * np.log10(1.0 + (k_try - 1) * q_min), detection.SNR_GRID_DB, curve,
                       left=float(curve[0]), right=float(curve[-1]))
        alive = np.flatnonzero(pk >= PD_LOOK_FLOOR)
        k_need = int(alive.max()) + 1 if len(alive) else 1
    K = max(1, min(k_need, k_cap))
    phi = rng.random(n)
    kk = np.arange(K)[None, :] + phi[:, None]                           # (n, K)
    snr_db = bm.x_avail_db - 40.0 * np.log10(1.0 + kk * q[:, None])
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
    if math.isfinite(bm.shape) and bm.shape <= 1e4:                    # as detection.pd_curve (_STEADY_SHAPE)
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

    def _ku17(self, rng, ex, ey, m_db: float, cluster_only: bool) -> np.ndarray:
        st, n = self.st, len(ex)
        snr0 = st.snr_ku * rm.db_to_lin(m_db)
        t1 = detection.threshold(1, rm.DETECT_PFA)
        shape = detection.shape_of(st.fl_ku, 1)
        looks = len(st.cl.beams) + 1
        g = _look_power(rng, shape, n, self.trip_s < self.t0_ku_s, looks)
        ok = np.zeros(n, dtype=bool)
        if not cluster_only:
            ok = _detect1(rng, snr0 * trk._tx_gain(ex, ey) * g[:, 0], t1)
        for j, (cx, cy, ku, kv) in enumerate(st.cl.beams):
            ok |= _detect1(rng, snr0 * st.cl.eirp * trk._tx_gain(ex - cx, ey - cy, ku, kv) * g[:, j + 1], t1)
        return ok

    def draw(self, snr_det: np.ndarray, rng: np.random.Generator) -> dict:
        """Success of the confirmation of each detection (signal SNR snr_det of the S look that detected): angle error
        per axis bias + K q / sqrt(SNR) x N(0, 1) in Ku beamwidths, the mode's looks decided exactly (single sample,
        Pfa of the Ku looks, fluctuation of the class; one power draw for looks within T0)."""
        st, n = self.st, len(snr_det)
        sig = trk.K_ML_S * st.q / np.sqrt(np.maximum(snr_det, 1e-12))
        ex = st.bias_ku + sig * rng.standard_normal(n)
        ey = st.bias_ku + sig * rng.standard_normal(n)
        if self.mode in ("KU1+7", "KU7"):
            ok = self._ku17(rng, ex, ey, float(self.margin_db), self.mode == "KU7")
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
    return ConfirmSpec(sid=sec["id"], mode=mode, margin_db=c["margin_db"], success_analytic=float(c["success"]),
                       p_min=p_min, st=st, t0_s_s=tgt.t0_s(f_s), t0_ku_s=tgt.t0_s(f_ku),
                       trip_s=2.0 * float(sec["range_km"]) * 1e3 / rm.C_MPS)


# ----------------------------------------------------------------------------- one sector
def sector_mc(sec: dict, plan: dict, index: int, seed: int = SEED, correlated: bool = True,
              eclipse: Tuple[float, float] = (0.0, 0.0), confirm: Optional["ConfirmSpec"] = None) -> dict:
    """Cumulative Pd of the test targets of one search sector, by draws, against the analytic model (M1); eclipse =
    (loss in dB the plan holds, share of the common S slots): the loss is given back and each echo is lost whole with
    that share (M2-6); confirm: the confirmation of every detected test target in range is drawn (M2-4)."""
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
        models[int(i)] = bin_model(plan["wf"][i], f, float(theta[i]), tgt, sec, float(plan["pfa"][i]),
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
        an[i] = analytic_pcum(bm.curve(), vmax * rr / (bm.r_bin_km * 1e3), bm.x_avail_db)
    if eclipse[0] > 0.0 or eclipse[1] > 0.0:
        models = {i: dataclasses.replace(bm, x_avail_db=bm.x_avail_db + eclipse[0]) for i, bm in models.items()}
        notes.add(f"eclipse drawn whole per echo ({100 * eclipse[1]:.2f} % of the time), the plan's mean "
                  f"{eclipse[0]:.3f} dB given back")
    snr_det: List[np.ndarray] = []
    in_range = np.array([0.0 if (i not in models or models[i].short) else weights[i] for i in range(nb)])
    pd_an = float(sum(p_w[i] * an[i] for i in models))
    pd_an_in = (float(sum(in_range[i] * an[i] for i in models) / in_range.sum()) if in_range.sum() > 0 else None)
    rng = np.random.default_rng([int(seed), int(index)])
    bins = np.array(sorted(models))
    probs = weights[bins] / weights[bins].sum()
    hits = tries = hits_in = tries_in = 0
    while tries < N_MAX:
        nb_draw = min(BATCH, N_MAX - tries)
        pick = bins[rng.choice(len(bins), nb_draw, p=probs)]
        for i in np.unique(pick):
            m = int((pick == i).sum())
            v = (np.full(m, vmax) if V_DRAW != "uniform" else rng.uniform(vmin, vmax, m))
            det, cut, snr = draw_targets(models[int(i)], rng, m, rr, v, t0, correlated, eclipse[1], True)
            if cut:
                notes.add(f"looks beyond {K_MAX_LOOKS} left out (pessimistic)")
            hits += int(det.sum())
            tries += m
            if not models[int(i)].short:
                hits_in += int(det.sum())
                tries_in += m
                snr_det.append(snr[det])
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
        row_conf = confirm.draw(np.concatenate(snr_det), np.random.default_rng([int(seed), int(index), 1]))
    return {"id": sec["id"], "status": "OK", "pd_required": pd_req, "targets": tries, "targets_in_range": tries_in,
            "pd_mc": hits / tries, "ci": list(ci), "pd_mc_in_range": pd_mc_in, "ci_in_range": list(ci_in),
            "pd_analytic": pd_an, "pd_analytic_in_range": pd_an_in,
            "short_share": float(1.0 - in_range.sum() / weights.sum()), "state": state, "why": why,
            "v_draw": V_DRAW, "correlated": correlated, "notes": sorted(notes), "confirmation": row_conf,
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


def timelines(mission: dict, out: dict, seed: int = SEED, progress_cb=None) -> dict:
    """N_REP drawn timelines of every face: per sector the false alarms per revisit and the confirmation latency of the
    true detections (95 % quantile and its interval), per face and band the waits, late updates and pauses, against
    the analytic scheduler of step 4.3 (M2-5, M2-7, M2-8)."""
    from . import scanorder as so
    secs = {s["id"]: s for s in mission["sectors"]}
    order = [s["id"] for s in mission["sectors"]]
    g = out.get("global", {})
    sph = mo._sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    sky = mission.get("global", {}).get("sky")
    scans = {r["id"]: so.sector_scan(secs[r["id"]], r, sph, sky) for r in out.get("sectors", [])
             if r.get("status") == "OK" and r["id"] in secs}
    tsec = {s["id"]: s for s in out.get("tracking", {}).get("sectors", [])}
    ssec = {s["id"]: s for s in out.get("scheduler", {}).get("sectors", [])}
    fa_by: Dict[str, Dict[int, List[float]]] = {}          # per sector and face: [false alarms, revisits counted]
    lat: Dict[str, List[float]] = {}
    faces = {}
    for k in range(4):
        per = {"S": {}, "Ku": {}}
        upd = {"S": [0, 0, 0], "Ku": [0, 0, 0]}
        for rep in range(N_REP):
            r = face_timeline(mission, out, k, rep, seed, scans)
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


# ----------------------------------------------------------------------------- mission
def plan_eclipse_db(out: dict) -> Dict[str, float]:
    """Eclipse loss (dB) each sector's kept plan holds: the global optimiser's last coupling of the sector (step 5.2)."""
    ecl: Dict[str, float] = {}
    for h in (out.get("global_opt") or {}).get("history", []):
        for sid, k in (h.get("couplings") or {}).items():
            ecl[sid] = float(k.get("_eclipse_db", 0.0))
    return ecl


def run(mission: dict, out: Optional[dict] = None, seed: int = SEED, progress_cb=None) -> dict:
    """Monte-Carlo of a mission: per search sector the cumulative Pd of the test targets with the eclipse drawn (M1,
    M2-6) and their confirmation (M2-4); the drawn timelines of the faces (false alarms, latencies, scheduler, M2).
    out: a run kept with its plans (optimizer.run(keep_plans=True)); None = computed here."""
    t_start = time.perf_counter()
    if out is None or not any(r.get("_plan") for r in out.get("sectors", [])):
        out = mo.run(mission, keep_plans=True)
    plans = {r["id"]: r.get("_plan") for r in out.get("sectors", [])}
    tsec = {t["id"]: t for t in out.get("tracking", {}).get("sectors", [])}
    ecl = plan_eclipse_db(out)
    rho = float((out.get("scheduler") or {}).get("slots", {}).get("S", 0.0))
    g = mission.get("global", {})
    rows = []
    for n, sec in enumerate(mission["sectors"]):
        plan = plans.get(sec["id"])
        if not plan:
            rows.append({"id": sec["id"], "status": "NO_PLAN"})
            continue
        spec = confirm_spec(sec, plan, tsec[sec["id"]], g) if sec["id"] in tsec else None
        rows.append(sector_mc(sec, plan, n, seed, eclipse=(ecl.get(sec["id"], 0.0), rho), confirm=spec))
        if progress_cb:
            progress_cb(n + 1, len(mission["sectors"]), sec["id"])
    tl = timelines(mission, out, seed, progress_cb)
    states = [r.get("state") for r in rows if r.get("status") == "OK"]
    states += [(r.get("confirmation") or {}).get("state") for r in rows]
    states += [r[x] for r in tl["sectors"] for x in ("fa_state", "latency_state")]
    states += [fb["state"] for fk in tl["faces"].values() for fb in fk.values()]
    states += [c["state"] for fk in tl["faces"].values() for fb in fk.values() for c in fb["classes"]]
    state = "NOK" if "NOK" in states else ("ALERTE" if "ALERTE" in states else "OK")
    return {"pass": "M2", "seed": int(seed), "ci_level": CI_LEVEL, "state": state, "sectors": rows,
            "eclipse_share_s": rho, "timelines": tl, "runtime_s": time.perf_counter() - t_start}


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
