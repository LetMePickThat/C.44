#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TRACKING -- confirmation and active tracking (AT) budget, added per face to the S-band search budget of
core.optimizer.run() (called at its end; the result goes under the report key "tracking").

Decisions of 2026-10-19 (tracking plan, steps 1-2), rebuilt at roadmap step 3.2 (23 October 2026, blocks A-E):
  - Angle measured on the detection echo itself (fully digital RX, ML estimator on the full aperture):
    sigma per axis = K x HPBW_TX / sqrt(SNR of the echo), K = Cramer-Rao bound of the aperture x margin
    (radar_model.angle_k_tx, step 1.1, ~0.49); residual S/Ku alignment bias confirmation.bias_mrad per axis. The
    echoes that cross the threshold have a random SNR, drawn here by quantiles of the target's Gamma law (3.2 E4).
  - Every S detection is confirmed: true targets AND false alarms (radar_model.DETECT_PFA per range-Doppler cell).
    Modes:  KU1+7 = one Ku look on the S estimate, then a 7-beam Ku cluster if it fails (two round trips);
            KU7   = 7-beam Ku cluster at once (one round trip);
            S2    = immediate S touch pointed on the detection (an S waveform of role CONFIRM: the sector's search
                    waveform, pointed; rejects the false alarms in S), then KU1+7 on the improved angle estimate.
    The cluster is sent as 7 successive looks, or at once (step 3.2 C): phase-only superposition of the 7 beams
    (power / 7, loss interpolated in the superposition table), or a partition of the face into 2 or 4
    sub-apertures (wider beams, gain N_sub / N, power / B); the cheapest is kept, the round trips are published.
    Each mode gets the smallest power margin reaching CONFIRM_P_MIN, solved continuously (bisection, 3.2 B6).
  - Ku frequency per sector (step 3.2 A): centres derived from the Ku band (usable_lo + k x ku_freq_step), the signal
    inside the band and outside the mission's Ku exclusions; the valid centres on the coarse step plus the centre-most
    one of every free fragment are evaluated, and the (frequency, mode) pair that minimises the mission's most loaded
    face is kept (ties: nearest the core-band centre). The S 5th harmonic on the Ku channel is flagged (a time rule
    for the scheduler, step 4.3), not a frequency rule.
  - Ku looks (step 3.2 B, E): pulses, PRIs, bandwidths of [tool.confirmation], both integrations, an agile variant of
    the noncoherent trains; the self-protection jammer of the sector and the sidelobe clutter of its surfaces per
    look (the P4 model of the search); a look that exceeds its need is shortened at saturated PAs, as the search.
    Range at scan theta = range at broadside x the scan factor h(f, theta) common to every waveform (exact, checked).
  - Active tracking per operator range bracket (sector key "tracking"): number of targets, update period, angular
    uncertainty-zone opening (full width at +/-2 sigma: sigma = opening / 4), range and radial-velocity accuracy
    (1 sigma). One Ku look per track update at the bracket's outer range, SNR = max(detection, accuracy):
    radar_model.tracking_snr_req; the dominant term is reported.
  - The S and Ku arrays of a face share its electrical power (energies add up) but have separate timelines (S time
    and Ku time are budgeted separately). Targets and false alarms are spread over the faces and scan angles like
    the sector's search positions.

Flagged [D] = decision, change on request.
"""
from __future__ import annotations

import copy
import dataclasses
import functools
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import detection
from . import environment as env
from . import isolation as iso
from . import optimizer as mo
from . import radar_model as rm
from . import repoint
from . import sky as skym
from . import trackmodel as tm
from .params import P

# Values from radar_system.toml ([confirmation], [tracking], [band.Ku], [tool.confirmation]).
_T = "tool.confirmation."
BAND = rm.BAND_KU
KU_REF_GHZ = round((BAND.core_lo_ghz + BAND.core_hi_ghz) / 2.0, 9)   # functions without a sector (fast estimate)
KU_FREQ_STEP_GHZ = P.get(_T + "ku_freq_step_mhz") / 1e3              # Ku centre grid (step 3.2, A2)
KU_FREQ_COARSE_GHZ = P.get(_T + "ku_freq_coarse_mhz") / 1e3          # Ku centres evaluated (A5)
KU_FREQ_QUIET_CENTRES = P.get(_T + "ku_freq_quiet_centres")          # quiet Ku spectrum (A8, 25 Oct 2026)
SHIP_ELEC_W = P.get("power.ship_elec_total_mw") * 1e6     # ship total shared by the faces (step 4.1)
K_ML_KU = rm.angle_k_tx("Ku")                        # Ku angle accuracy x sqrt(SNR), in Ku TX beamwidths (3.3)
K_ML_S = rm.angle_k_tx("S")                          # S angle accuracy x sqrt(SNR), in S TX beamwidths (CRLB x margin)
BIAS_MRAD = P.get("confirmation.bias_mrad")          # residual S/Ku alignment bias per axis (absolute angle, constant in u, v)
# return on 3.2 from step 5.4 (K5, user, 27 October 2026): the share chain_split of the chain requirement
# p_track_fail_max is left to the confirmation; the default minimum probability of confirming a true target follows
CONFIRM_P_MIN = 1.0 - P.get("tracking.chain_split") * P.get("tracking.p_track_fail_max")
CONFIRM_ATTEMPTS = int(P.get("confirmation.confirm_attempts"))   # K3: confirmations of one detection at most
CONFIRM_ATTEMPTS_MAX = int(P.get("confirmation.confirm_attempts_max"))   # K9 (a): up to this for weak detections
CONFIRM_MODES = ("KU1+7", "KU7", "S2")
CLUSTER_N = 7                   # hexagonal cluster: centre + first ring, spacing rm.BEAM_SPACING_HPBW
KAPPA_ITER = int(P.get("tool.tracking.kappa_iter"))     # return on 3.3 from 5.4: look / kappa fixed point
KAPPA_STEP_DB = float(P.get("tool.tracking.kappa_step_db"))
KAPPA_MODE = str(P.get("tool.tracking.kappa_mode"))     # decision 4: "energy" (approximation) or "resize" (exact)
if KAPPA_MODE not in ("energy", "resize"):
    raise ValueError(f"{P.path}: tool.tracking.kappa_mode must be 'energy' or 'resize'")
INIT_LOSS_MAX = P.get("tracking.p_loss_max")   # return on 3.3 (H7): alert above this initiation loss
KEEP_MC_OBJECTS = False         # step 5.4 (M3): sector_budget keeps the track states and update looks per bracket
ANGLE_ZONE_SIGMAS = P.get("tracking.angle_zone_sigmas")     # angular zone opening = full width at +/-2 sigma
MARGIN_MAX_DB = P.get(_T + "margin_db_max")          # power margins of the confirmation looks: 0 .. max dB,
MARGIN_TOL_DB = P.get(_T + "margin_db_tol")          # solved continuously to this tolerance (step 3.2, B6)
TOUCH_STEP_DB = P.get(_T + "s2_touch_step_db")       # S2: margin grid of the S touch (its Ku margin by bisection)
MARGIN_DB = np.arange(0.0, MARGIN_MAX_DB + 0.25, 0.5)   # report grid of confirm_tables() only (test bench)
FA_R4_MEAN = P.get("confirmation.fa_r4_mean")        # a false alarm's Ku look is sized for the threat at the declared
                                                     # range: mean of (R / R_det)^4 over a range uniform in [0, R_det]
# Ku look search space (confirmation and tracking, step 3.2 B)
KU_TAU = tuple(sorted({t for t in P.get(_T + "ku_tau_s") if t <= rm.tau_max_s(BAND)} | {rm.tau_max_s(BAND)}))
KU_N = tuple(P.get(_T + "ku_n_pulses"))
KU_PRI = tuple(P.get(_T + "ku_pri_s"))
KU_BW = tuple(P.get(_T + "ku_sig_bw_mhz"))
KU_INTEGRATIONS = tuple(P.get(_T + "ku_integrations"))     # both offered, the optimiser chooses (E5)
KU_CLUSTER_TX = tuple(P.get(_T + "ku_cluster_tx"))         # cluster sent as: sequential | superposition | partition (C1)
KU_DUTY_MAX = BAND.duty_max                          # Ku TRM duty ceiling
N_SNR_Q = P.get(_T + "n_snr_quantiles")              # quantiles of the detected-echo SNR
GH_NODES, GH_WEIGHTS = np.polynomial.hermite_e.hermegauss(P.get(_T + "gh_nodes"))
GH_WEIGHTS = GH_WEIGHTS / GH_WEIGHTS.sum()
_TX_LOSS_DB_PER_HPBW2 = 10.0 * math.log10(math.e) * 4.0 * math.log(2.0)   # 12.04 dB at 1 HPBW (radar_model.grid_loss_db)
_CLUSTER_XY = np.array([(0.0, 0.0)] + [(rm.BEAM_SPACING_HPBW * math.cos(math.radians(60 * j)),
                                        rm.BEAM_SPACING_HPBW * math.sin(math.radians(60 * j))) for j in range(6)])
if not set(KU_INTEGRATIONS) <= {"COHERENT", "NONCOHERENT"} or not KU_INTEGRATIONS:
    raise ValueError(f"{P.path}: tool.confirmation.ku_integrations must list COHERENT and / or NONCOHERENT")
if not set(KU_CLUSTER_TX) <= {"sequential", "superposition", "partition"} or "sequential" not in KU_CLUSTER_TX:
    raise ValueError(f"{P.path}: tool.confirmation.ku_cluster_tx: sequential (always) and optionally superposition, "
                     f"partition")
if min(KU_BW) <= 0 or max(KU_BW) > BAND.bi_max_mhz:
    raise ValueError(f"{P.path}: tool.confirmation.ku_sig_bw_mhz must lie in (0, {BAND.bi_max_mhz:g}] MHz")


# ----------------------------------------------------------------------------- Ku frequency (step 3.2, A)
def exclusions_ku(sec: dict) -> Tuple[Tuple[float, float], ...]:
    """The Ku spectrum the sector must not touch (GHz): copy of the mission's spectrum_exclusions_ghz["Ku"] made by
    optimizer.with_spectrum()."""
    return tuple((float(a), float(b)) for a, b in sec.get("ku_spectrum_exclusions_ghz", ()))


def ku_centre_grid() -> List[float]:
    n = int(math.floor((BAND.usable_hi_ghz - BAND.usable_lo_ghz) / KU_FREQ_STEP_GHZ + 1e-9))
    return [round(BAND.usable_lo_ghz + k * KU_FREQ_STEP_GHZ, 9) for k in range(n + 1)]


def ku_valid_centres(excl) -> List[float]:
    """Centres where at least the narrowest Ku signal fits inside the band and outside the exclusions (A2)."""
    return [f for f in ku_centre_grid() if mo.signal_fits(f, min(KU_BW), excl, BAND)]


def ku_centres(sec: dict, sky: Optional[dict] = None) -> List[float]:
    """Ku centres evaluated for a sector (A5): forced (ku_freq_ghz_candidates, kept where a signal fits); in a quiet
    Ku spectrum (no Ku exclusion, no jammer in the mission sky or on the sector) KU_FREQ_QUIET_CENTRES centres spread
    over the core band (25 October 2026, A8); else the valid centres on the coarse step plus the valid centre nearest
    the middle of every free fragment."""
    excl = exclusions_ku(sec)
    forced = sec.get("ku_freq_ghz_candidates")
    if forced:
        return sorted({round(float(f), 9) for f in forced if mo.signal_fits(float(f), min(KU_BW), excl, BAND)})
    valid = ku_valid_centres(excl)
    if KU_FREQ_QUIET_CENTRES > 0 and not excl and not env.jammers(sky) and sec.get("jammer_type") is None:
        return mo.quiet_centres(valid, KU_FREQ_QUIET_CENTRES, BAND)       # A8 (25 October 2026): quiet Ku spectrum
    k = int(round(KU_FREQ_COARSE_GHZ / KU_FREQ_STEP_GHZ))
    out = {f for f in valid if int(round((f - BAND.usable_lo_ghz) / KU_FREQ_STEP_GHZ)) % k == 0}
    for a, b in mo.free_fragments(excl, BAND):
        inside = [f for f in valid if a <= f <= b]
        if inside:
            out.add(min(inside, key=lambda f: (abs(f - (a + b) / 2.0), f)))
    return sorted(out)


def validate_ku_frequencies(sec: dict) -> List[str]:
    """Forced Ku centres (sector key ku_freq_ghz_candidates, A7): in the Ku band, and at least one where a Ku signal
    fits outside the exclusions."""
    fs = sec.get("ku_freq_ghz_candidates")
    if fs is None:
        return []
    if not isinstance(fs, (list, tuple)) or not fs or not all(isinstance(f, (int, float)) for f in fs):
        return ["ku_freq_ghz_candidates must be a list of frequencies (GHz)"]
    out = [f"ku_freq_ghz_candidates: {f:g} GHz is outside the Ku band" for f in fs if not BAND.contains(f)]
    if not out and not any(mo.signal_fits(f, min(KU_BW), exclusions_ku(sec), BAND) for f in fs):
        out.append("ku_freq_ghz_candidates: no Ku signal fits at these frequencies (band edges or spectrum exclusions)")
    return out


def _core_distance(f: float) -> float:
    return abs(f - KU_REF_GHZ)


# ----------------------------------------------------------------------------- detection statistics (step 3.2 E4)
def pd_single(snr, fluct):
    """Probability of detection of one look (single sample) at mean SNR snr (linear, array), Pfa = DETECT_PFA, for the
    target's fluctuation (a Swerling case or a Gamma shape, exact laws of core/detection.py)."""
    snr = np.maximum(np.asarray(snr, dtype=float), 1e-30)
    return _interp_grid(detection.pd_curve(1, fluct, rm.DETECT_PFA), 10.0 * np.log10(snr))


_G0, _GS = float(detection.SNR_GRID_DB[0]), float(detection.SNR_GRID_DB[1] - detection.SNR_GRID_DB[0])


def _interp_grid(curve: np.ndarray, x_db: np.ndarray) -> np.ndarray:
    """np.interp(x_db, detection.SNR_GRID_DB, curve) by index on the regular grid (step 3.2, Q6 E3): linear between
    grid points, flat beyond the ends; rounding differences only (~1e-16)."""
    n = len(curve)
    x = np.clip((x_db - _G0) / _GS, 0.0, n - 1.0)
    i = np.minimum(x.astype(np.intp), n - 2)
    fr = x - i
    return curve.take(i) * (1.0 - fr) + curve.take(i + 1) * fr


def _snr_ref(fluct) -> float:
    """Mean SNR (linear) of a look designed for Pd = DETECT_PD (single sample)."""
    return rm.db_to_lin(detection.snr_req_db(1, fluct, rm.DETECT_PFA, rm.DETECT_PD))


@functools.lru_cache(maxsize=256)
def detected_snr_quantiles(snr_mean: float, fluct) -> np.ndarray:
    """SNR of the echoes that crossed the threshold, at mean SNR snr_mean (N_SNR_Q equal-probability quantiles): the
    power is Gamma(k) with mean snr_mean (k = the look's shape; steady: one value), conditioned on detection by taking
    the strongest DETECT_PD of the law."""
    k = detection.shape_of(fluct, 1)
    if math.isinf(k) or k > 1e4:
        return np.full(N_SNR_Q, snr_mean)
    x = snr_mean * np.linspace(0.0, 40.0, 40001)
    y = k * x / snr_mean
    with np.errstate(divide="ignore", invalid="ignore"):
        dens = np.power(y, k - 1.0) * np.exp(-y - math.lgamma(k))   # density of y = k x / snr_mean (0^0 = 1)
    dens = np.where(np.isfinite(dens), dens, 0.0)             # y = 0 with k < 1: integrable, left out
    seg = 0.5 * (dens[1:] + dens[:-1]) * np.diff(y)           # trapezoids
    surv = np.concatenate([np.cumsum(seg[::-1])[::-1], [0.0]])   # P(Y > y_i)
    surv = surv / surv[0]
    u = (np.arange(N_SNR_Q) + 0.5) / N_SNR_Q
    out = np.interp(rm.DETECT_PD * (1.0 - u), surv[::-1], x[::-1])
    out.setflags(write=False)
    return out


@functools.lru_cache(maxsize=256)
def first_detection_snr_quantiles(fluct, pd_cum: float, q: float) -> np.ndarray:
    """SNR of the echo that sets off the confirmation (K2, return on 3.2 from step 5.4): the search look meets the
    cumulative criterion (pd_cum by the requirement range, looks spaced q = v RR / R, step 2.3), and the target is
    confirmed on the FIRST look that detects it, often far out and on a strong fluctuation; each look k of each phase
    weighs P(first detection there) and gives the detected-SNR law of a look at its mean SNR (single sample, Pfa of
    the Ku looks, as the design law it replaces). N_SNR_Q equal-probability quantiles of the mixture."""
    if q <= 0.0:
        return detected_snr_quantiles(_snr_ref(fluct), fluct)
    x_r = detection.required_snr_db(1, fluct, rm.DETECT_PFA, pd_cum, q=q, floor=rm.CUM_PD_FLOOR)
    curve = detection.pd_curve(1, fluct, rm.DETECT_PFA)
    phases = (np.arange(16) + 0.5) / 16
    vals, wts = [], []
    for phi in phases:
        miss = 1.0
        for k in range(2000):
            m_db = x_r - 40.0 * math.log10(1.0 + (k + phi) * q)
            p = float(np.interp(m_db, detection.SNR_GRID_DB, curve))
            w_k = miss * p
            if w_k > 1e-6:
                xd = detected_snr_quantiles(round(float(rm.db_to_lin(m_db)), 12), fluct)
                vals.append(np.asarray(xd))
                wts.append(np.full(len(xd), w_k / len(xd)))
            miss *= 1.0 - p
            if miss < 1e-6 or (m_db < x_r - 30.0 and p < 1e-6):
                break
    if not vals:
        return detected_snr_quantiles(_snr_ref(fluct), fluct)
    q_vals, _ = detection.quantiles(np.concatenate(vals), N_SNR_Q, np.concatenate(wts))
    return q_vals


def _tx_gain(dx, dy, ku: float = 1.0, kv: float = 1.0):
    """Two-axis pointing error (in full-face beamwidths) -> TX gain factor of a beam ku x kv beamwidths wide (Gaussian
    main lobe, as the grid loss)."""
    return 10.0 ** (-_TX_LOSS_DB_PER_HPBW2 * ((dx / ku) ** 2 + (dy / kv) ** 2) / 10.0)


# ----------------------------------------------------------------------------- the 7-beam cluster (step 3.2 C)
@dataclasses.dataclass(frozen=True)
class ClusterTx:
    """How the confirmation cluster is transmitted: beam centres and widths (full-face Ku beamwidths), EIRP of each
    beam relative to one full-face beam at full power, look energies one cluster costs, round trips."""
    name: str
    beams: Tuple[Tuple[float, float, float, float], ...]
    eirp: float
    looks: float


def superposition_loss_db(b: int) -> float:
    """TX loss of a phase-only superposition of b beams: the [multibeam] table, interpolated in log2(b) between its
    entries (7 beams: between 4 and 8, 0.76 dB; step 3.2 C2). The search keeps its own entries only."""
    xs = sorted(rm.MB_SUP_LOSS_DB)
    if b in rm.MB_SUP_LOSS_DB:
        return rm.MB_SUP_LOSS_DB[b]
    if b < xs[0] or b > xs[-1]:
        raise ValueError(f"superposition of {b} beams is outside the [multibeam] table {xs}")
    return float(np.interp(math.log2(b), [math.log2(x) for x in xs], [rm.MB_SUP_LOSS_DB[x] for x in xs]))


@functools.lru_cache(maxsize=4)
def hex_rings_xy(rings: int, spacing: float) -> np.ndarray:
    """Sites of a hexagonal lattice of step `spacing`, ring by ring from the centre (1, 7, 19, 37 ... sites)."""
    pts = [(0.0, 0.0)]
    a, b = np.array([1.0, 0.0]), np.array([0.5, math.sqrt(3.0) / 2.0])
    for r in range(1, rings + 1):
        ring = [tuple(spacing * (i * a + j * b)) for i in range(-r, r + 1) for j in range(-r, r + 1)
                if max(abs(i), abs(j), abs(i + j)) == r]
        pts.extend(sorted(ring, key=lambda p: math.atan2(p[1], p[0])))
    return np.array(pts)


# K11 (return on 3.2 from step 5.4): the cluster of attempt a is shifted by (a mod 3) / 3 (a + b) of its lattice, so
# that a target on a cell vertex (the worst point) is on a beam axis at the next attempt; the centre look of KU1+7
# stays on the estimate (most targets are close to it)
LATTICE_SHIFT = (np.array([1.0, 0.0]) + np.array([0.5, math.sqrt(3.0) / 2.0])) / 3.0


def attempt_shift(a: int, spacing: float) -> np.ndarray:
    """Shift of the lattice of attempt a: 0, +(a + b)/3, -(a + b)/3 (the two kinds of cell vertex, each on a beam axis
    within the first ring; +2/3 would put it in the second ring)."""
    return (0.0, 1.0, -1.0)[a % 3] * LATTICE_SHIFT * spacing


def cluster_variants() -> Tuple[ClusterTx, ...]:
    """The ways of sending the cluster that tool.confirmation.ku_cluster_tx offers; K9 (c') (return on 3.2 from step
    5.4): also the sequential clusters of 2 and 3 rings (19, 37 beams) for the weak first detections."""
    hexs = tuple((float(x), float(y), 1.0, 1.0) for x, y in _CLUSTER_XY)
    out = [ClusterTx("sequential", hexs, 1.0, float(CLUSTER_N))]
    for rings in (2, 3):
        big = tuple((float(x), float(y), 1.0, 1.0) for x, y in hex_rings_xy(rings, rm.BEAM_SPACING_HPBW))
        out.append(ClusterTx(f"sequential x{len(big)}", big, 1.0, float(len(big))))
    if "superposition" in KU_CLUSTER_TX:
        out.append(ClusterTx(f"superposition x{CLUSTER_N}", hexs,
                             1.0 / (CLUSTER_N * rm.db_to_lin(superposition_loss_db(CLUSTER_N))), 1.0))
    if "partition" in KU_CLUSTER_TX:
        s = rm.BEAM_SPACING_HPBW
        for b in rm.MB_PART_BEAMS:
            kinds = ["quarter"] if b == 4 else ["half_h", "half_v"]
            for kind in kinds:
                sa = rm.subaperture("Ku", kind)
                ku, kv = sa.hpbw_u / sa.full_hpbw_u, sa.hpbw_v / sa.full_hpbw_v
                if b == 4:      # a rhombus of the stretched hexagonal lattice around the estimate
                    pts = [(0.0, 0.0), (1.0, 0.0), (0.5, math.sqrt(3) / 2), (1.5, math.sqrt(3) / 2)]
                    cx, cy = 0.75, math.sqrt(3) / 4
                    beams = tuple(((x - cx) * s * ku, (y - cy) * s * kv, ku, kv) for x, y in pts)
                elif ku >= kv:  # two neighbours along the wide axis of the half beam
                    beams = ((-0.5 * s * ku, 0.0, ku, kv), (0.5 * s * ku, 0.0, ku, kv))
                else:
                    beams = ((0.0, -0.5 * s * kv, ku, kv), (0.0, 0.5 * s * kv, ku, kv))
                out.append(ClusterTx(f"partition {kind}", beams, (sa.n_elements / sa.n_total) / b, 1.0))
    return tuple(out)


def _cluster_pd(ex, ey, snr, fluct, cl: ClusterTx):
    """Probability that at least one beam of the cluster detects, for errors ex, ey (broadcast) and centre SNR snr."""
    miss = 1.0
    for cx, cy, ku, kv in cl.beams:
        miss = miss * (1.0 - pd_single(_tx_gain(ex - cx, ey - cy, ku, kv)[..., None] * cl.eirp * snr, fluct))
    return 1.0 - miss


def _ku_after_s(sigma_ku, bias_ku: float, snr_ku, fluct, cl: ClusterTx,
                attempts: int = 1) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """KU1+7 and KU7 statistics for angle errors of standard deviation sigma_ku per axis (Ku beamwidths, array over
    the leading axes) plus a bias per axis. snr_ku: centre-look SNR (array over margins, last axis).
    Returns (success KU1+7, Ku look energies KU1+7, success KU7), each averaged over the Gaussian errors.
    attempts (K3, return on 3.2 from step 5.4): a failed confirmation is tried again on other Ku frequencies (new
    target powers) with the SAME angle error (the same S detection): failure (1 - p(e))^attempts at each error node;
    the looks of the later attempts are paid only when the earlier ones fail."""
    ex = bias_ku + sigma_ku[..., None, None] * GH_NODES[:, None]            # (..., 12, 1)
    ey = bias_ku + sigma_ku[..., None, None] * GH_NODES[None, :]            # (..., 1, 12)
    w = GH_WEIGHTS[:, None] * GH_WEIGHTS[None, :]
    fail17 = fail7 = 1.0
    l17 = 0.0
    for a_ in range(max(1, int(attempts))):
        sx, sy = attempt_shift(a_, rm.BEAM_SPACING_HPBW)                   # K11: the cluster of attempt a_ shifted,
        p1 = pd_single(_tx_gain(ex, ey)[..., None] * snr_ku, fluct)        # the centre look stays on the estimate
        p7 = _cluster_pd(ex - sx, ey - sy, snr_ku, fluct, cl)
        p17 = p1 + (1.0 - p1) * p7
        l17 = l17 + fail17 * (1.0 + (1.0 - p1) * cl.looks)             # looks of attempt a_ when the earlier failed
        fail17 = fail17 * (1.0 - p17)
        fail7 = fail7 * (1.0 - p7)
    avg = lambda a_: np.tensordot(w, a_, axes=([0, 1], [-3, -2]))         # noqa: E731
    return avg(1.0 - fail17), avg(l17), avg(1.0 - fail7)


class ConfirmStats:
    """Confirmation statistics of the three modes as functions of the power margins, for one sector: the S echo law
    (shape at the S frequency), the Ku look law (at the Ku frequency), the S / Ku beamwidth ratio, the search lattice
    and the way the cluster is sent. Independent of range and scan angle (S and Ku beams widen together on the same
    face, and the detection always happens at the design SNR)."""

    def __init__(self, fl_s, fl_ku, f_s_ghz: float, f_ku_ghz: float, spacing_hpbw: Optional[float],
                 cl: ClusterTx, p_min: float = CONFIRM_P_MIN, x_det: Optional[np.ndarray] = None,
                 attempts: int = CONFIRM_ATTEMPTS):
        self.fl_s, self.fl_ku, self.cl = fl_s, fl_ku, cl
        self.p_min = float(p_min)                                     # step 5.1: mission / sector confirm_p_min
        self.attempts = max(1, int(attempts))                         # K3
        self.q = rm.hpbw_deg(f_s_ghz) / rm.hpbw_deg(f_ku_ghz)                 # S TX beamwidth in Ku TX beamwidths
        self.snr_s = _snr_ref(fl_s)
        # K2: the SNR of the echoes that set off the confirmation, from the first detection of the cumulative search
        # criterion when given, else the design single look
        self.x_det = detected_snr_quantiles(self.snr_s, fl_s) if x_det is None else np.asarray(x_det)   # (M,)
        self.sig_ku = K_ML_S * self.q / np.sqrt(self.x_det)
        self.bias_ku = BIAS_MRAD * 1e-3 / math.radians(rm.hpbw_deg(f_ku_ghz))  # in Ku beamwidths
        self.snr_ku = _snr_ref(fl_ku)                                          # Ku centre-look SNR at 0 dB margin
        self.touch_snr0 = self.snr_s * rm.db_to_lin(rm.grid_loss_db(spacing_hpbw) + rm.grid_rx_loss_db())
        self._memo: Dict[tuple, Tuple[float, float]] = {}

    def ku(self, mode: str, m_db: float) -> Tuple[float, float]:
        """(success, Ku look energies per target) of KU1+7 or KU7 at margin m_db."""
        key = (mode, round(m_db, 6))
        if key not in self._memo:
            s17, l17, s7 = (a.mean(axis=0) for a in _ku_after_s(self.sig_ku, self.bias_ku,
                                                              np.array([self.snr_ku * rm.db_to_lin(m_db)]),
                                                              self.fl_ku, self.cl, self.attempts))
            self._memo[("KU1+7", round(m_db, 6))] = (float(s17[0]), float(l17[0]))
            f7 = 1.0 - float(s7[0])                       # looks: the attempts paid while the earlier fail
            f7_1 = f7 ** (1.0 / self.attempts) if self.attempts > 1 else f7
            self._memo[("KU7", round(m_db, 6))] = (float(s7[0]),
                                                   float(self.cl.looks) * sum(f7_1 ** k for k in range(self.attempts)))
        return self._memo[key]

    def s2_vec(self, mt_db: np.ndarray, mk_db: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(success, Ku look energies per target) of S2 for paired margins (arrays of the same length): pointed S touch
        at mt_db (no grid loss, pointing error in S beams), then KU1+7 at mk_db on the combined angle estimate. The
        touch part is kept per touch grid (it does not depend on the Ku margin)."""
        mt_db, mk_db = np.asarray(mt_db, dtype=float), np.asarray(mk_db, dtype=float)
        key = tuple(np.round(mt_db, 6))
        tp = self._memo.get(("touch", key))
        if tp is None:
            sig_s, bias_s = self.sig_ku / self.q, self.bias_ku / self.q
            ex = bias_s + sig_s[:, None, None] * GH_NODES[:, None]
            ey = bias_s + sig_s[:, None, None] * GH_NODES[None, :]
            w = GH_WEIGHTS[:, None] * GH_WEIGHTS[None, :]
            t_snr = self.touch_snr0 * 10.0 ** (mt_db / 10.0)                                 # (n,)
            pt = pd_single(_tx_gain(ex, ey)[..., None] * t_snr, self.fl_s)                   # (M, 12, 12, n)
            pt = np.tensordot(w, pt, axes=([0, 1], [1, 2]))                                  # (M, n)
            sig2 = K_ML_S * self.q / np.sqrt(self.x_det[:, None] + t_snr[None, :])           # (M, n)
            # TX gains of the centre look and of the cluster beams on the combined estimate: they depend on the
            # touch margins only, not on the Ku margin of the bisection (step 3.2, Q6 E2)
            ex = self.bias_ku + sig2[..., None, None] * GH_NODES[:, None]                     # (M, n, 12, 1)
            ey = self.bias_ku + sig2[..., None, None] * GH_NODES[None, :]                     # (M, n, 1, 12)
            g1 = _tx_gain(ex, ey)                                                             # (M, n, 12, 12)
            g7 = [_tx_gain(ex - cx, ey - cy, ku, kv) * self.cl.eirp for cx, cy, ku, kv in self.cl.beams]
            tp = self._memo[("touch", key)] = (pt, g1, g7)
        pt, g1, g7 = tp
        snr = (self.snr_ku * 10.0 ** (mk_db / 10.0))[None, :, None, None]                    # paired with mt
        p1 = pd_single(g1 * snr, self.fl_ku)                                                  # (M, n, 12, 12)
        miss = 1.0
        for g in g7:
            miss = miss * (1.0 - pd_single(g * snr, self.fl_ku))
        p7 = 1.0 - miss
        w = GH_WEIGHTS[:, None] * GH_WEIGHTS[None, :]
        p17 = p1 + (1.0 - p1) * p7
        a = self.attempts                                     # K3: the Ku part tried again, same combined estimate
        s17 = np.tensordot(1.0 - (1.0 - p17) ** a, w, axes=([2, 3], [0, 1]))                  # (M, n)
        l17 = np.tensordot((1.0 + (1.0 - p1) * self.cl.looks) * sum((1.0 - p17) ** k for k in range(a)), w,
                           axes=([2, 3], [0, 1]))
        return (pt * s17).mean(axis=0), (pt * l17).mean(axis=0)

    def s2(self, m_touch_db: float, m_ku_db: float) -> Tuple[float, float]:
        """s2_vec for one pair of margins."""
        a, b = self.s2_vec(np.array([m_touch_db]), np.array([m_ku_db]))
        return float(a[0]), float(b[0])

    def s2_min_margins(self, mt_db: np.ndarray) -> np.ndarray:
        """Smallest Ku margin (dB, within MARGIN_TOL_DB) reaching CONFIRM_P_MIN after a touch at each mt_db, by a
        bisection run for every touch margin at once; NaN where even MARGIN_MAX_DB does not."""
        n = len(mt_db)
        top = self.s2_vec(mt_db, np.full(n, MARGIN_MAX_DB))[0] >= self.p_min
        zero = self.s2_vec(mt_db, np.zeros(n))[0] >= self.p_min
        lo, hi = np.zeros(n), np.full(n, MARGIN_MAX_DB)
        while (hi - lo).max() > MARGIN_TOL_DB:
            mid = 0.5 * (lo + hi)
            ok = self.s2_vec(mt_db, mid)[0] >= self.p_min
            lo, hi = np.where(ok, lo, mid), np.where(ok, mid, hi)
        return np.where(~top, np.nan, np.where(zero, 0.0, hi))

    def min_margin(self, f) -> Optional[float]:
        """Smallest margin (dB, within MARGIN_TOL_DB) at which f(margin) = success reaches CONFIRM_P_MIN; None if even
        MARGIN_MAX_DB does not (success increases with the margin)."""
        if f(MARGIN_MAX_DB) < self.p_min:
            return None
        if f(0.0) >= self.p_min:
            return 0.0
        lo, hi = 0.0, MARGIN_MAX_DB
        while hi - lo > MARGIN_TOL_DB:
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if f(mid) < self.p_min else (lo, mid)
        return hi


def confirm_tables(fluct, f_s_ghz: float, spacing_hpbw: Optional[float] = None, f_ku_ghz: float = KU_REF_GHZ,
                   cluster: str = "sequential") -> dict:
    """Confirmation statistics of the three modes on the report grid MARGIN_DB (test bench), with the same
    fluctuation for the S echo and the Ku look."""
    cl = next(c for c in cluster_variants() if c.name == cluster)
    st = ConfirmStats(fluct, fluct, f_s_ghz, f_ku_ghz, spacing_hpbw, cl)
    s17 = np.array([st.ku("KU1+7", m) for m in MARGIN_DB])
    s7 = np.array([st.ku("KU7", m) for m in MARGIN_DB])
    mt, mk = np.meshgrid(MARGIN_DB, MARGIN_DB, indexing="ij")
    a, b = st.s2_vec(mt.ravel(), mk.ravel())
    return {"KU1+7": (s17[:, 0], s17[:, 1]), "KU7": (s7[:, 0], s7[:, 1]),
            "S2": (a.reshape(mt.shape), b.reshape(mt.shape))}


# ----------------------------------------------------------------------------- Ku looks (step 3.2, B and E)
def _ku_agility(length_m: float, n: int, bw_mhz: float) -> float:
    """Excursion of the agile variant of a noncoherent Ku train (step 3.2 B7, the search rule B7-B8): (n - 1) c / 2L
    for n independent draws, within the Ku band minus the signal, on the Ku grid step; 0 if not even 2 draws fit."""
    if n < 2 or not length_m or length_m <= 0:
        return 0.0
    dfc = rm.C_MPS / (2.0 * length_m) / 1e6
    width = (BAND.usable_hi_ghz - BAND.usable_lo_ghz) * 1e3 - bw_mhz
    step = KU_FREQ_STEP_GHZ * 1e3
    a = math.floor(min((n - 1) * dfc, width) / step + 1e-9) * step
    return a if a >= dfc - 1e-9 else 0.0


@functools.lru_cache(maxsize=256)
def _ku_candidates(pack: Optional[rm.TrackingPack], length_m: float) -> Tuple[Tuple[rm.Waveform, Optional[rm.Waveform]], ...]:
    """(detection waveform, tracking waveform or None) pairs with the same timing: every pulse, PRI, pulse count,
    bandwidth and integration of the space, and the agile variant of each noncoherent train (B7; length_m: the
    target length that sets the excursion)."""
    out = []
    for integ in KU_INTEGRATIONS:
        for bw in KU_BW:
            for pri in KU_PRI:
                for n in KU_N:
                    for tau in KU_TAU:
                        if tau > KU_DUTY_MAX * pri:
                            continue
                        ags = [0.0]
                        if integ == "NONCOHERENT":
                            a = _ku_agility(length_m, n, bw)
                            if a > 0:
                                ags.append(a)
                        for ag in ags:
                            try:
                                det = rm.Waveform("KU-CONF", "CONFIRM", "Ku", tau, n, pri, bw, 1, integ, agility_mhz=ag)
                                trk = None if pack is None else rm.Waveform("KU-TRK", "TRACK", "Ku", tau, n, pri, bw, 1,
                                                                            integ, tracking_pack=pack, agility_mhz=ag)
                            except ValueError:
                                continue
                            out.append((det, trk))
    return tuple(out)


@functools.lru_cache(maxsize=4096)
def _ku_scan(f: float, theta: float) -> Optional[Tuple[float, float]]:
    """(h, h_bt): R(theta) / R(0) of the noise-limited range and of the self-protection burn-through range at f. As
    in the search (optimizer._scan_factor), the scan enters only through the array gain and the active mismatch, so
    both are the same for every waveform: checked on two dissimilar Ku waveforms; None if a model change breaks it
    (callers then solve() at theta)."""
    tgt = rm.Target("ref", 1.0, 1, 5000.0)
    hs = []
    for w in (rm.Waveform("ref1", "CONFIRM", "Ku", 20e-6, 1, 1e-3, 25.0, 1, "NONCOHERENT"),
              rm.Waveform("ref2", "CONFIRM", "Ku", 200e-6, 8, 2e-3, 100.0, 1, "COHERENT")):
        try:
            hs.append(rm.solve(w, f, float(theta), tgt).range_km / rm.solve(w, f, 0.0, tgt).range_km)
        except ValueError:
            return None
    if not math.isclose(hs[0], hs[1], rel_tol=1e-9):
        return None
    g = rm.gain_lin(1.0, f, float(theta)) / rm.gain_lin(1.0, f, 0.0)
    mm = 2.0 * (rm.active_mismatch(f, float(theta))["loss_db_per_pass"] - rm.active_mismatch(f, 0.0)["loss_db_per_pass"])
    return hs[0], math.sqrt(g * 10.0 ** (-mm / 10.0))


class _LookSpace:
    """The Ku candidates of one look type at one frequency (target, RX option, velocity span, jammer, track pack): the
    broadside parts of every candidate (noise range, burn-through range, required SNR with the position losses, look
    law; cold sky), and their costs as functions of the pulse scale."""

    def __init__(self, f: float, tgt: rm.Target, sll: float, v_span: Optional[float], spj: Optional[float],
                 pack: Optional[rm.TrackingPack], excl: Tuple[Tuple[float, float], ...], staggered: bool,
                 pointing: Optional[Tuple[str, float]] = None, pd: Optional[float] = None):
        self.f, self.pointing, self.pd = f, pointing, pd
        pairs = [(d, t) for d, t in _ku_candidates(pack, float(tgt.length_m or 0.0))
                 if mo.signal_fits(f, d.occupied_mhz, excl, BAND)]
        if staggered:
            pairs = [(dataclasses.replace(d, staggered=True) if d.n > 1 else d,
                      None if t is None else (dataclasses.replace(t, staggered=True) if t.n > 1 else t)) for d, t in pairs]
        rows = []
        for d, t in pairs:
            pd_ = mo.parts_at(d, f, 0.0, tgt, sll, v_span, None, None, spj, None, pointing, pd)
            if pd_ is None:
                continue
            det = pd_
            if t is not None:
                pt = mo.parts_at(t, f, 0.0, tgt, sll, v_span, None, None, spj, None, pointing, pd)
                if pt is None:
                    continue
                if rm.jammed_range_km(pt[0], pt[1]) < rm.jammed_range_km(pd_[0], pd_[1]):
                    pd_ = pt                                  # the accuracy drives this look (SNR = max(det, acc))
            rows.append((d, t, pd_, det))
        self.det = [r[0] for r in rows]
        self.trk = [r[1] for r in rows]
        self.rn = np.array([r[2][0] for r in rows])
        self.rbt = np.array([r[2][1] for r in rows])
        self.sdb = np.array([r[2][2] for r in rows])
        self.law = np.array([r[2][4] for r in rows], dtype=int)
        self.det_parts = [r[3] for r in rows]          # detection parts of each row (step 3.3: Pd of coasted looks)
        self.costs = mo.PositionCosts(self.det) if self.det else None
        self.blind = np.array([rm.blind_range_km(BAND, d.tau_s) for d in self.det])
        self.bw = np.array([d.sig_bw_mhz for d in self.det])


_SPACES: Dict[tuple, _LookSpace] = {}


def _space(f: float, tgt: rm.Target, sll: float, v_span: Optional[float], spj: Optional[float],
           pack: Optional[rm.TrackingPack], excl: Tuple[Tuple[float, float], ...], staggered: bool,
           pointing: Optional[Tuple[str, float]] = None, pd: Optional[float] = None) -> _LookSpace:
    key = (f, tgt, sll, v_span, spj, pack, excl, staggered, pointing, pd)
    sp = _SPACES.get(key)
    if sp is None:
        if len(_SPACES) >= 512:
            _SPACES.clear()
        sp = _SPACES[key] = _LookSpace(f, tgt, sll, v_span, spj, pack, excl, staggered, pointing, pd)
    return sp


def _clutter_gain(sp: _LookSpace, clutter: Optional[List[dict]], tgt: rm.Target) -> np.ndarray:
    """SNR increase G (>= 1, inf = blinded) that restores each candidate's look Pd against the sidelobe clutter of the
    surfaces (the search model of P4, R1 b1: clutter ratio y at the 99 % quantile of the group's pointings, for the
    look's range cell c / 2B and coherent blocks, through its Doppler filter, Pd averaged over the target speeds)."""
    if not clutter or not sp.det:
        return np.ones(len(sp.det))
    v_lo, v_hi = clutter[0]["v_lo"], clutter[0]["v_hi"]
    nv = len(env.target_velocities(v_lo, v_hi))
    a = np.zeros((len(sp.det), nv))
    for j, w in enumerate(sp.det):
        mb, t_b = rm.coherent_blocks(w.n, w.pri_s, w.tau_s, w.integration, tgt.swerling, tgt, sp.f)
        ppb = w.n // mb
        beta = (mo.B_REF_MHZ / w.sig_bw_mhz) / mb
        for c in clutter:
            rho = np.ones(nv)
            if w.integration == "COHERENT" and ppb >= 2 and c["closing_mps"] is not None and c["spread_mps"] is not None:
                pv = rm.oscillator_phase_var(sp.f, t_b, rm.C_MPS * w.pri_s / 2e3, w.sig_bw_mhz)
                rho = env.doppler_residual(c["closing_mps"], c["spread_mps"], v_lo, v_hi, sp.f, w.pri_s, ppb,
                                           w.doppler_window_db, pv)
            a[j] += beta * c["y"] * rho
    if not a.any():
        return np.ones(len(sp.det))
    curves = mo._curves(sp.law)
    pstar = mo._pd_at(curves, sp.law, sp.sdb)
    g = np.ones(len(sp.det))
    need = a.max(axis=1) > 0.0
    if need.any():
        g[need] = mo._clutter_gain(curves, sp.law[need], sp.sdb[need], pstar[need], a[need])
    return g


def _look_scale(sp: _LookSpace, idx: int, x: float) -> Tuple[float, float, float]:
    """(electrical J, Ku TX s, heat J) of candidate idx when it must deliver x times its full-pulse energy at the
    range: shortened at saturated PAs below 1 (no excess energy, step 3.2 B5, floor tau B >= 1), repeated above."""
    if x <= 1.0:
        e, t, _, eta = sp.costs(np.array([x]), np.array([idx]))
        e, t, eta = float(e[0]), float(t[0]), float(eta[0])
    else:
        e1, t1, _, eta1 = sp.costs(np.array([1.0]), np.array([idx]))
        e, t, eta = float(e1[0]) * x, float(t1[0]) * x, float(eta1[0])
    return e, t, e * (1.0 - eta)


def ku_look(theta: float, tgt: rm.Target, r_km: float, p_avg_max_w: float,
            pack: Optional[rm.TrackingPack] = None, rx_sll_db: Optional[float] = None,
            t_sys_k: Optional[float] = None, r_min_km: Optional[float] = None,
            v_span_mps: Optional[float] = None, f_ghz: float = KU_REF_GHZ, spj: Optional[float] = None,
            clutter: Optional[List[dict]] = None, excl: Tuple[Tuple[float, float], ...] = (),
            pointing: Optional[Tuple[str, float]] = None, pd: Optional[float] = None,
            bw_max_mhz: Optional[float] = None) -> Optional[dict]:
    """Cheapest Ku look reaching r_km at scan theta, for detection only (pack None) or for a track update (SNR =
    max(detection, accuracy)). Cost = max(energy / face average power, Ku TX time) at the design point, the pulse
    shortened to just reach r_km (step 3.2 B5). rx_sll_db: RX weighting of the volume (None = default option);
    t_sys_k: the sector's Ku budget T_sys at f (sky noise, None = cold sky; the range scales as its fourth root);
    spj: self-protection jammer density of the sector at f (step 3.2 E1); clutter: the surfaces of the look's group
    (step 3.2 E3); excl: the Ku exclusions (an agile train must stay outside them). Pulses whose blind range exceeds
    r_min_km are left out; a train whose PRI is below 2 r / c + tau + recovery is range-ambiguous at r_km, so it is
    staggered and pays the mean eclipsing loss. v_span_mps: velocity span of the look's velocity bank (None = 0 to the
    target speed, 0 = velocity known, e.g. a track update). pointing, pd: pointing-error law and Pd of a track update
    (step 3.3, core/trackmodel.py). None when no candidate reaches r_km."""
    sll = rm.RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    if abs(theta) > BAND.scan_limit_deg(f_ghz):
        return None
    tgt_c = dataclasses.replace(tgt, name="ku")
    hb = _ku_scan(f_ghz, float(theta))
    sky_f = (BAND.t_sys_k / t_sys_k) ** 0.25 if t_sys_k else 1.0
    best = None
    for staggered in (False, True):
        sp = _space(f_ghz, tgt_c, sll, v_span_mps, spj, pack, tuple(excl), staggered, pointing, pd)
        if not sp.det:
            continue
        # ambiguity at r_km decides which variant of each train plays: unstaggered where unambiguous, staggered else
        amb = np.array([d.n > 1 and d.pri_s < rm.unambiguous_pri_s(BAND, r_km, d.tau_s) for d in sp.det])
        ok = amb if staggered else ~amb
        if r_min_km is not None:
            ok &= sp.blind <= r_min_km + 1e-9
        if bw_max_mhz is not None:                       # a narrower look (more independent hop frequencies)
            ok &= np.array([d.sig_bw_mhz <= bw_max_mhz + 1e-9 for d in sp.det])
        if not ok.any():
            continue
        g = _clutter_gain(sp, clutter, tgt_c)
        if hb is None:
            rr = np.array([_solve_range(d, f_ghz, theta, tgt_c, sll, t_sys_k, v_span_mps, spj, pointing, pd)
                           for d in sp.det])
        else:
            with np.errstate(divide="ignore", invalid="ignore"):
                rr = rm.jammed_range_km(sp.rn * hb[0] * sky_f * g ** -0.25, sp.rbt * hb[1] * g ** -0.5)
        rr = np.where(ok & np.isfinite(g) & (rr >= r_km), rr, np.nan)
        if not np.isfinite(rr).any():
            continue
        s = np.where(np.isfinite(rr), (r_km / rr) ** 4, 1.0)
        e, t, _, _ = sp.costs(s)
        cost = np.where(np.isfinite(rr), np.maximum(e / p_avg_max_w, t), np.inf)
        j = int(np.argmin(cost))
        if best is None or cost[j] < best[0]:
            best = (float(cost[j]), sp, j, float(s[j]), float(rr[j]))
    if best is None:
        return None
    cost, sp, j, s, rr = best
    e, t, q = _look_scale(sp, j, s)
    w = sp.det[j]
    return {"cost": cost, "energy_J": e, "energy_rf_J": e - q, "pae": (e - q) / e if e else 0.0, "time_s": t,
            "driver": _driver(sp.trk[j], f_ghz, theta, tgt_c, sll, t_sys_k, v_span_mps, spj) if pack else "DET",
            "wf": w, "scale": s, "reach_km": rr, "space": sp, "index": j, "f_ghz": f_ghz}


def _solve_range(w, f, theta, tgt, sll, t_sys, v_span, spj, pointing=None, pd=None) -> float:
    try:
        return rm.solve(w, f, float(theta), tgt, rm.FACE_ELEC_POWER_MW, sll, t_sys, v_span,
                        spj_density_w_hz=spj, pointing=pointing, pd=pd).range_km
    except ValueError:
        return float("nan")


def _driver(trk, f, theta, tgt, sll, t_sys, v_span, spj) -> str:
    """SNR driver of a track update (DET, ANG, DOP, RNG)."""
    if trk is None:
        return "DET"
    try:
        d = rm.solve(trk, f, float(theta), tgt, rm.FACE_ELEC_POWER_MW, sll, t_sys, v_span, spj_density_w_hz=spj).driver
    except ValueError:
        return "-"
    return d.replace("Track-", "")


def look_cost(look: dict, x: float) -> Tuple[float, float, float]:
    """(electrical J, Ku TX s, heat J) of a chosen look when it must deliver x times the energy of its design point
    (a margin, a false alarm's (R / R_det)^4)."""
    return _look_scale(look["space"], look["index"], look["scale"] * x)


def ku_burn_through(theta: float, tgt: rm.Target, r_km: float, p_avg_max_w: float, rx_sll_db: Optional[float],
                    t_sys_k: Optional[float], f_ghz: float, spj: float, excl) -> dict:
    """Ku search look pointed in the jamming axis (step 3.2 E2, scheduled at 4.3): the cheapest Ku look that reaches
    r_km against the self-protection jammer, or else the longest reach of the space."""
    look = ku_look(theta, tgt, r_km, p_avg_max_w, rx_sll_db=rx_sll_db, t_sys_k=t_sys_k, f_ghz=f_ghz, spj=spj,
                   excl=excl)
    if look is not None:
        return {"reaches_requirement": True, "range_km": round(look["reach_km"], 1), "wf": _wf_label(look["wf"], f_ghz),
                "energy_J": look["energy_J"], "time_s": look["time_s"], "heat_J": look["energy_J"] - look["energy_rf_J"]}
    sll = rm.RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    sp = _space(f_ghz, dataclasses.replace(tgt, name="ku"), sll, None, spj, None, tuple(excl), False)
    hb = _ku_scan(f_ghz, float(theta))
    if not sp.det or hb is None:
        return {"reaches_requirement": False, "range_km": 0.0, "wf": "-"}
    sky_f = (BAND.t_sys_k / t_sys_k) ** 0.25 if t_sys_k else 1.0
    rr = rm.jammed_range_km(sp.rn * hb[0] * sky_f, sp.rbt * hb[1])
    j = int(np.nanargmax(rr))
    return {"reaches_requirement": False, "range_km": round(float(rr[j]), 1), "wf": _wf_label(sp.det[j], f_ghz)}


def _with_mission_settings(sec: dict, g: dict) -> dict:
    """The sector with the mission-level settings it does not override (step 5.1): confirm_p_min, and the per-axis
    navigation pointing error of the mission's angular acceleration (internal key)."""
    out = dict(sec)
    if "confirm_p_min" not in out and g.get("confirm_p_min") is not None:
        out["confirm_p_min"] = float(g["confirm_p_min"])
    if g.get("ship_accel_max_dps2") is not None:
        out["_nav_axis_sigma_deg"] = repoint.axis_sigma_deg(float(g["ship_accel_max_dps2"]))
    out["_burn_through_allowed"] = bool(g.get("burn_through_allowed", False))
    return out


def _wf_params(w: rm.Waveform, sec: dict, f_ghz: float) -> dict:
    """Numbers of a Ku look the compute budget needs (step 4.4): pulses, PRI, pulse, bandwidth, processing banks."""
    return {"n_pulses": w.n, "pri_ms": w.pri_s * 1e3, "tau_us": w.tau_s * 1e6, "sig_bw_mhz": w.sig_bw_mhz,
            **mo._processing_dict(w, sec, f_ghz)}


def _wf_label(w: rm.Waveform, f_ghz: float = KU_REF_GHZ) -> str:
    ag = f" agile {w.agility_mhz:g} MHz" if w.agility_mhz else ""
    return (f"{f_ghz:g} GHz {w.integration[:4]} tau {w.tau_s * 1e6:.0f} us x N {w.n} @ PRI {w.pri_s * 1e3:g} ms "
            f"B {w.sig_bw_mhz:.0f} MHz{ag}")


# ----------------------------------------------------------------------------- per-sector budget
def _groups(P: np.ndarray, r: dict) -> List[dict]:
    """Scan-angle groups of the sector (optimizer.BIN_DEG wide), with their positions per face, scan span, worst angle
    and the search waveform the optimiser chose there."""
    p_tot = P.sum(axis=0)
    by_lo = {round(b["theta_lo_deg"]): b for b in r.get("wf_by_bin", [])}
    out = []
    for lo in range(0, P.shape[1], mo.BIN_DEG):
        idx = [i for i in range(lo, min(lo + mo.BIN_DEG, P.shape[1])) if p_tot[i] > 0]
        b = by_lo.get(round(lo * mo.THETA_BIN_DEG))
        if not idx or b is None:
            continue
        out.append({"theta": (idx[-1] + 1) * mo.THETA_BIN_DEG, "theta_lo": idx[0] * mo.THETA_BIN_DEG,
                    "pos": P[:, idx].sum(axis=1), "wf": b["wf"],
                    "elec_J_per_position": b["elec_J_per_position"], "pae_at_scale": b["pae_at_scale"]})
    return out


class _SectorClutter:
    """Sidelobe clutter of the sector's surfaces for its Ku looks (step 3.2 E3): per scan group and look range, the
    clutter ratio y of each surface (full-face Ku beam, range cell of B_REF_MHZ) at the (1 - short_tol) quantile of the
    group's pointings, with the surface's Doppler (P4 Q2, Q4-Q8)."""

    def __init__(self, sec: dict, sph, m: np.ndarray, sky: Optional[dict], f: float, keep: np.ndarray):
        self.sec, self.f = sec, f
        self.surf = [s_ for s_ in env.surfaces(sky)]
        self.tol = sec.get("short_tol", mo.SHORT_TOL)
        self.sll = mo.rx_sll_db(sec)
        self.tgt = mo.sector_target(sec)
        sel = m & sph.scan_zone & (sph.theta <= BAND.scan_limit_deg(f)) & (keep > 0.0)
        ii, jj = np.nonzero(sel)
        self.theta = sph.theta[ii, jj]
        self.w = (sph.w * sph.d_omega)[ii, jj] * keep[ii, jj]
        self.own = sph.owner[ii, jj].astype(np.int64)
        self.dirs = skym.unit(sph.phi[ii, jj], sph.eps[ii, jj])
        self.v_lo, self.v_hi = float(sec.get("v_close_min_mps", 0.0)), float(sec["v_close_max_mps"])
        self._memo: Dict[tuple, Optional[List[dict]]] = {}

    def __call__(self, gr: dict, r_km: float) -> Optional[List[dict]]:
        key = (gr["theta_lo"], gr["theta"], round(r_km, 6))
        if key in self._memo:
            return self._memo[key]
        out = []
        cells = (self.theta >= gr["theta_lo"] - 1e-9) & (self.theta <= gr["theta"] + 1e-9)
        if cells.any():
            dr_ref = rm.C_MPS / (2.0 * mo.B_REF_MHZ * 1e6)
            for s_ in self.surf:
                if not s_.d_min_km < r_km < s_.d_horizon_km:
                    continue                                   # no iso-range ring at this range
                y = np.zeros(int(cells.sum()))
                d, o = self.dirs[cells], self.own[cells]
                for k in np.unique(o):
                    sk = o == k
                    y[sk] = env.clutter_over_signal(s_, int(k), d[sk], "Ku", self.f, self.sll, float(r_km), dr_ref,
                                                    self.tgt.rcs_at(self.f))
                yq = float(mo._weighted_quantile_by_bin(y, np.zeros(len(y), dtype=int), self.w[cells], 1.0 - self.tol,
                                                        1)[0])
                if yq > 0.0:
                    out.append({"y": yq, "closing_mps": s_.closing_mps, "spread_mps": s_.spread_mps,
                                "v_lo": self.v_lo, "v_hi": self.v_hi, "id": s_.id})
        self._memo[key] = out or None
        return self._memo[key]


def _s_touch_wf(w: dict) -> rm.Waveform:
    """The S2 touch (step 3.2 D1): the group's search waveform, pointed, as an S waveform of role CONFIRM."""
    return rm.Waveform("S-TOUCH", "CONFIRM", "S", w["tau_us"] * 1e-6, int(w["n_pulses"]), w["pri_ms"] * 1e-3,
                       w["sig_bw_mhz"], 1, w["integration"], agility_mhz=w.get("agility_mhz", 0.0) or 0.0)


def _harmonic_cover(r: dict, f_ku: float, bw_ku: float) -> List[Tuple[str, float, str]]:
    """S waveforms of the sector whose 5th-harmonic (or other order) band covers part of the Ku channel f_ku +/- B/2
    (step 3.2 A3): (name, covered share of the channel, harmonic bands)."""
    lo, hi = f_ku - bw_ku / 2e3, f_ku + bw_ku / 2e3
    out = []
    for w in r.get("wf_set", [r["wf"]]):
        occ = w["sig_bw_mhz"] + (w.get("agility_mhz") or 0.0)
        bands = iso.harmonic_bands_ghz(w["freq_ghz"], occ)
        cov = sum(max(0.0, min(b, hi) - max(a, lo)) for a, b in bands) / (hi - lo)
        if cov > 0.0:
            out.append((w["name"], min(cov, 1.0), ", ".join(f"{a:.3f}-{b:.3f}" for a, b in bands)))
    return out


# ----------------------------------------------------------------------------- active tracking (step 3.3)
def _start_bracket(brackets: List[dict], r_det: float) -> Optional[int]:
    """Index of the bracket where tracks start (C6): the one covering the confirmation range, else the farthest one
    when the target is confirmed beyond every bracket (designated by the S search at its entry); None otherwise."""
    if not brackets:
        return None
    for i, b in enumerate(brackets):
        if b["r_min_km"] <= r_det <= b["r_max_km"]:
            return i
    far = max(range(len(brackets)), key=lambda i: brackets[i]["r_max_km"])
    return far if r_det > brackets[far]["r_max_km"] else None


def _rounded_pack(st: "tm.TrackState") -> rm.TrackingPack:
    """The accuracy pack of a track state, on 3 significant digits (shares the look spaces between nearby states)."""
    pk = st.pack()
    r3 = lambda x: None if x is None else float(f"{x:.3g}")             # noqa: E731
    return rm.TrackingPack(r3(pk.sigma_theta_deg), r3(pk.sigma_vr_mps), r3(pk.sigma_r_m))


def _track_update(gr: dict, tgt: rm.Target, st: "tm.TrackState", r_km: float, ctx, pointing_hpbw=None,
                  bw_max_mhz: Optional[float] = None) -> Optional[dict]:
    """Cheapest update of a track (B1-B4): one look pointed at the prediction (Gaussian pointing law), or the 7-beam
    sequential cluster (cell law of the cluster, 7 looks), with the velocity and acceleration banks of the predicted
    errors (B3, E1) and Pd tracking.pd_update; with its loss probability per update (B6-B7, single looks; a cluster
    keeps Pd at every coasted update). None when neither reaches r_km."""
    p_avg_max_w, sll, t_ku, r_min_sec, f_ku, spj, excl, clutter = ctx
    tgt_t = tm.track_target(tgt, st)
    v_span = round(st.v_span_mps, 1)
    sig = st.pointing_hpbw if pointing_hpbw is None else pointing_hpbw
    pt1 = ("gauss", float(f"{sig:.3g}"))
    clut = clutter(gr, r_km) if clutter is not None else None
    kw = dict(rx_sll_db=sll, t_sys_k=t_ku, r_min_km=r_min_sec, v_span_mps=v_span, f_ghz=f_ku, spj=spj, clutter=clut,
              excl=excl, pd=st.pd_update, bw_max_mhz=bw_max_mhz)

    def cheapest(s):
        pack = _rounded_pack(s)
        single = ku_look(gr["theta"], tgt_t, r_km, p_avg_max_w, pack, pointing=pt1, **kw)
        cluster = ku_look(gr["theta"], tgt_t, r_km, p_avg_max_w, pack, pointing=("cluster", rm.BEAM_SPACING_HPBW), **kw)
        opts = []
        if single is not None:
            opts.append((single["cost"], "single", single, 1.0))
        if cluster is not None:
            opts.append((cluster["cost"] * CLUSTER_N, "cluster", cluster, float(CLUSTER_N)))
        return min(opts, key=lambda o: o[0])[1:] if opts else None

    best = cheapest(st)
    if best is None:
        return None
    mode, look, mult = best
    # return on 3.3 from step 5.4: the accuracies at a fluctuating SNR (kappa = E[SNR_ref / SNR | detection]), a fixed
    # point between the look and the noise parts it must reach
    # kappa rounded UP on a grid of KAPPA_STEP_DB (prudent by at most that step): the groups of a state then share
    # their look space (one per accuracy pack) and the fixed point stops sooner (computing time, user 28 Oct 2026)
    st_k, tl = st, None
    look0 = look
    for _ in range(KAPPA_ITER):
        kap, tl = _accuracy_kappa({"look": look, "mode": mode}, st, tgt, f_ku, sll, sig)
        kap = 10.0 ** (math.ceil(10.0 * math.log10(kap) / KAPPA_STEP_DB - 1e-9) * KAPPA_STEP_DB / 10.0)
        if kap <= st_k.kappa * (1.0 + 1e-9):
            break
        st_k, tl = st.with_kappa(kap), None
        if KAPPA_MODE == "energy":
            # decision 4 (user, 28 October 2026): the same waveform, its energy x kappa (no new look space): the SNR
            # rises by kappa, the noise share to reach falls by kappa (prudent when the detection, not the accuracy,
            # sized the look)
            sp_, j_ = look0["space"], look0["index"]
            e, t, q = _look_scale(sp_, j_, look0["scale"] * kap)
            look = dict(look0, energy_J=e, energy_rf_J=e - q, pae=(e - q) / e if e else 0.0, time_s=t,
                        cost=max(e / p_avg_max_w, t), scale=look0["scale"] * kap, snr_gain_db=10.0 * math.log10(kap))
            continue
        best = cheapest(st_k)
        if best is None:
            return None
        mode, look, mult = best
    if pointing_hpbw is None:
        # return on 3.3 from step 5.4: the misses of a track are correlated (a large filter error misses several
        # looks in a row): the chain's Pd conditioned on the misses so far, drawn jointly (montecarlo.miss_chain_pds)
        from . import montecarlo as _mc
        if tl is None:
            tl = _mc.track_look({"look": look, "mode": mode}, st, {"tgt": tgt, "f_ku": f_ku, "sll": sll}, sig)
        pds = _mc.miss_chain_pds(tl, st, st.nav_rad, tm.N_MISS_LOST, tgt.t0_s(f_ku))
    else:
        pds = [st.pd_update] * tm.N_MISS_LOST
    p_loss, _ = tm.loss_per_update(pds)
    return {"look": look, "mode": mode, "mult": mult, "p_loss": p_loss, "pds": pds, "state": st_k}


def _accuracy_kappa(upd: dict, st: "tm.TrackState", tgt: rm.Target, f_ku: float, sll: float,
                    pointing_hpbw: float) -> tuple:
    """(kappa, TrackLook) of a chosen look at its pointing (montecarlo.accuracy_kappa, the same look parts as the
    draws)."""
    from . import montecarlo as _mc
    tl = _mc.track_look(upd, st, {"tgt": tgt, "f_ku": f_ku, "sll": sll}, pointing_hpbw)
    return _mc.accuracy_kappa(tl, pointing_hpbw), tl


def _coast_pd(look: dict, sigma_hpbw: float, tgt_t: rm.Target, f_ku: float, sll: float, v_span: float,
              spj: Optional[float], pd_update: float = tm.PD_UPDATE) -> float:
    """Pd of the chosen look when it points with the error sigma_hpbw of a coasted track (B6): the detection law of
    the look with that pointing, at the SNR the look delivers (its design SNR, plus the margin when the accuracy and
    not the detection sized it)."""
    sp, j = look["space"], look["index"]
    d = sp.det[j]
    p0 = sp.det_parts[j]
    margin = 40.0 * math.log10(p0[0] / sp.rn[j]) if sp.rn[j] > 0 and p0[0] > sp.rn[j] else 0.0
    pk = mo.parts_at(d, f_ku, 0.0, tgt_t, sll, v_span, None, None, spj, None, ("gauss", float(f"{sigma_hpbw:.3g}")),
                     pd_update)
    if pk is None:
        return 0.0
    law = np.array([pk[4]])
    return float(mo._pd_at(mo._curves(law), law, np.array([p0[2] + margin + look.get("snr_gain_db", 0.0)]))[0])


def init_hop_freqs(r: dict, f_ku: float, bw_mhz: float, length_m: Optional[float], excl) -> int:
    """Independent Ku frequencies of the initiation looks (H1, return on 3.3 from step 5.4): centres of the Ku grid
    spaced at least c / 2L (rounded up to the grid step), the look inside the CORE band (no edge loss), outside the
    exclusions; the chosen f_ku is one of them. 1 when the target length is unknown. The S harmonic is a TIMING rule
    (step 3.2: the Ku looks of the face avoid the S slots), not a frequency one: it does not remove hop frequencies
    (user, 27 October 2026, after the trial)."""
    if not length_m or length_m <= 0:
        return 1
    step = KU_FREQ_STEP_GHZ
    dfc = math.ceil(rm.C_MPS / (2.0 * length_m) / 1e9 / step - 1e-9) * step
    half = bw_mhz / 2e3

    def ok(f: float) -> bool:
        return (f - half >= BAND.core_lo_ghz - 1e-9 and f + half <= BAND.core_hi_ghz + 1e-9
                and mo.signal_fits(f, bw_mhz, excl, BAND))
    cands = [f for f in ku_centre_grid() if ok(f)]
    if not cands:
        return 1
    chosen = [f_ku] if ok(f_ku) else []
    for f in sorted(cands, key=lambda x: abs(x - f_ku)):
        if all(abs(f - c) >= dfc - 1e-9 for c in chosen):
            chosen.append(f)
    return max(1, len(chosen))


def _miss_given_miss(look: dict, sigma_hpbw: float, tgt_t: rm.Target, f_ku: float, sll: float, v_span: float,
                     spj: Optional[float], pd_update: float) -> float:
    """Probability that a look misses given that a look on the same frequency within T0 missed (H2): the target power
    is the same (Gamma of the look's shape), the position losses independent: E[(1 - p(g))^2] / E[1 - p(g)]."""
    sp, j = look["space"], look["index"]
    d = sp.det[j]
    p0 = sp.det_parts[j]
    margin = 40.0 * math.log10(p0[0] / sp.rn[j]) if sp.rn[j] > 0 and p0[0] > sp.rn[j] else 0.0
    pk = mo.parts_at(d, f_ku, 0.0, tgt_t, sll, v_span, None, None, spj, None, ("gauss", float(f"{sigma_hpbw:.3g}")),
                     pd_update)
    if pk is None:
        return 1.0
    n, fl, pfa, pl, pw = mo._LAW_LIST[int(pk[4])]
    k = detection.shape_of(fl, n)
    steady = detection.mean_curve(n, detection.Fluct(math.inf), pfa, pl, pw)
    if not math.isfinite(k) or k > 1e4:
        return 1.0 - float(np.interp(p0[2] + margin, detection.SNR_GRID_DB, steady))
    from scipy.stats import gamma
    g = gamma.ppf((np.arange(64) + 0.5) / 64, k, scale=1.0 / k)
    p = np.interp(p0[2] + margin + 10.0 * np.log10(np.maximum(g, 1e-12)), detection.SNR_GRID_DB, steady)
    q = float(np.mean(1.0 - p))
    return float(np.mean((1.0 - p) ** 2)) / q if q > 0 else 0.0


def _init_state(st: "tm.TrackState") -> "tm.TrackState":
    """The track state an initiation look is sized on (lever B): Pd tracking.pd_init instead of the update's; the
    cheapest look of the space reaching it at the initiation pointing is then, for a fluctuating target, a
    frequency-agile noncoherent train (lever A, no forced pulse count)."""
    s = copy.copy(st)
    s.pd_update = max(float(tm.PD_INIT), float(st.pd_update))
    return s


def _research_look(gr: dict, tgt: rm.Target, st: "tm.TrackState", r_km: float, ctx) -> Optional[dict]:
    """The look of one beam of a local re-search (lever D): the Ku look of the space reaching r_km with Pd
    st.pd_update (pd_init) when the target is anywhere in a cell of the hexagonal lattice of the cluster."""
    p_avg_max_w, sll, t_ku, r_min_sec, f_ku, spj, excl, clutter = ctx
    tgt_t = tm.track_target(tgt, st)
    clut = clutter(gr, r_km) if clutter is not None else None
    kw = dict(rx_sll_db=sll, t_sys_k=t_ku, r_min_km=r_min_sec, v_span_mps=round(st.v_span_mps, 1), f_ghz=f_ku,
              spj=spj, clutter=clut, excl=excl, pd=st.pd_update)
    return ku_look(gr["theta"], tgt_t, r_km, p_avg_max_w, _rounded_pack(st), pointing=("cluster", rm.BEAM_SPACING_HPBW),
                   **kw)


def research_state(st: "tm.TrackState") -> "tm.TrackState":
    """The state a re-search beam is sized on (lever D): Pd tracking.pd_research."""
    s = _init_state(st)
    s.pd_update = max(float(tm.PD_RESEARCH), float(s.pd_update))
    return s


def init_look(gr: dict, tgt: rm.Target, st: "tm.TrackState", r_km: float, ctx, r: dict) -> Tuple[Optional[dict], int]:
    """(initiation look, independent Ku frequencies) of one bracket state (levers A, B, D): the cheapest look reaching
    pd_init at the initiation pointing; when it leaves fewer hop frequencies than the looks in a row that can fail
    (1 + init_research_attempts), the cheapest narrower look that gives more (more independent target draws)."""
    excl = ctx[6]
    st_i = _init_state(st)
    pt = max(tm.INIT_POINTING_MAX_HPBW, st.pointing_hpbw)
    upd = _track_update(gr, tgt, st_i, r_km, ctx, pointing_hpbw=pt)
    if upd is None:
        return None, 1
    n_f = init_hop_freqs(r, ctx[4], upd["look"]["wf"].occupied_mhz, tgt.length_m, excl)
    if n_f < 1 + tm.RESEARCH_ATTEMPTS:
        for bw in sorted((b for b in KU_BW if b < upd["look"]["wf"].sig_bw_mhz), reverse=True):
            u2 = _track_update(gr, tgt, st_i, r_km, ctx, pointing_hpbw=pt, bw_max_mhz=bw)
            if u2 is None:
                continue
            nf2 = init_hop_freqs(r, ctx[4], u2["look"]["wf"].occupied_mhz, tgt.length_m, excl)
            if nf2 > n_f:
                upd, n_f = u2, nf2
            if n_f >= 1 + tm.RESEARCH_ATTEMPTS:
                break
    return upd, n_f


def _fast_initiation_cost(gr: dict, tgt: rm.Target, st: "tm.TrackState", r_km: float, hand_theta: float, zone: float,
                          racc: Optional[float], vacc: float, ctx, r: dict, nav_deg: Optional[float]) -> Optional[dict]:
    """(expected looks, electrical J, Ku TX s, heat J, time to the established track, loss probability of one attempt,
    independent frequencies, mode) of one track initiation AS FAST AS POSSIBLE (H1-H11, return on 3.3 from step 5.4):
    the intervals of trackmodel.fast_initiation, the look sized for its pointing (INIT_POINTING_MAX_HPBW), Ku frequency
    hopping from look to look (H1-H2), a wait of T0 after a miss when only one frequency is free (H3), the misses
    degrading the pointing (H4) and the chain of trackmodel.init_chain (H5)."""
    p_avg_max_w, sll, t_ku, r_min_sec, f_ku, spj, excl, clutter = ctx
    nav_rad = math.radians(repoint.axis_sigma_deg() if nav_deg is None else float(nav_deg))
    dt_min = 2.0 * r_km * 1e3 / rm.C_MPS                     # H6: the first look leaves on the round trip
    seq = tm.fast_initiation(st, tgt, hand_theta, st.allowed["r"] if math.isfinite(st.allowed["r"]) else 1e3,
                             st.allowed["vr"], zone, racc, vacc, dt_min, nav_rad)
    st_i = _init_state(st)
    upd, n_f = init_look(gr, tgt, st, r_km, ctx, r)
    if upd is None:
        return None
    look = upd["look"]
    tgt_t = tm.track_target(tgt, st)
    v_span = round(st.v_span_mps, 1)
    t0 = tgt.t0_s(f_ku)
    t0_ok = t0 if math.isfinite(t0) else math.inf
    mode = "hop" if n_f >= 2 else "wait T0"
    retry = dt_min if n_f >= 2 else max(dt_min, t0 if math.isfinite(t0) else dt_min)
    # lever D: after a miss, a local re-search (cluster covering RESEARCH_SIGMAS of the predicted error), lost after
    # RESEARCH_ATTEMPTS re-searches in a row without a detection (D1-D5)
    n_fail = 1 + tm.RESEARCH_ATTEMPTS
    sig = tm.coast_from(st, seq["first_pred"], [retry] * (n_fail - 1), nav_rad)
    pds = [_coast_pd(look, max(sig[0], 1e-6), tgt_t, f_ku, sll, v_span, spj, st_i.pd_update)]
    st_r = research_state(st)
    rs = _research_look(gr, tgt, st_r, r_km, ctx)
    weights, beams_max, cov_min = [1.0], 1, 1.0
    for k in range(1, n_fail):
        if rs is None:
            pds.append(0.0)
            weights.append(1.0)
            continue
        nb, cov = tm.research_cluster(sig[k], rm.BEAM_SPACING_HPBW)
        p_beam = st_r.pd_update
        if n_f == 2 and k >= 2 and retry * 2 < t0_ok:   # H2: back on a frequency that missed within T0
            p_beam = 1.0 - _miss_given_miss(rs, max(sig[k], 1e-6), tgt_t, f_ku, sll, v_span, spj, st_r.pd_update)
        pds.append(cov * p_beam)
        weights.append(float(nb + (1 if k >= 2 else 0)))   # a shifted re-search keeps a beam on the estimate (K11)
        beams_max, cov_min = max(beams_max, nb), min(cov_min, cov)
    ch = tm.init_chain(seq["intervals"], pds, retry, weights)
    e, t, q = look_cost(look, 1.0)
    e_r, t_r, q_r = look_cost(rs, 1.0) if rs is not None else (0.0, 0.0, 0.0)
    v0 = ch["visits"][0] * upd["mult"]                       # pointed looks per established track
    vr = sum(v * w for v, w in zip(ch["visits"][1:], weights[1:]))   # re-search beams per established track
    return {"looks": ch["looks"], "energy_J": v0 * e + vr * e_r, "time_s": v0 * t + vr * t_r,
            "heat_J": v0 * q + vr * q_r, "time_to_track_s": ch["time_s"], "loss_prob": ch["loss_prob"],
            "freqs": n_f, "mode": mode, "looks_if_detected": len(seq["intervals"]),
            "research_beams_max": beams_max, "research_coverage_min": cov_min}


def _longest_period(tgt: rm.Target, gr: Optional[dict], ranges: List[float], t_s: float, zone: float,
                    racc: Optional[float], vacc: float, ctx, st_kw: Optional[dict] = None) -> Optional[float]:
    """Longest update period, below the operator's, at which every failing range of the bracket holds its
    requirements with a reachable look (A8, D2); None if even a very short period does not."""
    if gr is None:
        return None
    f_ku = ctx[4]

    def ok(t):
        for rk in ranges:
            st = tm.TrackState(tgt, t, rk, zone, racc, vacc, f_ku, 0.0, **(st_kw or {}))
            if not st.feasible or _track_update(gr, tgt, st, rk, ctx, pointing_hpbw=st.pointing_hpbw) is None:
                return False
        return True

    lo, hi = t_s / 1024.0, t_s
    if not ok(lo):
        return None
    for _ in range(tm.RR_ALERT_ITER):
        mid = math.sqrt(lo * hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def sector_budget(sec: dict, r: dict, P: np.ndarray, p_avg_max_w: float, ku_sky: Optional[dict] = None,
                  f_ku: float = KU_REF_GHZ, clutter: Optional[_SectorClutter] = None) -> dict:
    """Confirmation (all modes) and AT loads per face of one sector at the Ku frequency f_ku, per second, as fractions
    of the face resources: "e" = energy (shared S + Ku), "ts" = S TX time, "tk" = Ku TX time. ku_sky: the sector's Ku
    sky summary at f_ku (optimizer.sector_sky): every Ku look uses its budget T_sys; clutter: the sector's surfaces."""
    tgt = mo.sector_target(sec)                      # class + sector values (step 2.4)
    f_s = r["wf"]["freq_ghz"]
    fl_ku = rm.look_fluctuation(tgt, f_ku, tgt.swerling, 1, 0.0)    # one short Ku look (step 3.2 E4)
    fl_s = rm.look_fluctuation(tgt, f_s, tgt.swerling, 1, 0.0)      # the S echo
    r_det, rr = sec["range_km"], sec["rr_target_s"]
    sll = mo.rx_sll_db(sec)
    groups = _groups(P, r)
    p_face = P.sum(axis=1)
    p_all = float(p_face.sum())
    ku_lim = BAND.scan_limit_deg(f_ku)
    flags: List[str] = []
    t_ku = ku_sky["t_sys_budget_k"] if ku_sky is not None else None
    if ku_sky is not None:
        flags.extend(mo.sky_flags(ku_sky, "Ku"))
    spj = env.spj_density_w_hz(sec, f_ku)                       # step 3.2 E1
    excl = exclusions_ku(sec)
    rcs_sat = tgt.physical().rcs_at(f_ku)             # saturation: the class as it is, not its sizing RCS
    r_sat_ku = rm.min_unsaturated_range_km(f_ku, 0.0, rcs_sat)
    r_sat_att = rm.min_unsaturated_range_attenuated_km(f_ku, 0.0, rcs_sat)     # step 3.3, E3
    r_min_sec = mo.range_min_km(sec)          # shortest range of interest: Ku blind range limit (step 2.1)
    spacing = r.get("beam_spacing_hpbw")                 # the sector's search lattice (step 3.1, P5)
    angle_cells = rm.rx_beams_per_position(f_s, sll, spacing) * rm.RX_BEAM_SPACING_HPBW ** 2   # independent cells per position
    for b in sec.get("tracking", []):
        if b["r_min_km"] < r_min_sec:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km starts below the sector's shortest range of "
                         f"interest ({r_min_sec:.0f} km, range_min_km): Ku looks only need a blind range below that")
        if b["r_min_km"] < r_sat_att:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: below {r_sat_att:.2f} km the Ku echo of "
                         f"{rcs_sat:g} m2 clips the per-element ADCs even with the "
                         f"{rm.ATTENUATOR_MAX_DB['Ku']:g} dB attenuator (step 3.3, E3)")

    # --- confirmation: per scan-angle group, true detections and false alarms per face (per refresh), with the Ku
    #     look at the sector's detection range (S touch = the search look pointed, i.e. without grid loss)
    n_new = float(sec.get("new_targets_per_rr", 1.0)) + float(sec.get("objects_per_rr", 0.0))
    fa = np.zeros(4)
    cg = []
    ku_missing = 0.0
    v_span = mo.velocity_span_mps(sec)
    for gr in groups:
        w = gr["wf"]
        cells = rm.independent_cells(r_min_sec, r_det, w["sig_bw_mhz"], w["pulses_per_block"],
                                     w["velocity_hypotheses"] * w.get("acceleration_hypotheses", 1),
                                     angle_cells)          # independent cells per position (step 2.2 E1-E5, 2.4 C2)
        fa_g = gr["pos"] * cells * w.get("pfa", rm.DETECT_PFA)    # Pfa per cell of the waveform (step 2.3)
        fa += fa_g
        look = (ku_look(gr["theta"], tgt, r_det, p_avg_max_w, rx_sll_db=sll, t_sys_k=t_ku, r_min_km=r_min_sec,
                        v_span_mps=v_span, f_ghz=f_ku, spj=spj,
                        clutter=clutter(gr, r_det) if clutter is not None else None, excl=excl)
                if gr["theta"] <= ku_lim else None)
        if look is None:
            ku_missing += float(gr["pos"].sum())
        e_s = gr["elec_J_per_position"]           # search look of the group: shortened pulse, its PAE (step 3.1, C)
        cg.append({"true": n_new * gr["pos"] / p_all, "fa": fa_g, "e_s_rf": e_s * gr["pae_at_scale"], "e_s": e_s,
                   "ku": look, "touch": _s_touch_wf(w)})
    if ku_missing > 0:
        flags.append(f"{100 * ku_missing / p_all:.1f} % of the positions are out of Ku reach (scan limit, range, "
                     f"jammer or clutter): no Ku confirmation or track there")
    th_ku = max((gr["theta"] for gr in groups if gr["theta"] <= ku_lim), default=0.0)
    held = rm.rx_sll_held_up_to_deg(f_ku, sll, th_ku)
    if held < th_ku:
        flags.append(f"Ku RX sidelobes {sll:g} dB held only up to {held:g} deg scan at {f_ku:g} GHz (worst "
                     f"visible sidelobe {rm.sidelobe_visible_db(f_ku, th_ku, 'rx', sll):.1f} dB at {th_ku:g} deg: "
                     f"TRSA gap lobes)")

    def ku_cost(looks_true: float, looks_fa: float, m: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Electrical energy, Ku TX time and heat (electrical - RF energy) per face, per refresh; a margin m (linear)
        multiplies the energy each look must deliver, a false alarm's look needs FA_R4_MEAN of it."""
        e, t, q = np.zeros(4), np.zeros(4), np.zeros(4)
        for c in cg:
            lk = c["ku"]
            if lk is None:
                continue
            e1, t1, q1 = look_cost(lk, m)
            e2, t2, q2 = look_cost(lk, m * FA_R4_MEAN)
            nt, nf = c["true"] * looks_true, c["fa"] * looks_fa
            e += nt * e1 + nf * e2
            t += nt * t1 + nf * t2
            q += nt * q1 + nf * q2
        return e, t, q

    def s_touch_cost(m: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        n_c = [(c["true"] + c["fa"] * FA_R4_MEAN) * m for c in cg]
        e = sum((n * c["e_s"] for n, c in zip(n_c, cg)), np.zeros(4))
        e_rf = sum((n * c["e_s_rf"] for n, c in zip(n_c, cg)), np.zeros(4))
        return e, e_rf / rm.BAND_S.p_pk_total_w, e - e_rf                         # one beam at full peak power

    def load_of(e, ts, tk) -> float:
        return max(float((e / p_avg_max_w).max()), float(ts.max()), float(tk.max()))

    modes: Dict[str, dict] = {}
    p_min = float(sec.get("confirm_p_min", CONFIRM_P_MIN))          # step 5.1: sector, else mission, else K5
    # K2: the SNR of the first detection of the sector's cumulative search criterion
    q_cum = float(sec["v_close_max_mps"]) * float(sec["rr_target_s"]) / (float(sec["range_km"]) * 1e3)
    x_first = first_detection_snr_quantiles(fl_s, float(sec.get("pd", rm.DETECT_PD)), round(q_cum, 9))
    for mode in CONFIRM_MODES:
        why = ["part of the sector is out of Ku reach"] if ku_missing > 0 else []
        best = None
        # K9 (a): the fewest attempts (confirm_attempts .. confirm_attempts_max) with which the mode reaches p_min
        for att, cl in [(a, c) for a in range(CONFIRM_ATTEMPTS, max(CONFIRM_ATTEMPTS, CONFIRM_ATTEMPTS_MAX) + 1)
                        for c in cluster_variants()]:
            if best is not None and att > best[1]["attempts"]:
                break
            st = ConfirmStats(fl_s, fl_ku, f_s, f_ku, spacing, cl, p_min, x_first, att)
            if mode == "S2":
                mts = np.arange(0.0, MARGIN_MAX_DB + TOUCH_STEP_DB / 2.0, TOUCH_STEP_DB)
                mks = st.s2_min_margins(mts)
                ok = np.isfinite(mks)
                if not ok.any():
                    continue
                succs, lookss = st.s2_vec(mts[ok], mks[ok])
                for mt, mk, succ, looks in zip(mts[ok], mks[ok], succs, lookss):
                    succ, looks = float(succ), float(looks)
                    e_s, ts, q_s = s_touch_cost(rm.db_to_lin(mt))
                    e_k, tk, q_k = ku_cost(looks, 0.0, rm.db_to_lin(mk))
                    e, q = e_s + e_k, q_s + q_k
                    load = load_of(e, ts, tk)
                    if best is None or load < best[0]:
                        best = (load, {"margin_db": [round(float(mt), 3), round(float(mk), 3)], "success": succ,
                                       "ku_looks_true": looks, "ku_looks_fa": 0.0, "cluster_tx": cl.name,
                                       "attempts": st.attempts, "round_trips": 3,
                                       "s_touch_wf": "pointed search waveform (role CONFIRM)"},
                                e, ts, tk, q)
            else:
                mk = st.min_margin(lambda x: st.ku(mode, x)[0])
                if mk is None:
                    continue
                succ, looks = st.ku(mode, mk)
                # a false alarm fails every look of every attempt (K4: the attempts multiply its cost)
                l_fa = (cl.looks + (1.0 if mode == "KU1+7" else 0.0)) * st.attempts
                e, tk, q = ku_cost(looks, l_fa, rm.db_to_lin(mk))
                ts = np.zeros(4)
                load = load_of(e, ts, tk)
                if best is None or load < best[0]:
                    # K6: the 95 % latency counts the second attempt when more than 5 % of the first ones fail
                    base = 2 if mode == "KU1+7" else 1
                    s1 = ConfirmStats(fl_s, fl_ku, f_s, f_ku, spacing, cl, p_min, x_first, 1).ku(mode, mk)[0]
                    trips = base * (2 if (st.attempts > 1 and 1.0 - s1 > 0.05) else 1)
                    best = (load, {"margin_db": round(float(mk), 3), "success": succ, "ku_looks_true": looks,
                                   "ku_looks_fa": l_fa, "cluster_tx": cl.name, "attempts": st.attempts,
                                   "success_first_attempt": s1, "round_trips": trips}, e, ts, tk, q)
        if best is None:
            why.append(f"confirmation probability < {p_min:g} even with {MARGIN_MAX_DB:g} dB margin")
            modes[mode] = {"admissible": False, "why_not": "; ".join(why)}
            continue
        _, info, e, ts, tk, q = best
        modes[mode] = {**info, "admissible": not why, "why_not": "; ".join(why),
                       "e": e / p_avg_max_w / rr, "ts": ts / rr, "tk": tk / rr, "q_w": q / rr}

    # --- active tracking per bracket (step 3.3: track model of core/trackmodel.py)
    ku_obs = [(c["ku"]["energy_J"], c["ku"]["time_s"]) for c in cg if c["ku"] is not None]   # step 4.1 (D4)
    at = []
    n_tr_all = sum(float(b["n_targets"]) for b in sec.get("tracking", []))
    start = _start_bracket(sec.get("tracking", []), r_det)
    main_gr = max(groups, key=lambda g: float(g["pos"].sum())) if groups else None
    fl_conf = rm.look_fluctuation(tgt, f_ku, tgt.swerling, 1, 0.0)
    conf_snr = _snr_ref(fl_conf)
    hand_theta = K_ML_KU * math.radians(rm.hpbw_deg(f_ku)) / math.sqrt(conf_snr)    # Ku confirmation accuracy (C1)
    deceptive = bool(sec.get("jammer_deceptive")) and sec.get("jammer_type") is not None
    tm_ctx = (p_avg_max_w, sll, t_ku, r_min_sec, f_ku, spj, excl, clutter)
    for bi, b in enumerate(sec.get("tracking", [])):
        T = float(b["rr_s"])
        zone, racc, vacc = float(b["angle_zone_deg"]), b.get("range_acc_m"), float(b["vr_acc_mps"])
        r_lo = max(float(b["r_min_km"]), r_min_sec, 1e-3)
        r_hi = max(float(b["r_max_km"]), r_lo)
        ranges = [r_lo + (i + 0.5) * (r_hi - r_lo) / tm.N_RANGES for i in range(tm.N_RANGES)]        # D1
        # step 5.1: the bracket's own Pd of an update, the mission's navigation pointing error
        st_kw = {"pd_update": b.get("pd_update"), "nav_axis_deg": sec.get("_nav_axis_sigma_deg")}
        states = [tm.TrackState(tgt, T, rk, zone, racc, vacc, f_ku, 0.0, **st_kw) for rk in ranges]
        e_face, t_face, q_face = np.zeros(4), np.zeros(4), np.zeros(4)
        drivers: Dict[str, float] = {}
        wf_main, pos_main, unreachable = None, -1.0, 0.0
        n_cluster, n_upd, p_loss, infeasible = 0.0, 0.0, [], []
        mc_states = []                                  # step 5.4 (M3): (range, state, update) of the main group
        tracks_face = np.zeros(4)                       # step 4.3: tracks per face
        for gr in groups:
            share = gr["pos"] / p_all * b["n_targets"]                       # tracks per face in this group
            tracks_face += share
            if gr["theta"] > ku_lim:
                unreachable += float(share.sum())
                continue
            ok_w = 0.0
            for rk, st in zip(ranges, states):
                upd = _track_update(gr, tgt, st, rk, tm_ctx) if st.feasible else None
                if upd is None:
                    infeasible.append(rk)
                    continue
                ok_w += 1.0 / len(ranges)
                look, mult = upd["look"], upd["mult"] * (1.0 + (tm.DECEPTION_VERIFY_RATE if deceptive else 0.0))
                e, tk, q = look_cost(look, 1.0)
                ku_obs.append((e * upd["mult"], tk * upd["mult"]))
                w = share / len(ranges)
                e_face += w * e * mult
                t_face += w * tk * mult
                q_face += w * q * mult
                drivers[look["driver"]] = drivers.get(look["driver"], 0.0) + float(w.sum())
                n_upd += float(w.sum())
                n_cluster += float(w.sum()) if upd["mode"] == "cluster" else 0.0
                if gr is main_gr:
                    p_loss.append(upd["p_loss"])
                    if KEEP_MC_OBJECTS:
                        mc_states.append((rk, upd.get("state", st), upd))   # the state at its kappa
                if gr["pos"].sum() > pos_main:
                    wf_main, pos_main = look["wf"], float(gr["pos"].sum())
            unreachable += float(share.sum()) * (1.0 - ok_w)
        n_tr = float(b["n_targets"])
        pl = float(np.mean(p_loss)) if p_loss else 0.0
        loss_rate = n_tr * pl / T                                          # tracks lost per second (B7)
        # initiation (C) in the bracket where tracks start, and re-initiation of the lost tracks (B8), at the bracket's
        # start range (start bracket) or mid-range; each also costs a confirmation look of the main group
        r_init = min(r_det, r_hi) if bi == start else 0.5 * (r_lo + r_hi)
        st0 = tm.TrackState(tgt, T, r_init, zone, racc, vacc, f_ku, 0.0, **st_kw)
        new_rate = (n_new / rr) if bi == start else 0.0
        # return on 3.3 from step 5.4 (H1-H11): initiation as fast as possible, frequency hopping
        init = _fast_initiation_cost(main_gr, tgt, st0, r_init, hand_theta, zone, racc, vacc, tm_ctx, r,
                                     st_kw.get("nav_axis_deg")) if (main_gr is not None and st0.feasible) else None
        if init is not None:
            from . import scheduler as _sch
            lat_req = _sch.latency_requirement_s(sec)
            if init["time_to_track_s"] > lat_req:
                flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: track established "
                             f"{init['time_to_track_s']:.3g} s after the confirmation, above the {lat_req:.3g} s "
                             f"latency required (fast initiation, {init['mode']})")
            if init["loss_prob"] > INIT_LOSS_MAX:
                flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: {100 * init['loss_prob']:.2f} % of the "
                             f"initiations lost (above p_loss_max; {init['freqs']} Ku frequencies, {init['mode']})")
        conf_e = 0.0
        if cg and main_gr is not None:
            c0 = cg[groups.index(main_gr)]["ku"]
            conf_e = look_cost(c0, 1.0)[0] if c0 is not None else 0.0
        init_rate = new_rate + loss_rate
        if init is not None and n_tr_all > 0:
            fshare = p_face / max(p_all, 1e-12)
            e_face_s = fshare * init_rate * (init["energy_J"] + conf_e)
            t_init = fshare * init_rate * init["time_s"]
            q_init = fshare * init_rate * init["heat_J"]
        else:
            e_face_s = t_init = q_init = np.zeros(4)
        # A8: the operator's period, or the longest one that holds every requirement at the worst point (D2)
        rr_alert = None
        if infeasible:
            rr_alert = _longest_period(tgt, main_gr, sorted(set(infeasible)), T, zone, racc, vacc, tm_ctx, st_kw)
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: update period {T:g} s not tenable "
                         + (f"(longest tenable {rr_alert:.3g} s)" if rr_alert else "(no period tenable)")
                         + ": accept it, or relax the accuracies, the number of tracks or the range")
        mid = states[len(states) // 2]
        out_b = {**b, "sigma_theta_deg": math.degrees(mid.noise_theta),
                 "driver": max(drivers, key=drivers.get) if drivers else "-",
                 "energy_per_update_J": float(e_face.sum() / n_tr) if n_tr > 0 else 0.0,
                 "avg_power_kW": float((e_face.sum() / T + e_face_s.sum()) / 1e3),
                 "wf": _wf_label(wf_main, f_ku) if wf_main is not None else "-",
                 "wf_params": _wf_params(wf_main, sec, f_ku) if wf_main is not None else None,   # step 4.4
                 "e": (e_face / T + e_face_s) / p_avg_max_w, "tk": t_face / T + t_init,
                 "q_w": q_face / T + q_init, "unreachable_tracks": unreachable,
                 # step 3.3 outputs
                 "rr_tenable": not infeasible, "rr_alert_s": rr_alert,
                 "predicted_angle_sigma_mrad": 1e3 * math.sqrt(mid.p_cross[0, 0]) / mid.r_m,
                 "filtered_range_sigma_m": math.sqrt(mid.pf_range[0, 0]),
                 "filtered_vr_sigma_mps": math.sqrt(mid.pf_range[1, 1]),
                 "pointing_sigma_hpbw": mid.pointing_hpbw, "velocity_bank_mps": mid.v_span_mps,
                 "accel_bank_mps2": mid.a_bank_mps2,
                 "cluster_update_share": n_cluster / n_upd if n_upd > 0 else 0.0,
                 "p_loss_per_update": pl, "track_losses_per_s": loss_rate,
                 "initiations_per_s": init_rate, "init_looks": init["looks"] if init else None,
                 "init_time_s": init["time_to_track_s"] if init else None,
                 "init_loss_prob": init["loss_prob"] if init else None,
                 "init_freqs": init["freqs"] if init else None, "init_mode": init["mode"] if init else None,
                 "init_research_beams": init["research_beams_max"] if init else None,
                 "init_research_coverage": init["research_coverage_min"] if init else None,
                 "init_energy_J": init["energy_J"] if init else None,
                 "attenuation_db": rm.attenuation_needed_db(f_ku, 0.0, tgt.physical().rcs_at(f_ku), r_lo),
                 "deception_verification": deceptive, "tracks_face": tracks_face.tolist(),
                 "init_tk": (t_init if isinstance(t_init, np.ndarray) else np.zeros(4)).tolist()}
        if KEEP_MC_OBJECTS:                             # step 5.4 (M3): Python objects, not JSON
            out_b["_mc"] = {"states": mc_states, "tgt": tgt, "sll": sll, "f_ku": f_ku, "spj": spj,
                            "theta": main_gr["theta"] if main_gr is not None else 0.0, "st_kw": st_kw,
                            "hand_theta_rad": hand_theta, "gr": main_gr, "ctx": tm_ctx, "r": r,
                            "init_freqs": init["freqs"] if init else 1, "init_mode": init["mode"] if init else None}
        if unreachable > 0:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: {unreachable:.1f} track(s) out of Ku reach "
                         f"(scan limit, range, jammer, clutter or accuracy)")
        if deceptive:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: deceptive self-protection jammer: risk of "
                         f"track capture, {tm.DECEPTION_VERIFY_RATE:g} verification look(s) per update counted (F7)")
        at.append(out_b)

    # --- discrimination of the sector's class from the class it must be told from (F1-F5), on the track updates
    disc = None
    other = sec.get("discriminate_against") or ("DECOY" if tgt.target_type == "KKV" else None)
    if other is not None and other in rm.TARGETS and sec.get("tracking") and start is not None:
        b = sec["tracking"][start]
        T = float(b["rr_s"])
        st_d = tm.TrackState(tgt, T, min(r_det, float(b["r_max_km"])), float(b["angle_zone_deg"]),
                             b.get("range_acc_m"), float(b["vr_acc_mps"]), f_ku, 0.0, pd_update=b.get("pd_update"),
                             nav_axis_deg=sec.get("_nav_axis_sigma_deg"))
        p_disc = float(b.get("p_discrimination", tm.P_DISCRIMINATION))       # step 5.1: bracket key
        far = (math.radians(env.faraday_round_trip_deg(sec["tec_tecu"], sec["b_parallel_nt"], f_ku))
               if sec.get("tec_tecu") is not None and sec.get("b_parallel_nt") is not None else 0.0)
        disc = tm.discrimination(tgt, rm.TARGETS[other], f_ku, rm.db_to_lin(rm.lin_to_db(conf_snr)), far,
                                 math.sqrt(st_d.pf_cross[2, 2]), p_disc)
        disc["against"] = other
        r0 = min(r_det, float(b["r_max_km"]))
        if disc["looks"] is not None:
            disc["time_s"] = disc["looks"] * T
            disc["range_km"] = r0 - tgt.v_close_mps * disc["time_s"] / 1e3
            if disc["range_km"] < r_min_sec:
                flags.append(f"{tgt.target_type} / {other} discrimination at P {p_disc:g} reached at "
                             f"{disc['range_km']:.0f} km, below the shortest range of interest {r_min_sec:.0f} km")
        else:
            disc["time_s"] = disc["range_km"] = None
            flags.append(f"{tgt.target_type} / {other} discrimination does not reach P {p_disc:g} within "
                         f"{tm.DISC_N_MAX} track looks (P {disc['p_at_max_looks']:.3f})")

    # --- false tracks (C5): false alarms passing the Ku confirmation gate
    false_tracks = float(fa.sum()) / rr * rm.DETECT_PFA * tm.GATE_CELLS
    if false_tracks > tm.FALSE_TRACK_FLAG_PER_S:
        flags.append(f"{false_tracks:.2g} false tracks per second (false alarms through the Ku confirmation gate)")

    # --- S harmonic on the Ku channel of the same face (step 3.2 A3): flagged, a time rule for the scheduler
    rules = []
    bw_ku = max([c["ku"]["wf"].sig_bw_mhz for c in cg if c["ku"] is not None] or [min(KU_BW)])
    if iso.same_face_s_to_ku_verdict() == "frequency_rule":
        for name, cov, bands in _harmonic_cover(r, f_ku, bw_ku):
            flags.append(f"S waveform {name}: its harmonics ({bands} GHz) cover {100 * cov:.0f} % of the Ku channel "
                         f"{f_ku:g} GHz / {bw_ku:g} MHz on the same face")
            rules.append(f"{name} puts its harmonics on {bands} GHz, over {100 * cov:.0f} % of the Ku channel "
                         f"{f_ku:g} GHz / {bw_ku:g} MHz: on the same face, no Ku reception during this S transmission")

    # --- Ku burn-through search in the jamming axis (step 3.2 E2), for a sector the jammer puts out of search reach
    bt, bt_load = None, None
    if spj is not None and (r.get("sky") or {}).get("spj_out_of_reach"):
        th = max((gr["theta"] for gr in groups if gr["theta"] <= ku_lim), default=None)
        if th is not None:
            bt = ku_burn_through(th, tgt, r_det, p_avg_max_w, sll, t_ku, f_ku, spj, excl)
            allowed = bool(sec.get("_burn_through_allowed"))
            flags.append(f"Ku burn-through look in the jamming axis: {bt['range_km']:.0f} km "
                         f"({'reaches' if bt['reaches_requirement'] else 'does not reach'} the requirement, "
                         f"{bt['wf']}); " + ("budgeted, one look per revisit at P3' (burn-through allowed)" if allowed
                                             and bt["reaches_requirement"] else
                                             "not budgeted: burn-through forbidden by the mission" if not allowed else
                                             "not budgeted: no Ku look reaches the requirement"))
            # step 5.1 (C): allowed by the mission -> one look per revisit, shared by the sector's faces as its positions
            if allowed and bt["reaches_requirement"]:
                share = p_face / max(p_all, 1e-12)
                bt_load = {"e": share * bt["energy_J"] / rr / p_avg_max_w, "ts": np.zeros(4),
                           "tk": share * bt["time_s"] / rr, "q_w": share * bt["heat_J"] / rr}
                bt["budgeted"] = True
    return {"faces": [k for k in range(4) if p_face[k] > 0], "false_alarms_per_rr": fa, "modes": modes, "at": at,
            "p_share": (p_face / max(p_all, 1e-12)).tolist(),      # step 4.3: share of the sector per face
            "flags": flags, "ku_sky": ku_sky, "ku_min_unsaturated_range_km": r_sat_ku,
            "ku_min_unsaturated_range_attenuated_km": r_sat_att, "discrimination": disc,
            "false_tracks_per_s": false_tracks, "scheduler_rules": rules,
            "ku_obs_max": max(ku_obs, default=(0.0, 0.0)),
            "ku_freq_ghz": f_ku, "ku_burn_through": bt, "bt_load": bt_load,
            # step 4.4: the longest Ku confirmation look of the sector (its processing load)
            "ku_wf": next((_wf_params(lk["wf"], sec, f_ku) for lk in sorted(
                (c["ku"] for c in cg if c["ku"] is not None), key=lambda x: -x["time_s"])), None)}


# ----------------------------------------------------------------------------- mission budget
_RES = ("e", "ts", "tk")
_RES_NAME = {"e": "energy", "ts": "S time on air", "tk": "Ku time on air", "heat": "heat"}


def _face_loads(base: Dict[str, np.ndarray], sbs: Dict[str, Dict[float, dict]], choice: Dict[str, tuple]) -> Dict[str, np.ndarray]:
    """Face loads with each sector at its (Ku frequency, confirmation mode or None)."""
    tot = {k: v.copy() for k, v in base.items()}
    for sid, (f, mode) in choice.items():
        sb = sbs[sid][f]
        if mode is not None:
            for key in _RES:
                tot[key] += sb["modes"][mode][key]
        for b in sb["at"]:
            tot["e"] += b["e"]
            tot["tk"] += b["tk"]
        if sb.get("bt_load"):                          # step 5.1 (C): burn-through allowed by the mission
            tot["e"] += sb["bt_load"]["e"]
            tot["tk"] += sb["bt_load"]["tk"]
    return tot


def _max_load(tot: Dict[str, np.ndarray]) -> float:
    return float(max(tot[k].max() for k in _RES))


def _own_load(sb: dict, mode: Optional[str]) -> float:
    """A sector's own peak face load at one frequency: its confirmation mode plus its tracks."""
    tot = {k: np.zeros(4) for k in _RES}
    if mode is not None:
        for k in _RES:
            tot[k] += sb["modes"][mode][k]
    for b in sb["at"]:
        tot["e"] += b["e"]
        tot["tk"] += b["tk"]
    if sb.get("bt_load"):
        tot["e"] += sb["bt_load"]["e"]
        tot["tk"] += sb["bt_load"]["tk"]
    return _max_load(tot)


def _ku_freq_task(sec: dict, r: dict, Pm: np.ndarray, p_avg_max: float, eps_min: float, eps_max: float,
                  sky: Optional[dict], f: float) -> dict:
    """sector_budget of one sector at one Ku frequency, with its Ku sky and surfaces (worker entry point of budget)."""
    sph = mo._sphere(eps_min, eps_max)
    m = sph.mask(sec)
    ku_sky, keep = mo.sector_sky(sec, sph, m, f, "Ku", sky)
    clut = _SectorClutter(sec, sph, m, sky, f, keep) if env.surfaces(sky) else None
    return sector_budget(sec, r, Pm, p_avg_max, ku_sky, f, clut)


def _storage_checks(k: int, res: dict, sbs: Dict[str, dict], chosen: Dict[str, dict], recharge_w: float,
                    tx_w: float, t_frac: float) -> dict:
    """Face storage and transients of face k (step 4.1, D-E): the costliest observation (an S search dwell of the
    plan or a Ku look of the budget) minus the recharge during it must fit in the usable storage, its peak power in the
    storage discharge plus the feeder; the worst burst packs the transmissions of the longest revisit of the face
    together (deficit = RR (P_tx - recharge x time share)); transient = the peak power of an observation of each band
    and its slope on the pulse edge (outputs only)."""
    usable = rm.STORE_DOD * rm.FACE_STORE_J
    worst_obs, rr_max = 0.0, 0.0
    for r in res["sectors"]:
        if r.get("status") != "OK" or str(k) not in r.get("faces", {}):
            continue
        rr_max = max(rr_max, float(r["rr_target_s"]))
        for b in r.get("wf_by_bin", []):
            e = b["elec_J_per_position"] * b["wf"]["positions_per_dwell"]
            worst_obs = max(worst_obs, e - recharge_w * b["wf"]["dwell_ms"] / 1e3)
        sb = chosen.get(r["id"])
        if sb is not None and k in sb["faces"]:
            e, t = sb["ku_obs_max"]
            worst_obs = max(worst_obs, e - recharge_w * t)
    peak = {b: rm.BANDS[b].p_pk_total_w / rm.BANDS[b].pae_sat for b in ("S", "Ku")}
    burst = rr_max * (tx_w - recharge_w * t_frac)
    # longest contiguous transmission at the face's peak power the storage can carry (scheduler rule, step 4.3)
    over = max(peak.values()) - recharge_w
    t_cont = usable / over if over > 0 else math.inf
    return {"max_contiguous_tx_s": t_cont,"store_obs_deficit_MJ": max(0.0, worst_obs) / 1e6, "store_burst_deficit_MJ": max(0.0, burst) / 1e6,
            "store_usable_MJ": usable / 1e6, "store_obs_ok": worst_obs <= usable, "store_burst_ok": burst <= usable,
            "peak_MW": {b: v / 1e6 for b, v in peak.items()},
            "peak_ok": max(peak.values()) <= rm.FACE_STORE_MAX_W + recharge_w,
            "transient_MW_per_us": {b: peak[b] / (rm.BANDS[b].pae_t_edge_s * 1e6) / 1e6 for b in peak}}


def _thermal_checks(tx_heat_w: float, t_frac: float, k: int, res: dict) -> dict:
    """Face loop heat and burst (step 4.2, A and C): transmissions' heat + always-on electronics + pumps; worst burst =
    the heat of the longest revisit of the face released during its time on air, the cooling removing its capacity
    minus the steady load (deficit against the cold-plate buffer); longest contiguous transmission at the peak heat
    of the face (all TRMs of the costlier band at saturated PAE)."""
    cool = rm.face_cooling()
    on = rm.non_tx_power_w() + rm.PUMP_FRAC * tx_heat_w
    face_heat = tx_heat_w + on
    rr_max = max((float(r["rr_target_s"]) for r in res["sectors"]
                  if r.get("status") == "OK" and str(k) in r.get("faces", {})), default=0.0)
    spare = cool["capacity_w"] - on
    burst = rr_max * (tx_heat_w - spare * t_frac)
    peak = max(rm.BANDS[b].p_pk_total_w * (1.0 / rm.BANDS[b].pae_sat - 1.0) for b in ("S", "Ku"))
    t_cont = cool["buffer_j"] / (peak - spare) if peak > spare else math.inf
    return {"face_heat_W": face_heat, "burst_deficit_J": max(0.0, burst), "burst_ok": burst <= cool["buffer_j"],
            "t_cont_s": t_cont}


def budget(mission: dict, res: dict) -> dict:
    """Mission confirmation + AT budget on top of an optimizer.run() result: per sector the Ku frequency and the
    confirmation mode (with the reason) and the AT cost per bracket, per face the search / confirmation / AT shares and
    the verdict, and what to relax when a face does not fit."""
    g = res["global"]
    p_avg_max = g["face_elec_avail_kW"] * 1e3
    sph = mo._sphere(g["eps_min_deg"], g["eps_max_deg"])
    secs = {s["id"]: _with_mission_settings(s, mission.get("global", {})) for s in mission["sectors"]}
    base = {"e": np.zeros(4), "ts": np.zeros(4), "tk": np.zeros(4)}
    for k, v in res["faces_total"].items():
        base["e"][int(k)] = v["energy_fraction"]
        base["ts"][int(k)] = v["time_fraction"]
    # step 3.4 (E1): every face spends cal_duration / cal_interval of its S and Ku transmit time on calibration
    cal_s, cal_ku = rm.array_errors("S")["cal_time_frac"], rm.array_errors("Ku")["cal_time_frac"]
    base["ts"] += cal_s
    base["tk"] += cal_ku
    sky = mission.get("global", {}).get("sky")
    sbs: Dict[str, Dict[float, dict]] = {}
    n_cands: Dict[str, int] = {}
    jobs = []                                         # (sector, its result, its S positions per face, Ku frequency)
    for r in res["sectors"]:
        if r["status"] != "OK":
            continue
        sec = secs[r["id"]]
        m = sph.mask(sec)
        sec_s = dict(sec, _grid_spacing=r.get("beam_spacing_hpbw", rm.BEAM_SPACING_HPBW))   # its lattice (P5)
        P, _, _, _ = mo.search_positions(sec_s, sph, m, r["wf"]["freq_ghz"], sky)    # blinded positions left out
        Pm = np.array([P[k] for k in range(4)])
        fs = ku_centres(sec, sky) or [KU_REF_GHZ]
        n_cands[r["id"]] = len(fs)
        sbs[r["id"]] = {}
        jobs.extend((sec, r, Pm, f) for f in fs)
    # every (sector, Ku frequency) budget in parallel processes when allowed (step 3.2, Q6 E4: identical results)
    args = [[j[k] for j in jobs] for k in range(4)]
    n = len(jobs)
    eps = (g["eps_min_deg"], g["eps_max_deg"])
    if mo._freq_parallel_ok(n):
        out = list(mo._executor().map(_ku_freq_task, *args[:3], [p_avg_max] * n, [eps[0]] * n, [eps[1]] * n,
                                      [sky] * n, args[3]))
    else:
        out = [_ku_freq_task(*j[:3], p_avg_max, eps[0], eps[1], sky, j[3]) for j in jobs]
    for (sec, _, _, f), sb in zip(jobs, out):
        sbs[sec["id"]][f] = sb

    # (Ku frequency, confirmation mode): forced mode, or automatic (greedy on the mission's most loaded face, two
    # passes); ties: own load, then the frequency nearest the core-band centre (step 3.2 A4)
    choice: Dict[str, tuple] = {}
    forced: Dict[str, bool] = {}

    def options(sid: str) -> List[tuple]:
        want = secs[sid].get("confirm_mode", "auto")
        out = []
        for f, sb in sbs[sid].items():
            if want in CONFIRM_MODES:
                if "e" in sb["modes"][want]:
                    out.append((f, want))
            else:
                out.extend((f, md_) for md_ in CONFIRM_MODES if sb["modes"][md_]["admissible"])
        return out

    for sid in sbs:
        want = secs[sid].get("confirm_mode", "auto")
        forced[sid] = want in CONFIRM_MODES
        opts = options(sid)
        if not opts and forced[sid]:                       # forced mode impossible: fall back to an admissible one
            opts = [(f, md_) for f, sb in sbs[sid].items() for md_ in CONFIRM_MODES if sb["modes"][md_]["admissible"]]
        if opts:
            choice[sid] = min(opts, key=lambda o: (_own_load(sbs[sid][o[0]], o[1]), _core_distance(o[0]), o[0]))
        else:
            f0 = min(sbs[sid], key=lambda f: (_own_load(sbs[sid][f], None), _core_distance(f), f))
            choice[sid] = (f0, None)
    for _ in range(2):
        for sid in sbs:
            opts = options(sid)
            if forced[sid] or not opts:
                continue
            best = None
            for o in opts:
                key = (_max_load(_face_loads(base, sbs, {**choice, sid: o})), _own_load(sbs[sid][o[0]], o[1]),
                       _core_distance(o[0]), o[0])
                if best is None or key < best[0]:
                    best = (key, o)
            choice[sid] = best[1]
    tot = _face_loads(base, sbs, choice)

    # per-sector report
    out_secs, warn = [], []
    for sid, per_f in sbs.items():
        f, mode = choice[sid]
        sb = per_f[f]
        alts = {}
        for md_ in CONFIRM_MODES:
            m_ = sb["modes"][md_]
            alts[md_] = {"admissible": m_["admissible"], "why_not": m_["why_not"]}
            if "e" in m_:
                alts[md_]["mission_max_load"] = round(_max_load(_face_loads(base, sbs, {**choice, sid: (f, md_)})), 4)
                alts[md_]["faces"] = {str(k): {"energy_fraction": float(m_["e"][k]),             # step 5.2 (P7)
                                               "s_time_fraction": float(m_["ts"][k]),
                                               "ku_time_fraction": float(m_["tk"][k])} for k in sb["faces"]}
        want = secs[sid].get("confirm_mode")
        if mode is None:
            reason = "no admissible confirmation mode"
            warn.append(f"sector {sid}: no admissible confirmation mode at any Ku frequency "
                        f"({'; '.join(sb['modes'][m]['why_not'] for m in CONFIRM_MODES)})")
        elif forced[sid] and mode == want:
            reason = "forced by the operator"
            if not sb["modes"][mode]["admissible"]:
                warn.append(f"sector {sid}: forced confirmation mode {mode} breaks a constraint ({sb['modes'][mode]['why_not']})")
        elif forced[sid]:
            reason = f"forced mode {want} impossible ({sb['modes'][want]['why_not']})"
        else:
            others = [m for m in CONFIRM_MODES if m != mode and sb["modes"][m]["admissible"]]
            refused = [f"{m}: {sb['modes'][m]['why_not']}" for m in CONFIRM_MODES if not sb["modes"][m]["admissible"]]
            if not others:
                reason = "only admissible mode" + (f" ({'; '.join(refused)})" if refused else "")
            else:
                binding = max(_RES, key=lambda k: tot[k].max())
                reason = f"lowest mission face load (binding resource: {_RES_NAME[binding]})"
        conf = {}
        if mode is not None:
            md_ = sb["modes"][mode]
            conf = {k: v for k, v in md_.items() if k not in _RES + ("q_w",)}
            conf.update({"faces": {str(k): {"energy_fraction": float(md_["e"][k]), "s_time_fraction": float(md_["ts"][k]),
                                            "ku_time_fraction": float(md_["tk"][k])} for k in sb["faces"]}})
        out_secs.append({
            "id": sid, "ku_freq_ghz": f, "ku_freq_candidates": n_cands[sid], "confirm_mode": mode, "reason": reason,
            "forced": forced[sid], "false_alarms_per_rr": round(float(sb["false_alarms_per_rr"].sum()), 1),
            "confirmation": conf, "alternatives": alts,
            "at": [{**{k: v for k, v in b.items() if k not in ("e", "tk", "q_w")},
                    "faces": {str(k): {"energy_fraction": float(b["e"][k]), "ku_time_fraction": float(b["tk"][k])}
                              for k in sb["faces"]}} for b in sb["at"]],
            "ku_sky": sb["ku_sky"], "ku_min_unsaturated_range_km": sb["ku_min_unsaturated_range_km"],
            "ku_burn_through": (None if sb["ku_burn_through"] is None else {
                **sb["ku_burn_through"],
                **({"faces": {str(k): {"energy_fraction": float(sb["bt_load"]["e"][k]),
                                       "ku_time_fraction": float(sb["bt_load"]["tk"][k])}
                              for k in range(4) if sb["bt_load"]["tk"][k] > 0}} if sb.get("bt_load") else {})}),
            "scheduler_rules": sb["scheduler_rules"],
            "discrimination": sb.get("discrimination"), "false_tracks_per_s": sb.get("false_tracks_per_s"),
            "ku_min_unsaturated_range_attenuated_km": sb.get("ku_min_unsaturated_range_attenuated_km"),
            "false_alarms_per_rr_face": [float(x) for x in sb["false_alarms_per_rr"]], "p_share": sb["p_share"],
            "ku_confirm_wf": sb.get("ku_wf"), "confirm_p_min": float(secs[sid].get("confirm_p_min", CONFIRM_P_MIN)),
            "flags": sb["flags"]})
        for fl in sb["flags"]:
            warn.append(f"sector {sid}: {fl}")

    # per-face totals, verdict and what to relax
    chosen = {sid: sbs[sid][choice[sid][0]] for sid in sbs}
    mode_of = {sid: choice[sid][1] for sid in sbs}
    faces, advice = {}, []
    draws: List[float] = []
    heats: List[float] = []
    cool = rm.face_cooling()                           # step 4.2
    cap_in = g.get("face_elec_cap_mw")                 # step 4.1: optional operator cap per face
    caps = ([float(cap_in)] * 4 if isinstance(cap_in, (int, float)) else
            [float(c) for c in cap_in] if cap_in else [math.inf] * 4)
    for k in range(4):
        conf_e = sum(chosen[s]["modes"][mode_of[s]]["e"][k] for s in sbs if mode_of[s] is not None)
        at_e = sum(b["e"][k] for s in sbs for b in chosen[s]["at"])
        conf_ts = sum(chosen[s]["modes"][mode_of[s]]["ts"][k] for s in sbs if mode_of[s] is not None)
        conf_tk = sum(chosen[s]["modes"][mode_of[s]]["tk"][k] for s in sbs if mode_of[s] is not None)
        at_tk = sum(b["tk"][k] for s in sbs for b in chosen[s]["at"])
        bt_e = sum(chosen[s]["bt_load"]["e"][k] for s in sbs if chosen[s].get("bt_load"))       # step 5.1 (C)
        bt_tk = sum(chosen[s]["bt_load"]["tk"][k] for s in sbs if chosen[s].get("bt_load"))
        heat = float(res["faces_total"][str(k)].get("dissipation_kW", 0.0) * 1e3
                     + sum(chosen[s]["modes"][mode_of[s]]["q_w"][k] for s in sbs if mode_of[s] is not None)
                     + sum(b["q_w"][k] for s in sbs for b in chosen[s]["at"])
                     + sum(chosen[s]["bt_load"]["q_w"][k] for s in sbs if chosen[s].get("bt_load")))
        load = {key: float(tot[key][k]) for key in _RES}
        # step 4.1 (A): the face draws from the ship total in common, up to its feeder and the operator's cap
        tx_w = load["e"] * p_avg_max
        draw = rm.face_bus_draw_w(tx_w, heat)
        cap_w = min(rm.FACE_FEEDER_MAX_MW, caps[k]) * 1e6
        draws.append(draw)
        load["e"] = draw / cap_w
        recharge = max(0.0, cap_w * rm.CHAIN_EFF - rm.non_tx_power_w() - rm.PUMP_FRAC * heat)   # storage recharge
        store = _storage_checks(k, res, sbs, chosen, recharge, tx_w, load["ts"] + load["tk"])
        # step 4.2: heat of the face loop (transmissions, always-on electronics, pumps) against its cooling
        th = _thermal_checks(heat, load["ts"] + load["tk"], k, res)
        heats.append(th["face_heat_W"])
        load["heat"] = th["face_heat_W"] / cool["capacity_w"]
        worst = max(load, key=load.get)
        faces[str(k)] = {"search_energy": float(base["e"][k]), "confirm_energy": float(conf_e), "at_energy": float(at_e),
                         "total_energy": load["e"], "search_s_time": float(base["ts"][k] - cal_s),
                         "confirm_s_time": float(conf_ts), "total_s_time": load["ts"],
                         "confirm_ku_time": float(conf_tk), "at_ku_time": float(at_tk), "total_ku_time": load["tk"],
                         "burn_through_energy": float(bt_e), "burn_through_ku_time": float(bt_tk),
                         "calibration_s_time": cal_s, "calibration_ku_time": cal_ku,
                         "load": load[worst], "binding": _RES_NAME[worst],
                         "status": "OK" if load[worst] <= 1.0 else "OVER-SUBSCRIBED",
                         "p_left_kW": max(0.0, cap_w - draw) / 1e3,
                         # step 4.1: power from the ship bus, always-on consumption, storage, transients
                         "bus_draw_MW": draw / 1e6, "cap_MW": cap_w / 1e6, "non_tx_kW": rm.non_tx_power_w() / 1e3,
                         "pumps_kW": rm.PUMP_FRAC * heat / 1e3, **store,
                         "face_heat_kW": th["face_heat_W"] / 1e3, "cooling_capacity_kW": cool["capacity_w"] / 1e3,
                         "heat_load": load["heat"], "heat_flux_W_cm2": th["face_heat_W"] / rm.PHYS_AREA_M2 / 1e4,
                         "coolant_flow_L_s": th["face_heat_W"] / (rm.TH_CP * rm.TH_RISE) / rm.TH_RHO,
                         "heat_burst_deficit_MJ": th["burst_deficit_J"] / 1e6, "heat_burst_ok": th["burst_ok"],
                         "max_contiguous_tx_thermal_s": th["t_cont_s"],
                         "dissipation_kW": heat / 1e3, "dissipation_kW_m2": heat / 1e3 / rm.TOTAL_AREA_M2}
        if load[worst] > 1.0:
            advice.append(f"face F{k + 1}: heat {th['face_heat_W'] / 1e6:.2f} MW above the cooling capacity "
                          f"{cool['capacity_w'] / 1e6:.2f} MW (step 4.2)" if worst == "heat"
                          else _relax_advice(k, worst, load[worst], res, chosen, mode_of, secs))
    # step 4.1 (A): the ship total shared by the faces
    ship_w = SHIP_ELEC_W
    tot_draw = float(sum(draws))
    ship = {"draw_MW": tot_draw / 1e6, "budget_MW": ship_w / 1e6, "load": tot_draw / ship_w,
            "status": "OK" if tot_draw <= ship_w else "OVER-SUBSCRIBED",
            "suggested_allocation_MW": [round(d / tot_draw * ship_w / 1e6, 3) if tot_draw > 0 else ship_w / 4e6
                                        for d in draws]}
    if tot_draw > ship_w:
        advice.append(f"ship power: the faces draw {tot_draw / 1e6:.2f} MW, above the {ship_w / 1e6:g} MW ship budget")
    # step 4.2 (D): heat of the four face loops + conversion losses against the ship's rejection for the radar
    ship_heat = float(sum(heats)) + tot_draw * (1.0 - rm.CHAIN_EFF)
    ship.update({"heat_MW": ship_heat / 1e6, "heat_rejection_MW": rm.SHIP_HEAT_REJECTION_W / 1e6,
                 "heat_load": ship_heat / rm.SHIP_HEAT_REJECTION_W,
                 "heat_status": "OK" if ship_heat <= rm.SHIP_HEAT_REJECTION_W else "OVER-SUBSCRIBED"})
    if ship_heat > rm.SHIP_HEAT_REJECTION_W:
        advice.append(f"ship heat: the radar rejects {ship_heat / 1e6:.2f} MW, above the "
                      f"{rm.SHIP_HEAT_REJECTION_W / 1e6:g} MW reserved")
    for k in range(4):
        f_ = faces[str(k)]
        for key, msg in (("store_obs_ok", "an observation draws more than the face storage can give"),
                         ("store_burst_ok", f"the transmissions of a revisit, packed together, would exhaust the "
                                            f"face storage: the scheduler must spread them (at most "
                                            f"{f_['max_contiguous_tx_s']:.2f} s of contiguous transmission at peak power)"),
                         ("peak_ok", "the peak power of an observation exceeds the storage discharge plus the feeder"),
                         ("heat_burst_ok", f"the heat of a revisit, packed together, would exceed the cold-plate buffer: "
                                           f"at most {f_['max_contiguous_tx_thermal_s']:.2f} s of contiguous "
                                           f"transmission at peak heat (step 4.2)")):
            if not f_[key]:
                warn.append(f"face F{k + 1}: {msg} (step 4.1)")
    for a in advice:
        warn.append(a)
    rules = ([f"search lattice shifted every frame (grid_dithering, step 2.3): the cumulative detection criterion "
              f"draws the target position in its cell anew at each revisit"] if rm.GRID_DITHERING else [])
    rules += iso.scheduler_rules() + [f"sector {s['id']}: {t}" for s in out_secs for t in s["scheduler_rules"]]
    rules += [f"face F{k + 1}: at most {faces[str(k)]['max_contiguous_tx_thermal_s']:.2f} s of contiguous transmission at "
              f"peak heat, then cooling (cold-plate buffer {rm.face_cooling()['buffer_j'] / 1e6:.2f} MJ, step 4.2)"
              for k in range(4) if math.isfinite(faces[str(k)]["max_contiguous_tx_thermal_s"])]
    rules += [f"face F{k + 1}: at most {faces[str(k)]['max_contiguous_tx_s']:.2f} s of contiguous transmission at peak "
              f"power, then recharge (face storage {rm.STORE_DOD * rm.FACE_STORE_J / 1e6:g} MJ usable, step 4.1)"
              for k in range(4) if math.isfinite(faces[str(k)]["max_contiguous_tx_s"])]
    warn.extend(rm.calibration_checks())                    # step 3.4 (B5-B6)
    return {"pfa": rm.DETECT_PFA, "confirm_p_min": CONFIRM_P_MIN,
            "sectors": out_secs, "faces": faces, "advice": advice, "scheduler_rules": rules, "warnings": warn,
            # step 3.3 (F3): P(correct) between the library classes after a few looks at the design SNR (indicator)
            "separability": tm.separability(KU_REF_GHZ, _snr_ref(1)),
            # step 3.4: residual array errors (gain loss, sidelobe floor, boresight, calibration share of the time)
            "array_errors": {b: rm.array_errors(b) for b in ("S", "Ku")},
            "ship_power": ship}                          # step 4.1


def _relax_advice(k: int, key: str, load: float, res: dict, sbs: Dict[str, dict], choice: Dict[str, Optional[str]],
                  secs: Dict[str, dict]) -> str:
    """Which component (search / confirmation / AT) and sub-component to relax on an over-subscribed face."""
    items: List[Tuple[float, str, str]] = []
    if key in ("e", "ts"):
        for r in res["sectors"]:
            fc = r.get("faces", {}).get(str(k))
            if fc:
                v = fc["energy_fraction" if key == "e" else "time_fraction"]
                items.append((v, f"search {r['id']}", f"range {secs[r['id']]['range_km']:.0f} km or RR "
                                                      f"{secs[r['id']]['rr_target_s']:.0f} s"))
    for sid, sb in sbs.items():
        m = choice[sid]
        if m is not None and sb["modes"][m][key][k] > 0:
            items.append((float(sb["modes"][m][key][k]), f"confirmation {sid} ({m})",
                          f"{sb['false_alarms_per_rr'].sum():.0f} false alarms / RR: coarser range cell (res_max_m) "
                          f"or another mode"))
        if key in ("e", "tk"):
            for b in sb["at"]:
                if b[key][k] > 0:
                    acc = {"ANG": f"angle zone {b['angle_zone_deg']:g} deg", "DOP": f"velocity accuracy {b['vr_acc_mps']:g} m/s",
                           "RNG": f"range accuracy {b['range_acc_m']:g} m", "DET": "nothing on accuracy (detection-limited)"}
                    items.append((float(b[key][k]), f"AT {sid} {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km",
                                  f"{b['n_targets']:g} targets / {b['rr_s']:g} s, {acc.get(b['driver'], b['driver'])}"))
    items.sort(reverse=True)
    excess = load - 1.0
    parts = []
    for v, name, hint in items[:3]:
        cut = f", cut by {100 * excess / v:.0f} % to fit alone" if v >= excess else ""
        parts.append(f"{name} {100 * v:.0f} % ({hint}{cut})")
    return (f"face F{k + 1} over-subscribed x{load:.2f} on {_RES_NAME[key]}: largest items " + "; ".join(parts))
