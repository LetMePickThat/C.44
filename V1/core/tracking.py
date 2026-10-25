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
K_ML_KU = rm.angle_k_tx("Ku")                        # Ku angle accuracy x sqrt(SNR), in Ku TX beamwidths (3.3)
K_ML_S = rm.angle_k_tx("S")                          # S angle accuracy x sqrt(SNR), in S TX beamwidths (CRLB x margin)
BIAS_MRAD = P.get("confirmation.bias_mrad")          # residual S/Ku alignment bias per axis (absolute angle, constant in u, v)
CONFIRM_P_MIN = P.get("confirmation.p_min")          # minimum probability of confirming a true target
CONFIRM_MODES = ("KU1+7", "KU7", "S2")
CLUSTER_N = 7                   # hexagonal cluster: centre + first ring, spacing rm.BEAM_SPACING_HPBW
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
def cluster_variants() -> Tuple[ClusterTx, ...]:
    """The ways of sending the cluster that tool.confirmation.ku_cluster_tx offers."""
    hexs = tuple((float(x), float(y), 1.0, 1.0) for x, y in _CLUSTER_XY)
    out = [ClusterTx("sequential", hexs, 1.0, float(CLUSTER_N))]
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


def _ku_after_s(sigma_ku, bias_ku: float, snr_ku, fluct, cl: ClusterTx) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """KU1+7 and KU7 statistics for angle errors of standard deviation sigma_ku per axis (Ku beamwidths, array over
    the leading axes) plus a bias per axis. snr_ku: centre-look SNR (array over margins, last axis).
    Returns (success KU1+7, Ku look energies KU1+7, success KU7), each averaged over the Gaussian errors."""
    ex = bias_ku + sigma_ku[..., None, None] * GH_NODES[:, None]            # (..., 12, 1)
    ey = bias_ku + sigma_ku[..., None, None] * GH_NODES[None, :]            # (..., 1, 12)
    w = GH_WEIGHTS[:, None] * GH_WEIGHTS[None, :]
    p1 = pd_single(_tx_gain(ex, ey)[..., None] * snr_ku, fluct)            # (..., 12, 12, M)
    p7 = _cluster_pd(ex, ey, snr_ku, fluct, cl)
    s17 = p1 + (1.0 - p1) * p7
    l17 = 1.0 + (1.0 - p1) * cl.looks
    avg = lambda a: np.tensordot(w, a, axes=([0, 1], [-3, -2]))           # noqa: E731
    return avg(s17), avg(l17), avg(p7)


class ConfirmStats:
    """Confirmation statistics of the three modes as functions of the power margins, for one sector: the S echo law
    (shape at the S frequency), the Ku look law (at the Ku frequency), the S / Ku beamwidth ratio, the search lattice
    and the way the cluster is sent. Independent of range and scan angle (S and Ku beams widen together on the same
    face, and the detection always happens at the design SNR)."""

    def __init__(self, fl_s, fl_ku, f_s_ghz: float, f_ku_ghz: float, spacing_hpbw: Optional[float],
                 cl: ClusterTx):
        self.fl_s, self.fl_ku, self.cl = fl_s, fl_ku, cl
        self.q = rm.hpbw_deg(f_s_ghz) / rm.hpbw_deg(f_ku_ghz)                 # S TX beamwidth in Ku TX beamwidths
        self.snr_s = _snr_ref(fl_s)
        self.x_det = detected_snr_quantiles(self.snr_s, fl_s)                  # (M,)
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
                                                              self.fl_ku, self.cl))
            self._memo[("KU1+7", round(m_db, 6))] = (float(s17[0]), float(l17[0]))
            self._memo[("KU7", round(m_db, 6))] = (float(s7[0]), float(self.cl.looks))
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
        s17 = np.tensordot(p1 + (1.0 - p1) * p7, w, axes=([2, 3], [0, 1]))                    # (M, n)
        l17 = np.tensordot(1.0 + (1.0 - p1) * self.cl.looks, w, axes=([2, 3], [0, 1]))
        return (pt * s17).mean(axis=0), (pt * l17).mean(axis=0)

    def s2(self, m_touch_db: float, m_ku_db: float) -> Tuple[float, float]:
        """s2_vec for one pair of margins."""
        a, b = self.s2_vec(np.array([m_touch_db]), np.array([m_ku_db]))
        return float(a[0]), float(b[0])

    def s2_min_margins(self, mt_db: np.ndarray) -> np.ndarray:
        """Smallest Ku margin (dB, within MARGIN_TOL_DB) reaching CONFIRM_P_MIN after a touch at each mt_db, by a
        bisection run for every touch margin at once; NaN where even MARGIN_MAX_DB does not."""
        n = len(mt_db)
        top = self.s2_vec(mt_db, np.full(n, MARGIN_MAX_DB))[0] >= CONFIRM_P_MIN
        zero = self.s2_vec(mt_db, np.zeros(n))[0] >= CONFIRM_P_MIN
        lo, hi = np.zeros(n), np.full(n, MARGIN_MAX_DB)
        while (hi - lo).max() > MARGIN_TOL_DB:
            mid = 0.5 * (lo + hi)
            ok = self.s2_vec(mt_db, mid)[0] >= CONFIRM_P_MIN
            lo, hi = np.where(ok, lo, mid), np.where(ok, mid, hi)
        return np.where(~top, np.nan, np.where(zero, 0.0, hi))

    def min_margin(self, f) -> Optional[float]:
        """Smallest margin (dB, within MARGIN_TOL_DB) at which f(margin) = success reaches CONFIRM_P_MIN; None if even
        MARGIN_MAX_DB does not (success increases with the margin)."""
        if f(MARGIN_MAX_DB) < CONFIRM_P_MIN:
            return None
        if f(0.0) >= CONFIRM_P_MIN:
            return 0.0
        lo, hi = 0.0, MARGIN_MAX_DB
        while hi - lo > MARGIN_TOL_DB:
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if f(mid) < CONFIRM_P_MIN else (lo, mid)
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
            pointing: Optional[Tuple[str, float]] = None, pd: Optional[float] = None) -> Optional[dict]:
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
                "energy_J": look["energy_J"], "time_s": look["time_s"]}
    sll = rm.RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    sp = _space(f_ghz, dataclasses.replace(tgt, name="ku"), sll, None, spj, None, tuple(excl), False)
    hb = _ku_scan(f_ghz, float(theta))
    if not sp.det or hb is None:
        return {"reaches_requirement": False, "range_km": 0.0, "wf": "-"}
    sky_f = (BAND.t_sys_k / t_sys_k) ** 0.25 if t_sys_k else 1.0
    rr = rm.jammed_range_km(sp.rn * hb[0] * sky_f, sp.rbt * hb[1])
    j = int(np.nanargmax(rr))
    return {"reaches_requirement": False, "range_km": round(float(rr[j]), 1), "wf": _wf_label(sp.det[j], f_ghz)}


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


def _track_update(gr: dict, tgt: rm.Target, st: "tm.TrackState", r_km: float, ctx, pointing_hpbw=None) -> Optional[dict]:
    """Cheapest update of a track (B1-B4): one look pointed at the prediction (Gaussian pointing law), or the 7-beam
    sequential cluster (cell law of the cluster, 7 looks), with the velocity and acceleration banks of the predicted
    errors (B3, E1) and Pd tracking.pd_update; with its loss probability per update (B6-B7, single looks; a cluster
    keeps Pd at every coasted update). None when neither reaches r_km."""
    p_avg_max_w, sll, t_ku, r_min_sec, f_ku, spj, excl, clutter = ctx
    tgt_t = tm.track_target(tgt, st)
    pack = _rounded_pack(st)
    v_span = round(st.v_span_mps, 1)
    sig = st.pointing_hpbw if pointing_hpbw is None else pointing_hpbw
    pt1 = ("gauss", float(f"{sig:.3g}"))
    clut = clutter(gr, r_km) if clutter is not None else None
    kw = dict(rx_sll_db=sll, t_sys_k=t_ku, r_min_km=r_min_sec, v_span_mps=v_span, f_ghz=f_ku, spj=spj, clutter=clut,
              excl=excl, pd=tm.PD_UPDATE)
    single = ku_look(gr["theta"], tgt_t, r_km, p_avg_max_w, pack, pointing=pt1, **kw)
    cluster = ku_look(gr["theta"], tgt_t, r_km, p_avg_max_w, pack, pointing=("cluster", rm.BEAM_SPACING_HPBW), **kw)
    opts = []
    if single is not None:
        opts.append((single["cost"], "single", single, 1.0))
    if cluster is not None:
        opts.append((cluster["cost"] * CLUSTER_N, "cluster", cluster, float(CLUSTER_N)))
    if not opts:
        return None
    _, mode, look, mult = min(opts, key=lambda o: o[0])
    if mode == "single" and pointing_hpbw is None:
        pds = [_coast_pd(look, st.coast_pointing(k), tgt_t, f_ku, sll, v_span, spj) for k in range(tm.N_MISS_LOST)]
    else:
        pds = [tm.PD_UPDATE] * tm.N_MISS_LOST
    p_loss, _ = tm.loss_per_update(pds)
    return {"look": look, "mode": mode, "mult": mult, "p_loss": p_loss, "pds": pds}


def _coast_pd(look: dict, sigma_hpbw: float, tgt_t: rm.Target, f_ku: float, sll: float, v_span: float,
              spj: Optional[float]) -> float:
    """Pd of the chosen look when it points with the error sigma_hpbw of a coasted track (B6): the detection law of
    the look with that pointing, at the SNR the look delivers (its design SNR, plus the margin when the accuracy and
    not the detection sized it)."""
    sp, j = look["space"], look["index"]
    d = sp.det[j]
    p0 = sp.det_parts[j]
    margin = 40.0 * math.log10(p0[0] / sp.rn[j]) if sp.rn[j] > 0 and p0[0] > sp.rn[j] else 0.0
    pk = mo.parts_at(d, f_ku, 0.0, tgt_t, sll, v_span, None, None, spj, None, ("gauss", float(f"{sigma_hpbw:.3g}")),
                     tm.PD_UPDATE)
    if pk is None:
        return 0.0
    law = np.array([pk[4]])
    return float(mo._pd_at(mo._curves(law), law, np.array([p0[2] + margin]))[0])


def _initiation_cost(gr: dict, tgt: rm.Target, st: "tm.TrackState", r_km: float, hand_theta: float, zone: float,
                     racc: Optional[float], vacc: float, ctx) -> Optional[dict]:
    """(looks, electrical J, Ku TX s, heat J) of one track initiation (C1-C3): the looks until the requirements hold,
    each pointed with the mean predicted error of the initiation (single look or cluster, the cheaper)."""
    n, sig = tm.initiation(st, tgt, hand_theta, st.allowed["r"] if math.isfinite(st.allowed["r"]) else 1e3,
                           st.allowed["vr"], zone, racc, vacc)
    upd = _track_update(gr, tgt, st, r_km, ctx, pointing_hpbw=max(sig, st.pointing_hpbw))
    if upd is None:
        return None
    e, t, q = look_cost(upd["look"], 1.0)
    return {"looks": n, "energy_J": n * e * upd["mult"], "time_s": n * t * upd["mult"], "heat_J": n * q * upd["mult"]}


def _longest_period(tgt: rm.Target, gr: Optional[dict], ranges: List[float], t_s: float, zone: float,
                    racc: Optional[float], vacc: float, ctx) -> Optional[float]:
    """Longest update period, below the operator's, at which every failing range of the bracket holds its
    requirements with a reachable look (A8, D2); None if even a very short period does not."""
    if gr is None:
        return None
    f_ku = ctx[4]

    def ok(t):
        for rk in ranges:
            st = tm.TrackState(tgt, t, rk, zone, racc, vacc, f_ku, 0.0)
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
    r_sat_ku = rm.min_unsaturated_range_km(f_ku, 0.0, tgt.rcs_at(f_ku))
    r_sat_att = rm.min_unsaturated_range_attenuated_km(f_ku, 0.0, tgt.rcs_at(f_ku))     # step 3.3, E3
    r_min_sec = mo.range_min_km(sec)          # shortest range of interest: Ku blind range limit (step 2.1)
    spacing = r.get("beam_spacing_hpbw")                 # the sector's search lattice (step 3.1, P5)
    angle_cells = rm.rx_beams_per_position(f_s, sll, spacing) * rm.RX_BEAM_SPACING_HPBW ** 2   # independent cells per position
    for b in sec.get("tracking", []):
        if b["r_min_km"] < r_min_sec:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km starts below the sector's shortest range of "
                         f"interest ({r_min_sec:.0f} km, range_min_km): Ku looks only need a blind range below that")
        if b["r_min_km"] < r_sat_att:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: below {r_sat_att:.2f} km the Ku echo of "
                         f"{tgt.rcs_at(f_ku):g} m2 clips the per-element ADCs even with the "
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
    for mode in CONFIRM_MODES:
        why = ["part of the sector is out of Ku reach"] if ku_missing > 0 else []
        best = None
        for cl in cluster_variants():
            st = ConfirmStats(fl_s, fl_ku, f_s, f_ku, spacing, cl)
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
                                       "round_trips": 3, "s_touch_wf": "pointed search waveform (role CONFIRM)"},
                                e, ts, tk, q)
            else:
                mk = st.min_margin(lambda x: st.ku(mode, x)[0])
                if mk is None:
                    continue
                succ, looks = st.ku(mode, mk)
                l_fa = cl.looks + (1.0 if mode == "KU1+7" else 0.0)       # a false alarm fails every look
                e, tk, q = ku_cost(looks, l_fa, rm.db_to_lin(mk))
                ts = np.zeros(4)
                load = load_of(e, ts, tk)
                if best is None or load < best[0]:
                    best = (load, {"margin_db": round(float(mk), 3), "success": succ, "ku_looks_true": looks,
                                   "ku_looks_fa": l_fa, "cluster_tx": cl.name,
                                   "round_trips": 2 if mode == "KU1+7" else 1}, e, ts, tk, q)
        if best is None:
            why.append(f"confirmation probability < {CONFIRM_P_MIN} even with {MARGIN_MAX_DB:g} dB margin")
            modes[mode] = {"admissible": False, "why_not": "; ".join(why)}
            continue
        _, info, e, ts, tk, q = best
        modes[mode] = {**info, "admissible": not why, "why_not": "; ".join(why),
                       "e": e / p_avg_max_w / rr, "ts": ts / rr, "tk": tk / rr, "q_w": q / rr}

    # --- active tracking per bracket (step 3.3: track model of core/trackmodel.py)
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
        states = [tm.TrackState(tgt, T, rk, zone, racc, vacc, f_ku, 0.0) for rk in ranges]
        e_face, t_face, q_face = np.zeros(4), np.zeros(4), np.zeros(4)
        drivers: Dict[str, float] = {}
        wf_main, pos_main, unreachable = None, -1.0, 0.0
        n_cluster, n_upd, p_loss, infeasible = 0.0, 0.0, [], []
        for gr in groups:
            share = gr["pos"] / p_all * b["n_targets"]                       # tracks per face in this group
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
                w = share / len(ranges)
                e_face += w * e * mult
                t_face += w * tk * mult
                q_face += w * q * mult
                drivers[look["driver"]] = drivers.get(look["driver"], 0.0) + float(w.sum())
                n_upd += float(w.sum())
                n_cluster += float(w.sum()) if upd["mode"] == "cluster" else 0.0
                if gr is main_gr:
                    p_loss.append(upd["p_loss"])
                if gr["pos"].sum() > pos_main:
                    wf_main, pos_main = look["wf"], float(gr["pos"].sum())
            unreachable += float(share.sum()) * (1.0 - ok_w)
        n_tr = float(b["n_targets"])
        pl = float(np.mean(p_loss)) if p_loss else 0.0
        loss_rate = n_tr * pl / T                                          # tracks lost per second (B7)
        # initiation (C) in the bracket where tracks start, and re-initiation of the lost tracks (B8), at the bracket's
        # start range (start bracket) or mid-range; each also costs a confirmation look of the main group
        r_init = min(r_det, r_hi) if bi == start else 0.5 * (r_lo + r_hi)
        st0 = tm.TrackState(tgt, T, r_init, zone, racc, vacc, f_ku, 0.0)
        new_rate = (n_new / rr) if bi == start else 0.0
        init = _initiation_cost(main_gr, tgt, st0, r_init, hand_theta, zone, racc, vacc, tm_ctx) \
            if (main_gr is not None and st0.feasible) else None
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
            rr_alert = _longest_period(tgt, main_gr, sorted(set(infeasible)), T, zone, racc, vacc, tm_ctx)
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: update period {T:g} s not tenable "
                         + (f"(longest tenable {rr_alert:.3g} s)" if rr_alert else "(no period tenable)")
                         + ": accept it, or relax the accuracies, the number of tracks or the range")
        mid = states[len(states) // 2]
        out_b = {**b, "sigma_theta_deg": math.degrees(mid.noise_theta),
                 "driver": max(drivers, key=drivers.get) if drivers else "-",
                 "energy_per_update_J": float(e_face.sum() / n_tr) if n_tr > 0 else 0.0,
                 "avg_power_kW": float((e_face.sum() / T + e_face_s.sum()) / 1e3),
                 "wf": _wf_label(wf_main, f_ku) if wf_main is not None else "-",
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
                 "init_energy_J": init["energy_J"] if init else None,
                 "attenuation_db": rm.attenuation_needed_db(f_ku, 0.0, tgt.rcs_at(f_ku), r_lo),
                 "deception_verification": deceptive}
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
                             b.get("range_acc_m"), float(b["vr_acc_mps"]), f_ku, 0.0)
        far = (math.radians(env.faraday_round_trip_deg(sec["tec_tecu"], sec["b_parallel_nt"], f_ku))
               if sec.get("tec_tecu") is not None and sec.get("b_parallel_nt") is not None else 0.0)
        disc = tm.discrimination(tgt, rm.TARGETS[other], f_ku, rm.db_to_lin(rm.lin_to_db(conf_snr)), far,
                                 math.sqrt(st_d.pf_cross[2, 2]))
        disc["against"] = other
        r0 = min(r_det, float(b["r_max_km"]))
        if disc["looks"] is not None:
            disc["time_s"] = disc["looks"] * T
            disc["range_km"] = r0 - tgt.v_close_mps * disc["time_s"] / 1e3
            if disc["range_km"] < r_min_sec:
                flags.append(f"{tgt.target_type} / {other} discrimination at P {tm.P_DISCRIMINATION:g} reached at "
                             f"{disc['range_km']:.0f} km, below the shortest range of interest {r_min_sec:.0f} km")
        else:
            disc["time_s"] = disc["range_km"] = None
            flags.append(f"{tgt.target_type} / {other} discrimination does not reach P {tm.P_DISCRIMINATION:g} within "
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
    bt = None
    if spj is not None and (r.get("sky") or {}).get("spj_out_of_reach"):
        th = max((gr["theta"] for gr in groups if gr["theta"] <= ku_lim), default=None)
        if th is not None:
            bt = ku_burn_through(th, tgt, r_det, p_avg_max_w, sll, t_ku, f_ku, spj, excl)
            flags.append(f"Ku burn-through look in the jamming axis: {bt['range_km']:.0f} km "
                         f"({'reaches' if bt['reaches_requirement'] else 'does not reach'} the requirement, "
                         f"{bt['wf']}); triggered by the scheduler (step 4.3)")
    return {"faces": [k for k in range(4) if p_face[k] > 0], "false_alarms_per_rr": fa, "modes": modes, "at": at,
            "flags": flags, "ku_sky": ku_sky, "ku_min_unsaturated_range_km": r_sat_ku,
            "ku_min_unsaturated_range_attenuated_km": r_sat_att, "discrimination": disc,
            "false_tracks_per_s": false_tracks, "scheduler_rules": rules,
            "ku_freq_ghz": f_ku, "ku_burn_through": bt}


# ----------------------------------------------------------------------------- mission budget
_RES = ("e", "ts", "tk")
_RES_NAME = {"e": "energy", "ts": "S time on air", "tk": "Ku time on air"}


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
    return _max_load(tot)


def _ku_freq_task(sec: dict, r: dict, Pm: np.ndarray, p_avg_max: float, eps_min: float, eps_max: float,
                  sky: Optional[dict], f: float) -> dict:
    """sector_budget of one sector at one Ku frequency, with its Ku sky and surfaces (worker entry point of budget)."""
    sph = mo._sphere(eps_min, eps_max)
    m = sph.mask(sec)
    ku_sky, keep = mo.sector_sky(sec, sph, m, f, "Ku", sky)
    clut = _SectorClutter(sec, sph, m, sky, f, keep) if env.surfaces(sky) else None
    return sector_budget(sec, r, Pm, p_avg_max, ku_sky, f, clut)


def budget(mission: dict, res: dict) -> dict:
    """Mission confirmation + AT budget on top of an optimizer.run() result: per sector the Ku frequency and the
    confirmation mode (with the reason) and the AT cost per bracket, per face the search / confirmation / AT shares and
    the verdict, and what to relax when a face does not fit."""
    g = res["global"]
    p_avg_max = g["face_elec_avail_kW"] * 1e3
    sph = mo._sphere(g["eps_min_deg"], g["eps_max_deg"])
    secs = {s["id"]: s for s in mission["sectors"]}
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
            "ku_burn_through": sb["ku_burn_through"], "scheduler_rules": sb["scheduler_rules"],
            "discrimination": sb.get("discrimination"), "false_tracks_per_s": sb.get("false_tracks_per_s"),
            "ku_min_unsaturated_range_attenuated_km": sb.get("ku_min_unsaturated_range_attenuated_km"),
            "flags": sb["flags"]})
        for fl in sb["flags"]:
            warn.append(f"sector {sid}: {fl}")

    # per-face totals, verdict and what to relax
    chosen = {sid: sbs[sid][choice[sid][0]] for sid in sbs}
    mode_of = {sid: choice[sid][1] for sid in sbs}
    faces, advice = {}, []
    for k in range(4):
        conf_e = sum(chosen[s]["modes"][mode_of[s]]["e"][k] for s in sbs if mode_of[s] is not None)
        at_e = sum(b["e"][k] for s in sbs for b in chosen[s]["at"])
        conf_ts = sum(chosen[s]["modes"][mode_of[s]]["ts"][k] for s in sbs if mode_of[s] is not None)
        conf_tk = sum(chosen[s]["modes"][mode_of[s]]["tk"][k] for s in sbs if mode_of[s] is not None)
        at_tk = sum(b["tk"][k] for s in sbs for b in chosen[s]["at"])
        heat = float(res["faces_total"][str(k)].get("dissipation_kW", 0.0) * 1e3
                     + sum(chosen[s]["modes"][mode_of[s]]["q_w"][k] for s in sbs if mode_of[s] is not None)
                     + sum(b["q_w"][k] for s in sbs for b in chosen[s]["at"]))
        load = {key: float(tot[key][k]) for key in _RES}
        worst = max(load, key=load.get)
        faces[str(k)] = {"search_energy": float(base["e"][k]), "confirm_energy": float(conf_e), "at_energy": float(at_e),
                         "total_energy": load["e"], "search_s_time": float(base["ts"][k] - cal_s),
                         "confirm_s_time": float(conf_ts), "total_s_time": load["ts"],
                         "confirm_ku_time": float(conf_tk), "at_ku_time": float(at_tk), "total_ku_time": load["tk"],
                         "calibration_s_time": cal_s, "calibration_ku_time": cal_ku,
                         "load": load[worst], "binding": _RES_NAME[worst],
                         "status": "OK" if load[worst] <= 1.0 else "OVER-SUBSCRIBED",
                         "p_left_kW": max(0.0, 1.0 - load["e"]) * p_avg_max / 1e3,
                         "dissipation_kW": heat / 1e3, "dissipation_kW_m2": heat / 1e3 / rm.TOTAL_AREA_M2}
        if load[worst] > 1.0:
            advice.append(_relax_advice(k, worst, load[worst], res, chosen, mode_of, secs))
    for a in advice:
        warn.append(a)
    rules = ([f"search lattice shifted every frame (grid_dithering, step 2.3): the cumulative detection criterion "
              f"draws the target position in its cell anew at each revisit"] if rm.GRID_DITHERING else [])
    rules += iso.scheduler_rules() + [f"sector {s['id']}: {t}" for s in out_secs for t in s["scheduler_rules"]]
    warn.extend(rm.calibration_checks())                    # step 3.4 (B5-B6)
    return {"pfa": rm.DETECT_PFA, "confirm_p_min": CONFIRM_P_MIN,
            "sectors": out_secs, "faces": faces, "advice": advice, "scheduler_rules": rules, "warnings": warn,
            # step 3.3 (F3): P(correct) between the library classes after a few looks at the design SNR (indicator)
            "separability": tm.separability(KU_REF_GHZ, _snr_ref(1)),
            # step 3.4: residual array errors (gain loss, sidelobe floor, boresight, calibration share of the time)
            "array_errors": {b: rm.array_errors(b) for b in ("S", "Ku")}}


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
