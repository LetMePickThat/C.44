#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# Radar Performance Model - V6.0.11
#
# Port of V5.4.0 to the current space AESA (4 faces, S-band surveillance +
# Ku-band confirmation/tracking, fully digital TRM architecture).
#
# Tags used in comments:  [V] locked by user | [D] derived / proposed, to confirm
#                         [?] known conflict between sources, to settle
#
# Changes V6.0.11 -> V6.0.12 (array tilt)
#   - SECTOR_TILT_DEG 20 -> 25 deg [V]: updated figure for all four arrays (ship axis vertical, lighthouse
#     model). Sector area in (u, v), the owner masks, every eps/theta formula derived from the tilt, and
#     the 3D ship model's radar-mount cone (core/geometry.py, now reads this constant instead of a
#     separate hardcoded angle) all follow automatically; only the self-test's locked numeric checks and
#     mission_detection_envelope.csv needed recomputing.
#
# Changes V6.0.10 -> V6.0.11 (confirmation and active tracking budget, core/tracking.py)
#   - DETECT_PFA 1e-6 -> 1e-8 per range-Doppler cell [V]: every detection is confirmed in Ku, and at 1e-6 the
#     ~21 500 false alarms per face and refresh cost twice the search itself (search +33 %, total / 2.3).
#   - K_THETA 0.7 -> 0.5 [V]: ML angle estimator on the full digital aperture (Cramer-Rao ~0.44 plus margin).
#   - Tracking Doppler term: observation time = the dwell (N - 1) PRI + tau, not N x PRI (a single 10 us pulse was
#     credited with a whole PRI of Doppler observation).
#
# Changes V6.0.9 -> V6.0.10
#   - Mission inputs SECTOR_EPS_MIN_DEG / SECTOR_EPS_MAX_DEG (rear/front limits of the scan zone, eps from the XY plane):
#     adjustable per mission preparation and sent to the scheduler, independent of tilt and scan limit. Default -90/+90.
#   - Working geometry: ALL arrays tilted 20 deg toward the bow (SECTOR_TILT_DEG, estimated ~22 deg on the drawing).
#     Ship axis = vertical (lighthouse model: az swept about the axis, bow = zenith). Face normals at el = +20 deg.
#     Face boundaries stay the az = +/-45 deg meridian planes; in the face frame they become |u| <= w cos t - v sin t.
#     Sector area 2.198 -> 2.069, positions per face at 2.8 GHz 4577 -> ~4308 (total 4 faces ~17 230).
#   - Beam spacing 1.1 -> 1.04 HPBW [V] (worst-case optimum): closes coverage holes at hex vertices
#     (worst point ~-4.3 dB vs -4.9 dB; mean TX grid loss 1.81 dB vs 2.02 dB).
#   - SECTOR FIX: face boundary = the planes az = +/-45 deg between adjacent faces (4 faces x 90 deg), i.e. in (u, v)
#     |u| <= sin45*sqrt(1-v^2) (2u^2+v^2<=1), intersected with the scan-limit disc. The previous straight cut |u|<=sin45
#     overshot the plane at elevation ("diabolo" in angle space): area 2.463 -> 2.198 (-10.8 %), positions 5130 -> ~4577.
#
# Changes V6.0.8 -> V6.0.9
#   - Beam spacing locked [V] at 1.1 HPBW (was 1.4): consistent with geom_v1 (5250 positions at 3.06 GHz / 1.211 deg
#     <=> ~1.1 HPBW on the truncated disc) and near the worst-case optimum (~1.04). At 2.8 GHz: ~4590 positions,
#     mean TX grid loss 2.02 dB, crossover -3.6 dB, worst point -4.9 dB (Gaussian lobe).
#
# Changes V6.0.7 -> V6.0.8
#   - Sector shape corrected: a circle truncated by two straight cuts (not an oval). In (u, v) the sector is the
#     disk u^2 + v^2 <= sin^2(75 deg) cut by |u| <= sin(45 deg) (azimuth taken as asin(u), direction-cosine
#     convention, straight cut lines). V6.0.7 used |atan2(u, w)| <= 45 deg, whose cut lines are curved.
#     Area 2.461 (was 2.193): ~2835 positions at 2.8 GHz / 1.4 HPBW (was 2526).
#
# Changes V6.0.6 -> V6.0.7
#   - Search positions counted in direction-cosine space (u, v): a planar array scans with a beam of constant
#     width in (u, v) (angular width grows as 1/cos(theta)). Sector = {theta <= SECTOR_RADIUS, |az| <= SECTOR_AZ_HALF}
#     mapped to (u, v); cell = sqrt(3)/2 x (BEAM_SPACING_HPBW x lambda*0.886/D)^2.
#     At 2.8 GHz and 1.4 HPBW: ~2520 positions (was 4425 counted in angle space, which ignored beam broadening).
#
# Changes V6.0.5 -> V6.0.6
#   - KKV closing speed locked [V] at 25 km/s (was 30): 15 km/s max LEO crossing orbital speed + 7 km/s KKV
#     terminal speed (universe: 6-7 km/s at impact, 10+ with booster) + margin. Conservative design bound.
#
# Changes V6.0.4 -> V6.0.5
#   - Ku instantaneous bandwidth locked [V] at 200 MHz (hardware ceiling, BandConfig.bi_max_mhz); Waveform
#     refuses sig_bw_mhz above it. Needs per-TRM digital true-time delay at scan. S band: not defined (0 = no limit).
#   - Waveforms stay open (future input: dynamically designed by the central computer, no fixed catalogue).
#
# Changes V6.0.3 -> V6.0.4 : LOCKED [V] by the user (no value change, tags only)
#   - Ku band 13.5-14.5 GHz (scan 75 deg up to 14.25 GHz, 70 deg above), Ku illumination 0.88/0.88.
#   - Beam spacing 1.4 HPBW (grid loss 3.28 dB mean, ~4425 positions at 2.8 GHz).
#   - All loss terms: S: DSP 2.0, CFAR 0.5, calibration 0.5, pre-LNA 0.3, Ku blockage 0.2, TX chain 0.3,
#     polarisation 0.5 (dual-pol RX), buffer 5 J/TRM (droop 0.24 dB on WF-LRS), multibeam 0.8 dB @ 8 beams,
#     grid (spacing above), migration 1.0 dB compensated; Ku: 8 dB lumped.
#
# Changes V6.0.2 -> V6.0.3 (S band; Ku keeps its lumped 8 dB)
#   - TX_chain = 0.3 dB: PA output -> radiating element inside the integrated TRM (T/R switch or circulator,
#     output match, feed). No distribution network exists (power-only + optical command in), so only this
#     local path remains; by reciprocity it equals the pre-LNA path (0.3 dB). [?] 0 dB if the 2 kW is
#     specified at the radiating port. Array active-VSWR at scan is already in the scan loss.
#   - Polarisation loss 0.5 dB [?]: single TX polarisation, dual-polarised digital RX recombined (per-TRM ADC);
#     covers axial-ratio / cross-pol degradation with scan. POL_RX_DUAL=False gives 3.0 dB (single-pol RX on a
#     depolarising target).
#   - Energy-buffer droop: per-TRM buffer E_BUF_TRM_J = 5 J [?]. Pulse electrical energy = P_pk/PAE x tau,
#     taken from the buffer (recharge during the pulse neglected: conservative), P_RF proportional to stored
#     energy. x = E_pulse/E_buf; mean power droop x/2 -> loss = -10 log10(1 - x/2). WF-LRS: x = 0.107, 0.24 dB.
#     solve() refuses x > 0.5 (buffer undersized for the pulse). Inter-pulse recharge checked by the
#     average-power budget.
#
# Changes V6.0.1 -> V6.0.2
#   - Processing losses set to 2050 / deep-space values (user decision): DSP 3.0 -> 2.0 dB
#     (weighting/mismatch ~1.0 + range/Doppler straddle ~0.7 + finite precision ~0.3),
#     CFAR 1.5 -> 0.5 dB (homogeneous, known noise floor, no clutter, large reference window).
#   - Kept as is by decision: grid loss (mean over the cell, 1.4 HPBW), migration 1 dB (velocity hypotheses
#     calibrated on orbital mechanics), T_sys 346.5 K (deep-space value).
#
# Changes V6.0.0 -> V6.0.1 (consistency pass)
#   - Beam grid: ONE spacing parameter (BEAM_SPACING_HPBW, hexagonal lattice) now drives both the number of
#     search positions and the mean TX grid loss (closed form 1.672*spacing^2 dB, Gaussian main lobe).
#     Grid loss is ON by default: V6.0.0 was optimistic by ~3.3 dB (range x0.83) at the default spacing.
#     Spacing 0.92 reproduces the 1.41 dB quoted in the notes, but implies ~10200 positions.
#   - Time bases separated: dwell = (N-1)*PRI + tau (frame time, migration), coherent time (tau for a single
#     pulse, dwell for a coherent train; coherence loss). V6.0.0 used N*PRI for all three.
#   - Frame time optionally includes the echo wait (FRAME_INTERLEAVED = False).
#   - solve() now refuses a frequency outside the waveform band or a scan angle above the band limit.
#   - TX_chain loss placeholder (0 dB) in S band: reference plane of the 2 kW is still to be settled [?].
#   - NOT changed (needs a decision): scan broadening in the position count, migration model, T_sys/pre-LNA
#     definition, Ku loss decomposition, element factor exponent for S.
#   - Round-trip-time dependence of the coherence loss is deliberately NOT modelled: it only affects
#     pulse-to-pulse coherence and is bounded by the coherent span already used.
#
# Changes vs V5.4.0 (V6.0.0)
#  FIXES
#   - Detection SNR: Albersheim was mis-implemented (log10 argument used as dB) and the
#     integration gain was counted twice (-10log10(N) on the SNR AND N*tau in the energy
#     term). Replaced by Shnidman's TOTAL-ENERGY SNR requirement (C*X_inf), consistent
#     with the energy-form radar equation. Albersheim kept as a cross-check only.
#   - Explicit COHERENT / NONCOHERENT integration mode (V5.4.0 mixed both through N_eff).
#   - Signal bandwidth (range resolution/accuracy) separated from RX channel bandwidth.
#   - Scan loss no longer clamped at 70 deg (extrapolated up to the band scan limit).
#  NEW
#   - Two bands (S, Ku) with their own TRM count, peak power, losses, T_sys, scan limit.
#   - Only waveforms PLAYABLE at the chosen frequency are evaluated.
#     Role policy: S = SEARCH/TWS only (no tracking), Ku = CONFIRM/PROMOTE/TRACK only (no search).
#   - TX and RX gains separated (TX illumination 1.0, RX 0.88 in S) [V].
#   - Average-power / duty check against the electrical budget (PAE 75 % [V]).
#   - Range-migration loss (target closing speed) and range-ambiguity flag.
#   - Search-frame feasibility (positions x dwell vs refresh time).
#   - Multi-target table (ST / KKV / SHIP).
#  REMOVED
#   - All Ku-era waveforms (WF-C, ETHR, ET, PS, PA, PRD, PRA, LRV, AT), RX channel
#     accounting (obsolete with one ADC per TRM), cued-measurement driver.
#   - Only WF-LRS (S band) kept, as a code benchmark.
# ============================================================

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

MODEL_VERSION = "6.0.12"

# =============================================================================
# USER-EDITABLE PARAMETERS
# =============================================================================
# --- Scenario ---
FREQUENCY_GHZ: float = 2.8       # Operating frequency (GHz); selects the band and playable waveforms
SCAN_ANGLE_DEG: float = 0.0      # Scan angle off boresight (deg)
TARGET_NAME: str = "ST"          # "ST" | "KKV" | "SHIP"
FACE_ELEC_POWER_MW: float = 2.0  # [V] electrical power allocated to this face (total 8 MW over 4 faces)

# --- Detection ---
DETECT_PD: float = 0.9           # [V]
DETECT_PFA: float = 1e-8         # [V] per range-Doppler cell, single-step detection (1e-6 until 2026-10-19: false-alarm
                                 #     confirmations then cost twice the search; 1e-8 divides search + confirmation by 2.3)

# --- Power chain [V] ---
PAE: float = 0.75                                  # [V] chain PAE (replaces 65 %)
CHAIN_EFF: float = 0.95 * 0.995 * (1.0 - 0.03)     # [V] PSU x distribution x (1 - overhead) = 0.917

# --- Range migration [V] ---
MIGRATION_MODE: str = "COMPENSATED"   # "COMPENSATED" (residual loss) | "UNCOMPENSATED"
MIGRATION_RESIDUAL_DB: float = 1.0    # residual loss when compensated (keystone/Radon-type processing)

# --- Search beam grid [V] (spacing 1.1 HPBW locked) ---
# Hexagonal lattice of TX beam positions, spacing = BEAM_SPACING_HPBW x HPBW. The SAME spacing drives the number
# of positions (frame time) and the mean TX grid loss (RX is digital and pointed on the target: TX only).
#   1.1  -> ~4590 positions, mean loss ~2.0 dB, worst -4.9 dB (V6.0.9) ; 1.04 -> ~5130, mean 1.81 dB, worst -4.3 dB (locked, V6.0.10)
#   1.4  -> ~2830 positions, mean loss ~3.3 dB, worst point -7.9 dB (best mean range per energy)
#   0.92 -> ~10200 positions, mean loss 1.41 dB (the value quoted in the notes)
BEAM_SPACING_HPBW: float = 1.04
ENABLE_GRID_LOSS: bool = True
FRAME_INTERLEAVED: bool = True   # [D] echo wait hidden by interleaving positions (digital RX); else dwell + 2R/c

# --- Multibeam (TX) [V]: exact V5.6.0 definition unknown; RX loss = 0 (digital beamforming) ---
MB_L8_DB: float = 0.8
MB_P: float = 1.1

# --- Scan loss (law from V5.4.0, [V] 0.6 dB at 70 deg; extrapolated beyond) ---
POL_RX_DUAL: bool = True         # [V] dual-polarised digital RX (per-TRM ADC)
POL_LOSS_DB: float = 0.5 if POL_RX_DUAL else 3.0
E_BUF_TRM_J: float = 5.0         # [V] S-band per-TRM energy buffer (J)
ENABLE_SCAN_LOSS: bool = True
SCAN_LOSS_THETA_REF_DEG: float = 70.0
SCAN_LOSS_THETA0_DEG: float = 25.0
SCAN_LOSS_LMAX_DB: float = 0.6
SCAN_LOSS_POWER_P: float = 3.0

# --- Coherence / phase noise (V5.4.0 model, applied to COHERENT integration only) ---
ENABLE_COH_LOSS: bool = True
COH_T0_S: float = 0.25
COH_P: float = 2.0
COH_SIGMA_T_S: float = 20e-15
COH_ETA_MIN: float = 0.70

# --- Aperture physics ---
PHYS_AREA_M2: float = 13.25   # [V] emissive area per face
ETA_OHMIC: float = 0.96
SCAN_EXPONENT: float = 1.1    # element factor cos^n (Ku value of V5.6.0, kept for S) [D]
ARRAY_ENVELOPE_M: float = 16 * 0.260 + 15 * 0.002   # 4.19 m (square envelope, used for HPBW only)

# --- Surveillance sector (per face) [V] ---
SECTOR_RADIUS_DEG: float = 75.0
REFRESH_S: float = 120.0

# --- Tracking estimator constants (V5.4.0, K_THETA revised; used by Ku TRACK waveforms, core/tracking.py) ---
K_THETA: float = 0.5            # [V 2026-10-19] ML estimator on the full digital aperture (0.7 = classic sum/difference)
K_F: float = 0.5
RANGE_CRLB_KR: float = 1.3
# =============================================================================

C_MPS = 299_792_458.0
K_BOLTZ = 1.380_649e-23


def db_to_lin(x_db: float) -> float:
    return 10.0 ** (x_db / 10.0)


def lin_to_db(x: float) -> float:
    return 10.0 * math.log10(x) if x > 0 else -999.0


def lambda_m(freq_ghz: float) -> float:
    return C_MPS / (freq_ghz * 1e9)


# -----------------------------------------------------------------------------
# Bands
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class BandConfig:
    name: str
    usable_lo_ghz: float
    usable_hi_ghz: float
    core_lo_ghz: float
    core_hi_ghz: float
    edge_derating_db: float
    scan_max_deg: float          # scan limit up to scan_break_ghz
    scan_break_ghz: float
    scan_max_hi_deg: float       # scan limit above scan_break_ghz
    n_trm: int
    p_pk_trm_w: float
    eta_illum_tx: float
    eta_illum_rx: float
    t_sys_k: float
    losses_db: Dict[str, float]  # fixed implementation losses (summed)
    allowed_roles: Tuple[str, ...]
    pol_loss_db: float = 0.0      # polarisation mismatch (0 = included in the lumped losses)
    e_buf_trm_j: float = 0.0      # per-TRM energy buffer, J (0 = droop not modelled)
    bi_max_mhz: float = 0.0       # instantaneous bandwidth ceiling, MHz (0 = not defined)

    def contains(self, f_ghz: float) -> bool:
        return self.usable_lo_ghz <= f_ghz <= self.usable_hi_ghz

    def scan_limit_deg(self, f_ghz: float) -> float:
        return self.scan_max_deg if f_ghz <= self.scan_break_ghz else self.scan_max_hi_deg

    def edge_loss_db(self, f_ghz: float) -> float:
        """Raised-cosine derating outside the core band (shape of V5.4.0, scaled to the band)."""
        if not self.contains(f_ghz):
            return 100.0
        if self.core_lo_ghz <= f_ghz <= self.core_hi_ghz:
            return 0.0
        lmax = self.edge_derating_db
        if f_ghz < self.core_lo_ghz:
            x = (f_ghz - self.usable_lo_ghz) / (self.core_lo_ghz - self.usable_lo_ghz)
            return lmax * 0.5 * (1.0 + math.cos(math.pi * x))
        x = (f_ghz - self.core_hi_ghz) / (self.usable_hi_ghz - self.core_hi_ghz)
        return lmax * 0.5 * (1.0 - math.cos(math.pi * x))

    @property
    def p_pk_total_w(self) -> float:
        return self.n_trm * self.p_pk_trm_w

    @property
    def l_fixed_db(self) -> float:
        return sum(self.losses_db.values())


# Core band = V5.4.0 proportions (1/6 of the usable width on each side) [D];
# to be replaced by the rf_layer_s.py edge-derating table when available [?].
_S_LO, _S_HI = 2.7, 2.9
_KU_LO, _KU_HI = 13.5, 14.5   # [V] Ku band locked

BAND_S = BandConfig(
    name="S", usable_lo_ghz=_S_LO, usable_hi_ghz=_S_HI,
    core_lo_ghz=_S_LO + (_S_HI - _S_LO) / 6.0, core_hi_ghz=_S_HI - (_S_HI - _S_LO) / 6.0,
    edge_derating_db=1.5,
    scan_max_deg=75.0, scan_break_ghz=2.85, scan_max_hi_deg=70.0,           # [V]
    n_trm=4900, p_pk_trm_w=2000.0,                                          # [V] 9.8 MW peak / face
    eta_illum_tx=1.0, eta_illum_rx=0.88,                                    # [V]
    t_sys_k=346.5,                                                          # [V] S-band value (replaces 400 K)
    losses_db={"DSP": 2.0, "CFAR": 0.5, "calibration": 0.5,                # [V] 2050 deep-space values
               "pre_LNA": 0.3, "Ku_blockage": 0.2,
               "TX_chain": 0.3},                        # [V] PA output -> element, see header V6.0.3
    allowed_roles=("SEARCH", "TWS"),
    pol_loss_db=POL_LOSS_DB, e_buf_trm_j=E_BUF_TRM_J,
)

BAND_KU = BandConfig(
    name="Ku", usable_lo_ghz=_KU_LO, usable_hi_ghz=_KU_HI,
    core_lo_ghz=_KU_LO + (_KU_HI - _KU_LO) / 6.0, core_hi_ghz=_KU_HI - (_KU_HI - _KU_LO) / 6.0,
    edge_derating_db=1.5,
    scan_max_deg=75.0, scan_break_ghz=14.25, scan_max_hi_deg=70.0,          # [V] 5x the S thresholds
    n_trm=122500, p_pk_trm_w=75.0,                                          # [V] 9.19 MW peak / face
    eta_illum_tx=0.88, eta_illum_rx=0.88,                                   # [V]
    t_sys_k=400.0,                                                          # [V] V5.6.0
    losses_db={"lumped_impl": 8.0},                                         # [V] V5.6.0
    allowed_roles=("CONFIRM", "PROMOTE", "TRACK"),
    bi_max_mhz=200.0,                                                       # [V] Ku instantaneous bandwidth
)
BANDS: Dict[str, BandConfig] = {"S": BAND_S, "Ku": BAND_KU}


def face_avg_rf_w(elec_mw: float) -> float:
    """Average RF power a face can transmit (W) from its electrical power (MW): PSU, distribution, overhead, PAE."""
    return elec_mw * 1e6 * CHAIN_EFF * PAE


def band_for_frequency(f_ghz: float) -> Optional[BandConfig]:
    for b in BANDS.values():
        if b.contains(f_ghz):
            return b
    return None


# -----------------------------------------------------------------------------
# Targets
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class Target:
    name: str
    rcs_m2: float
    swerling: int
    v_close_mps: float


TARGETS: Dict[str, Target] = {
    "ST": Target("ST", 1000.0, 1, 5000.0),     # [V] Standard Target
    "KKV": Target("KKV", 0.135, 1, 25000.0),   # [V] RCS 0.12-0.15 m2 (mid), 25 km/s closing; SW1 assumed [D]
    "SHIP": Target("SHIP", 3000.0, 1, 5000.0), # [V] lateral RCS; SW1 and 5 km/s assumed [D]
}


# -----------------------------------------------------------------------------
# Gain / geometry
# -----------------------------------------------------------------------------
def aperture_m2(eta_illum: float, theta_deg: float) -> float:
    ef = max(math.cos(math.radians(theta_deg)) ** SCAN_EXPONENT, 1e-6)
    return PHYS_AREA_M2 * eta_illum * ETA_OHMIC * ef


def gain_lin(eta_illum: float, f_ghz: float, theta_deg: float) -> float:
    return 4.0 * math.pi * aperture_m2(eta_illum, theta_deg) / lambda_m(f_ghz) ** 2


def hpbw_deg(f_ghz: float, theta_deg: float = 0.0) -> float:
    d_eff = ARRAY_ENVELOPE_M * max(math.cos(math.radians(theta_deg)), 1e-6)
    return math.degrees(0.886 * lambda_m(f_ghz) / d_eff)


SECTOR_TILT_DEG: float = 25.0   # [V] all four arrays tilted toward +Z (nose) by this angle (ship axis vertical, lighthouse model)
# Mission-preparation inputs (sent to the scheduler), independent of tilt and scan limit:
SECTOR_EPS_MIN_DEG: float = -90.0   # rear limit: no scan below this elevation eps (-90 = no limit; physical floor ~ -55 deg)
SECTOR_EPS_MAX_DEG: float = 90.0    # front limit: no scan above this elevation eps (90 = no limit)


_SECTOR_AREA_CACHE: Dict[Tuple[float, float, float, float], float] = {}


def sector_area_uv() -> float:
    """Cached on the module parameters it depends on (solve() calls it for every SEARCH waveform)."""
    key = (SECTOR_RADIUS_DEG, SECTOR_TILT_DEG, SECTOR_EPS_MIN_DEG, SECTOR_EPS_MAX_DEG)
    if key not in _SECTOR_AREA_CACHE:
        _SECTOR_AREA_CACHE[key] = _sector_area_uv()
    return _SECTOR_AREA_CACHE[key]


def _sector_area_uv() -> float:
    """Area of the face sector in direction cosines (u, v) of the (tilted) face frame: disk of radius sin(R) (scan
    limit) cut by (i) the two PLANES phi = +/-45 deg between adjacent faces, (ii) the mission elevation window
    [SECTOR_EPS_MIN_DEG, SECTOR_EPS_MAX_DEG] (eps measured from the plane XY toward +Z).
    Face frame: direction = u e_u + v e_v + w n, n tilted by t toward +Z => eps = asin(v cos t + w sin t).
    The planes cut |u| <= w cos t - v sin t, i.e. |u| <= (-v sin t + cos t sqrt(1 + cos^2 t - 2 v^2)) / (1 + cos^2 t)
    (t = 0 gives sin45 sqrt(1 - v^2)). For a given v, eps decreases with |u|, so the eps window is an interval
    [u_lo, u_hi] on |u|."""
    r = math.sin(math.radians(SECTOR_RADIUS_DEG))
    c, sn = math.cos(math.radians(SECTOR_TILT_DEG)), math.sin(math.radians(SECTOR_TILT_DEG))
    zmin, zmax = math.sin(math.radians(SECTOR_EPS_MIN_DEG)), math.sin(math.radians(SECTOR_EPS_MAX_DEG))
    n = 20000
    tot = 0.0
    dv = 2.0 * r / n
    for i in range(n):
        v = -r + (i + 0.5) * dv
        disc = 1.0 + c * c - 2.0 * v * v
        hi = (-sn * v + c * math.sqrt(disc)) / (1.0 + c * c) if disc > 0 else 0.0
        hi = min(max(hi, 0.0), math.sqrt(max(r * r - v * v, 0.0)))
        lo = 0.0
        if sn > 1e-9:
            wmin = (zmin - v * c) / sn          # eps >= eps_min  <=>  w >= wmin
            if wmin > 0.0:
                hi = min(hi, math.sqrt(max(1.0 - v * v - wmin * wmin, 0.0)) if 1.0 - v * v - wmin * wmin > 0.0 else 0.0)
            wmax = (zmax - v * c) / sn          # eps <= eps_max  <=>  w <= wmax
            if wmax < 0.0:
                hi = 0.0
            elif wmax * wmax < 1.0 - v * v:
                lo = math.sqrt(1.0 - v * v - wmax * wmax)
        elif not (zmin <= v <= zmax):
            hi = 0.0
        if hi > lo:
            tot += 2.0 * (hi - lo) * dv
    return tot


def beam_cell_area_uv(f_ghz: float) -> float:
    """Area of one hexagonal cell of the beam lattice in (u, v): spacing = BEAM_SPACING_HPBW x 0.886 lambda / D."""
    h = 0.886 * lambda_m(f_ghz) / ARRAY_ENVELOPE_M
    return (math.sqrt(3.0) / 2.0) * (BEAM_SPACING_HPBW * h) ** 2


def n_search_positions(f_ghz: float) -> float:
    """Positions per face in the surveillance sector, counted in (u, v) (beam broadening with scan included)."""
    return sector_area_uv() / beam_cell_area_uv(f_ghz)


def grid_loss_db(spacing_hpbw: Optional[float] = None) -> float:
    """Mean TX loss over a regular hexagonal cell (Gaussian main lobe): L(r) = 12.04 (r/HPBW)^2 dB,
    mean r^2 over the cell = 5 s^2 / 36  ->  L = 1.672 s^2 dB. Corner (worst case) loss is 12.04 s^2 / 3."""
    s = BEAM_SPACING_HPBW if spacing_hpbw is None else spacing_hpbw
    return 10.0 * math.log10(math.e) * 4.0 * math.log(2.0) * (5.0 / 36.0) * s * s


# -----------------------------------------------------------------------------
# Loss terms
# -----------------------------------------------------------------------------
def scan_loss_db(theta_deg: float) -> float:
    if not ENABLE_SCAN_LOSS:
        return 0.0
    th = abs(theta_deg)
    if th <= SCAN_LOSS_THETA0_DEG:
        return 0.0
    u = (th - SCAN_LOSS_THETA0_DEG) / (SCAN_LOSS_THETA_REF_DEG - SCAN_LOSS_THETA0_DEG)
    return SCAN_LOSS_LMAX_DB * u ** SCAN_LOSS_POWER_P   # not clamped: 0.6 dB at 70 deg, ~0.82 dB at 75 deg


def multibeam_tx_loss_db(b_tx: int) -> float:
    if b_tx <= 1:
        return 0.0
    return MB_L8_DB * (math.log2(b_tx) / 3.0) ** MB_P


def coherence_loss_db(f_ghz: float, t_cpi_s: float) -> float:
    if not ENABLE_COH_LOSS or t_cpi_s <= 0.0:
        return 0.0
    s_lo2 = (t_cpi_s / COH_T0_S) ** COH_P
    s_jit2 = (2.0 * math.pi * f_ghz * 1e9 * COH_SIGMA_T_S) ** 2
    eta = max(math.exp(-(s_lo2 + s_jit2)), COH_ETA_MIN)
    return -10.0 * math.log10(eta)


def migration_loss_db(v_mps: float, t_span_s: float, sig_bw_mhz: float, n_pulses: int) -> Tuple[float, float]:
    """Return (loss_db, n_range_cells_crossed)."""
    dr = C_MPS / (2.0 * sig_bw_mhz * 1e6)
    n_cells = 1.0 + v_mps * t_span_s / dr
    if MIGRATION_MODE.upper() == "COMPENSATED":
        return (MIGRATION_RESIDUAL_DB if n_cells > 1.5 else 0.0), n_cells
    return 10.0 * math.log10(max(1.0, min(n_cells, float(n_pulses)))), n_cells


# -----------------------------------------------------------------------------
# Detection SNR requirement (TOTAL integrated energy SNR, for the energy-form radar equation)
# -----------------------------------------------------------------------------
def albersheim_total_snr_db(pd: float, pfa: float, n: int) -> float:
    """Steady target, noncoherent integration of n pulses. Cross-check only."""
    a = math.log(0.62 / pfa)
    b = math.log(pd / (1.0 - pd))
    per_pulse_db = -5.0 * math.log10(n) + (6.2 + 4.54 / math.sqrt(n + 0.44)) * math.log10(a + 0.12 * a * b + 1.7 * b)
    return per_pulse_db + 10.0 * math.log10(n)


def shnidman_total_snr_db(pd: float, pfa: float, n: int, swerling: int) -> float:
    """Shnidman empirical fit; returns the SNR required on the TOTAL energy of n noncoherently
    integrated pulses (= n x per-pulse SNR). Cases 0..4 (0 = steady)."""
    if not (0.1 <= pd <= 0.99) or not (1e-9 <= pfa <= 1e-2) or n < 1:
        raise ValueError("Shnidman fit outside validity range")
    eta = math.sqrt(-0.8 * math.log(4.0 * pfa * (1.0 - pfa))) \
        + (1.0 if pd > 0.5 else -1.0) * math.sqrt(-0.8 * math.log(4.0 * pd * (1.0 - pd)))
    alpha = 0.25 if n > 40 else 0.0
    x_inf = eta * (eta + 2.0 * math.sqrt(n / 2.0 + (alpha - 0.25)))
    k = {0: math.inf, 1: 1.0, 2: float(n), 3: 2.0, 4: 2.0 * n}[swerling]
    c_db = 0.0
    if math.isfinite(k):
        c1 = (((17.7006 * pd - 18.4496) * pd + 14.5339) * pd - 3.525) / k
        c2 = (math.exp(27.31 * pd - 25.14)
              + (pd - 0.8) * (0.7 * math.log(1e-5 / pfa) + (2.0 * n - 20.0) / 80.0)) / k
        c_db = c1 + (c2 if pd > 0.872 else 0.0)
    return 10.0 * math.log10(x_inf) + c_db


def detect_snr_req_db(integration: str, n: int, swerling: int) -> float:
    if integration == "COHERENT":
        if swerling not in (0, 1, 3):
            raise ValueError("COHERENT integration needs a slow-fluctuation case (SW0/SW1/SW3)")
        return shnidman_total_snr_db(DETECT_PD, DETECT_PFA, 1, swerling)   # one sample of N-fold energy
    return shnidman_total_snr_db(DETECT_PD, DETECT_PFA, n, swerling)


# -----------------------------------------------------------------------------
# Tracking requirement (Ku TRACK waveforms; formulas from V5.4.0)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class TrackingPack:
    sigma_theta_deg: float
    sigma_vr_mps: float
    sigma_r_m: Optional[float] = None


def tracking_snr_req(pack: TrackingPack, f_ghz: float, theta_deg: float, t_obs_s: float,
                     sig_bw_mhz: float) -> Tuple[float, str]:
    """SNR (linear) a track update needs for the pack's accuracies, and the dominant term (ANG / DOP / RNG).
    t_obs_s = coherent observation time of the Doppler measurement (the dwell, (N - 1) PRI + tau)."""
    lam = lambda_m(f_ghz)
    hp = math.radians(hpbw_deg(f_ghz, theta_deg))
    ang = (K_THETA * hp / math.radians(pack.sigma_theta_deg)) ** 2
    sigma_f = 2.0 * pack.sigma_vr_mps / lam
    dop = 0.5 * (K_F / (t_obs_s * sigma_f)) ** 2
    rng = 0.0
    if pack.sigma_r_m is not None:
        rng = 0.5 * ((RANGE_CRLB_KR * C_MPS) / (2.0 * sig_bw_mhz * 1e6 * pack.sigma_r_m)) ** 2
    comps = {"ANG": ang, "DOP": dop, "RNG": rng}
    dom = max(comps, key=comps.get)
    return comps[dom], dom


# -----------------------------------------------------------------------------
# Waveforms
# -----------------------------------------------------------------------------
DETECT_ROLES = ("SEARCH", "TWS", "CONFIRM", "PROMOTE")


@dataclass(frozen=True)
class Waveform:
    name: str
    role: str                 # SEARCH | TWS | CONFIRM | PROMOTE | TRACK
    band: str                 # "S" | "Ku"
    tau_s: float
    n: int                    # pulses per dwell
    pri_s: float
    sig_bw_mhz: float         # signal bandwidth (range resolution / accuracy)
    b_tx: int = 1
    integration: str = "NONCOHERENT"   # "COHERENT" | "NONCOHERENT"
    swerling: Optional[int] = None     # None -> target's case
    tracking_pack: Optional[TrackingPack] = None

    def __post_init__(self) -> None:
        if self.band not in BANDS:
            raise ValueError(f"{self.name}: unknown band {self.band}")
        if self.role not in BANDS[self.band].allowed_roles:
            raise ValueError(f"{self.name}: role {self.role} is not allowed in {self.band} band "
                             f"(allowed: {BANDS[self.band].allowed_roles})")
        bi = BANDS[self.band].bi_max_mhz
        if bi > 0 and self.sig_bw_mhz > bi:
            raise ValueError(f"{self.name}: signal bandwidth {self.sig_bw_mhz} MHz exceeds the {self.band} "
                             f"instantaneous bandwidth {bi} MHz")
        if self.role == "TRACK" and self.tracking_pack is None:
            raise ValueError(f"{self.name}: TRACK waveform needs a tracking_pack")
        if self.integration not in ("COHERENT", "NONCOHERENT"):
            raise ValueError(f"{self.name}: bad integration mode")
        if not (0.0 < self.tau_s <= self.pri_s) or self.n < 1 or self.b_tx < 1:
            raise ValueError(f"{self.name}: inconsistent timing")

    @property
    def duty(self) -> float:
        return self.tau_s / self.pri_s

    @property
    def dwell_s(self) -> float:
        """Time on target: first pulse start to last pulse end."""
        return (self.n - 1) * self.pri_s + self.tau_s

    @property
    def coherent_time_s(self) -> float:
        """Span over which phase must stay coherent: the pulse, or the whole train when integrated coherently."""
        return self.dwell_s if (self.integration == "COHERENT" or self.role == "TRACK") else self.tau_s


# --- Catalogue: WF-LRS (S) only, kept as code benchmark. Timing values come from the V5.4.0 Ku-era
# --- definition (S-band waveform design is abandoned/to be redefined [V]); integration set to
# --- NONCOHERENT because a 648 ms dwell cannot be integrated coherently on a moving target [D].
WF_LRS = Waveform(
    name="WF-LRS", role="SEARCH", band="S",
    tau_s=200e-6, n=24, pri_s=27e-3,          # PRI staggered 20/27/34 ms nominal in V5.4.0
    sig_bw_mhz=75.0, b_tx=4,
    integration="NONCOHERENT", swerling=None,
)
CATALOGUE: List[Waveform] = [WF_LRS]


def playable(f_ghz: float, theta_deg: float, catalogue: Optional[List[Waveform]] = None) -> List[Waveform]:
    band = band_for_frequency(f_ghz)
    if band is None or abs(theta_deg) > band.scan_limit_deg(f_ghz):
        return []
    return [w for w in (catalogue if catalogue is not None else CATALOGUE)
            if w.band == band.name and w.role in band.allowed_roles]


# -----------------------------------------------------------------------------
# Solver
# -----------------------------------------------------------------------------
@dataclass
class Result:
    wf: Waveform
    target: Target
    snr_req_db: float
    driver: str
    range_km: float
    losses: Dict[str, float]
    l_tot_db: float
    p_avg_used_w: float
    p_avg_max_w: float
    r_unamb_km: float
    n_cells: float
    frame: Optional[Dict[str, float]]


def solve(wf: Waveform, f_ghz: float, theta_deg: float, target: Target,
          elec_mw: float = FACE_ELEC_POWER_MW) -> Result:
    band = band_for_frequency(f_ghz)
    if band is None or band.name != wf.band:
        raise ValueError(f"{wf.name}: {f_ghz} GHz is not in its {wf.band} band")
    if abs(theta_deg) > band.scan_limit_deg(f_ghz):
        raise ValueError(f"{wf.name}: scan {theta_deg} deg exceeds the {band.scan_limit_deg(f_ghz):.0f} deg "
                         f"limit at {f_ghz} GHz")
    sw = wf.swerling if wf.swerling is not None else target.swerling

    if wf.role in DETECT_ROLES:
        snr_db = detect_snr_req_db(wf.integration, wf.n, sw)
        driver = "Detect-" + wf.integration[:4]
    else:
        snr_lin, dom = tracking_snr_req(wf.tracking_pack, f_ghz, theta_deg, wf.dwell_s, wf.sig_bw_mhz)
        snr_db, driver = lin_to_db(snr_lin), "Track-" + dom

    t_span = wf.dwell_s
    losses: Dict[str, float] = dict(band.losses_db)
    losses["edge"] = band.edge_loss_db(f_ghz)
    losses["scan"] = scan_loss_db(theta_deg)
    if band.pol_loss_db > 0:
        losses["polarisation"] = band.pol_loss_db
    if band.e_buf_trm_j > 0:
        x_buf = band.p_pk_trm_w / PAE * wf.tau_s / band.e_buf_trm_j
        if x_buf > 0.5:
            raise ValueError(f"{wf.name}: pulse needs {x_buf:.2f} of the TRM buffer (max 0.5): droop too large")
        losses["buffer_droop"] = -10.0 * math.log10(1.0 - x_buf / 2.0)
    losses["multibeam_tx"] = multibeam_tx_loss_db(wf.b_tx)
    if ENABLE_GRID_LOSS and wf.role in ("SEARCH", "TWS"):
        losses["grid_tx"] = grid_loss_db()
    if wf.integration == "COHERENT" or wf.role == "TRACK":
        losses["coherence"] = coherence_loss_db(f_ghz, wf.coherent_time_s)
    mig, n_cells = migration_loss_db(target.v_close_mps, t_span, wf.sig_bw_mhz, wf.n)
    losses["migration"] = mig
    l_tot = sum(losses.values())

    g_tx = gain_lin(band.eta_illum_tx, f_ghz, theta_deg)
    g_rx = gain_lin(band.eta_illum_rx, f_ghz, theta_deg)
    p_beam = band.p_pk_total_w / wf.b_tx
    num = p_beam * g_tx * g_rx * lambda_m(f_ghz) ** 2 * target.rcs_m2 * wf.tau_s * wf.n
    den = (4.0 * math.pi) ** 3 * K_BOLTZ * band.t_sys_k * db_to_lin(l_tot) * db_to_lin(snr_db)
    r_km = (num / den) ** 0.25 / 1000.0

    p_avg_used = band.p_pk_total_w * wf.duty
    p_avg_max = face_avg_rf_w(elec_mw)

    frame = None
    if wf.role == "SEARCH":
        pos = n_search_positions(f_ghz)
        t_pos = wf.dwell_s + (0.0 if FRAME_INTERLEAVED else 2.0 * r_km * 1000.0 / C_MPS)
        t_frame = pos * t_pos / wf.b_tx
        frame = {"positions": pos, "dwell_s": wf.dwell_s, "t_pos_s": t_pos, "t_frame_s": t_frame,
                 "ok": 1.0 if t_frame <= REFRESH_S else 0.0}

    return Result(wf, target, snr_db, driver, r_km, losses, l_tot, p_avg_used, p_avg_max,
                  C_MPS * wf.pri_s / 2.0 / 1000.0, n_cells, frame)


# -----------------------------------------------------------------------------
# Output
# -----------------------------------------------------------------------------
def report(f_ghz: float, theta_deg: float, target_name: str, elec_mw: float) -> None:
    band = band_for_frequency(f_ghz)
    tgt = TARGETS[target_name]
    print("=" * 100)
    print(f" RADAR PERFORMANCE MODEL V{MODEL_VERSION}")
    print("=" * 100)
    print(f"Frequency {f_ghz} GHz | scan {theta_deg} deg | target {tgt.name} "
          f"(RCS {tgt.rcs_m2} m2, SW{tgt.swerling}, {tgt.v_close_mps:.0f} m/s) | "
          f"Pd {DETECT_PD}, Pfa {DETECT_PFA:g} | face electrical power {elec_mw} MW")
    if band is None:
        print(f"-> {f_ghz} GHz is outside every band (S {_S_LO}-{_S_HI}, Ku {_KU_LO}-{_KU_HI} GHz). Nothing playable.")
        return
    lim = band.scan_limit_deg(f_ghz)
    print(f"Band {band.name}: {band.n_trm} TRM x {band.p_pk_trm_w:.0f} W = {band.p_pk_total_w/1e6:.2f} MW peak/face | "
          f"scan limit {lim:.0f} deg | T_sys {band.t_sys_k:.1f} K | fixed losses {band.l_fixed_db:.2f} dB")
    print(f"Gain TX {lin_to_db(gain_lin(band.eta_illum_tx, f_ghz, theta_deg)):.2f} dBi | "
          f"RX {lin_to_db(gain_lin(band.eta_illum_rx, f_ghz, theta_deg)):.2f} dBi | "
          f"HPBW {hpbw_deg(f_ghz, theta_deg):.2f} deg | "
          f"avg RF available {face_avg_rf_w(elec_mw)/1e6:.3f} MW")
    if abs(theta_deg) > lim:
        print(f"-> scan angle {theta_deg} deg exceeds the {lim:.0f} deg limit at {f_ghz} GHz. Nothing playable.")
        return
    wfs = playable(f_ghz, theta_deg)
    if not wfs:
        print(f"-> No waveform in the catalogue is playable in {band.name} band "
              f"(roles allowed: {', '.join(band.allowed_roles)}).")
        return

    print("-" * 100)
    for w in wfs:
        r = solve(w, f_ghz, theta_deg, tgt, elec_mw)
        print(f"{w.name} [{w.role}, {w.band}, {w.integration}, SW{w.swerling if w.swerling is not None else tgt.swerling}] "
              f"tau {w.tau_s*1e6:.0f} us, N {w.n}, PRI {w.pri_s*1e3:.1f} ms, B {w.sig_bw_mhz:.0f} MHz, Btx {w.b_tx}")
        print(f"  Required SNR (total energy): {r.snr_req_db:.2f} dB   driver: {r.driver}")
        print(f"  Range on {tgt.name}: {r.range_km:,.0f} km")
        ls = ", ".join(f"{k} {v:.2f}" for k, v in r.losses.items() if v)
        print(f"  Losses (dB): {ls} | total {r.l_tot_db:.2f}")
        print(f"  Migration: {r.n_cells:.0f} range cells crossed over the dwell ({MIGRATION_MODE.lower()})")
        print(f"  Average power: {r.p_avg_used_w/1e3:.1f} kW used of {r.p_avg_max_w/1e3:.0f} kW available "
              f"({100*r.p_avg_used_w/r.p_avg_max_w:.1f} %) | duty {100*w.duty:.2f} %")
        flag = "  <-- RANGE AMBIGUOUS (needs staggered PRI / ambiguity resolution)" if r.range_km > r.r_unamb_km else ""
        print(f"  Unambiguous range (nominal PRI): {r.r_unamb_km:,.0f} km{flag}")
        if r.frame:
            fr = r.frame
            ok = "OK" if fr["ok"] else "NOT FEASIBLE"
            print(f"  Search frame: {fr['positions']:.0f} positions, dwell {fr['dwell_s']*1e3:.0f} ms, "
                  f"frame time {fr['t_frame_s']:.0f} s vs refresh {REFRESH_S:.0f} s -> {ok}")
        print("  Ranges by target: " + " | ".join(
            f"{n} {solve(w, f_ghz, theta_deg, t, elec_mw).range_km:,.0f} km" for n, t in TARGETS.items()))
    print("=" * 100)


# -----------------------------------------------------------------------------
# Self-tests (regression / benchmark guards)
# -----------------------------------------------------------------------------
def selftest() -> int:
    fails = 0

    def check(label: str, cond: bool, detail: str = "") -> None:
        nonlocal fails
        print(f"[{'PASS' if cond else 'FAIL'}] {label} {detail}")
        fails += 0 if cond else 1

    pd, pfa = DETECT_PD, 1e-6   # reference values of the formula checks below (published at Pfa 1e-6)
    alb1 = albersheim_total_snr_db(pd, pfa, 1)
    check("Albersheim steady N=1 = 13.11 dB", abs(alb1 - 13.11) < 0.05, f"({alb1:.2f})")
    sh0 = shnidman_total_snr_db(pd, pfa, 1, 0)
    check("Shnidman SW0 N=1 within 0.3 dB of Albersheim", abs(sh0 - alb1) < 0.3, f"({sh0:.2f})")
    exact_sw1 = 10 * math.log10(math.log(pfa) / math.log(pd) - 1.0)
    sh1 = shnidman_total_snr_db(pd, pfa, 1, 1)
    check("Shnidman SW1 N=1 within 0.5 dB of exact 21.14 dB", abs(sh1 - exact_sw1) < 0.5, f"({sh1:.2f} vs {exact_sw1:.2f})")
    for n in (8, 24, 128):
        a, s = albersheim_total_snr_db(pd, pfa, n), shnidman_total_snr_db(pd, pfa, n, 0)
        check(f"Shnidman steady N={n} within 0.5 dB of Albersheim (total energy)", abs(a - s) < 0.5, f"({s:.2f} vs {a:.2f})")
    check("Coherent SW1 requirement independent of N (regression of V5.4.0 double count)",
          abs(detect_snr_req_db("COHERENT", 128, 1) - shnidman_total_snr_db(DETECT_PD, DETECT_PFA, 1, 1)) < 1e-9)
    check("SW1 penalty about 8 dB over steady at N=1", abs((sh1 - sh0) - 8.0) < 0.5, f"({sh1 - sh0:.2f})")

    check("Scan loss 0 dB at 25 deg", scan_loss_db(25.0) == 0.0)
    check("Scan loss 0.6 dB at 70 deg", abs(scan_loss_db(70.0) - 0.6) < 1e-9)
    check("Scan loss ~0.82 dB at 75 deg", abs(scan_loss_db(75.0) - 0.823) < 0.01, f"({scan_loss_db(75.0):.3f})")
    check("S scan limit 75 deg at 2.8 GHz, 70 deg at 2.88 GHz",
          BAND_S.scan_limit_deg(2.8) == 75.0 and BAND_S.scan_limit_deg(2.88) == 70.0)
    check("S core band has no edge loss; edges 1.5 dB",
          BAND_S.edge_loss_db(2.8) == 0.0 and abs(BAND_S.edge_loss_db(2.7) - 1.5) < 1e-9 and abs(BAND_S.edge_loss_db(2.9) - 1.5) < 1e-9)
    check("Search positions at 2.8 GHz in (u, v), spacing 1.04 HPBW, about 4167 (array tilt 25 deg)", abs(n_search_positions(2.8) - 4167) < 60, f"({n_search_positions(2.8):.0f})")
    check("Chain efficiency 0.917 and avg RF 1.375 MW at 2 MW", abs(CHAIN_EFF - 0.917) < 1e-3 and abs(2e6 * CHAIN_EFF * PAE - 1.375e6) < 3e3)

    check("S 2.8 GHz: WF-LRS playable", [w.name for w in playable(2.8, 0.0)] == ["WF-LRS"])
    check("Ku 14.0 GHz: nothing playable (no Ku waveform in catalogue, no search in Ku)", playable(14.0, 0.0) == [])
    check("5.0 GHz (out of band): nothing playable", playable(5.0, 0.0) == [])
    check("S 2.8 GHz at 80 deg scan: nothing playable", playable(2.8, 80.0) == [])
    for label, kwargs in (("SEARCH in Ku", dict(role="SEARCH", band="Ku")),
                          ("TRACK in S", dict(role="TRACK", band="S", tracking_pack=TrackingPack(0.05, 2.0)))):
        try:
            Waveform(name="X", tau_s=1e-5, n=1, pri_s=1e-3, sig_bw_mhz=10.0, **kwargs)
            check(f"Policy rejects {label}", False)
        except ValueError:
            check(f"Policy rejects {label}", True)

    lrs = solve(WF_LRS, 2.8, 0.0, TARGETS["ST"])
    check("RCS scaling R ~ sigma^0.25 (same speed)",
          abs(solve(WF_LRS, 2.8, 0.0, Target("a", 1000.0, 1, 5000.0)).range_km
              / solve(WF_LRS, 2.8, 0.0, Target("b", 10.0, 1, 5000.0)).range_km - 100 ** 0.25) < 1e-6)
    # Independent re-derivation of the radar equation for the WF-LRS benchmark (no solve() internals)
    lam = C_MPS / 2.8e9
    gt = 4 * math.pi * PHYS_AREA_M2 * 1.0 * ETA_OHMIC / lam ** 2
    gr = 4 * math.pi * PHYS_AREA_M2 * 0.88 * ETA_OHMIC / lam ** 2
    l_db = (2.0 + 0.5 + 0.5 + 0.3 + 0.2 + 0.3 + 0.5)
    l_db += -10 * math.log10(1 - (2000 / 0.75 * 200e-6 / 5.0) / 2) + MB_L8_DB * (2.0 / 3.0) ** MB_P + MIGRATION_RESIDUAL_DB + 1.672 * BEAM_SPACING_HPBW ** 2
    snr = db_to_lin(shnidman_total_snr_db(DETECT_PD, DETECT_PFA, 24, 1))
    r_hand = ((9.8e6 / 4) * gt * gr * lam ** 2 * 1000.0 * 200e-6 * 24
              / ((4 * math.pi) ** 3 * K_BOLTZ * 346.5 * db_to_lin(l_db) * snr)) ** 0.25 / 1000.0
    check("Radar equation matches an independent hand derivation (0.5 %)", abs(lrs.range_km / r_hand - 1) < 0.005,
          f"({lrs.range_km:.1f} vs {r_hand:.1f} km)")
    check("Benchmark WF-LRS on ST at 2.8 GHz in [4500, 6500] km", 4500 < lrs.range_km < 6500, f"({lrs.range_km:.0f} km)")
    check("Grid loss at spacing 0.92 reproduces the 1.41 dB of the notes", abs(grid_loss_db(0.92) - 1.41) < 0.02,
          f"({grid_loss_db(0.92):.3f} dB)")
    import random
    random.seed(7)
    s_, acc, cnt = 1.4, 0.0, 0
    a1, a2 = (s_, 0.0), (s_ / 2.0, s_ * math.sqrt(3.0) / 2.0)
    sites = [(i * a1[0] + j * a2[0], i * a1[1] + j * a2[1]) for i in range(-2, 3) for j in range(-2, 3)]
    while cnt < 60000:
        x, y = random.uniform(-s_, s_), random.uniform(-s_, s_)
        if all(x * x + y * y <= (x - cx) ** 2 + (y - cy) ** 2 for cx, cy in sites):
            acc += 12.0412 * (x * x + y * y)
            cnt += 1
    check("Closed-form grid loss matches Monte-Carlo over the hexagonal cell (0.1 dB)",
          abs(acc / cnt - grid_loss_db(1.4)) < 0.1, f"({acc / cnt:.2f} vs {grid_loss_db(1.4):.2f} dB)")
    check("Positions x cell area = sector area (u, v)", abs(n_search_positions(2.8) * beam_cell_area_uv(2.8) - sector_area_uv()) < 1e-9)
    check("Sector uv area = 2.0006 (disc R=sin75 cut by the az=+/-45 planes, array tilt 25 deg)", abs(sector_area_uv() - 2.0006) < 0.002, f"({sector_area_uv():.4f})")
    _e0, _e1 = SECTOR_EPS_MIN_DEG, SECTOR_EPS_MAX_DEG
    globals()["SECTOR_EPS_MIN_DEG"] = -20.0
    _n_lim = n_search_positions(2.8)
    globals()["SECTOR_EPS_MIN_DEG"], globals()["SECTOR_EPS_MAX_DEG"] = _e0, _e1
    check("Mission rear limit eps_min = -20 deg: ~3600 positions (lattice count of the 3D viewer: 3601)", abs(_n_lim - 3601) < 30, f"({_n_lim:.0f})")
    check("Default eps window (-90, +90) leaves the sector unchanged", abs(sector_area_uv() - 2.0006) < 0.002)
    check("uv count at small scan ~ angular count (cone 15 deg, 1 %)",
          abs(math.pi * math.sin(math.radians(15)) ** 2 / beam_cell_area_uv(2.8) /
              (math.pi * math.radians(15) ** 2 / ((math.sqrt(3) / 2) * (BEAM_SPACING_HPBW * math.radians(hpbw_deg(2.8))) ** 2)) - 1) < 0.03)
    check("Single-pulse coherent span is tau, not PRI",
          abs(Waveform(name="T", role="SEARCH", band="S", tau_s=4e-3, n=1, pri_s=27e-3, sig_bw_mhz=75.0,
                       integration="COHERENT").coherent_time_s - 4e-3) < 1e-12)
    check("WF-LRS dwell = (N-1)*PRI + tau", abs(WF_LRS.dwell_s - (23 * 27e-3 + 200e-6)) < 1e-12)
    try:
        Waveform(name="WIDE", role="CONFIRM", band="Ku", tau_s=1e-4, n=1, pri_s=1e-3, sig_bw_mhz=250.0,
                 integration="COHERENT")
        check("Ku waveform above 200 MHz instantaneous bandwidth is refused", False)
    except ValueError:
        check("Ku waveform above 200 MHz instantaneous bandwidth is refused", True)
    try:
        solve(Waveform(name="LONG", role="SEARCH", band="S", tau_s=4e-3, n=4, pri_s=40e-3, sig_bw_mhz=75.0,
                       b_tx=1, integration="NONCOHERENT"), 2.8, 0.0, TARGETS["ST"])
        check("solve() rejects a pulse exceeding the TRM buffer", False)
    except ValueError:
        check("solve() rejects a pulse exceeding the TRM buffer", True)
    for label, args in (("scan above limit", (WF_LRS, 2.8, 80.0, TARGETS["ST"])),
                        ("S waveform at a Ku frequency", (WF_LRS, 14.0, 0.0, TARGETS["ST"]))):
        try:
            solve(*args)
            check(f"solve() rejects {label}", False)
        except ValueError:
            check(f"solve() rejects {label}", True)

    ku_trk = Waveform(name="TEST-KU-TRK", role="TRACK", band="Ku", tau_s=40e-6, n=128, pri_s=500e-6,
                      sig_bw_mhz=75.0, integration="COHERENT", tracking_pack=TrackingPack(0.05, 2.0))
    check("Synthetic Ku TRACK waveform playable at 14.0 GHz",
          [w.name for w in playable(14.0, 0.0, [ku_trk])] == ["TEST-KU-TRK"])
    check("Synthetic Ku TRACK waveform not playable at 80 deg", playable(14.0, 80.0, [ku_trk]) == [])
    rk = solve(ku_trk, 14.0, 0.0, TARGETS["KKV"])
    check("Synthetic Ku TRACK solve returns a finite positive range", 0 < rk.range_km < 1e6, f"({rk.range_km:.0f} km)")

    print(f"\n{'ALL TESTS PASSED' if fails == 0 else str(fails) + ' TEST(S) FAILED'}")
    return 1 if fails else 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=f"Radar Performance Model V{MODEL_VERSION}")
    ap.add_argument("--freq", type=float, default=FREQUENCY_GHZ, help="frequency in GHz")
    ap.add_argument("--theta", type=float, default=SCAN_ANGLE_DEG, help="scan angle in deg")
    ap.add_argument("--target", default=TARGET_NAME, choices=sorted(TARGETS), help="target for the main range line")
    ap.add_argument("--elec-mw", type=float, default=FACE_ELEC_POWER_MW, help="electrical power of the face (MW)")
    ap.add_argument("--selftest", action="store_true", help="run regression tests")
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    report(args.freq, args.theta, args.target, args.elec_mw)
    return 0


if __name__ == "__main__":
    sys.exit(main())
