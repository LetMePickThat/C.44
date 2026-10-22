#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TRACKING -- confirmation and active tracking (AT) budget, added per face to the S-band search budget of
core.optimizer.run() (called at its end; the result goes under the report key "tracking").

Decisions of 2026-10-19 (tracking plan, steps 1-2):
  - Angle measured on the detection echo itself (fully digital RX, ML estimator on the full aperture):
    sigma per axis = K x HPBW_TX / sqrt(SNR of the echo), K = Cramer-Rao bound of the aperture x margin
    (radar_model.angle_k_tx, step 1.1, ~0.49); residual S/Ku alignment bias confirmation.bias_mrad per axis. The echoes that cross the threshold have a random SNR (Swerling), drawn here by quantiles.
  - Every S detection is confirmed: true targets AND false alarms (radar_model.DETECT_PFA per range-Doppler cell).
    Modes:  KU1+7 = one Ku look on the S estimate, then a 7-beam Ku cluster if it fails (two round trips);
            KU7   = 7-beam Ku cluster at once (one round trip);
            S2    = immediate S touch pointed on the detection (rejects the false alarms in S), then KU1+7 to start
                    the track with the improved angle estimate.
    Each mode gets the smallest power margin (0.5 dB steps) giving a confirmation probability >= CONFIRM_P_MIN.
    The mode is chosen automatically per sector: admissible modes (probability reachable, Ku scan coverage; latency is left to the scheduler, roadmap 4.3),
    then the one that minimises the most loaded face's max(energy, S time, Ku time) over the whole mission; the
    sector key "confirm_mode" forces one.
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

from . import isolation as iso
from . import optimizer as mo
from . import radar_model as rm
from .params import P

# Values from radar_system.toml ([confirmation], [tracking], [band.Ku], [tool.confirmation]).
_T = "tool.confirmation."
KU_FREQ_GHZ = P.get("confirmation.ku_freq_ghz")      # Ku confirmation / tracking frequency
K_ML_S = rm.angle_k_tx("S")                          # S angle accuracy x sqrt(SNR), in S TX beamwidths (CRLB x margin)
BIAS_MRAD = P.get("confirmation.bias_mrad")          # residual S/Ku alignment bias per axis (absolute angle, constant in u, v)
CONFIRM_P_MIN = P.get("confirmation.p_min")          # minimum probability of confirming a true target
CONFIRM_MODES = ("KU1+7", "KU7", "S2")
CLUSTER_N = 7                   # hexagonal cluster: centre + first ring, spacing rm.BEAM_SPACING_HPBW
ANGLE_ZONE_SIGMAS = P.get("tracking.angle_zone_sigmas")     # angular zone opening = full width at +/-2 sigma
MARGIN_DB = np.arange(0.0, P.get(_T + "margin_db_max") + P.get(_T + "margin_db_step") / 2.0,
                      P.get(_T + "margin_db_step"))  # power margins tried on confirmation looks
FA_R4_MEAN = P.get("confirmation.fa_r4_mean")        # a false alarm's Ku look is sized for the threat at the declared
                                                     # range: mean of (R / R_det)^4 over a range uniform in [0, R_det]
# Ku look search space (confirmation and tracking)
KU_TAU = tuple(P.get(_T + "ku_tau_s"))
KU_N = tuple(P.get(_T + "ku_n_pulses"))
KU_PRI = tuple(P.get(_T + "ku_pri_s"))
KU_BW = tuple(P.get(_T + "ku_sig_bw_mhz"))
KU_DUTY_MAX = rm.BAND_KU.duty_max                    # Ku TRM duty ceiling (not in the model yet)
N_SNR_Q = P.get(_T + "n_snr_quantiles")              # quantiles of the detected-echo SNR
GH_NODES, GH_WEIGHTS = np.polynomial.hermite_e.hermegauss(P.get(_T + "gh_nodes"))
GH_WEIGHTS = GH_WEIGHTS / GH_WEIGHTS.sum()
_TX_LOSS_DB_PER_HPBW2 = 10.0 * math.log10(math.e) * 4.0 * math.log(2.0)   # 12.04 dB at 1 HPBW (radar_model.grid_loss_db)
_CLUSTER_XY = np.array([(0.0, 0.0)] + [(rm.BEAM_SPACING_HPBW * math.cos(math.radians(60 * j)),
                                        rm.BEAM_SPACING_HPBW * math.sin(math.radians(60 * j))) for j in range(6)])


# ----------------------------------------------------------------------------- detection statistics
def pd_single(snr, swerling: int):
    """Probability of detection of one look (single sample) at mean SNR snr (linear, array), Pfa = DETECT_PFA.
    Closed forms for SW1/2 (chi2, 2 dof) and SW3/4 (4 dof); steady target by inverting the model's Shnidman fit."""
    snr = np.maximum(np.asarray(snr, dtype=float), 0.0)
    t = -math.log(rm.DETECT_PFA)
    if swerling in (1, 2):
        return np.exp(-t / (1.0 + snr))
    if swerling in (3, 4):
        return (1.0 + 2.0 * t * snr / (2.0 + snr) ** 2) * np.exp(-2.0 * t / (2.0 + snr))
    pds = np.linspace(0.1, 0.99, 90)
    req = np.array([rm.shnidman_total_snr_db(p, rm.DETECT_PFA, 1, 0) for p in pds])
    return np.interp(10.0 * np.log10(np.maximum(snr, 1e-12)), req, pds, left=0.0, right=0.99)


def _snr_ref(swerling: int) -> float:
    """Mean SNR (linear) of a look designed for Pd = DETECT_PD (single sample)."""
    return rm.db_to_lin(rm.shnidman_total_snr_db(rm.DETECT_PD, rm.DETECT_PFA, 1, swerling))


def detected_snr_quantiles(snr_mean: float, swerling: int) -> np.ndarray:
    """SNR of the echoes that crossed the threshold, at mean SNR snr_mean (N_SNR_Q equal-probability quantiles).
    The fluctuation is chi2 with 2 (SW1/2) or 4 (SW3/4) dof, conditioned on detection (probability DETECT_PD)."""
    if swerling == 0:
        return np.full(N_SNR_Q, snr_mean)
    k = 1.0 if swerling in (1, 2) else 2.0
    x = snr_mean * np.linspace(0.0, 40.0, 40001)
    y = k * x / snr_mean
    surv = np.exp(-y) * (1.0 + y) if k == 2.0 else np.exp(-y)
    u = (np.arange(N_SNR_Q) + 0.5) / N_SNR_Q
    return np.interp(rm.DETECT_PD * (1.0 - u), surv[::-1], x[::-1])


def _tx_gain(dx, dy):
    """Two-axis pointing error (in beamwidths) -> TX gain factor (Gaussian main lobe, as the grid loss)."""
    return 10.0 ** (-_TX_LOSS_DB_PER_HPBW2 * (dx * dx + dy * dy) / 10.0)


def _ku_after_s(sigma_ku, bias_ku: float, snr_ku, swerling: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """KU1+7 and KU7 statistics for angle errors of standard deviation sigma_ku per axis (Ku beamwidths, array over
    the leading axes) plus a bias per axis. snr_ku: centre-look SNR (array over margins, last axis).
    Returns (success KU1+7, Ku looks KU1+7, success KU7), each averaged over the Gaussian errors."""
    ex = bias_ku + sigma_ku[..., None, None] * GH_NODES[:, None]            # (..., 12, 1)
    ey = bias_ku + sigma_ku[..., None, None] * GH_NODES[None, :]            # (..., 1, 12)
    w = GH_WEIGHTS[:, None] * GH_WEIGHTS[None, :]
    g = np.stack([_tx_gain(ex - cx, ey - cy) for cx, cy in _CLUSTER_XY], axis=-1)   # (..., 12, 12, 7)
    pd = pd_single(g[..., None] * snr_ku, swerling)                         # (..., 12, 12, 7, M)
    p1 = pd[..., 0, :]
    p7 = 1.0 - np.prod(1.0 - pd, axis=-2)
    s17 = p1 + (1.0 - p1) * p7
    l17 = 1.0 + (1.0 - p1) * CLUSTER_N
    avg = lambda a: np.tensordot(w, a, axes=([0, 1], [-3, -2]))           # noqa: E731
    return avg(s17), avg(l17), avg(p7)


@functools.lru_cache(maxsize=64)
def confirm_tables(swerling: int, f_s_ghz: float) -> dict:
    """Confirmation statistics of the three modes vs the power margins (independent of range and scan angle:
    S and Ku beams widen together on the same face, and the detection always happens at the design SNR)."""
    q = rm.hpbw_deg(f_s_ghz) / rm.hpbw_deg(KU_FREQ_GHZ)                    # S TX beamwidth in Ku TX beamwidths
    x_det = detected_snr_quantiles(_snr_ref(swerling), swerling)          # (M,)
    sig_ku = K_ML_S * q / np.sqrt(x_det)
    bias_ku = BIAS_MRAD * 1e-3 / math.radians(rm.hpbw_deg(KU_FREQ_GHZ))   # in Ku beamwidths (boresight = u, v width)
    m_lin = 10.0 ** (MARGIN_DB / 10.0)
    snr_ku = _snr_ref(swerling) * m_lin                                    # Ku centre-look SNR per margin
    s17, l17, s7 = (a.mean(axis=0) for a in _ku_after_s(sig_ku, bias_ku, snr_ku, swerling))   # (n_margin,)
    # S2: pointed S touch (no grid loss, pointing error in S beams), then KU1+7 on the combined angle estimate
    sig_s, bias_s = sig_ku / q, bias_ku / q
    ex = bias_s + sig_s[:, None, None] * GH_NODES[:, None]
    ey = bias_s + sig_s[:, None, None] * GH_NODES[None, :]
    w = GH_WEIGHTS[:, None] * GH_WEIGHTS[None, :]
    touch_snr = _snr_ref(swerling) * rm.db_to_lin(rm.grid_loss_db() + rm.grid_rx_loss_db()) * m_lin   # (n_ms,)
    pt = pd_single(_tx_gain(ex, ey)[..., None] * touch_snr, swerling)                   # (M, 12, 12, n_ms)
    pt = np.tensordot(w, pt, axes=([0, 1], [1, 2]))                                    # (M, n_ms)
    sig2 = K_ML_S * q / np.sqrt(x_det[:, None] + touch_snr[None, :])                   # (M, n_ms)
    s2_succ = np.zeros((len(m_lin), len(m_lin)))
    s2_looks = np.zeros_like(s2_succ)
    for i in range(len(m_lin)):
        a17, b17, _ = _ku_after_s(sig2[:, i], bias_ku, snr_ku, swerling)                 # (M, n_mku)
        s2_succ[i] = (pt[:, i, None] * a17).mean(axis=0)
        s2_looks[i] = (pt[:, i, None] * b17).mean(axis=0)
    return {"KU1+7": (s17, l17), "KU7": (s7, np.full_like(s7, float(CLUSTER_N))), "S2": (s2_succ, s2_looks)}


# ----------------------------------------------------------------------------- Ku looks
def _ku_integration(swerling: int) -> str:
    return "COHERENT" if swerling in (0, 1, 3) else "NONCOHERENT"


@functools.lru_cache(maxsize=256)
def _ku_candidates(integ: str, pack: Optional[rm.TrackingPack]) -> Tuple[Tuple[rm.Waveform, Optional[rm.Waveform]], ...]:
    """(detection waveform, tracking waveform or None) pairs with the same timing."""
    out = []
    for bw in KU_BW:
        for pri in KU_PRI:
            for n in KU_N:
                for tau in KU_TAU:
                    if tau > KU_DUTY_MAX * pri:
                        continue
                    try:
                        det = rm.Waveform("KU-CONF", "CONFIRM", "Ku", tau, n, pri, bw, 1, integ)
                        trk = None if pack is None else rm.Waveform("KU-TRK", "TRACK", "Ku", tau, n, pri, bw, 1, integ,
                                                                    tracking_pack=pack)
                    except ValueError:
                        continue
                    out.append((det, trk))
    return tuple(out)


@functools.lru_cache(maxsize=1 << 17)
def _ku_reach(w: rm.Waveform, theta: float, tgt: rm.Target, rx_sll_db: float, t_sys_k: Optional[float] = None,
              v_span_mps: Optional[float] = None) -> Tuple[float, str]:
    """Range at full power and SNR driver of one Ku waveform (0 if not playable). tgt: frozen, named "ku" by the
    caller so that sectors sharing a threat share the cache."""
    try:
        r = rm.solve(w, KU_FREQ_GHZ, theta, tgt, rm.FACE_ELEC_POWER_MW, rx_sll_db, t_sys_k, v_span_mps)
        return r.range_km, r.driver
    except ValueError:
        return 0.0, "-"


def ku_look(theta: float, tgt: rm.Target, r_km: float, p_avg_max_w: float,
            pack: Optional[rm.TrackingPack] = None, rx_sll_db: Optional[float] = None,
            t_sys_k: Optional[float] = None, r_min_km: Optional[float] = None,
            v_span_mps: Optional[float] = None) -> Optional[dict]:
    """Cheapest Ku look reaching r_km at scan theta (power backed off to just reach it), for detection only
    (pack None) or for a track update (SNR = max(detection, accuracy)). Cost = max(energy / face average power,
    Ku TX time), as the search objective. None when no candidate reaches r_km. rx_sll_db: RX weighting of the
    volume (None = default option); t_sys_k: the sector's Ku budget T_sys (sky noise, None = cold sky). energy_J is
    electrical (RF / PAE), energy_rf_J the radiated energy. Step 2.1: pulses whose blind range exceeds r_min_km are
    left out; a train whose PRI is below 2 r / c + tau + recovery is range-ambiguous at r_km, so it is staggered and
    pays the mean eclipsing loss. v_span_mps: velocity span of the look's velocity bank (step 2.2; None = 0 to the
    target speed, 0 = velocity known, e.g. a track update)."""
    sll = rm.RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    best = None
    tgt_c = dataclasses.replace(tgt, name="ku")
    for det, trk in _ku_candidates(_ku_integration(tgt.swerling_case(KU_FREQ_GHZ)), pack):
        if r_min_km is not None and rm.blind_range_km(rm.BAND_KU, det.tau_s) > r_min_km + 1e-9:
            continue
        if det.n > 1 and det.pri_s < rm.unambiguous_pri_s(rm.BAND_KU, r_km, det.tau_s):
            det = dataclasses.replace(det, staggered=True)
            trk = None if trk is None else dataclasses.replace(trk, staggered=True)
        r_eff, driver = _ku_reach(det, theta, tgt_c, sll, t_sys_k, v_span_mps)
        if trk is not None:
            r_trk, d_trk = _ku_reach(trk, theta, tgt_c, sll, t_sys_k, v_span_mps)
            if r_trk < r_eff:
                r_eff, driver = r_trk, d_trk.replace("Track-", "")
            else:
                driver = "DET"
        if r_eff < r_km or r_eff <= 0:
            continue
        e_rf = rm.BAND_KU.p_pk_total_w * det.n * det.tau_s * (r_km / r_eff) ** 4
        eta = rm.pae_wf(det)
        e = e_rf / eta                                       # electrical (step 1.3); PAE kept under back-off (3.1)
        t = det.n * det.tau_s
        cost = max(e / p_avg_max_w, t)
        if best is None or cost < best["cost"]:
            best = {"cost": cost, "energy_J": e, "energy_rf_J": e_rf, "pae": eta, "time_s": t, "driver": driver,
                    "wf": det}
    return best


def _wf_label(w: rm.Waveform) -> str:
    return (f"{KU_FREQ_GHZ} GHz {w.integration[:4]} tau {w.tau_s * 1e6:.0f} us x N {w.n} @ PRI {w.pri_s * 1e3:g} ms "
            f"B {w.sig_bw_mhz:.0f} MHz")


# ----------------------------------------------------------------------------- per-sector budget
def _groups(P: np.ndarray, r: dict) -> List[dict]:
    """Scan-angle groups of the sector (optimizer.BIN_DEG wide), with their positions per face, worst angle and the
    search waveform the optimiser chose there."""
    p_tot = P.sum(axis=0)
    by_lo = {round(b["theta_lo_deg"]): b for b in r.get("wf_by_bin", [])}
    out = []
    for lo in range(0, P.shape[1], mo.BIN_DEG):
        idx = [i for i in range(lo, min(lo + mo.BIN_DEG, P.shape[1])) if p_tot[i] > 0]
        b = by_lo.get(round(lo * mo.THETA_BIN_DEG))
        if not idx or b is None:
            continue
        out.append({"theta": (idx[-1] + 1) * mo.THETA_BIN_DEG, "pos": P[:, idx].sum(axis=1), "wf": b["wf"],
                    "power_scale": b["power_scale"]})
    return out


def sector_budget(sec: dict, r: dict, P: np.ndarray, p_avg_max_w: float, ku_sky: Optional[dict] = None) -> dict:
    """Confirmation (all modes) and AT loads per face of one sector, per second, as fractions of the face resources:
    "e" = energy (shared S + Ku), "ts" = S TX time, "tk" = Ku TX time. ku_sky: the sector's Ku sky summary
    (optimizer.sector_sky): every Ku look uses its budget T_sys."""
    tgt = mo.sector_target(sec)                      # class + sector values (step 2.4)
    sw = tgt.swerling_case(KU_FREQ_GHZ)             # closed-form confirmation statistics: nearest Swerling case
    r_det, rr = sec["range_km"], sec["rr_target_s"]
    sll = mo.rx_sll_db(sec)
    f_s = r["wf"]["freq_ghz"]
    groups = _groups(P, r)
    p_face = P.sum(axis=1)
    p_all = float(p_face.sum())
    ku_lim = rm.BAND_KU.scan_limit_deg(KU_FREQ_GHZ)
    flags: List[str] = []
    t_ku = ku_sky["t_sys_budget_k"] if ku_sky is not None else None
    if ku_sky is not None:
        flags.extend(mo.sky_flags(ku_sky, "Ku"))
    r_sat_ku = rm.min_unsaturated_range_km(KU_FREQ_GHZ, 0.0, tgt.rcs_at(KU_FREQ_GHZ))
    r_min_sec = mo.range_min_km(sec)          # shortest range of interest: Ku blind range limit (step 2.1)
    angle_cells = rm.rx_beams_per_position(f_s, sll) * rm.RX_BEAM_SPACING_HPBW ** 2   # independent cells per position
    for b in sec.get("tracking", []):
        if b["r_min_km"] < r_min_sec:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km starts below the sector's shortest range of "
                         f"interest ({r_min_sec:.0f} km, range_min_km): Ku looks only need a blind range below that")
        if b["r_min_km"] < r_sat_ku:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: below {r_sat_ku:.1f} km the Ku echo of "
                         f"{tgt.rcs_at(KU_FREQ_GHZ):g} m2 clips the per-element ADCs (fixed gain; attenuator option at 3.3)")

    # --- confirmation: per scan-angle group, true detections and false alarms per face (per refresh), with the
    #     look costs at the design point (Ku look at the sector's detection range, S touch = the search look
    #     pointed, i.e. without grid loss). A margin m scales the energy; the TX time grows once the waveform is
    #     at full power.
    # true targets + non-threatening objects (step 2.5, E): an object seen by the search is confirmed like a target;
    # its Ku look at its own detection range costs the same as the threat's at r_det (R^4 / sigma is the same)
    n_new = float(sec.get("new_targets_per_rr", 1.0)) + float(sec.get("objects_per_rr", 0.0))
    fa = np.zeros(4)
    cg = []
    ku_missing = 0.0
    for gr in groups:
        w = gr["wf"]
        cells = rm.independent_cells(r_min_sec, r_det, w["sig_bw_mhz"], w["pulses_per_block"],
                                     w["velocity_hypotheses"] * w.get("acceleration_hypotheses", 1),
                                     angle_cells)          # independent cells per position (step 2.2 E1-E5, 2.4 C2)
        fa_g = gr["pos"] * cells * w.get("pfa", rm.DETECT_PFA)    # Pfa per cell of the waveform (step 2.3)
        fa += fa_g
        look = (ku_look(gr["theta"], tgt, r_det, p_avg_max_w, rx_sll_db=sll, t_sys_k=t_ku, r_min_km=r_min_sec,
                        v_span_mps=mo.velocity_span_mps(sec))
                if gr["theta"] <= ku_lim else None)
        if look is None:
            ku_missing += float(gr["pos"].sum())
        e_s_rf = rm.BAND_S.p_pk_total_w * w["n_pulses"] * w["tau_us"] * 1e-6 / w["b_tx"] * gr["power_scale"]
        cg.append({"true": n_new * gr["pos"] / p_all, "fa": fa_g, "e_s_rf": e_s_rf, "e_s": e_s_rf / w["pae"],
                   "ku": look})
    if ku_missing > 0:
        flags.append(f"{100 * ku_missing / p_all:.1f} % of the positions are out of Ku reach (scan limit or range): "
                     f"no Ku confirmation or track there")
    th_ku = max((gr["theta"] for gr in groups if gr["theta"] <= ku_lim), default=0.0)
    held = rm.rx_sll_held_up_to_deg(KU_FREQ_GHZ, sll, th_ku)
    if held < th_ku:
        flags.append(f"Ku RX sidelobes {sll:g} dB held only up to {held:g} deg scan at {KU_FREQ_GHZ} GHz (worst "
                     f"visible sidelobe {rm.sidelobe_visible_db(KU_FREQ_GHZ, th_ku, 'rx', sll):.1f} dB at {th_ku:g} deg: "
                     f"TRSA gap lobes)")

    def ku_cost(looks_true: float, looks_fa: float, m: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Electrical energy, Ku TX time and heat (electrical - RF energy) per face, per refresh."""
        e, t, q = np.zeros(4), np.zeros(4), np.zeros(4)
        for c in cg:
            lk = c["ku"]
            if lk is None:
                continue
            bo = lk["energy_rf_J"] / (rm.BAND_KU.p_pk_total_w * lk["time_s"])    # power back-off of the look
            n_eq = c["true"] * looks_true + c["fa"] * looks_fa * FA_R4_MEAN     # looks, in design-range equivalents
            e += n_eq * lk["energy_J"] * m
            q += n_eq * (lk["energy_J"] - lk["energy_rf_J"]) * m
            t += (c["true"] * looks_true * max(1.0, bo * m) + c["fa"] * looks_fa * max(1.0, bo * m * FA_R4_MEAN)) * lk["time_s"]
        return e, t, q

    def s_touch_cost(m: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        n_c = [(c["true"] + c["fa"] * FA_R4_MEAN) * m for c in cg]
        e = sum((n * c["e_s"] for n, c in zip(n_c, cg)), np.zeros(4))
        e_rf = sum((n * c["e_s_rf"] for n, c in zip(n_c, cg)), np.zeros(4))
        return e, e_rf / rm.BAND_S.p_pk_total_w, e - e_rf                         # one beam at full peak power

    tables = confirm_tables(sw, f_s)
    m_lin = 10.0 ** (MARGIN_DB / 10.0)
    modes: Dict[str, dict] = {}
    for mode in CONFIRM_MODES:
        succ, looks = tables[mode]
        why = []
        if ku_missing > 0:
            why.append("part of the sector is out of Ku reach")
        ok = np.argwhere(succ >= CONFIRM_P_MIN)
        if ok.size == 0:
            why.append(f"confirmation probability < {CONFIRM_P_MIN} even with {MARGIN_DB[-1]:.0f} dB margin")
            modes[mode] = {"admissible": False, "why_not": "; ".join(why)}
            continue
        best = None
        for ij in ok:
            if mode == "S2":
                i, j = ij
                e_s, ts, q_s = s_touch_cost(m_lin[i])
                e_k, tk, q_k = ku_cost(float(looks[i, j]), 0.0, m_lin[j])
                e, q = e_s + e_k, q_s + q_k
                info = {"margin_db": [float(MARGIN_DB[i]), float(MARGIN_DB[j])], "success": float(succ[i, j]),
                        "ku_looks_true": float(looks[i, j]), "ku_looks_fa": 0.0}
            else:
                j = int(ij[0])
                l_fa = float(CLUSTER_N + (1 if mode == "KU1+7" else 0))
                e, tk, q = ku_cost(float(looks[j]), l_fa, m_lin[j])
                ts = np.zeros(4)
                info = {"margin_db": float(MARGIN_DB[j]), "success": float(succ[j]),
                        "ku_looks_true": float(looks[j]), "ku_looks_fa": l_fa}
            load = max(float((e / p_avg_max_w).max()), float(ts.max()), float(tk.max()))
            if best is None or load < best[0]:
                best = (load, info, e, ts, tk, q)
            if mode != "S2":
                break                                      # smallest margin meeting the probability
        _, info, e, ts, tk, q = best
        modes[mode] = {**info, "admissible": not why, "why_not": "; ".join(why),
                       "e": e / p_avg_max_w / rr, "ts": ts / rr, "tk": tk / rr, "q_w": q / rr}

    # --- active tracking per bracket
    at = []
    for b in sec.get("tracking", []):
        pack = rm.TrackingPack(b["angle_zone_deg"] / ANGLE_ZONE_SIGMAS, b["vr_acc_mps"], b["range_acc_m"])
        e_face, t_face, q_face = np.zeros(4), np.zeros(4), np.zeros(4)
        drivers: Dict[str, float] = {}
        wf_main, pos_main, unreachable = None, -1.0, 0.0
        for gr in groups:
            look = (ku_look(gr["theta"], tgt, b["r_max_km"], p_avg_max_w, pack, sll, t_ku, max(b["r_min_km"], r_min_sec),
                            v_span_mps=0.0)                      # track update: velocity known (step 2.2)
                    if gr["theta"] <= ku_lim else None)
            share = gr["pos"] / p_all * b["n_targets"]                       # tracks per face in this group
            if look is None:
                unreachable += float(share.sum())
                continue
            e_face += share * look["energy_J"]
            t_face += share * look["time_s"]
            q_face += share * (look["energy_J"] - look["energy_rf_J"])
            drivers[look["driver"]] = drivers.get(look["driver"], 0.0) + float(share.sum())
            if gr["pos"].sum() > pos_main:
                wf_main, pos_main = look["wf"], float(gr["pos"].sum())
        n_tr = float(b["n_targets"])
        out_b = {**b, "sigma_theta_deg": pack.sigma_theta_deg,
                 "driver": max(drivers, key=drivers.get) if drivers else "-",
                 "energy_per_update_J": float(e_face.sum() / n_tr) if n_tr > 0 else 0.0,
                 "avg_power_kW": float(e_face.sum() / b["rr_s"] / 1e3),
                 "wf": _wf_label(wf_main) if wf_main is not None else "-",
                 "e": e_face / p_avg_max_w / b["rr_s"], "tk": t_face / b["rr_s"], "q_w": q_face / b["rr_s"],
                 "unreachable_tracks": unreachable}
        if unreachable > 0:
            flags.append(f"AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: {unreachable:.1f} track(s) out of Ku reach "
                         f"at {b['r_max_km']:.0f} km (scan limit or range)")
        at.append(out_b)
    # S harmonic in the Ku channel on the same face (isolation matrix: frequency rule) -> a scheduler rule
    rules = []
    if iso.same_face_s_to_ku_verdict() == "frequency_rule":
        ku_bws = sorted({c["ku"]["wf"].sig_bw_mhz for c in cg if c["ku"] is not None})
        for w in r.get("wf_set", [r["wf"]]):
            hit = [bw for bw in ku_bws if iso.harmonic_conflict(w["freq_ghz"], w["sig_bw_mhz"], KU_FREQ_GHZ, bw)]
            if hit:
                bands = ", ".join(f"{a:.3f}-{b:.3f}" for a, b in iso.harmonic_bands_ghz(w["freq_ghz"], w["sig_bw_mhz"]))
                rules.append(f"{w['name']} ({w['freq_ghz']} GHz, {w['sig_bw_mhz']:g} MHz) puts its harmonic on {bands} GHz, "
                             f"over the Ku channel {KU_FREQ_GHZ} GHz / {max(hit):g} MHz: on the same face, no Ku reception "
                             f"during this S transmission")
    return {"faces": [k for k in range(4) if p_face[k] > 0], "false_alarms_per_rr": fa, "modes": modes, "at": at,
            "flags": flags, "ku_sky": ku_sky, "ku_min_unsaturated_range_km": r_sat_ku, "scheduler_rules": rules}


# ----------------------------------------------------------------------------- mission budget
_RES = ("e", "ts", "tk")
_RES_NAME = {"e": "energy", "ts": "S time on air", "tk": "Ku time on air"}


def _face_loads(base: Dict[str, np.ndarray], sbs: Dict[str, dict], choice: Dict[str, str]) -> Dict[str, np.ndarray]:
    tot = {k: v.copy() for k, v in base.items()}
    for sid, sb in sbs.items():
        mode = choice.get(sid)
        if mode is not None:
            for key in _RES:
                tot[key] += sb["modes"][mode][key]
        for b in sb["at"]:
            tot["e"] += b["e"]
            tot["tk"] += b["tk"]
    return tot


def _max_load(tot: Dict[str, np.ndarray]) -> float:
    return float(max(tot[k].max() for k in _RES))


def budget(mission: dict, res: dict) -> dict:
    """Mission confirmation + AT budget on top of an optimizer.run() result: per sector the confirmation mode
    (with the reason) and the AT cost per bracket, per face the search / confirmation / AT shares and the verdict,
    and what to relax when a face does not fit."""
    g = res["global"]
    p_avg_max = g["face_elec_avail_kW"] * 1e3
    sph = mo._sphere(g["eps_min_deg"], g["eps_max_deg"])
    secs = {s["id"]: s for s in mission["sectors"]}
    base = {"e": np.zeros(4), "ts": np.zeros(4), "tk": np.zeros(4)}
    for k, v in res["faces_total"].items():
        base["e"][int(k)] = v["energy_fraction"]
        base["ts"][int(k)] = v["time_fraction"]
    sky = mission.get("global", {}).get("sky")
    sbs: Dict[str, dict] = {}
    for r in res["sectors"]:
        if r["status"] != "OK":
            continue
        sec = secs[r["id"]]
        m = sph.mask(sec)
        P, _, _, _ = mo.search_positions(sec, sph, m, r["wf"]["freq_ghz"], sky)      # blinded positions left out
        ku_sky, _ = mo.sector_sky(sec, sph, m, KU_FREQ_GHZ, "Ku", sky)
        sbs[r["id"]] = sector_budget(sec, r, np.array([P[k] for k in range(4)]), p_avg_max, ku_sky)

    # confirmation mode: forced, or automatic (greedy on the mission's most loaded face, two passes)
    choice: Dict[str, Optional[str]] = {}
    forced: Dict[str, bool] = {}
    for sid, sb in sbs.items():
        want = secs[sid].get("confirm_mode", "auto")
        forced[sid] = want in CONFIRM_MODES
        adm = [m for m in CONFIRM_MODES if sb["modes"][m]["admissible"]]
        if forced[sid]:
            choice[sid] = want if "e" in sb["modes"][want] else (adm[0] if adm else None)
        else:
            choice[sid] = min(adm, key=lambda m: max(sb["modes"][m][k].max() for k in _RES)) if adm else None
    for _ in range(2):
        for sid, sb in sbs.items():
            if forced[sid]:
                continue
            best = None
            for m in CONFIRM_MODES:
                if not sb["modes"][m]["admissible"]:
                    continue
                load = _max_load(_face_loads(base, sbs, {**choice, sid: m}))
                own = max(sb["modes"][m][k].max() for k in _RES)
                if best is None or (load, own) < best[0]:
                    best = ((load, own), m)
            if best is not None:
                choice[sid] = best[1]
    tot = _face_loads(base, sbs, choice)

    # per-sector report
    out_secs, warn = [], []
    for sid, sb in sbs.items():
        mode = choice[sid]
        alts = {}
        for m in CONFIRM_MODES:
            md_ = sb["modes"][m]
            alts[m] = {"admissible": md_["admissible"], "why_not": md_["why_not"]}
            if "e" in md_:
                alts[m]["mission_max_load"] = round(_max_load(_face_loads(base, sbs, {**choice, sid: m})), 4)
        if mode is None:
            reason = "no admissible confirmation mode"
            warn.append(f"sector {sid}: no admissible confirmation mode ({'; '.join(sb['modes'][m]['why_not'] for m in CONFIRM_MODES)})")
        elif forced[sid] and mode == secs[sid].get("confirm_mode"):
            reason = "forced by the operator"
            if not sb["modes"][mode]["admissible"]:
                warn.append(f"sector {sid}: forced confirmation mode {mode} breaks a constraint ({sb['modes'][mode]['why_not']})")
        elif forced[sid]:
            reason = f"forced mode {secs[sid]['confirm_mode']} impossible ({sb['modes'][secs[sid]['confirm_mode']]['why_not']})"
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
            "id": sid, "confirm_mode": mode, "reason": reason, "forced": forced[sid],
            "false_alarms_per_rr": round(float(sb["false_alarms_per_rr"].sum()), 1),
            "confirmation": conf, "alternatives": alts,
            "at": [{**{k: v for k, v in b.items() if k not in ("e", "tk", "q_w")},
                    "faces": {str(k): {"energy_fraction": float(b["e"][k]), "ku_time_fraction": float(b["tk"][k])}
                              for k in sb["faces"]}} for b in sb["at"]],
            "ku_sky": sb["ku_sky"], "ku_min_unsaturated_range_km": sb["ku_min_unsaturated_range_km"],
            "scheduler_rules": sb["scheduler_rules"],
            "flags": sb["flags"]})
        for fl in sb["flags"]:
            warn.append(f"sector {sid}: {fl}")

    # per-face totals, verdict and what to relax
    faces, advice = {}, []
    for k in range(4):
        conf_e = sum(sbs[s]["modes"][choice[s]]["e"][k] for s in sbs if choice[s] is not None)
        at_e = sum(b["e"][k] for s in sbs for b in sbs[s]["at"])
        conf_ts = sum(sbs[s]["modes"][choice[s]]["ts"][k] for s in sbs if choice[s] is not None)
        conf_tk = sum(sbs[s]["modes"][choice[s]]["tk"][k] for s in sbs if choice[s] is not None)
        at_tk = sum(b["tk"][k] for s in sbs for b in sbs[s]["at"])
        heat = float(res["faces_total"][str(k)].get("dissipation_kW", 0.0) * 1e3
                     + sum(sbs[s]["modes"][choice[s]]["q_w"][k] for s in sbs if choice[s] is not None)
                     + sum(b["q_w"][k] for s in sbs for b in sbs[s]["at"]))
        load = {key: float(tot[key][k]) for key in _RES}
        worst = max(load, key=load.get)
        faces[str(k)] = {"search_energy": float(base["e"][k]), "confirm_energy": float(conf_e), "at_energy": float(at_e),
                         "total_energy": load["e"], "search_s_time": float(base["ts"][k]),
                         "confirm_s_time": float(conf_ts), "total_s_time": load["ts"],
                         "confirm_ku_time": float(conf_tk), "at_ku_time": float(at_tk), "total_ku_time": load["tk"],
                         "load": load[worst], "binding": _RES_NAME[worst],
                         "status": "OK" if load[worst] <= 1.0 else "OVER-SUBSCRIBED",
                         "p_left_kW": max(0.0, 1.0 - load["e"]) * p_avg_max / 1e3,
                         "dissipation_kW": heat / 1e3, "dissipation_kW_m2": heat / 1e3 / rm.TOTAL_AREA_M2}
        if load[worst] > 1.0:
            advice.append(_relax_advice(k, worst, load[worst], res, sbs, choice, secs))
    for a in advice:
        warn.append(a)
    rules = ([f"search lattice shifted every frame (grid_dithering, step 2.3): the cumulative detection criterion "
              f"draws the target position in its cell anew at each revisit"] if rm.GRID_DITHERING else [])
    rules += iso.scheduler_rules() + [f"sector {s['id']}: {t}" for s in out_secs for t in s["scheduler_rules"]]
    return {"ku_freq_ghz": KU_FREQ_GHZ, "pfa": rm.DETECT_PFA, "confirm_p_min": CONFIRM_P_MIN,
            "sectors": out_secs, "faces": faces, "advice": advice, "scheduler_rules": rules, "warnings": warn}


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
