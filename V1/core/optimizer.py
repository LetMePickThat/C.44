#!/usr/bin/env python3
"""
OPTIMIZER (core/optimizer.py, ex mission_optimizer.py) V1.0 -- pre-mission waveform / budget optimiser for the S-band surveillance (search) function.

Input  : mission file (JSON or CSV) = global parameters + any number of watch sectors.
Output : per sector -> suggested WF, achieved range, RR estimate, radar budget (time, energy, average power, per face);
         mission totals per face; infeasibility flags with the missing power.

Uses core/radar_model.py (radar equation, losses, lattice, face geometry). Ship frame: Z = nose axis (+Z radar faces),
phi = azimuth about Z from +X toward +Y, eps = elevation from XY plane toward +Z. Faces at phi = 0/90/180/270, tilt 25 deg.

Design rules (flagged [D] = decision, change on request) -- optimiser v2 (AUDIT/SPEC_optimiseur_v2.md,
with the audit corrections):
  - [D] One waveform per BIN_DEG scan-angle bin (at most max_wf_per_sector distinct waveforms per sector, neighbouring
        bins merged by dynamic programming), each reaching the requested range in every 1-deg bin it covers; the pulse
        is then shortened per 1-deg bin to just reach the range ("no excess energy", step 3.1 C: PAs saturated, PAE of
        the shortened pulse, floor tau B >= waveform.pulse_tb_min).
  - [D] Objective = min max(energy fraction, time-on-air fraction) over the faces, via an alpha-swept scalarisation.
        Ranking: extra unscanned fraction caused by the frequency, then positions short of range (1 % tolerance),
        then the objective.
  - Range vs scan angle: in the model R(theta) = R(0) h(f, theta) with h independent of the waveform (checked at
    run time), so each candidate costs one solve() per frequency.
  - [D] Unambiguous range imposed on PRI (PRI >= 2 R / c) unless sector says "allow_ambiguous": true.
  - [D] Migration evaluated at v_close_max_mps; Pd/Pfa/loss budget are the model's locked values.
  - Step 1.4: a candidate frequency outside the S band is skipped, and a waveform whose spectrum f +/- B/2 leaves the
    band is not playable there (at the band edges, e.g. 2.7 / 2.9 GHz today, no signal fits).
  - RX weighting per sector (key "rx_sll_db", an option of radar_model.RX_SLL_OPTIONS_DB, default -25 dB): sets
    eta_illum_rx, so the range of every candidate; the report gives the RX beams formed per position (step 4.4).
  - Step 1.2 alerts per sector (flags): active-mismatch loss above tool.alerts.mismatch_db_per_pass in a populated
    scan bin, and RX sidelobe option not held up to the sector's widest scan (far / gap lobes of the visible region).
  - [D] Sectors are independent; overlapping sectors are each budgeted in full there (flagged, not deduplicated --
        merging distinct RCS/range/RR needs into one search is generally costlier than paying for each separately,
        since it has to meet the hardest of both at once). A sector spanning several faces is budgeted per face
        (each face has its own 2 MW).
  - [D] RR is stretched proportionally when a face is over-subscribed:
        RR_est = RR_target x max(1, sum of energy fractions, sum of time fractions).
  - TX multibeam (step 3.1, C): one beam, phase-only superposition (B beams, P / B each, table loss) or aperture
    partition (halves / quadrants, P / B and G N_sub / N per beam, larger cell); the time multiplex is modelled but
    not searched (tool.optimizer.tx_modes).
  - Energy = ELECTRICAL (step 1.3): a position costs P_pk n tau x trains / positions per dwell / PAE, the face has
    radar_model.face_elec_avail_w(face_elec_mw); the face heat (electrical - RF) is reported for step 4.2.
  - Tracking margin per face: the only S/Ku coupling is the face's electrical power, so the power left after search,
    (1 - sum of energy fractions) x P_elec, is reported with the equivalent Ku duty cycle (x Ku pae_sat / Ku peak).
  - Confirmation + active tracking (AT): core/tracking.py adds, per face, the cost of confirming every detection
    (true targets and false alarms, mode chosen per sector) and of the AT range brackets of each sector, and says
    whether search + confirmation + AT fit (report key "tracking").

Performance:
  - solve() range results are memoised per process (_range_cached), so repeated runs (e.g. re-running from the GUI
    after editing one sector) and sectors sharing a threat (same RCS / Swerling / speed / range) reuse them.
  - Sectors are optimised in parallel worker PROCESSES (not threads: the search is pure-Python CPU work, and threads
    would serialise on the GIL) once the mission has PARALLEL_MIN_SECTORS sectors or more. The pool is persistent
    and lazily started, so its start-up cost (~1-2 s on Windows) is paid once per session, and each worker keeps
    its own solve() cache and sphere raster between runs.
"""
import csv
import dataclasses
import functools
import json
import math
import os
import sys
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from concurrent.futures.process import BrokenProcessPool
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import detection
from . import environment as env
from . import frames
from . import radar_model as rm
from . import sky as skym
from .params import P

# Values from radar_system.toml ([geometry], [tool.optimizer], [tool.mission_defaults]).
_T = "tool.optimizer."
FACE_PHI = tuple(P.get("geometry.face_phi_deg"))
D_RES_DEG = P.get(_T + "sphere_res_deg")             # raster step (deg) on the (phi, eps) sphere
FREQ_STEP_GHZ = P.get(_T + "freq_step_mhz") / 1e3      # S centre grid derived from the band (step 3.1, A1-A2)
FREQ_COARSE_GHZ = P.get(_T + "freq_coarse_mhz") / 1e3
FREQ_FINE_SPAN_GHZ = P.get(_T + "freq_fine_span_mhz") / 1e3
EXCLUDED_FLAG_FRAC = P.get(_T + "excluded_flag_frac")
TIE_REL_TOL = P.get(_T + "tie_rel_tol")
if FREQ_COARSE_GHZ < FREQ_STEP_GHZ or abs(FREQ_COARSE_GHZ / FREQ_STEP_GHZ - round(FREQ_COARSE_GHZ / FREQ_STEP_GHZ)) > 1e-9:
    raise ValueError(f"{P.path}: tool.optimizer.freq_coarse_mhz must be a multiple of freq_step_mhz")
# Pulse lengths of the file below the TRM buffer limit, plus the limit itself (derived: 2.0 ms since step 1.3, follows
# the buffer, the PAE and the TRM peak power)
TAU_CAND = tuple(sorted({t for t in P.get(_T + "tau_s") if t < rm.tau_max_s(rm.BAND_S)}
                        | ({rm.tau_max_s(rm.BAND_S)} if math.isfinite(rm.tau_max_s(rm.BAND_S)) else set())))
N_CAND = tuple(P.get(_T + "n_pulses"))
TX_MODES = tuple(P.get(_T + "tx_modes"))             # S TX multibeam modes searched (step 3.1, C)
if not set(TX_MODES) <= set(rm.TX_MODES):
    raise ValueError(f"{P.path}: tool.optimizer.tx_modes {TX_MODES}: modes are {rm.TX_MODES}")
SIGBW_CAND = tuple(P.get(_T + "sig_bw_mhz"))
PRI_MIN_S = P.get(_T + "pri_min_s")                  # PRI of single-pulse and range-ambiguous candidates
RES_MAX_M_DEFAULT = P.get("tool.mission_defaults.res_max_m")   # range resolution requested if the sector gives none
RANGE_MIN_FRAC = P.get("tool.mission_defaults.range_min_frac") # sector range_min_km when absent, x range_km (step 2.1)
FA_PER_RR_DEFAULT = P.get("tool.mission_defaults.sector.fa_per_rr")   # false alarms per sector and revisit (2.3)
THETA_BIN_DEG = P.get(_T + "theta_bin_deg")
C = rm.C_MPS
SOLVE_CACHE_SIZE = P.get(_T + "solve_cache_size")    # memoised solve() ranges per process (~50 MB when full)
PARALLEL_MIN_SECTORS = P.get(_T + "parallel_min_sectors")   # below this, a sequential run (~0.2 s/sector) beats start-up
# 0 = automatic: ~physical cores; each worker holds a ~65 MB sphere raster
MAX_WORKERS = P.get(_T + "max_workers") or max(1, (os.cpu_count() or 2) // 2)
BIN_DEG = P.get(_T + "bin_deg")                      # scan-angle bins (in 1-deg bins) sharing one waveform
MAX_WF_PER_SECTOR = P.get(_T + "max_wf_per_sector")  # default cap on distinct waveforms per sector (sector key "max_wf_per_sector")
SHORT_TOL = P.get(_T + "short_tol")                  # positions-short-of-range fractions within it rank equal (sector key "short_tol")
ALPHAS = np.linspace(0.0, 1.0, P.get(_T + "n_alphas"))   # energy weight of the scalarised energy/time cost
SHORT_PENALTY = P.get(_T + "short_penalty")          # DP penalty per fraction of positions short of range (ranks range before cost)
TRACK_MARGIN_WARN = P.get(_T + "track_margin_warn")  # warn when a face keeps less than this fraction of its power for tracking
SKY_ABOVE_BUDGET_ALERT_DB = P.get("tool.alerts.sky_above_budget_db")   # flag positions this far above the sky budget
RR_T0_RATIO = P.get("tool.alerts.rr_t0_ratio")       # flag a revisit below this x the target T0 (step 2.4, B4)
SURFACE_GUARD_KM = P.get("tool.mission_defaults.sector.surface_guard_km")   # step 3.1, P4 Q9 (sector key)
SURFACE_CLASS_STEP = P.get("tool.environment.surface_class_step_frac")     # step 3.1, P4 R2 (c2)
B_REF_MHZ = 1.0      # bandwidth of the tabulated clutter ratio: sigma_c is linear in the range cell c / 2B (exact)


# ----------------------------------------------------------------------------- S frequency grid (step 3.1, A)
def snap(x):
    """Costs rounded to TIE_REL_TOL (relative): equal costs computed through different paths compare equal, and the
    first candidate (simplest, by the order of the search space) wins the tie (E6)."""
    m, e = np.frexp(np.asarray(x, dtype=float))
    q = 2.0 ** max(1, int(round(-math.log2(TIE_REL_TOL))))
    out = np.ldexp(np.round(m * q) / q, e)
    return float(out) if np.ndim(out) == 0 else out


def exclusions_s(sec: dict) -> Tuple[Tuple[float, float], ...]:
    """The S spectrum the sector must not touch (GHz): sector key spectrum_exclusions_ghz, a copy of the mission's
    global "spectrum_exclusions_ghz"["S"] made by with_spectrum()."""
    return tuple((float(a), float(b)) for a, b in sec.get("spectrum_exclusions_ghz", ()))


def with_spectrum(mission: dict) -> dict:
    """The mission with its global S exclusions copied into every sector (sector-level functions see them)."""
    ex = (mission.get("global", {}).get("spectrum_exclusions_ghz") or {}).get("S")
    if not ex:
        return mission
    out = dict(mission)
    out["sectors"] = [dict(s, spectrum_exclusions_ghz=[list(x) for x in ex]) for s in mission.get("sectors", [])]
    return out


def signal_fits(f: float, bw_mhz: float, excl: Tuple[Tuple[float, float], ...], band=None) -> bool:
    """[f - B/2, f + B/2] inside the usable band (step 1.4 hard rule) and touching no exclusion (step 3.1, A2)."""
    band = band or rm.BAND_S
    lo, hi = f - bw_mhz / 2e3, f + bw_mhz / 2e3
    if lo < band.usable_lo_ghz - 1e-12 or hi > band.usable_hi_ghz + 1e-12:
        return False
    return all(hi <= a + 1e-12 or lo >= b - 1e-12 for a, b in excl)


def centre_grid(band=None) -> List[float]:
    """Every centre of the derived grid: usable_lo + k x step, up to usable_hi."""
    band = band or rm.BAND_S
    n = int(math.floor((band.usable_hi_ghz - band.usable_lo_ghz) / FREQ_STEP_GHZ + 1e-9))
    return [round(band.usable_lo_ghz + k * FREQ_STEP_GHZ, 9) for k in range(n + 1)]


def free_fragments(excl: Tuple[Tuple[float, float], ...], band=None) -> List[Tuple[float, float]]:
    band = band or rm.BAND_S
    out, lo = [], band.usable_lo_ghz
    for a, b in sorted(excl):
        if a > lo:
            out.append((lo, min(a, band.usable_hi_ghz)))
        lo = max(lo, b)
    if lo < band.usable_hi_ghz:
        out.append((lo, band.usable_hi_ghz))
    return [(a, b) for a, b in out if b > a]


def valid_centres(bw_min_mhz: float, excl) -> List[float]:
    """Centres where at least the narrowest signal of the search space fits."""
    return [f for f in centre_grid() if signal_fits(f, bw_min_mhz, excl)]


def coarse_centres(bw_min_mhz: float, excl) -> List[float]:
    """First pass (A5-A6): the valid centres on the coarse step, plus the valid centre nearest the middle of every
    free fragment."""
    valid = valid_centres(bw_min_mhz, excl)
    lo, k = rm.BAND_S.usable_lo_ghz, int(round(FREQ_COARSE_GHZ / FREQ_STEP_GHZ))
    out = {f for f in valid if int(round((f - lo) / FREQ_STEP_GHZ)) % k == 0}
    for a, b in free_fragments(excl):
        inside = [f for f in valid if a <= f <= b]
        if inside:
            out.add(min(inside, key=lambda f: (abs(f - (a + b) / 2.0), f)))
    return sorted(out)


def fine_centres(f_best: float, bw_min_mhz: float, excl, done) -> List[float]:
    """Second pass: the valid centres within +/- FREQ_FINE_SPAN_GHZ of the best coarse one, not yet evaluated."""
    return [f for f in valid_centres(bw_min_mhz, excl)
            if abs(f - f_best) <= FREQ_FINE_SPAN_GHZ + 1e-9 and f not in done]


def validate_frequencies(sec: dict, excl) -> List[str]:
    """Forced centres (sector key freq_ghz_candidates, A9): in the S band, and at least one where a signal of the
    search space fits outside the exclusions."""
    fs = sec.get("freq_ghz_candidates")
    if fs is None:
        return []
    if not isinstance(fs, (list, tuple)) or not fs or not all(isinstance(f, (int, float)) for f in fs):
        return ["freq_ghz_candidates must be a list of frequencies (GHz)"]
    out = [f"freq_ghz_candidates: {f:g} GHz is outside the S band" for f in fs if not rm.BAND_S.contains(f)]
    if not out and not any(signal_fits(f, min(SIGBW_CAND), excl) for f in fs):
        out.append("freq_ghz_candidates: no signal of the search space fits at these frequencies (band edges or "
                   "spectrum exclusions)")
    return out


# ----------------------------------------------------------------------------- geometry
SECTOR_GEOMETRY_FIELDS = ("shape", "phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg",
                          "phi0_deg", "eps0_deg", "radius_deg", "frame_m", "frame_eps_offset_deg")
MASK_CACHE_SIZE = P.get(_T + "mask_cache_size")   # sector masks kept per sphere raster (the same sector is masked several times per run)


def sector_geometry_key(sec: dict) -> tuple:
    """Everything a sector's mask depends on (cache key of masks and 3D meshes)."""
    return tuple(sec.get(f) for f in SECTOR_GEOMETRY_FIELDS)


def face_frame(phi_deg: float) -> Tuple[Tuple[float, float, float], ...]:
    """(n, e_u, e_v) of the face at azimuth phi_deg, tilted by SECTOR_TILT_DEG toward +Z:
    direction d = u e_u + v e_v + w n."""
    t = math.radians(rm.SECTOR_TILT_DEG)
    c, s = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return ((math.cos(t) * c, math.cos(t) * s, math.sin(t)), (-s, c, 0.0),
            (-math.sin(t) * c, -math.sin(t) * s, math.cos(t)))


class Sphere:
    """Raster of the direction sphere (phi, eps), cell-centred, with for each direction its owner face (zone of
    minimal scan: boundaries = the az = +/-45 deg meridian planes, cut by the scan-limit cone), w = cos(theta) and
    theta. The optimiser uses it at D_RES_DEG; the 3D views and the fast estimate use a coarser one
    (core.geometry.SphereGrid), so every part of the tool sees the same faces and sectors."""

    def __init__(self, eps_min: float = -90.0, eps_max: float = 90.0, res_deg: float = D_RES_DEG):
        self.n_phi, self.n_eps = int(round(360.0 / res_deg)), int(round(180.0 / res_deg))
        phi = (np.arange(self.n_phi) + 0.5) * (360.0 / self.n_phi)
        eps = -90.0 + (np.arange(self.n_eps) + 0.5) * (180.0 / self.n_eps)
        self.phi, self.eps = np.meshgrid(phi, eps, indexing="ij")
        self.d_omega = math.radians(360.0 / self.n_phi) * math.radians(180.0 / self.n_eps) * np.cos(np.radians(self.eps))
        p, e = np.radians(self.phi), np.radians(self.eps)
        dx, dy, dz = np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)
        t = math.radians(rm.SECTOR_TILT_DEG)
        sin_lim = math.sin(math.radians(rm.SECTOR_RADIUS_DEG))
        self.owner = -np.ones(self.phi.shape, dtype=np.int8)
        self.w = np.zeros(self.phi.shape)
        for k, pk in enumerate(FACE_PHI):
            n, eu, ev = face_frame(pk)
            w = dx * n[0] + dy * n[1] + dz * n[2]
            u = dx * eu[0] + dy * eu[1]
            v = dx * ev[0] + dy * ev[1] + dz * ev[2]
            ok = (w > 0) & (np.abs(u) <= w * math.cos(t) - v * math.sin(t) + 1e-12) & \
                 (np.sqrt(u * u + v * v) <= sin_lim) & (w > self.w)
            self.owner[ok] = k
            self.w[ok] = w[ok]
        self.theta = np.degrees(np.arccos(np.clip(self.w, 0.0, 1.0)))
        self.scan_zone = self.scanned(eps_min, eps_max)
        self.window = (eps_min, eps_max, res_deg)
        self._masks: Dict[tuple, np.ndarray] = {}

    def scanned(self, eps_min: float = -90.0, eps_max: float = 90.0) -> np.ndarray:
        """Directions owned by a face AND inside the mission eps window."""
        return (self.owner >= 0) & (self.eps >= eps_min) & (self.eps <= eps_max)

    def mask(self, sec: dict) -> np.ndarray:
        """Directions inside a watch sector (box or cap). Cached per sector geometry: read-only array."""
        key = sector_geometry_key(sec)
        m = self._masks.get(key)
        if m is None:
            if len(self._masks) >= MASK_CACHE_SIZE:
                self._masks.clear()
            m = self._masks[key] = self._mask(sec)
            m.setflags(write=False)
        return m

    def _mask(self, sec: dict) -> np.ndarray:
        """Box or cap tested in the sector's own frame (step 3.0: the raster turned by frame_m, core/frames.py)."""
        phi, eps, eps_off = frames.sector_angles(sec, self.phi, self.eps)
        shape = sec.get("shape", "box")
        if shape == "cap":
            p0, e0, rad = math.radians(sec["phi0_deg"]), math.radians(sec["eps0_deg"]), sec["radius_deg"]
            p, e = np.radians(phi), np.radians(eps)
            cosang = np.sin(e) * math.sin(e0) + np.cos(e) * math.cos(e0) * np.cos(p - p0)
            return np.degrees(np.arccos(np.clip(cosang, -1, 1))) <= rad
        p1, p2 = sec["phi_min_deg"] % 360.0, sec["phi_max_deg"] % 360.0
        if sec["phi_max_deg"] - sec["phi_min_deg"] >= 360.0:
            mphi = np.ones(phi.shape, bool)
        elif p1 <= p2:
            mphi = (phi >= p1) & (phi <= p2)
        else:
            mphi = (phi >= p1) | (phi <= p2)
        return mphi & (eps >= sec["eps_min_deg"] + eps_off) & (eps <= sec["eps_max_deg"] + eps_off)


def grid_spacing(sec: dict) -> float:
    """Spacing of the search TX lattice (TX beamwidths) a sector is evaluated with (step 3.1, P5 E7): the internal key
    set by the optimiser while it tries rm.BEAM_SPACING_OPTIONS, else search_grid.beam_spacing_hpbw."""
    return float(sec.get("_grid_spacing", rm.BEAM_SPACING_HPBW))


def positions_table(sph: Sphere, m: np.ndarray, f_ghz: float, spacing_hpbw: Optional[float] = None,
                    keep: Optional[np.ndarray] = None) -> Tuple[Dict[int, np.ndarray], float]:
    """Beam positions per face and per 1-deg scan bin inside mask m, each raster cell counted for its scanned share keep
    (sector_sky; whole if None). Also the unscanned sector fraction (solid angle, geometry only)."""
    cell = rm.beam_cell_area_uv(f_ghz, spacing_hpbw)
    lim = rm.band_for_frequency(f_ghz).scan_limit_deg(f_ghz)
    nb = int(math.ceil(rm.SECTOR_RADIUS_DEG / THETA_BIN_DEG)) + 1
    out = {k: np.zeros(nb) for k in range(4)}
    tot_om = float((sph.d_omega * m).sum())
    sel = m & sph.scan_zone & (sph.theta <= lim)
    for k in range(4):
        s = sel & (sph.owner == k)
        if not s.any():
            continue
        wgt = sph.w[s] * sph.d_omega[s] / cell
        if keep is not None:
            wgt = wgt * keep[s]
        idx = np.minimum((sph.theta[s] / THETA_BIN_DEG).astype(int), nb - 1)
        np.add.at(out[k], idx, wgt)
    scanned_om = float((sph.d_omega * sel).sum())
    return out, (1.0 - scanned_om / tot_om) if tot_om > 0 else 1.0


# ----------------------------------------------------------------------------- sky noise (step 1.5)
_SKY_CACHE: Dict[tuple, Tuple[dict, np.ndarray]] = {}


def _env_key(sec: dict) -> tuple:
    """Sector values the environment terms depend on (range, target, resolution, speeds, surface guard)."""
    keys = ("range_km", "res_max_m", "v_close_min_mps", "v_close_max_mps", "surface_guard_km") + rm.SECTOR_TARGET_KEYS
    return tuple(sec.get(k) for k in keys)


def _environment(sec: dict, band_name: str, f_ghz: float, sll: float, dirs: np.ndarray, owners: np.ndarray,
                 thetas: np.ndarray, sky: Optional[dict]) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, dict]:
    """Environment terms at the evaluation directions (step 2.5, core/environment.py): rises in K (structure noise,
    stand-off jammers), the noise factor of the blockage (two-way) and of the surface clutter, the structure mask, and
    a summary (masked, shadowed and clutter-limited directions, main-lobe surfaces, limbs, ADC margin)."""
    tgt = sector_target(sec)
    jams = env.jammers(sky)
    surf = env.surfaces(sky)
    parts: Dict[str, np.ndarray] = {"structure": np.zeros(len(dirs))}
    parts.update({f"jammer {j.id}": np.zeros(len(dirs)) for j in jams})
    block = np.zeros(len(dirs))
    for k in np.unique(owners):
        s = owners == k
        parts["structure"][s] = env.structure_rise_k(int(k), dirs[s], band_name, f_ghz, sll)
        block[s] = env.blockage(int(k), dirs[s], band_name)[0]
        for j, v in env.jammer_rise_k(jams, int(k), dirs[s], band_name, f_ghz, sll).items():
            parts[f"jammer {j}"][s] = v
    loss_pass = env.block_loss_db(block)
    masked = loss_pass > env.BLOCK_MASK_DB
    f_block = rm.db_to_lin(2.0 * loss_pass)
    r_km = float(sec["range_km"])
    if band_name == "S":
        bw = admissible_bw_min(sec)
    else:
        bw = min(P.get("tool.confirmation.ku_sig_bw_mhz"))
    dr_m = C / (2.0 * bw * 1e6)
    snr = rm.db_to_lin(detection.snr_req_db(1, tgt.swerling_case(f_ghz), rm.DETECT_PFA, rm.DETECT_PD))
    x = np.zeros(len(dirs))
    info = {"surfaces": {}, "limbs": [], "main_lobe_surface": False, "adc_margin_db_min": None}
    hp = math.radians(rm.hpbw_deg(f_ghz))
    v_min, v_max = float(sec.get("v_close_min_mps", 0.0)), float(sec["v_close_max_mps"])
    s_env = _surface_env_s(sec, f_ghz, sll, dirs, owners, surf, tgt) if band_name == "S" else None
    for si, s_ in enumerate(surf):
        d = env.surface_distance_km(s_, dirs)
        in_disk = np.isfinite(d)
        # S: clutter per waveform (step 3.1, P4), out of the T_sys budget; the summary ratio is the full face's at the
        # pointing's requirement range, narrowest admissible band, one look; Ku: the step-2.5 noise factor
        rej = None if band_name == "S" else env.doppler_rejected(s_, v_min, v_max)
        xs = np.zeros(len(dirs))
        if band_name == "S":
            x_out = s_env["y"][None][si] * (B_REF_MHZ / bw) * snr
        elif not rej:
            for k in np.unique(owners):
                sel = owners == k
                xs[sel] = env.clutter_over_signal(s_, int(k), dirs[sel], band_name, f_ghz, sll, r_km, dr_m,
                                                  tgt.rcs_at(f_ghz)) * snr
        x += xs
        if band_name != "S":
            x_out = xs
        delta = np.arccos(np.clip(dirs @ np.array(s_.c), -1.0, 1.0))
        a = math.asin(min(s_.radius_km / s_.dist_km, 1.0))
        if np.any(np.abs(delta - a) < 2.0 * hp):
            info["limbs"].append(s_.id)
        if in_disk.any():
            info["main_lobe_surface"] = True
            adc = env.main_lobe_adc_margin_db(s_, dirs, thetas, f_ghz)
            m_ = float(np.min(adc))
            info["adc_margin_db_min"] = m_ if info["adc_margin_db_min"] is None else min(info["adc_margin_db_min"], m_)
        relevant = bool(in_disk.any()) or s_.d_min_km < r_km < s_.d_horizon_km    # main lobe or iso-range ring
        info["surfaces"][s_.id] = {"in_disk": in_disk, "shadowed": in_disk & (d < r_km), "distance_km": d,
                                   "doppler_rejected": (rej and relevant) if rej is not None else None,
                                   "relevant": relevant, "x": x_out}
    f_clutter = env.clutter_noise_factor(x)
    return parts, f_block * f_clutter, masked, {**info, "x": x, "f_block": f_block, "s_env": s_env}


def admissible_bw_min(sec: dict) -> float:
    """Narrowest S signal bandwidth meeting the sector's resolution (the widest of the space if none does)."""
    bws = [b for b in SIGBW_CAND if resolution_m(sec, b) <= sec.get("res_max_m", RES_MAX_M_DEFAULT) + 1e-9]
    return min(bws) if bws else max(SIGBW_CAND)


def surface_guard_km(sec: dict) -> float:
    """Guard in front of a surface (step 3.1, P4 Q9): sector key surface_guard_km, at least the coarsest admissible
    range resolution."""
    return max(float(sec.get("surface_guard_km", SURFACE_GUARD_KM)), resolution_m(sec, admissible_bw_min(sec)) / 1e3)


def tx_patterns() -> Tuple[Optional[str], ...]:
    """TX beams of the search space: the full face (None) and the partition sub-apertures (step 3.1, C)."""
    out = [None]
    for mode, b, cut in tx_variants():
        if mode == "partition":
            k = "quarter" if b == 4 else f"half_{cut}"
            if k not in out:
                out.append(k)
    return tuple(out)


def _surface_env_s(sec: dict, f_ghz: float, sll: float, dirs: np.ndarray, owners: np.ndarray, surf, tgt) -> Optional[dict]:
    """Surfaces seen by an S sector, per evaluation direction (step 3.1, P4): the requirement brought back to the
    surface, d - guard (Q9), rounded UP on SURFACE_CLASS_STEP x range_km (R2 c2: requirement classes; <= 0 = no
    volume), and the clutter ratio sigma_c / sigma_target of each surface at the class range, per TX beam of the search
    space (full face, sub-apertures, Q3), for a B_REF_MHZ range cell (linear in c / 2B)."""
    if not surf:
        return None
    r_req = float(sec["range_km"])
    guard = surface_guard_km(sec)
    r_eff = np.full(len(dirs), r_req)
    for s_ in surf:
        r_eff = np.minimum(r_eff, env.surface_distance_km(s_, dirs) - guard)
    step = SURFACE_CLASS_STEP * r_req
    no_volume = r_eff <= 0.0
    val = np.where(r_eff >= r_req, r_req, np.minimum(r_req, np.ceil(r_eff / step - 1e-9) * step))
    classes = np.array(sorted({round(float(v), 9) for v in val[~no_volume]}, reverse=True) or [r_req])
    k_idx = np.full(len(dirs), -1)
    for i, v in enumerate(classes):
        k_idx[(~no_volume) & (np.abs(val - v) < 1e-6)] = i
    dr_ref = C / (2.0 * B_REF_MHZ * 1e6)
    pats = tx_patterns()
    y = {p: np.zeros((len(surf), len(dirs))) for p in pats}
    for ki, rk in enumerate(classes):
        sel_k = k_idx == ki
        for si, s_ in enumerate(surf):
            if not s_.d_min_km < rk < s_.d_horizon_km:
                continue                                   # no iso-range ring at this range
            for face in np.unique(owners[sel_k]):
                sel = sel_k & (owners == face)
                for pt in pats:
                    y[pt][si, sel] = env.clutter_over_signal(s_, int(face), dirs[sel], "S", f_ghz, sll, float(rk),
                                                             dr_ref, tgt.rcs_at(f_ghz), tx_sub=pt)
    return {"classes": classes, "k_idx": k_idx, "no_volume": no_volume, "y": y, "surfaces": tuple(surf)}


def sector_sky(sec: dict, sph: Sphere, m: np.ndarray, f_ghz: float, band_name: str, sky: Optional[dict]) -> Tuple[dict, np.ndarray]:
    """T_sys over the sector's scanned positions at f in a band (core/sky.py): (summary, scanned share of each raster
    cell: 0 blinded, 1 scanned; step 3.1, P5: the refined cells are shared, point by point).
    T_sys is evaluated every sky.EVAL_STEP_HPBW TX beamwidths (blocks of raster cells share the value of their block
    centre), weighted by the position density w dOmega; the budget is the (1 - short_tol) position quantile of the
    positions not blinded. Step 2.5: the environment (core/environment.py) adds the structure noise and the stand-off
    jammers to the rise, and the blockage and surface clutter as noise factors (an effective T_sys: the range scales as
    its fourth root as for the sky); positions masked by the structure are removed with the blinded ones. Cached per
    process (sky, sector geometry and environment values, raster, frequency, band, RX option)."""
    sll, tol = rx_sll_db(sec), sec.get("short_tol", SHORT_TOL)
    key = _sky_cache_key(sec, sph, f_ghz, band_name, sky)
    hit = _SKY_CACHE.get(key)
    if hit is not None:
        return hit
    band = rm.BANDS[band_name]
    s_env = None
    sky_obj = skym.parse(sky)
    sel = m & sph.scan_zone & (sph.theta <= band.scan_limit_deg(f_ghz))
    ii, jj = np.nonzero(sel)
    keep = np.ones(sph.phi.shape)
    if ii.size == 0:
        res = ({"band": band_name, "t_sys_cold_k": band.t_sys_k, "t_sys_budget_k": band.t_sys_k, "rise_db": 0.0,
                "blinded_pct": 0.0, "blinded_by": [], "above_budget_pct": 0.0, "worst_rise_db": 0.0,
                "galactic": sky_obj.galactic, "blind_rise_db": skym.BLIND_RISE_DB, **_EMPTY_ENV}, keep)
    else:
        stride = max(1, int(round(skym.EVAL_STEP_HPBW * rm.hpbw_deg(f_ghz) / (360.0 / sph.n_phi))))
        ri = np.minimum(ii // stride * stride + stride // 2, sph.n_phi - 1)
        rj = np.minimum(jj // stride * stride + stride // 2, sph.n_eps - 1)
        own = sph.owner[ii, jj].astype(np.int64)
        fine = _refine_cells(sky_obj, sky, band, band_name, f_ghz, sll, sph, ii, jj, ri, rj, own)
        ri, rj = np.where(fine, ii, ri), np.where(fine, jj, rj)            # step 3.1, P5: each cell near hot sources
        uk, inv = np.unique((ri * sph.n_eps + rj) * 4 + own, return_inverse=True)
        cell, r_own = uk // 4, uk % 4
        dirs = skym.unit(sph.phi.flat[cell], sph.eps.flat[cell])
        rise, parts = skym.rise_k(sky_obj, band_name, f_ghz, sll, [face_frame(p) for p in FACE_PHI], dirs, r_own)
        e_parts, factor, masked, info = _environment(sec, band_name, f_ghz, sll, dirs, r_own, sph.theta.flat[cell], sky)
        parts.update(e_parts)
        rise = rise + sum(e_parts.values())
        t_base = rm.t_sys_with_rise(band, rise)
        with np.errstate(invalid="ignore"):
            parts["blockage"] = t_base * (info["f_block"] - 1.0)
            parts["clutter"] = np.where(np.isfinite(factor), t_base * (factor - info["f_block"]), np.inf)
        cell_parts = {k: v[inv] for k, v in parts.items()}
        t_sys = (t_base * factor)[inv]
        weights = (sph.w * sph.d_omega)[ii, jj]
        mk = masked[inv]
        split = fine & ~mk if REFINE_SUB > 1 else np.zeros(len(ii), dtype=bool)
        n2, ne = REFINE_SUB * REFINE_SUB, int((~split).sum())
        x_t, x_w = np.where(mk, 0.0, t_sys)[~split], np.where(mk, 0.0, weights)[~split]
        x_parts = {k: v[~split] for k, v in cell_parts.items()}
        if split.any():
            # step 3.1, P5 (option 3): a refined cell is split into SUB x SUB points of equal weight, each blinded or
            # not on its own noise rise (sky, structure, stand-off jammers) and entering the budget quantile on its
            # own, the blockage and clutter factors staying those of its centre; the cell is scanned for the share left
            sub_rise, sub_parts = _subsampled_rise(sky_obj, sky, band_name, f_ghz, sll, sph, ii[split], jj[split],
                                                   own[split])
            f_c, fb_c = np.repeat(factor[inv][split], n2), np.repeat(info["f_block"][inv][split], n2)
            tb_c = rm.t_sys_with_rise(band, sub_rise)
            with np.errstate(invalid="ignore"):
                sub_parts["blockage"] = tb_c * (fb_c - 1.0)
                sub_parts["clutter"] = np.where(np.isfinite(f_c), tb_c * (f_c - fb_c), np.inf)
            x_t = np.concatenate([x_t, tb_c * f_c])
            x_w = np.concatenate([x_w, np.repeat(weights[split], n2) / n2])
            x_parts = {k: np.concatenate([v, sub_parts[k]]) for k, v in x_parts.items()}
        summary, xb = skym.summarise(band, x_t, x_w, x_parts, tol, sky_obj)
        frac = np.zeros(len(ii))                       # blinded share of each scanned cell
        frac[~split] = xb[:ne]
        if split.any():
            frac[split] = xb[ne:].reshape(-1, n2).mean(axis=1)
        w_all = float(weights.sum())
        summary.update(_env_summary(info, inv, mk, weights, w_all, cell_parts["structure"], band, sec))
        frac[mk] = 1.0
        se = info.get("s_env")
        if se is not None:                             # step 3.1, P4: no volume in front of a surface (Q9)
            nv = se["no_volume"][inv] & (frac < 1.0)
            summary["no_volume_pct"] = 100.0 * float((weights * (1.0 - frac))[nv].sum()) / w_all if w_all > 0 else 0.0
            frac[nv] = 1.0
            s_env = {**se, "ii": ii, "jj": jj, "inv": inv, "w": weights, "theta": sph.theta[ii, jj],
                     "keep": 1.0 - frac, "w_all": w_all}
        keep[ii, jj] = 1.0 - frac
        res = (summary, keep)
    keep.setflags(write=False)
    if len(_SKY_CACHE) >= MASK_CACHE_SIZE:
        _SKY_CACHE.clear()
        _S_ENV.clear()
    _SKY_CACHE[key] = res
    if band_name == "S" and ii.size and s_env is not None:
        _S_ENV[key] = s_env                # after the clearing, so the two caches always hold the same keys
    return res


_S_ENV: Dict[tuple, dict] = {}
REFINE_RISE_DB = P.get("tool.sky.refine_rise_db")            # step 3.1, P5 (D5)
REFINE_RADIUS_HPBW = P.get("tool.sky.refine_radius_hpbw")    # step 3.1, P5 (D6)
REFINE_SUB = P.get("tool.sky.refine_subsamples")             # step 3.1, P5 (option 3): points per side of a refined cell



def _subsampled_rise(sky_obj, sky: Optional[dict], band_name: str, f_ghz: float, sll: float, sph: Sphere,
                     ii: np.ndarray, jj: np.ndarray, own: np.ndarray) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Noise rise (K) at REFINE_SUB x REFINE_SUB points spread uniformly inside each raster cell, cell after cell: sky
    (bodies, galaxy), structure and stand-off jammers, with their parts (step 3.1, P5). Returns (rise, parts)."""
    n = REFINE_SUB
    off = (np.arange(n) + 0.5) / n - 0.5
    dphi, deps = 360.0 / sph.n_phi, 180.0 / sph.n_eps
    a, b = np.meshgrid(off, off, indexing="ij")
    ph = (sph.phi[ii, jj][:, None] + a.ravel()[None, :] * dphi).ravel()
    ep = (sph.eps[ii, jj][:, None] + b.ravel()[None, :] * deps).ravel()
    ow = np.repeat(own, n * n)
    dirs = skym.unit(ph, ep)
    rise, parts = skym.rise_k(sky_obj, band_name, f_ghz, sll, [face_frame(p_) for p_ in FACE_PHI], dirs, ow)
    jams = env.jammers(sky)
    st = np.zeros(len(dirs))
    jp = {f"jammer {j.id}": np.zeros(len(dirs)) for j in jams}
    for k in np.unique(ow):
        sel = ow == k
        st[sel] = env.structure_rise_k(int(k), dirs[sel], band_name, f_ghz, sll)
        for jid, v in env.jammer_rise_k(jams, int(k), dirs[sel], band_name, f_ghz, sll).items():
            jp[f"jammer {jid}"][sel] = v
    parts = dict(parts, structure=st, **jp)
    rise = rise + st + sum(jp.values())
    return rise, parts


def _refine_cells(sky_obj, sky: Optional[dict], band, band_name: str, f_ghz: float, sll: float, sph: Sphere,
                  ii: np.ndarray, jj: np.ndarray, ri: np.ndarray, rj: np.ndarray, own: np.ndarray) -> np.ndarray:
    """Raster cells of a sector evaluated one by one instead of at their block centre (step 3.1, P5, S1-S3): the cells
    of a block whose sky rise exceeds REFINE_RISE_DB, and the cells within REFINE_RADIUS_HPBW TX beamwidths of a
    stand-off jammer, of the centre of a small body (radius up to that distance) or of the limb of a large one (the
    rise varies there faster than the block step). Exact at the raster resolution."""
    fine = np.zeros(len(ii), dtype=bool)
    jams = env.jammers(sky)
    if not sky_obj.bodies and not jams:
        return fine
    rad = math.radians(REFINE_RADIUS_HPBW * rm.hpbw_deg(f_ghz))
    d = skym.unit(sph.phi[ii, jj], sph.eps[ii, jj])
    for b in sky_obj.bodies:
        delta = np.arccos(np.clip(d @ np.array(b.c), -1.0, 1.0))
        fine |= (delta < b.radius_rad + rad) if b.radius_rad <= rad else (np.abs(delta - b.radius_rad) < rad)
    for j in jams:
        fine |= np.arccos(np.clip(d @ np.array(j.c), -1.0, 1.0)) < rad
    if sky_obj.bodies:
        uk, inv = np.unique((ri * sph.n_eps + rj) * 4 + own, return_inverse=True)
        cell = uk // 4
        rise, _ = skym.rise_k(sky_obj, band_name, f_ghz, sll, [face_frame(p) for p in FACE_PHI],
                              skym.unit(sph.phi.flat[cell], sph.eps.flat[cell]), uk % 4)
        hot = rm.t_sys_with_rise(band, rise) > band.t_sys_k * rm.db_to_lin(REFINE_RISE_DB)
        fine |= hot[np.ravel(inv)]
    return fine


def _sky_cache_key(sec: dict, sph: Sphere, f_ghz: float, band_name: str, sky: Optional[dict]) -> tuple:
    sll, tol = rx_sll_db(sec), sec.get("short_tol", SHORT_TOL)
    return (skym.sky_key(sky), sector_geometry_key(sec), _env_key(sec), sph.window, f_ghz, band_name, sll, tol)


def sector_env_s(sec: dict, sph: Sphere, m: np.ndarray, f_ghz: float, sky: Optional[dict]) -> Optional[dict]:
    """The S surface data of a sector at f (step 3.1, P4), None without surfaces: per raster cell of the scanned
    sector (ii, jj, weights, theta, blinded by the sky / structure / no volume) and per evaluation direction (inv):
    requirement class, clutter ratios per TX beam and surface."""
    sector_sky(sec, sph, m, f_ghz, "S", sky)
    return _S_ENV.get(_sky_cache_key(sec, sph, f_ghz, "S", sky))


_EMPTY_ENV = {"masked_by_structure_pct": 0.0, "structure_rise_k_max": 0.0, "surfaces": {}, "limbs": [],
              "main_lobe_surface": False, "adc_margin_db_min": None, "clutter_limited_pct": 0.0}


def _env_summary(info: dict, inv: np.ndarray, masked: np.ndarray, weights: np.ndarray, w_all: float,
                 structure_k: np.ndarray, band: "rm.BandConfig", sec: dict) -> dict:
    """Environment part of a sector sky summary (step 2.5)."""
    pct = lambda sel: 100.0 * float(weights[sel].sum()) / w_all if w_all > 0 else 0.0   # noqa: E731
    out = {"masked_by_structure_pct": pct(masked),
           "structure_rise_k_max": float(structure_k[~masked].max()) if (~masked).any() else 0.0,
           "clutter_limited_pct": pct(~masked & (info["x"][inv] >= 1.0)),
           "main_lobe_surface": info["main_lobe_surface"], "limbs": info["limbs"],
           "adc_margin_db_min": info["adc_margin_db_min"], "surfaces": {}}
    for sid, s in info["surfaces"].items():
        if not s["relevant"]:
            continue
        d = s["distance_km"][inv]
        sh = s["shadowed"][inv] & ~masked
        out["surfaces"][sid] = {"in_main_lobe_pct": pct(s["in_disk"][inv] & ~masked), "shadowed_pct": pct(sh),
                                "min_distance_km": float(d[sh].min()) if sh.any() else None,
                                "doppler_rejected": s["doppler_rejected"],
                                # (C/S) x SNR_req: 0 dB and above = clutter-limited (no power reaches the target)
                                "clutter_ratio_max_db": (rm.lin_to_db(float(s["x"].max()))
                                                         if float(s["x"].max()) > 0 else None)}
    return out


def search_positions(sec: dict, sph: Sphere, m: np.ndarray, f_ghz: float, sky: Optional[dict]):
    """Search positions per face and scan bin, the unscanned fraction (geometry: scan limit, eps window), the S sky
    summary and the range factor of the sky noise, (T_sys cold / T_sys budget)^(1/4). Blinded positions are not
    scanned (removed from the positions, reported in the summary)."""
    pos, unscanned = positions_table(sph, m, f_ghz, grid_spacing(sec))
    summary, keep = sector_sky(sec, sph, m, f_ghz, "S", sky)
    if (keep[m] < 1.0).any():
        pos, _ = positions_table(sph, m, f_ghz, grid_spacing(sec), keep)
    t_b = summary["t_sys_budget_k"]
    factor = (summary["t_sys_cold_k"] / t_b) ** 0.25 if t_b else 1.0
    return pos, unscanned, summary, factor


# ----------------------------------------------------------------------------- optimiser
def make_wf(name, tau, n, pri, bw, b, integ, staggered=False, agility=0.0, mode=None, cut=None, window=None):
    return rm.Waveform(name=name, role="SEARCH", band="S", tau_s=tau, n=n, pri_s=pri, sig_bw_mhz=bw,
                       b_tx=b, integration=integ, staggered=staggered, agility_mhz=agility, tx_mode=mode, tx_cut=cut,
                       doppler_window_db=window)


def doppler_windows(sec: dict, sky: Optional[dict]) -> Tuple[float, ...]:
    """Taylor Doppler weightings offered to the sector's coherent trains (step 3.1, P4 Q5): only where a surface whose
    scenario gives its closing speed and spread can clutter the sector (nearer than its range); elsewhere a weighting
    only costs its loss."""
    r = float(sec.get("range_km", 0.0))
    if any(s_.closing_mps is not None and s_.spread_mps is not None and s_.d_min_km < r for s_ in env.surfaces(sky)):
        return rm.DOPPLER_WINDOW_OPTIONS_DB
    return ()


def tx_label(w: dict) -> str:
    """Short label of the TX mode of a waveform dict: 1, SUPx4, PARTx2h, PARTx4, TDMx2."""
    m = w.get("tx_mode", "single")
    if m == "single":
        return "1"
    return {"superposition": "SUP", "partition": "PART", "tdm": "TDM"}[m] + f"x{w['b_tx']}" + (w.get("tx_cut") or "")


def tx_variants() -> List[Tuple[str, int, Optional[str]]]:
    """(mode, beams, cut) of the S TX multibeam space (step 3.1, C): one beam, the superposition's beams, the
    partition's (both cuts of the halves unless they are the same beam, radar_model.partition_cuts)."""
    out = []
    for mode in TX_MODES:
        if mode == "single":
            out.append(("single", 1, None))
        elif mode == "superposition":
            out.extend(("superposition", b, None) for b in rm.MB_SUP_LOSS_DB)
        elif mode == "partition":
            for b in rm.MB_PART_BEAMS:
                out.extend(("partition", b, c) for c in (rm.partition_cuts() if b == 2 else (None,)))
        else:
            out.extend(("tdm", b, None) for b in rm.MB_TDM_BEAMS)
    return out


def agility_for(sec: dict, n: int, bw_mhz: float) -> float:
    """Excursion of the agile variant of a noncoherent train (step 3.1, B7-B8): enough for n independent draws,
    (n - 1) c / 2L, within the usable band minus the signal, on the grid step; 0 if not even 2 draws fit."""
    if n < 2 or ("rcs_m2" not in sec and "target_type" not in sec):
        return 0.0                                     # no target given (partial sector): no agile variant
    tgt = sector_target(sec)
    if not tgt.length_m or tgt.length_m <= 0:
        return 0.0
    dfc = rm.C_MPS / (2.0 * tgt.length_m) / 1e6
    width = (rm.BAND_S.usable_hi_ghz - rm.BAND_S.usable_lo_ghz) * 1e3 - bw_mhz
    step = FREQ_STEP_GHZ * 1e3
    a = math.floor(min((n - 1) * dfc, width) / step + 1e-9) * step
    return a if a >= dfc - 1e-9 else 0.0


def range_min_km(sec: dict) -> float:
    """Shortest range of interest of a sector (step 2.1, C1): key "range_min_km", else RANGE_MIN_FRAC x range_km.
    A waveform whose blind range c (tau + limiter recovery) / 2 exceeds it is not playable for the sector."""
    return float(sec.get("range_min_km", RANGE_MIN_FRAC * sec["range_km"]))


def resolution_m(sec: dict, sig_bw_mhz: float) -> float:
    """Physical range resolution of a signal for this sector (its range sidelobe level, step 2.1, A7)."""
    return rm.range_resolution_m(sig_bw_mhz, rm.range_psl_db(rx_sll_db(sec)))


_LAWS: Dict[tuple, int] = {}                   # look laws of the process (interned: the parts cache holds their id)
_LAW_LIST: List[tuple] = []


def _law_id(law: tuple) -> int:
    i = _LAWS.get(law)
    if i is None:
        i = _LAWS[law] = len(_LAW_LIST)
        _LAW_LIST.append(law)
    return i


@functools.lru_cache(maxsize=SOLVE_CACHE_SIZE)
def _parts_cached(w, f: float, theta: float, tgt, rx_sll_db: float, v_span_mps: Optional[float] = None,
                  pfa: Optional[float] = None, cum: Optional[Tuple[float, float, float]] = None,
                  spj: Optional[float] = None, spacing: Optional[float] = None) -> Optional[tuple]:
    """(range against the noise, burn-through range of a self-protection jammer, required SNR of a look with the
    position losses (dB, the axis of its Pd curve), total loss, look-law id), keyed on everything the range depends on
    (Waveform and Target are frozen dataclasses; the target name is set to a constant by range_at). The model's
    module constants are assumed fixed for the life of the process."""
    try:
        r = rm.solve(w, f, theta, tgt, rm.FACE_ELEC_POWER_MW, rx_sll_db, v_span_mps=v_span_mps, pfa=pfa,
                     cum=cum, spj_density_w_hz=spj, grid_spacing_hpbw=spacing)
    except ValueError:
        return None
    return (r.range_noise_km, r.burn_through_km, r.snr_req_db + r.losses["positions"], r.l_tot_db,
            _law_id(r.look_law))


def _range_cached(w, f: float, theta: float, tgt, rx_sll_db: float, v_span_mps: Optional[float] = None,
                  pfa: Optional[float] = None, cum: Optional[Tuple[float, float, float]] = None) -> Optional[float]:
    p = _parts_cached(w, f, theta, tgt, rx_sll_db, v_span_mps, pfa, cum)
    return None if p is None else p[0]


def rx_sll_db(sec: dict) -> float:
    """RX peak sidelobe level of a sector (mission key "rx_sll_db", default option when absent)."""
    return float(sec.get("rx_sll_db", rm.RX_SLL_DEFAULT_DB))


def velocity_span_mps(sec: dict) -> float:
    """Velocity span the velocity bank of a sector covers (step 2.2, B4): v_close_max - v_close_min."""
    return float(sec["v_close_max_mps"] - sec.get("v_close_min_mps", 0.0))


def range_at(w, f, theta, tgt, rx_sll: Optional[float] = None, v_span: Optional[float] = None,
             pfa: Optional[float] = None, cum: Optional[Tuple[float, float, float]] = None,
             spacing: Optional[float] = None):
    """Range of a candidate (None: not playable). A phase-only superposition differs from its one-beam twin only by
    the power per beam (P / B) and its table loss, so its range is the twin's x (1 / (B L))^(1/4): the radar
    equation, exact, without a solve() of its own (step 3.1, C)."""
    p = parts_at(w, f, theta, tgt, rx_sll, v_span, pfa, cum, None, spacing)
    return None if p is None else p[0]


def parts_at(w, f, theta, tgt, rx_sll: Optional[float] = None, v_span: Optional[float] = None,
             pfa: Optional[float] = None, cum: Optional[Tuple[float, float, float]] = None, spj: Optional[float] = None,
             spacing: Optional[float] = None):
    """_parts_cached of a candidate; a superposition from its one-beam twin, exactly: the noise range x (1 / (B L))^(1/4),
    the burn-through range x (1 / (B L))^(1/2) (it goes as the square root of the power per beam over the losses)."""
    if w.tx_mode == "superposition":
        p = parts_at(dataclasses.replace(w, b_tx=1, tx_mode="single"), f, theta, tgt, rx_sll, v_span, pfa, cum, spj,
                     spacing)
        if p is None:
            return None
        k = 1.0 / (w.b_tx * rm.db_to_lin(rm.multibeam_tx_loss_db(w)))
        return (p[0] * k ** 0.25, p[1] * k ** 0.5, p[2], p[3] + rm.multibeam_tx_loss_db(w), p[4])
    return _parts_cached(w, f, theta, tgt if tgt.name == "cache" else dataclasses.replace(tgt, name="cache"),
                         rm.RX_SLL_DEFAULT_DB if rx_sll is None else float(rx_sll), v_span, pfa, cum, spj,
                         None if spacing is None else float(spacing))


def fa_per_rr(sec: dict) -> float:
    """False alarms tolerated per revisit for a sector (step 2.3, fa_mode 'fa_rate'; mission setting at 5.1)."""
    return float(sec.get("fa_per_rr", FA_PER_RR_DEFAULT))


def search_criterion(sec: dict) -> Tuple[float, float, float]:
    """(v_close_max, revisit, requirement range) of the cumulative search criterion of a sector (step 2.3)."""
    return float(sec["v_close_max_mps"]), float(sec["rr_target_s"]), float(sec["range_km"])


def cells_per_position(sec: dict, w, f: float) -> float:
    """Independent cells a search position of this waveform tests (step 2.2, E1-E5)."""
    d = _processing_dict(w, sec, f)
    angle = rm.rx_beams_per_position(f, rx_sll_db(sec), grid_spacing(sec)) * rm.RX_BEAM_SPACING_HPBW ** 2
    return rm.independent_cells(range_min_km(sec), sec["range_km"], w.sig_bw_mhz, d["pulses_per_block"],
                                d["velocity_hypotheses"] * d["acceleration_hypotheses"], angle)


def candidate_pfas(sec: dict, cands: List, f: float, positions: float) -> List[float]:
    """Pfa per cell of each candidate (step 2.3): the sector's tolerated false alarms over its positions and the
    candidate's cells ('fa_rate'), or detection.pfa ('pfa_cell'); rounded to 6 digits for the range cache. The cells
    do not depend on the TX mode (positions x cells per position is the same for a partition's larger cell), so they
    are computed once per pulse train."""
    memo: Dict[tuple, float] = {}
    out = []
    for w in cands:
        k = (w.tau_s, w.n, w.pri_s, w.sig_bw_mhz, w.integration, w.agility_mhz, w.staggered)
        if k not in memo:
            memo[k] = float(f"{rm.search_pfa(fa_per_rr(sec), positions, cells_per_position(sec, w, f)):.6g}")
        out.append(memo[k])
    return out


def candidate_waveforms(sec: dict, sky: Optional[dict] = None) -> List:
    """Search-space enumeration. Step 2.1: signal bandwidths meeting the sector's PHYSICAL resolution (else the
    widest, flagged), pulses whose blind range fits under range_min_km, trains with an unambiguous PRI >= 2 R / c +
    tau + recovery (no self-eclipsing), or, if the sector allows it, range-ambiguous staggered trains (PRI_MIN_S,
    mean eclipsing loss in solve())."""
    r_want = sec["range_km"]
    res_max = sec.get("res_max_m", RES_MAX_M_DEFAULT)
    bws = [b for b in SIGBW_CAND if resolution_m(sec, b) <= res_max + 1e-9] or [max(SIGBW_CAND)]
    r_min = range_min_km(sec)
    wins = doppler_windows(sec, sky)
    out = []
    for integ in ("COHERENT", "NONCOHERENT"):
        for mode, b, cut in tx_variants():
            for bw in bws:
                for n in N_CAND:
                    for tau in TAU_CAND:
                        blk = tau * (b if mode == "tdm" else 1)        # TRM pulse (time multiplex: B tau)
                        if rm.blind_range_km(rm.BAND_S, blk) > r_min + 1e-9:
                            continue
                        pri_unamb = rm.unambiguous_pri_s(rm.BAND_S, r_want, blk)
                        pri = max(blk, PRI_MIN_S) if (n == 1 or sec.get("allow_ambiguous")) else max(blk, pri_unamb)
                        stag = n > 1 and pri < pri_unamb
                        agile = agility_for(sec, n, bw) if integ == "NONCOHERENT" else 0.0
                        for ag in ((0.0, agile) if agile > 0 else (0.0,)):
                            for win in ((None,) + wins if integ == "COHERENT" and n > 1 else (None,)):
                                try:
                                    w = make_wf("c", tau, n, pri, bw, b, integ, staggered=stag, agility=ag,
                                                mode=mode, cut=cut, window=win)
                                except ValueError:
                                    continue
                                out.append(w)
    return out


class PositionCosts:
    """Cost of one position of the full-face lattice for every candidate, as a function of the pulse scale s ("no
    excess energy", step 3.1 C): the pulse is cut to s tau (floored at PULSE_TB_MIN / B), PAs saturated, PAE of the
    shortened pulse exact; vectorised form of radar_model.search_position_costs."""

    def __init__(self, cands: List):
        b = rm.BAND_S
        self.tau = np.array([w.tau_s for w in cands])
        self.n = np.array([w.n for w in cands], dtype=float)
        self.pri = np.array([w.pri_s for w in cands])
        self.trains = np.array([w.trains_per_dwell for w in cands], dtype=float)
        self.share = self.trains / np.array([rm.cells_per_dwell(w) for w in cands])
        self.floor = np.minimum(1.0, rm.PULSE_TB_MIN / (np.array([w.sig_bw_mhz for w in cands]) * 1e6) / self.tau)
        self.train = self.n > 1
        self._b = b
        # everything the cost function depends on: candidates with equal keys have the same cost at the same s
        self.keys = np.stack([self.tau, self.n, self.pri, self.trains, self.share, self.floor], axis=1)

    def subset(self, idx: np.ndarray) -> "PositionCosts":
        """The costs of the candidates idx only, in that order."""
        out = object.__new__(PositionCosts)
        for k in ("tau", "n", "pri", "trains", "share", "floor", "train", "keys"):
            setattr(out, k, getattr(self, k)[idx])
        out._b = self._b
        return out

    def __call__(self, s, idx=None):
        """(electrical J, s of TX time, effective scale, PAE) at scale s; s has the candidates on its first axis
        (all of them, or the rows idx)."""
        sl = slice(None) if idx is None else idx
        shp = (-1,) + (1,) * (np.ndim(s) - 1)
        tau, n, pri = self.tau[sl].reshape(shp), self.n[sl].reshape(shp), self.pri[sl].reshape(shp)
        trains, share = self.trains[sl].reshape(shp), self.share[sl].reshape(shp)
        s_eff = np.maximum(np.minimum(s, 1.0), self.floor[sl].reshape(shp))
        block = tau * s_eff * trains
        d = np.where(self.train[sl].reshape(shp), block / pri, rm.PAE_DUTY_REF)
        b = self._b
        eta = b.pae_sat * block / (block + b.pae_t_edge_s) * np.minimum(1.0, 1.0 - b.pae_duty_slope * (d - rm.PAE_DUTY_REF))
        t = n * tau * s_eff * share
        return b.p_pk_total_w * t / eta, t, s_eff, eta


def prune_dominated(r: np.ndarray, keys: np.ndarray) -> np.ndarray:
    """Indices of the candidates kept (step 3.1, E9, exact): among candidates with the same cost function of the pulse
    scale (PositionCosts.keys), one whose range is lower or equal in every bin (unplayable = lowest) than another's is
    removed; of identical rows the first (simplest) is kept. The scale (r_want / r)^4 and so the energy and time
    (increasing with the scale), the admissibility and the short positions can only be better for the one kept, so
    the optimum is unchanged."""
    rr_ = np.where(np.isnan(r), -np.inf, r)
    keep = np.ones(len(r), dtype=bool)
    _, grp = np.unique(keys[:, :-1], axis=0, return_inverse=True)   # all but the floor of the scale (last column)
    grp = np.ravel(grp)
    for g in np.unique(grp):
        idx = np.flatnonzero(grp == g)
        if len(idx) < 2:
            continue
        x = rr_[idx]
        fl = keys[idx, -1]
        # j dominates i if its range is >= everywhere and its scale floor <= (cost max(s, floor), increasing in s)
        ge = (x[None, :, :] >= x[:, None, :]).all(axis=2) & (fl[None, :] <= fl[:, None])   # ge[i, j]
        gt = (x[None, :, :] > x[:, None, :]).any(axis=2) | (fl[None, :] < fl[:, None])
        earlier = np.arange(len(idx))[None, :] < np.arange(len(idx))[:, None]
        dom = ge & (gt | earlier)
        np.fill_diagonal(dom, False)
        keep[idx[dom.any(axis=1)]] = False
    return np.flatnonzero(keep)


def _theta_centres(f: float, nb: int) -> np.ndarray:
    """Evaluation angle of each 1-deg scan bin: its centre, clamped to the scan limit at f."""
    lim = rm.band_for_frequency(f).scan_limit_deg(f)
    return np.minimum((np.arange(nb) + 0.5) * THETA_BIN_DEG, min(lim, rm.SECTOR_RADIUS_DEG))


@functools.lru_cache(maxsize=64)
def _scan_factor(f: float, nb: int) -> Optional[np.ndarray]:
    """h(f, theta_i) = R(theta_i) / R(0) at the 1-deg bin centres. In the model the scan angle only enters the range
    through the array gain and the scan loss, so h does not depend on the waveform (nor on the RX weighting), and one solve() per candidate (at
    theta = 0) gives its range in every bin. Checked here on two dissimilar waveforms; None if a model change ever
    breaks the property (callers then solve() every bin)."""
    th = _theta_centres(f, nb)
    tgt = rm.Target("ref", 1.0, 1, 5000.0)
    hs = []
    for w in (make_wf("ref1", 50e-6, 1, 1e-3, 1.0, 1, "NONCOHERENT"), make_wf("ref2", 400e-6, 12, 4e-3, 0.5, 4, "COHERENT")):
        try:
            r0 = rm.solve(w, f, 0.0, tgt).range_km
            hs.append(np.array([rm.solve(w, f, float(t), tgt).range_km / r0 for t in th]))
        except ValueError:                             # e.g. a frequency at the band edge: no signal fits there
            return None
    return hs[0] if np.allclose(hs[0], hs[1], rtol=1e-9, atol=0.0) else None


def _range_matrix(cands: List, f: float, tgt, nb: int, rx_sll: float, v_span: Optional[float] = None,
                  pfas: Optional[List[float]] = None, cum: Optional[Tuple[float, float, float]] = None,
                  spj: Optional[float] = None, spacing: Optional[float] = None) -> np.ndarray:
    """ranges[c, i] (km, full peak power) of candidate c in 1-deg bin i; NaN where the waveform is not playable."""
    rn, rbt, _, _ = _range_parts(cands, f, tgt, nb, rx_sll, v_span, pfas, cum, spj, spacing)
    return rm.jammed_range_km(rn, rbt)


@functools.lru_cache(maxsize=64)
def _bt_scan_factor(f: float, nb: int) -> np.ndarray:
    """R_bt(theta_i) / R_bt(0) of a self-protection jammer: R_bt^2 goes as G_tx / L, and L changes with the scan only
    by the active mismatch (two passes), the same for every waveform (exact)."""
    th = _theta_centres(f, nb)
    g = np.array([rm.gain_lin(1.0, f, float(t)) for t in th]) / rm.gain_lin(1.0, f, 0.0)
    mm = np.array([2.0 * rm.active_mismatch(f, float(t))["loss_db_per_pass"] for t in th])
    return np.sqrt(g * 10.0 ** (-(mm - 2.0 * rm.active_mismatch(f, 0.0)["loss_db_per_pass"]) / 10.0))


def _range_parts(cands: List, f: float, tgt, nb: int, rx_sll: float, v_span: Optional[float] = None,
                 pfas: Optional[List[float]] = None, cum: Optional[Tuple[float, float, float]] = None,
                 spj: Optional[float] = None, spacing: Optional[float] = None):
    """Per candidate and 1-deg bin: range against the noise and burn-through range (km, NaN where not playable), and
    per candidate the required SNR of a look with its position losses (dB) and its look-law id (-1: not playable)."""
    rn = np.full((len(cands), nb), np.nan)
    rbt = np.full((len(cands), nb), np.inf)
    sdb = np.full(len(cands), np.nan)
    law = np.full(len(cands), -1)
    h = _scan_factor(f, nb)
    hb = _bt_scan_factor(f, nb)
    th = _theta_centres(f, nb)
    for j, w in enumerate(cands):
        pfa = None if pfas is None else pfas[j]
        if h is not None:
            p = parts_at(w, f, 0.0, tgt, rx_sll, v_span, pfa, cum, spj, spacing)
            if p is not None:
                rn[j], rbt[j], sdb[j], law[j] = p[0] * h, p[1] * hb, p[2], p[4]
        else:
            for i, t in enumerate(th):
                p = parts_at(w, f, float(t), tgt, rx_sll, v_span, pfa, cum, spj, spacing)
                if p is not None:
                    rn[j, i], rbt[j, i], sdb[j], law[j] = p[0], p[1], p[2], p[4]
    return rn, rbt, sdb, law


# ----------------------------------------------------------------------------- environment per waveform (step 3.1, P4)
def _curves(law_ids: np.ndarray) -> np.ndarray:
    """Pd of one look vs its SNR with the position losses (detection.SNR_GRID_DB) of each look law, one row per id."""
    ids = sorted({int(i) for i in np.ravel(law_ids) if i >= 0})
    out = np.zeros((max(ids, default=-1) + 1, len(detection.SNR_GRID_DB)))
    for i in ids:
        n, fl, pfa, pl, pw = _LAW_LIST[i]
        out[i] = detection.mean_curve(n, fl, pfa, pl, pw)
    return out


_G0, _GS = float(detection.SNR_GRID_DB[0]), float(detection.SNR_GRID_DB[1] - detection.SNR_GRID_DB[0])


def _pd_at(curves: np.ndarray, rows: np.ndarray, snr_db: np.ndarray) -> np.ndarray:
    """Pd of the curves rows at snr_db (same shapes broadcast), linear between grid points, flat beyond the ends."""
    n = curves.shape[1]
    x = np.clip((snr_db - _G0) / _GS, 0.0, n - 1.000001)
    i = x.astype(int)
    fr = x - i
    flat = curves.ravel()
    k = rows * n + i
    return flat.take(k) * (1.0 - fr) + flat.take(k + 1) * fr


def _mean_pd_clutter(curves, rows, sdb, a, lg):
    """Pd of a look averaged over the target velocities (axis -1 of a), at the SNR sdb + 10 lg (lg = log10 of the
    SNR increase G) against noise + clutter: S / (N + C) with C / N = a x S / N (P4 Q7 b, R1 b1)."""
    s_lin = 10.0 ** ((sdb + 10.0 * lg)[..., None] / 10.0)
    eff = 10.0 * np.log10(s_lin / (1.0 + a * s_lin))
    return _pd_at(curves, rows[..., None], eff).mean(axis=-1)


def _clutter_gain(curves, rows, sdb, pstar, a, iters: int = 36) -> np.ndarray:
    """SNR increase G (>= 1) restoring the look's Pd pstar against the clutter, per entry (bisection on log10 G in
    [0, 30], vectorised, 36 steps: 4e-10 on log10 G, 1e-10 relative on the range, below tie_rel_tol); inf where no
    power restores it (S / (N + C) -> 1 / a). Identical entries are solved once (exact)."""
    key = np.column_stack([rows, sdb, pstar, a])
    uk, inv = np.unique(key, axis=0, return_inverse=True)
    inv = np.ravel(inv)
    if len(uk) < len(key):
        return _clutter_gain(curves, uk[:, 0].astype(int), uk[:, 1], uk[:, 2], uk[:, 3:], iters)[inv]
    lo, hi = np.zeros(len(sdb)), np.full(len(sdb), 30.0)
    feasible = _mean_pd_clutter(curves, rows, sdb, a, hi) >= pstar
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        ok = _mean_pd_clutter(curves, rows, sdb, a, mid) >= pstar
        hi, lo = np.where(ok, mid, hi), np.where(ok, lo, mid)
    return np.where(feasible, 10.0 ** hi, np.inf)


def _clutter_limit(curves, rows, pstar, rho, iters: int = 60) -> np.ndarray:
    """Largest clutter ratio y x beta for which some power still gives Pd pstar: mean_v Pd(1 / (rho_v t)) = pstar,
    solved for t (bisection on log10 t in [-30, 30]); inf when even the largest keeps it."""
    lo, hi = np.full(len(rows), -30.0), np.full(len(rows), 30.0)

    def pd(lt):
        return _pd_at(curves, rows[:, None], -10.0 * lt[:, None] - 10.0 * np.log10(np.maximum(rho, 1e-300))).mean(axis=1)
    always = pd(hi) >= pstar
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        ok = pd(mid) >= pstar
        lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
    return np.where(always, np.inf, 10.0 ** lo)


def _weighted_quantile_by_bin(y: np.ndarray, b: np.ndarray, w: np.ndarray, q: float, nb: int) -> np.ndarray:
    """Weighted q-quantile of y in each bin b (0 for an empty bin)."""
    out = np.zeros(nb)
    for i in np.unique(b):
        sel = b == i
        yy, ww = y[sel], w[sel]
        o = np.argsort(yy)
        cum = np.cumsum(ww[o])
        if cum[-1] <= 0:
            continue
        out[i] = yy[o][min(int(np.searchsorted(cum / cum[-1], q - 1e-12)), len(o) - 1)]
    return out


def _surface_columns(sec: dict, sph: Sphere, m: np.ndarray, f: float, tgt, cands: List, parts: List[tuple], se: dict,
                     sky_s: dict, factor: float, nb: int):
    """Positions and ranges of a sector that sees surfaces (step 3.1, P4), per column = 1-deg bin x requirement class
    (Q9-Q10, R2 c2). Per candidate: the clutter ratio of its TX beam at the class range (Q1, Q3), for its range cell
    c / 2B, over its coherent blocks M, through its Doppler filter (residual at each target velocity, Q4-Q8); a
    pointing where no candidate can beat the clutter is blinded (R3); per column the (1 - short_tol) quantile of the
    clutter ratio (Q2) sets the SNR increase G that restores the look's velocity-averaged Pd (R1 b1); the range is
    then the noise range x G^(-1/4) combined with the self-protection jammer's burn-through x G^(-1/2) (Q11).
    Returns (P per column, ranges [candidates, columns], column bin, requirement per column, updated sky summary)."""
    classes = se["classes"]
    K, ns = len(classes), len(se["surfaces"])
    tol = sec.get("short_tol", SHORT_TOL)
    inv, ii, jj = se["inv"], se["ii"], se["jj"]
    k_cell = se["k_idx"][inv]
    keep_c = se["keep"]
    base_ok = keep_c > 0.0
    w_k = se["w"] * keep_c                                 # scanned weight of each cell
    y_cell = {p: se["y"][p][:, inv] for p in se["y"]}
    pats = [rm.tx_subaperture(w) for w in cands]
    v_lo, v_hi = float(sec.get("v_close_min_mps", 0.0)), float(sec["v_close_max_mps"])
    nv = len(env.target_velocities(v_lo, v_hi))
    C_ = len(cands)
    rho = np.ones((C_, ns, nv))
    for j, w in enumerate(cands):
        if w.integration != "COHERENT" or w.n < 2:
            continue                                        # no Doppler rejection without coherent blocks (Q8)
        mb, t_b = rm.coherent_blocks(w.n, w.pri_s, w.tau_s, w.integration, tgt.swerling, tgt, f)
        ppb = w.n // mb
        if ppb < 2:
            continue
        pv = rm.oscillator_phase_var(f, t_b, C * w.pri_s / 2e3, w.sig_bw_mhz)
        for si, s_ in enumerate(se["surfaces"]):
            rho[j, si] = env.doppler_residual(s_.closing_mps, s_.spread_mps, v_lo, v_hi, f, w.pri_s, ppb,
                                              w.doppler_window_db, pv)
    laws = np.array([p[3] for p in parts])                  # [K, C]
    sdb = np.array([p[2] for p in parts])                   # [K, C]
    curves = _curves(laws)
    nblk = np.array([_LAW_LIST[int(i)][0] if i >= 0 else 1 for i in np.max(laws, axis=0)])
    beta = (B_REF_MHZ / np.array([w.sig_bw_mhz for w in cands])) / nblk
    valid = laws >= 0
    pstar = np.where(valid, _pd_at(curves, np.maximum(laws, 0), np.nan_to_num(sdb)), 1.0)
    # R3: pointings where no candidate can beat the clutter (additive criterion over the surfaces)
    alpha = np.zeros((K, C_, ns))                           # 1 / largest tolerable ratio, in y units
    for k in range(K):
        vk = np.flatnonzero(valid[k])
        for si in range(ns):
            t = _clutter_limit(curves, laws[k, vk], pstar[k, vk], rho[vk, si])
            alpha[k, vk, si] = beta[vk] / t                 # t inf -> 0 (never limited)
    clutter_blind = np.zeros(len(inv), dtype=bool)
    for k in range(K):
        cells = base_ok & (k_cell == k)
        if not cells.any():
            continue
        score = np.full(int(cells.sum()), np.inf)
        for pt in set(pats):
            rows = [j for j in range(C_) if pats[j] == pt and valid[k, j]]
            if not rows:
                continue
            a_rows = np.unique(alpha[k, rows], axis=0)
            yk = y_cell[pt][:, cells]                       # [ns, cells]
            score = np.minimum(score, (a_rows @ yk).min(axis=0))
        clutter_blind[np.flatnonzero(cells)[score >= 1.0]] = True
    usable = base_ok & ~clutter_blind & (k_cell >= 0)
    # positions per requirement class
    raster_cls = np.full(sph.phi.shape, -1)
    raster_cls[ii[usable], jj[usable]] = k_cell[usable]
    keep_r = np.ones(sph.phi.shape)
    keep_r[ii, jj] = keep_c
    Pc = np.zeros((4, K * nb))
    for k in range(K):
        pos_k, _ = positions_table(sph, m & (raster_cls == k), f, grid_spacing(sec), keep_r)
        Pc[:, k * nb:(k + 1) * nb] = np.array([pos_k[i] for i in range(4)])
    # clutter quantile per column, TX beam and surface (Q2)
    bins = np.minimum((se["theta"] / THETA_BIN_DEG).astype(int), nb - 1)
    yq = {pt: np.zeros((K * nb, ns)) for pt in y_cell}
    for k in range(K):
        sel = usable & (k_cell == k)
        if not sel.any():
            continue
        for pt in y_cell:
            for si in range(ns):
                yq[pt][k * nb:(k + 1) * nb, si] = _weighted_quantile_by_bin(y_cell[pt][si, sel], bins[sel],
                                                                            w_k[sel], 1.0 - tol, nb)
    # ranges per column
    rc = np.full((C_, K * nb), np.nan)
    col_k = np.repeat(np.arange(K), nb)
    col_b = np.tile(np.arange(nb), K)
    for k in range(K):
        rn, rbt, _, _ = parts[k]
        cols = np.arange(k * nb, (k + 1) * nb)
        g = np.ones((C_, nb))
        for pt in set(pats):
            rows = np.array([j for j in range(C_) if pats[j] == pt and valid[k, j]], dtype=int)
            if not len(rows):
                continue
            a = beta[rows, None, None] * np.einsum("jsv,cs->jcv", rho[rows], yq[pt][cols])   # [rows, nb, nv]
            need = (a.max(axis=2) > 0.0) & np.isfinite(rn[rows]) & (Pc[:, cols].sum(axis=0) > 0.0)[None, :]
            if not need.any():
                continue
            jr, cr = np.nonzero(need)
            g[rows[jr], cr] = _clutter_gain(curves, laws[k, rows[jr]], sdb[k, rows[jr]], pstar[k, rows[jr]],
                                            a[jr, cr])
        with np.errstate(divide="ignore", invalid="ignore"):
            r_k = rm.jammed_range_km(rn * factor * g ** -0.25, rbt * g ** -0.5)
        rc[:, cols] = np.where(np.isfinite(g), r_k, np.nan)
    # summary (copy: the sky summary is cached)
    w_all = se["w_all"]
    pct = (lambda sel: 100.0 * float(w_k[sel].sum()) / w_all if w_all > 0 else 0.0)  # noqa: E731
    sky_s = dict(sky_s)
    sky_s["clutter_limited_pct"] = pct(clutter_blind)
    sky_s["range_reduced_pct"] = pct(usable & (classes[np.maximum(k_cell, 0)] < float(sec["range_km"]) - 1e-9))
    if clutter_blind.any():
        sky_s["blinded_pct"] = sky_s.get("blinded_pct", 0.0) + pct(clutter_blind)
        sky_s["blinded_by"] = list(sky_s.get("blinded_by", [])) + ["clutter"]
    return Pc, rc, col_b, classes[col_k], sky_s


def _plan_to_bins(P: np.ndarray, plan: "_Plan", col_bin: np.ndarray, r_cols: np.ndarray) -> np.ndarray:
    """A plan over columns (1-deg bin x requirement class) as one over the bins (same waveform in a bin): positions
    summed, energy, time and PAE per position position-weighted, range and pulse scale of the most demanding
    populated column of the bin. Returns the positions per bin; the plan is converted in place."""
    nb = int(col_bin.max()) + 1
    pb = np.zeros((P.shape[0], nb))
    np.add.at(pb.T, col_bin, P.T)
    pt = P.sum(axis=0)
    choice, e, t, r, sc, pae = (np.full(nb, -1), np.zeros(nb), np.zeros(nb), np.full(nb, np.nan), np.ones(nb),
                                np.ones(nb))
    for b in range(nb):
        cols = np.flatnonzero((col_bin == b) & (pt > 0) & (plan.choice >= 0))
        if not len(cols):
            continue
        w = pt[cols]
        choice[b] = plan.choice[cols[0]]
        e[b] = float((w * plan.e_bin[cols]).sum() / w.sum())
        t[b] = float((w * plan.t_bin[cols]).sum() / w.sum())
        pae[b] = float((w * plan.e_bin[cols] * plan.pae_bin[cols]).sum() / max((w * plan.e_bin[cols]).sum(), 1e-300))
        c = cols[int(np.argmax(np.asarray(r_cols)[cols]))]
        r[b], sc[b] = plan.r_bin[c], plan.s_bin[c]
    plan.choice, plan.e_bin, plan.t_bin, plan.r_bin, plan.s_bin, plan.pae_bin = choice, e, t, r, sc, pae
    return pb


class _Plan:
    """Waveform assignment for one sector at one frequency: candidate index per 1-deg bin (-1 = no position)."""

    def __init__(self, choice, P, r, costs: PositionCosts, r_want, rr, p_avg_max_w):
        self.choice = choice
        idx = np.flatnonzero(choice >= 0)
        rc = r[choice[idx], idx]                                   # range of the chosen WF in each populated bin
        rw = np.broadcast_to(np.asarray(r_want, dtype=float), choice.shape)[idx]   # requirement per column (P4)
        s = np.minimum(1.0, (rw / rc) ** 4)                        # pulse scale: "no excess energy"
        e, t, s_eff, eta = costs(s, choice[idx])
        self.e_bin = np.zeros(len(choice))                         # electrical J per position
        self.t_bin = np.zeros(len(choice))                         # s of TX time per position
        self.r_bin = np.full(len(choice), np.nan)
        self.s_bin = np.ones(len(choice))                          # pulse scale applied (floored)
        self.pae_bin = np.ones(len(choice))                        # PAE of the shortened pulse
        self.e_bin[idx], self.t_bin[idx], self.s_bin[idx], self.pae_bin[idx] = e, t, s_eff, eta
        self.r_bin[idx] = rc
        self.fe = (P * self.e_bin).sum(axis=1) / (p_avg_max_w * rr)    # per face
        self.ft = (P * self.t_bin).sum(axis=1) / rr
        p_tot = P.sum(axis=0)
        self.short_pos = float(p_tot[idx][rc < rw].sum())
        self.short_frac = self.short_pos / float(p_tot.sum())
        self.objective = float(max(self.fe.max(), self.ft.max()))


def _plan_waveforms(P, r, costs: PositionCosts, r_want, rr, p_avg_max_w, max_wf, short_tol,
                    col_bin: Optional[np.ndarray] = None) -> Optional[_Plan]:
    """Best waveform plan at one frequency. Scan bins are grouped by BIN_DEG; each group gets one waveform, and
    neighbouring groups may share one so that at most max_wf distinct waveforms are used (dynamic programming over
    contiguous segments). Within a segment, a candidate is admissible if it reaches r_want in every populated bin;
    with none admissible, the candidate with the fewest positions short of range is used at full power. The
    energy/time trade-off is scalarised (alpha E + (1 - alpha) T) and alpha is swept to minimise max(E, T). With
    none admissible (step 3.1, P4 R4 b), the fewest positions short, then the longest reach, then the cost win."""
    p_tot = P.sum(axis=0)
    nb = len(p_tot)
    cb = np.arange(nb) if col_bin is None else np.asarray(col_bin)   # columns: 1-deg bins x requirement classes (P4)
    n_bins = int(cb.max()) + 1
    groups = [np.flatnonzero((cb >= lo) & (cb < lo + BIN_DEG) & (p_tot > 0)) for lo in range(0, n_bins, BIN_DEG)]
    groups = [g for g in groups if len(g)]
    if not groups:
        return None
    n_g = len(groups)
    playable = ~np.isnan(r)
    rw = np.broadcast_to(np.asarray(r_want, dtype=float), (nb,))[None, :]
    with np.errstate(invalid="ignore", divide="ignore"):
        s = np.where(playable, np.minimum(1.0, (rw / r) ** 4), 1.0)
        reach = playable & (r >= rw)
    e_s, t_s, _, _ = costs(s)                                              # shortened pulse (step 3.1, C)
    e_cost = e_s * p_tot[None, :] / (p_avg_max_w * rr)                     # energy fraction contributed per bin
    t_cost = t_s * p_tot[None, :] / rr                                     # time fraction contributed per bin
    short_p = np.where(reach, 0.0, p_tot[None, :])
    with np.errstate(invalid="ignore", divide="ignore"):
        ratio = np.where(playable, r / rw, -np.inf)                      # reach over the requirement (R4)
    # per-segment aggregates, segment = groups a..b
    seg = {}
    for a in range(n_g):
        bins = np.concatenate(groups[a:a + 1])
        e_s, t_s = e_cost[:, bins].sum(1), t_cost[:, bins].sum(1)
        adm, play, sh = reach[:, bins].all(1), playable[:, bins].all(1), short_p[:, bins].sum(1)
        worst = ratio[:, bins].min(1)
        seg[a, a] = (e_s, t_s, adm, play, sh, worst)
        for b in range(a + 1, n_g):
            bins = groups[b]
            e_s, t_s = e_s + e_cost[:, bins].sum(1), t_s + t_cost[:, bins].sum(1)
            adm, play = adm & reach[:, bins].all(1), play & playable[:, bins].all(1)
            sh = sh + short_p[:, bins].sum(1)
            worst = np.minimum(worst, ratio[:, bins].min(1))
            seg[a, b] = (e_s, t_s, adm, play, sh, worst)
    p_all = float(p_tot.sum())
    best = None
    for alpha in ALPHAS:
        pick = {}
        for (a, b), (e_s, t_s, adm, play, sh, worst) in seg.items():
            cost = snap(alpha * e_s + (1.0 - alpha) * t_s)
            if adm.any():
                c = int(np.argmin(np.where(adm, cost, np.inf)))
                pick[a, b] = (c, float(cost[c]))
            elif play.any():
                # step 3.1, P4 R4 (b): fewest positions short, then the longest reach (worst point of the segment
                # over its requirement), then the cost
                c = int(np.lexsort((cost, -snap(np.where(play, worst, -np.inf)), np.where(play, sh, np.inf)))[0])
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
                    cost = snap(prev[0] + p[1])
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
        plan = _Plan(choice, P, r, costs, r_want, rr, p_avg_max_w)
        key = (round(plan.short_frac / short_tol), snap(plan.objective))
        if best is None or key < best[0]:
            best = (key, plan)
    return best[1] if best else None


def _wf_dict(name: str, w, f: float, sec: dict) -> dict:
    v_max, v_min = sec["v_close_max_mps"], sec.get("v_close_min_mps", 0.0)
    return {"name": name, "band": "S", "freq_ghz": f, "integration": w.integration,
            "tau_us": round(w.tau_s * 1e6, 1), "n_pulses": w.n, "pri_ms": round(w.pri_s * 1e3, 3),
            "sig_bw_mhz": w.sig_bw_mhz, "tx_mode": w.tx_mode, "b_tx": w.b_tx, "tx_cut": w.tx_cut,
            "positions_per_dwell": rm.cells_per_dwell(w),
            # digital RX beams formed per TX beam (compute load, 4.4): x the sub-aperture cell for a partition
            "rx_beams_per_position": rm.rx_beams_per_position(f, rx_sll_db(sec), grid_spacing(sec))
            * (rm.cells_per_dwell(w) / w.b_tx if w.tx_mode == "partition" else 1.0),
            "dwell_ms": round(w.dwell_s * 1e3, 1), "duty_pct": round(100 * w.duty, 3), "pae": rm.pae_wf(w),
            "rx_deaf_us": round(rm.rx_deaf_after_pulse_s(rm.BAND_S, w.tau_s) * 1e6, 3),
            "unamb_range_km": round(C * w.pri_s / 2e3), "doppler_max_kHz": round(2 * v_max / rm.lambda_m(f) / 1e3, 1),
            "modulation": w.modulation or rm.modulation_for(f, w.sig_bw_mhz, v_max),
            "range_resolution_m": resolution_m(sec, w.sig_bw_mhz),
            "blind_range_km": rm.blind_range_km(rm.BAND_S, w.tx_block_s), "staggered": w.staggered,
            "agility_mhz": w.agility_mhz, "frequency_draws": rm.frequency_draws(sector_target(sec), w.agility_mhz),
            "doppler_window_db": w.doppler_window_db,
            "cells_in_pulse": rm.cells_in_pulse(v_max, w.tau_s, w.sig_bw_mhz),
            "velocity_hypotheses_1db": rm.velocity_hypotheses(v_max - v_min, w.tau_s, w.sig_bw_mhz),
            **_processing_dict(w, sec, f)}


@functools.lru_cache(maxsize=1 << 14)
def _sector_target(sec_key: Tuple) -> "rm.Target":
    return dataclasses.replace(rm.target_for_sector(dict(sec_key)), name="cache")


def sector_target(sec: dict) -> "rm.Target":
    """The sector's target (radar_model.target_for_sector, step 2.4), cached on its target keys; named "cache" so that
    sectors sharing a threat share the range cache."""
    keys = ("v_close_max_mps",) + rm.SECTOR_TARGET_KEYS
    return _sector_target(tuple((k, sec[k]) for k in keys if k in sec))


def _processing_dict(w, sec: dict, f: float) -> dict:
    """Step 2.2 processing of a search waveform for this sector at f: coherent blocks (with the target's T0 and
    fluctuation, step 2.4), velocity bank (hypotheses over the sector's span), acceleration hypotheses (step 2.4)."""
    tgt = sector_target(sec)
    m, t_b = rm.coherent_blocks(w.n, w.pri_s, w.tau_s, w.integration, tgt.swerling, tgt, f)
    n_hyp = rm.velocity_bank(round(velocity_span_mps(sec), 6), w.tau_s, w.sig_bw_mhz, w.dwell_s, w.n, m == 1)[0]         if rm.MIGRATION_MODE.upper() == "COMPENSATED" else 1
    n_acc = rm.acceleration_bank(tgt.accel_max_mps2, f, t_b if m < w.n else 0.0, w.dwell_s, w.sig_bw_mhz, m == 1)[0]
    return {"coherent_blocks": m, "pulses_per_block": w.n // m, "block_ms": round(t_b * 1e3, 3),
            "velocity_hypotheses": n_hyp, "acceleration_hypotheses": n_acc}


def _freq_ranges(sec: dict, sph: Sphere, m: np.ndarray, f: float, sky: Optional[dict], cands: List, tgt,
                 excl) -> Optional[tuple]:
    """(P, unscanned, ranges, sky summary, Pfas, T_sys factor) of a sector at one centre, None if nothing to scan.
    Step 3.1, X1: only the candidates whose occupied band fits at f are solved (the others are unplayable there)."""
    if rm.band_for_frequency(f) is not rm.BAND_S:
        return None                                    # outside the (possibly redefined) S band
    pos, unscanned, sky_s, factor = search_positions(sec, sph, m, f, sky)
    P = np.array([pos[k] for k in range(4)])
    if P.sum() <= 0:
        return None
    pfas = candidate_pfas(sec, cands, f, float(P.sum()))
    fit = [c for c, w in enumerate(cands) if signal_fits(f, w.occupied_mhz, excl)
           and not (w.staggered and sky_s.get("main_lobe_surface"))]   # A2, B8; step 2.5 C2 (folded clutter)
    nb = P.shape[1]
    spj = env.spj_density_w_hz(sec, f)                                  # step 3.1, P4 Q11-Q13
    se = sector_env_s(sec, sph, m, f, sky)
    v, rr, r_req = search_criterion(sec)
    classes = se["classes"] if se is not None else np.array([r_req])
    fc = [cands[c] for c in fit]
    parts = [_range_parts(fc, f, tgt, nb, rx_sll_db(sec), velocity_span_mps(sec), [pfas[c] for c in fit],
                          (v, rr, float(rk)), spj, grid_spacing(sec))
             for rk in classes]                                         # the cumulative criterion at each class range
    surfaces = not (se is None or (len(classes) == 1
                                   and not any(float(y.max(initial=0.0)) > 0.0 for y in se["y"].values())))

    def columns(pp, s_in):
        if not surfaces:
            rn, rbt, _, _ = pp[0]
            return P, rm.jammed_range_km(rn * factor, rbt), np.arange(nb), r_req, s_in
        return _surface_columns(sec, sph, m, f, tgt, fc, pp, se, s_in, factor, nb)
    Pc, rc, col_bin, r_want, sky_out = columns(parts, sky_s)
    if spj is not None and fit:
        # step 3.1, P4 R5 (b): where the self-protection jammer puts the requirement out of the search's reach, the
        # sector is planned against the noise alone; the scheduler points a Ku burn-through search in the jamming
        # axis (concept of the user, step 4.3)
        quiet = [(rn, np.full_like(rbt, np.inf), sd, lw) for rn, rbt, sd, lw in parts]
        Pq, rq, cbq, rwq, sky_q = columns(quiet, sky_s)
        if _short_columns(Pq, rq, rwq) < _short_columns(Pc, rc, r_want):
            Pc, rc, col_bin, r_want, sky_out = Pq, rq, cbq, rwq, dict(sky_q, spj_out_of_reach=True)
    r = np.full((len(cands), rc.shape[1]), np.nan)
    if fit:
        r[fit] = rc
    return Pc, unscanned, r, sky_out, pfas, factor, col_bin, r_want


def _short_columns(P: np.ndarray, r: np.ndarray, r_want) -> int:
    """Populated columns that no candidate reaches."""
    with np.errstate(invalid="ignore"):
        best = np.nanmax(np.where(np.isnan(r), -np.inf, r), axis=0) if len(r) else np.full(P.shape[1], -np.inf)
    return int(((P.sum(axis=0) > 0) & (best < np.broadcast_to(np.asarray(r_want, dtype=float), best.shape))).sum())


def _freq_task(sec: dict, eps_min: float, eps_max: float, f: float, sky: Optional[dict],
               spacings: Tuple[float, ...]) -> List[Optional[tuple]]:
    """Worker entry point of the parallel frequency evaluation (X3): one centre at every lattice spacing, in the same
    process so that the spacings share its sky, environment and law caches (step 3.1 A5)."""
    sph = _sphere(eps_min, eps_max)
    m = sph.mask(sec)
    cands, tgt, excl = candidate_waveforms(sec, sky), sector_target(sec), exclusions_s(sec)
    return [_freq_ranges(dict(sec, _grid_spacing=sp), sph, m, f, sky, cands, tgt, excl) for sp in spacings]


_IN_POOL_WORKER = False


def _mark_pool_worker() -> None:
    global _IN_POOL_WORKER
    _IN_POOL_WORKER = True


def _freq_parallel_ok(n_freqs: int) -> bool:
    """Frequencies of one sector in parallel: in the main process only (not inside a sector worker, nor under the
    parallel test runner, whose workers already fill the machine), with several frequencies and workers."""
    return (not _IN_POOL_WORKER and MAX_WORKERS > 1 and n_freqs > 2 and not os.environ.get("PYTEST_XDIST_WORKER")
            and os.environ.get("RADAR_FREQ_PARALLEL", "1") != "0")


def _best_plan(sec: dict, sph: Sphere, m: np.ndarray, p_avg_max_w: float, sky: Optional[dict] = None):
    """Frequency and waveform plan chosen for a sector: (f, P, unscanned, plan, cands, sky summary), or a status dict
    (NO_POSITIONS / NO_WAVEFORM) when there is none. sky: the mission sky (core/sky.py; None = cold sky with the
    worst-case galactic rise): blinded positions are left out, ranges scale with the sector's budget T_sys."""
    tgt = sector_target(sec)
    r_want, rr = sec["range_km"], sec["rr_target_s"]
    max_wf = max(1, int(sec.get("max_wf_per_sector", MAX_WF_PER_SECTOR)))
    short_tol = sec.get("short_tol", SHORT_TOL)
    cands = candidate_waveforms(sec, sky)
    if not cands:
        return {"id": sec["id"], "status": "NO_WAVEFORM",
                "note": f"no pulse of the search space is short enough: the shortest leaves "
                        f"{rm.blind_range_km(rm.BAND_S, min(TAU_CAND)):.1f} km blind, above the sector's range_min_km "
                        f"{range_min_km(sec):g} km (shorter pulses: search space, step 3.1)"}
    costs = PositionCosts(cands)                    # J and s of TX time per position vs the pulse scale (step 3.1, C)
    excl = exclusions_s(sec)
    bw_min = min(w.sig_bw_mhz for w in cands)
    evaluated: Dict[float, Optional[tuple]] = {}

    def evaluate_all(fs) -> None:
        """Positions, ranges and plan at each new centre and every lattice spacing of rm.BEAM_SPACING_OPTIONS (step
        3.1, P5 E7: exact search; None: no position or no plan there); the ranges are computed in parallel processes
        when allowed (step 3.1, X3: identical results)."""
        jobs = [(f, sp) for f in fs for sp in rm.BEAM_SPACING_OPTIONS if (f, sp) not in evaluated]
        if not jobs:
            return
        by_f: Dict[float, List[float]] = {}
        for f, sp in jobs:
            by_f.setdefault(f, []).append(sp)
        if _freq_parallel_ok(len(by_f)):
            ex = _executor()
            out = list(ex.map(_freq_task, [sec] * len(by_f), [sph.window[0]] * len(by_f), [sph.window[1]] * len(by_f),
                              list(by_f), [sky] * len(by_f), [tuple(v) for v in by_f.values()]))
            got = {(f, sp): x for (f, sps), xs in zip(by_f.items(), out) for sp, x in zip(sps, xs)}
            res = [got[j] for j in jobs]
        else:
            res = [_freq_ranges(dict(sec, _grid_spacing=sp), sph, m, f, sky, cands, tgt, excl) for f, sp in jobs]
        for (f, sp), x in zip(jobs, res):
            evaluated[f, sp] = None if x is None else _freq_plan(f, sp, x)

    def _freq_plan(f: float, sp: float, x: tuple) -> tuple:
        P, unscanned, r, sky_s, pfas, factor, col_bin, r_cols = x
        keep = prune_dominated(r, costs.keys)                   # step 3.1, E9 (exact)
        plan = _plan_waveforms(P, r[keep], costs.subset(keep), r_cols, rr, p_avg_max_w, max_wf, short_tol, col_bin)
        if plan is not None:
            plan.choice = np.where(plan.choice >= 0, keep[np.maximum(plan.choice, 0)], -1)
        return (f, P, unscanned, plan, sky_s, pfas, factor, col_bin, r_cols, sp)

    def best_of() -> Optional[tuple]:
        done = [x for x in evaluated.values() if x is not None]
        if not done:
            return None
        min_uns = min(x[2] for x in done)
        ranked = [x for x in done if x[3] is not None]
        # [D] a frequency that leaves more of the sector unscanned (e.g. > 2.85 GHz, 70 deg scan limit) never wins;
        # then range, cost, and the secondary criterion of E6 (energy, time, distinct waveforms, the frequency
        # nearest the centre of the core band)
        core = (rm.BAND_S.core_lo_ghz + rm.BAND_S.core_hi_ghz) / 2.0
        return min(ranked, key=lambda x: (round(x[2] - min_uns, 3), round(x[3].short_frac / short_tol),
                                          snap(x[3].objective), snap(float(x[3].fe.sum())),
                                          snap(float(x[3].ft.sum())), len(set(x[3].choice[x[3].choice >= 0])),
                                          round(abs(x[9] - rm.BEAM_SPACING_HPBW), 9), round(abs(x[0] - core), 9),
                                          x[0], x[9]), default=None)

    forced = sec.get("freq_ghz_candidates")
    if forced is not None:
        evaluate_all([float(f) for f in forced])
    else:
        evaluate_all(coarse_centres(bw_min, excl))
        b = best_of()
        if b is not None:
            evaluate_all(fine_centres(b[0], bw_min, excl, {fc for fc, _ in evaluated}))
    if not evaluated:
        return {"id": sec["id"], "status": "NO_WAVEFORM",
                "note": f"no S centre where a signal of the search space ({bw_min:g} MHz at least) fits inside the band "
                        f"and outside the spectrum exclusions"}
    if not any(x is not None for x in evaluated.values()):
        return {"id": sec["id"], "status": "NO_POSITIONS", "note": "empty sector, fully unscanned or blinded by the sky"}
    best = best_of()
    if best is None:
        return {"id": sec["id"], "status": "NO_WAVEFORM", "note": "no playable waveform in the search space"}
    f, P, unscanned, plan, sky_s, pfas, _, col_bin, r_cols, sp = best
    if len(col_bin) != int(col_bin.max()) + 1:
        P = _plan_to_bins(P, plan, col_bin, r_cols)          # step 3.1, P4: requirement classes back to 1-deg bins
    plan.freq_info = {"evaluated": len({fc for fc, _ in evaluated}), "valid_centres": len(valid_centres(bw_min, excl)),
                      "valid_centres_unexcluded": len(valid_centres(bw_min, ())),
                      "occupied_ghz": [round(f - max(cands[c].occupied_mhz for c in set(plan.choice[plan.choice >= 0]))
                                             / 2e3, 6),
                                       round(f + max(cands[c].occupied_mhz for c in set(plan.choice[plan.choice >= 0]))
                                             / 2e3, 6)]}
    return f, P, unscanned, plan, cands, sky_s, pfas, sp


def sector_plan(sec: dict, eps_min: float, eps_max: float, p_avg_max_w: float,
                sky: Optional[dict] = None) -> Optional[dict]:
    """The waveform plan run() would choose for this sector (same code path), per 1-deg scan bin: "wf" (Waveform or
    None where the sector has no position), "pulse_scale" (fraction of the pulse actually sent, "no excess energy":
    the energy fraction, losses of the full pulse kept, step 3.1 C),
    "short" (the sector's own range is not reached there, even at full power), and "freq_ghz". None if the sector
    has no plan. Used to test whether this sector also meets another one's need."""
    sph = _sphere(eps_min, eps_max)
    best = _best_plan(sec, sph, sph.mask(sec), p_avg_max_w, sky)
    if isinstance(best, dict):
        return None
    f, _, _, plan, cands, _, pfas, sp = best
    return {"freq_ghz": f, "beam_spacing_hpbw": sp, "wf": [cands[c] if c >= 0 else None for c in plan.choice],
            "pfa": [pfas[c] if c >= 0 else None for c in plan.choice],
            "pulse_scale": [float(s) for s in plan.s_bin],
            "short": [bool(r < sec["range_km"]) for r in plan.r_bin]}   # NaN (no position) compares False


def optimise_sector(sec: dict, sph: Sphere, m: np.ndarray, p_avg_max_w: float, sky: Optional[dict] = None) -> dict:
    best = _best_plan(sec, sph, m, p_avg_max_w, sky)
    if isinstance(best, dict):
        return best
    f, P, unscanned, plan, cands, sky_s, pfas, sp = best
    sec = dict(sec, _grid_spacing=sp)                       # the lattice spacing chosen (step 3.1, P5 E7)
    r_want, rr = sec["range_km"], sec["rr_target_s"]
    v_max = sec["v_close_max_mps"]
    tgt = sector_target(sec)
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
        d = _wf_dict(f"WF-{sec['id']}" + (f"-{n}" if len(order) > 1 else ""), cands[c], f, sec)
        d["pfa"] = pfas[c]
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
                          "pulse_scale": round(float(plan.s_bin[worst]), 3),
                          # cost of one position at the scale of the group's worst bin (step 3.1, C)
                          "elec_J_per_position": float(plan.e_bin[worst]),
                          "pae_at_scale": float(plan.pae_bin[worst])})

    faces = {}
    for k in range(4):
        if P[k].sum() <= 0:
            continue
        e_tot = float(plan.fe[k]) * p_avg_max_w * rr
        heat = float((P[k] * plan.e_bin * (1.0 - plan.pae_bin)).sum()) / rr
        faces[k] = {"positions": round(float(P[k].sum()), 1), "energy_J_per_refresh": e_tot,
                    "dissipation_kW": heat / 1e3, "dissipation_kW_m2": heat / 1e3 / rm.TOTAL_AREA_M2,
                    "energy_fraction": float(plan.fe[k]), "time_fraction": float(plan.ft[k]),
                    "avg_power_kW": e_tot / rr / 1e3,
                    "theta_max_deg": float(np.flatnonzero(P[k] > 0).max() + 1) * THETA_BIN_DEG}
    load = max(max(fc["energy_fraction"], fc["time_fraction"]) for fc in faces.values())
    f_e_max = max(fc["energy_fraction"] for fc in faces.values())
    f_t_max = max(fc["time_fraction"] for fc in faces.values())
    r_worst, r_best = float(plan.r_bin[populated[-1]]), float(plan.r_bin[populated[0]])
    th_max = min(float(populated[-1] + 1) * THETA_BIN_DEG, rm.band_for_frequency(f).scan_limit_deg(f))
    sll = rx_sll_db(sec)
    sll_held = rm.rx_sll_held_up_to_deg(f, sll, th_max)
    mismatch_max = max(rm.active_mismatch(f, float(t))["loss_db_per_pass"]
                       for t in _theta_centres(f, len(p_tot))[populated])
    main = max(wf_set, key=lambda d: d["positions"])
    w_main = cands[order[wf_set.index(main)]]
    out = {
        "id": sec["id"], "status": "OK",
        "wf": {k: v for k, v in main.items() if k not in ("theta_lo_deg", "theta_hi_deg", "positions")},
        "wf_set": wf_set,
        "wf_by_bin": wf_by_bin,
        "frequency": {"freq_ghz": f, "occupied_ghz": plan.freq_info["occupied_ghz"],
                      "centres_evaluated": plan.freq_info["evaluated"],
                      "centres_valid": plan.freq_info["valid_centres"],
                      "centres_excluded": plan.freq_info["valid_centres_unexcluded"] - plan.freq_info["valid_centres"]},
        "range_km": {"requested": r_want, "at_best_scan": round(r_best), "at_worst_scan": round(r_worst),
                     "positions_short_of_range_pct": round(100 * plan.short_frac, 1)},
        "unscanned_pct": round(100 * unscanned, 1),
        "positions_total": round(float(p_tot.sum()), 1),
        "rx_sll_db": sll,
        "range_min_km": range_min_km(sec),
        "range_resolution_m": {"requested": sec.get("res_max_m", RES_MAX_M_DEFAULT),
                               "achieved": max(d["range_resolution_m"] for d in wf_set)},
        "beam_spacing_hpbw": grid_spacing(sec),              # search lattice of the sector (step 3.1, P5 E7)
        "rx_beams_per_position": max(d["rx_beams_per_position"] for d in wf_set),
        "rx_sll_held_up_to_deg": sll_held,
        "mismatch_db_per_pass_max": mismatch_max,
        "sky": sky_s,
        "s_min_unsaturated_range_km": rm.min_unsaturated_range_km(f, 0.0, tgt.rcs_at(f)),
        "target": target_summary(tgt, f),
        "environment": environment_summary(sec, tgt, f, w_main, pfas[order[wf_set.index(main)]],
                                           max(cands, key=lambda w: rm.tx_beam_power_w(w) * rm.tx_gain_ratio(w)
                                               * w.tau_s * w.n)),
        "faces": {str(k): v for k, v in faces.items()},
        "rr_target_s": rr,
        "rr_alone_s": rr * load,                               # RR if this sector had the face to itself (energy and time)
        "time_to_impact_at_detection_s": (r_want * 1e3 / v_max - rr) if v_max > 0 else None,
    }
    out["flags"] = []
    res = out["range_resolution_m"]
    if res["achieved"] > res["requested"] + 1e-9:
        out["flags"].append(f"range resolution {res['requested']:g} m not met: {res['achieved']:.2f} m at best with the "
                            f"search space up to {max(SIGBW_CAND):g} MHz (wider signals at step 3.1)")
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
    if mismatch_max > rm.MISMATCH_ALERT_DB:
        out["flags"].append(f"active-mismatch loss up to {mismatch_max:.2f} dB per pass at {f} GHz "
                            f"(alert above {rm.MISMATCH_ALERT_DB:g} dB)")
    if sll_held < th_max:
        out["flags"].append(f"S RX sidelobes {sll:g} dB held only up to {sll_held:g} deg scan at {f} GHz (worst visible "
                            f"sidelobe {rm.sidelobe_visible_db(f, th_max, 'rx', sll):.1f} dB at {th_max:g} deg: TRSA "
                            f"gap lobes)")
    ev = out["environment"]
    if sky_s.get("spj_out_of_reach"):
        out["flags"].append(f"self-protection jammer {ev['jammer_type']}: the required {r_want:.0f} km is out of the "
                            f"search's reach, planned against the noise alone; the search gives the jamming axis and "
                            f"the scheduler points a Ku burn-through search there (step 4.3)")
    if ev.get("burn_through_km") is not None and ev["burn_through_km"] < r_want:
        out["flags"].append(f"self-protection jammer {ev['jammer_type']}: burn-through {ev['burn_through_km']:.0f} km "
                            f"with the main waveform, {ev['burn_through_best_km']:.0f} km at best with the most energetic "
                            f"of the search space (full power, broadside), below the required {r_want:.0f} km")
    if sky_s.get("main_lobe_surface") and sec.get("allow_ambiguous"):
        out["flags"].append("a surface is in the main lobe: range-ambiguous trains would fold its clutter onto the "
                            "targets, so they are left out for this sector")
    if rr < RR_T0_RATIO * tgt.t0_s(f):
        out["flags"].append(f"revisit {rr:g} s below {RR_T0_RATIO:g} x the target coherence time T0 {tgt.t0_s(f):.2f} s "
                            f"at {f} GHz: successive looks are correlated (the cumulative criterion assumes them "
                            f"independent)")
    out["flags"].extend(sky_flags(sky_s, "S"))
    fi = plan.freq_info
    if fi["valid_centres_unexcluded"] and (fi["valid_centres_unexcluded"] - fi["valid_centres"]) >             EXCLUDED_FLAG_FRAC * fi["valid_centres_unexcluded"]:
        out["flags"].append(f"spectrum exclusions remove {fi['valid_centres_unexcluded'] - fi['valid_centres']} of the "
                            f"{fi['valid_centres_unexcluded']} valid S centres (step 3.1, A10)")
    return out


def environment_summary(sec: dict, tgt: "rm.Target", f: float, w, pfa: float, w_best=None) -> dict:
    """Sector environment outputs not tied to a direction (step 2.5): burn-through range of a self-protection jammer
    (sector key jammer_type, at full power, broadside: with the main waveform, and with the most energetic waveform of
    the search space, w_best), Faraday rotation (sector keys tec_tecu and b_parallel_nt, round trip, S and Ku),
    non-threatening objects to confirm (objects_per_rr)."""
    out = {"jammer_type": sec.get("jammer_type"), "burn_through_km": None, "burn_through_best_km": None,
           "faraday_deg": None, "objects_per_rr": float(sec.get("objects_per_rr", 0.0))}
    jt = sec.get("jammer_type")
    if jt is not None:
        cls = env.JAMMER_CLASSES[jt]
        erp = env._erp(jt, sec.get("jammer_erp_dbw_mhz"))[0]
        for key, wf in (("burn_through_km", w), ("burn_through_best_km", w_best or w)):
            if not cls["S"]["lo"] <= f <= cls["S"]["hi"]:
                out[key] = math.inf
                continue
            try:
                r = rm.solve(wf, f, 0.0, tgt, rm.FACE_ELEC_POWER_MW, rx_sll_db(sec), v_span_mps=velocity_span_mps(sec),
                             pfa=pfa, cum=search_criterion(sec))
            except ValueError:
                continue
            out[key] = env.burn_through_km(rm.tx_beam_power_w(wf),
                                           rm.gain_lin(rm.BAND_S.eta_illum_tx, f, 0.0) * rm.tx_gain_ratio(wf), tgt.rcs_at(f),
                                           wf.tau_s * wf.n, r.snr_req_db, r.l_tot_db, erp)
    if sec.get("tec_tecu") is not None and sec.get("b_parallel_nt") is not None:
        ku = P.get("confirmation.ku_freq_ghz")
        out["faraday_deg"] = {"S": env.faraday_round_trip_deg(sec["tec_tecu"], sec["b_parallel_nt"], f),
                              "Ku": env.faraday_round_trip_deg(sec["tec_tecu"], sec["b_parallel_nt"], ku)}
    return out


def target_summary(tgt: "rm.Target", f: float) -> dict:
    """Target model of a sector at f (step 2.4): class, RCS (body + optics), fluctuation, T0, depolarisation,
    acceleration."""
    return {"type": tgt.target_type, "rcs_m2": tgt.rcs_at(f), "optics_rcs_m2": tgt.optics_rcs_m2(f),
            "swerling": tgt.swerling, "fluctuation": tgt.fluctuation(f) if tgt.swerling is None else f"SW{tgt.swerling}",
            "t0_s": tgt.t0_s(f), "depol_db": rm.lin_to_db(tgt.depol_lin(f)),
            "accel_max_mps2": tgt.accel_max_mps2}


def sky_flags(s: dict, label: str) -> List[str]:
    """Operator flags of a sector sky summary (sector_sky)."""
    out = []
    if s["blinded_pct"] > 0:
        out.append(f"{s['blinded_pct']:.2f} % of the {label} positions blinded by the sky ("
                   f"{', '.join(s['blinded_by'])}: T_sys rise above {s['blind_rise_db']:g} dB), not scanned")
    if (s["above_budget_pct"] > 0 and s["worst_rise_db"] is not None
            and s["worst_rise_db"] - s["rise_db"] > SKY_ABOVE_BUDGET_ALERT_DB):
        out.append(f"{s['above_budget_pct']:.2f} % of the {label} positions above the budget T_sys "
                   f"({s['t_sys_budget_k']:.1f} K): sky noise up to +{s['worst_rise_db']:.1f} dB, range reduced there")
    # environment (step 2.5)
    if s.get("masked_by_structure_pct", 0.0) > 0.0:
        out.append(f"{s['masked_by_structure_pct']:.2f} % of the {label} positions masked by the ship structure "
                   f"(blockage above {env.BLOCK_MASK_DB:g} dB per pass), not scanned")
    if s.get("range_reduced_pct", 0.0) > 0.0:
        out.append(f"{s['range_reduced_pct']:.2f} % of the {label} positions meet a surface before the required range: "
                   f"requirement brought back to the surface distance minus the guard (step 3.1, P4)")
    if s.get("no_volume_pct", 0.0) > 0.0:
        out.append(f"{s['no_volume_pct']:.2f} % of the {label} positions meet a surface within the guard: no volume to "
                   f"search there, not scanned")
    if s.get("clutter_limited_pct", 0.0) > 0.0:
        out.append(f"{s['clutter_limited_pct']:.2f} % of the {label} positions clutter-limited (surface echo in the "
                   f"target's range cell above what any power can beat): blinded")
    for sid, v in s.get("surfaces", {}).items():
        if v["shadowed_pct"] > 0.0:
            out.append(f"{v['shadowed_pct']:.2f} % of the {label} positions look at {sid} closer than the required "
                       f"range (down to {v['min_distance_km']:.0f} km): nothing can come from behind its surface")
        if v.get("doppler_rejected"):
            out.append(f"{sid} clutter rejected in Doppler (its closing-speed band misses the target span): needs "
                       f"coherent processing")
    if s.get("limbs"):
        out.append(f"clutter edge in the {label} sector (limb of {', '.join(s['limbs'])}): GO-CFAR advised there "
                   f"(choice per position at 4.3 / 5.2)")
    if s.get("adc_margin_db_min") is not None and s["adc_margin_db_min"] < 0.0:
        out.append(f"main-lobe surface echo clips the per-element {label} ADCs by {-s['adc_margin_db_min']:.1f} dB "
                   f"(fixed gain; attenuator option at 3.3)")
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


def _optimise_task(sec: dict, eps_min: float, eps_max: float, p_avg_max: float, sky: Optional[dict] = None) -> dict:
    """Worker-process entry point (module level so it can be pickled)."""
    sph = _sphere(eps_min, eps_max)
    return optimise_sector(sec, sph, sph.mask(sec), p_avg_max, sky)


def _executor() -> ProcessPoolExecutor:
    global _EXECUTOR
    if _EXECUTOR is None:
        _EXECUTOR = ProcessPoolExecutor(max_workers=MAX_WORKERS, initializer=_mark_pool_worker)   # spawned on demand
    return _EXECUTOR


def _run_sequential(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb, sky=None) -> Tuple[List[dict], bool]:
    results = []
    for i, sec in enumerate(sectors):
        if cancel_cb is not None and cancel_cb():
            return results, True
        results.append(_optimise_task(sec, eps_min, eps_max, p_avg_max, sky))
        if progress_cb is not None:
            progress_cb(i + 1, len(sectors), sec["id"])
    return results, False


def _run_parallel(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb, idle=None,
                  sky=None) -> Tuple[List[dict], bool]:
    """Sectors spread over the worker pool. Cancel is polled every 0.2 s: queued sectors are dropped at once,
    sectors already running finish in the background and their results are discarded. idle() is a short
    main-process job run while waiting (returns False when it has nothing left to do)."""
    global _EXECUTOR
    out: List[Optional[dict]] = [None] * len(sectors)
    try:
        ex = _executor()
        index = {ex.submit(_optimise_task, sec, eps_min, eps_max, p_avg_max, sky): i for i, sec in enumerate(sectors)}
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


def overlaps(d_omega: np.ndarray, masks: Dict[str, np.ndarray], min_sr: float = 1e-4) -> List[Tuple[str, str, float]]:
    """Pairwise sector overlaps (a, b, solid angle in sr) above min_sr. Each pair is evaluated only on the
    intersection of the two bounding boxes, so disjoint sectors cost nothing and the n^2 pair loop stays cheap as
    the sector count grows."""
    boxes = {sid: _mask_box(m) for sid, m in masks.items()}
    ids, out = list(masks), []
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            ba, bb = boxes[a], boxes[b]
            if ba is None or bb is None:
                continue
            r0, r1, c0, c1 = max(ba[0], bb[0]), min(ba[1], bb[1]), max(ba[2], bb[2]), min(ba[3], bb[3])
            if r0 >= r1 or c0 >= c1:
                continue
            sl = (slice(r0, r1), slice(c0, c1))
            ov = float((d_omega[sl] * (masks[a][sl] & masks[b][sl])).sum())
            if ov > min_sr:
                out.append((a, b, ov))
    return out


def run(mission: dict, progress_cb=None, cancel_cb=None, workers: Optional[int] = None) -> dict:
    """progress_cb(done, total, sector_id) is called after each sector (optional, e.g. for a UI
    progress bar). cancel_cb() is polled between sectors (every 0.2 s in parallel mode); if it returns True,
    run() stops early and returns the partial result with "cancelled": True (sectors not computed are absent).
    workers: None = automatic (parallel from PARALLEL_MIN_SECTORS sectors), 1 = sequential, > 1 = parallel.
    Step 3.0: the volumes and the sky are first turned into the ship frame with the mission navigation (core/frames.py)."""
    framed = frames.uses_frames(mission)
    mission, frame_alerts = frames.resolve(mission)
    mission = with_spectrum(mission)                 # step 3.1: the S exclusions seen by every sector
    g = mission.get("global", {})
    eps_min, eps_max = g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0)
    elec = g.get("face_elec_mw", rm.FACE_ELEC_POWER_MW)
    p_avg_max = rm.face_elec_avail_w(elec)
    sky = g.get("sky")
    skym.parse(sky)                  # refuses an invalid sky before any work
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
                                           idle=build_next_mask, sky=sky)
    else:
        results, cancelled = _run_sequential(sectors, eps_min, eps_max, p_avg_max, progress_cb, cancel_cb, sky)
    sph = _sphere(eps_min, eps_max)
    done_ids = {r["id"] for r in results}
    for sec in sectors:   # masks not built yet (sequential run, or workers faster than the main process)
        if sec["id"] in done_ids and sec["id"] not in masks:
            masks[sec["id"]] = sph.mask(sec)
    masks = {sec["id"]: masks[sec["id"]] for sec in sectors if sec["id"] in done_ids}   # mission order
    # mission totals per face
    tot = {k: {"energy_fraction": 0.0, "time_fraction": 0.0, "avg_power_kW": 0.0, "positions": 0.0,
               "dissipation_kW": 0.0} for k in range(4)}
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
    warn = [f"sectors {a} / {b} share {ov:.3f} sr: each is budgeted independently there -- fine for intentional "
            f"layered coverage (different RCS/range/RR on the same sky), check for a duplicated requirement if not"
            for a, b, ov in overlaps(sph.d_omega, masks)] + frame_alerts
    # tracking margin: power left after search (the S and Ku arrays only share the face's electrical power)
    for k, v in tot.items():
        p_avail_w = max(0.0, 1.0 - v["energy_fraction"]) * p_avg_max
        v["power_margin"] = 1.0 - v["energy_fraction"]
        v["p_available_kW"] = p_avail_w / 1e3
        v["ku_duty_pct"] = 100.0 * p_avail_w * rm.BAND_KU.pae_sat / rm.BAND_KU.p_pk_total_w
        v["time_margin_s"] = 1.0 - v["time_fraction"]      # S-band TX time only; Ku is a separate array
        if v["positions"] <= 0:
            continue
        if v["energy_fraction"] > 1.0 or v["time_fraction"] > 1.0:
            warn.append(f"face F{k + 1} over-subscribed: energy {100 * v['energy_fraction']:.0f} %, "
                        f"time on air {100 * v['time_fraction']:.0f} % (RR stretched x{dil[k]:.2f})")
        elif v["power_margin"] < TRACK_MARGIN_WARN:
            warn.append(f"face F{k + 1} tracking margin < {100 * TRACK_MARGIN_WARN:.0f} % "
                        f"({v['p_available_kW']:.0f} kW left for Ku)")
    out = {"global": {"eps_min_deg": eps_min, "eps_max_deg": eps_max, "face_elec_mw": elec,
                      "face_elec_avail_kW": p_avg_max / 1e3, "objective": "max(energy,time)",
                      "bin_deg": BIN_DEG, "max_wf_per_sector": MAX_WF_PER_SECTOR,
                      "sky_bodies": [b["id"] for b in (sky or {}).get("bodies", [])],
                      "sky_galactic": skym.parse(sky).galactic},
           "cancelled": cancelled,
           "sectors": results,
           "faces_total": {str(k): {kk: round(vv, 4) for kk, vv in v.items()} for k, v in tot.items()},
           "warnings": warn}
    if framed:   # step 3.0: the navigation used and each sector's frame (absent for a ship-frame mission)
        out["global"]["nav"] = frames.parse_nav(g.get("nav")).summary()
        fr = {sec["id"]: sec.get("frame", "ship") for sec in sectors}
        for r in results:
            r["frame"] = fr[r["id"]]
    from . import tracking   # imported here: core.tracking imports this module
    out["tracking"] = tracking.budget(mission, out)
    warn.extend(out["tracking"].pop("warnings"))
    return out


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
                    if k in ("id", "shape", "confirm_mode", "target_type", "fluctuation", "jammer_type", "frame"):
                        s[k] = v
                    elif k == "allow_ambiguous":
                        s[k] = v.lower() == "true"
                    elif k == "tracking":
                        s[k] = json.loads(v)      # AT range brackets, a JSON list in one column
                    else:
                        s[k] = float(v)
                secs.append(s)
        return {"global": {}, "sectors": secs}
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def print_report(res: dict) -> None:
    g = res["global"]
    print("=" * 100)
    print(f" MISSION PRE-OPTIMISATION | eps window [{g['eps_min_deg']:.0f}, {g['eps_max_deg']:.0f}] deg | "
          f"per face: {g['face_elec_mw']} MW elec -> {g['face_elec_avail_kW']:.0f} kW for transmission (electrical)")
    print("=" * 100)
    for r in res["sectors"]:
        if r["status"] != "OK":
            print(f"[{r['id']}] {r['status']}: {r.get('note','')}")
            continue
        print(f"[{r['id']}] {len(r['wf_set'])} waveform(s)")
        for w in r["wf_set"]:
            print(f"     {w['name']}  theta {w['theta_lo_deg']:.0f}-{w['theta_hi_deg']:.0f} deg ({w['positions']:.0f} pos)  "
                  f"{w['freq_ghz']} GHz {w['integration']}  tau {w['tau_us']} us x N {w['n_pulses']} "
                  f"@ PRI {w['pri_ms']} ms  B {w['sig_bw_mhz']:.0f} MHz  TX {tx_label(w)}  dwell {w['dwell_ms']} ms  "
                  f"(Dopp. max {w['doppler_max_kHz']} kHz)")
        rg = r["range_km"]
        print(f"     Range: wanted {rg['requested']:.0f} km | worst scan {rg['at_worst_scan']} km | best scan {rg['at_best_scan']} km"
              f" | positions: {r['positions_total']:.0f} (lattice {r.get('beam_spacing_hpbw', rm.BEAM_SPACING_HPBW):g} "
              f"HPBW) | RX sidelobes {r['rx_sll_db']:g} dB, {r['rx_beams_per_position']:.1f} RX beams / position")
        tg = r["target"]
        print(f"     Target: {tg['type']} | RCS {tg['rcs_m2']:.4g} m2 (optics {tg['optics_rcs_m2']:.3g}) | {tg['fluctuation']} | "
              f"T0 {tg['t0_s']:.2f} s | depol {tg['depol_db']:g} dB | accel max {tg['accel_max_mps2']:g} m/s2")
        ev = r["environment"]
        env_txt = []
        if ev["burn_through_km"] is not None:
            env_txt.append(f"burn-through {ev['jammer_type']} {ev['burn_through_km']:.0f} km "
                           f"(best {ev['burn_through_best_km']:.0f} km)")
        if ev["faraday_deg"] is not None:
            env_txt.append(f"Faraday {ev['faraday_deg']['S']:.0f} deg S / {ev['faraday_deg']['Ku']:.1f} deg Ku (no "
                           f"detection loss, dual-pol)")
        if ev["objects_per_rr"]:
            env_txt.append(f"{ev['objects_per_rr']:g} objects to confirm per RR")
        if env_txt:
            print("     Environment: " + " | ".join(env_txt))
        sk = r["sky"]
        if sk["t_sys_budget_k"] is not None:
            print(f"     Sky (S): budget T_sys {sk['t_sys_budget_k']:.1f} K (cold sky {sk['t_sys_cold_k']:.1f} K, "
                  f"+{sk['rise_db']:.2f} dB) | blinded {sk['blinded_pct']:.2f} % | galactic {sk['galactic']} | "
                  f"ADC unsaturated beyond {r['s_min_unsaturated_range_km']:.1f} km")
        for k, fc in r["faces"].items():
            print(f"     face {k}: {fc['positions']:.0f} pos, theta<={fc['theta_max_deg']:.0f} deg | energy {100*fc['energy_fraction']:.1f} % "
                  f"| avg {fc['avg_power_kW']:.0f} kW elec | heat {fc['dissipation_kW']:.0f} kW | time on air "
                  f"{100*fc['time_fraction']:.2f} %")
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
    trk = res.get("tracking")
    if trk:
        print("-" * 100)
        print(f" CONFIRMATION + ACTIVE TRACKING (Ku {trk['ku_freq_ghz']} GHz, Pfa {trk['pfa']:g}, "
              f"P confirm >= {trk['confirm_p_min']})")
        for s in trk["sectors"]:
            c = s["confirmation"]
            m = c.get("margin_db")
            m_txt = "-" if m is None else (f"S {m[0]:g} dB / Ku {m[1]:g} dB" if isinstance(m, list) else f"{m:g} dB")
            print(f"[{s['id']}] confirm {s['confirm_mode']} ({s['reason']}) | {s['false_alarms_per_rr']:.0f} false alarms / RR"
                  f" | P confirm {c.get('success', 0):.3f} | margin {m_txt} | Ku looks {c.get('ku_looks_true', 0):.2f} "
                  f"per target, {c.get('ku_looks_fa', 0):.0f} per false alarm")
            for b in s["at"]:
                print(f"     AT {b['r_min_km']:.0f}-{b['r_max_km']:.0f} km: {b['n_targets']:g} targets / {b['rr_s']:g} s, "
                      f"zone {b['angle_zone_deg']:g} deg, {b['range_acc_m']:g} m, {b['vr_acc_mps']:g} m/s -> "
                      f"{b['energy_per_update_J']:.0f} J/update, {b['avg_power_kW']:.1f} kW, driver {b['driver']} | {b['wf']}")
        for k, v in trk["faces"].items():
            if res["faces_total"][k]["positions"] > 0:
                print(f" face {k}: energy {100 * v['total_energy']:.1f} % (search {100 * v['search_energy']:.1f} + confirm "
                      f"{100 * v['confirm_energy']:.1f} + AT {100 * v['at_energy']:.1f}) | S time {100 * v['total_s_time']:.1f} % "
                      f"| Ku time {100 * v['total_ku_time']:.1f} % -> {v['status']} (load {v['load']:.2f}, {v['binding']})")
        for rule in trk.get("scheduler_rules", []):
            print(" SCHEDULER RULE (step 4.3):", rule)
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
