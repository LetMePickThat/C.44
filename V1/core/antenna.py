#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ANTENNA -- radiation pattern of one face computed from the array build-up (roadmap steps 1.1 and 1.2).

The element positions come from [aperture] (TRSA rows of the octagon, TRSA side, dead gaps, TRMs per TRSA side);
the elements are isotropic (the active mismatch is radar_model.active_mismatch). The pattern is evaluated near boresight
in units of lambda / D_env (D_env = the widest row, radar_model.ARRAY_ENVELOPE_M): there it does not depend on the
frequency, only on the element layout, so one computation per band holds for its whole band.

  TX beam : uniform illumination (PAs at saturation)            -> search lattice (radar_model.beam_cell_area_uv)
  RX beam : circular Taylor weighting (nbar = aperture.rx_taper_nbar) whose design level is fitted numerically so
            that the ACHIEVED peak sidelobe level of the real octagon equals each operator option of
            aperture.rx_sll_options_db (per watch volume, mission key "rx_sll_db") -> eta_illum_rx, RX beamwidth
  x_rms   : RMS extent of the aperture -> Cramer-Rao bound of the angle (radar_model.tracking_snr_req)
  mean    : (step 1.5) AZIMUTHAL MEAN of the power pattern vs the distance r to the beam axis, near window (step
            R_STEP, from the cuts) and visible region (step FAR_PROFILE_STEP, annulus means of the 2D grid): the noise
            a body in front of the face sends into the beam (core/sky.py) is T_b x (A eta / D_env^2) x the integral of
            this mean over the body seen in (u, v), in lambda / D_env units (normalisation exact by Parseval)
  far     : (step 1.2) sidelobes over the whole visible region of the band (radius (1 + sin scan limit) D / lambda
            at the top of the band), as a profile: peak level of the lobes within a distance r of the beam axis
            (cumulative maximum, step FAR_PROFILE_STEP). A lobe at distance r is visible once scanned to theta with
            r lambda / D <= 1 + sin(theta) (worst azimuth), so radar_model.sidelobe_visible_db() gives the worst
            sidelobe of a scan direction. Includes the gap lobes of the TRSA lattice (k lambda / TRSA pitch, weighted
            by the subarray factor of one TRSA: the ones next to the grating lobe reached about -17 dB in Ku with the former
            2 mm gaps, about -25 dB with the 0.5 mm gaps of step 1.2);
            the grating lobes (multiples of D / d_mean, d_mean = TRSA pitch / TRMs per side) and their near skirts
            are left to the mismatch model

User decisions (step 1.1, 2026-10-20): Taylor nbar 5 fitted on the octagon; options -25 / -30 / -35 dB, -25 by default;
one setting per volume for S and Ku; eta_illum_rx derived (no longer an entry); worst axis kept.

Symmetry: rows centred and palindromic (checked), so the array factor is real, sum of w cos(k x u) cos(k y v) over one
quadrant; with x <-> y symmetry too (the octagon) only the cuts 0..45 deg are evaluated. Each cut is a matrix product.

Results are cached in core/antenna_cache.json under a hash of their inputs (written only for the default parameter
file, so a test on a modified copy never rewrites it).
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass, replace
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

try:
    from . import params
    from .params import P
except ImportError:            # imported by core/radar_model.py run as a script
    import params
    from params import P

ALGO_VERSION = 5                    # bump when the computation below changes (invalidates the cache)
CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "antenna_cache.json")
_T = "tool.pattern."
R_MAX = P.get(_T + "r_max_lam_d")            # evaluated radius around boresight, lambda / D_env
R_STEP = P.get(_T + "r_step_lam_d")          # radial step of a cut
PHI_STEP_DEG = P.get(_T + "phi_step_deg")    # azimuth step between cuts
FIT_TOL_DB = P.get(_T + "fit_tol_db")        # tolerance of the fitted sidelobe level (achieved <= target)
FAR_STEP = P.get(_T + "far_step_lam_d")      # grid step of the far-sidelobe scan over the visible region
FAR_PROFILE_STEP = 0.5                       # radius step of the stored far-sidelobe profile, lambda / D_env


@dataclass(frozen=True)
class Beam:
    eta: float                       # illumination (taper) efficiency, 1 for uniform
    hpbw: float                      # -3 dB beamwidth, lambda / D_env: SMALLEST over the cuts (worst case)
    hpbw_x: float                    # along the TRSA rows (phi = 0)
    hpbw_diag: float                 # phi = 45 deg
    hpbw_max: float                  # largest over the cuts
    sll_db: float                    # achieved peak sidelobe level (outside the first null), dB below the peak
    design_sll_db: Optional[float]   # design level of the circular Taylor law (None = uniform)
    far_sll_db: Optional[float] = None   # peak sidelobe beyond the near window, over the visible region (step 1.2)
    far_sll_r: Optional[float] = None    # its distance to the beam axis, lambda / D_env
    far_profile_db: Optional[Tuple[float, ...]] = None   # peak level of the lobes within i x FAR_PROFILE_STEP
    mean_near: Optional[Tuple[float, ...]] = None   # azimuthal mean power (linear, peak = 1) at r = i x R_STEP <= R_MAX
    mean_far: Optional[Tuple[float, ...]] = None    # annulus mean power for r in [i, i + 1) x FAR_PROFILE_STEP, > R_MAX


@dataclass(frozen=True)
class BandPattern:
    n_elements: int
    x_rms_m: float                   # RMS extent of the (uniform) aperture along one axis, m
    tx: Beam
    rx: Dict[float, Beam]            # per sidelobe option (dB, negative)
    d_mean_m: float = 0.0            # mean element pitch (TRSA pitch / TRMs per side): grating lobes
    visible_r_lam_d: float = 0.0     # radius of the scanned far-sidelobe region, lambda / D_env


# ----------------------------------------------------------------------------- special functions (numpy only)
_TAU = (np.arange(256) + 0.5) * (math.pi / 256)


def bessel_j(n: int, x) -> np.ndarray:
    """J_n(x) by its integral (1/pi) int_0^pi cos(n t - x sin t) dt, midpoint rule (spectrally accurate here)."""
    x = np.asarray(x, dtype=float)
    return np.cos(n * _TAU - x[..., None] * np.sin(_TAU)).mean(axis=-1)


def j1_zeros(count: int) -> np.ndarray:
    """First `count` positive zeros of J1 (McMahon start, Newton with J1' = J0 - J1 / x)."""
    out = []
    for m in range(1, count + 1):
        b = (m + 0.25) * math.pi
        x = b - 3.0 / (8.0 * b)
        for _ in range(30):
            j0, j1 = float(bessel_j(0, x)), float(bessel_j(1, x))
            step = j1 / (j0 - j1 / x)
            x -= step
            if abs(step) < 1e-14:
                break
        out.append(x)
    return np.array(out)


def taylor_circular(design_sll_db: float, nbar: int, p: np.ndarray) -> np.ndarray:
    """Circular Taylor (Hansen) aperture distribution at normalised radius p in [0, 1] (unnormalised)."""
    a = math.acosh(10.0 ** (design_sll_db / 20.0)) / math.pi
    mu = np.r_[0.0, j1_zeros(nbar) / math.pi]
    sigma = mu[nbar] / math.sqrt(a * a + (nbar - 0.5) ** 2)
    g = np.zeros_like(p, dtype=float)
    for m in range(nbar):
        if m == 0:
            f = 1.0
        else:
            num = np.prod([1.0 - mu[m] ** 2 / (sigma ** 2 * (a * a + (n - 0.5) ** 2)) for n in range(1, nbar)])
            den = np.prod([1.0 - mu[m] ** 2 / mu[n] ** 2 for n in range(1, nbar) if n != m])
            f = -float(bessel_j(0, math.pi * mu[m])) * num / den
        g += f / float(bessel_j(0, math.pi * mu[m])) ** 2 * bessel_j(0, math.pi * mu[m] * p)
    return g


# ----------------------------------------------------------------------------- array layout
def _quadrant(rows: Sequence[int], trsa_side_m: float, gap_m: float, n_sub: int):
    """Distinct x >= 0 and y >= 0 element coordinates of one quadrant, their multiplicities (1 on an axis, 2 off it)
    and the presence matrix [y, x]."""
    if list(rows) != list(rows)[::-1]:
        raise ValueError("aperture.trsa_rows must be palindromic (symmetric array) for the pattern computation")
    pitch = trsa_side_m + gap_m
    d = trsa_side_m / n_sub
    sub = (np.arange(n_sub) - (n_sub - 1) / 2.0) * d
    pts = []
    for r, n in enumerate(rows):
        yc = (r - (len(rows) - 1) / 2.0) * pitch
        xc = (np.arange(n) - (n - 1) / 2.0) * pitch
        xx, yy = np.meshgrid((xc[:, None] + sub[None, :]).ravel(), yc + sub)
        pts.append(np.c_[xx.ravel(), yy.ravel()])
    pts = np.round(np.vstack(pts), 9)
    q = pts[(pts[:, 0] >= 0) & (pts[:, 1] >= 0)]
    xs, ix = np.unique(q[:, 0], return_inverse=True)
    ys, iy = np.unique(q[:, 1], return_inverse=True)
    present = np.zeros((len(ys), len(xs)))
    present[iy, ix] = 1.0
    mult = np.outer(np.where(ys == 0, 1.0, 2.0), np.where(xs == 0, 1.0, 2.0))
    return xs, ys, mult, present, len(pts)


def _cuts(xs, ys, wm, d_env, phis_deg, r) -> np.ndarray:
    """Power pattern |AF|^2 / |AF(0)|^2 along each cut: rows = cuts, columns = radius r (lambda / D_env)."""
    out = np.empty((len(phis_deg), len(r)))
    a0 = wm.sum()
    for i, phi in enumerate(np.radians(phis_deg)):
        cx = np.cos(2.0 * math.pi * np.outer(r * math.cos(phi), xs) / d_env)
        cy = np.cos(2.0 * math.pi * np.outer(r * math.sin(phi), ys) / d_env)
        out[i] = (((cy @ wm) * cx).sum(axis=1) / a0) ** 2
    return out


def _analyse(xs, ys, mult, present, w, d_env, symmetric_xy: bool, design: Optional[float]) -> Beam:
    phis = np.arange(0.0, (45.0 if symmetric_xy else 90.0) + 1e-9, PHI_STEP_DEG)
    r = np.arange(0.0, R_MAX + R_STEP / 2, R_STEP)
    g = _cuts(xs, ys, mult * w, d_env, phis, r)
    hp, sll = [], -math.inf
    for row in g:
        i = int(np.argmax(row < 0.5))
        hp.append(2.0 * float(np.interp(0.5, [row[i], row[i - 1]], [r[i], r[i - 1]])))
        rising = np.flatnonzero(np.diff(row[i:]) > 0)
        if rising.size:
            sll = max(sll, float(row[i + rising[0]:].max()))
    hp = np.array(hp)
    wphi = np.ones(len(phis))                      # trapezoid weights: the cuts span one symmetry sector end to end
    wphi[[0, -1]] = 0.5
    mean_near = tuple(float(x) for x in (wphi[:, None] * g).sum(axis=0) / wphi.sum())
    eta = float((mult * w).sum() ** 2 / ((mult * present).sum() * (mult * w * w).sum()))
    return Beam(eta=eta, hpbw=float(hp.min()), hpbw_x=float(hp[0]), hpbw_diag=float(np.interp(45.0, phis, hp)),
                hpbw_max=float(hp.max()), sll_db=10.0 * math.log10(sll), design_sll_db=design, mean_near=mean_near)


def _far(xs, ys, wm, d_env: float, r_vis: float, d_mean: float) -> Tuple[float, float, Tuple[float, ...], Tuple[float, ...]]:
    """Peak power (dB) and radius (lambda / D_env) of the strongest lobe beyond the near window R_MAX, over the
    visible region r <= r_vis of one quadrant (2D grid, AF = Cy W Cx^T), grating-lobe neighbourhoods excluded, the
    cumulative profile (peak dB within i x FAR_PROFILE_STEP; -inf inside the near window) and the annulus means
    (linear; 0 inside the near window and in bins without a kept point)."""
    r = np.arange(0.0, r_vis + FAR_STEP / 2, FAR_STEP)
    cx = np.cos(2.0 * math.pi * np.outer(r, xs) / d_env)
    cy = np.cos(2.0 * math.pi * np.outer(r, ys) / d_env)
    g = ((cy @ wm) @ cx.T / wm.sum()) ** 2                       # [v, u]
    uu, vv = np.meshgrid(r, r)
    rad = np.hypot(uu, vv)
    keep = (rad > R_MAX) & (rad <= r_vis)
    p_gl = d_env / d_mean                                        # grating-lobe lattice step, lambda / D_env
    for i in range(int(r_vis // p_gl) + 2):
        for j in range(int(r_vis // p_gl) + 2):
            if i or j:
                keep &= np.hypot(uu - i * p_gl, vv - j * p_gl) > R_MAX
    k = int(np.argmax(np.where(keep, g, -1.0)))
    n_bins = int(math.ceil(r_vis / FAR_PROFILE_STEP)) + 1
    idx = np.minimum((rad / FAR_PROFILE_STEP).astype(int), n_bins - 1)
    peak = np.zeros(n_bins)
    np.maximum.at(peak, idx[keep], g[keep])
    prof = np.maximum.accumulate(peak)
    with np.errstate(divide="ignore"):
        prof_db = tuple(float(x) for x in 10.0 * np.log10(prof))
    wq = np.where(uu == 0.0, 0.5, 1.0) * np.where(vv == 0.0, 0.5, 1.0)   # axis points are shared by two quadrants
    s_g, s_w = np.zeros(n_bins), np.zeros(n_bins)
    np.add.at(s_g, idx[keep], (wq * g)[keep])
    np.add.at(s_w, idx[keep], wq[keep])
    mean = tuple(float(x) for x in np.divide(s_g, s_w, out=np.zeros(n_bins), where=s_w > 0))
    return 10.0 * math.log10(float(g.flat[k])), float(rad.flat[k]), prof_db, mean


def _fit_taylor(xs, ys, mult, present, d_env, sym, target_db: float, nbar: int, edge_r: float) -> Tuple[Beam, np.ndarray]:
    """Weakest circular Taylor law (highest efficiency) whose achieved peak sidelobe level is <= target_db: the design
    level is searched by regula falsi between the target and the target - 40 dB."""
    xx, yy = np.meshgrid(xs, ys)
    p = np.minimum(np.hypot(xx, yy) / edge_r, 1.0)

    def weights(design: float) -> np.ndarray:
        return present * np.clip(taylor_circular(design, nbar, p), 0.0, None)

    def beam(design: float) -> Beam:
        return _analyse(xs, ys, mult, present, weights(design), d_env, sym, design)

    lo, hi = abs(target_db), abs(target_db) + 40.0
    b_lo, b_hi = beam(lo), beam(hi)
    if b_lo.sll_db <= target_db:
        return b_lo, weights(lo)
    if b_hi.sll_db > target_db:
        raise ValueError(f"no circular Taylor law (nbar {nbar}) reaches {target_db} dB on this array")
    f_lo, f_hi = b_lo.sll_db - target_db, b_hi.sll_db - target_db
    side = 0
    for _ in range(40):
        mid = (lo * f_hi - hi * f_lo) / (f_hi - f_lo)
        b = beam(mid)
        f = b.sll_db - target_db
        if f > 0:
            lo, f_lo, b_lo = mid, f, b
            if side == -1:
                f_hi /= 2.0
            side = -1
        else:
            hi, f_hi, b_hi = mid, f, b
            if side == 1:
                f_lo /= 2.0
            side = 1
        if -FIT_TOL_DB <= f <= 0.0 or hi - lo < 1e-3:
            break
    return b_hi, weights(b_hi.design_sll_db)


# ----------------------------------------------------------------------------- public entry
def _compute(rows, trsa_side_m, gap_m, n_sub, d_env, sll_options, nbar, edge, r_vis) -> BandPattern:
    xs, ys, mult, present, n = _quadrant(rows, trsa_side_m, gap_m, n_sub)
    sym = len(xs) == len(ys) and np.allclose(xs, ys) and np.array_equal(present, present.T)
    edge_r = d_env / 2.0 if edge == "inscribed" else float(np.hypot(*np.meshgrid(xs, ys))[present > 0].max()
                                                             + trsa_side_m / n_sub / 2.0)
    x_rms = math.sqrt(float((mult * present * xs[None, :] ** 2).sum() / (mult * present).sum()))
    d_mean = (trsa_side_m + gap_m) / n_sub

    def with_far(b: Beam, w: np.ndarray) -> Beam:
        far_db, far_r, prof, mean = _far(xs, ys, mult * w, d_env, r_vis, d_mean)
        return replace(b, far_sll_db=far_db, far_sll_r=far_r, far_profile_db=prof, mean_far=mean)

    tx = with_far(_analyse(xs, ys, mult, present, present, d_env, sym, None), present)
    rx = {float(s): with_far(*_fit_taylor(xs, ys, mult, present, d_env, sym, float(s), nbar, edge_r))
          for s in sll_options}
    return BandPattern(n_elements=n, x_rms_m=x_rms, tx=tx, rx=rx, d_mean_m=d_mean, visible_r_lam_d=r_vis)


def input_key(rows: Sequence[int], trsa_side_m: float, gap_m: float, n_sub: Dict[str, int], d_env: float,
              sll_options: Sequence[float], nbar: int, edge: str, visible_r: Dict[str, float]) -> Tuple[dict, str]:
    """Everything the patterns depend on, and its hash (the cache key)."""
    inputs = {"algo": ALGO_VERSION, "rows": list(rows), "side": trsa_side_m, "gap": gap_m, "n_sub": dict(n_sub),
              "d_env": d_env, "sll": [float(s) for s in sll_options], "nbar": nbar, "edge": edge,
              "visible_r": dict(visible_r), "r_max": R_MAX, "r_step": R_STEP, "phi_step": PHI_STEP_DEG,
              "fit_tol": FIT_TOL_DB, "far_step": FAR_STEP}
    return inputs, hashlib.sha256(json.dumps(inputs, sort_keys=True).encode("utf-8")).hexdigest()[:16]


def face_patterns(rows: Sequence[int], trsa_side_m: float, gap_m: float, n_sub: Dict[str, int], d_env: float,
                  sll_options: Sequence[float], nbar: int, edge: str,
                  visible_r: Dict[str, float]) -> Dict[str, BandPattern]:
    """Pattern of every band (n_sub = TRMs per TRSA side of each band; visible_r = radius of its visible region,
    lambda / D_env), from the cache when its inputs are unchanged."""
    inputs, key = input_key(rows, trsa_side_m, gap_m, n_sub, d_env, sll_options, nbar, edge, visible_r)
    cached = _read_cache(key)
    if cached is not None:
        return cached
    out = {b: _compute(rows, trsa_side_m, gap_m, k, d_env, sll_options, nbar, edge, visible_r[b])
           for b, k in n_sub.items()}
    if os.path.normcase(os.path.abspath(P.path)) == os.path.normcase(os.path.abspath(params.DEFAULT_PATH)):
        with open(CACHE_PATH, "w", encoding="utf-8") as fh:
            json.dump({"key": key, "inputs": inputs, "bands": {b: asdict(p) for b, p in out.items()}}, fh)
    return out


def _read_cache(key: str) -> Optional[Dict[str, BandPattern]]:
    try:
        with open(CACHE_PATH, encoding="utf-8") as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        return None
    if d.get("key") != key:
        return None
    def beam(v: dict) -> Beam:
        return Beam(**{**v, **{k: tuple(v[k]) if v.get(k) else None for k in ("far_profile_db", "mean_near", "mean_far")}})

    return {b: BandPattern(n_elements=p["n_elements"], x_rms_m=p["x_rms_m"], tx=beam(p["tx"]),
                           rx={float(s): beam(v) for s, v in p["rx"].items()}, d_mean_m=p["d_mean_m"],
                           visible_r_lam_d=p["visible_r_lam_d"]) for b, p in d["bands"].items()}

