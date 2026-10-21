#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SKY -- rise of the antenna noise temperature by direction (roadmap step 1.5).

The mission gives the sky in the SHIP frame, global key "sky" (later the global scenario simulator fills it: the radar
model never computes where a body is):

  "sky": {"bodies": [{"id": "SUN", "type": "sun", "phi_deg": 30.0, "eps_deg": 10.0, "angular_radius_deg": 0.267,
                      "tb_k": optional, a number or {"S": ..., "Ku": ...}}, ...],
          "galactic": {"pole_phi_deg": ..., "pole_eps_deg": ..., "centre_phi_deg": ..., "centre_eps_deg": ...}}

T_ant(beam) = cold sky (band.<b>.rx.t_ant_k, inside T_sys) + galactic rise + the sum over the bodies:
  galactic : T_gal(l, b) - sky.gal_t_high_k (already in t_ant_k), at the beam direction, x (f / gal_ref_ghz)^-index;
             without a galactic frame: the worst case gal_t_centre_k - gal_t_high_k everywhere (flagged).
  body     : T_b x (A eta_rx / D_env^2) x the integral, over the body seen in the face's (u, v) (front half only, the
             ground plane hides the rear), of the AZIMUTHAL-MEAN RX power pattern g(rho) of core/antenna.py, rho =
             distance to the beam axis in lambda / D_env. This is (1 / 4 pi) int G dOmega with G = G0 w g (element
             pattern cos theta, sky.noise_element_pattern) and dOmega = du dv / w; A eta / D^2 int g = 1 over the plane
             (Parseval), so a body filling the whole front half-space adds T_b. Near zone (rho <= tool.pattern.r_max):
             polar grid around the beam axis, points inside the body; beyond: rings of samples over the body disk.

The sector budget (core/optimizer.py, core/tracking.py) blinds the positions whose T_sys rises above sky.blind_rise_db
and takes the (1 - short_tol) position quantile of T_sys over the others (summarise()).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import antenna
from . import radar_model as rm
from .params import P

BAND_ORDER = ("S", "Ku")                          # order of the [S, Ku] values of [sky.tb_k]
BLIND_RISE_DB: float = P.get("sky.blind_rise_db")
NOISE_ELEMENT_PATTERN: str = P.get("sky.noise_element_pattern")
if NOISE_ELEMENT_PATTERN != "cos_front":
    raise ValueError(f"{P.path}: sky.noise_element_pattern '{NOISE_ELEMENT_PATTERN}' is not implemented (cos_front)")
GAL_REF_GHZ: float = P.get("sky.gal_ref_ghz")
GAL_T_HIGH_K: float = P.get("sky.gal_t_high_k")
GAL_T_PLANE_K: float = P.get("sky.gal_t_plane_k")
GAL_T_CENTRE_K: float = P.get("sky.gal_t_centre_k")
GAL_SIGMA_B: float = math.radians(P.get("sky.gal_sigma_b_deg"))
GAL_SIGMA_L: float = math.radians(P.get("sky.gal_sigma_l_deg"))
GAL_INDEX: float = P.get("sky.gal_spectral_index")
TB_K: Dict[str, Tuple[float, float]] = {k: tuple(float(x) for x in v) for k, v in P.section("sky.tb_k").items()}
for _k, _v in TB_K.items():
    if len(_v) != len(BAND_ORDER) or min(_v) < 0:
        raise ValueError(f"{P.path}: sky.tb_k.{_k} must be [S, Ku] brightness temperatures >= 0 K")
if not GAL_T_HIGH_K <= GAL_T_PLANE_K <= GAL_T_CENTRE_K:
    raise ValueError(f"{P.path}: [sky] needs gal_t_high_k <= gal_t_plane_k <= gal_t_centre_k")

_T = "tool.sky."
EVAL_STEP_HPBW: float = P.get(_T + "eval_step_hpbw")      # evaluation step over a sector, in TX beamwidths
NEAR_R_STEP: float = P.get(_T + "near_r_step_lam_d")
NEAR_PHI_STEP: float = math.radians(P.get(_T + "near_phi_step_deg"))
RING_LAM_D: float = P.get(_T + "body_ring_lam_d")
SMALL_BODY_LAM_D: float = P.get(_T + "small_body_lam_d")   # bodies up to this radius: fine disk samples only
RINGS_MAX: int = P.get(_T + "body_rings_max")
CHUNK: int = P.get(_T + "chunk")


# ----------------------------------------------------------------------------- mission input
@dataclass(frozen=True)
class Body:
    id: str
    type: str
    c: Tuple[float, float, float]      # unit vector to the centre, ship frame
    radius_rad: float
    tb_k: Tuple[float, float]          # [S, Ku]


@dataclass(frozen=True)
class Sky:
    bodies: Tuple[Body, ...] = ()
    pole: Optional[Tuple[float, float, float]] = None      # galactic frame in the ship frame (None = worst case)
    centre: Optional[Tuple[float, float, float]] = None

    @property
    def galactic(self) -> str:
        return "frame" if self.pole is not None else "worst case (no galactic frame given)"


def unit(phi_deg, eps_deg) -> np.ndarray:
    """Ship-frame unit vector(s): phi about Z from +X toward +Y, eps from the XY plane toward +Z (the optimizer frame)."""
    p, e = np.radians(phi_deg), np.radians(eps_deg)
    return np.stack([np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)], axis=-1)


def validate(sky: Optional[dict]) -> List[str]:
    """Operator-facing messages of the mission sky (empty = valid)."""
    if sky is None:
        return []
    if not isinstance(sky, dict):
        return ["sky must be an object with 'bodies' and an optional 'galactic' frame"]
    errs = []
    ids = []
    for i, b in enumerate(sky.get("bodies", [])):
        tag = f"sky body #{i + 1}" + (f" ({b.get('id')})" if isinstance(b, dict) and b.get("id") else "")
        if not isinstance(b, dict):
            errs.append(f"{tag}: must be an object")
            continue
        missing = [f for f in ("id", "type", "phi_deg", "eps_deg", "angular_radius_deg") if b.get(f) is None]
        if missing:
            errs.append(f"{tag}: missing {', '.join(missing)}")
            continue
        ids.append(b["id"])
        if not -90.0 <= b["eps_deg"] <= 90.0:
            errs.append(f"{tag}: eps_deg ({b['eps_deg']}) outside [-90, 90] deg")
        if not 0.0 < b["angular_radius_deg"] <= 90.0:
            errs.append(f"{tag}: angular_radius_deg ({b['angular_radius_deg']}) must be in ]0, 90] deg")
        tb = b.get("tb_k")
        if tb is None and b["type"] not in TB_K:
            errs.append(f"{tag}: type '{b['type']}' has no brightness temperature in sky.tb_k "
                        f"({', '.join(TB_K)}): give tb_k")
        elif isinstance(tb, dict):
            if set(tb) != set(BAND_ORDER) or min(tb.values()) < 0:
                errs.append(f"{tag}: tb_k must give S and Ku temperatures >= 0 K")
        elif tb is not None and (not isinstance(tb, (int, float)) or tb < 0):
            errs.append(f"{tag}: tb_k must be a temperature >= 0 K or {{'S': ..., 'Ku': ...}}")
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    if dupes:
        errs.append(f"duplicate sky body ids: {', '.join(map(str, dupes))}")
    g = sky.get("galactic")
    if g is not None:
        keys = ("pole_phi_deg", "pole_eps_deg", "centre_phi_deg", "centre_eps_deg")
        if not isinstance(g, dict) or any(g.get(k) is None for k in keys):
            errs.append(f"sky galactic frame needs {', '.join(keys)}")
        else:
            ang = math.degrees(math.acos(float(np.clip(unit(g["pole_phi_deg"], g["pole_eps_deg"])
                                                       @ unit(g["centre_phi_deg"], g["centre_eps_deg"]), -1, 1))))
            if abs(ang - 90.0) > 2.0:
                errs.append(f"sky galactic frame: pole and centre directions are {ang:.1f} deg apart (90 expected)")
    return errs


def parse(sky: Optional[dict]) -> Sky:
    """Mission sky -> Sky (ValueError with the validation messages if invalid). None = an empty sky."""
    errs = validate(sky)
    if errs:
        raise ValueError("; ".join(errs))
    if sky is None:
        return Sky()
    bodies = []
    for b in sky.get("bodies", []):
        tb = b.get("tb_k")
        if tb is None:
            tb_k = TB_K[b["type"]]
        elif isinstance(tb, dict):
            tb_k = tuple(float(tb[n]) for n in BAND_ORDER)
        else:
            tb_k = (float(tb),) * len(BAND_ORDER)
        bodies.append(Body(str(b["id"]), str(b["type"]), tuple(float(x) for x in unit(b["phi_deg"], b["eps_deg"])),
                           math.radians(b["angular_radius_deg"]), tb_k))
    g = sky.get("galactic")
    pole = centre = None
    if g is not None:
        pole = tuple(float(x) for x in unit(g["pole_phi_deg"], g["pole_eps_deg"]))
        centre = tuple(float(x) for x in unit(g["centre_phi_deg"], g["centre_eps_deg"]))
    return Sky(tuple(bodies), pole, centre)


def sky_key(sky: Optional[dict]) -> str:
    """Cache key of a mission sky."""
    return json.dumps(sky, sort_keys=True)


# ----------------------------------------------------------------------------- galactic background
def galactic_rise_k(sky: Sky, dirs: np.ndarray, f_ghz: float) -> np.ndarray:
    """Galactic brightness above its high-latitude minimum (inside t_ant_k) at the beam directions (main lobe)."""
    scale = (f_ghz / GAL_REF_GHZ) ** -GAL_INDEX
    if sky.pole is None:
        return np.full(len(dirs), (GAL_T_CENTRE_K - GAL_T_HIGH_K) * scale)
    p = np.array(sky.pole)
    c = np.array(sky.centre) - (np.array(sky.centre) @ p) * p
    c /= np.linalg.norm(c)
    y = np.cross(p, c)
    b = np.arcsin(np.clip(dirs @ p, -1.0, 1.0))
    lon = np.arctan2(dirs @ y, dirs @ c)
    eb = np.exp(-(b / GAL_SIGMA_B) ** 2)
    t = GAL_T_HIGH_K + (GAL_T_PLANE_K - GAL_T_HIGH_K) * eb + (GAL_T_CENTRE_K - GAL_T_PLANE_K) * eb * np.exp(-(lon / GAL_SIGMA_L) ** 2)
    return (t - GAL_T_HIGH_K) * scale


# ----------------------------------------------------------------------------- bodies
@dataclass(frozen=True)
class _Profile:
    near: np.ndarray        # azimuthal-mean power at r = i x antenna.R_STEP
    far: np.ndarray         # annulus means, bins of antenna.FAR_PROFILE_STEP
    norm: float             # A eta / D_env^2
    near_w: np.ndarray      # g x rho drho dphi on the near polar grid (one azimuth)
    near_rho: np.ndarray

    @property
    def near_full(self) -> float:
        """Near-zone integral when the whole near zone lies inside the body."""
        return float(self.near_w.sum()) * round(2.0 * math.pi / NEAR_PHI_STEP)

    def g_far(self, rho: np.ndarray) -> np.ndarray:
        idx = np.minimum((rho / antenna.FAR_PROFILE_STEP).astype(int), len(self.far) - 1)
        return np.where(rho > antenna.R_MAX, self.far[idx], 0.0)


@lru_cache(maxsize=16)
def _profile(band_name: str, rx_sll_db: float) -> _Profile:
    beam = rm.rx_beam(band_name, rx_sll_db)
    near = np.array(beam.mean_near)
    rho = (np.arange(int(round(antenna.R_MAX / NEAR_R_STEP))) + 0.5) * NEAR_R_STEP
    g = np.interp(rho, np.arange(len(near)) * antenna.R_STEP, near)
    return _Profile(near, np.array(beam.mean_far), rm.PHYS_AREA_M2 * beam.eta * rm.ETA_OHMIC / rm.ARRAY_ENVELOPE_M ** 2,
                    g * rho * NEAR_R_STEP * NEAR_PHI_STEP, rho)


def _disk_samples(body: Body, k_lam: float, ring_lam_d: float) -> Tuple[np.ndarray, np.ndarray]:
    """Directions sampling the body disk (equal-solid-angle rings about ring_lam_d apart, at most RINGS_MAX, 6 (2j + 1)
    points on ring j) and their solid angles."""
    n_r = int(min(max(math.ceil(body.radius_rad * k_lam / ring_lam_d), 1), RINGS_MAX))
    c = np.array(body.c)
    e1 = np.cross(c, [0.0, 0.0, 1.0] if abs(c[2]) < 0.9 else [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(c, e1)
    one_minus = 1.0 - math.cos(body.radius_rad)
    pts, dom = [], []
    for j in range(n_r):
        a = math.acos(1.0 - (j + 0.5) / n_r * one_minus)
        m = 6 * (2 * j + 1)
        psi = (np.arange(m) + 0.5) * (2.0 * math.pi / m)
        pts.append(math.cos(a) * c + math.sin(a) * (np.cos(psi)[:, None] * e1 + np.sin(psi)[:, None] * e2))
        dom.append(np.full(m, 2.0 * math.pi * one_minus / n_r / m))
    return np.vstack(pts), np.concatenate(dom)


def body_rise_k(body: Body, band_name: str, f_ghz: float, rx_sll_db: float, frame, dirs: np.ndarray) -> np.ndarray:
    """Antenna-temperature rise (K) from one body when the RX beams of a face (frame = (n, e_u, e_v)) point at dirs."""
    prof = _profile(band_name, float(rx_sll_db))
    k_lam = rm.ARRAY_ENVELOPE_M / rm.lambda_m(f_ghz)
    tb = body.tb_k[BAND_ORDER.index(band_name)]
    n, eu, ev = (np.asarray(x, dtype=float) for x in frame)
    c = np.array(body.c)
    ub, vb, wb = dirs @ eu, dirs @ ev, dirs @ n
    out = np.zeros(len(dirs))
    if body.radius_rad * k_lam <= SMALL_BODY_LAM_D:
        # small body (the Sun, the Moon): fine samples over its disk, the whole pattern (near and far)
        s, dom = _disk_samples(body, k_lam, 2.0 * NEAR_R_STEP)
        ws = s @ n
        keep = ws > 0.0
        if keep.any():
            us, vs, wt = s[keep] @ eu, s[keep] @ ev, ws[keep] * dom[keep] * k_lam ** 2
            r_near = np.arange(len(prof.near)) * antenna.R_STEP
            step = max(1, CHUNK * 64 // len(us))
            for lo in range(0, len(dirs), step):
                sl = slice(lo, lo + step)
                rho = np.hypot(us[None, :] - ub[sl, None], vs[None, :] - vb[sl, None]) * k_lam
                g = np.where(rho <= antenna.R_MAX, np.interp(rho, r_near, prof.near), prof.g_far(rho))
                out[sl] = (g * wt[None, :]).sum(axis=1)
        return tb * prof.norm * out
    # near zone (rho <= R_MAX): polar grid around each beam axis
    r_near_uv = antenna.R_MAX / k_lam
    delta = np.arccos(np.clip(dirs @ c, -1.0, 1.0))
    a_near = r_near_uv / np.maximum(wb, 0.05)                       # angular radius bound of the near zone
    full = (delta + a_near < body.radius_rad) & (np.hypot(ub, vb) + r_near_uv < 1.0)
    part = np.flatnonzero(~full & (delta - a_near <= body.radius_rad))
    out[full] = prof.near_full
    phis = (np.arange(round(2.0 * math.pi / NEAR_PHI_STEP)) + 0.5) * NEAR_PHI_STEP
    du = (np.outer(prof.near_rho, np.cos(phis)) / k_lam).ravel()
    dv = (np.outer(prof.near_rho, np.sin(phis)) / k_lam).ravel()
    gw = np.repeat(prof.near_w, len(phis))
    cu, cv, cw = float(eu @ c), float(ev @ c), float(n @ c)
    cos_r = math.cos(body.radius_rad)
    step = max(1, CHUNK * 64 // len(du))
    for lo in range(0, len(part), step):
        idx = part[lo:lo + step]
        uu = ub[idx, None] + du[None, :]
        vv = vb[idx, None] + dv[None, :]
        q = 1.0 - uu * uu - vv * vv
        inside = (q > 0.0) & (uu * cu + vv * cv + np.sqrt(np.maximum(q, 0.0)) * cw >= cos_r)
        out[idx] = (inside * gw[None, :]).sum(axis=1)
    # far part (rho > R_MAX): samples over the body disk, front half only
    s, dom = _disk_samples(body, k_lam, RING_LAM_D)
    ws = s @ n
    keep = ws > 0.0
    if keep.any():
        us, vs, wt = s[keep] @ eu, s[keep] @ ev, ws[keep] * dom[keep] * k_lam ** 2
        step = max(1, CHUNK * 64 // len(us))
        for lo in range(0, len(dirs), step):
            sl = slice(lo, lo + step)
            rho = np.hypot(us[None, :] - ub[sl, None], vs[None, :] - vb[sl, None]) * k_lam
            out[sl] += (prof.g_far(rho) * wt[None, :]).sum(axis=1)
    return tb * prof.norm * out


def rise_k(sky: Sky, band_name: str, f_ghz: float, rx_sll_db: float, frames: Sequence, dirs: np.ndarray,
           owners: np.ndarray) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Antenna-temperature rise at each beam direction (dirs, ship frame) of its owner face (index into frames):
    total and per contributor ("galactic" and each body id)."""
    parts = {"galactic": galactic_rise_k(sky, dirs, f_ghz)}
    for b in sky.bodies:
        r = np.zeros(len(dirs))
        for k in np.unique(owners):
            sel = owners == k
            r[sel] = body_rise_k(b, band_name, f_ghz, rx_sll_db, frames[int(k)], dirs[sel])
        parts[b.id] = r
    return sum(parts.values()), parts


# ----------------------------------------------------------------------------- sector budget
def summarise(band: rm.BandConfig, t_sys: np.ndarray, weights: np.ndarray, parts: Dict[str, np.ndarray],
              short_tol: float, sky: Sky) -> Tuple[dict, np.ndarray]:
    """Sector summary of the per-position T_sys (weights = position density): blinded positions (rise above
    BLIND_RISE_DB, with the main contributor), budget T_sys = (1 - short_tol) position quantile of the others, the
    positions above it. Returns (summary, blinded mask)."""
    t0 = band.t_sys_k
    blind = t_sys > t0 * rm.db_to_lin(BLIND_RISE_DB)
    w_all = float(weights.sum())
    out = {"band": band.name, "t_sys_cold_k": t0, "galactic": sky.galactic, "blind_rise_db": BLIND_RISE_DB}
    nb = ~blind
    if w_all <= 0.0 or not nb.any():
        out.update({"t_sys_budget_k": None, "rise_db": None, "blinded_pct": 100.0 if w_all > 0 else 0.0,
                    "blinded_by": [], "above_budget_pct": 0.0, "worst_rise_db": None})
        return out, blind
    order = np.argsort(t_sys[nb])
    ts, ws = t_sys[nb][order], weights[nb][order]
    cum = np.cumsum(ws) / ws.sum()
    budget = float(ts[min(int(np.searchsorted(cum, 1.0 - short_tol - 1e-12)), len(ts) - 1)])
    above = nb & (t_sys > budget * (1.0 + 1e-12))
    names = list(parts)
    by = {}
    if blind.any():
        main = np.argmax(np.stack([parts[k][blind] for k in names]), axis=0)
        for i, w in zip(main, weights[blind]):
            by[names[i]] = by.get(names[i], 0.0) + float(w)
    out.update({"t_sys_budget_k": budget, "rise_db": rm.lin_to_db(budget / t0),
                "blinded_pct": 100.0 * float(weights[blind].sum()) / w_all,
                "blinded_by": sorted(by, key=by.get, reverse=True),
                "above_budget_pct": 100.0 * float(weights[above].sum()) / w_all,
                "worst_rise_db": rm.lin_to_db(float(t_sys[nb].max()) / t0)})
    return out, blind
