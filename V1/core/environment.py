#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENVIRONMENT -- what surrounds a face (roadmap step 2.5): the ship structure, the surfaces of the bodies, the jammers.

Everything is in the SHIP frame (core/optimizer.face_frame); positions of bodies and jammers come from the mission sky
(later the global scenario simulator): the radar model never computes where an object is.

Ship structure ([ship] of radar_system.toml: a hull of revolution and parts, each of a material):
  masking   the aperture is a TUBE in the near field (Fraunhofer limit ~325 m in S): rays from points of the aperture
            (hexagonal rings) along the beam direction; f = fraction of rays hitting the structure; loss per pass
            -20 log(1 - f); a position is MASKED (not scanned) above ship.block_mask_db per pass. Table per face on a
            (u, v) grid, cached in core/structure_cache.npz (rebuilt when the geometry changes).
  noise     main lobe: f x (emissivity x T_phys of the parts hit) x the near-zone share of the RX pattern; far
            sidelobes: the structure seen from the array centre, integrated over the azimuthal-mean far profile of the
            RX pattern (as a body of core/sky.py). Reflections on the structure are not modelled (step 5.4).
Surfaces ([sky.surface]: radius and constant-gamma backscatter sigma0 = gamma cos(theta_i) per body type):
  shadow    a direction inside a body's disk ends on its surface at distance d: no target beyond (flagged when d is
            below the sector's range).
  clutter   at the target range r, the iso-range ring of the surface (area 2 pi R_b r dr / D per range cell dr)
            seen through the two-way pattern (TX x RX far profiles, element cos): C/S = sigma_c / sigma_target. Treated
            as noise: S >= SNR_req (N + C) gives the factor 1 / (1 - (C/S) SNR_req) on the noise, infinite (the position
            is clutter-limited) when (C/S) SNR_req >= 1. Rejected when the mission gives the surface's closing speed
            and spread and the target's speed span does not overlap it (Doppler processing).
  ADC       main-lobe surface echo per element, before beamforming (footprint x sigma0), against the clipping level.
Jammers ([jammers.*] classes, mission sky "jammers" for the stand-off ones):
  J (K) = EIRP density / (4 pi R^2) / k x A_eff(direction) on the RX pattern; sidelobe jammers nulled (the strongest
  first, up to jamming.max_nulled per face) by jamming.null_depth_db.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import antenna
from . import radar_model as rm
from .params import P, DEFAULT_PATH

BANDS = ("S", "Ku")
K_BOLTZ = rm.K_BOLTZ

# ----------------------------------------------------------------------------- ship structure (parameter file)
PART_TYPES = {"cylinder_z": ("radial_m", "z_m", "radius_m", "length_m"),
              "cylinder_rad": ("radial_m", "z_m", "radius_m", "length_m"),
              "box_rad": ("r_min_m", "r_max_m", "half_width_m", "z_min_m", "z_max_m"),
              "sphere": ("radial_m", "z_m", "radius_m")}


@dataclass(frozen=True)
class Material:
    emissivity: Tuple[float, float]       # RF emissivity [S, Ku]
    t_phys_k: float

    def eps_t(self, band: str) -> float:
        return self.emissivity[BANDS.index(band)] * self.t_phys_k


@dataclass(frozen=True)
class Part:
    name: str
    type: str
    material: str
    values: Tuple[Tuple[str, float], ...]
    azimuths_deg: Tuple[float, ...]

    def v(self, key: str) -> float:
        return dict(self.values)[key]


def _load_materials() -> Dict[str, Material]:
    names = sorted({p.path.split(".")[2] for p in P if p.path.startswith("ship.materials.")})
    out = {}
    for n in names:
        e = P.get(f"ship.materials.{n}.emissivity")
        t = P.get(f"ship.materials.{n}.t_phys_k")
        if len(e) != len(BANDS) or not all(0.0 <= x <= 1.0 for x in e) or t < 0:
            raise ValueError(f"{P.path}: ship.materials.{n}: emissivity [S, Ku] in [0, 1] and t_phys_k >= 0 needed")
        out[n] = Material(tuple(float(x) for x in e), float(t))
    return out


MATERIALS: Dict[str, Material] = _load_materials()
HULL_Z: Tuple[float, ...] = tuple(float(x) for x in P.get("ship.hull.profile_z_m"))
HULL_R: Tuple[float, ...] = tuple(float(x) for x in P.get("ship.hull.profile_r_m"))
HULL_MATERIAL: str = P.get("ship.hull.material")
ARRAY_RADIUS_M: float = P.get("ship.array_radius_m")
BLOCK_MASK_DB: float = P.get("ship.block_mask_db")


def _load_parts() -> Tuple[Part, ...]:
    out = []
    for name, v in P.section("ship.parts").items():
        if not isinstance(v, dict) or v.get("type") not in PART_TYPES:
            raise ValueError(f"{P.path}: ship.parts.{name}: 'type' must be one of {', '.join(PART_TYPES)}")
        keys = PART_TYPES[v["type"]]
        missing = [k for k in keys + ("azimuths_deg", "material") if k not in v]
        extra = sorted(set(v) - set(keys) - {"type", "azimuths_deg", "material"})
        if missing or extra:
            raise ValueError(f"{P.path}: ship.parts.{name} ({v['type']}) " + (f"misses {missing}" if missing
                             else f"has unknown keys {extra}") + f" (needs {', '.join(keys)}, azimuths_deg, material)")
        if v["material"] not in MATERIALS:
            raise ValueError(f"{P.path}: ship.parts.{name}: unknown material '{v['material']}' ({', '.join(MATERIALS)})")
        out.append(Part(name, v["type"], v["material"], tuple((k, float(v[k])) for k in keys),
                        tuple(float(a) for a in v["azimuths_deg"])))
    return tuple(out)


PARTS: Tuple[Part, ...] = _load_parts()


def _check_ship() -> None:
    """The profile and parts must match the locked overall dimensions of [ship] (1 %)."""
    if len(HULL_Z) != len(HULL_R) or len(HULL_Z) < 2 or min(HULL_R) < 0 or HULL_MATERIAL not in MATERIALS:
        raise ValueError(f"{P.path}: [ship.hull]: profile_z_m (nose to aft) and profile_r_m (>= 0) of the same length, "
                         f"and a known material, are needed")
    checks = {"length_m": max(HULL_Z) - min(HULL_Z), "hull_diameter_m": 2.0 * max(HULL_R),
              "width_m": 2.0 * max([max(HULL_R)] + [_part_reach(p) for p in PARTS])}
    for k, v in checks.items():
        ref = P.get(f"ship.{k}")
        if abs(v / ref - 1.0) > 0.01:
            raise ValueError(f"{P.path}: the ship geometry gives {k} = {v:.2f} m, ship.{k} is {ref} m (1 % tolerance)")


def _part_reach(p: Part) -> float:
    """Largest distance of a part to the ship axis."""
    if p.type == "box_rad":
        return math.hypot(p.v("r_max_m"), p.v("half_width_m"))
    if p.type == "cylinder_rad":
        return p.v("radial_m") + p.v("length_m") / 2.0
    return p.v("radial_m") + p.v("radius_m")


_check_ship()


def ship_fingerprint() -> str:
    """Hash of everything the structure tables depend on."""
    vals = {k: P.get(k) for k in ("ship.hull.profile_z_m", "ship.hull.profile_r_m", "ship.array_radius_m",
                                  "geometry.face_phi_deg", "geometry.tilt_deg", "tool.environment.aperture_ray_rings",
                                  "tool.environment.block_grid_step_uv", "tool.environment.struct_noise_dirs")}
    vals["parts"] = [(p.name, p.type, p.material, p.values, p.azimuths_deg) for p in PARTS]
    vals["envelope"] = rm.ARRAY_ENVELOPE_M
    vals["algo"] = 2                       # 2: rays parallel to a box face outside it no longer hit (step 2.5)
    return hashlib.sha256(json.dumps(vals, sort_keys=True, default=str).encode()).hexdigest()[:16]


# ----------------------------------------------------------------------------- ray casting
_EPS_T = 1e-6


def _first_root(a: np.ndarray, b: np.ndarray, c: np.ndarray, valid) -> np.ndarray:
    """Smallest root t > eps of a t^2 + 2 b t + c = 0 where valid(t) holds (inf otherwise)."""
    out = np.full(a.shape, np.inf)
    disc = b * b - a * c
    ok = (disc >= 0.0) & (np.abs(a) > 1e-14)
    sq = np.sqrt(np.where(ok, disc, 0.0))
    for sgn in (-1.0, 1.0):
        t = np.where(ok, (-b + sgn * sq) / np.where(ok, a, 1.0), np.inf)
        good = ok & (t > _EPS_T) & valid(t)
        out = np.where(good & (t < out), t, out)
    return out


def _hit_disc(o, d, z0, r_lo, r_hi) -> np.ndarray:
    """Annulus r_lo <= r <= r_hi in the plane z = z0 (a step of the hull profile, a cylinder cap)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (z0 - o[:, 2]) / d[:, 2]
    x, y = o[:, 0] + t * d[:, 0], o[:, 1] + t * d[:, 1]
    r = np.hypot(x, y)
    return np.where(np.isfinite(t) & (t > _EPS_T) & (r >= r_lo - 1e-9) & (r <= r_hi + 1e-9), t, np.inf)


def _hit_hull(o, d) -> np.ndarray:
    out = np.full(len(o), np.inf)
    for (z1, r1), (z2, r2) in zip(zip(HULL_Z, HULL_R), zip(HULL_Z[1:], HULL_R[1:])):
        if abs(z1 - z2) < 1e-12:
            out = np.minimum(out, _hit_disc(o, d, z1, min(r1, r2), max(r1, r2)))
            continue
        b = (r2 - r1) / (z2 - z1)
        a0 = r1 - b * z1
        k0 = a0 + b * o[:, 2]
        qa = d[:, 0] ** 2 + d[:, 1] ** 2 - b * b * d[:, 2] ** 2
        qb = o[:, 0] * d[:, 0] + o[:, 1] * d[:, 1] - b * k0 * d[:, 2]
        qc = o[:, 0] ** 2 + o[:, 1] ** 2 - k0 * k0
        zlo, zhi = min(z1, z2), max(z1, z2)

        def valid(t, o=o, d=d, zlo=zlo, zhi=zhi, a0=a0, b=b):
            z = o[:, 2] + t * d[:, 2]
            return (z >= zlo - 1e-9) & (z <= zhi + 1e-9) & (a0 + b * z >= 0.0)
        out = np.minimum(out, _first_root(qa, qb, qc, valid))
    return out


def _local(o, d, az_deg):
    """Coordinates along (e_r, e_t, z) of the azimuth az."""
    c, s = math.cos(math.radians(az_deg)), math.sin(math.radians(az_deg))
    er, et = np.array([c, s, 0.0]), np.array([-s, c, 0.0])
    return (o @ er, o @ et, o[:, 2]), (d @ er, d @ et, d[:, 2])


def _hit_part(p: Part, o, d) -> np.ndarray:
    out = np.full(len(o), np.inf)
    for az in p.azimuths_deg:
        (os_, ot, oz), (ds, dt, dz) = _local(o, d, az)
        if p.type == "sphere":
            cs, cz, rad = p.v("radial_m"), p.v("z_m"), p.v("radius_m")
            ps, pz = os_ - cs, oz - cz
            t = _first_root(ds * ds + dt * dt + dz * dz, ps * ds + ot * dt + pz * dz, ps * ps + ot * ot + pz * pz - rad * rad,
                            lambda t: np.ones(t.shape, bool))
        elif p.type == "cylinder_z":
            cs, cz, rad, ln = p.v("radial_m"), p.v("z_m"), p.v("radius_m"), p.v("length_m")
            ps = os_ - cs
            zlo, zhi = cz - ln / 2.0, cz + ln / 2.0
            t = _first_root(ds * ds + dt * dt, ps * ds + ot * dt, ps * ps + ot * ot - rad * rad,
                            lambda t, oz=oz, dz=dz, zlo=zlo, zhi=zhi: (oz + t * dz >= zlo) & (oz + t * dz <= zhi))
            for zc in (zlo, zhi):
                with np.errstate(divide="ignore", invalid="ignore"):
                    tc = (zc - oz) / dz
                hit = np.isfinite(tc) & (tc > _EPS_T) & ((ps + tc * ds) ** 2 + (ot + tc * dt) ** 2 <= rad * rad)
                t = np.minimum(t, np.where(hit, tc, np.inf))
        elif p.type == "cylinder_rad":
            cs, cz, rad, ln = p.v("radial_m"), p.v("z_m"), p.v("radius_m"), p.v("length_m")
            slo, shi = cs - ln / 2.0, cs + ln / 2.0
            pz = oz - cz
            t = _first_root(dt * dt + dz * dz, ot * dt + pz * dz, ot * ot + pz * pz - rad * rad,
                            lambda t, os_=os_, ds=ds, slo=slo, shi=shi: (os_ + t * ds >= slo) & (os_ + t * ds <= shi))
            for sc in (slo, shi):
                with np.errstate(divide="ignore", invalid="ignore"):
                    tc = (sc - os_) / ds
                hit = np.isfinite(tc) & (tc > _EPS_T) & ((ot + tc * dt) ** 2 + (pz + tc * dz) ** 2 <= rad * rad)
                t = np.minimum(t, np.where(hit, tc, np.inf))
        else:                                                     # box_rad: slab method
            lo = np.array([p.v("r_min_m"), -p.v("half_width_m"), p.v("z_min_m")])
            hi = np.array([p.v("r_max_m"), p.v("half_width_m"), p.v("z_max_m")])
            tn, tf = np.full(len(o), -np.inf), np.full(len(o), np.inf)
            miss = np.zeros(len(o), dtype=bool)
            for oc, dc, a, b in ((os_, ds, lo[0], hi[0]), (ot, dt, lo[1], hi[1]), (oz, dz, lo[2], hi[2])):
                par = np.abs(dc) < 1e-15
                miss |= par & ((oc < a) | (oc > b))           # parallel to this slab and outside it: no hit
                with np.errstate(divide="ignore", invalid="ignore"):
                    t1 = np.where(par, -np.inf, (a - oc) / np.where(par, 1.0, dc))
                    t2 = np.where(par, np.inf, (b - oc) / np.where(par, 1.0, dc))
                tn, tf = np.maximum(tn, np.minimum(t1, t2)), np.minimum(tf, np.maximum(t1, t2))
            t = np.where(~miss & (tf >= tn) & (tf > _EPS_T), np.where(tn > _EPS_T, tn, tf), np.inf)
        out = np.minimum(out, t)
    return out


def cast(o: np.ndarray, d: np.ndarray, chunk: int = 200_000) -> Tuple[np.ndarray, np.ndarray]:
    """First hit of rays (origins o, unit directions d) on the structure: (distance, material index into
    MATERIAL_NAMES, -1 = no hit)."""
    t_all = np.full(len(o), np.inf)
    m_all = np.full(len(o), -1, dtype=np.int16)
    with np.errstate(invalid="ignore", divide="ignore"):    # parallel rays give inf x 0 in the masked branches
        _cast_chunks(o, d, chunk, t_all, m_all)
    return t_all, m_all


def _cast_chunks(o, d, chunk, t_all, m_all) -> None:
    for lo in range(0, len(o), chunk):
        sl = slice(lo, lo + chunk)
        oo, dd = o[sl], d[sl]
        best = _hit_hull(oo, dd)
        mat = np.where(np.isfinite(best), MATERIAL_NAMES.index(HULL_MATERIAL), -1)
        for p in PARTS:
            t = _hit_part(p, oo, dd)
            closer = t < best
            best = np.where(closer, t, best)
            mat = np.where(closer, MATERIAL_NAMES.index(p.material), mat)
        t_all[sl], m_all[sl] = best, mat


MATERIAL_NAMES: Tuple[str, ...] = tuple(MATERIALS)

# ----------------------------------------------------------------------------- per-face structure tables
_TE = "tool.environment."
RAY_RINGS: int = P.get(_TE + "aperture_ray_rings")
GRID_STEP_UV: float = P.get(_TE + "block_grid_step_uv")
STRUCT_DIRS: int = P.get(_TE + "struct_noise_dirs")
RING_POINTS: int = P.get(_TE + "clutter_ring_points")
CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "structure_cache.npz")
_ORIGIN_OFFSET_M = 0.05                    # rays start this far in front of the array plate


def face_frame(k: int):
    """(n, e_u, e_v) of face k: the formula of core.optimizer.face_frame (which imports this module)."""
    t = math.radians(rm.SECTOR_TILT_DEG)
    phi = math.radians(face_phis()[k])
    c, s = math.cos(phi), math.sin(phi)
    return (np.array([math.cos(t) * c, math.cos(t) * s, math.sin(t)]), np.array([-s, c, 0.0]),
            np.array([-math.sin(t) * c, -math.sin(t) * s, math.cos(t)]))


def face_phis() -> Tuple[float, ...]:
    return tuple(P.get("geometry.face_phi_deg"))


def array_centre_m(k: int) -> np.ndarray:
    phi = math.radians(face_phis()[k])
    return np.array([ARRAY_RADIUS_M * math.cos(phi), ARRAY_RADIUS_M * math.sin(phi), 0.0])


def aperture_points(k: int) -> np.ndarray:
    """Ray origins over the aperture of face k: hexagonal lattice inside the envelope disk, in the face plane."""
    n, eu, ev = face_frame(k)
    rad = rm.ARRAY_ENVELOPE_M / 2.0
    s = rad / max(RAY_RINGS, 1)
    pts = [(i * s + j * s / 2.0, j * s * math.sqrt(3.0) / 2.0)
           for i in range(-RAY_RINGS, RAY_RINGS + 1) for j in range(-RAY_RINGS, RAY_RINGS + 1)]
    pts = [(x, y) for x, y in pts if math.hypot(x, y) <= rad + 1e-9] if RAY_RINGS > 0 else [(0.0, 0.0)]
    c = array_centre_m(k) + _ORIGIN_OFFSET_M * n
    return np.array([c + x * eu + y * ev for x, y in pts])


def _grid_uv() -> Tuple[np.ndarray, np.ndarray]:
    g = np.arange(-1.0, 1.0 + 1e-9, GRID_STEP_UV)
    return np.meshgrid(g, g, indexing="ij")


def _fib_sphere(n: int) -> np.ndarray:
    i = np.arange(n) + 0.5
    zz = 1.0 - 2.0 * i / n
    rr = np.sqrt(1.0 - zz * zz)
    ph = math.pi * (1.0 + 5 ** 0.5) * i
    return np.c_[rr * np.cos(ph), rr * np.sin(ph), zz]


def _build_tables() -> Dict[str, np.ndarray]:
    """Per face: blocked fraction and mean emissivity x T of the blocked rays per band on the (u, v) grid; the
    structure directions seen from the array centre (far-sidelobe noise) with their emissivity x T per band."""
    uu, vv = _grid_uv()
    q = 1.0 - uu * uu - vv * vv
    inside = q > 0.0
    out: Dict[str, np.ndarray] = {"step": np.array(GRID_STEP_UV)}
    epst = np.array([[m.eps_t(b) for b in BANDS] for m in MATERIALS.values()])          # (materials, bands)
    fib = _fib_sphere(STRUCT_DIRS)
    for k in range(len(face_phis())):
        n, eu, ev = face_frame(k)
        ww = np.sqrt(np.where(inside, q, 0.0))
        dirs = (uu[inside, None] * eu + vv[inside, None] * ev + ww[inside, None] * n)
        org = aperture_points(k)
        o = np.repeat(org[None, :, :], len(dirs), axis=0).reshape(-1, 3)
        d = np.repeat(dirs[:, None, :], len(org), axis=1).reshape(-1, 3)
        _, mat = cast(o, d)
        mat = mat.reshape(len(dirs), len(org))
        hit = mat >= 0
        f = np.zeros(uu.shape)
        f[inside] = hit.mean(axis=1)
        for bi, b in enumerate(BANDS):
            e = np.where(hit, epst[np.maximum(mat, 0), bi], 0.0).sum(axis=1) / np.maximum(hit.sum(axis=1), 1)
            arr = np.zeros(uu.shape)
            arr[inside] = e
            out[f"epst_{b}_{k}"] = arr
        out[f"f_{k}"] = f
        front = fib @ n > 0.0
        fd = fib[front]
        _, m1 = cast(np.repeat((array_centre_m(k) + _ORIGIN_OFFSET_M * n)[None, :], len(fd), axis=0), fd)
        sel = m1 >= 0
        out[f"sdirs_{k}"] = fd[sel]
        for bi, b in enumerate(BANDS):
            out[f"sepst_{b}_{k}"] = epst[m1[sel], bi]
    return out


@lru_cache(maxsize=1)
def tables() -> Dict[str, np.ndarray]:
    """The structure tables, from core/structure_cache.npz when its fingerprint matches (written only for the default
    parameter file, as the pattern cache)."""
    fp = ship_fingerprint()
    if os.path.exists(CACHE_PATH):
        try:
            with np.load(CACHE_PATH, allow_pickle=False) as z:
                if str(z["fingerprint"]) == fp:
                    return {k: z[k] for k in z.files if k != "fingerprint"}
        except (OSError, KeyError, ValueError):
            pass
    t = _build_tables()
    if os.path.abspath(P.path) == os.path.abspath(DEFAULT_PATH):
        np.savez_compressed(CACHE_PATH, fingerprint=np.array(fp), **t)
    return t


def cache_is_current() -> bool:
    if not os.path.exists(CACHE_PATH):
        return False
    with np.load(CACHE_PATH, allow_pickle=False) as z:
        return str(z["fingerprint"]) == ship_fingerprint()


def blockage(k: int, dirs: np.ndarray, band: str) -> Tuple[np.ndarray, np.ndarray]:
    """Blocked fraction of the aperture tube of face k toward dirs, and the mean emissivity x T (K) of what is hit."""
    t = tables()
    n, eu, ev = face_frame(k)
    i = np.clip(np.rint((dirs @ eu + 1.0) / GRID_STEP_UV).astype(int), 0, t[f"f_{k}"].shape[0] - 1)
    j = np.clip(np.rint((dirs @ ev + 1.0) / GRID_STEP_UV).astype(int), 0, t[f"f_{k}"].shape[1] - 1)
    return t[f"f_{k}"][i, j], t[f"epst_{band}_{k}"][i, j]


def block_loss_db(f: np.ndarray) -> np.ndarray:
    """Blockage loss per pass of a blocked fraction f of the aperture: -20 log(1 - f)."""
    return -20.0 * np.log10(np.maximum(1.0 - f, 1e-6))


# ----------------------------------------------------------------------------- noise of the structure
def structure_rise_k(k: int, dirs: np.ndarray, band: str, f_ghz: float, rx_sll_db: float) -> np.ndarray:
    """Antenna-temperature rise from the structure for the RX beams of face k at dirs: main lobe (blocked fraction x
    emissivity x T x near-zone share) + far sidelobes (structure directions over the far profile, as core/sky.py)."""
    from . import sky as skym
    prof = skym._profile(band, float(rx_sll_db))
    f, epst = blockage(k, dirs, band)
    near = f * epst * prof.norm * prof.near_full
    n, eu, ev = face_frame(k)
    grid = _far_structure_table(k, band, float(f_ghz), float(rx_sll_db))
    i = np.clip(np.rint((dirs @ eu + 1.0) / _FAR_STEP_UV).astype(int), 0, grid.shape[0] - 1)
    j = np.clip(np.rint((dirs @ ev + 1.0) / _FAR_STEP_UV).astype(int), 0, grid.shape[1] - 1)
    return near + grid[i, j]


_FAR_STEP_UV = 0.05          # the far-sidelobe structure noise varies slowly: table step in (u, v)


@lru_cache(maxsize=64)
def _far_structure_table(k: int, band: str, f_ghz: float, rx_sll_db: float) -> np.ndarray:
    """Far-sidelobe structure noise (K) of face k on a (u, v) grid: the structure directions seen from the array
    centre, weighted by emissivity x T, over the azimuthal-mean far RX profile (as a body of core/sky.py)."""
    from . import sky as skym
    prof = skym._profile(band, rx_sll_db)
    g = np.arange(-1.0, 1.0 + 1e-9, _FAR_STEP_UV)
    ub, vb = (x.ravel() for x in np.meshgrid(g, g, indexing="ij"))
    out = np.zeros(len(ub))
    t = tables()
    s, e = t[f"sdirs_{k}"], t[f"sepst_{band}_{k}"]
    if len(s):
        n, eu, ev = face_frame(k)
        k_lam = rm.ARRAY_ENVELOPE_M / rm.lambda_m(f_ghz)
        wt = (s @ n) * (4.0 * math.pi / STRUCT_DIRS) * k_lam ** 2 * e
        rho = np.hypot((s @ eu)[None, :] - ub[:, None], (s @ ev)[None, :] - vb[:, None]) * k_lam
        out = prof.norm * (prof.g_far(rho) * wt[None, :]).sum(axis=1)
    return out.reshape(len(g), len(g))


# ----------------------------------------------------------------------------- surfaces: shadow, clutter, ADC
SURFACES: Dict[str, Tuple[float, Tuple[float, float]]] = {
    k: (float(v["radius_km"]), tuple(float(x) for x in v["gamma_db"])) for k, v in P.section("sky.surface").items()}
for _k, (_r, _g) in SURFACES.items():
    if _r <= 0 or len(_g) != len(BANDS):
        raise ValueError(f"{P.path}: sky.surface.{_k}: radius_km > 0 and gamma_db [S, Ku] needed")


@dataclass(frozen=True)
class Surface:
    id: str
    c: Tuple[float, float, float]
    radius_km: float
    dist_km: float                 # to the centre
    gamma: Tuple[float, float]     # linear, [S, Ku]
    closing_mps: Optional[float]
    spread_mps: Optional[float]

    @property
    def d_min_km(self) -> float:
        return self.dist_km - self.radius_km

    @property
    def d_horizon_km(self) -> float:
        return math.sqrt(max(self.dist_km ** 2 - self.radius_km ** 2, 0.0))


def surfaces(sky: Optional[dict]) -> Tuple[Surface, ...]:
    """The bodies of the mission sky that have a scattering surface ([sky.surface]); distance from the angular radius."""
    out = []
    for b in (sky or {}).get("bodies", []):
        if b["type"] not in SURFACES:
            continue
        rad_km, g = SURFACES[b["type"]]
        a = math.radians(b["angular_radius_deg"])
        from .sky import unit
        out.append(Surface(str(b["id"]), tuple(float(x) for x in unit(b["phi_deg"], b["eps_deg"])), rad_km,
                           rad_km / math.sin(a), tuple(10.0 ** (x / 10.0) for x in g),
                           b.get("closing_mps"), b.get("closing_spread_mps")))
    return tuple(out)


def surface_distance_km(s: Surface, dirs: np.ndarray) -> np.ndarray:
    """Distance to the surface along dirs (inf outside the disk)."""
    cosd = dirs @ np.array(s.c)
    sin2 = np.maximum(1.0 - cosd * cosd, 0.0)
    disc = s.radius_km ** 2 - s.dist_km ** 2 * sin2
    return np.where((disc >= 0.0) & (cosd > 0.0), s.dist_km * cosd - np.sqrt(np.maximum(disc, 0.0)), np.inf)


def _ring(s: Surface, r_km: float) -> Optional[Tuple[np.ndarray, float]]:
    """Directions of the iso-range ring at r on the visible surface, and cos(incidence) there (None if no ring)."""
    if not s.d_min_km < r_km < s.d_horizon_km:
        return None
    cos_a = (r_km ** 2 + s.dist_km ** 2 - s.radius_km ** 2) / (2.0 * r_km * s.dist_km)
    cos_i = (s.dist_km ** 2 - r_km ** 2 - s.radius_km ** 2) / (2.0 * r_km * s.radius_km)
    a = math.acos(min(max(cos_a, -1.0), 1.0))
    c = np.array(s.c)
    e1 = np.cross(c, [0.0, 0.0, 1.0] if abs(c[2]) < 0.9 else [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(c, e1)
    psi = (np.arange(RING_POINTS) + 0.5) * (2.0 * math.pi / RING_POINTS)
    pts = math.cos(a) * c + math.sin(a) * (np.cos(psi)[:, None] * e1 + np.sin(psi)[:, None] * e2)
    return pts, max(cos_i, 0.0)


def _two_way_profile(band: str, rx_sll_db: float):
    tx, rx = rm.PATTERNS[band].tx, rm.rx_beam(band, rx_sll_db)
    near_r = np.arange(len(tx.mean_near)) * antenna.R_STEP

    def g(beam, rho):
        far = np.array(beam.mean_far)
        idx = np.minimum((rho / antenna.FAR_PROFILE_STEP).astype(int), len(far) - 1)
        return np.where(rho <= antenna.R_MAX, np.interp(rho, near_r, np.array(beam.mean_near)), far[idx])
    return lambda rho: g(tx, rho) * g(rx, rho)


def clutter_over_signal(s: Surface, k: int, dirs: np.ndarray, band: str, f_ghz: float, rx_sll_db: float,
                        r_km: float, dr_m: float, rcs_m2: float) -> np.ndarray:
    """Sidelobe clutter of surface s in the target's range cell (r, dr) over the target's echo, for beams of face k at
    dirs: sigma_c / sigma_target, sigma_c = sum over the iso-range ring of sigma0 dA x the two-way relative gain."""
    ring = _ring(s, r_km)
    if ring is None:
        return np.zeros(len(dirs))
    pts, cos_i = ring
    n, eu, ev = face_frame(k)
    wq = pts @ n
    keep = wq > 0.0
    if not keep.any():
        return np.zeros(len(dirs))
    pts, wq = pts[keep], wq[keep]
    gam = s.gamma[BANDS.index(band)]
    area = 2.0 * math.pi * s.radius_km * 1e3 * r_km * 1e3 * dr_m / (s.dist_km * 1e3)   # m2 of surface in the cell
    sig_pt = gam * cos_i * area / RING_POINTS
    k_lam = rm.ARRAY_ENVELOPE_M / rm.lambda_m(f_ghz)
    g2 = _two_way_profile(band, rx_sll_db)
    ub, vb, wb = dirs @ eu, dirs @ ev, np.maximum(dirs @ n, 1e-6)
    us, vs = pts @ eu, pts @ ev
    rho = np.hypot(us[None, :] - ub[:, None], vs[None, :] - vb[:, None]) * k_lam
    rel = (wq[None, :] / wb[:, None]) ** 2 * g2(rho)
    return sig_pt * rel.sum(axis=1) / rcs_m2


def doppler_rejected(s: Surface, v_min_mps: float, v_max_mps: float) -> bool:
    """The surface's closing-speed band, when the scenario gives it, does not overlap the target's span."""
    if s.closing_mps is None or s.spread_mps is None:
        return False
    return v_max_mps < s.closing_mps - s.spread_mps or v_min_mps > s.closing_mps + s.spread_mps


def clutter_noise_factor(x: np.ndarray) -> np.ndarray:
    """Noise factor of clutter treated as noise at the detection threshold: 1 / (1 - (C/S) SNR_req), inf at >= 1."""
    return np.where(x < 1.0, 1.0 / np.maximum(1.0 - x, 1e-12), np.inf)


def main_lobe_adc_margin_db(s: Surface, dirs: np.ndarray, thetas: np.ndarray, f_ghz: float) -> np.ndarray:
    """Clipping margin (dB, negative = the per-element ADCs clip) of the main-lobe surface echo: footprint of the TX
    beam on the surface x sigma0 at the surface distance, peak echo per element (radar_model, before beamforming)."""
    band = rm.band_for_frequency(f_ghz)
    d = surface_distance_km(s, dirs)
    out = np.full(len(dirs), np.inf)
    hit = np.isfinite(d)
    if not hit.any():
        return out
    c = np.array(s.c) * s.dist_km
    pts = dirs[hit] * d[hit, None]
    nrm = (pts - c) / s.radius_km
    cos_i = np.clip(-(nrm * dirs[hit]).sum(axis=1), 1e-3, 1.0)
    hp = math.radians(rm.hpbw_deg(f_ghz))
    w = np.maximum(np.cos(np.radians(thetas[hit])), 1e-3)
    footprint = (math.pi / 4.0) * (hp / w) ** 2 * (d[hit] * 1e3) ** 2 / cos_i
    sigma = s.gamma[BANDS.index(band.name)] * cos_i * footprint
    for i, idx in enumerate(np.flatnonzero(hit)):
        out[idx] = band.clip_ant_dbm - rm.echo_peak_dbm_per_element(f_ghz, float(thetas[idx]), float(sigma[i]),
                                                                      float(d[idx]))
    return out


# ----------------------------------------------------------------------------- jammers
JAM_NULL_DB: float = P.get("jamming.null_depth_db")
JAM_MAX_NULLED: int = P.get("jamming.max_nulled")
JAM_NULL_OFFSET: float = P.get("jamming.null_min_offset_hpbw")
JAM_NATURES = ("SOJ", "SPJ")


def _load_jammer_classes() -> Dict[str, dict]:
    names = list(dict.fromkeys(p.path.split(".")[1] for p in P if p.path.startswith("jammers.")))
    out = {}
    for n in names:
        nat = P.get(f"jammers.{n}.nature")
        if nat not in JAM_NATURES:
            raise ValueError(f"{P.path}: jammers.{n}.nature '{nat}' must be one of {', '.join(JAM_NATURES)}")
        v = {"nature": nat}
        for b in BANDS:
            pre = f"jammers.{n}.band.{b}"
            v[b] = {"erp_dbw_mhz": float(P.get(pre + ".erp_dbw_mhz"))}
            keys = {p.path[len(pre) + 1:] for p in P if p.path.startswith(pre + ".")}
            extra = keys - {"erp_dbw_mhz", "cover_lo_ghz", "cover_hi_ghz"}
            if extra:
                raise ValueError(f"{P.path}: {pre} has unknown keys {sorted(extra)}")
            band = rm.BANDS[b]
            v[b]["lo"] = float(P.get(pre + ".cover_lo_ghz")) if "cover_lo_ghz" in keys else band.usable_lo_ghz
            v[b]["hi"] = float(P.get(pre + ".cover_hi_ghz")) if "cover_hi_ghz" in keys else band.usable_hi_ghz
        out[n] = v
    return out


JAMMER_CLASSES: Dict[str, dict] = _load_jammer_classes()


@dataclass(frozen=True)
class Jammer:
    id: str
    cls: str
    c: Tuple[float, float, float]
    range_km: float
    erp_dbw_mhz: Tuple[float, float]       # [S, Ku]

    def density_w_hz(self, f_ghz: float) -> float:
        b = rm.band_for_frequency(f_ghz)
        if b is None:
            return 0.0
        cov = JAMMER_CLASSES[self.cls][b.name]
        if not cov["lo"] <= f_ghz <= cov["hi"]:
            return 0.0
        return 10.0 ** (self.erp_dbw_mhz[BANDS.index(b.name)] / 10.0) / 1e6


def _erp(cls: str, override) -> Tuple[float, float]:
    base = tuple(JAMMER_CLASSES[cls][b]["erp_dbw_mhz"] for b in BANDS)
    if override is None:
        return base
    if isinstance(override, dict):
        return tuple(float(override.get(b, base[i])) for i, b in enumerate(BANDS))
    return (float(override),) * len(BANDS)


def validate_jammers(sky: Optional[dict]) -> List[str]:
    errs = []
    for i, j in enumerate((sky or {}).get("jammers", [])):
        tag = f"jammer {j.get('id', i)}"
        for k in ("id", "class", "phi_deg", "eps_deg", "range_km"):
            if k not in j:
                errs.append(f"{tag}: missing '{k}'")
        if j.get("class") is not None and j["class"] not in JAMMER_CLASSES:
            errs.append(f"{tag}: unknown class '{j['class']}' ({', '.join(JAMMER_CLASSES)})")
        elif j.get("class") is not None and JAMMER_CLASSES[j["class"]]["nature"] != "SOJ":
            errs.append(f"{tag}: class '{j['class']}' is a self-protection jammer: give it to a sector (jammer_type)")
        if j.get("range_km") is not None and j["range_km"] <= 0:
            errs.append(f"{tag}: range_km must be > 0")
    return errs


def jammers(sky: Optional[dict]) -> Tuple[Jammer, ...]:
    from .sky import unit
    return tuple(Jammer(str(j["id"]), j["class"], tuple(float(x) for x in unit(j["phi_deg"], j["eps_deg"])),
                        float(j["range_km"]), _erp(j["class"], j.get("erp_dbw_mhz")))
                 for j in (sky or {}).get("jammers", []))


def jammer_rise_k(jams: Sequence[Jammer], k: int, dirs: np.ndarray, band: str, f_ghz: float,
                  rx_sll_db: float) -> Dict[str, np.ndarray]:
    """Antenna-temperature rise (K) of each stand-off jammer for the RX beams of face k at dirs, after the adaptive
    nulling of the sidelobe jammers (the strongest first, up to JAM_MAX_NULLED)."""
    out = {j.id: np.zeros(len(dirs)) for j in jams}
    if not jams:
        return out
    beam = rm.rx_beam(band, rx_sll_db)
    n, eu, ev = face_frame(k)
    k_lam = rm.ARRAY_ENVELOPE_M / rm.lambda_m(f_ghz)
    near_r = np.arange(len(beam.mean_near)) * antenna.R_STEP
    far = np.array(beam.mean_far)
    a_peak = rm.PHYS_AREA_M2 * beam.eta * rm.ETA_OHMIC
    ub, vb = dirs @ eu, dirs @ ev
    t = np.zeros((len(jams), len(dirs)))
    side = np.zeros((len(jams), len(dirs)), dtype=bool)
    for i, j in enumerate(jams):
        c = np.array(j.c)
        wj = float(c @ n)
        if wj <= 0.0:
            continue
        rho = np.hypot(float(c @ eu) - ub, float(c @ ev) - vb) * k_lam
        g = np.where(rho <= antenna.R_MAX, np.interp(rho, near_r, np.array(beam.mean_near)),
                     far[np.minimum((rho / antenna.FAR_PROFILE_STEP).astype(int), len(far) - 1)])
        flux = j.density_w_hz(f_ghz) / (4.0 * math.pi * (j.range_km * 1e3) ** 2)
        t[i] = flux * a_peak * wj * g / K_BOLTZ
        side[i] = rho > JAM_NULL_OFFSET * beam.hpbw
    nulled = np.zeros_like(side)
    order = np.argsort(-np.where(side, t, -1.0), axis=0)
    for r in range(min(JAM_MAX_NULLED, len(jams))):
        idx = order[r]
        cols = np.arange(len(dirs))
        nulled[idx, cols] |= side[idx, cols]
    t = np.where(nulled, t * 10.0 ** (-JAM_NULL_DB / 10.0), t)
    for i, j in enumerate(jams):
        out[j.id] = t[i]
    return out


def burn_through_km(p_beam_w: float, g_tx: float, rcs_m2: float, energy_s: float, snr_req_db: float,
                    l_tot_db: float, erp_dbw_mhz: float) -> float:
    """Self-protection jammer: range below which S / J0 reaches the required SNR, from the energy-form radar equation
    with the jammer noise J0 = EIRP density x G_rx lambda^2 / ((4 pi)^2 R^2):
    R_bt = sqrt(P G_t sigma tau N / (4 pi EIRP_density SNR_req L))."""
    j0 = 10.0 ** (erp_dbw_mhz / 10.0) / 1e6
    return math.sqrt(p_beam_w * g_tx * rcs_m2 * energy_s / (4.0 * math.pi * j0 * rm.db_to_lin(snr_req_db)
                                                             * rm.db_to_lin(l_tot_db))) / 1e3


# ----------------------------------------------------------------------------- Faraday rotation
def faraday_round_trip_deg(tec_tecu: float, b_parallel_nt: float, f_ghz: float) -> float:
    """Faraday rotation over the round trip, 2 x 2.36e4 B TEC / f^2 (SI), the line-of-sight TEC and field given by the
    scenario. The dual-pol GLRT receiver recovers the energy: no detection loss (polarimetric use at 3.3)."""
    return math.degrees(2.0 * 2.36e4 * b_parallel_nt * 1e-9 * tec_tecu * 1e16 / (f_ghz * 1e9) ** 2)
