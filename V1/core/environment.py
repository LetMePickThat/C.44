#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENVIRONMENT -- what surrounds a face (roadmap step 2.5): the ship structure, the surfaces of the bodies, the jammers.

Everything is in the SHIP frame (core/optimizer.face_frame); positions of bodies and jammers come from the mission sky
(later the global scenario simulator): the radar model never computes where an object is.

Ship structure ([ship] of radar_system.toml: the low-poly 3D model ship.model_path, triangles placed in the ship frame,
each of a material by region; the overall dimensions and the four apertures are checked against it at load time):
  masking   the aperture is a TUBE in the near field (Fraunhofer limit ~325 m in S): rays from points of the aperture
            (hexagonal rings) along the beam direction, exact ray-triangle test on the model; f = fraction of rays
            hitting the structure; loss per pass
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

# ----------------------------------------------------------------------------- ship structure (3D model)
@dataclass(frozen=True)
class Material:
    emissivity: Tuple[float, float]       # RF emissivity [S, Ku]
    t_phys_k: float

    def eps_t(self, band: str) -> float:
        return self.emissivity[BANDS.index(band)] * self.t_phys_k


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
MATERIAL_NAMES: Tuple[str, ...] = tuple(MATERIALS)
APP_DIR = os.path.dirname(os.path.abspath(DEFAULT_PATH))
MODEL_PATH: str = os.path.join(APP_DIR, P.get("ship.model_path"))
MODEL_NOSE_AXIS: str = P.get("ship.model_nose_axis")
MODEL_ARRAY_Z_M: float = P.get("ship.model_array_station_z_m")
ARRAY_RADIUS_M: float = P.get("ship.array_radius_m")
BLOCK_MASK_DB: float = P.get("ship.block_mask_db")
DEFAULT_MATERIAL: str = P.get("ship.default_material")
_REGION_BOUNDS = ("abs_x_min_m", "abs_x_max_m", "abs_y_min_m", "abs_y_max_m", "z_min_m", "z_max_m")


def load_obj(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Vertices (model frame) and triangles (vertex indices; polygons split in fans) of a Wavefront OBJ."""
    v, tris = [], []
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if line.startswith("v "):
                    v.append(line.split()[1:4])
                elif line.startswith("f "):
                    idx = [int(t.split("/")[0]) for t in line.split()[1:]]
                    idx = [i - 1 if i > 0 else len(v) + i for i in idx]          # OBJ: 1-based, negative = relative
                    tris.extend((idx[0], idx[i], idx[i + 1]) for i in range(1, len(idx) - 1))
    except OSError as e:
        raise ValueError(f"{P.path}: ship.model_path: cannot read the 3D model {path} ({e})") from e
    if not v or not tris:
        raise ValueError(f"{P.path}: ship.model_path: {path} holds no vertices or faces")
    return np.array(v, dtype=float), np.array(tris, dtype=np.int64)


def _to_ship_frame(v: np.ndarray) -> np.ndarray:
    """Model frame -> ship frame: nose toward +Z, z = 0 at the aperture station (a '-Z' nose: 180 deg about X)."""
    if MODEL_NOSE_AXIS == "-Z":
        return np.c_[v[:, 0], -v[:, 1], -(v[:, 2] - MODEL_ARRAY_Z_M)]
    if MODEL_NOSE_AXIS == "+Z":
        return np.c_[v[:, 0], v[:, 1], v[:, 2] - MODEL_ARRAY_Z_M]
    raise ValueError(f"{P.path}: ship.model_nose_axis '{MODEL_NOSE_AXIS}' must be '-Z' or '+Z'")


def _load_regions() -> Tuple[Tuple[str, str, Dict[str, float]], ...]:
    try:
        regions = P.section("ship.material_regions")
    except KeyError:
        return ()
    out = []
    for name, v in regions.items():
        if not isinstance(v, dict) or v.get("material") not in MATERIALS:
            raise ValueError(f"{P.path}: ship.material_regions.{name}: 'material' must be one of {', '.join(MATERIALS)}")
        extra = sorted(set(v) - set(_REGION_BOUNDS) - {"material"})
        if extra:
            raise ValueError(f"{P.path}: ship.material_regions.{name} has unknown keys {extra} (bounds: "
                             f"{', '.join(_REGION_BOUNDS)})")
        out.append((name, v["material"], {k: float(v[k]) for k in _REGION_BOUNDS if k in v}))
    return tuple(out)


MATERIAL_REGIONS = _load_regions()


def _triangle_materials(tri: np.ndarray) -> np.ndarray:
    """Material index of each triangle: the first region holding its centroid, else ship.default_material."""
    if DEFAULT_MATERIAL not in MATERIALS:
        raise ValueError(f"{P.path}: ship.default_material '{DEFAULT_MATERIAL}' is not one of {', '.join(MATERIALS)}")
    c = tri.mean(axis=1)
    ax, ay, z = np.abs(c[:, 0]), np.abs(c[:, 1]), c[:, 2]
    out = np.full(len(tri), MATERIAL_NAMES.index(DEFAULT_MATERIAL), dtype=np.int16)
    done = np.zeros(len(tri), dtype=bool)
    for _, mat, b in MATERIAL_REGIONS:
        sel = ~done
        for arr, name, lower in ((ax, "abs_x_min_m", True), (ax, "abs_x_max_m", False), (ay, "abs_y_min_m", True),
                                 (ay, "abs_y_max_m", False), (z, "z_min_m", True), (z, "z_max_m", False)):
            if name in b:
                sel &= arr >= b[name] if lower else arr <= b[name]
        out[sel] = MATERIAL_NAMES.index(mat)
        done |= sel
    return out


_V_MODEL, MODEL_FACES = load_obj(MODEL_PATH)
MODEL_VERTICES: np.ndarray = _to_ship_frame(_V_MODEL)      # ship frame, metres
TRIANGLES: np.ndarray = MODEL_VERTICES[MODEL_FACES]         # (triangles, 3 corners, xyz)
TRI_MATERIAL: np.ndarray = _triangle_materials(TRIANGLES)
del _V_MODEL


def ship_fingerprint() -> str:
    """Hash of everything the structure tables depend on (the model file itself, its placement, the materials)."""
    vals = {k: P.get(k) for k in ("ship.model_nose_axis", "ship.model_array_station_z_m", "ship.array_radius_m",
                                  "ship.default_material", "geometry.face_phi_deg", "geometry.tilt_deg",
                                  "tool.environment.aperture_ray_rings", "tool.environment.block_grid_step_uv",
                                  "tool.environment.struct_noise_dirs")}
    with open(MODEL_PATH, "rb") as fh:
        vals["model_sha256"] = hashlib.sha256(fh.read()).hexdigest()
    vals["regions"] = MATERIAL_REGIONS
    vals["materials"] = MATERIAL_NAMES
    vals["envelope"] = rm.ARRAY_ENVELOPE_M
    vals["algo"] = 3                       # 3: the 3D model (triangles) replaces the primitives
    return hashlib.sha256(json.dumps(vals, sort_keys=True, default=str).encode()).hexdigest()[:16]


# ----------------------------------------------------------------------------- ray casting
def _first_hits(o: np.ndarray, dirs: np.ndarray, idx: Optional[np.ndarray] = None,
                chunk: int = 2048) -> Tuple[np.ndarray, np.ndarray]:
    """Nearest hit of rays from ONE origin o along the unit directions dirs on the model triangles idx (all when
    None): (distance, triangle index, -1 = no hit). Exact: a ray hits a triangle when its direction lies in the cone
    the triangle spans from o (the three triple products d.(b x c), d.(c x a), d.(a x b) have the sign of
    a.(b x c)); the distance is a.(b x c) / d.n, n = the triangle normal."""
    ids = np.arange(len(TRIANGLES)) if idx is None else np.asarray(idx)
    tri = TRIANGLES[ids]
    a, b, c = (tri[:, i] - o for i in range(3))
    bc, ca, ab = np.cross(b, c), np.cross(c, a), np.cross(a, b)
    tr = np.einsum("ij,ij->i", a, bc)
    keep = np.abs(tr) > 1e-12                                  # o in the plane of the triangle: no crossing
    sg = np.sign(tr[keep])[:, None]
    ids, tr = ids[keep], np.abs(tr[keep])
    m = np.concatenate([bc[keep] * sg, ca[keep] * sg, ab[keep] * sg]).T     # (3, 3 k)
    k = len(ids)
    t_out = np.full(len(dirs), np.inf)
    i_out = np.full(len(dirs), -1, dtype=np.int64)
    if k == 0:
        return t_out, i_out
    for lo in range(0, len(dirs), chunk):
        s = dirs[lo:lo + chunk] @ m
        h = (s[:, :k] >= 0.0) & (s[:, k:2 * k] >= 0.0) & (s[:, 2 * k:] >= 0.0)
        rows = np.flatnonzero(h.any(axis=1))
        if not len(rows):
            continue
        sr = s[rows]
        with np.errstate(divide="ignore", invalid="ignore"):
            dist = np.where(h[rows], tr / (sr[:, :k] + sr[:, k:2 * k] + sr[:, 2 * k:]), np.inf)
        j = np.argmin(dist, axis=1)
        t_out[lo + rows] = dist[np.arange(len(rows)), j]
        i_out[lo + rows] = ids[j]
    return t_out, i_out


def cast(o: np.ndarray, d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """First hit of rays (origins o, unit directions d) on the structure: (distance, material index into
    MATERIAL_NAMES, -1 = no hit)."""
    o, d = np.atleast_2d(np.asarray(o, float)), np.atleast_2d(np.asarray(d, float))
    t_all = np.full(len(o), np.inf)
    m_all = np.full(len(o), -1, dtype=np.int16)
    uo, inv = np.unique(o, axis=0, return_inverse=True)
    for i, oo in enumerate(uo):
        sel = np.flatnonzero(inv.ravel() == i)
        t, j = _first_hits(oo, d[sel])
        t_all[sel] = t
        m_all[sel] = np.where(j >= 0, TRI_MATERIAL[np.maximum(j, 0)], -1)
    return t_all, m_all


def _front_triangles(k: int) -> np.ndarray:
    """Triangles with a corner in front of the aperture plane of face k (the only ones its rays can meet)."""
    n = face_frame(k)[0]
    return np.flatnonzero(((TRIANGLES - array_centre_m(k)) @ n > 1e-4).any(axis=1))


def _hull_body_vertices() -> np.ndarray:
    """Vertices of the hull body: the connected piece of the model with the longest axial extent (labels spread
    along the triangle edges, smallest label wins)."""
    lab = np.arange(len(MODEL_VERTICES))
    f = MODEL_FACES
    while True:
        m = lab[f].min(axis=1)
        new = lab.copy()
        for i in range(3):
            np.minimum.at(new, f[:, i], m)
        new = new[new]                                         # pointer jumping
        if np.array_equal(new, lab):
            break
        lab = new
    best, ext = None, -1.0
    for g in np.unique(lab):
        z = MODEL_VERTICES[lab == g, 2]
        if np.ptp(z) > ext:
            best, ext = g, float(np.ptp(z))
    return MODEL_VERTICES[lab == best]


def _check_ship() -> None:
    """The model must match the locked overall dimensions of [ship] (1 %), and each aperture centre must sit on a
    radar panel front face of the model whose normal is the face normal (geometry.tilt_deg, face_phi_deg) within
    0.5 deg: a tilt or an array position that no longer matches the model is refused."""
    hull = _hull_body_vertices()
    checks = {"length_m": float(np.ptp(MODEL_VERTICES[:, 2])),
              "width_m": 2.0 * float(np.hypot(MODEL_VERTICES[:, 0], MODEL_VERTICES[:, 1]).max()),
              "hull_diameter_m": 2.0 * float(np.hypot(hull[:, 0], hull[:, 1]).max())}
    for k, v in checks.items():
        ref = P.get(f"ship.{k}")
        if abs(v / ref - 1.0) > 0.01:
            raise ValueError(f"{P.path}: the 3D model gives {k} = {v:.2f} m, ship.{k} is {ref} m (1 % tolerance)")
    for k in range(len(face_phis())):
        n = face_frame(k)[0]
        t, j = _first_hits(array_centre_m(k) + 0.05 * n, -n[None, :])
        if j[0] < 0 or abs(t[0] - 0.05) > 0.02:
            raise ValueError(f"{P.path}: face {k}: the aperture centre (ship.array_radius_m {ARRAY_RADIUS_M} m at the "
                             f"station ship.model_array_station_z_m) is not on a panel front face of the 3D model")
        tri = TRIANGLES[j[0]]
        nt = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        ang = math.degrees(math.acos(min(1.0, abs(float(nt @ n)) / float(np.linalg.norm(nt)))))
        if ang > 0.5:
            raise ValueError(f"{P.path}: face {k}: the radar panel of the 3D model is {ang:.1f} deg away from the face "
                             f"normal (geometry.tilt_deg {rm.SECTOR_TILT_DEG}, geometry.face_phi_deg)")


# ----------------------------------------------------------------------------- per-face structure tables
_TE = "tool.environment."
RAY_RINGS: int = P.get(_TE + "aperture_ray_rings")
GRID_STEP_UV: float = P.get(_TE + "block_grid_step_uv")
STRUCT_DIRS: int = P.get(_TE + "struct_noise_dirs")
RING_POINTS: int = P.get(_TE + "clutter_ring_points")
DOPPLER_GRID: int = P.get(_TE + "doppler_grid_points")
CLUTTER_V_SAMPLES: int = P.get(_TE + "clutter_velocity_samples")
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


_check_ship()


def panel_corners_m(k: int) -> np.ndarray:
    """Corners of the radar panel front face of face k in the 3D model (ship frame), in order around the normal."""
    n, eu, ev = face_frame(k)
    d = MODEL_VERTICES - array_centre_m(k)
    p = MODEL_VERTICES[(np.abs(d @ n) < 2e-3) & (np.linalg.norm(d, axis=1) < 2.0 * rm.ARRAY_ENVELOPE_M)]
    q = p - array_centre_m(k)
    return p[np.argsort(np.arctan2(q @ ev, q @ eu))]


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
        tri_front = _front_triangles(k)
        mat = np.full((len(dirs), len(org)), -1, dtype=np.int16)
        for i, o in enumerate(org):
            _, j = _first_hits(o, dirs, tri_front)
            mat[:, i] = np.where(j >= 0, TRI_MATERIAL[np.maximum(j, 0)], -1)
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
        _, j1 = _first_hits(array_centre_m(k) + _ORIGIN_OFFSET_M * n, fd, tri_front)
        sel = j1 >= 0
        out[f"sdirs_{k}"] = fd[sel]
        for bi, b in enumerate(BANDS):
            out[f"sepst_{b}_{k}"] = epst[TRI_MATERIAL[j1[sel]], bi]
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
        tmp = f"{CACHE_PATH}.{os.getpid()}.tmp"   # atomic: parallel test workers may rebuild it at the same time
        with open(tmp, "wb") as fh:
            np.savez_compressed(fh, fingerprint=np.array(fp), **t)
        os.replace(tmp, CACHE_PATH)
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


def _profile_gain(beam, rho):
    """Azimuthal-mean power pattern of a beam at the distance rho (lambda / D_env) of its axis."""
    near_r = np.arange(len(beam.mean_near)) * antenna.R_STEP
    far = np.array(beam.mean_far)
    idx = np.minimum((rho / antenna.FAR_PROFILE_STEP).astype(int), len(far) - 1)
    return np.where(rho <= antenna.R_MAX, np.interp(rho, near_r, np.array(beam.mean_near)), far[idx])


def _two_way_profile(band: str, rx_sll_db: float):
    tx, rx = rm.PATTERNS[band].tx, rm.rx_beam(band, rx_sll_db)
    return lambda rho: _profile_gain(tx, rho) * _profile_gain(rx, rho)


_SUB_NEAR_STEP = 0.02                      # (u, v) step of the fine sub-aperture pattern table, lambda / D_env


@lru_cache(maxsize=8)
def _subaperture_tables(band: str, kind: str):
    """Power pattern of a partition sub-aperture (step 3.1, P4 Q3) tabulated on the half plane v >= 0 (|AF| is even
    in (u, v) -> (-u, -v)): a fine grid inside R_MAX (main lobe, near sidelobes) and the far grid (step
    tool.pattern.far_step_lam_d) up to 2 D_env / lambda at the top of the band (any direction of the hemisphere)."""
    sa = rm.subaperture(band, kind)
    r_far = 2.0 * rm.ARRAY_ENVELOPE_M / rm.lambda_m(rm.BANDS[band].usable_hi_ghz) + 1.0
    out = []
    for r, step in ((antenna.R_MAX, _SUB_NEAR_STEP), (r_far, antenna.FAR_STEP)):
        n = int(math.ceil(r / step))
        us, vs = np.arange(-n, n + 1) * step, np.arange(0, n + 1) * step
        out.append((step, n, sa.power_grid(us, vs)))
    return tuple(out)


def subaperture_gain(band: str, kind: str, du: np.ndarray, dv: np.ndarray) -> np.ndarray:
    """Power pattern of a sub-aperture at offsets (du, dv) from its axis (lambda / D_env): bilinear on its tables."""
    (s1, n1, g1), (s2, n2, g2) = _subaperture_tables(band, kind)
    du, dv = np.where(dv < 0, -du, du), np.abs(dv)

    def look(step, n, g, u, v):
        x = np.clip(u / step + n, 0.0, 2 * n - 1e-9)
        y = np.clip(v / step, 0.0, n - 1e-9)
        i, j = x.astype(int), y.astype(int)
        fx, fy = x - i, y - j
        return ((1 - fx) * (1 - fy) * g[j, i] + fx * (1 - fy) * g[j, i + 1] + (1 - fx) * fy * g[j + 1, i]
                + fx * fy * g[j + 1, i + 1])
    near = (np.abs(du) <= antenna.R_MAX) & (dv <= antenna.R_MAX)
    return np.where(near, look(s1, n1, g1, du, dv), look(s2, n2, g2, du, dv))


def clutter_over_signal(s: Surface, k: int, dirs: np.ndarray, band: str, f_ghz: float, rx_sll_db: float,
                        r_km: float, dr_m: float, rcs_m2: float, tx_sub: Optional[str] = None) -> np.ndarray:
    """Sidelobe clutter of surface s in the target's range cell (r, dr) over the target's echo, for beams of face k at
    dirs: sigma_c / sigma_target, sigma_c = sum over the iso-range ring of sigma0 dA x the two-way relative gain.
    tx_sub: the TX beam of a partition sub-aperture (its 2D pattern in the axes of the face; step 3.1, P4 Q3)."""
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
    du, dv = (us[None, :] - ub[:, None]) * k_lam, (vs[None, :] - vb[:, None]) * k_lam
    rho = np.hypot(du, dv)
    if tx_sub is None:
        g = g2(rho)
    else:
        g = subaperture_gain(band, tx_sub, du, dv) * _profile_gain(rm.rx_beam(band, rx_sll_db), rho)
    rel = (wq[None, :] / wb[:, None]) ** 2 * g
    return sig_pt * rel.sum(axis=1) / rcs_m2


def doppler_rejected(s: Surface, v_min_mps: float, v_max_mps: float) -> bool:
    """The surface's closing-speed band, when the scenario gives it, does not overlap the target's span."""
    if s.closing_mps is None or s.spread_mps is None:
        return False
    return v_max_mps < s.closing_mps - s.spread_mps or v_min_mps > s.closing_mps + s.spread_mps


def doppler_weights(n: int, window_db: Optional[float]) -> np.ndarray:
    """Weights of the n pulses of a coherent block: uniform, or the 1D Taylor law (nbar = waveform.range_taylor_nbar)
    at window_db sampled at the pulse centres (step 3.1, P4 Q5)."""
    if window_db is None:
        return np.ones(n)
    nbar = rm.RANGE_TAYLOR_NBAR
    a = math.acosh(10.0 ** (abs(window_db) / 20.0)) / math.pi
    s2 = nbar ** 2 / (a ** 2 + (nbar - 0.5) ** 2)
    ms = range(1, nbar)
    x = (np.arange(n) - (n - 1) / 2.0) / n
    w = np.ones(n)
    for m in ms:
        num = np.prod([1.0 - m ** 2 / s2 / (a ** 2 + (k - 0.5) ** 2) for k in ms])
        den = np.prod([1.0 - m ** 2 / k ** 2 for k in ms if k != m])
        w += 2.0 * (-1) ** (m + 1) * num / (2.0 * den) * np.cos(2.0 * math.pi * m * x)
    return w


def target_velocities(v_min_mps: float, v_max_mps: float) -> np.ndarray:
    """Target closing speeds where the clutter residual is evaluated: CLUTTER_V_SAMPLES midpoints of the span (the Pd
    is averaged over them, uniform)."""
    if v_max_mps <= v_min_mps:
        return np.array([float(v_max_mps)])
    return v_min_mps + (np.arange(CLUTTER_V_SAMPLES) + 0.5) / CLUTTER_V_SAMPLES * (v_max_mps - v_min_mps)


def _norm_cdf(z: np.ndarray) -> np.ndarray:
    return 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))


@lru_cache(maxsize=4096)
def doppler_residual(closing_mps: Optional[float], spread_mps: Optional[float], v_min_mps: float, v_max_mps: float,
                     f_ghz: float, pri_s: float, n: int, window_db: Optional[float], phase_var: float) -> Tuple[float, ...]:
    """Clutter power left in the target's Doppler filter, relative to no Doppler discrimination (step 3.1, P4 Q4-Q8),
    at each target velocity of target_velocities: the surface's Doppler spectrum (Gaussian, mean closing_mps, 1 sigma
    spread_mps, folded over the PRF) through the filter |sum w_k e^(j 2 pi k x)|^2 / (sum w)^2 of an n-pulse coherent
    block, plus the white floor of the residual phase (oscillator + platform, variance phase_var) through the filter's
    noise bandwidth. 1 when the scenario gives no closing speed or the block has one pulse (no rejection)."""
    vt = target_velocities(v_min_mps, v_max_mps)
    if closing_mps is None or spread_mps is None or n < 2:
        return tuple(1.0 for _ in vt)
    lam = rm.lambda_m(f_ghz)
    m = DOPPLER_GRID
    x = np.arange(m) / m                                     # Doppler over one PRF, in cycles per PRI
    xc, sx = (2.0 * closing_mps / lam * pri_s) % 1.0, 2.0 * spread_mps / lam * pri_s
    if sx > 4.0:                                             # spread over many PRFs: uniform folded spectrum
        p = np.full(m, 1.0 / m)
    else:
        edges = (np.arange(m + 1) - 0.5) / m
        k_max = int(math.ceil(6.0 * sx)) + 1
        cdf = sum(_norm_cdf((edges + k - xc) / max(sx, 1e-12)) for k in range(-k_max, k_max + 1))
        p = np.diff(cdf)
        p = p / p.sum()
    w = doppler_weights(n, window_db)
    h = np.abs(np.exp(2j * math.pi * np.outer(x, np.arange(n))) @ w) ** 2 / w.sum() ** 2   # filter at offset x
    conv = np.real(np.fft.ifft(np.fft.fft(p) * np.conj(np.fft.fft(h))))   # conv[t] = sum_x p(x) h(x - t)
    xt = (2.0 * vt / lam * pri_s) % 1.0
    rho = np.interp(xt, np.r_[x, 1.0], np.r_[conv, conv[0]])
    enbw = float((w * w).sum() / w.sum() ** 2)
    return tuple(float(r) for r in np.clip((1.0 - phase_var) * rho + phase_var * enbw, 0.0, 1.0))


def spj_density_w_hz(sec: dict, f_ghz: float) -> Optional[float]:
    """EIRP density (W/Hz) of the sector's self-protection jammer at f (barrage over its class band, P4 Q12), None
    when the sector has none or f is outside its band."""
    jt = sec.get("jammer_type")
    if jt is None:
        return None
    b = rm.band_for_frequency(f_ghz)
    cov = JAMMER_CLASSES[jt][b.name]
    if not cov["lo"] <= f_ghz <= cov["hi"]:
        return None
    return 10.0 ** (_erp(jt, sec.get("jammer_erp_dbw_mhz"))[BANDS.index(b.name)] / 10.0) / 1e6


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
