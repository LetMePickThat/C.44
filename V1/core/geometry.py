#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEOMETRY -- shared ship-frame geometry and PyVista mesh builders for the radar coverage 3D views
(mission-preparation and results tabs; the plotter itself is driven by gui/scene_view.py).

Single source of truth for the ship frame: the direction raster IS the optimiser's (SphereGrid subclasses
core.optimizer.Sphere: same face frame, ownership and sector masks), so the 3D picture never drifts from the
radar model.

Ship frame (confirmed by the user): Z = ship longitudinal axis, +Z = nose (radar faces + telescope),
-Z = aft (thrusters, behind the radiators); X = radiator axis; Y = third axis; phi = azimuth about Z
from +X toward +Y; eps = elevation from the XY plane toward +Z. Look and default view inherited from the
earlier volumes.html viewer.

This module only builds geometry (numpy / pyvista). It has no Qt / Plotter dependency so it can be
unit-tested headlessly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pyvista as pv

# ----------------------------------------------------------------------------- radar model (shared)
# One module object for the whole application, so a setting changed on the model (tilt, eps window...) applies
# to the 3D view, the fast estimate and the exact calculation alike.
from . import optimizer as mo
from . import environment as env
from . import radar_model as rm
from .params import P

FACE_PHI_DEG: Tuple[float, float, float, float] = mo.FACE_PHI
VIZ_FREQ_GHZ = P.get("tool.display.viz_freq_ghz")   # reference frequency used for the viz-only scan limit

# Ship model: built from the structure primitives of radar_system.toml ([ship], core/environment.py), the same ones the
# masking and the structure noise use, so the picture never drifts from the calculation (step 2.5). Scene units: the
# ship is SHIP_LENGTH long, the centre of the radar arrays (z = 0, the arrays' axial station) at the origin.
SHIP_LENGTH = 1.1
SHIP_LENGTH_M = P.get("ship.length_m")
_M = SHIP_LENGTH / SHIP_LENGTH_M              # scene units per metre
RP = env.ARRAY_RADIUS_M * _M                  # radial position of the face arrays

# Palette (inherited from the volumes.html viewer).
COLORS: Dict[str, str] = {
    "F1": "#2f7fd0", "F2": "#e08a1e", "F3": "#2fa35a", "F4": "#d6453d",
    "void": "#8a95a3", "excl": "#b9854a", "ship": "#7b8794",
    "axis_x": "#d9a21b", "axis_y": "#9a5fd0", "axis_z": "#d6453d",
    "muted": "#93a1b2", "fg": "#18222d",
}
# Ship-model colour: a single off-white dominant colour for the whole hull; the only accents kept outside of
# it are the per-face rim (FACE_COLORS) on the radar array plates.
SHIP_COLORS: Dict[str, str] = {"hull": "#e2e0da"}
FACE_COLORS: Tuple[str, str, str, str] = (COLORS["F1"], COLORS["F2"], COLORS["F3"], COLORS["F4"])
FACE_NAMES: Tuple[str, str, str, str] = ("F1", "F2", "F3", "F4")

SECTOR_PALETTE: Tuple[str, ...] = (
    "#1f5fae", "#c0562a", "#1f8a52", "#b03a4a", "#7a4fb0", "#1a9c9c", "#9c7a1a", "#4a5fd0",
)


def sector_color(i: int) -> str:
    return SECTOR_PALETTE[i % len(SECTOR_PALETTE)]


# ----------------------------------------------------------------------------- face frame
def face_normal(phi_deg: float) -> np.ndarray:
    return np.array(mo.face_frame(phi_deg)[0])


def array_centre(phi_deg: float) -> np.ndarray:
    """True physical position of the face's array plate: on the hull, offset RP from the ship axis at the
    array's axial station (scene origin) -- same point as build_ship()'s array disc centre and
    build_face_normals()'s line start. The coverage/budget maths keeps treating the radar as a point at the
    ship's axis (exact and scale-invariant, see core.optimizer.face_frame); this is purely cosmetic, so the
    unit-sphere overlay visually meets the antenna instead of floating at the ship's centreline -- at 1:1 km
    scale the same offset (a few metres) is negligible against the sector's range, so build_sector_volume_km()
    is left at the origin."""
    return np.array([RP * math.cos(math.radians(phi_deg)), RP * math.sin(math.radians(phi_deg)), 0.0])


# ----------------------------------------------------------------------------- direction sphere raster
class SphereGrid(mo.Sphere):
    """The optimiser's direction-sphere raster (core.optimizer.Sphere: same faces, same sector masks) at a resolution
    suited to interactive 3D views, plus the grid-line vertices the mesh builders need."""

    def __init__(self, res_deg: float = 1.0):
        super().__init__(res_deg=res_deg)
        self.res_deg = res_deg

    def sector_mask(self, sec: dict) -> np.ndarray:
        """Boolean mask of a mission watch sector (box or cap); cached, read-only."""
        return self.mask(sec)

    def vertex_grid(self, radius: float = 1.0) -> np.ndarray:
        """Grid-line vertices (n_phi+1, n_eps+1, 3) bracketing the cell-centred samples above.
        Cached per radius (the grid is fixed): callers must not modify the returned array."""
        cache = self.__dict__.setdefault("_vgrid_cache", {})
        if radius not in cache:
            phi_e = np.linspace(0.0, 360.0, self.n_phi + 1)
            eps_e = np.linspace(-90.0, 90.0, self.n_eps + 1)
            P, E = np.meshgrid(np.radians(phi_e), np.radians(eps_e), indexing="ij")
            ce = np.cos(E)
            x = radius * ce * np.cos(P)
            y = radius * ce * np.sin(P)
            z = radius * np.sin(E)
            cache[radius] = np.stack([x, y, z], axis=-1)
        return cache[radius]


def _vertex_ids(n_phi: int, n_eps: int) -> np.ndarray:
    """Vertex index grid (n_phi+1, n_eps+1) with the topology of the sphere: the phi=360 column is the
    phi=0 one (seam) and each pole row collapses to a single vertex."""
    idx = np.arange((n_phi + 1) * (n_eps + 1)).reshape(n_phi + 1, n_eps + 1)
    idx[n_phi, :] = idx[0, :]
    idx[:, 0] = idx[0, 0]
    idx[:, n_eps] = idx[0, n_eps]
    return idx


def _compact(vgrid: np.ndarray, cells: np.ndarray, n_per_cell: int) -> Tuple[np.ndarray, np.ndarray]:
    """Keep only the vertices used by `cells` (n, n_per_cell); returns (points, VTK cell array)."""
    used, inv = np.unique(cells, return_inverse=True)
    conn = inv.reshape(-1, n_per_cell).astype(np.int64)
    flat = np.hstack([np.full((conn.shape[0], 1), n_per_cell, dtype=np.int64), conn]).ravel()
    return vgrid.reshape(-1, 3)[used], flat


def quads_for_mask(vgrid: np.ndarray, mask: np.ndarray) -> pv.PolyData:
    """Build a PolyData covering the True cells of `mask` (shape n_phi x n_eps), using the
    (n_phi+1, n_eps+1, 3) vertex grid from SphereGrid.vertex_grid(). Quads, except triangles on the
    pole rows; seam and pole vertices are shared, so the surface is closed where the mask is."""
    if not mask.any():
        return pv.PolyData()
    n_phi, n_eps = mask.shape
    idx = _vertex_ids(n_phi, n_eps)
    i0, i1, i2, i3 = idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]
    body = mask.copy()
    body[:, 0] = body[:, -1] = False
    south, north = np.zeros_like(mask), np.zeros_like(mask)
    south[:, 0], north[:, -1] = mask[:, 0], mask[:, -1]
    quads = np.stack([i0[body], i1[body], i2[body], i3[body]], axis=1)
    tris = np.concatenate([np.stack([i0[south], i2[south], i3[south]], axis=1),    # i0 == i1 = south pole
                           np.stack([i0[north], i1[north], i2[north]], axis=1)])   # i2 == i3 = north pole
    used, inv = np.unique(np.concatenate([quads.ravel(), tris.ravel()]), return_inverse=True)
    q, t = inv[:quads.size].reshape(-1, 4), inv[quads.size:].reshape(-1, 3)
    faces = np.concatenate([np.hstack([np.full((len(q), 1), 4), q]).ravel(),
                            np.hstack([np.full((len(t), 1), 3), t]).ravel()]).astype(np.int64)
    return pv.PolyData(vgrid.reshape(-1, 3)[used], faces)


def boundary_for_mask(vgrid: np.ndarray, mask: np.ndarray) -> pv.PolyData:
    """Outline of the True cells of `mask`: every grid edge between a True cell and a False one (phi wraps
    around, pole edges have zero length and are skipped). Same result as a VTK boundary-edge extraction on
    quads_for_mask(), computed directly on the raster."""
    if not mask.any():
        return pv.PolyData()
    n_phi, n_eps = mask.shape
    idx = _vertex_ids(n_phi, n_eps)
    # meridian edges: vertex column i lies between cells i-1 and i (wrapping)
    mer = mask != np.roll(mask, 1, axis=0)
    ii, jj = np.nonzero(mer)
    edges = [np.stack([idx[ii, jj], idx[ii, jj + 1]], axis=1)]
    # parallel edges: vertex row j (1..n_eps-1) lies between cells j-1 and j
    par = mask[:, 1:] != mask[:, :-1]
    ii, jj = np.nonzero(par)
    edges.append(np.stack([idx[ii, jj + 1], idx[ii + 1, jj + 1]], axis=1))
    pts, lines = _compact(vgrid, np.concatenate(edges), 2)
    return pv.PolyData(pts, lines=lines)


# ----------------------------------------------------------------------------- scene builders
@dataclass
class FaceSectorScene:
    """The 4 face-responsibility sectors (base layer), coloured by face, plus their outlines."""
    surfaces: Dict[int, pv.PolyData]
    outlines: Dict[int, pv.PolyData]


def build_face_sectors(grid: SphereGrid, eps_min: float = -90.0, eps_max: float = 90.0,
                        radius: float = 1.0) -> FaceSectorScene:
    vgrid = grid.vertex_grid(radius)
    scanned = grid.scanned(eps_min, eps_max)
    surfaces, outlines = {}, {}
    for k in range(4):
        mask = scanned & (grid.owner == k)
        surf, out = quads_for_mask(vgrid, mask), boundary_for_mask(vgrid, mask)
        off = array_centre(FACE_PHI_DEG[k])   # anchor on the array's true position, not the ship's axis
        surf.points = surf.points + off
        out.points = out.points + off
        surfaces[k], outlines[k] = surf, out
    return FaceSectorScene(surfaces=surfaces, outlines=outlines)


def build_sector_overlay(grid: SphereGrid, sec: dict, eps_min: float = -90.0, eps_max: float = 90.0,
                          radius: float = 1.015) -> Tuple[pv.PolyData, pv.PolyData]:
    """Mission watch-sector overlay (slightly above the face layer), clipped to the scanned zone and anchored,
    per face, on that face's array (see array_centre) rather than the ship's axis -- a sector that straddles
    two faces (box or cap spanning a face boundary) is built as one piece per face and appended, since each
    piece has its own array offset. Returns (surface, outline)."""
    mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
    vgrid = grid.vertex_grid(radius)
    surfs, outs = [], []
    for k in range(4):
        fmask = mask & (grid.owner == k)
        if not fmask.any():
            continue
        s, o = quads_for_mask(vgrid, fmask), boundary_for_mask(vgrid, fmask)
        off = array_centre(FACE_PHI_DEG[k])
        s.points, o.points = s.points + off, o.points + off
        surfs.append(s)
        outs.append(o)
    if not surfs:
        return pv.PolyData(), pv.PolyData()
    surf, out = surfs[0], outs[0]
    for s, o in zip(surfs[1:], outs[1:]):
        surf, out = surf.append_polydata(s), out.append_polydata(o)
    return surf, out


def build_sector_volume_km(grid: SphereGrid, sec: dict, eps_min: float = -90.0,
                           eps_max: float = 90.0) -> Tuple[pv.PolyData, pv.PolyData]:
    """True-scale watch volume (1 scene unit = 1 km): the sector's cap at its required range sec["range_km"],
    closed by walls down to the radar at the origin, clipped to the scanned zone. Returns (volume surface,
    outline of the cap)."""
    r = float(sec["range_km"])
    vgrid = grid.vertex_grid(1.0)
    mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
    if not mask.any():
        return pv.PolyData(), pv.PolyData()
    cap, outline = quads_for_mask(vgrid, mask), boundary_for_mask(vgrid, mask)
    cap.points = cap.points * r
    outline.points = outline.points * r
    seg = outline.lines.reshape(-1, 3)[:, 1:]
    origin = outline.n_points
    tris = np.hstack([np.full((len(seg), 1), 3), np.full((len(seg), 1), origin), seg]).astype(np.int64).ravel()
    walls = pv.PolyData(np.vstack([outline.points, [[0.0, 0.0, 0.0]]]), tris)
    return cap.append_polydata(walls), outline


SECTOR_GEOMETRY_FIELDS = mo.SECTOR_GEOMETRY_FIELDS
sector_geometry_key = mo.sector_geometry_key   # everything build_sector_overlay() depends on (mesh cache key)


def build_axes(length: float = 1.35) -> List[dict]:
    return [
        {"line": pv.Line((0, 0, 0), (length, 0, 0)), "color": COLORS["axis_x"], "text": "X",
         "label_pos": (length * 1.1, 0, 0)},
        {"line": pv.Line((0, 0, 0), (0, length, 0)), "color": COLORS["axis_y"], "text": "Y",
         "label_pos": (0, length * 1.1, 0)},
        {"line": pv.Line((0, 0, 0), (0, 0, length)), "color": COLORS["axis_z"], "text": "Z (nose)",
         "label_pos": (0, 0, length * 1.1)},
        {"line": None, "color": COLORS["muted"], "text": "-Z thrusters",
         "label_pos": (0, 0, _z(1023) - 0.08), "no_line": True},
    ]


def build_face_normals(length: float = 1.12) -> List[dict]:
    out = []
    for k, pk in enumerate(FACE_PHI_DEG):
        n = face_normal(pk)
        start = array_centre(pk)
        tip = start + n * length
        arrow = pv.Line(start, tip)
        out.append({"k": k, "mesh": arrow, "color": FACE_COLORS[k], "name": FACE_NAMES[k],
                    "label_pos": tuple(start + n * 1.26)})
    return out


def _lathe(z_m: List[float], r_m: List[float], n_theta: int = 48) -> pv.PolyData:
    """Surface of revolution about Z of a (z, r) profile in metres (repeat a z for a step, r = 0 closes an end)."""
    t = np.linspace(0.0, 2 * math.pi, n_theta + 1)
    r = np.array(r_m)[:, None] * _M
    x, yy = r * np.cos(t), r * np.sin(t)
    z = np.repeat(np.array(z_m)[:, None] * _M, n_theta + 1, axis=1)
    return pv.StructuredGrid(x, yy, z).extract_surface(algorithm="dataset_surface").clean()


def _flat(mesh: pv.PolyData) -> pv.PolyData:
    """Per-face normals, so smooth shading keeps flat panels flat."""
    return mesh.compute_normals(split_vertices=True, feature_angle=30.0)


def _part_meshes(part: "env.Part") -> List[pv.PolyData]:
    """Meshes of one structure part of [ship.parts], at each of its azimuths."""
    out = []
    for az in part.azimuths_deg:
        c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
        er, et, ez = np.array([c, s_, 0.0]), np.array([-s_, c, 0.0]), np.array([0.0, 0.0, 1.0])
        if part.type == "box_rad":
            box = pv.Box(bounds=(part.v("r_min_m") * _M, part.v("r_max_m") * _M, -part.v("half_width_m") * _M,
                                 part.v("half_width_m") * _M, part.v("z_min_m") * _M, part.v("z_max_m") * _M))
            m = np.eye(4)
            m[:3, :3] = np.array((er, et, ez)).T
            out.append(_flat(box.transform(m, inplace=False)))
            continue
        centre = er * part.v("radial_m") * _M + ez * part.v("z_m") * _M
        if part.type == "sphere":
            out.append(pv.Sphere(center=centre, radius=part.v("radius_m") * _M))
        else:
            out.append(pv.Cylinder(center=centre, direction=ez if part.type == "cylinder_z" else er,
                                   radius=part.v("radius_m") * _M, height=part.v("length_m") * _M))
    return out


def build_ship() -> dict:
    """Ship from the structure primitives of the parameter file (core/environment.py): hull of revolution, parts
    (telescopes, fins, laser turrets, booms, radiator rails and wings...), and the four radar arrays on the
    radar-mount cone (one per face, envelope of the real aperture)."""
    hull = [{"mesh": _lathe(list(env.HULL_Z), list(env.HULL_R)), "color": SHIP_COLORS["hull"]}]
    for part in env.PARTS:
        hull.extend({"mesh": m, "color": SHIP_COLORS["hull"]} for m in _part_meshes(part))
    arrays = []
    rad = rm.ARRAY_ENVELOPE_M / 2.0 * _M
    for k, pk in enumerate(FACE_PHI_DEG):
        n = face_normal(pk)
        center = array_centre(pk)
        # hull-coloured array plate on a slightly larger plate in the face colour (F1..F4 rim)
        arrays.append({"k": k, "mesh": pv.Cylinder(center=center, direction=n, radius=rad * 1.06,
                                                   height=0.25 * _M, resolution=8), "color": FACE_COLORS[k]})
        hull.append({"mesh": pv.Cylinder(center=np.array(center) + n * 0.15 * _M, direction=n, radius=rad,
                                         height=0.1 * _M, resolution=8), "color": SHIP_COLORS["hull"]})
    return {"hull": hull, "arrays": arrays}


def km_axes_step(max_range_km: float) -> Tuple[float, int]:
    """(graduation step, number of graduations) of the 1:1 km axes: a round step (1-2-5 x 10^n) giving about 4 to 6
    graduations, the last one at or beyond max_range_km."""
    target = max_range_km / 4.0
    base = 10.0 ** math.floor(math.log10(target))
    step = max(m * base for m in (1.0, 2.0, 5.0) if m * base <= target)
    return step, int(math.ceil(max_range_km / step - 1e-9))


MOON_RADIUS_KM = 1737.4


def build_background_sphere(altitude_km: float, radius_km: float = MOON_RADIUS_KM) -> pv.PolyData:
    """Background reference sphere for the 1:1 km view (e.g. the Moon): radius_km, centred radius_km + altitude_km
    away along -Z, so the radar sits altitude_km above its surface. Textured (moon_texture())."""
    return pv.Sphere(radius=radius_km, center=(0.0, 0.0, -(radius_km + altitude_km)),
                     theta_resolution=96, phi_resolution=96, texture_coordinates=True)


MOON_TEXTURE_PATH = Path(__file__).resolve().parent.parent / "assets" / "textures" / "moon.jpg"


@lru_cache(maxsize=1)
def moon_texture() -> np.ndarray:
    """Moon surface texture (equirectangular, 2:1): the real photo at MOON_TEXTURE_PATH when present (drop a
    higher- or lower-res equirectangular JPG/PNG there to change it -- 8k (Solar System Scope, CC BY 4.0) as of
    writing), else a low-res procedural fallback so the app still runs without that asset."""
    if MOON_TEXTURE_PATH.exists():
        return pv.Texture(str(MOON_TEXTURE_PATH)).to_array()
    return _procedural_moon_texture()


@lru_cache(maxsize=1)
def _procedural_moon_texture(width: int = 256, height: int = 128) -> np.ndarray:
    """Fallback used only when MOON_TEXTURE_PATH is missing: a low-res procedural Moon surface (equirectangular
    grey, mare patches and craters), no network access or bundled image asset needed."""
    rng = np.random.default_rng(1)
    u = np.linspace(0.0, 1.0, width, endpoint=False)
    v = np.linspace(0.0, 1.0, height, endpoint=False)
    uu, vv = np.meshgrid(u, v)

    def du(cu):   # longitude distance, wrapped
        return np.minimum(np.abs(uu - cu), 1.0 - np.abs(uu - cu))

    grey = np.full((height, width), 0.62)
    for cu, cv, r, depth in zip(rng.uniform(0, 1, 6), rng.uniform(0.2, 0.8, 6),
                                rng.uniform(0.08, 0.18, 6), rng.uniform(0.08, 0.16, 6)):
        grey -= depth * np.exp(-(du(cu) ** 2 + (vv - cv) ** 2) / (2 * r ** 2))   # dark mare patches
    for cu, cv, r in zip(rng.uniform(0, 1, 140), rng.uniform(0.05, 0.95, 140), rng.uniform(0.01, 0.05, 140)):
        d = np.sqrt(du(cu) ** 2 + (vv - cv) ** 2) / r
        grey -= 0.12 * np.exp(-d ** 2 / (2 * 0.35 ** 2)) * (d < 1.0)            # crater floor
        grey += 0.10 * np.exp(-(d - 1.0) ** 2 / (2 * 0.12 ** 2))                # crater rim
    grey = np.clip(grey + rng.normal(0.0, 0.015, size=grey.shape), 0.0, 1.0)
    rgb = np.stack([grey, grey, grey * 0.98], axis=-1)   # the faintest warm-grey tint
    return (rgb * 255).astype(np.uint8)
