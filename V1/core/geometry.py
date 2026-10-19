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
from . import radar_model as rm

FACE_PHI_DEG: Tuple[float, float, float, float] = mo.FACE_PHI
VIZ_FREQ_GHZ = 2.8   # reference frequency used for the viz-only scan limit (matches the optimiser)

# Ship model (cosmetic): traced from the reference side / front drawing, in drawing pixels (axial y downward from
# the nose tip, radial r from the ship axis), scaled to SHIP_LENGTH and placed so the centre of the radar arrays sits at
# the direction-sphere origin. RP = radial position of the face arrays.
SHIP_LENGTH = 1.1
_PX = SHIP_LENGTH / 950.0          # axial (Z) scale: nose tip y = 73 px, aft end y = 1023 px

# Real hull proportions (locked spec): length 136.4 m; max hull diameter 16.8 m (the hull tapers along its
# length, this is its widest point, y = 910/940 px in the drawing); max overall width 76.5 m, at the widest
# point of the radiator wings (not the hull), y = 659-780 px tier. The drawing's own radial proportions are
# only approximate, so the radial (X/Y) scale is recalibrated on the hull diameter rather than inherited
# from _PX, and the radiator panels get a further, separate stretch recalibrated on the overall width;
# everything else (telescopes, booms, rails, arrays) stays tied to the hull's radial scale.
SHIP_LENGTH_M, HULL_DIAMETER_M, SHIP_WIDTH_M = 136.4, 16.8, 76.5
_HULL_RADIUS_PX = 57.0
_PX_R = (HULL_DIAMETER_M / 2.0 / SHIP_LENGTH_M * SHIP_LENGTH) / _HULL_RADIUS_PX
_RADIATOR_SPAN_PX = 238.0
_RADIATOR_SCALE = (SHIP_WIDTH_M / 2.0 / SHIP_LENGTH_M * SHIP_LENGTH) / (_RADIATOR_SPAN_PX * _PX_R)

_Y_RADAR = 173.0                   # axial position of the four radar arrays (on the 20 deg cone behind the nose)
_TILT = math.tan(math.radians(20.0))
RP = (24.0 + _TILT * (_Y_RADAR - 156.0) + 1.5) * _PX_R

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
        surfaces[k] = quads_for_mask(vgrid, mask)
        outlines[k] = boundary_for_mask(vgrid, mask)
    return FaceSectorScene(surfaces=surfaces, outlines=outlines)


def build_sector_overlay(grid: SphereGrid, sec: dict, eps_min: float = -90.0, eps_max: float = 90.0,
                          radius: float = 1.015) -> Tuple[pv.PolyData, pv.PolyData]:
    """Mission watch-sector overlay (slightly above the face layer), clipped to the scanned zone.
    Returns (surface, outline)."""
    vgrid = grid.vertex_grid(radius)
    mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
    return quads_for_mask(vgrid, mask), boundary_for_mask(vgrid, mask)


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
        start = np.array([RP * math.cos(math.radians(pk)), RP * math.sin(math.radians(pk)), 0.0])
        tip = start + n * length
        arrow = pv.Line(start, tip)
        out.append({"k": k, "mesh": arrow, "color": FACE_COLORS[k], "name": FACE_NAMES[k],
                    "label_pos": tuple(n * 1.26)})
    return out


def _z(y_px: float) -> float:
    return (_Y_RADAR - y_px) * _PX


def _lathe(profile: List[Tuple[float, float]], n_theta: int = 48) -> pv.PolyData:
    """Surface of revolution about Z of a (y_px, r_px) profile (repeat a y for a step, r = 0 closes an end)."""
    y, r = np.array(profile, dtype=float).T
    t = np.linspace(0.0, 2 * math.pi, n_theta + 1)
    R = r[:, None] * _PX_R
    x, yy = R * np.cos(t), R * np.sin(t)
    z = np.repeat([[_z(v)] for v in y], n_theta + 1, axis=1)
    return pv.StructuredGrid(x, yy, z).extract_surface(algorithm="dataset_surface").clean()


def _local_box(frame, lo, hi) -> pv.PolyData:
    """Box with bounds lo..hi (px) in a local (e1, e2, e3) frame centred on the ship axis; e1/e2 (radial) use
    the hull's radial scale, e3 (axial, always ez) uses the length scale."""
    box = pv.Box(bounds=(lo[0] * _PX_R, hi[0] * _PX_R, lo[1] * _PX_R, hi[1] * _PX_R, lo[2] * _PX, hi[2] * _PX))
    m = np.eye(4)
    m[:3, :3] = np.array(frame).T
    return _flat(box.transform(m, inplace=False))


def _flat(mesh: pv.PolyData) -> pv.PolyData:
    """Per-face normals, so smooth shading keeps flat panels flat."""
    return mesh.compute_normals(split_vertices=True, feature_angle=30.0)


def build_ship() -> dict:
    """Ship hull traced from the reference drawing and renders (reference only): laser turret in the nose, body of
    revolution built from straight cylinder/cone segments only (neck, 20 deg radar-mount cone, long taper, fin-band
    cylinder, main cylinder, taper to the widest point, widest-point cylinder, taper to the engine), four hexagonal
    radar arrays on the 20 deg cone just behind the nose (one per face), four cylindrical telescopes, four ball
    turrets, four 45 deg booms, stepped radiator wings along +/-X, aerospike aft."""
    nose = [(73, 0), (75, 9), (79, 14), (86, 19), (95, 21), (100, 20), (105, 17), (112, 18)]
    body = [(112, 18),                          # nose / body junction
            (116, 24),                          # neck: cone
            (156, 24),                          # neck: cylinder
            (190, 24 + _TILT * 34),             # 20 deg radar-mount cone
            (190, 43),                          # shoulder step
            (390, 48),                          # long cone
            (390, 50),                          # fin-band step
            (490, 50),                          # fin-band cylinder
            (590, 52),                          # cone
            (905, 52),                          # main cylinder (long, spans the radiator roots)
            (910, 57),                          # cone to the widest point
            (940, 57),                          # widest-point cylinder
            (995, 40)]                          # cone down to the engine
    engine = [(995, 40), (1023, 17), (1023, 12)]     # aerospike: annular throat around a recessed central plug
    plug = [(1023, 12), (1012, 9), (1012, 0)]
    hull = [{"mesh": _lathe([(112, 0)] + body + [(995, 0)]), "color": SHIP_COLORS["hull"]},
            {"mesh": _lathe(nose + [(112, 0)]), "color": SHIP_COLORS["hull"]},
            {"mesh": _lathe([(995, 0)] + engine), "color": SHIP_COLORS["hull"]},
            {"mesh": _lathe(plug), "color": SHIP_COLORS["hull"]}]
    z_tank = _z(565)
    for k in range(4):
        c, s_ = math.cos(k * math.pi / 2), math.sin(k * math.pi / 2)
        er, et, ez = (c, s_, 0.0), (-s_, c, 0.0), (0.0, 0.0, 1.0)
        # telescope: cylinder along the ship axis, objective facing the nose
        hull.append({"mesh": pv.Cylinder(center=(58 * _PX_R * c, 58 * _PX_R * s_, _z(310)), direction=ez,
                                         radius=12.5 * _PX_R, height=44 * _PX), "color": SHIP_COLORS["hull"]})
        # ribbed band (fins) and laser turret on its arm: a half-pill (shaft + hemisphere cap, no stripe/disc),
        # flush with the hull surface at z_tank (hull radius 51.5 px there)
        hull.append({"mesh": _local_box((er, et, ez), (48, -3, _Y_RADAR - 490), (67, 3, _Y_RADAR - 390)),
                     "color": SHIP_COLORS["hull"]})
        hull.append({"mesh": pv.Cylinder(center=(60.5 * _PX_R * c, 60.5 * _PX_R * s_, z_tank), direction=er,
                                         radius=14 * _PX_R, height=18 * _PX_R), "color": SHIP_COLORS["hull"]})
        hull.append({"mesh": pv.Sphere(center=(69.5 * _PX_R * c, 69.5 * _PX_R * s_, z_tank), radius=14 * _PX_R,
                                       direction=er, end_phi=90), "color": SHIP_COLORS["hull"]})
        # 45 deg boom with its root fitting
        cb, sb = math.cos(k * math.pi / 2 + math.pi / 4), math.sin(k * math.pi / 2 + math.pi / 4)
        hull.append({"mesh": pv.Cylinder(center=(97 * _PX_R * cb, 97 * _PX_R * sb, z_tank), direction=(cb, sb, 0.0),
                                         radius=1.5 * _PX_R, height=66 * _PX_R), "color": SHIP_COLORS["hull"]})
        hull.append({"mesh": pv.Sphere(center=(58 * _PX_R * cb, 58 * _PX_R * sb, z_tank), radius=8 * _PX_R),
                     "color": SHIP_COLORS["hull"]})
    # radiator root rails along +/-X (the wide section of the side view; the body itself stays narrow, front view)
    for sgn in (-1, 1):
        hull.append({"mesh": _local_box(((sgn, 0, 0), (0, sgn, 0), (0, 0, 1)), (48, -20, _Y_RADAR - 905),
                                        (66, 20, _Y_RADAR - 590)), "color": SHIP_COLORS["hull"]})
    # radiator wings in the XZ plane: stepped tiers (y_top, y_bottom, half span), split into their panel rows;
    # the root (60 px, near the hull) keeps the hull's radial scale, the tip (span) gets the extra stretch that
    # recalibrates the panel on the real overall width (see _RADIATOR_SCALE above).
    for y0, y1, span, rows in ((597, 657, 193, 3), (659, 780, 238, 2), (782, 843, 187, 1), (845, 905, 136, 1)):
        for i in range(rows):
            ya = y0 + (y1 - y0) * i / rows + 0.8
            yb = y0 + (y1 - y0) * (i + 1) / rows - 0.8
            for sgn in (-1, 1):
                xa, xb = sorted((sgn * 60 * _PX_R, sgn * span * _PX_R * _RADIATOR_SCALE))
                hull.append({"mesh": _flat(pv.Box(bounds=(xa, xb, -1.5 * _PX_R, 1.5 * _PX_R, _z(yb), _z(ya)))),
                             "color": SHIP_COLORS["hull"]})
    arrays = []
    for k, pk in enumerate(FACE_PHI_DEG):
        n = face_normal(pk)
        c, s_ = math.cos(math.radians(pk)), math.sin(math.radians(pk))
        er, et, ez = (c, s_, 0.0), (-s_, c, 0.0), (0.0, 0.0, 1.0)
        center = (RP * c, RP * s_, 0.0)
        # hull-coloured array plate, on a slightly larger plate in the face colour (F1..F4 rim) -- the only
        # accent kept outside of the ship's dominant colour
        arrays.append({"k": k, "mesh": pv.Cylinder(center=center, direction=n, radius=18.5 * _PX_R, height=2 * _PX_R,
                                                   resolution=6), "color": FACE_COLORS[k]})
        hull.append({"mesh": pv.Cylinder(center=np.array(center) + n * 1.2 * _PX_R, direction=n, radius=16 * _PX_R,
                                         height=1 * _PX_R, resolution=6), "color": SHIP_COLORS["hull"]})
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
