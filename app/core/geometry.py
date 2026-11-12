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
from typing import Dict, List, Optional, Tuple

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

# Ship model: the 3D model of radar_system.toml ([ship] model_path, placed by core/environment.py), the same triangles
# the masking and the structure noise use, so the picture never drifts from the calculation. Scene units: the ship is
# SHIP_LENGTH long, the centre of the radar apertures (z = 0, their axial station) at the origin.
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


def build_sector_split(grid: SphereGrid, sec: dict, eps_min: float = -90.0, eps_max: float = 90.0,
                       radius: float = 1.015) -> Tuple[pv.PolyData, pv.PolyData, pv.PolyData]:
    """E3 (user, 10 November 2026): a watched volume on the sphere, (its executed volumes: the parts each face scans,
    one piece per face, cell data "face" = its index; the outline of the WHOLE watched volume;
    its coverage hole: the part no face scans at this attitude)."""
    full = grid.sector_mask(sec)
    scanned = grid.scanned(eps_min, eps_max)
    vgrid = grid.vertex_grid(radius)
    surfs = []
    for k in range(4):
        fmask = full & scanned & (grid.owner == k)
        if not fmask.any():
            continue
        s = quads_for_mask(vgrid, fmask)        # (from the ship's centre, as the outline: they coincide, user)
        s.cell_data["face"] = np.full(s.n_cells, k, np.int8)
        surfs.append(s)
    executed = pv.PolyData()
    for s in surfs:
        executed = s if not executed.n_points else executed.append_polydata(s)
    hole_mask = full & ~scanned
    hole = quads_for_mask(vgrid, hole_mask) if hole_mask.any() else pv.PolyData()
    return executed, (boundary_for_mask(vgrid, full) if full.any() else pv.PolyData()), hole


# ----------------------------------------------------------------------------- smooth edges (display, 10 Nov 2026)
# The masks are decided cell by cell on the (phi, eps) raster of the ship frame: an edge oblique to that raster (a
# volume in a frame turned by the attitude, the faces' scan-limit cones) showed 0.5 deg steps. For the drawing, signed
# margins (deg, > 0 inside) are evaluated at the raster's vertices and the surface is cut where they vanish (linear
# between vertices): smooth edges to a fraction of a cell. The budget keeps the raster.
SMOOTH_PAD_DEG = 2.0                    # cells within this outside a volume are kept for the cut


def _dirs_angles(d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """phi, eps (deg, ship frame) of unit directions (n, 3)."""
    return (np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 360.0,
            np.degrees(np.arcsin(np.clip(d[:, 2], -1.0, 1.0))))


PLAYED_EDGE_DEG = 1.0               # "show effectively played": margin given to the cells in / out of the relief


def sector_margin(sec: dict, phi: np.ndarray, eps: np.ndarray) -> np.ndarray:
    """Signed angular margin (deg, > 0 inside) of ship-frame directions to a sector's edge (sec resolved in the
    ship frame, its frame_m applied). Display key "_played_relief" (a relief code, core/optimizer.relief_code; user,
    12 November 2026: "show effectively played"): the directions another volume serves are taken out."""
    m = _shape_margin(sec, phi, eps)
    code = sec.get("_played_relief")
    if code:
        rel = mo.relief_cells(code, np.asarray(phi, float), np.asarray(eps, float))
        m = np.minimum(m, np.where(rel, -PLAYED_EDGE_DEG, PLAYED_EDGE_DEG))
    return m


PLAYED_EDGE_TOL_DEG = 0.1           # a cut point this far inside the volume's own shape lies on the relief's boundary


def relief_edge_function(sec: dict):
    """"Show effectively played" (user, 12 November 2026): on_relief(d) -> True for the unit directions on the boundary
    of the part another volume serves, and not on the volume's own edges; no walls, no outline there (they followed the
    relieving volume's cone from the ship to the range and hid the relief). None without a played relief."""
    code = sec.get("_played_relief")
    if not code:
        return None

    def on_relief(d):
        # next to the relieved cells (one cell grown) and inside the volume's own shape: the relief's boundary only
        # (user, 12 November 2026: "inside the shape" alone also took the coverage hole's rim, the horizon and the
        # faces' scan limits, whose walls vanished: a whole-sphere volume has no own edge at all)
        phi, eps = _dirs_angles(np.asarray(d, float).reshape(-1, 3))
        near = mo.relief_cells(code, phi, eps, grow=1)
        return near & (_shape_margin(sec, phi, eps) > PLAYED_EDGE_TOL_DEG)
    return on_relief


def _drop_segments(lines: pv.PolyData, skip_of) -> pv.PolyData:
    """The line segments of `lines` not lying (both ends) where skip_of(unit directions) is True."""
    if skip_of is None or not lines.n_points or not lines.n_lines:
        return lines
    pts = np.asarray(lines.points, float)
    seg = lines.lines.reshape(-1, 3)[:, 1:]
    sk = np.asarray(skip_of(pts / np.maximum(np.linalg.norm(pts, axis=1), 1e-12)[:, None]), bool)
    seg = seg[~(sk[seg[:, 0]] & sk[seg[:, 1]])]
    if not len(seg):
        return pv.PolyData()
    return pv.PolyData(pts, lines=np.column_stack([np.full(len(seg), 2), seg]).astype(np.int64).ravel())


def _shape_margin(sec: dict, phi: np.ndarray, eps: np.ndarray) -> np.ndarray:
    """sector_margin of the volume's own shape."""
    from . import frames
    p, e, off = frames.sector_angles(sec, phi, eps)
    shape = sec.get("shape", "box")
    if shape == "cap":
        p0, e0 = math.radians(sec["phi0_deg"]), math.radians(sec["eps0_deg"])
        pr, er = np.radians(p), np.radians(e)
        cosang = np.sin(er) * math.sin(e0) + np.cos(er) * math.cos(e0) * np.cos(pr - p0)
        return float(sec["radius_deg"]) - np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0)))
    if shape == "rect":
        p0, e0 = math.radians(sec["phi0_deg"]), math.radians(sec["eps0_deg"])
        c = np.array([math.cos(e0) * math.cos(p0), math.cos(e0) * math.sin(p0), math.sin(e0)])
        east = np.array([-math.sin(p0), math.cos(p0), 0.0])
        north = np.cross(c, east)
        r = math.radians(float(sec.get("rot_deg", 0.0)))
        e1, n1 = math.cos(r) * east + math.sin(r) * north, -math.sin(r) * east + math.cos(r) * north
        pr, er = np.radians(p), np.radians(e)
        dd = np.stack([np.cos(er) * np.cos(pr), np.cos(er) * np.sin(pr), np.sin(er)], axis=-1)
        az = np.degrees(np.arctan2(dd @ e1, dd @ c))
        el = np.degrees(np.arcsin(np.clip(dd @ n1, -1.0, 1.0)))
        return np.minimum(float(sec["half_width_deg"]) - np.abs(az), float(sec["half_height_deg"]) - np.abs(el))
    m_eps = np.minimum(e - (sec["eps_min_deg"] + off), (sec["eps_max_deg"] + off) - e)
    span = float(sec["phi_max_deg"]) - float(sec["phi_min_deg"])
    if span >= 360.0:
        return m_eps
    span = span % 360.0 or 360.0
    centre = float(sec["phi_min_deg"]) + 0.5 * span
    dphi = np.abs((p - centre + 180.0) % 360.0 - 180.0)
    return np.minimum(m_eps, (0.5 * span - dphi) * np.cos(np.radians(e)))


def face_margin(k: int, d: np.ndarray) -> np.ndarray:
    """Signed margin (deg, > 0 inside) of ship-frame unit directions d (n, 3) to the zone face k owns (the rule of
    optimizer.face_of: its side of the +/-45 deg meridian planes, inside its scan-limit cone)."""
    t = math.radians(rm.SECTOR_TILT_DEG)
    sin_lim = math.sin(math.radians(rm.SECTOR_RADIUS_DEG))
    n, eu, ev = mo.face_frame(FACE_PHI_DEG[k])
    w, u, v = d @ np.asarray(n), d @ np.asarray(eu), d @ np.asarray(ev)
    m = np.minimum.reduce([w, w * math.cos(t) - v * math.sin(t) - np.abs(u), sin_lim - np.sqrt(u * u + v * v)])
    return np.degrees(m)


def _region(grid: SphereGrid, sec: dict, radius: float) -> pv.PolyData:
    """The raster cells near the sector (margin at their centre > -SMOOTH_PAD_DEG), as a surface at radius."""
    near = sector_margin(sec, grid.phi.ravel(), grid.eps.ravel()).reshape(grid.phi.shape) > -SMOOTH_PAD_DEG
    if not near.any():
        return pv.PolyData()
    return quads_for_mask(grid.vertex_grid(radius), near).triangulate()


def _cut(region: pv.PolyData, f: np.ndarray) -> pv.PolyData:
    """The part of region where f >= 0 (linear between vertices)."""
    if not len(f) or float(np.max(f)) < 0.0:
        return pv.PolyData()
    r = region.copy()
    r.point_data["f"] = f
    out = r.clip_scalar(scalars="f", value=0.0, invert=False)
    return pv.PolyData() if not out.n_points else out.extract_surface(algorithm="dataset_surface")


def build_face_sectors_smooth(grid: SphereGrid, eps_min: float = -90.0, eps_max: float = 90.0,
                              radius: float = 1.0) -> "FaceSectorScene":
    """build_face_sectors with smooth edges (the faces' zones: their +/-45 deg planes, their scan-limit cones, the
    eps window), anchored on each face's array as before."""
    region = quads_for_mask(grid.vertex_grid(radius), np.ones(grid.phi.shape, bool)).triangulate()
    d = region.points / np.linalg.norm(region.points, axis=1)[:, None]
    _phi, eps = _dirs_angles(d)
    m_win = np.minimum(eps - eps_min, eps_max - eps)
    surfaces, outlines = {}, {}
    for k in range(4):
        f = np.minimum(face_margin(k, d), m_win)
        surf = _cut(region, f)
        r = region.copy()
        r.point_data["f"] = f
        out = r.contour(isosurfaces=[0.0], scalars="f") if float(np.max(f)) > 0.0 else pv.PolyData()
        off = array_centre(FACE_PHI_DEG[k])
        if surf.n_points:
            surf.points = surf.points + off
        if out.n_points:
            out.points = out.points + off
        surfaces[k], outlines[k] = surf, out
    return FaceSectorScene(surfaces=surfaces, outlines=outlines)


def build_sector_split_smooth(grid: SphereGrid, sec: dict, eps_min: float = -90.0, eps_max: float = 90.0,
                              radius: float = 1.015) -> Tuple[pv.PolyData, pv.PolyData, pv.PolyData]:
    """build_sector_split with smooth edges: (executed volumes, cell data "face"; the outline of the whole watched
    volume; the coverage hole)."""
    region = _region(grid, sec, radius)
    if not region.n_points:
        return pv.PolyData(), pv.PolyData(), pv.PolyData()
    d = region.points / np.linalg.norm(region.points, axis=1)[:, None]
    phi, eps = _dirs_angles(d)
    s = sector_margin(sec, phi, eps)
    m_win = np.minimum(eps - eps_min, eps_max - eps)
    cover = []
    executed = pv.PolyData()
    for k in range(4):
        mk = np.minimum(face_margin(k, d), m_win)
        cover.append(mk)
        part = _cut(region, np.minimum(s, mk))
        if part.n_points:
            part.cell_data["face"] = np.full(part.n_cells, k, np.int8)
            executed = part if not executed.n_points else executed.append_polydata(part)
    hole = _cut(region, np.minimum(s, -np.max(cover, axis=0)))
    r = region.copy()
    r.point_data["s"] = s
    outline = r.contour(isosurfaces=[0.0], scalars="s") if float(np.max(s)) > 0.0 else pv.PolyData()
    outline = _drop_segments(outline, relief_edge_function(sec))      # (played: the volume's own edges only)
    return executed, outline, hole


GAUGE_GREEN, GAUGE_RED = "#3cb371", "#d6453d"     # range gauge: the range held, the shortfall (D)


# ----------------------------------------------------------------------------- the ground's horizon (G2, G3)
# User, 10 November 2026 (pass on the geometry): a volume cut by the ground (the useful range, step 3.1 P4 Q9) was drawn
# by placing each vertex of its mesh at the range its direction needs. Across a body's horizon that range jumps (just
# below, the ray meets the ground at the horizon distance; just above, it misses it and goes to the full range), so
# every triangle and outline segment astride the horizon joined a ground point to a far point: fans of slivers. Now the
# meshes are cut along the horizon (limb_margin_deg = 0), the ground side refined near it (the ground distance falls
# steeply there and coarse chords passed under the surface), the two sides joined by the tangent cone between the
# horizon distance and the full range, and the outline densified with a clean step at the crossing.
LIMB_BAND_DEG = 3.0                    # G2: the ground side is refined within this of the horizon
LIMB_REFINE_DEG = 0.25                 # G2: ... to edges of at most this (unit sphere)
LIMB_LINE_STEP_DEG = 0.1               # G3: the outline densified to this
_LIMB_TOL_DEG = 0.1                    # a point of a cut lies on the horizon within this (the cut is linear between
                                       # the raster's points and the refinement puts its points back on the sphere:
                                       # up to ~0.05 deg off along a small horizon; 1e-3 left walls down to the ship)


def limb_function(sky: Optional[dict], sec: dict):
    """(margin_of(d) -> deg above the horizon of the nearest surface body (min over the bodies; > 0: sky),
    limb_range_of(d) -> the useful range at the horizon of the body there (km: its horizon distance - guard, within
    the required range)) for the mission sky in the ship frame; None without a surface."""
    from . import cost_estimate as ce
    from . import optimizer as mo
    surf = ce._sky_surfaces(sky)
    r_req0 = float(sec.get("range_km", 0.0) or 0.0)
    # only the bodies whose surface the volume can reach (user, 11 November 2026: the Earth in the axis of a 2000 km
    # volume seen from the Moon cut it along its horizon for nothing, and that cut drew a cone down to the ship)
    surf = [s_ for s_ in surf if s_.dist_km - s_.radius_km < r_req0]
    if not surf:
        return None
    cs = np.array([s_.c for s_ in surf], float)
    rad = np.degrees(np.arcsin(np.clip([s_.radius_km / s_.dist_km for s_ in surf], 0.0, 1.0)))
    d_h = np.sqrt(np.maximum([s_.dist_km ** 2 - s_.radius_km ** 2 for s_ in surf], 0.0))
    r_req = float(sec.get("range_km", 0.0))
    guard = mo.surface_guard_km(sec)

    def margins(d):
        d = np.asarray(d, float).reshape(-1, 3)
        return np.degrees(np.arccos(np.clip(d @ cs.T, -1.0, 1.0))) - rad[None, :]

    def margin_of(d):
        return margins(d).min(axis=1)

    def limb_range_of(d):
        j = np.argmin(margins(d), axis=1)
        return np.clip(d_h[j] - guard, 0.0, r_req)
    return margin_of, limb_range_of


def _unit(poly: pv.PolyData) -> np.ndarray:
    return poly.points / np.linalg.norm(poly.points, axis=1)[:, None]


def _refine_near_limb(poly: pv.PolyData, margin_of) -> pv.PolyData:
    """G2: the cells within LIMB_BAND_DEG of the horizon subdivided to LIMB_REFINE_DEG, back on the unit sphere."""
    if not poly.n_points:
        return poly
    tri = poly.triangulate().clean(tolerance=0.0)
    pts = _unit(tri)
    cells = tri.faces.reshape(-1, 4)[:, 1:].copy()
    # conforming (user, 11 November 2026): refining the band's cells alone left hanging points where they met the
    # coarse ones, a crack along the band's edge whose walls drew a cone down to the ship; refining the whole piece
    # cost x5 triangles. The band's cells are split in 4, their neighbours in 2 or 3 on the edges split (no crack).
    lim = math.radians(LIMB_REFINE_DEG)
    for _ in range(6):
        near = margin_of(pts) > -LIMB_BAND_DEG
        a, b, c = pts[cells[:, 0]], pts[cells[:, 1]], pts[cells[:, 2]]
        longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                          np.linalg.norm(a - c, axis=1)], axis=0)
        mark = near[cells].any(axis=1) & (longest > lim)
        if not mark.any():
            break
        pts, cells = _refine_pass(pts, cells, mark)
    faces = np.column_stack([np.full(len(cells), 3), cells]).astype(np.int64).ravel()
    return pv.PolyData(pts, faces)


def _refine_pass(pts: np.ndarray, cells: np.ndarray, mark: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """One conforming refinement of a unit-sphere triangle mesh: every edge of a marked triangle split at its middle
    (on the sphere); a triangle with 3 edges split -> 4, 2 -> 3, 1 -> 2 (the same middle point on both sides)."""
    e = np.sort(np.stack([cells[:, [0, 1]], cells[:, [1, 2]], cells[:, [2, 0]]], axis=1), axis=2)
    uniq, inv = np.unique(e.reshape(-1, 2), axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    split = np.zeros(len(uniq), bool)
    split[inv[mark].ravel()] = True
    ids = np.flatnonzero(split)
    mid_of = np.full(len(uniq), -1, np.int64)
    mid_of[ids] = len(pts) + np.arange(len(ids))
    mid = pts[uniq[ids, 0]] + pts[uniq[ids, 1]]
    pts = np.vstack([pts, mid / np.linalg.norm(mid, axis=1)[:, None]])
    m = mid_of[inv]                                            # (n, 3): middle of edges 01, 12, 20, or -1
    cnt = (m >= 0).sum(axis=1)
    out = [cells[cnt == 0]]
    sel = cnt == 3
    if sel.any():
        v, q = cells[sel], m[sel]
        out += [np.column_stack([v[:, 0], q[:, 0], q[:, 2]]), np.column_stack([q[:, 0], v[:, 1], q[:, 1]]),
                np.column_stack([q[:, 2], q[:, 1], v[:, 2]]), np.column_stack([q[:, 0], q[:, 1], q[:, 2]])]
    for n_split in (1, 2):
        sel = cnt == n_split
        if not sel.any():
            continue
        v, q = cells[sel], m[sel]
        # rotate: 1 split -> the split edge first (01); 2 split -> the unsplit edge last (20)
        k = np.argmax(q >= 0, axis=1) if n_split == 1 else (np.argmin(q >= 0, axis=1) + 1) % 3
        r = (k[:, None] + np.arange(3)[None, :]) % 3
        v, q = np.take_along_axis(v, r, axis=1), np.take_along_axis(q, r, axis=1)
        if n_split == 1:
            out += [np.column_stack([v[:, 0], q[:, 0], v[:, 2]]), np.column_stack([q[:, 0], v[:, 1], v[:, 2]])]
        else:
            out += [np.column_stack([q[:, 0], v[:, 1], q[:, 1]]), np.column_stack([v[:, 0], q[:, 0], q[:, 1]]),
                    np.column_stack([v[:, 0], q[:, 1], v[:, 2]])]
    return pts, np.vstack(out)


def split_at_limb(poly: pv.PolyData, limb) -> List[Tuple[str, pv.PolyData]]:
    """A unit-sphere mesh cut along the horizon: [("sky", ...), ("ground", ... refined near the horizon)]; the whole
    mesh as ("sky", ...) without a surface."""
    if limb is None or not poly.n_points:
        return [("sky", poly)]
    margin_of = limb[0]
    m = margin_of(_unit(poly))
    out = []
    up = _cut(poly, m)
    if up.n_points:
        out.append(("sky", up))
    down = _cut(poly, -m)
    if down.n_points:
        out.append(("ground", _refine_near_limb(down, margin_of)))
    return out


def _on_limb(poly: pv.PolyData, margin_of) -> np.ndarray:
    return np.abs(margin_of(_unit(poly))) <= _LIMB_TOL_DEG


def _boundary_segments(cap: pv.PolyData) -> Tuple[np.ndarray, np.ndarray]:
    edges = cap.extract_feature_edges(boundary_edges=True, feature_edges=False, manifold_edges=False,
                                      non_manifold_edges=False)
    if not edges.n_points:
        return np.zeros((0, 3)), np.zeros((0, 2), np.int64)
    return np.asarray(edges.points), edges.lines.reshape(-1, 3)[:, 1:]


def _walls_except_limb(cap_unit: pv.PolyData, placed: pv.PolyData, margin_of, skip_of=None) -> pv.PolyData:
    """Walls from the cap's free edges down to the radar, but for the edges on the horizon (G2: joined by the tangent
    cone instead) and those where skip_of(unit directions) is True (the played relief's boundary). cap_unit: the cap
    on the unit sphere (its directions), placed: the same cap at range."""
    pts, seg = _boundary_segments(placed)
    if not len(seg):
        return pv.PolyData()
    if margin_of is not None:
        on = np.abs(margin_of(pts / np.linalg.norm(pts, axis=1)[:, None])) <= _LIMB_TOL_DEG
        seg = seg[~(on[seg[:, 0]] & on[seg[:, 1]])]
    if skip_of is not None and len(seg):
        sk = np.asarray(skip_of(pts / np.linalg.norm(pts, axis=1)[:, None]), bool)
        seg = seg[~(sk[seg[:, 0]] & sk[seg[:, 1]])]
    if not len(seg):
        return pv.PolyData()
    o = len(pts)
    tris = np.hstack([np.full((len(seg), 1), 3), np.full((len(seg), 1), o), seg]).astype(np.int64).ravel()
    return pv.PolyData(np.vstack([pts, [[0.0, 0.0, 0.0]]]), tris)


def _limb_wall(sky_placed: pv.PolyData, limb) -> pv.PolyData:
    """G2: the tangent cone between the horizon distance and the range the sky side reaches, along the sky side's
    edges on the horizon."""
    if limb is None or not sky_placed.n_points:
        return pv.PolyData()
    margin_of, limb_range_of = limb
    pts, seg = _boundary_segments(sky_placed)
    if not len(seg):
        return pv.PolyData()
    u = pts / np.linalg.norm(pts, axis=1)[:, None]
    on = np.abs(margin_of(u)) <= _LIMB_TOL_DEG
    seg = seg[on[seg[:, 0]] & on[seg[:, 1]]]
    if not len(seg):
        return pv.PolyData()
    low = u * limb_range_of(u)[:, None]
    n = len(pts)
    a, b = seg[:, 0], seg[:, 1]
    tris = np.vstack([np.column_stack([np.full(len(seg), 3), a, b, n + b]),
                      np.column_stack([np.full(len(seg), 3), a, n + b, n + a])]).astype(np.int64).ravel()
    return pv.PolyData(np.vstack([pts, low]), tris)


def place_outline(outline: pv.PolyData, r_of, limb) -> pv.PolyData:
    """G3: an outline on the unit sphere placed at r_of(directions) (km), densified to LIMB_LINE_STEP_DEG and, where it
    crosses a horizon, stepped there (the ground side at the horizon distance, the sky side at its own range) instead
    of a segment from a ground point to a far point."""
    if not outline.n_points:
        return outline
    if limb is None:
        out = outline.copy()
        out.points = _unit(out) * np.asarray(r_of(_unit(out)), float)[:, None]
        return out
    margin_of, limb_range_of = limb
    u0 = _unit(outline)
    seg = outline.lines.reshape(-1, 3)[:, 1:]
    pts, lines = [], []
    for a, b in seg:
        pa, pb = u0[a], u0[b]
        ang = math.degrees(math.acos(float(np.clip(pa @ pb, -1.0, 1.0))))
        n = max(1, int(math.ceil(ang / LIMB_LINE_STEP_DEG)))
        t = np.linspace(0.0, 1.0, n + 1)
        q = pa[None, :] * (1.0 - t)[:, None] + pb[None, :] * t[:, None]
        q = q / np.linalg.norm(q, axis=1)[:, None]
        m = margin_of(q)
        r = np.asarray(r_of(q), float)
        for i in range(n):
            p0, p1 = q[i] * r[i], q[i + 1] * r[i + 1]
            if (m[i] > 0.0) != (m[i + 1] > 0.0) and m[i] != m[i + 1]:
                f = m[i] / (m[i] - m[i + 1])
                c = q[i] * (1.0 - f) + q[i + 1] * f
                c = c / np.linalg.norm(c)
                r_lo = float(limb_range_of(c[None, :])[0])
                r_hi = float(r[i] if m[i] > 0 else r[i + 1])       # (the sky side's range is continuous there)
                lo, hi = c * r_lo, c * max(r_hi, r_lo)
                c0, c1 = (hi, lo) if m[i] > 0 else (lo, hi)
                chain = [p0, c0, c1, p1]
            else:
                chain = [p0, p1]
            k = len(pts)
            pts.extend(chain)
            lines.extend([[2, k + j, k + j + 1] for j in range(len(chain) - 1)])
    return pv.PolyData(np.asarray(pts, float), lines=np.asarray(lines, np.int64).ravel())


def held_ratio_function(grid: SphereGrid, sec: dict, eps_min: float, eps_max: float, sky: Optional[dict] = None,
                        alloc: Optional[dict] = None, sid: Optional[str] = None):
    """The range gauge's ratio_of(d) for a watched volume, definition (c) (pass R2, user 10 November 2026): the share
    of the required range held, rho, from the final allocation rule over all the volumes (cost_estimate.
    live_allocation), the same in every direction a face scans (A3), 0 in the coverage hole, capped by the useful range
    (the ground). None without an allocation."""
    from . import optimizer as mo
    if alloc is None:
        return None
    rec = alloc.get("by_sector", {}).get(str(sid if sid is not None else sec.get("id")))
    rho = float(rec["rho"]) if rec else 0.0
    useful_of = useful_ratio_function(sec, sky)
    rc = (alloc.get("reach") or {}).get(str(sid if sid is not None else sec.get("id")))
    code = ((alloc.get("relief") or {}).get(str(sid if sid is not None else sec.get("id"))) or {}).get("code")

    def ratio_of(d, face=None, ignore_ground=False):
        d = np.asarray(d, float).reshape(-1, 3)
        if face is None:
            owner, _w, _u, _v = mo.face_of(d[:, 0], d[:, 1], d[:, 2])
            eps = np.degrees(np.arcsin(np.clip(d[:, 2], -1.0, 1.0)))
            ok = (owner >= 0) & (eps >= eps_min) & (eps <= eps_max)
        else:                               # (directions known to be face's)
            ok = np.ones(len(d), bool)
        held = np.where(ok, rho, 0.0)
        rel = None
        if code:                              # (the relief, user 11 November 2026: where another volume meets the need,
            dphi = np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 360.0   # it is held: drawn green, not red)
            deps = np.degrees(np.arcsin(np.clip(d[:, 2], -1.0, 1.0)))
            rel = mo.relief_cells(code, dphi, deps, grow=1)   # (its smooth outline in the edge cells: no red rim)
            held = np.where(rel, 1.0, held)
        if not ignore_ground:                 # (ignore_ground: a point of the sky side ON the horizon, its tangent ray
            held = np.minimum(held, np.clip(np.asarray(useful_of(d), float), 0.0, 1.0))   # grazing the ground)
        if rc and rc.get("r_want_km"):                 # (the range reachable at full pulse at the face's scan angle)
            _o, w_, _u, _v = mo.face_of(d[:, 0], d[:, 1], d[:, 2])
            th = np.degrees(np.arccos(np.clip(w_, 0.0, 1.0)))
            cap = np.interp(np.clip(th, rc["th"][0], rc["th"][-1]), rc["th"], rc["r_reach_km"]) / rc["r_want_km"]
            capped = np.minimum(held, np.clip(cap, 0.0, 1.0))
            held = capped if rel is None else np.where(rel, held, capped)   # (its own reach: not the coverer's)
        return held
    ratio_of.eps_window = (eps_min, eps_max)
    ratio_of.useful_of = useful_of
    ratio_of.limb = limb_function(sky, sec)                   # G2: the shells cut along the horizon
    return ratio_of


def useful_ratio_function(sec: dict, sky: Optional[dict] = None):
    """useful_of(d): the share of the required range a direction needs (1; d - guard over the range where it meets a
    surface, 0 where that is <= 0: the ground rule, step 3.1 P4 Q9)."""
    from . import cost_estimate as ce
    surf = ce._sky_surfaces(sky)
    r_req = float(sec.get("range_km", 0.0)) or 1.0
    if not surf:
        return lambda d: np.ones(len(np.asarray(d).reshape(-1, 3)))
    return lambda d: np.clip(ce.useful_range_km(sec, surf, d) / r_req, 0.0, 1.0)


def build_range_gauge(grid: SphereGrid, sec: dict, ratio_of, radius: float,
                      normalise: bool = False, played: bool = False) -> Tuple[pv.PolyData, pv.PolyData]:
    """Range gauge (user, 10 November 2026) of a watched volume: (green: the shell at the range held in each
    direction, radius x ratio_of(directions); red: the part of the requested shell at radius where the range held
    falls short, coverage hole included). ratio_of(d (n, 3)) -> the held share of the required range.
    played ("show effectively played", user 12 November 2026): only what is transmitted, no red: the volume held,
    holed where nothing scans (the coverage hole, a shortfall)."""
    region = _region(grid, sec, 1.0)
    if not region.n_points:
        return pv.PolyData(), pv.PolyData()
    d = region.points / np.linalg.norm(region.points, axis=1)[:, None]
    phi, eps = _dirs_angles(d)
    req = _cut(region, sector_margin(sec, phi, eps))              # the whole requested volume, smooth edges
    if not req.n_points:
        return pv.PolyData(), pv.PolyData()
    useful_of = getattr(ratio_of, "useful_of", None) or (lambda d_: np.ones(len(d_)))
    limb = getattr(ratio_of, "limb", None)
    dr = req.points / np.linalg.norm(req.points, axis=1)[:, None]
    rho = np.asarray(ratio_of(dr), float)
    u_req = np.asarray(useful_of(dr), float)
    # green: the covered part (cut smoothly by the faces' margins, as the executed volumes; the ratio jumps to 0 at a
    # coverage edge and a cut on it drew slivers down to the ship), at the range held there
    eps_win = getattr(ratio_of, "eps_window", None) or (-90.0, 90.0)
    # ONE piece (user, 11 November 2026: dark stripes on a cone): the requested volume cut by the faces' coverage, not
    # face by face; the face pieces met with small notches along the volume's outline, whose walls folded on themselves
    # (a double layer from the ship to the rim at each seam); each point takes the face that scans it best
    dq = req.points / np.linalg.norm(req.points, axis=1)[:, None]
    covered = _cut(req, _coverage_margin(dq, eps_win))
    sides: Dict[str, List[pv.PolyData]] = {}
    if covered.n_points:
        dg = covered.points / np.linalg.norm(covered.points, axis=1)[:, None]
        piece = _cut(covered, np.asarray(useful_of(dg), float) - 0.01)   # (no volume where the ground is within guard)
        if piece.n_points:
            piece.cell_data.clear()
            piece.point_data.clear()
            for side, sub in (split_at_limb(piece, limb) if not normalise else [("sky", piece)]):
                sides.setdefault(side, []).append(sub.triangulate())
    green = pv.PolyData()
    for side, subs in sides.items():
        m = subs[0]
        for s_ in subs[1:]:
            m = m.append_polydata(s_)
        m = pv.PolyData(np.asarray(m.points, float), m.faces).clean(tolerance=1e-7, absolute=False)
        if not m.n_cells or m.n_points < 3:
            continue
        m = pv.PolyData(np.asarray(m.points, float), m.faces)
        dg = m.points / np.linalg.norm(m.points, axis=1)[:, None]
        best = np.argmax([face_margin(k, dg) for k in range(4)], axis=0)    # (the face that scans it best)
        rg = np.zeros(m.n_points)
        edge = (np.abs(limb[0](dg)) <= _LIMB_TOL_DEG) if (side == "sky" and limb is not None) else None
        for k in range(4):
            pk = np.flatnonzero(best == k)
            if not len(pk):
                continue
            v = np.clip(np.asarray(ratio_of(dg[pk], face=k), float), 0.0, 1.0)
            if edge is not None and edge[pk].any():           # (the sky side's edge on the horizon: the sky's range,
                e = edge[pk]                                   # not the ground its tangent ray grazes: it drooped)
                try:
                    v[e] = np.clip(np.asarray(ratio_of(dg[pk][e], face=k, ignore_ground=True), float), 0.0, 1.0)
                except TypeError:                              # (a ratio_of without the option)
                    pass
            rg[pk] = v
        rg = np.clip(rg, 0.0, 1.0)
        if side == "ground" and limb is not None:              # (a point ON the horizon: its tangent ray misses the
            rg = np.minimum(rg, _limb_share(limb, dg, sec))   # ground numerically, it took the full range: needles)
        if normalise:                                          # (sphere: the range held over the range needed)
            rg = np.clip(rg / np.maximum(np.asarray(useful_of(dg), float), 1e-6), 0.0, 1.0)
        m.cell_data.clear()
        m.point_data["gauge_r"] = rg
        m = m.fill_holes(math.radians(2.5 * float(getattr(grid, "res_deg", 1.0))))   # (pin holes: at most a cell or so)
        r_pts = np.asarray(m.point_data["gauge_r"], float)
        m.points = m.points / np.linalg.norm(m.points, axis=1)[:, None] * (r_pts * radius)[:, None]
        m.point_data.clear()
        green = m if not green.n_points else green.append_polydata(m)
    red = _cut(req, 0.999 * u_req - rho) if not played else pv.PolyData()   # short of the range needed (the ground:
    if red.n_points and not normalise:                                         # not needed); played: not drawn
        reds = pv.PolyData()
        for side, sub in split_at_limb(red, limb):            # G2: cut along the horizon
            dd = sub.points / np.linalg.norm(sub.points, axis=1)[:, None]
            u_ = np.asarray(useful_of(dd), float)
            if side == "ground" and limb is not None:          # (as the green: never beyond the horizon)
                u_ = np.minimum(u_, _limb_share(limb, dd, sec))
            elif limb is not None:                             # (the sky side's edge on the horizon: the sky's range)
                u_ = np.where(np.abs(limb[0](dd)) <= _LIMB_TOL_DEG, 1.0, u_)
            sub.points = dd * u_[:, None] * radius
            reds = sub if not reds.n_points else reds.append_polydata(sub)
        red = reds
    elif red.n_points:
        red.points = red.points / np.linalg.norm(red.points, axis=1)[:, None] * radius
    if not normalise:                                          # the volume's limits (user, 11 November 2026)
        # along the faces' coverage hole the walls are the missing part's (red, down to the ship), not the held
        # part's (user, 11 November 2026: the hole's side was drawn green)
        def rim(d_):
            return np.abs(_coverage_margin(d_, eps_win)) <= _RIM_TOL_DEG
        on_relief = relief_edge_function(sec)                  # (played: no walls along the relief's boundary)

        def skip_green(d_):
            s_ = rim(d_)
            return s_ if on_relief is None else s_ | on_relief(d_)
        green = _gauge_walls(green, limb, lambda d_: np.zeros(len(d_)), skip_of=skip_green)
        red = _gauge_walls(red, limb, lambda d_: np.where(rim(d_), 0.0, np.clip(np.asarray(ratio_of(d_), float),
                                                                                 0.0, 1.0) * radius),
                           skip_of=on_relief)
    return green, red


def _limb_share(limb, d: np.ndarray, sec: dict) -> np.ndarray:
    """The horizon's distance in the directions d, as a share of the volume's range (the ground side never beyond)."""
    r_req = float(sec.get("range_km", 0.0) or 0.0)
    if r_req <= 0.0:
        return np.ones(len(d))
    return np.clip(np.asarray(limb[1](d), float) / r_req, 0.0, 1.0)


_RIM_TOL_DEG = 0.25                    # a point of a cut lies on the coverage hole's edge within this (0.05 left
                                       # green walls between the red ones along the dead zone: stripes)


def scan_margin(k: int, d: np.ndarray) -> np.ndarray:
    """Signed margin (deg, > 0 inside) of ship-frame unit directions to face k's scan-limit cone, in front of it --
    what the face CAN scan (face_margin adds the +/-45 deg planes that share the sphere between the faces)."""
    sin_lim = math.sin(math.radians(rm.SECTOR_RADIUS_DEG))
    n, eu, ev = mo.face_frame(FACE_PHI_DEG[k])
    w, u, v = d @ np.asarray(n), d @ np.asarray(eu), d @ np.asarray(ev)
    return np.degrees(np.minimum(w, sin_lim - np.sqrt(u * u + v * v)))


def _coverage_margin(d: np.ndarray, eps_win) -> np.ndarray:
    """Signed margin (deg, > 0 covered) of unit directions to the faces' coverage: some face's scan-limit cone, the
    eps window (user, 12 November 2026: with the faces' shared zones, a seam between two faces had a margin 0 and was
    taken for the dead zone's edge: walls left out there, dark stripes on a cone, red lines down to the ship)."""
    d = np.asarray(d, float).reshape(-1, 3)
    eps = np.degrees(np.arcsin(np.clip(d[:, 2], -1.0, 1.0)))
    m_win = np.minimum(eps - eps_win[0], eps_win[1] - eps)
    return np.max([np.minimum(scan_margin(k, d), m_win) for k in range(4)], axis=0)


def _gauge_walls(shell: pv.PolyData, limb, floor_of, skip_of=None) -> pv.PolyData:
    """The range gauge as volumes, not only their outer face (user, 11 November 2026): the shell (all faces merged,
    so that no wall stands at a seam between two faces) closed by walls from its free edges down to floor_of(d) (km;
    0: the radar, for the part held; the range held, for the part short of it), and along the horizon by the tangent
    cone between the shell and the horizon distance (no lower than the floor)."""
    if not shell.n_points:
        return shell
    span = float(np.linalg.norm(shell.points, axis=1).max())
    sh = shell.clean(tolerance=1e-7, absolute=False)
    edges = sh.extract_feature_edges(boundary_edges=True, feature_edges=False, manifold_edges=False,
                                     non_manifold_edges=False)
    if not edges.n_points:
        return sh
    pts = np.asarray(edges.points, float)
    seg = edges.lines.reshape(-1, 3)[:, 1:]
    r = np.linalg.norm(pts, axis=1)
    u = pts / np.maximum(r, 1e-12)[:, None]
    low = np.minimum(np.asarray(floor_of(u), float), r)
    if skip_of is not None and len(seg):                      # (edges whose walls another shell draws)
        sk = np.asarray(skip_of(u), bool)
        seg = seg[~(sk[seg[:, 0]] & sk[seg[:, 1]])]
    if limb is not None:                                       # an edge on the horizon: the cone down to its distance
        margin_of, limb_range_of = limb
        on = np.abs(margin_of(u)) <= _LIMB_TOL_DEG
        both = on[seg[:, 0]] & on[seg[:, 1]]
        lr = np.minimum(np.asarray(limb_range_of(u), float), r)
        low = np.where(on, np.maximum(lr, low), low)
        keep = ~(both & (np.abs(r[seg[:, 0]] - low[seg[:, 0]]) < 1e-6 * span)
                 & (np.abs(r[seg[:, 1]] - low[seg[:, 1]]) < 1e-6 * span))   # (no strip of zero height)
        seg = seg[keep]
    n = len(pts)
    a, b = seg[:, 0], seg[:, 1]
    tris = np.vstack([np.column_stack([np.full(len(seg), 3), a, b, n + b]),
                      np.column_stack([np.full(len(seg), 3), a, n + b, n + a])]).astype(np.int64).ravel()
    walls = pv.PolyData(np.vstack([pts, u * low[:, None]]), tris)
    return sh.append_polydata(walls)


def lateral_edges(outline_unit: pv.PolyData, r_of, turn_deg: float = 30.0) -> pv.PolyData:
    """The volume's lateral edges (user, 11 November 2026): lines from the radar to the corners of its outline (the
    points where the outline turns by more than turn_deg), each at the range its direction needs (r_of, km)."""
    if not outline_unit.n_points:
        return pv.PolyData()
    o = outline_unit.clean(tolerance=1e-9, absolute=False)
    pts = o.points / np.linalg.norm(o.points, axis=1)[:, None]
    seg = o.lines.reshape(-1, 3)[:, 1:]
    nb: Dict[int, List[int]] = {}
    for a, b in seg:
        nb.setdefault(int(a), []).append(int(b))
        nb.setdefault(int(b), []).append(int(a))
    corners = []
    for i, js in nb.items():
        if len(js) != 2:
            continue
        v1, v2 = pts[js[0]] - pts[i], pts[js[1]] - pts[i]
        n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
        if n1 <= 0 or n2 <= 0:
            continue
        ang = math.degrees(math.acos(float(np.clip(-(v1 @ v2) / (n1 * n2), -1.0, 1.0))))
        if ang >= turn_deg:
            corners.append(i)
    if not corners:
        return pv.PolyData()
    c = pts[corners]
    tips = c * np.asarray(r_of(c), float)[:, None]
    lines = np.column_stack([np.full(len(c), 2), np.zeros(len(c), np.int64), np.arange(1, len(c) + 1)]).ravel()
    return pv.PolyData(np.vstack([[[0.0, 0.0, 0.0]], tips]), lines=lines.astype(np.int64))


def build_gauge_outline_km(grid: SphereGrid, sec: dict, sky: Optional[dict] = None) -> pv.PolyData:
    """The range gauge's edges: the whole volume's outline at the range it needs (following the ground, G3) and its
    lateral edges down to the radar (user, 11 November 2026)."""
    _ex, outline, _hole = build_sector_split_smooth(grid, sec, radius=1.0)
    if not outline.n_points:
        return pv.PolyData()
    r = float(sec["range_km"])
    useful_of = useful_ratio_function(sec, sky)

    def r_of(d):
        return np.asarray(useful_of(d), float) * r
    out = place_outline(outline, r_of, limb_function(sky, sec))
    lat = lateral_edges(outline, r_of)
    return out if not lat.n_points else out.append_polydata(lat)


def _walls(cap: pv.PolyData) -> pv.PolyData:
    """Triangles from the cap's free edges down to the origin (the radar)."""
    edges = cap.extract_feature_edges(boundary_edges=True, feature_edges=False, manifold_edges=False,
                                      non_manifold_edges=False)
    if not edges.n_points:
        return pv.PolyData()
    seg = edges.lines.reshape(-1, 3)[:, 1:]
    o = edges.n_points
    tris = np.hstack([np.full((len(seg), 1), 3), np.full((len(seg), 1), o), seg]).astype(np.int64).ravel()
    return pv.PolyData(np.vstack([edges.points, [[0.0, 0.0, 0.0]]]), tris)


def build_sector_volume_km_smooth(grid: SphereGrid, sec: dict, eps_min: float = -90.0, eps_max: float = 90.0,
                                  face: Optional[int] = None) -> pv.PolyData:
    """build_sector_volume_km with smooth edges: the cap of the part face scans at the required range, closed by
    walls down to the radar (km)."""
    executed, _out, _hole = build_sector_split_smooth(grid, sec, eps_min, eps_max, radius=1.0)
    if face is not None and executed.n_points:
        executed = executed.extract_cells(np.flatnonzero(executed.cell_data["face"] == face)).extract_surface(algorithm="dataset_surface")
    if not executed.n_points:
        return pv.PolyData()
    cap = executed.copy()
    cap.points = cap.points * float(sec["range_km"])
    walls = _walls(cap)
    return cap if not walls.n_points else cap.append_polydata(walls)


def build_sector_volumes_km_smooth(grid: SphereGrid, sec: dict, eps_min: float = -90.0, eps_max: float = 90.0,
                                   sky: Optional[dict] = None) -> Tuple[List[Tuple[int, pv.PolyData]], pv.PolyData]:
    """The executed volumes of a watched volume at its required range, one per face, each closed by walls down to the
    radar, and the outline of the whole watched volume (km); one cut for all."""
    executed, outline, _hole = build_sector_split_smooth(grid, sec, eps_min, eps_max, radius=1.0)
    r = float(sec["range_km"])
    useful_of = useful_ratio_function(sec, sky)
    limb = limb_function(sky, sec)                         # G2, G3: the horizon of the ground bodies

    def r_of(d):
        return np.asarray(useful_of(d), float) * r
    on_relief = relief_edge_function(sec)                  # (played: no walls along the relief's boundary)
    parts = []
    for k in range(4):
        if not executed.n_points:
            break
        ids = np.flatnonzero(executed.cell_data["face"] == k)
        if not len(ids):
            continue
        cap0 = executed.extract_cells(ids).extract_surface(algorithm="dataset_surface")
        cap0.cell_data.clear()
        vol = pv.PolyData()
        for side, sub in split_at_limb(cap0, limb):         # each side at the range its directions need
            cap = sub.copy()
            cap.points = _unit(sub) * r_of(_unit(sub))[:, None]
            walls = _walls_except_limb(sub, cap, None if limb is None else limb[0], on_relief)
            piece = cap if not walls.n_points else cap.append_polydata(walls)
            if side == "sky":
                cone = _limb_wall(cap, limb)
                piece = piece if not cone.n_points else piece.append_polydata(cone)
            vol = piece if not vol.n_points else vol.append_polydata(piece)
        if vol.n_points:
            parts.append((k, vol))
    if outline.n_points:
        outline = place_outline(outline, r_of, limb)
    return parts, outline


def build_sector_outline_km_smooth(grid: SphereGrid, sec: dict) -> pv.PolyData:
    """The smooth outline of the whole watched volume at its required range (km)."""
    _ex, outline, _hole = build_sector_split_smooth(grid, sec, radius=1.0)
    if not outline.n_points:
        return pv.PolyData()
    outline.points = outline.points * float(sec["range_km"])
    return outline


def build_sector_volume_km(grid: SphereGrid, sec: dict, eps_min: float = -90.0,
                           eps_max: float = 90.0, face: Optional[int] = None) -> Tuple[pv.PolyData, pv.PolyData]:
    """True-scale watch volume (1 scene unit = 1 km): the sector's cap at its required range sec["range_km"],
    closed by walls down to the radar at the origin, clipped to the scanned zone (face: only the part that face
    scans, its executed volume, E3). Returns (volume surface, outline of the cap)."""
    r = float(sec["range_km"])
    vgrid = grid.vertex_grid(1.0)
    mask = grid.sector_mask(sec) & grid.scanned(eps_min, eps_max)
    if face is not None:
        mask = mask & (grid.owner == face)
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


def build_sector_outline_km(grid: SphereGrid, sec: dict) -> pv.PolyData:
    """The outline of the WHOLE watched volume at its required range (km), coverage holes included (E3)."""
    full = grid.sector_mask(sec)
    if not full.any():
        return pv.PolyData()
    out = boundary_for_mask(grid.vertex_grid(1.0), full)
    out.points = out.points * float(sec["range_km"])
    return out


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
         "label_pos": (0, 0, float(env.MODEL_VERTICES[:, 2].min()) * _M - 0.08), "no_line": True},   # aft end
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


def _flat(mesh: pv.PolyData) -> pv.PolyData:
    """Per-face normals, so smooth shading keeps flat panels flat."""
    return mesh.compute_normals(split_vertices=True, feature_angle=30.0)


def build_ship() -> dict:
    """Ship from the 3D model of the parameter file (ship.model_path, placed in the ship frame by
    core/environment.py: the same triangles as the masking and the structure noise), and the four radar apertures on
    the model's radar panels (one per face, envelope of the real aperture, rimmed in the face colour)."""
    faces = np.c_[np.full(len(env.MODEL_FACES), 3), env.MODEL_FACES].ravel()
    hull = [{"mesh": _flat(pv.PolyData(env.MODEL_VERTICES * _M, faces)), "color": SHIP_COLORS["hull"]}]
    arrays = []
    for k, pk in enumerate(FACE_PHI_DEG):
        n = face_normal(pk)
        centre = np.asarray(array_centre(pk))
        corners = env.panel_corners_m(k) * _M             # the model's panel front face, same orientation
        # on the panel: a slab of its own outline in the face colour (F1..F4 rim), a hull-coloured one inside
        arrays.append({"k": k, "mesh": _panel_slab(corners, centre, n, 0.97, 0.01, 0.11), "color": FACE_COLORS[k]})
        hull.append({"mesh": _panel_slab(corners, centre, n, 0.88, 0.11, 0.16), "color": SHIP_COLORS["hull"]})
    return {"hull": hull, "arrays": arrays}


def _panel_slab(corners: np.ndarray, centre: np.ndarray, n: np.ndarray, scale: float, z0_m: float,
                z1_m: float) -> pv.PolyData:
    """Prism of the panel outline shrunk by scale about its centre, from z0_m to z1_m in front of the face."""
    base = centre + (corners - centre) * scale + n * z0_m * _M
    poly = pv.PolyData(base, faces=np.r_[len(base), np.arange(len(base))])
    return _flat(poly.extrude(n * (z1_m - z0_m) * _M, capping=True))


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
