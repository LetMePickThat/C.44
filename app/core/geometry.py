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


def sector_margin(sec: dict, phi: np.ndarray, eps: np.ndarray) -> np.ndarray:
    """Signed angular margin (deg, > 0 inside) of ship-frame directions to a sector's edge (sec resolved in the
    ship frame, its frame_m applied)."""
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
    return executed, outline, hole


GAUGE_GREEN, GAUGE_RED = "#3cb371", "#d6453d"     # range gauge: the range held, the shortfall (D)


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

    def ratio_of(d, face=None):
        d = np.asarray(d, float).reshape(-1, 3)
        if face is None:
            owner, _w, _u, _v = mo.face_of(d[:, 0], d[:, 1], d[:, 2])
            eps = np.degrees(np.arcsin(np.clip(d[:, 2], -1.0, 1.0)))
            ok = (owner >= 0) & (eps >= eps_min) & (eps <= eps_max)
        else:                               # (directions known to be face's)
            ok = np.ones(len(d), bool)
        return np.minimum(np.where(ok, rho, 0.0), np.clip(np.asarray(useful_of(d), float), 0.0, 1.0))
    ratio_of.eps_window = (eps_min, eps_max)
    ratio_of.useful_of = useful_of
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
                      normalise: bool = False) -> Tuple[pv.PolyData, pv.PolyData]:
    """Range gauge (user, 10 November 2026) of a watched volume: (green: the shell at the range held in each
    direction, radius x ratio_of(directions); red: the part of the requested shell at radius where the range held
    falls short, coverage hole included). ratio_of(d (n, 3)) -> the held share of the required range."""
    region = _region(grid, sec, 1.0)
    if not region.n_points:
        return pv.PolyData(), pv.PolyData()
    d = region.points / np.linalg.norm(region.points, axis=1)[:, None]
    phi, eps = _dirs_angles(d)
    req = _cut(region, sector_margin(sec, phi, eps))              # the whole requested volume, smooth edges
    if not req.n_points:
        return pv.PolyData(), pv.PolyData()
    useful_of = getattr(ratio_of, "useful_of", None) or (lambda d_: np.ones(len(d_)))
    dr = req.points / np.linalg.norm(req.points, axis=1)[:, None]
    rho = np.asarray(ratio_of(dr), float)
    u_req = np.asarray(useful_of(dr), float)
    # green: the covered part (cut smoothly by the faces' margins, as the executed volumes; the ratio jumps to 0 at a
    # coverage edge and a cut on it drew slivers down to the ship), at the range held there
    eps_win = getattr(ratio_of, "eps_window", None) or (-90.0, 90.0)
    green, _outline, _hole = build_sector_split_smooth(grid, sec, eps_win[0], eps_win[1], radius=1.0)
    pieces = []
    for k in range(4):                                         # face by face: each point takes its own piece's face
        if not green.n_points:
            break
        ids = np.flatnonzero(green.cell_data["face"] == k)
        if not len(ids):
            continue
        piece = green.extract_cells(ids).extract_surface(algorithm="dataset_surface")
        dg = piece.points / np.linalg.norm(piece.points, axis=1)[:, None]
        piece = _cut(piece, np.asarray(useful_of(dg), float) - 0.01)   # (no volume where the ground is within guard)
        if not piece.n_points:
            continue
        dg = piece.points / np.linalg.norm(piece.points, axis=1)[:, None]
        rg = np.clip(np.asarray(ratio_of(dg, face=k), float), 0.0, 1.0)
        if normalise:                                          # (sphere: the range held over the range needed)
            rg = np.clip(rg / np.maximum(np.asarray(useful_of(dg), float), 1e-6), 0.0, 1.0)
        piece.points = dg * (rg * radius)[:, None]
        piece.cell_data.clear()
        pieces.append(piece)
    green = pv.PolyData()
    for piece in pieces:
        green = piece if not green.n_points else green.append_polydata(piece)
    red = _cut(req, 0.999 * u_req - rho)                       # short of the range needed (the ground: not needed)
    if red.n_points:
        dd = red.points / np.linalg.norm(red.points, axis=1)[:, None]
        red.points = dd * (radius if normalise else np.asarray(useful_of(dd), float)[:, None] * radius)
    return green, red


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

    def at_range(poly: pv.PolyData) -> None:              # each point at the range its direction needs (the ground)
        dd = poly.points / np.linalg.norm(poly.points, axis=1)[:, None]
        poly.points = dd * (np.asarray(useful_of(dd), float) * r)[:, None]
    parts = []
    for k in range(4):
        if not executed.n_points:
            break
        ids = np.flatnonzero(executed.cell_data["face"] == k)
        if not len(ids):
            continue
        cap = executed.extract_cells(ids).extract_surface(algorithm="dataset_surface")
        at_range(cap)
        walls = _walls(cap)
        parts.append((k, cap if not walls.n_points else cap.append_polydata(walls)))
    if outline.n_points:
        at_range(outline)
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
