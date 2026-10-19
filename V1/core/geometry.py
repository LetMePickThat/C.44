#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEOMETRY -- shared ship-frame geometry and PyVista mesh builders for the radar coverage 3D views
(mission-preparation and results tabs; the plotter itself is driven by gui/scene_view.py).

Single source of truth for the ship frame: everything here is derived from the SAME constants and
formulas as WF_RANGE_CALC_V6.0.10.py / core.optimizer (SECTOR_TILT_DEG, FACE_PHI, face-frame
projection, scan limit) so the 3D picture never drifts from the radar model.

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
from typing import Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv

# ----------------------------------------------------------------------------- radar model (shared)
# The SAME module object as the optimiser's, so a setting changed on the model (tilt, eps window...) applies
# to the 3D view, the fast estimate and the exact calculation alike.
from .optimizer import wf

FACE_PHI_DEG: Tuple[float, float, float, float] = (0.0, 90.0, 180.0, 270.0)
VIZ_FREQ_GHZ = 2.8   # reference frequency used for the viz-only scan limit (matches the optimiser)

# Ship-model cosmetic constants: face mount height on the hull and radial offset.
ZN = 0.17
RP = 0.078

# Palette (inherited from the volumes.html viewer).
COLORS: Dict[str, str] = {
    "F1": "#2f7fd0", "F2": "#e08a1e", "F3": "#2fa35a", "F4": "#d6453d",
    "void": "#8a95a3", "excl": "#b9854a", "ship": "#7b8794",
    "axis_x": "#d9a21b", "axis_y": "#9a5fd0", "axis_z": "#d6453d",
    "muted": "#93a1b2", "fg": "#18222d",
}
FACE_COLORS: Tuple[str, str, str, str] = (COLORS["F1"], COLORS["F2"], COLORS["F3"], COLORS["F4"])
FACE_NAMES: Tuple[str, str, str, str] = ("F1", "F2", "F3", "F4")

SECTOR_PALETTE: Tuple[str, ...] = (
    "#1f5fae", "#c0562a", "#1f8a52", "#b03a4a", "#7a4fb0", "#1a9c9c", "#9c7a1a", "#4a5fd0",
)


def sector_color(i: int) -> str:
    return SECTOR_PALETTE[i % len(SECTOR_PALETTE)]


# ----------------------------------------------------------------------------- face frame
def face_normal(phi_deg: float, tilt_deg: Optional[float] = None) -> np.ndarray:
    tilt_deg = wf.SECTOR_TILT_DEG if tilt_deg is None else tilt_deg
    p, t = math.radians(phi_deg), math.radians(tilt_deg)
    return np.array([math.cos(t) * math.cos(p), math.cos(t) * math.sin(p), math.sin(t)])


def face_frame(phi_deg: float, tilt_deg: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (n, e_u, e_v) of the face frame: d = u*e_u + v*e_v + w*n (same convention as
    core.optimizer.Sphere / WF_RANGE_CALC header)."""
    tilt_deg = wf.SECTOR_TILT_DEG if tilt_deg is None else tilt_deg
    p, t = math.radians(phi_deg), math.radians(tilt_deg)
    c, s = math.cos(p), math.sin(p)
    n = np.array([math.cos(t) * c, math.cos(t) * s, math.sin(t)])
    e_u = np.array([-s, c, 0.0])
    e_v = np.array([-math.sin(t) * c, -math.sin(t) * s, math.cos(t)])
    return n, e_u, e_v


def default_scan_limit_deg(f_ghz: float = VIZ_FREQ_GHZ) -> float:
    band = wf.band_for_frequency(f_ghz)
    return band.scan_limit_deg(f_ghz)


# ----------------------------------------------------------------------------- direction sphere raster
class SphereGrid:
    """Raster of the unit direction sphere (phi, eps), cell-centred, with per-cell face ownership
    (zone of minimal scan: boundaries = the az = +/-45 deg meridian planes, cut by the scan-limit cone).
    Same math as core.optimizer.Sphere, vectorised; resolution is tunable for interactive 3D views.
    """

    def __init__(self, res_deg: float = 1.0, tilt_deg: Optional[float] = None,
                 scan_limit_deg: Optional[float] = None):
        self.res_deg = res_deg
        self.tilt_deg = wf.SECTOR_TILT_DEG if tilt_deg is None else tilt_deg
        self.scan_limit_deg = default_scan_limit_deg() if scan_limit_deg is None else scan_limit_deg
        self.n_phi = max(4, int(round(360.0 / res_deg)))
        self.n_eps = max(2, int(round(180.0 / res_deg)))
        phi = (np.arange(self.n_phi) + 0.5) * (360.0 / self.n_phi)
        eps = -90.0 + (np.arange(self.n_eps) + 0.5) * (180.0 / self.n_eps)
        self.phi, self.eps = np.meshgrid(phi, eps, indexing="ij")  # shape (n_phi, n_eps), cell centres
        p, e = np.radians(self.phi), np.radians(self.eps)
        self.dx = np.cos(e) * np.cos(p)
        self.dy = np.cos(e) * np.sin(p)
        self.dz = np.sin(e)
        self.d_omega = math.radians(360.0 / self.n_phi) * math.radians(180.0 / self.n_eps) * np.cos(e)
        self._compute_ownership()

    def _compute_ownership(self) -> None:
        t = math.radians(self.tilt_deg)
        sin_lim = math.sin(math.radians(self.scan_limit_deg))
        self.owner = -np.ones(self.phi.shape, dtype=np.int8)
        best_w = np.full(self.phi.shape, -1.0)
        for k, pk in enumerate(FACE_PHI_DEG):
            n, e_u, e_v = face_frame(pk, self.tilt_deg)
            w = self.dx * n[0] + self.dy * n[1] + self.dz * n[2]
            u = self.dx * e_u[0] + self.dy * e_u[1]
            v = self.dx * e_v[0] + self.dy * e_v[1] + self.dz * e_v[2]
            ok = ((w > 0) & (np.abs(u) <= w * math.cos(t) - v * math.sin(t) + 1e-12) &
                  (u * u + v * v <= sin_lim * sin_lim) & (w > best_w))
            self.owner[ok] = k
            best_w[ok] = w[ok]
        self.w = np.clip(best_w, 0.0, 1.0)
        self.theta = np.degrees(np.arccos(self.w))

    def scanned(self, eps_min: float = -90.0, eps_max: float = 90.0) -> np.ndarray:
        """Cells owned by a face AND inside the mission eps window (SECTOR_EPS_MIN/MAX_DEG)."""
        return (self.owner >= 0) & (self.eps >= eps_min) & (self.eps <= eps_max)

    def sector_mask(self, sec: dict) -> np.ndarray:
        """Boolean mask of a mission watch sector (box or cap), identical semantics to
        core.optimizer.Sphere.mask -- kept in sync deliberately (same formulas, numpy dtypes)."""
        shape = sec.get("shape", "box")
        if shape == "cap":
            p0, e0, rad = math.radians(sec["phi0_deg"]), math.radians(sec["eps0_deg"]), sec["radius_deg"]
            p, e = np.radians(self.phi), np.radians(self.eps)
            cosang = np.sin(e) * math.sin(e0) + np.cos(e) * math.cos(e0) * np.cos(p - p0)
            return np.degrees(np.arccos(np.clip(cosang, -1, 1))) <= rad
        p1, p2 = sec["phi_min_deg"] % 360.0, sec["phi_max_deg"] % 360.0
        if sec["phi_max_deg"] - sec["phi_min_deg"] >= 360.0:
            mphi = np.ones(self.phi.shape, bool)
        elif p1 <= p2:
            mphi = (self.phi >= p1) & (self.phi <= p2)
        else:
            mphi = (self.phi >= p1) | (self.phi <= p2)
        return mphi & (self.eps >= sec["eps_min_deg"]) & (self.eps <= sec["eps_max_deg"])

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


SECTOR_GEOMETRY_FIELDS = ("shape", "phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg",
                          "phi0_deg", "eps0_deg", "radius_deg")


def sector_geometry_key(sec: dict) -> tuple:
    """Everything build_sector_overlay() depends on in a sector (cache key for its meshes)."""
    return tuple(sec.get(f) for f in SECTOR_GEOMETRY_FIELDS)


def build_axes(length: float = 1.35) -> List[dict]:
    return [
        {"line": pv.Line((0, 0, 0), (length, 0, 0)), "color": COLORS["axis_x"], "text": "X",
         "label_pos": (length * 1.1, 0, 0)},
        {"line": pv.Line((0, 0, 0), (0, length, 0)), "color": COLORS["axis_y"], "text": "Y",
         "label_pos": (0, length * 1.1, 0)},
        {"line": pv.Line((0, 0, 0), (0, 0, length)), "color": COLORS["axis_z"], "text": "Z (nose)",
         "label_pos": (0, 0, length * 1.1)},
        {"line": None, "color": COLORS["muted"], "text": "-Z thrusters",
         "label_pos": (0, 0, -0.55), "no_line": True},
    ]


def build_face_normals(tilt_deg: Optional[float] = None, length: float = 1.12) -> List[dict]:
    out = []
    for k, pk in enumerate(FACE_PHI_DEG):
        n = face_normal(pk, tilt_deg)
        start = np.array([RP * math.cos(math.radians(pk)), RP * math.sin(math.radians(pk)), 0.0])
        tip = start + n * length
        arrow = pv.Line(start, tip)
        out.append({"k": k, "mesh": arrow, "color": FACE_COLORS[k], "name": FACE_NAMES[k],
                    "label_pos": tuple(n * 1.26)})
    return out


def build_ship() -> dict:
    """Simple schematic hull (reference only), offset so the face-mount point sits at the
    direction-sphere origin (0,0,0)."""
    z0 = -ZN
    body = pv.Cylinder(center=(0, 0, -0.02 + z0), direction=(0, 0, 1), radius=0.07, height=0.62)
    nose = pv.Cone(center=(0, 0, 0.34 + z0), direction=(0, 0, 1), radius=0.07, height=0.1, resolution=28)
    dome = pv.Sphere(center=(0, 0, 0.39 + z0), radius=0.045)
    engine = pv.Cone(center=(0, 0, -0.37 + z0), direction=(0, 0, -1), radius=0.05, height=0.08, resolution=20)
    radiator = pv.Box(bounds=(-0.33, 0.33, -0.004, 0.004, -0.14 + z0 - 0.055, -0.14 + z0 + 0.055))
    arrays = []
    for k, pk in enumerate(FACE_PHI_DEG):
        n = face_normal(pk)
        center = (RP * math.cos(math.radians(pk)), RP * math.sin(math.radians(pk)), 0.0)
        array_disc = pv.Cylinder(center=center, direction=n, radius=0.058, height=0.014)
        arrays.append({"k": k, "mesh": array_disc, "color": FACE_COLORS[k]})
    hull = [{"mesh": body, "color": COLORS["ship"]}, {"mesh": nose, "color": COLORS["ship"]},
            {"mesh": dome, "color": "#2c3a4a"}, {"mesh": engine, "color": "#4a525c"},
            {"mesh": radiator, "color": "#d9a21b"}]
    return {"hull": hull, "arrays": arrays}


def km_axes_step(max_range_km: float) -> Tuple[float, int]:
    """(graduation step, number of graduations) of the 1:1 km axes: a round step (1-2-5 x 10^n) giving about 4 to 6
    graduations, the last one at or beyond max_range_km."""
    target = max_range_km / 4.0
    base = 10.0 ** math.floor(math.log10(target))
    step = max(m * base for m in (1.0, 2.0, 5.0) if m * base <= target)
    return step, int(math.ceil(max_range_km / step - 1e-9))
