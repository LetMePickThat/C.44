"""Star background of the scenario tool's 3D views (user, 29 October 2026): the stars of the Yale Bright Star
Catalogue (to about magnitude 6.5) at their J2000 directions, and on demand the lines of the 88 constellations
(assets/sky, tools/gen_sky.py).

The stars are at infinity: the same sky from anywhere in the Solar System (no parallax at this scale). They are drawn
in a renderer of their own under the scene (render window layer 0; the scene's renderer goes to layer 1 and is
drawn over them), whose camera only takes the scene camera's orientation and view angle: the stars never enter the
scene's bounds or its clipping range, and every body, object and line of the scene hides them. The global view is
ICRF (identity); the ship view is in the ship's frame, so the sky turns with the ship's attitude (set_rotation).

A star is a round point: its size and brightness from its V magnitude (one actor per size class), its colour from
its B-V colour index (blackbody colours, M. Charity's table, D).
"""
from __future__ import annotations

import csv
import json
from functools import lru_cache
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import pyvista as pv
import vtk

SKY_DIR = Path(__file__).resolve().parent.parent / "assets" / "sky"
STARS_FILE = SKY_DIR / "stars.csv"
LINES_FILE = SKY_DIR / "constellation_lines.json"
ATTRIBUTIONS = ("Stars: Yale Bright Star Catalogue, 5th rev. ed. (Hoffleit & Warren 1991), NASA HEASARC",
                "Constellation lines: d3-celestial, (c) 2015 Olaf Frohn, BSD 3-clause licence")

# size (px) and brightness of a star from its V magnitude (D, assistant's choice): the brightest stars SIZE_PX[0],
# magnitude 6.5 SIZE_PX[1]; brightness from 1 (magnitude <= 1) down to FAINT at 6.5
SIZE_PX = (5.5, 1.5)
SIZE_CLASSES = 8                                  # point sizes (one actor each)
MAG_RANGE = (-1.5, 6.5)
FAINT = 0.5
BV_RGB = ((-0.4, (155, 176, 255)), (-0.2, (170, 191, 255)), (0.0, (202, 215, 255)), (0.3, (248, 247, 255)),
          (0.6, (255, 244, 234)), (1.0, (255, 222, 180)), (1.5, (255, 204, 111)), (2.0, (255, 180, 90)))
BV_DEFAULT = 0.6                                  # a star without B-V: about solar
LINE_COLOR = "#5b7bb0"                            # constellation lines: discreet over the dark background
LINE_OPACITY = 0.55
SKY_RADIUS = 1.0                                  # the sky's sphere (its own camera at its centre)


def _unit(ra_deg: np.ndarray, dec_deg: np.ndarray) -> np.ndarray:
    ra, dec = np.radians(ra_deg), np.radians(dec_deg)
    return np.column_stack([np.cos(dec) * np.cos(ra), np.cos(dec) * np.sin(ra), np.sin(dec)])


@lru_cache(maxsize=1)
def stars() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(unit directions ICRF (n, 3), V magnitudes (n,), B-V (n,), NaN when unknown); empty when the file is missing."""
    try:
        with open(STARS_FILE, encoding="utf-8") as fh:
            rows = list(csv.DictReader(fh))
    except OSError:
        return np.zeros((0, 3)), np.zeros(0), np.zeros(0)
    ra = np.array([float(r["ra_deg"]) for r in rows])
    dec = np.array([float(r["dec_deg"]) for r in rows])
    mag = np.array([float(r["vmag"]) for r in rows])
    bv = np.array([float(r["bv"]) if r["bv"] else np.nan for r in rows])
    return _unit(ra, dec), mag, bv


@lru_cache(maxsize=1)
def constellation_lines() -> List[np.ndarray]:
    """Each line of the constellations as unit directions ICRF (k, 3); empty when the file is missing."""
    try:
        with open(LINES_FILE, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError):
        return []
    out = []
    for key, lines in data.items():
        if key.startswith("_"):
            continue
        for ln in lines:
            p = np.asarray(ln, float)
            if len(p) >= 2:
                out.append(_unit(p[:, 0], p[:, 1]))
    return out


def star_rgb(mag: np.ndarray, bv: np.ndarray) -> np.ndarray:
    """uint8 (n, 3): the colour of the B-V index, dimmed with the magnitude."""
    x = np.where(np.isfinite(bv), bv, BV_DEFAULT)
    keys = np.array([k for k, _c in BV_RGB])
    cols = np.array([c for _k, c in BV_RGB], float)
    rgb = np.column_stack([np.interp(x, keys, cols[:, c]) for c in range(3)])
    lo, hi = 1.0, MAG_RANGE[1]
    b = 1.0 - (1.0 - FAINT) * np.clip((mag - lo) / (hi - lo), 0.0, 1.0)
    return np.clip(rgb * b[:, None], 0, 255).astype(np.uint8)


def star_size(mag: np.ndarray) -> np.ndarray:
    """px: the brightest SIZE_PX[0], magnitude 6.5 SIZE_PX[1], linear in magnitude."""
    f = np.clip((mag - MAG_RANGE[0]) / (MAG_RANGE[1] - MAG_RANGE[0]), 0.0, 1.0)
    return SIZE_PX[0] + (SIZE_PX[1] - SIZE_PX[0]) * f


def _actor(mesh: pv.PolyData, rgb: bool) -> vtk.vtkActor:
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(mesh)
    if rgb:
        mesh.point_data.active_scalars_name = "rgb"
        mapper.SetColorModeToDirectScalars()
        mapper.SetScalarModeToUsePointData()
        mapper.ScalarVisibilityOn()
    else:
        mapper.ScalarVisibilityOff()
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().LightingOff()
    return actor


class Sky:
    """The star background of a pyvista plotter: a renderer on layer 0 under the plotter's own renderer."""

    def __init__(self, plotter, background: str):
        self.plotter = plotter
        win = plotter.ren_win
        win.SetNumberOfLayers(max(2, win.GetNumberOfLayers()))
        self.ren = vtk.vtkRenderer()
        self.ren.SetLayer(0)
        self.ren.InteractiveOff()                     # the clicks and the camera stay the scene's
        self.ren.SetBackground(*pv.Color(background).float_rgb)
        plotter.renderer.SetLayer(1)                  # the scene over the sky (a layer above 0 keeps its colours)
        win.AddRenderer(self.ren)
        self.star_actors = self._stars()
        self.line_actor = self._lines()
        self._m = vtk.vtkMatrix4x4()
        for a in self.star_actors + ([self.line_actor] if self.line_actor else []):
            a.SetUserMatrix(self._m)
            self.ren.AddActor(a)
        plotter.renderer.AddObserver("StartEvent", lambda *_a: self.sync())
        self.set_visible(True, False)

    def _stars(self) -> List[vtk.vtkActor]:
        d, mag, bv = stars()
        if not len(d):
            return []
        size = star_size(mag)
        edges = np.linspace(SIZE_PX[1], SIZE_PX[0], SIZE_CLASSES + 1)
        cls = np.clip(np.digitize(size, edges[1:-1]), 0, SIZE_CLASSES - 1)
        rgb = star_rgb(mag, bv)
        out = []
        for c in range(SIZE_CLASSES):
            sel = cls == c
            if not sel.any():
                continue
            mesh = pv.PolyData(SKY_RADIUS * d[sel])
            mesh.point_data["rgb"] = rgb[sel]
            a = _actor(mesh, True)
            p = a.GetProperty()
            p.SetPointSize(float(0.5 * (edges[c] + edges[c + 1])))
            p.SetRenderPointsAsSpheres(True)
            out.append(a)
        return out

    def _lines(self) -> Optional[vtk.vtkActor]:
        lines = constellation_lines()
        if not lines:
            return None
        pts = np.concatenate(lines) * SKY_RADIUS
        conn = []
        k = 0
        for ln in lines:
            conn += [len(ln)] + list(range(k, k + len(ln)))
            k += len(ln)
        a = _actor(pv.PolyData(pts, lines=np.array(conn)), False)
        p = a.GetProperty()
        p.SetColor(*pv.Color(LINE_COLOR).float_rgb)
        p.SetOpacity(LINE_OPACITY)
        p.SetLineWidth(1.0)
        return a

    def set_dots(self, dirs: np.ndarray, rgb: np.ndarray, size_px: float) -> None:
        """Dots at directions (n, 3) of the scene's axes as seen from its camera (the planets too small to draw,
        user 29 October 2026): on the sky's sphere, under the scene (a body in front hides them), at size_px."""
        a = getattr(self, "_dots", None)
        if a is not None:
            self.ren.RemoveActor(a)
            self._dots = None
        if not len(dirs):
            return
        mesh = pv.PolyData(SKY_RADIUS * np.asarray(dirs, float))
        mesh.point_data["rgb"] = np.asarray(rgb, np.uint8)
        a = _actor(mesh, True)
        a.GetProperty().SetPointSize(float(size_px))
        a.GetProperty().SetRenderPointsAsSpheres(True)
        self.ren.AddActor(a)
        self._dots = a

    def set_visible(self, stars_on: bool, lines_on: bool) -> None:
        """Skybox (stars_on) or the plain background; the constellation lines with the skybox only."""
        for a in self.star_actors:
            a.SetVisibility(stars_on)
        if self.line_actor is not None:
            self.line_actor.SetVisibility(stars_on and lines_on)

    def set_rotation(self, r: Optional[np.ndarray]) -> None:
        """The sky's directions as drawn: r (3, 3) @ ICRF direction (the ship view: ICRF -> ship frame); None: ICRF."""
        r = np.eye(3) if r is None else np.asarray(r, float)
        for i in range(3):
            for j in range(3):
                self._m.SetElement(i, j, float(r[i, j]))
        self._m.Modified()

    def sync(self) -> None:
        """Before each render: the sky's camera at the sphere's centre, looking where the scene camera looks."""
        cam = self.plotter.renderer.GetActiveCamera()
        sc = self.ren.GetActiveCamera()
        d = np.asarray(cam.GetDirectionOfProjection(), float)
        sc.SetPosition(0.0, 0.0, 0.0)
        sc.SetFocalPoint(*d)
        sc.SetViewUp(*cam.GetViewUp())
        sc.SetViewAngle(cam.GetViewAngle())
        sc.SetParallelProjection(cam.GetParallelProjection())
        sc.SetClippingRange(0.1 * SKY_RADIUS, 10.0 * SKY_RADIUS)
