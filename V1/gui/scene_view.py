#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENE_VIEW -- the 3D coverage view shared by the mission-preparation and results tabs: a display bar (one check
box per face responsibility zone, show/hide every watch volume, 1:1 scale and its graduated km axes) above a
PyVista plotter holding the static ship scene, the face layer and the watch-sector overlays.

The owning tab only says WHAT to draw (set_scene: eps window + one SectorDraw per sector) and which sectors are
hidden (set_hidden); the view builds the meshes (unit-sphere caps, or true-size volumes in km in 1:1 scale),
caches them, and renders once per update. The camera always turns about the scene centre (lock_pivot).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Set

import numpy as np
import pyvista as pv
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QCheckBox, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
from pyvistaqt import QtInteractor

from core import geometry as geo

KM_AXES_PREFIX = "km_axes_"
KM_AXES_LINES = [f"{KM_AXES_PREFIX}{a}" for a in ("X", "Y", "Z")]   # the 3D axis meshes (labels are 2D actors)


# ----------------------------------------------------------------------------- display bar
class DisplayBar(QWidget):
    faces_changed = pyqtSignal()
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()
    scale_changed = pyqtSignal(bool)
    scale_axes_changed = pyqtSignal(bool)
    background_changed = pyqtSignal(bool)
    altitude_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(4, 2, 4, 2)
        h.addWidget(QLabel("Responsibility zones:"))
        self.cb_faces = []
        for k in range(4):
            cb = QCheckBox(geo.FACE_NAMES[k])
            cb.setChecked(True)
            cb.setStyleSheet(f"color: {geo.FACE_COLORS[k]}; font-weight: bold;")
            cb.toggled.connect(self.faces_changed)
            h.addWidget(cb)
            self.cb_faces.append(cb)
        h.addSpacing(16)
        h.addWidget(QLabel("Mission volumes:"))
        b_all, b_none = QPushButton("Show all"), QPushButton("Hide all")
        b_all.clicked.connect(self.show_all_sectors)
        b_none.clicked.connect(self.hide_all_sectors)
        h.addWidget(b_all)
        h.addWidget(b_none)
        h.addSpacing(16)
        self.cb_true_scale = QCheckBox("Scale 1:1")
        self.cb_true_scale.setToolTip("Draw each watch volume at its true size (1 unit = 1 km), from the radar out to "
                                      "its required range, and zoom to fit them all. Untick to return to the "
                                      "default view of the unit sphere.")
        self.cb_true_scale.toggled.connect(self.scale_changed)
        h.addWidget(self.cb_true_scale)
        self.cb_background = QCheckBox("Background")
        self.cb_background.setEnabled(False)   # only meaningful in 1:1 scale
        self.cb_background.setToolTip("Show a background sphere the size of the Moon (1:1 scale only), at its "
                                      "true distance: Moon radius + the altitude box, along -Z.")
        self.cb_background.toggled.connect(self.background_changed)
        self.cb_true_scale.toggled.connect(self.cb_background.setEnabled)
        h.addWidget(self.cb_background)
        h.addWidget(QLabel("Altitude (km):"))
        self.sb_altitude = QDoubleSpinBox()
        self.sb_altitude.setRange(0.0, 1.0e7)
        self.sb_altitude.setDecimals(1)
        self.sb_altitude.setValue(100.0)
        self.sb_altitude.setMaximumWidth(90)
        self.sb_altitude.valueChanged.connect(self.altitude_changed)
        h.addWidget(self.sb_altitude)
        self.cb_scale_axes = QCheckBox("Scale axes")
        self.cb_scale_axes.setChecked(True)
        self.cb_scale_axes.setEnabled(False)   # only meaningful in 1:1 scale
        self.cb_scale_axes.setToolTip("Show / hide the X, Y, Z axes graduated in km (1:1 scale only)")
        self.cb_scale_axes.toggled.connect(self.scale_axes_changed)
        self.cb_true_scale.toggled.connect(self.cb_scale_axes.setEnabled)
        h.addWidget(self.cb_scale_axes)
        h.addStretch(1)

    def face_visible(self, k: int) -> bool:
        return self.cb_faces[k].isChecked()

    def true_scale(self) -> bool:
        return self.cb_true_scale.isChecked()

    def scale_axes_visible(self) -> bool:
        """Graduated km axes shown: 1:1 scale on and the "Scale axes" box ticked."""
        return self.cb_true_scale.isChecked() and self.cb_scale_axes.isChecked()

    def background_visible(self) -> bool:
        """Background (Moon-size) sphere shown: 1:1 scale on and the "Background" box ticked."""
        return self.cb_true_scale.isChecked() and self.cb_background.isChecked()

    def altitude_km(self) -> float:
        return self.sb_altitude.value()


# ----------------------------------------------------------------------------- plotter helpers (no render)
def face_actor_names(k: int) -> tuple:
    """Plotter actor names of face k's responsibility zone (surface, outline)."""
    return f"face_surf_{k}", f"face_outline_{k}"


def sector_actor_names(sid: str) -> tuple:
    """Plotter actor names of a mission watch sector (surface, outline, label)."""
    return f"sec_{sid}_surf", f"sec_{sid}_outline", f"sec_{sid}_label"


def _matching_actors(plotter, names: Iterable[str]) -> List[str]:
    """Actor keys for the given names. add_point_labels(name=X) registers its actor as "X-labels", hence the
    prefix match."""
    wanted = tuple(names)
    return [key for key in plotter.actors if key in wanted or key.startswith(tuple(n + "-" for n in wanted))]


def set_actors_visible(plotter, names, visible: bool) -> None:
    """Show/hide named actors without rebuilding them (missing names are ignored)."""
    for key in _matching_actors(plotter, names):
        plotter.actors[key].SetVisibility(bool(visible))


def has_actor(plotter, name: str) -> bool:
    return bool(_matching_actors(plotter, [name]))


def remove_actors(plotter, names) -> None:
    for key in _matching_actors(plotter, names):
        plotter.remove_actor(key, render=False)


def add_static_scene(plotter) -> None:
    """The parts of the scene that never change (axes, hull, face normals, equator)."""
    for ax in geo.build_axes():
        if ax.get("no_line"):
            plotter.add_point_labels([ax["label_pos"]], [ax["text"]], text_color=ax["color"],
                                     font_size=12, shape=None, show_points=False, render=False)
            continue
        plotter.add_mesh(ax["line"], color=ax["color"], line_width=2, name=f"axis_{ax['text']}", render=False)
        plotter.add_point_labels([ax["label_pos"]], [ax["text"]], text_color=ax["color"],
                                 font_size=12, shape=None, show_points=False, render=False)
    ship = geo.build_ship()
    for i, part in enumerate(ship["hull"]):
        plotter.add_mesh(part["mesh"], color=part["color"], smooth_shading=True, name=f"hull_{i}", render=False,
                         silhouette={"color": "#7b8794", "line_width": 1})   # off-white hull on the white background
    for d in ship["arrays"]:
        plotter.add_mesh(d["mesh"], color=d["color"], name=f"hull_array_{d['k']}", render=False)
    for n in geo.build_face_normals():
        plotter.add_mesh(n["mesh"], color=n["color"], line_width=3, name=f"normal_{n['k']}", render=False)
        plotter.add_point_labels([n["label_pos"]], [n["name"]], text_color=n["color"],
                                 font_size=14, shape=None, show_points=False, render=False)
    ang = np.linspace(0, 2 * math.pi, 181)
    eq_pts = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], axis=1)
    plotter.add_mesh(pv.lines_from_points(eq_pts), color=geo.COLORS["muted"], opacity=0.45,
                     line_width=1, name="equator", render=False)
    plotter.set_background("white")


def set_default_camera(plotter) -> None:
    """Default iso view framing the unit sphere; its centre is the rotation pivot."""
    r = 5.5
    eye = (r * math.cos(math.radians(24)) * math.cos(math.radians(-58)),
           r * math.cos(math.radians(24)) * math.sin(math.radians(-58)),
           r * math.sin(math.radians(24)))
    plotter.camera.up = (0, 0, 1)
    plotter.camera.position = eye
    plotter.camera.focal_point = (0, 0, 0.12)
    plotter.camera.clipping_range = (0.05, 50.0)
    plotter._pivot = (0.0, 0.0, 0.12)


def lock_pivot(plotter) -> None:
    """Keep the scene centred: the camera always turns about plotter._pivot (the scene centre set by
    set_default_camera / fit_camera_to_actors). VTK turns about the focal point, which a pan (middle button,
    shift + left button) moves off-centre; before every render a drifted camera is translated back so its focal point
    is the pivot again -- orientation and distance unchanged, so rotation and zoom work and panning has no effect."""
    renderer = plotter.renderer

    def recentre(*_args) -> None:
        pivot = getattr(plotter, "_pivot", None)
        if pivot is None:
            return
        cam = renderer.GetActiveCamera()
        off = np.asarray(pivot) - np.asarray(cam.GetFocalPoint())
        if np.abs(off).max() <= 1e-9 * max(1.0, cam.GetDistance()):
            return
        cam.SetFocalPoint(*pivot)
        cam.SetPosition(*(np.asarray(cam.GetPosition()) + off))
        renderer.ResetCameraClippingRange()

    plotter.ren_win.AddObserver("StartEvent", recentre)


def fit_camera_to_actors(plotter, names) -> None:
    """Keep the view direction, re-centre and zoom so the named actors (and the radar at the origin) fit
    the view; their centre becomes the rotation pivot. Does nothing if none of the actors exists."""
    keys = _matching_actors(plotter, names)
    if not keys:
        return
    b = np.array([plotter.actors[k].GetBounds() for k in keys])
    bounds = (min(b[:, 0].min(), 0.0), max(b[:, 1].max(), 0.0), min(b[:, 2].min(), 0.0),
              max(b[:, 3].max(), 0.0), min(b[:, 4].min(), 0.0), max(b[:, 5].max(), 0.0))
    plotter.reset_camera(render=False, bounds=bounds)
    plotter._pivot = tuple(plotter.camera.GetFocalPoint())   # rotations now turn about the centre of the volumes


def set_km_axes(plotter, max_range_km: Optional[float]) -> None:
    """1:1 scale reference: the ship-frame X, Y, Z axes drawn from the radar in km, graduated every round step and
    labelled in km, so the scale turns with the view and holds at any angle. None removes them. Rebuilt only when
    the graduation changes."""
    key = geo.km_axes_step(max_range_km) if max_range_km and max_range_km > 0 else None
    if key == getattr(plotter, "_km_axes_key", None):
        return
    remove_actors(plotter, [k for k in plotter.actors if k.startswith(KM_AXES_PREFIX)])
    plotter._km_axes_key = key
    if key is None:
        return
    step, n = key
    length = step * n
    tick = 0.025 * length
    axes = (("X", np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), geo.COLORS["axis_x"]),
            ("Y", np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), geo.COLORS["axis_y"]),
            ("Z (nose)", np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), geo.COLORS["axis_z"]))
    for name, d, t, color in axes:
        pts = [np.zeros(3), d * length]
        for k in range(1, n + 1):
            pts += [d * k * step - t * tick, d * k * step + t * tick]
        lines = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(n + 1)])
        tag = name.split()[0]
        plotter.add_mesh(pv.PolyData(np.array(pts), lines=lines), color=color, line_width=2,
                         name=f"{KM_AXES_PREFIX}{tag}", render=False)
        plotter.add_point_labels([d * k * step + t * tick for k in range(1, n + 1)],
                                 [f"{k * step:g} km" if k == n else f"{k * step:g}" for k in range(1, n + 1)],
                                 text_color=color, font_size=10, shape=None, show_points=False,
                                 name=f"{KM_AXES_PREFIX}ticks_{tag}", render=False)
        plotter.add_point_labels([d * length * 1.08], [name], text_color=color, font_size=13, bold=True,
                                 shape=None, show_points=False, name=f"{KM_AXES_PREFIX}name_{tag}", render=False)


BACKGROUND_ACTOR = "background_sphere"


@lru_cache(maxsize=1)
def _moon_texture() -> pv.Texture:
    tex = pv.Texture(geo.moon_texture())
    tex.interpolate = True
    tex.mipmap = True   # the real texture can be a few k pixels across; mipmaps avoid shimmering at a distance
    return tex


def set_background_sphere(plotter, visible: bool, altitude_km: Optional[float]) -> None:
    """The Moon-textured background sphere (1:1 scale only): added/updated when visible, removed otherwise.
    Rebuilt only when the altitude changes."""
    if not visible:
        remove_actors(plotter, [BACKGROUND_ACTOR])
        plotter._background_key = None
        return
    key = round(altitude_km, 3)
    if key != getattr(plotter, "_background_key", None):
        plotter._background_key = key
        plotter.add_mesh(geo.build_background_sphere(altitude_km), texture=_moon_texture(),
                         name=BACKGROUND_ACTOR, render=False)
    plotter.actors[BACKGROUND_ACTOR].SetVisibility(True)
    plotter.renderer.ResetCameraClippingRange()


def set_reference_scene_visible(plotter, visible: bool) -> None:
    """Show/hide the unit-scale reference scene (hull, axes, face normals, equator and their labels) -- every actor
    that is not a face zone, a watch sector, the km axes or the background sphere. Hidden in 1:1 scale, where it is
    a ~1 km speck whose labels pile up on the radar; the graduated km axes (and the background sphere) replace it."""
    for key, actor in plotter.actors.items():
        if not key.startswith(("sec_", "face_", KM_AXES_PREFIX, BACKGROUND_ACTOR)):
            actor.SetVisibility(bool(visible))


# ----------------------------------------------------------------------------- the view
@dataclass
class SectorDraw:
    """One watch sector as the owning tab wants it drawn."""
    sid: str                     # actor key (unique per sector)
    sec: dict                    # geometry source: shape fields, and range_km for the 1:1 volume
    color: str
    text: Optional[str] = None   # label; None = no label


class SceneView(QWidget):
    """Display bar + plotter. Re-exposes the bar's show/hide-all buttons; everything else (faces, 1:1 scale,
    km axes) is handled here."""
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()

    def __init__(self, grid: geo.SphereGrid, face_opacity: float, sector_opacity: float, parent=None):
        super().__init__(parent)
        self.grid = grid
        self.face_opacity, self.sector_opacity = face_opacity, sector_opacity
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        self.display_bar = DisplayBar()
        self.display_bar.faces_changed.connect(self.apply_visibility)
        self.display_bar.show_all_sectors.connect(self.show_all_sectors)
        self.display_bar.hide_all_sectors.connect(self.hide_all_sectors)
        self.display_bar.scale_changed.connect(self._on_scale_changed)
        self.display_bar.scale_axes_changed.connect(lambda _on: self.apply_visibility())
        self.display_bar.background_changed.connect(lambda _on: self._update_background())
        self.display_bar.altitude_changed.connect(lambda _v: self._update_background())
        v.addWidget(self.display_bar)
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(500)
        v.addWidget(self.plotter.interactor)
        add_static_scene(self.plotter)
        lock_pivot(self.plotter)
        set_default_camera(self.plotter)

        self._eps: Optional[tuple] = None             # eps window of the scene on screen
        self._face_key: Optional[tuple] = None        # eps window of the face layer on screen
        self._sectors: List[SectorDraw] = []
        self._hidden: Set[str] = set()
        self._actor_names: Set[str] = set()
        self._cache: Dict[str, tuple] = {}            # sid -> (geometry key, colour, poly, outline, label text)

    # ------------------------------------------------------------------ public API
    def set_scene(self, eps_min: float, eps_max: float, sectors: List[SectorDraw],
                  hidden: Optional[Set[str]] = None) -> None:
        """Draw this eps window and these sectors (only what changed is rebuilt), hiding the `hidden` sector ids
        (None: keep the current ones), then render once."""
        self._eps, self._sectors = (eps_min, eps_max), list(sectors)
        if hidden is not None:
            self._hidden = set(hidden)
        self._build_faces()
        self._build_sectors()
        self.apply_visibility()

    def set_hidden(self, sids: Set[str]) -> None:
        self._hidden = set(sids)
        self.apply_visibility()

    def apply_visibility(self) -> None:
        """Re-applied after every rebuild, since rebuilt actors come back visible; renders."""
        bar, p = self.display_bar, self.plotter
        for k in range(4):
            set_actors_visible(p, face_actor_names(k), bar.face_visible(k) and not bar.true_scale())
        set_reference_scene_visible(p, not bar.true_scale())
        for key, actor in p.actors.items():
            if key.startswith(KM_AXES_PREFIX):
                actor.SetVisibility(bar.scale_axes_visible())
        for d in self._sectors:
            set_actors_visible(p, sector_actor_names(d.sid), d.sid not in self._hidden)
        p.render()

    # ------------------------------------------------------------------ internals
    def _build_faces(self) -> None:
        if self._face_key == self._eps:
            return  # the face layer only depends on the eps window, not on the watch sectors
        self._face_key = self._eps
        scene = geo.build_face_sectors(self.grid, *self._eps)
        for k in range(4):
            surf_name, out_name = face_actor_names(k)
            if scene.surfaces[k].n_points:
                self.plotter.add_mesh(scene.surfaces[k], color=geo.FACE_COLORS[k], opacity=self.face_opacity,
                                      culling="back", name=surf_name, render=False)
            else:
                self.plotter.remove_actor(surf_name, render=False)
            if scene.outlines[k].n_points:
                self.plotter.add_mesh(scene.outlines[k], color=geo.COLORS["fg"], line_width=1, name=out_name,
                                      render=False)
            else:
                self.plotter.remove_actor(out_name, render=False)

    def _build_sectors(self) -> None:
        """Only sectors whose geometry / colour / label changed are rebuilt. In 1:1 scale each sector is a true-size
        volume in km (radar at the origin out to its required range), else a cap on the unit sphere."""
        true_scale = self.display_bar.true_scale()
        eps_min, eps_max = self._eps
        new_names: Set[str] = set()
        new_cache: Dict[str, tuple] = {}
        for d in self._sectors:
            surf_name, out_name, lbl_name = sector_actor_names(d.sid)
            key = (geo.sector_geometry_key(d.sec), eps_min, eps_max, d.sec.get("range_km") if true_scale else None)
            cached = self._cache.get(d.sid)
            # a duplicate id (mid-edit) shares actor names with the earlier sector: always redraw, last one wins
            same_geom = cached is not None and cached[0] == key and d.sid not in new_cache
            if same_geom:
                poly, outline = cached[2], cached[3]
            else:
                build = geo.build_sector_volume_km if true_scale else geo.build_sector_overlay
                try:
                    poly, outline = build(self.grid, d.sec, eps_min, eps_max)
                except (KeyError, TypeError, ValueError):
                    continue  # incomplete sector mid-edit: no 3D overlay
            same_mesh = same_geom and cached[1] == d.color
            if poly.n_points:
                if not (same_mesh and has_actor(self.plotter, surf_name)):
                    self.plotter.add_mesh(poly, color=d.color, opacity=self.sector_opacity,
                                          culling=None if true_scale else "back", name=surf_name, render=False)
                new_names.add(surf_name)
            if outline.n_points:
                if not (same_mesh and has_actor(self.plotter, out_name)):
                    self.plotter.add_mesh(outline, color=d.color, line_width=2, name=out_name, render=False)
                new_names.add(out_name)
            text = d.text if poly.n_points else None
            if text is not None:
                if not (same_mesh and cached[4] == text and has_actor(self.plotter, lbl_name)):
                    self.plotter.add_point_labels([poly.points.mean(axis=0)], [text], name=lbl_name,
                                                  text_color=d.color, font_size=10, shape_opacity=0.6,
                                                  show_points=False, render=False)
                new_names.add(lbl_name)
            new_cache[d.sid] = (key, d.color, poly, outline, text)
        remove_actors(self.plotter, self._actor_names - new_names)   # incl. the "-labels" actors
        self._actor_names, self._cache = new_names, new_cache
        ranges = [d.sec.get("range_km") for d in self._sectors if isinstance(d.sec.get("range_km"), (int, float))]
        set_km_axes(self.plotter, max(ranges, default=None) if true_scale else None)

    def _on_scale_changed(self, true_scale: bool) -> None:
        """1:1 on: redraw the volumes in km and zoom to fit them all; off: back to the default unit-sphere view."""
        if self._eps is not None:
            self._build_sectors()
        if true_scale:
            fit_camera_to_actors(self.plotter, [n for n in self._actor_names if n.endswith("_surf")]
                                 + (KM_AXES_LINES if self.display_bar.scale_axes_visible() else []))
        else:
            set_default_camera(self.plotter)
        self._update_background()
        self.apply_visibility()

    def _update_background(self) -> None:
        bar = self.display_bar
        set_background_sphere(self.plotter, bar.background_visible(), bar.altitude_km())
        self.plotter.render()
