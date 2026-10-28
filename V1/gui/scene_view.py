#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENE_VIEW -- the 3D coverage view shared by the mission-preparation and results tabs: a display bar (one check
box per face responsibility zone, show/hide every watch volume, 1:1 scale and its graduated km axes) above a
PyVista plotter holding the static ship scene, the face layer and the watch-sector overlays.

The owning tab only says WHAT to draw (set_scene: eps window + one SectorDraw per sector) and which sectors are
hidden (set_hidden); the view builds the meshes (unit-sphere caps, or true-size volumes in km in 1:1 scale),
caches them, and renders once per update. The camera always turns about the scene centre (lock_pivot).

Sector labels can be left-dragged off their volume to de-clutter crowded scenes; a thin leader line keeps them
tied to it, and a double-click snaps a moved label back. PyVista's label actors aren't individually pickable
(add_point_labels always adds them non-pickable), so hit-testing estimates each label's on-screen box from its
text instead of querying VTK -- see _label_hit. Dragged positions are in the current coordinate system (km in
1:1 scale, unit-sphere otherwise) and are cleared on a scale-mode switch (_on_scale_changed).

View frame (step 3.0): everything is drawn in the SHIP frame; the "ICRF" view only places the camera in the inertial
frame of the mission navigation (set_navigation) and adds the ICRF axes, so the ship appears with its attitude.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Set

import numpy as np
import pyvista as pv
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout,
                             QWidget)
from pyvistaqt import QtInteractor

from core import geometry as geo
from core.params import P
from gui.widgets import DecimalSpinBox

KM_AXES_PREFIX = "km_axes_"
KM_AXES_LINES = [f"{KM_AXES_PREFIX}{a}" for a in ("X", "Y", "Z")]   # the 3D axis meshes (labels are 2D actors)
ICRF_AXES = (("X", "X ICRF (vernal equinox)"), ("Y", "Y ICRF"), ("Z", "Z ICRF (celestial pole)"))
ICRF_AXIS_NAMES = [f"icrf_axis_{a}" for a, _ in ICRF_AXES]
SECTOR_LABEL_FONT_SIZE = 13   # shared by _build_sectors (draws) and _label_hit (estimates the on-screen box)


# ----------------------------------------------------------------------------- display bar
class DisplayBar(QWidget):
    faces_changed = pyqtSignal()
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()
    scale_changed = pyqtSignal(bool)
    scale_axes_changed = pyqtSignal(bool)
    background_changed = pyqtSignal(bool)
    altitude_changed = pyqtSignal(float)
    view_frame_changed = pyqtSignal(str)

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
        self.sb_altitude = DecimalSpinBox()
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
        h.addSpacing(16)
        h.addWidget(QLabel("View:"))
        self.cb_view_frame = QComboBox()
        self.cb_view_frame.addItems(["ship", "ICRF"])
        self.cb_view_frame.setToolTip("Ship: the ship frame. ICRF: the camera in the inertial frame of the mission "
                                      "navigation (attitude), with the ICRF axes; the ship appears with its attitude")
        self.cb_view_frame.currentTextChanged.connect(self.view_frame_changed)
        h.addWidget(self.cb_view_frame)
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
    """Plotter actor names of a mission watch sector (surface, outline, label, leader line to a dragged label)."""
    return f"sec_{sid}_surf", f"sec_{sid}_outline", f"sec_{sid}_label", f"sec_{sid}_leader"


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


def set_default_camera(plotter, r_view: Optional[np.ndarray] = None) -> None:
    """Default iso view framing the unit sphere; its centre is the rotation pivot. r_view (3x3, ship <- view frame,
    e.g. ICRF): the same view taken in that frame."""
    r = 5.5
    eye = np.array((r * math.cos(math.radians(24)) * math.cos(math.radians(-58)),
                    r * math.cos(math.radians(24)) * math.sin(math.radians(-58)),
                    r * math.sin(math.radians(24))))
    up, focal = np.array((0.0, 0.0, 1.0)), np.array((0.0, 0.0, 0.12))
    if r_view is not None:
        eye, up = r_view @ eye + focal, r_view @ up
    plotter.camera.up = tuple(up)
    plotter.camera.position = tuple(eye)
    plotter.camera.focal_point = tuple(focal)
    plotter.camera.clipping_range = (0.05, 50.0)
    plotter._pivot = tuple(focal)


def set_icrf_axes(plotter, r_is: Optional[np.ndarray], length: float = 1.6) -> None:
    """ICRF axes drawn in the ship frame (r_is: ship <- ICRF), or removed (None)."""
    remove_actors(plotter, ICRF_AXIS_NAMES + [n + "_label" for n in ICRF_AXIS_NAMES])
    if r_is is None:
        return
    for k, ((a, text), name) in enumerate(zip(ICRF_AXES, ICRF_AXIS_NAMES)):
        tip = r_is[:, k] * length
        plotter.add_mesh(pv.Line((0, 0, 0), tuple(tip)), color=geo.COLORS["muted"], line_width=2, name=name,
                         render=False)
        plotter.add_point_labels([tuple(tip * 1.08)], [text], text_color=geo.COLORS["muted"], font_size=11,
                                 shape=None, show_points=False, name=name + "_label", render=False)


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    k = axis / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx)


_ARC: Dict[int, dict] = {}                    # per arcball style: its plotter, radius and drag state


def _arc_ball(st: dict, x: float, y: float) -> np.ndarray:
    w, h = st["plotter"].ren_win.GetSize()
    r = max(st["radius"] * 0.5 * min(w, h), 1.0)
    v = np.array([(x - 0.5 * w) / r, (y - 0.5 * h) / r, 0.0])
    d2 = v[0] ** 2 + v[1] ** 2
    if d2 < 1.0:
        v[2] = math.sqrt(1.0 - d2)
    else:
        v[:2] /= math.sqrt(d2)
    return v


def _arc_move(style, _ev) -> None:
    st = _ARC[id(style)]
    if st["p"] is None:
        style.OnMouseMove()
        return
    plotter = st["plotter"]
    p1 = _arc_ball(st, *style.GetInteractor().GetEventPosition())
    p0 = st["p"]
    axis = np.cross(p0, p1)
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        return
    angle = math.atan2(s, float(p0 @ p1))
    cam = plotter.renderer.GetActiveCamera()
    f, pos = np.asarray(cam.GetFocalPoint()), np.asarray(cam.GetPosition())
    back = pos - f
    up = np.asarray(cam.GetViewUp())
    z = back / np.linalg.norm(back)
    y = up - (up @ z) * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    rot = _rotation(axis[0] * x + axis[1] * y + axis[2] * z, -angle)   # screen x right, y up, z to the viewer
    cam.SetPosition(*(f + rot @ back))                                  # the scene follows the hand
    cam.SetViewUp(*(rot @ y))
    cam.OrthogonalizeViewUp()
    plotter.renderer.ResetCameraClippingRange()
    st["p"] = p1
    style.GetInteractor().Render()


def install_arcball(plotter, radius_frac: float):
    """Left-drag rotation by a virtual trackball (Shoemake arcball, user 29 October 2026): the press and the current
    point are put on a sphere of radius radius_frac x the half-height of the view around its centre; inside it the
    scene turns as a ball under the hand, outside (left / right of the centre, top / bottom) the points lie on its rim
    and the drag turns the view about the line of sight (clockwise / counterclockwise). The camera turns about its
    focal point (the pivot of lock_pivot). Built on pyvista's own trackball style (its capture of the release events
    and the observers pyvista puts on the style stay valid: install it BEFORE adding them); Shift / Ctrl + left drag
    keep VTK's pan / spin; the wheel, the right button and the interactor's observers (clicks) are unchanged."""
    from pyvista.plotting.render_window_interactor import InteractorStyleTrackballCamera

    class _Arcball(InteractorStyleTrackballCamera):
        def _press(self, *a):
            iren = self.GetInteractor()
            if iren.GetShiftKey() or iren.GetControlKey():
                super()._press(*a)
                return
            st = _ARC[id(self)]
            st["p"] = _arc_ball(st, *iren.GetEventPosition())
            self.StartRotate()

        def _release(self, *a):
            st = _ARC[id(self)]
            if st["p"] is None:
                super()._release(*a)
                return
            st["p"] = None
            self.EndRotate()

    style = _Arcball(plotter.iren)
    _ARC[id(style)] = {"plotter": plotter, "radius": float(radius_frac), "p": None}
    style.add_observer("MouseMoveEvent", _arc_move)
    plotter.iren.style = style
    return style


def lock_pivot(plotter) -> None:
    """Keep the scene centred: the camera always turns about plotter._pivot (the scene centre set by
    set_default_camera / fit_camera_to_actors). VTK turns about the focal point, which shift + left-button pan
    moves off-centre; before every render a drifted camera is translated back so its focal point is the pivot
    again -- orientation and distance unchanged, so rotation and zoom work and that pan has no effect.

    A middle-button pan is the deliberate exception, for manually recentring on a mission's approximate
    barycentre (fit_camera_to_actors only fits the currently visible sectors, which is not always what's
    wanted): recentre() is suspended for as long as the middle button is held (plotter._panning), so VTK's own
    pan runs unopposed, and on release the pivot jumps to wherever the pan left the focal point -- later
    rotations then turn about that new point."""
    renderer = plotter.renderer
    plotter._panning = False

    def recentre(*_args) -> None:
        if getattr(plotter, "_panning", False):
            return
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

    def start_pan(*_args) -> None:
        plotter._panning = True

    def end_pan(*_args) -> None:
        plotter._panning = False
        plotter._pivot = tuple(renderer.GetActiveCamera().GetFocalPoint())

    plotter.ren_win.AddObserver("StartEvent", recentre)
    plotter.iren.add_observer("MiddleButtonPressEvent", start_pan)
    # Not MiddleButtonReleaseEvent: any extra observer on that specific event -- on the interactor (silently
    # swallowed, the same wall pyvista hits for Left/Right, see add_observer()'s source) or on the interactor
    # style (fires, but then blocks the style's own internal EndPan(), leaving its state stuck mid-pan and
    # every later rotation/zoom dead) -- breaks one or the other. EndInteractionEvent is the style's own
    # generic "an interaction just ended" signal (rotate, zoom or pan): observing it is the documented,
    # unintrusive way to react to the end of any of them, so it does not fight VTK's own bookkeeping; the
    # plotter._panning check is almost redundant here (a rotate/zoom leaves the focal point where it was, so
    # re-syncing the pivot to it then is a harmless no-op), kept only to skip that pointless resync.
    plotter.iren.add_observer("EndInteractionEvent", lambda *a: end_pan() if plotter._panning else None)


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


ARCBALL_RADIUS = float(P.get("tool.display.arcball_radius"))   # mouse rotation of the 3D views (user)
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
        self.display_bar.view_frame_changed.connect(lambda _f: self._apply_view_frame())
        v.addWidget(self.display_bar)
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(500)
        self.plotter.interactor.setToolTip("Drag a label to reposition it (a line keeps it linked to its volume); "
                                           "double-click a moved label to snap it back.")
        v.addWidget(self.plotter.interactor)
        add_static_scene(self.plotter)
        lock_pivot(self.plotter)
        install_arcball(self.plotter, ARCBALL_RADIUS)   # before the label drag: its release observer goes on the style
        set_default_camera(self.plotter)

        self._eps: Optional[tuple] = None             # eps window of the scene on screen
        self._face_key: Optional[tuple] = None        # eps window of the face layer on screen
        self._sectors: List[SectorDraw] = []
        self._hidden: Set[str] = set()
        self._actor_names: Set[str] = set()
        self._cache: Dict[str, tuple] = {}            # sid -> (geometry key, colour, poly, outline, label text)

        self._label_pos: Dict[str, np.ndarray] = {}   # sid -> dragged label point (world coords), only if moved
        self._drag: Optional[dict] = None              # active drag: {"sid", "z"} while the mouse button is down
        self._r_is: Optional[np.ndarray] = None        # ship <- ICRF of the mission navigation (step 3.0)
        self._install_label_drag()

    # ------------------------------------------------------------------ view frame (step 3.0)
    def set_navigation(self, r_si: Optional[np.ndarray]) -> None:
        """Attitude of the mission navigation (ship -> ICRF), None if invalid; the ICRF view follows it."""
        r_is = None if r_si is None else np.asarray(r_si).T
        same = (r_is is None and self._r_is is None) or (
            r_is is not None and self._r_is is not None and np.array_equal(r_is, self._r_is))
        self._r_is = r_is
        self.display_bar.cb_view_frame.setEnabled(r_is is not None)
        if not same and self.display_bar.cb_view_frame.currentText() == "ICRF":
            self._apply_view_frame()

    def _icrf_view(self) -> bool:
        return self.display_bar.cb_view_frame.currentText() == "ICRF" and self._r_is is not None

    def _apply_view_frame(self) -> None:
        set_icrf_axes(self.plotter, self._r_is if self._icrf_view() else None)
        if not self.display_bar.true_scale():
            set_default_camera(self.plotter, self._r_is if self._icrf_view() else None)
        self.apply_visibility()

    # ------------------------------------------------------------------ draggable labels
    def _install_label_drag(self) -> None:
        """Left-drag a sector label to reposition it (a leader line keeps it tied to its volume); double-click a
        moved label to snap it back. Hit-testing is approximate (PyVista's label actors aren't individually
        pickable -- see add_point_labels source -- so this estimates each label's screen footprint from its text
        instead of querying VTK), so the box is kept generous rather than pixel-exact.

        The press observer is registered directly on the interactor (not through pyvista's add_observer) at a
        priority above the interactor style's own, and aborts the event (via GetCommand(tag).SetAbortFlag) when
        it hits a label, so the trackball style never starts its rotate -- cheaper and much safer than swapping
        the style out for the drag, which was tried first and left release events undeliverable: VTK only
        relays *ButtonReleaseEvent reliably to an ATTACHED style (pyvista hits the same wall for Left/Right --
        see add_observer()'s source -- and works around it the same way, through the style), so detaching it
        for the drag's duration meant our own release observer, and the middle-button pan below, silently
        stopped firing until the interactor was recreated."""
        iren = self.plotter.iren
        tag = iren.interactor.AddObserver("LeftButtonPressEvent", self._on_label_press, 10.0)
        self._label_press_tag = tag
        iren.add_observer("MouseMoveEvent", self._on_label_move)
        iren.add_observer("LeftButtonReleaseEvent", self._on_label_release)
        iren.add_observer("LeftButtonDoubleClickEvent", self._on_label_double_click)

    def _label_anchor_and_point(self, sid: str) -> Optional[tuple]:
        """(anchor, current label point) of sid's label in world coords, None if it has none right now."""
        cached = self._cache.get(sid)
        if cached is None or cached[4] is None:   # cached[4] = label text, None = no label drawn
            return None
        anchor = cached[2].points.mean(axis=0)
        return anchor, self._label_pos.get(sid, anchor), cached[4]

    def _label_hit(self, x: int, y: int) -> Optional[str]:
        """sid of the topmost sector label whose estimated screen box contains (x, y), None if none does."""
        ren = self.plotter.renderer
        best, best_d2 = None, None
        for d in self._sectors:
            if d.sid in self._hidden:
                continue
            found = self._label_anchor_and_point(d.sid)
            if found is None:
                continue
            _anchor, point, text = found
            ren.SetWorldPoint(*point, 1.0)
            ren.WorldToDisplay()
            dx, dy, _dz = ren.GetDisplayPoint()
            lines = text.split("\n")
            w = 10 + max(len(ln) for ln in lines) * SECTOR_LABEL_FONT_SIZE * 0.56
            h = 10 + len(lines) * SECTOR_LABEL_FONT_SIZE * 1.3
            x0, x1 = dx - 6, dx + w
            y0, y1 = dy - 6, dy + h
            if x0 <= x <= x1 and y0 <= y <= y1:
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                d2 = (x - cx) ** 2 + (y - cy) ** 2
                if best_d2 is None or d2 < best_d2:
                    best, best_d2 = d.sid, d2
        return best

    def _on_label_press(self, obj, _event) -> None:
        x, y = self.plotter.iren.get_event_position()
        sid = self._label_hit(x, y)
        if sid is None:
            return
        found = self._label_anchor_and_point(sid)
        if found is None:
            return
        _anchor, point, _text = found
        ren = self.plotter.renderer
        ren.SetWorldPoint(*point, 1.0)
        ren.WorldToDisplay()
        z = ren.GetDisplayPoint()[2]           # depth of the drag plane: parallel to the screen, through the label
        self._drag = {"sid": sid, "z": z}
        obj.GetCommand(self._label_press_tag).SetAbortFlag(1)   # don't let the trackball style also start a rotate

    def _on_label_move(self, _obj, _event) -> None:
        if self._drag is None:
            return
        x, y = self.plotter.iren.get_event_position()
        ren = self.plotter.renderer
        ren.SetDisplayPoint(x, y, self._drag["z"])
        ren.DisplayToWorld()
        wx, wy, wz, ww = ren.GetWorldPoint()
        self._label_pos[self._drag["sid"]] = np.array([wx, wy, wz]) / (ww if ww else 1.0)
        self._redraw_label(self._drag["sid"])

    def _on_label_release(self, _obj, _event) -> None:
        self._drag = None

    def _on_label_double_click(self, _obj, _event) -> None:
        x, y = self.plotter.iren.get_event_position()
        sid = self._label_hit(x, y)
        if sid is None or self._label_pos.pop(sid, None) is None:
            return
        self._redraw_label(sid)

    def _redraw_label(self, sid: str) -> None:
        """Re-place sid's label actor (and its leader line, added/removed as needed) without rebuilding the
        sector mesh -- cheap enough to call on every mouse-move while dragging."""
        cached = self._cache.get(sid)
        if cached is None or cached[4] is None:
            return
        _key, color, poly, _outline, text = cached
        _surf_name, _out_name, lbl_name, leader_name = sector_actor_names(sid)
        anchor = poly.points.mean(axis=0)
        point = self._label_pos.get(sid, anchor)
        self.plotter.add_point_labels([point], [text], name=lbl_name, text_color="white", font_size=SECTOR_LABEL_FONT_SIZE,
                                      bold=True, shape_color="black", shape_opacity=0.75, show_points=False,
                                      render=False)
        self._actor_names.add(lbl_name)
        if sid in self._label_pos:
            self.plotter.add_mesh(pv.Line(anchor, point), color=color, line_width=1, name=leader_name, render=False)
            self._actor_names.add(leader_name)
        else:
            remove_actors(self.plotter, [leader_name])
            self._actor_names.discard(leader_name)
        self.apply_visibility()

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
        set_reference_scene_visible(p, not bar.true_scale())   # ICRF axes included
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
            surf_name, out_name, lbl_name, leader_name = sector_actor_names(d.sid)
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
            anchor = poly.points.mean(axis=0)
            point = self._label_pos.get(d.sid, anchor)
            if text is not None:
                rebuild_label = not (same_mesh and cached[4] == text and has_actor(self.plotter, lbl_name))
                if rebuild_label:
                    # Fixed white-on-near-opaque-black readout: text_color=d.color (the sector's own colour, e.g.
                    # green) used to sit on a translucent grey box over the same-coloured sector mesh -- unreadable.
                    self.plotter.add_point_labels([point], [text], name=lbl_name,
                                                  text_color="white", font_size=SECTOR_LABEL_FONT_SIZE, bold=True,
                                                  shape_color="black", shape_opacity=0.75,
                                                  show_points=False, render=False)
                new_names.add(lbl_name)
                if d.sid in self._label_pos:   # dragged away from its volume: show the link back to it
                    if rebuild_label or not has_actor(self.plotter, leader_name):
                        self.plotter.add_mesh(pv.Line(anchor, point), color=d.color, line_width=1, name=leader_name,
                                              render=False)
                    new_names.add(leader_name)
            new_cache[d.sid] = (key, d.color, poly, outline, text)
        remove_actors(self.plotter, self._actor_names - new_names)   # incl. the "-labels" actors
        self._actor_names, self._cache = new_names, new_cache
        self._label_pos = {sid: p for sid, p in self._label_pos.items() if sid in new_cache}
        ranges = [d.sec.get("range_km") for d in self._sectors if isinstance(d.sec.get("range_km"), (int, float))]
        set_km_axes(self.plotter, max(ranges, default=None) if true_scale else None)

    def _on_scale_changed(self, true_scale: bool) -> None:
        """1:1 on: redraw the volumes in km and zoom to fit them all; off: back to the default unit-sphere view."""
        self._label_pos.clear()   # dragged label points are in the old coordinate system (km vs. unit sphere)
        if self._eps is not None:
            self._build_sectors()
        if true_scale:
            fit_camera_to_actors(self.plotter, [n for n in self._actor_names if n.endswith("_surf")]
                                 + (KM_AXES_LINES if self.display_bar.scale_axes_visible() else []))
        else:
            set_default_camera(self.plotter, self._r_is if self._icrf_view() else None)
        self._update_background()
        self.apply_visibility()

    def _update_background(self) -> None:
        bar = self.display_bar
        set_background_sphere(self.plotter, bar.background_visible(), bar.altitude_km())
        self.plotter.render()
