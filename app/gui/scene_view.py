#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENE_VIEW -- the 3D coverage view shared by the mission-preparation and results tabs: the display controls (one
switch per face responsibility zone, show/hide every watch volume, 1:1 scale and its graduated km axes, the view's
frame, the sky) in the "3D view" panel of the tab's side column, and a PyVista plotter holding the static ship
scene, the face layer and the watch-sector overlays.
Aligned with the Scenario Editor (user, 10 November 2026, pass V2) through the shared parts of gui/view3d.py: the
black of space and the stars (in the ship frame, only when the mission has a navigation), the options panel in
sections over two columns with "Recenter view" and "UI size", the right-button turntable (the left button left to
the labels' drag and, later, to picking).
A linked scenario (pass S2, gui/scenario_link.py) adds its objects at the instant t: their MIL-STD symbols on the
unit sphere at their direction (in 1:1 scale at their range), their trajectories over t +/- SCN_TRAIL_S in the colour
of their identity, their names, and the data box of the object clicked (name, class, identity, range, range rate,
face).

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
import time
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Set

import numpy as np
import pyvista as pv
from PyQt5.QtCore import QObject, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import QGroupBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget
from pyvistaqt import QtInteractor

from core import geometry as geo
from core.params import P
from gui import controls as uc
from gui import view3d as v3                     # (used inside functions only: view3d imports this module)
from gui.widgets import DecimalSpinBox
from sim import scenario as scn

KM_AXES_PREFIX = "km_axes_"
KM_AXES_LINES = [f"{KM_AXES_PREFIX}{a}" for a in ("X", "Y", "Z")]   # the 3D axis meshes (labels are 2D actors)
ICRF_AXES = (("X", "X ICRF (vernal equinox)"), ("Y", "Y ICRF"), ("Z", "Z ICRF (celestial pole)"))
ICRF_AXIS_NAMES = [f"icrf_axis_{a}" for a, _ in ICRF_AXES]
SECTOR_LABEL_FONT_SIZE = 13   # shared by _build_sectors (draws) and _label_hit (estimates the on-screen box), x UI size
FACE_OUTLINE_COLOR = "#9aa0aa"   # the zones' outlines, light on the black of space (view3d.MUTED_COLOR)
SCN = "scn_"                     # the linked scenario's actors (pass S2)
SCN_TRAIL_S = float(P.get("tool.display.pmiss_scenario_trail_s"))   # its trajectories over t +/- this (user)
SCN_LABEL_FONT, SCN_BOX_FONT = 11, 12   # its names, its data box: pt at UI size 100 %
SCN_PICK = 0.6                   # a click within this x the symbol's size of its centre picks the object
SCN_ARC_DEG = 2.0                # unit sphere: a trajectory's step longer than this is drawn as an arc along it (D)


def _sphere_arcs(u: np.ndarray) -> np.ndarray:
    """A path of unit directions (m, 3) drawn on the unit sphere: every step longer than SCN_ARC_DEG cut along its
    great circle (a straight segment would cross the sphere)."""
    a, b = u[:-1], u[1:]
    ang = np.arccos(np.clip(np.einsum("ij,ij->i", a, b), -1.0, 1.0))
    k = np.maximum(1, np.ceil(np.degrees(ang) / SCN_ARC_DEG)).astype(int)     # the pieces of each step
    step = np.repeat(np.arange(len(k)), k)
    f = (np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k) + 1) / np.repeat(k, k)   # 1/k .. 1 in each step
    w = np.repeat(ang, k)
    s_w = np.sin(w)
    small = s_w < 1e-9
    c0 = np.where(small, 1.0 - f, np.sin((1.0 - f) * w) / np.where(small, 1.0, s_w))   # (slerp; linear when tiny)
    c1 = np.where(small, f, np.sin(f * w) / np.where(small, 1.0, s_w))
    return np.concatenate([u[:1], c0[:, None] * a[step] + c1[:, None] * b[step]])


# ----------------------------------------------------------------------------- display bar
class DisplayBar(QWidget):
    """The view's switches (pass V2, user 10 November 2026: as the Scenario Editor's, in the "3D view" panel of the
    tab's side column, built by SceneView.view_panel; replaces the "Layers" menu over the view, B14)."""
    faces_changed = pyqtSignal()
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()
    scale_changed = pyqtSignal(bool)
    scale_axes_changed = pyqtSignal(bool)
    background_changed = pyqtSignal(bool)
    altitude_changed = pyqtSignal(float)
    view_frame_changed = pyqtSignal(str)
    sky_changed = pyqtSignal()
    scenario_changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cb_faces = []
        for k in range(4):
            cb = uc.Toggle(geo.FACE_NAMES[k], checked=True, color=geo.FACE_COLORS[k],
                           tip=f"Responsibility zone of face {geo.FACE_NAMES[k]}")
            cb.toggled.connect(self.faces_changed)
            self.cb_faces.append(cb)
        self.bt_show_all = QPushButton("Show all")
        self.bt_show_all.clicked.connect(lambda _c=False: self.show_all_sectors.emit())
        self.bt_hide_all = QPushButton("Hide all")
        self.bt_hide_all.clicked.connect(lambda _c=False: self.hide_all_sectors.emit())
        self.cb_true_scale = uc.Toggle("scale 1:1", tip="Draw each watch volume at its true size (1 unit = 1 km), "
                                       "from the radar out to its required range, and zoom to fit them all. Off: the "
                                       "default view of the unit sphere.")
        self.cb_true_scale.toggled.connect(self.scale_changed)
        self.cb_background = uc.Toggle("Moon background", tip="A background sphere the size of the Moon (1:1 scale "
                                       "only), at its true distance: Moon radius + the altitude, along -Z.")
        self.cb_background.setEnabled(False)   # only meaningful in 1:1 scale
        self.cb_background.toggled.connect(self.background_changed)
        self.cb_true_scale.toggled.connect(self.cb_background.setEnabled)
        self.sb_altitude = DecimalSpinBox()
        self.sb_altitude.setRange(0.0, 1.0e7)
        self.sb_altitude.setDecimals(1)
        self.sb_altitude.setValue(100.0)
        self.sb_altitude.setMaximumWidth(90)
        self.sb_altitude.valueChanged.connect(self.altitude_changed)
        self.cb_scale_axes = uc.Toggle("scale axes", checked=True,
                                       tip="The X, Y, Z axes graduated in km (1:1 scale only)")
        self.cb_scale_axes.setEnabled(False)   # only meaningful in 1:1 scale
        self.cb_scale_axes.toggled.connect(self.scale_axes_changed)
        self.cb_true_scale.toggled.connect(self.cb_scale_axes.setEnabled)
        self.tg_frame = uc.Toggle("ICRF", off_text="ship", two_way=True,
                                  tip="Ship: the ship frame. ICRF: the camera in the inertial frame of the mission "
                                      "navigation (attitude), with the ICRF axes; the ship appears with its attitude")
        self.tg_frame.toggled.connect(lambda on: self.view_frame_changed.emit("ICRF" if on else "ship"))
        self.tg_sky = uc.Toggle("stars", checked=True, tip="The stars, in the ship frame of the mission navigation "
                                "(only with a navigation)")
        self.tg_sky.toggled.connect(lambda _on: self.sky_changed.emit())
        self.cb_const = uc.Toggle("constellations", tip="The constellations' lines over the stars")
        self.cb_const.toggled.connect(lambda _on: self.sky_changed.emit())
        self.tg_scn_objects = uc.Toggle("objects", checked=True, tip="The linked scenario's objects at t: their "
                                        "symbols at their direction (1:1 scale: at their range); a click on one: "
                                        "its data box")
        self.tg_scn_trails = uc.Toggle("trajectories", checked=True,
                                       tip=f"Their trajectories over t +/- {SCN_TRAIL_S:g} s, in their identity's colour")
        self.tg_scn_labels = uc.Toggle("labels", tip="Their names")
        for tg in (self.tg_scn_objects, self.tg_scn_trails, self.tg_scn_labels):
            tg.toggled.connect(lambda _on: self.scenario_changed.emit())
        self.set_scenario_enabled(False)

    def set_scenario_enabled(self, on: bool) -> None:
        """The scenario's switches: only with a linked scenario."""
        for tg in (self.tg_scn_objects, self.tg_scn_trails, self.tg_scn_labels):
            tg.setEnabled(on)

    def sections(self) -> list:
        """(title, items) of the view's options (view3d.options_grid)."""
        return [(v3.title("Responsibility zones"), [v3.row(cb) for cb in self.cb_faces]),
                (v3.title("Mission volumes"), [v3.row(self.bt_show_all, self.bt_hide_all)]),
                (v3.title("Scale"), [v3.row(self.cb_true_scale), v3.row(self.cb_scale_axes), v3.row(self.cb_background),
                                     v3.row(uc.labelled("altitude (km)", self.sb_altitude))]),
                (v3.title("Frame and sky"), [v3.row(self.tg_frame), v3.row(self.tg_sky), v3.row(self.cb_const)]),
                (v3.title("Scenario"), [v3.row(self.tg_scn_objects), v3.row(self.tg_scn_trails),
                                        v3.row(self.tg_scn_labels)])]

    def face_visible(self, k: int) -> bool:
        return self.cb_faces[k].isChecked()

    def true_scale(self) -> bool:
        return self.cb_true_scale.isChecked()

    def scale_axes_visible(self) -> bool:
        """Graduated km axes shown: 1:1 scale on and the "scale axes" switch on."""
        return self.cb_true_scale.isChecked() and self.cb_scale_axes.isChecked()

    def background_visible(self) -> bool:
        """Background (Moon-size) sphere shown: 1:1 scale on and the "Moon background" switch on."""
        return self.cb_true_scale.isChecked() and self.cb_background.isChecked()

    def altitude_km(self) -> float:
        return self.sb_altitude.value()

    def icrf(self) -> bool:
        return self.tg_frame.isChecked()


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


def _ui(plotter, size: float) -> int:
    """A font size at the view's UI size (view3d.UiSize)."""
    return max(1, round(size * getattr(plotter, "_ui_scale", 1.0)))


def add_static_labels(plotter) -> None:
    """The labels of the static scene (axes, face normals) at the view's UI size; again at each UI size change (the
    same names: replaced)."""
    for ax in geo.build_axes():
        plotter.add_point_labels([ax["label_pos"]], [ax["text"]], text_color=ax["color"], font_size=_ui(plotter, 12),
                                 shape=None, show_points=False, name=f"lbl_axis_{ax['text']}", render=False)
    for n in geo.build_face_normals():
        plotter.add_point_labels([n["label_pos"]], [n["name"]], text_color=n["color"], font_size=_ui(plotter, 14),
                                 shape=None, show_points=False, name=f"lbl_normal_{n['k']}", render=False)


def add_static_scene(plotter) -> None:
    """The parts of the scene that never change (axes, hull, face normals, equator)."""
    for ax in geo.build_axes():
        if not ax.get("no_line"):
            plotter.add_mesh(ax["line"], color=ax["color"], line_width=2, name=f"axis_{ax['text']}", render=False)
    ship = geo.build_ship()
    for i, part in enumerate(ship["hull"]):
        plotter.add_mesh(part["mesh"], color=part["color"], smooth_shading=True, name=f"hull_{i}", render=False,
                         silhouette={"color": "#7b8794", "line_width": 1})   # off-white hull on the white background
    for d in ship["arrays"]:
        plotter.add_mesh(d["mesh"], color=d["color"], name=f"hull_array_{d['k']}", render=False)
    for n in geo.build_face_normals():
        plotter.add_mesh(n["mesh"], color=n["color"], line_width=3, name=f"normal_{n['k']}", render=False)
    add_static_labels(plotter)
    ang = np.linspace(0, 2 * math.pi, 181)
    eq_pts = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], axis=1)
    plotter.add_mesh(pv.lines_from_points(eq_pts), color=geo.COLORS["muted"], opacity=0.45,
                     line_width=1, name="equator", render=False)


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
        plotter.add_point_labels([tuple(tip * 1.08)], [text], text_color=geo.COLORS["muted"], font_size=_ui(plotter, 11),
                                 shape=None, show_points=False, name=name + "_label", render=False)


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    k = axis / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx)


CLICK_DRAG_PX = 4.0                           # px: a press and release this close: a click, not a drag
_TURN: Dict[int, dict] = {}                   # per turntable style: its plotter, rate, axis and drag state


TURN_SMOOTH_S = 1e-3 * float(P.get("tool.display.mouse_turn_smooth_ms"))   # mouse rotation's glide (user)
TURN_TICK_MS = 8                              # ms: the glide's timer, about 120 turns a second (finer than the mouse's)
TURN_REST_PX = 0.02                           # px: a glide left with less than this to go ends there


def _turn_move(style, _ev) -> None:
    """The drag's pixels go to the glide (user, 8 November 2026: the view turned by whole-pixel jumps at each mouse
    event, a jagged motion): each tick turns the camera by its share of what is left, in fractions of a pixel."""
    st = _TURN[id(style)]
    if st["p"] is None:
        style.OnMouseMove()
        return
    x1, y1 = style.GetInteractor().GetEventPosition()
    dx, dy = x1 - st["p"][0], y1 - st["p"][1]
    if not dx and not dy:
        return
    st["p"] = (x1, y1)
    if st["smooth"] <= 0.0:                                             # no glide: at once, as before
        _turn_apply(style, dx, dy)
        return
    st["todo"][0] += dx
    st["todo"][1] += dy
    if not st["timer"].isActive():
        st["t"] = time.perf_counter()
        st["timer"].start()


def _turn_tick(style) -> None:
    st = _TURN.get(id(style))
    if st is None:
        return
    now = time.perf_counter()
    dt, st["t"] = now - st["t"], now
    todo = st["todo"]
    k = 1.0 - math.exp(-max(dt, 1e-4) / st["smooth"])                  # frame-rate independent share
    if math.hypot(*todo) * (1.0 - k) < TURN_REST_PX:                   # the end of the glide: what is left
        k = 1.0
        st["timer"].stop()
    dx, dy = todo[0] * k, todo[1] * k
    todo[0] -= dx
    todo[1] -= dy
    _turn_apply(style, dx, dy)


def flush_turn(style) -> None:
    """The glide finished at once (tests)."""
    st = _TURN[id(style)]
    st["timer"].stop()
    dx, dy = st["todo"]
    st["todo"] = [0.0, 0.0]
    if dx or dy:
        _turn_apply(style, dx, dy)


def _turn_apply(style, dx: float, dy: float) -> None:
    st = _TURN[id(style)]
    plotter = st["plotter"]
    cam = plotter.renderer.GetActiveCamera()
    f, pos = np.asarray(cam.GetFocalPoint()), np.asarray(cam.GetPosition())
    back = pos - f
    up = np.asarray(cam.GetViewUp())
    z = back / np.linalg.norm(back)
    y = up - (up @ z) * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)                                                  # screen x right, y up, z to the viewer
    a = np.asarray(st["axis"](), float)
    a = a / np.linalg.norm(a)
    from_pole = math.acos(float(np.clip(z @ a, -1.0, 1.0)))
    tilt = float(np.clip(dy * st["rate"], -from_pole, math.pi - from_pole))   # stops at the poles
    rot = _rotation(a, -dx * st["rate"]) @ _rotation(x, tilt)           # the scene follows the hand
    cam.SetPosition(*(f + rot @ back))
    cam.SetViewUp(*(rot @ y))
    cam.OrthogonalizeViewUp()
    plotter.renderer.ResetCameraClippingRange()
    style.GetInteractor().Render()


def install_turntable(plotter, deg_per_px: float, button: str = "left", axis=lambda: (0.0, 0.0, 1.0),
                      on_click=None):
    """Left-drag (button "right": right-drag) rotation as a turntable (user, 30 October 2026, replaces the virtual
    trackball, whose drags rolled the view away from its centre and whose rate depended on the place of the press): a
    horizontal drag turns the azimuth about axis() (the view's own vertical), a vertical one the elevation about the
    screen's horizontal, stopped at the poles of that axis; deg_per_px everywhere in the view. A level view stays
    level. The camera turns about its focal point (the pivot of lock_pivot). Built on pyvista's own trackball style
    (its capture of the release events and the observers pyvista puts on the style stay valid: install it BEFORE
    adding them); Shift / Ctrl + left drag keep VTK's pan / spin; the wheel, the right button and the interactor's
    observers (clicks) are unchanged.
    button "right" (the scenario tool, user 29 October 2026: the left button already picks and centres): the right
    drag turns the view (no more right-drag zoom: the wheel zooms), a plain left drag does nothing to the camera
    (Shift / Ctrl + left drag still pan / spin). on_click(x, y), button "right": the right button released where it was
    pressed, a click and not a drag (user, 1 November 2026)."""
    from pyvista.plotting.render_window_interactor import InteractorStyleTrackballCamera

    right = button == "right"

    class _Turntable(InteractorStyleTrackballCamera):
        def _press(self, *a):
            iren = self.GetInteractor()
            if iren.GetShiftKey() or iren.GetControlKey():
                super()._press(*a)
                return
            if not right:
                self._start()

        def _release(self, *a):
            st = _TURN[id(self)]
            if st["p"] is None:
                if not right or st["vtk"]:
                    super()._release(*a)
                return
            if not right:
                self._end()

        def _start(self):
            st = _TURN[id(self)]
            st["p"] = st["p0"] = self.GetInteractor().GetEventPosition()
            self.StartRotate()

        def _end(self):
            st = _TURN[id(self)]
            if st["p"] is not None:
                st["p"] = None
                self.EndRotate()
                x, y = self.GetInteractor().GetEventPosition()
                if on_click is not None and math.hypot(x - st["p0"][0], y - st["p0"][1]) <= CLICK_DRAG_PX:
                    on_click(x, y)

    style = _Turntable(plotter.iren)
    timer = QTimer(plotter if isinstance(plotter, QObject) else None)   # (kept in _TURN either way)
    timer.setInterval(TURN_TICK_MS)
    timer.setTimerType(Qt.PreciseTimer)
    timer.timeout.connect(lambda: _turn_tick(style))
    _TURN[id(style)] = {"plotter": plotter, "rate": math.radians(deg_per_px), "axis": axis, "p": None, "vtk": True,
                        "smooth": TURN_SMOOTH_S, "todo": [0.0, 0.0], "timer": timer, "t": 0.0}
    if right:
        # observers on the style's right-button events replace VTK's own (the right-drag zoom)
        style.add_observer("RightButtonPressEvent", lambda *_a: style._start())
        style.add_observer("RightButtonReleaseEvent", lambda *_a: style._end())
    style.add_observer("MouseMoveEvent", _turn_move)
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
    key = None if key is None else key + (getattr(plotter, "_ui_scale", 1.0),)   # (rebuilt at a new UI size)
    if key == getattr(plotter, "_km_axes_key", None):
        return
    remove_actors(plotter, [k for k in plotter.actors if k.startswith(KM_AXES_PREFIX)])
    plotter._km_axes_key = key
    if key is None:
        return
    step, n = key[:2]
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
                                 text_color=color, font_size=_ui(plotter, 10), shape=None, show_points=False,
                                 name=f"{KM_AXES_PREFIX}ticks_{tag}", render=False)
        plotter.add_point_labels([d * length * 1.08], [name], text_color=color, font_size=_ui(plotter, 13), bold=True,
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
        if (not key.startswith(("sec_", "face_", KM_AXES_PREFIX, BACKGROUND_ACTOR, SCN))
                and key not in getattr(plotter, "_managed_keys", ())):
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
    """Plotter + its "3D view" panel (view_panel, placed by the owning tab in its side column). Re-exposes the bar's
    show/hide-all buttons; everything else (faces, 1:1 scale, km axes, frame, sky, UI size) is handled here."""
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
        self.display_bar.sky_changed.connect(self._update_sky)
        self.display_bar.scenario_changed.connect(lambda: (self._scn_visibility(), self.plotter.render()))
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(500)
        self.plotter.interactor.setToolTip("Right drag: turn the view. Drag a label to reposition it (a line keeps it "
                                           "linked to its volume); double-click a moved label to snap it back.")
        v.addWidget(self.plotter.interactor)
        self.ui_size = v3.UiSize(self)                 # the view's icons and texts (shared: gui/view3d.py)
        self.ui_size.changed.connect(self._ui_scale_changed)
        self.plotter._ui_scale = self.ui_size.scale
        keys = set(self.plotter.actors)
        self.sky = v3.space_background(self.plotter)   # the black of space, the stars under the scene
        self.plotter._managed_keys = set(self.plotter.actors) - keys   # (actors shown by their own rules)
        add_static_scene(self.plotter)
        # before the label drag: its release observer goes on the style
        v3.install_rotation(self.plotter, axis=self._turn_axis)
        set_default_camera(self.plotter)
        self.bt_recentre = v3.recenter_button(self.recentre, "Back to the default view (unit sphere), or in 1:1 scale "
                                              "to the view of every volume; in the frame chosen")
        self._panel_rows: List[QWidget] = []

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
        self._update_sky()
        # the linked scenario (pass S2)
        self._scn_link = None                          # gui/scenario_link.ScenarioLink
        self._scn_t = 0.0
        self._scn_ident: List[str] = []                # each object's identity
        self._scn_rows = np.zeros(0, int)              # the objects shown (present at t)
        self._scn_q = np.zeros((0, 3))                 # their places in the scene
        self._scn_state: Optional[tuple] = None        # (range km, range rate m/s, face) of each object at t
        self._scn_sel: Optional[int] = None            # the object whose data box is shown
        self._press_xy: Optional[tuple] = None
        self._scn_sym = v3.SymbolLayer(self.plotter, SCN + "sym_")
        self._scn_lbl = self._managed(v3.Labels(self.plotter, v3.TEXT_COLOR, SCN_LABEL_FONT))
        self._scn_box = self._managed(v3.Labels(self.plotter, v3.TEXT_COLOR, SCN_BOX_FONT, box=True))
        self.plotter.camera.AddObserver("ModifiedEvent", lambda *_a: self._scn_place())
        self._scn_visibility()

    def _managed(self, lab: "v3.Labels") -> "v3.Labels":
        """A labels actor shown by its own rule (not by the reference scene's visibility)."""
        lab.actor.SetVisibility(False)
        self.plotter._managed_keys |= {k for k, a in self.plotter.actors.items() if a is lab.actor}
        return lab

    # ------------------------------------------------------------------ the linked scenario (pass S2)
    def set_scenario(self, link, t: float = 0.0) -> None:
        """The scenario's objects at t (link: gui/scenario_link.ScenarioLink, None: none)."""
        if link is not self._scn_link:
            self._scn_sel = None
            self._scn_ident = [] if link is None else [
                scn.identity_of(link.sc, oid, link.truth.group_of[j + 1] or None) for j, oid in enumerate(link.g.ids)]
        self._scn_link, self._scn_t = link, float(t)
        self.display_bar.set_scenario_enabled(link is not None)
        self._draw_scenario()
        self.plotter.render()

    def _scn_at(self, t: float) -> tuple:
        """(present, unit directions, range km, range rate m/s, face) of every object at t, linear between the
        run's records (the face of the nearer one)."""
        g = self._scn_link.g
        ts = g.t_s
        if len(ts) < 2:
            i, w = 0, 0.0
            j = 0
        else:
            i = int(np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 2))
            w = float(np.clip((t - ts[i]) / max(ts[i + 1] - ts[i], 1e-12), 0.0, 1.0))
            j = i + 1
        present = g.exists[i] & g.exists[j] if w > 0.0 else g.exists[i].copy()
        d = (1.0 - w) * g.dir_ship[i] + w * g.dir_ship[j]
        d = d / np.maximum(np.linalg.norm(d, axis=1), 1e-300)[:, None]
        rng = (1.0 - w) * g.range_km[i] + w * g.range_km[j]
        rr = (1.0 - w) * g.range_rate_mps[i] + w * g.range_rate_mps[j]
        face = g.face[i if w < 0.5 else j]
        return present & np.all(np.isfinite(d), axis=1), d, rng, rr, face

    def _scn_xyz(self, d: np.ndarray, rng: np.ndarray) -> np.ndarray:
        """Places in the scene: on the unit sphere at the direction, in 1:1 scale at the range (km)."""
        return d * np.asarray(rng)[..., None] if self.display_bar.true_scale() else d

    def _draw_scenario(self) -> None:
        link = self._scn_link
        if link is None:
            self._scn_rows, self._scn_q, self._scn_state = np.zeros(0, int), np.zeros((0, 3)), None
            self._scn_sym.set(np.zeros((0, 3)), [])
            remove_actors(self.plotter, [SCN + "trails"])
            self._scn_visibility()
            return
        g = link.g
        present, d, rng, rr, face = self._scn_at(self._scn_t)
        rows = np.flatnonzero(present)
        self._scn_rows, self._scn_state = rows, (rng, rr, face)
        self._scn_q = self._scn_xyz(d[rows], rng[rows])
        self._scn_sym.set(self._scn_q, [v3.symbol_path(self._scn_ident[j], g.classes[j]) for j in rows])
        # the trajectories over t +/- SCN_TRAIL_S: the records in the window and both ends, where present
        ts = g.t_s
        lo, hi = self._scn_t - SCN_TRAIL_S, self._scn_t + SCN_TRAIL_S
        ks = np.flatnonzero((ts > lo) & (ts < hi))
        e0, e1 = self._scn_at(max(lo, ts[0])), self._scn_at(min(hi, ts[-1]))
        ex = np.vstack([e0[0][None], g.exists[ks], e1[0][None]])                   # (m, n)
        dd = np.concatenate([e0[1][None], g.dir_ship[ks], e1[1][None]])             # (m, n, 3)
        rr_ = np.vstack([e0[2][None], g.range_km[ks], e1[2][None]])
        ex = ex & np.all(np.isfinite(dd), axis=2)
        dd = dd / np.maximum(np.linalg.norm(dd, axis=2), 1e-300)[..., None]
        xyz = self._scn_xyz(dd, rr_)
        pts, lines, cols = [], [], []
        n = 0
        for j in np.flatnonzero(ex.sum(axis=0) >= 2):                             # its runs of presence
            edges = np.diff(np.concatenate([[0], ex[:, j].astype(np.int8), [0]]))
            rgb = v3.rgb(v3.IDENTITY_COLORS.get(self._scn_ident[j], "#888888"))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                if b - a < 2:
                    continue
                seg = xyz[a:b, j] if self.display_bar.true_scale() else _sphere_arcs(xyz[a:b, j])
                pts.append(seg)
                lines.append(np.r_[len(seg), np.arange(n, n + len(seg))])
                cols.append(np.tile(rgb, (len(seg), 1)))
                n += len(seg)
        if lines:
            poly = pv.PolyData(np.concatenate(pts), lines=np.concatenate(lines))
            poly["rgb"] = np.concatenate(cols).astype(np.uint8)
            self.plotter.add_mesh(poly, scalars="rgb", rgb=True, line_width=2, name=SCN + "trails", render=False)
        else:
            remove_actors(self.plotter, [SCN + "trails"])
        if self._scn_sel is not None and self._scn_sel not in set(rows.tolist()):
            self._scn_sel = None                       # (no longer present at t)
        self._scn_place()
        self._scn_visibility()

    def _scn_box_text(self, j: int) -> str:
        g = self._scn_link.g
        rng, rr, face = self._scn_state
        f = int(face[j])
        return "\n".join([g.ids[j], f"{g.classes[j]}, {self._scn_ident[j]}", f"range {rng[j]:.1f} km",
                          f"range rate {rr[j]:+.0f} m/s", f"face {geo.FACE_NAMES[f] if 0 <= f < 4 else 'none'}"])

    def _scn_place(self) -> None:
        """After a camera, scale or UI size change: the symbols facing the camera, the names right of them, the data
        box at the selected symbol's top right corner."""
        if not len(self._scn_q):
            return
        self._scn_sym.place()
        corners = v3.billboard(self.plotter, self._scn_q, v3.SYMBOL_PX * getattr(self.plotter, "_ui_scale", 1.0))
        corners = corners.reshape(-1, 4, 3)
        g = self._scn_link.g
        self._scn_lbl.set(0.5 * (corners[:, 1] + corners[:, 2]), [g.ids[j] for j in self._scn_rows])
        if self._scn_sel is not None:
            k = int(np.flatnonzero(self._scn_rows == self._scn_sel)[0])
            self._scn_box.set(corners[k, 2][None], [self._scn_box_text(self._scn_sel)])

    def _scn_visibility(self) -> None:
        bar, on = self.display_bar, self._scn_link is not None
        objects = on and bar.tg_scn_objects.isChecked() and len(self._scn_q) > 0
        self._scn_sym.set_visible(objects)
        self._scn_lbl.actor.SetVisibility(objects and bar.tg_scn_labels.isChecked())
        self._scn_box.actor.SetVisibility(objects and self._scn_sel is not None)
        set_actors_visible(self.plotter, [SCN + "trails"], on and bar.tg_scn_trails.isChecked())

    def _pick_scenario(self, x: int, y: int) -> None:
        """A left click on an object's symbol: its data box (elsewhere: none)."""
        if self._scn_link is None or not self.display_bar.tg_scn_objects.isChecked() or not len(self._scn_q):
            return
        ren = self.plotter.renderer
        best, best_d = None, SCN_PICK * v3.SYMBOL_PX * getattr(self.plotter, "_ui_scale", 1.0)
        fwd = np.asarray(self.plotter.camera.GetDirectionOfProjection())
        pos = np.asarray(self.plotter.camera.GetPosition())
        for k, q in enumerate(self._scn_q):
            if (q - pos) @ fwd <= 0.0:
                continue                                # (behind the camera)
            ren.SetWorldPoint(*q, 1.0)
            ren.WorldToDisplay()
            dx, dy, _dz = ren.GetDisplayPoint()
            dist = float(np.hypot(dx - x, dy - y))
            if dist <= best_d:
                best, best_d = int(self._scn_rows[k]), dist
        if best != self._scn_sel:
            self._scn_sel = best
            self._scn_place()
            self._scn_visibility()
            self.plotter.render()

    # ------------------------------------------------------------------ "3D view" panel (pass V2)
    def add_panel_row(self, w: QWidget) -> None:
        """A line of the owner's at the top of the panel (e.g. the results' "Colour by"); before view_panel()."""
        self._panel_rows.append(w)

    def view_panel(self) -> QGroupBox:
        """The view's options, as the Scenario Editor's (gui/view3d.py): Recenter view, UI size, the switches in
        sections over two columns. Built once; the owning tab puts it at the top of its side column."""
        if getattr(self, "_view_panel", None) is not None:
            return self._view_panel
        box = QGroupBox("3D view")
        col = QVBoxLayout(box)
        col.setContentsMargins(8, 4, 8, 6)
        col.setSpacing(4)
        head = QHBoxLayout()
        head.addStretch(1)
        head.addWidget(self.bt_recentre)
        col.addLayout(head)
        for w in self._panel_rows:
            col.addWidget(v3.row(w))
        col.addWidget(self.ui_size.widget())
        col.addLayout(v3.options_grid(self.display_bar.sections()))
        self._view_panel = box
        return box

    def recentre(self) -> None:
        """Back to the default view in the frame chosen (unit sphere), or in 1:1 scale to the view of every volume."""
        if self.display_bar.true_scale():
            fit_camera_to_actors(self.plotter, [n for n in self._actor_names if n.endswith("_surf")]
                                 + (KM_AXES_LINES if self.display_bar.scale_axes_visible() else []))
        else:
            set_default_camera(self.plotter, self._r_is if self._icrf_view() else None)
        self.plotter.render()

    def _ui_scale_changed(self, scale: float) -> None:
        """The labels at the new UI size: the static ones, the ICRF axes, the km axes, the sectors'."""
        v3.apply_ui_scale(self.plotter, scale)
        add_static_labels(self.plotter)
        set_icrf_axes(self.plotter, self._r_is if self._icrf_view() else None)
        if self._eps is not None:
            ranges = [d.sec.get("range_km") for d in self._sectors if isinstance(d.sec.get("range_km"), (int, float))]
            set_km_axes(self.plotter, max(ranges, default=None) if self.display_bar.true_scale() else None)
        for sid in list(self._cache):
            self._redraw_label(sid)                    # (renders through apply_visibility)
        self._scn_place()
        self.apply_visibility()

    def _update_sky(self) -> None:
        """The stars in the ship frame of the mission navigation (they turn with its attitude); none without one."""
        bar, nav = self.display_bar, self._r_is is not None
        bar.tg_sky.setEnabled(nav)
        bar.cb_const.setEnabled(nav)
        self.sky.set_rotation(self._r_is)
        self.sky.set_visible(nav and bar.tg_sky.isChecked(), bar.cb_const.isChecked())
        self.plotter.render()

    # ------------------------------------------------------------------ view frame (step 3.0)
    def set_navigation(self, r_si: Optional[np.ndarray]) -> None:
        """Attitude of the mission navigation (ship -> ICRF), None if invalid; the ICRF view follows it."""
        r_is = None if r_si is None else np.asarray(r_si).T
        same = (r_is is None and self._r_is is None) or (
            r_is is not None and self._r_is is not None and np.array_equal(r_is, self._r_is))
        self._r_is = r_is
        self.display_bar.tg_frame.setEnabled(r_is is not None)
        if not same and self.display_bar.icrf():
            self._apply_view_frame()
        if not same:
            self._update_sky()

    def _icrf_view(self) -> bool:
        return self.display_bar.icrf() and self._r_is is not None

    def _turn_axis(self) -> np.ndarray:
        """The vertical of the mouse rotation: the one of the default view (set_default_camera)."""
        return self._r_is[:, 2] if self._icrf_view() else np.array([0.0, 0.0, 1.0])

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
            font = _ui(self.plotter, SECTOR_LABEL_FONT_SIZE)
            w = 10 + max(len(ln) for ln in lines) * font * 0.56
            h = 10 + len(lines) * font * 1.3
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
        self._press_xy = (x, y)                       # (a click without a drag: picking, pass S2)
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
        dragged, self._drag = self._drag is not None, None
        if not dragged and self._press_xy is not None:
            x, y = self.plotter.iren.get_event_position()
            if np.hypot(x - self._press_xy[0], y - self._press_xy[1]) <= CLICK_DRAG_PX:
                self._pick_scenario(x, y)
        self._press_xy = None

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
        self.plotter.add_point_labels([point], [text], name=lbl_name, text_color="white",
                                      font_size=_ui(self.plotter, SECTOR_LABEL_FONT_SIZE),
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
        if hasattr(self, "_scn_sym"):
            self._scn_visibility()
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
                self.plotter.add_mesh(scene.outlines[k], color=FACE_OUTLINE_COLOR, line_width=1, name=out_name,
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
                font = _ui(self.plotter, SECTOR_LABEL_FONT_SIZE)
                if rebuild_label:
                    # Fixed white-on-near-opaque-black readout: text_color=d.color (the sector's own colour, e.g.
                    # green) used to sit on a translucent grey box over the same-coloured sector mesh -- unreadable.
                    self.plotter.add_point_labels([point], [text], name=lbl_name,
                                                  text_color="white", font_size=font, bold=True,
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
        if self._scn_link is not None:
            self._draw_scenario()                      # (unit sphere / km)
        self._update_background()
        self.apply_visibility()

    def _update_background(self) -> None:
        bar = self.display_bar
        set_background_sphere(self.plotter, bar.background_visible(), bar.altitude_km())
        self.plotter.render()
