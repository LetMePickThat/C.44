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
"global" (user, 10 November 2026: where the volumes lie in real space): the Scenario Editor's view (gui/sim_view.py,
read only, no playback bar: the instant is the mission's) in place of the sphere, its Info and options column beside
it: the central body, the orbits, the objects, the ship, and the mission's volumes at their true size from the ship's
place at t, turned by its attitude at t.

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

import json
import math
import time
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Set

import numpy as np
import pyvista as pv
from PyQt5.QtCore import QObject, QRectF, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider, QSplitter,
                             QStackedWidget, QVBoxLayout, QWidget)
from pyvistaqt import QtInteractor

from core import frames
from core import geometry as geo
from core.params import P
from gui import controls as uc
from gui import view3d as v3                     # (used inside functions only: view3d imports this module)
from gui.widgets import DecimalSpinBox
from sim import scenario as scn

ICRF_AXES = (("X", "X (node of the equator)"), ("Y", "Y"), ("Z", "Z (pole)"))   # the view frame's axes (H3)
ICRF_AXIS_NAMES = [f"icrf_axis_{a}" for a, _ in ICRF_AXES]
LABEL_ASKED = "#a8aeb8"        # the 3D label's first line: what is asked (grey; user, 11 November 2026)
PLAYED_FULL_SHARE = 0.995                     # "show effectively played": a volume served at least this much hidden
LABEL_OK, LABEL_NOK = "#1e7d45", "#e0574f"   # its second line: the worst range and the revisit achieved
                                               # (green darkened 12 Nov 2026, user: unreadable on the gauge's green)
SECTOR_LABEL_FONT_SIZE = 13   # shared by _build_sectors (draws) and _label_hit (estimates the on-screen box), x UI size
DIAGNOSIS_START_PX = 230       # the live load + Diagnosis panel's height at start (5.5 A; then set with the mouse)
FACE_OUTLINE_COLOR = "#9aa0aa"   # the zones' outlines, light on the black of space (view3d.MUTED_COLOR)
SCN = "scn_"                     # the linked scenario's actors (pass S2)
PLAY_SPEEDS = (1, 10, 100, 1000)                 # the playback's speeds (as the Scenario Editor's first ones)
PLAY_HEAVY_S = 2.0                               # light playback (user, 10 November 2026): the volumes, the gauge and the
#   live load rebuilt at most this often while playing (D), at once on pause; never during a slider drag (on release)
PLAY_TICK_MS = 200                               # one displayed instant at most this often (the live load: ~50 ms per volume)
LOAD_OK, LOAD_HIGH, LOAD_OVER = "#3cb371", "#e08a1e", "#c0392b"   # the live load: under the ceiling, at it, above 100 %
LOAD_BEYOND = "#6e0f0f"          # the energy bar's part beyond 100 % of the face reference, drawn over it again (user)
LOAD_LENT = "#8a8f99"            # the hatching of the power a face lends to the others (user, 10 November 2026)
SCN_TRAIL_S = float(P.get("tool.display.pmiss_scenario_trail_s"))   # its trajectories over t +/- this (user)
SCN_LABEL_FONT, SCN_BOX_FONT = 11, 12   # its names, its data box: pt at UI size 100 %
SCN_PICK = 0.6                   # a click within this x the symbol's size of its centre picks the object
SCN_ARC_DEG = 2.0                # unit sphere: a trajectory's step longer than this is drawn as an arc along it (D)


def _anchor(poly: pv.PolyData, outline: pv.PolyData) -> np.ndarray:
    """A sector label's anchor: the middle of its executed volumes, else of its outline (a volume all hole)."""
    pts = poly.points if poly.n_points else outline.points
    return pts.mean(axis=0) if len(pts) else np.zeros(3)


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
def _key_sky(sky: Optional[dict]) -> Optional[str]:
    """The sky as a cache key (the ground rule of the gauge depends on it)."""
    if not sky:
        return None
    return json.dumps(sky, sort_keys=True, default=lambda o: np.asarray(o, float).round(6).tolist()
                      if isinstance(o, (np.ndarray, np.generic)) else str(o))


PROFILE_POLL_MS = 700                    # option (c): poll of the cost profiles computed in the background

def _energy_text(served_w: float, base_w: float, asked_w: float) -> str:
    """The energy bar's text: the power drawn and the tasks above the always-on; when the rule cannot serve all that
    is asked of the face, the power drawn over the power asked (user, 11 November 2026)."""
    def fmt(w: float) -> str:
        return f"{w / 1e6:.2f} MW" if w >= 1e6 else f"{w / 1e3:.1f} kW"
    if asked_w > served_w * 1.005 + 1.0:
        return f"{fmt(served_w)} / {fmt(asked_w)} asked"
    return f"{fmt(served_w)} ({(served_w - base_w) / 1e3:+.1f})"


LOAD_BASE = "#8a8f99"                            # the always-on part of the energy bar (grey)

# The ESTIMATE badge's names of the background tasks (core/background.status labels): (shown, what it is for), in
# the order of the calculation (user, 13 November 2026: "CALCULATING ESTIMATE - step k/4: <phase>", the figures being
# estimated and the phase running, not an estimate OF the phase). The badge shows the first one running in this
# order (the earliest step), the others counted.
BADGE_ORDER = ("cost profiles", "relief", "redundancy check", "estimate")
BADGE_ACTIVITY = {
    "cost profiles": ("volume costs", "time and energy per position of each volume edited (the generic table is "
                                      "used meanwhile)"),
    "relief": ("overlaps", "the parts of a volume another volume already covers (taken out of its budget)"),
    "redundancy check": ("redundancy check", "the volumes whose need another volume meets"),
    "estimate": ("face loads", "the fast estimate: resources of each face, range and revisit of each volume "
                               "(the figures shown meanwhile are the previous ones)"),
}


def badge_step(label: str) -> str:
    """"step k/4: <phase>" of a background task (its place in the calculation, BADGE_ORDER)."""
    name = BADGE_ACTIVITY.get(label, (label, ""))[0]
    if label in BADGE_ORDER:
        return f"step {BADGE_ORDER.index(label) + 1}/{len(BADGE_ORDER)}: {name}"
    return name

class FaceLoadColumn(QWidget):
    """One face's live load: its name, then one bar per resource (energy, S time, Ku time), each with its own fill,
    its hatched parts (moved shares, power lent), its part beyond 100 % drawn over it in a darker red, the load
    ceiling marked on the time bars (pass R2, user 10 November 2026). The energy bar stacked (user, 11 November 2026:
    the always-on, ~71 kW per face, hid the variation of the tasks): the always-on in grey ("base"), the tasks in the
    bar's colour, the power drawn written on it ("text")."""

    ROW_H = 18

    def __init__(self, k: int, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.k = k
        self.bars: Optional[List[dict]] = None
        self.setMinimumWidth(130)
        self.setMinimumHeight(self.ROW_H * 4 + 2)

    def set_bars(self, bars: Optional[List[dict]], tip: str) -> None:
        self.bars = bars
        self.setToolTip(tip)
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802 -- Qt override
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, False)
        w, rh = self.width(), self.ROW_H
        f = p.font()
        f.setBold(True)
        p.setFont(f)
        p.setPen(QColor(geo.FACE_COLORS[self.k]))
        p.drawText(QRectF(0, 0, w, rh), Qt.AlignLeft | Qt.AlignVCenter, geo.FACE_NAMES[self.k])
        f.setBold(False)
        p.setFont(f)
        x0 = 22.0
        bw = max(10.0, w - x0 - 2)
        text_col = self.palette().color(self.foregroundRole())
        for i, b in enumerate(self.bars or ({"name": n, "val": 0.0} for n in ("E", "S", "Ku"))):
            y = rh * (i + 1) + 1
            r = QRectF(x0, y, bw, rh - 3)
            p.setPen(text_col)
            p.drawText(QRectF(0, y, x0 - 3, rh - 3), Qt.AlignRight | Qt.AlignVCenter, b["name"])
            p.setPen(QPen(QColor("#5a5f69")))
            p.setBrush(QBrush(self.palette().color(self.backgroundRole()).darker(115)))
            p.drawRect(r)
            if self.bars is None:
                continue
            val = max(0.0, float(b["val"]))
            fill = min(val, 1.0)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(b["color"]))
            p.drawRect(QRectF(x0, y, bw * fill, rh - 3))
            base = min(max(float(b.get("base", 0.0)), 0.0), fill)
            if base > 0:                                   # the always-on, under the tasks
                p.setBrush(QColor(LOAD_BASE))
                p.drawRect(QRectF(x0, y, bw * base, rh - 3))
            if val > 1.0:                                  # the part beyond 100 %, drawn over the bar again
                p.setBrush(QColor(LOAD_BEYOND))
                p.drawRect(QRectF(x0, y, bw * min(val - 1.0, 1.0), rh - 3))
            mv = min(float(b.get("moved", 0.0)), fill)
            if mv > 0:                                     # shares moved from a neighbour, served here
                p.setBrush(QBrush(QColor("#1b1d22"), Qt.BDiagPattern))
                p.drawRect(QRectF(x0 + bw * (fill - mv), y, bw * mv, rh - 3))
            ln = min(float(b.get("lent", 0.0)), 1.0 - fill)
            if ln > 0:                                     # power lent to the other faces
                p.setBrush(QBrush(QColor(LOAD_LENT), Qt.FDiagPattern))
                p.drawRect(QRectF(x0 + bw * fill, y, bw * ln, rh - 3))
            if b.get("ceiling"):
                p.setPen(QPen(QColor("#f0f0f0"), 1, Qt.DashLine))
                xc = x0 + bw * min(float(b["ceiling"]), 1.0)
                p.drawLine(int(xc), int(y), int(xc), int(y + rh - 3))
            pct = 100 * val                                # (small loads with decimals: 0.13 %, not 0 %)
            txt = b.get("text") or (f"{pct:.0f} %" if pct >= 10 or pct == 0 else
                                    f"{pct:.1f} %" if pct >= 1 else f"{pct:.2f} %")
            p.setPen(QColor("#000000"))                    # (a shadow: readable on any fill)
            p.drawText(r.translated(1, 1), Qt.AlignCenter, txt)
            p.setPen(QColor("#ffffff"))
            p.drawText(r, Qt.AlignCenter, txt)
        p.end()


class DisplayBar(QWidget):
    """The view's switches (pass V2, user 10 November 2026: as the Scenario Editor's, in the "3D view" panel of the
    tab's side column, built by SceneView.view_panel; replaces the "Layers" menu over the view, B14)."""
    faces_changed = pyqtSignal()
    show_all_sectors = pyqtSignal()
    hide_all_sectors = pyqtSignal()
    view_frame_changed = pyqtSignal(str)
    sky_changed = pyqtSignal()
    scenario_changed = pyqtSignal()
    global_changed = pyqtSignal(bool)
    gauge_changed = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        # U4 (user, 11 November 2026): one switch for the four faces' responsibility zones (was one per face)
        self.tg_zones = uc.Toggle("zones F1-F4", checked=True,
                                  tip="Responsibility zones of the four faces (the part of the sky each face scans)")
        self.tg_zones.toggled.connect(self.faces_changed)
        self._global_was_available = False
        self.bt_show_all = QPushButton("Show all")
        self.bt_show_all.clicked.connect(lambda _c=False: self.show_all_sectors.emit())
        self.bt_hide_all = QPushButton("Hide all")
        self.bt_hide_all.clicked.connect(lambda _c=False: self.hide_all_sectors.emit())
        self.tg_frame = uc.Toggle("equatorial frame",           # (one-sided: aligned with the others, 12 Nov 2026)
                                  tip="Sphere view. Off: the ship frame. On: the camera in the central body's equatorial "
                                      "axes (its pole up, as the Scenario Editor's global view; no orbit: ICRF), "
                                      "with these axes; the ship appears with the attitude of the navigation")
        self.tg_frame.toggled.connect(lambda on: self.view_frame_changed.emit("equatorial" if on else "ship"))
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
        # the orbits of the global view, as the Scenario Editor's (user, 12 November 2026: they could not be hidden);
        # at start: the planned trajectory as the editor's global profile ([tool.display.view_global]); all orbits
        # and the orbital velocity vector off (user, 12 November 2026)
        prof = P.section("tool.display.view_global")
        self.tg_planned = uc.Toggle("planned trajectory", checked=bool(prof.get("planned_trajectory", True)),
                                    tip="Global view: each body's trajectory from t to the run's end")
        self.tg_all_orbits = uc.Toggle("all orbits", checked=False,      # (off at start: user, 12 November 2026)
                                       tip="Global view: the orbits of every planet and moon")
        self.tg_orbit_vec = uc.Toggle("orbital velocity vector", checked=False,   # (off at start: user, 12 Nov)
                                      tip="Global view: each body's free fall from t under the local gravity alone")
        for tg in (self.tg_scn_objects, self.tg_scn_trails, self.tg_scn_labels, self.tg_planned, self.tg_all_orbits,
                   self.tg_orbit_vec):
            tg.toggled.connect(lambda _on: self.scenario_changed.emit())
        self.tg_global = uc.Toggle("global", off_text="sphere", two_way=True,
                                   tip="Sphere: everything projected on the sphere around the ship (zones, volumes, "
                                       "the linked scenario's objects). Global: true size, as the Scenario Editor's "
                                       "views: the central body, the ship on its flight plan (the linked scenario's "
                                       "navigation, else the mission's) and the volumes from it; no object (the "
                                       "plan rests on the flight plan, not on a truth that cannot be known)")
        self.tg_global.toggled.connect(self.global_changed)
        self.tg_gauge = uc.Toggle("range gauge", checked=True, tip="Each watched volume filled in green up to the range held in each "
                                  "direction, in red where it falls short (coverage hole included); capacity: the "
                                  "volume alone on its faces, all their power, shared equally between its beam "
                                  "positions. Sphere: the range over the required one; global: in km")
        self.tg_gauge.toggled.connect(self.gauge_changed)
        self.set_scenario_enabled(False)

    def set_scenario_enabled(self, on: bool) -> None:
        """The scenario's switches: only with a linked scenario."""
        for tg in (self.tg_scn_objects, self.tg_scn_trails, self.tg_scn_labels):
            tg.setEnabled(on)

    def set_global_available(self, on: bool) -> None:
        """The global mode: with a linked scenario or a navigation with an orbit (its flight plan)."""
        self.tg_global.setEnabled(on)
        if not on and self.tg_global.isChecked():
            self.tg_global.setChecked(False)          # (no flight plan: back to the sphere)
        elif on and not self._global_was_available and not self.tg_global.isChecked():
            self.tg_global.setChecked(True)           # U5 (user, 11 November 2026): a flight plan opens in Global
        self._global_was_available = on

    def display_sections(self) -> list:
        """(title, switches) of the options of both modes, in the order of the two columns."""
        return [("Display", [self.tg_gauge, self.tg_zones, self.tg_sky, self.cb_const, self.tg_frame])]

    def scenario_sections(self) -> list:
        """(title, switches) of the linked scenario's options (shown only with a scenario: user, 12 November 2026);
        the orbits: global view only."""
        return [("Scenario objects", [self.tg_scn_objects, self.tg_scn_labels, self.tg_scn_trails]),
                ("Orbits (global view)", [self.tg_planned, self.tg_all_orbits, self.tg_orbit_vec])]

    def face_visible(self, k: int) -> bool:
        return self.tg_zones.isChecked()

    def icrf(self) -> bool:
        return self.tg_frame.isChecked()


# ----------------------------------------------------------------------------- plotter helpers (no render)
def face_actor_names(k: int) -> tuple:
    """Plotter actor names of face k's responsibility zone (surface, outline)."""
    return f"face_surf_{k}", f"face_outline_{k}"


HOLE_COLOR, HOLE_OPACITY = "#8a8a8a", 0.35    # a watched volume's coverage hole on the sphere (E3, D)


def sector_hole_name(sid: str) -> str:
    return f"sec_{sid}_hole"


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
    rerun_requested = pyqtSignal()              # the badge clicked while the estimate is held (user, 13 Nov 2026)
    sector_selected = pyqtSignal(str)           # a line of the diagnosis naming a volume clicked (5.5 A)

    def __init__(self, grid: geo.SphereGrid, face_opacity: float, sector_opacity: float, parent=None,
                 exec_face_colours: bool = False, diagnosis: bool = False):
        """exec_face_colours: the executed volumes in their face's colour (the preparation), else in the sector's (the
        results' "Colour by"). diagnosis: the Diagnosis panel under the live load (the preparation, 5.5 A)."""
        super().__init__(parent)
        self.exec_face_colours = exec_face_colours
        self.grid = grid
        self.face_opacity, self.sector_opacity = face_opacity, sector_opacity
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        self.display_bar = DisplayBar()
        self.display_bar.faces_changed.connect(self.apply_visibility)
        self.display_bar.show_all_sectors.connect(self.show_all_sectors)
        self.display_bar.hide_all_sectors.connect(self.hide_all_sectors)
        self.display_bar.view_frame_changed.connect(lambda _f: self._apply_view_frame())
        self.display_bar.sky_changed.connect(self._update_sky)
        self.display_bar.scenario_changed.connect(lambda: (self._scn_visibility(), self.plotter.render()))
        self.display_bar.scenario_changed.connect(self._on_global_option)
        self.display_bar.faces_changed.connect(self._sync_global_options)
        self.display_bar.sky_changed.connect(self._sync_global_options)
        self.display_bar.global_changed.connect(self._on_global)
        self.display_bar.gauge_changed.connect(self._on_gauge)
        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(500)
        v3.keep_scale_on_resize(self.plotter)          # (user, 13 November 2026: the zoom kept on a resize)
        self.plotter.interactor.setToolTip("Right drag: turn the view. Drag a label to reposition it (a line keeps it "
                                           "linked to its volume); double-click a moved label to snap it back.")
        self._stack = QStackedWidget()                 # the sphere (0) or the global view (1, built when first asked)
        self._stack.addWidget(self.plotter.interactor)
        self.estimate_allowed = lambda: True           # (the preparation: only with a scenario, user 13 Nov 2026)
        self.diagnosis = None                          # (5.5 point A: what limits the mission, from the live load)
        top_v = v
        if diagnosis:                                  # (user, 13 November 2026: its height set with the mouse, as
            top = QWidget()                            # the left column's width) the view and its bars over it
            top_v = QVBoxLayout(top)
            top_v.setContentsMargins(0, 0, 0, 0)
        top_v.addWidget(self._stack, 1)                # (the view takes the free height: 5.5 A, user)
        top_v.addWidget(self._build_play_bar())        # (user, 10 November 2026)
        if diagnosis:
            from gui.diagnosis_panel import DiagnosisPanel
            self.diagnosis = DiagnosisPanel()
            self.diagnosis.sector_selected.connect(self.sector_selected)
            bottom = QWidget()                         # (user, 13 November 2026: the split between the view and the
            bot_v = QVBoxLayout(bottom)                # information under it, above the live load and its badge)
            bot_v.setContentsMargins(0, 0, 0, 0)
            bot_v.setSpacing(0)
            bot_v.addWidget(self._build_load_bar())
            bot_v.addWidget(self.diagnosis, 1)
            self.diagnosis.folded.connect(lambda f, b_=bottom: b_.setMaximumHeight(
                b_.sizeHint().height() if f else 16777215))
            split = QSplitter(Qt.Vertical)
            split.addWidget(top)
            split.addWidget(bottom)
            split.setChildrenCollapsible(False)
            split.setStretchFactor(0, 1)               # (a taller window grows the view, not the panel)
            split.setStretchFactor(1, 0)
            split.setSizes([10000, DIAGNOSIS_START_PX])
            v.addWidget(split)
        else:
            v.addWidget(self._build_load_bar())
        self._sim = None                               # gui/sim_view.SimView, read only
        self._sim_run = None                           # the run it shows
        self._sim_mission: Optional[tuple] = None      # (mission, hidden sector ids) for its volumes
        self.ui_size = v3.UiSize(self)                 # the view's icons and texts (shared: gui/view3d.py)
        self.ui_size.changed.connect(self._ui_scale_changed)
        self.ui_size.changed.connect(self._sync_global_options)
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
        self._r_view: Optional[np.ndarray] = None      # ship <- the view frame (H3: the central body's equator)
        self._sky_ship: Optional[dict] = None          # the mission sky in the ship frame (the ground rule)
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
        self._play_bar.setVisible(link is not None)
        if link is not None and not self._playing and abs(self._play_t - self._scn_t) > 1e-9:
            self._play_t = self._scn_t                 # (the mission's instant changed: the display follows)
            self.sl_play.blockSignals(True)
            self.sl_play.setValue(int(round(10000 * self._scn_t / max(self._play_end(), 1e-9))))
            self.sl_play.blockSignals(False)
            self.lb_play.setText(f"{self._scn_t:8.1f} s  {link.utc(self._scn_t)}")
        if link is None and self._playing:
            self.toggle_play()
        self.display_bar.set_scenario_enabled(link is not None)
        self._show_scn_opts(link is not None)
        self._draw_scenario()
        self.plotter.render()
        self._check_global()
        self._update_badge()                           # (no scenario: no estimate, said on the badge)

    # ------------------------------------------------------------------ the global view
    def _global_shown(self) -> bool:
        return self._sim is not None and self._stack.currentIndex() == 1

    def set_mission_volumes(self, mission: dict, hidden_ids=()) -> None:
        """The mission whose volumes the global view draws (the owner's mission on screen), and the ids of its
        sectors hidden."""
        self._sim_mission = (mission, set(hidden_ids))
        try:                                                # the sky in the ship frame (the ground rule)
            self._sky_ship = frames.resolve(mission)[0].get("global", {}).get("sky")
        except (ValueError, KeyError, TypeError):
            self._sky_ship = None
        self._check_global()

    # ------------------------------------------------------------------ playback and live load (user, 10 November 2026)
    def _build_play_bar(self) -> QWidget:
        """The linked scenario's playback, as the Scenario Editor's bar: it moves the DISPLAY only (views, volumes,
        gauge, live load); "Use this instant" puts the instant shown into the mission (its navigation)."""
        bar = QWidget()
        h = QHBoxLayout(bar)
        h.setContentsMargins(4, 2, 4, 2)
        self.bt_back, self.bt_play, self.bt_fwd = QPushButton("|<"), QPushButton(), QPushButton(">|")
        self.bt_play.setIcon(uc.std_icon(self.bt_play, "SP_MediaPlay"))
        for b in (self.bt_back, self.bt_play, self.bt_fwd):
            b.setFixedWidth(40)
        self.bt_back.clicked.connect(lambda _c=False: self.set_instant(0.0))
        self.bt_fwd.clicked.connect(lambda _c=False: self.set_instant(self._play_end()))
        self.bt_play.clicked.connect(lambda _c=False: self.toggle_play())
        self.cb_speed = QComboBox()
        self.cb_speed.addItems([f"x{s}" for s in PLAY_SPEEDS])
        self.cb_speed.setCurrentIndex(2)
        self.sl_play = QSlider(Qt.Horizontal)
        self.sl_play.setRange(0, 10000)
        self.sl_play.valueChanged.connect(self._on_slider)        # coalesced: the latest position only
        self.sl_play.sliderPressed.connect(self._on_slider_pressed)
        self.sl_play.sliderReleased.connect(self._on_slider_released)
        self.lb_play = QLabel("")
        self.bt_use = QPushButton("Use this instant")
        self.bt_use.setToolTip("Put the instant shown into the mission: its navigation and sky are the ship's there "
                               "(the playback alone moves only the display)")
        self.bt_use.clicked.connect(lambda _c=False: self.use_instant and self.use_instant(self._play_t))
        for w in (self.bt_back, self.bt_play, self.bt_fwd, self.cb_speed):
            h.addWidget(w)
        h.addWidget(self.sl_play, 1)
        h.addWidget(self.lb_play)
        h.addWidget(self.bt_use)
        self._play_t = 0.0
        self._playing = False
        self._play_timer = QTimer(self)
        self._play_timer.setSingleShot(True)            # (rescheduled by _play_tick: the clicks always get their turn)
        self._play_timer.setInterval(PLAY_TICK_MS)
        self._play_timer.timeout.connect(self._play_tick)
        self.instant_source = None                     # t -> (draws, global with nav and sky at t) (the owner)
        self.use_instant = None                        # t -> None: the mission's instant set to t (the owner)
        self.instant_shown = None                      # (t, mission global at t) -> None: the owner's panels follow
        self._play_bar = bar
        bar.setVisible(False)
        return bar

    def _build_load_bar(self) -> QWidget:
        """The live load (pass R2, user 10 November 2026): the final allocation rule on the fast estimate at the
        instant shown; one column per face, one bar per resource (energy: 100 % = the face reference
        power.face_elec_mw, the part beyond drawn again in a darker red over the bar; S time, Ku time: share of the
        time, the load ceiling marked); hatched: on the face serving them, the shares moved from a neighbouring face
        (overlap of the cones), on the energy bar of a face lending power, the part it lends (between its use and
        100 %)."""
        bar = QWidget()
        h = QHBoxLayout(bar)
        h.setContentsMargins(4, 0, 4, 2)
        lb = QLabel("Live load:")
        lb.setToolTip("The final allocation rule at the instant shown (fast estimate, recalibrated by the last exact "
                      "run of each volume when there is one). Energy: 100 % = the face reference (ship pool / 4); "
                      "beyond, power taken from the others' share (darker red). Time: lower bound (energy / peak "
                      "power) for a volume not recalibrated")
        self._load_lbl = lb
        h.addWidget(lb)
        self._load_badge = uc.source_badge("estimate")          # U3: estimate, recalibrated or still computing
        self._load_badge.mousePressEvent = self._on_badge_clicked   # (held estimate: a click reruns it)
        h.addWidget(self._load_badge)
        self._load_cols = {}
        for k in range(4):
            col = FaceLoadColumn(k)
            h.addWidget(col, 1)
            self._load_cols[k] = col
        self._load_status = QLabel("")
        self._load_status.setMinimumWidth(110)
        h.addWidget(self._load_status)
        self._mission_global: dict = {}
        self._calib: dict = {}
        self._achieved: dict = {}                      # sid -> {"worst_km", "rr_s", "ok"} (the labels' 2nd line)
        self._achieved_fixed = False                   # (set by the owner: the exact results, not the live estimate)
        self._load_ceiling = 1.0
        return bar

    def set_achieved(self, achieved: Optional[dict]) -> None:
        """The owner's figures for the labels' second line (the Results: the exact calculation); None: the live
        estimate's."""
        self._achieved_fixed = achieved is not None
        self._achieved = dict(achieved or {})
        self._refresh_labels()

    def label_lines(self, sid: str, sec: dict) -> tuple:
        """(asked, achieved, ok) of a volume's 3D label: 'S01  R=20000 km  RR=30 s' (grey) over 'worst 20286 km  RR
        30 s' (green when the range and the revisit are held, red else); achieved None when not known."""
        asked = (f"{sec.get('id', sid)}  R={float(sec.get('range_km', 0) or 0):.0f} km  "
                 f"RR={float(sec.get('rr_target_s', 0) or 0):g} s")
        a = self._achieved.get(sid)
        if not a:
            return asked, None, True
        return asked, v3.achieved_text(a), bool(a["ok"])

    def _label_text(self, sid: str, asked: str, ach: Optional[str]) -> str:
        """The label's text as cached (its lines; a change redraws it)."""
        text = asked if ach is None else f"{asked}\n{ach}"
        for t, _ok in v3.label_extra_lines(self._achieved.get(sid)):
            text += "\n" + t
        return text

    def _refresh_labels(self) -> None:
        """The labels again with the figures achieved (they change with the instant and the estimate)."""
        for d in getattr(self, "_sectors", []) or []:
            cached = self._cache.get(d.sid)
            if cached is None or cached[4] is None:
                continue
            asked, ach, ok = self.label_lines(d.sid, d.sec)
            text = self._label_text(d.sid, asked, ach)
            if text != cached[4]:
                self._cache[d.sid] = cached[:4] + (text,) + tuple(cached[5:])
                self._redraw_label(d.sid)
            self._colour_outline(d)
        if self._sim is not None and hasattr(self._sim, "set_achieved"):
            self._sim.set_achieved(self._achieved)

    def _outline_colour(self, d) -> str:
        """The outline of a volume: its own colour; with the range gauge on (user, 12 November 2026: a red outline
        read as a shortfall), the colour of its achieved line -- green when its range and revisit are held, red
        else, grey while not known."""
        if not self.display_bar.tg_gauge.isChecked():
            return d.color
        a = (getattr(self, "_achieved", None) or {}).get(d.sid)
        if not a:
            return FACE_OUTLINE_COLOR
        return geo.GAUGE_GREEN if a.get("ok") else geo.GAUGE_RED

    def _colour_outline(self, d) -> None:
        """Set the outline actor's colour (the achieved figures changed: no mesh rebuilt)."""
        actor = self.plotter.renderer.actors.get(sector_actor_names(d.sid)[1])
        if actor is not None:
            actor.GetProperty().SetColor(*[c / 255.0 for c in v3.rgb(self._outline_colour(d))])

    def set_calibration(self, calib: dict) -> None:
        """Pass R3d (M1): {volume id as drawn: its ratios exact / estimate} (core/calibration), for the live load."""
        self._calib = dict(calib or {})

    def set_mission_global(self, g: dict) -> None:
        """The mission's settings the allocation rule reads (face_elec_mw, realloc_mode, load_ceiling)."""
        self._mission_global = dict(g or {})
        self._load_ceiling = float(self._mission_global.get("load_ceiling") or 1.0)

    def _allocation(self, draws: List["SectorDraw"], sky: Optional[dict]) -> Optional[dict]:
        """The final allocation rule over every volume of the mission (hidden ones included: the radar executes
        them; hiding is the display's), at the instant shown; None without the fast-estimate table."""
        from core import cost_estimate as ce
        table = ce.cached_table()
        if table is None or self._eps is None or not self.estimate_allowed():
            return None
        try:
            return ce.live_allocation_async(table, [{**d.sec, "id": d.sid} for d in draws], self.grid, *self._eps,
                                            sky, self._mission_global, calib=self._calib, relief="background",
                                            tag="view")      # (its worker process: the window never waits, 12 Nov)
        except Exception:  # noqa: BLE001 -- a mission mid-edit the rule refuses: no live load
            return None

    def _poll_profiles(self) -> None:
        """Option (c): something computed in the background became ready (a cost profile, a relief): the volumes and
        the live load again; the badge follows whatever still runs (core/background)."""
        from core import background, cost_profile
        if cost_profile.generation() != self._profile_gen:
            self._profile_poll_on = False
            if self._eps is not None:
                self.set_scene(self._eps[0], self._eps[1], self._sectors)
        elif background.pending():
            self._update_badge()
            QTimer.singleShot(PROFILE_POLL_MS, self._poll_profiles)
        else:
            self._profile_poll_on = False
            self._update_badge()

    def watch_background(self) -> None:
        """A computation started elsewhere (the redundancy check): the badge shows it until it ends."""
        if not getattr(self, "_profile_poll_on", False):
            from core import cost_profile
            self._profile_poll_on, self._profile_gen = True, cost_profile.generation()
            QTimer.singleShot(PROFILE_POLL_MS, self._poll_profiles)
        self._update_badge()

    def _badge_busy(self) -> None:
        """The badge says 'computing' BEFORE an estimated update that runs on the window's thread (user, 11 November
        2026: the window froze with nothing said), painted at once."""
        lb = getattr(self, "_load_badge", None)
        if lb is None or getattr(self, "_exact_load", None) is not None:
            return
        uc.set_source(lb, "estimating", detail="updating the view",
                      tip="The fast estimate is being calculated: the view is being updated", progress=0.0)
        lb.repaint()

    def refresh_estimate(self) -> None:
        """The held estimate released: the live load and the global view's volumes computed again."""
        if self._eps is not None:
            self.set_scene(self._eps[0], self._eps[1], self._sectors)
        sim = getattr(self, "_sim", None)
        if sim is not None and self._global_shown():
            sim._draw_volumes(force=True)
        self._update_badge()

    def _on_badge_clicked(self, event) -> None:
        from core import cost_estimate as ce
        if ce.held():
            self.rerun_requested.emit()

    def _update_badge(self) -> None:
        """The ESTIMATE badge (U3): a progress bar while anything of the estimate is computed (the cost profiles, the
        reliefs, the redundancy check: core/background, user 11 November 2026), else where the figures come from."""
        from core import background
        lb = getattr(self, "_load_badge", None)
        if lb is None or getattr(self, "_exact_load", None) is not None:   # (the Results tab: EXACT stays)
            return
        a = getattr(self, "_last_alloc", None) or {}
        gauge = " (the range gauge uses the same figures)"
        from core import cost_estimate as ce
        if ce.held():                                  # (user, 13 November 2026: held after an edit of the volumes)
            uc.set_source(lb, "obsolete", tip="The volumes were edited since this estimate: the figures shown are the "
                          "former ones. Click to compute the estimate again (each edit no longer starts it)")
            lb.setCursor(Qt.PointingHandCursor)
            return
        lb.unsetCursor()
        if not self.estimate_allowed():               # (user, 13 November 2026: nothing before a scenario)
            uc.set_source(lb, "noscenario", tip="No scenario loaded: the fast estimate starts once a scenario (the "
                          "flight plan) gives the navigation and the sky")
            return
        if background.estimate_paused():              # (user, 13 November 2026: the exact calculation first)
            uc.set_source(lb, "paused", tip="An exact calculation is running: the fast estimate is paused (its "
                          "processes suspended, nothing new started) and resumes when it ends; the figures shown are "
                          "the last ones")
            return
        st = background.status()
        if st or a.get("estimating"):
            # (user, 12 November 2026: "ESTIMATE · computing · estimate" said nothing) the badge names the most
            # specific task running, the others counted; the tooltip says each and what it is for
            st = sorted(st, key=lambda s_: BADGE_ORDER.index(s_["label"]) if s_["label"] in BADGE_ORDER else 99)
            if a.get("estimating") and not any(s_["label"] == "cost profiles" for s_ in st):
                st.insert(0, {"label": "cost profiles", "detail": "", "fraction": 0.0})
            def jobs(d_):                              # (the trackers say "x2" for two jobs at once)
                return f" ({d_[1:]} jobs)" if d_.startswith("x") else (" " + d_ if d_ else "")
            names = [badge_step(s_["label"]) + jobs(s_["detail"]) for s_ in st]
            detail = names[0] + (f" +{len(names) - 1}" if len(names) > 1 else "")
            tip = ("Fast estimate being calculated (the figures shown meanwhile are the previous ones). Running:\n"
                   + "\n".join(f"- {n}: {BADGE_ACTIVITY.get(s_['label'], ('', ''))[1]}" for n, s_ in zip(names, st))
                   + "\nSteps: " + ", ".join(f"{i + 1} {BADGE_ACTIVITY[l_][0]}" for i, l_ in enumerate(BADGE_ORDER)))
            if a.get("estimating"):
                tip += "\nVolumes whose costs are not known yet: " + ", ".join(a["estimating"]) + gauge
            uc.set_source(lb, "estimating", detail=detail, tip=tip, progress=st[0]["fraction"])
        elif a.get("not_recalibrated"):
            uc.set_source(lb, "estimate", tip="Fast estimate (~20 %); not recalibrated by an exact calculation: "
                          + ", ".join(a["not_recalibrated"]) + gauge)
        elif a:
            uc.set_source(lb, "recalibrated", tip="Fast estimate corrected by the last exact calculation of every "
                          "volume" + gauge)
        else:
            uc.set_source(lb, "estimate", tip="Fast estimate")

    def set_load_ceiling(self, ceiling: float) -> None:
        self.set_mission_global({**self._mission_global, "load_ceiling": ceiling})

    def set_exact_load(self, result: Optional[dict]) -> None:
        """The live load replaced by an exact result's (the Results tab, user 13 November 2026: the bar showed the
        live estimate of the mission, the same figures as the preparation): per face the power drawn (always-on in
        grey), the S and Ku transmit time, at the real revisits of the margin reuse when there is one; badge EXACT.
        None: back to the live estimate."""
        self._exact_load = result
        if result is not None:
            self._show_exact_load()

    def _show_exact_load(self) -> None:
        res = self._exact_load or {}
        mr = res.get("margin_reuse") or {}
        pa_real = (mr.get("power_allocation") or {}).get("faces") or {}
        pa_nom = (res.get("power_allocation") or {}).get("faces") or {}
        trk_f = (res.get("tracking") or {}).get("faces") or {}
        ceil = float(mr.get("load_ceiling") or (res.get("verdict") or {}).get("load_ceiling")
                     or self._mission_global.get("load_ceiling") or 1.0)
        ref = float(P.get("power.face_elec_mw")) * 1e6
        real = bool(pa_real)
        for k, col in self._load_cols.items():
            f = pa_real.get(str(k)) or pa_nom.get(str(k)) or {}
            fn = pa_nom.get(str(k)) or f
            e_w, base = float(f.get("alloc_MW", 0.0)) * 1e6, float(f.get("always_on_MW", 0.0)) * 1e6
            use = e_w / ref if ref > 0 else 0.0
            rr = ((mr.get("faces") or {}).get(str(k)) or {}).get("resources_real") or {}
            tf = trk_f.get(str(k)) or {}
            t_s = float(rr.get("S time", tf.get("total_s_time", 0.0)) if real else tf.get("total_s_time", 0.0))
            t_k = float(rr.get("Ku time", tf.get("total_ku_time", 0.0)) if real else tf.get("total_ku_time", 0.0))
            e_load = float(f.get("load", 0.0))
            bars = [{"name": "E", "val": use, "moved": 0.0, "lent": 0.0, "ceiling": None, "base": base / ref,
                     "color": LOAD_OVER if use > 1.0 else LOAD_HIGH if e_load >= ceil * (1 - 1e-3) else LOAD_OK,
                     "text": _energy_text(e_w, base, 0.0)}]
            for name, v in (("S", t_s), ("Ku", t_k)):
                bars.append({"name": name, "val": v, "moved": 0.0, "lent": 0.0, "ceiling": ceil,
                             "color": LOAD_OVER if v > 1.0 else LOAD_HIGH if v >= ceil * (1 - 1e-3) else LOAD_OK})
            tip = [f"{geo.FACE_NAMES[k]} (exact calculation" + (", at the real revisits of the margin reuse)" if real
                                                                else ")"),
                   f"  energy {e_w / 1e6:.3f} MW drawn (always-on {base / 1e6:.3f} MW; "
                   f"{float(fn.get('alloc_MW', 0.0)):.3f} MW at the revisits asked)",
                   f"  S time {100 * t_s:.2f} %, Ku time {100 * t_k:.3f} % (ceiling {100 * ceil:.0f} %)"]
            col.set_bars(bars, "\n".join(tip))
        st = (res.get("verdict") or {}).get("state") or ""
        self._load_status.setText(st)
        self._load_status.setStyleSheet(f"color: {LOAD_OK if st == 'OK' else LOAD_HIGH if st in ('ALERT', 'DEGRADED') else LOAD_OVER};"
                                        " font-weight: bold;")
        self._load_status.setToolTip("The verdict of the exact calculation")
        self._load_lbl.setText("Load (exact):")
        uc.set_source(self._load_badge, "exact", tip="Loads of the exact calculation shown in this tab"
                      + (" at the real revisits of the margin reuse" if real else ""))

    def _show_load(self, draws: List["SectorDraw"], sky: Optional[dict]) -> None:
        if getattr(self, "_exact_load", None) is not None:      # (the Results tab: the exact result's loads)
            self._show_exact_load()
            return
        a = self._allocation(draws, sky)
        if self.diagnosis is not None:
            from core import diagnosis as dg
            try:
                self.diagnosis.set_diagnosis(dg.live(a, [{**d.sec, "id": d.sid} for d in draws]))
            except Exception:  # noqa: BLE001 -- a result mid-edit: no diagnosis rather than no load
                self.diagnosis.set_diagnosis(None)
        if a is None:
            self._last_alloc = None
            for col in self._load_cols.values():
                col.set_bars(None, "")
            self._load_status.setText("")
            from core import background                # (the first estimate still in its worker: redrawn when ready)
            if background.pending() and not getattr(self, "_profile_poll_on", False):
                from core import cost_profile
                self._profile_poll_on, self._profile_gen = True, cost_profile.generation()
                QTimer.singleShot(PROFILE_POLL_MS, self._poll_profiles)
            self._update_badge()
            return
        if not self._achieved_fixed and a.get("achieved") != self._achieved:     # the labels' second line
            self._achieved = dict(a.get("achieved") or {})
            self._refresh_labels()
        self._last_alloc = a                           # U3 (and R3d M1): where these figures come from
        self._update_badge()
        from core import background
        if ((a.get("estimating") or a.get("relief_pending") or background.pending())
                and not getattr(self, "_profile_poll_on", False)):
            from core import cost_profile               # option (c): redrawn when ready (and the relief, A)
            self._profile_poll_on, self._profile_gen = True, cost_profile.generation()
            QTimer.singleShot(PROFILE_POLL_MS, self._poll_profiles)
        ref = float(a["ref_w"])
        ceil = float(a.get("ceiling", 1.0))
        use = {k: a["faces"][str(k)]["energy"] / ref for k in range(4)}
        borrow = sum(max(0.0, u - 1.0) for u in use.values())
        spare = sum(max(0.0, 1.0 - u) for u in use.values())
        lend = min(1.0, borrow / spare) if spare > 0 else 0.0
        always_on = {}                                   # (the stacked energy bar: the always-on part)
        for r in a["demands"]:
            if str(r.get("label", "")).startswith("always-on"):
                for kk, fc in r.get("faces", {}).items():
                    always_on[int(kk)] = always_on.get(int(kk), 0.0) + float(fc.get("energy", 0.0))
        vols = {}
        for r in a["demands"]:
            for k in r.get("faces", {}):
                vols.setdefault(int(k), []).append(r)
        for k, col in self._load_cols.items():
            f = a["faces"][str(k)]
            bars = []
            e_load = f["energy load"]                 # (against the face's feeder or the operator's cap)
            st = LOAD_OVER if use[k] > 1.0 else LOAD_HIGH if e_load >= ceil * (1 - 1e-3) else LOAD_OK
            e_w = float(f["energy"])
            bars.append({"name": "E", "val": use[k], "moved": f["moved"]["energy"] / ref,
                         "lent": max(0.0, 1.0 - use[k]) * lend, "color": st, "ceiling": None,
                         "base": always_on.get(k, 0.0) / ref,
                         "text": _energy_text(e_w, always_on.get(k, 0.0), (a.get("asked_w") or [0.0] * 4)[k])})
            for name, key in (("S", "S time"), ("Ku", "Ku time")):
                v = f[key]
                bars.append({"name": name, "val": v, "moved": f["moved"][key], "lent": 0.0, "ceiling": ceil,
                             "color": LOAD_OVER if v > 1.0 else LOAD_HIGH if v >= ceil * (1 - 1e-3) else LOAD_OK})
            tip = [f"{geo.FACE_NAMES[k]}: energy {f['energy'] / 1e6:.2f} MW drawn ({100 * use[k]:.0f} % of the "
                   f"{ref / 1e6:g} MW reference; limit {f['energy cap'] / 1e6:g} MW)"]
            if use[k] > 1.0:
                tip.append(f"  {(use[k] - 1.0) * ref / 1e6:.2f} MW taken from the other faces' share")
            elif bars[0]["lent"] > 0:
                tip.append(f"  {bars[0]['lent'] * ref / 1e6:.2f} MW lent to the other faces")
            if any(f["moved"].values()):
                tip.append(f"  serving shares of a neighbour: {f['moved']['energy'] / 1e6:.2f} MW, "
                           f"S {100 * f['moved']['S time']:.1f} %, Ku {100 * f['moved']['Ku time']:.1f} %")
            tip.append(f"  S time >= {100 * f['S time']:.1f} %, Ku time {100 * f['Ku time']:.1f} % "
                       f"(ceiling {100 * ceil:.0f} %" + (f"; not recalibrated: {', '.join(a['not_recalibrated'])}, "
                                                          f"their time a lower bound)" if a.get("not_recalibrated")
                                                          else "; recalibrated by the exact runs)"))
            for r in vols.get(k, []):
                if r["kind"] == "search":
                    tip.append(f"  {r['label']}: range {100 * r['rho']:.0f} %, revisit {r['rr_tenable_s']:.3g} s, "
                               f"{100 * r['faces'][str(k)]['share']:.0f} % here")
                elif r["kind"] == "track":
                    tip.append(f"  {r['label']}: {r['tracks_tenable']:.3g} tracks")
            col.set_bars(bars, "\n".join(tip))
        n_bad = sum(1 for r in a["demands"] if r["level"] < 1.0 - 1e-6)
        st = a["status"]
        self._load_status.setText(st if not n_bad else f"{st} ({n_bad})")
        self._load_status.setStyleSheet(f"color: {LOAD_OK if st == 'OK' else LOAD_HIGH if st == 'DEGRADED' else LOAD_OVER};"
                                        " font-weight: bold;")
        lines = [f"allocation: {st}; binding: {', '.join(a['binding']) or 'none'}"]
        lines += [f"{r['label']}: range {100 * r['rho']:.0f} %, revisit {r['rr_tenable_s']:.3g} s"
                  for r in a["demands"] if r["kind"] == "search"]
        lines += a["flags"]
        self._load_status.setToolTip("\n".join(lines))

    def _play_end(self) -> float:
        link = self._scn_link
        return float(link.duration) if link is not None else 0.0

    def _on_slider(self, v: int) -> None:
        """Playback slider (perf, user 10 November 2026: a drag queued every intermediate instant, ~3 s each): the
        latest position only, taken when the event loop is free."""
        self._slider_t = v / 10000.0 * self._play_end()
        if not getattr(self, "_slider_due", False):
            self._slider_due = True
            QTimer.singleShot(0, self._apply_slider)

    def _apply_slider(self) -> None:
        self._slider_due = False
        self.set_instant(self._slider_t, from_slider=True)

    def _on_slider_pressed(self) -> None:
        self._dragging = True

    def _on_slider_released(self) -> None:
        self._dragging = False
        self.settle()

    def _light(self) -> bool:
        """Light playback: while the slider is dragged, or while playing between two full rebuilds (PLAY_HEAVY_S)."""
        if getattr(self, "_dragging", False):
            return True
        return self._playing and time.perf_counter() - getattr(self, "_last_heavy", 0.0) < PLAY_HEAVY_S

    def settle(self) -> None:
        """The full display at the instant shown (after a drag, on pause): volumes, gauge, live load rebuilt."""
        if getattr(self, "_light_pending", False) and self._scn_link is not None:
            self.set_instant(self._play_t, from_slider=True, heavy=True)

    def set_instant(self, t: float, from_slider: bool = False, heavy: bool = False) -> None:
        """The DISPLAY at the instant t of the linked run: volumes resolved with the ship's navigation there, the
        scenario's objects, the global view, the gauge, the live load (the mission itself unchanged). Light playback
        (user, 10 November 2026: a drag or a fast playback stalled on the rebuilds): during a drag, and while playing
        between two full rebuilds, only the ship, the objects and the instant follow, the volumes already built turn
        with their frames; the rest is rebuilt on release, on pause, or every PLAY_HEAVY_S while playing."""
        light = not heavy and self._light()
        if self._scn_link is None or self.instant_source is None:
            return
        t = min(max(float(t), 0.0), self._play_end())
        self._play_t = t
        if not from_slider:
            self.sl_play.blockSignals(True)
            self.sl_play.setValue(int(round(10000 * t / max(self._play_end(), 1e-9))))
            self.sl_play.blockSignals(False)
        self.lb_play.setText(f"{t:8.1f} s  {self._scn_link.utc(t)}")
        try:
            draws, g = self.instant_source(t)
        except Exception:  # noqa: BLE001 -- an instant the model refuses: the display kept
            return
        self._sky_ship = g.get("sky")
        n = frames.Nav(g["nav"][0]) if g.get("nav") else None
        self.set_navigation(None if n is None else n.r_si, None if n is None else n.body)
        if self.instant_shown is not None:
            try:
                self.instant_shown(t, g)
            except Exception:  # noqa: BLE001 -- a panel's text: never stops the playback
                pass
        self._light_pending = light
        if light:
            if self._sim is not None:
                self._sim.defer_rebuild = True
            try:
                self.set_scenario(self._scn_link, t)
            finally:
                if self._sim is not None:
                    self._sim.defer_rebuild = False
            return
        self._last_heavy = time.perf_counter()
        if self._eps is not None and not self._global_shown():
            self._in_instant = True
            try:
                self.set_scene(self._eps[0], self._eps[1], draws)
            finally:
                self._in_instant = False
            self._sphere_stale = None
        elif self._eps is not None:          # (perf: the sphere hidden in the global mode, rebuilt when shown again)
            self._sphere_stale = draws
        self._show_load(draws, g.get("sky"))
        self.set_scenario(self._scn_link, t)

    def toggle_play(self) -> None:
        self._playing = not self._playing
        self.bt_play.setIcon(uc.std_icon(self.bt_play, "SP_MediaPause" if self._playing else "SP_MediaPlay"))
        if self._playing:
            if self._play_t >= self._play_end():
                self._play_t = 0.0
            self._play_wall = time.perf_counter()
            self._play_timer.start()
        else:
            self._play_timer.stop()
            self.bt_play.repaint()                     # (the pause shown at once; the full rebuild after)
            QTimer.singleShot(0, self.settle)

    def _play_tick(self) -> None:
        """One instant of the playback (user, 11 November 2026: a pause clicked during a rebuild was sometimes lost:
        a tick already queued moved on, and a second click restarted the playback): nothing once paused; the next
        tick scheduled after this one, so a click is always handled between two instants."""
        if not self._playing:
            return
        now = time.perf_counter()
        dt = (now - self._play_wall) * PLAY_SPEEDS[self.cb_speed.currentIndex()]
        self._play_wall = now
        t = self._play_t + dt
        if t >= self._play_end():
            t = self._play_end()
            self.toggle_play()
        self.set_instant(t)
        if self._playing:
            self._play_timer.start()

    def set_played_only(self, on: bool) -> None:
        """"Show effectively played" (the Watch sectors' switch): only what the volumes scan, the parts another
        volume serves taken out, in both modes."""
        self.played_only = bool(on)
        self._cache = {}
        if self._eps is not None:
            self._build_sectors()
        if self._sim is not None:
            self._sim.played_only = self.played_only
            self._sim._draw_volumes(force=True)
        self.apply_visibility()

    def _on_gauge(self, on: bool) -> None:
        """The range gauge in both modes: the sectors rebuilt, the global view told."""
        self._cache = {}
        if self._eps is not None:
            self._build_sectors()
        if self._sim is not None:
            self._sim.range_gauge = on
            self._sim._draw_volumes(force=True)
        self.apply_visibility()

    def _nav_msg(self) -> Optional[dict]:
        """The mission's navigation message with an orbit (the global mode's flight plan without a scenario)."""
        if self._sim_mission is None:
            return None
        nav = self._sim_mission[0].get("global", {}).get("nav")
        if frames.validate_nav(nav) or not nav:
            return None
        return nav[0] if frames.Nav(nav[0]).has_orbit else None

    def _check_global(self) -> None:
        """The global mode available with a flight plan; shown: brought up to date."""
        self.display_bar.set_global_available(self._scn_link is not None or self._nav_msg() is not None)
        if self._global_shown():
            self._update_global()

    def _on_global(self, on: bool) -> None:
        self.display_bar.tg_frame.setEnabled(not on and self._r_is is not None)   # (the sphere's frame only)
        for tg in (self.display_bar.tg_planned, self.display_bar.tg_all_orbits, self.display_bar.tg_orbit_vec):
            tg.setEnabled(on)                                                    # (the orbits: global view only)
        if on and (self._scn_link is not None or self._nav_msg() is not None):
            if self._sim is None:
                from gui.sim_view import SimView             # (late: sim_view imports this module)
                self._sim = SimView(self.grid, read_only=True)
                self._sim.estimate_allowed = lambda: self.estimate_allowed()
                self._sim.range_gauge = self.display_bar.tg_gauge.isChecked()
                self._sim.set_achieved(self._achieved)       # (the labels' second line)
                self._sim.sector_labels_always = True        # (the volumes' labels whatever the objects' labels)
                for i in range(self._sim.play_bar.count()):   # (the instant is the mission's: no playback)
                    w = self._sim.play_bar.itemAt(i).widget()
                    if w is not None:
                        w.setVisible(False)
                # the view alone (user, 10 November 2026: no Info, no options column of the editor in the RMP)
                self._sim.side_panel.setParent(None)
                self._sim.side_panel.hide()
                self._stack.addWidget(self._sim)
            self._stack.setCurrentIndex(1)
            self._sync_global_options()
            self._update_global()
        else:
            self._stack.setCurrentIndex(0)
            stale = getattr(self, "_sphere_stale", None)
            if stale is not None and self._eps is not None:      # the instants played meanwhile
                self._sphere_stale = None
                self._in_instant = True
                try:
                    self.set_scene(self._eps[0], self._eps[1], stale)
                finally:
                    self._in_instant = False
            self.plotter.render()

    def _sync_global_options(self, *_a) -> None:
        """The panel's options applied to the global view too (user, 12 November 2026: the same options in both
        modes): UI size, responsibility zones, stars and constellations, the scenario's trails and labels (its objects
        through _update_global)."""
        sim, bar = getattr(self, "_sim", None), self.display_bar
        if sim is None:
            return
        if abs(sim.ui_size.scale - self.ui_size.scale) > 1e-9:
            sim.ui_size.scale = self.ui_size.scale
            sim.ui_size.show()
            sim.ui_size.changed.emit(sim.ui_size.scale)
        for mine, theirs in ((bar.tg_zones, sim.cb_zones), (bar.tg_sky, sim.tg_sky), (bar.cb_const, sim.cb_const),
                             (bar.tg_scn_trails, sim.cb_trails), (bar.tg_scn_labels, sim.cb_object_labels),
                             (bar.tg_planned, sim.cb_planned), (bar.tg_all_orbits, sim.cb_all_orbits),
                             (bar.tg_orbit_vec, sim.cb_orbit)):
            if theirs.isChecked() != mine.isChecked():
                theirs.setChecked(mine.isChecked())

    def _update_global(self) -> None:
        """The global view (user, 10 November 2026): the flight plan only, the linked scenario's ship (its states and
        attitude as flown, no object) at t, else the mission's navigation alone; the mission's volumes from the ship;
        the view alone, centred on OWNSHIP."""
        from gui import scenario_link as slink
        from gui.sim_view import OWNSHIP
        link, sim = self._scn_link, self._sim
        try:
            if link is not None:
                objects = self.display_bar.tg_scn_objects.isChecked()    # (user, 12 November 2026: as the sphere)
                key, t = ("link", id(link), objects), self._scn_t
                if self._sim_run != key:
                    sc, truth, g = (link.sc, link.truth, link.g) if objects else slink.ship_only(link)
                    sim.show_run(sc, truth, g, f"flight plan of {link.name}")
                    self._sync_global_options()      # (the run's view profile: the panel's choices again)
            else:
                nav = self._nav_msg()
                if nav is None:
                    return
                key, t = ("nav", json.dumps(nav, sort_keys=True, default=str)), 0.0
                if self._sim_run != key:
                    sc, truth, g = slink.nav_run(nav)
                    sim.show_run(sc, truth, g, "mission navigation")
        except Exception as exc:  # noqa: BLE001 -- a flight plan the model refuses: said, the view kept
            sim.status.setText(f"global view: {exc}")
            return
        if self._sim_mission is not None:
            mkey = json.dumps([self._sim_mission[0], sorted(self._sim_mission[1])], sort_keys=True, default=str)
            if mkey != getattr(self, "_sim_mission_key", None) or self._sim_run != key:
                self._sim_mission_key = mkey          # (perf: handed over at every instant, it forced a rebuild of
                sim.set_mission(*self._sim_mission)   # every volume, the 0.5 deg rule never applied)
        new_run = self._sim_run != key
        self._sim_run = key
        sim.set_time(t)
        sim.select(OWNSHIP)
        if new_run or sim.centre != OWNSHIP:           # centred on OWNSHIP, always (user, 10 November 2026): framed
            sim.set_centre(OWNSHIP)                    # at a new run, then followed (the zoom chosen kept)
            self._frame_volumes()

    def _frame_volumes(self) -> None:
        """The camera at a distance where the mission's farthest volume fits the view (OWNSHIP alone framed the view
        to a few tens of km, the volumes out of it); its direction kept."""
        sim = self._sim
        ranges = [float(s_.get("range_km", 0.0)) for s_ in (self._sim_mission or ({}, ()))[0].get("sectors", [])
                  if isinstance(s_.get("range_km"), (int, float))]
        r_max = max(ranges, default=0.0)
        if r_max <= 0.0:
            return
        cam = sim.plotter.camera
        f, pos = np.asarray(cam.GetFocalPoint(), float), np.asarray(cam.GetPosition(), float)
        u = pos - f
        n = float(np.linalg.norm(u))
        if n <= 0.0:
            return
        d = 1.2 * r_max / math.tan(math.radians(0.5 * cam.GetViewAngle()))
        cam.SetPosition(*(f + u / n * d))
        sim.plotter.renderer.ResetCameraClippingRange()
        sim.plotter.render()

    def close_views(self) -> None:
        """Both plotters closed (the window closes)."""
        if self._sim is not None:
            if hasattr(self._sim, "stop_background"):
                self._sim.stop_background()
            self._sim.plotter.close()
        self.plotter.close()

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
        """Places in the scene: on the unit sphere at the direction."""
        return d

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
                seg = _sphere_arcs(xyz[a:b, j])
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
        bar = self.display_bar
        box = QGroupBox("3D view")
        col = QVBoxLayout(box)
        col.setContentsMargins(8, 4, 8, 6)
        col.setSpacing(4)
        # (user, 12 November 2026: "le bazar visuellement") the mode and the view on top, the volumes' buttons on their
        # own line, then the options by section, two columns; the same options in both modes
        head = QHBoxLayout()
        head.addWidget(bar.tg_global)
        head.addStretch(1)
        head.addWidget(self.bt_recentre)
        col.addLayout(head)
        line2 = QHBoxLayout()                          # (user, 12 November 2026: one subject per line)
        line2.addWidget(self.ui_size.widget())
        line2.addStretch(1)
        col.addLayout(line2)
        line3 = QHBoxLayout()
        line3.addWidget(QLabel("Volumes:"))
        line3.addWidget(bar.bt_show_all)
        line3.addWidget(bar.bt_hide_all)
        line3.addStretch(1)
        col.addLayout(line3)
        for w in self._panel_rows:
            col.addWidget(v3.row(w))
        # one grid for every switch (user, 12 November 2026: "aligner tous les toggles"): two equal columns, each
        # switch at the left of its cell, the section titles over both
        grid = QGridLayout()
        grid.setContentsMargins(0, 4, 0, 0)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(5)
        grid.setColumnMinimumWidth(0, 8)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        r, self._scn_opts = 0, []
        for sections, is_scn in ((bar.display_sections(), False), (bar.scenario_sections(), True)):
            for head_text, toggles in sections:
                lb = v3.title(head_text)
                grid.addWidget(lb, r, 0, 1, 3)
                r += 1
                for i, tg in enumerate(toggles):
                    grid.addWidget(tg, r + i // 2, 1 + i % 2, Qt.AlignLeft | Qt.AlignVCenter)
                r += (len(toggles) + 1) // 2
                if is_scn:
                    self._scn_opts += [lb] + list(toggles)
        col.addLayout(grid)
        self._show_scn_opts(self._scn_link is not None)
        for tg in (bar.tg_planned, bar.tg_all_orbits, bar.tg_orbit_vec):
            tg.setEnabled(bar.tg_global.isChecked())
        self._view_panel = box
        return box

    def _show_scn_opts(self, on: bool) -> None:
        """The scenario's options (titles and switches) shown only with a linked scenario."""
        for w in getattr(self, "_scn_opts", []):
            w.setVisible(on)

    def _on_global_option(self) -> None:
        """A scenario option changed: the global view follows (its objects: another run shown)."""
        self._sync_global_options()
        if self._global_shown():
            self._update_global()

    def recentre(self) -> None:
        """Back to the default view in the frame chosen (global: on OWNSHIP, the volumes framed)."""
        if self._global_shown() and getattr(self, "_sim", None) is not None:
            from gui.sim_view import OWNSHIP
            self._sim.set_centre(OWNSHIP)
            self._frame_volumes()
            return
        set_default_camera(self.plotter, self._r_view if self._icrf_view() else None)
        self.plotter.render()

    def _ui_scale_changed(self, scale: float) -> None:
        """The labels at the new UI size: the static ones, the frame's axes, the sectors'."""
        v3.apply_ui_scale(self.plotter, scale)
        add_static_labels(self.plotter)
        set_icrf_axes(self.plotter, self._r_view if self._icrf_view() else None)
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
    def set_navigation(self, r_si: Optional[np.ndarray], body: Optional[str] = None) -> None:
        """Attitude of the mission navigation (ship -> ICRF), None if invalid, and its central body; the equatorial
        view follows them (H3: the central body's equatorial axes, ICRF without a body)."""
        r_is = None if r_si is None else np.asarray(r_si).T
        r_view = None if r_is is None else r_is @ frames.equator_axes(body if body in frames.BODY_RADIUS_KM else None)
        same = (r_is is None and self._r_is is None) or (
            r_is is not None and self._r_is is not None and np.array_equal(r_is, self._r_is)
            and np.array_equal(r_view, self._r_view))
        self._r_is, self._r_view = r_is, r_view
        self.display_bar.tg_frame.setEnabled(r_is is not None and not self.display_bar.tg_global.isChecked())
        if not same and self.display_bar.icrf():
            self._apply_view_frame()
        if not same:
            self._update_sky()

    def _icrf_view(self) -> bool:
        return self.display_bar.icrf() and self._r_is is not None

    def _turn_axis(self) -> np.ndarray:
        """The vertical of the mouse rotation: the one of the default view (set_default_camera)."""
        return self._r_view[:, 2] if self._icrf_view() else np.array([0.0, 0.0, 1.0])

    def _apply_view_frame(self) -> None:
        set_icrf_axes(self.plotter, self._r_view if self._icrf_view() else None)
        set_default_camera(self.plotter, self._r_view if self._icrf_view() else None)
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
        anchor = _anchor(cached[2], cached[3])
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

    def _add_label(self, sid: str, point, text: str, lbl_name: str) -> None:
        """A volume's 3D label (user, 11 November 2026): what is asked in grey, under it what is achieved in green
        (held) or red (not held), on a black box (view3d.LineLabels: one text per line, each its colour)."""
        if getattr(self, "_line_labels", None) is None:
            self._line_labels = v3.LineLabels(self.plotter, SECTOR_LABEL_FONT_SIZE)
        sec = next((d.sec for d in self._sectors if d.sid == sid), {})
        asked, ach, ok = self.label_lines(sid, sec)
        lines = [(asked, LABEL_ASKED)] + ([(ach, LABEL_OK if ok else LABEL_NOK)] if ach else [])
        for t, ok_ in v3.label_extra_lines(self._achieved.get(sid)):   # (who handles the need, the confirmation)
            lines.append((t, LABEL_OK if ok_ else LABEL_NOK))
        self._line_labels.put(sid, point, lines)
        self._line_labels.set_visible(sid, sid not in self._hidden)
        remove_actors(self.plotter, [lbl_name, lbl_name + "_2"])     # (the former single-colour labels)
        self._actor_names.discard(lbl_name)

    def _redraw_label(self, sid: str) -> None:
        """Re-place sid's label actor (and its leader line, added/removed as needed) without rebuilding the
        sector mesh -- cheap enough to call on every mouse-move while dragging."""
        cached = self._cache.get(sid)
        if cached is None or cached[4] is None:
            return
        _key, color, poly, _outline, text = cached[:5]
        _surf_name, _out_name, lbl_name, leader_name = sector_actor_names(sid)
        anchor = _anchor(poly, _outline)
        point = self._label_pos.get(sid, anchor)
        self._add_label(sid, point, text, lbl_name)
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
        self._badge_busy()                             # (the update below runs on the window's thread)
        self._eps, self._sectors = (eps_min, eps_max), list(sectors)
        if hidden is not None:
            self._hidden = set(hidden)
        self._build_faces()
        self._build_sectors()
        self.apply_visibility()
        if not getattr(self, "_in_instant", False):
            self._show_load(self._sectors, self._sky_ship)
        self._update_badge()                           # (the 'updating' state never left behind)

    def set_hidden(self, sids: Set[str]) -> None:
        self._hidden = set(sids)
        self.apply_visibility()
        # (user, 13 November 2026: a sector ticked off stayed drawn in the global view) the global view's hidden too
        sm = getattr(self, "_sim_mission", None)
        if sm is not None and sm[1] != self._hidden:
            self._sim_mission = (sm[0], set(self._hidden))
            if self._global_shown():
                self._update_global()

    def apply_visibility(self) -> None:
        """Re-applied after every rebuild, since rebuilt actors come back visible; renders."""
        bar, p = self.display_bar, self.plotter
        for k in range(4):
            set_actors_visible(p, face_actor_names(k), bar.face_visible(k))
        for d in self._sectors:
            shown = d.sid not in self._hidden and d.sid not in getattr(self, "_played_hidden", set())
            set_actors_visible(p, sector_actor_names(d.sid) + (sector_hole_name(d.sid),), shown)
            if getattr(self, "_line_labels", None) is not None:
                self._line_labels.set_visible(d.sid, shown)
        if getattr(self, "_line_labels", None) is not None:
            self._line_labels.keep([d.sid for d in self._sectors])
        if hasattr(self, "_scn_sym"):
            self._scn_visibility()
        p.render()

    # ------------------------------------------------------------------ internals
    def _build_faces(self) -> None:
        if self._face_key == self._eps:
            return  # the face layer only depends on the eps window, not on the watch sectors
        self._face_key = self._eps
        scene = geo.build_face_sectors_smooth(self.grid, *self._eps)   # (smooth edges, 10 November 2026)
        for k in range(4):
            surf_name, out_name = face_actor_names(k)
            if scene.surfaces[k].n_points:
                # both sides drawn (user, 10 November 2026): a zone seen from inside the sphere, on its far side,
                # keeps its colour (back faces were culled: the far half of the sphere showed black)
                self.plotter.add_mesh(scene.surfaces[k], color=geo.FACE_COLORS[k], opacity=self.face_opacity,
                                      culling=None, name=surf_name, render=False)
            else:
                self.plotter.remove_actor(surf_name, render=False)
            if scene.outlines[k].n_points:
                self.plotter.add_mesh(scene.outlines[k], color=FACE_OUTLINE_COLOR, line_width=1, name=out_name,
                                      render=False)
            else:
                self.plotter.remove_actor(out_name, render=False)

    def _build_sectors(self) -> None:
        """Only sectors whose geometry / colour / label changed are rebuilt (E3, user 10 November 2026): the WHOLE
        watched volume's outline in its colour, its executed volumes (the parts the faces scan at this attitude) in
        the colour of their face (exec_face_colours; else the sector's colour: the results' "Colour by"), its
        coverage hole in grey; on the unit sphere (true size: the global mode)."""
        eps_min, eps_max = self._eps
        new_names: Set[str] = set()
        new_cache: Dict[str, tuple] = {}
        gauge = self.display_bar.tg_gauge.isChecked()
        played = bool(getattr(self, "played_only", False))
        alloc = (self._allocation(self._sectors, self._sky_ship) if gauge or played else None)   # (gauge (c): all)
        self._played_hidden = set()
        for d in self._sectors:
            surf_name, out_name, lbl_name, leader_name = sector_actor_names(d.sid)
            rho = (alloc or {}).get("by_sector", {}).get(d.sid, {}).get("rho") if gauge else None
            # "show effectively played" (user, 12 November 2026): the part another volume serves (the relief of the
            # fast estimate at the instant shown) taken out; a volume served in full not drawn at all
            rel = ((alloc or {}).get("relief") or {}).get(d.sid) if played else None
            if rel and float(rel.get("share", 0.0)) >= PLAYED_FULL_SHARE:
                self._played_hidden.add(d.sid)
            sec_d = dict(d.sec, _played_relief=rel["code"]) if rel and rel.get("code") else d.sec
            d = SectorDraw(d.sid, sec_d, d.color, d.text) if sec_d is not d.sec else d
            key = (geo.sector_geometry_key(d.sec), eps_min, eps_max, gauge,
                   None if rho is None else round(float(rho), 4), d.sec.get("range_km") if gauge else None,
                   _key_sky(self._sky_ship) if gauge else None, d.sec.get("_played_relief"), played)
            cached = self._cache.get(d.sid)
            # a duplicate id (mid-edit) shares actor names with the earlier sector: always redraw, last one wins
            same_geom = cached is not None and cached[0] == key and d.sid not in new_cache
            hole_name = sector_hole_name(d.sid)
            if same_geom:
                poly, outline, hole = cached[2], cached[3], cached[5]
            else:
                try:
                    poly, outline, hole = geo.build_sector_split_smooth(self.grid, d.sec, eps_min, eps_max)
                    ratio_of = (geo.held_ratio_function(self.grid, d.sec, eps_min, eps_max, self._sky_ship, alloc,
                                                        d.sid) if gauge else None)
                    if ratio_of is not None:                # the range gauge in place of the executed volumes
                        poly, hole = geo.build_range_gauge(self.grid, d.sec, ratio_of, 1.015, normalise=True,
                                                           played=played)
                        poly.cell_data.pop("face", None) if poly.n_points else None
                    if played and hole.n_points:            # (played: what is not transmitted is not drawn)
                        hole = type(hole)()
                except (KeyError, TypeError, ValueError):
                    continue  # incomplete sector mid-edit: no 3D overlay
            same_mesh = same_geom and cached[1] == d.color
            if poly.n_points:
                if not (same_mesh and has_actor(self.plotter, surf_name)):
                    if gauge:
                        self.plotter.add_mesh(poly, color=geo.GAUGE_GREEN, opacity=0.6, culling=None, lighting=False,
                                              name=surf_name, render=False)
                    elif self.exec_face_colours and "face" in poly.cell_data:
                        rgb = np.array([v3.rgb(geo.FACE_COLORS[k]) for k in poly.cell_data["face"]], np.uint8)
                        self.plotter.add_mesh(poly, scalars=rgb, rgb=True, preference="cell",
                                              opacity=self.sector_opacity, culling=None, name=surf_name, render=False)
                    else:
                        self.plotter.add_mesh(poly, color=d.color, opacity=self.sector_opacity,
                                              culling=None, name=surf_name, render=False)   # (both sides, as the zones)
                new_names.add(surf_name)
            if hole.n_points:
                if not (same_mesh and has_actor(self.plotter, hole_name)):
                    self.plotter.add_mesh(hole, color=geo.GAUGE_RED if gauge else HOLE_COLOR,
                                          opacity=0.45 if gauge else HOLE_OPACITY, culling=None, name=hole_name,
                                          render=False)
                new_names.add(hole_name)
            if outline.n_points:
                if not (same_mesh and has_actor(self.plotter, out_name)):
                    self.plotter.add_mesh(outline, color=self._outline_colour(d), line_width=2, name=out_name,
                                          render=False)
                else:
                    self._colour_outline(d)
                new_names.add(out_name)
            asked, ach, _ok = self.label_lines(d.sid, d.sec)
            text = self._label_text(d.sid, asked, ach) if (poly.n_points or outline.n_points) else None
            anchor = _anchor(poly, outline)
            point = self._label_pos.get(d.sid, anchor)
            if text is not None:
                rebuild_label = not (same_mesh and cached[4] == text and getattr(self, "_line_labels", None) is not None
                                     and d.sid in self._line_labels._items)
                font = _ui(self.plotter, SECTOR_LABEL_FONT_SIZE)
                if rebuild_label:
                    # Fixed readout on a near-opaque black box: text_color=d.color (the sector's own colour, e.g.
                    # green) used to sit on a translucent grey box over the same-coloured sector mesh -- unreadable.
                    self._add_label(d.sid, point, text, lbl_name)
                new_names.add(lbl_name)
                if chr(10) in text:
                    new_names.add(lbl_name + "_2")
                if d.sid in self._label_pos:   # dragged away from its volume: show the link back to it
                    if rebuild_label or not has_actor(self.plotter, leader_name):
                        self.plotter.add_mesh(pv.Line(anchor, point), color=d.color, line_width=1, name=leader_name,
                                              render=False)
                    new_names.add(leader_name)
            new_cache[d.sid] = (key, d.color, poly, outline, text, hole)
        remove_actors(self.plotter, self._actor_names - new_names)   # incl. the "-labels" actors
        self._actor_names, self._cache = new_names, new_cache
        self._label_pos = {sid: p for sid, p in self._label_pos.items() if sid in new_cache}
