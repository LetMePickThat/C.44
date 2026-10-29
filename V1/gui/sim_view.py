#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SIM_VIEW -- the 3D views and the playback of the global simulator, on the right of the Scenario tab (roadmap phase 6,
passes S6 and S7; framing V1-V5 and L1-L6 validated by the user on 29 October 2026, block F of 28 October).

  V1 one plotter, a selector Global view / Ship view;
  V2 "Propagate" in a background thread (progress bar, cancelled by a newer request); "auto" re-propagates 1 s after
     every valid change while rows x steps stays below tool.display.sim_auto_max_cells;
  V3 global view, true scale (km, ICRF axes from the central body): the Earth and the Moon in every scenario, at
     their true places and turning (Earth: textured TEXTURE_DIR/earth2.jpg, turned by its sidereal angle; Moon:
     textured, near side toward the Earth) and the Moon's orbit over a sidereal month; the trajectories over the
     whole run coloured by class (ship in black), the Sun (textured, at its true place and size, 1 AU away,
     lighting the scene with parallel rays: day and night sides), the Earth's atmosphere as a halo at its limb
     (user, 29 October 2026, in the ship view too), markers for appearances, manoeuvres,
     proportional-navigation starts and impacts; the ship drawn as a triad showing its attitude, of constant size
     on screen; a double-click on a body (OWNSHIP included, in either view; the Earth, the Moon, the Sun in the
     global view) centres the view on it and follows it, on nothing back to the default centre; zoom with the wheel at any time, playback included
     (each frame leaves MIN_IDLE_MS to the event loop);
  V4 ship view (ship frame): the ship model, faces, objects at their exact direction and at a log range scale from
     tool.display.sim_ship_view_r[0] (nearest) to [1] (farthest), or at 1:1 in km (option; the ship then shown by its
     axes, the volumes at their true size), range rings of about constant size on screen whose round range values
     follow the zoom, lines of sight coloured by owner face,
     grey when occulted or outside every face, orange when masked by the structure;
  V5 the volumes of the linked mission at the current time (the navigation at t, sim/engine.py): built once, then
     turned with their frame at every frame (rebuilt past VOL_REBUILD_DEG of turn);
  performance (user, 29 October 2026, measured): the dynamic actors are created once and their points updated in
     place, labels by one vtkLabeledDataMapper each, one render per frame (the Qt repaint);
  L1 play / pause, step +/- (journal step), slider, speed x1 .. x100 000 (no reverse: user, 29 October 2026); time from the epoch and UTC;
  L2 tool.display.sim_fps frames per second: positions interpolated at each frame (Hermite), the face / aspect / RCS
     read from the geometry at the nearest recorded instant;
  L3 trails of tool.display.sim_trail_s, labels, velocity vectors (global) and noses (ship view) on demand;
  L4 the events (appearances, manoeuvres, PN starts, impacts, attitude events, shadow) as marks on the playback
     slider (user, 29 October 2026, replaces the list): a balloon gives their detail, a click goes there;
  L5 information on the object clicked: state, range, range rate, face, theta, aspect,
     RCS S and Ku, occultation, masking; for OWNSHIP: state, altitude, nose, angular rate, shadow; dark background;
  L6 "Run" writes the journal (sim/run.py, background thread), "Open run..." replays a journal without recomputing;
  B14 (user, 29 October 2026): the view's controls in the playback bar under the 3D view, the information in a
     column of its own; the propagation automatic ("Propagate" in the window's header only for a scenario too large), its
     state in the window's status bar; "Record run..." (L6 "Run") and "Open run..." in the window's menu.
No RF here (F9): step 6.4 adds the looks and the tracks to the same player.
"""
from __future__ import annotations

import copy
import math
import os
from functools import lru_cache
import time
from datetime import timedelta
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
                             QPlainTextEdit, QProgressBar, QPushButton, QShortcut, QVBoxLayout, QWidget)
from PyQt5.QtGui import QFontDatabase, QKeySequence
from pyvistaqt import QtInteractor

from core import frames
from core import geometry as cgeo
from core import optimizer as opt
from core.params import P
from gui import scene_view as sv
from gui import controls as uc
from sim import attitude as att
from sim import engine
from sim import ephemeris as eph
from sim import geometry as sgeo
from sim import propagate as prop
from sim import scenario as scn

TRAIL_S = float(P.get("tool.display.sim_trail_s"))
FPS = int(P.get("tool.display.sim_fps"))
SHIP_R0, SHIP_R1 = (float(x) for x in P.get("tool.display.sim_ship_view_r"))
AUTO_MAX_CELLS = float(P.get("tool.display.sim_auto_max_cells"))
MAX_VIEW_KM = float(P.get("tool.display.sim_max_view_km"))     # farthest zoom out (user)
MIN_RING_KM = float(P.get("tool.display.sim_min_ring_km"))     # nearest zoom in: the innermost ring (user)
MODEL_SCREEN = float(P.get("tool.display.sim_model_screen"))   # ship view, log scale: model size on screen (user)
MODEL_AXES_UNITS = 1.35                           # length of the model's axes in scene units (core.geometry.build_axes)
LOG_E0 = math.log10(MIN_RING_KM)                  # log scale: MIN_RING_KM at a quarter of the half-height
LOG_SPAN_MIN = 0.6                                # ... and at least 0.6 decade from there to the half-height (2 km)
LOG_FAR_U = 0.8                                   # ... the farthest object of the run at this x the half-height at start
LOG_R_FLOOR = 0.03                                # ... objects nearer than MIN_RING_KM drawn at least this x it
LOG_MIN_SPAN = 10.0                               # ship view log scale: nearest .. farthest at least one decade
ARCBALL_RADIUS = float(P.get("tool.display.arcball_radius"))   # mouse rotation (user)
SPEEDS = (1, 10, 100, 1000, 10000, 100000)     # x10 000 and x100 000: the Moon on its orbit (user)
VOL_REBUILD_DEG = 5.0                             # V5: a volume is turned in place, rebuilt past this turn of its frame
RING_STEP_FRAC = 0.25                             # V4: range rings spaced this share of the visible half-height apart
RING_COVER = 1.1                                  # ... out to this x the farthest corner of the 3D view
RING_MIN_SIN_EL = 0.2                             # (a view almost in the rings' plane: at most 5 x the half-height)
RING_LABEL_C, RING_LABEL_S = math.cos(math.radians(22.5)), math.sin(math.radians(22.5))   # ring values at az 22.5
SPOKE_STEP_DEG = 45.0                             # ship view: azimuth spokes every this
SPOKE_LABEL_FRAC = 0.9                            # ... labelled at this x the visible half-height
RING_MAX = 60                                     # rings at most
DECADE_STEPS = (0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)   # log scale: ring step in decades
RING_POINTS = 121
RING_LOG10_KM = (-300.0, 300.0)                   # log scale: rings over the whole view (user), within the floats
TRAIL_SAMPLES = 25                                # L3: points of a trail in the global view
# the objects and their geometry (points, trajectories, trails, range projection) in the colour of their
# classification (user, 29 October 2026), the identity frames' colours of the symbology (tools/gen_symbols.py)
IDENTITY_COLORS = {"friendly": "#4FC3F7", "hostile": "#FF4D4D", "suspect": "#FF9A3C", "neutral": "#5CD65C",
                   "unknown": "#FFE24D", "pending": "#FFE24D"}
OWNSHIP_IDENTITY = "friendly"                     # OWNSHIP is friendly (user): always drawn in that colour
SHIP_COLOR = IDENTITY_COLORS[OWNSHIP_IDENTITY]    # its point, trajectory and trail
KM_PER_MODEL_UNIT = 1e-3 / cgeo._M                # 1:1: the model's scene units to km (its true size)
BG_COLOR = "#0b1020"                              # dark background of the 3D views
TEXT_COLOR = "#e6e6e6"
SIDE_PANEL_W = 340                                # px: the column of the information (at start)
MUTED_COLOR = "#9aa0aa"
EARTH_COLOR = "#4a78b5"
ATM_COLOR = "#8cc4ff"                             # the Earth's atmosphere: a halo seen at the limb
ATM_SHELLS = tuple(1.0 + 0.04 * (k / 10) ** 1.5 for k in range(1, 11))   # its shells' radii / the Earth's: denser
ATM_OPACITY = 0.07                                # at the surface, fading out (outermost ~250 km, exaggerated to be
                                                  # readable); the opacity of each shell
SUN_COLOR = "#e0a800"
OCC_COLOR = "#a0a0a0"
MASK_COLOR = "#e07b00"
MIN_IDLE_MS = 15                                  # event-loop time kept between two frames (mouse live while playing)
SIDEREAL_MONTH_S = 27.321661 * 86400.0            # the Moon's orbit drawn over one sidereal month
PROJ_ARC_POINTS = 24                              # points of the arc linking an object to its range projection
FOLLOW_MIN_KM = 50.0                              # "follow the ship": framed span at least this
TRIAD_FRAC = 0.03                                 # ship triad length, share of the camera distance
TEXTURE_DIR = cgeo.MOON_TEXTURE_PATH.parent       # assets/textures (equirectangular, 2:1)
TEXTURE_FILES = {"earth": "earth2.jpg", "moon": "moon.jpg", "sun": "sun.jpg"}   # the Earth: earth2 (user, 29 October 2026)
NEAR_FRAC = 0.01                                  # near clipping plane at least this x the camera distance
NEAR_TOL = 1e-9                                   # near / far clipping ratio: the Sun at 1 AU, objects at a few km
                                                  # (bodies drawn front faces only: no self-occlusion far away)
BODY_NAMES = {"Earth": "earth", "Moon": "moon", "Sun": "sun"}   # the bodies the view can centre on
OWNSHIP = "<ship>"                                # the ship's key in the views (row "ship" of the truth), apart
                                                  # from any object id; shown under its scenario name (own_name())
CENTRAL = "central body"
DYN = "dyn_"                                      # prefix of the actors redrawn at every frame
CLICK_PX = 15.0                                   # a click picks the body drawn within this (pixels)
VOL = "vol_"                                      # prefix of the mission-volume actors


class _Cancelled(Exception):
    pass


class SimWorker(QThread):
    """Runs job(progress_cb) in the background; a newer request cancels it at its next progress call."""
    progress = pyqtSignal(str, float)
    done = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, job: Callable, parent=None):
        super().__init__(parent)
        self.job, self.cancelled = job, False

    def _cb(self, stage: str, f: float) -> None:
        if self.cancelled:
            raise _Cancelled()
        self.progress.emit(stage, f)

    def run(self) -> None:
        try:
            self.done.emit(self.job(self._cb))
        except _Cancelled:
            pass
        except Exception as exc:                    # shown to the operator; the view keeps the last result
            self.failed.emit(f"{type(exc).__name__}: {exc}")


def propagate_job(sc: dict) -> Callable:
    def job(cb):
        truth = prop.propagate(sc, progress_cb=lambda k, n: cb("propagation", k / max(n, 1)))
        cb("geometry", 0.0)
        return sc, truth, sgeo.geometry(sc, truth)
    return job


def run_job(path: str) -> Callable:
    def job(cb):
        from sim import run as simrun
        simrun.run(path, cb)
        sc, truth, g, _nav, _meta = engine.load_journal(engine.journal_paths(path)[0])
        return sc, truth, g
    return job


# attributes with no effect on the scenario's physics (user, 29 October 2026): changing them shows the run again
# without propagating: an object's or a group's classification (its identity, "identity") and the scenario's name.
# (Its characterisation, the class, changes the RCS and the dynamics: it propagates.)
NON_PHYSICAL_ITEM_KEYS = ("identity",)
NON_PHYSICAL_META_KEYS = ("name",)
NON_PHYSICAL_SHIP_KEYS = ("name",)                # the ship's name (user, 29 October 2026)


def _physics(sc: dict) -> dict:
    """A copy without the attributes that do not change the propagation."""
    c = copy.deepcopy(sc)
    for k in NON_PHYSICAL_META_KEYS:
        c.get("meta", {}).pop(k, None)
    for k in NON_PHYSICAL_SHIP_KEYS:
        c.get("ship", {}).pop(k, None)
    for kind in ("objects", "groups"):
        for o in c.get(kind, []):
            for k in NON_PHYSICAL_ITEM_KEYS:
                o.pop(k, None)
    return c


def _renaming(old: Optional[dict], new: dict) -> Optional[Dict[str, str]]:
    """{old id: new id} ({} for no renaming) when the scenarios differ only by the names of objects and groups (and
    the references to them: a group's "around", an attitude's target) and by attributes with no effect on the
    physics (NON_PHYSICAL_*), else None: no propagation needed."""
    if old is None:
        return None
    kinds = ("objects", "groups")
    if any(len(old.get(k, [])) != len(new.get(k, [])) for k in kinds):
        return None
    names = {a["id"]: b["id"] for k in kinds for a, b in zip(old.get(k, []), new.get(k, [])) if a["id"] != b["id"]}
    if len(set(names.values())) != len(names):
        return None
    c = _physics(old)

    def retarget(wps):
        for w in wps or []:
            a = w.get("attitude", {})
            if a.get("type") == "target" and a.get("id") in names:
                a["id"] = names[a["id"]]

    retarget(c.get("ship", {}).get("attitude"))
    for k in kinds:
        for o in c.get(k, []):
            o["id"] = names.get(o["id"], o["id"])
            if k == "groups" and o.get("around") in names:
                o["around"] = names[o["around"]]
            retarget(o.get("attitude"))
    return names if c == _physics(new) else None


def cells(sc: dict) -> float:
    """Rows x steps of a normalised scenario (the cost of a propagation)."""
    n = 1 + len(sc.get("objects", [])) + sum(int(g.get("n", 0)) for g in sc.get("groups", []))
    return n * float(sc["meta"]["duration_s"]) / float(sc["meta"]["step_s"])


def _polyline(pts: np.ndarray) -> Optional[pv.PolyData]:
    pts = pts[np.isfinite(pts[:, 0])]
    return pv.lines_from_points(pts) if len(pts) >= 2 else None


def eph_sun_radius_km() -> float:
    return sgeo.SUN_RADIUS_KM


def _rgb(hex_color: str) -> Tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


@lru_cache(maxsize=4)
def _texture(body: str) -> Optional[pv.Texture]:
    """Equirectangular texture of a body (TEXTURE_DIR/TEXTURE_FILES[body]; the Moon falls back on the procedural
    one)."""
    path = TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")
    if path.exists():
        tex = pv.Texture(str(path))
    elif body == "moon":
        tex = pv.Texture(cgeo.moon_texture())
    else:
        return None
    tex.interpolate = True
    tex.mipmap = True
    return tex


@lru_cache(maxsize=4)
def _body_mesh(radius_km: float, n_lon: int = 180, n_lat: int = 90) -> pv.PolyData:
    """Sphere in the body-fixed frame (Z the pole, X the prime meridian) with equirectangular texture coordinates:
    u = (longitude + 180) / 360 (the map's left edge at -180 deg), v = (latitude + 90) / 180 (north at the top)."""
    lon = np.radians(np.linspace(-180.0, 180.0, n_lon + 1))
    lat = np.radians(np.linspace(-90.0, 90.0, n_lat + 1))
    lo, la = np.meshgrid(lon, lat, indexing="ij")
    pts = radius_km * np.stack([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)], axis=-1).reshape(-1, 3)
    uv = np.stack([(np.degrees(lo) + 180.0) / 360.0, (np.degrees(la) + 90.0) / 180.0], axis=-1).reshape(-1, 2)
    i, j = np.meshgrid(np.arange(n_lon), np.arange(n_lat), indexing="ij")
    a = (i * (n_lat + 1) + j).ravel()
    quads = np.stack([np.full(a.size, 4), a, a + n_lat + 1, a + n_lat + 2, a + 1], axis=1).ravel()
    mesh = pv.PolyData(pts, faces=quads)
    mesh.active_texture_coordinates = uv
    return mesh.compute_normals(auto_orient_normals=True)


def _earth_matrix(epoch, t_s: float) -> np.ndarray:
    """Earth-fixed -> ICRF: a turn by the Greenwich mean sidereal time about the pole (ICRF Z, as the propagation,
    P2; IAU 1982 GMST, UT1 taken as UTC)."""
    d = ((epoch - eph._J2000).total_seconds() + t_s) / 86400.0
    th = math.radians((280.46061837 + 360.98564736629 * d) % 360.0)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


SUN_ROTATION = dict(P.get("sim.sun_rotation"))     # IAU pole and prime meridian of the Sun


def _sun_matrix(epoch, t_s: float) -> np.ndarray:
    """Sun-fixed -> ICRF (user, 29 October 2026: the bodies turn on themselves): Z the IAU pole, X the prime meridian
    W = w0 + rate x days from J2000 (IAU WGCCRE: R = Rz(ra0 + 90) Rx(90 - dec0) Rz(W))."""
    d = ((epoch - eph._J2000).total_seconds() + t_s) / 86400.0
    a = math.radians(float(SUN_ROTATION["pole_ra_deg"]) + 90.0)
    b = math.radians(90.0 - float(SUN_ROTATION["pole_dec_deg"]))
    w = math.radians((float(SUN_ROTATION["w0_deg"]) + float(SUN_ROTATION["rate_deg_day"]) * d) % 360.0)

    def rz(x):
        c, s = math.cos(x), math.sin(x)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    cb, sb = math.cos(b), math.sin(b)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cb, -sb], [0.0, sb, cb]])
    return rz(a) @ rx @ rz(w)


def _moon_matrix(moon_geo_km: np.ndarray) -> np.ndarray:
    """Moon-fixed -> ICRF: Z the IAU pole of [sim.bodies], X (longitude 0) toward the Earth (synchronous rotation,
    librations left out)."""
    b = scn.bodies()["moon"]
    z = frames.unit(float(b["pole_ra_deg"]), float(b["pole_dec_deg"]))
    x = -moon_geo_km - (-moon_geo_km @ z) * z
    x = x / np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def _nice(x: float) -> float:
    """The 1-2-5 round value nearest to x in log (ring ranges)."""
    e = math.floor(math.log10(x))
    m = x / 10.0 ** e
    best = min((1.0, 2.0, 5.0, 10.0), key=lambda c: abs(math.log10(m / c)))
    return best * 10.0 ** e


def _pairs(n: int) -> np.ndarray:
    """Lines of segment j from point j to point n + j."""
    return np.column_stack([np.full(n, 2), np.arange(n), n + np.arange(n)]).ravel()


def _add_atmosphere(p, name: str):
    """The Earth's atmosphere: translucent shells (ATM_SHELLS x a unit radius, placed by the Earth's matrix) drawn
    by their far (inner) faces only, so the Earth hides their middle and they show as a halo at the limb, brighter
    at the surface where they overlap."""
    shells = pv.merge([_body_mesh(k) for k in ATM_SHELLS])
    return p.add_mesh(shells, color=ATM_COLOR, opacity=ATM_OPACITY, lighting=False, culling="front", name=name,
                      render=False)


class _Labels:
    """Text labels at 3D points, always on top (vtkLabeledDataMapper): one actor, its points and texts updated in
    place (pyvista's add_point_labels rebuilds its whole pipeline at each call)."""

    def __init__(self, plotter, color: str, size: int):
        from vtkmodules.vtkCommonCore import vtkPoints, vtkStringArray
        from vtkmodules.vtkCommonDataModel import vtkPolyData
        from vtkmodules.vtkRenderingCore import vtkActor2D
        from vtkmodules.vtkRenderingLabel import vtkLabeledDataMapper
        self.points, self.texts = vtkPoints(), vtkStringArray()
        self.texts.SetName("labels")
        self.poly = vtkPolyData()
        self.poly.SetPoints(self.points)
        self.poly.GetPointData().AddArray(self.texts)
        self.mapper = vtkLabeledDataMapper()
        self.mapper.SetInputData(self.poly)
        self.mapper.SetLabelModeToLabelFieldData()
        self.mapper.SetFieldDataName("labels")
        tp = self.mapper.GetLabelTextProperty()
        tp.SetColor(*(c / 255.0 for c in _rgb(color)))
        tp.SetFontSize(size)
        tp.SetShadow(False)
        tp.SetBold(False)
        tp.SetItalic(False)
        tp.SetJustificationToLeft()
        self.actor = vtkActor2D()
        self.actor.SetMapper(self.mapper)
        plotter.add_actor(self.actor, reset_camera=False, render=False)
        self._last: List[str] = []

    def set(self, pts: np.ndarray, texts: List[str]) -> None:
        if list(texts) != self._last:
            self.texts.SetNumberOfValues(len(texts))
            for i, t in enumerate(texts):
                self.texts.SetValue(i, f" {t}")
            self._last = list(texts)
        self.points.SetNumberOfPoints(len(pts))
        for i, q in enumerate(pts):
            self.points.SetPoint(i, float(q[0]), float(q[1]), float(q[2]))
        self.points.Modified()
        self.texts.Modified()
        self.poly.Modified()
        self.actor.SetVisibility(True)


INFO_LABEL_W = 17                                 # characters: the labels' column of the Info box


def _kv(label: str, value: str) -> str:
    """One line of the Info box: its label, then its value (one piece of information per line)."""
    return f"{label:<{INFO_LABEL_W}}{value}"


def _vec_lines(label: str, vec, unit: str, dec: int) -> List[str]:
    """A vector on three lines, one component each."""
    return [_kv(f"{label} {a}", f"{float(x):.{dec}f} {unit}") for a, x in zip("xyz", vec)]


def _title(text: str) -> QLabel:
    """A section title among the view's switches."""
    lb = QLabel(f"{text}:")
    lb.setStyleSheet("font-weight: bold;")
    return lb


def _segments(a: np.ndarray, b: np.ndarray) -> pv.PolyData:
    pts = np.empty((2 * len(a), 3))
    pts[0::2], pts[1::2] = a, b
    lines = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(len(a))])
    return pv.PolyData(pts, lines=lines)


class SimView(QWidget):
    """3D views (S6) and playback (S7) of the global simulator."""
    run_shown = pyqtSignal()                      # a run (propagation or journal) is shown

    def __init__(self, grid: Optional[cgeo.SphereGrid] = None, get_path: Callable[[], Optional[str]] = lambda: None,
                 parent=None):
        super().__init__(parent)
        self.grid = grid or cgeo.SphereGrid(float(P.get("tool.display.grid_res_deg")))
        self.get_path = get_path
        self.pending: Optional[dict] = None       # latest valid scenario from the editor
        self.sc: Optional[dict] = None
        self.truth = None
        self.g: Optional[sgeo.Geometry] = None
        self.t = 0.0
        self.worker: Optional[SimWorker] = None
        self._queued: Optional[Callable] = None
        self.mission: Optional[dict] = None       # D (phase 6 merge): the mission being edited, drawn live
        self._hidden_ids: set = set()             # its sectors hidden (ticked off in the mission panel)
        self._reset_dynamic()
        self._r_log = (1.0, 10.0)
        self.selected: Optional[str] = None

        # ---- B14 (user, 29 October 2026, replaces the ribbon B13): the view's controls (kind, centre, recentre,
        # "Layers") in the playback bar, the events as marks on its slider; the propagation automatic, its
        # state in the window's status bar (status_widgets), "Propagate" in the window's header only when the
        # scenario is too large for the automatic one; "Open run..." and "Record run..." in the window's menu
        # (file_commands); the playback bar under the view
        self.tg_view = uc.Toggle("Ship view", off_text="Global view",          # user: a switch, not a list
                                 tip="The global view (inertial, around the central body) or the ship view")
        self.tg_view.toggled.connect(lambda _on: (self._show_view_options(), self._rebuild()))
        self.bt_prop = uc.tool_button("Propagate", self.propagate, "SP_BrowserReload",
                                      tip="Propagate the scenario on screen (too large for the automatic propagation)")
        self.bt_prop.setVisible(False)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setMaximumWidth(160)
        self.progress.setFormat("%p %")
        self.status = QLabel("No propagation yet")

        # ---- L3 / V3 / V5 toggles
        self.cb_labels = uc.Toggle("labels", True)
        self.cb_vectors = uc.Toggle("vectors / noses")
        self.cb_trails = uc.Toggle("trails", True)
        # the body the view is centred on and follows: chosen by a double-click on it in the view (an object,
        # OWNSHIP, or in the global view the Earth, the Moon, the Sun), back to the default by a double-click on
        # nothing (user, 29 October 2026: no "centre on" list)
        self.centre = CENTRAL
        self.cb_volumes = uc.Toggle("mission volumes", True)
        self.cb_bodies = uc.Toggle("celestial objects")
        self.cb_bodies.setToolTip("Ship view: the Earth, the Moon and the Sun at their true directions (1:1: true "
                                  "distances and sizes; log scale: the view's distance, their true apparent size)")
        self.cb_bodies.toggled.connect(lambda _on: self._draw_frame())
        self.cb_true = uc.Toggle("1:1 scale")
        self.cb_true.setToolTip("Ship view: true distances in km (off: log range scale from the nearest to the "
                                "farthest object of the run)")
        self.cb_true.toggled.connect(lambda _on: self._rebuild() if not self._global() else None)
        for cb in (self.cb_labels, self.cb_vectors, self.cb_trails):
            cb.toggled.connect(lambda _on: self._draw_frame())
        self.cb_labels.toggled.connect(lambda _on: (self._place_volumes(), self._render()))   # the sectors' labels
        self.cb_volumes.toggled.connect(lambda _on: self._draw_volumes(force=True))
        self.cb_proj = uc.Toggle("range projection")
        self.cb_proj.setToolTip("Ship view: a copy of each object in the plane of the range rings, at its range from "
                                "the ship (read on the rings), linked to the object by the arc of the sphere of that "
                                "range around the ship")
        self.cb_proj.toggled.connect(lambda _on: self._draw_frame())
        # D: the faces' responsibility zones (ship view), one switch for the four (user, 29 October 2026)
        self.cb_zones = uc.Toggle("responsibility zones", True)
        self.cb_zones.toggled.connect(lambda _on: (self._show_faces(), self._render()))
        # the axes the ship carries (user, 29 October 2026): its X, Y, Z axes and the faces' (arrays') normals of
        # the model, the 1:1 ship axes, the attitude triad of the global view
        self.cb_axes = uc.Toggle("ship axes", True)
        self.cb_axes.toggled.connect(lambda _on: (self._show_axes(), self._draw_frame()))
        self._common = (self.cb_labels, self.cb_vectors, self.cb_trails, self.cb_axes)
        self._ship_only = (self.cb_volumes, self.cb_zones, self.cb_true, self.cb_bodies, self.cb_proj)
        self.bt_recentre = uc.tool_button("Recentre", self.recentre, "SP_DialogResetButton",
                                          tip="Back to the view's centre (the ship, the central body or the body "
                                              "double-clicked), zoom and orientation kept: undoes a pan")

        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(400)
        sv.lock_pivot(self.plotter)
        sv.install_arcball(self.plotter, ARCBALL_RADIUS)   # left drag: virtual trackball (roll outside its circle)
        self.plotter.iren.interactor.AddObserver("LeftButtonPressEvent", self._on_click, 1.0)
        self.plotter.set_background(BG_COLOR)
        self.plotter.renderer.SetNearClippingPlaneTolerance(NEAR_TOL)
        self.plotter.renderer.AddObserver("StartEvent", lambda *_a: self._clip())
        self.plotter.add_axes(color=TEXT_COLOR)
        self.plotter.renderer.GetActiveCamera().AddObserver("ModifiedEvent", lambda *_a: self._on_camera())
        # user, 29 October 2026: the view's options under the Info box, in the side column (no longer in the
        # playback bar), one per line; the layers as on / off switches (the ship view's own ones and their title
        # shown in the ship view only)
        self._ship_titles = (_title("Ship view"),)
        self.view_panel = QGroupBox("3D view")
        vcol = QVBoxLayout(self.view_panel)
        vcol.setContentsMargins(8, 4, 8, 6)
        vcol.setSpacing(4)
        head = QHBoxLayout()
        head.addWidget(self.tg_view)
        head.addStretch(1)
        head.addWidget(self.bt_recentre)
        vcol.addLayout(head)
        for w in (_title("Layers"),) + self._common + self._ship_titles + self._ship_only:
            vcol.addWidget(w)

        # ---- L1 playback
        play = QHBoxLayout()
        self.bt_back = QPushButton("|<")
        self.bt_back.clicked.connect(lambda: self.step(-1))
        self.bt_play = QPushButton("Play")
        self.bt_play.clicked.connect(self.toggle_play)
        self.bt_fwd = QPushButton(">|")
        self.bt_fwd.clicked.connect(lambda: self.step(1))
        self.cb_speed = QComboBox()
        self.cb_speed.addItems([f"x{s}" for s in SPEEDS])
        self.cb_speed.setCurrentIndex(1)
        self.slider = uc.MarkedSlider()                 # L4: the events as marks on it
        self.slider.setRange(0, 10000)
        self.slider.valueChanged.connect(self._on_slider)
        self.time_label = QLabel()
        for w in (self.bt_back, self.bt_play, self.bt_fwd, self.cb_speed):
            play.addWidget(w)
        play.addWidget(self.slider, 1)
        play.addWidget(self.time_label)
        QShortcut(QKeySequence(Qt.Key_Space), self, self.toggle_play, context=Qt.WindowShortcut)   # user: Space
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)                # rescheduled by _tick: the event loop always gets its turn
        self.timer.timeout.connect(self._tick)
        self._wall = 0.0
        self._playing = False

        # ---- L5 information
        # (user, 29 October 2026, replaces R3 / B12: a box over the 3D view) the information on the body clicked in
        # the view (a double-click also centres on it) in a column of its own (side_panel, placed by the window right
        # of the options column)
        self.info = QPlainTextEdit()
        self.info.setReadOnly(True)
        self.info.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
        # one piece of information per line (user, 29 October 2026), no horizontal scroll: a line too long for the
        # column wraps under itself
        self.info.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        self.info.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.info.setPlaceholderText("Click a body in the 3D view")
        info_box = QGroupBox("Info")
        il = QVBoxLayout(info_box)
        il.setContentsMargins(4, 4, 4, 4)
        il.addWidget(self.info)
        self.side_panel = QWidget()                               # the column: the Info box, the view's options
        sl = QVBoxLayout(self.side_panel)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.addWidget(info_box, 1)
        sl.addWidget(self.view_panel)
        self.side_panel.setMinimumWidth(220)
        self.events: List[Tuple[float, str]] = []                 # L4: (t, text), sorted
        self.play_bar = play
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.plotter.interactor, 1)
        lay.addLayout(play)
        self._show_view_options()

        self.debounce = QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(1000)
        self.debounce.timeout.connect(self.propagate)
        self._enable_playback(False)

    # ------------------------------------------------------------------ V2 computations
    def set_scenario(self, sc: dict) -> None:
        """The editor's scenario after a valid change (normalised); a change of object or group names, or of
        attributes with no effect on the physics (classification, scenario name), only updates the run shown,
        without propagating again (user, 29 October 2026)."""
        self.pending = sc
        busy = self.worker is not None and self.worker.isRunning()
        names = _renaming(self.sc, sc) if (self.truth is not None and not busy) else None
        if names is not None:
            self.debounce.stop()
            self._rename(sc, names)
            return
        small = cells(sc) <= AUTO_MAX_CELLS
        self.bt_prop.setVisible(not small)
        if small:
            self.debounce.start()

    def _rename(self, sc: dict, names: Dict[str, str]) -> None:
        """The run shown under the new names (rows, group members, attitudes, events), the scenario kept."""
        tr, g = self.truth, self.g
        group = [names.get(x, x) if x else x for x in tr.group_of]
        ids = []
        for i, oid in enumerate(tr.ids):
            if i and group[i] and "#" in oid:
                ids.append(f"{group[i]}#{oid.rsplit('#', 1)[1]}")
            else:
                ids.append(names.get(oid, oid))
        row = dict(zip(tr.ids, ids))
        tr.ids, tr.group_of = ids, group
        tr.att_ids = [row.get(x, x) for x in tr.att_ids]
        tr.att_events = [dict(e, id=row.get(e.get("id"), e.get("id"))) for e in tr.att_events]
        g.ids = [row.get(x, x) for x in g.ids]
        if self.selected:
            self.selected = row.get(self.selected, self.selected)
        self.sc = sc
        self._navigation = engine.Navigation(sc, tr)
        self._fill_lists(row)
        self._fill_events()
        self._colour_rows()
        if self._global():
            self._draw_trajectories()                 # a classification changed: the trajectories' colours too
        self._draw_frame()

    def own_name(self) -> str:
        """The ship's name in the views (the scenario's ship.name, OWNSHIP by default)."""
        return scn.ship_name(self.sc) if self.sc else OWNSHIP

    def _colour_rows(self) -> None:
        """Each row's colour: the colour of its classification (OWNSHIP: friendly)."""
        tr = self.truth
        self._row_ident = ["ship"] + [scn.identity_of(self.sc, tr.ids[i], tr.group_of[i] or None)
                                      for i in range(1, len(tr.ids))]
        self._row_rgb = np.array([_rgb(SHIP_COLOR if i == 0 else IDENTITY_COLORS.get(c, "#888888"))
                                  for i, c in enumerate(self._row_ident)], np.uint8)

    def _fill_lists(self, row: Optional[Dict[str, str]] = None) -> None:
        """The body the view is centred on kept (renamed by row), the default centre if it is no longer in the run."""
        cur = (row or {}).get(self.centre, self.centre)
        self.centre = cur if cur in [OWNSHIP] + self.g.ids + list(BODY_NAMES) else CENTRAL

    def set_centre(self, name: str) -> None:
        """Centre the view on this body (CENTRAL: the default centre) and frame what is around it."""
        self.centre = name or CENTRAL
        self._apply_centre(reframe=True)

    def propagate(self) -> None:
        if self.pending is not None:
            self._start(propagate_job(self.pending), "propagation")

    def run_journal(self) -> None:
        path = self.get_path()
        if path is None:
            QMessageBox.information(self, "Simulator", "Save the scenario first: the journal is written next to it.")
            return
        self._start(run_job(path), "run")

    def open_journal(self) -> None:
        from gui.scenario_tab import SCENARIO_DIR            # the journals are written next to their scenarios
        start = os.path.dirname(self.get_path()) if self.get_path() else SCENARIO_DIR
        path, _ = QFileDialog.getOpenFileName(self, "Open a run", start, "Simulator journal (*.npz)")
        if not path:
            return
        try:
            sc, truth, g, _nav, _meta = engine.load_journal(path)
        except (OSError, ValueError, KeyError) as exc:
            QMessageBox.critical(self, "Simulator", f"{path}: {exc}")
            return
        self.show_run(sc, truth, g, f"journal {path}")

    def _start(self, job: Callable, what: str) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.worker.cancelled = True
            self._queued = (job, what)
            return
        self.worker = SimWorker(job, self)
        self.worker.progress.connect(lambda st, f: (self.progress.setValue(int(1000 * f)),
                                                    self.status.setText(f"{st}...")))
        self.worker.done.connect(lambda res: self.show_run(*res, label=what))
        self.worker.failed.connect(lambda msg: self.status.setText(f"failed: {msg}"))
        self.worker.finished.connect(self._after_worker)
        self.status.setText(f"{what}...")
        self.worker.start()

    def _after_worker(self) -> None:
        if self._queued is not None:
            job, what = self._queued
            self._queued = None
            self.worker = None
            self._start(job, what)

    def show_run(self, sc: dict, truth, g: sgeo.Geometry, label: str = "") -> None:
        """Display a propagated run (from the worker or a journal)."""
        self.sc, self.truth, self.g = sc, truth, g
        self.progress.setValue(1000)
        self.status.setText(f"{label}: {len(g.ids)} objects, {float(sc['meta']['duration_s']):g} s")
        self.t = min(self.t, self.duration())
        rng = g.range_km[np.isfinite(g.range_km)]
        lo, hi = (max(float(rng.min()), 1e-3), max(float(rng.max()), 1e-3)) if rng.size else (1.0, 10.0)
        if hi < LOG_MIN_SPAN * lo:                   # one object, or all at one range: the log scale keeps a span
            mid = math.sqrt(lo * hi)
            lo, hi = mid / math.sqrt(LOG_MIN_SPAN), mid * math.sqrt(LOG_MIN_SPAN)
        self._r_log = (lo, hi)
        self._log_span = None                             # range zoom: from the run's farthest object
        self._log_d = None
        self._navigation = engine.Navigation(sc, truth)
        self._triad_rgb = np.array([_rgb(cgeo.COLORS[k]) for k in ("axis_x", "axis_y", "axis_z")], np.uint8)
        self._colour_rows()
        self._fill_lists()
        self._fill_events()
        self._enable_playback(True)
        self._rebuild()
        self.run_shown.emit()

    # ------------------------------------------------------------------ time
    def duration(self) -> float:
        return float(self.sc["meta"]["duration_s"]) if self.sc else 0.0

    def set_controls_visible(self, on: bool) -> None:
        """P1 (user, 29 October 2026): no control of the view (its panel, the events, the playback bar, the state)
        before a file is created or opened; the 3D view alone."""
        self._controls = on
        self.view_panel.setVisible(on)
        for i in range(self.play_bar.count()):
            w = self.play_bar.itemAt(i).widget()
            if w is not None:
                w.setVisible(on)
        for w in (self.status, self.progress, self.bt_prop):
            if not on:
                w.setVisible(False)
        if on:
            self.status.setVisible(True)
            self.progress.setVisible(True)
        self.side_panel.setVisible(on)

    def _enable_playback(self, on: bool) -> None:
        for w in (self.bt_back, self.bt_play, self.bt_fwd, self.slider, self.cb_speed):
            w.setEnabled(on)

    def set_time(self, t: float) -> None:
        self.t = float(np.clip(t, 0.0, self.duration()))
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(10000 * self.t / max(self.duration(), 1e-9))))
        self.slider.blockSignals(False)
        self._draw_frame()
        if not self._playing:
            self._draw_volumes()

    def _on_slider(self, v: int) -> None:
        if self.sc is not None:
            self.set_time(v / 10000.0 * self.duration())

    def step(self, k: int) -> None:
        if self.truth is None:
            return
        tl = self.truth.t_s
        i = int(np.searchsorted(tl, self.t + (1e-9 if k > 0 else -1e-9), side="right" if k > 0 else "left"))
        i = i if k > 0 else i - 1
        self.set_time(float(tl[int(np.clip(i, 0, len(tl) - 1))]))

    def toggle_play(self) -> None:
        if self._playing:
            self._playing = False
            self.timer.stop()
            self.bt_play.setText("Play")
            self._draw_volumes()
        elif self.sc is not None:
            if self.t >= self.duration():
                self.t = 0.0
            self._playing = True
            self._wall = time.perf_counter()
            self.timer.start(0)
            self.bt_play.setText("Pause")

    def _tick(self) -> None:
        """One frame; the next is scheduled after it with at least MIN_IDLE_MS left to the event loop, so the mouse
        (zoom, rotation) stays live during the playback whatever a frame costs."""
        if not self._playing:
            return
        now = time.perf_counter()
        dt = (now - self._wall) * SPEEDS[self.cb_speed.currentIndex()]
        self._wall = now
        t = self.t + dt
        if t >= self.duration():
            self.toggle_play()
        self.set_time(t)
        self._draw_volumes()
        if self._playing:
            spent_ms = 1000.0 * (time.perf_counter() - now)
            self.timer.start(int(max(MIN_IDLE_MS, 1000.0 / max(FPS, 1) - spent_ms)))

    def _k(self) -> int:
        """Index of the recorded instant nearest to t."""
        return int(np.clip(np.rint(np.interp(self.t, self.g.t_s, np.arange(len(self.g.t_s)))), 0, len(self.g.t_s) - 1))

    # ------------------------------------------------------------------ L4 events
    def _fill_events(self) -> None:
        ev: List[Tuple[float, str]] = []
        tr, sc = self.truth, self.sc
        for i, oid in enumerate(tr.ids):
            if i and tr.t_appear_s[i] > 0.0:
                ev.append((float(tr.t_appear_s[i]), f"{oid} appears"))
            if np.isfinite(tr.t_pn_start_s[i]):
                ev.append((float(tr.t_pn_start_s[i]), f"{oid} starts terminal guidance (PN)"))
            if np.isfinite(tr.t_impact_s[i]):
                ev.append((float(tr.t_impact_s[i]), f"{oid} impacts the {tr.body.name}"))
        for o in sc.get("objects", []) + sc.get("groups", []):
            for m in o.get("manoeuvres", []):
                if m["type"] in ("impulse", "burn"):
                    ev.append((float(m["t_s"]), f"{o['id']} {m['type']}"))
        for e in tr.att_events:
            ev.append((float(e["t_s"]), f"{e['id']} {e['kind']}"))
        for a, b in engine._periods(self.g.t_s, self.g.ship_shadow > 0.0):
            ev.append((a, "ship enters the shadow"))
            if b < self.duration():
                ev.append((b, "ship leaves the shadow"))
        self.events = sorted(ev)
        span = max(self.duration(), 1e-9)
        self.slider.set_marks([(int(round(10000 * t / span)), f"{t:.1f} s  {text}") for t, text in self.events])

    # ------------------------------------------------------------------ drawing
    def _global(self) -> bool:
        return not self.tg_view.isChecked()

    def _show_view_options(self) -> None:
        """B4 / B14: the ship view's own switches (and their titles) shown in the ship view only."""
        for w in self._ship_only + self._ship_titles:
            w.setEnabled(not self._global())
            w.setVisible(not self._global())

    def status_widgets(self) -> list:
        """The propagation's state, for the window's status bar."""
        return [self.status, self.progress]

    def file_commands(self) -> list:
        """(text, slot, icon) of the simulator in the window's menu."""
        return [("Record run...", self.run_journal, "SP_MediaPlay"),
                ("Open run...", self.open_journal, "SP_DialogOpenButton")]

    def _rebuild(self) -> None:
        """The static layer of the current view, then the frame."""
        p = self.plotter
        p.clear()
        p.set_background(BG_COLOR)
        p.remove_all_lights()
        self._reset_dynamic()
        if self.truth is None:
            if not self._global() and self.mission is not None:     # D: the mission alone, before any propagation
                self._build_ship()
                self._draw_volumes(force=True)
            p.render()
            return
        if self._global():
            self._build_global()
        else:
            self._build_ship()
        self._draw_frame()
        self._draw_volumes(force=True)

    def _build_global(self) -> None:
        p, tr = self.plotter, self.truth
        body = tr.body
        for b in ("earth", "moon"):                    # both bodies in every scenario, placed at each frame
            rad = float(scn.bodies()[b]["radius_km"])
            tex = _texture(b)
            p.add_mesh(_body_mesh(rad), texture=tex, color=None if tex is not None else EARTH_COLOR,
                       smooth_shading=True, ambient=0.12, culling="back", name=f"body_{b}", render=False)
        _add_atmosphere(p, "body_earth_atm")
        tex = _texture("sun")                          # unit sphere, scaled and placed by _place_bodies
        p.add_mesh(_body_mesh(1.0), texture=tex, color=None if tex is not None else SUN_COLOR, lighting=False,
                   culling="back", name="body_sun", render=False)
        self._sun_light = pv.Light(light_type="scene light", intensity=1.0)
        self._sun_light.positional = False
        p.add_light(self._sun_light)
        head = pv.Light(light_type="headlight", intensity=0.25)    # the night side stays readable
        p.add_light(head)
        orbit = self._moon_orbit()
        p.add_mesh(pv.lines_from_points(orbit), color="#8a8a8a", line_width=1, opacity=0.6, name="moon_orbit",
                   render=False)
        self._draw_trajectories()
        pts, labels = [], []
        for t, text in self._marker_events():
            r = sgeo.states_at(tr, t)[0]
            oid = text.split()[0]
            if oid in tr.ids and np.isfinite(r[tr.row(oid), 0]):
                pts.append(r[tr.row(oid)])
                labels.append(text)
        if pts:
            p.add_point_labels(np.array(pts), labels, font_size=9, point_size=8, point_color=MUTED_COLOR,
                               shape=None, text_color=MUTED_COLOR, name="markers", render=False)
        extent = max(float(np.nanmax(np.abs(tr.r_km))) if np.isfinite(tr.r_km).any() else 0.0, 2.0 * body.radius_km)
        self._home = (-extent, extent, -extent, extent, -extent, extent)
        p.reset_camera(render=False, bounds=self._home)
        p._pivot = (0.0, 0.0, 0.0)
        self._centre_pending = True

    def _moon_orbit(self) -> np.ndarray:
        """The Moon's path around the Earth over one sidereal month centred on the run (the Earth's around the Moon
        when the Moon is the central body), km from the central body."""
        mid = 0.5 * self.duration()
        ts = mid + np.linspace(-0.5, 0.5, 361) * SIDEREAL_MONTH_S
        ep = scn.epoch(self.sc)
        pts = np.array([eph.moon_icrf_km(eph.centuries_tt(ep, float(t))) for t in ts])
        return pts if self.truth.body.name == "earth" else -pts

    def _place_bodies(self) -> None:
        """V3: the Earth turned by its sidereal angle (pole on the ICRF Z axis as in the propagation, P2), the Moon
        with its near side toward the Earth (synchronous rotation, IAU pole), at their true places at t."""
        pos_b = self._bodies_at()
        moon_geo = pos_b["moon"] - pos_b["earth"]
        for b, pos, m in (("earth", pos_b["earth"], _earth_matrix(scn.epoch(self.sc), self.t)),
                          ("moon", pos_b["moon"], _moon_matrix(moon_geo)),
                          ("sun", pos_b["sun"], eph_sun_radius_km() * _sun_matrix(scn.epoch(self.sc), self.t))):
            a = self.plotter.actors.get(f"body_{b}")
            if a is not None:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = m, pos
                a.user_matrix = u
            if b == "earth" and "body_earth_atm" in self.plotter.actors:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = float(scn.bodies()["earth"]["radius_km"]) * np.eye(3), pos
                self.plotter.actors["body_earth_atm"].user_matrix = u
        light = getattr(self, "_sun_light", None)
        if light is not None:                          # sunlight: parallel rays from the Sun's direction
            light.position = tuple(pos_b["sun"])
            light.focal_point = (0.0, 0.0, 0.0)

    def _bodies_at(self) -> Dict[str, np.ndarray]:
        """Earth, Moon and Sun at t, km from the central body (true places)."""
        T = eph.centuries_tt(scn.epoch(self.sc), self.t)
        moon, sun = eph.moon_icrf_km(T), eph.sun_icrf_km(T)
        centre = np.zeros(3) if self.truth.body.name == "earth" else moon
        return {"earth": -centre, "moon": moon - centre, "sun": sun - centre}

    def _body_radius(self, b: str) -> float:
        return eph_sun_radius_km() if b == "sun" else float(scn.bodies()[b]["radius_km"])

    def _clip(self) -> None:
        """Before every render: the near plane at NEAR_FRAC of the camera's distance to its focal point (at least),
        the far one as VTK computes it (the Sun at 1 AU stays drawn). With the camera inside the scene's bounds (the
        rings all around) VTK put the near plane at NEAR_TOL x far, almost on the camera, and the depth test lost
        all precision: the rings and lines showed through the hull (user, 29 October 2026)."""
        ren = self.plotter.renderer
        cam = ren.GetActiveCamera()
        self._in_camera = True                        # a clipping change only: no zoom bound, no ring update
        try:
            ren.ResetCameraClippingRange()
            near, far = cam.GetClippingRange()
            near = max(near, NEAR_FRAC * cam.GetDistance())
            cam.SetClippingRange(near, max(far, 1.01 * near))
        finally:
            self._in_camera = False

    def _max_camera_distance(self) -> Optional[float]:
        """Camera distance at which the half-height of the view (its smaller size on a landscape screen: the bound is
        then reached in every direction, user 29 October 2026) stands for MAX_VIEW_KM (global view and ship
        view at 1:1: km; ship view in log scale: its extended log law)."""
        if self.truth is None:
            return None
        cam = self.plotter.camera
        w, h = self.plotter.ren_win.GetSize()
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) * min(w / max(h, 1), 1.0)
        edge = MAX_VIEW_KM if (self._global() or self.cb_true.isChecked()) else self._ring_radius(MAX_VIEW_KM)
        return edge / k if k > 0 and edge > 0 else None

    def _min_camera_distance(self) -> Optional[float]:
        """Camera distance at which the innermost range ring (RING_STEP_FRAC of the half-height) stands for
        MIN_RING_KM (global view and ship view at 1:1: km; ship view in log scale: its extended log law)."""
        if self.truth is None:
            return None
        cam = self.plotter.camera
        w, h = self.plotter.ren_win.GetSize()
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) * min(w / max(h, 1), 1.0) * RING_STEP_FRAC
        ring = MIN_RING_KM if (self._global() or self.cb_true.isChecked()) else self._ring_radius(MIN_RING_KM)
        return ring / k if k > 0 and ring > 0 else None

    def _on_camera(self) -> None:
        """Every camera change (wheel, buttons, follow): the zoom out held at MAX_VIEW_KM, then the range rings."""
        if getattr(self, "_in_camera", False):
            return
        self._in_camera = True
        try:
            cam = self.plotter.camera
            if self._log_zoom():
                # log scale: the camera keeps its distance, the zoom changes the span of the scale (range zoom)
                d = cam.GetDistance()
                if abs(d - self._log_d) > 1e-9 * self._log_d:
                    span = float(np.clip(self._span() * d / self._log_d, LOG_SPAN_MIN, self._log_span_max()))
                    f = np.asarray(cam.GetFocalPoint())
                    u = np.asarray(cam.GetPosition()) - f
                    cam.SetPosition(*(f + u / np.linalg.norm(u) * self._log_d))
                    self.plotter.renderer.ResetCameraClippingRange()
                    if span != self._log_span:
                        self._log_span = span
                        self._redraw_soon()
            d_max, d_min = (None, None) if self._log_zoom() else (self._max_camera_distance(),
                                                                 self._min_camera_distance())
            d = cam.GetDistance()
            target = d_max if (d_max is not None and d > d_max) else d_min if (d_min is not None and d < d_min) else None
            if target is not None:
                f = np.asarray(cam.GetFocalPoint())
                u = np.asarray(cam.GetPosition()) - f
                cam.SetPosition(*(f + u / np.linalg.norm(u) * target))
                self.plotter.renderer.ResetCameraClippingRange()
        finally:
            self._in_camera = False
        self._scale_model()
        self._update_rings()

    def _log_zoom(self) -> bool:
        """The ship view in log scale, built: its zoom is the range zoom."""
        return (self.truth is not None and not self._global() and not self.cb_true.isChecked()
                and getattr(self, "_log_d", None) is not None)

    def _redraw_soon(self) -> None:
        """One frame redrawn after the camera event (the objects follow the scale)."""
        if not getattr(self, "_redraw_pending", False):
            self._redraw_pending = True

            def go():
                self._redraw_pending = False
                if not self._playing:
                    self._draw_frame()
            QTimer.singleShot(0, go)

    def zoom(self, f: float) -> None:
        """Move the camera toward its focal point by f (as the wheel; the triad keeps its size on screen)."""
        self.plotter.camera.Dolly(f)
        self.plotter.renderer.ResetCameraClippingRange()
        if not self._playing:
            self._draw_frame()

    def _target(self) -> Optional[np.ndarray]:
        """Position (view coordinates) of the body the view is centred on at t; None: the default centre."""
        name = self.centre
        if self.truth is None or name in ("", CENTRAL):
            return None
        if not self._global():
            return np.zeros(3) if name == OWNSHIP else getattr(self, "_screen_pts", {}).get(name)
        if name in BODY_NAMES:
            return self._bodies_at()[BODY_NAMES[name]]
        r, _ = sgeo.states_at(self.truth, self.t)
        i = 0 if name == OWNSHIP else (self.truth.row(name) if name in self.truth.ids else None)
        return None if i is None or not np.isfinite(r[i, 0]) else r[i]

    def _apply_centre(self, reframe: bool = False) -> None:
        """Centre the camera on the chosen body (reframe: also a distance framing what is around it), or back to the
        default view of the central body / the ship."""
        if self.truth is None:
            return
        p = self.plotter
        cam = p.camera
        tgt = self._target()
        if tgt is None:
            if reframe:
                if self._global():
                    p.reset_camera(render=False, bounds=getattr(self, "_home", None))
                    p._pivot = (0.0, 0.0, 0.0)
                else:
                    self._ship_home()
                p.render()
            return
        if reframe:
            view = np.asarray(cam.GetPosition()) - np.asarray(cam.GetFocalPoint())
            view = view / max(np.linalg.norm(view), 1e-12)
            if self._global() and self.centre in BODY_NAMES:
                dist = 4.0 * self._body_radius(BODY_NAMES[self.centre])
            elif self._global():
                r, _ = sgeo.states_at(self.truth, self.t)
                d = np.linalg.norm(r - tgt, axis=1)
                d = d[np.isfinite(d) & (d > 1e-9)]
                span = float(np.clip(d.max() if d.size else 0.0, FOLLOW_MIN_KM, 4.0 * self.truth.body.radius_km))
                dist = 4.0 * span
            elif self.cb_true.isChecked():
                dist = max(1.0, 0.3 * float(np.linalg.norm(tgt))) if np.linalg.norm(tgt) > 0 else 0.3 * self._r_log[1]
            else:                                      # log scale: the range zoom keeps the camera distance
                dist = self._log_d if self._log_zoom() else 1.5
            cam.SetFocalPoint(*tgt)
            cam.SetPosition(*(tgt + dist * view))
        else:                                          # follow: the camera keeps its offset from the body
            off = tgt - np.asarray(cam.GetFocalPoint())
            cam.SetFocalPoint(*tgt)
            cam.SetPosition(*(np.asarray(cam.GetPosition()) + off))
        p._pivot = tuple(tgt)
        p.renderer.ResetCameraClippingRange()
        if reframe:
            p.render()

    def _marker_events(self) -> List[Tuple[float, str]]:
        out = []
        tr = self.truth
        for i, oid in enumerate(tr.ids):
            if i and tr.t_appear_s[i] > 0.0:
                out.append((float(tr.t_appear_s[i]), f"{oid} appears"))
            if np.isfinite(tr.t_pn_start_s[i]):
                out.append((float(tr.t_pn_start_s[i]), f"{oid} PN"))
            if np.isfinite(tr.t_impact_s[i]):
                out.append((max(float(tr.t_impact_s[i]) - 1e-6, 0.0), f"{oid} impact"))
        for o in self.sc.get("objects", []):
            for m in o.get("manoeuvres", []):
                if m["type"] in ("impulse", "burn"):
                    out.append((float(m["t_s"]), f"{o['id']} {m['type']}"))
        return out

    def _build_ship(self) -> None:
        """V4: the ship model, scaled at every camera change (_scale_model, user 29 October 2026): log scale, a
        constant size on screen with its faces, normals and axes; 1:1, its true size in km (the face layer, normals and
        axes, of unit size, left out; the ship's axes are the dynamic ones); no equator (a ring without a range)."""
        p = self.plotter
        true = self.cb_true.isChecked()
        acts, labels, axes = [], [], []                  # axes: the model's axes and normals (cb_axes)
        ship = cgeo.build_ship()
        for i, part in enumerate(ship["hull"]):
            acts.append(p.add_mesh(part["mesh"], color=part["color"], smooth_shading=True, name=f"model_hull_{i}",
                                   render=False))
        for d in ship["arrays"]:
            acts.append(p.add_mesh(d["mesh"], color=d["color"], name=f"model_array_{d['k']}", render=False))
        if not true:
            for ax in cgeo.build_axes():
                if not ax.get("no_line"):
                    acts.append(p.add_mesh(ax["line"], color=ax["color"], line_width=2, name=f"model_axis_{ax['text']}",
                                           render=False))
                    axes.append(acts[-1])
                labels.append((np.array([ax["label_pos"]], float), [ax["text"]], ax["color"], 12, True))
            for n in cgeo.build_face_normals():
                acts.append(p.add_mesh(n["mesh"], color=n["color"], line_width=3, name=f"model_normal_{n['k']}",
                                       render=False))
                axes.append(acts[-1])
                labels.append((np.array([n["label_pos"]], float), [n["name"]], n["color"], 14, True))
            # (no equator of the model: a ring carrying no range, turning with the ship, user 29 October 2026)
            scene = cgeo.build_face_sectors(self.grid, -90.0, 90.0)
            for k in range(4):
                if scene.surfaces[k].n_points:
                    acts.append(p.add_mesh(scene.surfaces[k], color=cgeo.FACE_COLORS[k], opacity=0.15, culling="back",
                                           name=f"face_{k}", render=False))
        labs = [(_Labels(p, c, z), pts, t, ax) for pts, t, c, z, ax in labels]
        self._model = {"actors": acts, "labels": [x[:3] for x in labs], "s": None,
                       "axes": axes + [x[0].actor for x in labs if x[3]]}
        self._show_faces()
        self._show_axes()
        p.set_background(BG_COLOR)
        p.enable_lightkit()
        self._centre_pending = True
        self._ship_home()

    def _model_scale(self) -> float:
        """Scale of the model (scene units -> drawn units): 1:1 its true size in km; log scale MODEL_SCREEN x the
        visible half-height at the ship over the length of its axes (build_axes: MODEL_AXES_UNITS)."""
        if self.cb_true.isChecked():
            return KM_PER_MODEL_UNIT
        cam = self.plotter.camera
        d0 = float(np.linalg.norm(np.asarray(cam.GetPosition())))
        half = d0 * math.tan(math.radians(0.5 * cam.GetViewAngle()))
        return MODEL_SCREEN * half / MODEL_AXES_UNITS

    def _scale_model(self) -> None:
        """The model, its labels and (log scale) the mission volumes, which lie on its face layer, at the scale of
        the moment; nothing done when it has not changed."""
        model = getattr(self, "_model", None)
        if model is None or self._global():
            return
        sc = self._model_scale()
        if model["s"] is not None and abs(sc - model["s"]) <= 1e-9 * max(sc, 1e-300):
            return
        model["s"] = sc
        for a in model["actors"]:
            a.SetScale(sc, sc, sc)
        for lab, pts, texts in model["labels"]:
            lab.set(pts * sc, texts)
        self._show_axes()                             # set() shows a label again
        self._place_volumes()

    def _show_axes(self) -> None:
        """The model's axes and the faces' normals (with their labels) shown by the "ship axes" switch."""
        for a in (getattr(self, "_model", None) or {}).get("axes", []):
            a.SetVisibility(self.cb_axes.isChecked())

    def _show_faces(self) -> None:
        """D: the faces' responsibility zones, the four by one switch."""
        for k in range(4):
            a = self.plotter.actors.get(f"face_{k}")
            if a is not None:
                a.SetVisibility(self.cb_zones.isChecked())

    def set_mission(self, mission: Optional[dict], hidden=()) -> None:
        """D (phase 6 merge): the mission being edited (its volumes drawn in the ship view, live) and the ids of its
        sectors hidden; None: no volumes."""
        self.mission = mission
        self._hidden_ids = set(hidden)
        if self._global():
            return
        if self.truth is None:
            self._rebuild()
        else:
            self._draw_volumes(force=True)

    def _place_volumes(self) -> None:
        """Mission volumes: their turn (frame at t against the one they were built with) and, in log scale, the
        model's scale (they lie on its face layer); their labels with them."""
        vol = self._vol
        if not vol:
            return
        k = 1.0 if self.cb_true.isChecked() else (self._model or {}).get("s") or 1.0
        pts = []
        for rot, acts, c in zip(vol.get("rot") or [np.eye(3)] * len(vol["per_sector"]), vol["per_sector"],
                                vol["centres"]):
            u = np.eye(4)
            u[:3, :3] = k * rot
            for a in acts:
                a.user_matrix = u
            pts.append(k * (rot @ c))
        lab = vol.get("labels")
        if lab is not None:
            if pts and self.cb_labels.isChecked():
                lab.set(np.array(pts), vol["texts"])
            else:
                lab.actor.SetVisibility(False)

    def _ship_home(self) -> None:
        """Default camera of the ship view: the iso view, out to the farthest object (SHIP_R1, or its range in km)."""
        p = self.plotter
        self._log_d = None                                # (the camera moves below: not a range zoom)
        sv.set_default_camera(p)
        k = (self._r_log[1] / 1.2) if self.cb_true.isChecked() else max(1.0, SHIP_R1 / 1.5)
        # the camera aims at the ship's centre, the centre of the range rings and the pivot of the rotations (the
        # mission-preparation default aims 0.12 above it: the rings moved as the view turned, user 29 October 2026)
        pos = (np.asarray(p.camera.position) - np.asarray(p.camera.focal_point)) * k
        if self.truth is not None:
            # the default camera on the side away from the central body (else it may stand inside it: celestial
            # objects drawn, both scales)
            r, _ = sgeo.states_at(self.truth, self.t)
            dc = -(r[0] / np.linalg.norm(r[0])) @ self.truth.attitude("ship", self.t)
            if pos @ dc > 0.0:
                pos = pos - 2.0 * (pos @ dc) * dc
        p.camera.focal_point = (0.0, 0.0, 0.0)
        p.camera.position = tuple(pos)
        p._pivot = (0.0, 0.0, 0.0)
        p.renderer.ResetCameraClippingRange()
        if not self.cb_true.isChecked():
            self._log_d = float(np.linalg.norm(pos))              # the log scale keeps this camera distance
            if getattr(self, "_log_span", None) is None:
                self._log_span = self._log_span_init()

    def recentre(self) -> None:
        """Back to the view's centre (button "Recentre"): the body double-clicked, else the ship (ship view)
        or the central body (global view); the zoom and the orientation kept (undoes a pan)."""
        if self.truth is None:
            return
        tgt = self._target()
        tgt = np.zeros(3) if tgt is None else np.asarray(tgt, float)
        cam = self.plotter.camera
        off = tgt - np.asarray(cam.GetFocalPoint())
        cam.SetFocalPoint(*tgt)
        cam.SetPosition(*(np.asarray(cam.GetPosition()) + off))
        self.plotter._pivot = tuple(tgt)
        self.plotter.renderer.ResetCameraClippingRange()
        self._render()

    def _ship_r(self, rng_km: np.ndarray) -> np.ndarray:
        """Radius drawn for ranges in the ship view: the range itself (1:1, km) or the log scale."""
        return np.asarray(rng_km, float) if self.cb_true.isChecked() else self._radius(np.asarray(rng_km, float))

    # log scale, range zoom (log PPI, user 29 October 2026): the camera keeps its distance _log_d; the zoom (wheel,
    # buttons) changes the scale itself. A radius r stands for the range 10^e(r), e(r) = e0 + (r / H - 1/4) S / (3/4):
    # MIN_RING_KM at a quarter of the half-height H of the view, 10^(e0 + S) km at H; S (decades) from the zoom, kept
    # between LOG_SPAN_MIN and the span to MAX_VIEW_KM. No bound can contradict another; the rings fill the view.
    def _log_h(self) -> float:
        cam = self.plotter.camera
        d = self._log_d if self._log_d is not None else cam.GetDistance()      # (while the home moves the camera)
        return d * math.tan(math.radians(0.5 * cam.GetViewAngle()))

    def _span(self) -> float:
        """The span of the range zoom (decades), set from the run at its first use."""
        if getattr(self, "_log_span", None) is None:
            self._log_span = self._log_span_init()
        return self._log_span

    def _log_e(self, r: float) -> float:
        return LOG_E0 + (r / self._log_h() - 0.25) * self._span() / 0.75

    def _log_r(self, e):
        return self._log_h() * (0.25 + 0.75 * (np.asarray(e, float) - LOG_E0) / self._span())

    def _ship_range(self, r: float) -> Optional[float]:
        """Range (km) drawn at radius r in the ship view (the inverse of _ship_r, the log law extended)."""
        if self.cb_true.isChecked():
            return r if r > 0 else None
        e = self._log_e(r)
        # far zoomed out (or in) the extended log law leaves any meaning (and the floats, 1e308): no ring there
        return 10.0 ** e if RING_LOG10_KM[0] <= e <= RING_LOG10_KM[1] else None

    def _ring_radius(self, rng_km: float) -> float:
        """Radius of a range ring or a body's surface: _ship_r without the floor of the objects."""
        if self.cb_true.isChecked():
            return float(rng_km)
        return float(self._log_r(math.log10(max(rng_km, 1e-300))))

    def _range_projection(self, pos: np.ndarray, rgb: np.ndarray) -> None:
        """Option "range projection" (user, 29 October 2026): each object copied into the plane of the range rings at
        its drawn range from the ship (the nearest point of the ring of that radius: its direction projected on the
        plane), linked to the object by the arc of the sphere of that radius around the ship (every point of it at
        the same range); log scale and 1:1 alike."""
        _h, e1, e2, _c = self._ring_frame()
        nrm = np.cross(e1, e2)
        rad = np.linalg.norm(pos, axis=1)
        u = pos / np.maximum(rad, 1e-300)[:, None]
        p = u - (u @ nrm)[:, None] * nrm[None, :]
        pn = np.linalg.norm(p, axis=1)
        p = np.where((pn > 1e-9)[:, None], p / np.maximum(pn, 1e-300)[:, None], e1[None, :])   # on the axis: e1
        proj = p * rad[:, None]
        self._poly(f"{DYN}proj_pts", proj, rgb=rgb, point_size=7, render_points_as_spheres=True, opacity=0.8)
        th = np.arccos(np.clip(np.sum(u * p, axis=1), -1.0, 1.0))
        s = np.linspace(0.0, 1.0, PROJ_ARC_POINTS)
        n = len(pos)
        sin_th = np.maximum(np.sin(th), 1e-12)
        a = np.sin(np.outer(1.0 - s, th)).T / sin_th[:, None]               # slerp from u (s = 0) to p (s = 1)
        b = np.sin(np.outer(s, th)).T / sin_th[:, None]
        small = th < 1e-6
        a[small], b[small] = (1.0 - s)[None, :], s[None, :]
        arc = (a[:, :, None] * u[:, None, :] + b[:, :, None] * p[:, None, :]) * rad[:, None, None]
        lines = np.hstack([np.r_[PROJ_ARC_POINTS, j * PROJ_ARC_POINTS + np.arange(PROJ_ARC_POINTS)] for j in range(n)])
        self._poly(f"{DYN}proj_arcs", arc.reshape(-1, 3), lines=lines, rgb=np.repeat(rgb, PROJ_ARC_POINTS, axis=0),
                   line_width=1.5, opacity=0.8)

    def _draw_trajectories(self) -> None:
        """Global view: the whole trajectories, one mesh per classification (OWNSHIP apart), in its colour."""
        p, tr = self.plotter, self.truth
        for name in [n for n in p.actors if n.startswith("traj_")]:
            p.remove_actor(name, render=False)
        by_id: Dict[str, List[pv.PolyData]] = {}
        for i in range(len(tr.ids)):
            line = _polyline(tr.r_km[:, i])
            if line is not None:
                by_id.setdefault(self._row_ident[i], []).append(line)
        for c, lines in by_id.items():
            p.add_mesh(pv.merge(lines), color=SHIP_COLOR if c == "ship" else IDENTITY_COLORS.get(c, "#888888"),
                       line_width=3 if c == "ship" else 1.5, name=f"traj_{c}", render=False)

    def _ring_frame(self):
        """(visible half-height at the ship, the rings' plane axes e1, e2, the radius they must reach: the farthest
        corner of the 3D view in that plane). The render window's size (the Qt widget's may differ)."""
        cam = self.plotter.camera
        pos = np.asarray(cam.GetPosition())
        d0 = float(np.linalg.norm(pos))
        half = d0 * math.tan(math.radians(0.5 * cam.GetViewAngle()))
        w, h = self.plotter.ren_win.GetSize()
        aspect = w / max(h, 1)
        # the rings lie in the ship's XY plane (the PPI rings option, in the screen's plane, removed: user,
        # 29 October 2026, hard to read in practice)
        e1, e2 = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
        sin_el = abs(float(pos[2])) / max(d0, 1e-12)
        cover = RING_COVER * max(half * math.hypot(1.0, aspect), half / max(sin_el, RING_MIN_SIN_EL))
        return half, e1, e2, cover

    def _log_span_max(self) -> float:
        """Farthest range zoom: the farthest ring the view shows (its corners) at MAX_VIEW_KM."""
        half, _, _, cover = self._ring_frame()
        u = max(cover / max(half, 1e-300), 1.0)
        return max(LOG_SPAN_MIN, (math.log10(MAX_VIEW_KM) - LOG_E0) * 0.75 / (u - 0.25))

    def _update_rings(self) -> None:
        """Range rings of the ship view, spaced RING_STEP_FRAC of the visible half-height apart around the ship and
        out to the corners of the whole 3D view (partly drawn there, user 29 October 2026), so they stay at about the
        same place on screen whatever the zoom; each carries the round range (1-2-5) of its radius at this zoom."""
        if self.truth is None or self._global() or getattr(self, "_in_rings", False):
            return
        self._in_rings = True
        try:
            half, e1, e2, cover = self._ring_frame()
            step = RING_STEP_FRAC * half
            n = min(RING_MAX, int(math.ceil(cover / step)))
            ang = np.linspace(0.0, 2.0 * math.pi, RING_POINTS)
            pts, lines, lab_p, lab_t, seen = [], [], [], [], set()
            # always at the same place on screen (user, 29 October 2026): only the range they carry follows the zoom
            # (1:1: round steps in km; log: the log law of the view extended beyond its nearest / farthest objects)
            if self.cb_true.isChecked():
                s1 = _nice(step)
                values = [k * s1 for k in range(1, n + 1)]
            else:
                # equally spaced on screen (user): a constant step in decades, a round one (DECADE_STEPS), the ranges
                # 10^(k x step) labelled to 3 figures
                per = self._span() / (0.75 * self._log_h())                       # decades per unit of radius
                dd = min(DECADE_STEPS, key=lambda c: abs(math.log(c / max(step * per, 1e-12))))
                e_of = self._log_e
                k0, k1 = math.ceil(e_of(0.5 * step) / dd), math.floor(e_of(cover) / dd)
                values = [10.0 ** (k * dd) for k in range(k0, min(k1, k0 + RING_MAX - 1) + 1)
                          if RING_LOG10_KM[0] <= k * dd <= RING_LOG10_KM[1]]
            for nice in values:
                # (the zoom bounds hold MAX_VIEW_KM at the half-height: beyond, the corners keep their rings)
                if nice in seen or nice < MIN_RING_KM:
                    continue
                seen.add(nice)
                rad = self._ring_radius(nice)
                off = len(pts) * RING_POINTS
                pts.append(rad * (np.cos(ang)[:, None] * e1 + np.sin(ang)[:, None] * e2))
                lines.append(np.r_[RING_POINTS, off + np.arange(RING_POINTS)])
                lab_p.append(rad * (RING_LABEL_C * e1 + RING_LABEL_S * e2))   # between two azimuth spokes
                lab_t.append(f"{nice:g} km" if self.cb_true.isChecked() else f"{nice:.3g} km")
            if pts:
                self._poly(f"{DYN}rings", np.vstack(pts), lines=np.hstack(lines), color="#4a5266", line_width=1)
                self._labels(f"{DYN}rings_lbl", np.array(lab_p), lab_t, MUTED_COLOR, 9)
            # azimuth spokes in the rings' plane: from the ship's centre out to the corners of the view every 45 deg,
            # phi from +X toward +Y (the ship frame's convention), labelled inside the view
            names = (f"{DYN}spokes", f"{DYN}spokes_lbl")
            az = np.radians(np.arange(0.0, 360.0, SPOKE_STEP_DEG))
            d = np.cos(az)[:, None] * e1 + np.sin(az)[:, None] * e2
            self._poly(names[0], np.r_[np.zeros((len(az), 3)), cover * d], lines=_pairs(len(az)),
                       color="#3a4256", line_width=1)
            self._labels(names[1], SPOKE_LABEL_FRAC * half * d,
                         [f"{a:g}°" for a in np.arange(0.0, 360.0, SPOKE_STEP_DEG)], MUTED_COLOR, 9)
        finally:
            self._in_rings = False

    def _radius(self, rng_km: np.ndarray) -> np.ndarray:
        """V4, log scale: the radius of the objects' ranges by the range zoom's law; under MIN_RING_KM toward the
        centre, at least LOG_R_FLOOR of the half-height."""
        r = self._log_r(np.log10(np.maximum(np.asarray(rng_km, float), 1e-300)))
        return np.maximum(r, LOG_R_FLOOR * self._log_h())

    def _log_span_init(self) -> float:
        """The span at start: the farthest object of the run at LOG_FAR_U of the half-height."""
        far = max(self._r_log[1], 10.0 * MIN_RING_KM)
        span = 0.75 * (math.log10(far) - LOG_E0) / (LOG_FAR_U - 0.25)
        return float(np.clip(span, LOG_SPAN_MIN, self._log_span_max()))

    # ------------------------------------------------------------------ dynamic layer (updated in place)
    def _reset_dynamic(self) -> None:
        """After plotter.clear(): forget the dynamic actors, labels and volumes (they were removed with it)."""
        self._dyn: Dict[str, list] = {}               # name -> [topology key, mesh, actor]
        self._lab: Dict[str, "_Labels"] = {}
        self._vol: Optional[dict] = None
        self._model: Optional[dict] = None
        self._used: set = set()

    def _poly(self, name: str, pts: np.ndarray, lines: Optional[np.ndarray] = None, rgb: Optional[np.ndarray] = None,
              cell_rgb: Optional[np.ndarray] = None, **style) -> None:
        """Show a dynamic polydata: its actor is created once and its points (and colours) updated in place; it is
        rebuilt only when its topology (point count, lines) changes. Hidden when not drawn in a frame."""
        self._used.add(name)
        e = self._dyn.get(name)
        if len(pts) == 0:
            if e is not None:
                e[2].SetVisibility(False)
            return
        key = (len(pts), None if lines is None else lines.tobytes(), rgb is not None, cell_rgb is not None)
        if e is not None and e[0] == key:
            mesh = e[1]
            mesh.points = np.asarray(pts, float)
            if rgb is not None:
                mesh.point_data["rgb"] = rgb
            if cell_rgb is not None:
                mesh.cell_data["rgb"] = cell_rgb
            mesh.Modified()
            e[2].SetVisibility(True)
            return
        if e is not None:
            self.plotter.remove_actor(e[2], render=False)
        mesh = pv.PolyData(np.asarray(pts, float), lines=lines) if lines is not None else pv.PolyData(np.asarray(pts, float))
        if rgb is not None:
            mesh.point_data["rgb"] = rgb
        if cell_rgb is not None:
            mesh.cell_data["rgb"] = cell_rgb
        kw = dict(style)
        if rgb is not None or cell_rgb is not None:
            kw.update(scalars="rgb", rgb=True, preference="point" if rgb is not None else "cell")
        actor = self.plotter.add_mesh(mesh, name=name, render=False, **kw)
        self._dyn[name] = [key, mesh, actor]

    def _labels(self, name: str, pts: np.ndarray, texts: List[str], color: str, size: int = 10) -> None:
        self._used.add(name)
        lab = self._lab.get(name)
        if lab is None:
            lab = self._lab[name] = _Labels(self.plotter, color, size)
        lab.set(np.asarray(pts, float), texts)

    def _hide_unused(self) -> None:
        for name, e in self._dyn.items():
            if name not in self._used:
                e[2].SetVisibility(False)
        for name, lab in self._lab.items():
            if name not in self._used:
                lab.actor.SetVisibility(False)
        self._used = set()

    def _render(self) -> None:
        """One render per frame: the Qt repaint renders the window (plotter.render() would render it a second
        time)."""
        self.plotter.update()

    def _draw_frame(self) -> None:
        self._update_time_label()
        if self.truth is None:
            return
        self._used = set()
        if self._global():
            self._frame_global()
        else:
            self._frame_ship()
        self._hide_unused()
        self._show_info()
        self._render()

    def _update_time_label(self) -> None:
        if self.sc is None:
            self.time_label.setText("")
            return
        utc = scn.epoch(self.sc) + timedelta(seconds=self.t)
        self.time_label.setText(f"t {self.t:9.2f} s   {utc.strftime('%Y-%m-%d %H:%M:%S')} UTC")

    def _frame_global(self) -> None:
        p, tr = self.plotter, self.truth
        r, v = sgeo.states_at(tr, self.t)
        ex = np.isfinite(r[:, 0])
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False
        self._place_bodies()
        idx = np.flatnonzero(ex)
        rgb = self._row_rgb[idx]
        self._poly(f"{DYN}pts", r[idx], rgb=rgb, point_size=8, render_points_as_spheres=True)
        if self.cb_labels.isChecked():
            self._labels(f"{DYN}labels", r[idx], [self.own_name() if i == 0 else tr.ids[i] for i in idx], TEXT_COLOR)
        if self.cb_trails.isChecked() and len(idx):
            ts = np.linspace(max(0.0, self.t - TRAIL_S), self.t, TRAIL_SAMPLES)
            past = np.array([sgeo.states_at(tr, float(t))[0] for t in ts])[:, idx]       # (samples, n, 3)
            ok = np.isfinite(past[..., 0]).all(axis=0)
            past = past[:, ok]
            n = past.shape[1]
            if n:
                pts = past.transpose(1, 0, 2).reshape(-1, 3)
                lines = np.hstack([np.r_[TRAIL_SAMPLES, j * TRAIL_SAMPLES + np.arange(TRAIL_SAMPLES)]
                                   for j in range(n)])
                self._poly(f"{DYN}trails", pts, lines=lines, rgb=np.repeat(rgb[ok], TRAIL_SAMPLES, axis=0),
                           line_width=3, opacity=0.8)
        if self.cb_vectors.isChecked() and len(idx):
            a, b = r[idx], r[idx] + v[idx] * TRAIL_S
            self._poly(f"{DYN}vel", np.r_[a, b], lines=_pairs(len(idx)), color=MUTED_COLOR, line_width=1)
        # ship attitude triad, enlarged (V3), of constant size on screen ("ship axes" switch)
        if self.cb_axes.isChecked():
            m = tr.attitude("ship", self.t)
            L = TRIAD_FRAC * p.camera.GetDistance()
            tri = np.r_[np.repeat(r[0][None, :], 3, axis=0), r[0] + L * m.T]
            self._poly(f"{DYN}triad", tri, lines=_pairs(3), cell_rgb=self._triad_rgb, line_width=3)

    def _frame_ship(self) -> None:
        tr, g = self.truth, self.g
        r, _v = sgeo.states_at(tr, self.t)
        m = tr.attitude("ship", self.t)
        if self.cb_bodies.isChecked():
            self._ship_bodies(r[0], m)
        rows = np.arange(1, len(tr.ids))
        ro = r[rows]
        ex = np.isfinite(ro[:, 0])
        if not ex.any():
            self._screen_pts = {}
            self._update_rings()
            return
        los = ro[ex] - r[0]
        rng = np.linalg.norm(los, axis=1)
        d = (los / rng[:, None]) @ m
        pos = d * self._ship_r(rng)[:, None]
        face = opt.face_of(d[:, 0], d[:, 1], d[:, 2])[0]
        k = self._k()
        js = np.flatnonzero(ex)
        occ = g.occulted_by[k, js] >= 0
        msk = g.masked[k, js]
        # line of sight coloured by owner face, grey when occulted or outside every face, orange when masked (all
        # the objects at once: 1000+ objects, user 29 October 2026)
        face_rgb = np.array([_rgb(c) for c in cgeo.FACE_COLORS[:4]] + [_rgb(OCC_COLOR)], np.uint8)
        fi = np.where(np.asarray(face) < 0, 4, np.asarray(face)).astype(int)
        los_rgb = face_rgb[fi]
        los_rgb[np.asarray(msk, bool)] = _rgb(MASK_COLOR)
        los_rgb[np.asarray(occ, bool) | (np.asarray(face) < 0)] = _rgb(OCC_COLOR)
        n = len(js)
        self._poly(f"{DYN}los", np.r_[np.zeros((n, 3)), pos], lines=_pairs(n), cell_rgb=los_rgb, line_width=1.5,
                   opacity=0.7)
        rgb = self._row_rgb[js + 1]
        self._poly(f"{DYN}pts", pos, rgb=rgb, point_size=9, render_points_as_spheres=True)
        if self.cb_proj.isChecked():
            self._range_projection(pos, rgb)
        if self.cb_labels.isChecked():
            self._labels(f"{DYN}labels", pos, [g.ids[j] for j in js], TEXT_COLOR)
        if self.cb_vectors.isChecked():
            nose = g.nose_ship[k, js]
            okn = np.isfinite(nose[:, 0])
            if okn.any():
                ln = TRIAD_FRAC * self.plotter.camera.GetDistance() if self.cb_true.isChecked() else 0.15
                self._poly(f"{DYN}nose", np.r_[pos[okn], pos[okn] + ln * nose[okn]], lines=_pairs(int(okn.sum())),
                           color=TEXT_COLOR, line_width=2)
        if self.cb_trails.isChecked():                # every object's trail in one pass (one polyline each)
            sel = (g.t_s >= self.t - TRAIL_S) & (g.t_s <= self.t)
            rr = g.range_km[sel][:, js]                                    # (n_t, n)
            ok = np.isfinite(rr)
            cnt = ok.sum(axis=0)
            keep = np.flatnonzero(cnt >= 2)
            if len(keep):
                rad = self._ship_r(np.where(ok, rr, 1.0))
                pts_all = g.dir_ship[sel][:, js] * rad[..., None]
                take = ok[:, keep].T                                       # (n_keep, n_t): object-major, time order
                pts = pts_all[:, keep].transpose(1, 0, 2)[take]
                c = cnt[keep]
                heads = np.cumsum(np.r_[0, c[:-1] + 1])                    # where each polyline's count goes
                lines = np.empty(int(c.sum()) + len(c), int)
                body = np.ones(len(lines), bool)
                body[heads] = False
                lines[heads] = c
                lines[body] = np.arange(int(c.sum()))
                self._poly(f"{DYN}trails", pts, lines=lines, rgb=np.repeat(rgb[keep], c, axis=0), line_width=2,
                           opacity=0.6)
        self._screen_pts = {g.ids[j]: pos[jj] for jj, j in enumerate(js)}
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False
        if self.cb_true.isChecked() and self.cb_axes.isChecked():   # 1:1: the ship's axes, constant on screen
            L = TRIAD_FRAC * self.plotter.camera.GetDistance()
            self._poly(f"{DYN}ship_axes", np.r_[np.zeros((3, 3)), L * np.eye(3)], lines=_pairs(3),
                       cell_rgb=self._triad_rgb, line_width=3)
            self._labels(f"{DYN}ship_axes_lbl", 1.05 * L * np.eye(3), ["X", "Y", "Z (nose)"], TEXT_COLOR)
        self._update_rings()

    def _ship_bodies(self, r_ship: np.ndarray, m_ship: np.ndarray) -> None:
        """The Earth, the Moon and the Sun in the ship view (box "celestial objects", user 29 October 2026), at their
        true directions from the ship, textured and turned as in the global view (body-fixed -> ship frame): at 1:1
        at their true distance and size; in log scale at the view's distance of their range (the extended log law)
        with their true apparent size seen from the ship (radius = drawn distance x R / range)."""
        bodies = self._bodies_at()
        mats = {"earth": _earth_matrix(scn.epoch(self.sc), self.t),
                "moon": _moon_matrix(bodies["moon"] - bodies["earth"]), "sun": _sun_matrix(scn.epoch(self.sc), self.t)}
        pts, names = [], []
        for b in ("earth", "moon", "sun"):
            rel = bodies[b] - r_ship                                       # km, ICRF axes
            rng = float(np.linalg.norm(rel))
            rad_true = self._body_radius(b)
            if rng <= rad_true:
                continue
            if self.cb_true.isChecked():
                dist, size = rng, rad_true
            else:
                # log scale: the SURFACE at the view's distance of the altitude (the body stays out of the ship model,
                # the rings read its altitude), the true apparent size kept: centre D, radius D R / range
                k = rad_true / rng
                surf = self._ring_radius(max(rng - rad_true, 1e-6))
                dist = surf / (1.0 - k)
                size = dist * k
            centre = (rel / rng) @ m_ship * dist
            name = f"{DYN}sbody_{b}"
            self._used.add(name)
            e = self._dyn.get(name)
            if e is None:
                tex = _texture(b)
                act = self.plotter.add_mesh(_body_mesh(1.0), texture=tex, smooth_shading=True, culling="back",
                                            color=None if tex is not None else (SUN_COLOR if b == "sun" else EARTH_COLOR),
                                            lighting=(b != "sun"), ambient=0.12, name=name, render=False)
                e = self._dyn[name] = [None, None, act]
            u = np.eye(4)
            u[:3, :3] = size * (m_ship.T @ mats[b])
            u[:3, 3] = centre
            e[2].user_matrix = u
            e[2].SetVisibility(True)
            if b == "earth":                                               # the atmosphere's halo at the limb
                self._used.add(name + "_atm")
                h = self._dyn.get(name + "_atm")
                if h is None:
                    h = self._dyn[name + "_atm"] = [None, None, _add_atmosphere(self.plotter, name + "_atm")]
                ua = np.eye(4)
                ua[:3, :3], ua[:3, 3] = size * np.eye(3), centre
                h[2].user_matrix = ua
                h[2].SetVisibility(True)
            pts.append(centre)
            names.append(b.capitalize())
        if pts and self.cb_labels.isChecked():
            self._labels(f"{DYN}sbody_lbl", np.array(pts), names, MUTED_COLOR)

    # ------------------------------------------------------------------ V5 mission volumes
    def _clear_volumes(self) -> None:
        if self._vol:
            for a in self._vol["actors"]:
                self.plotter.remove_actor(a, render=False)
            if self._vol.get("labels") is not None:
                self.plotter.remove_actor(self._vol["labels"].actor, render=False)
        self._vol = None

    def _draw_volumes(self, force: bool = False) -> None:
        """V5: the mission's volumes at t. Built once from the navigation at t (the sectors resolved in the ship
        frame); then each one only TURNED with the change of its frame (ship -> sector matrix at t against the one
        it was built with), rebuilt when a frame has turned by more than VOL_REBUILD_DEG (the clipping by the faces
        and the scan window is the ship's) or when asked (force)."""
        if self._global() or self.mission is None or not self.cb_volumes.isChecked():
            if self._vol:
                self._clear_volumes()
                self._render()
            return
        try:
            mis = self.mission
            # the scenario's navigation at t once propagated; before, the mission's own (D, phase 6 merge)
            msg = [self._navigation.message(self.t)] if self.truth is not None else mis.get("global", {}).get("nav")
            nav = frames.parse_nav(msg)
            if self._vol is not None and not force:
                turns, rebuild = [], False
                for raw, m_ref, off_ref in self._vol["frames"]:
                    m_t, off_t = frames.sector_matrix(raw, nav)
                    rot = np.eye(3) if m_t is None else m_t.T @ m_ref
                    rebuild |= (math.degrees(float(np.linalg.norm(att.rotvec(rot)))) > VOL_REBUILD_DEG
                                or abs(off_t - off_ref) > VOL_REBUILD_DEG)
                    turns.append(rot)
                if not rebuild:
                    self._vol["rot"] = turns
                    self._place_volumes()
                    return
            m2 = copy.deepcopy(mis)
            m2.pop("t_nav_s", None)
            if msg is None:
                m2.setdefault("global", {}).pop("nav", None)
            else:
                m2.setdefault("global", {})["nav"] = msg
            m2["global"].pop("sky", None)
            res, _ = frames.resolve(m2)
        except (OSError, ValueError, KeyError) as exc:
            self.status.setText(f"mission volumes: {exc}")
            return
        self._clear_volumes()
        gl = res.get("global", {})
        e0, e1 = float(gl.get("eps_min_deg", -90.0)), float(gl.get("eps_max_deg", 90.0))
        vol = {"actors": [], "frames": [], "per_sector": [], "centres": [], "texts": []}
        for i, (raw, sec) in enumerate(zip(mis.get("sectors", []), res.get("sectors", []))):
            if (raw.get("id") or f"_idx{i}") in self._hidden_ids:
                continue
            try:
                build = cgeo.build_sector_volume_km if self.cb_true.isChecked() else cgeo.build_sector_overlay
                poly, outline = build(self.grid, sec, e0, e1)
            except (KeyError, TypeError, ValueError):
                continue
            col = cgeo.sector_color(i)
            acts = []
            if poly.n_points:
                acts.append(self.plotter.add_mesh(poly, color=col, opacity=0.35 if not self.cb_true.isChecked() else 0.2,
                                                  culling=None if self.cb_true.isChecked() else "back",
                                                  name=f"{VOL}{i}_s", render=False))
            if outline.n_points:
                acts.append(self.plotter.add_mesh(outline, color=col, line_width=2, name=f"{VOL}{i}_o", render=False))
            m_ref, off_ref = frames.sector_matrix(raw, nav)
            vol["frames"].append((raw, np.eye(3) if m_ref is None else m_ref, off_ref))
            vol["per_sector"].append(acts)
            vol["actors"] += acts
            vol["centres"].append(np.asarray(poly.center if poly.n_points else outline.center, float))
            vol["texts"].append(f"{raw.get('id', '?')}  R={raw.get('range_km', 0):.0f} km  "
                                f"RR={raw.get('rr_target_s', 0):.0f} s")
        vol["rot"] = [np.eye(3)] * len(vol["per_sector"])
        vol["labels"] = _Labels(self.plotter, TEXT_COLOR, 11) if vol["per_sector"] else None
        self._vol = vol
        self._place_volumes()
        self._render()

    # ------------------------------------------------------------------ L5 information
    def _on_click(self, obj, _event) -> None:
        """L5 (user, 29 October 2026, no lists): a click near an object (OWNSHIP included) shows its information; a
        double-click centres the view on it, in the global view also on the Earth, the Moon or the Sun (a click on
        its disc), and on nothing back to the default centre."""
        if self.truth is None:
            return
        if self._global():
            r, _ = sgeo.states_at(self.truth, self.t)
            pts = {(OWNSHIP if i == 0 else oid): r[i] for i, oid in enumerate(self.truth.ids) if np.isfinite(r[i, 0])}
        else:
            pts = dict(getattr(self, "_screen_pts", {}), **{OWNSHIP: np.zeros(3)})
        x, y = obj.GetEventPosition()
        ren = self.plotter.renderer
        best, best_d = None, CLICK_PX
        for oid, w in pts.items():
            ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
            ren.WorldToDisplay()
            dx, dy, _ = ren.GetDisplayPoint()
            dd = math.hypot(dx - x, dy - y)
            if dd < best_d:
                best, best_d = oid, dd
        double = obj.GetRepeatCount() >= 1
        if best is not None:
            self._select(best)
            if double:
                self.set_centre(best)
        elif double:
            self.set_centre(self._body_under(x, y) or CENTRAL)

    def _body_under(self, x: float, y: float) -> Optional[str]:
        """The Earth, the Moon or the Sun whose disc on screen holds the display point (x, y) (global view), the
        nearest to the camera first; None: none."""
        if not self._global():
            return None
        ren = self.plotter.renderer
        cam = np.asarray(self.plotter.camera.GetPosition())
        up = np.asarray(self.plotter.camera.GetViewUp(), float)
        at = self._bodies_at()
        hits = []
        for name, key in BODY_NAMES.items():
            c = at[key]
            pc, pe = [], []
            for w, out in ((c, pc), (c + self._body_radius(key) * up, pe)):
                ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
                ren.WorldToDisplay()
                out.extend(ren.GetDisplayPoint()[:2])
            if math.hypot(pc[0] - x, pc[1] - y) <= max(math.hypot(pe[0] - pc[0], pe[1] - pc[1]), CLICK_PX):
                hits.append((float(np.linalg.norm(c - cam)), name))
        return min(hits)[1] if hits else None

    def _select(self, oid: str) -> None:
        self.selected = oid or None
        self._show_info()

    def _set_info(self, text: str) -> None:
        """The information on the selected body, in the Info box of the side column (empty: no body chosen)."""
        self.info.setPlainText(text)

    def _show_info(self) -> None:
        if self.truth is None or not self.selected:
            self._set_info("")
            return
        if self.selected == OWNSHIP:
            self._show_ship_info()
            return
        if self.selected not in self.g.ids:
            self._set_info("")
            return
        tr, g = self.truth, self.g
        i, j, k = tr.row(self.selected), g.ids.index(self.selected), self._k()

        r, v = sgeo.states_at(tr, self.t)
        if not np.isfinite(r[i, 0]):
            self._set_info("\n".join([_kv("name", self.selected), _kv("characterisation", g.classes[j]),
                                      _kv("t", f"{self.t:.2f} s"), _kv("state", "does not exist at t")]))
            return
        los = r[i] - r[0]
        rng = float(np.linalg.norm(los))
        rr = 1e3 * float(los @ (v[i] - v[0])) / rng
        d = (los / rng) @ tr.attitude("ship", self.t)
        f, w, _, _ = opt.face_of(d[0], d[1], d[2])
        f = int(f)
        ident = scn.identity_of(self.sc, self.selected, tr.group_of[i] or None)
        occ = g.occulted_by[k, j]
        lines = [_kv("name", self.selected), _kv("characterisation", g.classes[j]), _kv("classification", ident),
                 _kv("t", f"{self.t:.2f} s")]
        lines += _vec_lines("position", r[i], "km", 3) + _vec_lines("velocity", v[i], "km/s", 5)
        lines += [_kv("frame", f"ICRF, from the {tr.body.name}"),
                  _kv("range", f"{rng:.3f} km"), _kv("range rate", f"{rr:+.1f} m/s"),
                  _kv("face", f"F{f + 1}" if f >= 0 else "none")]
        if f >= 0:
            lines.append(_kv("theta", f"{math.degrees(math.acos(min(1.0, float(w)))):.2f} deg"))
        lines += [_kv("recorded at", f"{g.t_s[k]:.1f} s"), _kv("aspect", f"{g.aspect_deg[k, j]:.1f} deg"),
                  _kv("RCS S", f"{g.rcs_s_dbsm[k, j]:.1f} dBsm"), _kv("RCS Ku", f"{g.rcs_ku_dbsm[k, j]:.1f} dBsm"),
                  _kv("occulted", sgeo.BODIES[occ] if occ >= 0 else "no")]
        if np.isfinite(g.block_loss_db[k, j]):
            lines += [_kv("masked", "yes" if g.masked[k, j] else "no"),
                      _kv("block loss", f"{max(0.0, float(g.block_loss_db[k, j])):.2f} dB per pass")]
        self._set_info("\n".join(lines))

    def _show_ship_info(self) -> None:
        tr, g = self.truth, self.g
        r, v = sgeo.states_at(tr, self.t)
        m = tr.attitude("ship", self.t)
        w = engine.ship_rate_dps(tr, self.t)
        k = self._k()
        alt = float(np.linalg.norm(r[0])) - tr.body.radius_km
        lines = [_kv("name", self.own_name()), _kv("classification", OWNSHIP_IDENTITY), _kv("t", f"{self.t:.2f} s")]
        lines += _vec_lines("position", r[0], "km", 3) + _vec_lines("velocity", v[0], "km/s", 5)
        lines += [_kv("frame", f"ICRF, from the {tr.body.name}"), _kv("altitude", f"{alt:.1f} km"),
                  _kv("speed", f"{1e3 * float(np.linalg.norm(v[0])):.1f} m/s")]
        lines += _vec_lines("nose (Z)", m[:, 2], "(ICRF)", 4)
        lines += _vec_lines("rate", w, "deg/s (ship)", 3)
        lines += [_kv("rate", f"{float(np.linalg.norm(w)):.3f} deg/s"), _kv("recorded at", f"{g.t_s[k]:.1f} s"),
                  _kv("Sun hidden", f"{100.0 * float(g.ship_shadow[k]):.0f} %")]
        self._set_info("\n".join(lines))
