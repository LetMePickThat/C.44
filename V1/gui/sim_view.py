#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SIM_VIEW -- the 3D views and the playback of the global simulator, on the right of the Scenario tab (roadmap phase 6,
passes S6 and S7; framing V1-V5 and L1-L6 validated by the user on 29 October 2026, block F of 28 October).

  V1 one plotter, a selector Global view / Ship view;
  V2 "Propagate" in a background thread (progress bar, cancelled by a newer request); "auto" re-propagates 1 s after
     every valid change while rows x steps stays below tool.display.sim_auto_max_cells;
  V3 global view, true scale (km, ICRF axes from the central body): the Earth and the Moon in every scenario, at
     their true places and turning (Earth: textured TEXTURE_DIR/earth.jpg, turned by its sidereal angle; Moon:
     textured, near side toward the Earth) and the Moon's orbit over a sidereal month; the trajectories over the
     whole run coloured by class (ship in black), the Sun (textured, at its true place and size, 1 AU away,
     lighting the scene with parallel rays: day and night sides), markers for appearances, manoeuvres,
     proportional-navigation starts and impacts; the ship drawn as a triad showing its attitude, of constant size
     on screen; "centre on" (or a double-click on a body, OWNSHIP included, in either view) centres the view on
     a body and follows it; Zoom + / - (and the wheel) at any time, playback included
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
  L1 play / pause, step +/- (journal step), slider, speed x1 .. x100 000, reverse; time from the epoch and UTC;
  L2 tool.display.sim_fps frames per second: positions interpolated at each frame (Hermite), the face / aspect / RCS
     read from the geometry at the nearest recorded instant;
  L3 trails of tool.display.sim_trail_s, labels, velocity vectors (global) and noses (ship view) on demand;
  L4 the events (appearances, manoeuvres, PN starts, impacts, attitude events, shadow) in a list: a click goes there;
  L5 information on the object clicked (or chosen in the list): state, range, range rate, face, theta, aspect,
     RCS S and Ku, occultation, masking; for OWNSHIP: state, altitude, nose, angular rate, shadow; dark background;
  L6 "Run" writes the journal (sim/run.py, background thread), "Open run..." replays a journal without recomputing.
No RF here (F9): step 6.4 adds the looks and the tracks to the same player.
"""
from __future__ import annotations

import copy
import math
from functools import lru_cache
import time
from datetime import timedelta
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
                             QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QSlider, QSplitter, QVBoxLayout,
                             QWidget)
from pyvistaqt import QtInteractor

from core import frames
from core import geometry as cgeo
from core import optimizer as opt
from core import radar_model as rm
from core.params import P
from gui import scene_view as sv
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
ARCBALL_RADIUS = float(P.get("tool.display.arcball_radius"))   # mouse rotation (user)
SPEEDS = (1, 10, 100, 1000, 10000, 100000)     # x10 000 and x100 000: the Moon on its orbit (user)
VOL_REBUILD_DEG = 5.0                             # V5: a volume is turned in place, rebuilt past this turn of its frame
RING_STEP_FRAC = 0.25                             # V4: range rings spaced this share of the visible half-height apart
RING_COVER = 1.1                                  # ... out to this x the farthest corner of the 3D view
RING_MIN_SIN_EL = 0.2                             # (a view almost in the rings' plane: at most 5 x the half-height)
RING_MAX = 60                                     # rings at most
DECADE_STEPS = (0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)   # log scale: ring step in decades
RING_POINTS = 121
RING_LOG10_KM = (-300.0, 300.0)                   # log scale: rings over the whole view (user), within the floats
TRAIL_SAMPLES = 25                                # L3: points of a trail in the global view
CLASS_COLORS = {c: cgeo.SECTOR_PALETTE[i % len(cgeo.SECTOR_PALETTE)] for i, c in enumerate(sorted(rm.TARGETS))}
SHIP_COLOR = "#ffffff"
BG_COLOR = "#0b1020"                              # dark background of the 3D views
TEXT_COLOR = "#e6e6e6"
MUTED_COLOR = "#9aa0aa"
EARTH_COLOR = "#4a78b5"
SUN_COLOR = "#e0a800"
OCC_COLOR = "#a0a0a0"
MASK_COLOR = "#e07b00"
ZOOM_STEP = 1.5                                   # one press of Zoom + / Zoom -
MIN_IDLE_MS = 15                                  # event-loop time kept between two frames (mouse live while playing)
SIDEREAL_MONTH_S = 27.321661 * 86400.0            # the Moon's orbit drawn over one sidereal month
FOLLOW_MIN_KM = 50.0                              # "follow the ship": framed span at least this
TRIAD_FRAC = 0.03                                 # ship triad length, share of the camera distance
TEXTURE_DIR = cgeo.MOON_TEXTURE_PATH.parent       # assets/textures: moon.jpg, earth.jpg (equirectangular, 2:1)
NEAR_TOL = 1e-9                                   # near / far clipping ratio: the Sun at 1 AU, objects at a few km
                                                  # (bodies drawn front faces only: no self-occlusion far away)
BODY_NAMES = {"Earth": "earth", "Moon": "moon", "Sun": "sun"}   # the bodies the view can centre on
OWNSHIP = "OWNSHIP"                               # the ship's name in the views (row "ship" of the truth)
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
    """Equirectangular texture of a body (TEXTURE_DIR/<body>.jpg; the Moon falls back on the procedural one)."""
    path = TEXTURE_DIR / f"{body}.jpg"
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


def _turn_x_to(u: np.ndarray) -> np.ndarray:
    """A rotation that takes +X to the unit vector u."""
    x = np.array([1.0, 0.0, 0.0])
    c = float(x @ u)
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        return np.diag([-1.0, -1.0, 1.0])
    k = np.cross(x, u)
    k = k / np.linalg.norm(k)
    return att.expm(k * math.acos(c))


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


def _segments(a: np.ndarray, b: np.ndarray) -> pv.PolyData:
    pts = np.empty((2 * len(a), 3))
    pts[0::2], pts[1::2] = a, b
    lines = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(len(a))])
    return pv.PolyData(pts, lines=lines)


class SimView(QWidget):
    """3D views (S6) and playback (S7) of the global simulator."""

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
        self._mission = None                      # (path, mission dict) of the linked mission
        self._reset_dynamic()
        self._r_log = (1.0, 10.0)
        self.selected: Optional[str] = None

        # ---- V1, V2, L6 bar
        top = QHBoxLayout()
        self.cb_view = QComboBox()
        self.cb_view.addItems(["Global view", "Ship view"])
        self.cb_view.currentIndexChanged.connect(lambda _i: self._rebuild())
        top.addWidget(self.cb_view)
        self.bt_prop = QPushButton("Propagate")
        self.bt_prop.clicked.connect(self.propagate)
        top.addWidget(self.bt_prop)
        self.cb_auto = QCheckBox("auto")
        self.cb_auto.setChecked(True)
        top.addWidget(self.cb_auto)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setMaximumWidth(160)
        self.progress.setFormat("%p %")
        top.addWidget(self.progress)
        self.bt_run = QPushButton("Run")
        self.bt_run.setToolTip("Propagate the saved scenario and write its journal (.npz) and summary next to it")
        self.bt_run.clicked.connect(self.run_journal)
        top.addWidget(self.bt_run)
        self.bt_open = QPushButton("Open run...")
        self.bt_open.clicked.connect(self.open_journal)
        top.addWidget(self.bt_open)
        top.addStretch(1)
        self.status = QLabel("No propagation yet")
        top.addWidget(self.status)

        # ---- L3 / V3 / V5 toggles
        opts = QHBoxLayout()
        self.cb_labels = QCheckBox("labels")
        self.cb_labels.setChecked(True)
        self.cb_vectors = QCheckBox("vectors / noses")
        self.cb_trails = QCheckBox("trails")
        self.cb_trails.setChecked(True)
        self.cb_centre = QComboBox()
        self.cb_centre.addItem(CENTRAL)
        self.cb_centre.setToolTip("The view is centred on this body and follows it (double-click a body in the view "
                                  "to choose it; zoom with the wheel or the buttons, also while playing)")
        self.cb_volumes = QCheckBox("mission volumes")
        self.cb_volumes.setChecked(True)
        self.cb_bodies = QCheckBox("celestial objects")
        self.cb_bodies.setToolTip("Ship view: the Earth, the Moon and the Sun at their true directions (1:1: true "
                                  "distances and sizes; log scale: the view's distance, their true apparent size)")
        self.cb_bodies.toggled.connect(lambda _on: self._draw_frame())
        self.cb_true = QCheckBox("1:1 scale")
        self.cb_true.setToolTip("Ship view: true distances in km (off: log range scale from the nearest to the "
                                "farthest object of the run)")
        self.cb_true.toggled.connect(lambda _on: self._rebuild() if not self._global() else None)
        for cb in (self.cb_labels, self.cb_vectors, self.cb_trails):
            cb.toggled.connect(lambda _on: self._draw_frame())
        self.cb_centre.currentTextChanged.connect(lambda _t: self._apply_centre(reframe=True))
        self.cb_volumes.toggled.connect(lambda _on: self._draw_volumes(force=True))
        for cb in (self.cb_labels, self.cb_vectors, self.cb_trails, self.cb_volumes, self.cb_true, self.cb_bodies):
            opts.addWidget(cb)
        opts.addWidget(QLabel("centre on"))
        opts.addWidget(self.cb_centre)
        for text, f in (("Zoom +", ZOOM_STEP), ("Zoom -", 1.0 / ZOOM_STEP)):
            b = QPushButton(text)
            b.clicked.connect(lambda _c, ff=f: self.zoom(ff))
            opts.addWidget(b)
        opts.addStretch(1)

        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(400)
        sv.lock_pivot(self.plotter)
        sv.install_arcball(self.plotter, ARCBALL_RADIUS)   # left drag: virtual trackball (roll outside its circle)
        self.plotter.iren.interactor.AddObserver("LeftButtonPressEvent", self._on_click, 1.0)
        self.plotter.set_background(BG_COLOR)
        self.plotter.renderer.SetNearClippingPlaneTolerance(NEAR_TOL)
        self.plotter.add_axes(color=TEXT_COLOR)
        self.plotter.renderer.GetActiveCamera().AddObserver("ModifiedEvent", lambda *_a: self._on_camera())

        # ---- L1 playback
        play = QHBoxLayout()
        self.bt_back = QPushButton("|<")
        self.bt_back.clicked.connect(lambda: self.step(-1))
        self.bt_play = QPushButton("Play")
        self.bt_play.clicked.connect(self.toggle_play)
        self.bt_fwd = QPushButton(">|")
        self.bt_fwd.clicked.connect(lambda: self.step(1))
        self.cb_rev = QCheckBox("reverse")
        self.cb_speed = QComboBox()
        self.cb_speed.addItems([f"x{s}" for s in SPEEDS])
        self.cb_speed.setCurrentIndex(1)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 10000)
        self.slider.valueChanged.connect(self._on_slider)
        self.time_label = QLabel()
        for w in (self.bt_back, self.bt_play, self.bt_fwd, self.cb_rev, self.cb_speed):
            play.addWidget(w)
        play.addWidget(self.slider, 1)
        play.addWidget(self.time_label)
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)                # rescheduled by _tick: the event loop always gets its turn
        self.timer.timeout.connect(self._tick)
        self._wall = 0.0
        self._playing = False

        # ---- L4 events, L5 information
        self.events = QListWidget()
        self.events.itemClicked.connect(lambda it: self.set_time(float(it.data(Qt.UserRole))))
        info = QWidget()
        il = QVBoxLayout(info)
        il.setContentsMargins(0, 0, 0, 0)
        self.cb_object = QComboBox()
        self.cb_object.currentTextChanged.connect(self._select)
        il.addWidget(self.cb_object)
        self.info = QPlainTextEdit()
        self.info.setReadOnly(True)
        il.addWidget(self.info)
        low = QSplitter(Qt.Horizontal)
        low.addWidget(self.events)
        low.addWidget(info)

        split = QSplitter(Qt.Vertical)
        split.addWidget(self.plotter.interactor)
        split.addWidget(low)
        split.setSizes([600, 180])
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(top)
        lay.addLayout(opts)
        lay.addWidget(split, 1)
        lay.addLayout(play)

        self.debounce = QTimer(self)
        self.debounce.setSingleShot(True)
        self.debounce.setInterval(1000)
        self.debounce.timeout.connect(self.propagate)
        self._enable_playback(False)

    # ------------------------------------------------------------------ V2 computations
    def set_scenario(self, sc: dict) -> None:
        """The editor's scenario after a valid change (normalised)."""
        self.pending = sc
        if self.cb_auto.isChecked() and cells(sc) <= AUTO_MAX_CELLS:
            self.debounce.start()

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
        path, _ = QFileDialog.getOpenFileName(self, "Open a run", "", "Simulator journal (*.npz)")
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
        self._r_log = (max(float(rng.min()), 1e-3), max(float(rng.max()), 1e-3)) if rng.size else (1.0, 10.0)
        self._mission = None
        self._navigation = engine.Navigation(sc, truth)
        self._triad_rgb = np.array([_rgb(cgeo.COLORS[k]) for k in ("axis_x", "axis_y", "axis_z")], np.uint8)
        self._row_rgb = np.array([_rgb(SHIP_COLOR if i == 0 else CLASS_COLORS.get(c, "#888888"))
                                  for i, c in enumerate(truth.classes)], np.uint8)
        for cb, first in ((self.cb_object, [""]), (self.cb_centre, [CENTRAL])):
            cur = cb.currentText()
            cb.blockSignals(True)
            cb.clear()
            extra = list(BODY_NAMES) if cb is self.cb_centre else []
            cb.addItems(first + [OWNSHIP] + g.ids + extra)
            if cur in [OWNSHIP] + g.ids + extra:
                cb.setCurrentText(cur)
            cb.blockSignals(False)
        self._fill_events()
        self._enable_playback(True)
        self._rebuild()

    # ------------------------------------------------------------------ time
    def duration(self) -> float:
        return float(self.sc["meta"]["duration_s"]) if self.sc else 0.0

    def _enable_playback(self, on: bool) -> None:
        for w in (self.bt_back, self.bt_play, self.bt_fwd, self.slider, self.cb_rev, self.cb_speed):
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
            if not self.cb_rev.isChecked() and self.t >= self.duration():
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
        dt = (now - self._wall) * SPEEDS[self.cb_speed.currentIndex()] * (-1.0 if self.cb_rev.isChecked() else 1.0)
        self._wall = now
        t = self.t + dt
        if t <= 0.0 or t >= self.duration():
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
        self.events.clear()
        for t, text in sorted(ev):
            it = QListWidgetItem(f"{t:10.1f} s  {text}")
            it.setData(Qt.UserRole, t)
            self.events.addItem(it)

    # ------------------------------------------------------------------ drawing
    def _global(self) -> bool:
        return self.cb_view.currentIndex() == 0

    def _rebuild(self) -> None:
        """The static layer of the current view, then the frame."""
        p = self.plotter
        p.clear()
        p.set_background(BG_COLOR)
        p.remove_all_lights()
        self._reset_dynamic()
        if self.truth is None:
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
        by_cls: Dict[str, List[pv.PolyData]] = {}
        for i in range(len(tr.ids)):
            line = _polyline(tr.r_km[:, i])
            if line is not None:
                by_cls.setdefault(tr.classes[i] or "ship", []).append(line)
        for c, lines in by_cls.items():
            p.add_mesh(pv.merge(lines), color=SHIP_COLOR if c == "ship" else CLASS_COLORS.get(c, "#888888"),
                       line_width=3 if c == "ship" else 1.5, name=f"traj_{c}", render=False)
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
        su = pos_b["sun"] / np.linalg.norm(pos_b["sun"])
        for b, pos, m in (("earth", pos_b["earth"], _earth_matrix(scn.epoch(self.sc), self.t)),
                          ("moon", pos_b["moon"], _moon_matrix(moon_geo)),
                          ("sun", pos_b["sun"], eph_sun_radius_km() * np.eye(3))):
            a = self.plotter.actors.get(f"body_{b}")
            if a is not None:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = m, pos
                a.user_matrix = u
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

    def _max_camera_distance(self) -> Optional[float]:
        """Camera distance at which the horizontal edge of the view stands for MAX_VIEW_KM (global view and ship
        view at 1:1: km; ship view in log scale: its extended log law)."""
        if self.truth is None:
            return None
        cam = self.plotter.camera
        w, h = self.plotter.interactor.width(), self.plotter.interactor.height()
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) * max(w / max(h, 1), 1.0)
        edge = MAX_VIEW_KM if (self._global() or self.cb_true.isChecked()) else self._ring_radius(MAX_VIEW_KM)
        return edge / k if k > 0 and edge > 0 else None

    def _on_camera(self) -> None:
        """Every camera change (wheel, buttons, follow): the zoom out held at MAX_VIEW_KM, then the range rings."""
        if getattr(self, "_in_camera", False):
            return
        self._in_camera = True
        try:
            d_max = self._max_camera_distance()
            cam = self.plotter.camera
            if d_max is not None and cam.GetDistance() > d_max:
                f = np.asarray(cam.GetFocalPoint())
                u = np.asarray(cam.GetPosition()) - f
                cam.SetPosition(*(f + u / np.linalg.norm(u) * d_max))
                self.plotter.renderer.ResetCameraClippingRange()
        finally:
            self._in_camera = False
        self._update_rings()

    def zoom(self, f: float) -> None:
        """Move the camera toward its focal point by f (as the wheel; the triad keeps its size on screen)."""
        self.plotter.camera.Dolly(f)
        self.plotter.renderer.ResetCameraClippingRange()
        if not self._playing:
            self._draw_frame()

    def _target(self) -> Optional[np.ndarray]:
        """Position (view coordinates) of the body the view is centred on at t; None: the default centre."""
        name = self.cb_centre.currentText()
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
            if self._global() and self.cb_centre.currentText() in BODY_NAMES:
                dist = 4.0 * self._body_radius(BODY_NAMES[self.cb_centre.currentText()])
            elif self._global():
                r, _ = sgeo.states_at(self.truth, self.t)
                d = np.linalg.norm(r - tgt, axis=1)
                d = d[np.isfinite(d) & (d > 1e-9)]
                span = float(np.clip(d.max() if d.size else 0.0, FOLLOW_MIN_KM, 4.0 * self.truth.body.radius_km))
                dist = 4.0 * span
            elif self.cb_true.isChecked():
                dist = max(1.0, 0.3 * float(np.linalg.norm(tgt))) if np.linalg.norm(tgt) > 0 else 0.3 * self._r_log[1]
            else:
                dist = 1.5
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
        """V4: the ship model and its faces (log scale: unit sizes) or nothing but its axes (1:1, km: the model and
        the face layer would be a 1 km symbol); the range rings are dynamic (_update_rings)."""
        p = self.plotter
        if not self.cb_true.isChecked():
            sv.add_static_scene(p)
            scene = cgeo.build_face_sectors(self.grid, -90.0, 90.0)
            for k in range(4):
                if scene.surfaces[k].n_points:
                    p.add_mesh(scene.surfaces[k], color=cgeo.FACE_COLORS[k], opacity=0.15, culling="back",
                               name=f"face_{k}", render=False)
        p.set_background(BG_COLOR)
        p.enable_lightkit()
        self._centre_pending = True
        self._ship_home()

    def _ship_home(self) -> None:
        """Default camera of the ship view: the iso view, out to the farthest object (SHIP_R1, or its range in km)."""
        p = self.plotter
        sv.set_default_camera(p)
        k = (self._r_log[1] / 1.2) if self.cb_true.isChecked() else max(1.0, SHIP_R1 / 1.5)
        pos = np.asarray(p.camera.position) * k
        if self.truth is not None:
            # the default camera on the side away from the central body (else it may stand inside it: celestial
            # objects drawn, both scales)
            r, _ = sgeo.states_at(self.truth, self.t)
            dc = -(r[0] / np.linalg.norm(r[0])) @ self.truth.attitude("ship", self.t)
            if pos @ dc > 0.0:
                pos = pos - 2.0 * (pos @ dc) * dc
        p.camera.position = tuple(pos)
        p.renderer.ResetCameraClippingRange()

    def _ship_r(self, rng_km: np.ndarray) -> np.ndarray:
        """Radius drawn for ranges in the ship view: the range itself (1:1, km) or the log scale."""
        return np.asarray(rng_km, float) if self.cb_true.isChecked() else self._radius(np.asarray(rng_km, float))

    def _ship_range(self, r: float) -> Optional[float]:
        """Range (km) drawn at radius r in the ship view (the inverse of _ship_r, the log law extended)."""
        if self.cb_true.isChecked():
            return r if r > 0 else None
        a, b = (math.log10(x) for x in self._r_log)
        if b - a < 1e-9:
            return None
        e = a + (r - SHIP_R0) / (SHIP_R1 - SHIP_R0) * (b - a)
        # far zoomed out (or in) the extended log law leaves any meaning (and the floats, 1e308): no ring there
        return 10.0 ** e if RING_LOG10_KM[0] <= e <= RING_LOG10_KM[1] else None

    def _ring_radius(self, rng_km: float) -> float:
        """Radius of a range ring: _ship_r without the bounds of the log scale (the inverse of _ship_range)."""
        if self.cb_true.isChecked():
            return float(rng_km)
        a, b = (math.log10(x) for x in self._r_log)
        if b - a < 1e-9:
            return 0.5 * (SHIP_R0 + SHIP_R1)
        return SHIP_R0 + (SHIP_R1 - SHIP_R0) * (math.log10(rng_km) - a) / (b - a)

    def _update_rings(self) -> None:
        """Range rings of the ship view, spaced RING_STEP_FRAC of the visible half-height apart around the ship and
        out to the corners of the whole 3D view (partly drawn there, user 29 October 2026), so they stay at about the
        same place on screen whatever the zoom; each carries the round range (1-2-5) of its radius at this zoom."""
        if self.truth is None or self._global() or getattr(self, "_in_rings", False):
            return
        self._in_rings = True
        try:
            cam = self.plotter.camera
            pos = np.asarray(cam.GetPosition())
            d0 = float(np.linalg.norm(pos))
            half = d0 * math.tan(math.radians(0.5 * cam.GetViewAngle()))
            w, h = self.plotter.interactor.width(), self.plotter.interactor.height()
            aspect = w / max(h, 1)
            sin_el = abs(float(pos[2])) / max(d0, 1e-12)       # the rings lie in the ship's XY plane
            cover = RING_COVER * max(half * math.hypot(1.0, aspect), half / max(sin_el, RING_MIN_SIN_EL))
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
                a, b = (math.log10(x) for x in self._r_log)
                per = (b - a) / (SHIP_R1 - SHIP_R0) if b - a > 1e-9 else 1.0      # decades per unit of radius
                dd = min(DECADE_STEPS, key=lambda c: abs(math.log(c / max(step * per, 1e-12))))
                e_of = lambda r: a + (r - SHIP_R0) * per                          # noqa: E731
                k0, k1 = math.ceil(e_of(0.5 * step) / dd), math.floor(e_of(cover) / dd)
                values = [10.0 ** (k * dd) for k in range(k0, min(k1, k0 + RING_MAX - 1) + 1)
                          if RING_LOG10_KM[0] <= k * dd <= RING_LOG10_KM[1]]
            for nice in values:
                if nice in seen or nice > MAX_VIEW_KM:
                    continue
                seen.add(nice)
                rad = self._ring_radius(nice)
                off = len(pts) * RING_POINTS
                pts.append(np.stack([rad * np.cos(ang), rad * np.sin(ang), np.zeros_like(ang)], axis=1))
                lines.append(np.r_[RING_POINTS, off + np.arange(RING_POINTS)])
                lab_p.append([rad, 0.0, 0.0])
                lab_t.append(f"{nice:g} km" if self.cb_true.isChecked() else f"{nice:.3g} km")
            if pts:
                self._poly(f"{DYN}rings", np.vstack(pts), lines=np.hstack(lines), color="#4a5266", line_width=1)
                self._labels(f"{DYN}rings_lbl", np.array(lab_p), lab_t, MUTED_COLOR, 9)
        finally:
            self._in_rings = False

    def _radius(self, rng_km: np.ndarray) -> np.ndarray:
        """V4: log range scale from SHIP_R0 (nearest object of the run) to SHIP_R1 (farthest)."""
        a, b = (math.log10(x) for x in self._r_log)
        if b - a < 1e-9:
            return np.full(len(rng_km), 0.5 * (SHIP_R0 + SHIP_R1))
        x = np.clip((np.log10(np.maximum(rng_km, 1e-6)) - a) / (b - a), 0.0, 1.0)
        return SHIP_R0 + (SHIP_R1 - SHIP_R0) * x

    # ------------------------------------------------------------------ dynamic layer (updated in place)
    def _reset_dynamic(self) -> None:
        """After plotter.clear(): forget the dynamic actors, labels and volumes (they were removed with it)."""
        self._dyn: Dict[str, list] = {}               # name -> [topology key, mesh, actor]
        self._lab: Dict[str, "_Labels"] = {}
        self._vol: Optional[dict] = None
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
            self._labels(f"{DYN}labels", r[idx], [OWNSHIP if i == 0 else tr.ids[i] for i in idx], TEXT_COLOR)
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
        # ship attitude triad, enlarged (V3), of constant size on screen
        m = tr.attitude("ship", self.t)
        L = TRIAD_FRAC * p.camera.GetDistance()
        tri = np.r_[np.repeat(r[0][None, :], 3, axis=0), r[0] + L * m.T]
        self._poly(f"{DYN}triad", tri, lines=_pairs(3), cell_rgb=self._triad_rgb, line_width=3)
        # Sun direction from the central body: an arrow built once, turned and scaled
        su = self._bodies_at()["sun"]
        su = su / np.linalg.norm(su)
        R = tr.body.radius_km
        self._used.add(f"{DYN}sun")
        a = self._dyn.get(f"{DYN}sun")
        if a is None:
            act = p.add_mesh(pv.Arrow(start=(0.0, 0.0, 0.0), direction=(1.0, 0.0, 0.0), scale=1.0), color=SUN_COLOR,
                             name=f"{DYN}sun", render=False)
            a = self._dyn[f"{DYN}sun"] = [None, None, act]
        u = np.eye(4)
        u[:3, :3] = 1.5 * R * _turn_x_to(su)
        u[:3, 3] = 1.1 * R * su
        a[2].user_matrix = u
        a[2].SetVisibility(True)
        self._labels(f"{DYN}sun_lbl", (su * 2.7 * R)[None, :], ["Sun"], SUN_COLOR)

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
        los_rgb = np.array([_rgb(OCC_COLOR if (o or f < 0) else MASK_COLOR if mk else cgeo.FACE_COLORS[int(f)])
                            for f, o, mk in zip(face, occ, msk)], np.uint8)
        n = len(js)
        self._poly(f"{DYN}los", np.r_[np.zeros((n, 3)), pos], lines=_pairs(n), cell_rgb=los_rgb, line_width=1.5,
                   opacity=0.7)
        rgb = self._row_rgb[js + 1]
        self._poly(f"{DYN}pts", pos, rgb=rgb, point_size=9, render_points_as_spheres=True)
        if self.cb_labels.isChecked():
            self._labels(f"{DYN}labels", pos, [g.ids[j] for j in js], TEXT_COLOR)
        if self.cb_vectors.isChecked():
            nose = g.nose_ship[k, js]
            okn = np.isfinite(nose[:, 0])
            if okn.any():
                ln = TRIAD_FRAC * self.plotter.camera.GetDistance() if self.cb_true.isChecked() else 0.15
                self._poly(f"{DYN}nose", np.r_[pos[okn], pos[okn] + ln * nose[okn]], lines=_pairs(int(okn.sum())),
                           color=TEXT_COLOR, line_width=2)
        if self.cb_trails.isChecked():
            sel = (g.t_s >= self.t - TRAIL_S) & (g.t_s <= self.t)
            pts, lines, cols, off = [], [], [], 0
            for jj, j in enumerate(js):
                dd, rr = g.dir_ship[sel, j], g.range_km[sel, j]
                ok = np.isfinite(rr)
                c = int(ok.sum())
                if c >= 2:
                    pts.append(dd[ok] * self._ship_r(rr[ok])[:, None])
                    lines.append(np.r_[c, off + np.arange(c)])
                    cols.append(np.repeat(rgb[jj][None, :], c, axis=0))
                    off += c
            if pts:
                self._poly(f"{DYN}trails", np.vstack(pts), lines=np.hstack(lines), rgb=np.vstack(cols), line_width=2,
                           opacity=0.6)
        self._screen_pts = {g.ids[j]: pos[jj] for jj, j in enumerate(js)}
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False
        if self.cb_true.isChecked():                  # 1:1: the ship's axes, of constant size on screen
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
                "moon": _moon_matrix(bodies["moon"] - bodies["earth"]), "sun": np.eye(3)}
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
            pts.append(centre)
            names.append(b.capitalize())
        if pts and self.cb_labels.isChecked():
            self._labels(f"{DYN}sbody_lbl", np.array(pts), names, MUTED_COLOR)

    # ------------------------------------------------------------------ V5 mission volumes
    def _clear_volumes(self) -> None:
        if self._vol:
            for a in self._vol["actors"]:
                self.plotter.remove_actor(a, render=False)
        self._vol = None

    def _draw_volumes(self, force: bool = False) -> None:
        """V5: the mission's volumes at t. Built once from the navigation at t (the sectors resolved in the ship
        frame); then each one only TURNED with the change of its frame (ship -> sector matrix at t against the one
        it was built with), rebuilt when a frame has turned by more than VOL_REBUILD_DEG (the clipping by the faces
        and the scan window is the ship's) or when asked (force)."""
        if self.truth is None or self._global() or not self.sc.get("mission") or not self.cb_volumes.isChecked():
            if self._vol:
                self._clear_volumes()
                self._render()
            return
        try:
            if self._mission is None or self._mission[0] != self.sc["mission"]:
                from core import mission as cm
                self._mission = (self.sc["mission"], cm.load_mission(self.sc["mission"]))
            nav = frames.parse_nav([self._navigation.message(self.t)])
            mis = self._mission[1]
            if self._vol is not None and not force:
                turns, rebuild = [], False
                for raw, m_ref, off_ref in self._vol["frames"]:
                    m_t, off_t = frames.sector_matrix(raw, nav)
                    rot = np.eye(3) if m_t is None else m_t.T @ m_ref
                    rebuild |= (math.degrees(float(np.linalg.norm(att.rotvec(rot)))) > VOL_REBUILD_DEG
                                or abs(off_t - off_ref) > VOL_REBUILD_DEG)
                    turns.append(rot)
                if not rebuild:
                    for rot, acts in zip(turns, self._vol["per_sector"]):
                        u = np.eye(4)
                        u[:3, :3] = rot
                        for a in acts:
                            a.user_matrix = u
                    return
            m2 = copy.deepcopy(mis)
            m2.setdefault("global", {})["nav"] = [self._navigation.message(self.t)]
            m2["global"].pop("sky", None)
            res, _ = frames.resolve(m2)
        except (OSError, ValueError, KeyError) as exc:
            self.status.setText(f"mission volumes: {exc}")
            return
        self._clear_volumes()
        gl = res.get("global", {})
        e0, e1 = float(gl.get("eps_min_deg", -90.0)), float(gl.get("eps_max_deg", 90.0))
        vol = {"actors": [], "frames": [], "per_sector": []}
        for i, (raw, sec) in enumerate(zip(mis.get("sectors", []), res.get("sectors", []))):
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
        self._vol = vol
        self._render()

    # ------------------------------------------------------------------ L5 information
    def _on_click(self, obj, _event) -> None:
        """L5: a click near a body shows its information; a double-click also centres the view on it (OWNSHIP
        included)."""
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
        if best is None:
            return
        self.cb_object.setCurrentText(best)
        if obj.GetRepeatCount() >= 1:                  # double-click
            self.cb_centre.setCurrentText(best)

    def _select(self, oid: str) -> None:
        self.selected = oid or None
        self._show_info()

    def _show_info(self) -> None:
        if self.truth is None or not self.selected:
            self.info.setPlainText("")
            return
        if self.selected == OWNSHIP:
            self._show_ship_info()
            return
        if self.selected not in self.g.ids:
            self.info.setPlainText("")
            return
        tr, g = self.truth, self.g
        i, j, k = tr.row(self.selected), g.ids.index(self.selected), self._k()

        r, v = sgeo.states_at(tr, self.t)
        if not np.isfinite(r[i, 0]):
            self.info.setPlainText(f"{self.selected} ({g.classes[j]}): does not exist at t = {self.t:.2f} s")
            return
        los = r[i] - r[0]
        rng = float(np.linalg.norm(los))
        rr = 1e3 * float(los @ (v[i] - v[0])) / rng
        d = (los / rng) @ tr.attitude("ship", self.t)
        f, w, _, _ = opt.face_of(d[0], d[1], d[2])
        f = int(f)
        lines = [f"{self.selected}  ({g.classes[j]})   t = {self.t:.2f} s",
                 f"position  {np.array2string(r[i], precision=3)} km (ICRF, from the {tr.body.name})",
                 f"velocity  {np.array2string(v[i], precision=5)} km/s",
                 f"range {rng:.3f} km   range rate {rr:+.1f} m/s",
                 (f"face F{f + 1}   theta {math.degrees(math.acos(min(1.0, float(w)))):.2f} deg" if f >= 0
                  else "no face"),
                 f"at the recorded {g.t_s[k]:.1f} s:  aspect {g.aspect_deg[k, j]:.1f} deg   "
                 f"RCS S {g.rcs_s_dbsm[k, j]:.1f} dBsm, Ku {g.rcs_ku_dbsm[k, j]:.1f} dBsm",
                 f"occulted: {sgeo.BODIES[g.occulted_by[k, j]] if g.occulted_by[k, j] >= 0 else 'no'}   "
                 f"masked by the structure: {'yes' if g.masked[k, j] else 'no'} "
                 f"({max(0.0, float(g.block_loss_db[k, j])):.2f} dB per pass)" if np.isfinite(g.block_loss_db[k, j]) else
                 f"occulted: {sgeo.BODIES[g.occulted_by[k, j]] if g.occulted_by[k, j] >= 0 else 'no'}"]
        self.info.setPlainText("\n".join(lines))

    def _show_ship_info(self) -> None:
        tr, g = self.truth, self.g
        r, v = sgeo.states_at(tr, self.t)
        m = tr.attitude("ship", self.t)
        w = engine.ship_rate_dps(tr, self.t)
        k = self._k()
        alt = float(np.linalg.norm(r[0])) - tr.body.radius_km
        lines = [f"{OWNSHIP}   t = {self.t:.2f} s",
                 f"position  {np.array2string(r[0], precision=3)} km (ICRF, from the {tr.body.name}), "
                 f"altitude {alt:.1f} km",
                 f"velocity  {np.array2string(v[0], precision=5)} km/s ({1e3 * float(np.linalg.norm(v[0])):.1f} m/s)",
                 f"nose (Z) toward {np.array2string(m[:, 2], precision=4)} (ICRF)",
                 f"angular rate {np.array2string(w, precision=3)} deg/s (ship frame), {float(np.linalg.norm(w)):.3f} deg/s",
                 f"at the recorded {g.t_s[k]:.1f} s: solar disk hidden {100.0 * float(g.ship_shadow[k]):.0f} %"]
        self.info.setPlainText("\n".join(lines))
