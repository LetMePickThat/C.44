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
     tool.display.sim_ship_view_r[0] (nearest) to [1] (farthest), range rings, lines of sight coloured by owner face,
     grey when occulted or outside every face, orange when masked by the structure;
  V5 the volumes of the linked mission at the current time (mission at t, sim/engine.py), redrawn when paused and
     every 1 s of simulated time while playing;
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
from sim import engine
from sim import ephemeris as eph
from sim import geometry as sgeo
from sim import propagate as prop
from sim import scenario as scn

TRAIL_S = float(P.get("tool.display.sim_trail_s"))
FPS = int(P.get("tool.display.sim_fps"))
SHIP_R0, SHIP_R1 = (float(x) for x in P.get("tool.display.sim_ship_view_r"))
AUTO_MAX_CELLS = float(P.get("tool.display.sim_auto_max_cells"))
SPEEDS = (1, 10, 100, 1000, 10000, 100000)     # x10 000 and x100 000: the Moon on its orbit (user)
VOLUME_EVERY_S = 1.0                              # V5: mission volumes redrawn every this much simulated time
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
        self._vol_t: Optional[float] = None
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
        for cb in (self.cb_labels, self.cb_vectors, self.cb_trails):
            cb.toggled.connect(lambda _on: self._draw_frame())
        self.cb_centre.currentTextChanged.connect(lambda _t: self._apply_centre(reframe=True))
        self.cb_volumes.toggled.connect(lambda _on: self._draw_volumes(force=True))
        for cb in (self.cb_labels, self.cb_vectors, self.cb_trails, self.cb_volumes):
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
        self.plotter.iren.interactor.AddObserver("LeftButtonPressEvent", self._on_click, 1.0)
        self.plotter.set_background(BG_COLOR)
        self.plotter.renderer.SetNearClippingPlaneTolerance(NEAR_TOL)
        self.plotter.add_axes(color=TEXT_COLOR)

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
        self._vol_t = None
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
            self._draw_volumes(force=True)

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
            self._draw_volumes(force=True)
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
        self._vol_t = None
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
                    sv.set_default_camera(p)
                    cam.position = tuple(np.asarray(cam.position) * max(1.0, SHIP_R1 / 1.5))
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
        p = self.plotter
        sv.add_static_scene(p)
        p.set_background(BG_COLOR)
        p.enable_lightkit()
        self._centre_pending = True
        scene = cgeo.build_face_sectors(self.grid, -90.0, 90.0)
        for k in range(4):
            if scene.surfaces[k].n_points:
                p.add_mesh(scene.surfaces[k], color=cgeo.FACE_COLORS[k], opacity=0.15, culling="back",
                           name=f"face_{k}", render=False)
        r_min, r_max = self._r_log
        for e in range(int(math.floor(math.log10(r_min))), int(math.ceil(math.log10(r_max))) + 1):
            rk = 10.0 ** e
            if not r_min * 0.999 <= rk <= r_max * 1.001:
                continue
            rad = self._radius(np.array([rk]))[0]
            ang = np.linspace(0.0, 2.0 * math.pi, 121)
            ring = np.stack([rad * np.cos(ang), rad * np.sin(ang), np.zeros_like(ang)], axis=1)
            p.add_mesh(pv.lines_from_points(ring), color="#4a5266", line_width=1, name=f"ring_{e}", render=False)
            p.add_point_labels([[rad, 0.0, 0.0]], [f"{rk:g} km"], font_size=9, shape=None, show_points=False,
                               text_color=MUTED_COLOR, name=f"ring_lbl_{e}", render=False)
        sv.set_default_camera(p)
        p.camera.position = tuple(np.asarray(p.camera.position) * max(1.0, SHIP_R1 / 1.5))      # objects out to SHIP_R1

    def _radius(self, rng_km: np.ndarray) -> np.ndarray:
        """V4: log range scale from SHIP_R0 (nearest object of the run) to SHIP_R1 (farthest)."""
        a, b = (math.log10(x) for x in self._r_log)
        if b - a < 1e-9:
            return np.full(len(rng_km), 0.5 * (SHIP_R0 + SHIP_R1))
        x = np.clip((np.log10(np.maximum(rng_km, 1e-6)) - a) / (b - a), 0.0, 1.0)
        return SHIP_R0 + (SHIP_R1 - SHIP_R0) * x

    def _clear_dynamic(self, prefix: str = DYN) -> None:
        sv.remove_actors(self.plotter, [k for k in list(self.plotter.actors) if k.startswith(prefix)])

    def _draw_frame(self) -> None:
        self._update_time_label()
        if self.truth is None:
            return
        self._clear_dynamic()
        if self._global():
            self._frame_global()
        else:
            self._frame_ship()
        self._show_info()
        self.plotter.render()

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
        cols = [SHIP_COLOR if i == 0 else CLASS_COLORS.get(tr.classes[i], "#888888") for i in range(len(tr.ids))]
        idx = np.flatnonzero(ex)
        pts = pv.PolyData(r[idx])                      # one actor for every body point (cheap frames)
        pts["rgb"] = np.array([_rgb(cols[i]) for i in idx], np.uint8)
        p.add_mesh(pts, scalars="rgb", rgb=True, point_size=8, render_points_as_spheres=True,
                   name=f"{DYN}pts", render=False)
        if self.cb_labels.isChecked():
            ids = [OWNSHIP if i == 0 else tr.ids[i] for i in np.flatnonzero(ex)]
            p.add_point_labels(r[ex], ids, font_size=10, shape=None, show_points=False, text_color=TEXT_COLOR,
                               always_visible=True, name=f"{DYN}labels", render=False)
        if self.cb_trails.isChecked():
            ts = np.linspace(max(0.0, self.t - TRAIL_S), self.t, 25)
            past = np.array([sgeo.states_at(tr, float(t))[0] for t in ts])
            for i in np.flatnonzero(ex):
                line = _polyline(past[:, i])
                if line is not None:
                    p.add_mesh(line, color=cols[i], line_width=3, opacity=0.8, name=f"{DYN}trail_{i}", render=False)
        if self.cb_vectors.isChecked() and ex.any():
            p.add_mesh(_segments(r[ex], r[ex] + v[ex] * TRAIL_S), color=MUTED_COLOR, line_width=1,
                       name=f"{DYN}vel", render=False)
        # ship attitude triad, enlarged (V3)
        m = tr.attitude("ship", self.t)
        L = TRIAD_FRAC * p.camera.GetDistance()        # the triad keeps its size on screen at any zoom
        for k, c in enumerate((cgeo.COLORS["axis_x"], cgeo.COLORS["axis_y"], cgeo.COLORS["axis_z"])):
            p.add_mesh(pv.Line(tuple(r[0]), tuple(r[0] + L * m[:, k])), color=c, line_width=3,
                       name=f"{DYN}triad_{k}", render=False)
        # Sun direction from the central body
        T = eph.centuries_tt(scn.epoch(self.sc), self.t)
        moon, sun = eph.moon_icrf_km(T), eph.sun_icrf_km(T)
        centre_geo = np.zeros(3) if tr.body.name == "earth" else moon
        su = (sun - centre_geo) / np.linalg.norm(sun - centre_geo)
        R = tr.body.radius_km
        p.add_mesh(pv.Arrow(start=tuple(su * 1.1 * R), direction=tuple(su), scale=1.5 * R), color=SUN_COLOR,
                   name=f"{DYN}sun", render=False)
        p.add_point_labels([su * 2.7 * R], ["Sun"], font_size=10, shape=None, show_points=False,
                           text_color=SUN_COLOR, name=f"{DYN}sun_lbl", render=False)

    def _frame_ship(self) -> None:
        p, tr, g = self.plotter, self.truth, self.g
        r, _v = sgeo.states_at(tr, self.t)
        m = tr.attitude("ship", self.t)
        rows = np.arange(1, len(tr.ids))
        ro = r[rows]
        ex = np.isfinite(ro[:, 0])
        if not ex.any():
            self._screen_pts = {}
            return
        los = ro[ex] - r[0]
        rng = np.linalg.norm(los, axis=1)
        d = (los / rng[:, None]) @ m
        pos = d * self._radius(rng)[:, None]
        face = opt.face_of(d[:, 0], d[:, 1], d[:, 2])[0]
        k = self._k()
        js = np.flatnonzero(ex)
        occ = g.occulted_by[k, js] >= 0
        msk = g.masked[k, js]
        cols = []
        for f, o, mk in zip(face, occ, msk):
            cols.append(OCC_COLOR if (o or f < 0) else MASK_COLOR if mk else cgeo.FACE_COLORS[int(f)])
        for c in set(cols):
            sel = [i for i, x in enumerate(cols) if x == c]
            p.add_mesh(_segments(np.zeros((len(sel), 3)), pos[sel]), color=c, line_width=1.5, opacity=0.7,
                       name=f"{DYN}los_{c}", render=False)
        for jj, j in enumerate(js):
            c = CLASS_COLORS.get(g.classes[j], "#555555")
            p.add_mesh(pv.PolyData(pos[jj][None, :]), color=c, point_size=9, render_points_as_spheres=True,
                       name=f"{DYN}pt_{j}", render=False)
        if self.cb_labels.isChecked():
            p.add_point_labels(pos, [g.ids[j] for j in js], font_size=10, shape=None, show_points=False,
                               text_color=TEXT_COLOR, always_visible=True, name=f"{DYN}labels", render=False)
        if self.cb_vectors.isChecked():
            nose = g.nose_ship[k, js]
            okn = np.isfinite(nose[:, 0])
            if okn.any():
                p.add_mesh(_segments(pos[okn], pos[okn] + 0.15 * nose[okn]), color=TEXT_COLOR, line_width=2,
                           name=f"{DYN}nose", render=False)
        if self.cb_trails.isChecked():
            sel = (g.t_s >= self.t - TRAIL_S) & (g.t_s <= self.t)
            for j in js:
                dd, rr = g.dir_ship[sel, j], g.range_km[sel, j]
                ok = np.isfinite(rr)
                if ok.sum() >= 2:
                    line = pv.lines_from_points(dd[ok] * self._radius(rr[ok])[:, None])
                    p.add_mesh(line, color=CLASS_COLORS.get(g.classes[j], "#555555"), line_width=2, opacity=0.6,
                               name=f"{DYN}trail_{j}", render=False)
        self._screen_pts = {g.ids[j]: pos[jj] for jj, j in enumerate(js)}
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False

    # ------------------------------------------------------------------ V5 mission volumes
    def _draw_volumes(self, force: bool = False) -> None:
        if self.truth is None or self._global() or not self.sc.get("mission"):
            self._clear_dynamic(VOL)
            return
        if not self.cb_volumes.isChecked():
            self._clear_dynamic(VOL)
            self.plotter.render()
            return
        if not force and self._vol_t is not None and abs(self.t - self._vol_t) < VOLUME_EVERY_S:
            return
        self._vol_t = self.t
        try:
            if self._mission is None or self._mission[0] != self.sc["mission"]:
                from core import mission as cm
                self._mission = (self.sc["mission"], cm.load_mission(self.sc["mission"]))
            mis, _alerts = engine.mission_at(self.sc, self.truth, self.t, self._mission[1])
            res, _ = frames.resolve(mis)
        except (OSError, ValueError, KeyError) as exc:
            self.status.setText(f"mission volumes: {exc}")
            return
        self._clear_dynamic(VOL)
        gl = res.get("global", {})
        e0, e1 = float(gl.get("eps_min_deg", -90.0)), float(gl.get("eps_max_deg", 90.0))
        for i, sec in enumerate(res.get("sectors", [])):
            try:
                poly, outline = cgeo.build_sector_overlay(self.grid, sec, e0, e1)
            except (KeyError, TypeError, ValueError):
                continue
            col = cgeo.sector_color(i)
            if poly.n_points:
                self.plotter.add_mesh(poly, color=col, opacity=0.35, culling="back", name=f"{VOL}{i}_s", render=False)
            if outline.n_points:
                self.plotter.add_mesh(outline, color=col, line_width=2, name=f"{VOL}{i}_o", render=False)
        self.plotter.render()

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
