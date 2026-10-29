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
  V4 ship view (ship frame): the ship model, faces, objects at their exact direction and at 1:1 in km (the ship
     shown by its axes, the volumes at their true size; the log range scale removed, user 29 October 2026: it bent the
     paths too much), range rings of about constant size on screen whose round range values
     follow the zoom, lines of sight coloured by owner face,
     grey when occulted or outside every face, orange when masked by the structure;
  V5 the volumes of the linked mission at the current time (the navigation at t, sim/engine.py): built once, then
     turned with their frame at every frame (rebuilt past VOL_REBUILD_DEG of turn);
  performance (user, 29 October 2026, measured): the dynamic actors are created once and their points updated in
     place, labels by one vtkLabeledDataMapper each, one render per frame (the Qt repaint);
  L1 play / pause, step +/- (journal step), slider, speed x1 .. x100 000 (no reverse: user, 29 October 2026); time from the epoch and UTC;
  L2 tool.display.sim_fps frames per second: positions interpolated at each frame (Hermite), the face / aspect / RCS
     read from the geometry at the nearest recorded instant;
  L3 trails of a chosen duration (tool.display.sim_trail_s at start), planned trajectories (from t to the run's
     end), labels, orbital velocity vectors of a chosen duration and noses (ship view) on demand;
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
import json
import math
import os
from functools import lru_cache
import time
import weakref
from datetime import timedelta
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QMessageBox, QProgressBar, QPushButton, QShortcut, QVBoxLayout, QWidget)
from PyQt5.QtGui import QKeySequence
from pyvistaqt import QtInteractor

from core import frames
from core import geometry as cgeo
from core import optimizer as opt
from core.params import P
from gui import scene_view as sv
from gui import controls as uc
from gui import sky as gsky
from sim import attitude as att
from sim import engine
from sim import ephemeris as eph
from sim import geometry as sgeo
from sim import propagate as prop
from sim import scenario as scn

TRAIL_S = float(P.get("tool.display.sim_trail_s"))
ORBIT_VECTOR_S = float(P.get("tool.display.sim_orbit_vector_s"))   # L3: the orbital vector at start, s
FPS = int(P.get("tool.display.sim_fps"))
AUTO_MAX_CELLS = float(P.get("tool.display.sim_auto_max_cells"))
MAX_VIEW_KM = float(P.get("tool.display.sim_max_view_km"))     # farthest zoom out (user)
MIN_RING_KM = float(P.get("tool.display.sim_min_ring_km"))     # nearest zoom in: the innermost ring (user)
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
RING_HYST = 2.0                                   # the rings' step kept while within this factor of the ideal one (D)
RING_POINTS = 121
PATH_MAX_PTS = 400                                # L3, global view: points of a whole-run path (trail, planned)
COAST_STEP_S = 10.0                               # L3: RK4 step of the orbital velocity vector's free fall, s (D)
COAST_STEPS = (20, 200)                           # ... its step count kept between these (smooth, 1000+ objects)
# the objects and their geometry (points, trajectories, trails, range projection) in the colour of their
# classification (user, 29 October 2026), the identity frames' colours of the symbology (tools/gen_symbols.py)
IDENTITY_COLORS = {"friendly": "#4FC3F7", "hostile": "#FF4D4D", "suspect": "#FF9A3C", "neutral": "#5CD65C",
                   "unknown": "#FFE24D", "pending": "#FFE24D"}
OWNSHIP_IDENTITY = "friendly"                     # OWNSHIP is friendly (user): always drawn in that colour
SHIP_COLOR = IDENTITY_COLORS[OWNSHIP_IDENTITY]    # its point, trajectory and trail
KM_PER_MODEL_UNIT = 1e-3 / cgeo._M                # 1:1: the model's scene units to km (its true size)
OPTION_COLUMNS = 2                                # the view's switches: two columns per section (user)
BG_COLOR = "#000000"                              # background of the 3D views: the black of space (user, 29 October 2026)
TEXT_COLOR = "#e6e6e6"
# the widest Info lines of an object or the ship (side_width: the column fits them and the bodies' facts)
INFO_WIDEST = ("classif.\tsuspect", "frame\tICRF / earth", "pos x\t-1234567.890 km", "vel x\t-12.34567 km/s",
               "block loss\t12.34 dB / pass", "rate x\t-12.345 deg/s", "range\t1234567.890 km", "Sun hidden\t100 %",
               "CPA in\t> 12345.6 s (after the end)", "CPA range\t< 1234567.890 km")
SYMBOL_DIR = cgeo.MOON_TEXTURE_PATH.parent.parent / "symbols"   # assets/symbols: <classification>_<class>.png
SYMBOL_PX = 42                                    # px: a symbol on screen in the 3D views (user, 30 October 2026: 28 x 1.5)
SELECT_PX = 66                                    # px: the square around the selected body (44 x 1.5)
# multitracks (user, 29 October 2026): objects of one identity and class closer than this on screen (centre to
# centre, x the UI size) drawn as one symbol with a doubled outline, their number under it in the frame's colour
MULTITRACK_PX = float(P.get("tool.display.sim_multitrack_px"))
MULTITRACK_OFF_KM = float(P.get("tool.display.sim_multitrack_off_km"))   # ... none when zoomed in under this (user)
MT_COUNT_GAP_PX = 4.5                             # ... the number this far under the symbol (D)
MT_SEL = "<multi>"                                # a multitrack's selection key: MT_SEL + its members' ids, "|"
SELECT_COLOR = "#ffffff"
# "UI size" of the view's options (user, 29 October 2026): the icons and texts of the 3D view (symbols, the
# selection's square, labels: ids, range and azimuth marks, sectors, axes, events, bodies) x one of these
UI_SCALES = (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0)
UI_SCALE_START = 1.0                              # (the sizes below: those of the former 150 %, user 30 October 2026)
BOX_GAP = 0.15                                    # data box: its gap to the symbol, share of the symbol's size (D)
CPA_REFINE_ITER = 40                              # golden-section steps refining the closest approach between records
MUTED_COLOR = "#9aa0aa"
EARTH_COLOR = "#4a78b5"
ATM_COLOR = "#8cc4ff"                             # the Earth's atmosphere: a halo seen at the limb
ATM_SHELLS = tuple(1.0 + 0.04 * (k / 10) ** 1.5 for k in range(1, 11))   # its shells' radii / the Earth's: denser
ATM_OPACITY = 0.07                                # at the surface, fading out (outermost ~250 km, exaggerated to be
                                                  # readable); the opacity of each shell
SUN_COLOR = "#e0a800"
SUN_HALO_R = float(P.get("tool.display.sim_sun_halo_r"))            # the Sun's halo: outer radius / the Sun's
SUN_HALO_OPACITY = float(P.get("tool.display.sim_sun_halo_opacity"))   # at the limb, exponential to 0 at the edge
SUN_HALO_SCALE = float(P.get("tool.display.sim_sun_halo_scale"))       # its e-folding length / the Sun's radius
SUN_LIMB_U = float(P.get("tool.display.sim_sun_limb_u"))               # limb darkening, linear coefficient
# the Sun as seen from space (user, 30 October 2026): the photosphere's warm white (~5772 K), the map's granulation
# kept at SUN_GRANULATION, darker and more orange toward the limb, a warm white glare around it
SUN_WHITE = (255, 244, 234)
SUN_GRANULATION = 0.15
SUN_LIMB_COLOR = (205, 105, 35)
SUN_GLARE_COLOR = (255, 238, 210)
OCC_COLOR = "#a0a0a0"
MASK_COLOR = "#e07b00"
MIN_IDLE_MS = 15                                  # event-loop time kept between two frames (mouse live while playing)
SIDEREAL_MONTH_S = 27.321661 * 86400.0            # the Moon's orbit drawn over one sidereal month
SIDEREAL_YEAR_S = 365.256363 * 86400.0            # the Earth's orbit around the Sun drawn over one sidereal year
# a celestial body selected shows its orbit and those of the bodies orbiting it (user, 29 October 2026; not for the
# scenario's objects): its parent, its period
BODY_PARENT = {"moon": ("earth", SIDEREAL_MONTH_S), "earth": ("sun", SIDEREAL_YEAR_S)}
BODY_ORBIT_POINTS = 361
DATA_FONT = 15                                    # pt: the objects' data boxes and the celestial bodies' names,
#                                                   one size (user, 30 October 2026), x the UI size
BODY_AMBIENT = 0.12                               # global view: a body's night side, share of its lit colour
BODY_ORBIT_NEAR = 61                               # ... its points about t, computed at each frame (D)
BODY_ORBIT_COLOR = "#8a8a8a"                      # (a body without a colour of its own)
ORBIT_MIN_PEAK = 170.0                            # an orbit's colour: its brightest channel at least this (D)
ORBIT_HUE_SHARE = 0.2                             # ... a coloured part of the map at least this share (D)
PROJ_DASH_DEG = 4.0                               # the arc to the range projection dashed (user): one dash + gap
PROJ_DASHES = (2, 90)                             # ... per this many degrees of arc, dashes per arc kept between (D)
FOLLOW_MIN_KM = 50.0                              # "follow the ship": framed span at least this
TRIAD_FRAC = 0.03                                 # ship triad length, share of the camera distance
TEXTURE_DIR = cgeo.MOON_TEXTURE_PATH.parent       # assets/textures (equirectangular, 2:1)
TEXTURE_FILES = {"earth": "earth2.jpg", "moon": "moon.jpg", "sun": "sun.jpg"}   # the Earth: earth2 (user, 29 October 2026)
# the planets (user, 29 October 2026): positions sim/ephemeris.py, size, pole and rotation [sim.planets], textures
# <name>.jpg (Saturn's rings saturn_rings.png); too small on screen, a dot and their name
PLANET_NAMES = {"Mercury": "mercury", "Venus": "venus", "Mars": "mars", "Jupiter": "jupiter", "Saturn": "saturn",
                "Uranus": "uranus", "Neptune": "neptune", "Pluto": "pluto"}
PLANET_DATA = {k: dict(P.get(f"sim.planets.{k}")) for k in PLANET_NAMES.values()}
PLANET_MARK_PX = float(P.get("tool.display.sim_planet_mark_px"))
PLANET_DOT_PX = 5.0                               # ... the dot's size, px (D)
PLANET_COLORS = {"mercury": "#a8a29a", "venus": "#e8d6a8", "mars": "#d2694a", "jupiter": "#d8b98c",
                 "saturn": "#e3cf96", "uranus": "#9fd9e0", "neptune": "#5b7fe0", "pluto": "#c8b49a",
                 "sun": "#e0a800", "earth": "#4a78b5", "moon": "#bfbfbf"}   # (D; the Sun, the Earth, the Moon too)
SATURN_RINGS_KM = tuple(float(x) for x in P.get("tool.display.sim_saturn_rings_km"))
BODY_PARENT.update({p: ("sun", SIDEREAL_YEAR_S * eph.PLANET_ELEMENTS[p][0][0] ** 1.5)    # (Kepler: P ~ a^1.5)
                    for p in PLANET_NAMES.values()})
BODY_PARENT.update({m: (el["planet"], float(el["period_d"]) * 86400.0) for m, el in eph._MOONS.items()})
NEAR_FRAC = 0.01                                  # near clipping plane at least this x the camera distance
NEAR_TOL = 1e-9                                   # near / far clipping ratio: the Sun at 1 AU, objects at a few km
                                                  # (bodies drawn front faces only: no self-occlusion far away)
# the major moons (user, 29 October 2026): positions sim/ephemeris.py ([sim.moons], JPL mean elements), spheres of
# their mean radius in a colour of their own (Phobos: its shape model, assets/models/Phobos.obj, and its map); a
# moon drawn as a dot only once it parts from its planet's on screen (MOON_PART_PX)
MOON_NAMES = {n.capitalize(): n for n in eph.MOON_NAMES}
MOON_DATA = {n: dict(eph._MOONS[n]) for n in eph.MOON_NAMES}
MOON_PART_PX = 12.0                               # px: a moon's dot (and name) only this far from its planet's (D)
MOON_COLOR = "#b4b0aa"                            # (D) and a few of their own:
MOON_COLORS = {"io": "#e3cf6a", "europa": "#e6dfd0", "ganymede": "#a99d8f", "callisto": "#6f665c",
               "titan": "#d9a45a", "enceladus": "#f2f2f2", "iapetus": "#8f7f6c", "triton": "#d3c4bb",
               "charon": "#a39e98", "phobos": "#8a7d70", "deimos": "#a29484"}
MODEL_DIR = cgeo.MOON_TEXTURE_PATH.parent.parent / "models"
# the moons drawn by their shape model (assets/models/<Name>.obj, km): the model's axes taken as the body frame's
# (IAU: X the long axis, toward the planet; Z the pole, the short axis), checked on each model, 29 October 2026
# (axes: the model's columns taken as X, Y, Z, or "principal": its long, middle, short axes; scale: None, km,
# or "radius": to the moon's mean radius)
SHAPE_MODELS = {"phobos": ("Phobos.obj", (2, 0, 1), None),       # the model: Z long, Y short (Stickney at 49 W)
                "deimos": ("Deimos.obj", (0, 1, 2), None),       # the model in the IAU frame already
                "amalthea": ("Amalthea.obj", (0, 1, 2), None),   # Stooke (PDS): X long, Z short, km
                "thebe": ("Thebe.obj", (0, 1, 2), None),
                "larissa": ("Larissa.obj", (0, 1, 2), None),
                "hyperion": ("Hyperion.obj", "principal", None),  # Thomas (PDS): chaotic rotation, axes by convention
                # (its map laid in the model's frame: its great crater on the model's deepest basin)
                "iapetus": ("Iapetus.obj", (0, 1, 2), "radius")}  # Schenk: unit radius ~1.45, Z the pole (its ridge)
TEXTURE_MAX_PX = 8192                                     # a map wider than this read at this width (the GPU)
# the maps' longitudes (user's maps, checked on known features, 29 October 2026): (east longitude of the left edge,
# 1: east to the right / -1: west to the right); the others: -180, east to the right (0 at the centre)
MAP_LON = {**{n: (0.0, 1) for n in ("io", "europa", "mimas", "enceladus", "tethys", "dione", "rhea", "titan",
                                    "iapetus")}}      # 0 to 360 E from the left: Pele, Loki; Herschel, Odysseus, Xanadu
# (Ganymede, Callisto, Phobos: the default, checked on Galileo Regio and Osiris, Valhalla, Stickney)
PLANET_COLORS.update({n: MOON_COLORS.get(n, MOON_COLOR) for n in MOON_DATA})   # (their dots)
BODY_NAMES = {"Earth": "earth", "Moon": "moon", "Sun": "sun", **PLANET_NAMES, **MOON_NAMES}   # the bodies to centre on
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


def eph_sun_radius_km() -> float:
    return sgeo.SUN_RADIUS_KM


def _rgb(hex_color: str) -> Tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


@lru_cache(maxsize=4)
@lru_cache(maxsize=None)
def _body_color(body: str) -> str:
    """A body's dominant colour (its orbit's, user 30 October 2026): of its map's eight colours (median cut; a mean
    washes them out: the Earth's came out grey), the most frequent coloured one, else its plain colour; brightened up to
    ORBIT_MIN_PEAK on its brightest channel to stand out on the black."""
    path = TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")
    rgb = None
    if path.exists():
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        im = Image.open(path)
        im.draft("RGB", (256, 128))
        q = im.convert("RGB").resize((128, 64)).quantize(colors=8)
        pal = q.getpalette()
        counts = sorted(q.getcolors(), reverse=True)                # (count, index), most frequent first
        total = float(sum(c for c, _i in counts))

        def sat(i):
            c = np.array(pal[3 * i:3 * i + 3], float)
            return (c.max() - c.min()) / max(c.max(), 1.0)
        # a coloured part over ORBIT_HUE_SHARE of the map before a grey or white one (the Earth: its oceans, not its
        # clouds); else the most frequent
        hue = [i for c, i in counts if sat(i) > 0.2 and c / total >= ORBIT_HUE_SHARE]
        idx = hue[0] if hue else counts[0][1]
        rgb = np.array(pal[3 * idx:3 * idx + 3], float)
        if sat(idx) < 0.12 and body in PLANET_COLORS:       # a grey map (the Earth's earth2, Titan's radar one):
            rgb = None                                       # its plain colour (blue, orange) says it better
    if rgb is None:
        col = PLANET_COLORS.get(body, MOON_COLORS.get(body, EARTH_COLOR if body == "earth" else MOON_COLOR))
        rgb = np.array(_rgb(col), float)
    peak = max(float(rgb.max()), 1.0)
    if peak < ORBIT_MIN_PEAK:
        rgb = rgb * ORBIT_MIN_PEAK / peak
    return "#%02x%02x%02x" % tuple(int(round(min(255.0, x))) for x in rgb)


def _has_texture(body: str) -> bool:
    return (TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")).exists() or body == "moon"


@lru_cache(maxsize=None)
def _texture(body: str) -> Optional[pv.Texture]:
    """Equirectangular texture of a body (TEXTURE_DIR/TEXTURE_FILES[body]; the Moon falls back on the procedural
    one), read once: in RGB (an indexed or transparent map would draw wrong), at most TEXTURE_MAX_PX wide; repeated
    (a shape model's seam triangles reach past u = 1)."""
    path = TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")
    if path.exists():
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        im = Image.open(path)
        if im.width > TEXTURE_MAX_PX:
            im.draft("RGB", (TEXTURE_MAX_PX, TEXTURE_MAX_PX // 2))      # (a JPEG decoded at a smaller scale)
            if im.width > TEXTURE_MAX_PX:
                im = im.resize((TEXTURE_MAX_PX, max(1, round(im.height * TEXTURE_MAX_PX / im.width))),
                               Image.BILINEAR)
        tex = pv.Texture(np.asarray(im.convert("RGB")))                  # (pyvista: the first row at the top)
    elif body == "moon":
        tex = pv.Texture(cgeo.moon_texture())
    else:
        return None
    tex.interpolate = True
    tex.mipmap = True
    tex.repeat = True
    return tex


@lru_cache(maxsize=8)
def _body_mesh(radius_km: float, n_lon: int = 180, n_lat: int = 90, lon_left: float = -180.0,
               sense: int = 1) -> pv.PolyData:
    """Sphere in the body-fixed frame (Z the pole, X the prime meridian) with equirectangular texture coordinates:
    u = sense (longitude east - lon_left) / 360 (MAP_LON: the map's left edge at lon_left, east to the right for
    sense 1, west for -1; by default -180, east to the right), v = (latitude + 90) / 180 (north at the top)."""
    lon = np.radians(np.linspace(-180.0, 180.0, n_lon + 1))
    lat = np.radians(np.linspace(-90.0, 90.0, n_lat + 1))
    lo, la = np.meshgrid(lon, lat, indexing="ij")
    pts = radius_km * np.stack([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)], axis=-1).reshape(-1, 3)
    uv = np.stack([sense * (np.degrees(lo) - lon_left) / 360.0, (np.degrees(la) + 90.0) / 180.0],
                  axis=-1).reshape(-1, 2)                     # (continuous across the map's edge: texture repeat)
    i, j = np.meshgrid(np.arange(n_lon), np.arange(n_lat), indexing="ij")
    a = (i * (n_lat + 1) + j).ravel()
    quads = np.stack([np.full(a.size, 4), a, a + n_lat + 1, a + n_lat + 2, a + 1], axis=1).ravel()
    mesh = pv.PolyData(pts, faces=quads)
    mesh.active_texture_coordinates = uv
    return mesh.compute_normals(auto_orient_normals=True)


def _phobos_mesh() -> Optional[pv.PolyData]:
    return _shape_mesh("phobos")


@lru_cache(maxsize=None)                          # (asked for every moon at every frame: all kept)
def _shape_mesh(name: str) -> Optional[pv.PolyData]:
    """A moon's shape model (km, SHAPE_MODELS) in its body frame (IAU: X its long axis, toward its planet; Z its pole,
    the short axis), centred on its centroid, with an equirectangular mapping of its map computed from the shape
    (planetocentric longitude east from X, latitude: the models' own coordinates do not make one map); the triangles
    across the 180 deg meridian split there. None without a model (a sphere then)."""
    if name not in SHAPE_MODELS or not (MODEL_DIR / SHAPE_MODELS[name][0]).exists():
        return None
    m = pv.read(str(MODEL_DIR / SHAPE_MODELS[name][0])).triangulate()
    _file, axes, scale = SHAPE_MODELS[name]
    p = np.asarray(m.points, float)
    p = p - p.mean(axis=0)
    native = p                                               # (its map: laid in the model's own frame)
    if axes == "principal":                                  # its long, middle, short axes as X, Y, Z
        w, vec = np.linalg.eigh(np.cov(p.T))
        vec = vec[:, np.argsort(w)[::-1]]
        if np.linalg.det(vec) < 0:
            vec[:, 2] = -vec[:, 2]
        p = p @ vec
    else:
        p = p[:, list(axes)]
        native = p                                           # (a body frame by its axes: the map's too)
    if scale == "radius":                                    # (a model of unit size: at the moon's mean radius)
        p = p * float(eph._MOONS[name]["radius_km"]) / float(np.linalg.norm(p, axis=1).mean())
    faces = m.faces.reshape(-1, 4)[:, 1:]
    tri = p[faces].reshape(-1, 3)                            # one point per corner: each triangle its own UVs
    src = native[faces].reshape(-1, 3)
    d = src / np.linalg.norm(src, axis=1)[:, None]
    lon = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    left, sense = MAP_LON.get(name, (-180.0, 1))
    u = ((sense * (lon - left) / 360.0) % 1.0).reshape(-1, 3)
    v = (np.degrees(np.arcsin(np.clip(d[:, 2], -1.0, 1.0))) + 90.0) / 180.0
    wrap = u.max(axis=1) - u.min(axis=1) > 0.5               # across the seam: its small u past 1 (texture repeat)
    u[wrap] = np.where(u[wrap] < 0.5, u[wrap] + 1.0, u[wrap])
    n = len(faces)
    out = pv.PolyData(tri, faces=np.hstack([np.full((n, 1), 3), np.arange(3 * n).reshape(-1, 3)]).ravel())
    out.active_texture_coordinates = np.c_[u.ravel(), v]
    return out.compute_normals(split_vertices=False, auto_orient_normals=True)


def _synchronous_matrix(rel: np.ndarray, vel: np.ndarray) -> np.ndarray:
    """Body-fixed -> ICRF of a moon locked on its planet: X toward the planet, Z along its orbit's normal."""
    x = -rel / np.linalg.norm(rel)
    z = np.cross(rel, vel)
    z = z / np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


@lru_cache(maxsize=1)
def _rings_mesh() -> pv.PolyData:
    """Saturn's rings in its equatorial plane (km), the texture across them from the inner to the outer edge."""
    ring = pv.Disc(inner=SATURN_RINGS_KM[0], outer=SATURN_RINGS_KM[1], r_res=1, c_res=180)
    r = np.linalg.norm(ring.points[:, :2], axis=1)
    ring.active_texture_coordinates = np.c_[(r - SATURN_RINGS_KM[0]) / (SATURN_RINGS_KM[1] - SATURN_RINGS_KM[0]),
                                            np.full(len(r), 0.5)]
    return ring


def _rings_texture() -> Optional[pv.Texture]:
    path = TEXTURE_DIR / "saturn_rings.png"
    if not path.exists():
        return None
    tex = pv.Texture(str(path))
    tex.interpolate = True
    return tex


def _earth_matrix(epoch, t_s: float) -> np.ndarray:
    """Earth-fixed -> ICRF: a turn by the Greenwich mean sidereal time about the pole (ICRF Z, as the propagation,
    P2; IAU 1982 GMST, UT1 taken as UTC)."""
    d = ((epoch - eph._J2000).total_seconds() + t_s) / 86400.0
    th = math.radians((280.46061837 + 360.98564736629 * d) % 360.0)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


SUN_ROTATION = dict(P.get("sim.sun_rotation"))     # IAU pole and prime meridian of the Sun


def _sun_matrix(epoch, t_s: float) -> np.ndarray:
    """Sun-fixed -> ICRF (user, 29 October 2026: the bodies turn on themselves)."""
    return _iau_matrix(SUN_ROTATION, epoch, t_s)


def _iau_matrix(rot: dict, epoch, t_s: float) -> np.ndarray:
    """Body-fixed -> ICRF of a body with an IAU rotation (the Sun, the planets): Z the IAU pole, X the prime meridian
    W = w0 + rate x days from J2000 (IAU WGCCRE: R = Rz(ra0 + 90) Rx(90 - dec0) Rz(W))."""
    d = ((epoch - eph._J2000).total_seconds() + t_s) / 86400.0
    a = math.radians(float(rot["pole_ra_deg"]) + 90.0)
    b = math.radians(90.0 - float(rot["pole_dec_deg"]))
    w = math.radians((float(rot["w0_deg"]) + float(rot["rate_deg_day"]) * d) % 360.0)

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


def _as_groups(rows) -> List[np.ndarray]:
    """Rows (indices) as groups of one, or groups (arrays of indices) as given."""
    return [np.atleast_1d(np.asarray(g, int)) for g in rows]


def _group_mean(x: np.ndarray, groups: List[np.ndarray]) -> np.ndarray:
    """x (..., n, 3) -> (..., n_groups, 3): each group the mean of its members where they exist (a multitrack's
    path, user 29 October 2026), a group of one its member."""
    out = np.empty(x.shape[:-2] + (len(groups), 3))
    one = np.array([len(g) == 1 for g in groups], bool)
    if one.any():
        out[..., one, :] = x[..., np.array([g[0] for g, o in zip(groups, one) if o], int), :]
    import warnings
    for gi in np.flatnonzero(~one):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)      # (no member there yet: NaN)
            out[..., gi, :] = np.nanmean(x[..., groups[gi], :], axis=-2)
    return out


def _polylines(pts: np.ndarray, ok: np.ndarray):
    """One polyline per column of pts (n_t, n, 3) through its points where ok (n_t, n), in time order, all in one
    pass (1000+ objects): (points, VTK lines, point count of each kept column, kept columns); a column of fewer than
    two points is left out; None: none left."""
    cnt = ok.sum(axis=0)
    keep = np.flatnonzero(cnt >= 2)
    if not len(keep):
        return None
    pts = pts[:, keep].transpose(1, 0, 2)[ok[:, keep].T]                  # object-major, time order
    c = cnt[keep]
    heads = np.cumsum(np.r_[0, c[:-1] + 1])                                # where each polyline's count goes
    lines = np.empty(int(c.sum()) + len(c), int)
    body = np.ones(len(lines), bool)
    body[heads] = False
    lines[heads] = c
    lines[body] = np.arange(int(c.sum()))
    return pts, lines, c, keep


def _value_box(value: float, lo: float, hi: float, dec: int, suffix: str, tip: str) -> QDoubleSpinBox:
    """A number beside a view switch (its duration)."""
    w = QDoubleSpinBox()
    w.setRange(lo, hi)
    w.setDecimals(dec)
    w.setValue(value)
    w.setSuffix(suffix)
    w.setKeyboardTracking(False)                  # redrawn once the number is typed, not at each digit
    w.setToolTip(tip)
    return w


def _row(*widgets: QWidget) -> QWidget:
    """Widgets side by side on one line of the options."""
    w = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    for x in widgets:
        lay.addWidget(x)
    lay.addStretch(1)
    return w


def _add_atmosphere(p, name: str):
    """The Earth's atmosphere: translucent shells (ATM_SHELLS x a unit radius, placed by the Earth's matrix) drawn
    by their far (inner) faces only, so the Earth hides their middle and they show as a halo at the limb, brighter
    at the surface where they overlap."""
    shells = pv.merge([_body_mesh(k) for k in ATM_SHELLS])
    return p.add_mesh(shells, color=ATM_COLOR, opacity=ATM_OPACITY, lighting=False, culling="front", name=name,
                      render=False)


def _add_sun_halo(p, name: str):
    """The Sun's halo, its glare (user, 29 then 30 October 2026): a ring from the solar radius to SUN_HALO_R x it
    (unit radius, placed and scaled by _place_halo), always facing the camera (vtkFollower), in SUN_GLARE_COLOR, its
    opacity SUN_HALO_OPACITY at the limb falling exponentially (SUN_HALO_SCALE) to 0 at its edge; the Sun's near half
    hides its middle. Not pickable (the clicks pick the disc)."""
    from vtkmodules.vtkRenderingCore import vtkFollower, vtkPolyDataMapper
    ring = pv.Disc(inner=1.0, outer=SUN_HALO_R, r_res=40, c_res=96)
    r = np.linalg.norm(ring.points[:, :2], axis=1)
    end = math.exp(-(SUN_HALO_R - 1.0) / SUN_HALO_SCALE)
    alpha = SUN_HALO_OPACITY * np.clip((np.exp(-(r - 1.0) / SUN_HALO_SCALE) - end) / (1.0 - end), 0.0, 1.0)
    rgb = np.array(SUN_GLARE_COLOR, float)
    ring.point_data.set_scalars(np.column_stack([np.tile(rgb, (len(r), 1)), 255.0 * alpha]).astype(np.uint8), "rgba")
    mapper = vtkPolyDataMapper()
    mapper.SetInputData(ring)
    mapper.SetColorModeToDirectScalars()
    act = vtkFollower()
    act.SetMapper(mapper)
    act.SetCamera(p.camera)
    act.GetProperty().LightingOff()
    p.add_actor(act, name=name, reset_camera=False, render=False)
    act.PickableOff()                              # after add_actor, which sets it pickable
    return act


def _add_sun_limb(p, name: str):
    """The Sun's limb darkening (user, 30 October 2026): a disc over the Sun's face, facing the camera (vtkFollower),
    in SUN_LIMB_COLOR with the opacity u (1 - mu), mu = sqrt(1 - (r/R)^2), I(mu) = 1 - u (1 - mu) the visible's
    linear law; placed by _place_limb in front of the Sun, at the radius of its outline. Not pickable."""
    from vtkmodules.vtkRenderingCore import vtkFollower, vtkPolyDataMapper
    disc = pv.Disc(inner=0.0, outer=1.0, r_res=40, c_res=96)
    r = np.clip(np.linalg.norm(disc.points[:, :2], axis=1), 0.0, 1.0)
    alpha = SUN_LIMB_U * (1.0 - np.sqrt(1.0 - r * r))
    rgb = np.array(SUN_LIMB_COLOR, float)
    disc.point_data.set_scalars(np.column_stack([np.tile(rgb, (len(r), 1)), 255.0 * alpha]).astype(np.uint8), "rgba")
    mapper = vtkPolyDataMapper()
    mapper.SetInputData(disc)
    mapper.SetColorModeToDirectScalars()
    act = vtkFollower()
    act.SetMapper(mapper)
    act.SetCamera(p.camera)
    act.GetProperty().LightingOff()
    p.add_actor(act, name=name, reset_camera=False, render=False)
    act.PickableOff()
    return act


def _place_limb(p, act, centre, radius_km: float) -> None:
    """The limb disc in front of the Sun (toward the camera, at its near point), at the radius of the Sun's outline
    seen from there (it hides nothing in front of the Sun: a body between stays over it)."""
    c = np.asarray(centre, float)
    cam = np.asarray(p.camera.GetPosition())
    to_cam = cam - c
    d = float(np.linalg.norm(to_cam))
    if d <= radius_km:
        act.SetVisibility(False)
        return
    u = to_cam / d
    front = c + u * radius_km * 0.999                     # (just behind the near point: no fight with the sphere)
    r_out = (d - radius_km) * radius_km / math.sqrt(d * d - radius_km * radius_km)
    act.SetCamera(p.camera)
    act.SetPosition(*(float(x) for x in front))
    act.SetScale(float(r_out))


@lru_cache(maxsize=1)
def _sun_texture() -> Optional[pv.Texture]:
    """The Sun's map as the photosphere seen from space: its warm white, the map's granulation kept as at most
    SUN_GRANULATION toward the limb's warmer colour (the map's orange washed out)."""
    path = TEXTURE_DIR / TEXTURE_FILES["sun"]
    if not path.exists():
        return None
    from PIL import Image
    a = np.asarray(Image.open(path).convert("L"), float)
    g = (a - a.mean()) / max(a.std(), 1e-6)                # granulation, unit contrast
    k = np.clip(SUN_GRANULATION * (1.0 - np.clip(g, -2.0, 2.0) / 2.0) / 2.0, 0.0, SUN_GRANULATION)
    white, warm = np.array(SUN_WHITE, float), np.array(SUN_LIMB_COLOR, float)
    img = (white + k[..., None] * (warm - white)).clip(0, 255).astype(np.uint8)   # (its dimmer parts warmer, not grey)
    tex = pv.Texture(img)
    tex.interpolate = True
    tex.mipmap = True
    return tex


def _place_halo(p, act, centre, radius_km: float) -> None:
    """The halo around the Sun drawn at centre with radius_km (its scene units)."""
    act.SetCamera(p.camera)
    act.SetPosition(*(float(x) for x in centre))
    act.SetScale(float(radius_km))


class _Labels:
    """Text labels at 3D points, always on top (vtkLabeledDataMapper): one actor, its points and texts updated in
    place (pyvista's add_point_labels rebuilds its whole pipeline at each call)."""

    def __init__(self, plotter, color: str, size: int, box: bool = False):
        # box: a data box (several lines, its top-left corner at the point, user 29 October 2026)
        self.box = box
        # the view's "UI size" (plotter._ui_scale) applies to every label of the plotter (rescale)
        self.base_size = size
        if not hasattr(plotter, "_ui_labels"):
            plotter._ui_labels = weakref.WeakSet()
        plotter._ui_labels.add(self)
        self._plotter = weakref.ref(plotter)
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
        tp.SetFontSize(max(1, round(size * getattr(plotter, "_ui_scale", 1.0))))
        tp.SetShadow(False)
        tp.SetBold(False)
        tp.SetItalic(False)
        tp.SetJustificationToLeft()
        if box:
            tp.SetVerticalJustificationToTop()
        self.actor = vtkActor2D()
        self.actor.SetMapper(self.mapper)
        plotter.add_actor(self.actor, reset_camera=False, render=False)
        self._last: List[str] = []

    def rescale(self) -> None:
        """The font at the plotter's UI size."""
        p = self._plotter()
        size = max(1, round(self.base_size * getattr(p, "_ui_scale", 1.0)))
        self.mapper.GetLabelTextProperty().SetFontSize(size)
        self.mapper.Modified()

    def set(self, pts: np.ndarray, texts: List[str]) -> None:
        if list(texts) != self._last:
            self.texts.SetNumberOfValues(len(texts))
            for i, t in enumerate(texts):
                self.texts.SetValue(i, t if self.box else f" {t}")
            self._last = list(texts)
        self.points.SetNumberOfPoints(len(pts))
        for i, q in enumerate(pts):
            self.points.SetPoint(i, float(q[0]), float(q[1]), float(q[2]))
        self.points.Modified()
        self.texts.Modified()
        self.poly.Modified()
        self.actor.SetVisibility(True)


BODY_SEL = "<body>"                               # a celestial body's selection key: BODY_SEL + its name
CELESTIAL_INFO = TEXTURE_DIR.parent / "celestial_info.json"   # the Info box of a body (display only)


@lru_cache(maxsize=1)
def celestial_info() -> dict:
    """The facts shown for the Earth, the Moon and the Sun (assets/celestial_info.json), {} if missing."""
    try:
        with open(CELESTIAL_INFO, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def _kv(label: str, value: str) -> str:
    """One row of the Info box (controls.InfoTable): its label, a tab, its value."""
    return f"{label}\t{value}"


def _dist_text(d: float) -> str:
    """A distance (km) in km, or in m below 1 km (to 0.1 mm below 1 m)."""
    return f"{d:.3f} km" if d >= 1.0 else f"{1e3 * d:.1f} m" if d >= 1e-3 else f"{1e3 * d:.4f} m"


def body_info_lines(name: str) -> List[str]:
    """The facts of a celestial body (celestial_info.json), one per row, under their short labels."""
    info = celestial_info()
    labels = info.get("labels", {})
    facts = info.get("bodies", {}).get(name, {})
    return [_kv("name", name)] + [_kv(labels.get(f, f), facts.get(f, "")) for f in info.get("fields", [])]


def _vec_lines(label: str, vec, unit: str, dec: int) -> List[str]:
    """A vector on three lines, one component each."""
    return [_kv(f"{label} {a}", f"{float(x):.{dec}f} {unit}".rstrip()) for a, x in zip("xyz", vec)]


def symbol_path(identity: str, cls: Optional[str], multi: bool = False) -> Optional[str]:
    """The symbol of a classification / characterisation couple (tools/gen_symbols.py), "ownship" for the ship,
    multi: its multitrack (doubled outline); None when its file is missing."""
    name = "ownship" if identity == "ownship" else f"{'multi_' if multi else ''}{identity}_{cls}"
    path = SYMBOL_DIR / f"{name}.png"
    return str(path) if path.exists() else None


@lru_cache(maxsize=64)
def _symbol_texture(path: str) -> pv.Texture:
    tex = pv.Texture(path)
    tex.interpolate = True
    return tex


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
    picked = pyqtSignal(str)                      # a body clicked in the view (OWNSHIP, an object id, a multitrack)

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
        self._r_far = 10.0                        # km: the run's farthest object (the ship view's home)
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
        self.cb_vectors = uc.Toggle("noses", tip="Ship view: each object's nose (its Z axis)")
        self.cb_trails = uc.Toggle("trails", True)
        # user, 29 October 2026: the trails' duration, the planned trajectories, the orbital velocity vectors
        self.sp_trail = _value_box(TRAIL_S, 0.0, 1e7, 0, " s", "Flight time drawn by each trail, s")
        self.cb_planned = uc.Toggle("planned trajectory", True, tip="Each body's trajectory from t to the run's end")
        self.cb_orbit = uc.Toggle("orbital velocity vector", tip="Each body's path from its state at t under the "
                                  "local gravity alone (no burn, no guidance) over the duration beside (ship view: "
                                  "relative to the ship's own free fall)")
        self.sp_orbit = _value_box(ORBIT_VECTOR_S, 1.0, 1e6, 0, " s",
                                   "Length of the orbital velocity vector: the free fall over this time, s")
        self.sp_orbit.setEnabled(False)
        for cb, sp in ((self.cb_trails, self.sp_trail), (self.cb_orbit, self.sp_orbit)):
            cb.toggled.connect(sp.setEnabled)
            sp.valueChanged.connect(lambda _x: self._draw_frame())
        # the body the view is centred on and follows: chosen by a double-click on it in the view (an object,
        # OWNSHIP, or in the global view the Earth, the Moon, the Sun), back to the default by a double-click on
        # nothing (user, 29 October 2026: no "centre on" list)
        self.centre = CENTRAL
        self.cb_volumes = uc.Toggle("mission volumes", True)
        self.cb_bodies = uc.Toggle("celestial objects")
        self.cb_bodies.setToolTip("Ship view: the Earth, the Moon and the Sun at their true directions, "
                                  "distances and sizes")
        self.cb_bodies.toggled.connect(lambda _on: self._draw_frame())
        for cb in (self.cb_labels, self.cb_vectors, self.cb_trails, self.cb_planned, self.cb_orbit):
            cb.toggled.connect(lambda _on: self._draw_frame())
        self.cb_labels.toggled.connect(lambda _on: (self._place_volumes(), self._render()))   # the sectors' labels
        self.cb_volumes.toggled.connect(lambda _on: self._draw_volumes(force=True))
        self.cb_proj = uc.Toggle("range projection", True)      # on by default (user)
        self.cb_proj.setToolTip("Ship view: a copy of each object in the plane of the range rings, at its range from "
                                "the ship (read on the rings), linked to the object by the arc of the sphere of that "
                                "range around the ship")
        self.cb_proj.toggled.connect(lambda _on: self._draw_frame())
        # partial: the objects and their copies in the plane, linked by the dashed arcs; full (user, 29 October
        # 2026): the copies only, a 2D view in the plane of the rings (the objects, their arcs not drawn)
        self.tg_proj_full = uc.Toggle("full", False, off_text="partial",
                                      tip="Range projection: the objects and their copies (partial), or the copies "
                                          "only, every body and path in the plane of the rings (full)")
        self.cb_proj.toggled.connect(self.tg_proj_full.setEnabled)
        self.tg_proj_full.toggled.connect(lambda _on: self._draw_frame())
        # D: the faces' responsibility zones (ship view), one switch for the four (user, 29 October 2026)
        self.cb_zones = uc.Toggle("responsibility zones", True)
        self.cb_zones.toggled.connect(lambda _on: (self._show_faces(), self._render()))
        # the axes the ship carries (user, 29 October 2026): its X, Y, Z axes and the faces' (arrays') normals of
        # the model, the 1:1 ship axes, the attitude triad of the global view
        self.cb_axes = uc.Toggle("ship axes", True)
        self.cb_axes.toggled.connect(lambda _on: self._draw_frame())
        # the objects drawn by their symbol (classification frame, characterisation glyph), else by a dot (user)
        self.cb_symbols = uc.Toggle("symbols", True)
        self.cb_symbols.toggled.connect(lambda _on: self._draw_frame())
        # every celestial body's orbit, whatever the selection (user, 30 October 2026; the global view)
        self.cb_all_orbits = uc.Toggle("all orbits", False, tip="Global view: the orbits of every planet and moon, "
                                       "whatever body is selected")
        self.cb_all_orbits.toggled.connect(lambda _on: self._draw_frame())
        # the star background (user, 29 October 2026): the real sky (skybox) or the plain background; the
        # constellation lines on demand
        self.tg_sky = uc.Toggle("skybox", False, off_text="plain",        # plain at start, both views (user)
                                tip="The stars at their true directions (Yale Bright Star Catalogue), or the plain "
                                    "background")
        self.cb_const = uc.Toggle("constellations", tip="The lines of the 88 constellations over the stars")
        self.cb_const.setEnabled(False)
        self.tg_sky.toggled.connect(self.cb_const.setEnabled)
        for cb in (self.tg_sky, self.cb_const):
            cb.toggled.connect(lambda _on: self._show_sky())
        # the layers are the same in both views (user, 29 October 2026): a switch set in one holds in the other
        self._common = (self.tg_sky, self.cb_const, self.cb_labels, self.cb_symbols, self.cb_all_orbits, self.cb_planned, _row(self.cb_trails, self.sp_trail),
                        _row(self.cb_orbit, self.sp_orbit), self.cb_axes)
        self._ship_only = (self.cb_vectors, self.cb_volumes, self.cb_zones, self.cb_bodies,
                           _row(self.cb_proj, self.tg_proj_full))
        # UI size (user, 29 October 2026): the icons and texts of the 3D view, - / +
        self.ui_scale = UI_SCALE_START
        self.bt_ui_minus = uc.tool_button("-", lambda: self.set_ui_scale(-1),
                                          tip="Smaller icons and texts in the 3D view")
        self.bt_ui_plus = uc.tool_button("+", lambda: self.set_ui_scale(1), tip="Larger icons and texts in the 3D view")
        self.lb_ui = QLabel()
        self.bt_recentre = uc.tool_button("Recentre", self.recentre, "SP_DialogResetButton",
                                          tip="Back to the view's centre (the ship, the central body or the body "
                                              "double-clicked) and level with the reference plane (ICRF XY, ship "
                                              "view: the ship's XY plane); zoom and direction of view kept")

        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(400)
        sv.lock_pivot(self.plotter)
        # right drag: virtual trackball (roll outside its circle); the left button picks and centres (user,
        # 29 October 2026)
        sv.install_arcball(self.plotter, ARCBALL_RADIUS, button="right")
        self.plotter.iren.interactor.AddObserver("LeftButtonPressEvent", self._on_click, 1.0)
        # the manoeuvre node (user, 29 October 2026): its own presses before the click above
        from gui.maneuver_node import NodeTool
        self.node = NodeTool(self)
        self.node.install(self.plotter.iren.interactor)
        self.plotter.set_background(BG_COLOR)
        self.plotter._ui_scale = self.ui_scale
        self.sky = gsky.Sky(self.plotter, BG_COLOR)        # the stars under the scene (their own renderer)
        self.sky.set_visible(self.tg_sky.isChecked(), self.cb_const.isChecked())   # plain at start (user)
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
        vcol.addWidget(_row(QLabel("UI size"), self.bt_ui_minus, self.lb_ui, self.bt_ui_plus))
        self._show_ui_scale()
        for title, ws in ((_title("Layers"), self._common), (self._ship_titles[0], self._ship_only)):
            vcol.addWidget(title)
            grid = QGridLayout()                       # two columns per section (user, 29 October 2026)
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setVerticalSpacing(4)
            for k, w in enumerate(ws):
                grid.addWidget(w, k // OPTION_COLUMNS, k % OPTION_COLUMNS)
            vcol.addLayout(grid)

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
        self.info = uc.InfoTable()                                # label and value columns, the window's font
        # one piece of information per line, no wrapping and no horizontal scroll (user, 29 October 2026): the
        # column's width at start fits the longest line (side_width)
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
        self._cpa_cache = {}                          # the closest approaches of the run shown
        # global view: the paths through every n-th truth record, n fixed for the run (no shimmer while playing)
        self._path_stride = max(1, math.ceil(len(truth.t_s) / PATH_MAX_PTS))
        self._coast_third = None                      # the other bodies' grid of the orbital vectors (_coast)
        self.progress.setValue(1000)
        self.status.setText(f"{label}: {len(g.ids)} objects, {float(sc['meta']['duration_s']):g} s")
        self.t = min(self.t, self.duration())
        rng = g.range_km[np.isfinite(g.range_km)]
        self._r_far = max(float(rng.max()), 1e-3) if rng.size else 10.0
        self._navigation = engine.Navigation(sc, truth)
        self.node.invalidate()                        # the node's preview from the new run
        self._triad_rgb = np.array([_rgb(cgeo.COLORS[k]) for k in ("axis_x", "axis_y", "axis_z")], np.uint8)
        self._colour_rows()
        self._fill_lists()
        self._fill_events()
        self._enable_playback(True)
        self._rebuild()
        self.run_shown.emit()

    # ------------------------------------------------------------------ time
    def duration(self) -> float:
        """The run's length: the scenario's, or up to the ship's impact when its manoeuvres crash it (the run ends
        there, user 29 October 2026)."""
        if not self.sc:
            return 0.0
        d = float(self.sc["meta"]["duration_s"])
        return min(d, float(self.truth.t_s[-1])) if self.truth is not None and len(self.truth.t_s) else d

    def scenario_duration(self) -> float:
        """The scenario's length (the manoeuvre node's paths: beyond an impact the edit may avoid)."""
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
        for o in [dict(sc.get("ship", {}), id="ship")] + sc.get("objects", []) + sc.get("groups", []):
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
        """B4 / B14: the ship view's own switches (and their titles) shown in the ship view only; hidden, they keep
        their room, so the "3D view" box keeps the height of the taller of its two states (user, 29 October 2026)."""
        for w in self._ship_only + self._ship_titles:
            sp = w.sizePolicy()
            sp.setRetainSizeWhenHidden(True)
            w.setSizePolicy(sp)
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
        self._actor_cache = {}
        body = tr.body
        for b in ("earth", "moon"):                    # both bodies in every scenario, placed at each frame
            rad = float(scn.bodies()[b]["radius_km"])
            tex = _texture(b)
            p.add_mesh(_body_mesh(rad), texture=tex, color=None if tex is not None else EARTH_COLOR,
                       smooth_shading=True, ambient=BODY_AMBIENT, culling="back", name=f"body_{b}", render=False)
        _add_atmosphere(p, "body_earth_atm")
        tex = _sun_texture()                           # unit sphere, scaled and placed by _place_bodies
        p.add_mesh(_body_mesh(1.0), texture=tex, color=None if tex is not None else SUN_COLOR, lighting=False,
                   culling="back", name="body_sun", render=False)
        _add_sun_halo(p, "body_sun_halo")
        _add_sun_limb(p, "body_sun_limb")
        for name in PLANET_NAMES.values():             # unit spheres, scaled and placed by _place_bodies, lit by
            tex = _texture(name)                        # the Sun (its point light)
            p.add_mesh(_body_mesh(1.0), texture=tex, color=None if tex is not None else PLANET_COLORS[name],
                       smooth_shading=True, ambient=BODY_AMBIENT, culling="back", name=f"body_{name}", render=False)
        tex = _rings_texture()
        p.add_mesh(_rings_mesh(), texture=tex, color=None if tex is not None else PLANET_COLORS["saturn"],
                   lighting=False, opacity=1.0 if tex is not None else 0.5, name="body_saturn_rings", render=False)
        for name in MOON_DATA:                          # the moons: unit spheres, Phobos its shape (km), lit
            self._add_moon_mesh(p, name, f"body_{name}", lit=True)
        # the Sun the only light (user, 30 October 2026): a point light at its centre, no fall-off, so that every
        # body is lit from its own direction to the Sun (a light of parallel rays from its direction seen from the
        # central body lit Jupiter or Phobos from a wrong side); no headlight: the night sides dark but for the
        # bodies' small ambient part (BODY_AMBIENT)
        self._sun_light = pv.Light(light_type="scene light", intensity=1.0)
        self._sun_light.positional = True
        self._sun_light.cone_angle = 180.0                  # (a point light, not a spot)
        self._sun_light.attenuation_values = (1.0, 0.0, 0.0)
        p.add_light(self._sun_light)
        times, pts, labels, rows = [], [], [], []
        for t, text in self._marker_events():         # the events' places, drawn at each frame (_draw_markers)
            r = sgeo.states_at(tr, t)[0]
            oid = text.split()[0]
            if oid in tr.ids and np.isfinite(r[tr.row(oid), 0]):
                times.append(t)
                pts.append(r[tr.row(oid)])
                labels.append(text)
                rows.append(tr.row(oid))
        self._markers = (np.array(times, float), np.array(pts, float).reshape(-1, 3), labels, np.array(rows, int))
        extent = max(float(np.nanmax(np.abs(tr.r_km))) if np.isfinite(tr.r_km).any() else 0.0, 2.0 * body.radius_km)
        self._home = (-extent, extent, -extent, extent, -extent, extent)
        p.reset_camera(render=False, bounds=self._home)
        p._pivot = (0.0, 0.0, 0.0)
        self._centre_pending = True

    def _body_orbits(self) -> None:
        """Global view: the celestial body selected (user, 29 October 2026): its orbit around its parent (the Moon
        around the Earth, the Earth around the Sun; none for the Sun) and those of the bodies orbiting it, each over
        one period centred on t, about its parent's place at t; the scenario's central body's orbit always (user,
        29 October 2026: the Moon's in a lunar scenario). Each orbit its own actor (the GPU draws in single precision
        about an actor's centre: one actor for all, Phobos' orbit at 2e8 km from the scene's centre was drawn in
        16 km steps), the BODY_ORBIT_POINTS of the period outside a window about t, and BODY_ORBIT_NEAR points
        computed again at each frame inside it, an actor of their own (the chords of a period in 360 left the true
        path by up to 8700 km for Mars: the body's own place lies on its line, whatever the zoom)."""
        sel = self.selected or ""
        central = self.truth.body.name
        shown = [central] if central in BODY_PARENT else []
        if self.cb_all_orbits.isChecked():               # every body's (user, 30 October 2026)
            shown += [b for b in BODY_PARENT if b not in shown]
        if sel.startswith(BODY_SEL):
            key = BODY_NAMES.get(sel[len(BODY_SEL):])
            for b in ([key] if key in BODY_PARENT else []) + [b for b, (par, _p) in BODY_PARENT.items() if par == key]:
                if b not in shown:
                    shown.append(b)
        if not shown:
            return
        now = self._bodies_at()
        for b in shown:
            parent, period = BODY_PARENT[b]
            ts, rel = self._orbit_shape(b)                   # the body about its parent over the period
            step = period / BODY_ORBIT_POINTS
            nc = getattr(self, "_near_cache", {})
            self._near_cache = nc
            hit = nc.get(b)
            if (hit is not None and hit[0] == id(self.truth) and hit[1] is ts and hit[2][0] + step <= self.t
                    <= hit[2][-1] - step):                   # the body still well inside it: kept (all orbits:
                near_t, near, t_lo, t_hi = hit[2], hit[3], hit[2][0], hit[2][-1]    # 51 at each frame, user)
            else:
                lo, hi = self.t - 2.0 * step, self.t + 2.0 * step
                t_lo = ts[ts < lo][-1] if (ts < lo).any() else lo   # (from the coarse points beside it)
                t_hi = ts[ts > hi][0] if (ts > hi).any() else hi
                near_t = np.linspace(t_lo, t_hi, BODY_ORBIT_NEAR)
                near = np.array([self._orbit_rel(b, float(t)) for t in near_t])
                nc[b] = (id(self.truth), ts, near_t, near)
            before, after = ts < t_lo + 1e-9, ts > t_hi - 1e-9
            pieces = [rel[before], rel[after]]
            pts = [p_ for p_ in pieces if len(p_) >= 1]
            if before.any():
                pts[0] = np.vstack([pts[0], near[:1]])
            if after.any():
                pts[-1] = np.vstack([near[-1:], pts[-1]])
            coarse = [p_ for p_ in pts if len(p_) >= 2]
            if coarse:
                off = np.cumsum([0] + [len(p_) for p_ in coarse])
                lines = np.hstack([np.r_[len(p_), off[k] + np.arange(len(p_))] for k, p_ in enumerate(coarse)])
                self._poly(f"{DYN}orbit_{b}", now[parent] + np.vstack(coarse), lines=lines, color=_body_color(b),
                           line_width=1, opacity=0.7)                # (its body's dominant colour, user)
            self._poly(f"{DYN}orbit_{b}_near", now[parent] + near, lines=np.r_[len(near), np.arange(len(near))],
                       color=_body_color(b), line_width=1, opacity=0.7)

    def _orbit_rel(self, b: str, t: float) -> np.ndarray:
        """The body b about its parent at t (km, ICRF axes)."""
        T = eph.centuries_tt(scn.epoch(self.sc), t)
        parent = BODY_PARENT[b][0]
        if b in MOON_DATA:
            return eph.satellite_planetocentric_km(b, T)
        if b in PLANET_DATA:
            return eph.planet_helio_icrf_km(b, T)
        if b == "earth":
            return -eph.sun_icrf_km(T)
        if b == "moon" and parent == "earth":
            return eph.moon_icrf_km(T)
        raise KeyError(b)

    def _orbit_shape(self, b: str) -> Tuple[np.ndarray, np.ndarray]:
        """(times, the body b about its parent over one period centred on t, km), computed again when t has moved a
        twentieth of the period (cached: the ephemeris at every frame would slow the playback)."""
        parent, period = BODY_PARENT[b]
        cache = getattr(self, "_orbit_cache", {})
        self._orbit_cache = cache
        hit = cache.get(b)
        if hit is not None and hit[0] == id(self.truth) and abs(hit[1] - self.t) < period / 20.0:
            return hit[2], hit[3]
        ts = self.t + np.linspace(-0.5, 0.5, BODY_ORBIT_POINTS) * period
        out = np.array([self._orbit_rel(b, float(t)) for t in ts])
        cache[b] = (id(self.truth), self.t, ts, out)
        return ts, out

    def _place_bodies(self) -> None:
        """V3: the Earth turned by its sidereal angle (pole on the ICRF Z axis as in the propagation, P2), the Moon
        with its near side toward the Earth (synchronous rotation, IAU pole), at their true places at t."""
        pos_b = self._bodies_at()
        moon_geo = pos_b["moon"] - pos_b["earth"]
        for b, pos, m in (("earth", pos_b["earth"], _earth_matrix(scn.epoch(self.sc), self.t)),
                          ("moon", pos_b["moon"], _moon_matrix(moon_geo)),
                          ("sun", pos_b["sun"], eph_sun_radius_km() * _sun_matrix(scn.epoch(self.sc), self.t))):
            a = self._actor(f"body_{b}")
            if a is not None:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = m, pos
                a.user_matrix = u
            if b == "earth" and self._actor("body_earth_atm") is not None:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = float(scn.bodies()["earth"]["radius_km"]) * np.eye(3), pos
                self._actor("body_earth_atm").user_matrix = u
        halo = self._actor("body_sun_halo")
        if halo is not None:
            _place_halo(self.plotter, halo, pos_b["sun"], eph_sun_radius_km())
        limb = self._actor("body_sun_limb")
        if limb is not None:
            self._limbs = [(limb, pos_b["sun"], eph_sun_radius_km())]
            _place_limb(self.plotter, limb, pos_b["sun"], eph_sun_radius_km())
        ep = scn.epoch(self.sc)
        drawn = {}
        for name in PLANET_NAMES.values():
            rad = float(PLANET_DATA[name]["radius_km"])
            m = _iau_matrix(PLANET_DATA[name], ep, self.t)
            for actor, k in ((f"body_{name}", rad), ("body_saturn_rings", 1.0) if name == "saturn" else (None, 0.0)):
                a = self._actor(actor) if actor else None
                if a is not None:
                    u = np.eye(4)
                    u[:3, :3], u[:3, 3] = k * m, pos_b[name]
                    a.user_matrix = u
            drawn[name] = (pos_b[name], rad)
        for name in ("sun", "earth", "moon"):          # a dot too when too small (user, 29 October 2026)
            drawn[name] = (pos_b[name], self._body_radius(name))
        for name in MOON_DATA:                          # the major moons
            a = self._actor(f"body_{name}")
            if a is not None:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = self._moon_matrix_of(name, True), pos_b[name]
                a.user_matrix = u
            drawn[name] = (pos_b[name], self._body_radius(name))
        self._planets_drawn = drawn
        self._planet_marks()
        light = getattr(self, "_sun_light", None)
        if light is not None:                          # the Sun's point light at its place (around it: at 0)
            light.position = tuple(pos_b["sun"])
            light.focal_point = tuple(pos_b["sun"] + np.array([0.0, 0.0, -1.0]))

    def _bodies_at(self) -> Dict[str, np.ndarray]:
        """Earth, Moon and Sun at t, km from the central body (true places)."""
        T = eph.centuries_tt(scn.epoch(self.sc), self.t)
        moon, sun = eph.moon_icrf_km(T), eph.sun_icrf_km(T)
        geo_ = {"earth": np.zeros(3), "moon": moon, "sun": sun}
        geo_.update({p: eph.planet_icrf_km(p, T) for p in PLANET_NAMES.values()})
        geo_.update({m: geo_[MOON_DATA[m]["planet"]] + eph.satellite_planetocentric_km(m, T) for m in MOON_DATA})
        centre = geo_[self.truth.body.name]          # (any body as the central one, user 30 October 2026)
        return {b: x - centre for b, x in geo_.items()}

    def _actor(self, name: str):
        """An actor by its name (the dynamic ones, else the bodies', kept: plotter.actors rebuilds its whole dict at
        each access, too slow for the fifty celestial bodies at every frame)."""
        e = self._dyn.get(name)
        if e is not None:
            return e[2]
        cache = self.__dict__.setdefault("_actor_cache", {})
        if name not in cache:
            cache[name] = self.plotter.actors.get(name)
        return cache[name]

    def _add_moon_mesh(self, p, name: str, actor: str, lit: bool = False):
        """A moon's actor: its shape model (km, placed by its matrix alone), else a unit sphere (scaled by its
        radius), in its colour; its map put on when it is first drawn as a sphere (_map_on: the maps are large);
        lit: by the scene's lights (the global view: the Sun), else unlit (the ship view, unchanged: user)."""
        mesh = _shape_mesh(name)
        sphere = _body_mesh(1.0, 180, 90, *MAP_LON.get(name, (-180.0, 1)))
        a = p.add_mesh(mesh if mesh is not None else sphere, color=MOON_COLORS.get(name, MOON_COLOR),
                       lighting=lit, smooth_shading=lit, ambient=BODY_AMBIENT if lit else 0.0,
                       culling=None if mesh is not None else "back", name=actor, render=False)
        if _has_texture(name):
            self.__dict__.setdefault("_maps_pending", {})[actor] = name
        return a

    def _map_on(self, actor_name: str, a) -> None:
        """A moon's map, read and put on its actor the first time it is drawn as a sphere."""
        pending = getattr(self, "_maps_pending", {})
        name = pending.pop(actor_name, None)
        if name is None or a is None:
            return
        tex = _texture(name)
        if tex is not None:
            a.texture = tex
            a.GetProperty().SetColor(1.0, 1.0, 1.0)

    def _moon_matrix_of(self, name: str, scaled: bool) -> np.ndarray:
        """A moon's body-fixed -> ICRF matrix (locked on its planet), times its radius for a sphere."""
        T = eph.centuries_tt(scn.epoch(self.sc), self.t)
        rel = eph.satellite_planetocentric_km(name, T)
        vel = eph.satellite_planetocentric_km(name, T + 60.0 / 86400.0 / 36525.0) - rel
        m = _synchronous_matrix(rel, vel)
        shape = _shape_mesh(name) is not None
        return m if (shape or not scaled) else float(MOON_DATA[name]["radius_km"]) * m

    def _body_radius(self, b: str) -> float:
        if b in MOON_DATA:
            return float(MOON_DATA[b]["radius_km"])
        if b in PLANET_DATA:
            return float(PLANET_DATA[b]["radius_km"])
        return eph_sun_radius_km() if b == "sun" else float(scn.bodies()[b]["radius_km"])

    def _planet_marks(self) -> set:
        """The celestial bodies as drawn (_planets_drawn: centre, radius, in the view's axes; the planets, the Sun,
        the Earth, the Moon): those under PLANET_MARK_PX of radius on screen hidden and drawn as a dot in their
        colour with their name (user, 29 October 2026), the others as their textured sphere (and Saturn's rings, the
        Earth's atmosphere, the Sun's halo). Each frame and each camera change; the set of the bodies marked."""
        drawn = getattr(self, "_planets_drawn", {})
        cam = self.plotter.camera
        pos = np.asarray(cam.GetPosition())
        h_px = max(self.plotter.ren_win.GetSize()[1], 1)
        k = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / h_px          # world per px at unit distance
        fwd = np.asarray(cam.GetDirectionOfProjection())
        up = np.asarray(cam.GetViewUp(), float)
        up = up - (up @ fwd) * fwd
        up /= max(float(np.linalg.norm(up)), 1e-12)
        right = np.cross(fwd, up)
        dist = cam.GetDistance()
        dirs, rgb, lab_p, lab_t, marked = [], [], [], [], set()
        prefix = "" if self._global() else f"{DYN}sbody_"
        extra = {"saturn": ("saturn_rings",), "earth": ("earth_atm",), "sun": ("sun_halo", "sun_limb")}
        scr = {}
        for name, (c, rad) in drawn.items():
            rel = np.asarray(c, float) - pos
            d = float(np.linalg.norm(rel))
            scr[name] = (rel / max(d, 1e-300), d)
        for name, (c, rad) in drawn.items():
            rel = np.asarray(c, float) - pos
            d = float(np.linalg.norm(rel))
            small = d > rad and rad / max(d * k, 1e-300) < PLANET_MARK_PX      # (inside it: its sphere)
            if small and name in MOON_DATA and MOON_DATA[name]["planet"] in scr:
                up_, dp = scr[MOON_DATA[name]["planet"]]   # on its planet's dot: neither drawn nor named (user)
                ang = math.acos(float(np.clip(up_ @ (rel / d), -1.0, 1.0)))
                if ang / k < MOON_PART_PX:
                    for actor in (f"{prefix}{name}" if prefix else f"body_{name}",):
                        a = self._actor(actor)
                        if a is not None:
                            a.SetVisibility(False)
                    marked.add(name)
                    continue
            for part in (name,) + tuple(x.split("_", 1)[1] for x in extra.get(name, ())):
                key = name if part == name else f"{name}_{part}"
                actor_name = f"{prefix}{key}" if prefix else f"body_{key}"
                a = self._actor(actor_name)
                if a is not None:
                    a.SetVisibility(not small)
                    if not small:
                        self._map_on(actor_name, a)
            if small:
                marked.add(name)
                u = rel / d
                dirs.append(u)
                rgb.append(_rgb(PLANET_COLORS[name]))
                lab_p.append(pos + dist * u + 6.0 * k * dist * right)     # its name beside the dot
                lab_t.append(next(n for n, key in BODY_NAMES.items() if key == name))
        self.sky.set_dots(np.array(dirs).reshape(-1, 3), np.array(rgb, np.uint8).reshape(-1, 3), PLANET_DOT_PX)
        if lab_t and self.cb_labels.isChecked():        # (the names follow "labels", user 30 October 2026)
            self._labels(f"{DYN}planet_lbl", np.array(lab_p), lab_t, MUTED_COLOR, DATA_FONT)
        else:
            lab = self._lab.get(f"{DYN}planet_lbl")
            if lab is not None:
                lab.actor.SetVisibility(False)
        return marked

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
        then reached in every direction, user 29 October 2026) stands for MAX_VIEW_KM (km)."""
        if self.truth is None:
            return None
        cam = self.plotter.camera
        w, h = self.plotter.ren_win.GetSize()
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) * min(w / max(h, 1), 1.0)
        return MAX_VIEW_KM / k if k > 0 else None

    def _min_camera_distance(self) -> Optional[float]:
        """Camera distance at which the innermost range ring (RING_STEP_FRAC of the half-height) stands for
        MIN_RING_KM (km)."""
        if self.truth is None:
            return None
        cam = self.plotter.camera
        w, h = self.plotter.ren_win.GetSize()
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) * min(w / max(h, 1), 1.0) * RING_STEP_FRAC
        return MIN_RING_KM / k if k > 0 else None

    def _on_camera(self) -> None:
        """Every camera change (wheel, buttons, follow): the zoom out held at MAX_VIEW_KM, then the range rings."""
        if getattr(self, "_in_camera", False):
            return
        self._in_camera = True
        try:
            cam = self.plotter.camera
            d_max, d_min = self._max_camera_distance(), self._min_camera_distance()
            d = cam.GetDistance()
            target = d_max if (d_max is not None and d > d_max) else d_min if (d_min is not None and d < d_min) else None
            if target is not None:
                f = np.asarray(cam.GetFocalPoint())
                u = np.asarray(cam.GetPosition()) - f
                cam.SetPosition(*(f + u / np.linalg.norm(u) * target))
                self.plotter.renderer.ResetCameraClippingRange()
        finally:
            self._in_camera = False
        self._place_symbols()                         # SYMBOL_PX on screen whatever the camera does
        self._place_scaled()                          # the axes and noses too (user)
        self.node.place()                             # the manoeuvre node too
        for act, c, rad in getattr(self, "_limbs", []):   # the Sun's limb disc in front of it, from the camera
            if act.GetVisibility():
                _place_limb(self.plotter, act, c, rad)
        if self.truth is not None:
            self._planet_marks()                      # (a planet's dot or sphere, as the zoom makes it)
        self._update_rings()
        mt = getattr(self, "_mt_input", None)         # the multitracks formed again when the zoom changes them
        if mt is not None and self.truth is not None and not self._playing:
            if self._mt_signature(mt[0], self._multitracks(*mt)) != getattr(self, "_mt_sig", ()):
                self._redraw_soon()

    def _redraw_soon(self) -> None:
        """One frame drawn after the camera event (not within it)."""
        if not getattr(self, "_redraw_pending", False):
            self._redraw_pending = True

            def go():
                self._redraw_pending = False
                if self.truth is not None and not self._playing and not getattr(self.plotter, "_closed", False):
                    self._draw_frame()                # (not once the view is closed: its window is gone)
            QTimer.singleShot(0, go)

    def edit_manoeuvre(self, row_id: Optional[str], owner: Optional[dict] = None, m: Optional[dict] = None,
                       on_edit: Optional[Callable[[bool], None]] = None,
                       on_release: Optional[Callable[[], None]] = None) -> None:
        """The manoeuvre chosen in the editor (gui/maneuver_node.py): its node on the path of the body row_id;
        on_release() when a click in the view lets it go; None: no node."""
        self.node.set_target(row_id, owner, m, on_edit, on_release)
        if self.truth is not None:
            self._draw_frame()

    def manoeuvre_values_changed(self) -> None:
        """The editor's box changed the manoeuvre's values: the node and the planned trajectory follow at once."""
        self.node.invalidate()
        if self.truth is not None and self.node.row() is not None:
            self._draw_frame()

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

    def _on_vessel(self) -> bool:
        """The view is centred on a vessel (OWNSHIP or an object; the ship view's default centre is the ship), not
        on the central body, the Earth, the Moon or the Sun."""
        if self.centre in ("", CENTRAL):
            return not self._global()
        return self.centre not in BODY_NAMES

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
            else:
                dist = max(1.0, 0.3 * float(np.linalg.norm(tgt))) if np.linalg.norm(tgt) > 0 else 0.3 * self._r_far
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
        for o in [dict(self.sc.get("ship", {}), id="ship")] + self.sc.get("objects", []):
            for m in o.get("manoeuvres", []):
                if m["type"] in ("impulse", "burn"):
                    out.append((float(m["t_s"]), f"{o['id']} {m['type']}"))
        return out

    def _build_ship(self) -> None:
        """V4: the ship model at its true size in km (the ship's axes are the dynamic ones); no equator (a ring
        without a range)."""
        p = self.plotter
        acts = []
        ship = cgeo.build_ship()
        for i, part in enumerate(ship["hull"]):
            acts.append(p.add_mesh(part["mesh"], color=part["color"], smooth_shading=True, name=f"model_hull_{i}",
                                   render=False))
        for d in ship["arrays"]:
            acts.append(p.add_mesh(d["mesh"], color=d["color"], name=f"model_array_{d['k']}", render=False))
        for a in acts:                                   # the model's scene units to km
            a.SetScale(KM_PER_MODEL_UNIT, KM_PER_MODEL_UNIT, KM_PER_MODEL_UNIT)
        self._model = {"actors": acts}
        self._show_faces()
        p.set_background(BG_COLOR)
        p.enable_lightkit()
        self._centre_pending = True
        self._ship_home()

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
        """Mission volumes: their turn (frame at t against the one they were built with); their labels with them."""
        vol = self._vol
        if not vol:
            return
        pts = []
        for rot, acts, c in zip(vol.get("rot") or [np.eye(3)] * len(vol["per_sector"]), vol["per_sector"],
                                vol["centres"]):
            u = np.eye(4)
            u[:3, :3] = rot
            for a in acts:
                a.user_matrix = u
            pts.append(rot @ c)
        lab = vol.get("labels")
        if lab is not None:
            if pts and self.cb_labels.isChecked():
                lab.set(np.array(pts), vol["texts"])
            else:
                lab.actor.SetVisibility(False)

    def _ship_home(self) -> None:
        """Default camera of the ship view: the iso view, out to the farthest object of the run (km)."""
        p = self.plotter
        sv.set_default_camera(p)
        k = self._r_far / 1.2
        # the camera aims at the ship's centre, the centre of the range rings and the pivot of the rotations (the
        # mission-preparation default aims 0.12 above it: the rings moved as the view turned, user 29 October 2026)
        pos = (np.asarray(p.camera.position) - np.asarray(p.camera.focal_point)) * k
        if self.truth is not None:
            # the default camera on the side away from the central body (else it may stand inside it: celestial
            # objects drawn)
            r, _ = sgeo.states_at(self.truth, self.t)
            dc = -(r[0] / np.linalg.norm(r[0])) @ self.truth.attitude("ship", self.t)
            if pos @ dc > 0.0:
                pos = pos - 2.0 * (pos @ dc) * dc
        p.camera.focal_point = (0.0, 0.0, 0.0)
        p.camera.position = tuple(pos)
        p._pivot = (0.0, 0.0, 0.0)
        p.renderer.ResetCameraClippingRange()

    def recentre(self) -> None:
        """Back to the view's centre (button "Recentre"): the body double-clicked, else the ship (ship view)
        or the central body (global view), the zoom and the direction of view kept (undoes a pan); and the view
        realigned with the reference plane, the equatorial plane of the central body (user, 29 October 2026): its
        roll undone, the body's north pole up on screen (ship view: that pole in the ship's frame at t)."""
        if self.truth is None:
            return
        tgt = self._target()
        tgt = np.zeros(3) if tgt is None else np.asarray(tgt, float)
        cam = self.plotter.camera
        off = tgt - np.asarray(cam.GetFocalPoint())
        cam.SetFocalPoint(*tgt)
        cam.SetPosition(*(np.asarray(cam.GetPosition()) + off))
        pole = np.asarray(self.truth.body.pole, float)               # ICRF
        if not self._global():
            pole = pole @ self.truth.attitude("ship", self.t)         # in the ship's frame
        fwd = np.asarray(cam.GetDirectionOfProjection(), float)
        if self._on_vessel():
            # centred on a vessel (user, 29 October 2026): the camera parallel to the equatorial plane of the body
            # it orbits, looking level at it (the heading kept), the pole up
            flat = fwd - (fwd @ pole) * pole
            if np.linalg.norm(flat) < 1e-6:                          # looking along the pole: any level heading
                flat = np.cross(pole, [1.0, 0.0, 0.0] if abs(pole[0]) < 0.9 else [0.0, 1.0, 0.0])
            fwd = flat / np.linalg.norm(flat)
            cam.SetPosition(*(tgt - cam.GetDistance() * fwd))
        up = pole - (pole @ fwd) * fwd                               # the pole, square to the line of sight
        if np.linalg.norm(up) < 1e-6:                                # looking along the normal: keep the heading
            up = np.asarray(cam.GetViewUp(), float)
        cam.SetViewUp(*(up / np.linalg.norm(up)))
        self.plotter._pivot = tuple(tgt)
        self.plotter.renderer.ResetCameraClippingRange()
        self._render()

    @staticmethod
    def _ship_range(r: float) -> Optional[float]:
        """Range (km) drawn at radius r in the ship view (1:1)."""
        return r if r > 0 else None

    def _range_projection(self, pos: np.ndarray, rgb: np.ndarray) -> None:
        """Option "range projection" (user, 29 October 2026): each object copied into the plane of the range rings at
        its drawn range from the ship (the nearest point of the ring of that radius: its direction projected on the
        plane), linked to the object by the arc of the sphere of that radius around the ship (every point of it at
        the same range)."""
        rad = np.linalg.norm(pos, axis=1)
        u = pos / np.maximum(rad, 1e-300)[:, None]
        proj = self._to_plane(pos)
        p = proj / np.maximum(rad, 1e-300)[:, None]
        self._poly(f"{DYN}proj_pts", proj, rgb=rgb, point_size=7, render_points_as_spheres=True, opacity=0.8)
        th = np.arccos(np.clip(np.sum(u * p, axis=1), -1.0, 1.0))
        # dashed (user, 29 October 2026: not to be taken for a trajectory or a trail): k dashes per arc, each over
        # the first half of its k-th of the arc, 3 points along it
        k = np.clip(np.ceil(np.degrees(th) / PROJ_DASH_DEG), *PROJ_DASHES).astype(int)
        arc_of = np.repeat(np.arange(len(pos)), k)                           # the arc of each dash
        i = np.arange(len(arc_of)) - np.repeat(np.cumsum(k) - k, k)          # its rank on the arc
        s = (i[:, None] + np.array([0.0, 0.25, 0.5])) / k[arc_of][:, None]   # (dashes, 3), 0 at the object
        t = th[arc_of][:, None]
        sin_t = np.maximum(np.sin(t), 1e-12)
        small = t < 1e-6
        a = np.where(small, 1.0 - s, np.sin((1.0 - s) * t) / sin_t)         # slerp from u (s = 0) to p (s = 1)
        b = np.where(small, s, np.sin(s * t) / sin_t)
        arc = (a[..., None] * u[arc_of][:, None, :] + b[..., None] * p[arc_of][:, None, :]) * rad[arc_of][:, None, None]
        nd = len(arc_of)
        lines = np.column_stack([np.full(nd, 3), 3 * np.arange(nd)[:, None] + np.arange(3)]).ravel()
        self._poly(f"{DYN}proj_arcs", arc.reshape(-1, 3), lines=lines, rgb=np.repeat(rgb[arc_of], 3, axis=0),
                   line_width=1.5, opacity=0.8)

    def _to_plane(self, pts: np.ndarray) -> np.ndarray:
        """Points of the ship view moved into the plane of the range rings at the same drawn range (the nearest
        point of the ring of that radius: the direction projected on the plane; on the plane's axis, toward e1)."""
        _h, e1, e2, _c = self._ring_frame()
        nrm = np.cross(e1, e2)
        pts = np.asarray(pts, float)
        rad = np.linalg.norm(pts, axis=-1)
        u = pts / np.maximum(rad, 1e-300)[..., None]
        p = u - (u @ nrm)[..., None] * nrm
        pn = np.linalg.norm(p, axis=-1)
        p = np.where((pn > 1e-9)[..., None], p / np.maximum(pn, 1e-300)[..., None], e1)
        return p * rad[..., None]

    def _full_projection(self) -> bool:
        """Ship view, range projection "full": every body and path drawn in the plane of the rings."""
        return not self._global() and self.cb_proj.isChecked() and self.tg_proj_full.isChecked()

    def _truth_lines(self, name: str, rows, rgb: np.ndarray, t0: float, t1: float, **style) -> None:
        """Global view: the rows' paths over [t0, t1] in their colours, through the truth records on the run's grid
        (_path_stride) between the interpolated states at t0 and t1; not drawn when empty. rows: indices, or groups
        of indices (a multitrack: one path, its members' mean)."""
        if t1 <= t0 or not len(rows):
            return
        tr = self.truth
        groups = _as_groups(rows)
        ks = np.arange(0, len(tr.t_s), self._path_stride)
        ks = ks[(tr.t_s[ks] > t0) & (tr.t_s[ks] < t1)]
        pts = _group_mean(np.concatenate([sgeo.states_at(tr, t0)[0][None], tr.r_km[ks],
                                          sgeo.states_at(tr, t1)[0][None]]), groups)
        pl = _polylines(pts, np.isfinite(pts[..., 0]))
        if pl is not None:
            pts, lines, c, keep = pl
            self._poly(name, pts, lines=lines, rgb=np.repeat(rgb[keep], c, axis=0), **style)

    def _coast(self, r: np.ndarray, v: np.ndarray) -> np.ndarray:
        """The free fall of the states r, v (n, 3) at t over the orbital vector's duration (prop.coast: the run's
        gravity, no burn, no guidance), positions (n_pts, n, 3) from the central body."""
        dur = self.sp_orbit.value()
        need = self.t + dur
        third = getattr(self, "_coast_third", None)
        if prop.THIRD_BODY and (third is None or third[0] < need):            # the bodies' grid over the fall
            span = max(need, 2.0 * self.duration())
            self._coast_third = third = (span, prop.ThirdBodies(self.truth.body.name, scn.epoch(self.sc), span))
        n = int(np.clip(math.ceil(dur / COAST_STEP_S), *COAST_STEPS))
        return prop.coast(self.truth.body, third[1] if third else None, r, v, self.t, dur, n)

    def _ship_coast(self, r: np.ndarray, v: np.ndarray, rs: np.ndarray, vs: np.ndarray) -> np.ndarray:
        """Ship view: the rows' free falls from t as the ship view draws a body (as "planned trajectory" does, user
        29 October 2026): relative to OWNSHIP's own free fall at the same instant, in the ship's frame at that instant
        (its attitude of the run, held at the run's end beyond it), at 1:1. Without a
        manoeuvre it lies on the planned trajectory."""
        path = self._coast(np.r_[r[:1], rs], np.r_[v[:1], vs])                    # (n_pts, 1 + n, 3)
        ts = self.t + np.linspace(0.0, self.sp_orbit.value(), len(path))
        ms = np.array([self.truth.attitude("ship", float(min(t, self.duration()))) for t in ts])
        rel = np.einsum("kni,kij->knj", path[:, 1:] - path[:, :1], ms)           # ship frame at each instant
        rng = np.linalg.norm(rel, axis=2)
        ok = np.isfinite(rng) & (rng > 0)
        rad = np.where(ok, rng, 1.0)
        return np.where(ok[..., None], rel / np.where(ok, rng, 1.0)[..., None] * rad[..., None], np.nan)

    def _coast_lines(self, path: np.ndarray, rgb: np.ndarray) -> None:
        if self._full_projection():
            path = self._to_plane(path)
        pl = _polylines(path, np.isfinite(path[..., 0]))
        if pl is not None:
            pts, lines, c, keep = pl
            self._poly(f"{DYN}vel", pts, lines=lines, rgb=np.repeat(rgb[keep], c, axis=0), line_width=2)

    def _ship_lines(self, name: str, js, rgb: np.ndarray, sel: np.ndarray, **style) -> None:
        """Ship view: the objects' (columns js of the geometry, or groups of columns: a multitrack's mean) paths
        through the records sel, in their colours."""
        g = self.g
        groups = _as_groups(js)
        if not groups:
            return
        cols = np.unique(np.concatenate(groups))
        where = {c: k for k, c in enumerate(cols)}
        rr = g.range_km[sel][:, cols]                                      # (n_t, n)
        ok = np.isfinite(rr)
        pts = g.dir_ship[sel][:, cols] * np.where(ok, rr, np.nan)[..., None]
        pts = _group_mean(pts, [np.array([where[c] for c in gr]) for gr in groups])
        pl = _polylines(np.where(np.isfinite(pts), pts, 0.0), np.isfinite(pts[..., 0]))
        if pl is not None:
            pts, lines, c, keep = pl
            if self._full_projection():
                pts = self._to_plane(pts)
            self._poly(name, pts, lines=lines, rgb=np.repeat(rgb[keep], c, axis=0), **style)

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
            ang = np.linspace(0.0, 2.0 * math.pi, RING_POINTS)
            pts, lines, lab_p, lab_t, seen = [], [], [], [], set()
            # always at the same place on screen (user, 29 October 2026): only the range they carry follows the zoom
            # (round steps in km)
            # the step kept while it stays within RING_HYST of the ideal one (user, 29 October 2026: a slight camera
            # move near a rounding threshold redrew every ring); only the outer rings come and go with the view
            keep = getattr(self, "_ring_keep", {})
            s1 = keep.get("true")
            if s1 is None or not 1.0 / RING_HYST <= s1 / step <= RING_HYST:
                s1 = _nice(step)
            keep["true"] = s1
            values = [k * s1 for k in range(1, min(RING_MAX, int(math.ceil(cover / s1))) + 1)]
            self._ring_keep = keep
            for nice in values:
                # (the zoom bounds hold MAX_VIEW_KM at the half-height: beyond, the corners keep their rings)
                if nice in seen or nice < MIN_RING_KM:
                    continue
                seen.add(nice)
                rad = nice
                off = len(pts) * RING_POINTS
                pts.append(rad * (np.cos(ang)[:, None] * e1 + np.sin(ang)[:, None] * e2))
                lines.append(np.r_[RING_POINTS, off + np.arange(RING_POINTS)])
                lab_p.append(rad * (RING_LABEL_C * e1 + RING_LABEL_S * e2))   # between two azimuth spokes
                lab_t.append(f"{nice:g} km")
            if pts:
                self._poly(f"{DYN}rings", np.vstack(pts), lines=np.hstack(lines), color="#4a5266", line_width=1)
                self._labels(f"{DYN}rings_lbl", np.array(lab_p), lab_t, MUTED_COLOR, 14)
            # azimuth spokes in the rings' plane: from the ship's centre out to the corners of the view every 45 deg,
            # phi from +X toward +Y (the ship frame's convention), labelled inside the view
            names = (f"{DYN}spokes", f"{DYN}spokes_lbl")
            az = np.radians(np.arange(0.0, 360.0, SPOKE_STEP_DEG))
            d = np.cos(az)[:, None] * e1 + np.sin(az)[:, None] * e2
            self._poly(names[0], np.r_[np.zeros((len(az), 3)), cover * d], lines=_pairs(len(az)),
                       color="#3a4256", line_width=1)
            self._labels(names[1], SPOKE_LABEL_FRAC * half * d,
                         [f"{a:g}°" for a in np.arange(0.0, 360.0, SPOKE_STEP_DEG)], MUTED_COLOR, 14)
        finally:
            self._in_rings = False

    # ------------------------------------------------------------------ dynamic layer (updated in place)
    def _reset_dynamic(self) -> None:
        """After plotter.clear(): forget the dynamic actors, labels and volumes (they were removed with it)."""
        self._dyn: Dict[str, list] = {}               # name -> [topology key, mesh, actor]
        self._limbs: list = []                        # (the Sun's limb disc, its centre, its radius) as placed
        self._lab: Dict[str, "_Labels"] = {}
        self._vol: Optional[dict] = None
        self._model: Optional[dict] = None
        self._used: set = set()
        self._sym_pts: Dict[str, np.ndarray] = {}     # the shown symbols' centres (_place_symbols)
        self._box_pts: Dict[str, np.ndarray] = {}     # the data boxes' bodies (_place_symbols)

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

    def _row_symbol(self, i: int) -> Optional[str]:
        """The symbol file of a row of the truth (0: OWNSHIP)."""
        if i == 0:
            return symbol_path("ownship", None)
        return symbol_path(self._row_ident[i], self.truth.classes[i])

    def _draw_markers(self) -> None:
        """Global view: the places of the events still to come (user, 29 October 2026: a past event's mark goes),
        each in its body's colour (its path's, its identity's), with its text at the UI size."""
        mk = getattr(self, "_markers", None)
        if mk is None or not len(mk[0]):
            return
        t, pts, labels, rows = mk
        keep = t >= self.t - 1e-9
        self._poly(f"{DYN}markers", pts[keep], rgb=self._row_rgb[rows[keep]], point_size=8,
                   render_points_as_spheres=True)
        self._labels(f"{DYN}markers_lbl", pts[keep], [x for x, k in zip(labels, keep) if k], MUTED_COLOR, 14)

    def _path_groups(self, rows: np.ndarray, clusters: List[dict]) -> Tuple[List[np.ndarray], np.ndarray]:
        """The rows as the paths draw them: each lone row, then each multitrack (its members, one path), with their
        colours."""
        lone = np.ones(len(rows), bool)
        for c in clusters:
            lone[c["k"]] = False
        groups = [np.array([i]) for i in rows[lone]] + [rows[c["k"]] for c in clusters]
        return groups, self._row_rgb[[g[0] for g in groups]].reshape(-1, 3)

    def _frame_clusters(self, rows: np.ndarray, pts: np.ndarray) -> List[dict]:
        return self._multitracks(rows, pts) if self.cb_symbols.isChecked() else []

    def _draw_bodies(self, rows: np.ndarray, pts: np.ndarray, r: np.ndarray, v: np.ndarray, dot_px: int,
                     clusters: Optional[List[dict]] = None) -> None:
        """The truth rows drawn at pts (the view's places): their symbols (or dots) and data boxes; with the
        symbols, the multitracks (user, 29 October 2026): one symbol, one box and their number for each group of
        objects of one identity and class within MULTITRACK_PX of one another on screen."""
        rows, pts = np.asarray(rows, int), np.asarray(pts, float)
        symbols = self.cb_symbols.isChecked()
        if clusters is None:
            clusters = self._frame_clusters(rows, pts)
        self._mt_input = (rows, pts) if symbols else None
        self._mt_sig = self._mt_signature(rows, clusters)
        lone = np.ones(len(rows), bool)
        for c in clusters:
            lone[c["k"]] = False
        tr = self.truth
        self._mt_of = {tr.ids[rows[k]]: c["q"] for c in clusters for k in c["k"]}
        self._mt_click = {MT_SEL + "|".join(tr.ids[rows[k]] for k in c["k"]): c["q"] for c in clusters}
        cq = np.array([c["q"] for c in clusters]).reshape(-1, 3)
        if symbols:
            self._symbols(np.r_[pts[lone], cq], [self._row_symbol(i) for i in rows[lone]] +
                          [symbol_path(c["ident"], c["cls"], multi=True) for c in clusters])
            self._mt_counts(clusters)
        else:
            self._poly(f"{DYN}pts", pts, rgb=self._row_rgb[rows], point_size=dot_px, render_points_as_spheres=True)
        if self.cb_labels.isChecked():
            self._databox(f"{DYN}labels", np.r_[pts[lone], cq],
                          self._box_texts(rows[lone], r, v) + [self._mt_box(c, rows, r, v) for c in clusters])

    def _display_xy(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Display pixels (n, 2) of world points (n, 3), and which lie in front of the camera."""
        ren = self.plotter.renderer
        w, h = self.plotter.ren_win.GetSize()
        m4 = ren.GetActiveCamera().GetCompositeProjectionTransformMatrix(ren.GetTiledAspectRatio(), -1.0, 1.0)
        mm = np.array([[m4.GetElement(i, j) for j in range(4)] for i in range(4)])
        p = np.c_[np.asarray(pts, float), np.ones(len(pts))] @ mm.T
        ok = p[:, 3] > 1e-12
        wq = np.where(ok, p[:, 3], 1.0)
        return np.c_[(p[:, 0] / wq + 1.0) * 0.5 * w, (p[:, 1] / wq + 1.0) * 0.5 * h], ok

    def _multitracks(self, rows: np.ndarray, pts: np.ndarray) -> List[dict]:
        """The groups (two or more) of objects of one identity and class linked on screen by distances under
        MULTITRACK_PX x the UI size (one to the next: a chain is one group); OWNSHIP never; none when the view
        shows less than MULTITRACK_OFF_KM in height at its focal point: k (indices into rows), q (their mean place),
        ident, cls."""
        cam = self.plotter.camera                     # zoomed in (the view's height at its focal point): none (user)
        if len(rows) < 2 or 2.0 * cam.GetDistance() * math.tan(math.radians(0.5 * cam.GetViewAngle())) < MULTITRACK_OFF_KM:
            return []
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        from scipy.spatial import cKDTree
        xy, ok = self._display_xy(pts)
        tr = self.truth
        keys: Dict[Tuple[str, str], List[int]] = {}
        for k, i in enumerate(rows):
            if i > 0 and ok[k]:
                keys.setdefault((self._row_ident[i], tr.classes[i]), []).append(k)
        out = []
        lim = MULTITRACK_PX * self.ui_scale
        for (ident, cls), ks in keys.items():
            if len(ks) < 2:
                continue
            ks = np.array(ks)
            pairs = cKDTree(xy[ks]).query_pairs(lim, output_type="ndarray")
            if not len(pairs):
                continue
            g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(ks), len(ks)))
            n, lab = connected_components(g, directed=False)
            for c in range(n):
                mem = ks[lab == c]
                if len(mem) >= 2:
                    out.append(dict(k=mem, q=pts[mem].mean(axis=0), ident=ident, cls=cls))
        return out

    @staticmethod
    def _mt_signature(rows: np.ndarray, clusters: List[dict]) -> tuple:
        return tuple(sorted(tuple(sorted(int(rows[k]) for k in c["k"])) for c in clusters))

    def _mt_below(self, q: np.ndarray) -> np.ndarray:
        """Under each multitrack's symbol (its number), at the UI size."""
        cam = self.plotter.camera
        fwd = np.asarray(cam.GetDirectionOfProjection())
        up = np.asarray(cam.GetViewUp(), float)
        up = up - (up @ fwd) * fwd
        up /= max(float(np.linalg.norm(up)), 1e-12)
        h_px = max(self.plotter.ren_win.GetSize()[1], 1)
        wpp = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / h_px
        depth = np.maximum((q - np.asarray(cam.GetPosition())) @ fwd, 1e-12)[:, None]
        return q - depth * wpp * (0.5 * SYMBOL_PX * self.ui_scale + MT_COUNT_GAP_PX) * up

    def _mt_counts(self, clusters: List[dict]) -> None:
        """The number of objects under each multitrack's symbol, in its frame's colour (one label set per
        identity); kept there by the camera changes (_place_symbols)."""
        self._mt_lbl = {}
        by: Dict[str, List[dict]] = {}
        for c in clusters:
            by.setdefault(c["ident"], []).append(c)
        for ident, cs in by.items():
            name = f"{DYN}mt_n_{ident}"
            q = np.array([c["q"] for c in cs])
            new = name not in self._lab
            self._labels(name, self._mt_below(q), [str(len(c["k"])) for c in cs],
                         IDENTITY_COLORS.get(ident, TEXT_COLOR), 15)
            if new:
                tp = self._lab[name].mapper.GetLabelTextProperty()
                tp.SetJustificationToCentered()
                tp.SetVerticalJustificationToTop()
                tp.BoldOn()
            self._mt_lbl[name] = q

    def _mt_box(self, c: dict, rows: np.ndarray, r: np.ndarray, v: np.ndarray) -> str:
        """A multitrack's data box: its number and class, its members' mean range and closing speed to OWNSHIP."""
        i = rows[c["k"]]
        rel, vrel = r[i] - r[0], v[i] - v[0]
        rng = np.linalg.norm(rel, axis=1)
        close = -1e3 * np.einsum("ij,ij->i", rel, vrel) / np.maximum(rng, 1e-12)
        return "\n".join([f"{len(i)} x {c['cls']}", f"range {_dist_text(float(rng.mean()))}",
                          f"Vc {float(close.mean()):+.0f} m/s"])

    def _symbols(self, pts: np.ndarray, paths: List[Optional[str]]) -> None:
        """The objects' symbols: square billboards facing the camera, SYMBOL_PX on screen, one textured mesh per
        symbol (all its objects at once: 1000+ objects); a row without a symbol file falls back on a white dot. Their
        centres are kept (_sym_pts) so that a camera change resizes and turns them (_place_symbols)."""
        pts = np.asarray(pts, float)
        groups: Dict[str, List[int]] = {}
        missing = []
        for n, path in enumerate(paths):
            (groups.setdefault(path, []) if path else missing).append(n)
        self._sym_pts = getattr(self, "_sym_pts", {})
        for path, rows in groups.items():
            q = pts[rows]
            corners = self._sym_corners(q)
            name = f"{DYN}sym_{os.path.basename(path)}"
            self._used.add(name)
            self._sym_pts[name] = q
            e = self._dyn.get(name)
            if e is not None and e[0] == len(rows):
                e[1].points = corners
                e[1].Modified()
                e[2].SetVisibility(True)
                continue
            if e is not None:
                self.plotter.remove_actor(e[2], render=False)
            n = len(rows)
            faces = np.hstack([np.full((n, 1), 4), (4 * np.arange(n))[:, None] + np.arange(4)]).ravel()
            mesh = pv.PolyData(corners, faces=faces)
            mesh.active_texture_coordinates = np.tile([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], (n, 1))
            actor = self.plotter.add_mesh(mesh, texture=_symbol_texture(path), lighting=False, name=name,
                                          render=False)
            self._dyn[name] = [n, mesh, actor]
        if missing:
            self._poly(f"{DYN}pts", pts[missing], color="#ffffff", point_size=8, render_points_as_spheres=True)

    def _sym_corners(self, q: np.ndarray, px: float = SYMBOL_PX) -> np.ndarray:
        """The four corners of each symbol centred on q, facing the camera, px (SYMBOL_PX) x the UI size on
        screen."""
        px = px * self.ui_scale
        cam = self.plotter.camera
        pos = np.asarray(cam.GetPosition())
        fwd = np.asarray(cam.GetDirectionOfProjection())
        up = np.asarray(cam.GetViewUp(), float)
        up = up - (up @ fwd) * fwd
        up /= np.linalg.norm(up)
        right = np.cross(fwd, up)
        h_px = max(self.plotter.ren_win.GetSize()[1], 1)
        k = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / h_px * 0.5 * px
        half = np.maximum((q - pos) @ fwd, 1e-12)[:, None] * k                 # world size of SYMBOL_PX / 2
        return np.stack([q - half * right - half * up, q + half * right - half * up,
                         q + half * right + half * up, q - half * right + half * up], axis=1).reshape(-1, 3)

    def _place_symbols(self) -> None:
        """After a camera change: every shown symbol (and the selection's square) resized and turned toward the
        camera."""
        for name, q in getattr(self, "_sym_pts", {}).items():
            e = self._dyn.get(name)
            if e is not None and e[2].GetVisibility() and e[0] == len(q):
                e[1].points = self._sym_corners(q)
                e[1].Modified()
        for name, q in self._box_pts.items():         # the data boxes beside their symbols
            lab = self._lab.get(name)
            if lab is not None and lab.actor.GetVisibility() and len(q) == len(lab._last):
                lab.set(self._box_anchor(q), lab._last)
        for name, q in getattr(self, "_mt_lbl", {}).items():     # the multitracks' numbers under their symbols
            lab = self._lab.get(name)
            if lab is not None and lab.actor.GetVisibility() and len(q) == len(lab._last):
                lab.set(self._mt_below(q), lab._last)
        e = self._dyn.get(f"{DYN}select")
        q = getattr(self, "_sel_pt", None)
        if e is not None and q is not None and e[2].GetVisibility():
            e[1].points = self._sym_corners(q[None], SELECT_PX)
            e[1].Modified()

    def _screen_len(self, a: np.ndarray) -> np.ndarray:
        """Length (scene units) at the points a (n, 3) that TRIAD_FRAC of the camera distance stands for at the
        focal point: the same length on screen wherever the point is, whatever the zoom."""
        cam = self.plotter.camera
        fwd = np.asarray(cam.GetDirectionOfProjection())
        depth = (np.asarray(a, float) - np.asarray(cam.GetPosition())) @ fwd
        return TRIAD_FRAC * np.maximum(depth, 1e-9 * cam.GetDistance())

    def _scaled(self, name: str, a: np.ndarray, dirs: np.ndarray, **style) -> None:
        """Segments from the points a (n, 3) along the unit directions dirs (n, 3) (axes, noses) of a constant
        length on screen, kept so by every camera change (_place_scaled, user 29 October 2026: the axes follow
        the zoom)."""
        a, dirs = np.asarray(a, float), np.asarray(dirs, float)
        self._scaled_segs = getattr(self, "_scaled_segs", {})
        self._scaled_segs[name] = (a, dirs)
        self._poly(name, np.r_[a, a + self._screen_len(a)[:, None] * dirs], lines=_pairs(len(a)), **style)

    def _place_scaled(self) -> None:
        """After a camera change: the axes and noses (_scaled) and the 1:1 ship axes' labels at their length."""
        for name, (a, dirs) in getattr(self, "_scaled_segs", {}).items():
            e = self._dyn.get(name)
            if e is not None and e[2].GetVisibility() and e[0][0] == 2 * len(a):
                e[1].points = np.r_[a, a + self._screen_len(a)[:, None] * dirs]
                e[1].Modified()
        lab = self._lab.get(f"{DYN}ship_axes_lbl")
        if lab is not None and lab.actor.GetVisibility() and not self._global():
            lab.set(1.05 * self._screen_len(np.zeros((1, 3)))[:, None] * np.eye(3), ["X", "Y", "Z (nose)"])

    def _draw_selection(self) -> None:
        """A square around the selected body where the view draws it (an object or OWNSHIP; none for the Earth, the
        Moon, the Sun), SELECT_PX on screen, kept facing the camera (_place_symbols)."""
        sel, q = self.selected, None
        if sel and sel.startswith(MT_SEL):                # a multitrack: its first member's
            sel = sel[len(MT_SEL):].split("|")[0]
        if sel and sel in getattr(self, "_mt_of", {}):    # absorbed in a multitrack: around it (user)
            q = np.asarray(self._mt_of[sel], float)
        elif sel and not sel.startswith(BODY_SEL):
            if self._global():
                r = getattr(self, "_frame_r", None)
                i = 0 if sel == OWNSHIP else (self.truth.row(sel) if sel in self.truth.ids else None)
                if r is not None and i is not None and np.isfinite(r[i, 0]):
                    q = np.asarray(r[i], float)
            else:
                q = np.zeros(3) if sel == OWNSHIP else getattr(self, "_screen_pts", {}).get(sel)
        self._sel_pt = q
        if q is not None:
            self._poly(f"{DYN}select", self._sym_corners(np.asarray(q, float)[None], SELECT_PX),
                       lines=np.array([5, 0, 1, 2, 3, 0]), color=SELECT_COLOR, line_width=2)

    def select(self, oid: Optional[str]) -> None:
        """Select a body (an object's id, OWNSHIP, None: none) as a click on it in the view would: the square
        around it and its Info (the scenario tool's tree, user 29 October 2026)."""
        self.selected = oid or None
        self._draw_frame()

    def _labels(self, name: str, pts: np.ndarray, texts: List[str], color: str, size: int = 15,
                box: bool = False) -> None:
        self._used.add(name)
        lab = self._lab.get(name)
        if lab is None:
            lab = self._lab[name] = _Labels(self.plotter, color, size, box)
        lab.set(np.asarray(pts, float), texts)

    def _box_anchor(self, q: np.ndarray) -> np.ndarray:
        """The top-left corner of each body's data box: right of its selection square (BOX_GAP of a symbol apart,
        user 30 October 2026: right of the symbol, the square covered it when the body was selected), level with the
        symbol's top, at the UI size; the box never covers the symbol nor its square."""
        cam = self.plotter.camera
        fwd = np.asarray(cam.GetDirectionOfProjection())
        up = np.asarray(cam.GetViewUp(), float)
        up = up - (up @ fwd) * fwd
        up /= np.linalg.norm(up)
        right = np.cross(fwd, up)
        h_px = max(self.plotter.ren_win.GetSize()[1], 1)
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) / h_px * SYMBOL_PX * self.ui_scale
        half = np.maximum((q - np.asarray(cam.GetPosition())) @ fwd, 1e-12)[:, None] * k   # half a symbol, world
        side = max(1.0, SELECT_PX / SYMBOL_PX) + 2.0 * BOX_GAP          # (in half symbols)
        return q + side * half * right + half * up

    def _databox(self, name: str, pts: np.ndarray, texts: List[str]) -> None:
        """The bodies' data boxes (user, 29 October 2026), beside their symbols, kept there by the camera changes
        (_place_symbols)."""
        pts = np.asarray(pts, float)
        self._box_pts[name] = pts
        self._labels(name, self._box_anchor(pts), texts, TEXT_COLOR, DATA_FONT, box=True)

    def _box_texts(self, rows: np.ndarray, r: np.ndarray, v: np.ndarray) -> List[str]:
        """The data box of each truth row (0: OWNSHIP): its name, altitude and orbital speed; for an object also its
        range, closing speed, time to and range of its closest approach to OWNSHIP (_cpa_all)."""
        tr = self.truth
        alt = np.linalg.norm(r[rows], axis=1) - tr.body.radius_km
        spd = np.linalg.norm(v[rows], axis=1)
        rel, vrel = r[rows] - r[0], v[rows] - v[0]
        rng = np.linalg.norm(rel, axis=1)
        close = -1e3 * np.einsum("ij,ij->i", rel, vrel) / np.maximum(rng, 1e-12)        # m/s, > 0 closing
        t_c, d_c, reached = self._cpa_all()
        out = []
        for n, i in enumerate(rows):
            lines = [self.own_name() if i == 0 else tr.ids[i], f"alt {alt[n]:.1f} km", f"v {spd[n]:.3f} km/s"]
            if i:
                lines += [f"rng {_dist_text(rng[n])}", f"Vc {close[n]:+.0f} m/s"]
                tc, dc, ok = t_c[i - 1], d_c[i - 1], reached[i - 1]
                if np.isfinite(tc):
                    if not ok:                        # still closing at the run's end
                        lines += [f"CPA > {tc - self.t:.0f} s", f"dCPA < {_dist_text(dc)}"]
                    else:
                        lines += ["CPA receding" if tc <= self.t + 1e-6 else f"CPA {tc - self.t:.1f} s",
                                  f"dCPA {_dist_text(dc)}"]
            out.append("\n".join(lines))
        return out

    def _cpa_all(self):
        """The closest approach to OWNSHIP of every object (geometry column j = truth row j + 1) from t on, all at
        once (the data boxes, 1000+ objects): (time s, range km, reached in the run) arrays, NaN without a future
        record. The record of the smallest future range, refined by the relative motion at that record (straight
        line, within its two neighbours); cached per record index of t."""
        g, tr = self.g, self.truth
        k0 = int(np.searchsorted(g.t_s, self.t - 1e-9))
        c = getattr(self, "_cpa_all_cache", None)
        if c is not None and c[0] is g and c[1] == k0:
            t_c, d_c, reached = c[2]
        else:
            n = g.range_km.shape[1]
            t_c, d_c, reached = np.full(n, np.nan), np.full(n, np.nan), np.ones(n, bool)
            rr = g.range_km[k0:]
            if len(rr):
                ok = np.isfinite(rr)
                has = ok.any(axis=0)
                ia = k0 + np.argmin(np.where(ok, rr, np.inf), axis=0)
                kt = np.clip(np.searchsorted(tr.t_s, g.t_s[ia] - 1e-9), 0, len(tr.t_s) - 1)
                rows = np.arange(n) + 1
                rel = tr.r_km[kt, rows] - tr.r_km[kt, 0]
                vrel = tr.v_kms[kt, rows] - tr.v_kms[kt, 0]
                dt = -np.einsum("ij,ij->i", rel, vrel) / np.maximum(np.einsum("ij,ij->i", vrel, vrel), 1e-30)
                last = len(g.t_s) - 1
                lo = g.t_s[np.maximum(ia - 1, 0)] - g.t_s[ia]
                hi = g.t_s[np.minimum(ia + 1, last)] - g.t_s[ia]
                dt = np.clip(np.nan_to_num(dt), lo, hi)
                t_c = np.where(has, g.t_s[ia] + dt, np.nan)
                d_c = np.where(has, np.linalg.norm(rel + vrel * dt[:, None], axis=1), np.nan)
                rate = np.nan_to_num(g.range_rate_mps[ia, np.arange(n)], nan=1.0)
                reached = ~((ia == last) & (rate < 0))
            self._cpa_all_cache = (g, k0, (t_c, d_c, reached))
        return np.where(np.isfinite(t_c), np.maximum(t_c, self.t), np.nan), d_c, reached

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
        self._draw_selection()
        self._hide_unused()
        self._show_info()
        self._render()

    def _update_time_label(self) -> None:
        if self.sc is None:
            self.time_label.setText("")
            return
        utc = scn.epoch(self.sc) + timedelta(seconds=self.t)
        self.time_label.setText(f"t {self.t:9.2f} s   {utc.strftime('%Y-%m-%d %H:%M:%S')} UTC")

    def set_ui_scale(self, step: int) -> None:
        """UI size one step down (-1) or up (+1) among UI_SCALES: the labels, symbols and the selection's square."""
        k = int(np.argmin([abs(x - self.ui_scale) for x in UI_SCALES]))
        self.ui_scale = UI_SCALES[int(np.clip(k + step, 0, len(UI_SCALES) - 1))]
        self.plotter._ui_scale = self.ui_scale
        for lab in list(getattr(self.plotter, "_ui_labels", ())):
            lab.rescale()
        self._show_ui_scale()
        self._place_symbols()
        if self.truth is not None:
            self._draw_frame()
        else:
            self._render()

    def _show_ui_scale(self) -> None:
        self.lb_ui.setText(f"{100 * self.ui_scale:.0f} %")
        self.bt_ui_minus.setEnabled(self.ui_scale > UI_SCALES[0])
        self.bt_ui_plus.setEnabled(self.ui_scale < UI_SCALES[-1])

    def _show_sky(self) -> None:
        self.sky.set_visible(self.tg_sky.isChecked(), self.cb_const.isChecked())
        self._render()

    def _frame_global(self) -> None:
        tr = self.truth
        self.sky.set_rotation(None)                   # ICRF
        r, v = sgeo.states_at(tr, self.t)
        self._frame_r = r                             # the bodies as drawn (the selection's square)
        ex = np.isfinite(r[:, 0])
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False
        self._place_bodies()
        self._body_orbits()
        idx = np.flatnonzero(ex)
        clusters = self._frame_clusters(idx, r[idx])
        self._draw_bodies(idx, r[idx], r, v, 8, clusters)
        # the paths: one per lone body and one per multitrack (its members' mean, user 29 October 2026)
        groups, rgb = self._path_groups(idx, clusters)
        if self.cb_trails.isChecked():
            self._truth_lines(f"{DYN}trails", groups, rgb, max(0.0, self.t - self.sp_trail.value()), self.t,
                              line_width=3, opacity=0.8)
        if self.cb_planned.isChecked():               # from t to the run's end, OWNSHIP's thicker
            own = np.array([len(g) == 1 and g[0] == 0 for g in groups], bool)
            mine = np.array([len(g) == 1 and g[0] == self.node.row() for g in groups], bool)   # its live path
            keep = [k for k in range(len(groups)) if not own[k] and not mine[k]]
            self._truth_lines(f"{DYN}planned", [groups[k] for k in keep], rgb[keep], self.t, self.duration(),
                              line_width=1.5)
            keep = [k for k in range(len(groups)) if own[k] and not mine[k]]
            self._truth_lines(f"{DYN}planned_ship", [groups[k] for k in keep], rgb[keep], self.t, self.duration(),
                              line_width=3)
        self.node.draw()
        self._draw_markers()
        if self.cb_orbit.isChecked() and len(groups):     # free fall from t under the local gravity
            self._coast_lines(self._coast(_group_mean(r, groups), _group_mean(v, groups)), rgb)
        # ship attitude triad, enlarged (V3), of constant size on screen ("ship axes" switch)
        if self.cb_axes.isChecked():
            m = tr.attitude("ship", self.t)
            self._scaled(f"{DYN}triad", np.repeat(r[0][None, :], 3, axis=0), m.T, cell_rgb=self._triad_rgb,
                         line_width=3)

    def _frame_ship(self) -> None:
        tr, g = self.truth, self.g
        r, v = sgeo.states_at(tr, self.t)
        m = tr.attitude("ship", self.t)
        self.sky.set_rotation(m.T)                    # the sky in the ship's frame: it turns with the attitude
        self._planets_drawn = {}
        if self.cb_bodies.isChecked():
            self._ship_bodies(r[0], m)
        else:
            self._planet_marks()                      # (no planet: no dot)
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
        pos = d * np.asarray(rng, float)[:, None]
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
        full = self._full_projection()
        if full:                                      # the copies in the plane stand for the objects
            pos = self._to_plane(pos)
        # one line of sight, projection copy and path per lone object and per multitrack (its centre), no nose for
        # a multitrack's members (user, 29 October 2026)
        clusters = self._frame_clusters(js + 1, pos)
        lone = np.ones(len(js), bool)
        for c in clusters:
            lone[c["k"]] = False
        cq = np.array([c["q"] for c in clusters]).reshape(-1, 3)
        sight = np.r_[pos[lone], cq]
        n = len(sight)
        self._poly(f"{DYN}los", np.r_[np.zeros((n, 3)), sight], lines=_pairs(n),
                   cell_rgb=np.r_[los_rgb[lone], los_rgb[[c["k"][0] for c in clusters]].reshape(-1, 3)],
                   line_width=1.5, opacity=0.7)
        self._draw_bodies(js + 1, pos, r, v, 9, clusters)
        groups, rgb = self._path_groups(js + 1, clusters)
        cols = [gr - 1 for gr in groups]              # (the geometry's columns)
        if self.cb_proj.isChecked() and not full:
            self._range_projection(sight, rgb)
        if self.cb_vectors.isChecked():
            nose = g.nose_ship[k, js]
            okn = np.isfinite(nose[:, 0]) & lone
            if okn.any():                             # constant on screen, whatever the zoom
                self._scaled(f"{DYN}nose", pos[okn], nose[okn], color=TEXT_COLOR, line_width=2)
        if self.cb_trails.isChecked():                # every object's trail in one pass (one polyline each)
            sel = (g.t_s >= self.t - self.sp_trail.value()) & (g.t_s <= self.t)
            self._ship_lines(f"{DYN}trails", cols, rgb, sel, line_width=2, opacity=0.6)
        if self.cb_planned.isChecked():               # the records from t to the run's end
            keep = [kk for kk, gr in enumerate(groups) if not (len(gr) == 1 and gr[0] == self.node.row())]
            self._ship_lines(f"{DYN}planned", [cols[kk] for kk in keep], rgb[keep], g.t_s >= self.t, line_width=1.5)
        self.node.draw()
        if self.cb_orbit.isChecked():                 # the free falls relative to OWNSHIP's own (user)
            self._coast_lines(self._ship_coast(r, v, _group_mean(r, groups), _group_mean(v, groups)), rgb)
        self._screen_pts = {g.ids[j]: pos[jj] for jj, j in enumerate(js)}
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False
        if self.cb_axes.isChecked():                  # the ship's axes, constant on screen
            self._scaled(f"{DYN}ship_axes", np.zeros((3, 3)), np.eye(3), cell_rgb=self._triad_rgb, line_width=3)
            self._labels(f"{DYN}ship_axes_lbl", 1.05 * self._screen_len(np.zeros((1, 3)))[:, None] * np.eye(3),
                         ["X", "Y", "Z (nose)"], TEXT_COLOR, 15)
        self._update_rings()

    def _ship_bodies(self, r_ship: np.ndarray, m_ship: np.ndarray) -> None:
        """The Earth, the Moon and the Sun in the ship view (box "celestial objects", user 29 October 2026), at their
        true directions from the ship, textured and turned as in the global view (body-fixed -> ship frame), at
        their true distance and size."""
        bodies = self._bodies_at()
        self._ship_discs = {}                              # key: (centre, radius) as drawn, for the clicks
        mats = {"earth": _earth_matrix(scn.epoch(self.sc), self.t),
                "moon": _moon_matrix(bodies["moon"] - bodies["earth"]), "sun": _sun_matrix(scn.epoch(self.sc), self.t)}
        mats.update({p: _iau_matrix(PLANET_DATA[p], scn.epoch(self.sc), self.t) for p in PLANET_NAMES.values()})
        mats.update({m: self._moon_matrix_of(m, False) for m in MOON_DATA})
        pts, names = [], []
        self._planets_drawn = {}
        for b in ("earth", "moon", "sun") + tuple(PLANET_NAMES.values()) + tuple(MOON_DATA):
            rel = bodies[b] - r_ship                                       # km, ICRF axes
            rng = float(np.linalg.norm(rel))
            rad_true = self._body_radius(b)
            if rng <= rad_true:
                continue
            dist, size = rng, rad_true
            centre = (rel / rng) @ m_ship * dist
            self._ship_discs[b] = (centre, size)
            name = f"{DYN}sbody_{b}"
            self._used.add(name)
            e = self._dyn.get(name)
            if e is None and b in MOON_DATA:
                e = self._dyn[name] = [None, None, self._add_moon_mesh(self.plotter, b, name)]
            if e is None:
                tex = _sun_texture() if b == "sun" else _texture(b)
                col = SUN_COLOR if b == "sun" else PLANET_COLORS.get(b, EARTH_COLOR)
                act = self.plotter.add_mesh(_body_mesh(1.0), texture=tex, smooth_shading=True, culling="back",
                                            color=None if tex is not None else col,
                                            lighting=(b in ("earth", "moon")), ambient=0.12, name=name, render=False)
                e = self._dyn[name] = [None, None, act]
            u = np.eye(4)
            shape = _shape_mesh(b) is not None                          # (a shape model: in km already)
            u[:3, :3] = (1.0 if shape else size) * (m_ship.T @ mats[b])
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
            self._planets_drawn[b] = (centre, size)                         # a dot when too small (_planet_marks)
            if b in PLANET_DATA:
                if b == "saturn":
                    self._used.add(f"{DYN}sbody_saturn_rings")
                    h = self._dyn.get(f"{DYN}sbody_saturn_rings")
                    if h is None:
                        tex = _rings_texture()
                        h = self._dyn[f"{DYN}sbody_saturn_rings"] = [None, None, self.plotter.add_mesh(
                            _rings_mesh(), texture=tex, color=None if tex is not None else PLANET_COLORS["saturn"],
                            lighting=False, name=f"{DYN}sbody_saturn_rings", render=False)]
                    ur = np.eye(4)
                    ur[:3, :3], ur[:3, 3] = m_ship.T @ mats[b], centre
                    h[2].user_matrix = ur
            if b == "sun":                                                 # its halo, at its true size too
                self._used.add(name + "_halo")
                h = self._dyn.get(name + "_halo")
                if h is None:
                    h = self._dyn[name + "_halo"] = [None, None, _add_sun_halo(self.plotter, name + "_halo")]
                _place_halo(self.plotter, h[2], centre, size)
                self._used.add(name + "_limb")
                lb = self._dyn.get(name + "_limb")
                if lb is None:
                    lb = self._dyn[name + "_limb"] = [None, None, _add_sun_limb(self.plotter, name + "_limb")]
                lb[2].SetVisibility(True)
                self._limbs = [(lb[2], centre, size)]
                _place_limb(self.plotter, lb[2], centre, size)
                h[2].SetVisibility(True)
            pts.append(centre)
            names.append(b.capitalize())
        marked = self._planet_marks()
        if pts and self.cb_labels.isChecked():          # (the planets and the bodies drawn as dots: _planet_marks)
            keep = [k for k, n in enumerate(names) if n.lower() not in PLANET_DATA and n.lower() not in MOON_DATA
                    and n.lower() not in marked]
            if keep:
                self._labels(f"{DYN}sbody_lbl", np.array(pts)[keep], [names[k] for k in keep], MUTED_COLOR, DATA_FONT)

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
                poly, outline = cgeo.build_sector_volume_km(self.grid, sec, e0, e1)
            except (KeyError, TypeError, ValueError):
                continue
            col = cgeo.sector_color(i)
            acts = []
            if poly.n_points:
                acts.append(self.plotter.add_mesh(poly, color=col, opacity=0.2, culling=None,
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
        vol["labels"] = _Labels(self.plotter, TEXT_COLOR, 16) if vol["per_sector"] else None
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
        for key, q in getattr(self, "_mt_click", {}).items():     # a multitrack: one target, its members not
            for oid in key[len(MT_SEL):].split("|"):
                pts.pop(oid, None)
            pts[key] = q
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
        members = best[len(MT_SEL):].split("|") if best and best.startswith(MT_SEL) else [best]
        if self.node.target is not None and not any(self.node.owns("ship" if b == OWNSHIP else b) for b in members):
            self.node.release()                       # another body or nothing: the manoeuvre let go (user)
            self._draw_frame()
        if best is not None:
            self._select(best)
            self.picked.emit(best)                    # the editor's tree follows (user, 29 October 2026)
            if double:
                self.set_centre(members[0])           # (a multitrack: centred on its first member)
            return
        body = self._body_under(x, y)
        if body is not None:
            self._select(BODY_SEL + body)                # a celestial body: its facts (user, 29 October 2026)
        if double:
            self.set_centre(body or CENTRAL)

    def _body_under(self, x: float, y: float) -> Optional[str]:
        """The celestial body whose disc on screen holds the display point (x, y) (global view; ship view: as drawn
        by "celestial objects"), the nearest to the camera first, but a planet before its moons when the point is on
        both (user, 30 October 2026: a click near a planet and its moons is the planet's); None: none."""
        if self._global():
            at = self._bodies_at()
            discs = {name: (at[key], self._body_radius(key)) for name, key in BODY_NAMES.items()}
        else:
            discs = {name: getattr(self, "_ship_discs", {})[key] for name, key in BODY_NAMES.items()
                     if self.cb_bodies.isChecked() and key in getattr(self, "_ship_discs", {})}
        ren = self.plotter.renderer
        cam = np.asarray(self.plotter.camera.GetPosition())
        up = np.asarray(self.plotter.camera.GetViewUp(), float)
        hits = []
        for name, (c, rad) in discs.items():
            pc, pe = [], []
            for w, out in ((c, pc), (c + rad * up, pe)):
                ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
                ren.WorldToDisplay()
                out.extend(ren.GetDisplayPoint()[:2])
            if math.hypot(pc[0] - x, pc[1] - y) <= max(math.hypot(pe[0] - pc[0], pe[1] - pc[1]), CLICK_PX):
                hits.append((float(np.linalg.norm(c - cam)), name))
        names = {n for _d, n in hits}
        hits = [h for h in hits if not (BODY_NAMES[h[1]] in MOON_DATA and next(
            (n for n, k in BODY_NAMES.items() if k == MOON_DATA[BODY_NAMES[h[1]]]["planet"]), None) in names)]
        return min(hits)[1] if hits else None

    def _select(self, oid: str) -> None:
        self.select(oid)

    def _show_multitrack_info(self, ids: List[str]) -> None:
        """A multitrack clicked (user, 29 October 2026): its number, identity and class, then each member and its
        range to OWNSHIP (zoom in to tell them apart)."""
        tr = self.truth
        ids = [x for x in ids if x in tr.ids]
        if not ids:
            self._set_info("")
            return
        r, _v = sgeo.states_at(tr, self.t)
        i0 = tr.row(ids[0])
        ident, cls = self._row_ident[i0], tr.classes[i0]
        lines = [_kv("name", f"{len(ids)} x {cls}"), _kv("charact.", cls), _kv("classif.", ident),
                 _kv("t", f"{self.t:.2f} s"), _kv("members", "range")]
        for oid in ids:
            d = r[tr.row(oid)] - r[0]
            lines.append(_kv(oid, _dist_text(float(np.linalg.norm(d))) if np.isfinite(d[0]) else "not there"))
        self._set_info("\n".join(lines), symbol_path(ident, cls, multi=True))

    def _set_info(self, text: str, icon: Optional[str] = None) -> None:
        """The information on the selected body, in the Info box of the side column (empty: no body chosen); icon:
        the object's symbol, beside its name."""
        self.info.setPlainText(text)
        self.info.set_icon(icon)

    def _show_info(self) -> None:
        if self.truth is None or not self.selected:
            self._set_info("")
            return
        if self.selected == OWNSHIP:
            self._show_ship_info()
            return
        if self.selected.startswith(BODY_SEL):
            self._show_body_info(self.selected[len(BODY_SEL):])
            return
        if self.selected.startswith(MT_SEL):
            self._show_multitrack_info(self.selected[len(MT_SEL):].split("|"))
            return
        if self.selected not in self.g.ids:
            self._set_info("")
            return
        tr, g = self.truth, self.g
        i, j, k = tr.row(self.selected), g.ids.index(self.selected), self._k()

        r, v = sgeo.states_at(tr, self.t)
        if not np.isfinite(r[i, 0]):
            self._set_info("\n".join([_kv("name", self.selected), _kv("charact.", g.classes[j]),
                                      _kv("t", f"{self.t:.2f} s"), _kv("state", "not there at t")]),
                           self._row_symbol(i))
            return
        los = r[i] - r[0]
        rng = float(np.linalg.norm(los))
        rr = 1e3 * float(los @ (v[i] - v[0])) / rng
        d = (los / rng) @ tr.attitude("ship", self.t)
        f, w, _, _ = opt.face_of(d[0], d[1], d[2])
        f = int(f)
        ident = scn.identity_of(self.sc, self.selected, tr.group_of[i] or None)
        occ = g.occulted_by[k, j]
        # short labels (user): charact. = characterisation (class), classif. = classification (identity)
        lines = [_kv("name", self.selected), _kv("charact.", g.classes[j]), _kv("classif.", ident),
                 _kv("t", f"{self.t:.2f} s")]
        lines += _vec_lines("pos", r[i], "km", 3) + _vec_lines("vel", v[i], "km/s", 5)
        lines += [_kv("frame", f"ICRF / {tr.body.name}"),
                  _kv("range", f"{rng:.3f} km"), _kv("range rate", f"{rr:+.1f} m/s")]
        lines += self._cpa_lines(i, j)                # closest approach to OWNSHIP (user, 29 October 2026)
        lines.append(_kv("face", f"F{f + 1}" if f >= 0 else "none"))
        if f >= 0:
            lines.append(_kv("theta", f"{math.degrees(math.acos(min(1.0, float(w)))):.2f} deg"))
        lines += [_kv("recorded", f"{g.t_s[k]:.1f} s"), _kv("aspect", f"{g.aspect_deg[k, j]:.1f} deg"),
                  _kv("RCS S", f"{g.rcs_s_dbsm[k, j]:.1f} dBsm"), _kv("RCS Ku", f"{g.rcs_ku_dbsm[k, j]:.1f} dBsm"),
                  _kv("occulted", g.body_names[occ] if occ >= 0 else "no")]
        if np.isfinite(g.block_loss_db[k, j]):
            lines += [_kv("masked", "yes" if g.masked[k, j] else "no"),
                      _kv("block loss", f"{max(0.0, float(g.block_loss_db[k, j])):.2f} dB / pass")]
        self._set_info("\n".join(lines), self._row_symbol(i))           # its symbol beside its name

    def _cpa(self, i: int, j: int) -> Optional[Tuple[float, float, bool]]:
        """The closest approach to OWNSHIP from t on (user, 29 October 2026): (its time, its range km, reached in the
        run: False when the object is still closing at the run's end). The recorded ranges give the instant, refined
        between its neighbours on the interpolated states (golden section); kept while t stays before it."""
        c = getattr(self, "_cpa_cache", {}).get(i)
        if c is not None and c[0] <= self.t <= c[1]:
            return c[2]
        g, tr = self.g, self.truth
        fut = np.flatnonzero(g.t_s >= self.t)
        rr = g.range_km[fut, j] if len(fut) else np.empty(0)
        ok = np.isfinite(rr)
        if not ok.any():
            return None
        ia = int(fut[np.flatnonzero(ok)[int(np.argmin(rr[ok]))]])

        def rng(t: float) -> float:
            r, _ = sgeo.states_at(tr, t)
            d = float(np.linalg.norm(r[i] - r[0]))
            return d if np.isfinite(d) else math.inf

        a = max(self.t, float(g.t_s[max(ia - 1, 0)]))
        b = min(self.duration(), float(g.t_s[min(ia + 1, len(g.t_s) - 1)]))
        q = (math.sqrt(5.0) - 1.0) / 2.0
        x1, x2 = b - q * (b - a), a + q * (b - a)
        f1, f2 = rng(x1), rng(x2)
        for _ in range(CPA_REFINE_ITER):
            if b - a < 1e-6:
                break
            if f1 <= f2:
                b, x2, f2 = x2, x1, f1
                x1 = b - q * (b - a)
                f1 = rng(x1)
            else:
                a, x1, f1 = x1, x2, f2
                x2 = a + q * (b - a)
                f2 = rng(x2)
        cands = [(rng(x), x) for x in (a, b, 0.5 * (a + b))] + [(float(g.range_km[ia, j]), float(g.t_s[ia]))]
        d, t_cpa = min(cands)
        reached = not (ia == len(g.t_s) - 1 and np.isfinite(g.range_rate_mps[ia, j]) and g.range_rate_mps[ia, j] < 0)
        res = (max(t_cpa, self.t), d, reached)
        self._cpa_cache = getattr(self, "_cpa_cache", {})
        self._cpa_cache[i] = (min(self.t, t_cpa), t_cpa, res)
        return res

    def _cpa_lines(self, i: int, j: int) -> List[str]:
        """The Info rows of the closest approach: time before it, its range."""
        c = self._cpa(i, j)
        if c is None:
            return []
        t_cpa, d, reached = c
        dist = _dist_text(d)
        if not reached:                               # still closing at the run's end
            return [_kv("CPA in", f"> {t_cpa - self.t:.1f} s (after the end)"), _kv("CPA range", f"< {dist}")]
        return [_kv("CPA in", f"{t_cpa - self.t:.1f} s" if t_cpa > self.t + 1e-6 else "now (receding)"),
                _kv("CPA range", dist)]

    def _show_body_info(self, name: str) -> None:
        """A celestial body's facts (assets/celestial_info.json), one per line, and nothing else (user)."""
        self._set_info("\n".join(body_info_lines(name)))

    def side_width(self) -> int:
        """px: the side column just wide enough for the widest Info rows (the bodies' facts, INFO_WIDEST) and for
        the view's options under it."""
        lines = [x for b in BODY_NAMES for x in body_info_lines(b)] + list(INFO_WIDEST)
        return max(self.info.needed_width(lines) + 24, self.view_panel.sizeHint().width())

    def _show_ship_info(self) -> None:
        tr, g = self.truth, self.g
        r, v = sgeo.states_at(tr, self.t)
        m = tr.attitude("ship", self.t)
        w = engine.ship_rate_dps(tr, self.t)
        k = self._k()
        alt = float(np.linalg.norm(r[0])) - tr.body.radius_km
        lines = [_kv("name", self.own_name()), _kv("classif.", OWNSHIP_IDENTITY), _kv("t", f"{self.t:.2f} s")]
        lines += _vec_lines("pos", r[0], "km", 3) + _vec_lines("vel", v[0], "km/s", 5)
        lines += [_kv("frame", f"ICRF / {tr.body.name}"), _kv("altitude", f"{alt:.1f} km"),
                  _kv("speed", f"{1e3 * float(np.linalg.norm(v[0])):.1f} m/s")]
        lines += _vec_lines("nose", m[:, 2], "", 4)                        # the ship's Z axis, ICRF
        lines += _vec_lines("rate", w, "deg/s", 3)                         # ship frame
        lines += [_kv("rate", f"{float(np.linalg.norm(w)):.3f} deg/s"), _kv("recorded", f"{g.t_s[k]:.1f} s"),
                  _kv("Sun hidden", f"{100.0 * float(g.ship_shadow[k]):.0f} %")]
        self._set_info("\n".join(lines), symbol_path("ownship", None))
