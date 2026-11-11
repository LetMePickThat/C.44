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
  L3 trails of a chosen duration (each view's profile, tool.display.view_*), planned trajectories (from t to the run's
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

import atexit
import dataclasses
import copy
import json
import math
import os
import sys
from functools import lru_cache
import time
import weakref
from datetime import timedelta
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv
from vtkmodules.util.numpy_support import numpy_to_vtk, vtk_to_numpy
from PyQt5.QtCore import QProcess, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (QButtonGroup, QMenu, QToolButton, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QMessageBox, QProgressBar, QPushButton, QShortcut, QVBoxLayout, QWidget)
from PyQt5.QtGui import QCursor, QKeySequence
from pyvistaqt import QtInteractor

from core import frames
from core import geometry as cgeo
from core import optimizer as opt
from core.params import P
from gui import scene_view as sv
from gui import controls as uc
from gui import sky as gsky
from gui import view3d as v3
from sim import attitude as att
from sim import encounter as enc
from sim import engine
from sim import ephemeris as eph
from sim import fast
from sim import geometry as sgeo
from sim import preview
from sim import propagate as prop
from sim import scenario as scn

FPS =int(P.get("tool.display.sim_fps"))
BOX_TEXT_S = float(P.get("tool.display.sim_box_text_s"))   # the data boxes' texts during the playback: at most once per this
BODY_SHAPES = {"SHIP": ("cylinder", 130.0, 15.0),   # the objects at their size in the 3D views: shape, length and
               "KKV": ("cylinder", 1.5, 0.5),      # diameter, m; a cylinder along the nose (user, 8 November 2026)
               "DEBRIS": ("sphere", 1.0, 1.0), "DECOY": ("sphere", 1.0, 1.0)}
MARK_FONT = 14                                   # events' labels: pt at UI size 100 %
MARK_GROUP_PX = 8.0                               # ... of one kind, points this close on screen: one label (D, 8 Nov 2026)
DRAG_FLEET_HZ = 10.0                             # node drag: the whole preview drawn at most this often (D, 8 Nov 2026)
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # (the real computation runs there)
AUTO_MAX_CELLS = float(P.get("tool.display.sim_auto_max_cells"))   # (former propagation; unused by the live view)
PROP_TIP_IDLE = "Only for large scenarios, after a change: smaller ones propagate by themselves"
PROP_TIP_NEEDED = "The scenario changed and is too large for the automatic propagation: propagate it"
MAX_VIEW_MARGIN = float(P.get("tool.display.sim_max_view_margin"))   # farthest zoom out: the solar system (user)
FARTHEST_APHELION_KM = max(el[0] * (1.0 + el[1]) for el, _r in eph.PLANET_ELEMENTS.values()) * eph.AU_KM
MIN_CAMERA_KM = 1e-3 * float(P.get("tool.display.sim_min_camera_m"))   # nearest zoom in, to any focal point (user)
TURN_DEG_PX = v3.TURN_DEG_PX                      # mouse rotation (shared: gui/view3d.py)
SPEEDS = (1, 10, 100, 1000, 10000, 100000)     # x10 000 and x100 000: the Moon on its orbit (user)
VOL_REBUILD_DEG = 0.5                             # V5: a volume is turned in place, rebuilt past this turn of its frame
#   or this move of the central body seen from the ship (G1, user 10 November 2026: 5 deg let a volume low on the
#   horizon pass through the ground until the next rebuild; was 5)
RING_STEP_FRAC = 0.25                             # V4: range rings spaced this share of the visible half-height apart
RING_COVER = 1.1                                  # ... out to this x the farthest corner of the 3D view
RING_MIN_SIN_EL = 0.2                             # (a view almost in the rings' plane: at most 5 x the half-height)
RING_LABEL_OFF = 0.02                             # ring values beside the azimuth 0 and 180 lines, this x the half-height (D)
SPOKE_STEP_DEG = 45.0                             # ship view: azimuth spokes every this
SPOKE_LABEL_FRAC = 0.9                            # ... labelled at this x the visible half-height
RING_MAX = 60                                     # rings at most
RING_HYST = 2.0                                   # the rings' step kept while within this factor of the ideal one (D)
RING_POINTS = 121
RING_HIDDEN_DOT_PX = tuple(float(x) for x in P.get("tool.display.sim_ring_hidden_dot_px"))
RING_HIDDEN_BISECT = 12                           # the ends of a hidden arc refined to 2^-this of a ring's step (D)
RING_COLOR = "#4a5266"
PATH_MAX_PTS = 400                                # L3, global view: points of a whole-run path (trail, planned) at least
PATH_TURN_DEG = 1.0                               # ... finer, so that a path turns at most this per segment (user, 11
PATH_BUDGET_PTS = 400_000                         # Nov 2026: a longer scenario showed an orbit's segments again), within
                                                  # this many points for all the paths together (frame time)
COAST_STEP_S = 10.0                               # L3: RK4 step of the orbital velocity vector's free fall, s (D)
COAST_STEPS = (20, 200)                           # ... its step count kept between these (smooth, 1000+ objects)
# the objects and their geometry (points, trajectories, trails, range projection) in the colour of their
# classification (user, 29 October 2026), the identity frames' colours of the symbology (tools/gen_symbols.py), both
# read from tool.display.identity_colors
IDENTITY_COLORS = v3.IDENTITY_COLORS                # (shared: gui/view3d.py)
if set(IDENTITY_COLORS) != set(scn.IDENTITIES):
    raise ValueError(f"tool.display.identity_colors: one colour per identity expected, {', '.join(scn.IDENTITIES)}")
OWNSHIP_IDENTITY = "friendly"                     # OWNSHIP is friendly (user): always drawn in that colour
SHIP_COLOR = IDENTITY_COLORS[OWNSHIP_IDENTITY]    # its point, trajectory and trail
KM_PER_MODEL_UNIT = 1e-3 / cgeo._M                # 1:1: the model's scene units to km (its true size)
# the view's options panel, background and text colours: shared with the mission preparation (gui/view3d.py)
OPTION_COLUMNS, PAIR_NAME, PAIR_PAD, OPTION_MARGIN = v3.OPTION_COLUMNS, v3.PAIR_NAME, v3.PAIR_PAD, v3.OPTION_MARGIN
BG_COLOR, TEXT_COLOR = v3.BG_COLOR, v3.TEXT_COLOR
# the widest Info lines of an object or the ship (side_width: the column fits them and the bodies' facts)
INFO_WIDEST = ("classification\tsuspect", "reference frame\tjupiter equator", "position x\t-1234567.890 km", "velocity x\t-12.34567 km/s",
               "block loss\t12.34 dB / pass", "rate x\t-12.345 deg/s", "range\t1234567.890 km", "Sun hidden\t100 %",
               "longitude of ascending node\t359.999 deg", "closest approach in\t> 12345.6 s (after the end)", "closest distance\t< 1234567.890 km",
               "radar cross section (Ku band)\t-123.4 dBsm")
SYMBOL_DIR, SYMBOL_PX = v3.SYMBOL_DIR, v3.SYMBOL_PX   # assets/symbols: <classification>_<class>.png (shared)
SELECT_PX = 66                                    # px: the square around the selected body (44 x 1.5)
# multitracks (user, 29 October 2026): objects of one identity and class closer than this on screen (centre to
# centre, x the UI size) drawn as one symbol with a doubled outline, their number under it in the frame's colour
MULTITRACK_PX = float(P.get("tool.display.sim_multitrack_px"))
MULTITRACK_OFF_KM = float(P.get("tool.display.sim_multitrack_off_km"))   # ... none when zoomed in under this (user)
MT_COUNT_GAP_PX = 4.5                             # ... the number this far under the symbol (D)
MT_SEL = "<multi>"                                # a multitrack's selection key: MT_SEL + its members' ids, "|"
SELECT_COLOR = "#ffffff"
BODY_KNOT_S = 30.0                                # s of the run between two computations of the bodies' places
ORBIT_KEEP_PX = 0.25                              # px: a body's orbit moved less than this on screen stays as drawn
CLIP_EVERY = 15                                   # frames between two clipping computations, the camera still
# the data boxes kept apart (user, 1 November 2026, _place_boxes): (x, y) directions of the eight places, the first the
# usual one, right of the symbol
BOX_PLACES = ((1, 0), (-1, 0), (0, -1), (0, 1), (1, -1), (-1, -1), (1, 1), (-1, 1))
BOX_RINGS = 3                                     # ... tried at 1, 2, 3 times the symbol's distance
BOX_MAX_PLACED = 200                              # ... above this number of boxes, each stays beside its symbol
BOX_CHAR_W, BOX_LINE_H = 0.56, 1.2                # estimated width of a character, height of a line: x the font's size
BOX_PAD_PX, BOX_RETURN_PX = 4.0, 12.0             # free margin of a new place; of the usual place, to go back to it
NODE_BOX_PX = 70.0                                # half width kept free around the manoeuvre node
# the target and its marks (user, 1 November 2026): shapes in half widths, a NaN point lifts the pen
TARGET_PX = 92                                    # px: the four bars around the target's symbol (D)
MARK_PX = 20                                      # px: a node or closest-approach mark on a planned path (D)
_UP = (float("nan"), 0.0)
TARGET_BARS = ((0.0, 0.62), (0.0, 1.0), _UP, (0.0, -0.62), (0.0, -1.0), _UP,
               (0.62, 0.0), (1.0, 0.0), _UP, (-0.62, 0.0), (-1.0, 0.0))
NODE_UP = ((0.0, 1.0), (-0.87, -0.5), (0.87, -0.5), (0.0, 1.0))             # the target's nodes: triangles
NODE_DOWN = ((0.0, -1.0), (-0.87, 0.5), (0.87, 0.5), (0.0, -1.0))
EQ_NODE_UP = ((0.0, 1.0), (-0.7, 0.0), (0.0, -0.4), (0.7, 0.0), (0.0, 1.0))  # the equatorial nodes: kites
EQ_NODE_DOWN = ((0.0, -1.0), (-0.7, 0.0), (0.0, 0.4), (0.7, 0.0), (0.0, -1.0))
CPA_CROSS = ((-1.0, -1.0), (1.0, 1.0), _UP, (-1.0, 1.0), (1.0, -1.0))
UI_SCALES, UI_SCALE_START, UI_BUTTON_PX = v3.UI_SCALES, v3.UI_SCALE_START, v3.UI_BUTTON_PX   # "UI size" (view3d)
PLAY_BUTTON_PX = 40                               # the width of the |<, play / pause and >| buttons, px
BOX_GAP = 0.15                                    # data box: its gap to the symbol, share of the symbol's size (D)
CPA_LIVE_CHUNK = 64                               # live closest approaches: objects per vectorised pass (memory)
CPA_REFINE_ITER = 40                             # golden-section steps refining the closest approach between records
MUTED_COLOR = v3.MUTED_COLOR
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
MIN_IDLE_MS = 1                                   # event-loop time kept between two frames (mouse live while playing)
SIDEREAL_MONTH_S = 27.321661 * 86400.0            # the Moon's orbit drawn over one sidereal month
SIDEREAL_YEAR_S = 365.256363 * 86400.0            # the Earth's orbit around the Sun drawn over one sidereal year
# a celestial body selected shows its orbit and those of the bodies orbiting it (user, 29 October 2026; not for the
# scenario's objects): its parent, its period
BODY_PARENT = {"moon": ("earth", SIDEREAL_MONTH_S), "earth": ("sun", SIDEREAL_YEAR_S)}
BODY_ORBIT_POINTS = 361
FPS_FONT = 14                                     # pt: the frame-rate counter
DATA_FONT = 15                                    # pt: the objects' data boxes and the celestial bodies' names,
#                                                   one size (user, 30 October 2026), x the UI size
BODY_AMBIENT = 0.12                               # global view: a body's night side, share of its lit colour
BODY_ORBIT_NEAR = 61                               # ... its points about t, computed at each frame (D)
BODY_ORBIT_COLOR = "#8a8a8a"                      # (a body without a colour of its own)
ORBIT_ALPHA = 0.7                                 # an orbit: its body's colour at this share over the background
ORBIT_MIN_PEAK = 170.0                            # an orbit's colour: its brightest channel at least this (D)
ORBIT_HUE_SHARE = 0.2                             # ... a coloured part of the map at least this share (D)
PROJ_ARC_DEG = 2.0                                # the arc to the range projection: one point per this many degrees
PROJ_ARC_PTS = (3, 91)                            # ... points per arc kept between (D)
# the lines told apart (user, 30 October 2026): a trail fades into the past, a planned trajectory dashed, the orbital
# velocity vector lighter with an arrowhead, the arcs to the range projection dotted (not dashed like a plan)
TRAIL_ALPHA = float(P.get("tool.display.sim_trail_alpha"))
PLANNED_DASH_PX = tuple(float(x) for x in P.get("tool.display.sim_planned_dash_px"))
PROJ_DOT_PX = tuple(float(x) for x in P.get("tool.display.sim_proj_dot_px"))
VEL_WHITE = float(P.get("tool.display.sim_vel_white"))
VEL_ARROW_PX = float(P.get("tool.display.sim_vel_arrow_px"))
VIEWS = (("global", "Global view", "Inertial, around the central body"),
         ("ship", "Ship view", "Around the ship, in the ship's frame: the world turns when the ship turns"),
         ("orbital", "Orbital view", "Around the ship, in its local orbital frame (X prograde, Y normal, Z radial): "
                                     "the ship's model turns, the objects keep their azimuth"))
# each view's button colour and display profile at start (user, 30 October 2026)
VIEW_PROFILES = {mode: dict(P.section(f"tool.display.view_{mode}")) for mode, _text, _tip in VIEWS}
VIEW_COLORS = {mode: prof.pop("color") for mode, prof in VIEW_PROFILES.items()}
VIEW_BUTTON_IDLE = float(P.get("tool.display.sim_view_button_idle"))
ARROW_DEG = 25.0                                  # the arrowhead's wings from its shaft (D)
SCREEN_MARGIN = 1.2                               # dashes cut beyond this x the view's half-width and half-height
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
NEAR_TOL = 1e-15                                  # near / far clipping ratio: Pluto's orbit drawn, the ship at 50 m
# logarithmic depth (user, 30 October 2026): no near plane serves both the ship at 50 m and Saturn at 60 000 km (a linear
# depth buffer told neither its rings from the planet nor a path from a body); bounds in km, 1 mm to beyond Pluto
LOG_DEPTH_NEAR_KM, LOG_DEPTH_FAR_KM = 1e-6, 1e11
LOG_DEPTH_GLSL = (f"gl_FragDepth = clamp(log(1.0 / (gl_FragCoord.w * {LOG_DEPTH_NEAR_KM:e})) / "
                  f"{math.log(LOG_DEPTH_FAR_KM / LOG_DEPTH_NEAR_KM):f}, 0.0, 1.0);\n")
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


def _ground_view(nav) -> Optional[Tuple[np.ndarray, float]]:
    """G1 (user, 10 November 2026): the central body seen from the ship at the navigation of a volume build: the
    direction of its centre in the ship frame and its apparent radius (deg); None without an orbit. The volumes are
    cut by the ground (the useful range): they are rebuilt when it has moved by more than VOL_REBUILD_DEG."""
    if nav is None or not nav.has_orbit or not nav.body_radius_km:
        return None
    d = float(np.linalg.norm(nav.r))
    if d <= 0.0:
        return None
    u = nav.r_si.T @ (-np.asarray(nav.r, float) / d)
    return u, math.degrees(math.asin(min(1.0, float(nav.body_radius_km) / d)))


def _ground_moved(ref, now) -> bool:
    """True when the central body seen from the ship has turned or grown by more than VOL_REBUILD_DEG since ref."""
    if ref is None or now is None:
        return (ref is None) != (now is None)
    ang = math.degrees(math.acos(float(np.clip(ref[0] @ now[0], -1.0, 1.0))))
    return ang > VOL_REBUILD_DEG or abs(ref[1] - now[1]) > VOL_REBUILD_DEG


_WORKERS = set()                                  # the workers alive, whatever became of their views


@atexit.register
def _stop_workers() -> None:
    """A thread still running at the exit would abort the tool. The preview worker waits for its next job on a
    condition and never reads `cancelled`: its own stop() (10 November 2026: waiting for it without it kept the process
    alive after the tool's window was closed, and a sequential pytest session from ending)."""
    for w in list(_WORKERS):
        if hasattr(w, "stop"):
            w.stop()
        else:
            w.cancelled = True
            w.wait()


class SimWorker(QThread):
    """Runs job(progress_cb) in the background; a newer request cancels it at its next progress call. No parent: a
    view closed or deleted meanwhile must not destroy a running thread (it aborts the tool)."""
    progress = pyqtSignal(str, float)
    done = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, job: Callable):
        super().__init__()
        self.job, self.cancelled = job, False
        _WORKERS.add(self)
        self.finished.connect(lambda: _WORKERS.discard(self))

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


class _PreviewWorker(QThread):
    """The scenario editor's previews during a node drag (SimView._preview_async): one thread, the last scenario
    submitted computed next (the ones in between dropped), each result emitted with its request number. No parent,
    kept in _WORKERS while it runs (as SimWorker)."""
    ready = pyqtSignal(int, object, object, object, float, int, str)

    def __init__(self):
        super().__init__()
        import threading
        self._cv = threading.Condition()
        self._job: Optional[tuple] = None
        self._stop = False
        _WORKERS.add(self)
        self.finished.connect(lambda: _WORKERS.discard(self))

    def submit(self, seq: int, sc: dict, edits: int) -> None:
        with self._cv:
            self._job = (seq, sc, edits)
            self._cv.notify()

    def stop(self) -> None:
        with self._cv:
            self._stop, self._job = True, None
            self._cv.notify()
        self.wait()

    def run(self) -> None:
        while True:
            with self._cv:
                while self._job is None and not self._stop:
                    self._cv.wait()
                if self._stop:
                    return
                (seq, sc, edits), self._job = self._job, None
            t0 = time.perf_counter()
            try:
                truth = preview.preview(sc)
                g = sgeo.lazy_geometry(sc, truth)
                self.ready.emit(seq, sc, truth, g, 1e3 * (time.perf_counter() - t0), edits, "")
            except Exception as exc:                  # (a scenario the model refuses: the former view kept)
                self.ready.emit(seq, sc, None, None, 0.0, edits, f"{type(exc).__name__}: {exc}")


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


def _first_records(tr, n: int):
    """The truth's first n records (views; L5: the instants whose geometry is computed)."""
    return dataclasses.replace(tr, t_s=tr.t_s[:n], r_km=tr.r_km[:n], v_kms=tr.v_kms[:n], att_t_s=tr.att_t_s[:n],
                               att_q=tr.att_q[:n])


@lru_cache(maxsize=None)
def _unit_shape(kind: str) -> Tuple[np.ndarray, np.ndarray]:
    """(points (p, 3), triangles (f, 3)) of a shape of length and diameter 1 along +Z, centred on the origin."""
    if kind == "cylinder":
        m = pv.Cylinder(center=(0.0, 0.0, 0.0), direction=(0.0, 0.0, 1.0), radius=0.5, height=1.0, resolution=16)
    else:
        m = pv.Sphere(radius=0.5, theta_resolution=12, phi_resolution=8)
    m = m.triangulate().clean()
    return np.asarray(m.points, float), m.faces.reshape(-1, 4)[:, 1:].copy()


def _quat_z(q: np.ndarray) -> np.ndarray:
    """(n, 3) the body Z axis in ICRF of Hamilton quaternions [w, x, y, z] (n, 4) body -> ICRF."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.c_[2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)]


def cells(sc: dict) -> float:
    """Rows x steps of a normalised scenario (the cost of a propagation)."""
    n = 1 + len(sc.get("objects", [])) + sum(int(g.get("n", 0)) for g in sc.get("groups", []))
    return n * float(sc["meta"]["duration_s"]) / float(sc["meta"]["step_s"])


def eph_sun_radius_km() -> float:
    return sgeo.SUN_RADIUS_KM


_rgb = v3.rgb


COLOR_CACHE = TEXTURE_DIR / ".orbit_colors.json"   # E2: the maps' dominant colours (local, not in the repository)
_COLOR_DISK: Optional[dict] = None


def _color_key(body: str, path) -> str:
    """What a cached colour depends on: the map (name, size, date) and the rule's values."""
    st = path.stat()
    plain = PLANET_COLORS.get(body, MOON_COLORS.get(body, ""))
    return f"{body}|{path.name}|{st.st_size}|{st.st_mtime_ns}|{ORBIT_MIN_PEAK}|{ORBIT_HUE_SHARE}|{plain}"


@lru_cache(maxsize=None)
def _body_color(body: str) -> str:
    """_body_color_of, read from COLOR_CACHE when its map has not changed (E2, user 8 November 2026: the 51 maps
    decoded for their colour cost 0.8 s of a scenario's opening)."""
    global _COLOR_DISK
    path = TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")
    if not path.exists():
        return _body_color_of(body)
    if _COLOR_DISK is None:
        try:
            with open(COLOR_CACHE, encoding="utf-8") as fh:
                _COLOR_DISK = dict(json.load(fh))
        except (OSError, ValueError, TypeError):
            _COLOR_DISK = {}
    key = _color_key(body, path)
    got = _COLOR_DISK.get(key)
    if isinstance(got, str):
        return got
    col = _body_color_of(body)
    _COLOR_DISK = {k: v for k, v in _COLOR_DISK.items() if not k.startswith(f"{body}|")}
    _COLOR_DISK[key] = col
    try:                                             # (best effort: a read-only folder only costs the time)
        tmp = f"{COLOR_CACHE}.{os.getpid()}.tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(_COLOR_DISK, fh, indent=0, sort_keys=True)
        os.replace(tmp, COLOR_CACHE)
    except OSError:
        pass
    return col


def _body_color_of(body: str) -> str:
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


@lru_cache(maxsize=None)
def _orbit_line_color(body: str) -> str:
    """The colour an orbit is drawn in: its body's at ORBIT_ALPHA over the background, drawn opaque (user, 8 November
    2026: the orbits flickered as the view turned. A translucent line goes through VTK's order-independent
    translucency, outside the window's multisampling: 1 px stairs without antialiasing that crawled at each step of
    a rotation; opaque, the multisampling smooths it, 9 grey levels across the line instead of 1)."""
    c, bg = np.array(_rgb(_body_color(body)), float), np.array(_rgb(BG_COLOR), float)
    return "#%02x%02x%02x" % tuple(int(round(x)) for x in ORBIT_ALPHA * c + (1.0 - ORBIT_ALPHA) * bg)


def _has_texture(body: str) -> bool:
    return (TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")).exists() or body == "moon"


_PIX: dict = {}                                   # E3: maps being decoded ahead, by key ("sun" its photosphere)
_PIX_POOL = None
PREFETCH_WORKERS = 4                              # E3: maps decoded at once (D)


def _map_pixels(body: str) -> Optional[np.ndarray]:
    """A body's map in RGB (an indexed or transparent map would draw wrong), at most TEXTURE_MAX_PX wide; None: no
    map file."""
    path = TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")
    if not path.exists():
        return None
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    im = Image.open(path)
    if im.width > TEXTURE_MAX_PX:
        im.draft("RGB", (TEXTURE_MAX_PX, TEXTURE_MAX_PX // 2))      # (a JPEG decoded at a smaller scale)
        if im.width > TEXTURE_MAX_PX:
            im = im.resize((TEXTURE_MAX_PX, max(1, round(im.height * TEXTURE_MAX_PX / im.width))), Image.BILINEAR)
    return np.asarray(im.convert("RGB"))


def _sun_pixels() -> Optional[np.ndarray]:
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
    return (white + k[..., None] * (warm - white)).clip(0, 255).astype(np.uint8)   # (its dimmer parts warmer, not grey)


def prefetch_maps() -> None:
    """E3 (user, 8 November 2026: 1.5 s of a scenario's opening decoding maps one after the other): the maps the
    global view puts on at once (the Earth, the Moon, the Sun, the planets) decoded in background threads, several at
    a time, from the moment the view is created; each taken (and forgotten here) when first drawn. Once."""
    global _PIX_POOL
    if _PIX_POOL is not None:
        return
    from concurrent.futures import ThreadPoolExecutor
    _PIX_POOL = ThreadPoolExecutor(max_workers=PREFETCH_WORKERS, thread_name_prefix="maps")
    _PIX["sun"] = _PIX_POOL.submit(_sun_pixels)
    for b in ("earth", "moon", *PLANET_NAMES.values()):
        _PIX[b] = _PIX_POOL.submit(_map_pixels, b)


def _pixels(key: str) -> Optional[np.ndarray]:
    """A map's pixels: decoded ahead (waited for if under way), else now."""
    fut = _PIX.pop(key, None)
    if fut is not None:
        try:
            return fut.result()
        except Exception:                            # noqa: BLE001 -- decoded again here, its error shown as before
            pass
    return _sun_pixels() if key == "sun" else _map_pixels(key)


@lru_cache(maxsize=None)
def _texture(body: str) -> Optional[pv.Texture]:
    """Equirectangular texture of a body (TEXTURE_DIR/TEXTURE_FILES[body]; the Moon falls back on the procedural
    one), read once (_map_pixels, decoded ahead by prefetch_maps for the bodies drawn at once); repeated (a shape
    model's seam triangles reach past u = 1)."""
    path = TEXTURE_DIR / TEXTURE_FILES.get(body, f"{body}.jpg")
    if path.exists():
        tex = pv.Texture(_pixels(body))                                  # (pyvista: the first row at the top)
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


def _set_user_matrix(actor, u: np.ndarray) -> None:
    """actor.user_matrix = u (4, 4) at a tenth of its cost: 50 bodies are placed at each frame."""
    m = actor.GetUserMatrix()
    if m is None:
        actor.user_matrix = u
    else:
        m.DeepCopy(u.ravel().tolist())


VBO_NEAR_PLANE_SHIFT_SCALE = 5                   # vtkOpenGLVertexBufferObject::NEAR_PLANE_SHIFT_SCALE (enum not wrapped)


def _near_camera_precision(actor, mesh) -> None:
    """The actor drawn by a polydata mapper whose vertex buffer is shifted to the camera's near plane, rebuilt as the
    camera moves (VTK's default shifts it to the data's centre once: a 4000 km trail kept in float32 about its middle
    is drawn in 0.1 m steps, its segments in all directions seen from a few metres, user 8 November 2026)."""
    from vtkmodules.vtkRenderingCore import vtkPolyDataMapper
    m = vtkPolyDataMapper()
    m.ShallowCopy(actor.GetMapper())                 # (scalars, colour mode, array: as pyvista set them)
    m.SetInputData(mesh)
    m.SetVBOShiftScaleMethod(VBO_NEAR_PLANE_SHIFT_SCALE)
    actor.SetMapper(m)


def _set_points(mesh, pts: np.ndarray) -> None:
    """mesh.points = pts, in place when the count is the same (pyvista's setter costs 0.1 ms: 100 meshes per frame)."""
    data = mesh.GetPoints().GetData()
    view = vtk_to_numpy(data)
    if view.shape != pts.shape:
        mesh.points = pts
        return
    view[:] = pts
    data.Modified()
    mesh.GetPoints().Modified()


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


def _occluded(pts: np.ndarray, eye: np.ndarray, discs: list) -> Tuple[np.ndarray, np.ndarray]:
    """Which points are hidden from eye by a sphere of discs (centre, radius): (mask, distance from eye to the first
    surface on the way to each point, its own distance when none)."""
    d = np.asarray(pts, float) - eye
    dist = np.linalg.norm(d, axis=-1)
    u = d / np.maximum(dist, 1e-300)[..., None]
    entry = dist.copy()
    for c, rad in discs:
        m = np.asarray(c, float) - eye
        along = u @ m
        h2 = rad * rad - (m @ m - along * along)
        first = along - np.sqrt(np.maximum(h2, 0.0))
        hit = (h2 > 0.0) & (first > 0.0) & (first < entry)
        entry = np.where(hit, first, entry)
    return entry < dist, entry


def _cam_axes(cam, size) -> Tuple[np.ndarray, np.ndarray, float]:
    """The camera's position, its axes (rows: right, up, forward) and the world length of one pixel at unit depth."""
    pos = np.asarray(cam.GetPosition(), float)
    fwd = np.asarray(cam.GetDirectionOfProjection(), float)
    up = np.asarray(cam.GetViewUp(), float)
    up = up - (up @ fwd) * fwd
    up /= max(float(np.linalg.norm(up)), 1e-12)
    k = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / max(size[1], 1)
    return pos, np.array([np.cross(fwd, up), up, fwd]), k


def _dashes(pts: np.ndarray, counts: np.ndarray, rgb: np.ndarray, cam, size, on_px: float, off_px: float):
    """Polylines (pts in order, counts points each, rgb per point) cut into dashes of on_px every on_px + off_px on
    screen (user, 30 October 2026), each polyline's dashes counted from its start; the parts out of the view (beyond
    SCREEN_MARGIN, behind the near plane) left out: (points, VTK lines, rgb per point), two points per dash."""
    pos, axes, k = _cam_axes(cam, size)
    q = (np.asarray(pts, float) - pos) @ axes.T                          # camera axes: x right, y up, z depth
    pid = np.repeat(np.arange(len(counts)), counts)
    i0 = np.flatnonzero(pid[:-1] == pid[1:])
    a, b, sp = q[i0], q[i0 + 1], pid[i0]
    t0, t1 = np.zeros(len(i0)), np.ones(len(i0))
    keep = np.ones(len(i0), bool)
    tx, ty = SCREEN_MARGIN * 0.5 * size[0] * k, SCREEN_MARGIN * 0.5 * size[1] * k
    zn = 1e-6 * cam.GetDistance()                     # (not the clipping range: stale before a render)
    for f in (lambda p: p[:, 2] - zn, lambda p: tx * p[:, 2] - p[:, 0], lambda p: tx * p[:, 2] + p[:, 0],
              lambda p: ty * p[:, 2] - p[:, 1], lambda p: ty * p[:, 2] + p[:, 1]):
        fa, fb = f(a), f(b)
        keep &= ~((fa < 0) & (fb < 0))
        with np.errstate(divide="ignore", invalid="ignore"):
            tc = fa / (fa - fb)
        t0 = np.where((fa < 0) & (fb >= 0), np.maximum(t0, tc), t0)
        t1 = np.where((fb < 0) & (fa >= 0), np.minimum(t1, tc), t1)
    keep &= t1 > t0
    p0, p1 = a + t0[:, None] * (b - a), a + t1[:, None] * (b - a)
    z0, z1 = np.where(keep, p0[:, 2], 1.0), np.where(keep, p1[:, 2], 1.0)   # (kept: in front of the camera)
    s0, s1 = p0[:, :2] / z0[:, None] / k, p1[:, :2] / z1[:, None] / k     # px
    ln = np.where(keep, np.linalg.norm(s1 - s0, axis=1), 0.0)
    if not len(ln):
        return np.zeros((0, 3)), None, np.zeros((0, 3))
    base = np.cumsum(ln) - ln
    first = np.r_[True, sp[1:] != sp[:-1]]
    start = base - base[first][np.cumsum(first) - 1]                     # px along its polyline from its start
    per = on_px + off_px
    jmin = np.floor((start - on_px) / per).astype(np.int64) + 1
    jmax = np.ceil((start + ln) / per).astype(np.int64) - 1
    n = np.where(keep & (ln > 0), np.maximum(jmax - jmin + 1, 0), 0)
    if not n.sum():
        return np.zeros((0, 3)), None, np.zeros((0, 3))
    seg = np.repeat(np.arange(len(n)), n)
    j = jmin[seg] + np.arange(int(n.sum())) - np.repeat(np.cumsum(n) - n, n)
    sa = np.clip(j * per - start[seg], 0.0, ln[seg]) / ln[seg]
    sb = np.clip(j * per + on_px - start[seg], 0.0, ln[seg]) / ln[seg]

    def at(s):                                                           # screen share -> point (perspective)
        t = s * z0[seg] / ((1.0 - s) * z1[seg] + s * z0[seg])
        return pos + (p0[seg] + t[:, None] * (p1[seg] - p0[seg])) @ axes

    out = np.empty((2 * len(seg), 3))
    out[0::2], out[1::2] = at(sa), at(sb)
    return out, _pairs_seq(len(seg)), np.repeat(np.asarray(rgb)[i0][seg], 2, axis=0)


def _pairs_seq(n: int) -> np.ndarray:
    """Lines of segment j from point 2j to point 2j + 1."""
    return np.column_stack([np.full(n, 2), 2 * np.arange(n), 2 * np.arange(n) + 1]).ravel()


def _arrowheads(tips: np.ndarray, back: np.ndarray, rgb: np.ndarray, cam, size, px: float):
    """Two wings of px on screen at each tip, ARROW_DEG from the shaft coming from back, in the screen's plane."""
    pos, axes, k = _cam_axes(cam, size)
    tips, back = np.asarray(tips, float), np.asarray(back, float)
    view = tips - pos
    depth = view @ axes[2]
    ok = depth > 0
    vv = view / np.maximum(np.linalg.norm(view, axis=1), 1e-300)[:, None]
    d = tips - back
    d = d - np.sum(d * vv, axis=1)[:, None] * vv
    dn = np.linalg.norm(d, axis=1)
    ok &= dn > 0
    d = d / np.maximum(dn, 1e-300)[:, None]
    side = np.cross(d, vv)
    ln = (px * k * depth)[:, None]
    c, s = math.cos(math.radians(ARROW_DEG)), math.sin(math.radians(ARROW_DEG))
    tips, d, side, ln, rgb = tips[ok], d[ok], side[ok], ln[ok], np.asarray(rgb)[ok]
    out = np.empty((4 * len(tips), 3))
    out[0::4], out[1::4] = tips, tips - ln * (c * d + s * side)
    out[2::4], out[3::4] = tips, tips - ln * (c * d - s * side)
    return out, _pairs_seq(2 * len(tips)), np.repeat(rgb, 4, axis=0).astype(np.uint8)


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


def _view_button_style(color: str) -> str:
    """A view's button: its colour always (user, 30 October 2026), pale, full while its view is shown."""
    r, g, b = _rgb(color)
    return (f"QToolButton {{ background: rgba({r}, {g}, {b}, {VIEW_BUTTON_IDLE:.0%}); border: 1px solid {color}; "
            f"border-radius: 3px; padding: 3px 8px; }} "
            f"QToolButton:checked {{ background: {color}; color: white; }}")


_row, _pair, _title = v3.row, v3.pair, v3.title   # the options panel (shared: gui/view3d.py)


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
    img = _pixels("sun")
    if img is None:
        return None
    tex = pv.Texture(img)
    tex.interpolate = True
    tex.mipmap = True
    return tex


def _place_halo(p, act, centre, radius_km: float) -> None:
    """The halo around the Sun drawn at centre with radius_km (its scene units)."""
    act.SetCamera(p.camera)
    act.SetPosition(*(float(x) for x in centre))
    act.SetScale(float(radius_km))


_Labels = v3.Labels                               # labels at 3D points (shared: gui/view3d.py)


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


symbol_path, _symbol_texture = v3.symbol_path, v3.symbol_texture   # (shared: gui/view3d.py)


def _path_stride(truth, n_records: float) -> int:
    """Every how many truth records the global view's paths are drawn: at most PATH_TURN_DEG of turn per segment for
    the fastest-turning body (each body's median turn per record: an impulse is not a turn), at least
    PATH_MAX_PTS points per path, at most PATH_BUDGET_PTS for all the paths."""
    count = max(1, math.ceil(n_records / PATH_MAX_PTS))
    try:
        v = np.asarray(truth.v_kms, float)
        if len(v) < 3:
            return count
        k = np.arange(0, len(v) - 1, max(1, (len(v) - 1) // 2000))       # (a sample of the records: periodic paths)
        a, b = v[k], v[k + 1]
        na, nb = np.linalg.norm(a, axis=2), np.linalg.norm(b, axis=2)
        ok = (na > 1e-9) & (nb > 1e-9)
        cosang = np.einsum("ijk,ijk->ij", a, b) / np.where(ok, na * nb, 1.0)
        ang = np.where(ok, np.degrees(np.arccos(np.clip(cosang, -1.0, 1.0))), np.nan)
        if not np.isfinite(ang).any():
            return count
        with np.errstate(all="ignore"):
            per_body = np.nanmedian(ang, axis=0)                  # (each body's usual turn: an impulse is not one)
        if not np.isfinite(per_body).any():
            return count
        per_rec = float(np.nanmax(per_body))                      # (deg of turn per record, the fastest body)
        turn = max(1, int(PATH_TURN_DEG / max(per_rec, 1e-9))) if per_rec > 0 else count
        budget = max(1, math.ceil(n_records * v.shape[1] / PATH_BUDGET_PTS))
        return max(1, min(count, max(turn, budget)))
    except (AttributeError, ValueError, IndexError):
        return count


def _segments(a: np.ndarray, b: np.ndarray) -> pv.PolyData:
    pts = np.empty((2 * len(a), 3))
    pts[0::2], pts[1::2] = a, b
    lines = np.hstack([[2, 2 * i, 2 * i + 1] for i in range(len(a))])
    return pv.PolyData(pts, lines=lines)


class SimView(QWidget):
    """3D views (S6) and playback (S7) of the global simulator."""
    run_shown = pyqtSignal()                      # a run (propagation or journal) is shown
    picked = pyqtSignal(str)                      # a body clicked in the view (OWNSHIP, an object id, a multitrack)
    # an event's mark clicked that belongs to a manoeuvre (user, 8 November 2026): its owner's kind ("ship",
    # "objects", "groups"), id ("ship" for the ship) and index in the owner's manoeuvres (after picked)
    manoeuvre_picked = pyqtSignal(str, str, int)

    def __init__(self, grid: Optional[cgeo.SphereGrid] = None, get_path: Callable[[], Optional[str]] = lambda: None,
                 parent=None, read_only: bool = False):
        """read_only: a run looked at, never edited (the mission preparation's global view, user 10 November 2026):
        no manoeuvre node."""
        super().__init__(parent)
        self.read_only = read_only
        self.range_gauge = False                  # the mission preparation's range gauge (user, 10 November 2026)
        prefetch_maps()                                # E3: the maps decoded while the window opens
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
        # state and "Propagate" in the window's status bar (status_widgets); "Open run..." and "Record run..." in the window's menu
        # (file_commands); the playback bar under the view
        # three views side by side (user, 30 October 2026: the orbital view a third one beside the two)
        self.view_mode = "global"
        self.view_buttons: Dict[str, QToolButton] = {}
        self._view_group = QButtonGroup(self)
        self._view_group.setExclusive(True)
        for mode, text, tip in VIEWS:
            b = QToolButton()
            b.setText(text)
            b.setCheckable(True)
            b.setChecked(mode == self.view_mode)
            b.setToolTip(tip)
            b.setStyleSheet(_view_button_style(VIEW_COLORS[mode]))
            b.clicked.connect(lambda _c=False, m=mode: self.set_view(m))
            self._view_group.addButton(b)
            self.view_buttons[mode] = b
        # always there, greyed unless a large scenario changed (user, 30 October 2026), in the status bar
        self.bt_prop = uc.tool_button("Propagate", self.propagate, "SP_BrowserReload", tip=PROP_TIP_IDLE)
        self.bt_prop.setEnabled(False)
        self._launched: Optional[dict] = None     # the scenario of the last propagation started
        self.bt_prop.setVisible(False)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setMaximumWidth(160)
        self.progress.setFormat("%p %")
        self.status = QLabel("No propagation yet")

        # ---- L3 / V3 / V5 toggles (their states at start: the shown view's profile, _apply_profile)
        # the names of the celestial bodies apart from the objects' data boxes (user, 30 October 2026); the boxes whole
        # (full), or only the id, the range and the closing speed (partial)
        self.cb_celestial_labels = uc.Toggle("celestial labels", tip="The names of the celestial bodies")
        self.cb_object_labels = uc.Toggle("object labels", tip="The objects' data boxes and the sectors' names")
        self.tg_object_full = uc.Toggle("full", two_way=True,
                                        tip="Object labels: the whole data box (full), or the id, the range and the "
                                            "closing speed only (partial)")
        self.lb_object_partial = QLabel("partial")
        self.cb_object_labels.toggled.connect(self.tg_object_full.setEnabled)
        self.cb_object_labels.toggled.connect(self.lb_object_partial.setEnabled)
        self.tg_object_full.toggled.connect(lambda _on: self._draw_frame())
        self.cb_vectors = uc.Toggle("noses", tip="Ship view: each object's nose (its Z axis)")
        self.cb_trails = uc.Toggle("trails")
        # user, 29 October 2026: the trails' duration, the planned trajectories, the orbital velocity vectors
        self.sp_trail = _value_box(0.0, 0.0, 1e7, 0, " s", "Flight time drawn by each trail, s")
        self.cb_planned = uc.Toggle("planned trajectory", tip="Each body's trajectory from t to the run's end")
        self.cb_orbit = uc.Toggle("orbital velocity vector", tip="Each body's path from its state at t under the "
                                  "local gravity alone (no burn, no guidance) over the duration beside (ship view: "
                                  "relative to the ship's own free fall)")
        self.sp_orbit = _value_box(1.0, 1.0, 1e6, 0, " s",
                                   "Length of the orbital velocity vector: the free fall over this time, s")
        for cb, sp in ((self.cb_trails, self.sp_trail), (self.cb_orbit, self.sp_orbit)):
            cb.toggled.connect(sp.setEnabled)
            sp.valueChanged.connect(lambda _x: self._draw_frame())
        # the body the view is centred on and follows: chosen by a double-click on it in the view (an object,
        # OWNSHIP, or in the global view the Earth, the Moon, the Sun), back to the default by a double-click on
        # nothing (user, 29 October 2026: no "centre on" list)
        self.centre = CENTRAL
        self.cb_volumes = uc.Toggle("mission volumes")
        self.cb_bodies = uc.Toggle("celestial objects")
        self.cb_bodies.setToolTip("Ship view: the Earth, the Moon and the Sun at their true directions, "
                                  "distances and sizes")
        self.cb_bodies.toggled.connect(lambda _on: self._draw_frame())
        for cb in (self.cb_celestial_labels, self.cb_object_labels, self.cb_vectors, self.cb_trails, self.cb_planned,
                   self.cb_orbit):
            cb.toggled.connect(lambda _on: self._draw_frame())
        self.cb_object_labels.toggled.connect(lambda _on: (self._place_volumes(), self._render()))   # the sectors' labels
        self.cb_volumes.toggled.connect(lambda _on: self._draw_volumes(force=True))
        self.cb_proj = uc.Toggle("range projection")
        self.cb_proj.setToolTip("Ship view: a copy of each object in the plane of the range rings, at its range from "
                                "the ship (read on the rings), linked to the object by the arc of the sphere of that "
                                "range around the ship")
        self.cb_proj.toggled.connect(lambda _on: self._draw_frame())
        # partial: the objects and their copies in the plane, linked by the dashed arcs; full (user, 29 October
        # 2026): the copies only, a 2D view in the plane of the rings (the objects, their arcs not drawn)
        self.tg_proj_full = uc.Toggle("full", two_way=True,
                                      tip="Range projection: the objects and their copies (partial), or the copies "
                                          "only, every body and path in the plane of the rings (full)")
        self.lb_proj_partial = QLabel("partial")
        self.cb_proj.toggled.connect(self.tg_proj_full.setEnabled)
        self.cb_proj.toggled.connect(self.lb_proj_partial.setEnabled)
        self.tg_proj_full.toggled.connect(lambda _on: self._draw_frame())
        # D: the faces' responsibility zones (ship view), one switch for the four (user, 29 October 2026)
        self.cb_zones = uc.Toggle("responsibility zones")
        self.cb_zones.toggled.connect(lambda _on: (self._show_faces(), self._render()))
        # the axes the ship carries (user, 29 October 2026): its X, Y, Z axes and the faces' (arrays') normals of
        # the model, the 1:1 ship axes, the attitude triad of the global view
        self.cb_los = uc.Toggle("array lines", tip="Ship view: a line from the ship to each object, in the "
                                "colour of the array tracking it (masked, occulted or out of every array: grey)")
        self.cb_los.toggled.connect(lambda _on: self._draw_frame())   # (user, 30 October 2026)
        self.cb_events = uc.Toggle("events", tip="Global view: the events' marks and their texts on the paths")
        self.cb_events.toggled.connect(lambda _on: self._draw_frame())
        self.cb_axes = uc.Toggle("ship axes")
        self.cb_axes.toggled.connect(lambda _on: self._draw_frame())
        # the objects drawn by their symbol (classification frame, characterisation glyph), else by a dot (user)
        self.cb_symbols = uc.Toggle("symbols")
        self.cb_symbols.toggled.connect(lambda _on: self._draw_frame())
        # every celestial body's orbit, whatever the selection (user, 30 October 2026; the global view)
        self.cb_all_orbits = uc.Toggle("all orbits", tip="Global view: the orbits of every planet and moon, "
                                       "whatever body is selected")
        self.cb_all_orbits.toggled.connect(lambda _on: self._draw_frame())
        # the star background (user, 29 October 2026): the real sky (skybox) or the plain background; the
        # constellation lines on demand
        self.tg_sky = uc.Toggle("skybox",
                                tip="The stars at their true directions (Yale Bright Star Catalogue), or the plain "
                                    "background")
        self.cb_const = uc.Toggle("constellations", tip="The lines of the 88 constellations over the stars")
        self.tg_sky.toggled.connect(self.cb_const.setEnabled)
        for cb in (self.tg_sky, self.cb_const):
            cb.toggled.connect(lambda _on: self._show_sky())
        # each view its own settings (user, 30 October 2026, replaces one setting for every view): the profiles at
        # start (VIEW_PROFILES), each one kept while another view is shown; the UI size stays common
        self._profiles = {mode: dict(prof) for mode, prof in VIEW_PROFILES.items()}
        self._profile_widgets = {
            "skybox": self.tg_sky, "constellations": self.cb_const,
            "celestial_labels": self.cb_celestial_labels, "object_labels": self.cb_object_labels,
            "object_labels_full": self.tg_object_full, "symbols": self.cb_symbols, "events": self.cb_events, "all_orbits": self.cb_all_orbits, "planned_trajectory": self.cb_planned,
            "trails": self.cb_trails, "trail_s": self.sp_trail, "orbital_vector": self.cb_orbit,
            "orbital_vector_s": self.sp_orbit, "ship_axes": self.cb_axes, "noses": self.cb_vectors,
            "mission_volumes": self.cb_volumes, "responsibility_zones": self.cb_zones,
            "celestial_objects": self.cb_bodies, "array_lines": self.cb_los, "range_projection": self.cb_proj,
            "range_projection_full": self.tg_proj_full}
        # the same columns in both sections (user, 30 October 2026): each switch at the same place in its cell, a pair
        # on a whole line, the durations under one another, the pairs' two-way switches in the second column
        wide = max(self.cb_trails.sizeHint().width(), self.cb_orbit.sizeHint().width())
        self.cb_trails.setMinimumWidth(wide)
        self.cb_orbit.setMinimumWidth(wide)
        self._common = (_row(self.tg_sky), _row(self.cb_const), _row(self.cb_celestial_labels), _row(self.cb_symbols),
                        _pair(self.cb_object_labels, self.lb_object_partial, self.tg_object_full),
                        _row(self.cb_planned), _row(self.cb_all_orbits),
                        _row(self.cb_trails, self.sp_trail), _row(self.cb_axes),
                        _row(self.cb_orbit, self.sp_orbit), _row(self.cb_events), _row(self.cb_volumes))
        proj_pair = _pair(self.cb_proj, self.lb_proj_partial, self.tg_proj_full)
        self._ship_only = (_row(self.cb_vectors), _row(self.cb_zones), _row(self.cb_bodies),
                           _row(self.cb_los)) + proj_pair
        # UI size (user, 29 October 2026): the icons and texts of the 3D view, - / + (shared: gui/view3d.py)
        self.ui_scale = UI_SCALE_START
        self.ui_size = v3.UiSize(self)
        self.ui_size.changed.connect(self._ui_scale_changed)
        self.bt_ui_minus, self.bt_ui_plus, self.lb_ui = self.ui_size.minus, self.ui_size.plus, self.ui_size.label
        self._array_view = np.zeros(3)                       # the apertures in the view's frame (_frame_ship)
        self.bt_recentre = v3.recenter_button(self.recentre, "Global view: back to the central body or the body double-clicked, level with "
                                    "the central body's equator; zoom and direction of view kept. Ship view: back "
                                    "to the view at start. Orbital view: on the ship, from the zenith, 0 deg up, "
                                    "90 deg to the right; zoom kept")

        self.plotter = QtInteractor(self)
        self.plotter.setMinimumWidth(400)
        # right drag: turntable about the view's vertical; the left button picks and centres (user, 29 October 2026)
        v3.install_rotation(self.plotter, axis=self._turn_axis, on_click=self._on_right_click)
        self.target_key: Optional[str] = None          # the target's selection key (_rows_of)
        self._glyphs: Dict[str, tuple] = {}
        self._poly_color: Dict[str, Optional[str]] = {}  # the colour each dynamic actor has (_poly)
        self._body_knots: tuple = (None, {})            # the ephemerides at the knots around t (_bodies_at)
        self._moon_mats: Dict[str, np.ndarray] = {}     # each moon's last turn (_ship_bodies)
        self._mt_codes: tuple = (None, np.zeros(0, int), [])   # (run, each row's code of identity and class, identities)
        self._orbit_last: Dict[str, tuple] = {}         # each orbit as last drawn (_orbit_kept)
        self._clip_state = (-1, 0)                      # the camera's time at the last clipping, frames since (_clip)
        self._box_slots: Dict[tuple, int] = {}          # each data box's place (_place_boxes)
        self.plotter.iren.interactor.AddObserver("LeftButtonPressEvent", self._on_click, 1.0)
        # the manoeuvre node (user, 29 October 2026): its own presses before the click above
        from gui.maneuver_node import NodeTool
        self.node = NodeTool(self)
        self.node.enabled = not self.read_only
        self.node.install(self.plotter.iren.interactor)
        self.plotter._ui_scale = self.ui_scale
        self.sky = v3.space_background(self.plotter)       # the black of space, the stars under the scene
        self._apply_profile(self.view_mode)
        self.plotter.renderer.SetNearClippingPlaneTolerance(NEAR_TOL)
        self.plotter.renderer.AddObserver("StartEvent", lambda *_a: self._clip())
        # a frame-rate counter at the top right (user, 30 October 2026): the renders of the last second
        self._fps_times: List[float] = []
        self._add_fps()
        self.plotter.renderer.AddObserver("StartEvent", lambda *_a: self._fps_tick())
        self.plotter.add_axes(color=TEXT_COLOR)
        self.plotter.renderer.GetActiveCamera().AddObserver("ModifiedEvent", lambda *_a: self._on_camera())
        # user, 29 October 2026: the view's options under the Info box, in the side column (no longer in the
        # playback bar), one per line; the layers as on / off switches (the ship view's own ones and their title
        # shown in the ship view only)
        self._ship_titles = (_title("Ship view display options"),)   # (named by _show_view_options)
        self.view_panel = QGroupBox("3D view")
        vcol = QVBoxLayout(self.view_panel)
        vcol.setContentsMargins(8, 4, 8, 6)
        vcol.setSpacing(4)
        head = QHBoxLayout()
        for b in self.view_buttons.values():
            head.addWidget(b)
        head.addStretch(1)
        head.addWidget(self.bt_recentre)
        vcol.addLayout(head)
        vcol.addWidget(self.ui_size.widget())
        vcol.addLayout(v3.options_grid(((_title("General display options"), self._common),
                                        (self._ship_titles[0], self._ship_only[:-3] + (proj_pair,)))))

        # ---- L1 playback
        play = QHBoxLayout()
        self.bt_back = QPushButton("|<")
        self.bt_back.clicked.connect(lambda: self.step(-1))
        self.bt_play = QPushButton()                  # the symbols, not the words (user, 30 October 2026)
        self.bt_play.setIcon(uc.std_icon(self.bt_play, "SP_MediaPlay"))
        self.bt_play.setToolTip("Play / pause (Space)")
        self.bt_play.clicked.connect(self.toggle_play)
        self.bt_fwd = QPushButton(">|")
        self.bt_fwd.clicked.connect(lambda: self.step(1))
        for b in (self.bt_back, self.bt_play, self.bt_fwd):        # one size (user, 30 October 2026)
            b.setFixedSize(PLAY_BUTTON_PX, self.bt_back.sizeHint().height())
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
        self._valid = True                            # the editor's scenario passes its validation (set_valid)

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
        self.load_timer = QTimer(self)                # a scenario just loaded: at the next turn of the event loop
        self.load_timer.setSingleShot(True)
        self.load_timer.setInterval(0)
        self.load_timer.timeout.connect(self.propagate)
        # the scenario editor (fork RADARv6_RS, user 8 November 2026): while editing, the scenario's preview
        # (sim/preview, approximate, computed at once) shown at each change, a node drag included; once validated, the
        # real computation (sim/run, its journal the radar's entry point) in a separate process, then its run shown
        self._live = False                            # a scenario was loaded: its changes are previewed
        self._mode: Optional[str] = None              # what is shown: "preview" or "run" (a journal)
        self._preview_sc: Optional[dict] = None       # the scenario waiting for its preview (one per event-loop turn)
        self._preview_ms = 0.0                        # the last preview's cost
        self.preview_timer = QTimer(self)
        self.preview_timer.setSingleShot(True)
        self.preview_timer.setInterval(0)
        self.preview_timer.timeout.connect(self._do_preview)
        self._preview_seq = 0                         # each preview asked; a background one shown only if newer than
        self._preview_shown = 0                       # ... the one shown (its number)
        self._pworker: Optional[_PreviewWorker] = None   # a node drag's previews (_preview_async)
        self._fleet_wall = 0.0                        # ... the last one drawn (wall clock), the one waiting its turn
        self._fleet_held: Optional[tuple] = None
        self.fleet_timer = QTimer(self)
        self.fleet_timer.setSingleShot(True)
        self.fleet_timer.timeout.connect(self._fleet_due)
        self.scenario_source: Optional[Callable[[], dict]] = None   # the editor's scenario now (a node drag)
        self.calc: Optional[QProcess] = None          # the real computation under way
        self._calc_path: Optional[str] = None
        self._reframe_next = True                     # the next run shown frames the camera (a file loaded)
        self._enable_playback(False)

    # ------------------------------------------------------------------ V2 computations
    def set_scenario(self, sc: dict) -> None:
        """The editor's scenario after a valid change (normalised); a change of object or group names, or of
        attributes with no effect on the physics (classification, scenario name), only updates the run shown,
        without propagating again (user, 29 October 2026)."""
        self.pending = sc
        if self.truth is not None and self.sc is not None:
            names = _renaming(self.sc, sc)             # no effect on the paths (classification, names): the view
            if names is not None:                      # at once (user, 29 October 2026), nothing computed again
                self._rename(sc, names)
                return
        self._stop_calc()                              # (a real computation of the former scenario: stale)
        if self._live:
            self._preview_soon(sc)

    def _update_prop(self, busy: Optional[bool] = None) -> None:
        """"Propagate" active only when the user has to act: a large scenario, valid, changed since its last
        propagation, none under way."""
        self.bt_prop.setEnabled(False)                # (nothing to start: the preview follows the edits)

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
        """The preview of the scenario now."""
        if self.pending is not None:
            self._live = True
            self._preview_soon(self.pending)

    def _preview_soon(self, sc: dict) -> None:
        """The preview of sc at the next turn of the event loop (several changes meanwhile: the last one)."""
        self._preview_sc = sc
        if not self.preview_timer.isActive():
            self.preview_timer.start()

    def _do_preview(self) -> None:
        """The scenario's preview computed and shown (sim/preview: approximate, user 8 November 2026)."""
        sc, self._preview_sc = self._preview_sc, None
        if sc is None or getattr(self.plotter, "_closed", False):
            return
        self._preview_seq += 1                         # (a node drag's preview still under way: stale)
        self._preview_shown = self._preview_seq
        t0 = time.perf_counter()
        try:
            truth = preview.preview(sc)
        except Exception as exc:                       # (a scenario the model refuses: the former view kept)
            self.status.setText(f"preview failed: {type(exc).__name__}: {exc}")
            return
        g = sgeo.lazy_geometry(sc, truth)
        self._show_preview(sc, truth, g, 1e3 * (time.perf_counter() - t0), self.node.edits)

    def _preview_async(self, sc: dict) -> None:
        """During a node drag (user, 8 November 2026: the node lagged behind the mouse, the whole preview computed
        and drawn at each mouse event, 0.4 s): the preview computed in a background thread, the last scenario sent
        winning, shown when ready; the node follows the mouse meanwhile."""
        self._preview_seq += 1
        if self._pworker is None:
            self._pworker = _PreviewWorker()
            self._pworker.ready.connect(self._preview_ready)
            self._pworker.start()
        self._pworker.submit(self._preview_seq, sc, self.node.edits)

    def _preview_ready(self, seq: int, sc: dict, truth, g, ms: float, edits: int, err: str) -> None:
        if seq <= self._preview_shown or getattr(self.plotter, "_closed", False):
            return                                     # (older than the one shown, or than the release's own preview)
        if err:
            self._preview_shown = seq
            self.status.setText(f"preview failed: {err}")
            return
        wait = 1.0 / DRAG_FLEET_HZ - (time.perf_counter() - self._fleet_wall)
        if self.node.dragging() and wait > 0.0:        # the fleet drawn DRAG_FLEET_HZ times a second at most: the
            self._fleet_held = (seq, sc, truth, g, ms, edits)   # mouse's other events move the node alone
            if not self.fleet_timer.isActive():
                self.fleet_timer.start(int(math.ceil(1e3 * wait)))
            return
        self._fleet_held = None
        self._preview_shown = seq                      # (newer edits may be under way: shown meanwhile, behind them)
        self._fleet_wall = time.perf_counter()
        self._show_preview(sc, truth, g, ms, edits)

    def _fleet_due(self) -> None:
        held, self._fleet_held = self._fleet_held, None
        if held is not None and held[0] > self._preview_shown and not getattr(self.plotter, "_closed", False):
            self._preview_shown = held[0]
            self._fleet_wall = time.perf_counter()
            self._show_preview(*held[1:])

    def _show_preview(self, sc: dict, truth, g, ms: float, edits: int) -> None:
        self._preview_ms = ms
        same = (self.truth is not None and list(self.truth.ids) == list(truth.ids)
                and self.truth.body.name == truth.body.name and self._mode == "preview")
        self._mode = "preview"
        self.show_run(sc, truth, g, "preview", edits=edits, rebuild=not same)
        self.status.setText(f"preview (approximate): {len(g.ids)} objects, {float(sc['meta']['duration_s']):g} s, "
                            f"{self._preview_ms:.0f} ms")

    def _run_complete(self) -> bool:
        """The run shown is whole (a preview is computed in one go, a journal read whole)."""
        return True

    def node_edited(self, final: bool) -> None:
        """The manoeuvre node changed a value (user, 8 November 2026): the whole preview follows during a drag, in the
        background (every body reading the edited one too: a KKV's interception); at the release the editor's
        validation sends the scenario (set_scenario)."""
        if final or not self._live or self.scenario_source is None:
            return
        try:
            sc = self.scenario_source()
        except Exception:                              # (an editor state that does not normalise: wait)
            return
        self._preview_async(sc)

    def propagate_loaded(self) -> None:
        """A scenario just created or opened: its preview at once (user, 30 October 2026); not for a scenario with
        problems (pending would still be the former one)."""
        self._reframe_next = True                     # (its run framed by the camera, the next edits keep it)
        self._stop_calc()
        if self._valid and self.pending is not None:
            self._live = True
            self._preview_soon(self.pending)

    def stop_background(self) -> None:
        """The window closes: the playback, the preview to come and the real computation stopped before the view
        goes (a frame drawn without its render window)."""
        self._playing = False
        self.timer.stop()
        self.load_timer.stop()
        self.debounce.stop()
        self.preview_timer.stop()
        self.fleet_timer.stop()
        if self._pworker is not None:
            self._pworker.stop()
            self._pworker = None
        self._stop_calc()
        self._queued = None
        if self.worker is not None and self.worker.isRunning():
            self.worker.cancelled = True
            self.worker.wait()

    # ------------------------------------------------------------------ the real computation (validated scenario)
    def run_journal(self) -> None:
        """The validated scenario's real computation (sim/run: the full model, its journal written next to the
        scenario, the radar's entry point), in a separate process (the view stays fluid), then its run shown."""
        path = self.get_path()
        if path is None:
            QMessageBox.information(self, "Simulator", "Save the scenario first: the journal is written next to it.")
            return
        if not self._valid:
            QMessageBox.information(self, "Simulator", "The scenario has problems: correct them before computing it.")
            return
        self._stop_calc()
        proc = QProcess(self)
        proc.setProgram(sys.executable)
        proc.setArguments(["-u", "-m", "sim.run", path])
        proc.setWorkingDirectory(APP_DIR)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(self._calc_output)
        proc.finished.connect(self._calc_finished)
        self.calc, self._calc_path = proc, path
        self.status.setText("real computation: starting...")
        self.progress.setValue(0)
        proc.start()

    def _calc_output(self) -> None:
        """The computation's progress (lines "<stage> <percent> %" ended by a carriage return, sim/run.main)."""
        if self.calc is None:
            return
        text = bytes(self.calc.readAllStandardOutput()).decode("utf-8", "replace")
        self._calc_log = (getattr(self, "_calc_log", "") + text)[-2000:]
        parts = [x.strip() for x in text.replace("\n", "\r").split("\r") if x.strip()]
        if not parts:
            return
        last = parts[-1].split()
        if len(last) >= 3 and last[-1] == "%":
            try:
                f = float(last[-2]) / 100.0
            except ValueError:
                return
            self.progress.setValue(int(round(1000 * f)))
            self.status.setText(f"real computation: {last[0]} {100 * f:.0f} %")

    def _calc_finished(self, code: int, _status=None) -> None:
        proc, path = self.calc, self._calc_path
        self.calc = None
        if proc is None or path is None:
            return
        if code != 0:
            tail = getattr(self, "_calc_log", "").strip().splitlines()
            self.status.setText(f"real computation failed ({code}): {tail[-1] if tail else ''}")
            return
        try:
            sc, truth, g, _nav, _meta = engine.load_journal(engine.journal_paths(path)[0])
        except (OSError, ValueError, KeyError) as exc:
            self.status.setText(f"real computation: its journal could not be read: {exc}")
            return
        same = (self.truth is not None and list(self.truth.ids) == list(truth.ids)
                and self.truth.body.name == truth.body.name)
        self._mode = "run"
        self.show_run(sc, truth, g, "computed run", rebuild=not same)

    def _stop_calc(self) -> None:
        """The real computation under way stopped (the scenario changed, the window closes)."""
        proc, self.calc = self.calc, None
        self._calc_log = ""
        if proc is not None:
            try:
                proc.finished.disconnect()
                proc.readyReadStandardOutput.disconnect()
            except TypeError:
                pass
            proc.kill()
            proc.waitForFinished(2000)

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
        self._reframe_next = True
        self._stop_calc()
        self._mode = "run"
        self.show_run(sc, truth, g, f"journal {path}")

    def _start(self, job: Callable, what: str) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.worker.cancelled = True
            self._queued = (job, what)
            return
        self.worker = SimWorker(job)
        self.worker.progress.connect(lambda st, f: (self.progress.setValue(int(1000 * f)),
                                                    self.status.setText(f"{st}...")))
        n = self.node.edits                           # the manoeuvres' changes this run takes in
        self.worker.done.connect(lambda res, n=n: self.show_run(*res, label=what, edits=n))
        self.worker.failed.connect(self._on_failed)
        self.worker.finished.connect(self._after_worker)
        self.status.setText(f"{what}...")
        self.worker.start()

    def _on_failed(self, msg: str) -> None:
        self.status.setText(f"failed: {msg}")
        self._launched = None                         # (still to propagate)

    def _after_worker(self) -> None:
        if self._queued is not None:
            job, what = self._queued
            self._queued = None
            self.worker = None
            self._start(job, what)
        else:
            self._update_prop(busy=False)

    def show_run(self, sc: dict, truth, g: sgeo.Geometry, label: str = "", edits: Optional[int] = None,
                 rebuild: bool = True) -> None:
        """Display a propagated run (from the worker, a journal or the live front); edits: the manoeuvres' changes it
        takes in (the node's count when it was launched; None: all of them); rebuild False (L5: the same bodies and
        central body as the run shown, an edit taken up by the front): the scene kept, its data replaced."""
        if getattr(self.plotter, "_closed", False):    # (the view closed while the worker ran)
            return
        self._run_edits = self.node.edits if edits is None else edits
        self._live_cache = None
        # the camera framed again only for a file loaded, a journal or another central body: a propagation after an
        # edit (the manoeuvre node, a form) keeps the zoom and the place chosen (user, 8 November 2026)
        keep = (not self._reframe_next and self.truth is not None and truth is not None
                and self.truth.body.name == truth.body.name)
        self._reframe_next = False
        self.sc, self.truth, self.g = sc, truth, g
        self._mt_input = None                          # (rows of the former run: fewer objects now, until the next frame)
        self._cpa_cache = {}                          # the closest approaches of the run shown
        # global view: the paths through every n-th truth record, n fixed for the run (no shimmer while playing)
        n_all = float(sc["meta"]["duration_s"]) / prop.LOG_STEP_S + 1.0      # (L5: the live run grows)
        self._path_stride = _path_stride(truth, max(len(truth.t_s), n_all))
        self._coast_third = None                      # the other bodies' grid of the orbital vectors (_coast)
        self.progress.setValue(1000)
        self.status.setText(f"{label}: {len(g.ids)} objects, {float(sc['meta']['duration_s']):g} s")
        self.t = min(self.t, self.duration())
        rng = g.range_km[np.isfinite(g.range_km)]
        self._r_far = max(float(rng.max()), 1e-3) if rng.size else 10.0
        self._navigation = engine.Navigation(sc, truth)
        if rebuild or not self.node.dragging():       # the node's preview from the new run (during a drag, kept: it
            self.node.invalidate()                    # starts before the edited manoeuvre, where the runs are alike)
        self._triad_rgb = np.array([_rgb(cgeo.COLORS[k]) for k in ("axis_x", "axis_y", "axis_z")], np.uint8)
        self._colour_rows()
        self._fill_lists()
        self._fill_events()
        self._enable_playback(True)
        if rebuild:
            self._rebuild(keep_camera=keep)
        else:
            self._compute_markers()
            self._draw_frame()
            self._draw_volumes()
        self.run_shown.emit()

    # ------------------------------------------------------------------ time
    def duration(self) -> float:
        """The run's length: the scenario's, or up to the ship's impact when its manoeuvres crash it (the run ends
        there, user 29 October 2026)."""
        if not self.sc:
            return 0.0
        d = float(self.sc["meta"]["duration_s"])
        return min(d, float(self.truth.t_s[-1])) if self.truth is not None and len(self.truth.t_s) else d

    def span(self) -> float:
        """The slider's length (L5): the scenario's, or the run's once complete (it may end at the ship's impact)."""
        return self.duration() if self._run_complete() else self.scenario_duration()

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
        for w in (self.bt_back, self.bt_fwd, self.slider, self.cb_speed):
            w.setEnabled(on)
        self.bt_play.setEnabled(on and self._valid)

    def set_valid(self, ok: bool) -> None:
        """The editor's validation (user, 30 October 2026): a scenario with problems cannot be played, Play greyed
        and any playback stopped; step and slider still show the last run."""
        self._valid = ok
        self._update_prop()
        if self._playing and not ok:
            self.toggle_play()
        self.bt_play.setEnabled(ok and self.truth is not None)

    def set_time(self, t: float) -> None:
        self.t = float(np.clip(t, 0.0, self.duration()))
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(10000 * self.t / max(self.span(), 1e-9))))
        self.slider.blockSignals(False)
        self._draw_frame()
        if not self._playing:
            self._draw_volumes()

    def _on_slider(self, v: int) -> None:
        if self.sc is not None:
            self.set_time(v / 10000.0 * self.span())    # (L5: beyond the front, held at it)

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
            self.bt_play.setIcon(uc.std_icon(self.bt_play, "SP_MediaPlay"))
            self.bt_play.repaint()                    # (the pause shown at once, user 11 November 2026)
            QTimer.singleShot(0, self._after_pause)
        elif self.sc is not None and self._valid:     # (the Space shortcut ignores the greyed button)
            if self.t >= self.duration() and self._run_complete():
                self.t = 0.0
            self._playing = True
            self._wall = time.perf_counter()
            self.timer.start(0)
            self.bt_play.setIcon(uc.std_icon(self.bt_play, "SP_MediaPause"))

    def _after_pause(self) -> None:
        if not self._playing:
            self._draw_frame()                        # (the data boxes' texts at t)
            self._draw_volumes()

    def _tick(self) -> None:
        """One frame; the next is scheduled after it with at least MIN_IDLE_MS left to the event loop, so the mouse
        (zoom, rotation) stays live during the playback whatever a frame costs."""
        if not self._playing:
            return
        now = time.perf_counter()
        dt = (now - self._wall) * SPEEDS[self.cb_speed.currentIndex()]
        self._wall = now
        t = self.t + dt
        if t >= self.duration():                      # L5: the end of the run, or the front: the clock waits there
            t = self.duration()
            if self._run_complete():
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
        if self.node.dragging():                       # (during a node drag, the former shadow marks: 46 ms each,
            shade = [e for e in getattr(self, "events", []) if e[1].startswith("ship ") and "shadow" in e[1]]
        else:                                          # taken again at the release)
            shade = []
            for a, b in engine._periods(self.g.t_s, self.g.ship_shadow > 0.0):
                shade.append((a, "ship enters the shadow"))
                if b < self.duration():
                    shade.append((b, "ship leaves the shadow"))
        self.events = sorted(ev + shade)
        span = max(self.span(), 1e-9)
        self.slider.set_marks([(int(round(10000 * t / span)), f"{t:.1f} s  {text}") for t, text in self.events])

    # ------------------------------------------------------------------ drawing
    def _global(self) -> bool:
        return self.view_mode == "global"

    def _orbital(self) -> bool:
        return self.view_mode == "orbital"

    def set_view(self, mode: str) -> None:
        """The view shown: "global", "ship" or "orbital"."""
        self.view_buttons[mode].setChecked(True)
        if mode == self.view_mode:
            return
        self._profiles[self.view_mode] = self._read_profile()
        self.view_mode = mode
        self.centre = CENTRAL                         # each switch: the view's default place (user, 30 October 2026)
        self._apply_profile(mode)
        self._show_view_options()
        self._rebuild()

    def _read_profile(self) -> dict:
        """The settings on screen (a duration still being typed included)."""
        prof = {}
        for key, w in self._profile_widgets.items():
            if isinstance(w, QDoubleSpinBox):
                w.interpretText()
                prof[key] = w.value()
            else:
                prof[key] = w.isChecked()
        return prof

    def _apply_profile(self, mode: str) -> None:
        """This view's settings on screen, without a drawing per switch: the caller draws once after."""
        prof = self._profiles[mode]
        for key, w in self._profile_widgets.items():
            w.blockSignals(True)
            if isinstance(w, QDoubleSpinBox):
                w.setValue(float(prof[key]))
            else:
                w.setChecked(bool(prof[key]))
                w.set_on_color(VIEW_COLORS[mode])     # the switches in their view's colour (user, 30 October 2026)
            w.blockSignals(False)
        self.sp_trail.setEnabled(self.cb_trails.isChecked())    # (their signals blocked above)
        self.sp_orbit.setEnabled(self.cb_orbit.isChecked())
        self.tg_proj_full.setEnabled(self.cb_proj.isChecked())
        self.tg_object_full.setEnabled(self.cb_object_labels.isChecked())
        self.lb_object_partial.setEnabled(self.cb_object_labels.isChecked())
        self.lb_proj_partial.setEnabled(self.cb_proj.isChecked())
        self.cb_const.setEnabled(self.tg_sky.isChecked())
        self.sky.set_visible(self.tg_sky.isChecked(), self.cb_const.isChecked())

    def _turn_axis(self) -> np.ndarray:
        """The vertical of the mouse rotation: the central body's pole (global view, the reference of "Recenter
        view"), the view's Z (ship: the ship's Z, normal to the range rings; orbital: the local radial)."""
        if self._global() and self.truth is not None:
            return self.truth.body.equator()[:, 2]
        return np.array([0.0, 0.0, 1.0])

    def _view_axes(self, t: float) -> np.ndarray:
        """The axes of the ship and orbital views at t (columns, ICRF): the ship's attitude, or the local orbital
        frame of OWNSHIP (pnr: X prograde, Y normal to the orbit, Z radial outward; user 30 October 2026: the
        objects keep their azimuth when the ship turns, its model turns)."""
        tr = self.truth
        if self._orbital():
            r, v = sgeo.states_at(tr, float(t))
            return prop.pnr_matrix(r[0], v[0])
        return tr.attitude("ship", float(min(t, self.duration())))

    def _azimuth_label(self, phi: float) -> float:
        """The azimuth shown for the angle phi from +X toward +Y: the ship view's own; the orbital view's a bearing
        (user, 30 October 2026): 0 deg prograde, 90 deg on its right (seen from the zenith: -Y), clockwise."""
        return (360.0 - phi) % 360.0 if self._orbital() else phi

    def _ship_to_view(self, t: float) -> np.ndarray:
        """Ship frame -> the view's axes at t (the identity in the ship view)."""
        if not self._orbital():
            return np.eye(3)
        return self._view_axes(t).T @ self.truth.attitude("ship", float(min(t, self.duration())))

    def _show_view_options(self) -> None:
        """B4 / B14: the ship view's own switches (and their titles) shown in the ship view only; hidden, they keep
        their room, so the "3D view" box keeps the height of the taller of its two states (user, 29 October 2026); the
        title names the view shown (user, 30 October 2026)."""
        self._ship_titles[0].setText(f"{'Orbital' if self._orbital() else 'Ship'} view display options:")
        for w in self._ship_only + self._ship_titles:
            sp = w.sizePolicy()
            sp.setRetainSizeWhenHidden(True)
            w.setSizePolicy(sp)
            w.setEnabled(not self._global())
            w.setVisible(not self._global())

    def status_widgets(self) -> list:
        """"Propagate" and the propagation's state, for the window's status bar."""
        return [self.status, self.progress]                # (L5: no Propagate button)

    def file_commands(self) -> list:
        """(text, slot, icon) of the simulator in the window's menu."""
        return [("Record run...", self.run_journal, "SP_MediaPlay"),
                ("Open run...", self.open_journal, "SP_DialogOpenButton")]

    def _rebuild(self, keep_camera: bool = False) -> None:
        """The static layer of the current view, then the frame; keep_camera: the camera as it was (a run of the same
        scenario shown again), else the view's default framing."""
        p = self.plotter
        cam = p.camera
        saved = (cam.GetPosition(), cam.GetFocalPoint(), cam.GetViewUp(), cam.GetViewAngle(),
                 getattr(p, "_pivot", None)) if keep_camera else None
        p.clear()
        if getattr(self, "_front_ren", None) is not None:
            self._front_ren.RemoveAllViewProps()       # (the layer over the scene: the node's former actors)
        p.set_background(BG_COLOR)
        p.remove_all_lights()
        self._add_fps()                               # (the clear took it away)
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
        if saved is not None:
            cam = p.camera
            cam.SetPosition(*saved[0])
            cam.SetFocalPoint(*saved[1])
            cam.SetViewUp(*saved[2])
            cam.SetViewAngle(saved[3])
            if saved[4] is not None:
                p._pivot = saved[4]
            p.renderer.ResetCameraClippingRange()
            self._centre_pending = False              # (a centred body followed, not framed again)
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
        self._add_model()                              # OWNSHIP's model, placed at each frame (user, 30 October 2026)
        self._compute_markers()
        extent = max(float(np.nanmax(np.abs(tr.r_km))) if np.isfinite(tr.r_km).any() else 0.0, 2.0 * body.radius_km)
        self._home = (-extent, extent, -extent, extent, -extent, extent)
        p.camera.up = tuple(self._turn_axis())         # level with the mouse rotation's vertical
        p.reset_camera(render=False, bounds=self._home)
        p._pivot = (0.0, 0.0, 0.0)
        self._centre_pending = True

    def _compute_markers(self) -> None:
        """The events' places, drawn at each frame (_draw_markers); L5: those within the run computed so far."""
        tr = self.truth
        t_end = float(tr.t_s[-1])
        times, pts, labels, rows, refs = [], [], [], [], []
        for t, text, ref in self._marker_events():
            if t > t_end + 1e-9:
                continue
            r = sgeo.states_at(tr, t)[0]
            oid = text.split()[0]
            if oid in tr.ids and np.isfinite(r[tr.row(oid), 0]):
                times.append(t)
                pts.append(r[tr.row(oid)])
                labels.append(text)
                rows.append(tr.row(oid))
                refs.append(ref)
        self._markers = (np.array(times, float), np.array(pts, float).reshape(-1, 3), labels, np.array(rows, int))
        self._marker_refs = refs

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
            if (hit is not None and hit[0] == self._sky_key() and hit[1] is ts and hit[2][0] + step <= self.t
                    <= hit[2][-1] - step):                   # the body still well inside it: kept (all orbits:
                near_t, near, t_lo, t_hi = hit[2], hit[3], hit[2][0], hit[2][-1]    # 51 at each frame, user)
            else:
                lo, hi = self.t - 2.0 * step, self.t + 2.0 * step
                t_lo = ts[ts < lo][-1] if (ts < lo).any() else lo   # (from the coarse points beside it)
                t_hi = ts[ts > hi][0] if (ts > hi).any() else hi
                near_t = np.linspace(t_lo, t_hi, BODY_ORBIT_NEAR)
                near = self._orbit_rel_many(b, near_t)
                nc[b] = (self._sky_key(), ts, near_t, near)
            if self._orbit_kept(b, now[parent], rel, near):
                continue
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
                self._poly(f"{DYN}orbit_{b}", now[parent] + np.vstack(coarse), lines=lines, color=_orbit_line_color(b),
                           line_width=1)                             # (its body's dominant colour, user)
            self._poly(f"{DYN}orbit_{b}_near", now[parent] + near, lines=np.r_[len(near), np.arange(len(near))],
                       color=_orbit_line_color(b), line_width=1)

    def _orbit_kept(self, b: str, parent: np.ndarray, rel: np.ndarray, near: np.ndarray) -> bool:
        """Whether the orbit of b, drawn about its parent's former place, stays as it is: same points and the parent
        moved less than ORBIT_KEEP_PX on screen where the orbit is nearest to the camera (user, 1 November 2026: its 51
        orbits were sent to the graphics card at each frame). Else it remembers the place of this drawing."""
        last = self._orbit_last.get(b)
        if last is None or last[0] is not rel or last[1] is not near or last[3] != self._sky_key():
            self._orbit_last[b] = (rel, near, parent, self._sky_key(), float(np.linalg.norm(rel, axis=1).max()))
            return False
        cam = self.plotter.camera
        depth = max(float(np.linalg.norm(parent - np.asarray(cam.GetPosition()))) - last[4], 0.0)
        px_size = depth * 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / max(self.plotter.ren_win.GetSize()[1], 1)
        if float(np.linalg.norm(parent - last[2])) >= ORBIT_KEEP_PX * px_size:
            self._orbit_last[b] = (*last[:2], parent, *last[3:])
            return False
        for name in (f"{DYN}orbit_{b}", f"{DYN}orbit_{b}_near"):
            e = self._dyn.get(name)
            if e is not None:
                self._used.add(name)
                e[2].SetVisibility(True)
        return True

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

    def _orbit_rel_many(self, b: str, ts: np.ndarray) -> np.ndarray:
        """_orbit_rel at the times ts, (n, 3) km: a moon's at once (E1, user 8 November 2026: one call per point cost
        most of a playback frame and 1.2 s of a lunar scenario's opening), the others one by one."""
        if b in MOON_DATA:
            T = eph.centuries_tt(scn.epoch(self.sc), 0.0) + np.asarray(ts, float) / (86400.0 * 36525.0)
            return eph.satellite_planetocentric_many_km(b, T)
        return np.array([self._orbit_rel(b, float(t)) for t in ts]).reshape(-1, 3)

    def _orbit_shape(self, b: str) -> Tuple[np.ndarray, np.ndarray]:
        """(times, the body b about its parent over one period centred on t, km), computed again when t has moved a
        twentieth of the period (cached: the ephemeris at every frame would slow the playback)."""
        parent, period = BODY_PARENT[b]
        cache = getattr(self, "_orbit_cache", {})
        self._orbit_cache = cache
        hit = cache.get(b)
        if hit is not None and hit[0] == self._sky_key() and abs(hit[1] - self.t) < period / 20.0:
            return hit[2], hit[3]
        ts = self.t + np.linspace(-0.5, 0.5, BODY_ORBIT_POINTS) * period
        out = self._orbit_rel_many(b, ts)
        cache[b] = (self._sky_key(), self.t, ts, out)
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
                _set_user_matrix(a, u)
            if b == "earth" and self._actor("body_earth_atm") is not None:
                u = np.eye(4)
                u[:3, :3], u[:3, 3] = float(scn.bodies()["earth"]["radius_km"]) * np.eye(3), pos
                _set_user_matrix(self._actor("body_earth_atm"), u)
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
                    _set_user_matrix(a, u)
            drawn[name] = (pos_b[name], rad)
        for name in ("sun", "earth", "moon"):          # a dot too when too small (user, 29 October 2026)
            drawn[name] = (pos_b[name], self._body_radius(name))
        for name in MOON_DATA:                          # the major moons
            a = self._actor(f"body_{name}")
            if a is not None and a.GetVisibility():    # (a hidden one when it shows: _planet_marks)
                self._place_moon(a, name)
            drawn[name] = (pos_b[name], self._body_radius(name))
        self._planets_drawn = drawn
        self._planet_marks()
        light = getattr(self, "_sun_light", None)
        if light is not None:                          # the Sun's point light at its place (around it: at 0)
            light.position = tuple(pos_b["sun"])
            light.focal_point = tuple(pos_b["sun"] + np.array([0.0, 0.0, -1.0]))

    def _place_moon(self, a, name: str) -> None:
        """A moon's actor in the global view at t (two ephemeris calls for its turn: only the moons shown)."""
        u = np.eye(4)
        u[:3, :3], u[:3, 3] = self._moon_matrix_of(name, True), self._bodies_at()[name]
        _set_user_matrix(a, u)

    def _sky_key(self) -> tuple:
        """What the celestial bodies' places depend on (their caches' key; L5: not the run object, which the live
        front replaces as it grows): the epoch and the central body."""
        return (self.sc["meta"].get("epoch_utc"), self.truth.body.name)

    def _bodies_at(self) -> Dict[str, np.ndarray]:
        """Earth, Moon and Sun at t, km from the central body (true places); once per instant of the run shown. The
        50 bodies' ephemerides are computed at every BODY_KNOT_S of the run, a parabola through the three knots
        around t between them (about 3 ms per frame saved; 3 m at most, for Metis); at the highest speeds, where a
        frame jumps over the knots, directly."""
        c = getattr(self, "_bodies_cache", None)
        if c is not None and c[0] == self._sky_key() and c[1] == self.t:
            return c[2]
        if c is not None and c[0] == self._sky_key() and abs(self.t - c[1]) < BODY_KNOT_S:
            k = int(round(self.t / BODY_KNOT_S))
            u = self.t / BODY_KNOT_S - k
            knots = self._body_knots if self._body_knots[0] == self._sky_key() else (self._sky_key(), {})
            self._body_knots = knots
            for j in (k - 1, k, k + 1):
                if j not in knots[1]:
                    knots[1][j] = self._bodies_exact(j * BODY_KNOT_S)
            if len(knots[1]) > 6:
                for j in [j for j in knots[1] if abs(j - k) > 1]:
                    del knots[1][j]
            a, b, d = (knots[1][k + j] for j in (-1, 0, 1))
            w_a, w_b, w_d = 0.5 * u * (u - 1.0), 1.0 - u * u, 0.5 * u * (u + 1.0)
            out = {n: w_a * a[n] + w_b * b[n] + w_d * d[n] for n in b}
        else:
            out = self._bodies_exact(self.t)
        self._bodies_cache = (self._sky_key(), self.t, out)
        return out

    def _bodies_exact(self, t: float) -> Dict[str, np.ndarray]:
        """The bodies at t from the ephemerides."""
        T = eph.centuries_tt(scn.epoch(self.sc), t)
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
        scr, discs = {}, []
        for name, (c, rad) in drawn.items():
            rel = np.asarray(c, float) - pos
            d = float(np.linalg.norm(rel))
            scr[name] = (rel / max(d, 1e-300), d)
            if d > rad and rad / max(d * k, 1e-300) >= PLANET_MARK_PX:     # drawn as its sphere: it hides names
                discs.append((name, np.asarray(c, float) - pos, rad))

        def behind(q: np.ndarray, own: str) -> bool:
            """A name at q hidden by another body's sphere between it and the camera (user, 30 October 2026)."""
            rel = np.asarray(q, float) - pos
            dq = float(np.linalg.norm(rel))
            u = rel / max(dq, 1e-300)
            for other, oc, rad in discs:
                t = float(oc @ u)
                if other != own and 0.0 < t < dq and float(np.linalg.norm(oc - t * u)) < rad:
                    return True
            return False

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
                    if not small and not prefix and name in MOON_DATA and not a.GetVisibility():
                        self._place_moon(a, name)
                    a.SetVisibility(not small)
                    if not small:
                        self._map_on(actor_name, a)
            if small:
                marked.add(name)
                u = rel / d
                dirs.append(u)
                rgb.append(_rgb(PLANET_COLORS[name]))
                if not behind(c, name):
                    lab_p.append(pos + dist * u + 6.0 * k * dist * right)     # its name beside the dot
                    lab_t.append(next(n for n, key in BODY_NAMES.items() if key == name))
        self.sky.set_dots(np.array(dirs).reshape(-1, 3), np.array(rgb, np.uint8).reshape(-1, 3), PLANET_DOT_PX)
        if lab_t and self.cb_celestial_labels.isChecked():
            self._labels(f"{DYN}planet_lbl", np.array(lab_p), lab_t, MUTED_COLOR, DATA_FONT)
        else:
            lab = self._lab.get(f"{DYN}planet_lbl")
            if lab is not None:
                lab.actor.SetVisibility(False)
        # the ship view's names at the spheres' centres (_ship_bodies): none on a dot, none behind another body
        names = [(q, n) for q, n in getattr(self, "_sbody_names", [])
                 if n.lower() not in PLANET_DATA and n.lower() not in MOON_DATA and n.lower() not in marked
                 and not behind(q, n.lower())]
        if names and self.cb_celestial_labels.isChecked() and not self._global():
            self._labels(f"{DYN}sbody_lbl", np.array([q for q, _n in names]), [n for _q, n in names], MUTED_COLOR,
                         DATA_FONT)
        else:
            lab = self._lab.get(f"{DYN}sbody_lbl")
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
        self._log_depth()
        when, age = self._clip_state
        if cam.GetMTime() == when and age < CLIP_EVERY:    # (the camera still: VTK's walk through 176 actors, 4 ms)
            self._clip_state = (when, age + 1)
            return
        self._in_camera = True                        # a clipping change only: no zoom bound, no ring update
        try:
            ren.ResetCameraClippingRange()
            near, far = cam.GetClippingRange()
            near = max(near, NEAR_FRAC * cam.GetDistance())
            cam.SetClippingRange(near, max(far, 1.01 * near))
        finally:
            self._in_camera = False
        self._clip_state = (cam.GetMTime(), 0)

    def _log_depth(self) -> None:
        """Every actor of the scene writes a logarithmic depth (LOG_DEPTH_*): a relative precision of 3e-6 at any
        distance. All of them or none: two depth laws do not compare. Only the actors new since the last render."""
        ren = self.plotter.renderer
        stamp = ren.GetViewProps().GetMTime()
        if stamp == getattr(self, "_depth_stamp", None):
            return
        self._depth_stamp = stamp
        actors = ren.GetActors()
        actors.InitTraversal()
        for _ in range(actors.GetNumberOfItems()):
            a = actors.GetNextActor()
            sp = a.GetShaderProperty()
            if sp.GetNumberOfShaderReplacements() == 0:
                if a.GetProperty().GetRenderPointsAsSpheres():    # (after VTK's own: its spheres' normals are there)
                    sp.AddFragmentShaderReplacement("//VTK::Coincident::Impl", False,
                                                    "//VTK::Coincident::Impl\n" + LOG_DEPTH_GLSL, False)
                else:
                    sp.AddFragmentShaderReplacement("//VTK::Depth::Impl", True, LOG_DEPTH_GLSL, False)

    def _add_fps(self) -> None:
        """The frame-rate counter: a text at the view's top right corner."""
        from vtkmodules.vtkRenderingCore import vtkTextActor
        a = vtkTextActor()
        a.SetInput("- fps")
        tp = a.GetTextProperty()
        tp.SetFontSize(FPS_FONT)
        tp.SetColor(*pv.Color(MUTED_COLOR).float_rgb)
        tp.SetJustificationToRight()
        tp.SetVerticalJustificationToTop()
        a.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
        a.GetPositionCoordinate().SetValue(0.99, 0.985)
        a.PickableOff()
        self.plotter.renderer.AddViewProp(a)
        self._fps_actor = a

    def _fps_tick(self) -> None:
        """Before each render: the renders per second over the last second ("-" with fewer than two: an idle view
        draws only when something changes)."""
        now = time.perf_counter()
        ts = [t for t in self._fps_times if now - t <= 1.0] + [now]
        self._fps_times = ts
        a = getattr(self, "_fps_actor", None)
        if a is not None:
            a.SetInput(f"{(len(ts) - 1) / (ts[-1] - ts[0]):.0f} fps" if len(ts) >= 2 and ts[-1] > ts[0] else "- fps")

    def _max_camera_distance(self) -> Optional[float]:
        """Camera distance at which the half-height of the view (its smaller size on a landscape screen: the bound is
        then reached in every direction, user 29 October 2026) stands for _max_view_km()."""
        if self.truth is None:
            return None
        cam = self.plotter.camera
        w, h = self.plotter.ren_win.GetSize()
        k = math.tan(math.radians(0.5 * cam.GetViewAngle())) * min(w / max(h, 1), 1.0)
        return self._max_view_km() / k if k > 0 else None

    def _max_view_km(self) -> float:
        """The farthest zoom out (user, 30 October 2026: the whole solar system, not much more): MAX_VIEW_MARGIN x
        (the focal point to the Sun + the farthest aphelion, Pluto's), km of half-height."""
        T = eph.centuries_tt(scn.epoch(self.sc), self.t)
        sun = eph.sun_icrf_km(T) - eph.body_icrf_km(self.truth.body.name, T)          # from the central body
        if self._global():
            focal = np.asarray(self.plotter.camera.GetFocalPoint())
            to_sun = float(np.linalg.norm(sun - focal))
        else:
            to_sun = float(np.linalg.norm(sun))                 # (the ship view: about the ship)
        return MAX_VIEW_MARGIN * (to_sun + FARTHEST_APHELION_KM)

    def _min_camera_distance(self) -> Optional[float]:
        """The nearest zoom in: MIN_CAMERA_KM from the focal point, whatever it is (user, 30 October 2026)."""
        return None if self.truth is None else MIN_CAMERA_KM

    def _on_camera(self) -> None:
        """Every camera change (wheel, buttons, follow): the zoom out held at _max_view_km(), then the range rings."""
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
        if self.truth is not None and self._global():
            self._draw_markers()                      # the events' labels grouped again as the zoom changes
        for name in list(getattr(self, "_screen_src", {})):   # dashes and arrowheads constant on screen
            self._draw_screen(name)
        if self._proj_src is not None:
            self._draw_proj_pts()
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
        self.node.edits += 1
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

    def _apply_centre(self, reframe: bool = False, notify: bool = True) -> None:
        """Centre the camera on the chosen body (reframe: also a distance framing what is around it), or back to the
        default view of the central body / the ship. Its three camera changes count for one (_on_camera), none with
        notify False (the frame drawn after it follows the camera)."""
        if self.truth is None:
            return
        p = self.plotter
        cam = p.camera
        tgt = self._target()
        self._in_camera = tgt is not None
        if tgt is None:
            if reframe:
                if self._global():
                    p.camera.up = tuple(self._turn_axis())
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
        self._in_camera = False
        if notify or reframe:
            self._on_camera()
        if reframe:
            p.render()

    def _marker_events(self) -> List[Tuple[float, str, Tuple[str, str, int]]]:
        """(time, text, origin) of the events marked on the paths; origin = (owner kind "ship" / "objects" / "groups",
        owner id, index of the manoeuvre in the owner's list, -1: none), what a click on the mark selects (user,
        8 November 2026): an impulse or a burn and its node, the start of PN and its homing-guidance line, an
        appearance or an impact its body only. A group's impulses and burns on its first member (#0)."""
        out = []
        tr = self.truth
        sc = self.sc or {}
        owners = {"ship": ("ship", "ship", sc.get("ship", {}))}
        owners.update({o.get("id"): ("objects", o.get("id"), o) for o in sc.get("objects", [])})
        groups = {g.get("id"): g for g in sc.get("groups", [])}

        def owner_of(oid: str):
            if oid in owners:
                return owners[oid]
            gid = oid.rsplit("#", 1)[0] if "#" in oid else None
            return ("groups", gid, groups[gid]) if gid in groups else (None, None, {})

        def pn_index(o: dict) -> int:
            return next((k for k, m in enumerate(o.get("manoeuvres", [])) if m.get("type") == "pn"), -1)
        for i, oid in enumerate(tr.ids):
            kind, own, o = owner_of(oid)
            body = (kind or "", own or oid, -1)
            if i and tr.t_appear_s[i] > 0.0:
                out.append((float(tr.t_appear_s[i]), f"{oid} appears", body))
            if np.isfinite(tr.t_pn_start_s[i]):
                out.append((float(tr.t_pn_start_s[i]), f"{oid} PN", (kind or "", own or oid, pn_index(o))))
            if np.isfinite(tr.t_impact_s[i]):
                out.append((max(float(tr.t_impact_s[i]) - 1e-6, 0.0), f"{oid} impact", body))
        marked = [(kind, own, o, own) for kind, own, o in owners.values()]
        marked += [("groups", gid, g, f"{gid}#0") for gid, g in groups.items()]
        for kind, own, o, row_id in marked:
            for k, m in enumerate(o.get("manoeuvres", [])):
                if m.get("type") in ("impulse", "burn"):
                    out.append((float(m["t_s"]), f"{row_id} {m['type']}", (kind, own, k)))
        return out

    def _add_model(self) -> None:
        """The ship model (hull and arrays) at its true size in km, in the ship's frame."""
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
        self._model_r_km = KM_PER_MODEL_UNIT * max(float(np.max(np.linalg.norm(part["mesh"].points, axis=1)))
                                                   for part in ship["hull"] + ship["arrays"])

    def _place_model(self, centre: np.ndarray, m: np.ndarray) -> None:
        """The model's origin (the apertures) at centre, turned by its attitude m (ship frame to scene), the scale to
        km in the same matrix (one actor per part: the GPU's float32 kept near the part, user 30 October 2026)."""
        from vtkmodules.vtkCommonMath import vtkMatrix4x4
        t = np.eye(4)
        t[:3, :3] = KM_PER_MODEL_UNIT * np.asarray(m, float)
        t[:3, 3] = centre
        mat = vtkMatrix4x4()
        mat.DeepCopy(t.ravel())
        for a in self._model["actors"]:
            a.SetScale(1.0, 1.0, 1.0)
            a.SetUserMatrix(mat)

    def _build_ship(self) -> None:
        """V4: the ship model at its true size in km (the ship's axes are the dynamic ones); no equator (a ring
        without a range)."""
        p = self.plotter
        self._add_model()
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

    def set_achieved(self, achieved: Optional[dict]) -> None:
        """The figures achieved per volume id (the owner's: the live estimate or the exact results), the labels'
        second line (user, 11 November 2026)."""
        if achieved != getattr(self, "_achieved", None):
            self._achieved = dict(achieved or {})
            if self._vol:
                self._place_volumes()
                self._render()

    def set_mission(self, mission: Optional[dict], hidden=()) -> None:
        """D (phase 6 merge): the mission being edited (its volumes drawn in the ship view, live) and the ids of its
        sectors hidden; None: no volumes. A mission in the one-file format (user, 11 November 2026) carries its flight
        plan, its last exact result and its profile: left out here (the view only draws the volumes; the mission is
        copied at every rebuild)."""
        if mission is not None:
            mission = {k: v for k, v in mission.items() if k not in ("results", "profile", "flight_plan")}
        self.mission = mission
        self._hidden_ids = set(hidden)
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
        s_v = getattr(self, "_vol_view", np.eye(3)) if self._orbital() else np.eye(3)   # (the orbital view's turn)
        origin = self._array_view
        if self._global() and self.truth is not None:
            # the global view (user, 10 November 2026: where the volumes lie in space): from the apertures at the
            # ship's place at t, turned by its attitude at t (ship -> ICRF, the global view's axes)
            r, _v = sgeo.states_at(self.truth, self.t)
            s_v = self.truth.attitude("ship", self.t)
            origin = r[0] + s_v @ sgeo.ARRAY_KM
            if not np.all(np.isfinite(origin)):
                for acts in vol["per_sector"]:
                    for a in acts:
                        a.SetVisibility(False)
                return
        for rot, acts, c in zip(vol.get("rot") or [np.eye(3)] * len(vol["per_sector"]), vol["per_sector"],
                                vol["centres"]):
            rot = s_v @ rot
            u = np.eye(4)
            u[:3, :3] = rot
            u[:3, 3] = origin
            for a in acts:
                a.SetVisibility(True)
            for a in acts:
                _set_user_matrix(a, u)
            pts.append(rot @ c + origin)
        ll = vol.get("line_labels")
        if ll is not None:
            from gui.scene_view import LABEL_ASKED, LABEL_NOK, LABEL_OK
            shown = bool(pts) and self.cb_object_labels.isChecked()
            ach = getattr(self, "_achieved", {}) or {}
            ids = vol.get("ids") or [str(i) for i in range(len(vol["texts"]))]
            for i, (pt, text, sid) in enumerate(zip(pts, vol["texts"], ids)):
                a = ach.get(sid)
                lines = [(text, LABEL_ASKED)]
                if a:
                    lines.append((v3.achieved_text(a),
                                  LABEL_OK if a["ok"] else LABEL_NOK))
                for t, ok_ in v3.label_extra_lines(a):
                    lines.append((t, LABEL_OK if ok_ else LABEL_NOK))
                ll.put(sid, pt, lines)
            ll.keep(ids[:len(pts)])
            ll.set_all_visible(shown)

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
            dc = -(r[0] / np.linalg.norm(r[0])) @ self._view_axes(self.t)
            if pos @ dc > 0.0:
                pos = pos - 2.0 * (pos @ dc) * dc
            if self._orbital():                        # from the zenith, 0 deg (prograde) up on screen (user)
                pos = np.array([0.0, 0.0, float(np.linalg.norm(pos))])
                p.camera.up = (1.0, 0.0, 0.0)
        p.camera.focal_point = (0.0, 0.0, 0.0)
        p.camera.position = tuple(pos)
        p._pivot = (0.0, 0.0, 0.0)
        p.renderer.ResetCameraClippingRange()

    def recentre(self) -> None:
        """Back to the view's centre (button "Recenter view"). Global view: the body double-clicked, else the central
        body, the zoom and the direction of view kept (undoes a pan); and the view realigned with the reference
        plane, the equatorial plane of the central body (user, 29 October 2026): its roll undone, the body's north
        pole up on screen. Ship view (user, 30 October 2026): back to the view at start, on the ship (_ship_home).
        Orbital view (user, 30 October 2026): on the ship, from the zenith, 0 deg up and 90 deg to the right (as
        _ship_home), the zoom kept."""
        if self.truth is None:
            return
        cam = self.plotter.camera
        if self._orbital():
            self.centre = CENTRAL
            dist = cam.GetDistance()
            cam.SetFocalPoint(0.0, 0.0, 0.0)
            cam.SetPosition(0.0, 0.0, dist)
            cam.SetViewUp(1.0, 0.0, 0.0)
            self.plotter._pivot = (0.0, 0.0, 0.0)
            self.plotter.renderer.ResetCameraClippingRange()
            self._render()
            return
        if not self._global():
            self.centre = CENTRAL
            self._ship_home()
            self._render()
            return
        tgt = self._target()
        tgt = np.zeros(3) if tgt is None else np.asarray(tgt, float)
        off = tgt - np.asarray(cam.GetFocalPoint())
        cam.SetFocalPoint(*tgt)
        cam.SetPosition(*(np.asarray(cam.GetPosition()) + off))
        pole = self.truth.body.equator()[:, 2]                       # up: the pole of the body's rotation sense
        fwd =np.asarray(cam.GetDirectionOfProjection(), float)
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
        self._proj_src = (proj, rgb)
        self._draw_proj_pts()
        th = np.arccos(np.clip(np.sum(u * p, axis=1), -1.0, 1.0))
        # dotted on screen (user, 29 then 30 October 2026: not to be taken for a trail or a planned trajectory,
        # dashed): each arc as a polyline, a point every PROJ_ARC_DEG, cut by _screen_lines
        k = np.clip(np.ceil(np.degrees(th) / PROJ_ARC_DEG) + 1, *PROJ_ARC_PTS).astype(int)
        arc_of = np.repeat(np.arange(len(pos)), k)                           # the arc of each point
        i = np.arange(len(arc_of)) - np.repeat(np.cumsum(k) - k, k)          # its rank on the arc
        s = (i / (k[arc_of] - 1))[:, None]                                   # 0 at the object, 1 in the plane
        t = th[arc_of][:, None]
        sin_t = np.maximum(np.sin(t), 1e-12)
        small = t < 1e-6
        a = np.where(small, 1.0 - s, np.sin((1.0 - s) * t) / sin_t)         # slerp from u (s = 0) to p (s = 1)
        b = np.where(small, s, np.sin(s * t) / sin_t)
        arc = (a * u[arc_of] + b * p[arc_of]) * rad[arc_of][:, None]
        self._screen_lines(f"{DYN}proj_arcs", arc, k, np.repeat(rgb, k, axis=0), PROJ_DOT_PX, front=True,
                           line_width=1.5, opacity=0.8)

    def _draw_proj_pts(self) -> None:
        proj, rgb = self._proj_src
        self._poly(f"{DYN}proj_pts", self._in_front(proj), rgb=rgb, point_size=7, render_points_as_spheres=True,
                   opacity=0.8)

    def _in_front(self, pts: np.ndarray) -> np.ndarray:
        """Points hidden by a celestial body (or inside it) brought halfway to its surface along their line of sight:
        the same place on screen, in front of the body (user, 30 October 2026: the projections stay shown)."""
        discs = list(getattr(self, "_planets_drawn", {}).values())
        if not discs or not len(pts):
            return pts
        eye = np.asarray(self.plotter.camera.GetPosition(), float)
        hid, entry = _occluded(pts, eye, discs)
        if not hid.any():
            return pts
        out = np.array(pts, float)
        d = out[hid] - eye
        out[hid] = eye + d * (0.5 * entry[hid] / np.linalg.norm(d, axis=1))[:, None]
        return out

    def _screen_lines(self, name: str, pts: np.ndarray, counts: np.ndarray, rgb: np.ndarray, pattern, *,
                      tips: Optional[tuple] = None, front: bool = False, **style) -> None:
        """Lines whose look is set on screen (user, 30 October 2026): the polylines (pts, counts, rgb per point)
        dashed by pattern (on, off px at UI size 100 %), and / or arrowheads at tips (their points, the points
        before, their rgb); drawn again at each camera change (_on_camera). front: kept in front of the bodies
        (_in_front)."""
        self._screen_src[name] = (pts, counts, rgb, pattern, tips, front, style)
        self._draw_screen(name)

    def _draw_screen(self, name: str) -> None:
        pts, counts, rgb, pattern, tips, front, style = self._screen_src[name]
        if front:
            pts = self._in_front(pts)
        cam, size = self.plotter.camera, self.plotter.ren_win.GetSize()
        if min(size) <= 0:                             # (not shown yet: the widget's size)
            size = (max(self.plotter.width(), 1), max(self.plotter.height(), 1))
        if pattern is not None:
            q, lines, c = _dashes(pts, counts, rgb, cam, size, pattern[0] * self.ui_scale, pattern[1] * self.ui_scale)
            self._poly(name, q, lines=lines, rgb=c, **style)
        if tips is not None:
            q, lines, c = _arrowheads(*tips, cam, size, VEL_ARROW_PX * self.ui_scale)
            self._poly(name + "_tips", q, lines=lines, rgb=c, **style)

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

    def _truth_lines(self, name: str, rows, rgb: np.ndarray, t0: float, t1: float, look: str = "", **style) -> None:
        """Global view: the rows' paths over [t0, t1] in their colours, through the truth records on the run's grid
        (_path_stride) between the interpolated states at t0 and t1; not drawn when empty. rows: indices, or groups
        of indices (a multitrack: one path, its members' mean). look "trail": fading from TRAIL_ALPHA at t1 to 0 at
        t0; "planned": dashed on screen."""
        if t1 <= t0 or not len(rows):
            return
        tr = self.truth
        groups = _as_groups(rows)
        ks = np.arange(0, len(tr.t_s), self._path_stride)
        ks = ks[(tr.t_s[ks] > t0) & (tr.t_s[ks] < t1)]
        pts = _group_mean(np.concatenate([sgeo.states_at(tr, t0)[0][None], tr.r_km[ks],
                                          sgeo.states_at(tr, t1)[0][None]]), groups)
        self._styled_lines(name, pts, rgb, np.r_[t0, tr.t_s[ks], t1], t0, t1, look, **style)

    def _styled_lines(self, name: str, pts: np.ndarray, rgb: np.ndarray, ts: np.ndarray, t0: float, t1: float,
                      look: str, **style) -> None:
        """The paths pts (n_t, n, 3) at the times ts in their colours rgb (n, 3), drawn as look says (_truth_lines)."""
        ok = np.isfinite(pts[..., 0])
        pl = _polylines(np.where(ok[..., None], pts, 0.0), ok)
        if pl is None:
            return
        q, lines, c, keep = pl
        col = np.repeat(rgb[keep], c, axis=0)
        if look == "planned":
            self._screen_lines(name, q, c, col, PLANNED_DASH_PX, **style)
            return
        if look == "trail":                            # its opacity along it, from its times (object-major order)
            tt = np.broadcast_to(np.asarray(ts, float)[:, None], ok.shape)[:, keep].T[ok[:, keep].T]
            alpha = TRAIL_ALPHA * np.clip((tt - t0) / max(t1 - t0, 1e-12), 0.0, 1.0)
            col = np.c_[col, 255.0 * alpha].astype(np.uint8)
            style.pop("opacity", None)
        self._poly(name, q, lines=lines, rgb=col, **style)

    def _coast(self, r: np.ndarray, v: np.ndarray) -> np.ndarray:
        """The free fall of the states r, v (n, 3) at t over the orbital vector's duration (prop.coast: the run's
        gravity, no burn, no guidance), positions (n_pts, n, 3) from the central body."""
        dur = self.sp_orbit.value()
        need = self.t + dur
        third = getattr(self, "_coast_third", None)
        if prop.THIRD_BODY and (third is None or third[0] < need):            # the bodies' grid over the fall
            span = max(need, 2.0 * self.duration())
            self._coast_third = third = (span, prop.view_third_bodies(self.truth.body.name, scn.epoch(self.sc), span))
        n = int(np.clip(math.ceil(dur / COAST_STEP_S), *COAST_STEPS))
        return prop.coast(self.truth.body, third[1] if third else None, r, v, self.t, dur, n)

    def _ship_coast(self, r: np.ndarray, v: np.ndarray, rs: np.ndarray, vs: np.ndarray) -> np.ndarray:
        """Ship view: the rows' free falls from t as the ship view draws a body (as "planned trajectory" does, user
        29 October 2026): relative to OWNSHIP's own free fall at the same instant, in the ship's frame at that instant
        (its attitude of the run, held at the run's end beyond it), at 1:1. Without a
        manoeuvre it lies on the planned trajectory."""
        path = self._coast(np.r_[r[:1], rs], np.r_[v[:1], vs])                    # (n_pts, 1 + n, 3)
        ts = self.t + np.linspace(0.0, self.sp_orbit.value(), len(path))
        if self._orbital():                            # the local orbital frame along OWNSHIP's own fall
            vs_ = np.gradient(path[:, 0], ts, axis=0) if len(ts) > 1 else np.r_[v[:1]]
            ms = prop.pnr_matrix(path[:, 0], vs_)
        else:
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
            light = (1.0 - VEL_WHITE) * np.asarray(rgb[keep], float) + VEL_WHITE * 255.0   # (user, 30 October 2026)
            col = np.repeat(light, c, axis=0).astype(np.uint8)
            self._poly(f"{DYN}vel", pts, lines=lines, rgb=col, line_width=2)
            end = np.cumsum(c) - 1                                         # each path's last point, its arrowhead
            self._screen_lines(f"{DYN}vel_arrow", pts, c, col, None, tips=(pts[end], pts[end - 1], light),
                               line_width=2)

    def _ship_lines(self, name: str, js, rgb: np.ndarray, sel: np.ndarray, look: str = "", **style) -> None:
        """Ship view: the objects' (columns js of the geometry, or groups of columns: a multitrack's mean) paths
        through the records sel, in their colours, drawn as look says (_truth_lines)."""
        g = self.g
        groups = _as_groups(js)
        if not groups:
            return
        cols = np.unique(np.concatenate(groups))
        where = {c: k for k, c in enumerate(cols)}
        if look == "planned":                          # the records on the run's grid, the next ones in full
            pts, ts = self._planned_records(cols, sel)
        else:
            pts, ts = self._view_records(sel, cols), g.t_s[sel]
        pts = _group_mean(pts, [np.array([where[c] for c in gr]) for gr in groups])
        if self._full_projection():                    # (the gaps kept: NaN)
            pts = np.where(np.isfinite(pts), self._to_plane(np.where(np.isfinite(pts), pts, 0.0)), np.nan)
        self._styled_lines(name, pts, rgb, ts, self.t - self.sp_trail.value(), self.t, look, **style)

    def _view_records(self, sel: np.ndarray, cols: np.ndarray) -> np.ndarray:
        """The objects (columns cols of the geometry) at the records sel, in the view's frame (n_t, n, 3)."""
        g, tr = self.g, self.truth
        if self._orbital() and len(tr.t_s) == len(g.t_s):   # each record in the orbital frame at its instant
            rs = tr.r_km[sel][:, :1]
            ax = prop.pnr_matrix(rs[:, 0], tr.v_kms[sel][:, 0])               # (n_t, 3, 3)
            return np.einsum("kni,kij->knj", tr.r_km[sel][:, cols + 1] - rs, ax)
        rr = g.range_km[sel][:, cols]                                  # (n_t, n)
        # (the geometry's are from the apertures, the view's from the centre of gravity)
        return sgeo.ARRAY_KM + g.dir_ship[sel][:, cols] * np.where(np.isfinite(rr), rr, np.nan)[..., None]

    def _planned_records(self, cols: np.ndarray, sel: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """The planned paths of the columns cols: the records sel up to the next point of the run's grid
        (_path_stride), then that grid, computed once per run and view (every record at every frame, 2400 x 200
        points at x10, took 40 of the frame's 48 ms): (n_t, n, 3) and their times."""
        g = self.g
        idx = np.flatnonzero(sel)
        if not len(idx):
            return np.zeros((0, len(cols), 3)), np.zeros(0)
        c = getattr(self, "_grid_paths", None)
        if c is None or c[0] is not g or c[1] != self._orbital():
            ks = np.arange(0, len(g.t_s), self._path_stride)
            c = self._grid_paths = (g, self._orbital(), ks, self._view_records(ks, np.arange(g.range_km.shape[1])))
        ks, grid = c[2], c[3]
        n = int(np.searchsorted(ks, idx[0]))           # the first grid point from here on
        near = idx[idx < ks[n]] if n < len(ks) else idx
        return (np.concatenate([self._view_records(near, cols), grid[n:][:, cols]]),
                np.r_[g.t_s[near], g.t_s[ks[n:]]])

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
                # (the zoom bounds hold _max_view_km() at the half-height: beyond, the corners keep their rings)
                if nice in seen:
                    continue
                seen.add(nice)
                rad = nice
                off = len(pts) * RING_POINTS
                pts.append(rad * (np.cos(ang)[:, None] * e1 + np.sin(ang)[:, None] * e2))
                lines.append(np.r_[RING_POINTS, off + np.arange(RING_POINTS)])
                # each value where its ring crosses the azimuths 0 and 180 (user, 30 October 2026)
                lab_p += [sign * rad * e1 + RING_LABEL_OFF * half * e2 for sign in (1.0, -1.0)]
                lab_t += [f"{nice:g} km" if nice >= 1.0 else f"{1e3 * nice:.3g} m"] * 2
            if pts:
                self._poly(f"{DYN}rings", np.vstack(pts), lines=np.hstack(lines), color=RING_COLOR, line_width=1)
                self._labels(f"{DYN}rings_lbl", np.array(lab_p), lab_t, MUTED_COLOR, 14)
                q, counts = self._hidden_arcs(np.array(sorted(seen)), e1, e2)
                self._screen_lines(f"{DYN}rings_hidden", q, counts, np.tile(_rgb(RING_COLOR), (len(q), 1)).astype(np.uint8),
                                   RING_HIDDEN_DOT_PX, line_width=1)
            # azimuth spokes in the rings' plane: from the ship's centre out to the corners of the view every 45 deg,
            # phi from +X toward +Y (the ship frame's convention), labelled inside the view
            names = (f"{DYN}spokes", f"{DYN}spokes_lbl")
            az = np.radians(np.arange(0.0, 360.0, SPOKE_STEP_DEG))
            d = np.cos(az)[:, None] * e1 + np.sin(az)[:, None] * e2
            self._poly(names[0], np.r_[np.zeros((len(az), 3)), cover * d], lines=_pairs(len(az)),
                       color="#3a4256", line_width=1)
            self._labels(names[1], SPOKE_LABEL_FRAC * half * d,
                         [f"{self._azimuth_label(a):g}°" for a in np.arange(0.0, 360.0, SPOKE_STEP_DEG)],
                         MUTED_COLOR, 14)
        finally:
            self._in_rings = False

    def _hidden_arcs(self, radii: np.ndarray, e1: np.ndarray, e2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """The parts of the range rings (radii) hidden by a celestial body (user, 30 October 2026, drawn dotted): each
        point brought halfway to the surface along its line of sight, the same place on screen in front of the body
        (the depth test hides the ring itself): (points, count of each arc)."""
        none = np.zeros((0, 3)), np.zeros(0, int)
        eye = np.asarray(self.plotter.camera.GetPosition(), float)
        step = 2.0 * math.pi / (RING_POINTS - 1)
        th = step * np.arange(RING_POINTS - 1)

        def at(rad, a):
            return rad[..., None] * (np.cos(a)[..., None] * e1 + np.sin(a)[..., None] * e2)

        grid = at(radii[:, None], th[None, :])
        discs = [d for d in getattr(self, "_planets_drawn", {}).values() if _occluded(grid, eye, [d])[0].any()]
        if not discs:
            return none
        hid = _occluded(grid, eye, discs)[0]
        rows, a_hid, a_vis, spans = [], [], [], []                # one (hidden side, visible side) per end of an arc
        for i in np.flatnonzero(hid.any(axis=1)):
            h = hid[i]
            if h.all():
                spans.append((radii[i], 0.0, 2.0 * math.pi, None))
                continue
            starts = np.flatnonzero(~h & np.roll(h, -1))          # the point before each hidden arc
            ends = np.flatnonzero(h & np.roll(~h, -1))            # the last point of each
            if ends[0] < starts[0]:
                ends = np.roll(ends, -1)
            for s, e in zip(starts, ends):
                spans.append((radii[i], None, None, len(rows)))
                rows += [radii[i], radii[i]]
                a_hid += [th[s] + step, th[e] + (2.0 * math.pi if e < s else 0.0)]
                a_vis += [th[s], th[e] + step + (2.0 * math.pi if e < s else 0.0)]
        rad = np.array(rows, float)
        a, b = np.array(a_hid, float), np.array(a_vis, float)
        for _ in range(RING_HIDDEN_BISECT if len(a) else 0):      # the place where each arc ends, within the step
            mid = 0.5 * (a + b)
            h = _occluded(at(rad, mid), eye, discs)[0]
            a, b = np.where(h, mid, a), np.where(h, b, mid)
        arcs = []
        for r, a0, a1, k in spans:
            if k is not None:
                a0, a1 = a[k], a[k + 1]
            inner = step * np.arange(math.floor(a0 / step) + 1, math.ceil(a1 / step))
            ring = at(np.array(r), np.r_[a0, inner, a1])
            entry = _occluded(ring, eye, discs)[1]
            d = ring - eye
            arcs.append(eye + d * (0.5 * entry / np.linalg.norm(d, axis=1))[:, None])
        return np.vstack(arcs), np.array([len(x) for x in arcs], int)

    # ------------------------------------------------------------------ dynamic layer (updated in place)
    def _reset_dynamic(self) -> None:
        """After plotter.clear(): forget the dynamic actors, labels and volumes (they were removed with it)."""
        self._dyn: Dict[str, list] = {}               # name -> [topology key, mesh, actor]
        self._screen_src: Dict[str, tuple] = {}       # the lines drawn on screen's terms (_screen_lines)
        self._proj_src: Optional[tuple] = None        # the range projection's copies and colours
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
        front = style.pop("front", False)             # drawn over everything (the manoeuvre node, 8 November 2026)
        self._used.add(name)
        e = self._dyn.get(name)
        if len(pts) == 0:
            if e is not None:
                e[2].SetVisibility(False)
            return
        key = (len(pts), None if lines is None else lines.tobytes(), rgb is not None, cell_rgb is not None)
        if e is not None and (e[0] == key or (lines is not None and e[0][1] is not None and e[0][2:] == key[2:])):
            mesh = e[1]
            _set_points(mesh, np.asarray(pts, float))
            if e[0] != key:                            # other lines (dashes at each frame): the actor kept
                mesh.lines = lines
                e[0] = key
            if rgb is not None:
                mesh.point_data["rgb"] = rgb
            if cell_rgb is not None:
                mesh.cell_data["rgb"] = cell_rgb
            mesh.Modified()
            if style.get("color", self._poly_color.get(name)) != self._poly_color.get(name):
                self._poly_color[name] = style["color"]    # (one name may serve several bodies: the node's path, a mark)
                e[2].GetProperty().SetColor(*pv.Color(style["color"]).float_rgb)
            e[2].SetVisibility(True)
            return
        if e is not None:
            self.plotter.remove_actor(e[2], render=False)
            if getattr(self, "_front_ren", None) is not None:
                self._front_ren.RemoveActor(e[2])
        mesh = pv.PolyData(np.asarray(pts, float), lines=lines) if lines is not None else pv.PolyData(np.asarray(pts, float))
        if rgb is not None:
            mesh.point_data["rgb"] = rgb
        if cell_rgb is not None:
            mesh.cell_data["rgb"] = cell_rgb
        kw = dict(style)
        if rgb is not None or cell_rgb is not None:
            kw.update(scalars="rgb", rgb=True, preference="point" if rgb is not None else "cell")
        actor = self.plotter.add_mesh(mesh, name=name, render=False, **kw)
        _near_camera_precision(actor, mesh)
        if front:                                     # moved to the layer over the scene: never hidden by a body
            self.plotter.remove_actor(actor, render=False)
            actor.GetProperty().LightingOff()
            self._front_renderer().AddActor(actor)
        self._dyn[name] = [key, mesh, actor]
        self._poly_color[name] = style.get("color")

    def _front_renderer(self):
        """The layer drawn over the scene with its camera (its depth cleared: what it holds is never behind a body,
        user 8 November 2026: the node's radial handle went into the Moon when zoomed out)."""
        r = getattr(self, "_front_ren", None)
        cam = self.plotter.renderer.GetActiveCamera()
        if r is None:
            from vtkmodules.vtkRenderingCore import vtkRenderer
            win = self.plotter.ren_win
            win.SetNumberOfLayers(max(3, win.GetNumberOfLayers()))
            r = self._front_ren = vtkRenderer()
            r.SetLayer(win.GetNumberOfLayers() - 1)
            r.InteractiveOff()
            win.AddRenderer(r)
        if r.GetActiveCamera() is not cam:
            r.SetActiveCamera(cam)
        return r

    def _row_symbol(self, i: int) -> Optional[str]:
        """The symbol file of a row of the truth (0: OWNSHIP)."""
        if i == 0:
            return symbol_path("ownship", None)
        return symbol_path(self._row_ident[i], self.truth.classes[i])

    def _draw_markers(self) -> None:
        """Global view: the places of the events still to come (user, 29 October 2026: a past event's mark goes),
        each in its body's colour (its path's, its identity's), with its text at the UI size."""
        mk = getattr(self, "_markers", None)
        if mk is None or not len(mk[0]) or not self.cb_events.isChecked():
            return
        t, pts, labels, rows = mk
        keep = t >= self.t - 1e-9
        self._poly(f"{DYN}markers", pts[keep], rgb=self._row_rgb[rows[keep]], point_size=8,
                   render_points_as_spheres=True)
        q, texts = self._marker_groups(pts[keep], [x for x, k in zip(labels, keep) if k])
        self._labels(f"{DYN}markers_lbl", q, texts, MUTED_COLOR, MARK_FONT)

    def _row_noses(self, rows: np.ndarray, r: np.ndarray, v: np.ndarray) -> np.ndarray:
        """(n, 3) unit nose (body Z, ICRF) of the truth rows at t: their attitude when they fly waypoints, else their
        class's rule (sgeo.NOSE_RULES: inertial or relative velocity, or toward the ship), as the geometry's."""
        tr = self.truth
        out = np.full((len(rows), 3), np.nan)
        if len(tr.att_ids) and len(tr.att_t_s) >= 2:
            col = {oid: j for j, oid in enumerate(tr.att_ids)}
            ks = [(n, col[tr.ids[i]]) for n, i in enumerate(rows) if tr.ids[i] in col]
            if ks:
                ta = tr.att_t_s
                k = int(np.clip(np.searchsorted(ta, self.t, side="right") - 1, 0, len(ta) - 2))
                u = float(np.clip((self.t - ta[k]) / max(ta[k + 1] - ta[k], 1e-12), 0.0, 1.0))
                n_, j_ = np.array(ks).T
                q0, q1 = tr.att_q[k, j_], tr.att_q[k + 1, j_]
                q1 = np.where((np.einsum("ij,ij->i", q0, q1) < 0.0)[:, None], -q1, q1)   # (the short way)
                q = (1.0 - u) * q0 + u * q1
                out[n_] = _quat_z(q / np.linalg.norm(q, axis=1)[:, None])
        for n, i in enumerate(rows):
            if np.isfinite(out[n, 0]):
                continue
            rule = sgeo.NOSE_RULES.get(tr.classes[i], "ship")
            out[n] = v[i] if rule == "inertial" else v[i] - v[0] if rule == "relative" else r[0] - r[i]
        return out / np.maximum(np.linalg.norm(out, axis=1), 1e-12)[:, None]

    def _draw_shapes(self, rows: np.ndarray, pos: np.ndarray, noses: np.ndarray) -> None:
        """The objects (truth rows) at their size (BODY_SHAPES, user 8 November 2026): cylinders along their nose,
        spheres; at pos (view's km), noses in the view's axes. One mesh per class, its points relative to the
        camera's focal point (an actor matrix in double precision: the GPU's float32 kept near where one zooms in;
        2000 km from the Moon's centre it would be off by about a decimetre)."""
        tr = self.truth
        ok = np.isfinite(pos[:, 0]) & np.isfinite(noses[:, 0])
        origin = np.asarray(self.plotter.camera.GetFocalPoint(), float)
        by: Dict[str, List[int]] = {}
        for n, i in enumerate(rows):
            if ok[n] and tr.classes[i] in BODY_SHAPES:
                by.setdefault(tr.classes[i], []).append(n)
        for cls, ns in by.items():
            kind, length_m, diam_m = BODY_SHAPES[cls]
            unit, tri = _unit_shape(kind)
            ns = np.asarray(ns)
            z = noses[ns]
            a = np.where((np.abs(z[:, 0]) > 0.9)[:, None], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0])
            x = np.cross(z, a)
            x /= np.linalg.norm(x, axis=1)[:, None]
            y = np.cross(z, x)
            local = unit * (1e-3 * np.array([diam_m, diam_m, length_m]))       # km
            pts = (pos[ns] - origin)[:, None, :] + np.einsum("pj,njk->npk", local, np.stack([x, y, z], axis=1))
            pts = pts.reshape(-1, 3)
            rgb = np.repeat(self._row_rgb[rows[ns]], len(unit), axis=0)
            name = f"{DYN}shape_{cls}"
            self._used.add(name)
            e = self._dyn.get(name)
            key = ("shape", len(ns))
            if e is None or e[0] != key:
                if e is not None:
                    self.plotter.remove_actor(e[2], render=False)
                faces = (tri[None] + (len(unit) * np.arange(len(ns)))[:, None, None]).reshape(-1, 3)
                mesh = pv.PolyData(pts, np.c_[np.full(len(faces), 3), faces].ravel())
                mesh.point_data["rgb"] = rgb
                actor = self.plotter.add_mesh(mesh, name=name, scalars="rgb", rgb=True, render=False)
                _near_camera_precision(actor, mesh)
                e = self._dyn[name] = [key, mesh, actor]
            else:
                _set_points(e[1], pts)
                e[1].point_data["rgb"] = rgb
                e[1].Modified()
            from vtkmodules.vtkCommonMath import vtkMatrix4x4
            mat = vtkMatrix4x4()
            for c in range(3):
                mat.SetElement(c, 3, float(origin[c]))
            e[2].SetUserMatrix(mat)
            e[2].SetVisibility(True)

    def _marker_groups(self, pts: np.ndarray, labels: List[str]) -> Tuple[np.ndarray, List[str]]:
        """The events' labels, those of one kind close on screen as one (user, 8 November 2026, as the multitracks:
        "KKV_OS#0 appears" ... 15 times one on another -> "15 x KKV_OS appears"): linked when their points are within
        MARK_GROUP_PX or their texts would overlap (fast.multitrack_labels); one label at its first member's place."""
        if len(labels) < 2:
            return pts, labels
        same: Dict[tuple, int] = {}                   # one body's events at one place (a KKV appearing and starting
        p1, l1 = [], []                               # its PN at once): one label, their kinds joined
        for p_, x in zip(pts, labels):
            oid, kind = x.split(" ", 1) if " " in x else (x, "")
            key = (oid, tuple(np.round(p_, 6)))
            if key in same:
                l1[same[key]] += f", {kind}"
            else:
                same[key] = len(l1)
                p1.append(p_)
                l1.append(x)
        pts, labels = np.array(p1, float).reshape(-1, 3), l1
        if len(labels) < 2:
            return pts, labels
        xy, ok = self._display_xy(pts)
        kinds: Dict[str, int] = {}
        code = np.array([kinds.setdefault(x.split(" ", 1)[1] if " " in x else x, len(kinds)) for x in labels])
        code = np.where(ok, code, -1)
        font = MARK_FONT * self.ui_scale
        bw = BOX_CHAR_W * font * max(len(x) for x in labels)
        bh = BOX_LINE_H * font
        lab = fast.multitrack_labels(np.nan_to_num(xy), code, MARK_GROUP_PX, float(bw), float(bh),
                                     np.zeros(len(labels), np.bool_))
        q, texts, done = [], [], set()
        for k, x in enumerate(labels):
            if lab[k] < 0:
                q.append(pts[k])
                texts.append(x)
            elif lab[k] not in done:
                done.add(lab[k])
                mem = np.flatnonzero(lab == lab[k])
                ids = [labels[j].split(" ", 1)[0] for j in mem]
                kind = x.split(" ", 1)[1] if " " in x else x
                groups = list(dict.fromkeys(i.rsplit("#", 1)[0] for i in ids))
                if len(groups) == 1:
                    texts.append(f"{len(mem)} x {groups[0]} {kind}")
                else:
                    names = ", ".join(groups[:3]) + (", ..." if len(groups) > 3 else "")
                    texts.append(f"{len(mem)} x {kind} ({names})")
                q.append(pts[mem[0]])
        return np.array(q, float).reshape(-1, 3), texts

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
        if self.cb_object_labels.isChecked():
            key = (rows[lone].tobytes(), self._mt_sig, self.tg_object_full.isChecked())
            last = getattr(self, "_box_last", None)
            now = time.perf_counter()
            if self._playing and last is not None and last[0] == key and now - last[1] < BOX_TEXT_S:
                texts = last[2]                       # (their places follow at every frame)
            else:
                texts = self._box_texts(rows[lone], r, v) + [self._mt_box(c, rows, r, v) for c in clusters]
                self._box_last = (key, now, texts)
            self._databox(f"{DYN}labels", np.r_[pts[lone], cq], texts)

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

    def _mt_box_px(self, rows: np.ndarray) -> Tuple[float, float]:
        """px: the (width, height) of a lone object's data box, the widest of rows' names (0, 0: no data boxes), the
        multitracks' second link (user, 8 November 2026: boxes of objects too close overlapped, a multitrack instead)."""
        if not self.cb_object_labels.isChecked() or not len(rows):
            return 0.0, 0.0
        full = self.tg_object_full.isChecked()
        name = max(len(self.truth.ids[i]) for i in rows)
        chars, lines = (max(name, 28), 7) if full else (max(name, 23), 3)   # (_box_texts: its longest line, its lines)
        font = DATA_FONT * self.ui_scale
        return BOX_CHAR_W * font * chars, BOX_LINE_H * font * lines

    def _multitracks(self, rows: np.ndarray, pts: np.ndarray) -> List[dict]:
        """The groups (two or more) of objects of one identity and class linked on screen by distances under
        MULTITRACK_PX x the UI size, except an object the view shows less than MULTITRACK_OFF_KM high at its own depth
        (zoomed in on it; judged at the focal point before, user 8 November 2026: the far objects behind a ship viewed
        close stayed apart), or whose data boxes would overlap beside their symbols, whatever the zoom (user,
        8 November 2026: their boxes piled up; zoomed in on them, they part on screen and split) (one to the next: a
        chain is one group); OWNSHIP never: k (indices into rows), q (their mean place), ident, cls."""
        if len(rows) < 2:
            return []
        xy, ok = self._display_xy(pts)
        cam = self.plotter.camera
        fwd = np.asarray(cam.GetDirectionOfProjection())
        depth = (np.asarray(pts, float) - np.asarray(cam.GetPosition())) @ fwd
        near = 2.0 * depth * math.tan(math.radians(0.5 * cam.GetViewAngle())) < MULTITRACK_OFF_KM
        tr = self.truth
        lim = MULTITRACK_PX * self.ui_scale
        bw, bh = self._mt_box_px(rows[rows > 0])
        if fast.USE_NUMBA:
            c = self._mt_codes
            if c[0] is not self.truth or len(c[1]) != len(tr.ids) or c[2] != self._row_ident:
                names: Dict[Tuple[str, str], int] = {}
                code = np.array([names.setdefault((self._row_ident[i], tr.classes[i]), len(names)) for i in range(len(tr.ids))])
                c = self._mt_codes = (self.truth, code, list(self._row_ident))
            code = np.where(ok & (rows > 0), c[1][rows], -1)
            labels = fast.multitrack_labels(np.nan_to_num(xy), code, float(lim), float(bw), float(bh), near)
            keep = np.flatnonzero(labels >= 0)
            keep = keep[np.argsort(labels[keep], kind="stable")]
            groups = np.split(keep, np.flatnonzero(np.diff(labels[keep])) + 1) if len(keep) else []
            return [dict(k=mem, q=pts[mem].mean(axis=0), ident=self._row_ident[rows[mem[0]]], cls=tr.classes[rows[mem[0]]])
                    for mem in groups]
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        from scipy.spatial import cKDTree
        keys: Dict[Tuple[str, str], List[int]] = {}
        for k, i in enumerate(rows):
            if i > 0 and ok[k]:
                keys.setdefault((self._row_ident[i], tr.classes[i]), []).append(k)
        out = []
        for (ident, cls), ks in keys.items():
            if len(ks) < 2:
                continue
            ks = np.array(ks)
            pairs = cKDTree(xy[ks]).query_pairs(lim, output_type="ndarray").reshape(-1, 2)
            pairs = pairs[~near[ks][pairs].any(axis=1)]
            if bw > 0.0 and bh > 0.0:                 # boxes that would overlap (Chebyshev on the box's axes)
                box = cKDTree(xy[ks] / [bw, bh]).query_pairs(1.0 - 1e-9, p=np.inf, output_type="ndarray").reshape(-1, 2)
                pairs = np.r_[pairs, box]
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
        """A multitrack's data box: its number and class, its members' mean range and closing speed to OWNSHIP, and
        the closest approach of the member passing nearest (user, 30 October 2026: CPA and dCPA, as a lone object's)."""
        i = rows[c["k"]]
        r, v = self._live_rv(r, v)                    # (the node's body on its live path)
        rel, vrel = r[i] - r[0], v[i] - v[0]
        rng = np.linalg.norm(rel, axis=1)
        close = -1e3 * np.einsum("ij,ij->i", rel, vrel) / np.maximum(rng, 1e-12)
        lines = [f"{len(i)} x {c['cls']}", f"range {_dist_text(float(rng.mean()))}",
                 f"closing speed {float(close.mean()):+.0f} m/s"]
        if not self.tg_object_full.isChecked():
            return "\n".join(lines)
        t_c, d_c, reached = self._cpa_now()
        dc = d_c[i - 1]
        if np.isfinite(dc).any():
            m = int(i[np.nanargmin(dc)]) - 1
            lines += self._cpa_box(t_c[m], d_c[m], reached[m])
        return "\n".join(lines)

    def _cpa_box(self, tc: float, dc: float, ok: bool) -> List[str]:
        """The CPA and dCPA lines of a data box (none without a future record)."""
        if not np.isfinite(tc):
            return []
        if not ok:                                    # still closing at the run's end
            return [f"closest approach in > {tc - self.t:.0f} s", f"closest distance < {_dist_text(dc)}"]
        return ["closest approach: receding" if tc <= self.t + 1e-6 else f"closest approach in {tc - self.t:.1f} s",
                f"closest distance {_dist_text(dc)}"]

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
            _near_camera_precision(actor, mesh)       # (a few mm across at hundreds of km, user 8 November 2026)
            self._dyn[name] = [n, mesh, actor]
        if missing:
            self._poly(f"{DYN}pts", pts[missing], color="#ffffff", point_size=8, render_points_as_spheres=True)

    def _sym_corners(self, q: np.ndarray, px: float = SYMBOL_PX) -> np.ndarray:
        """The four corners of each symbol centred on q, facing the camera, px (SYMBOL_PX) x the UI size on
        screen; range projection "full": kept in front of the bodies, the copies stand for the objects."""
        if self._full_projection():
            q = self._in_front(q)
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
        # each symbol drawn nearer the camera along its line of sight by the ship model's radius (same place on
        # screen): OWNSHIP's stays over its model up close (user, 30 October 2026)
        rel = np.asarray(q, float) - pos
        dist = np.linalg.norm(rel, axis=1)
        pull = np.minimum(getattr(self, "_model_r_km", 0.0), 0.9 * dist)
        q = q - (pull / np.maximum(dist, 1e-300))[:, None] * rel
        return v3.billboard(self.plotter, q, px)        # (shared: gui/view3d.py)

    def _place_symbols(self) -> None:
        """After a camera change: every shown symbol (and the selection's square) resized and turned toward the
        camera."""
        for name, q in getattr(self, "_sym_pts", {}).items():
            e = self._dyn.get(name)
            if e is not None and e[2].GetVisibility() and e[0] == len(q):
                e[1].points = self._sym_corners(q)
                e[1].Modified()
        self._place_boxes()                           # the data boxes around their symbols
        for name, q in getattr(self, "_mt_lbl", {}).items():     # the multitracks' numbers under their symbols
            lab = self._lab.get(name)
            if lab is not None and lab.actor.GetVisibility() and len(q) == len(lab._last):
                lab.set(self._mt_below(q), lab._last)
        for name, (pts, shape, px) in self._glyphs.items():
            e = self._dyn.get(name)
            if e is not None and e[2].GetVisibility() and e[0][0] == len(pts) * len(shape):
                e[1].points = self._glyph_pts(pts, shape, px)
                e[1].Modified()
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
            lab.set(1.05 * self._screen_len(np.zeros((1, 3)))[:, None] * getattr(self, "_axes_dirs", np.eye(3)),
                    ["X", "Y", "Z (nose)"])

    def _draw_selection(self) -> None:
        """A square around the selected body where the view draws it (an object or OWNSHIP; none for the Earth, the
        Moon, the Sun), SELECT_PX on screen, kept facing the camera (_place_symbols)."""
        sel, q = self.selected, None
        if self.read_only:                                # (mission preparation: OWNSHIP selected for its Info alone,
            sel = None                                    # no square around it; user, 11 November 2026)
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

    def _box_obstacles(self, xy: np.ndarray, half: float) -> list:
        """Display rectangles (x0, y0, x1, y1) no box may cover: the symbols at xy, the manoeuvre node, the texts of the
        event marks."""
        out = [np.c_[xy - half, xy + half]]
        glyph = self.node._glyph
        if glyph is not None:
            c, ok = self._display_xy(np.asarray(glyph[0], float)[None])
            if ok[0]:
                out.append(np.c_[c - NODE_BOX_PX * self.ui_scale, c + NODE_BOX_PX * self.ui_scale])
        lab = self._lab.get(f"{DYN}markers_lbl")
        if lab is not None and lab.actor.GetVisibility() and len(lab._last):
            c, ok = self._display_xy(lab._pts)
            sz = self._box_size(lab._last)
            out.append(np.c_[c, c + sz][ok])
        return out

    def _box_size(self, texts: List[str]) -> np.ndarray:
        """px: the estimated (width, height) on screen of each text of the data boxes' font."""
        font = DATA_FONT * self.ui_scale
        rows = [t.split("\n") for t in texts]
        return np.array([[BOX_CHAR_W * font * max(map(len, r)), BOX_LINE_H * font * len(r)] for r in rows]).reshape(-1, 2)

    def _place_boxes(self) -> None:
        """The data boxes around their symbols without overlapping (user, 1 November 2026): each takes the first free
        place among eight around its symbol, on BOX_RINGS rings, in the window; beside the symbol when that is free, else
        the place it had (no jumps from one frame to the next). A box moved past the first ring gets a leader line.
        Beyond BOX_MAX_PLACED boxes, each beside its symbol."""
        items = []
        for name, q in self._box_pts.items():
            lab = self._lab.get(name)
            if lab is not None and lab.actor.GetVisibility() and len(q) == len(lab._last):
                items += [(name, n, q[n], lab._last[n]) for n in range(len(q))]
        lines: List[np.ndarray] = []
        if len(items) > BOX_MAX_PLACED or not items:
            for name in {it[0] for it in items}:
                self._lab[name].set(self._box_anchor(self._box_pts[name]), self._lab[name]._last)
            self._poly(f"{DYN}box_leaders", np.zeros((0, 3)))
            return
        qs = np.array([it[2] for it in items])
        xy, front = self._display_xy(qs)
        sizes = self._box_size([it[3] for it in items])
        half = 0.5 * SYMBOL_PX * self.ui_scale
        side = half * (max(1.0, SELECT_PX / SYMBOL_PX) + 2.0 * BOX_GAP)
        w, h = self.plotter.ren_win.GetSize()
        taken = np.concatenate(self._box_obstacles(xy[front], half) + [np.zeros((len(items), 4))])
        n_taken = len(taken) - len(items)
        cam = self.plotter.camera
        pos, fwd = np.asarray(cam.GetPosition()), np.asarray(cam.GetDirectionOfProjection())
        up = np.asarray(cam.GetViewUp(), float)
        up = up - (up @ fwd) * fwd
        up /= np.linalg.norm(up)
        right = np.cross(fwd, up)
        wpp = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / max(h, 1)  # world size of a pixel at depth 1
        anchors = {name: self._box_anchor(self._box_pts[name]) for name in {it[0] for it in items}}
        last = self._box_slots
        self._box_slots = {}

        def free(x: float, y: float, bw: float, bh: float, pad: float, n: int) -> bool:
            if x < 0.0 or y - bh < 0.0 or x + bw > w or y > h:
                return False
            t = taken[:n]
            return not np.any((x < t[:, 2] + pad) & (x + bw > t[:, 0] - pad) & (y - bh < t[:, 3] + pad) & (y > t[:, 1] - pad))

        for k, (name, n, q, text) in enumerate(items):
            if not front[k]:
                continue
            bw, bh = sizes[k]
            places = [(side * r * dx - (bw if dx < 0 else 0.5 * bw if dx == 0 else 0.0),
                       (half if r == 1 and dy == 0 else side * r * dy) + (bh if dy > 0 else 0.5 * bh if dy == 0 and r > 1 else 0.0))
                      for r in range(1, BOX_RINGS + 1) for dx, dy in BOX_PLACES]
            key = (name, text.split("\n", 1)[0])
            was = last.get(key)
            tries = [(0, BOX_RETURN_PX)] + ([(was, 0.0)] if was is not None else [])
            tries += [(c, BOX_PAD_PX) for c in range(len(places))]
            slot = next((c for c, pad in tries if free(xy[k, 0] + places[c][0], xy[k, 1] + places[c][1], bw, bh, pad,
                                                       n_taken + k)), was or 0)
            self._box_slots[key] = slot
            x, y = xy[k, 0] + places[slot][0], xy[k, 1] + places[slot][1]
            taken[n_taken + k] = (x, y - bh, x + bw, y)
            scale = max(float((q - pos) @ fwd), 1e-12) * wpp
            anchors[name][n] = q + places[slot][0] * scale * right + places[slot][1] * scale * up
            if slot >= len(BOX_PLACES):
                lines.append(np.array([q, anchors[name][n]]))
        for name, a in anchors.items():
            self._lab[name].set(a, self._lab[name]._last)
        pts = np.array(lines).reshape(-1, 3)
        self._poly(f"{DYN}box_leaders", np.r_[pts[0::2], pts[1::2]], lines=_pairs(len(lines)) if lines else None,
                   color=MUTED_COLOR, line_width=1)

    def _databox(self, name: str, pts: np.ndarray, texts: List[str]) -> None:
        """The bodies' data boxes (user, 29 October 2026), beside their symbols, kept there by the camera changes
        (_place_symbols)."""
        pts = np.asarray(pts, float)
        self._box_pts[name] = pts
        self._labels(name, self._box_anchor(pts), texts, TEXT_COLOR, DATA_FONT, box=True)

    def _box_texts(self, rows: np.ndarray, r: np.ndarray, v: np.ndarray) -> List[str]:
        """The data box of each truth row (0: OWNSHIP): its name, altitude and orbital speed; for an object also its
        range, closing speed, time to and range of its closest approach to OWNSHIP (_cpa_all). Partial: the name, and
        for an object its range and closing speed."""
        tr = self.truth
        r, v = self._live_rv(r, v)                    # (the node's body on its live path)
        alt = np.linalg.norm(r[rows], axis=1) - tr.body.radius_km
        spd = np.linalg.norm(v[rows], axis=1)
        rel, vrel = r[rows] - r[0], v[rows] - v[0]
        rng = np.linalg.norm(rel, axis=1)
        close = -1e3 * np.einsum("ij,ij->i", rel, vrel) / np.maximum(rng, 1e-12)        # m/s, > 0 closing
        t_c, d_c, reached = self._cpa_now()
        full = self.tg_object_full.isChecked()
        out = []
        for n, i in enumerate(rows):
            lines = [self.own_name() if i == 0 else tr.ids[i]]
            if full:
                lines += [f"altitude {alt[n]:.1f} km", f"speed {spd[n]:.3f} km/s"]
            if i:
                lines += [f"range {_dist_text(rng[n])}", f"closing speed {close[n]:+.0f} m/s"]
            if i and full:
                lines += self._cpa_box(t_c[i - 1], d_c[i - 1], reached[i - 1])
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

    # ------------------------------------------------------------------ live values while a manoeuvre is edited
    def _live_track(self) -> Optional[Tuple[int, np.ndarray, np.ndarray, dict]]:
        """While the run shown is behind the manoeuvres' changes (user, 8 November 2026: the boxes follow the node as
        it is pulled, not the full propagation that comes after): the node's body row, its path on the truth's
        records with the node's real-time preview from its start on (km, km/s), and that preview. None otherwise."""
        if self.truth is None or self.node.edits == getattr(self, "_run_edits", self.node.edits):
            return None
        i = self.node.row()
        c = self.node._preview() if i is not None else None
        if c is None:
            return None
        tr = self.truth
        r, v = tr.r_km[:, i].copy(), tr.v_kms[:, i].copy()
        late = tr.t_s >= c["t0"]
        for a in range(3):
            r[late, a] = np.interp(tr.t_s[late], c["ts"], c["r"][:, a])
            v[late, a] = np.interp(tr.t_s[late], c["ts"], c["v"][:, a])
        return i, r, v, c

    def _live_rv(self, r: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """The states at t (rows: truth rows) with the node's body on its live path (a manoeuvre before t)."""
        lt = self._live_track()
        if lt is None or self.t < lt[3]["t0"]:
            return r, v
        i, _r, _v, c = lt
        r, v = r.copy(), v.copy()
        r[i] = [np.interp(self.t, c["ts"], c["r"][:, a]) for a in range(3)]
        v[i] = [np.interp(self.t, c["ts"], c["v"][:, a]) for a in range(3)]
        return r, v

    def _cpa_now(self):
        """_cpa_all, the closest approaches on the live path while a manoeuvre is edited: the node's body's, every
        object's when it is OWNSHIP's (the same straight-line refinement about the nearest record); cached per
        preview and t."""
        t_c, d_c, reached = self._cpa_all()
        lt = self._live_track()
        if lt is None:
            return t_c, d_c, reached
        i, r_i, v_i, c = lt
        key = self._live_cache
        if key is not None and key[0] is c and key[1] == self.t and key[2] is self.g:
            return key[3]
        tr = self.truth
        rows = np.arange(1, len(tr.ids)) if i == 0 else np.array([i])
        k0 = int(np.searchsorted(tr.t_s, self.t - 1e-9))
        ts = tr.t_s[k0:]
        t_c, d_c, reached = t_c.copy(), d_c.copy(), reached.copy()
        if len(ts):
            r0 = (r_i if i == 0 else tr.r_km[:, 0])[k0:]
            v0 = (v_i if i == 0 else tr.v_kms[:, 0])[k0:]
            last = len(ts) - 1
            for s in range(0, len(rows), CPA_LIVE_CHUNK):
                rr = rows[s:s + CPA_LIVE_CHUNK]
                ro, vo = tr.r_km[k0:, rr], tr.v_kms[k0:, rr]
                if i != 0:
                    ro, vo = r_i[k0:, None], v_i[k0:, None]
                rel, vrel = ro - r0[:, None], vo - v0[:, None]
                rng = np.linalg.norm(rel, axis=2)
                ok = np.isfinite(rng)
                has = ok.any(axis=0)
                ia = np.argmin(np.where(ok, rng, np.inf), axis=0)
                cols = np.arange(len(rr))
                rel_a, vrel_a = rel[ia, cols], vrel[ia, cols]
                dt = -np.einsum("ij,ij->i", rel_a, vrel_a) / np.maximum(np.einsum("ij,ij->i", vrel_a, vrel_a), 1e-30)
                dt = np.clip(np.nan_to_num(dt), ts[np.maximum(ia - 1, 0)] - ts[ia], ts[np.minimum(ia + 1, last)] - ts[ia])
                j = rr - 1
                t_c[j] = np.where(has, np.maximum(ts[ia] + dt, self.t), np.nan)
                d_c[j] = np.where(has, np.linalg.norm(rel_a + vrel_a * dt[:, None], axis=1), np.nan)
                reached[j] = ~((ia == last) & (np.einsum("ij,ij->i", rel_a, vrel_a) < 0))   # still closing at the end
        self._live_cache = (c, self.t, self.g, (t_c, d_c, reached))
        return t_c, d_c, reached

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
        self._screen_src = {}                         # the lines drawn on screen's terms (_screen_lines)
        self._proj_src = None
        if self._global():
            self._frame_global()
        else:
            self._frame_ship()
        self._draw_selection()
        self._draw_encounter()
        self._hide_unused()
        self._place_boxes()
        self._show_info()
        self._render()

    def _update_time_label(self) -> None:
        if self.sc is None:
            self.time_label.setText("")
            return
        utc = scn.epoch(self.sc) + timedelta(seconds=self.t)
        self.time_label.setText(f"time {self.t:9.2f} s   {utc.strftime('%Y-%m-%d %H:%M:%S')} UTC")

    def set_ui_scale(self, step: int) -> None:
        """UI size one step down (-1) or up (+1) among UI_SCALES: the labels, symbols and the selection's square."""
        self.ui_size.step(step)                       # (-> _ui_scale_changed)

    def _ui_scale_changed(self, scale: float) -> None:
        self.ui_scale = scale
        v3.apply_ui_scale(self.plotter, scale)
        self._place_symbols()
        if self.truth is not None:
            self._draw_frame()
        else:
            self._render()

    def _show_sky(self) -> None:
        self.sky.set_visible(self.tg_sky.isChecked(), self.cb_const.isChecked())
        self._render()

    def _frame_global(self) -> None:
        tr = self.truth
        self.sky.set_rotation(None)                   # ICRF
        r, v = sgeo.states_at(tr, self.t)
        self._frame_r = r                             # the bodies as drawn (the selection's square)
        ex = np.isfinite(r[:, 0])
        self._apply_centre(reframe=getattr(self, "_centre_pending", False), notify=False)
        self._centre_pending = False
        self._place_bodies()
        self._body_orbits()
        idx = np.flatnonzero(ex)
        clusters = self._frame_clusters(idx, r[idx])
        self._draw_bodies(idx, r[idx], r, v, 8, clusters)
        obj = idx[idx > 0]                            # the objects at their size (user, 8 November 2026)
        self._draw_shapes(obj, r[obj], self._row_noses(obj, r, v))
        # the paths: one per lone body and one per multitrack (its members' mean, user 29 October 2026)
        groups, rgb = self._path_groups(idx, clusters)
        if self.cb_trails.isChecked():
            self._truth_lines(f"{DYN}trails", groups, rgb, max(0.0, self.t - self.sp_trail.value()), self.t,
                              "trail", line_width=3)
        if self.cb_planned.isChecked():               # from t to the run's end, OWNSHIP's thicker
            own = np.array([len(g) == 1 and g[0] == 0 for g in groups], bool)
            mine = np.array([len(g) == 1 and g[0] == self.node.row() for g in groups], bool)   # its live path
            keep = [k for k in range(len(groups)) if not own[k] and not mine[k]]
            self._truth_lines(f"{DYN}planned", [groups[k] for k in keep], rgb[keep], self.t, self.duration(),
                              "planned", line_width=1.5)
            keep = [k for k in range(len(groups)) if own[k] and not mine[k]]
            self._truth_lines(f"{DYN}planned_ship", [groups[k] for k in keep], rgb[keep], self.t, self.duration(),
                              "planned", line_width=3)
        self.node.draw()
        self._draw_markers()
        if self.cb_orbit.isChecked() and len(groups):     # free fall from t under the local gravity
            self._coast_lines(self._coast(_group_mean(r, groups), _group_mean(v, groups)), rgb)
        # ship attitude triad, enlarged (V3), of constant size on screen ("ship axes" switch)
        m = tr.attitude("ship", self.t)
        if np.isfinite(r[0, 0]) and self._model is not None:   # the ship model there too (user, 30 October 2026)
            self._place_model(r[0] + m @ sgeo.ARRAY_KM, m)     # (OWNSHIP's point: the centre of gravity)
        if self.cb_axes.isChecked():
            self._scaled(f"{DYN}triad", np.repeat(r[0][None, :], 3, axis=0), m.T, cell_rgb=self._triad_rgb,
                         line_width=3)

    def _frame_ship(self) -> None:
        tr, g = self.truth, self.g
        r, v = sgeo.states_at(tr, self.t)
        m = self._view_axes(self.t)                   # the ship's attitude, or the local orbital frame
        s_v = self._ship_to_view(self.t)
        # the origin of the ship and orbital views is the centre of gravity: the rings, the distances and OWNSHIP's
        # symbol are around it, the model turns about it; the lines of sight and the volumes leave from the apertures
        # (user, 30 October 2026)
        self._array_view = sgeo.ARRAY_KM @ s_v.T
        if self._model is not None:
            self._place_model(self._array_view, s_v)
        self.sky.set_rotation(m.T)                    # the sky in the view's frame: it turns with it
        self._planets_drawn = {}
        self._sbody_names = []
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
        los = ro[ex] - sgeo.array_state(tr, self.t, r[0], v[0])[0]
        d = (los / np.linalg.norm(los, axis=1)[:, None]) @ tr.attitude("ship", self.t)   # ship frame: the faces
        face = opt.face_of(d[:, 0], d[:, 1], d[:, 2])[0]
        pos = (ro[ex] - r[0]) @ m                                            # the view's
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
        if self.cb_los.isChecked():                   # the array-object lines (toggle, user 30 October 2026)
            self._poly(f"{DYN}los", np.r_[np.tile(self._array_view, (n, 1)), sight], lines=_pairs(n),
                       cell_rgb=np.r_[los_rgb[lone], los_rgb[[c["k"][0] for c in clusters]].reshape(-1, 3)],
                       line_width=1.5, opacity=0.7)
        # OWNSHIP's symbol at its centre of gravity (user, 30 October 2026; never in a multitrack)
        self._draw_bodies(np.r_[0, js + 1], np.r_[np.zeros((1, 3)), pos], r, v, 9,
                          [dict(c, k=np.asarray(c["k"]) + 1) for c in clusters])
        if not full:                                  # the objects at their size, in the view's axes (user)
            self._draw_shapes(js + 1, pos, self._row_noses(js + 1, r, v) @ m)
        groups, rgb = self._path_groups(js + 1, clusters)
        cols = [gr - 1 for gr in groups]              # (the geometry's columns)
        if self.cb_proj.isChecked() and not full:
            self._range_projection(sight, rgb)
        if self.cb_vectors.isChecked():
            nose = g.nose_ship[k, js] @ s_v.T
            okn = np.isfinite(nose[:, 0]) & lone
            if okn.any():                             # constant on screen, whatever the zoom
                self._scaled(f"{DYN}nose", pos[okn], nose[okn], color=TEXT_COLOR, line_width=2)
        if self.cb_trails.isChecked():                # every object's trail in one pass (one polyline each)
            sel = (g.t_s >= self.t - self.sp_trail.value()) & (g.t_s <= self.t)
            self._ship_lines(f"{DYN}trails", cols, rgb, sel, "trail", line_width=2)
        if self.cb_planned.isChecked():               # the records from t to the run's end
            keep = [kk for kk, gr in enumerate(groups) if not (len(gr) == 1 and gr[0] == self.node.row())]
            self._ship_lines(f"{DYN}planned", [cols[kk] for kk in keep], rgb[keep], g.t_s >= self.t, "planned",
                             line_width=1.5)
        self.node.draw()
        if self.cb_orbit.isChecked():                 # the free falls relative to OWNSHIP's own (user)
            self._coast_lines(self._ship_coast(r, v, _group_mean(r, groups), _group_mean(v, groups)), rgb)
        self._screen_pts = {g.ids[j]: pos[jj] for jj, j in enumerate(js)}
        self._apply_centre(reframe=getattr(self, "_centre_pending", False))
        self._centre_pending = False
        self._axes_dirs = s_v.T                       # (rows: the ship's axes in the view)
        if self.cb_axes.isChecked():                  # the ship's axes, constant on screen
            self._scaled(f"{DYN}ship_axes", np.zeros((3, 3)), s_v.T, cell_rgb=self._triad_rgb, line_width=3)
            self._labels(f"{DYN}ship_axes_lbl", 1.05 * self._screen_len(np.zeros((1, 3)))[:, None] * s_v.T,
                         ["X", "Y", "Z (nose)"], TEXT_COLOR, 15)
        self._vol_view = s_v
        self._place_volumes()
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
        cam = self.plotter.camera
        pos = np.asarray(cam.GetPosition())
        px = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / max(self.plotter.ren_win.GetSize()[1], 1)
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
            if b in MOON_DATA:                                             # (two ephemeris calls: a moon wider than
                seen = size >= px * float(np.linalg.norm(centre - pos))    # a pixel only; 41 moons took 9 ms a frame)
                if seen or b not in self._moon_mats:
                    self._moon_mats[b] = self._moon_matrix_of(b, False)
                mats[b] = self._moon_mats[b]
            u = np.eye(4)
            shape = _shape_mesh(b) is not None                          # (a shape model: in km already)
            u[:3, :3] = (1.0 if shape else size) * (m_ship.T @ mats[b])
            u[:3, 3] = centre
            _set_user_matrix(e[2], u)
            e[2].SetVisibility(True)
            if b == "earth":                                               # the atmosphere's halo at the limb
                self._used.add(name + "_atm")
                h = self._dyn.get(name + "_atm")
                if h is None:
                    h = self._dyn[name + "_atm"] = [None, None, _add_atmosphere(self.plotter, name + "_atm")]
                ua = np.eye(4)
                ua[:3, :3], ua[:3, 3] = size * np.eye(3), centre
                _set_user_matrix(h[2], ua)
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
                    _set_user_matrix(h[2], ur)
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
        self._sbody_names = list(zip(pts, names))      # (named by _planet_marks, at each camera change too)
        self._planet_marks()

    # ------------------------------------------------------------------ V5 mission volumes
    def _clear_volumes(self) -> None:
        if self._vol:
            for a in self._vol["actors"]:
                self.plotter.remove_actor(a, render=False)
            if self._vol.get("labels") is not None:
                self.plotter.remove_actor(self._vol["labels"].actor, render=False)
            if self._vol.get("line_labels") is not None:
                self._vol["line_labels"].clear()
        self._vol = None

    def _poll_profiles(self) -> None:
        """Option (c): a volume's cost profile became ready in the background: the range gauge again."""
        from core import cost_profile
        if cost_profile.generation() != self._profile_gen:
            self._profile_poll_on = False
            self._draw_volumes(force=True)
        elif cost_profile.pending():
            QTimer.singleShot(700, self._poll_profiles)
        else:
            self._profile_poll_on = False

    def _draw_volumes(self, force: bool = False) -> None:
        """V5: the mission's volumes at t. Built once from the navigation at t (the sectors resolved in the ship
        frame); then each one only TURNED with the change of its frame (ship -> sector matrix at t against the one
        it was built with), rebuilt when a frame has turned by more than VOL_REBUILD_DEG (the clipping by the faces
        and the scan window is the ship's) or when asked (force)."""
        if self.mission is None or not self.cb_volumes.isChecked():
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
                turns, rebuild = [], _ground_moved(self._vol.get("ground"), _ground_view(nav))
                for raw, m_ref, off_ref in self._vol["frames"]:
                    m_t, off_t = frames.sector_matrix(raw, nav)
                    rot = np.eye(3) if m_t is None else m_t.T @ m_ref
                    rebuild |= (math.degrees(float(np.linalg.norm(att.rotvec(rot)))) > VOL_REBUILD_DEG
                                or abs(off_t - off_ref) > VOL_REBUILD_DEG)
                    turns.append(rot)
                if not rebuild or getattr(self, "defer_rebuild", False):   # (light playback: turned, rebuilt later)
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
            frames.parse_nav(m2["global"].get("nav"))     # (a navigation the model refuses: no volume at all)
        except (OSError, ValueError, KeyError) as exc:
            self.status.setText(f"mission volumes: {exc}")
            return
        # each sector resolved alone (10 November 2026): a sector its frame refuses at t (a limb above the ship) is
        # left out and said, the others drawn
        resolved, skipped = [], []
        for i, sec_raw in enumerate(m2.get("sectors", [])):
            errs = frames.validate_sector(sec_raw, m2["global"], str(sec_raw.get("id", f"#{i + 1}")))
            try:
                resolved.append(None if errs else frames.resolve_sector(sec_raw, m2["global"]))
            except (ValueError, KeyError, TypeError) as exc:
                errs = [str(exc)]
                resolved.append(None)
            if errs:
                skipped.append(errs[0])
        res = {"global": m2["global"], "sectors": resolved}
        if skipped:
            self.status.setText("mission volumes not drawn: " + "; ".join(skipped))
        self._clear_volumes()
        gl = res.get("global", {})
        e0, e1 = float(gl.get("eps_min_deg", -90.0)), float(gl.get("eps_max_deg", 90.0))
        try:                                         # the sky at t in the ship frame: the ground rule (P4 Q9)
            sky_ship = engine.sky_at(self.sc, self.truth, self.t) if self.truth is not None else None
        except Exception:  # noqa: BLE001 -- no sky: the volumes at their full range
            sky_ship = None
        vol = {"actors": [], "frames": [], "per_sector": [], "centres": [], "texts": [], "ground": _ground_view(nav)}
        alloc = None
        if self.range_gauge:                         # gauge (c): the final allocation rule over every volume (R2)
            from core import cost_estimate as ce
            table = ce.cached_table()
            secs = [{**sec, "id": raw.get("id") or f"_idx{i}"}
                    for i, (raw, sec) in enumerate(zip(mis.get("sectors", []), res.get("sectors", []))) if sec is not None]
            try:
                from core import calibration          # pass R3d (M1): the last exact run of each volume
                raws = mis.get("sectors", [])
                calib = calibration.view_calibration(raws, [raw.get("id") or f"_idx{i}" for i, raw in enumerate(raws)])
                alloc = (ce.live_allocation_async(table, secs, self.grid, e0, e1, sky_ship, gl, calib=calib,
                                                  relief="background", tag="global")
                         if table is not None else None)
            except Exception:  # noqa: BLE001 -- the rule refuses a mission mid-edit: no gauge
                alloc = None
            if (not alloc or alloc.get("stale") or alloc.get("estimating") or alloc.get("relief_pending")) \
                    and table is not None and not getattr(self, "_profile_poll_on", False):
                from core import cost_profile          # option (c): the gauge redrawn when the profiles are ready
                self._profile_poll_on, self._profile_gen = True, cost_profile.generation()
                QTimer.singleShot(700, self._poll_profiles)
        for i, (raw, sec) in enumerate(zip(mis.get("sectors", []), res.get("sectors", []))):
            if sec is None:
                continue
            if (raw.get("id") or f"_idx{i}") in self._hidden_ids:
                continue
            # E3 (user, 10 November 2026): the executed volumes, one per face in its colour, and the outline of
            # the WHOLE watched volume at its range in the sector's colour (its coverage hole: inside it, not filled)
            try:
                ratio_of = (cgeo.held_ratio_function(self.grid, sec, e0, e1, sky_ship, alloc, raw.get("id") or f"_idx{i}")
                            if self.range_gauge else None)
                if ratio_of is not None:                     # the range gauge (km): held range, shortfall
                    green, red = cgeo.build_range_gauge(self.grid, sec, ratio_of, float(sec["range_km"]))
                    parts = [("green", green, None), ("red", red, None)]
                    outline = cgeo.build_gauge_outline_km(self.grid, sec, sky_ship)   # (its limits: edges)
                else:
                    parts, outline = cgeo.build_sector_volumes_km_smooth(self.grid, sec, e0, e1, sky_ship)
                    parts = [(k, p_, None) for k, p_ in parts]
            except (KeyError, TypeError, ValueError):
                continue
            col = cgeo.sector_color(i)
            acts = []
            for k, poly_k, _out_k in parts:
                if poly_k.n_points:
                    color = {"green": cgeo.GAUGE_GREEN, "red": cgeo.GAUGE_RED}.get(k) or cgeo.FACE_COLORS[k]
                    gauge = k in ("green", "red")                 # (gauge: unlit, its colours kept at any range)
                    acts.append(self.plotter.add_mesh(poly_k, color=color, opacity=0.5 if gauge else 0.2, culling=None,
                                                      lighting=not gauge, name=f"{VOL}{i}_s{k}", render=False))
            if outline.n_points:
                acts.append(self.plotter.add_mesh(outline, color=col, line_width=2, name=f"{VOL}{i}_o", render=False))
            polys = [p_ for _k, p_, _o in parts if p_.n_points]
            poly = polys[0] if polys else pv.PolyData()
            m_ref, off_ref = frames.sector_matrix(raw, nav)
            vol["frames"].append((raw, np.eye(3) if m_ref is None else m_ref, off_ref))
            vol["per_sector"].append(acts)
            vol["actors"] += acts
            vol["centres"].append(np.asarray(poly.center if poly.n_points else outline.center, float))
            vol["texts"].append(f"{raw.get('id', '?')}  R={raw.get('range_km', 0):.0f} km  "
                                f"RR={raw.get('rr_target_s', 0):g} s")
            vol.setdefault("ids", []).append(str(raw.get("id") or f"_idx{i}"))
        vol["rot"] = [np.eye(3)] * len(vol["per_sector"])
        vol["labels"] = _Labels(self.plotter, TEXT_COLOR, 16) if vol["per_sector"] else None
        if vol["per_sector"]:                          # two lines, two colours (user, 11 November 2026)
            if vol["labels"] is not None:
                self.plotter.remove_actor(vol["labels"].actor, render=False)
            vol["labels"] = None
            vol["line_labels"] = v3.LineLabels(self.plotter, 16, box=False)
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
        x, y = obj.GetEventPosition()
        if self.read_only:                            # (read only: OWNSHIP's Info alone, user 10 November 2026)
            if obj.GetRepeatCount() >= 1 and self._pick(x, y) == OWNSHIP:
                self.centre = OWNSHIP                 # a double-click on OWNSHIP: the view back on it, the zoom
                self.recentre()                       # kept (user, 11 November 2026)
            return
        best, best_d = self._pick(x, y, with_distance=True)
        ref = None
        hit = self._marker_at(x, y)
        if hit is not None and hit[0] < best_d:      # an event's mark nearer than any body: its body (and manoeuvre)
            _d, row, ref = hit
            best = OWNSHIP if row == 0 else self.truth.ids[row]
        double = obj.GetRepeatCount() >= 1
        members = best[len(MT_SEL):].split("|") if best and best.startswith(MT_SEL) else [best]
        if self.node.target is not None and not any(self.node.owns("ship" if b == OWNSHIP else b) for b in members):
            self.node.release()                       # another body or nothing: the manoeuvre let go (user)
            self._draw_frame()
        if best is not None:
            self._select(best)
            self.picked.emit(best)                    # the editor's tree follows (user, 29 October 2026)
            if ref is not None and ref[2] >= 0:       # ... and its action list, the node shown (user, 8 Nov 2026)
                self.manoeuvre_picked.emit(*ref)
            if double:
                self.set_centre(members[0])           # (a multitrack: centred on its first member)
            return
        body = self._body_under(x, y)
        if body is not None:
            self._select(BODY_SEL + body)                # a celestial body: its facts (user, 29 October 2026)
        if double:
            self.set_centre(body or CENTRAL)

    def _marker_at(self, x: float, y: float) -> Optional[Tuple[float, int, Tuple[str, str, int]]]:
        """(distance px, truth row, origin) of the event's mark drawn nearest the display point (x, y), within
        CLICK_PX (global view, events shown, marks still to come as _draw_markers); None: none."""
        mk = getattr(self, "_markers", None)
        refs = getattr(self, "_marker_refs", None)
        if mk is None or refs is None or not len(mk[0]) or not self._global() or not self.cb_events.isChecked():
            return None
        t, pts, _labels, rows = mk
        ren = self.plotter.renderer
        best = None
        for k in np.flatnonzero(t >= self.t - 1e-9):
            w = pts[k]
            ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
            ren.WorldToDisplay()
            dx, dy, _ = ren.GetDisplayPoint()
            dd = math.hypot(dx - x, dy - y)
            if dd < CLICK_PX and (best is None or dd < best[0]):
                best = (dd, int(rows[k]), refs[k])
        return best

    def _pick(self, x: float, y: float, with_distance: bool = False):
        """The selection key of the body drawn nearest the display point (x, y), within CLICK_PX; None: none
        (with_distance: (key, its distance in px, CLICK_PX when none))."""
        if self._global():
            r, _ = sgeo.states_at(self.truth, self.t)
            pts = {(OWNSHIP if i == 0 else oid): r[i] for i, oid in enumerate(self.truth.ids) if np.isfinite(r[i, 0])}
        else:
            pts = dict(getattr(self, "_screen_pts", {}), **{OWNSHIP: np.zeros(3)})
        for key, q in getattr(self, "_mt_click", {}).items():     # a multitrack: one target, its members not
            for oid in key[len(MT_SEL):].split("|"):
                pts.pop(oid, None)
            pts[key] = q
        ren = self.plotter.renderer
        best, best_d = None, CLICK_PX
        for oid, w in pts.items():
            ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
            ren.WorldToDisplay()
            dx, dy, _ = ren.GetDisplayPoint()
            dd = math.hypot(dx - x, dy - y)
            if dd < best_d:
                best, best_d = oid, dd
        return (best, best_d) if with_distance else best

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

    # ------------------------------------------------------------------ the target (user, 1 November 2026)
    def _rows_of(self, key: Optional[str]) -> Optional[np.ndarray]:
        """The truth rows of a selection key (OWNSHIP, an object, a multitrack: its members); None: a celestial body,
        nothing, or bodies absent from this run."""
        if not key or key.startswith(BODY_SEL):
            return None
        tr = self.truth
        ids = key[len(MT_SEL):].split("|") if key.startswith(MT_SEL) else [key]
        rows = [0 if x == OWNSHIP else tr.row(x) for x in ids if x == OWNSHIP or x in tr.ids]
        return np.array(rows, int) if rows else None

    def _on_right_click(self, x: float, y: float) -> None:
        """A right click (not a drag) on a unit other than the one selected: its menu (a body must be selected)."""
        sel = self._rows_of(self.selected) if self.truth is not None and not self.read_only else None
        if sel is None:
            return
        key = self._pick(x, y)
        rows = self._rows_of(key)
        if rows is not None and not np.intersect1d(rows, sel).size:
            QTimer.singleShot(0, lambda: self._target_menu(key))     # (out of the interactor's event)

    def _target_menu(self, key: str) -> None:
        menu = QMenu(self)
        define = menu.addAction("Define as target")
        if menu.exec_(QCursor.pos()) is define:
            self.define_target(key)

    def define_target(self, key: Optional[str]) -> None:
        """The target (one at a time: it replaces the former one): a unit, or a multitrack, whose members are the ones
        of its planned path."""
        self.target_key = key
        self._draw_frame()

    def _track(self, rows: np.ndarray) -> Tuple[np.ndarray, np.ndarray, object]:
        """The planned path of rows on the truth's records, (n_t, 3) km and km/s, their mean for several (as the
        multitrack's path is drawn), the manoeuvre node's real-time preview from its start on when the node's body is
        one of them; and that preview (None: none)."""
        tr = self.truth
        r, v = tr.r_km[:, rows].copy(), tr.v_kms[:, rows].copy()
        pre = None
        row = self.node.row()
        at = np.flatnonzero(rows == row) if row is not None else ()
        if len(at):
            pre = self.node._preview()
            if pre is not None:
                late = tr.t_s >= pre["t0"]
                for c in range(3):
                    r[late, at[0], c] = np.interp(tr.t_s[late], pre["ts"], pre["r"][:, c])
                    v[late, at[0], c] = np.interp(tr.t_s[late], pre["ts"], pre["v"][:, c])
        whole = [np.arange(len(rows))]
        return _group_mean(r, whole)[:, 0], _group_mean(v, whole)[:, 0], pre

    def _mark_at(self, t: float, q: np.ndarray) -> np.ndarray:
        """A place of the central body's frame at the time t as the view draws the planned paths."""
        if self._global():
            return np.asarray(q, float)
        q = (q - sgeo.states_at(self.truth, t)[0][0]) @ self._view_axes(min(t, self.duration()))
        return self._to_plane(q[None])[0] if self._full_projection() else q

    def _glyph(self, name: str, q: np.ndarray, shape: tuple, px: float, **style) -> None:
        """The polyline shape (points in half widths, a NaN point lifts the pen) at each of the points q, px on screen
        and facing the camera, kept so by the camera changes (_place_symbols)."""
        q = np.asarray(q, float)
        self._glyphs[name] = (q, shape, px)
        cuts = [-1] + [n for n, p in enumerate(shape) if p[0] != p[0]] + [len(shape)]
        one = [np.arange(a + 1, b) for a, b in zip(cuts, cuts[1:]) if b - a > 2]
        lines = np.concatenate([np.r_[len(s), s + n * len(shape)] for n in range(len(q)) for s in one])
        self._poly(name, self._glyph_pts(q, shape, px), lines=lines, **style)

    def _glyph_pts(self, q: np.ndarray, shape: tuple, px: float) -> np.ndarray:
        c = self._sym_corners(q, px).reshape(-1, 4, 3)
        mid, right, up = 0.5 * (c[:, 0] + c[:, 2]), 0.5 * (c[:, 1] - c[:, 0]), 0.5 * (c[:, 3] - c[:, 0])
        s = np.nan_to_num(np.array(shape, float))
        return (mid[:, None] + s[None, :, 0, None] * right[:, None] + s[None, :, 1, None] * up[:, None]).reshape(-1, 3)

    def _target_pt(self, rows: np.ndarray) -> Optional[np.ndarray]:
        """Where the view draws the target: its multitrack, or its members' mean."""
        key = self.target_key
        ids = key[len(MT_SEL):].split("|") if key.startswith(MT_SEL) else [key]
        if ids[0] in getattr(self, "_mt_of", {}):
            return np.asarray(self._mt_of[ids[0]], float)
        if self._global():
            pts = sgeo.states_at(self.truth, self.t)[0][rows]
        else:
            shown = getattr(self, "_screen_pts", {})
            pts = np.array([np.zeros(3) if x == OWNSHIP else shown.get(x, np.full(3, np.nan)) for x in ids])
        pts = pts[np.isfinite(pts[:, 0])]
        return pts.mean(axis=0) if len(pts) else None

    def _rgb_text(self, row: int) -> str:
        return "#%02x%02x%02x" % tuple(int(x) for x in self._row_rgb[row])

    def _encounter_planes(self, sel: np.ndarray, tgt: Optional[np.ndarray], tracks: tuple) -> tuple:
        """(orbit normals of the selected path and of the target's, its passages through the equator and through the
        target's plane at the same instant), kept while the run, the pair and the node's preview stay."""
        r_a, v_a, pre_a, r_b, v_b, pre_b = tracks
        tr = self.truth
        pair = None if tgt is None else tgt.tobytes()
        c = getattr(self, "_enc_cache", None)
        if (c is None or c[0] is not tr or c[1] != sel.tobytes() or c[2] != pair or c[3] is not pre_a
                or c[4] is not pre_b):
            n_a = enc.orbit_normal(r_a, v_a)
            n_b = None if tgt is None else enc.orbit_normal(r_b, v_b)
            c = self._enc_cache = (tr, sel.tobytes(), pair, pre_a, pre_b, n_a, n_b,
                                   enc.crossings(tr.t_s, r_a, tr.body.equator()[:, 2]),
                                   [] if tgt is None else enc.crossings(tr.t_s, r_a, n_b))
        return c[5:]

    def _node_marks(self, name: str, found: list, label: str, angle, color: str, up: tuple, down: tuple,
                    at: list, texts: list) -> None:
        """The nodes found drawn on the planned path, ascending as up and descending as down, with their texts."""
        for shape, asc in ((up, True), (down, False)):
            now = [x for x in found if x.ascending == asc]
            if now:
                q = np.array([self._mark_at(x.t_s, x.r_km) for x in now])
                self._glyph(f"{DYN}{name}{int(asc)}", q, shape, MARK_PX, color=color, line_width=2)
                at.append(q)
                side, what = "Ascending" if asc else "Descending", "inclination" if label == "equator" else "plane angle"
                texts.extend(f"{side} node ({label})\n{what} {angle(x):.2f} deg\nin {x.t_s - self.t:.0f} s" for x in now)

    def _draw_encounter(self) -> None:
        """The target's four bars; on the planned path of the selected body its next passages through the equator of
        the central body and, with a target, through the target's plane at the same instant (where a plane change
        aligns the two) with the planes' angle; the closest approach of the two bodies at one instant marked on both
        paths. They follow the manoeuvre node at each of its changes (_track)."""
        tgt = self._rows_of(self.target_key)
        if tgt is not None:
            q = self._target_pt(tgt)
            if q is not None:
                self._glyph(f"{DYN}target", q[None], TARGET_BARS, TARGET_PX, color=self._rgb_text(tgt[0]), line_width=3)
        sel = self._rows_of(self.selected)
        if sel is None or not self.cb_planned.isChecked() or self.read_only:   # (no orbital nodes in the mission
            return                                                          # preparation; user, 11 November 2026)
        if tgt is not None and np.intersect1d(sel, tgt).size:
            tgt = None                                 # (the target selected: no pair)
        tr = self.truth
        r_a, v_a, pre_a = self._track(sel)
        r_b, v_b, pre_b = self._track(tgt) if tgt is not None else (None, None, None)
        n_a, n_b, equator, nodes = self._encounter_planes(sel, tgt, (r_a, v_a, pre_a, r_b, v_b, pre_b))
        at, texts = [], []
        color = self._rgb_text(sel[0])                # the nodes in the colour of the path they lie on
        if self._global() or 0 not in sel:            # (the ship view draws no path of its own for OWNSHIP)
            self._node_marks("eqnode", enc.next_nodes(equator, self.t), "equator",
                             lambda x: enc.angle_deg(n_a[x.k], tr.body.equator()[:, 2]), color, EQ_NODE_UP, EQ_NODE_DOWN,
                             at, texts)
            self._node_marks("tgnode", enc.next_nodes(nodes, self.t), "target plane",
                             lambda x: enc.angle_deg(n_a[x.k], n_b[x.k]), color, NODE_UP, NODE_DOWN,
                             at, texts)
        cpa = None if tgt is None else enc.closest_approach(tr.t_s, r_a, v_a, r_b, v_b, self.t)
        if cpa is not None:
            ends = [(row, q, tag) for row, q, tag in ((sel, cpa.r_a_km, "a"), (tgt, cpa.r_b_km, "b"))
                    if self._global() or 0 not in row]
            for row, q, tag in ends:
                self._glyph(f"{DYN}cpa_{tag}", self._mark_at(cpa.t_s, q)[None], CPA_CROSS, MARK_PX,
                            color=self._rgb_text(row[0]), line_width=2)
            if ends:
                over, under = ("", "") if cpa.reached else ("> ", "< ")
                at.append(self._mark_at(cpa.t_s, ends[0][1])[None])
                texts.append(f"Closest approach\ndistance {under}{_dist_text(cpa.range_km)}\n"
                             f"in {over}{cpa.t_s - self.t:.1f} s\nrelative speed {cpa.v_rel_kms:.3f} km/s")
        if at:
            self._databox(f"{DYN}encounter", np.concatenate(at), texts)

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
        lines = [_kv("name", f"{len(ids)} x {cls}"), _kv("characterisation", cls), _kv("classification", ident),
                 _kv("time", f"{self.t:.2f} s"), _kv("members", "range")]
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

        r, v = self._live_rv(*sgeo.states_at(tr, self.t))   # (live while a manoeuvre is edited)
        if not np.isfinite(r[i, 0]):
            self._set_info("\n".join([_kv("name", self.selected), _kv("characterisation", g.classes[j]),
                                      _kv("time", f"{self.t:.2f} s"), _kv("state", "not there at t")]),
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
        lines = [_kv("name", self.selected), _kv("characterisation", g.classes[j]), _kv("classification", ident),
                 _kv("time", f"{self.t:.2f} s")]
        lines += self._orbit_lines(r[i], v[i])
        lines += [_kv("range", f"{rng:.3f} km"), _kv("range rate", f"{rr:+.1f} m/s")]
        lines += self._cpa_lines(i, j)                # closest approach to OWNSHIP (user, 29 October 2026)
        lines.append(_kv("radar face", f"F{f + 1}" if f >= 0 else "none"))
        if f >= 0:
            lines.append(_kv("angle off the face axis", f"{math.degrees(math.acos(min(1.0, float(w)))):.2f} deg"))
        lines += [_kv("recorded at", f"{g.t_s[k]:.1f} s"), _kv("aspect angle", f"{g.aspect_deg[k, j]:.1f} deg"),
                  _kv("radar cross section (S band)", f"{g.rcs_s_dbsm[k, j]:.1f} dBsm"), _kv("radar cross section (Ku band)", f"{g.rcs_ku_dbsm[k, j]:.1f} dBsm"),
                  _kv("hidden by", g.body_names[occ] if occ >= 0 else "no")]
        if np.isfinite(g.block_loss_db[k, j]):
            lines += [_kv("masked by the ship", "yes" if g.masked[k, j] else "no"),
                      _kv("loss through the ship", f"{max(0.0, float(g.block_loss_db[k, j])):.2f} dB / pass")]
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
        cands = [(rng(x), x) for x in (a, b, 0.5 * (a + b))] + [(rng(float(g.t_s[ia])), float(g.t_s[ia]))]
        d, t_cpa = min(cands)
        reached = not (ia == len(g.t_s) - 1 and np.isfinite(g.range_rate_mps[ia, j]) and g.range_rate_mps[ia, j] < 0)
        res = (max(t_cpa, self.t), d, reached)
        self._cpa_cache = getattr(self, "_cpa_cache", {})
        self._cpa_cache[i] = (min(self.t, t_cpa), t_cpa, res)
        return res

    def _cpa_lines(self, i: int, j: int) -> List[str]:
        """The Info rows of the closest approach: time before it, its range (live while a manoeuvre of this body or
        of OWNSHIP is edited)."""
        lt = self._live_track()
        if lt is not None and lt[0] in (0, i):
            t_c, d_c, ok = self._cpa_now()
            c = (float(t_c[i - 1]), float(d_c[i - 1]), bool(ok[i - 1])) if np.isfinite(t_c[i - 1]) else None
        else:
            c = self._cpa(i, j)
        if c is None:
            return []
        t_cpa, d, reached = c
        dist = _dist_text(d)
        if not reached:                               # still closing at the run's end
            return [_kv("closest approach in", f"> {t_cpa - self.t:.1f} s (after the end)"),
                    _kv("closest distance", f"< {dist}")]
        return [_kv("closest approach in", f"{t_cpa - self.t:.1f} s" if t_cpa > self.t + 1e-6 else "now (receding)"),
                _kv("closest distance", dist)]

    def _orbit_lines(self, r: np.ndarray, v: np.ndarray) -> List[str]:
        """The state and the orbital elements in the central body's equatorial axes (user, 1 November 2026)."""
        body = self.truth.body
        r, v = r @ body.equator(), v @ body.equator()
        el = prop.state_to_elements(r, v, body.mu)
        return (_vec_lines("position", r, "km", 3) + _vec_lines("velocity", v, "km/s", 5) +
                [_kv("reference frame", f"{body.name} equator"), _kv("semi-major axis", f"{el['a_km']:.3f} km"),
                 _kv("eccentricity", f"{el['e']:.6f}"), _kv("inclination", f"{el['i_deg']:.3f} deg"),
                 _kv("longitude of ascending node", f"{el['raan_deg']:.3f} deg"),
                 _kv("argument of periapsis", f"{el['argp_deg']:.3f} deg"),
                 _kv("true anomaly", f"{el['nu_deg']:.3f} deg")])

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
        r, v = self._live_rv(*sgeo.states_at(tr, self.t))   # (live while a manoeuvre is edited)
        m = tr.attitude("ship", self.t)
        w = engine.ship_rate_dps(tr, self.t)
        k = self._k()
        alt = float(np.linalg.norm(r[0])) - tr.body.radius_km
        lines = [_kv("name", self.own_name()), _kv("classification", OWNSHIP_IDENTITY), _kv("time", f"{self.t:.2f} s")]
        lines += self._orbit_lines(r[0], v[0])
        lines += [_kv("altitude", f"{alt:.1f} km"),
                  _kv("speed", f"{1e3 * float(np.linalg.norm(v[0])):.1f} m/s")]
        lines += _vec_lines("nose direction", m[:, 2] @ tr.body.equator(), "", 4)    # the ship's Z axis, body equator
        lines += _vec_lines("turn rate", w, "deg/s", 3)                         # ship frame
        lines += [_kv("turn rate", f"{float(np.linalg.norm(w)):.3f} deg/s"), _kv("recorded at", f"{g.t_s[k]:.1f} s"),
                  _kv("Sun hidden", f"{100.0 * float(g.ship_shadow[k]):.0f} %")]
        self._set_info("\n".join(lines), symbol_path("ownship", None))
