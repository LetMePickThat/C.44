#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VIEW3D -- the parts of a 3D view shared by the two tools (user, 10 November 2026: the Radar Mission Preparation's view
aligned with the Scenario Editor's, one component each, one behaviour to maintain):

  - the background: the black of space (BG_COLOR), light texts (TEXT_COLOR), the stars under the scene (gui/sky.py)
    -- space_background;
  - the view's options panel: switches in sections over OPTION_COLUMNS columns, pairs in a small frame (row, pair,
    title, options_grid), the "Recenter view" button (recenter_button) and the "UI size" control (UiSize);
  - the labels at 3D points that follow the UI size and are blanked behind the camera (Labels, apply_ui_scale);
  - the mouse rotation: a turntable on the right button about the view's vertical, the camera turning about the
    scene's pivot, the left button left to picking (install_rotation; gui/scene_view.install_turntable, lock_pivot);
  - the objects' symbols (MIL-STD-2525C, tools/gen_symbols.py) as square billboards facing the camera, SYMBOL_PX on
    screen x the UI size, and the identity colours (symbol_path, symbol_texture, billboard, SymbolLayer;
    IDENTITY_COLORS, tool.display.identity_colors).

First user: gui/sim_view.py (the Scenario Editor, unchanged on screen); then gui/scene_view.py (pass V2).
"""
from __future__ import annotations

import math
import os
import weakref
from functools import lru_cache
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import pyvista as pv
from PyQt5.QtCore import QEvent, QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QWidget
from vtkmodules.util.numpy_support import numpy_to_vtk

from core.params import P
from gui import scene_view as sv
from gui import sky as gsky

BG_COLOR = "#000000"                              # background of the 3D views: the black of space (user, 29 October 2026)
TEXT_COLOR = "#e6e6e6"
MUTED_COLOR = "#9aa0aa"
TURN_DEG_PX = float(P.get("tool.display.mouse_turn_deg_px"))   # mouse rotation (user)
OPTION_COLUMNS = 2                                # the view's switches: two columns, one grid for its sections (user)
PAIR_NAME = "optionPair"
PAIR_PAD = 2                                      # px above and under the switches in a pair's frame
OPTION_MARGIN = 5                                 # px left of every switch: a pair's frame fits in it, all aligned
# "UI size" of the view's options (user, 29 October 2026): the icons and texts of the 3D view (symbols, the
# selection's square, labels: ids, range and azimuth marks, sectors, axes, events, bodies) x one of these
UI_SCALES = (0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0)
UI_SCALE_START = 1.0                              # (the sizes below: those of the former 150 %, user 30 October 2026)
UI_BUTTON_PX = 28                                 # the width of the UI size's - and + buttons, px
SYMBOL_DIR = Path(__file__).resolve().parent.parent / "assets" / "symbols"   # <classification>_<class>.png
SYMBOL_PX = 42                                    # px: a symbol on screen in the 3D views (user, 30 October 2026: 28 x 1.5)
# the objects in the colour of their classification (user, 29 October 2026), the identity frames' colours of the
# symbology (tools/gen_symbols.py), both read from tool.display.identity_colors
IDENTITY_COLORS = dict(P.section("tool.display.identity_colors"))


def rgb(hex_color: str) -> Tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


# ----------------------------------------------------------------------------- background
def space_background(plotter) -> gsky.Sky:
    """The black of space behind the scene and the stars under it (their own renderer); returns the sky."""
    plotter.set_background(BG_COLOR)
    return gsky.Sky(plotter, BG_COLOR)


# ----------------------------------------------------------------------------- mouse
class _KeepScale(QObject):
    """keep_scale_on_resize's event filter on a plotter's widget."""

    def __init__(self, plotter):
        super().__init__(plotter.interactor)
        self._p = weakref.ref(plotter)

    def eventFilter(self, obj, ev):  # noqa: N802 (Qt)
        if ev.type() == QEvent.Resize:
            h0, h1 = ev.oldSize().height(), ev.size().height()
            p = self._p()
            if p is not None and h0 >= MIN_RESIZE_PX and h1 >= MIN_RESIZE_PX and h0 != h1:
                try:
                    cam = p.renderer.GetActiveCamera()
                    k = h1 / h0
                    if cam.GetParallelProjection():
                        cam.SetParallelScale(cam.GetParallelScale() * k)
                    else:
                        t = math.tan(math.radians(cam.GetViewAngle()) / 2.0) * k
                        cam.SetViewAngle(min(MAX_VIEW_ANGLE_DEG, max(MIN_VIEW_ANGLE_DEG, 2.0 * math.degrees(math.atan(t)))))
                    p.renderer.ResetCameraClippingRange()
                except Exception:  # noqa: BLE001 -- a plotter being closed
                    pass
        return False


MIN_RESIZE_PX = 50                 # (the first layout passes of a window: not a resize by the operator)
MIN_VIEW_ANGLE_DEG, MAX_VIEW_ANGLE_DEG = 0.5, 170.0


def keep_scale_on_resize(plotter) -> None:
    """The view keeps its zoom when its widget is resized (user, 13 November 2026: the Diagnosis panel or the left
    column moved with the mouse): VTK keeps the vertical view angle, so a shorter view showed the scene smaller; the
    angle (the parallel scale in a parallel projection) follows the height, the pixels per unit stay."""
    f = _KeepScale(plotter)
    plotter.interactor.installEventFilter(f)
    plotter._keep_scale = f


def install_rotation(plotter, axis: Callable[[], Sequence[float]], on_click: Optional[Callable] = None) -> None:
    """The mouse rotation of both tools (user, 29 October 2026): right drag = turntable about the view's vertical
    axis(), TURN_DEG_PX per pixel, the camera turning about the scene's pivot; a right click without a drag calls
    on_click; the left button is left to picking."""
    sv.lock_pivot(plotter)
    sv.install_turntable(plotter, TURN_DEG_PX, button="right", axis=axis, on_click=on_click)


# ----------------------------------------------------------------------------- options panel
def row(*widgets: QWidget) -> QWidget:
    """Widgets side by side on one line of the options."""
    w = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(OPTION_MARGIN, 0, 0, 0)
    for x in widgets:
        lay.addWidget(x)
    lay.addStretch(1)
    return w


def pair(first: QWidget, off_label: QLabel, second: QWidget) -> Tuple[QFrame, QWidget, QWidget]:
    """Settings working as a pair, in a small frame over the whole line (user, 30 October 2026): (the frame, the
    widget of the first column, the one of the second), the two-way switch in line with that column's switches, the
    name of its off side before it."""
    frame = QFrame()
    frame.setObjectName(PAIR_NAME)
    frame.setStyleSheet(f"#{PAIR_NAME} {{ border: 1px solid {MUTED_COLOR}; border-radius: 3px; }}")
    frame.setAttribute(Qt.WA_TransparentForMouseEvents)     # (it lies over the switches)
    left = QWidget()
    lay = QHBoxLayout(left)
    lay.setContentsMargins(OPTION_MARGIN, PAIR_PAD, 0, PAIR_PAD)
    lay.addWidget(first)
    lay.addStretch(1)
    lay.addWidget(off_label)
    right = row(second)
    right.layout().setContentsMargins(OPTION_MARGIN, PAIR_PAD, 0, PAIR_PAD)
    return frame, left, right


def title(text: str) -> QLabel:
    """A section title among the view's switches."""
    lb = QLabel(f"{text}:")
    lb.setStyleSheet("font-weight: bold;")
    return lb


Item = Union[QWidget, Tuple[QFrame, QWidget, QWidget]]


def options_grid(sections: Sequence[Tuple[QLabel, Sequence[Item]]]) -> QGridLayout:
    """The view's switches (user, 30 October 2026): one grid for all its sections, OPTION_COLUMNS columns, each
    section under its title; a pair (see pair) on a whole line, its two widgets in the two columns."""
    grid = QGridLayout()
    grid.setContentsMargins(0, 0, 0, 0)
    grid.setVerticalSpacing(4)
    line = 0
    for head, ws in sections:
        grid.addWidget(head, line, 0, 1, OPTION_COLUMNS)
        line += 1
        col = 0
        for w in ws:
            if isinstance(w, tuple):
                line += 1 if col else 0
                col = 0
                grid.addWidget(w[0], line, 0, 1, OPTION_COLUMNS)
                grid.addWidget(w[1], line, 0)
                grid.addWidget(w[2], line, 1)
                line += 1
            else:
                grid.addWidget(w, line, col)
                col = (col + 1) % OPTION_COLUMNS
                line += 0 if col else 1
        line += 1 if col else 0
    for col in range(OPTION_COLUMNS):
        grid.setColumnStretch(col, 1)
    return grid


def recenter_button(slot: Callable[[], None], tip: str) -> QPushButton:
    """The view's "Recenter view" (a plain button, user 30 October 2026)."""
    b = QPushButton("Recenter view")
    b.clicked.connect(lambda _c=False: slot())
    b.setToolTip(tip)
    return b


class UiSize(QObject):
    """The view's "UI size" (user, 29 October 2026): - / + among UI_SCALES and the percentage between them; changed
    sends the new scale (the owner applies it: apply_ui_scale, then its own symbols)."""
    changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scale = UI_SCALE_START
        self.minus = QPushButton("-")                 # plain buttons, seen as clickable (user, 30 October 2026)
        self.minus.clicked.connect(lambda _c=False: self.step(-1))
        self.minus.setToolTip("Smaller icons and texts in the 3D view")
        self.plus = QPushButton("+")
        self.plus.clicked.connect(lambda _c=False: self.step(1))
        self.plus.setToolTip("Larger icons and texts in the 3D view")
        for b in (self.minus, self.plus):
            b.setFixedWidth(UI_BUTTON_PX)
        self.label = QLabel()
        self.show()

    def widget(self) -> QWidget:
        """Its line of the options panel."""
        return row(QLabel("UI size"), self.minus, self.label, self.plus)

    def step(self, k: int) -> None:
        """One step down (-1) or up (+1)."""
        i = int(np.argmin([abs(x - self.scale) for x in UI_SCALES]))
        self.scale = UI_SCALES[int(np.clip(i + k, 0, len(UI_SCALES) - 1))]
        self.show()
        self.changed.emit(self.scale)

    def show(self) -> None:
        self.label.setText(f"{100 * self.scale:.0f} %")
        self.minus.setEnabled(self.scale > UI_SCALES[0])
        self.plus.setEnabled(self.scale < UI_SCALES[-1])


def apply_ui_scale(plotter, scale: float) -> None:
    """The plotter's labels (Labels) at this UI size."""
    plotter._ui_scale = scale
    for lab in list(getattr(plotter, "_ui_labels", ())):
        lab.rescale()


# ----------------------------------------------------------------------------- labels
class Labels:
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
        self.points.SetDataTypeToDouble()             # float32 (VTK's default): 30 m steps at the Moon, labels shake zoomed in
        self.texts.SetName("labels")
        self.poly = vtkPolyData()
        self.poly.SetPoints(self.points)
        self.poly.GetPointData().AddArray(self.texts)
        self.mapper = vtkLabeledDataMapper()
        self.mapper.SetInputData(self.poly)
        self.mapper.SetLabelModeToLabelFieldData()
        self.mapper.SetFieldDataName("labels")
        tp = self.mapper.GetLabelTextProperty()
        tp.SetColor(*(c / 255.0 for c in rgb(color)))
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
        self._pts = np.zeros((0, 3))
        self._shown: Optional[np.ndarray] = None      # the labels written with their text (the others blank)
        # the mapper draws the points behind the camera too, mirrored onto the screen (user, 8 November 2026: data
        # boxes of bodies out of sight piled up in the sky): before each render, their texts blanked
        me = weakref.ref(self)
        plotter.renderer.AddObserver("StartEvent", lambda *_a: me() is not None and me()._cull())

    def rescale(self) -> None:
        """The font at the plotter's UI size."""
        p = self._plotter()
        size = max(1, round(self.base_size * getattr(p, "_ui_scale", 1.0)))
        self.mapper.GetLabelTextProperty().SetFontSize(size)
        self.mapper.Modified()

    def _front(self) -> np.ndarray:
        """Which points lie in front of the camera."""
        p = self._plotter()
        if p is None or not len(self._pts):
            return np.ones(len(self._pts), bool)
        cam = p.renderer.GetActiveCamera()
        fwd = np.asarray(cam.GetDirectionOfProjection(), float)
        return (self._pts - np.asarray(cam.GetPosition(), float)) @ fwd > 0.0

    def _write(self, shown: np.ndarray) -> None:
        """The texts, blank where not shown."""
        self.texts.SetNumberOfValues(len(self._last))
        for i, t in enumerate(self._last):
            self.texts.SetValue(i, "" if not shown[i] else (t if self.box else f" {t}"))
        self._shown = shown
        self.texts.Modified()
        self.poly.Modified()

    def _cull(self) -> None:
        """Before a render: the labels of the points behind the camera blanked (only when that changed)."""
        if not self.actor.GetVisibility() or len(self._pts) != len(self._last):
            return
        shown = self._front()
        if self._shown is None or not np.array_equal(shown, self._shown):
            self._write(shown)

    def set(self, pts: np.ndarray, texts: List[str]) -> None:
        same = list(texts) == self._last
        if not same:
            self._last = list(texts)
            self._shown = None
        pts = np.asarray(pts, float).reshape(-1, 3)
        if same and np.array_equal(pts, self._pts):    # (fixed: every text mapper would be rebuilt for nothing)
            self.actor.SetVisibility(True)
            return
        self._pts = pts.copy()
        self.points.SetData(numpy_to_vtk(pts, deep=True))
        self.points.Modified()
        shown = self._front() if len(self._pts) == len(self._last) else np.ones(len(self._last), bool)
        if self._shown is None or not np.array_equal(shown, self._shown):
            self._write(shown)
        self.poly.Modified()
        self.actor.SetVisibility(True)


# ----------------------------------------------------------------------------- symbols
def symbol_path(identity: str, cls: Optional[str], multi: bool = False) -> Optional[str]:
    """The symbol of a classification / characterisation couple (tools/gen_symbols.py), "ownship" for the ship,
    multi: its multitrack (doubled outline); None when its file is missing."""
    name = "ownship" if identity == "ownship" else f"{'multi_' if multi else ''}{identity}_{cls}"
    path = SYMBOL_DIR / f"{name}.png"
    return str(path) if path.exists() else None


@lru_cache(maxsize=64)
def symbol_texture(path: str) -> pv.Texture:
    tex = pv.Texture(path)
    tex.interpolate = True
    return tex


def billboard(plotter, q: np.ndarray, px: float) -> np.ndarray:
    """The four corners (4 n, 3) of a square of px on screen centred on each point of q (n, 3), facing the camera."""
    cam = plotter.camera
    pos = np.asarray(cam.GetPosition())
    fwd = np.asarray(cam.GetDirectionOfProjection())
    up = np.asarray(cam.GetViewUp(), float)
    up = up - (up @ fwd) * fwd
    up /= np.linalg.norm(up)
    right = np.cross(fwd, up)
    h_px = max(plotter.ren_win.GetSize()[1], 1)
    k = 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / h_px * 0.5 * px
    q = np.asarray(q, float)
    half = np.maximum((q - pos) @ fwd, 1e-12)[:, None] * k                 # world size of px / 2
    return np.stack([q - half * right - half * up, q + half * right - half * up,
                     q + half * right + half * up, q - half * right + half * up], axis=1).reshape(-1, 3)


class SymbolLayer:
    """Symbols at 3D points (the mission preparation's scenario objects; the Scenario Editor keeps its own placement,
    on the same billboard): one textured mesh per symbol file, resized and turned toward the camera by place();
    actor names start with prefix (the owner's visibility rules)."""

    def __init__(self, plotter, prefix: str):
        self.plotter, self.prefix = plotter, prefix
        self._items: Dict[str, list] = {}           # name -> [n, mesh, actor, points]

    def names(self) -> List[str]:
        return list(self._items)

    def set(self, pts: np.ndarray, paths: List[Optional[str]]) -> None:
        """The symbols of these points (a point without a symbol file: none)."""
        pts = np.asarray(pts, float).reshape(-1, 3)
        groups: Dict[str, List[int]] = {}
        for n, path in enumerate(paths):
            if path:
                groups.setdefault(path, []).append(n)
        px = SYMBOL_PX * getattr(self.plotter, "_ui_scale", 1.0)
        for name in set(self._items) - {f"{self.prefix}{os.path.basename(p)}" for p in groups}:
            self._items[name][2].SetVisibility(False)
            self._items[name][3] = np.zeros((0, 3))
        for path, rows in groups.items():
            name = f"{self.prefix}{os.path.basename(path)}"
            q = pts[rows]
            corners = billboard(self.plotter, q, px)
            e = self._items.get(name)
            if e is not None and e[0] == len(rows):
                e[1].points = corners
                e[1].Modified()
                e[3] = q
                continue
            n = len(rows)
            faces = np.hstack([np.full((n, 1), 4), (4 * np.arange(n))[:, None] + np.arange(4)]).ravel()
            mesh = pv.PolyData(corners, faces=faces)
            mesh.active_texture_coordinates = np.tile([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], (n, 1))
            actor = self.plotter.add_mesh(mesh, texture=symbol_texture(path), lighting=False, name=name,
                                          render=False)
            self._items[name] = [n, mesh, actor, q]

    def place(self) -> None:
        """After a camera or UI size change: every symbol resized and turned toward the camera."""
        px = SYMBOL_PX * getattr(self.plotter, "_ui_scale", 1.0)
        for e in self._items.values():
            if len(e[3]) and e[0] == len(e[3]):
                e[1].points = billboard(self.plotter, e[3], px)
                e[1].Modified()

    def set_visible(self, on: bool) -> None:
        for e in self._items.values():
            e[2].SetVisibility(bool(on) and len(e[3]) > 0)


def achieved_text(a: dict) -> str:
    """The 3D label's second line: the worst range held over the directions that see the sky, the range the ground
    leaves where it cuts the volume (terrain mask), the revisit held."""
    parts = []
    rl = a.get("relief")                               # (user, 11 November 2026: the part another volume serves;
    if rl and a.get("worst_km") is None and not a.get("ground_km"):     # who serves it: relief_text, a line of its own)
        return f"need met  RR {a['rr_s']:.3g} s"
    lim = {"reach": " (reach limit)", "power": " (power limit)"}.get(a.get("limit"), "")
    if a.get("worst_km") is not None:
        parts.append(f"worst {a['worst_km']:.0f} km" + lim)
        lim = ""
    g = a.get("ground_km")
    if g:
        parts.append(f"ground {g[0]:.0f}-{g[1]:.0f} km")
    parts.append(f"RR {a['rr_s']:.3g} s" + lim)
    return ", ".join(parts[:-1]) + "  " + parts[-1] if len(parts) > 1 else parts[0]


RELIEF_TOTAL_SHARE = 0.995          # a share served by others at least this: "totally" (display rounding only)


def relief_text(a: dict) -> Optional[str]:
    """The 3D label's line under the achieved one when other volumes meet part of the need (user, 12 November 2026):
    'handled totally by SP' or 'handled partially by SP (21 %)'; None when no other volume serves it."""
    rl = a.get("relief")
    if not rl or not rl.get("by"):
        return None
    by = ", ".join(rl["by"])
    share = float(rl.get("share", 0.0) or 0.0)
    if share >= RELIEF_TOTAL_SHARE or (a.get("worst_km") is None and not a.get("ground_km")):
        return f"handled totally by {by}"
    return f"handled partially by {by} ({100 * share:.0f} %)"


def label_extra_lines(a: Optional[dict]) -> list:
    """The 3D label's lines under the achieved one: [(text, ok)] -- who handles the need, the confirmation alert."""
    if not a:
        return []
    out = []
    t = relief_text(a)
    if t:
        out.append((t, True))
    if a.get("confirm") is False:
        out.append((no_confirmation_text(a), False))
    return out


def no_confirmation_text(a: dict) -> str:
    """The 3D label's alert line when the detections cannot be confirmed: the longest range where they can."""
    cm = a.get("confirm_max_km")
    return ("no confirmation (Ku)" + (f" beyond ~{cm:.0f} km" if cm else ": its detections cannot become tracks"))


class LineLabels:
    """Labels of several lines, each line its own colour (user, 11 November 2026: a volume's label, what is asked in
    grey over what is achieved in green or red): one vtkTextActor per line, placed on the screen under the previous one
    before every render from the label's 3D point (VTK's label mappers draw one colour per label, drop a blank first
    line and hide a label laid over another). Hidden when the point is behind the camera."""

    def __init__(self, plotter, size: int, box: bool = True):
        self.base_size, self.box = size, box
        self._plotter = weakref.ref(plotter)
        self._items: Dict[str, dict] = {}            # key -> {"pt", "lines": [(text, colour)], "actors", "visible"}
        self._selected: Optional[str] = None        # (5.5 E1: the selected volume's label framed)
        me = weakref.ref(self)
        plotter.renderer.AddObserver("StartEvent", lambda *_a: me() is not None and me()._place())

    def _size(self) -> int:
        p = self._plotter()
        return max(1, round(self.base_size * getattr(p, "_ui_scale", 1.0)))

    def put(self, key: str, point, lines: Sequence[Tuple[str, str]]) -> None:
        """The label `key` at the 3D point: lines [(text, colour)], top to bottom."""
        from vtkmodules.vtkRenderingCore import vtkTextActor
        p = self._plotter()
        if p is None:
            return
        it = self._items.get(key)
        if it is None:
            it = self._items[key] = {"actors": [], "visible": True}
        while len(it["actors"]) < len(lines):
            a = vtkTextActor()
            tp = a.GetTextProperty()
            tp.SetBold(True)
            tp.SetShadow(False)
            tp.SetJustificationToLeft()
            tp.SetVerticalJustificationToTop()
            if self.box:
                tp.SetBackgroundColor(0.0, 0.0, 0.0)
                tp.SetBackgroundOpacity(0.75)
            p.add_actor(a, reset_camera=False, render=False)
            it["actors"].append(a)
        while len(it["actors"]) > len(lines):
            p.remove_actor(it["actors"].pop(), render=False)
        for a, (text, colour) in zip(it["actors"], lines):
            a.SetInput(f" {text} ")
            a.GetTextProperty().SetColor(*(c / 255.0 for c in rgb(colour)))
            a.GetTextProperty().SetFontSize(self._size())
        it["pt"] = np.asarray(point, float).reshape(3)
        it["lines"] = list(lines)
        self._frame(key)

    def _frame(self, key: str) -> None:
        on = key == self._selected
        for a in (self._items.get(key) or {}).get("actors", []):
            tp = a.GetTextProperty()
            tp.SetFrame(1 if on else 0)
            if on:
                tp.SetFrameColor(1.0, 1.0, 1.0)
                tp.SetFrameWidth(2)

    def set_selected(self, key: Optional[str]) -> None:
        """The label of the selected volume framed in white (5.5 E1), the others not."""
        old, self._selected = self._selected, key
        for k in {old, key} - {None}:
            self._frame(k)

    def keep(self, keys) -> None:
        """Only these labels kept (the others removed)."""
        for k in [k for k in self._items if k not in set(keys)]:
            self.remove(k)

    def remove(self, key: str) -> None:
        it = self._items.pop(key, None)
        p = self._plotter()
        if it is not None and p is not None:
            for a in it["actors"]:
                p.remove_actor(a, render=False)

    def clear(self) -> None:
        for k in list(self._items):
            self.remove(k)

    def set_visible(self, key: str, on: bool) -> None:
        if key in self._items:
            self._items[key]["visible"] = bool(on)

    def set_all_visible(self, on: bool) -> None:
        for it in self._items.values():
            it["visible"] = bool(on)

    def _place(self) -> None:
        """Before a render: each line at its label's point on the screen, under the previous one."""
        p = self._plotter()
        if p is None or not self._items:
            return
        ren = p.renderer
        cam = ren.GetActiveCamera()
        pos, fwd = np.asarray(cam.GetPosition(), float), np.asarray(cam.GetDirectionOfProjection(), float)
        line_h = self._size() * 1.3
        for it in self._items.values():
            front = float((it["pt"] - pos) @ fwd) > 0.0
            show = it["visible"] and front
            if show:
                ren.SetWorldPoint(*it["pt"], 1.0)
                ren.WorldToDisplay()
                x, y, _z = ren.GetDisplayPoint()
            for i, a in enumerate(it["actors"]):
                a.SetVisibility(show)
                if show:
                    a.SetDisplayPosition(int(x), int(y - i * line_h))
