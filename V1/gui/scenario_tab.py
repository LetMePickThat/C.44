#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO_TAB -- the scenario editor of the global simulator (roadmap phase 6, pass S5; block F validated by the user on
28 October 2026): a tree of the scenario (time and body, ship, objects, groups), a form per element generated from a
field description, validation at every change (sim.scenario.validate, one line in the window's status bar), JSON load
and save (the window's menu), attitude waypoints edited by a list (W1-W3). On the right the 3D views and the
playback (gui/sim_view.py, passes S6 and S7).
"""
from __future__ import annotations

import copy
import json
import os
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QEvent, QSize, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QKeySequence
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QInputDialog, QFileDialog, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QListWidget,
                             QListWidgetItem, QMenu, QMessageBox, QDialog, QDialogButtonBox, QPushButton, QShortcut, QScrollArea, QSizePolicy, QSpinBox, QSplitter,
                             QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from core import radar_model as rm
from core.params import P
from gui import controls as uc
from gui.widgets import DecimalSpinBox, FlowLayout
from sim import attitude as att
from sim import scenario as scn

# field kinds: "text", "float", "int", "choice", "vecN" (N numbers), "json" (a dict or list typed as JSON)
Field = Tuple[str, str, str, dict]          # (key, label, kind, options)

_BIG = 1e12
COMPACT_TREE_H = 120                       # px: least height of the tree over the options (compact column)
TREE_MAX_SHARE = 0.5                       # the tree at most this share of the column
FORM_MIN_W = 280                           # px: the narrowest the forms start at
SPIN_MAX_W = 110                           # px: a number field's wish (a bound of 1e12 asked for 13 digits, 180 px)
FIELD_MIN_W = 48                           # px: the narrowest a field shrinks to with the column (it is never cut)
SCENARIO_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scenarios")


def _f(key: str, label: str, kind: str = "float", **opt) -> Field:
    return key, label, kind, opt


ORBIT_FIELDS: Dict[str, List[Field]] = {
    "elements": [_f("a_km", "semi-major axis (km)", min=1.0), _f("e", "eccentricity", min=0.0, max=0.999999, dec=6),
                 _f("i_deg", "inclination (deg)", min=0.0, max=180.0), _f("raan_deg", "RAAN (deg)", min=-360.0, max=360.0),
                 _f("argp_deg", "argument of perigee (deg)", min=-360.0, max=360.0),
                 _f("nu_deg", "true anomaly (deg)", min=-360.0, max=360.0)],
    "state": [_f("r_km", "position (km, ICRF)", "vec3"), _f("v_kms", "velocity (km/s, ICRF)", "vec3", dec=6)],
    "relative": [_f("range_km", "range from the ship (km)", min=0.001), _f("phi_deg", "phi (deg)", min=-360.0, max=360.0),
                 _f("eps_deg", "eps (deg)", min=-90.0, max=90.0),
                 _f("frame", "direction frame", "choice", choices=("ship", "inertial")),
                 _f("v_close_mps", "closing speed (m/s)", min=-_BIG),
                 _f("v_transverse_mps", "transverse velocity (m/s)", "vec2")],
}
# attitude by waypoints (pass S3a, sim/attitude.py), edited by WaypointList (W1-W3)
_ROLL = _f("roll_deg", "roll (deg)", min=-360.0, max=360.0)
ATT_DIRECTIONS = ("prograde", "retrograde", "normal+", "normal-", "radial_out", "radial_in")
ATT_DEFAULTS = {"fixed": {"type": "fixed", "frame": "orbital", "yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": 0.0},
                "target": {"type": "target", "id": "ship", "roll_deg": 0.0}}
ATT_DEFAULTS.update({t: {"type": t, "roll_deg": 0.0} for t in ATT_DIRECTIONS})
MANOEUVRE_FIELDS: Dict[str, List[Field]] = {
    "impulse": [_f("t_s", "time (s)", min=0.0), _f("dv_mps", "delta-v (m/s)", "vec3"),
                _f("frame", "frame", "choice", choices=("rtn", "inertial"))],
    "burn": [_f("t_s", "start (s)", min=0.0), _f("duration_s", "duration (s)", min=0.001),
             _f("accel_mps2", "acceleration (m/s2)", "vec3"), _f("frame", "frame", "choice", choices=("rtn", "inertial"))],
    "singer": [],
    "pn": [_f("n", "navigation constant", min=0.1, default=float(P.get("sim.pn_n"))), _f("start_range_km", "start below range (km, or empty: time)", "json"),
           _f("t_s", "start time (s, or empty: range)", "json"),
           _f("target", "target (ship, an object, <group>#k)", "text", default="ship")],
}
MANOEUVRE_DEFAULTS = {"impulse": {"type": "impulse", "t_s": 0.0, "dv_mps": [0.0, 0.0, 0.0], "frame": "rtn"},
                      "burn": {"type": "burn", "t_s": 0.0, "duration_s": 10.0, "accel_mps2": [0.0, 0.0, 0.0],
                               "frame": "rtn"},
                      "singer": {"type": "singer"},
                      "pn": {"type": "pn", "start_range_km": 100.0}}
# a new relative object 1000 km ahead along the nose (+Z; the default attitude is prograde: along the track); at
# phi = eps = 0 (+X = toward the central body in the orbital reference) it started below the surface (user, 29 Oct 2026)
ORBIT_DEFAULTS = {"elements": {"type": "elements", "a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 0.0,
                               "argp_deg": 0.0, "nu_deg": 0.0},
                  "state": {"type": "state", "r_km": [7000.0, 0.0, 0.0], "v_kms": [0.0, 7.546, 0.0]},
                  "relative": {"type": "relative", "range_km": 1000.0, "phi_deg": 0.0, "eps_deg": 90.0, "frame": "ship",
                               "v_close_mps": 5000.0, "v_transverse_mps": [0.0, 0.0]}}


def new_scenario() -> dict:
    """A minimal scenario to start from (an example, not a requirement): one ship on a low circular orbit, one KKV."""
    return {"meta": {"name": "new scenario", "epoch_utc": "2050-01-01T00:00:00Z", "duration_s": 600.0, "seed": 1},
            "central_body": "earth",
            "ship": {"orbit": copy.deepcopy(ORBIT_DEFAULTS["elements"]),
                     "attitude": copy.deepcopy(att.DEFAULT_SHIP)},
            "objects": [{"id": "KKV-1", "class": "KKV", "t_appear_s": 0.0,
                         "state": copy.deepcopy(ORBIT_DEFAULTS["relative"]), "manoeuvres": []}],
            "groups": []}


# ----------------------------------------------------------------------------- generic form
# (user, 29 October 2026) the fields always end at the column's edge, never cut by it: each one shrinks with the
# column down to FIELD_MIN_W and grows when the column is widened; the labels wrap, the button rows flow
class _Narrow:
    """Mixin of a field: its minimum width at FIELD_MIN_W at most (the column, not the content, sets its width)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        pol = self.sizePolicy()                    # a spin box's "Minimum" kept it at its hint: shrinkable now
        pol.setHorizontalPolicy(QSizePolicy.Preferred)
        self.setSizePolicy(pol)

    def minimumSizeHint(self) -> QSize:
        s = super().minimumSizeHint()
        return QSize(min(s.width(), FIELD_MIN_W), s.height())


class _Spin(_Narrow, DecimalSpinBox):
    def sizeHint(self) -> QSize:                   # the column's width at its true need (user, 29 October 2026)
        s = super().sizeHint()
        return QSize(min(s.width(), SPIN_MAX_W), s.height())


class _IntSpin(_Narrow, QSpinBox):
    pass


class _Combo(_Narrow, QComboBox):
    pass


class _WrapLabel(QLabel):
    """A label on one line when the column allows it, on several (at the words) when it is narrower."""

    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setWordWrap(True)

    def sizeHint(self) -> QSize:
        m = self.contentsMargins()
        fm = self.fontMetrics()
        return QSize(fm.horizontalAdvance(self.text()) + m.left() + m.right() + 2, fm.height() + m.top() + m.bottom())

    def minimumSizeHint(self) -> QSize:
        fm = self.fontMetrics()
        word = max((fm.horizontalAdvance(x) for x in self.text().split()), default=0)
        return QSize(word + 2, fm.height())


def _button_row(buttons: Sequence[QPushButton]) -> FlowLayout:
    """Buttons side by side, on more lines when the column is narrower than the row."""
    row = FlowLayout()
    for b in buttons:
        row.addWidget(b)
    return row


def _clear(layout) -> None:
    """Empty a layout: each widget detached at once (no stale children until the deferred delete), then deleted."""
    while layout.count():
        it = layout.takeAt(0)
        w = it.widget()
        if w is not None:
            w.setParent(None)
            w.deleteLater()


class VecEdit(QWidget):
    changed = pyqtSignal()

    def __init__(self, n: int, decimals: int = 3, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.boxes = []
        for _ in range(n):
            b = _Spin()
            b.setRange(-_BIG, _BIG)
            b.setDecimals(decimals)
            b.valueChanged.connect(lambda _v: self.changed.emit())
            lay.addWidget(b)
            self.boxes.append(b)

    def set_value(self, v) -> None:
        for b, x in zip(self.boxes, list(v or []) + [0.0] * len(self.boxes)):
            b.blockSignals(True)
            b.setValue(float(x))
            b.blockSignals(False)

    def value(self) -> List[float]:
        return [b.value() for b in self.boxes]


class FormWidget(QWidget):
    """A form built from a list of fields; edits a dict in place and emits `changed` after each edit."""
    changed = pyqtSignal()

    def __init__(self, fields: Sequence[Field], data: dict, parent=None):
        super().__init__(parent)
        self.data = data
        self.widgets: Dict[str, Tuple[str, QWidget]] = {}
        # a grid, not a QFormLayout: that one keeps each label on one line, whatever the column's width
        lay = QGridLayout(self)
        lay.setColumnStretch(1, 1)                 # the fields to the column's edge
        for row, (key, label, kind, opt) in enumerate(fields):
            w = self._make(key, kind, opt)
            self.widgets[key] = (kind, w)
            lay.addWidget(_WrapLabel(label), row, 0)
            lay.addWidget(w, row, 1)

    def _make(self, key: str, kind: str, opt: dict) -> QWidget:
        v = self.data.get(key, opt.get("default"))      # an absent key shows the value the model will use
        if kind == "float":
            w = _Spin()
            w.setRange(float(opt.get("min", -_BIG)), float(opt.get("max", _BIG)))
            w.setDecimals(int(opt.get("dec", 3)))
            w.setValue(float(v) if isinstance(v, (int, float)) else 0.0)
            w.valueChanged.connect(lambda x, k=key: self._set(k, float(x)))
        elif kind == "int":
            w = _IntSpin()
            w.setRange(int(opt.get("min", 0)), int(opt.get("max", 10 ** 9)))
            w.setValue(int(v) if isinstance(v, int) else int(opt.get("min", 0)))
            w.valueChanged.connect(lambda x, k=key: self._set(k, int(x)))
        elif kind == "choice":
            w = _Combo()
            choices = list(opt["choices"]() if callable(opt["choices"]) else opt["choices"])
            w.addItems(choices)
            if v in choices:
                w.setCurrentIndex(choices.index(v))
            elif choices and v is not None:
                # a value the list does not offer (e.g. the default target "ship" in the ship's own waypoints): the
                # choice shown is stored, so what is shown is what is validated (user, 29 October 2026)
                self.data[key] = choices[0]
            w.currentTextChanged.connect(lambda x, k=key: self._set(k, x if x else None))
        elif kind.startswith("vec"):
            w = VecEdit(int(kind[3:]), int(opt.get("dec", 3)))
            w.set_value(v)
            w.changed.connect(lambda k=key, ww=w: self._set(k, ww.value()))
        elif kind == "json":
            w = QLineEdit(json.dumps(v) if v is not None else "")
            w.editingFinished.connect(lambda k=key, ww=w: self._set_json(k, ww))
        else:
            w = QLineEdit("" if v is None else str(v))
            w.editingFinished.connect(lambda k=key, ww=w: self._set(k, ww.text().strip()))
        return w

    def _set(self, key: str, value) -> None:
        if value is None or value == "":
            self.data.pop(key, None)
        else:
            self.data[key] = value
        self.changed.emit()

    def _set_json(self, key: str, w: QLineEdit) -> None:
        txt = w.text().strip()
        if not txt:
            self._set(key, None)
            return
        try:
            self._set(key, json.loads(txt))
            w.setStyleSheet("")
        except json.JSONDecodeError:
            w.setStyleSheet("background: #ffd6d6")          # not JSON: kept on screen, not applied


def _typed_form(title: str, types: Sequence[str], fields: Dict[str, List[Field]], defaults: Dict[str, dict],
                holder: dict, key: str, type_key: str, on_change: Callable[[], None]) -> QWidget:
    """A type selector over a form that is rebuilt for the type chosen; holder[key] is replaced by the type's default
    when the type changes (the values of another type do not carry over)."""
    box = QWidget()
    lay = QVBoxLayout(box)
    lay.setContentsMargins(0, 0, 0, 0)
    row = QHBoxLayout()
    row.addWidget(QLabel(title))
    combo = _Combo()
    combo.addItems(list(types))
    cur = holder.get(key, {}).get(type_key)
    if cur in types:
        combo.setCurrentIndex(list(types).index(cur))
    row.addWidget(combo, 1)
    lay.addLayout(row)
    area = QVBoxLayout()
    lay.addLayout(area)

    def build():
        _clear(area)
        form = FormWidget(fields[combo.currentText()], holder[key])
        form.changed.connect(on_change)
        area.addWidget(form)

    def retype(t: str):
        if holder.get(key, {}).get(type_key) != t:
            holder[key] = copy.deepcopy(defaults[t])
            build()                                  # first: the form may settle a default (a choice offered)
            on_change()
            return
        build()

    if holder.get(key, {}).get(type_key) not in types:
        holder[key] = copy.deepcopy(defaults[combo.currentText()])
    build()
    combo.currentTextChanged.connect(retype)
    return box


class ManoeuvreList(QWidget):
    """The manoeuvres of an object or a group: a list, add buttons per type, the form of the one selected."""

    def __init__(self, items: list, on_change: Callable[[], None], parent=None):
        super().__init__(parent)
        self.items, self.on_change = items, on_change
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.list = QListWidget()
        lay.addWidget(self.list)
        buttons = []
        for t in MANOEUVRE_FIELDS:
            b = QPushButton(f"+ {t}")
            b.clicked.connect(lambda _c, tt=t: self._add(tt))
            buttons.append(b)
        rm_b = QPushButton("Remove")
        rm_b.clicked.connect(self._remove)
        lay.addLayout(_button_row(buttons + [rm_b]))
        self.form_area = QVBoxLayout()
        lay.addLayout(self.form_area)
        self.list.currentRowChanged.connect(self._show)
        self._refresh()

    @staticmethod
    def _label(m: dict) -> str:
        t = m.get("type")
        if t == "impulse":
            return f"impulse at {m.get('t_s', 0):g} s, dv {m.get('dv_mps')} m/s ({m.get('frame', 'rtn')})"
        if t == "burn":
            return f"burn at {m.get('t_s', 0):g} s for {m.get('duration_s', 0):g} s, {m.get('accel_mps2')} m/s2"
        if t == "pn":
            start = (f"below {m['start_range_km']:g} km" if "start_range_km" in m else f"at {m.get('t_s', 0):g} s")
            return f"terminal guidance (PN) {start} toward {m.get('target', 'ship')}"
        return "random manoeuvre (Singer, class)"

    def _refresh(self, select: Optional[int] = None) -> None:
        self.list.blockSignals(True)
        self.list.clear()
        for m in self.items:
            self.list.addItem(self._label(m))
        self.list.blockSignals(False)
        if self.items:
            self.list.setCurrentRow(len(self.items) - 1 if select is None else select)
        else:
            self._show(-1)

    def _add(self, t: str) -> None:
        self.items.append(copy.deepcopy(MANOEUVRE_DEFAULTS[t]))
        self._refresh()
        self.on_change()

    def _remove(self) -> None:
        i = self.list.currentRow()
        if 0 <= i < len(self.items):
            del self.items[i]
            self._refresh()
            self.on_change()

    def _show(self, i: int) -> None:
        _clear(self.form_area)
        if not (0 <= i < len(self.items)):
            return
        m = self.items[i]
        form = FormWidget(MANOEUVRE_FIELDS[m["type"]], m)

        def changed():
            self.list.item(i).setText(self._label(m))
            self.on_change()
        form.changed.connect(changed)
        self.form_area.addWidget(form)


class WaypointList(QWidget):
    """Attitude waypoints of the ship or of a SHIP / KKV object (W1): a list kept sorted by time, add / remove, the
    form of the point selected (time, slew rate except on the first point, attitude by type)."""

    def __init__(self, items: list, on_change: Callable[[], None], targets: Callable[[], List[str]],
                 duration: Callable[[], float], parent=None):
        super().__init__(parent)
        self.items, self.on_change, self.targets, self.duration = items, on_change, targets, duration
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.list = QListWidget()
        self.list.setMaximumHeight(120)
        lay.addWidget(self.list)
        buttons = []
        for text, slot in (("+ point", self._add), ("Remove", self._remove)):
            b = QPushButton(text)
            b.clicked.connect(slot)
            buttons.append(b)
        lay.addLayout(_button_row(buttons))
        self.form_area = QVBoxLayout()
        lay.addLayout(self.form_area)
        self.list.currentRowChanged.connect(self._show)
        self._refresh()

    @staticmethod
    def label(w: dict, first: bool) -> str:
        a = w.get("attitude", {})
        t = a.get("type", "?")
        what = (f"fixed {a.get('frame', 'orbital')} ({a.get('yaw_deg', 0):g}, {a.get('pitch_deg', 0):g}, "
                f"{a.get('roll_deg', 0):g})" if t == "fixed" else
                f"toward {a.get('id')}" if t == "target" else t)
        rate = "" if first else f", {w.get('rate_dps', 0):g} deg/s"
        return f"t {w.get('t_s', 0):g} s: {what}{rate}"

    def _refresh(self, select: Optional[int] = None) -> None:
        self.list.blockSignals(True)
        self.list.clear()
        for i, w in enumerate(self.items):
            self.list.addItem(self.label(w, i == 0))
        self.list.blockSignals(False)
        if self.items:
            self.list.setCurrentRow(len(self.items) - 1 if select is None else select)
        else:
            self._show(-1)

    def _add(self) -> None:
        t = min(float(self.items[-1].get("t_s", 0.0)) + 10.0, self.duration()) if self.items else 0.0
        w = {"t_s": t, "attitude": copy.deepcopy(ATT_DEFAULTS["prograde"])}
        if self.items:
            w["rate_dps"] = min(5.0, att.RATE_MAX_DPS)
        self.items.append(w)
        self._refresh()
        self.on_change()

    def _remove(self) -> None:
        i = self.list.currentRow()
        if 0 <= i < len(self.items):
            del self.items[i]
            if self.items:
                self.items[0].pop("rate_dps", None)
                self.items[0]["t_s"] = 0.0
            self._refresh()
            self.on_change()

    def _sort(self, w: dict) -> None:
        """Keep the points sorted by time; the first carries no slew rate, the others need one."""
        order = sorted(self.items, key=lambda x: float(x.get("t_s", 0.0)))
        if any(a is not b for a, b in zip(order, self.items)):
            self.items[:] = order
            for i, x in enumerate(self.items):
                if i == 0:
                    x.pop("rate_dps", None)
                else:
                    x.setdefault("rate_dps", min(5.0, att.RATE_MAX_DPS))
            self._refresh(select=next(i for i, x in enumerate(self.items) if x is w))

    def _show(self, i: int) -> None:
        _clear(self.form_area)
        if not (0 <= i < len(self.items)):
            return
        w = self.items[i]
        before = copy.deepcopy(w)
        fields = [_f("t_s", "time (s, start of the slew)", min=0.0)]
        if i > 0:
            fields.append(_f("rate_dps", "slew rate (deg/s)", min=0.001, max=att.RATE_MAX_DPS))
        form = FormWidget(fields, w)
        att_fields: Dict[str, List[Field]] = {
            "fixed": [_f("frame", "frame", "choice", choices=att.FIXED_FRAMES),
                      _f("yaw_deg", "yaw (deg)", min=-360.0, max=360.0),
                      _f("pitch_deg", "pitch (deg)", min=-90.0, max=90.0), _ROLL],
            "target": [_f("id", "target", "choice", choices=self.targets), _ROLL]}
        att_fields.update({t: [_ROLL] for t in ATT_DIRECTIONS})

        def changed():
            k = next(j for j, x in enumerate(self.items) if x is w)
            item = self.list.item(k)
            if item is not None:
                item.setText(self.label(w, k == 0))
            self.on_change()
            self._sort(w)
        form.changed.connect(changed)
        self.form_area.addWidget(form)
        self.form_area.addWidget(_typed_form("attitude", att.TYPES, att_fields, ATT_DEFAULTS, w, "attitude", "type",
                                             changed))
        if w != before:                              # the form settled a value not offered (a target): validated
            changed()


# ----------------------------------------------------------------------------- the tab
class ScenarioTab(QWidget):
    """Scenario editor: tree, forms, validation messages, JSON files (the separate scenario tool)."""
    scenario_changed = pyqtSignal(dict)           # the scenario on screen, after each valid change (for S6 / S7)
    title_changed = pyqtSignal(str)               # the file name (with " *" when modified)

    def __init__(self, grid=None, view: bool = True, parent=None):
        super().__init__(parent)
        self.sc: dict = new_scenario()
        self.path: Optional[str] = None
        self.modified = False
        self._mission_cache: Optional[tuple] = None   # (path, mtime, mission) of the mission file drawn
        # R1-R6, B14 (user, 29 October 2026): one column (the form of the element chosen over the tree), the 3D
        # view for the rest; "+" over the tree adds an object or a group, right-click and the Del key on an element
        # duplicate or delete it; the files in the window's menu (file_commands), the file name and the validation
        # in its status bar (status_widgets)
        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.currentItemChanged.connect(lambda cur, _prev: self._show(cur))
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._element_menu)
        delete = QShortcut(QKeySequence.Delete, self.tree, activated=self.delete)
        delete.setContext(Qt.WidgetShortcut)
        add = QMenu(self)
        add.addAction("Object", self.add_object)
        add.addAction("Group", self.add_group)
        tree_box = QWidget()
        tl = QVBoxLayout(tree_box)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.setSpacing(0)
        head = QHBoxLayout()
        head.addWidget(QLabel("Elements"))
        head.addStretch(1)
        head.addWidget(uc.menu_button("+", add, tip="Add an object or a group", arrow=False))
        tl.addLayout(head)
        tl.addWidget(self.tree, 1)
        self.file_label = QLabel()
        self.status = QPushButton("")                          # R4: the validation in one line
        self.status.setFlat(True)
        self.status.clicked.connect(self._show_problems)
        self.messages = QListWidget()                          # the validation's messages (status: tooltip, click)

        self.form_holder = QWidget()
        self.form_layout = QVBoxLayout(self.form_holder)
        self.form_layout.setContentsMargins(4, 0, 4, 0)

        if view:                                   # passes S6 / S7: 3D views and playback
            from gui.sim_view import SimView
            self.preview = SimView(grid, get_path=lambda: None if self.modified else self.path)
            self.scenario_changed.connect(self.preview.set_scenario)
        else:
            self.preview = QLabel("3D preview and playback (not built)")
            self.preview.setAlignment(Qt.AlignCenter)

        form_scroll = QScrollArea()
        form_scroll.setWidgetResizable(True)
        form_scroll.setFrameShape(QScrollArea.NoFrame)
        form_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)   # the fields narrow with the column
        form_scroll.setWidget(self.form_holder)
        # (user, 29 October 2026, replaces B6) the tree on top, the options of the element chosen in it under it:
        # the tree at the height of its rows, the options take the rest
        split_col = QSplitter(Qt.Vertical)
        split_col.addWidget(tree_box)
        split_col.addWidget(form_scroll)
        split_col.setStretchFactor(0, 0)
        split_col.setStretchFactor(1, 1)
        split_col.setSizes([COMPACT_TREE_H, 800])
        self._col_split = split_col
        split_col.installEventFilter(self)                     # B9: the fit again when the column changes height
        self.tree.itemExpanded.connect(lambda _it: self._fit_form())
        self.tree.itemCollapsed.connect(lambda _it: self._fit_form())
        self.left_col = split_col
        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.left_col)
        # (user, 29 October 2026) the information in a column right of the options column
        self.side_col = getattr(self.preview, "side_panel", None)
        if self.side_col is not None:
            split.addWidget(self.side_col)
        split.addWidget(self.preview)
        for i in range(split.count()):
            split.setStretchFactor(i, 1 if split.widget(i) is self.preview else 0)
        self._split = split
        QTimer.singleShot(0, self._fit_columns)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(split, 1)
        self.loaded = False                       # P1: the 3D view alone until a file is created or opened
        self._rebuild_tree()
        self._validate()
        self._set_loaded(False)

    def _set_loaded(self, on: bool) -> None:
        """P1 (user, 29 October 2026): before a file is created or opened, the 3D view alone (no editor, no
        controls, no playback, no status); after, everything."""
        self.loaded = on
        self.left_col.setVisible(on)
        for w in (self.file_label, self.status):
            w.setVisible(on)
        if hasattr(self.preview, "set_controls_visible"):
            self.preview.set_controls_visible(on)

    def _element_menu(self, pos) -> None:
        """Right-click on an object or a group: duplicate or delete it."""
        it = self.tree.itemAt(pos)
        if it is None or it.data(0, Qt.UserRole)[1] < 0:
            return
        self.tree.setCurrentItem(it)
        menu = QMenu(self)
        menu.addAction("Duplicate", self.duplicate)
        menu.addAction("Delete", self.delete)
        menu.exec_(self.tree.viewport().mapToGlobal(pos))

    def primary_action(self) -> Optional[QWidget]:
        """The mode's main action for the window's header: "Propagate", shown only for a scenario too large for
        the automatic propagation."""
        return getattr(self.preview, "bt_prop", None)

    def file_commands(self) -> list:
        """(text, slot, icon) of the scenario in the window's menu (before a file is created or opened, only New
        and Open are enabled: needs_file)."""
        own = [("New...", self.new, "SP_FileIcon"), ("Open...", self.open, "SP_DialogOpenButton"),
               ("Save", self.save, "SP_DialogSaveButton"), ("Save as...", self.save_as, "SP_DialogSaveButton"),
               ("Scenario properties...", lambda: self.edit_properties(), "SP_FileDialogDetailedView"),
               ("Export mission at t...", self.export_mission, "SP_ArrowRight")]
        return own + (self.preview.file_commands() if hasattr(self.preview, "file_commands") else [])

    def status_widgets(self) -> list:
        """The file name and the validation, then the propagation's state, for the window's status bar."""
        own = [self.file_label, self.status]
        return own + (self.preview.status_widgets() if hasattr(self.preview, "status_widgets") else [])

    def _show_problems(self) -> None:
        err = [self.messages.item(i).text() for i in range(self.messages.count())]
        if err and err != ["Scenario valid"]:
            QMessageBox.information(self, "Scenario", f"{len(err)} problem(s):\n\n" + "\n".join(err))

    def _fit_columns(self) -> None:
        """At start (and when a file is opened): the column at the width the forms need, the Info column at the
        width of its widest rows, the rest of the window to the 3D view (user, 29 October 2026)."""
        w0 = max(self.left_col.sizeHint().width(), self.form_holder.minimumSizeHint().width() + 24, FORM_MIN_W)
        if self.side_col is None:
            total = max(self._split.width(), w0 + 400)
            self._split.setSizes([w0, total - w0])
        else:
            side = self.preview.side_width()              # fits the longest Info line (no wrapping)
            total = max(self._split.width(), w0 + side + 400)
            self._split.setSizes([w0, side, total - w0 - side])
        self._fit_form()

    # ------------------------------------------------------------------ files
    def _confirm_discard(self) -> bool:
        if not self.modified:
            return True
        return QMessageBox.question(self, "Scenario", "Discard the unsaved changes?",
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes

    def new(self) -> None:
        """P1: a new file, its properties asked first (Cancel: no file created)."""
        if not self._confirm_discard():
            return
        sc = new_scenario()
        if self.edit_properties(sc, "New scenario"):
            self._set_scenario(sc, None)

    def open(self) -> None:
        if not self._confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(self, "Open a scenario", self._dir(), "Scenario (*.json)")
        if path:
            self.open_path(path)

    def open_path(self, path: str) -> None:
        try:
            with open(path, encoding="utf-8") as fh:
                sc = json.load(fh)
        except (OSError, json.JSONDecodeError) as exc:
            QMessageBox.critical(self, "Scenario", f"{path}: {exc}")
            return
        if not isinstance(sc, dict):
            QMessageBox.critical(self, "Scenario", f"{path}: not a scenario (a JSON object is needed)")
            return
        sc.setdefault("objects", [])
        sc.setdefault("groups", [])
        self._set_scenario(sc, path)          # loaded as written: the problems show in the validation list

    def save(self) -> None:
        if self.path is None:
            self.save_as()
        else:
            self._write(self.path)

    def save_as(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save the scenario", self._dir(), "Scenario (*.json)")
        if path:
            self._write(path if path.endswith(".json") else path + ".json")

    def _write(self, path: str) -> None:
        err = scn.validate(self.sc)
        if err and QMessageBox.question(self, "Scenario", f"The scenario has {len(err)} problem(s). Save anyway?",
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return
        try:
            with open(path, "w", encoding="utf-8", newline="\n") as fh:
                json.dump(self.sc, fh, indent=1, ensure_ascii=False)
        except OSError as exc:
            QMessageBox.critical(self, "Scenario", f"{path}: {exc}")
            return
        self.path, self.modified = path, False
        self._update_title()

    def export_mission(self) -> None:
        """Pass S4 (N9): the scenario's mission with the navigation and the sky at an instant t, for the mission
        preparation (the scenario is propagated up to t)."""
        if self.path is None or self.modified:
            QMessageBox.information(self, "Scenario", "Save the scenario first.")
            return
        try:
            sc = scn.load(self.path)
        except scn.ScenarioError as exc:
            QMessageBox.critical(self, "Scenario", str(exc))
            return
        if not sc.get("mission"):
            QMessageBox.information(self, "Scenario", "The scenario names no mission file.")
            return
        dur = float(sc["meta"]["duration_s"])
        t, ok = QInputDialog.getDouble(self, "Export mission at t", "t (s from the epoch):", 0.0, 0.0, dur, 3)
        if not ok:
            return
        out, _ = QFileDialog.getSaveFileName(self, "Save the mission", self._dir(), "Mission (*.json)")
        if not out:
            return
        from core import mission as cm
        from sim import engine, propagate
        try:
            short = copy.deepcopy(sc)
            short["meta"]["duration_s"] = max(t, float(sc["meta"]["step_s"]))
            QApplication.setOverrideCursor(Qt.WaitCursor)
            try:
                m, alerts = engine.mission_at(sc, propagate.propagate(short), t)
            finally:
                QApplication.restoreOverrideCursor()
            cm.save_mission_json(out if out.endswith(".json") else out + ".json", m)
        except (scn.ScenarioError, OSError, ValueError) as exc:
            QMessageBox.critical(self, "Scenario", str(exc))
            return
        QMessageBox.information(self, "Scenario", "Mission written." + ("\n\n" + "\n".join(alerts) if alerts else ""))

    def _dir(self) -> str:
        """The file dialogs' folder: the current file's, else the tool's scenarios folder (user, 29 October 2026)."""
        return os.path.dirname(self.path) if self.path else SCENARIO_DIR

    def _set_scenario(self, sc: dict, path: Optional[str]) -> None:
        self.sc, self.path, self.modified = sc, path, False
        self._set_loaded(True)
        self._rebuild_tree()
        self._validate()
        self._update_title()

    def _update_title(self) -> None:
        name = os.path.basename(self.path) if self.path else "(not saved)"
        self.file_label.setText(name + (" *" if self.modified else ""))
        self.title_changed.emit(self.file_label.text())

    # ------------------------------------------------------------------ tree
    def _rebuild_tree(self, select: Optional[Tuple[str, int]] = None) -> None:
        self.tree.blockSignals(True)
        self.tree.clear()
        self.n_ship = QTreeWidgetItem([scn.ship_name(self.sc)])      # the ship by its name (user: OWNSHIP)
        self.n_ship.setData(0, Qt.UserRole, ("ship", -1))
        self.n_obj = QTreeWidgetItem(["Objects"])
        self.n_obj.setData(0, Qt.UserRole, ("objects", -1))
        self.n_grp = QTreeWidgetItem(["Groups"])
        self.n_grp.setData(0, Qt.UserRole, ("groups", -1))
        self.tree.addTopLevelItems([self.n_ship, self.n_obj, self.n_grp])     # (the properties: a dialog, P1)
        found = None
        for kind, parent in (("objects", self.n_obj), ("groups", self.n_grp)):
            for i, o in enumerate(self.sc.get(kind, [])):
                it = QTreeWidgetItem([self._item_label(kind, o)])
                it.setData(0, Qt.UserRole, (kind, i))
                parent.addChild(it)
                if select == (kind, i):
                    found = it
        self.tree.expandAll()
        self.tree.blockSignals(False)
        self.tree.setCurrentItem(found or self.n_ship)

    @staticmethod
    def _item_label(kind: str, o: dict) -> str:
        if kind == "groups":
            return f"{o.get('id', '?')} ({o.get('n', '?')} x {o.get('class', '?')} around {o.get('around', '?')})"
        return f"{o.get('id', '?')} ({o.get('class', '?')})"

    def _selected(self) -> Tuple[str, int]:
        it = self.tree.currentItem()
        return it.data(0, Qt.UserRole) if it is not None else ("ship", -1)

    def _unique_id(self, base: str) -> str:
        used = set(scn.object_ids(self.sc))
        i = 1
        while f"{base}-{i}" in used:
            i += 1
        return f"{base}-{i}"

    def add_object(self) -> None:
        self.sc.setdefault("objects", []).append(
            {"id": self._unique_id("OBJ"), "class": "GENERIC", "t_appear_s": 0.0,
             "state": copy.deepcopy(ORBIT_DEFAULTS["relative"]), "manoeuvres": []})
        self._changed(rebuild=("objects", len(self.sc["objects"]) - 1))

    def add_group(self) -> None:
        objs = self.sc.get("objects", [])
        if not objs:
            QMessageBox.information(self, "Scenario", "A group is drawn around an object: add an object first.")
            return
        self.sc.setdefault("groups", []).append(
            {"id": self._unique_id("GRP"), "class": "DECOY", "n": 10, "around": objs[0]["id"], "pos_sigma_m": 1000.0,
             "vel_sigma_mps": 10.0, "t_appear_s": 0.0, "manoeuvres": []})
        self._changed(rebuild=("groups", len(self.sc["groups"]) - 1))

    def duplicate(self) -> None:
        kind, i = self._selected()
        if kind in ("objects", "groups") and i >= 0:
            o = copy.deepcopy(self.sc[kind][i])
            o["id"] = self._unique_id(o.get("id", "OBJ").split("-")[0])
            self.sc[kind].append(o)
            self._changed(rebuild=(kind, len(self.sc[kind]) - 1))

    def delete(self) -> None:
        kind, i = self._selected()
        if kind in ("objects", "groups") and i >= 0:
            del self.sc[kind][i]
            self._changed(rebuild=(kind, max(0, i - 1)) if self.sc[kind] else ("ship", -1))

    # ------------------------------------------------------------------ forms
    def _show(self, item: Optional[QTreeWidgetItem]) -> None:
        _clear(self.form_layout)
        if item is None:
            return
        kind, i = item.data(0, Qt.UserRole)
        if kind == "ship":
            w = self._ship_form()
        elif kind in ("objects", "groups") and i >= 0:
            w = self._object_form(kind, i)
        else:
            w = QLabel("Select an element in the tree above, or add one with '+'.")
        self.form_layout.addWidget(w)
        self.form_layout.addStretch(1)
        QTimer.singleShot(0, self._fit_form)
        self._select_in_view(kind, i)

    def _select_in_view(self, kind: str, i: int) -> None:
        """The unit chosen in the tree selected in the 3D view too, its Info shown (user, 29 October 2026): the ship,
        an object, a group by its first member; a heading of the tree changes nothing."""
        if not hasattr(self.preview, "select"):
            return
        from gui.sim_view import OWNSHIP
        if kind == "ship":
            self.preview.select(OWNSHIP)
        elif kind in ("objects", "groups") and 0 <= i < len(self.sc.get(kind, [])):
            oid = self.sc[kind][i].get("id")
            self.preview.select(f"{oid}#0" if kind == "groups" else oid)

    def eventFilter(self, obj, ev) -> bool:
        if obj is getattr(self, "_col_split", None) and ev.type() == QEvent.Resize:
            QTimer.singleShot(0, self._fit_form)
        return super().eventFilter(obj, ev)

    def _tree_need(self) -> int:
        """px: the height the tree shows all its rows in, with its header (at least COMPACT_TREE_H)."""
        t = self.tree
        rows, stack = 0, [t.topLevelItem(i) for i in range(t.topLevelItemCount())]
        while stack:
            it = stack.pop()
            rows += 1
            if it.isExpanded():
                stack.extend(it.child(j) for j in range(it.childCount()))
        row_h = t.sizeHintForRow(0) if t.topLevelItemCount() else 0
        row_h = row_h if row_h > 0 else t.fontMetrics().height() + 4
        buttons = self._col_split.widget(0).sizeHint().height() - t.sizeHint().height()
        return max(rows * row_h + 2 * t.frameWidth() + 4 + max(buttons, 0), COMPACT_TREE_H)

    def _fit_form(self) -> None:
        """B9 (user, 29 October 2026): the tree on top at the height of its rows (at most TREE_MAX_SHARE of the
        column, it scrolls beyond), the options of the element chosen under it take the rest (they scroll when
        taller)."""
        split = getattr(self, "_col_split", None)
        if split is None:
            return
        total = sum(split.sizes())
        if total <= 0:
            return
        h = min(self._tree_need(), int(TREE_MAX_SHARE * total))
        split.setSizes([h, total - h])

    def edit_properties(self, sc: Optional[dict] = None, title: str = "Scenario properties") -> bool:
        """P1 (user, 29 October 2026): the properties valid for the whole file (name, epoch, duration, truth step,
        seed, navigation errors and log, sky types, central body, mission file) in a dialog, opened when a file is
        created and from the menu; OK applies them (to sc, the scenario shown by default), Cancel leaves them."""
        target = self.sc if sc is None else sc
        work = {"meta": copy.deepcopy(target.get("meta", {})), "central_body": target.get("central_body"),
                "mission": target.get("mission")}
        work = {k: v for k, v in work.items() if v is not None}
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        lay = QVBoxLayout(dlg)
        lay.addWidget(self._meta_form(work))
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        lay.addWidget(buttons)
        if dlg.exec_() != QDialog.Accepted:
            return False
        target["meta"] = work["meta"]
        for key in ("central_body", "mission"):
            if work.get(key) not in (None, ""):
                target[key] = work[key]
            else:
                target.pop(key, None)
        if sc is None:
            self._changed()
            self._show(self.tree.currentItem())         # a form showing the duration (waypoints) follows it
        return True

    def _meta_form(self, holder: dict) -> QWidget:
        box = QWidget()
        lay = QVBoxLayout(box)
        meta = holder.setdefault("meta", {})
        f1 = FormWidget([_f("name", "name", "text"), _f("epoch_utc", "epoch (UTC, ISO 8601)", "text"),
                         _f("duration_s", "duration (s)", min=0.001), _f("step_s", "truth step (s, empty: default)",
                                                                           min=0.0, dec=4,
                                                                           default=float(P.get("sim.step_s"))),
                         _f("seed", "seed", "int", default=1),
                         _f("nav_errors", "navigation errors (true / false)", "json"),
                         _f("log_nav", "store the 100 Hz navigation (true / false)", "json"),
                         _f("sun_type", "Sun type (sky)", "choice", choices=lambda: scn.sky_types("sun")),
                         _f("earth_type", "Earth type (sky)", "choice", choices=lambda: scn.sky_types("earth"))], meta)
        lay.addWidget(f1)
        f2 = FormWidget([_f("central_body", "central body", "choice",
                            choices=lambda: list(scn.bodies()) or ["earth", "moon"]),
                         _f("mission", "mission file (relative to the scenario)", "text")], holder)
        lay.addWidget(f2)
        return box

    def _ship_form(self) -> QWidget:
        box = QWidget()
        lay = QVBoxLayout(box)
        ship = self.sc.setdefault("ship", {})
        name = FormWidget([_f("name", "name", "text", default=scn.DEFAULT_SHIP_NAME)], ship)   # renamable (user)

        def renamed():
            self.n_ship.setText(0, scn.ship_name(self.sc))
            self._changed()
        name.changed.connect(renamed)
        lay.addWidget(name)
        lay.addWidget(_typed_form("orbit", ("elements", "state"), ORBIT_FIELDS, ORBIT_DEFAULTS, ship, "orbit", "type",
                                  self._changed))
        lay.addWidget(QLabel("Attitude (waypoints)"))
        lay.addWidget(WaypointList(ship.setdefault("attitude", copy.deepcopy(att.DEFAULT_SHIP)), self._changed,
                                   lambda: list(scn.object_ids(self.sc)), self._duration))
        return box

    def _duration(self) -> float:
        d = self.sc.get("meta", {}).get("duration_s")
        return float(d) if isinstance(d, (int, float)) and d > 0 else 1e9

    def _waypoints_box(self, o: dict) -> QWidget:
        """The waypoints of a SHIP / KKV object: none = the class's default attitude (sim/geometry.py G5)."""
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        text = "flies attitude waypoints (else the class's default attitude)"
        cb = QCheckBox()                             # the text in a label beside it, that wraps (and toggles it)
        cb.setAccessibleName(text)
        cb.setChecked("attitude" in o)
        lab = _WrapLabel(text)
        lab.mousePressEvent = lambda _ev: cb.toggle()
        row = QHBoxLayout()
        row.addWidget(cb, 0, Qt.AlignTop)
        row.addWidget(lab, 1)
        lay.addLayout(row)
        area = QVBoxLayout()
        lay.addLayout(area)

        def build():
            _clear(area)
            if "attitude" in o:
                area.addWidget(WaypointList(o["attitude"], self._changed,
                                            lambda: ["ship"] + [x for x in scn.object_ids(self.sc) if x != o.get("id")],
                                            self._duration))

        def toggle(on: bool):
            if on and "attitude" not in o:
                o["attitude"] = [{"t_s": 0.0, "attitude": copy.deepcopy(ATT_DEFAULTS["target"])}]
            elif not on:
                o.pop("attitude", None)
            build()
            self._changed()
        cb.toggled.connect(toggle)
        build()
        return box

    def _object_form(self, kind: str, i: int) -> QWidget:
        o = self.sc[kind][i]
        box = QWidget()
        lay = QVBoxLayout(box)
        classes = sorted(rm.TARGETS)
        # terms (user, 29 October 2026): "characterisation" for the class, "classification" for the identity
        fields = [_f("id", "id", "text"), _f("class", "characterisation", "choice", choices=classes),
                  _f("identity", "classification", "choice", choices=scn.IDENTITIES, default=scn.DEFAULT_IDENTITY),
                  _f("t_appear_s", "appears at (s)", min=0.0)]
        if kind == "objects":
            fields.append(_f("jammer", "jammer", "choice", choices=[""] + sorted(scn.jammer_classes())))
        else:
            fields += [_f("n", "count", "int", min=1),
                       _f("around", "around", "choice", choices=["ship"] + [x["id"] for x in self.sc.get("objects", [])]),
                       _f("pos_sigma_m", "position spread (m, 1 sigma)", min=0.0),
                       _f("vel_sigma_mps", "velocity spread (m/s, 1 sigma)", min=0.0)]
        fields.append(_f("overrides", "class overrides (JSON)", "json"))
        form = FormWidget(fields, o)
        cls0 = o.get("class")

        def changed():
            if o.get("class") != cls0 and o.get("class") not in att.FLYING_CLASSES:
                o.pop("attitude", None)
            self._changed(relabel=(kind, i))
            if o.get("class") != cls0:
                self._show(self.tree.currentItem())      # the waypoints box follows the class
        form.changed.connect(changed)
        lay.addWidget(form)
        if kind == "objects" and o.get("class") in att.FLYING_CLASSES:
            lay.addWidget(self._waypoints_box(o))
        if kind == "objects":
            lay.addWidget(_typed_form("initial state", ("relative", "elements", "state"), ORBIT_FIELDS, ORBIT_DEFAULTS,
                                      o, "state", "type", self._changed))
        lay.addWidget(QLabel("Manoeuvres"))
        lay.addWidget(ManoeuvreList(o.setdefault("manoeuvres", []), self._changed))
        return box

    # ------------------------------------------------------------------ changes and validation
    def _changed(self, rebuild: Optional[Tuple[str, int]] = None, relabel: Optional[Tuple[str, int]] = None) -> None:
        self.modified = True
        self._update_title()
        if rebuild is not None:
            self._rebuild_tree(select=rebuild)
        elif relabel is not None:
            kind, i = relabel
            parent = self.n_obj if kind == "objects" else self.n_grp
            if i < parent.childCount():
                parent.child(i).setText(0, self._item_label(kind, self.sc[kind][i]))
        self._validate()

    def _push_mission(self) -> None:
        """The mission file the scenario names (relative to the scenario), its volumes drawn in the ship view; read
        again when it changes on disk."""
        if not hasattr(self.preview, "set_mission"):
            return
        ref = self.sc.get("mission")
        if not ref:
            self._mission_cache = None
            self.preview.set_mission(None)
            return
        path = os.path.normpath(os.path.join(self._dir(), ref))
        try:
            mtime = os.path.getmtime(path)
            if self._mission_cache is None or self._mission_cache[:2] != (path, mtime):
                from core import mission as cm
                self._mission_cache = (path, mtime, cm.load_mission(path))
        except (OSError, ValueError) as exc:
            self._mission_cache = None
            self.preview.set_mission(None)
            self.status.setToolTip(f"mission file: {exc}")
            return
        self.preview.set_mission(self._mission_cache[2])

    def _validate(self) -> None:
        err = scn.validate(self.sc)
        self.messages.clear()
        st = self.status                                       # R4: the validation in one line
        st.setText("Scenario valid" if not err else f"{len(err)} problem(s)")     # no symbols in the UI (user)
        st.setStyleSheet(f"color: {'darkgreen' if not err else 'darkred'}; font-weight: bold;")
        st.setToolTip("\n".join(err) if err else "")
        if not err:
            it = QListWidgetItem("Scenario valid")
            it.setForeground(QColor("darkgreen"))
            self.messages.addItem(it)
            if self.loaded:
                self.scenario_changed.emit(scn.normalise(self.sc))
                self._push_mission()
            return
        for e in err:
            it = QListWidgetItem(e)
            it.setForeground(QColor("darkred"))
            self.messages.addItem(it)
