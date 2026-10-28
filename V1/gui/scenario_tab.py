#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO_TAB -- the scenario editor of the global simulator (roadmap phase 6, pass S5; block F validated by the user on
28 October 2026): a tree of the scenario (time and body, ship, objects, groups), a form per element generated from a
field description, validation at every change (sim.scenario.validate, messages listed under the forms), JSON load and
save. The 3D preview (S6) and the playback (S7) come next; the place they take is reserved on the right.
"""
from __future__ import annotations

import copy
import json
import os
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QComboBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QListWidget,
                             QListWidgetItem, QMessageBox, QPlainTextEdit, QPushButton, QSpinBox, QSplitter,
                             QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from core import radar_model as rm
from gui.widgets import DecimalSpinBox
from sim import attitude as att
from sim import scenario as scn

# field kinds: "text", "float", "int", "choice", "vecN" (N numbers), "json" (a dict or list typed as JSON)
Field = Tuple[str, str, str, dict]          # (key, label, kind, options)

_BIG = 1e12


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
# attitude by waypoints (pass S3a, sim/attitude.py): edited as JSON until a waypoint editor exists
ATTITUDE_FIELD = _f("attitude", "attitude waypoints [{t_s, rate_dps, attitude}, ...] (JSON)", "json")
MANOEUVRE_FIELDS: Dict[str, List[Field]] = {
    "impulse": [_f("t_s", "time (s)", min=0.0), _f("dv_mps", "delta-v (m/s)", "vec3"),
                _f("frame", "frame", "choice", choices=("rtn", "inertial"))],
    "burn": [_f("t_s", "start (s)", min=0.0), _f("duration_s", "duration (s)", min=0.001),
             _f("accel_mps2", "acceleration (m/s2)", "vec3"), _f("frame", "frame", "choice", choices=("rtn", "inertial"))],
    "singer": [],
    "pn": [_f("n", "navigation constant", min=0.1), _f("start_range_km", "start below range (km, or empty: time)", "json"),
           _f("t_s", "start time (s, or empty: range)", "json")],
}
MANOEUVRE_DEFAULTS = {"impulse": {"type": "impulse", "t_s": 0.0, "dv_mps": [0.0, 0.0, 0.0], "frame": "rtn"},
                      "burn": {"type": "burn", "t_s": 0.0, "duration_s": 10.0, "accel_mps2": [0.0, 0.0, 0.0],
                               "frame": "rtn"},
                      "singer": {"type": "singer"},
                      "pn": {"type": "pn", "start_range_km": 100.0}}
ORBIT_DEFAULTS = {"elements": {"type": "elements", "a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 0.0,
                               "argp_deg": 0.0, "nu_deg": 0.0},
                  "state": {"type": "state", "r_km": [7000.0, 0.0, 0.0], "v_kms": [0.0, 7.546, 0.0]},
                  "relative": {"type": "relative", "range_km": 1000.0, "phi_deg": 0.0, "eps_deg": 0.0, "frame": "ship",
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
            b = DecimalSpinBox()
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
        lay = QFormLayout(self)
        for key, label, kind, opt in fields:
            w = self._make(key, kind, opt)
            self.widgets[key] = (kind, w)
            lay.addRow(label, w)

    def _make(self, key: str, kind: str, opt: dict) -> QWidget:
        v = self.data.get(key)
        if kind == "float":
            w = DecimalSpinBox()
            w.setRange(float(opt.get("min", -_BIG)), float(opt.get("max", _BIG)))
            w.setDecimals(int(opt.get("dec", 3)))
            w.setValue(float(v) if isinstance(v, (int, float)) else 0.0)
            w.valueChanged.connect(lambda x, k=key: self._set(k, float(x)))
        elif kind == "int":
            w = QSpinBox()
            w.setRange(int(opt.get("min", 0)), int(opt.get("max", 10 ** 9)))
            w.setValue(int(v) if isinstance(v, int) else int(opt.get("min", 0)))
            w.valueChanged.connect(lambda x, k=key: self._set(k, int(x)))
        elif kind == "choice":
            w = QComboBox()
            choices = list(opt["choices"]() if callable(opt["choices"]) else opt["choices"])
            w.addItems(choices)
            if v in choices:
                w.setCurrentIndex(choices.index(v))
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
    combo = QComboBox()
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
            on_change()
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
        row = QHBoxLayout()
        for t in MANOEUVRE_FIELDS:
            b = QPushButton(f"+ {t}")
            b.clicked.connect(lambda _c, tt=t: self._add(tt))
            row.addWidget(b)
        rm_b = QPushButton("Remove")
        rm_b.clicked.connect(self._remove)
        row.addWidget(rm_b)
        lay.addLayout(row)
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
            return f"terminal guidance (PN) {start}"
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


# ----------------------------------------------------------------------------- the tab
class ScenarioTab(QWidget):
    """Scenario editor: tree, forms, validation messages, JSON files."""
    scenario_changed = pyqtSignal(dict)           # the scenario on screen, after each valid change (for S6 / S7)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sc: dict = new_scenario()
        self.path: Optional[str] = None
        self.modified = False

        top = QHBoxLayout()
        for text, slot in (("New", self.new), ("Open...", self.open), ("Save", self.save), ("Save as...", self.save_as)):
            b = QPushButton(text)
            b.clicked.connect(slot)
            top.addWidget(b)
        top.addStretch(1)
        self.file_label = QLabel()
        top.addWidget(self.file_label)

        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.currentItemChanged.connect(lambda cur, _prev: self._show(cur))
        tree_buttons = QHBoxLayout()
        for text, slot in (("+ object", self.add_object), ("+ group", self.add_group), ("Duplicate", self.duplicate),
                           ("Delete", self.delete)):
            b = QPushButton(text)
            b.clicked.connect(slot)
            tree_buttons.addWidget(b)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.addWidget(self.tree, 1)
        ll.addLayout(tree_buttons)

        self.form_holder = QWidget()
        self.form_layout = QVBoxLayout(self.form_holder)
        self.form_layout.setContentsMargins(4, 0, 4, 0)
        self.messages = QListWidget()
        self.messages.setMaximumHeight(140)
        middle = QWidget()
        ml = QVBoxLayout(middle)
        ml.setContentsMargins(0, 0, 0, 0)
        ml.addWidget(self.form_holder, 1)
        ml.addWidget(QLabel("Validation"))
        ml.addWidget(self.messages)

        self.preview = QLabel("3D preview and playback: passes S6 and S7")
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setStyleSheet("color: gray")

        split = QSplitter(Qt.Horizontal)
        split.addWidget(left)
        split.addWidget(middle)
        split.addWidget(self.preview)
        split.setSizes([260, 460, 680])
        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addWidget(split, 1)
        self._rebuild_tree()
        self._validate()

    # ------------------------------------------------------------------ files
    def _confirm_discard(self) -> bool:
        if not self.modified:
            return True
        return QMessageBox.question(self, "Scenario", "Discard the unsaved changes?",
                                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes

    def new(self) -> None:
        if self._confirm_discard():
            self._set_scenario(new_scenario(), None)

    def open(self) -> None:
        if not self._confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(self, "Open a scenario", self._dir(), "Scenario (*.json)")
        if not path:
            return
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

    def _dir(self) -> str:
        return os.path.dirname(self.path) if self.path else os.getcwd()

    def _set_scenario(self, sc: dict, path: Optional[str]) -> None:
        self.sc, self.path, self.modified = sc, path, False
        self._rebuild_tree()
        self._validate()
        self._update_title()

    def _update_title(self) -> None:
        name = os.path.basename(self.path) if self.path else "(not saved)"
        self.file_label.setText(name + (" *" if self.modified else ""))

    # ------------------------------------------------------------------ tree
    def _rebuild_tree(self, select: Optional[Tuple[str, int]] = None) -> None:
        self.tree.blockSignals(True)
        self.tree.clear()
        self.n_meta = QTreeWidgetItem(["Time and body"])
        self.n_meta.setData(0, Qt.UserRole, ("meta", -1))
        self.n_ship = QTreeWidgetItem(["Ship"])
        self.n_ship.setData(0, Qt.UserRole, ("ship", -1))
        self.n_obj = QTreeWidgetItem(["Objects"])
        self.n_obj.setData(0, Qt.UserRole, ("objects", -1))
        self.n_grp = QTreeWidgetItem(["Groups"])
        self.n_grp.setData(0, Qt.UserRole, ("groups", -1))
        self.tree.addTopLevelItems([self.n_meta, self.n_ship, self.n_obj, self.n_grp])
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
        self.tree.setCurrentItem(found or self.n_meta)

    @staticmethod
    def _item_label(kind: str, o: dict) -> str:
        if kind == "groups":
            return f"{o.get('id', '?')} ({o.get('n', '?')} x {o.get('class', '?')} around {o.get('around', '?')})"
        return f"{o.get('id', '?')} ({o.get('class', '?')})"

    def _selected(self) -> Tuple[str, int]:
        it = self.tree.currentItem()
        return it.data(0, Qt.UserRole) if it is not None else ("meta", -1)

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
            self._changed(rebuild=(kind, max(0, i - 1)) if self.sc[kind] else ("meta", -1))

    # ------------------------------------------------------------------ forms
    def _show(self, item: Optional[QTreeWidgetItem]) -> None:
        _clear(self.form_layout)
        if item is None:
            return
        kind, i = item.data(0, Qt.UserRole)
        if kind == "meta":
            w = self._meta_form()
        elif kind == "ship":
            w = self._ship_form()
        elif kind in ("objects", "groups") and i >= 0:
            w = self._object_form(kind, i)
        else:
            w = QLabel("Select an element, or add one with the buttons below the tree.")
        self.form_layout.addWidget(w)
        self.form_layout.addStretch(1)

    def _meta_form(self) -> QWidget:
        box = QWidget()
        lay = QVBoxLayout(box)
        meta = self.sc.setdefault("meta", {})
        f1 = FormWidget([_f("name", "name", "text"), _f("epoch_utc", "epoch (UTC, ISO 8601)", "text"),
                         _f("duration_s", "duration (s)", min=0.001), _f("step_s", "truth step (s, empty: default)",
                                                                           min=0.0, dec=4),
                         _f("seed", "seed", "int")], meta)
        f1.changed.connect(self._changed)
        lay.addWidget(f1)
        holder = self.sc
        f2 = FormWidget([_f("central_body", "central body", "choice",
                            choices=lambda: list(scn.bodies()) or ["earth", "moon"]),
                         _f("mission", "mission file (relative to the scenario)", "text")], holder)
        f2.changed.connect(self._changed)
        lay.addWidget(f2)
        return box

    def _ship_form(self) -> QWidget:
        box = QWidget()
        lay = QVBoxLayout(box)
        ship = self.sc.setdefault("ship", {})
        lay.addWidget(_typed_form("orbit", ("elements", "state"), ORBIT_FIELDS, ORBIT_DEFAULTS, ship, "orbit", "type",
                                  self._changed))
        fa = FormWidget([ATTITUDE_FIELD], ship)
        fa.changed.connect(self._changed)
        lay.addWidget(fa)
        return box

    def _object_form(self, kind: str, i: int) -> QWidget:
        o = self.sc[kind][i]
        box = QWidget()
        lay = QVBoxLayout(box)
        classes = sorted(rm.TARGETS)
        fields = [_f("id", "id", "text"), _f("class", "class", "choice", choices=classes),
                  _f("t_appear_s", "appears at (s)", min=0.0)]
        if kind == "objects":
            fields.append(_f("jammer", "jammer", "choice", choices=[""] + sorted(scn.jammer_classes())))
        else:
            fields += [_f("n", "count", "int", min=1),
                       _f("around", "around object", "choice", choices=[x["id"] for x in self.sc.get("objects", [])]),
                       _f("pos_sigma_m", "position spread (m, 1 sigma)", min=0.0),
                       _f("vel_sigma_mps", "velocity spread (m/s, 1 sigma)", min=0.0)]
        fields.append(_f("overrides", "class overrides (JSON)", "json"))
        if kind == "objects" and o.get("class") in att.FLYING_CLASSES:
            fields.append(ATTITUDE_FIELD)
        form = FormWidget(fields, o)
        form.changed.connect(lambda: self._changed(relabel=(kind, i)))
        lay.addWidget(form)
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

    def _validate(self) -> None:
        err = scn.validate(self.sc)
        self.messages.clear()
        if not err:
            it = QListWidgetItem("Scenario valid")
            it.setForeground(QColor("darkgreen"))
            self.messages.addItem(it)
            self.scenario_changed.emit(scn.normalise(self.sc))
            return
        for e in err:
            it = QListWidgetItem(e)
            it.setForeground(QColor("darkred"))
            self.messages.addItem(it)
