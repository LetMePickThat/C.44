#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PREP_TAB -- mission-preparation tab: watch-sector form + real-time 3D view + fast cost estimate (per-face gauges) +
alerts (incl. redundant watch volumes, core/redundancy.py) + mission-prep validation. "Run calculation" hands a frozen
copy of the mission to the main window, which runs the optimiser and opens the results tab; while a results tab is
open the mission is locked (set_locked), so the results always describe the mission on screen.
"""
from __future__ import annotations

import copy
from typing import Dict, List, Optional

import numpy as np
from PyQt5.QtCore import QThread, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QMessageBox, QProgressBar, QPushButton, QScrollArea, QSpinBox, QSplitter,
    QVBoxLayout, QWidget,
)

from core import cost_estimate as ce
from core import geometry as geo
from core import mission as md
from core import redundancy as red
from gui.scene_view import SceneView, SectorDraw

DEBOUNCE_MS = 200
REDUNDANCY_DEBOUNCE_MS = 600    # the redundancy check plans sectors with the optimiser: wait for typing to settle
ALERT_COLOR = "#c0392b"


class RedundancyWorker(QThread):
    """Runs core.redundancy.find_redundancies() off the UI thread (it plans the earlier sectors with the optimiser).
    `token` is handed back untouched so the caller can tell whether the result is still current."""
    finished_ok = pyqtSignal(object, object)   # list of redundancy.Finding, token
    failed = pyqtSignal(str)

    def __init__(self, mission: dict, grid, token, parent=None):
        super().__init__(parent)
        self.mission, self.grid, self.token = mission, grid, token

    def run(self) -> None:
        try:
            findings = red.find_redundancies(self.mission, self.grid)
        except Exception as exc:  # noqa: BLE001 -- a failed check must not take the window down
            self.failed.emit(f"redundancy check failed: {exc}")
            return
        self.finished_ok.emit(findings, self.token)


class PrepTab(QWidget):
    run_requested = pyqtSignal(dict)   # frozen (deep) copy of a valid mission to optimise

    def __init__(self, mission: dict, grid: geo.SphereGrid, parent=None):
        super().__init__(parent)
        self.mission: dict = mission
        self.grid = grid
        self.cost_table = ce.load_table()
        self.locked = False
        self._cur_index: int = -1
        self._updating_form = False
        self._hidden_sectors: set = set()     # id() of the sector dicts whose volume is hidden in 3D
        # redundancy check (core/redundancy.py): findings are valid while the mission signature they were computed
        # for is the current one; "Keep both" acknowledgements are per sector content and covering sectors
        self._redundancy: Dict[int, red.Finding] = {}   # id() of the redundant sector dict -> finding
        self._redundancy_sig: Optional[str] = None      # signature the findings belong to
        self._redundancy_requested: Optional[str] = None
        self._redundancy_error = ""
        self._redundancy_ack: set = set()
        self._red_worker: Optional[RedundancyWorker] = None
        self._red_timer = QTimer(self)
        self._red_timer.setSingleShot(True)
        self._red_timer.setInterval(REDUNDANCY_DEBOUNCE_MS)
        self._red_timer.timeout.connect(self._start_redundancy_check)

        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(DEBOUNCE_MS)
        self._debounce.timeout.connect(self._refresh_all)

        self._build_ui()
        self._reload_sector_list()
        self._schedule_refresh()

    # ------------------------------------------------------------------ UI construction
    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(splitter)

        self.view = SceneView(self.grid, face_opacity=0.28, sector_opacity=0.5)
        self.view.show_all_sectors.connect(lambda: self._set_all_sectors_visible(True))
        self.view.hide_all_sectors.connect(lambda: self._set_all_sectors_visible(False))
        splitter.addWidget(self.view)

        panel = QScrollArea()
        panel.setWidgetResizable(True)
        inner = QWidget()
        panel.setWidget(inner)
        v = QVBoxLayout(inner)

        self.lbl_locked = QLabel("Mission locked while the results tab is open: close it to edit the mission.")
        self.lbl_locked.setWordWrap(True)
        self.lbl_locked.setStyleSheet("background: #e08a1e; color: white; font-weight: bold; padding: 6px;")
        self.lbl_locked.setVisible(False)
        v.addWidget(self.lbl_locked)
        v.addWidget(self._build_global_group())
        v.addWidget(self._build_sector_list_group())
        v.addWidget(self._build_sector_form_group())
        v.addWidget(self._build_gauges_group())
        v.addWidget(self._build_alerts_group())
        v.addWidget(self._build_actions_group())
        v.addStretch(1)

        # no horizontal scrolling: the panel is always at least as wide as its content
        panel.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        min_w = (inner.minimumSizeHint().width() + panel.verticalScrollBar().sizeHint().width()
                 + 2 * panel.frameWidth())
        panel.setMinimumWidth(min_w)
        panel.setMaximumWidth(max(460, min_w))

        splitter.addWidget(panel)
        splitter.setChildrenCollapsible(False)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

    def set_locked(self, locked: bool) -> None:
        """Read-only mission while a results tab is open: every editing control is disabled (selecting a sector,
        showing / hiding volumes, the 3D view, validation and mission export stay available)."""
        self.locked = locked
        self.lbl_locked.setVisible(locked)
        for w in self._edit_widgets:
            w.setEnabled(not locked)

    def _build_global_group(self) -> QGroupBox:
        box = QGroupBox("Global scan zone")
        self._edit_widgets: List[QWidget] = [box]
        form = QFormLayout(box)
        self.sb_eps_min = self._spin(-90, 90, 1, self.mission["global"].get("eps_min_deg", -90.0))
        self.sb_eps_max = self._spin(-90, 90, 1, self.mission["global"].get("eps_max_deg", 90.0))
        self.sb_elec = self._spin(0.1, 8.0, 0.1, self.mission["global"].get("face_elec_mw", 2.0))
        form.addRow("Rear limit eps_min (deg)", self.sb_eps_min)
        form.addRow("Front limit eps_max (deg)", self.sb_eps_max)
        form.addRow("Electrical power / face (MW)", self.sb_elec)
        for sb in (self.sb_eps_min, self.sb_eps_max, self.sb_elec):
            sb.valueChanged.connect(self._on_global_changed)
        return box

    def _build_sector_list_group(self) -> QGroupBox:
        box = QGroupBox("Watch sectors")
        v = QVBoxLayout(box)
        self.list_sectors = QListWidget()
        self.list_sectors.setSelectionMode(QAbstractItemView.SingleSelection)
        self.list_sectors.currentRowChanged.connect(self._on_select_sector)
        self.list_sectors.itemChanged.connect(self._on_sector_check_changed)
        self.list_sectors.setToolTip("Tick / untick a sector to show / hide its volume in the 3D view")
        v.addWidget(self.list_sectors)
        row = QHBoxLayout()
        btn_add_box = QPushButton("+ Box")
        btn_add_cap = QPushButton("+ Cap")
        btn_dup = QPushButton("Duplicate")
        btn_del = QPushButton("Delete")
        btn_add_box.clicked.connect(lambda: self._add_sector("box"))
        btn_add_cap.clicked.connect(lambda: self._add_sector("cap"))
        btn_dup.clicked.connect(self._duplicate_sector)
        btn_del.clicked.connect(self._delete_sector)
        for b in (btn_add_box, btn_add_cap, btn_dup, btn_del):
            row.addWidget(b)
            self._edit_widgets.append(b)
        v.addLayout(row)
        return box

    def _build_sector_form_group(self) -> QGroupBox:
        box = QGroupBox("Selected sector")
        self._edit_widgets.append(box)
        v = QVBoxLayout(box)
        top = QFormLayout()
        self.ed_id = QComboBox()  # editable combo used as a simple validated text field
        self.ed_id.setEditable(True)
        self.ed_id.editTextChanged.connect(self._on_id_changed)
        self.cb_shape = QComboBox()
        self.cb_shape.addItems(["box", "cap"])
        self.cb_shape.currentTextChanged.connect(self._on_shape_changed)
        top.addRow("id", self.ed_id)
        top.addRow("shape", self.cb_shape)
        v.addLayout(top)

        self.grp_box = QGroupBox("Zone (box)")
        fb = QFormLayout(self.grp_box)
        self.sb_phi_min = self._spin(0, 360, 1, 0.0)
        self.sb_phi_max = self._spin(0, 360, 1, 90.0)
        self.sb_box_eps_min = self._spin(-90, 90, 1, 0.0)
        self.sb_box_eps_max = self._spin(-90, 90, 1, 60.0)
        fb.addRow("phi_min (deg)", self.sb_phi_min)
        fb.addRow("phi_max (deg)", self.sb_phi_max)
        fb.addRow("eps_min (deg)", self.sb_box_eps_min)
        fb.addRow("eps_max (deg)", self.sb_box_eps_max)
        v.addWidget(self.grp_box)

        self.grp_cap = QGroupBox("Zone (cap)")
        fc = QFormLayout(self.grp_cap)
        self.sb_phi0 = self._spin(0, 360, 1, 90.0)
        self.sb_eps0 = self._spin(-90, 90, 1, 20.0)
        self.sb_radius = self._spin(0.1, 180, 1, 15.0)
        fc.addRow("phi0 (deg)", self.sb_phi0)
        fc.addRow("eps0 (deg)", self.sb_eps0)
        fc.addRow("radius (deg)", self.sb_radius)
        v.addWidget(self.grp_cap)

        common = QFormLayout()
        self.sb_rcs = self._spin(0.01, 1_000_000, 1, 1000.0)
        self.sb_swerling = QSpinBox()
        self.sb_swerling.setRange(0, 4)
        self.sb_swerling.setValue(1)
        self.sb_swerling.valueChanged.connect(self._on_field_changed)
        self.sb_range = self._spin(1, 50000, 10, 2000.0)
        self.sb_rr = self._spin(0.1, 600, 1, 30.0)
        self.sb_vmin = self._spin(0, 40000, 100, 1000.0)
        self.sb_vmax = self._spin(0, 40000, 100, 5000.0)
        common.addRow("RCS (m2)", self.sb_rcs)
        common.addRow("Swerling", self.sb_swerling)
        common.addRow("Required range (km)", self.sb_range)
        common.addRow("Target RR (s)", self.sb_rr)
        common.addRow("Closing speed min (m/s)", self.sb_vmin)
        common.addRow("Closing speed max (m/s)", self.sb_vmax)
        v.addLayout(common)

        for sb in (self.sb_phi_min, self.sb_phi_max, self.sb_box_eps_min, self.sb_box_eps_max,
                   self.sb_phi0, self.sb_eps0, self.sb_radius, self.sb_rcs, self.sb_range,
                   self.sb_rr, self.sb_vmin, self.sb_vmax):
            sb.valueChanged.connect(self._on_field_changed)

        self.grp_box.setVisible(True)
        self.grp_cap.setVisible(False)
        return box

    def _build_gauges_group(self) -> QGroupBox:
        box = QGroupBox("Per-face budget (fast estimate)")
        v = QVBoxLayout(box)
        self.gauges: Dict[int, Dict[str, QWidget]] = {}
        for k in range(4):
            row_box = QGroupBox(f"{geo.FACE_NAMES[k]} (phi={geo.FACE_PHI_DEG[k]:.0f} deg)")
            row_box.setStyleSheet(f"QGroupBox::title {{ color: {geo.FACE_COLORS[k]}; }}")
            form = QFormLayout(row_box)
            pb_e = QProgressBar(); pb_e.setRange(0, 100); pb_e.setFormat("energy %p%")
            pb_t = QProgressBar(); pb_t.setRange(0, 100); pb_t.setFormat("time on air >= %p%")
            lbl_kw = QLabel("0 kW")
            form.addRow("Energy", pb_e)
            form.addRow("Time on air (lower bound)", pb_t)
            lbl_margin = QLabel("-")
            form.addRow("Average power", lbl_kw)
            form.addRow("Tracking margin", lbl_margin)
            v.addWidget(row_box)
            self.gauges[k] = {"energy": pb_e, "time": pb_t, "kw": lbl_kw, "margin": lbl_margin}
        note = QLabel("Fast estimate (precomputed table): indicative while editing. "
                       "The exact time on air comes from 'Run calculation'.")
        note.setWordWrap(True)
        note.setStyleSheet("color: gray; font-size: 11px;")
        v.addWidget(note)
        return box

    def _build_alerts_group(self) -> QGroupBox:
        box = QGroupBox("Alerts")
        v = QVBoxLayout(box)
        self.list_alerts = QListWidget()
        self.list_alerts.setMaximumHeight(140)
        self.list_alerts.itemDoubleClicked.connect(self._on_alert_activated)
        v.addWidget(self.list_alerts)
        return box

    def _build_actions_group(self) -> QGroupBox:
        box = QGroupBox("Import / export / calculation")
        v = QVBoxLayout(box)
        row1 = QHBoxLayout()
        b_imp = QPushButton("Import JSON/CSV")
        b_exp_j = QPushButton("Export JSON")
        b_exp_c = QPushButton("Export CSV")
        b_imp.clicked.connect(self._on_import)
        b_exp_j.clicked.connect(lambda: self._on_export("json"))
        b_exp_c.clicked.connect(lambda: self._on_export("csv"))
        for b in (b_imp, b_exp_j, b_exp_c):
            row1.addWidget(b)
        v.addLayout(row1)

        b_validate = QPushButton("Validate mission prep")
        b_validate.clicked.connect(self._on_validate)
        v.addWidget(b_validate)

        b_run = QPushButton("Run calculation")
        b_run.setStyleSheet("font-weight: bold;")
        b_run.clicked.connect(self._on_run)
        v.addWidget(b_run)
        self._edit_widgets += [b_imp, b_run]
        return box

    @staticmethod
    def _spin(lo: float, hi: float, step: float, value: float) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setSingleStep(step)
        sb.setDecimals(2)
        sb.setValue(value)
        return sb

    # ------------------------------------------------------------------ sector list helpers
    def _sector_ids(self) -> List[str]:
        return [s.get("id", "") for s in self.mission["sectors"]]

    def _reload_sector_list(self) -> None:
        self.list_sectors.blockSignals(True)
        self.list_sectors.clear()
        live = {id(sec) for sec in self.mission["sectors"]}
        self._hidden_sectors &= live          # forget deleted sectors (their ids may be reused)
        for sec in self.mission["sectors"]:
            item = QListWidgetItem()
            self._decorate_item(item, sec)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked if id(sec) in self._hidden_sectors else Qt.Checked)
            self.list_sectors.addItem(item)
        self.list_sectors.blockSignals(False)
        if self.mission["sectors"]:
            idx = min(max(self._cur_index, 0), len(self.mission["sectors"]) - 1)
            self.list_sectors.setCurrentRow(idx)
        else:
            self._cur_index = -1
            self._load_form(None)

    @staticmethod
    def _sector_label(sec: dict) -> str:
        shape = sec.get("shape", "box")
        return f"{sec.get('id', '?')}  [{shape}]  R={sec.get('range_km', 0):.0f} km  RCS={sec.get('rcs_m2', 0):g} m2"

    def _decorate_item(self, item: QListWidgetItem, sec: dict) -> None:
        """Sector list entry: label, plus a red 'need already met' tag while the sector is redundant."""
        f = self._active_finding(sec)
        if f is None:
            item.setText(self._sector_label(sec))
            item.setForeground(QColor())
            item.setToolTip("")
            return
        met = "need met" if f.full else f"{100 * f.fraction:.0f} % of need met"
        item.setText(f"{self._sector_label(sec)}   ! {met} by {', '.join(f.by)}")
        item.setForeground(QColor(ALERT_COLOR))
        item.setToolTip("Double-click the matching alert to resolve")

    def _refresh_sector_items(self) -> None:
        self.list_sectors.blockSignals(True)
        for row, sec in enumerate(self.mission["sectors"]):
            if row < self.list_sectors.count():
                self._decorate_item(self.list_sectors.item(row), sec)
        self.list_sectors.blockSignals(False)

    def _add_sector(self, shape: str) -> None:
        sec = md.new_sector(self._sector_ids(), shape=shape)
        self.mission["sectors"].append(sec)
        self._cur_index = len(self.mission["sectors"]) - 1
        self._reload_sector_list()
        self._schedule_refresh()

    def _duplicate_sector(self) -> None:
        if self._cur_index < 0:
            return
        dup = md.duplicate_sector(self.mission["sectors"][self._cur_index], self._sector_ids())
        self.mission["sectors"].insert(self._cur_index + 1, dup)
        self._cur_index += 1
        self._reload_sector_list()
        self._schedule_refresh()

    def _delete_sector(self) -> None:
        if self._cur_index < 0:
            return
        del self.mission["sectors"][self._cur_index]
        self._cur_index = min(self._cur_index, len(self.mission["sectors"]) - 1)
        self._reload_sector_list()
        self._schedule_refresh()

    # ------------------------------------------------------------------ 3D visibility
    def _on_sector_check_changed(self, item: QListWidgetItem) -> None:
        row = self.list_sectors.row(item)
        if not 0 <= row < len(self.mission["sectors"]):
            return
        key = id(self.mission["sectors"][row])
        if item.checkState() == Qt.Checked:
            self._hidden_sectors.discard(key)
        else:
            self._hidden_sectors.add(key)
        self._push_hidden()

    def _set_all_sectors_visible(self, visible: bool) -> None:
        self._hidden_sectors = set() if visible else {id(s) for s in self.mission["sectors"]}
        self.list_sectors.blockSignals(True)
        for row in range(self.list_sectors.count()):
            self.list_sectors.item(row).setCheckState(Qt.Checked if visible else Qt.Unchecked)
        self.list_sectors.blockSignals(False)
        self._push_hidden()

    @staticmethod
    def _draw_sid(i: int, sec: dict) -> str:
        return sec.get("id") or f"_idx{i}"

    def _hidden_sids(self) -> set:
        return {self._draw_sid(i, sec) for i, sec in enumerate(self.mission["sectors"])
                if id(sec) in self._hidden_sectors}

    def _push_hidden(self) -> None:
        self.view.set_hidden(self._hidden_sids())

    # ------------------------------------------------------------------ form <-> model
    def _on_select_sector(self, row: int) -> None:
        self._cur_index = row
        sec = self.mission["sectors"][row] if 0 <= row < len(self.mission["sectors"]) else None
        self._load_form(sec)

    def _load_form(self, sec: Optional[dict]) -> None:
        self._updating_form = True
        try:
            enabled = sec is not None
            for w in (self.ed_id, self.cb_shape, self.sb_rcs, self.sb_swerling, self.sb_range,
                      self.sb_rr, self.sb_vmin, self.sb_vmax, self.grp_box, self.grp_cap):
                w.setEnabled(enabled)
            if not sec:
                return
            self.ed_id.setEditText(sec.get("id", ""))
            shape = sec.get("shape", "box")
            self.cb_shape.setCurrentText(shape)
            self.grp_box.setVisible(shape == "box")
            self.grp_cap.setVisible(shape == "cap")
            self.sb_phi_min.setValue(sec.get("phi_min_deg", 0.0))
            self.sb_phi_max.setValue(sec.get("phi_max_deg", 90.0))
            self.sb_box_eps_min.setValue(sec.get("eps_min_deg", 0.0))
            self.sb_box_eps_max.setValue(sec.get("eps_max_deg", 60.0))
            self.sb_phi0.setValue(sec.get("phi0_deg", 90.0))
            self.sb_eps0.setValue(sec.get("eps0_deg", 20.0))
            self.sb_radius.setValue(sec.get("radius_deg", 15.0))
            self.sb_rcs.setValue(sec.get("rcs_m2", 1000.0))
            self.sb_swerling.setValue(int(sec.get("swerling", 1)))
            self.sb_range.setValue(sec.get("range_km", 2000.0))
            self.sb_rr.setValue(sec.get("rr_target_s", 30.0))
            self.sb_vmin.setValue(sec.get("v_close_min_mps", 1000.0))
            self.sb_vmax.setValue(sec.get("v_close_max_mps", 5000.0))
        finally:
            self._updating_form = False

    def _on_id_changed(self, text: str) -> None:
        if self._updating_form or self._cur_index < 0:
            return
        self.mission["sectors"][self._cur_index]["id"] = text
        self._decorate_item(self.list_sectors.item(self._cur_index), self.mission["sectors"][self._cur_index])
        self._schedule_refresh()

    def _on_shape_changed(self, shape: str) -> None:
        if self._updating_form or self._cur_index < 0:
            return
        sec = self.mission["sectors"][self._cur_index]
        sec["shape"] = shape
        self.grp_box.setVisible(shape == "box")
        self.grp_cap.setVisible(shape == "cap")
        self._sync_form_to_model()

    def _on_field_changed(self, *_args) -> None:
        if self._updating_form or self._cur_index < 0:
            return
        self._sync_form_to_model()

    def _sync_form_to_model(self) -> None:
        sec = self.mission["sectors"][self._cur_index]
        sec["shape"] = self.cb_shape.currentText()
        if sec["shape"] == "box":
            sec["phi_min_deg"] = self.sb_phi_min.value()
            sec["phi_max_deg"] = self.sb_phi_max.value()
            sec["eps_min_deg"] = self.sb_box_eps_min.value()
            sec["eps_max_deg"] = self.sb_box_eps_max.value()
            for f in ("phi0_deg", "eps0_deg", "radius_deg"):
                sec.pop(f, None)
        else:
            sec["phi0_deg"] = self.sb_phi0.value()
            sec["eps0_deg"] = self.sb_eps0.value()
            sec["radius_deg"] = self.sb_radius.value()
            for f in ("phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg"):
                sec.pop(f, None)
        sec["rcs_m2"] = self.sb_rcs.value()
        sec["swerling"] = self.sb_swerling.value()
        sec["range_km"] = self.sb_range.value()
        sec["rr_target_s"] = self.sb_rr.value()
        sec["v_close_min_mps"] = self.sb_vmin.value()
        sec["v_close_max_mps"] = self.sb_vmax.value()
        self._decorate_item(self.list_sectors.item(self._cur_index), sec)
        self._schedule_refresh()

    def _on_global_changed(self, *_args) -> None:
        self.mission["global"]["eps_min_deg"] = self.sb_eps_min.value()
        self.mission["global"]["eps_max_deg"] = self.sb_eps_max.value()
        self.mission["global"]["face_elec_mw"] = self.sb_elec.value()
        self._schedule_refresh()

    # ------------------------------------------------------------------ refresh (debounced)
    def _schedule_refresh(self) -> None:
        self._debounce.start()

    def _refresh_all(self) -> None:
        eps_min = self.mission["global"].get("eps_min_deg", -90.0)
        eps_max = self.mission["global"].get("eps_max_deg", 90.0)
        self._update_gauges(eps_min, eps_max)
        self._schedule_redundancy_check()
        self._update_alerts(eps_min, eps_max)
        self._update_scene(eps_min, eps_max)

    def _update_scene(self, eps_min: float, eps_max: float) -> None:
        draws = []
        for i, sec in enumerate(self.mission["sectors"]):
            sid = self._draw_sid(i, sec)
            text = (f"{sid}\nR={sec.get('range_km', 0):.0f} km  RCS={sec.get('rcs_m2', 0):g} m2\n"
                    f"RR={sec.get('rr_target_s', 0):.0f} s  V={sec.get('v_close_min_mps', 0):.0f}"
                    f"-{sec.get('v_close_max_mps', 0):.0f} m/s")
            draws.append(SectorDraw(sid, sec, geo.sector_color(i), text))
        self.view.set_scene(eps_min, eps_max, draws, hidden=self._hidden_sids())

    def _update_gauges(self, eps_min: float, eps_max: float) -> None:
        totals = {k: {"energy": 0.0, "time_min": 0.0, "kw": 0.0} for k in range(4)}
        p_avg_max = self.mission["global"].get("face_elec_mw", 2.0) * 1e6 * geo.wf.CHAIN_EFF * geo.wf.PAE
        if self.cost_table is not None:
            for sec in self.mission["sectors"]:
                try:
                    faces = ce.estimate_sector_faces(self.cost_table, sec, self.grid, eps_min, eps_max, p_avg_max)
                except (KeyError, TypeError, ZeroDivisionError):
                    continue
                for k, fb in faces.items():
                    totals[k]["energy"] += fb.energy_fraction
                    totals[k]["time_min"] += fb.time_fraction_min
                    totals[k]["kw"] += fb.avg_power_kw
        for k in range(4):
            g = self.gauges[k]
            g["energy"].setValue(min(100, round(100 * totals[k]["energy"])))
            over_e = totals[k]["energy"] > 1.0
            g["energy"].setStyleSheet("QProgressBar::chunk { background-color: #c0392b; }" if over_e else "")
            g["time"].setValue(min(100, round(100 * totals[k]["time_min"])))
            g["kw"].setText(f"{totals[k]['kw']:.0f} kW")
            # tracking margin: power left for Ku after search (the arrays only share the face's power)
            p_left_w = max(0.0, 1.0 - totals[k]["energy"]) * p_avg_max
            g["margin"].setText(f"{p_left_w / 1e3:.0f} kW (Ku duty {100 * p_left_w / geo.wf.BAND_KU.p_pk_total_w:.1f} %)")

    def _update_alerts(self, eps_min: float, eps_max: float) -> None:
        self.list_alerts.clear()
        sectors = self.mission["sectors"]
        self._add_redundancy_alerts()
        masks = {}
        for sec in sectors:
            sid = sec.get("id")
            if not sid:
                continue
            try:
                m = self.grid.sector_mask(sec)
            except (KeyError, TypeError):
                continue
            scanned = m & self.grid.scanned(eps_min, eps_max)
            masks[sid] = m
            tot_om = float((self.grid.d_omega * m).sum())
            scanned_om = float((self.grid.d_omega * scanned).sum())
            if tot_om > 0 and scanned_om / tot_om < 0.995:
                pct = 100 * (1 - scanned_om / tot_om)
                self.list_alerts.addItem(f"{sid}: {pct:.0f} % outside the scanned zone (scan limit or eps window)")
        ids = list(masks.keys())
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                ov = float((self.grid.d_omega * (masks[ids[i]] & masks[ids[j]])).sum())
                if ov > 1e-4:
                    self.list_alerts.addItem(f"{ids[i]} / {ids[j]} overlap: budgeted twice")
        for k in range(4):
            e = self.gauges[k]["energy"].value()
            if e >= 100:
                self.list_alerts.addItem(f"{geo.FACE_NAMES[k]}: estimated energy over-subscribed (>= 100 %)")
            elif e > 90:
                self.list_alerts.addItem(f"{geo.FACE_NAMES[k]}: estimated tracking margin < 10 % of the face power")
        if masks:
            scanned_all = self.grid.scanned(eps_min, eps_max)
            covered = np.zeros_like(scanned_all)
            for m in masks.values():
                covered |= m
            free_om = float((self.grid.d_omega * (scanned_all & ~covered)).sum())
            all_om = float((self.grid.d_omega * scanned_all).sum())
            if all_om > 0 and free_om / all_om > 0.005:
                self.list_alerts.addItem(f"{100 * free_om / all_om:.0f} % of the scanned volume is not covered by "
                                         "any watch sector (unallocated)")
        if self.list_alerts.count() == 0:
            self.list_alerts.addItem("No alerts")

    # ------------------------------------------------------------------ redundancy (an earlier sector already meets the need)
    def _schedule_redundancy_check(self) -> None:
        if red.mission_signature(self.mission) != self._redundancy_requested:
            self._red_timer.start()

    def _start_redundancy_check(self) -> None:
        if self._red_worker is not None and self._red_worker.isRunning():
            return  # _on_redundancy_done starts the next check if the mission changed meanwhile
        sig = red.mission_signature(self.mission)
        self._redundancy_requested = sig
        token = (sig, [id(s) for s in self.mission["sectors"]])
        self._red_worker = RedundancyWorker(copy.deepcopy(self.mission), self.grid, token, self)
        self._red_worker.finished_ok.connect(self._on_redundancy_done)
        self._red_worker.failed.connect(self._on_redundancy_failed)
        self._red_worker.start()

    def _on_redundancy_done(self, findings: list, token: tuple) -> None:
        sig, ids = token
        if sig != red.mission_signature(self.mission):
            self._redundancy_requested = None
            self._red_timer.start()       # stale: the mission changed while the check ran
            return
        self._redundancy = {ids[f.index]: f for f in findings}
        self._redundancy_sig, self._redundancy_error = sig, ""
        self._refresh_sector_items()
        self._update_alerts(self.mission["global"].get("eps_min_deg", -90.0),
                            self.mission["global"].get("eps_max_deg", 90.0))

    def _on_redundancy_failed(self, message: str) -> None:
        self._redundancy_error = message
        self._redundancy_sig = red.mission_signature(self.mission)
        self._redundancy = {}
        self._schedule_refresh()

    @staticmethod
    def _ack_key(sec: dict, f: red.Finding) -> tuple:
        return id(sec), red.sector_signature(sec), tuple(f.by)

    def _active_finding(self, sec: dict) -> Optional[red.Finding]:
        """Current finding for this sector, None if none, out of date, or acknowledged ("Keep both")."""
        if self._redundancy_sig is None or self._redundancy_sig != red.mission_signature(self.mission):
            return None
        f = self._redundancy.get(id(sec))
        return None if f is None or self._ack_key(sec, f) in self._redundancy_ack else f

    def _add_redundancy_alerts(self) -> None:
        if self._redundancy_sig != red.mission_signature(self.mission):
            if len(self.mission["sectors"]) > 1:
                item = QListWidgetItem("Checking whether earlier sectors already meet the later needs...")
                item.setForeground(QColor("gray"))
                self.list_alerts.addItem(item)
            return
        if self._redundancy_error:
            self.list_alerts.addItem(self._redundancy_error)
        for i, sec in enumerate(self.mission["sectors"]):
            f = self._active_finding(sec)
            if f is None:
                continue
            by = ", ".join(f.by)
            if f.full:
                text = f"{f.sid}: need already met by {by} -- double-click to keep {by} only"
            elif f.pieces:
                text = (f"{f.sid}: {100 * f.fraction:.0f} % of its volume already met by {by} -- "
                        f"double-click to trim {f.sid}")
            else:
                text = f"{f.sid}: {100 * f.fraction:.0f} % of its volume already met by {by} -- double-click for details"
            item = QListWidgetItem(text)
            item.setForeground(QColor(ALERT_COLOR))
            item.setData(Qt.UserRole, i)
            self.list_alerts.addItem(item)

    def _on_alert_activated(self, item: QListWidgetItem) -> None:
        if self.locked:
            return
        index = item.data(Qt.UserRole)
        if index is None or not 0 <= index < len(self.mission["sectors"]):
            return
        sec = self.mission["sectors"][index]
        f = self._active_finding(sec)
        if f is None:
            return
        sid, by = sec["id"], ", ".join(f.by)
        need = (f"RCS {sec['rcs_m2']:g} m2 SW{sec.get('swerling', 1)} at {sec['range_km']:.0f} km, "
                f"RR {sec['rr_target_s']:.0f} s, closing up to {sec['v_close_max_mps']:.0f} m/s")
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Redundant watch volume")
        b_apply = None
        if f.full:
            box.setText(f"The need of {sid} ({need}) is already met by {by} over its whole scanned volume, "
                        f"with the waveforms the optimiser chooses for {by}.")
            box.setInformativeText(f"Keeping only {by} meets both needs without budgeting the same search twice."
                                   f"\n\n{f.detail}")
            b_apply = box.addButton(f"Keep {by} only (delete {sid})", QMessageBox.AcceptRole)
        else:
            box.setText(f"The need of {sid} ({need}) is already met by {by} over {100 * f.fraction:.0f} % of its "
                        f"scanned volume, with the waveforms the optimiser chooses for {by}.")
            if f.pieces:
                lines = [f"  {p['id']}: phi {p['phi_min_deg']:g} -> {p['phi_max_deg']:g} deg, "
                         f"eps {p['eps_min_deg']:g} -> {p['eps_max_deg']:g} deg" for p in f.pieces]
                text = (f"{f.detail}\n\nProposed: restrict {sid} to the part not already covered:\n"
                        + "\n".join(lines))
                if f.caveat:
                    text += f"\n\nNote: {f.caveat}."
                box.setInformativeText(text)
                b_apply = box.addButton(f"Trim {sid}", QMessageBox.AcceptRole)
            else:
                box.setInformativeText(f"{f.detail}\n\nNo automatic trim: {f.note}.")
        b_keep = box.addButton("Keep both", QMessageBox.ActionRole)
        box.addButton(QMessageBox.Cancel)
        box.exec_()
        clicked = box.clickedButton()
        if clicked is b_keep:
            self._redundancy_ack.add(self._ack_key(sec, f))
            self._refresh_sector_items()
            self._schedule_refresh()
        elif b_apply is not None and clicked is b_apply:
            self.mission["sectors"][index:index + 1] = [] if f.full else [dict(p) for p in f.pieces]
            self._cur_index = min(index, len(self.mission["sectors"]) - 1)
            self._reload_sector_list()
            self._schedule_refresh()

    # ------------------------------------------------------------------ import / export / validate / run
    def _on_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import a mission", "",
                                               "Mission (*.json *.csv)")
        if not path:
            return
        try:
            self.mission = md.load_mission(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Import", f"Import failed: {exc}")
            return
        self.sb_eps_min.setValue(self.mission["global"].get("eps_min_deg", -90.0))
        self.sb_eps_max.setValue(self.mission["global"].get("eps_max_deg", 90.0))
        self.sb_elec.setValue(self.mission["global"].get("face_elec_mw", 2.0))
        self._cur_index = 0
        self._reload_sector_list()
        self._schedule_refresh()

    def _on_export(self, fmt: str) -> None:
        ext = "json" if fmt == "json" else "csv"
        path, _ = QFileDialog.getSaveFileName(self, "Export the mission", f"mission.{ext}",
                                               f"Mission (*.{ext})")
        if not path:
            return
        try:
            if fmt == "json":
                md.save_mission_json(path, self.mission)
            else:
                md.save_mission_csv(path, self.mission)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed: {exc}")

    def _on_validate(self) -> bool:
        errs = md.validate_mission(self.mission)
        if errs:
            QMessageBox.warning(self, "Invalid mission preparation",
                                 "\n".join(f"- {e}" for e in errs))
            return False
        QMessageBox.information(self, "Mission preparation", "Mission preparation is valid.")
        return True

    def _on_run(self) -> None:
        errs = md.validate_mission(self.mission)
        if errs:
            QMessageBox.warning(self, "Invalid mission preparation",
                                "\n".join(f"- {e}" for e in errs))
            return
        self.run_requested.emit(copy.deepcopy(self.mission))
