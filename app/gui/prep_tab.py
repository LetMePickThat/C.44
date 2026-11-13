#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PREP_TAB -- mission-preparation tab: watch-sector form (incl. confirmation mode and active-tracking range brackets) +
real-time 3D view + fast cost estimate (per-face gauges) +
alerts (incl. redundant watch volumes, core/redundancy.py) + mission-prep validation. "Calculate" hands a frozen
copy of the mission to the main window, which runs the optimiser and opens the results tab; the mission stays
editable (user, 13 November 2026): the results tab is then said outdated (its badge) until the mission is back to the
one it was computed for; set_locked is kept but no longer called with True.
"""
from __future__ import annotations

import copy
import json
import math
import os
from datetime import datetime, timezone
from typing import Dict, List, Optional

import numpy as np
from PyQt5.QtCore import QThread, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QKeySequence
from PyQt5.QtWidgets import (
    QLineEdit,
    QApplication,
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QListWidget, QListWidgetItem, QMenu, QMessageBox, QProgressBar, QPushButton, QScrollArea,
    QShortcut, QSlider, QSpinBox, QToolButton,
    QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from core import cost_estimate as ce
from core import frames
from core import geometry as geo
from core import margin
from core import mission as md
from core import optimizer as mo
from core import radar_model as rm
from core import redundancy as red
from core import repoint
from core import tracking as trk
from core import trackmodel as tm
from gui import controls as uc
from gui import scenario_link as slink
from sim import attitude as att
from gui.scene_view import SceneView, SectorDraw
from gui.widgets import DecimalSpinBox

DEBOUNCE_MS = 200
REDUNDANCY_DEBOUNCE_MS = 600    # the redundancy check plans sectors with the optimiser: wait for typing to settle
ALERT_COLOR = "#c0392b"
# AT range-bracket table: (mission field, column header, tooltip)
AT_COLUMNS = (
    ("r_min_km", "Rmin\n(km)", "Near limit of the range bracket (km)"),
    ("r_max_km", "Rmax\n(km)", "Far limit of the range bracket (km); tracks are costed at this range"),
    ("n_targets", "Targets", "Number of targets in active tracking in this bracket"),
    ("rr_s", "RR\n(s)", "Track update period (s)"),
    ("angle_zone_deg", "Zone\n(deg)", "Opening of the angular uncertainty zone = full width at +/-2 sigma "
                                       "(sigma = zone / 4), per axis"),
    ("range_acc_m", "sR\n(m)", "Range accuracy, 1 sigma (m)"),
    ("vr_acc_mps", "sV\n(m/s)", "Radial-velocity accuracy, 1 sigma (m/s)"),
    ("pd_update", "Pd\nupd", "Pd of a track update (step 5.1; default tracking.pd_update)"),
    ("p_discrimination", "P\ndiscr", "Probability of telling the target from the class it must be told from "
                                      "(step 5.1; default tracking.p_discrimination)"),
    ("rr_min_s", "RR\nmin (s)", "Shortest update period when the margin speeds up the tracks (Reallocation "
                                "PRIO_TRACK, step 5.2 P7); 0 = the default floor tool.global_opt.rr_min_track_s"),
)
AT_DEFAULTS = {**md.BRACKET_DEFAULTS, "pd_update": tm.PD_UPDATE, "p_discrimination": tm.P_DISCRIMINATION,
               "rr_min_s": 0.0}
# step 5.1: mission settings (stored in the mission only when they differ from the parameter file)
MISSION_DEFAULTS = {"confirm_p_min": trk.CONFIRM_P_MIN, "ship_rate_max_dps": repoint.RATE_MAX_DPS,
                    "ship_accel_max_dps2": repoint.ACCEL_MAX_DPS2,
                    "load_ceiling": margin.CEILING_DEFAULT}             # step 5.2 (P7)
CONFIRM_TIPS = {
    "auto": "Chosen per sector by the calculation: lowest load of the most loaded face, P confirm >= 0.98",
    "KU1+7": "One Ku look on the S estimate, then a 7-beam Ku cluster if it fails",
    "KU7": "7-beam Ku cluster at once (one round trip)",
    "S2": "Immediate pointed S touch (rejects false alarms in S), then Ku look(s) to start the track",
}


class RedundancyWorker(QThread):
    """Runs core.redundancy.find_redundancies() off the UI thread (it plans the earlier sectors with the optimiser).
    `token` is handed back untouched so the caller can tell whether the result is still current."""
    finished_ok = pyqtSignal(object, object)   # (list of redundancy.Finding, pair_notes dict), token
    failed = pyqtSignal(str)

    def __init__(self, mission: dict, grid, token, parent=None, served=None):
        super().__init__(parent)
        self.mission, self.grid, self.token, self.served = mission, grid, token, served

    def run(self) -> None:
        try:
            result = red.find_redundancies_process(self.mission, self.grid, served=self.served)
        except Exception as exc:  # noqa: BLE001 -- a failed check must not take the window down
            self.failed.emit(f"redundancy check failed: {exc}")
            return
        self.finished_ok.emit(result, self.token)


PANEL_VIEW_MIN_W = 700                 # px left at least to the 3D view when the panel must scroll sideways


class PrepTab(QWidget):
    run_requested = pyqtSignal(dict)   # frozen (deep) copy of a valid mission to optimise
    profile_requested = pyqtSignal()   # U2: the calculation along the flight plan (the Profile mode runs it)
    results_reopened = pyqtSignal()    # one file: a mission opened (its stored exact result may be shown)
    title_changed = pyqtSignal(str)    # the file name, " *" when modified (pass V3)

    def __init__(self, mission: dict, grid: geo.SphereGrid, parent=None):
        super().__init__(parent)
        self.mission: dict = mission
        self.grid = grid
        # the mission file (pass V3, user 10 November 2026, as the Scenario Editor): its path (None before a first
        # save), the mission as last saved or opened (modified = the mission on screen differs from it)
        self.path: Optional[str] = None
        self._saved_sig = self._signature()
        self.file_label = QLabel()
        self.cost_table = ce.load_table()
        self.locked = False
        self._cur_index: int = -1
        self._updating_form = False
        self._hidden_sectors: set = set()     # id() of the sector dicts whose volume is hidden in 3D
        # redundancy check (core/redundancy.py): findings are valid while the mission signature they were computed
        # for is the current one; "Keep both" acknowledgements are per sector content and covering sectors
        self._redundancy: Dict[int, red.Finding] = {}   # id() of the redundant sector dict -> finding
        self._redundancy_notes: Dict[tuple, str] = {}    # (earlier id, later id) -> why overlapping isn't redundant
        self._redundancy_sig: Optional[str] = None      # signature the findings belong to
        self._redundancy_requested: Optional[str] = None
        self._redundancy_error = ""
        self._redundancy_ack: set = set()
        self._unreachable: Dict[str, float] = {}   # sector id -> worst % of positions short of range at full power
        self._red_worker: Optional[RedundancyWorker] = None
        self._red_timer = QTimer(self)
        self._red_timer.setSingleShot(True)
        self._red_timer.setInterval(REDUNDANCY_DEBOUNCE_MS)
        self._red_timer.timeout.connect(self._start_redundancy_check)

        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(DEBOUNCE_MS)
        self._debounce.timeout.connect(self._refresh_begin)

        self._build_ui()
        self._reload_sector_list()
        self._schedule_refresh()
        self._update_title()

    # ------------------------------------------------------------------ UI construction
    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(splitter)

        self.view = SceneView(self.grid, face_opacity=0.28, sector_opacity=0.5, exec_face_colours=True,
                              diagnosis=True)
        self.view.sector_selected.connect(self._select_sector_by_sid)    # (5.5 A: a line of the diagnosis)
        self.view.instant_source = self._instant                      # the playback (user, 10 November 2026)
        self.view.use_instant = lambda t: self._set_scn_t_and_apply(t)
        self.view.instant_shown = self._on_instant_shown          # (the navigation panel follows the playback)
        self.view.show_all_sectors.connect(lambda: self._set_all_sectors_visible(True))
        self.view.rerun_requested.connect(self._rerun_estimate)   # (the held estimate rerun, 13 November 2026)
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
        v.addWidget(self.view.view_panel())         # the view's options, as the Scenario Editor's (pass V2)
        v.addWidget(self._build_global_group())
        v.addWidget(self._build_mission_settings_group())
        scn_box = self._build_scenario_group()       # (user, 11 November 2026: hidden, a duplicate of the playback
        scn_box.setVisible(False)                     # bar under the view; its controls still hold the instant,
        v.addWidget(scn_box)                          # Remove is in the File menu, its alerts in the Alerts list)
        v.addWidget(self._build_nav_group())
        v.addWidget(self._build_sector_list_group())
        v.addWidget(self._build_sector_form_group())
        # (user, 11 November 2026: the per-face budget removed, a duplicate of the live load under the view)
        v.addWidget(self._build_alerts_group())
        self._build_actions()
        v.addStretch(1)

        # no horizontal scrolling: the panel is always at least as wide as its content. No maximum: the splitter
        # handle is free to make it as wide as the user wants (bounded only by the 3D view's own minimum width).
        min_w = (inner.minimumSizeHint().width() + panel.verticalScrollBar().sizeHint().width()
                 + 2 * panel.frameWidth())
        scr = QApplication.primaryScreen()
        avail = scr.availableGeometry().width() if scr is not None else 1 << 20
        if min_w + PANEL_VIEW_MIN_W <= avail:
            panel.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            panel.setMinimumWidth(min_w)
        else:                                    # a screen too narrow (e.g. 1920 px): the panel scrolls sideways
            panel.setMinimumWidth(min(min_w, max(avail - PANEL_VIEW_MIN_W, 320)))

        splitter.insertWidget(0, panel)          # (user, 29 October 2026) the panel on the left of the 3D view
        splitter.setChildrenCollapsible(False)
        splitter.setStretchFactor(0, 0)           # the panel at its width, the 3D view takes the rest
        splitter.setStretchFactor(1, 1)

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
        self.sb_eps_min = self._spin(-90, 90, 1, self.mission["global"].get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"]))
        self.sb_eps_max = self._spin(-90, 90, 1, self.mission["global"].get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"]))
        self.sb_elec = self._spin(0.0, md.FACE_FEEDER_MAX_MW, 0.1, self._cap_value())
        self.sb_elec.setSpecialValueText("shared (no cap)")            # step 4.1: faces share the ship total
        self.sb_elec.setToolTip("Optional cap of the mean electrical power of each face (MW). The faces share the "
                                f"ship's {md.SHIP_ELEC_TOTAL_MW:g} MW; 0 = no cap (a list of 4 caps can be given in the "
                                "mission file)")
        form.addRow("Rear limit eps_min (deg)", self.sb_eps_min)
        form.addRow("Front limit eps_max (deg)", self.sb_eps_max)       # (the power cap: Mission settings)
        for sb in (self.sb_eps_min, self.sb_eps_max, self.sb_elec):
            sb.valueChanged.connect(self._on_global_changed)
        return box

    def _build_mission_settings_group(self) -> QGroupBox:
        """Step 5.1: mission settings (interim interface: a full HMI replaces it in the global-simulation phase)."""
        box = QGroupBox("Mission settings")
        self._edit_widgets.append(box)
        form = QFormLayout(box)
        self.sb_p_min = self._spin(0.5, 0.9999, 0.01, MISSION_DEFAULTS["confirm_p_min"])
        self.sb_p_min.setDecimals(4)
        self.sb_p_min.setToolTip("Probability of confirming a true target (a volume may set its own)")
        self.sb_rate = self._spin(0.1, 360.0, 1.0, MISSION_DEFAULTS["ship_rate_max_dps"])
        self.sb_accel = self._spin(0.1, 1000.0, 1.0, MISSION_DEFAULTS["ship_accel_max_dps2"])
        self.sb_rate.setToolTip("Largest angular rate of the ship (re-pointing in flight: migration between faces)")
        self.sb_accel.setToolTip("Largest angular acceleration of the ship (navigation pointing error); the band loss "
                                 "stays sized by the parameter file, a harsher mission is flagged")
        self.cb_realloc = QComboBox()                                   # step 5.2 (P7)
        self.cb_realloc.addItems(margin.MODES)
        self.cb_realloc.setToolTip("Who gets the free margin, and who keeps its specification in saturation: NONE "
                                   "(margin left free), PRIO_SEARCH (shorter search revisits, down to their RR min), "
                                   "PRIO_TRACK (faster track cadence). The verdict stays on the revisits asked")
        self.sb_ceiling = self._spin(0.05, 1.0, 0.05, MISSION_DEFAULTS["load_ceiling"])
        self.sb_ceiling.setDecimals(2)
        self.sb_ceiling.setToolTip("Load ceiling: utilisation of the binding resource never exceeded by the "
                                   "reallocation; above it a safety reserve (alert when a face uses it)")
        self.chk_bt = QCheckBox("Ku burn-through allowed (P3')")
        self.chk_bt.setToolTip("Forbidden by default: the Ku look in a self-protection jammer's axis is only reported. "
                               "Allowed: budgeted (one look per revisit) and scheduled after the track updates")
        form.addRow("Confirmation P min", self.sb_p_min)
        form.addRow("Ship rate max (deg/s)", self.sb_rate)
        form.addRow("Ship acceleration max (deg/s2)", self.sb_accel)
        form.addRow("Reallocation", self.cb_realloc)
        form.addRow("Load ceiling", self.sb_ceiling)
        form.addRow("Power cap / face (MW)", self.sb_elec)   # (built with the scan zone, its handler kept; user, 12 Nov)
        form.addRow("", self.chk_bt)
        for sb in (self.sb_p_min, self.sb_rate, self.sb_accel, self.sb_ceiling):
            sb.valueChanged.connect(self._on_mission_settings_changed)
        self.cb_realloc.currentTextChanged.connect(self._on_mission_settings_changed)
        self.chk_bt.toggled.connect(self._on_mission_settings_changed)
        return box

    def _load_mission_settings(self) -> None:
        g = self.mission["global"]
        for sb in (self.sb_p_min, self.sb_rate, self.sb_accel, self.sb_ceiling, self.cb_realloc, self.chk_bt):
            sb.blockSignals(True)
        self.sb_p_min.setValue(g.get("confirm_p_min", MISSION_DEFAULTS["confirm_p_min"]))
        self.sb_rate.setValue(g.get("ship_rate_max_dps", MISSION_DEFAULTS["ship_rate_max_dps"]))
        self.sb_accel.setValue(g.get("ship_accel_max_dps2", MISSION_DEFAULTS["ship_accel_max_dps2"]))
        self.sb_ceiling.setValue(g.get("load_ceiling", MISSION_DEFAULTS["load_ceiling"]))
        self.cb_realloc.setCurrentText(g.get("realloc_mode", margin.MODE_DEFAULT))
        self.chk_bt.setChecked(bool(g.get("burn_through_allowed", False)))
        for sb in (self.sb_p_min, self.sb_rate, self.sb_accel, self.sb_ceiling, self.cb_realloc, self.chk_bt):
            sb.blockSignals(False)

    def _on_mission_settings_changed(self, *_args) -> None:
        g = self.mission["global"]
        for key, sb in (("confirm_p_min", self.sb_p_min), ("ship_rate_max_dps", self.sb_rate),
                        ("ship_accel_max_dps2", self.sb_accel), ("load_ceiling", self.sb_ceiling)):
            if abs(sb.value() - MISSION_DEFAULTS[key]) > 1e-12:
                g[key] = sb.value()
            else:
                g.pop(key, None)
        if self.cb_realloc.currentText() != margin.MODE_DEFAULT:
            g["realloc_mode"] = self.cb_realloc.currentText()
        else:
            g.pop("realloc_mode", None)
        if self.chk_bt.isChecked():
            g["burn_through_allowed"] = True
        else:
            g.pop("burn_through_allowed", None)
        self._schedule_refresh()

    def _build_nav_group(self) -> QGroupBox:
        """Navigation of the mission (step 3.0, core/frames.py), read only (user, 12 November 2026: the navigation
        comes from the flight plan, the Scenario Editor's; no hand-typed navigation any more): the mission's instant
        of the linked scenario, or the navigation of the mission file (test missions), or none."""
        box = QGroupBox("Navigation (attitude and orbit)")
        self._edit_widgets.append(box)
        outer = QVBoxLayout(box)
        outer.setContentsMargins(6, 6, 6, 6)
        self.lbl_nav_summary = QLabel("")
        self.lbl_nav_summary.setWordWrap(True)
        self.lbl_nav_summary.setTextFormat(Qt.RichText)
        outer.addWidget(self.lbl_nav_summary)
        self.lbl_nav = QLabel("")
        self.lbl_nav.setWordWrap(True)
        self.lbl_nav.setStyleSheet("color: gray; font-size: 11px;")
        outer.addWidget(self.lbl_nav)
        self._load_nav_form()
        self._show_link()
        return box

    # ------------------------------------------------------------------ a scenario as a use case (pass S1)
    def _build_scenario_group(self) -> QGroupBox:
        """A scenario computed by the Scenario Editor (its run, gui/scenario_link.py) gives the navigation and the
        sky of its ship at an instant t (user, 10 November 2026); the navigation is then read only (Unlink: kept,
        editable again)."""
        box = QGroupBox("Flight plan (linked scenario)")       # U1: the mission's flight plan, not another file
        self._edit_widgets.append(box)
        self._link: Optional[slink.ScenarioLink] = None
        self._link_problem = ""                        # a link of the mission file that could not be opened
        form = QFormLayout(box)
        self.lbl_scn = QLabel()
        self.lbl_scn.setWordWrap(True)
        self.btn_scn_load = QPushButton("From a scenario...")
        self.btn_scn_load.setToolTip("A scenario of the Scenario Editor: its run gives the navigation and the sky of "
                                     "the ship at the instant chosen (its run computed first if needed)")
        self.btn_scn_load.clicked.connect(lambda _c=False: self.load_scenario())
        self.btn_scn_unlink = QPushButton("Remove")
        self.btn_scn_unlink.setToolTip("Forget the scenario: the navigation and the sky of the instant shown are "
                                       "kept, editable again")
        self.btn_scn_unlink.clicked.connect(lambda _c=False: self.unlink_scenario())
        buttons = QHBoxLayout()
        buttons.addWidget(self.btn_scn_load)
        buttons.addWidget(self.btn_scn_unlink)
        buttons.addStretch(1)
        self.sl_scn_t = QSlider(Qt.Horizontal)        # in tenths of a second
        self.sl_scn_t.setRange(0, 0)
        self.sb_scn_t = DecimalSpinBox()
        self.sb_scn_t.setDecimals(1)
        self.sb_scn_t.setRange(0.0, 0.0)
        self.sb_scn_t.setSuffix(" s")
        self.sl_scn_t.valueChanged.connect(lambda v: self.sb_scn_t.setValue(v / 10.0))
        self.sb_scn_t.valueChanged.connect(self._on_scn_t)
        t_row = QHBoxLayout()
        t_row.addWidget(self.sl_scn_t, 1)
        t_row.addWidget(self.sb_scn_t)
        self.lbl_scn_utc = QLabel("")
        self.lbl_scn_alerts = QLabel("")
        self.lbl_scn_alerts.setWordWrap(True)
        self.lbl_scn_alerts.setStyleSheet(f"color: {ALERT_COLOR}; font-size: 11px;")
        self._scn_timer = QTimer(self)                # the navigation follows the instant (debounced)
        self._scn_timer.setSingleShot(True)
        self._scn_timer.setInterval(150)
        self._scn_timer.timeout.connect(self._apply_scenario_t)
        form.addRow(self.lbl_scn)
        form.addRow(buttons)
        form.addRow("t (from the epoch)", t_row)
        form.addRow("UTC", self.lbl_scn_utc)
        form.addRow(self.lbl_scn_alerts)
        return box

    def _show_link(self) -> None:
        """The scenario's state; the navigation read only while linked."""
        if not hasattr(self, "lbl_nav_summary"):      # (the navigation group not built yet)
            return
        link = self._link
        if link is not None:
            src = (self.mission.get(slink.FLIGHT_PLAN_KEY) or {}).get("source")
            text = (f"<b>{link.name}</b>" + (f" (copied from {os.path.basename(str(src))})" if src else "")
                    + f", {link.duration:g} s: the navigation and the sky below are the ship's at t.")
        elif self._link_problem:
            text = f"Scenario not available: {self._link_problem}. The navigation below is the one saved."
        else:
            text = "None: the navigation below is typed by hand."
        self.lbl_scn.setText(text)
        for w in (self.sl_scn_t, self.sb_scn_t):
            w.setEnabled(link is not None)
        self.btn_scn_unlink.setEnabled(link is not None or bool(self._link_problem))
        self._nav_summary()
        if link is None:
            self.lbl_scn_utc.setText("")
            self.lbl_scn_alerts.setText("")

    def _set_scn_t(self, t: float) -> None:
        """The instant's controls at t, over the run (no navigation update)."""
        dur = self._link.duration if self._link is not None else 0.0
        for w in (self.sl_scn_t, self.sb_scn_t):
            w.blockSignals(True)
        self.sl_scn_t.setRange(0, int(round(10 * dur)))
        self.sb_scn_t.setRange(0.0, dur)
        self.sb_scn_t.setValue(min(max(float(t), 0.0), dur))
        self.sl_scn_t.setValue(int(round(10 * self.sb_scn_t.value())))
        for w in (self.sl_scn_t, self.sb_scn_t):
            w.blockSignals(False)
        if self._link is not None:
            self.lbl_scn_utc.setText(self._link.utc(self.sb_scn_t.value()))
            self.lbl_scn_alerts.setText("; ".join(self._link.stale))

    def _set_scn_t_and_apply(self, t: float) -> None:
        """"Use this instant": the mission's instant set to the one shown, its navigation and sky applied."""
        if self._link is None or self.locked:
            return
        self._set_scn_t(t)
        self._apply_scenario_t()

    def _on_scn_t(self, t: float) -> None:
        self.sl_scn_t.blockSignals(True)
        self.sl_scn_t.setValue(int(round(10 * t)))
        self.sl_scn_t.blockSignals(False)
        if self._link is not None:
            self.view.set_scenario(self._link, t)      # the objects follow the slider at once (pass S2)
        self._scn_timer.start()

    def _open_link(self, path: str, ask: bool) -> Optional[slink.ScenarioLink]:
        """The scenario's run, computed first when missing (or recomputed when stale) if the user agrees (ask);
        None when not available (said when ask)."""
        try:
            slink.check_scenario(path)                    # (a mission or an invalid file: said, no run offered)
            if not os.path.exists(slink.journal_path(path)):
                if not ask:
                    self._link_problem = f"no run computed for {os.path.basename(path)}"
                    return None
                if QMessageBox.question(self, "Scenario", f"No run of {os.path.basename(path)} computed yet. "
                                        "Compute it now?", QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.Yes) != QMessageBox.Yes:
                    return None
                if not slink.compute(self, path):
                    return None
            link = slink.load(path)
            if link.stale and ask:
                answer = QMessageBox.question(
                    self, "Scenario", "The run may not match: " + "; ".join(link.stale) + ".\n\nRecompute it now? "
                    "(No: use it as it is, with an alert)", QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                    QMessageBox.Yes)
                if answer == QMessageBox.Cancel:
                    return None
                if answer == QMessageBox.Yes:
                    if not slink.compute(self, path):
                        return None
                    link = slink.load(path)
            return link
        except slink.LinkError as exc:
            if ask:
                QMessageBox.critical(self, "Scenario", str(exc))
            else:
                self._link_problem = str(exc)
            return None

    def store_result(self, mission_used: dict, result: dict) -> None:
        """One file (user, 11 November 2026): the last exact result kept in the mission, with the signature of the
        mission it was computed for (reopened with the mission while it still fits)."""
        link = mission_used.get(slink.LINK_KEY) if isinstance(mission_used.get(slink.LINK_KEY), dict) else {}
        # the mission computed (its flight plan left out: the mission's own copy), so that a result kept after edits is
        # still shown on the volumes it was computed for (user, 13 November 2026: results stale, no lock)
        used = {k: v for k, v in mission_used.items() if k not in slink.DERIVED_KEYS + (slink.FLIGHT_PLAN_KEY,)}
        self.mission[slink.RESULTS_KEY] = {"exact": result, "computed": result.get("_computed_local", ""),
                                          "t_s": link.get("t_s"),
                                          "mission_sig": slink.mission_signature(mission_used),
                                          "mission": copy.deepcopy(used)}
        self._update_title()

    def stored_result(self) -> Optional[dict]:
        """The last exact result kept in the mission ({"exact", "computed", "mission_sig", "mission"}), None if none."""
        r = self.mission.get(slink.RESULTS_KEY)
        return r if isinstance(r, dict) and isinstance(r.get("exact"), dict) else None

    def stored_result_fits(self) -> bool:
        r = self.mission.get(slink.RESULTS_KEY)
        return isinstance(r, dict) and r.get("mission_sig") == slink.mission_signature(self.mission)

    def _open_embedded(self, sc: dict) -> Optional[slink.ScenarioLink]:
        """The flight plan copied in the mission: its run from the cache, computed first when missing (a progress
        dialog, no question: it is the mission's own flight plan)."""
        try:
            path = slink.embedded_path(sc)
            if not os.path.exists(slink.journal_path(path)) and not slink.compute(self, path):
                self._link_problem = "the flight plan's run was not computed"
                return None
            return slink.load(path)
        except (slink.LinkError, OSError) as exc:
            self._link_problem = str(exc)
            return None

    def load_scenario(self, path: Optional[str] = None) -> None:
        """A scenario as the mission's use case: its ship's navigation and sky at t = 0 (then the instant chosen)."""
        if self.locked:
            return
        if path is None:
            start = os.path.dirname(self._link.path) if self._link is not None else slink.SCENARIO_DIR
            path, _ = QFileDialog.getOpenFileName(self, "Load a scenario", start, "Scenario (*.json)")
            if not path:
                return
        link = self._open_link(path, ask=True)
        if link is None:
            return
        from datetime import datetime, timezone
        self.mission[slink.FLIGHT_PLAN_KEY] = {                # one file (user, 11 November 2026): a COPY
            "scenario": link.sc, "source": slink.relative(path, self.path),
            "imported_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
        if not link.stale:
            slink.copy_run(path, slink.embedded_path(link.sc))
        self._link, self._link_problem = link, ""
        self._scenario_dirty = True                   # (written with the mission at the next save)
        self.mission[slink.LINK_KEY] = {"t_s": 0.0}
        self._show_link()
        self._set_scn_t(0.0)
        self._apply_scenario_t()

    def _apply_scenario_t(self) -> None:
        """The mission's navigation and sky: the scenario's ship at t (a change of the mission)."""
        if self._link is None or self.locked:
            return
        t = self.sb_scn_t.value()
        try:
            nav, sky, alerts = self._link.nav_sky(t)
        except Exception as exc:  # noqa: BLE001 -- a bad run: said, the navigation kept
            self.lbl_scn_alerts.setText(f"navigation at t not available: {exc}")
            return
        g = self.mission["global"]
        g["nav"], g["sky"] = [nav], sky
        self.mission[slink.LINK_KEY]["t_s"] = float(t)
        self._load_nav_form()
        self.lbl_scn_utc.setText(self._link.utc(t))
        self.lbl_scn_alerts.setText("; ".join(self._link.stale + alerts))
        self.view.set_scenario(self._link, t)          # its objects at t (pass S2)
        self._schedule_refresh()

    def unlink_scenario(self) -> None:
        """Forget the scenario: the navigation and the sky shown kept, editable again."""
        if self.locked:
            return
        self.mission.pop(slink.LINK_KEY, None)
        self.mission.pop(slink.FLIGHT_PLAN_KEY, None)
        self._scenario_dirty = True
        self._link, self._link_problem = None, ""
        self._show_link()
        self.view.set_scenario(None)
        self._schedule_refresh()

    def _relink(self) -> None:
        """At a mission's opening: its scenario reopened (its run as computed; the navigation kept as saved)."""
        self._link, self._link_problem = None, ""
        self._converted = False
        info = self.mission.get(slink.LINK_KEY)
        fp = self.mission.get(slink.FLIGHT_PLAN_KEY)
        if isinstance(fp, dict) and isinstance(fp.get("scenario"), dict):
            self._link = self._open_embedded(fp["scenario"])
        elif isinstance(info, dict) and info.get("path"):       # the former format: a link to a scenario file
            path = slink.resolve(str(info["path"]), self.path)
            link = self._open_link(path, ask=False)
            if link is not None:
                self._link = link
                self.mission[slink.FLIGHT_PLAN_KEY] = {"scenario": link.sc, "source": str(info["path"])}
                info.pop("path", None)
                if not link.stale:
                    slink.copy_run(path, slink.embedded_path(link.sc))
                self._converted = True                          # (to be saved in the one-file format)
                self._scenario_dirty = True
        self._show_link()
        if self._link is not None:
            self._set_scn_t(float(info.get("t_s", 0.0)))
            # the navigation and the sky of the mission's instant (user, 12 November 2026: a work file stores only
            # t_nav_s, sim/document; they were left out until "Use this instant", and the exact calculation ran
            # without orbit, attitude nor central body)
            self._apply_scenario_t()
        self.view.set_scenario(self._link, self.sb_scn_t.value())

    def _nav_msg(self) -> dict:
        nav = self.mission["global"].get("nav")
        return dict(nav[0]) if isinstance(nav, list) and nav and isinstance(nav[0], dict) else {}

    @staticmethod
    def _ref_axes(ref: str, n: "frames.Nav") -> Optional[np.ndarray]:
        """Matrix reference -> ICRF of the attitude angles (H2, the Scenario Editor's convention, sim/attitude.py):
        orbital (X = -R, Y = N, Z = T; None without an orbit), equatorial (the central body's axes; no body: ICRF)."""
        if ref in ("orbital", "rtn"):
            return att.orbital_reference(n.r, n.v) if n.has_orbit else None
        return frames.equator_axes(n.body)

    @staticmethod
    def _euler_of(e: np.ndarray) -> tuple:
        """(yaw, pitch, roll) deg of a ship -> reference matrix e = sim.attitude.euler(yaw, pitch, roll)."""
        z = e[:, 2]
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(z[1])))))
        yaw = math.degrees(math.atan2(float(z[0]), float(z[2])))
        m = (att.rot_y(math.radians(yaw)) @ att.rot_x(-math.radians(pitch))).T @ e
        roll = -math.degrees(math.atan2(float(m[1, 0]), float(m[0, 0])))
        return yaw, pitch, roll

    def _load_nav_form(self) -> None:
        """The navigation shown (read only) from the mission."""
        self._nav_status()

    @staticmethod
    def _att_ref(msg: dict) -> str:
        """The message's attitude reference in the current convention (the former icrf / rtn read as equatorial /
        orbital; orbital without an orbit: equatorial)."""
        ref = {"icrf": "equatorial", "rtn": "orbital"}.get(msg.get("attitude_ref"), msg.get("attitude_ref"))
        if ref not in frames.ATTITUDE_REFS:
            ref = "orbital" if msg.get("central_body") else "equatorial"
        return "equatorial" if ref == "orbital" and not msg.get("central_body") else ref

    def _nav_status(self) -> None:
        """Operator feedback of the navigation, and the attitude of the 3D view's ICRF option."""
        errs = frames.validate_nav(self.mission["global"].get("nav"))
        if errs:
            self.lbl_nav.setText("; ".join(errs))
            self.lbl_nav.setStyleSheet(f"color: {ALERT_COLOR}; font-size: 11px;")
            self.lbl_nav.setVisible(True)
            self.view.set_navigation(None)
            return
        n = frames.parse_nav(self.mission["global"].get("nav"))
        text = "No navigation: identity attitude, no orbit." if self.mission["global"].get("nav") is None else ""
        if n.has_orbit:
            text = (f"Altitude {np.linalg.norm(n.r) - n.body_radius_km:.1f} km above the {n.body}; "
                    f"limb {n.limb_depression_deg(0.0):.2f} deg below the local horizontal.")
        self.lbl_nav.setText(text)
        self.lbl_nav.setStyleSheet("color: gray; font-size: 11px;")
        self.lbl_nav.setVisible(getattr(self, "_link", None) is None)   # (a flight plan: the summary says it, at t)
        self.view.set_navigation(n.r_si, n.body)
        self._nav_summary()

    def _on_instant_shown(self, t: float, g: dict) -> None:
        """The playback shows the instant t (user, 12 November 2026: the panel stayed at the mission's t = 0 s)."""
        nav = g.get("nav")
        self._shown = (float(t), dict(nav[0]) if isinstance(nav, list) and nav else {})
        self._nav_summary()

    def _nav_summary(self) -> None:
        """The read-only summary of the navigation: with a flight plan, the ship at the instant SHOWN by the playback
        (central body, altitude and speed, attitude angles in their reference) and the mission's instant, the one the
        exact calculation uses; without one, the mission file's navigation."""
        if not hasattr(self, "lbl_nav_summary"):
            return
        link = getattr(self, "_link", None)
        msg = self._nav_msg()
        shown = getattr(self, "_shown", None) if link is not None else None
        t_m = (float((self.mission.get(slink.LINK_KEY) or {}).get("t_s", 0.0))
               if isinstance(self.mission.get(slink.LINK_KEY), dict) else 0.0)
        if shown is not None:
            msg = shown[1]
        if link is None and not msg:
            self.lbl_nav_summary.setText("No flight plan: volumes in the ship frame only. The other frames need a "
                                         "scenario (Scenario Editor, File menu) or a navigation in the mission file.")
            return
        if link is not None:
            t = shown[0] if shown is not None else t_m
            lines = [f"<b>Instant shown t = {t:.1f} s</b> ({link.utc(t)})"]
        else:
            lines = ["<b>Navigation of the mission file</b>" + (f" ({msg['label']})" if msg.get("label") else "")
                     + (f", {msg['epoch']}" if msg.get("epoch") else "")]
        if not msg or frames.validate_nav([msg]):
            lines.append("<span style='color:%s'>no navigation at this instant</span>" % ALERT_COLOR)
        else:
            n = frames.Nav(msg)
            if n.has_orbit:
                lines.append(f"{n.body.capitalize()}: altitude {np.linalg.norm(n.r) - n.body_radius_km:,.0f} km, "
                             f"speed {np.linalg.norm(n.v):.3f} km/s")
            ref = self._att_ref(msg)
            a = self._ref_axes(ref, n)
            if a is not None:
                y, p_, r_ = (round(x, 1) + 0.0 for x in self._euler_of(a.T @ n.r_si))   # (no "-0.0")
                lines.append(f"attitude ({ref}): yaw {y:.1f}, pitch {p_:.1f}, roll {r_:.1f} deg")
        if link is not None:
            same = shown is None or abs(shown[0] - t_m) < 0.05
            lines.append(f"<span style='color:{'gray' if same else ALERT_COLOR}'>Exact calculation at the mission "
                         f"instant t = {t_m:g} s" + ("" if same else ": ‘Use this instant’ to compute "
                                                     "the instant shown") + "</span>")
        self.lbl_nav_summary.setText("<br>".join(lines))

    def _ship_sector(self, sec: dict) -> Optional[dict]:
        """The sector in the ship frame for the 3D view, the gauges and the alerts (None if its frame is invalid)."""
        g = self.mission["global"]
        if frames.validate_sector(sec, g, "") or (sec.get("frame", "ship") != "ship" and frames.validate_nav(g.get("nav"))):
            return None
        try:
            return frames.resolve_sector(sec, g)
        except (KeyError, TypeError, ValueError):
            return None

    def _build_sector_list_group(self) -> QGroupBox:
        box = QGroupBox("Watch sectors")
        v = QVBoxLayout(box)
        self.list_sectors = QListWidget()
        self.list_sectors.setSelectionMode(QAbstractItemView.SingleSelection)
        self.list_sectors.currentRowChanged.connect(self._on_select_sector)
        self.list_sectors.itemChanged.connect(self._on_sector_check_changed)
        self.list_sectors.setToolTip("Tick / untick a sector to show / hide its volume in the 3D view; right-click: "
                                     "duplicate or delete it (Del key)")
        # B14 (user, 29 October 2026): "+" over the list adds a sector; right-click and the Del key on a sector
        add = QMenu(self)
        add.addAction("Watched volume (rect, real space)", lambda: self._add_sector("rect"))
        add.addAction("Cap sector", lambda: self._add_sector("cap"))
        add.addAction("Box sector (phi / eps limits)", lambda: self._add_sector("box"))
        self.btn_add_sector = uc.menu_button("+", add, tip="Add a watch sector", arrow=False)
        self._edit_widgets.append(self.btn_add_sector)
        self.tg_played = uc.Toggle("show effectively played",
                                   tip="Only what the volumes scan: the parts another volume already serves (its need "
                                       "met there: the relief of the fast estimate at the instant shown) taken out, a "
                                       "volume served in full not drawn")
        self.tg_played.toggled.connect(self.view.set_played_only)
        head = QHBoxLayout()
        head.addWidget(self.tg_played)
        head.addStretch(1)
        head.addWidget(self.btn_add_sector)
        v.addLayout(head)
        v.addWidget(self.list_sectors)
        self.list_sectors.setContextMenuPolicy(Qt.CustomContextMenu)
        self.list_sectors.customContextMenuRequested.connect(self._sector_menu)
        delete = QShortcut(QKeySequence.Delete, self.list_sectors, activated=self._delete_sector_unlocked)
        delete.setContext(Qt.WidgetShortcut)
        return box

    def _sector_menu(self, pos) -> None:
        """Right-click on a sector: duplicate or delete it (the mission unlocked)."""
        if self.locked or self.list_sectors.itemAt(pos) is None:
            return
        menu = QMenu(self)
        menu.addAction("Duplicate", self._duplicate_sector)
        menu.addAction("Delete", self._delete_sector)
        menu.exec_(self.list_sectors.viewport().mapToGlobal(pos))

    def _delete_sector_unlocked(self) -> None:
        if not self.locked:
            self._delete_sector()

    def _build_sector_form_group(self) -> QGroupBox:
        box = QGroupBox("Selected sector")
        self._edit_widgets.append(box)
        v = QVBoxLayout(box)
        top = QFormLayout()
        self.ed_id = QLineEdit()  # a text field (user, 12 November 2026: not a drop-down list)
        self.ed_id.textChanged.connect(self._on_id_changed)
        self.cb_shape = QComboBox()
        self.cb_shape.addItems(list(md.SHAPES))
        self.cb_shape.setToolTip("rect: a watched volume in real space, a centre and half-openings measured on the "
                                 "sky (the same shape wherever it lies); cap: a cone about a centre; box: limits in "
                                 "phi and eps of the frame (narrows toward its poles: for the ship frame)")
        self.cb_shape.currentTextChanged.connect(self._on_shape_changed)
        self.cb_frame = QComboBox()
        self.cb_frame.addItems(frames.FRAMES)
        self.cb_frame.setToolTip("Frame of the volume (step 3.0): ship (tied to the ship), inertial (ICRF: phi = RA, "
                                 "eps = Dec), rtn (local orbital), velocity (X = the ship velocity: the direction of travel at "
                                 "phi 0, eps 0 on any orbit), limb (eps counted from the limb of the central "
                                 "body, box only), equatorial (the central body's equator and pole)")
        self.cb_frame.currentTextChanged.connect(self._on_frame_changed)
        top.addRow("id", self.ed_id)
        top.addRow("shape", self.cb_shape)
        top.addRow("frame", self.cb_frame)
        v.addLayout(top)

        self.grp_limb = QGroupBox("Limb")
        fl = QFormLayout(self.grp_limb)
        self.sb_limb_alt = self._spin(0, 100000, 10, frames.LIMB_ALT_KM_DEFAULT)
        self.sb_limb_alt.setToolTip("Grazing altitude of the limb above the central body (km): eps of the box is "
                                    "counted from this limb")
        fl.addRow("limb altitude (km)", self.sb_limb_alt)
        # (user, 13 November 2026) the limb watch: targets appearing above the limb at h_min..h_max; the band, the
        # range and the revisit are then computed at the instant (frames.popup_resolve)
        self.chk_limb_watch = QCheckBox("limb watch (targets appear above the limb)")
        self.chk_limb_watch.setToolTip("The eps band, the required range and the revisit are computed from the target "
                                       "altitudes, the orbit and the reaction delay (a target is detected with the "
                                       "volume's Pd within the delay after it rises above the limb)")
        self.sb_h_min = self._spin(0, 100000, 10, 0.0)
        self.sb_h_max = self._spin(0, 100000, 10, 200.0)
        self.sb_h_min.setToolTip("Lowest target altitude above the central body (km): the limb the band starts from")
        self.sb_h_max.setToolTip("Highest target altitude above the central body (km): the top of the band and the "
                                 "farthest emergence")
        fl.addRow(self.chk_limb_watch)
        fl.addRow("target altitude min (km)", self.sb_h_min)
        fl.addRow("target altitude max (km)", self.sb_h_max)
        v.addWidget(self.grp_limb)

        self.grp_box = QGroupBox("Zone (box)")
        fb = QFormLayout(self.grp_box)
        self.sb_phi_min = self._spin(0, 360, 1, md.SECTOR_DEFAULTS_BOX["phi_min_deg"])
        self.sb_phi_max = self._spin(0, 360, 1, md.SECTOR_DEFAULTS_BOX["phi_max_deg"])
        self.sb_box_eps_min = self._spin(-90, 90, 1, md.SECTOR_DEFAULTS_BOX["eps_min_deg"])
        self.sb_box_eps_max = self._spin(-90, 90, 1, md.SECTOR_DEFAULTS_BOX["eps_max_deg"])
        fb.addRow("phi_min (deg)", self.sb_phi_min)
        fb.addRow("phi_max (deg)", self.sb_phi_max)
        fb.addRow("eps_min (deg)", self.sb_box_eps_min)
        fb.addRow("eps_max (deg)", self.sb_box_eps_max)
        v.addWidget(self.grp_box)

        self.grp_cap = QGroupBox("Zone (cap)")
        fc = QFormLayout(self.grp_cap)
        self.sb_phi0 = self._spin(0, 360, 1, md.SECTOR_DEFAULTS_CAP["phi0_deg"])
        self.sb_eps0 = self._spin(-90, 90, 1, md.SECTOR_DEFAULTS_CAP["eps0_deg"])
        self.sb_radius = self._spin(0.1, 180, 1, md.SECTOR_DEFAULTS_CAP["radius_deg"])
        fc.addRow("phi0 (deg)", self.sb_phi0)
        fc.addRow("eps0 (deg)", self.sb_eps0)
        fc.addRow("radius (deg)", self.sb_radius)
        v.addWidget(self.grp_cap)

        self.grp_rect = QGroupBox("Zone (rect: centre and half-openings)")
        fr_ = QFormLayout(self.grp_rect)
        self.sb_rect = {k: self._spin(lo, hi, 1, md.SECTOR_DEFAULTS_RECT[k]) for k, lo, hi in (
            ("phi0_deg", 0, 360), ("eps0_deg", -90, 90), ("half_width_deg", 0.1, 180),
            ("half_height_deg", 0.1, 90), ("rot_deg", -180, 180))}
        for k, text in (("phi0_deg", "centre phi0 (deg)"), ("eps0_deg", "centre eps0 (deg)"),
                        ("half_width_deg", "half width (deg)"), ("half_height_deg", "half height (deg)"),
                        ("rot_deg", "rotation (deg)")):
            fr_.addRow(text, self.sb_rect[k])
        self.grp_rect.setVisible(False)
        v.addWidget(self.grp_rect)

        common = QFormLayout()
        # step 5.1: target class of the library (its values shown; an edited RCS / Swerling becomes an override)
        self.cb_class = QComboBox()
        self.cb_class.addItem("(none: RCS and Swerling below)", None)
        for name in rm.TARGET_TYPES:
            self.cb_class.addItem(name, name)
        self.cb_class.currentIndexChanged.connect(self._on_class_changed)
        self.lbl_class = QLabel("")
        self.lbl_class.setWordWrap(True)
        self.lbl_class.setStyleSheet("color: gray; font-size: 11px;")
        common.addRow("Target class", self.cb_class)
        common.addRow("", self.lbl_class)
        self.sb_rcs = self._spin(0.01, 1_000_000, 1, md.SECTOR_DEFAULTS_BOX["rcs_m2"])
        self.sb_swerling = QSpinBox()
        self.sb_swerling.setKeyboardTracking(False)
        self.sb_swerling.setRange(0, 4)
        self.sb_swerling.setValue(md.SECTOR_DEFAULTS_BOX["swerling"])
        self.sb_swerling.valueChanged.connect(self._on_field_changed)
        self.sb_range = self._spin(1, 50000, 10, md.SECTOR_DEFAULTS_BOX["range_km"])
        self.sb_rr = self._spin(0.1, 600, 1, md.SECTOR_DEFAULTS_BOX["rr_target_s"])
        self.sb_vmin = self._spin(0, 40000, 100, md.SECTOR_DEFAULTS_BOX["v_close_min_mps"])
        self.sb_vmax = self._spin(0, 40000, 100, md.SECTOR_DEFAULTS_BOX["v_close_max_mps"])
        self.cb_rx_sll = QComboBox()
        for i, sll in enumerate(md.RX_SLL_CHOICES):
            beam = rm.rx_beam("S", sll)
            self.cb_rx_sll.addItem(f"{sll:g}", sll)
            self.cb_rx_sll.setItemData(i, f"RX weighting efficiency {beam.eta:.3f} (RX gain {rm.lin_to_db(beam.eta):+.2f} dB vs "
                                          f"uniform), RX beamwidth {beam.hpbw:.3f} lambda/D", Qt.ToolTipRole)
        self.cb_rx_sll.setToolTip("Peak sidelobe level of the digital RX weighting in this volume (S and Ku): lower "
                                  "rejects strong off-axis echoes / interference better, at the cost of RX gain")
        self.cb_rx_sll.currentIndexChanged.connect(self._on_field_changed)
        self._set_rx_sll(md.SECTOR_DEFAULTS_BOX["rx_sll_db"])
        common.addRow("RCS (m2)", self.sb_rcs)
        common.addRow("Swerling", self.sb_swerling)
        common.addRow("Required range (km)", self.sb_range)
        # step 5.5 (user, 13 November 2026): the shortest range of interest; 0 = the default (range_min_frac x range)
        self.sb_range_min = self._spin(0.0, 50000, 10, 0.0)
        self.sb_range_min.setToolTip(
            "Shortest range of interest of the volume: the blind range of its pulses must stay below it, the false "
            "alarms are counted from it, and the views draw the volume from it. (default) = "
            f"{100 * mo.RANGE_MIN_FRAC:g} % of the required range")
        self._update_range_min_text()
        self.sb_range.valueChanged.connect(lambda _v: self._update_range_min_text())
        self.sb_range_min.valueChanged.connect(self._on_field_changed)
        common.addRow("Shortest range (km)", self.sb_range_min)
        common.addRow("Target RR (s)", self.sb_rr)
        # (user, 13 November 2026) a target that appears (unmasking, the limb watch): N looks at the same range within
        # the reaction delay, the revisit computed
        self.chk_popup = QCheckBox("the target appears (no approach)")
        self.chk_popup.setToolTip("The target appears inside the volume (unmasking) instead of closing in from beyond "
                                  "its range: detected with the volume's Pd within the delay, by N looks at the same "
                                  "range; the revisit is computed (delay / N, at the least load)")
        self.sb_popup_delay = self._spin(0.1, 3600, 5, frames.POPUP_DELAY_S_DEFAULT)
        self.sb_popup_delay.setToolTip("Reaction delay: the target is detected within this time after it appears")
        self.lbl_popup = QLabel("")
        self.lbl_popup.setWordWrap(True)
        self.lbl_popup.setStyleSheet("color: gray; font-size: 11px;")
        common.addRow("", self.chk_popup)
        common.addRow("Detection delay (s)", self.sb_popup_delay)
        common.addRow("", self.lbl_popup)
        common.addRow("Closing speed min (m/s)", self.sb_vmin)
        common.addRow("Closing speed max (m/s)", self.sb_vmax)
        # step 4.3: scheduler priority of the volume
        self.chk_self_prot = QCheckBox("self-protection (P1)")
        self.chk_self_prot.setToolTip("A self-protection volume is searched at priority P1, before confirmations; "
                                      "otherwise it is a search volume of priority P4_<search priority>")
        self.chk_self_prot.toggled.connect(self._on_field_changed)
        self.sb_search_prio = QSpinBox()
        self.sb_search_prio.setKeyboardTracking(False)
        self.sb_search_prio.setRange(1, 99)
        self.sb_search_prio.setToolTip("Sub-priority of a search volume (P4_1 is served first)")
        self.sb_search_prio.valueChanged.connect(self._on_field_changed)
        common.addRow("Priority", self.chk_self_prot)
        common.addRow("Search priority (P4_n)", self.sb_search_prio)
        common.addRow("RX sidelobes (dB)", self.cb_rx_sll)
        # step 5.1: detection settings of the volume (stored only when they differ from the parameter file)
        self.sb_pd = self._spin(0.5, 0.9999, 0.01, rm.DETECT_PD)
        self.sb_pd.setDecimals(4)
        self.sb_pd.setToolTip("Cumulative Pd of the search reached at the required range")
        self.sb_fa = self._spin(0.0001, 1e6, 1.0, mo.FA_PER_RR_DEFAULT)
        self.sb_fa.setDecimals(4)
        self.cb_fa_unit = QComboBox()
        self.cb_fa_unit.addItems(["per revisit", "per second"])
        fa_row = QHBoxLayout()
        fa_row.addWidget(self.sb_fa)
        fa_row.addWidget(self.cb_fa_unit)
        self.sb_sec_pmin = self._spin(0.0, 0.9999, 0.01, 0.0)
        self.sb_sec_pmin.setDecimals(4)
        self.sb_sec_pmin.setSpecialValueText("(mission)")
        self.sb_sec_pmin.setToolTip("Probability of confirming a true target in this volume; (mission) = the "
                                    "mission's value")
        common.addRow("Pd (cumulative)", self.sb_pd)
        common.addRow("False alarms tolerated", fa_row)
        common.addRow("Confirmation P min", self.sb_sec_pmin)
        self.sb_rr_min = self._spin(0.0, 600, 0.1, 0.0)                 # step 5.2 (P4)
        self.sb_rr_min.setDecimals(3)
        self.sb_rr_min.setSpecialValueText("(none)")
        self.sb_rr_min.setToolTip("Shortest revisit wanted when the mission's free margin is reused (Reallocation "
                                  "PRIO_SEARCH); (none) = the default floor tool.global_opt.rr_min_search_s")
        common.addRow("RR min (s)", self.sb_rr_min)
        for w in (self.sb_pd, self.sb_fa, self.sb_sec_pmin, self.sb_rr_min):
            w.valueChanged.connect(self._on_field_changed)
        self.cb_fa_unit.currentIndexChanged.connect(self._on_field_changed)
        v.addLayout(common)
        v.addWidget(self._build_tracking_group())

        for sb in (self.sb_phi_min, self.sb_phi_max, self.sb_box_eps_min, self.sb_box_eps_max,
                   self.sb_phi0, self.sb_eps0, self.sb_radius, self.sb_rcs, self.sb_range,
                   self.sb_rr, self.sb_vmin, self.sb_vmax, self.sb_limb_alt, *self.sb_rect.values(),
                   self.sb_h_min, self.sb_h_max, self.sb_popup_delay):
            sb.valueChanged.connect(self._on_field_changed)
        self.chk_limb_watch.toggled.connect(self._on_field_changed)
        self.chk_popup.toggled.connect(self._on_field_changed)
        self.grp_limb.setVisible(False)

        self.grp_box.setVisible(True)
        self.grp_cap.setVisible(False)
        return box

    def _set_rx_sll(self, sll: float) -> None:
        i = self.cb_rx_sll.findData(float(sll))
        self.cb_rx_sll.setCurrentIndex(i if i >= 0 else self.cb_rx_sll.findData(md.SECTOR_DEFAULTS_BOX["rx_sll_db"]))

    def _build_tracking_group(self) -> QGroupBox:
        """Confirmation mode, new targets per refresh and the active-tracking (AT) range brackets of the sector."""
        self.grp_at = QGroupBox("Confirmation and active tracking (AT)")
        v = QVBoxLayout(self.grp_at)
        form = QFormLayout()
        self.cb_confirm = QComboBox()
        for i, mode in enumerate(md.CONFIRM_MODE_CHOICES):
            self.cb_confirm.addItem(mode)
            self.cb_confirm.setItemData(i, CONFIRM_TIPS[mode], Qt.ToolTipRole)
        self.cb_confirm.setToolTip("Confirmation of every detection (true targets and false alarms)")
        self.cb_confirm.currentTextChanged.connect(self._on_field_changed)
        self.sb_new_targets = self._spin(0, 10000, 1, md.SECTOR_DEFAULTS_BOX["new_targets_per_rr"])
        self.sb_new_targets.setToolTip("True targets newly detected per refresh (confirmed like the false alarms)")
        self.sb_new_targets.valueChanged.connect(self._on_field_changed)
        form.addRow("Confirmation mode", self.cb_confirm)
        form.addRow("New targets / RR", self.sb_new_targets)
        v.addLayout(form)

        self.table_at = QTableWidget(0, len(AT_COLUMNS))
        self.table_at.setHorizontalHeaderLabels([c[1] for c in AT_COLUMNS])
        for c, (_f, _h, tip) in enumerate(AT_COLUMNS):
            self.table_at.horizontalHeaderItem(c).setToolTip(tip)
        self.table_at.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_at.verticalHeader().setVisible(False)
        self.table_at.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_at.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table_at.setMaximumHeight(160)
        self.table_at.itemChanged.connect(self._on_at_item_changed)
        v.addWidget(self.table_at)
        row = QHBoxLayout()
        self.btn_at_add = QPushButton("+ Range bracket")
        self.btn_at_del = QPushButton("Remove bracket")
        self.btn_at_add.clicked.connect(self._add_bracket)
        self.btn_at_del.clicked.connect(self._remove_bracket)
        row.addWidget(self.btn_at_add)
        row.addWidget(self.btn_at_del)
        v.addLayout(row)
        note = QLabel("Zone = full width of the angular uncertainty zone at +/-2 sigma; sR and sV are 1 sigma. "
                      "Brackets stay within the required range (targets are tracked once detected).")
        note.setWordWrap(True)
        note.setStyleSheet("color: gray; font-size: 11px;")
        v.addWidget(note)
        return self.grp_at

    def _build_alerts_group(self) -> QGroupBox:
        box = QGroupBox("Alerts")
        v = QVBoxLayout(box)
        self.list_alerts = QListWidget()
        self.list_alerts.setMaximumHeight(140)
        self.list_alerts.itemDoubleClicked.connect(self._on_alert_activated)
        v.addWidget(self.list_alerts)
        return box

    def _build_actions(self) -> None:
        """B14 (user, 29 October 2026): "Calculate", the mode's main action, in the window's header (it validates
        first: no separate "Validate"); import and export in the window's menu (file_commands)."""
        # U2 (user, 11 November 2026): ONE main action, two explicit kinds of calculation; the button runs the last
        # one chosen, its arrow chooses
        self.b_run = uc.tool_button("Calculate", self._on_calculate, "SP_MediaPlay")
        self._calc_menu = QMenu(self.b_run)
        self._act_exact = self._calc_menu.addAction("Exact, at the mission's instant",
                                                    lambda: self._set_calc_kind("exact", run=True))
        self._act_profile = self._calc_menu.addAction("Along the flight plan (profile)",
                                                      lambda: self._set_calc_kind("profile", run=True))
        self.b_run.setMenu(self._calc_menu)
        self.b_run.setPopupMode(QToolButton.MenuButtonPopup)
        self._calc_kind = "exact"
        self._set_calc_kind("exact")
        self._edit_widgets.append(self.b_run)

    def _update_range_min_text(self) -> None:
        """The empty shortest range shows the default it stands for (range_min_frac x the required range)."""
        self.sb_range_min.setSpecialValueText(f"(default: {mo.RANGE_MIN_FRAC * self.sb_range.value():.0f} km)")

    @staticmethod
    def _spin(lo: float, hi: float, step: float, value: float) -> QDoubleSpinBox:
        sb = DecimalSpinBox()
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
        f = self._shown_finding(sec)
        if f is None:
            item.setText(self._sector_label(sec))
            item.setForeground(QColor())
            item.setToolTip("")
            return
        # always the share, never a bare "!" (user, 11 November 2026: read as a missing figure); the red says the alert
        item.setText(f"{self._sector_label(sec)}   {self._cover_text(str(sec.get('id')), f)}")
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

    def _select_sector_by_sid(self, sid: str) -> None:
        """Select in the list the volume a line of the diagnosis names (5.5 A)."""
        for i, sec in enumerate(self.mission.get("sectors", [])):
            if self._draw_sid(i, sec) == sid:
                self.list_sectors.setCurrentRow(i)
                return

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
            for w in (self.ed_id, self.cb_shape, self.cb_frame, self.grp_limb, self.sb_rcs,
                      self.sb_swerling, self.sb_range,
                      self.sb_rr, self.sb_vmin, self.sb_vmax, self.cb_rx_sll, self.grp_box, self.grp_cap, self.grp_rect,
                      self.grp_at):
                w.setEnabled(enabled)
            self._fill_at_table(sec)
            if not sec:
                return
            self.ed_id.setText(sec.get("id", ""))
            shape = sec.get("shape", "box")
            self.cb_shape.setCurrentText(shape)
            self.grp_box.setVisible(shape == "box")
            self.grp_cap.setVisible(shape == "cap")
            self.grp_rect.setVisible(shape == "rect")
            for k, sb in self.sb_rect.items():
                sb.setValue(sec.get(k, md.SECTOR_DEFAULTS_RECT[k]))
            fr = sec.get("frame", "ship")
            self.cb_frame.setCurrentText(fr)
            self.grp_limb.setVisible(fr == "limb")
            self.sb_limb_alt.setValue(sec.get("limb_alt_km", frames.LIMB_ALT_KM_DEFAULT))
            self.chk_limb_watch.setChecked(bool(sec.get("limb_watch")))
            self.sb_h_min.setValue(float(sec.get("limb_h_min_km", 0.0)))
            self.sb_h_max.setValue(float(sec.get("limb_h_max_km", 200.0)))
            self.chk_popup.setChecked(bool(sec.get("popup")))
            self.sb_popup_delay.setValue(float(sec.get("popup_delay_s", frames.POPUP_DELAY_S_DEFAULT)))
            self.sb_phi_min.setValue(sec.get("phi_min_deg", md.SECTOR_DEFAULTS_BOX["phi_min_deg"]))
            self.sb_phi_max.setValue(sec.get("phi_max_deg", md.SECTOR_DEFAULTS_BOX["phi_max_deg"]))
            self.sb_box_eps_min.setValue(sec.get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"]))
            self.sb_box_eps_max.setValue(sec.get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"]))
            self.sb_phi0.setValue(sec.get("phi0_deg", md.SECTOR_DEFAULTS_CAP["phi0_deg"]))
            self.sb_eps0.setValue(sec.get("eps0_deg", md.SECTOR_DEFAULTS_CAP["eps0_deg"]))
            self.sb_radius.setValue(sec.get("radius_deg", md.SECTOR_DEFAULTS_CAP["radius_deg"]))
            # a sector naming a target class (step 2.4) shows the class values; the class itself is chosen at 5.1
            cls = rm.TARGETS.get(sec.get("target_type"))
            rcs_shown = sec.get("rcs_m2", cls.rcs_m2 if cls else md.SECTOR_DEFAULTS_BOX["rcs_m2"])
            sw_shown = sec.get("swerling", cls.swerling_case(2.8) if cls else md.SECTOR_DEFAULTS_BOX["swerling"])
            self.sb_rcs.setValue(rcs_shown)
            self.sb_swerling.setValue(int(sw_shown))
            self._target_shown = (self.sb_rcs.value(), self.sb_swerling.value())
            self.sb_range.setValue(sec.get("range_km", md.SECTOR_DEFAULTS_BOX["range_km"]))
            self.sb_range_min.setValue(float(sec.get("range_min_km", 0.0) or 0.0))
            self.sb_rr.setValue(sec.get("rr_target_s", md.SECTOR_DEFAULTS_BOX["rr_target_s"]))
            self.chk_self_prot.setChecked(bool(sec.get("self_protection", False)))
            self.sb_search_prio.setValue(int(sec.get("search_priority", 1)))
            self.sb_vmin.setValue(sec.get("v_close_min_mps", md.SECTOR_DEFAULTS_BOX["v_close_min_mps"]))
            self.sb_vmax.setValue(sec.get("v_close_max_mps", md.SECTOR_DEFAULTS_BOX["v_close_max_mps"]))
            self._set_rx_sll(sec.get("rx_sll_db", md.SECTOR_DEFAULTS_BOX["rx_sll_db"]))
            self.cb_confirm.setCurrentText(sec.get("confirm_mode", "auto"))
            self.sb_new_targets.setValue(sec.get("new_targets_per_rr", md.SECTOR_DEFAULTS_BOX["new_targets_per_rr"]))
            # step 5.1
            i = self.cb_class.findData(sec.get("target_type"))
            self.cb_class.setCurrentIndex(i if i >= 0 else 0)
            self._show_class(sec)
            self.sb_pd.setValue(sec.get("pd", rm.DETECT_PD))
            per_s = sec.get("fa_rate_per_s") is not None
            self.cb_fa_unit.setCurrentIndex(1 if per_s else 0)
            self.sb_fa.setValue(sec["fa_rate_per_s"] if per_s else sec.get("fa_per_rr", mo.FA_PER_RR_DEFAULT))
            self.sb_sec_pmin.setValue(sec.get("confirm_p_min", 0.0))
            self.sb_rr_min.setValue(sec.get("rr_min_s", 0.0))
        finally:
            self._updating_form = False
        self._update_popup_info(sec)

    def _update_popup_info(self, sec: Optional[dict]) -> None:
        """(user, 13 November 2026) a target that appears: the values computed at the mission instant (frames.
        popup_resolve) shown under the delay; the inputs they replace (revisit; for the limb watch the range, the eps
        band and the limb altitude) greyed."""
        lw = bool(sec and sec.get("frame") == "limb" and sec.get("limb_watch"))
        pop = bool(sec and (sec.get("popup") or lw))
        self.sb_h_min.setEnabled(lw)
        self.sb_h_max.setEnabled(lw)
        self.sb_popup_delay.setEnabled(pop)
        self.sb_rr.setEnabled(sec is not None and not pop)
        self.sb_range.setEnabled(sec is not None and not lw)
        self.sb_limb_alt.setEnabled(sec is not None and not lw)
        for sb in (self.sb_box_eps_min, self.sb_box_eps_max):
            sb.setEnabled(sec is not None and not lw)
        if not pop:
            self.lbl_popup.setText("")
            return
        try:
            nav = frames.parse_nav(self.mission.get("global", {}).get("nav")) if lw else None
            r = frames.popup_resolve(sec, nav)
        except (ValueError, KeyError, TypeError, ZeroDivisionError) as exc:
            self.lbl_popup.setText(f"not computable: {exc}")
            return
        txt = (f"computed: {r['_popup_n']} look(s) at the same range within {r['_popup_t_eff_s']:.0f} s "
               f"-> revisit {r['rr_target_s']:.1f} s")
        if lw:
            txt += (f"; range {r['range_km']:.0f} km (farthest emergence); band 0 -> {r['eps_max_deg']:.2f} deg above "
                    f"the limb; {r['_popup_dwell_s']:.0f} s in the band at most (at the mission instant)")
        self.lbl_popup.setText(txt)

    # ------------------------------------------------------------------ AT range brackets
    def _fill_at_table(self, sec: Optional[dict]) -> None:
        brackets = sec.get("tracking", []) if sec else []
        self.table_at.blockSignals(True)
        self.table_at.setRowCount(len(brackets))
        for r, b in enumerate(brackets):
            for c, (f, _h, _t) in enumerate(AT_COLUMNS):
                item = QTableWidgetItem(f"{b.get(f, AT_DEFAULTS[f]):g}")
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.table_at.setItem(r, c, item)
        self.table_at.blockSignals(False)
        self.btn_at_del.setEnabled(bool(brackets))

    def _on_at_item_changed(self, item: QTableWidgetItem) -> None:
        if self._updating_form or self._cur_index < 0:
            return
        sec = self.mission["sectors"][self._cur_index]
        b = sec["tracking"][item.row()]
        field = AT_COLUMNS[item.column()][0]
        try:
            b[field] = float(item.text().replace(",", "."))
        except ValueError:
            pass   # not a number: the cell goes back to the stored value
        if field in ("pd_update", "p_discrimination", "rr_min_s") and b.get(field) == AT_DEFAULTS[field]:
            b.pop(field)          # step 5.1: an optional bracket setting at the file's value is not stored
        elif field == "rr_min_s" and b.get(field, 0.0) < 0:
            b.pop(field)
        self.table_at.blockSignals(True)
        item.setText(f"{b.get(field, AT_DEFAULTS[field]):g}")
        self.table_at.blockSignals(False)
        self._schedule_refresh()

    def _add_bracket(self) -> None:
        if self._cur_index < 0:
            return
        sec = self.mission["sectors"][self._cur_index]
        new = md.new_bracket(sec)
        brackets = sec.setdefault("tracking", [])
        brackets.append(new)
        brackets.sort(key=lambda b: b["r_min_km"])
        self._fill_at_table(sec)
        self.table_at.selectRow(next(i for i, b in enumerate(brackets) if b is new))
        self._schedule_refresh()

    def _remove_bracket(self) -> None:
        if self._cur_index < 0:
            return
        sec = self.mission["sectors"][self._cur_index]
        brackets = sec.get("tracking", [])
        row = self.table_at.currentRow()
        if not brackets:
            return
        brackets.pop(row if 0 <= row < len(brackets) else len(brackets) - 1)
        self._fill_at_table(sec)
        self._schedule_refresh()

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
        self.grp_rect.setVisible(shape == "rect")
        self._sync_form_to_model()

    def _on_frame_changed(self, fr: str) -> None:
        self.grp_limb.setVisible(fr == "limb")
        if self._updating_form or self._cur_index < 0:
            return
        self._sync_form_to_model()

    def _on_class_changed(self, *_args) -> None:
        """Step 5.1: a new class drops the RCS / Swerling overrides and shows the class values."""
        if self._updating_form or self._cur_index < 0:
            return
        sec = self.mission["sectors"][self._cur_index]
        name = self.cb_class.currentData()
        if name is None:
            sec.pop("target_type", None)
        else:
            sec["target_type"] = name
            sec.pop("rcs_m2", None)
            sec.pop("swerling", None)
        self._load_form(sec)
        self._sync_form_to_model()

    def _show_class(self, sec: dict) -> None:
        """The class values of a sector, and its overrides, under the class choice."""
        name = sec.get("target_type")
        if name is None or name not in rm.TARGETS:
            self.lbl_class.setText("")
            return
        t = rm.target_for_sector(sec)
        over = [k for k in rm.SECTOR_TARGET_KEYS if k != "target_type" and k in sec]
        self.lbl_class.setText(f"{name} ({t.policy()} presentation): RCS S {t.rcs_at(2.8):.3g} m2 / Ku "
                               f"{t.rcs_at(14.0):.3g} m2, {t.fluctuation(2.8)}, accel {t.accel_max_mps2:g} m/s2"
                               + (f" | overrides: {', '.join(over)}" if over else ""))

    def _on_field_changed(self, *_args) -> None:
        if self._updating_form or self._cur_index < 0:
            return
        self._sync_form_to_model()

    def _sync_form_to_model(self) -> None:
        sec = self.mission["sectors"][self._cur_index]
        sec["shape"] = self.cb_shape.currentText()
        sec["frame"] = self.cb_frame.currentText()
        if sec["frame"] == "limb":
            sec["limb_alt_km"] = self.sb_limb_alt.value()
        else:
            sec.pop("limb_alt_km", None)
        lw = sec["frame"] == "limb" and self.chk_limb_watch.isChecked()     # (13 November 2026: the limb watch)
        if lw:
            sec.update(limb_watch=True, limb_h_min_km=self.sb_h_min.value(), limb_h_max_km=self.sb_h_max.value())
        else:
            for k in ("limb_watch", "limb_h_min_km", "limb_h_max_km"):
                sec.pop(k, None)
        if self.chk_popup.isChecked():
            sec["popup"] = True
        else:
            sec.pop("popup", None)
        if (lw or self.chk_popup.isChecked()) and abs(self.sb_popup_delay.value() - frames.POPUP_DELAY_S_DEFAULT) > 1e-9:
            sec["popup_delay_s"] = self.sb_popup_delay.value()
        else:
            sec.pop("popup_delay_s", None)
        rect_keys = tuple(self.sb_rect)
        if sec["shape"] == "box":
            sec["phi_min_deg"] = self.sb_phi_min.value()
            sec["phi_max_deg"] = self.sb_phi_max.value()
            sec["eps_min_deg"] = self.sb_box_eps_min.value()
            sec["eps_max_deg"] = self.sb_box_eps_max.value()
            for f in ("phi0_deg", "eps0_deg", "radius_deg") + rect_keys:
                sec.pop(f, None)
        elif sec["shape"] == "rect":
            for k, sb in self.sb_rect.items():
                sec[k] = sb.value()
            for f in ("phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg", "radius_deg"):
                sec.pop(f, None)
        else:
            sec["phi0_deg"] = self.sb_phi0.value()
            sec["eps0_deg"] = self.sb_eps0.value()
            sec["radius_deg"] = self.sb_radius.value()
            for f in ("phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg", "half_width_deg",
                      "half_height_deg", "rot_deg"):
                sec.pop(f, None)
        # a class sector keeps its class values until the operator edits them (they then become sector overrides)
        if sec.get("target_type") is None or self.sb_rcs.value() != getattr(self, "_target_shown", (None, None))[0]:
            sec["rcs_m2"] = self.sb_rcs.value()
        if sec.get("target_type") is None or self.sb_swerling.value() != getattr(self, "_target_shown", (None, None))[1]:
            sec["swerling"] = self.sb_swerling.value()
        sec["range_km"] = self.sb_range.value()
        if self.sb_range_min.value() > 0.0:                  # (0: the default, range_min_frac x range_km)
            sec["range_min_km"] = self.sb_range_min.value()
        else:
            sec.pop("range_min_km", None)
        sec["rr_target_s"] = self.sb_rr.value()
        sec["self_protection"] = self.chk_self_prot.isChecked()          # step 4.3
        sec["search_priority"] = self.sb_search_prio.value()
        sec["v_close_min_mps"] = self.sb_vmin.value()
        sec["v_close_max_mps"] = self.sb_vmax.value()
        sec["rx_sll_db"] = float(self.cb_rx_sll.currentData())
        sec["confirm_mode"] = self.cb_confirm.currentText()
        sec["new_targets_per_rr"] = self.sb_new_targets.value()
        # step 5.1: detection settings, kept in the mission only when they differ from the parameter file
        if abs(self.sb_pd.value() - rm.DETECT_PD) > 1e-12:
            sec["pd"] = self.sb_pd.value()
        else:
            sec.pop("pd", None)
        sec.pop("fa_per_rr", None)
        sec.pop("fa_rate_per_s", None)
        if self.cb_fa_unit.currentIndex() == 1:
            sec["fa_rate_per_s"] = self.sb_fa.value()
        elif abs(self.sb_fa.value() - mo.FA_PER_RR_DEFAULT) > 1e-12:
            sec["fa_per_rr"] = self.sb_fa.value()
        if self.sb_sec_pmin.value() > 0.0:
            sec["confirm_p_min"] = self.sb_sec_pmin.value()
        else:
            sec.pop("confirm_p_min", None)
        if self.sb_rr_min.value() > 0.0:
            sec["rr_min_s"] = self.sb_rr_min.value()
        else:
            sec.pop("rr_min_s", None)
        self._show_class(sec)
        self._update_popup_info(sec)
        self._decorate_item(self.list_sectors.item(self._cur_index), sec)
        self._schedule_refresh()

    def _on_global_changed(self, *_args) -> None:
        self.mission["global"]["eps_min_deg"] = self.sb_eps_min.value()
        self.mission["global"]["eps_max_deg"] = self.sb_eps_max.value()
        if not isinstance(self.mission["global"].get("face_elec_mw"), list) or self.sb_elec.value() > 0:
            self.mission["global"]["face_elec_mw"] = self.sb_elec.value()     # a list of 4 caps is kept untouched
        self._schedule_refresh()

    def _cap_value(self) -> float:
        """The cap shown by the power spin box: the common cap, the largest of a list of 4, 0 = none (step 4.1)."""
        v = self.mission["global"].get("face_elec_mw", md.GLOBAL_DEFAULTS["face_elec_mw"]) or 0.0
        return float(max(v)) if isinstance(v, list) else float(v)

    # ------------------------------------------------------------------ refresh (debounced)
    def _schedule_refresh(self) -> None:
        self._debounce.start()

    def _volumes_sig(self) -> str:
        """The volumes as the estimate reads them, in a canonical form: the form writes a volume's defaults on its
        first edit (search_priority 1, self_protection false, fa_per_rr at its default left out), numbers as floats
        (an edit undone gives the volumes estimated back)."""
        defaults = {"search_priority": 1, "self_protection": False, "fa_per_rr": mo.FA_PER_RR_DEFAULT}

        def canon(v):
            if isinstance(v, bool) or v is None:
                return v
            if isinstance(v, (int, float)):
                return float(v)
            if isinstance(v, dict):
                return {k: canon(x) for k, x in v.items()}
            if isinstance(v, (list, tuple)):
                return [canon(x) for x in v]
            return v
        secs = []
        for s in self.mission.get("sectors", []):
            c = {}
            for k, v in s.items():
                if v is None or (k in defaults and canon(v) == canon(defaults[k])):
                    continue
                c[k] = canon(v)
            secs.append(c)
        return json.dumps(secs, sort_keys=True, default=str)

    def _hold_estimate(self) -> None:
        """(user, 13 November 2026) an edit of the volumes holds the fast estimate (each edit, even a small one,
        started a whole computation): \"OBSOLETE ESTIMATE - RERUN\" on the badge until it is clicked; back to the
        volumes last estimated (an edit undone): current again. A mission opened is estimated at once."""
        from core import cost_estimate as ce
        sig = self._volumes_sig()
        if getattr(self, "_est_sig", None) is None:
            self._est_sig = sig
        ce.hold(sig != self._est_sig)

    def _rerun_estimate(self) -> None:
        from core import cost_estimate as ce
        self._est_sig = self._volumes_sig()
        ce.hold(False)
        self._schedule_refresh()
        if hasattr(self.view, "refresh_estimate"):
            self.view.refresh_estimate()

    def _refresh_begin(self) -> None:
        """An estimated recalculation (user, 11 November 2026: the window froze before the badge was drawn): the badge
        says 'updating' and the wait cursor shows, painted; the work starts at the next turn of the event loop."""
        if hasattr(self.view, "_badge_busy"):
            self.view._badge_busy()
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QTimer.singleShot(30, self._refresh_end)

    def _refresh_end(self) -> None:
        try:
            self._refresh_all()
        finally:
            QApplication.restoreOverrideCursor()

    def _refresh_all(self) -> None:
        self._hold_estimate()
        self._update_title()
        if hasattr(self, "_calc_kind"):
            self._set_calc_kind(self._calc_kind)      # U2: the button says the instant the exact calculation uses
        self.view.set_mission_volumes(self.mission, self._hidden_sector_ids())
        eps_min = self.mission["global"].get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"])
        eps_max = self.mission["global"].get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"])
        self._update_gauges(eps_min, eps_max)
        self._schedule_redundancy_check()
        self._update_alerts(eps_min, eps_max)
        self._update_scene(eps_min, eps_max)

    def _instant(self, t: float) -> tuple:
        """The playback's display at t (user, 10 November 2026): the sectors resolved with the linked scenario's
        navigation at t, and that global (nav and sky at t); the mission unchanged."""
        nav, sky, _alerts = self._link.nav_sky(t)
        g = dict(self.mission["global"], nav=[nav], sky=sky)
        draws = []
        for i, sec in enumerate(self.mission["sectors"]):
            if frames.validate_sector(sec, g, ""):
                continue
            try:
                ship = frames.resolve_sector(sec, g)
            except (KeyError, TypeError, ValueError):
                continue
            draws.append(SectorDraw(self._draw_sid(i, sec), ship, geo.sector_color(i), self._sector_text(sec)))
        return draws, g

    def _sector_text(self, sec: dict) -> str:
        return (f"{sec.get('id', '?')}\nR={sec.get('range_km', 0):.0f} km  RCS={sec.get('rcs_m2', 0):g} m2\n"
                f"RR={sec.get('rr_target_s', 0):.0f} s  V={sec.get('v_close_min_mps', 0):.0f}"
                f"-{sec.get('v_close_max_mps', 0):.0f} m/s")

    def _update_scene(self, eps_min: float, eps_max: float) -> None:
        self.view.set_mission_global(self.mission["global"])
        try:                                     # pass R3d (M1): ratios of the last exact run of each volume
            from core import calibration
            secs = self.mission["sectors"]
            self.view.set_calibration(calibration.view_calibration(secs, [self._draw_sid(i, s)
                                                                          for i, s in enumerate(secs)]))
        except Exception:  # noqa: BLE001 -- a volume mid-edit: not recalibrated
            self.view.set_calibration({})
        draws = []
        for i, sec in enumerate(self.mission["sectors"]):
            sid = self._draw_sid(i, sec)
            text = (f"{sid}\nR={sec.get('range_km', 0):.0f} km  RCS={sec.get('rcs_m2', 0):g} m2\n"
                    f"RR={sec.get('rr_target_s', 0):.0f} s  V={sec.get('v_close_min_mps', 0):.0f}"
                    f"-{sec.get('v_close_max_mps', 0):.0f} m/s")
            ship = self._ship_sector(sec)
            if ship is not None:
                draws.append(SectorDraw(sid, ship, geo.sector_color(i), text))
        self.view.set_scene(eps_min, eps_max, draws, hidden=self._hidden_sids())

    def _profiles_ready(self, gen: int, eps: tuple) -> None:
        """A cost profile computed in the background: the range alerts again (user, 11 November 2026: they kept
        saying 'being computed' until the next edit)."""
        from core import cost_profile
        if cost_profile.generation() != gen:
            self._update_gauges(*eps)
            self._update_alerts(*eps)
        elif cost_profile.pending():
            QTimer.singleShot(700, lambda: self._profiles_ready(gen, eps))

    def _update_gauges(self, eps_min: float, eps_max: float) -> None:
        """The volumes out of reach for the alerts (fast estimate; the per-face budget it also filled was removed,
        user 11 November 2026: the live load shows the faces' loads)."""
        totals = {k: {"energy": 0.0, "time_min": 0.0, "kw": 0.0, "at_kw": 0.0, "at_ku_time": 0.0} for k in range(4)}
        p_avg_max = rm.face_elec_avail_w(rm.FACE_ELEC_POWER_MW)       # the optimiser's reference (step 4.1)
        self._unreachable = {}
        self._at_unreachable = {}
        try:                                                          # the sky in the ship frame: the ground rule
            sky_ship = frames.resolve(self.mission)[0].get("global", {}).get("sky")
        except (ValueError, KeyError, TypeError):
            sky_ship = None
        if self.cost_table is not None:
            for sec in self.mission["sectors"]:
                ship = self._ship_sector(sec)
                if ship is None:
                    continue
                try:
                    faces = ce.estimate_sector_faces(self.cost_table, ship, self.grid, eps_min, eps_max, p_avg_max,
                                                     sky=sky_ship)
                except (KeyError, TypeError, ZeroDivisionError):
                    continue
                worst = 0.0
                at_unreach = 0.0
                for k, fb in faces.items():
                    totals[k]["energy"] += fb.energy_fraction
                    totals[k]["time_min"] += fb.time_fraction_min
                    totals[k]["kw"] += fb.avg_power_kw
                    totals[k]["at_kw"] += fb.at_avg_power_kw
                    totals[k]["at_ku_time"] += fb.at_ku_time_fraction
                    worst = max(worst, fb.unreachable_pct)
                    at_unreach += fb.at_unreachable_tracks
                sid = sec.get("id")
                if sid and worst > 0.5:
                    self._unreachable[sid] = (worst, None)   # (the table's: replaced below by the volume's profile)
                if sid and at_unreach > 0.05:
                    self._at_unreachable[sid] = at_unreach
        # the range out of reach from each volume's cost profile (user, 11 November 2026: the table's single-look
        # criterion said S01 at 25 000 km out of reach while the optimiser's laws reach 43 500 km at broadside; the
        # same figures as the live load and the range gauge); the table's kept only while a profile is computed
        if self.cost_table is not None:
            try:
                ships = [{**s_, "id": s_.get("id")} for s_ in (self._ship_sector(x) for x in self.mission["sectors"])
                         if s_ is not None]
                # (the view's calibration: the same inputs as the live load, so that one computation serves both,
                # user 11 November 2026: it was computed twice per edit, ~1.8 s each on a large mission)
                try:
                    from core import calibration
                    secs_ = self.mission["sectors"]
                    calib = calibration.view_calibration(secs_, [self._draw_sid(i, s_) for i, s_ in enumerate(secs_)])
                except Exception:  # noqa: BLE001 -- a volume mid-edit: not recalibrated
                    calib = {}
                alloc = ce.live_allocation_async(self.cost_table, ships, self.grid, eps_min, eps_max, sky_ship,
                                                 self.mission["global"], calib=calib, relief="background",
                                                 tag="prep") or {}
                pending = set(alloc.get("estimating", []))
                self._unreachable = {sid: (None, None) for sid in pending}   # (the table's verdict never shown:
                for sid, o in (alloc.get("out_of_reach") or {}).items():    # single-look, ~x150 too pessimistic)
                    self._unreachable[sid] = (100.0 * o["share"], o["max_km"])
                self._confirm_alerts = {sid: "none" for sid in alloc.get("no_confirmation", [])}
                self._confirm_max = dict(alloc.get("confirm_max_km") or {})
                self._confirm_alerts.update({sid: "depends" for sid in alloc.get("confirmation_depends", [])})
                self._relief = dict(alloc.get("relief") or {})          # A (user, 11 November 2026)
                served = dict(alloc.get("served") or {})
                if served and red.round_served(served) != red.round_served(getattr(self, "_served", {}) or {}):
                    self._served = served                                # B: the findings again with them
                    self._schedule_redundancy_check()
                # the cover, the relief and the check from one pass (user, 12 November 2026), with this estimate
                if alloc.get("redundancy") is not None and not alloc.get("stale") \
                        and not alloc.get("relief_pending"):
                    self._apply_redundancy(alloc)
                if pending or alloc.get("relief_pending") or alloc.get("stale") or not alloc:                                  # the alerts again once the profiles are ready
                    from core import cost_profile
                    gen = cost_profile.generation()
                    QTimer.singleShot(700, lambda g=gen, e=(eps_min, eps_max): self._profiles_ready(g, e))
            except Exception:  # noqa: BLE001 -- a mission mid-edit: the table's alerts kept
                pass

    def _update_alerts(self, eps_min: float, eps_max: float) -> None:
        self.list_alerts.clear()
        sectors = self.mission["sectors"]
        if self.cost_table is not None and not self.cost_table.is_current:
            self.list_alerts.addItem("Fast estimate: core/cost_table.json was built with other radar parameters "
                                     "(radar_system.toml changed) -- rebuild it: python tools/gen_cost_table.py")
        self._add_redundancy_alerts()
        with_finding = {str(s.get("id")) for s in sectors if self._shown_finding(s) is not None}
        for sid, v in sorted((getattr(self, "_relief", {}) or {}).items()):     # A: what the budgets take out
            if sid in with_finding:                  # (said on the finding's line: covered / taken out)
                continue
            self.list_alerts.addItem(f"{sid}: {100 * v['share']:.0f} % of its volume served by {', '.join(v['by'])}"
                                     f" (need met there): taken out of {sid}'s budget")
        if self.lbl_scn_alerts.text():                # (the flight plan's group hidden: its alerts here)
            self.list_alerts.addItem("Flight plan: " + self.lbl_scn_alerts.text())
        for err in frames.validate(self.mission):   # navigation and frames (step 3.0)
            self.list_alerts.addItem(err)
        try:
            for alert in frames.resolve(self.mission)[1]:
                self.list_alerts.addItem(alert)
        except ValueError:
            pass
        for sec in sectors:   # confirmation / AT inputs (also checked by 'Calculate')
            for err in md.validate_tracking(sec, str(sec.get("id", "?"))):
                self.list_alerts.addItem(err)
        masks = {}
        scanned_all = self.grid.scanned(eps_min, eps_max)
        for sec in sectors:
            sid = sec.get("id")
            if not sid:
                continue
            ship = self._ship_sector(sec)
            if ship is None:
                continue
            try:
                m = self.grid.sector_mask(ship)
            except (KeyError, TypeError):
                continue
            masks[sid] = m
            cov = mo.split_by_face(self.grid, ship, eps_min, eps_max)    # E2: the executed volumes per face
            if cov["sr"] > 0 and cov["hole"] > 0.005:
                parts = ", ".join(f"{geo.FACE_NAMES[k]} {100 * s:.0f} %" for k, s in sorted(cov["faces"].items()))
                self.list_alerts.addItem(f"{sid}: coverage hole {100 * cov['hole']:.0f} % (no face scans it at this "
                                         f"attitude: scan limits or eps window){'; executed: ' + parts if parts else ''}")
            if sid in self._unreachable:
                pct, max_km = self._unreachable[sid]
                self.list_alerts.addItem(f"{sid}: range check pending (its cost profile is being computed)"
                                         if pct is None else
                                         f"{sid}: {pct:.0f} % of the volume out of reach even at full power"
                                         + (f" (at most ~{max_km:.0f} km)" if max_km else ""))
            conf = getattr(self, "_confirm_alerts", {}).get(sid)
            if conf == "none":
                cm = getattr(self, "_confirm_max", {}).get(sid)
                self.list_alerts.addItem(f"{sid}: its detections cannot be confirmed (no admissible confirmation mode "
                                         "at any search beam spacing)"
                                         + (f"; Ku confirmation possible up to ~{cm:.0f} km" if cm else "")
                                         + ": mission not tenable")
            elif conf == "depends":
                self.list_alerts.addItem(f"{sid}: confirmation only with some search beam spacings: the exact "
                                         "calculation says whether the optimiser's choice allows it")
            if sid in self._at_unreachable:
                self.list_alerts.addItem(f"{sid}: ~{self._at_unreachable[sid]:.1f} AT track(s) estimated out of Ku "
                                         "reach at a bracket's outer range (scan limit or range)")
        notes_current = self._redundancy_notes if self._redundancy_sig == self._red_sig() else {}
        for a, b, ov in mo.overlaps(self.grid.d_omega, masks):
            # both directions were tested (a covering b, and b covering a) -- show whichever apply, independently
            note_ab, note_ba = notes_current.get((a, b)), notes_current.get((b, a))
            if note_ab and note_ba:
                text = f"{a} / {b} share {ov:.3f} sr -- {note_ab} | {note_ba}"
            elif note_ab or note_ba:
                text = f"{a} / {b} share {ov:.3f} sr: {note_ab or note_ba}"
            else:
                text = (f"{a} / {b} share {ov:.3f} sr, budgeted independently -- fine for intentional layered "
                        "coverage, check for a duplicated requirement if not")
            self.list_alerts.addItem(text)
        if masks:
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
    def _red_sig(self) -> str:
        """The redundancy findings' signature: the mission's, and the shares the fast estimate serves (B, user 11
        November 2026: a covering volume judged with what the allocation gives it)."""
        return red.mission_signature(self.mission) + "|" + json.dumps(
            red.round_served(getattr(self, "_served", {}) or {}), sort_keys=True)

    def _schedule_redundancy_check(self) -> None:
        from core import cost_estimate as ce
        if ce.held():
            return    # (the estimate held after an edit: the check waits for the rerun, 13 November 2026)
        if red.RELIEF_ON and self.cost_table is not None:
            return    # (the fast estimate's relief pass brings the findings: _apply_redundancy)
        if self._red_sig() != self._redundancy_requested:
            self._red_timer.start()

    def _apply_redundancy(self, alloc: dict) -> None:
        """The findings of the fast estimate's relief pass (core/redundancy.relief_package: the same pairs, instant,
        shares served and plans as the relief, user 12 November 2026; a separate check ran > 3 min), with the cover
        ("X % covered by ...") beside the relief ("Y % taken out of its budget")."""
        rd = alloc.get("redundancy") or {}
        found = rd.get("findings") or {}
        self._cover = dict(alloc.get("cover") or {})
        self._redundancy = {id(s): found[str(s.get("id"))] for s in self.mission["sectors"]
                            if str(s.get("id")) in found}
        self._redundancy_notes = dict(rd.get("notes") or {})
        self._redundancy_sig, self._redundancy_error = self._red_sig(), ""
        self._redundancy_requested = self._redundancy_sig
        self._refresh_sector_items()

    def _shown_finding(self, sec: dict) -> Optional[red.Finding]:
        """The finding shown for a volume: only the volume the relief actually cuts (user, 12 November 2026: two
        volumes meeting each other's need both said "covered by" the other, while only one of them is cut in the
        emissions played); without the relief ([tool.redundancy] relief = false) every finding, as before."""
        f = self._active_finding(sec)
        if f is None or not red.RELIEF_ON:
            return f
        rel = (getattr(self, "_relief", {}) or {}).get(str(sec.get("id"))) or {}
        return f if float(rel.get("share", 0.0)) > 0.0 else None

    def _cover_text(self, sid: str, f: red.Finding) -> str:
        """"X % covered by A, B": the part the relief takes out of the volume's budget and the volumes that search it
        (as the 3D label "handled partially by"); without the relief, the check's share."""
        rel = (getattr(self, "_relief", {}) or {}).get(sid) or {}
        if red.RELIEF_ON and rel.get("by"):
            return f"{100 * float(rel.get('share', 0.0)):.0f} % covered by {', '.join(rel['by'])}"
        return f"{100 * f.fraction:.0f} % covered by {', '.join(f.by)}"

    def _start_redundancy_check(self) -> None:
        if self._red_worker is not None and self._red_worker.isRunning():
            return  # _on_redundancy_done starts the next check if the mission changed meanwhile
        sig = self._red_sig()
        self._redundancy_requested = sig
        token = (sig, [id(s) for s in self.mission["sectors"]])
        self._red_worker = RedundancyWorker(copy.deepcopy(self.mission), self.grid, token, self,
                                            served=dict(getattr(self, "_served", {}) or {}))
        self._red_worker.finished_ok.connect(self._on_redundancy_done)
        self._red_worker.failed.connect(self._on_redundancy_failed)
        self._red_worker.start()
        if hasattr(self.view, "watch_background"):    # (the ESTIMATE badge shows it, user 11 November 2026)
            QTimer.singleShot(50, self.view.watch_background)

    def _on_redundancy_done(self, result: tuple, token: tuple) -> None:
        findings, notes = result
        sig, ids = token
        if sig != self._red_sig():
            self._redundancy_requested = None
            self._red_timer.start()       # stale: the mission changed while the check ran
            return
        self._redundancy = {ids[f.index]: f for f in findings}
        self._redundancy_notes = notes
        self._redundancy_sig, self._redundancy_error = sig, ""
        self._refresh_sector_items()
        self._update_alerts(self.mission["global"].get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"]),
                            self.mission["global"].get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"]))

    def _on_redundancy_failed(self, message: str) -> None:
        self._redundancy_error = message
        self._redundancy_sig = self._red_sig()
        self._redundancy = {}
        self._redundancy_notes = {}
        self._schedule_refresh()

    @staticmethod
    def _ack_key(sec: dict, f: red.Finding) -> tuple:
        return id(sec), red.sector_signature(sec), tuple(f.by)

    def _active_finding(self, sec: dict) -> Optional[red.Finding]:
        """Current finding for this sector, None if none, out of date, or acknowledged ("Keep both")."""
        if self._redundancy_sig is None or self._redundancy_sig != self._red_sig():
            return None
        f = self._redundancy.get(id(sec))
        return None if f is None or self._ack_key(sec, f) in self._redundancy_ack else f

    def _add_redundancy_alerts(self) -> None:
        if self._redundancy_sig != self._red_sig():
            if len(self.mission["sectors"]) > 1:
                item = QListWidgetItem("Checking whether earlier sectors already meet the later needs...")
                item.setForeground(QColor("gray"))
                self.list_alerts.addItem(item)
            return
        if self._redundancy_error:
            self.list_alerts.addItem(self._redundancy_error)
        for i, sec in enumerate(self.mission["sectors"]):
            f = self._shown_finding(sec)
            if f is None:
                continue
            by = ", ".join(f.by)
            cov = self._cover_text(str(f.sid), f)
            if f.full:
                text = f"{f.sid}: {cov} -- double-click to keep {by} only"
            elif f.pieces:
                text = f"{f.sid}: {cov} -- double-click to trim {f.sid}"
            else:
                text = f"{f.sid}: {cov} -- double-click for details"
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
        tgt = mo.sector_target(sec)
        need = (f"{tgt.target_type} RCS {tgt.rcs_at(2.8):g} m2 "
                f"{'SW%d' % tgt.swerling if tgt.swerling is not None else tgt.fluct_s} at {sec['range_km']:.0f} km, "
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
    def _hidden_sector_ids(self) -> set:
        """The hidden sectors as the global view names them (their id, else "_idx<i>")."""
        return {(s.get("id") or f"_idx{i}") for i, s in enumerate(self.mission.get("sectors", []))
                if id(s) in self._hidden_sectors}

    # ------------------------------------------------------------------ the mission file (pass V3)
    def _signature(self) -> str:
        return json.dumps(self.mission, sort_keys=True, default=str)

    @property
    def modified(self) -> bool:
        return self._signature() != self._saved_sig

    def _update_title(self) -> None:
        self.file_label.setText(uc.file_title(self.path, self.modified))
        self.title_changed.emit(self.file_label.text())

    def _dir(self) -> str:
        """The file dialogs' folder: the current file's."""
        return os.path.dirname(self.path) if self.path else ""

    def confirm_discard(self) -> bool:
        return uc.confirm_discard(self, "Mission", self.modified, save=self.save, is_modified=lambda: self.modified)

    def file_commands(self) -> list:
        """(text, slot, icon, edits the mission) of the window's menu (user, 11 November 2026: one file, the mission,
        holding its flight plan, its last exact result and its profile; "Submenu::Command"). Open also takes a CSV
        (a mission shown unsaved)."""
        return [("New", self.new, "SP_FileIcon", True), ("Open...", self.open, "SP_DialogOpenButton", True),
                ("Save", self.save, "SP_DialogSaveButton", False),
                ("Save as...", self.save_as, "SP_DialogSaveButton", False),
                ("---", None, None, False),
                ("Radar configuration::Import...", self.import_radar_config, "SP_DialogOpenButton", True),
                ("Radar configuration::Export...", self.export_radar_config, "SP_DialogSaveButton", False)]
        # (user, 12 November 2026: "Flight plan from a scenario", "Remove the flight plan" and "Export mission as
        # CSV" removed, not useful: the work file holds the scenario, Open takes a CSV still)

    # ------------------------------------------------------------------ radar configuration (user, 12 November 2026)
    # the volumes and their parameters with the mission settings, without navigation, sky nor scenario: exported from
    # one mission, imported into another (it replaces that one's volumes and settings); scenarios/radar_configs
    RADAR_CONFIG_FORMAT = "radar_config"
    RADAR_CONFIG_DIR = os.path.join(slink.APP_DIR, "scenarios", "radar_configs")
    _NOT_CONFIG = ("nav", "sky")                     # (global keys of the situation, not of the radar)

    def _config_globals(self, g: dict) -> dict:
        return {k: copy.deepcopy(v) for k, v in g.items() if k not in self._NOT_CONFIG and not str(k).startswith("_")}

    def export_radar_config(self) -> None:
        """The volumes and the mission settings to a radar configuration file (scenarios/radar_configs)."""
        os.makedirs(self.RADAR_CONFIG_DIR, exist_ok=True)
        stem = os.path.splitext(os.path.basename(self.path))[0] if self.path else "radar_config"
        path, _ = QFileDialog.getSaveFileName(self, "Export the radar configuration",
                                              os.path.join(self.RADAR_CONFIG_DIR, f"{stem}.json"),
                                              "Radar configuration (*.json)")
        if not path:
            return
        cfg = {"format": self.RADAR_CONFIG_FORMAT, "version": 1,
               "saved_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
               "source": os.path.basename(self.path) if self.path else None,
               "global": self._config_globals(self.mission.get("global", {})),
               "sectors": copy.deepcopy(self.mission.get("sectors", []))}
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(cfg, fh, indent=1, ensure_ascii=False)
        except OSError as exc:
            QMessageBox.critical(self, "Export", f"Could not write {path}: {exc}")

    def import_radar_config(self) -> None:
        """A radar configuration replaces the mission's volumes and settings (its navigation, sky and flight plan
        kept), after confirmation."""
        if self.locked:
            return
        os.makedirs(self.RADAR_CONFIG_DIR, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(self, "Import a radar configuration", self.RADAR_CONFIG_DIR,
                                              "Radar configuration (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as fh:
                cfg = json.load(fh)
            if not isinstance(cfg, dict) or cfg.get("format") != self.RADAR_CONFIG_FORMAT:
                raise ValueError("not a radar configuration (export one with File > Radar configuration > Export)")
            sectors, g_cfg = list(cfg.get("sectors") or []), dict(cfg.get("global") or {})
            trial = {"global": dict(self.mission.get("global", {}), **g_cfg), "sectors": sectors}
            errs = md.validate_mission(trial)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Import", f"Could not import {os.path.basename(path)}: {exc}")
            return
        n_old = len(self.mission.get("sectors", []))
        text = (f"Replace the {n_old} volume(s) and the mission settings with the {len(sectors)} volume(s) of "
                f"{os.path.basename(path)}? The navigation and the flight plan are kept.")
        if errs:
            text += "\n\nThe configuration has problems (shown in the alerts):\n- " + "\n- ".join(errs[:6])
        if QMessageBox.question(self, "Import a radar configuration", text) != QMessageBox.Yes:
            return
        g = self.mission.setdefault("global", {})
        for k in [k for k in g if k not in self._NOT_CONFIG and not str(k).startswith("_")]:
            g.pop(k)
        g.update(copy.deepcopy(g_cfg))
        self.mission["sectors"] = copy.deepcopy(sectors)
        self._hidden_sectors = set()
        self.sb_eps_min.setValue(g.get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"]))
        self.sb_eps_max.setValue(g.get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"]))
        self.sb_elec.setValue(self._cap_value())
        self._load_mission_settings()
        self._cur_index = 0
        self._reload_sector_list()
        self._schedule_refresh()
        self._update_title()

    def new(self) -> None:
        if self.locked or not self.confirm_discard():
            return
        self._load(md.empty_mission(), None)

    def open(self) -> None:
        if self.locked or not self.confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(self, "Open a mission", slink.SCENARIO_DIR,   # (user, 11 Nov 2026: always)
                                              "Mission (*.json *.csv);;Mission (*.json);;Mission as CSV (*.csv)")
        if path:
            with uc.busy(self, f"Opening {os.path.basename(path)}..."):
                self.open_path(path)

    def open_path(self, path: str) -> bool:
        """The work file (user, 11 November 2026: one file for the Scenario Editor and the mission preparation,
        sim/document.py), or a former mission / scenario file (converted: shown unsaved), or a CSV (no file)."""
        if path.lower().endswith(".csv"):
            try:
                mission = md.load_mission(path)
            except Exception as exc:  # noqa: BLE001
                QMessageBox.critical(self, "Open", f"Could not open {path}: {exc}")
                return False
            self._load(mission, None, saved=False)
            return True
        from sim import document
        try:
            with open(path, encoding="utf-8") as fh:
                is_doc = document.kind(json.load(fh)) == "document"
            doc = document.load(path)
        except Exception as exc:  # noqa: BLE001 -- bad path / file: say so and keep the mission shown
            QMessageBox.critical(self, "Open", f"Could not open {path}: {exc}")
            return False
        mission = dict(doc.mission)
        t = float(mission.pop("t_nav_s", 0.0) or 0.0)
        if doc.scenario is not None:
            mission[slink.FLIGHT_PLAN_KEY] = {"scenario": doc.scenario, "source": os.path.basename(path)}
            mission[slink.LINK_KEY] = {"t_s": t}
        for k, v in doc.extras.items():
            mission[k] = v
        self._load(mission, path, saved=is_doc)
        return True

    def _sections(self) -> dict:
        """The work file's sections this tool writes: the mission (its instant as t_nav_s), its results and profile,
        and the scenario only when it was imported or removed here (else the one on disk is kept: the Scenario
        Editor's)."""
        m = self.mission
        mission = {k: v for k, v in m.items()
                   if k not in (slink.FLIGHT_PLAN_KEY, slink.LINK_KEY) + slink.DERIVED_KEYS}
        link = m.get(slink.LINK_KEY)
        if isinstance(link, dict):
            mission["t_nav_s"] = float(link.get("t_s", 0.0))
        out = {"mission": mission, "results": m.get(slink.RESULTS_KEY), "profile": m.get(slink.PROFILE_KEY)}
        if getattr(self, "_scenario_dirty", False):
            fp = m.get(slink.FLIGHT_PLAN_KEY)
            out["scenario"] = fp.get("scenario") if isinstance(fp, dict) else None
        return out

    def check_external(self) -> None:
        """The work file changed on disk by the Scenario Editor (user, 11 November 2026): its scenario section
        reloaded as the flight plan (the run recomputed if needed); the mission's own edits kept."""
        if not self.path or not os.path.exists(self.path) or self.locked:
            return
        mtime = os.path.getmtime(self.path)
        if getattr(self, "_disk_mtime", None) is None or mtime == self._disk_mtime:
            self._disk_mtime = mtime
            return
        self._disk_mtime = mtime
        from sim import document
        disk = document.read_section(self.path, "scenario")
        fp = self.mission.get(slink.FLIGHT_PLAN_KEY) or {}
        if getattr(self, "_scenario_dirty", False) or not isinstance(disk, dict) or \
                json.dumps(disk, sort_keys=True, default=str) == json.dumps(fp.get("scenario"), sort_keys=True,
                                                                            default=str):
            return
        was_saved = not self.modified
        self.mission[slink.FLIGHT_PLAN_KEY] = {"scenario": disk, "source": os.path.basename(self.path)}
        self.mission.setdefault(slink.LINK_KEY, {"t_s": 0.0})
        self._relink()
        if was_saved:
            self._saved_sig = self._signature()
        self._update_title()
        self.lbl_scn_alerts.setText("flight plan reloaded: the scenario was changed in the Scenario Editor")
        self._schedule_refresh()

    def save(self) -> bool:
        return self.save_as() if self.path is None else self._write(self.path)

    def save_as(self) -> bool:
        path, _ = QFileDialog.getSaveFileName(self, "Save the mission",
                                              self.path or os.path.join(slink.SCENARIO_DIR, "mission.json"),
                                              "Mission (*.json)")
        if not path:
            return False
        return self._write(path if path.lower().endswith(".json") else path + ".json")

    def _write(self, path: str) -> bool:
        errs = md.validate_mission(self.mission)
        if errs and QMessageBox.question(self, "Mission", f"The mission has {len(errs)} problem(s). Save anyway?",
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
            return False
        if self._link is not None and isinstance(self.mission.get(slink.LINK_KEY), dict):
            self.mission[slink.LINK_KEY]["path"] = slink.relative(self._link.path, path)   # (relative to the file)
        try:                                          # the work file: this tool's sections (user, 11 Nov 2026)
            from sim import document
            sections = self._sections()
            if path != self.path or not os.path.exists(path):
                fp = self.mission.get(slink.FLIGHT_PLAN_KEY)
                sections["scenario"] = fp.get("scenario") if isinstance(fp, dict) else None
            document.save_sections(path, sections, check=False)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Save", f"Save failed: {exc}")
            return False
        self._scenario_dirty = False
        self._disk_mtime = os.path.getmtime(path)
        self.path, self._saved_sig = path, self._signature()
        self._update_title()
        return True

    def _load(self, mission: dict, path: Optional[str], saved: bool = True) -> None:
        self.set_mission(mission)
        self.path = path
        self._scenario_dirty = not saved and slink.FLIGHT_PLAN_KEY in mission   # (a converted file: written in full)
        self._disk_mtime = os.path.getmtime(path) if path and os.path.exists(path) else None
        self._relink()
        self._saved_sig = self._signature() if saved and not getattr(self, "_converted", False) else None
        self._update_title()
        self.results_reopened.emit()                  # (the main window reopens a stored exact result if it fits)

    def _on_import(self) -> None:
        """A CSV mission (or a JSON one): shown unsaved, with no file (Save asks where)."""
        if self.locked or not self.confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(self, "Import a mission", self._dir(), "Mission (*.csv *.json)")
        if not path:
            return
        try:
            mission = md.load_mission(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Import", f"Import failed: {exc}")
            return
        self._load(mission, None, saved=False)

    def set_mission(self, mission: dict) -> None:
        """Show and edit this mission (its file and state: _load)."""
        self.mission = mission
        self.mission.setdefault("global", dict(md.GLOBAL_DEFAULTS))
        self.mission.setdefault("sectors", [])
        self._hidden_sectors = set()
        self._est_sig = None                    # (a mission opened: estimated at once, not held)
        self.sb_eps_min.setValue(self.mission["global"].get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"]))
        self.sb_eps_max.setValue(self.mission["global"].get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"]))
        self.sb_elec.setValue(self._cap_value())
        self._load_mission_settings()
        self._load_nav_form()
        self._cur_index = 0
        self._reload_sector_list()
        self._schedule_refresh()

    def _on_export(self, fmt: str) -> None:
        ext = "json" if fmt == "json" else "csv"
        stem = os.path.splitext(self.path)[0] if self.path else "mission"
        path, _ = QFileDialog.getSaveFileName(self, "Export the mission", f"{stem}.{ext}",
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

    def _check_mission(self) -> bool:
        """Mission-prep validation; lists the errors in a dialog when there are some."""
        errs = md.validate_mission(self.mission)
        if errs:
            QMessageBox.warning(self, "Invalid mission preparation", "\n".join(f"- {e}" for e in errs))
        return not errs

    def _set_calc_kind(self, kind: str, run: bool = False) -> None:
        self._calc_kind = kind
        if kind == "exact":
            t = (self.mission.get(slink.LINK_KEY) or {}).get("t_s") if isinstance(self.mission.get(slink.LINK_KEY),
                                                                                    dict) else None
            self.b_run.setText("Calculate: exact" + (f" (t = {float(t):g} s)" if t is not None else ""))
            self.b_run.setToolTip("The exact calculation at the mission's instant (set it with 'Use this instant' "
                                  "under the view); opens the Results. The arrow: the other kind")
        else:
            self.b_run.setText("Calculate: flight plan")
            self.b_run.setToolTip("Along the linked scenario's flight plan: each instant by the fast estimate, the "
                                  "losses recomputed exactly; shown in Profile. The arrow: the other kind")
        if run:
            self._on_calculate()

    def _on_calculate(self) -> None:
        if self._calc_kind == "profile":
            if self._link is None:
                QMessageBox.information(self, "Calculate", "A flight plan is needed: link a computed scenario first.")
                return
            self.profile_requested.emit()
        else:
            self._on_run()

    def _on_run(self) -> None:
        if self._check_mission():
            self.run_requested.emit(slink.for_calculation(self.mission))
