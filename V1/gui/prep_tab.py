#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PREP_TAB -- mission-preparation tab: watch-sector form (incl. confirmation mode and active-tracking range brackets) +
real-time 3D view + fast cost estimate (per-face gauges) +
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
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QListWidget, QListWidgetItem, QMessageBox, QProgressBar, QPushButton, QScrollArea, QSpinBox,
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

    def __init__(self, mission: dict, grid, token, parent=None):
        super().__init__(parent)
        self.mission, self.grid, self.token = mission, grid, token

    def run(self) -> None:
        try:
            result = red.find_redundancies(self.mission, self.grid)
        except Exception as exc:  # noqa: BLE001 -- a failed check must not take the window down
            self.failed.emit(f"redundancy check failed: {exc}")
            return
        self.finished_ok.emit(result, self.token)


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
        v.addWidget(self._build_mission_settings_group())
        v.addWidget(self._build_nav_group())
        v.addWidget(self._build_sector_list_group())
        v.addWidget(self._build_sector_form_group())
        v.addWidget(self._build_gauges_group())
        v.addWidget(self._build_alerts_group())
        v.addWidget(self._build_actions_group())
        v.addStretch(1)

        # no horizontal scrolling: the panel is always at least as wide as its content. No maximum: the splitter
        # handle is free to make it as wide as the user wants (bounded only by the 3D view's own minimum width).
        panel.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        min_w = (inner.minimumSizeHint().width() + panel.verticalScrollBar().sizeHint().width()
                 + 2 * panel.frameWidth())
        panel.setMinimumWidth(min_w)

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
        self.sb_eps_min = self._spin(-90, 90, 1, self.mission["global"].get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"]))
        self.sb_eps_max = self._spin(-90, 90, 1, self.mission["global"].get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"]))
        self.sb_elec = self._spin(0.0, md.FACE_FEEDER_MAX_MW, 0.1, self._cap_value())
        self.sb_elec.setSpecialValueText("shared (no cap)")            # step 4.1: faces share the ship total
        self.sb_elec.setToolTip("Optional cap of the mean electrical power of each face (MW). The faces share the "
                                f"ship's {md.SHIP_ELEC_TOTAL_MW:g} MW; 0 = no cap (a list of 4 caps can be given in the "
                                "mission file)")
        form.addRow("Rear limit eps_min (deg)", self.sb_eps_min)
        form.addRow("Front limit eps_max (deg)", self.sb_eps_max)
        form.addRow("Power cap / face (MW)", self.sb_elec)
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
        """Navigation message of the mission (step 3.0, core/frames.py): attitude, optional orbit."""
        box = QGroupBox("Navigation (attitude and orbit)")
        self._edit_widgets.append(box)
        self._updating_nav = False
        form = QFormLayout(box)
        self.ed_nav_label = QLineEdit()
        self.ed_nav_epoch = QLineEdit()
        self.ed_nav_epoch.setPlaceholderText("optional, ISO 8601 UTC, e.g. 2055-03-21T12:00:00Z")
        self.cb_att_ref = QComboBox()
        self.cb_att_ref.addItems(frames.ATTITUDE_REFS)
        self.cb_att_ref.setToolTip("Frame of the yaw / pitch / roll below: icrf (inertial) or rtn (local orbital: "
                                   "X radial, Y along track, Z orbit normal; needs the orbit)")
        self.sb_yaw, self.sb_pitch, self.sb_roll = (self._spin(-180, 180, 1, 0.0), self._spin(-90, 90, 1, 0.0),
                                                    self._spin(-180, 180, 1, 0.0))
        for sb in (self.sb_yaw, self.sb_pitch, self.sb_roll):
            sb.setDecimals(3)
        self.ed_quat = QLineEdit()
        self.ed_quat.setToolTip("q_ship_to_icrf = [w, x, y, z] (Hamilton, scalar first: v_icrf = R(q) v_ship); "
                                "editable, the angles follow")
        self.cb_body = QComboBox()
        self.cb_body.addItems(["(no orbit)"] + list(frames.BODY_RADIUS_KM))
        self.sb_r = [self._spin(-1e9, 1e9, 100, 0.0) for _ in range(3)]
        self.sb_v = [self._spin(-1e3, 1e3, 0.1, 0.0) for _ in range(3)]
        for sb in self.sb_r + self.sb_v:
            sb.setDecimals(4)
        self.lbl_nav = QLabel("")
        self.lbl_nav.setWordWrap(True)
        self.lbl_nav.setStyleSheet("color: gray; font-size: 11px;")
        att = QHBoxLayout()
        for sb in (self.sb_yaw, self.sb_pitch, self.sb_roll):
            att.addWidget(sb)
        rr, vv = QHBoxLayout(), QHBoxLayout()
        for sb in self.sb_r:
            rr.addWidget(sb)
        for sb in self.sb_v:
            vv.addWidget(sb)
        form.addRow("Label", self.ed_nav_label)
        form.addRow("Epoch", self.ed_nav_epoch)
        form.addRow("Attitude relative to", self.cb_att_ref)
        form.addRow("Yaw / pitch / roll (deg, 3-2-1)", att)
        form.addRow("Quaternion ship -> ICRF", self.ed_quat)
        form.addRow("Central body", self.cb_body)
        form.addRow("r from the body (km, ICRF)", rr)
        form.addRow("v (km/s, ICRF)", vv)
        form.addRow(self.lbl_nav)
        for sb in (self.sb_yaw, self.sb_pitch, self.sb_roll):
            sb.valueChanged.connect(lambda _v: self._on_nav_changed("angles"))
        for sb in self.sb_r + self.sb_v:
            sb.valueChanged.connect(lambda _v: self._on_nav_changed("orbit"))
        self.cb_att_ref.currentTextChanged.connect(lambda _t: self._on_nav_changed("ref"))
        self.cb_body.currentTextChanged.connect(lambda _t: self._on_nav_changed("orbit"))
        self.ed_quat.editingFinished.connect(lambda: self._on_nav_changed("quat"))
        self.ed_nav_label.editingFinished.connect(lambda: self._on_nav_changed("text"))
        self.ed_nav_epoch.editingFinished.connect(lambda: self._on_nav_changed("text"))
        self._load_nav_form()
        return box

    def _nav_msg(self) -> dict:
        nav = self.mission["global"].get("nav")
        return dict(nav[0]) if isinstance(nav, list) and nav and isinstance(nav[0], dict) else {}

    @staticmethod
    def _ref_axes(ref: str, n: "frames.Nav") -> Optional[np.ndarray]:
        """Matrix ICRF -> the attitude reference frame (None: RTN without a valid orbit)."""
        if ref == "rtn":
            return n.rtn() if n.has_orbit else None
        return np.eye(3)

    def _load_nav_form(self) -> None:
        """Navigation widgets from the mission (the angles in the message's attitude_ref frame)."""
        self._updating_nav = True
        try:
            msg = self._nav_msg()
            self.ed_nav_label.setText(str(msg.get("label") or ""))
            self.ed_nav_epoch.setText(str(msg.get("epoch") or ""))
            body = msg.get("central_body")
            self.cb_body.setCurrentText(body if body in frames.BODY_RADIUS_KM else "(no orbit)")
            for sbs, key in ((self.sb_r, "r_km"), (self.sb_v, "v_kms")):
                vec = msg.get(key) or [0.0, 0.0, 0.0]
                for sb, c in zip(sbs, vec):
                    sb.setValue(float(c))
            self.cb_att_ref.setCurrentText(msg.get("attitude_ref", "icrf"))
            self._show_attitude(msg)
        finally:
            self._updating_nav = False
        self._nav_status()

    def _show_attitude(self, msg: dict) -> None:
        q = msg.get("q_ship_to_icrf", [1.0, 0.0, 0.0, 0.0])
        self.ed_quat.setText(", ".join(f"{c:.9f}" for c in q))
        if frames.validate_nav([msg]):
            return
        n = frames.Nav(msg)
        a = self._ref_axes(msg.get("attitude_ref", "icrf"), n)
        if a is not None:
            for sb, val in zip((self.sb_yaw, self.sb_pitch, self.sb_roll), frames.euler_321_of(a @ n.r_si)):
                sb.setValue(val)

    def _on_nav_changed(self, source: str) -> None:
        if self._updating_nav:
            return
        msg = self._nav_msg()
        msg.setdefault("q_ship_to_icrf", [1.0, 0.0, 0.0, 0.0])
        for key, ed in (("label", self.ed_nav_label), ("epoch", self.ed_nav_epoch)):
            text = ed.text().strip()
            if text:
                msg[key] = text
            else:
                msg.pop(key, None)
        body = self.cb_body.currentText()
        if body in frames.BODY_RADIUS_KM:
            msg["central_body"] = body
            msg["r_km"] = [sb.value() for sb in self.sb_r]
            msg["v_kms"] = [sb.value() for sb in self.sb_v]
        else:
            for key in ("central_body", "r_km", "v_kms"):
                msg.pop(key, None)
        msg["attitude_ref"] = self.cb_att_ref.currentText()
        if source == "quat":
            try:
                q = [float(c) for c in self.ed_quat.text().replace(";", ",").split(",")]
                if len(q) == 4:
                    msg["q_ship_to_icrf"] = q
            except ValueError:
                pass
        elif source == "angles":
            n = frames.Nav(msg) if not frames.validate_nav([msg]) else None
            a = self._ref_axes(msg["attitude_ref"], n) if n is not None else None
            if a is not None:
                r_ref = frames.euler_321(self.sb_yaw.value(), self.sb_pitch.value(), self.sb_roll.value())
                msg["q_ship_to_icrf"] = frames.matrix_to_quat(a.T @ r_ref)
        self.mission["global"]["nav"] = [msg]
        if source in ("quat", "ref", "orbit"):   # the angles follow the quaternion (kept), in the chosen frame
            self._updating_nav = True
            try:
                self._show_attitude(msg)
            finally:
                self._updating_nav = False
        else:
            self.ed_quat.setText(", ".join(f"{c:.9f}" for c in msg["q_ship_to_icrf"]))
        self._nav_status()
        self._schedule_refresh()

    def _nav_status(self) -> None:
        """Operator feedback of the navigation, and the attitude of the 3D view's ICRF option."""
        errs = frames.validate_nav(self.mission["global"].get("nav"))
        if errs:
            self.lbl_nav.setText("; ".join(errs))
            self.lbl_nav.setStyleSheet(f"color: {ALERT_COLOR}; font-size: 11px;")
            self.view.set_navigation(None)
            return
        n = frames.parse_nav(self.mission["global"].get("nav"))
        text = "No navigation: identity attitude, no orbit." if self.mission["global"].get("nav") is None else ""
        if n.has_orbit:
            text = (f"Altitude {np.linalg.norm(n.r) - n.body_radius_km:.1f} km above the {n.body}; "
                    f"limb {n.limb_depression_deg(0.0):.2f} deg below the local horizontal.")
        self.lbl_nav.setText(text)
        self.lbl_nav.setStyleSheet("color: gray; font-size: 11px;")
        self.view.set_navigation(n.r_si)

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
        self.cb_frame = QComboBox()
        self.cb_frame.addItems(frames.FRAMES)
        self.cb_frame.setToolTip("Frame of the volume (step 3.0): ship (tied to the ship), inertial (ICRF: phi = RA, "
                                 "eps = Dec), rtn (local orbital), limb (eps counted from the limb of the central "
                                 "body, box only), two_dir (Z = primary direction, X toward the secondary), track (cap around "
                                 "a track or a designation, radius k sigma; step 3.3)")
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
        v.addWidget(self.grp_limb)
        self.grp_two_dir = QGroupBox("Two directions (ICRF)")
        ft = QFormLayout(self.grp_two_dir)
        self.sb_two_dir = {k: self._spin(-90 if "dec" in k else 0, 90 if "dec" in k else 360, 1, 0.0)
                           for k in frames.TWO_DIR_KEYS}
        for k, label in zip(frames.TWO_DIR_KEYS, ("primary RA (deg)", "primary Dec (deg)", "secondary RA (deg)",
                                                   "secondary Dec (deg)")):
            ft.addRow(label, self.sb_two_dir[k])
        v.addWidget(self.grp_two_dir)
        self.grp_track = QGroupBox("Track / designation (ICRF, cap of k sigma)")      # step 3.3, F8
        ftr = QFormLayout(self.grp_track)
        self.sb_track = {"track_ra_deg": self._spin(0, 360, 1, 0.0), "track_dec_deg": self._spin(-90, 90, 1, 0.0),
                         "track_sigma_deg": self._spin(0.001, 30, 3, 0.1)}
        for k, label in zip(frames.TRACK_KEYS, ("centre RA (deg)", "centre Dec (deg)", "sigma (deg)")):
            ftr.addRow(label, self.sb_track[k])
        v.addWidget(self.grp_track)

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
        common.addRow("Target RR (s)", self.sb_rr)
        common.addRow("Closing speed min (m/s)", self.sb_vmin)
        common.addRow("Closing speed max (m/s)", self.sb_vmax)
        # step 4.3: scheduler priority of the volume
        self.chk_self_prot = QCheckBox("self-protection (P1)")
        self.chk_self_prot.setToolTip("A self-protection volume is searched at priority P1, before confirmations; "
                                      "otherwise it is a search volume of priority P4_<search priority>")
        self.chk_self_prot.toggled.connect(self._on_field_changed)
        self.sb_search_prio = QSpinBox()
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
                   self.sb_rr, self.sb_vmin, self.sb_vmax, self.sb_limb_alt, *self.sb_two_dir.values(),
                   *self.sb_track.values()):
            sb.valueChanged.connect(self._on_field_changed)
        self.grp_limb.setVisible(False)
        self.grp_two_dir.setVisible(False)
        self.grp_track.setVisible(False)

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
        note = QLabel("Fast estimate (precomputed table + one Ku look per face for active tracking): indicative "
                       "while editing. The exact budget (incl. confirmation) comes from 'Run calculation'.")
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
            for w in (self.ed_id, self.cb_shape, self.cb_frame, self.grp_limb, self.grp_two_dir, self.grp_track,
                      self.sb_rcs,
                      self.sb_swerling, self.sb_range,
                      self.sb_rr, self.sb_vmin, self.sb_vmax, self.cb_rx_sll, self.grp_box, self.grp_cap, self.grp_at):
                w.setEnabled(enabled)
            self._fill_at_table(sec)
            if not sec:
                return
            self.ed_id.setEditText(sec.get("id", ""))
            shape = sec.get("shape", "box")
            self.cb_shape.setCurrentText(shape)
            self.grp_box.setVisible(shape == "box")
            self.grp_cap.setVisible(shape == "cap")
            fr = sec.get("frame", "ship")
            self.cb_frame.setCurrentText(fr)
            self.grp_limb.setVisible(fr == "limb")
            self.grp_two_dir.setVisible(fr == "two_dir")
            self.grp_track.setVisible(fr == "track")
            self.sb_limb_alt.setValue(sec.get("limb_alt_km", frames.LIMB_ALT_KM_DEFAULT))
            for k, sb in self.sb_two_dir.items():
                sb.setValue(sec.get(k, 0.0))
            for k, sb in self.sb_track.items():
                sb.setValue(sec.get(k, 0.1 if k == "track_sigma_deg" else 0.0))
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
        self._sync_form_to_model()

    def _on_frame_changed(self, fr: str) -> None:
        self.grp_limb.setVisible(fr == "limb")
        self.grp_two_dir.setVisible(fr == "two_dir")
        self.grp_track.setVisible(fr == "track")
        if fr == "track" and self.cb_shape.currentText() != "cap":
            self.cb_shape.setCurrentText("cap")              # a track sector is a cap (step 3.3, F8)
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
        for k, sb in self.sb_two_dir.items():
            if sec["frame"] == "two_dir":
                sec[k] = sb.value()
            else:
                sec.pop(k, None)
        for k, sb in self.sb_track.items():
            if sec["frame"] == "track":
                sec[k] = sb.value()
            else:
                sec.pop(k, None)
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
        # a class sector keeps its class values until the operator edits them (they then become sector overrides)
        if sec.get("target_type") is None or self.sb_rcs.value() != getattr(self, "_target_shown", (None, None))[0]:
            sec["rcs_m2"] = self.sb_rcs.value()
        if sec.get("target_type") is None or self.sb_swerling.value() != getattr(self, "_target_shown", (None, None))[1]:
            sec["swerling"] = self.sb_swerling.value()
        sec["range_km"] = self.sb_range.value()
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

    def _refresh_all(self) -> None:
        eps_min = self.mission["global"].get("eps_min_deg", md.GLOBAL_DEFAULTS["eps_min_deg"])
        eps_max = self.mission["global"].get("eps_max_deg", md.GLOBAL_DEFAULTS["eps_max_deg"])
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
            ship = self._ship_sector(sec)
            if ship is not None:
                draws.append(SectorDraw(sid, ship, geo.sector_color(i), text))
        self.view.set_scene(eps_min, eps_max, draws, hidden=self._hidden_sids())

    def _update_gauges(self, eps_min: float, eps_max: float) -> None:
        totals = {k: {"energy": 0.0, "time_min": 0.0, "kw": 0.0, "at_kw": 0.0, "at_ku_time": 0.0} for k in range(4)}
        p_avg_max = rm.face_elec_avail_w(rm.FACE_ELEC_POWER_MW)       # the optimiser's reference (step 4.1)
        self._unreachable = {}
        self._at_unreachable = {}
        if self.cost_table is not None:
            for sec in self.mission["sectors"]:
                ship = self._ship_sector(sec)
                if ship is None:
                    continue
                try:
                    faces = ce.estimate_sector_faces(self.cost_table, ship, self.grid, eps_min, eps_max, p_avg_max)
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
                    self._unreachable[sid] = worst
                if sid and at_unreach > 0.05:
                    self._at_unreachable[sid] = at_unreach
        for k in range(4):
            g = self.gauges[k]
            e_pct = 100 * totals[k]["energy"]
            g["energy"].setValue(min(100, round(e_pct)))
            g["energy"].setFormat(f"energy {e_pct:.0f}%")   # exact value even past 100 -- the bar fill still clips
            over_e = totals[k]["energy"] > 1.0
            g["energy"].setStyleSheet("QProgressBar::chunk { background-color: #c0392b; }" if over_e else "")
            t_pct = 100 * totals[k]["time_min"]
            g["time"].setValue(min(100, round(t_pct)))
            g["time"].setFormat(f"time on air >= {t_pct:.0f}%")
            g["kw"].setText(f"{totals[k]['kw']:.0f} kW")
            # tracking margin: power left on top of search + AT (the arrays only share the face's power)
            p_left_w = max(0.0, 1.0 - totals[k]["energy"]) * p_avg_max
            at_kw = totals[k]["at_kw"]
            at_duty_pct = 100 * totals[k]["at_ku_time"]
            margin = f"{p_left_w / 1e3:.0f} kW left"
            if at_kw > 0.05 or at_duty_pct > 0.5:
                over = at_duty_pct > 100 * trk.KU_DUTY_MAX
                margin += f" -- AT est. {at_kw:.1f} kW, Ku duty {at_duty_pct:.1f} %"
                if over:
                    margin += f" (> {100 * trk.KU_DUTY_MAX:.0f} % ceiling)"
            g["margin"].setText(margin)
            g["margin"].setStyleSheet("color: #c0392b;" if at_duty_pct > 100 * trk.KU_DUTY_MAX else "")

    def _update_alerts(self, eps_min: float, eps_max: float) -> None:
        self.list_alerts.clear()
        sectors = self.mission["sectors"]
        if self.cost_table is not None and not self.cost_table.is_current:
            self.list_alerts.addItem("Fast estimate: core/cost_table.json was built with other radar parameters "
                                     "(radar_system.toml changed) -- rebuild it: python tools/gen_cost_table.py")
        self._add_redundancy_alerts()
        for err in frames.validate(self.mission):   # navigation and frames (step 3.0)
            self.list_alerts.addItem(err)
        try:
            for alert in frames.resolve(self.mission)[1]:
                self.list_alerts.addItem(alert)
        except ValueError:
            pass
        for sec in sectors:   # confirmation / AT inputs (also checked by 'Validate mission prep')
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
            tot_om = float((self.grid.d_omega * m).sum())
            scanned_om = float((self.grid.d_omega * (m & scanned_all)).sum())
            if tot_om > 0 and scanned_om / tot_om < 0.995:
                pct = 100 * (1 - scanned_om / tot_om)
                self.list_alerts.addItem(f"{sid}: {pct:.0f} % outside the scanned zone (scan limit or eps window)")
            if sid in self._unreachable:
                self.list_alerts.addItem(f"{sid}: {self._unreachable[sid]:.0f} % of positions can't reach the "
                                         "required range even at full power (budget shown is a floor, not the real cost)")
            if sid in self._at_unreachable:
                self.list_alerts.addItem(f"{sid}: ~{self._at_unreachable[sid]:.1f} AT track(s) estimated out of Ku "
                                         "reach at a bracket's outer range (scan limit or range)")
        notes_current = self._redundancy_notes if self._redundancy_sig == red.mission_signature(self.mission) else {}
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
        for k in range(4):
            e = self.gauges[k]["energy"].value()
            if e >= 100:
                self.list_alerts.addItem(f"{geo.FACE_NAMES[k]}: estimated energy over-subscribed (>= 100 %)")
            elif e > 90:
                self.list_alerts.addItem(f"{geo.FACE_NAMES[k]}: estimated tracking margin < 10 % of the face power")
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

    def _on_redundancy_done(self, result: tuple, token: tuple) -> None:
        findings, notes = result
        sig, ids = token
        if sig != red.mission_signature(self.mission):
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
        self._redundancy_sig = red.mission_signature(self.mission)
        self._redundancy = {}
        self._redundancy_notes = {}
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

    def _check_mission(self) -> bool:
        """Mission-prep validation; lists the errors in a dialog when there are some."""
        errs = md.validate_mission(self.mission)
        if errs:
            QMessageBox.warning(self, "Invalid mission preparation", "\n".join(f"- {e}" for e in errs))
        return not errs

    def _on_validate(self) -> None:
        if self._check_mission():
            QMessageBox.information(self, "Mission preparation", "Mission preparation is valid.")

    def _on_run(self) -> None:
        if self._check_mission():
            self.run_requested.emit(copy.deepcopy(self.mission))
