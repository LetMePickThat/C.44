#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RESULTS_TAB -- optimisation result tab (closable). Same 3D view as the mission-preparation tab, but sectors are
coloured by result (achieved RR / target RR, or % energy), with WF / range / RR / energy / flag labels; per-sector
verdict (step 5.3: state, limiting resource, constraints in alert or failed), per-sector
summary table, waveforms by scan-angle bin, per-face totals, confirmation + active tracking (AT) budget (search +
confirmation + AT per face, confirmation mode per sector, AT cost per range bracket, what to relax); JSON/CSV export. It shows the frozen copy of the mission
that was optimised.
"""
from __future__ import annotations

import csv
import json

from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QMessageBox,
    QPushButton, QScrollArea, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from core import frames
from core import geometry as geo
from core import optimizer as mo
from core import radar_model as rm
from core import validation
from gui.scene_view import SceneView, SectorDraw


class VerifyWorker(QThread):
    """core.validation.verify() of one advice on a background thread (step 5.3, P2): several whole recomputations."""
    progress = pyqtSignal(str)
    done = pyqtSignal(dict)
    failed = pyqtSignal(str)

    def __init__(self, mission: dict, advice: dict, parent=None):
        super().__init__(parent)
        self.mission, self.advice = mission, advice

    def run(self) -> None:
        try:
            res = validation.verify(self.mission, self.advice, progress=self.progress.emit)
        except Exception as exc:  # noqa: BLE001 -- surface any model error to the UI instead of crashing
            self.failed.emit(f"verification failed: {exc}")
            return
        self.done.emit(res)


def _metric_color(ratio: float) -> str:
    """Green (<=1) -> amber (~1.5) -> red (>=2.5), for a 'how far over target' ratio."""
    ratio = max(0.0, min(ratio, 2.5))
    if ratio <= 1.0:
        return "#2fa35a"
    if ratio <= 1.5:
        t = (ratio - 1.0) / 0.5
        return _lerp_color("#2fa35a", "#e08a1e", t)
    t = (ratio - 1.5) / 1.0
    return _lerp_color("#e08a1e", "#d6453d", t)


def _energy_color(frac: float) -> str:
    """Energy fraction of a face's average power: green (<=70 %, i.e. >= 30 % left for tracking) -> amber (100 %)
    -> red (>=150 %). A fraction only exceeds 1 when the face is over-subscribed, so the RR scale above would stay
    green."""
    frac = max(0.0, min(frac, 1.5))
    if frac <= 0.7:
        return "#2fa35a"
    if frac <= 1.0:
        return _lerp_color("#2fa35a", "#e08a1e", (frac - 0.7) / 0.3)
    return _lerp_color("#e08a1e", "#d6453d", (frac - 1.0) / 0.5)


def _lerp_color(c0: str, c1: str, t: float) -> str:
    a = tuple(int(c0[i:i + 2], 16) for i in (1, 3, 5))
    b = tuple(int(c1[i:i + 2], 16) for i in (1, 3, 5))
    m = tuple(round(a[i] + (b[i] - a[i]) * t) for i in range(3))
    return f"#{m[0]:02x}{m[1]:02x}{m[2]:02x}"


class ResultsTab(QWidget):
    def __init__(self, mission: dict, result: dict, grid: geo.SphereGrid, parent=None):
        super().__init__(parent)
        self.mission = mission
        self.result = result
        self.partial = bool(result.get("cancelled"))
        self.exported = False                 # set once the result has been exported (closing asks otherwise)
        self.sectors_by_id = {s["id"]: s for s in frames.resolve(mission)[0]["sectors"]}   # ship frame (step 3.0)
        self.tracking_by_id = {s["id"]: s for s in result.get("tracking", {}).get("sectors", [])}
        self.reuse_by_id = (result.get("margin_reuse") or {}).get("sectors", {})       # step 5.2 (P4)
        self.grid = grid
        self.eps_min = result["global"]["eps_min_deg"]
        self.eps_max = result["global"]["eps_max_deg"]
        self._hidden_sectors: set = set()     # ids of the sectors whose volume is hidden in 3D

        self._build_ui()
        self._update_scene()
        self._fill_tables()
        self.table_sectors.itemChanged.connect(self._on_sector_check_changed)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Horizontal)
        outer.addWidget(splitter)

        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        if self.partial:
            n_done, n_total = len(self.result["sectors"]), len(self.mission["sectors"])
            banner = QLabel(f"PARTIAL RESULT: calculation cancelled after {n_done} of {n_total} sectors. "
                            "Sectors not computed are not shown, and the per-face totals only include the "
                            "computed sectors.")
            banner.setWordWrap(True)
            banner.setStyleSheet("background: #d6453d; color: white; font-weight: bold; padding: 6px;")
            lv.addWidget(banner)
        top = QHBoxLayout()
        top.addWidget(QLabel("Colour sectors by:"))
        self.cb_metric = QComboBox()
        self.cb_metric.addItems(["Achieved RR / target RR", "Energy % of this sector (worst face)",
                                 "Energy % of the face total (most loaded face)"])
        self.cb_metric.currentIndexChanged.connect(lambda _i: self._update_scene())
        top.addWidget(self.cb_metric)
        top.addStretch(1)
        lv.addLayout(top)
        self.view = SceneView(self.grid, face_opacity=0.18, sector_opacity=0.55)
        self.view.set_navigation(frames.parse_nav(mission.get("global", {}).get("nav")).r_si)
        self.view.show_all_sectors.connect(lambda: self._set_all_sectors_visible(True))
        self.view.hide_all_sectors.connect(lambda: self._set_all_sectors_visible(False))
        lv.addWidget(self.view)
        splitter.addWidget(left)

        panel = QScrollArea()
        panel.setWidgetResizable(True)
        inner = QWidget()
        panel.setWidget(inner)
        v = QVBoxLayout(inner)

        v.addWidget(self._build_verdict_group())
        v.addWidget(self._build_sector_table_group())
        v.addWidget(self._build_bin_table_group())
        v.addWidget(self._build_face_table_group())
        v.addWidget(self._build_tracking_group())
        v.addWidget(self._build_warnings_group())
        v.addWidget(self._build_export_group())
        v.addStretch(1)
        splitter.addWidget(panel)
        splitter.setChildrenCollapsible(False)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

    def _build_verdict_group(self) -> QGroupBox:
        """Step 5.3 (P1): the mission verdict, the limiting resource and every constraint in alert or failed."""
        box = QGroupBox("Verdict")
        v = QVBoxLayout(box)
        self.lbl_verdict = QLabel("")
        self.lbl_verdict.setWordWrap(True)
        v.addWidget(self.lbl_verdict)
        self.table_verdict = QTableWidget()
        cols = ["state", "object", "constraint", "priority", "load %", "value", "limit", "note"]
        self.table_verdict.setColumnCount(len(cols))
        self.table_verdict.setHorizontalHeaderLabels(cols)
        self.table_verdict.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_verdict.horizontalHeader().setStretchLastSection(True)
        self.table_verdict.setToolTip("Resource: OK up to the load ceiling, ALERT up to 100 % (safety reserve in use), "
                                      "NOK above. Requirement: OK up to 100 %, NOK above. Transient: ALERT above 100 %.")
        v.addWidget(self.table_verdict)
        # step 5.3 (P2): the quantified relaxation advice, estimated; one of them verified on request
        self.table_advice = QTableWidget()
        cols = ["n", "constraint", "kind", "on", "change", "law / note", "verified"]
        self.table_advice.setColumnCount(len(cols))
        self.table_advice.setHorizontalHeaderLabels(cols)
        self.table_advice.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_advice.horizontalHeader().setStretchLastSection(True)
        self.table_advice.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_advice.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table_advice.setToolTip("Estimates from the load laws of each lever; 'Verify' recomputes the mission with "
                                     "the change and bisects for the smallest relaxation that holds (several minutes)")
        v.addWidget(self.table_advice)
        h = QHBoxLayout()
        self.btn_verify = QPushButton("Verify the selected advice")
        self.btn_verify.clicked.connect(self._verify_advice)
        self.lbl_verify = QLabel("")
        self.lbl_verify.setWordWrap(True)
        h.addWidget(self.btn_verify)
        h.addWidget(self.lbl_verify, 1)
        v.addLayout(h)
        self._verify_worker = None
        return box

    def _fill_advice(self) -> None:
        adv = (self.result.get("verdict") or {}).get("advice", [])
        self.table_advice.setRowCount(len(adv))
        self.btn_verify.setEnabled(bool(adv) and not self.partial)
        for i, a in enumerate(adv):
            change = validation.changes_text(a, most=len(a["changes"])) if a["changes"] else "-"
            ver = a.get("verified")
            ver_txt = ("-" if ver is None else "holds: " + ", ".join(f"{c['key']} {c['verified']:.4g}" if isinstance(
                c["verified"], (int, float)) else f"{c['key']} {c['verified']}" for c in ver["changes"])
                if ver["holds"] else "does not hold")
            vals = [str(a["n"]), f"{a['object']} {a['quantity']} ({a['state']})", a["kind"], a["item"] or "-",
                    change, "; ".join(x for x in (a["law"], "approximate" if a["approximate"] else "", a["note"],
                                                 ", ".join(a["others_left"])) if x), ver_txt]
            for col, val in enumerate(vals):
                self.table_advice.setItem(i, col, QTableWidgetItem(val))

    def _verify_advice(self) -> None:
        adv = (self.result.get("verdict") or {}).get("advice", [])
        row = self.table_advice.currentRow()
        if not 0 <= row < len(adv) or not adv[row]["changes"]:
            self.lbl_verify.setText("Select an advice with a change first.")
            return
        a = adv[row]
        self.btn_verify.setEnabled(False)
        self.lbl_verify.setText(f"Verifying advice {a['n']} (each step recomputes the whole mission)...")
        self._verify_worker = VerifyWorker(self.mission, a, self)
        self._verify_worker.progress.connect(lambda s: self.lbl_verify.setText(f"advice {a['n']}: {s}"))
        self._verify_worker.done.connect(lambda res: self._on_verified(a, res))
        self._verify_worker.failed.connect(lambda msg: (self.lbl_verify.setText(msg), self.btn_verify.setEnabled(True)))
        self._verify_worker.start()

    def _on_verified(self, a: dict, res: dict) -> None:
        a["verified"] = res
        self.lbl_verify.setText(f"advice {a['n']}: " + ("holds" if res["holds"] else "does not hold")
                                + f" after {len(res['runs'])} recomputations")
        self.btn_verify.setEnabled(True)
        self._fill_advice()

    def _fill_verdict(self) -> None:
        vd = self.result.get("verdict")
        if not vd:
            self.lbl_verdict.setText("No verdict in this result.")
            self.table_verdict.setRowCount(0)
            return
        lim = vd.get("limiting")
        sat = vd.get("saturation") or {}
        c = vd["counts"]
        txt = (f"{vd['state']}: {c['NOK']} NOK, {c['ALERT']} alert, {c['OK']} OK (load ceiling "
               f"{100 * vd['load_ceiling']:.0f} %)")
        if lim:
            txt += f"\nLimiting resource: {lim['object']} {lim['resource']} at {100 * lim['load']:.1f} %"
        if vd.get("nok_priorities"):
            txt += "\nPriorities failed: " + ", ".join(vd["nok_priorities"])
        txt += f"\nSaturation at the specification: {sat.get('state_at_spec')} (mode {sat.get('mode')})"
        self.lbl_verdict.setText(txt)
        colour = {"OK": "#2fa35a", "ALERT": "#e08a1e", "NOK": "#d6453d"}
        self.lbl_verdict.setStyleSheet(f"color: {colour.get(vd['state'], '#d6453d')}; font-weight: bold;")
        rows = sorted((r for r in vd["constraints"] if r["state"] != "OK"),
                      key=lambda r: (r["state"] != "NOK", r["object"], r["quantity"]))
        self.table_verdict.setRowCount(len(rows))

        def num(x) -> str:
            return f"{x:.4g}" if isinstance(x, (int, float)) and not isinstance(x, bool) else ("-" if x is None else str(x))
        for i, r in enumerate(rows):
            vals = [r["state"], r["object"], r["quantity"], r["priority"] or "-",
                    "-" if r["load"] is None else f"{100 * r['load']:.1f}",
                    num(r["value"]) + (f" {r['unit']}" if r["unit"] and r["value"] is not None else ""),
                    num(r["limit"]), r["note"]]
            for col, val in enumerate(vals):
                item = QTableWidgetItem(val)
                if col == 0:
                    item.setForeground(QColor(colour[r["state"]]))
                self.table_verdict.setItem(i, col, item)

    def _build_sector_table_group(self) -> QGroupBox:
        box = QGroupBox("Per-sector summary")
        v = QVBoxLayout(box)
        self.table_sectors = QTableWidget()
        cols = ["id", "status", "WF", "energy % (worst face)", "time on air % (worst face)",
                "worst range (km)", "best range (km)", "target RR (s)", "RR alone (s)", "RR in mission (s)", "flags"]
        self.table_sectors.setColumnCount(len(cols))
        self.table_sectors.setHorizontalHeaderLabels(cols)
        self.table_sectors.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_sectors.horizontalHeader().setStretchLastSection(True)
        self.table_sectors.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table_sectors.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table_sectors.currentCellChanged.connect(lambda row, *_: self._on_sector_row(row))
        v.addWidget(self.table_sectors)
        return box

    def _build_bin_table_group(self) -> QGroupBox:
        self.grp_bins = QGroupBox("Waveforms by scan-angle bin (select a sector above)")
        v = QVBoxLayout(self.grp_bins)
        self.table_bins = QTableWidget()
        cols = ["scan angle (deg)", "positions", "WF", "integration", "tau (us)", "N", "PRI (ms)", "B (MHz)",
                "TX beams", "dwell (ms)", "range at bin max (km)", "pulse scale"]
        self.table_bins.setColumnCount(len(cols))
        self.table_bins.setHorizontalHeaderLabels(cols)
        self.table_bins.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        v.addWidget(self.table_bins)
        return self.grp_bins

    def _build_face_table_group(self) -> QGroupBox:
        box = QGroupBox("Per-face totals")
        v = QVBoxLayout(box)
        self.table_faces = QTableWidget()
        cols = ["face", "positions", "energy %", "avg power (kW)", "time on air %", "status",
                "tracking margin (kW / Ku duty %)"]
        self.table_faces.setColumnCount(len(cols))
        self.table_faces.setHorizontalHeaderLabels(cols)
        self.table_faces.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_faces.horizontalHeader().setStretchLastSection(True)
        v.addWidget(self.table_faces)
        return box

    def _build_tracking_group(self) -> QGroupBox:
        box = QGroupBox("Search + confirmation + active tracking (AT)")
        v = QVBoxLayout(box)
        self.lbl_trk_verdict = QLabel("")
        self.lbl_trk_verdict.setWordWrap(True)
        v.addWidget(self.lbl_trk_verdict)
        self.table_trk_faces = QTableWidget()
        cols = ["face", "search E %", "confirm E %", "AT E %", "total E %", "S time %", "Ku time %", "status"]
        self.table_trk_faces.setColumnCount(len(cols))
        self.table_trk_faces.setHorizontalHeaderLabels(cols)
        self.table_trk_faces.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_trk_faces.horizontalHeader().setStretchLastSection(True)
        v.addWidget(self.table_trk_faces)
        self.table_trk_sectors = QTableWidget()
        cols = ["id", "confirmation", "reason", "P confirm", "margin (dB)", "false alarms / RR", "Ku looks / target"]
        self.table_trk_sectors.setColumnCount(len(cols))
        self.table_trk_sectors.setHorizontalHeaderLabels(cols)
        self.table_trk_sectors.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_trk_sectors.horizontalHeader().setStretchLastSection(True)
        v.addWidget(self.table_trk_sectors)
        self.grp_at = QGroupBox("AT range brackets (select a sector above)")
        pv = QVBoxLayout(self.grp_at)
        self.table_at = QTableWidget()
        cols = ["bracket (km)", "targets", "RR (s)", "RR held", "zone (deg)", "sR (m)", "sV (m/s)", "J / update", "kW",
                "driver", "pointing (HPBW)", "loss / update", "init looks", "Ku waveform"]
        self.table_at.setColumnCount(len(cols))
        self.table_at.setHorizontalHeaderLabels(cols)
        self.table_at.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table_at.horizontalHeader().setStretchLastSection(True)
        self.table_at.setToolTip("driver = term setting the SNR of a track update: DET (detection), ANG (angle zone), "
                                 "DOP (velocity accuracy), RNG (range accuracy)")
        pv.addWidget(self.table_at)
        v.addWidget(self.grp_at)
        return box

    def _build_warnings_group(self) -> QGroupBox:
        box = QGroupBox("Warnings")
        v = QVBoxLayout(box)
        self.lbl_warnings = QLabel("None" if not self.result.get("warnings") else "")
        self.lbl_warnings.setWordWrap(True)
        v.addWidget(self.lbl_warnings)
        return box

    def _build_export_group(self) -> QGroupBox:
        box = QGroupBox("Export")
        h = QHBoxLayout(box)
        b_json = QPushButton("Export JSON (full result)")
        b_csv = QPushButton("Export CSV (summary)")
        b_json.clicked.connect(self.export_json)
        b_csv.clicked.connect(self._export_csv)
        h.addWidget(b_json)
        h.addWidget(b_csv)
        return box

    # ------------------------------------------------------------------ 3D scene
    def _sector_energy(self, r: dict) -> float:
        """Worst energy fraction of this sector alone, over the faces it uses."""
        return max((fc["energy_fraction"] for fc in r.get("faces", {}).values()), default=0.0)

    def _sector_time(self, r: dict) -> float:
        """Worst time-on-air fraction of this sector alone, over the faces it uses."""
        return max((fc["time_fraction"] for fc in r.get("faces", {}).values()), default=0.0)

    def _face_total_energy(self, r: dict) -> float:
        """Total load (all sectors) of the most loaded face this sector uses."""
        totals = self.result["faces_total"]
        return max((totals[k]["energy_fraction"] for k in r.get("faces", {})), default=0.0)

    def _sector_color(self, r: dict) -> str:
        metric = self.cb_metric.currentIndex()
        if metric == 0:
            rr_t = r.get("rr_target_s") or 1.0
            return _metric_color((r.get("rr_est_mission_s") or rr_t) / rr_t)
        if metric == 1:
            return _energy_color(self._sector_energy(r))
        return _energy_color(self._face_total_energy(r))

    def _sector_text(self, r: dict) -> str:
        sid = r["id"]
        if r["status"] != "OK":
            return f"{sid}\n{r['status']}"
        w = r["wf"]
        rg = r["range_km"]
        n_wf = len(r.get("wf_set", [w]))
        text = (f"{sid}  {w['freq_ghz']} GHz, " +
                (f"{n_wf} WFs" if n_wf > 1 else w["integration"][:4]) + "\n"
                f"range: worst {rg['at_worst_scan']} / best {rg['at_best_scan']} km | RX sidelobes "
                f"{r.get('rx_sll_db', rm.RX_SLL_DEFAULT_DB):g} dB\n"
                f"RR: target {r['rr_target_s']:.0f}s alone {r['rr_alone_s']:.0f}s "
                f"mission {r.get('rr_est_mission_s', r['rr_target_s']):.0f}s\n"
                f"sector: energy {100 * self._sector_energy(r):.0f} % / time {100 * self._sector_time(r):.0f} %"
                f" | face total energy {100 * self._face_total_energy(r):.0f} %")
        mr = self.reuse_by_id.get(sid)
        if mr and mr["g"] > 1.0:
            text += f"\nRR real {mr['rr_real_s']:.3g}s (margin reuse x{mr['g']:.3g}, {mr['limited_by']})"
        trk = self.tracking_by_id.get(sid)
        if trk:
            n_tr = sum(b["n_targets"] for b in trk["at"])
            text += (f"\nconfirm {trk['confirm_mode'] or '-'} ({trk['false_alarms_per_rr']:.0f} FA/RR)"
                     + (f" | AT {n_tr:g} tracks" if trk["at"] else ""))
        if r.get("flags"):
            text += f"\n! {len(r['flags'])} flag(s)"
        return text

    def _update_scene(self) -> None:
        draws = [SectorDraw(r["id"], self.sectors_by_id[r["id"]],
                            self._sector_color(r) if r["status"] == "OK" else "#8a95a3", self._sector_text(r))
                 for r in self.result["sectors"] if r["id"] in self.sectors_by_id]
        self.view.set_scene(self.eps_min, self.eps_max, draws, hidden=self._hidden_sectors)

    # ------------------------------------------------------------------ 3D visibility
    def _on_sector_check_changed(self, item: QTableWidgetItem) -> None:
        if item.column() != 0:
            return
        sid = item.data(Qt.UserRole)
        if item.checkState() == Qt.Checked:
            self._hidden_sectors.discard(sid)
        else:
            self._hidden_sectors.add(sid)
        self.view.set_hidden(self._hidden_sectors)

    def _set_all_sectors_visible(self, visible: bool) -> None:
        self._hidden_sectors = set() if visible else {r["id"] for r in self.result["sectors"]}
        self.table_sectors.blockSignals(True)
        for row in range(self.table_sectors.rowCount()):
            self.table_sectors.item(row, 0).setCheckState(Qt.Checked if visible else Qt.Unchecked)
        self.table_sectors.blockSignals(False)
        self.view.set_hidden(self._hidden_sectors)

    # ------------------------------------------------------------------ tables
    def _fill_tables(self) -> None:
        secs = self.result["sectors"]
        self.table_sectors.setRowCount(len(secs))
        for i, r in enumerate(secs):
            if r["status"] != "OK":
                vals = [r["id"], r["status"], "-", "-", "-", "-", "-", "-", "-", "-", r.get("note", "")]
            else:
                w, rg = r["wf"], r["range_km"]
                n_wf = len(r.get("wf_set", [w]))
                vals = [r["id"], r["status"],
                        (f"{n_wf} WFs, {w['freq_ghz']} GHz" if n_wf > 1 else
                         f"{w['freq_ghz']} GHz {w['integration'][:4]} tau{w['tau_us']}us N{w['n_pulses']}"),
                        f"{100 * self._sector_energy(r):.1f}", f"{100 * self._sector_time(r):.1f}",
                        str(rg["at_worst_scan"]), str(rg["at_best_scan"]),
                        f"{r['rr_target_s']:.0f}", f"{r['rr_alone_s']:.0f}",
                        f"{r.get('rr_est_mission_s', r['rr_target_s']):.0f}",
                        "; ".join(r.get("flags", [])) or "-"]
            for c, val in enumerate(vals):
                item = QTableWidgetItem(str(val))
                if c == 0:   # tick box = show the sector's volume in 3D
                    item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                    item.setCheckState(Qt.Unchecked if r["id"] in self._hidden_sectors else Qt.Checked)
                    item.setData(Qt.UserRole, r["id"])
                    item.setToolTip("Tick / untick to show / hide this sector's volume in the 3D view")
                self.table_sectors.setItem(i, c, item)

        ft = self.result["faces_total"]
        self.table_faces.setRowCount(len(ft))
        for i, (k, v) in enumerate(sorted(ft.items(), key=lambda kv: int(kv[0]))):
            load = max(v["energy_fraction"], v["time_fraction"])
            status = "OK" if load <= 1 else f"OVER-SUBSCRIBED x{load:.2f}"
            margin = (f"{v['p_available_kW']:.0f} kW / {v['ku_duty_pct']:.1f} %" if "p_available_kW" in v else "-")
            vals = [geo.FACE_NAMES[int(k)], f"{v['positions']:.0f}", f"{100 * v['energy_fraction']:.1f}",
                    f"{v['avg_power_kW']:.0f}", f"{100 * v['time_fraction']:.1f}", status, margin]
            for c, val in enumerate(vals):
                self.table_faces.setItem(i, c, QTableWidgetItem(str(val)))

        self._fill_verdict()
        self._fill_advice()
        self._fill_tracking_tables()
        warnings = self.result.get("warnings", [])
        self.lbl_warnings.setText("\n".join(warnings) if warnings else "None")
        if secs:
            self.table_sectors.selectRow(0)
            self._on_sector_row(0)

    def _on_sector_row(self, row: int) -> None:
        self._fill_bin_table(row)
        self._fill_at_table(row)

    def _fill_tracking_tables(self) -> None:
        trk = self.result.get("tracking")
        if not trk:
            self.lbl_trk_verdict.setText("No confirmation / AT budget in this result.")
            return
        faces = [(k, v) for k, v in sorted(trk["faces"].items(), key=lambda kv: int(kv[0]))
                 if self.result["faces_total"][k]["positions"] > 0]
        over = [geo.FACE_NAMES[int(k)] for k, v in faces if v["status"] != "OK"]
        if over:
            self.lbl_trk_verdict.setText("Does NOT fit on " + ", ".join(over) + ":\n" + "\n".join(trk["advice"]))
            self.lbl_trk_verdict.setStyleSheet("color: #d6453d; font-weight: bold;")
        else:
            worst = max((v["load"] for _k, v in faces), default=0.0)
            self.lbl_trk_verdict.setText(f"Search + confirmation + AT fit on every face (most loaded: {100 * worst:.0f} %).")
            self.lbl_trk_verdict.setStyleSheet("color: #2fa35a; font-weight: bold;")
        self.table_trk_faces.setRowCount(len(faces))
        for i, (k, v) in enumerate(faces):
            status = "OK" if v["status"] == "OK" else f"OVER x{v['load']:.2f}"
            vals = [geo.FACE_NAMES[int(k)], f"{100 * v['search_energy']:.1f}", f"{100 * v['confirm_energy']:.1f}",
                    f"{100 * v['at_energy']:.1f}", f"{100 * v['total_energy']:.1f}", f"{100 * v['total_s_time']:.1f}",
                    f"{100 * v['total_ku_time']:.1f}", f"{status} ({v['binding']})"]
            for c, val in enumerate(vals):
                item = QTableWidgetItem(val)
                if c == 7 and v["status"] != "OK":
                    item.setForeground(QColor("#d6453d"))
                self.table_trk_faces.setItem(i, c, item)
        self.table_trk_sectors.setRowCount(len(trk["sectors"]))
        for i, s_ in enumerate(trk["sectors"]):
            c_ = s_["confirmation"]
            m = c_.get("margin_db")
            m_txt = "-" if m is None else (f"S {m[0]:g} / Ku {m[1]:g}" if isinstance(m, list) else f"{m:g}")
            vals = [s_["id"], s_["confirm_mode"] or "-", s_["reason"], f"{c_.get('success', 0):.3f}", m_txt,
                    f"{s_['false_alarms_per_rr']:.0f}", f"{c_.get('ku_looks_true', 0):.2f}"]
            for c, val in enumerate(vals):
                item = QTableWidgetItem(val)
                if c == 2:
                    alts = "\n".join(f"{mm}: " + (f"mission max load {a['mission_max_load']:.3f}" if "mission_max_load" in a
                                                  else "-") + ("" if a["admissible"] else f" (not admissible: {a['why_not']})")
                                     for mm, a in s_["alternatives"].items())
                    item.setToolTip(alts)
                self.table_trk_sectors.setItem(i, c, item)

    def _fill_at_table(self, row: int) -> None:
        secs = self.result["sectors"]
        r = secs[row] if 0 <= row < len(secs) else None
        brackets = self.tracking_by_id[r["id"]]["at"] if r and r["id"] in self.tracking_by_id else []
        self.grp_at.setTitle(f"AT range brackets -- {r['id']}" if r else "AT range brackets (select a sector above)")
        self.table_at.setRowCount(len(brackets))
        for i, b in enumerate(brackets):
            # step 3.3: the operator's period held, or the longest tenable one (alert)
            held = ("yes" if b.get("rr_tenable", True) else
                    f"no, {b['rr_alert_s']:.3g} s" if b.get("rr_alert_s") else "no")
            vals = [f"{b['r_min_km']:g}-{b['r_max_km']:g}", f"{b['n_targets']:g}", f"{b['rr_s']:g}", held,
                    f"{b['angle_zone_deg']:g}", f"{b['range_acc_m']:g}", f"{b['vr_acc_mps']:g}",
                    f"{b['energy_per_update_J']:.3g}", f"{b['avg_power_kW']:.3g}", b["driver"],
                    f"{b.get('pointing_sigma_hpbw', 0.0):.2f}", f"{b.get('p_loss_per_update', 0.0):.2g}",
                    f"{b.get('init_looks') if b.get('init_looks') is not None else '-'}", b["wf"]]
            for c, val in enumerate(vals):
                self.table_at.setItem(i, c, QTableWidgetItem(val))

    def _fill_bin_table(self, row: int) -> None:
        secs = self.result["sectors"]
        r = secs[row] if 0 <= row < len(secs) else None
        bins = r.get("wf_by_bin", []) if r and r["status"] == "OK" else []
        self.grp_bins.setTitle(f"Waveforms by scan-angle bin -- {r['id']}" if r else
                               "Waveforms by scan-angle bin (select a sector above)")
        self.table_bins.setRowCount(len(bins))
        for i, b in enumerate(bins):
            w = b["wf"]
            vals = [f"{b['theta_lo_deg']:.0f}-{b['theta_hi_deg']:.0f}", f"{b['positions']:.0f}", w["name"],
                    w["integration"], w["tau_us"], w["n_pulses"], w["pri_ms"], f"{w['sig_bw_mhz']:.0f}", mo.tx_label(w),
                    w["dwell_ms"], b["range_km_at_bin_max"], f"{b['pulse_scale']:.2f}"]
            for c, val in enumerate(vals):
                self.table_bins.setItem(i, c, QTableWidgetItem(str(val)))

    # ------------------------------------------------------------------ export
    def export_json(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export the result", "mission_report.json",
                                               "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(self.result, fh, indent=2, ensure_ascii=False)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed: {exc}")
            return
        self.exported = True

    def _export_csv(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export the summary", "mission_report.csv",
                                               "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(["id", "status", "freq_ghz", "integration", "tau_us", "n_pulses", "n_wf",
                            "energy_pct_worst_face", "time_pct_worst_face",
                            "range_worst_km", "range_best_km", "rr_target_s", "rr_alone_s",
                            "rr_mission_s", "confirm_mode", "false_alarms_per_rr", "at_tracks", "at_kW", "flags"])
                for r in self.result["sectors"]:
                    if r["status"] != "OK":
                        w.writerow([r["id"], r["status"]] + [""] * 16 + [r.get("note", "")])
                        continue
                    t_ = self.tracking_by_id.get(r["id"], {"confirm_mode": "", "false_alarms_per_rr": "", "at": []})
                    wf_, rg = r["wf"], r["range_km"]
                    w.writerow([r["id"], r["status"], wf_["freq_ghz"], wf_["integration"], wf_["tau_us"],
                                wf_["n_pulses"], len(r.get("wf_set", [wf_])),
                                round(100 * self._sector_energy(r), 2), round(100 * self._sector_time(r), 2),
                                rg["at_worst_scan"], rg["at_best_scan"], r["rr_target_s"],
                                r["rr_alone_s"], r.get("rr_est_mission_s", r["rr_target_s"]),
                                t_["confirm_mode"], t_["false_alarms_per_rr"],
                                sum(b["n_targets"] for b in t_["at"]),
                                round(sum(b["avg_power_kW"] for b in t_["at"]), 2),
                                "; ".join(r.get("flags", []))])
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed: {exc}")
            return
        self.exported = True
