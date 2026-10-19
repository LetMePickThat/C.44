#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RESULTS_WINDOW -- Window 2: optimisation result. Same 3D rendering as the mission-preparation
window, but sectors are coloured by result (achieved RR / target RR, or % energy), with
WF / range / RR / energy / flag labels; per-sector summary table and per-face totals; JSON/CSV export.
"""
from __future__ import annotations

import csv
import json
from typing import Dict, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractItemView, QComboBox, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QMainWindow, QMessageBox,
    QPushButton, QScrollArea, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)
from pyvistaqt import QtInteractor

import geometry as geo
from view_controls import DisplayBar

GRID_RES_DEG = 0.5


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


class ResultsWindow(QMainWindow):
    def __init__(self, mission: dict, result: dict):
        super().__init__()
        self.mission = mission
        self.result = result
        self.partial = bool(result.get("cancelled"))
        self.setWindowTitle("Optimisation result" + (" -- PARTIAL (calculation cancelled)" if self.partial else ""))
        self.resize(1400, 860)
        self.sectors_by_id = {s["id"]: s for s in mission["sectors"]}
        self.grid = geo.SphereGrid(res_deg=GRID_RES_DEG)
        self.eps_min = result["global"]["eps_min_deg"]
        self.eps_max = result["global"]["eps_max_deg"]
        self._hidden_sectors: set = set()     # ids of the sectors whose volume is hidden in 3D
        self._overlay_meshes: dict = {}       # (sid, 1:1 scale) -> (surface, outline), built once

        self._build_ui()
        geo.add_static_scene(self.plotter)
        self._rebuild_face_sectors()
        self._rebuild_sector_overlays()
        self._fill_tables()
        self.table_sectors.itemChanged.connect(self._on_sector_check_changed)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        splitter = QSplitter(Qt.Horizontal)
        self.setCentralWidget(splitter)

        left = QWidget()
        lv = QVBoxLayout(left)
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
        self.cb_metric.currentIndexChanged.connect(self._rebuild_sector_overlays)
        top.addWidget(self.cb_metric)
        top.addStretch(1)
        lv.addLayout(top)
        self.display_bar = DisplayBar()
        self.display_bar.faces_changed.connect(self._apply_visibility)
        self.display_bar.show_all_sectors.connect(lambda: self._set_all_sectors_visible(True))
        self.display_bar.hide_all_sectors.connect(lambda: self._set_all_sectors_visible(False))
        self.display_bar.scale_changed.connect(self._on_scale_changed)
        self.display_bar.scale_axes_changed.connect(lambda _on: self._apply_visibility())
        lv.addWidget(self.display_bar)
        self.plotter = QtInteractor(left)
        lv.addWidget(self.plotter.interactor)
        splitter.addWidget(left)

        panel = QScrollArea()
        panel.setWidgetResizable(True)
        panel.setMaximumWidth(680)
        inner = QWidget()
        panel.setWidget(inner)
        v = QVBoxLayout(inner)

        v.addWidget(self._build_sector_table_group())
        v.addWidget(self._build_bin_table_group())
        v.addWidget(self._build_face_table_group())
        v.addWidget(self._build_warnings_group())
        v.addWidget(self._build_export_group())
        v.addStretch(1)
        splitter.addWidget(panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

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
        self.table_sectors.currentCellChanged.connect(lambda row, *_: self._fill_bin_table(row))
        v.addWidget(self.table_sectors)
        return box

    def _build_bin_table_group(self) -> QGroupBox:
        self.grp_bins = QGroupBox("Waveforms by scan-angle bin (select a sector above)")
        v = QVBoxLayout(self.grp_bins)
        self.table_bins = QTableWidget()
        cols = ["scan angle (deg)", "positions", "WF", "integration", "tau (us)", "N", "PRI (ms)", "B (MHz)",
                "Btx", "dwell (ms)", "range at bin max (km)", "power scale"]
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
        b_json.clicked.connect(self._export_json)
        b_csv.clicked.connect(self._export_csv)
        h.addWidget(b_json)
        h.addWidget(b_csv)
        return box

    # ------------------------------------------------------------------ 3D scene
    def _rebuild_face_sectors(self) -> None:
        scene = geo.build_face_sectors(self.grid, self.eps_min, self.eps_max)
        for k in range(4):
            if scene.surfaces[k].n_points:
                self.plotter.add_mesh(scene.surfaces[k], color=geo.FACE_COLORS[k], opacity=0.18,
                                       culling="back", name=f"face_surf_{k}", render=False)
            if scene.outlines[k].n_points:
                self.plotter.add_mesh(scene.outlines[k], color=geo.COLORS["fg"], line_width=1,
                                       name=f"face_outline_{k}", render=False)

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

    def _rebuild_sector_overlays(self, *_args) -> None:
        for r in self.result["sectors"]:
            sid = r["id"]
            sec = self.sectors_by_id.get(sid)
            surf_name, out_name, lbl_name = geo.sector_actor_names(sid)
            if sec is None:
                continue
            true_scale = self.display_bar.true_scale()
            if (sid, true_scale) not in self._overlay_meshes:   # geometry is fixed here: a metric change only recolours
                build = geo.build_sector_volume_km if true_scale else geo.build_sector_overlay
                self._overlay_meshes[sid, true_scale] = build(self.grid, sec, self.eps_min, self.eps_max)
            poly, outline = self._overlay_meshes[sid, true_scale]
            if r["status"] != "OK":
                color = "#8a95a3"
            else:
                color = self._sector_color(r)
            if poly.n_points:
                self.plotter.add_mesh(poly, color=color, opacity=0.55, culling=None if true_scale else "back",
                                      name=surf_name, render=False)
            if outline.n_points:
                self.plotter.add_mesh(outline, color=color, line_width=2, name=out_name, render=False)
            centre = poly.points.mean(axis=0) if poly.n_points else None
            if centre is None:
                continue
            if r["status"] != "OK":
                text = f"{sid}\n{r['status']}"
            else:
                w = r["wf"]
                rg = r["range_km"]
                n_wf = len(r.get("wf_set", [w]))
                text = (f"{sid}  {w['freq_ghz']} GHz, " +
                        (f"{n_wf} WFs" if n_wf > 1 else w["integration"][:4]) + "\n"
                        f"range: worst {rg['at_worst_scan']} / best {rg['at_best_scan']} km\n"
                        f"RR: target {r['rr_target_s']:.0f}s alone {r['rr_alone_s']:.0f}s "
                        f"mission {r.get('rr_est_mission_s', r['rr_target_s']):.0f}s\n"
                        f"sector: energy {100 * self._sector_energy(r):.0f} % / time {100 * self._sector_time(r):.0f} %"
                        f" | face total energy {100 * self._face_total_energy(r):.0f} %")
                if r.get("flags"):
                    text += f"\n! {len(r['flags'])} flag(s)"
            self.plotter.add_point_labels([centre], [text], name=lbl_name, text_color=color,
                                           font_size=10, shape_opacity=0.6, show_points=False, render=False)
        ranges = [self.sectors_by_id[r["id"]]["range_km"] for r in self.result["sectors"] if r["id"] in self.sectors_by_id]
        geo.set_km_axes(self.plotter, max(ranges, default=None) if self.display_bar.true_scale() else None)
        self._apply_visibility()   # the one render of this rebuild

    def _on_scale_changed(self, true_scale: bool) -> None:
        """1:1 on: redraw the volumes in km (radar out to the required range) and zoom to fit them all; off: back to
        the default unit-sphere view."""
        self._rebuild_sector_overlays()
        if true_scale:
            geo.fit_camera_to_actors(self.plotter, [geo.sector_actor_names(r["id"])[0] for r in self.result["sectors"]]
                                     + (geo.KM_AXES_LINES if self.display_bar.scale_axes_visible() else []))
        else:
            geo.set_default_camera(self.plotter)
        self._apply_visibility()

    # ------------------------------------------------------------------ 3D visibility
    def _on_sector_check_changed(self, item: QTableWidgetItem) -> None:
        if item.column() != 0:
            return
        sid = item.data(Qt.UserRole)
        if item.checkState() == Qt.Checked:
            self._hidden_sectors.discard(sid)
        else:
            self._hidden_sectors.add(sid)
        self._apply_visibility()

    def _set_all_sectors_visible(self, visible: bool) -> None:
        self._hidden_sectors = set() if visible else {r["id"] for r in self.result["sectors"]}
        self.table_sectors.blockSignals(True)
        for row in range(self.table_sectors.rowCount()):
            self.table_sectors.item(row, 0).setCheckState(Qt.Checked if visible else Qt.Unchecked)
        self.table_sectors.blockSignals(False)
        self._apply_visibility()

    def _apply_visibility(self) -> None:
        """Re-applied after every rebuild (colour metric change), since rebuilt actors come back visible."""
        for k in range(4):
            geo.set_actors_visible(self.plotter, geo.face_actor_names(k),
                                   self.display_bar.face_visible(k) and not self.display_bar.true_scale())
        geo.set_reference_scene_visible(self.plotter, not self.display_bar.true_scale())
        geo.set_km_axes_visible(self.plotter, self.display_bar.scale_axes_visible())
        for r in self.result["sectors"]:
            geo.set_actors_visible(self.plotter, geo.sector_actor_names(r["id"]), r["id"] not in self._hidden_sectors)
        self.plotter.render()

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

        warnings = self.result.get("warnings", [])
        self.lbl_warnings.setText("\n".join(warnings) if warnings else "None")
        if secs:
            self.table_sectors.selectRow(0)
            self._fill_bin_table(0)

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
                    w["integration"], w["tau_us"], w["n_pulses"], w["pri_ms"], f"{w['sig_bw_mhz']:.0f}", w["b_tx"],
                    w["dwell_ms"], b["range_km_at_bin_max"], f"{b['power_scale']:.2f}"]
            for c, val in enumerate(vals):
                self.table_bins.setItem(i, c, QTableWidgetItem(str(val)))

    # ------------------------------------------------------------------ export
    def _export_json(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export the result", "mission_report.json",
                                               "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(self.result, fh, indent=2, ensure_ascii=False)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed: {exc}")

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
                            "rr_mission_s", "flags"])
                for r in self.result["sectors"]:
                    if r["status"] != "OK":
                        w.writerow([r["id"], r["status"]] + [""] * 12 + [r.get("note", "")])
                        continue
                    wf_, rg = r["wf"], r["range_km"]
                    w.writerow([r["id"], r["status"], wf_["freq_ghz"], wf_["integration"], wf_["tau_us"],
                                wf_["n_pulses"], len(r.get("wf_set", [wf_])),
                                round(100 * self._sector_energy(r), 2), round(100 * self._sector_time(r), 2),
                                rg["at_worst_scan"], rg["at_best_scan"], r["rr_target_s"],
                                r["rr_alone_s"], r.get("rr_est_mission_s", r["rr_target_s"]),
                                "; ".join(r.get("flags", []))])
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed: {exc}")
