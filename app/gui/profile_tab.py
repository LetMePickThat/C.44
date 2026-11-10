#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PROFILE TAB -- the mission judged along the flight plan of its linked scenario (option A, framed with the user on
10 November 2026, decisions P1 to P5; Choix_architecturaux, last entry of 10 November).

The profile (core/time_profile.py, pass T1) evaluates the mission at the instants sampled along the flight plan (P1),
each instant with its own optimal plan (the final allocation rule on the fast estimate), and judges each watched volume
by its availability (P2, P3); the instants where the fast estimate flags a loss are recomputed exactly (P4, P5, pass
T3). This tab (pass T2) shows it:
  - per volume, a timeline coloured by state (held / degraded / lost, from the resources) with, under it, the share of
    the volume the faces cover at each instant (grey: the coverage hole is a measure of its own, user 10 November
    2026), and its availability (shares of the time, longest outage against its revisit, covered share);
  - per face, the loads (energy, S time, Ku time) along the flight plan, the load ceiling dotted;
  - the events (state changes, instants flagged for the exact computation);
  - a click on a timeline or an event sets the instant shown by the mission preparation (instant_picked).

Profile format (produced by core/time_profile.compute):
  {"t_s": [instants], "duration_s": float, "load_ceiling": float,
   "volumes": {id: {"state": ["held" | "degraded" | "lost"], "rho": [...], "rr_s": [...], "hole": [...],
                    "rr_asked_s": float}},
   "faces": {"0".."3": {"energy": [...], "S time": [...], "Ku time": [...]}},
   "availability": {id: {"held": share, "degraded": share, "lost": share, "longest_outage_s": s,
                         "outage_over_rr": ratio, "covered_mean": share, "covered_min": share}},
   "events": [{"t_s": t, "object": id, "what": text, "flagged": bool}],
   "exact": {str(t_s): {"verdict": state, ...}}}
"""
from __future__ import annotations

import json
import os
from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import QRectF, QThread, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (QHBoxLayout, QHeaderView, QLabel, QProgressBar, QPushButton, QSplitter, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)

from core import geometry as geo

STATE_COLORS = {"held": "#2fa35a", "degraded": "#c8551e", "lost": "#d6453d"}   # the verdict's colours (R3d)
NO_DATA = "#bdbdbd"
RESOURCES = ("energy", "S time", "Ku time")
ROW_H = 18                                       # px per timeline row
LABEL_W = 110                                    # px of the row labels


class ProfileWorker(QThread):
    """core.time_profile.compute off the UI thread, with progress and a cooperative cancel."""
    progress = pyqtSignal(int, int)
    finished_ok = pyqtSignal(dict)
    failed = pyqtSignal(str)

    def __init__(self, mission: dict, link, parent=None):
        super().__init__(parent)
        self.mission, self.link = mission, link
        self.cancelled = False

    def run(self) -> None:
        try:
            from core import time_profile
            prof = time_profile.compute(self.mission, self.link, progress_cb=lambda d, n: self.progress.emit(d, n),
                                        cancel_cb=lambda: self.cancelled)
        except Exception as exc:  # noqa: BLE001 -- shown in the tab
            self.failed.emit(f"{type(exc).__name__}: {exc}")
            return
        self.finished_ok.emit(prof)


class Timeline(QWidget):
    """Rows over the flight plan's duration: one per volume (coloured by state), one per face and resource (the load
    as a line, the ceiling dotted); a click emits the instant under the pointer."""
    picked = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.prof: Optional[dict] = None
        self.cursor_t: Optional[float] = None
        self.setMouseTracking(True)
        self.setMinimumHeight(4 * ROW_H)

    def set_profile(self, prof: Optional[dict]) -> None:
        self.prof = prof
        n = 0 if prof is None else len(prof.get("volumes", {})) + 4 * len(RESOURCES) + 2
        self.setMinimumHeight(max(4, n) * ROW_H + 8)
        self.update()

    def set_cursor(self, t: Optional[float]) -> None:
        self.cursor_t = t
        self.update()

    def _x(self, t: float) -> float:
        dur = max(float((self.prof or {}).get("duration_s") or 1.0), 1e-9)
        return LABEL_W + (self.width() - LABEL_W - 6) * min(max(t / dur, 0.0), 1.0)

    def _t(self, x: float) -> float:
        dur = float((self.prof or {}).get("duration_s") or 0.0)
        w = max(self.width() - LABEL_W - 6, 1)
        return dur * min(max((x - LABEL_W) / w, 0.0), 1.0)

    def mousePressEvent(self, event) -> None:
        if self.prof and event.x() >= LABEL_W:
            self.picked.emit(self._t(event.x()))

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("white"))
        prof = self.prof
        if not prof or not prof.get("t_s"):
            p.setPen(QColor("#777777"))
            p.drawText(self.rect(), Qt.AlignCenter, "No profile yet: compute it (a linked scenario is needed).")
            return
        ts = [float(t) for t in prof["t_s"]]
        dur = float(prof.get("duration_s") or ts[-1])
        edges = [0.0] + [0.5 * (a + b) for a, b in zip(ts, ts[1:])] + [dur]   # each instant holds until the next
        y = 4
        for vid, v in prof.get("volumes", {}).items():
            p.setPen(QColor("#222222"))
            p.drawText(QRectF(2, y, LABEL_W - 6, ROW_H), Qt.AlignVCenter | Qt.AlignLeft, str(vid))
            holes = v.get("hole") or []
            for i, st in enumerate(v.get("state", [])):
                x0, x1 = self._x(edges[i]), self._x(edges[i + 1])
                p.fillRect(QRectF(x0, y + 2, max(x1 - x0, 1.0), ROW_H - 9), QColor(STATE_COLORS.get(st, NO_DATA)))
                cov = 1.0 - min(max(float(holes[i]), 0.0), 1.0) if i < len(holes) and holes[i] is not None else 1.0
                p.fillRect(QRectF(x0, y + ROW_H - 6, max(x1 - x0, 1.0), 4), QColor("#eeeeee"))   # coverage band
                p.fillRect(QRectF(x0, y + ROW_H - 6, max((x1 - x0), 1.0), 4),
                           QColor(int(255 * (1 - cov) + 90 * cov), int(255 * (1 - cov) + 90 * cov),
                                  int(255 * (1 - cov) + 90 * cov)))
            y += ROW_H
        y += ROW_H // 2
        ceil = float(prof.get("load_ceiling") or 0.0)
        for k in range(4):
            f = prof.get("faces", {}).get(str(k), {})
            for res in RESOURCES:
                vals = f.get(res) or []
                p.setPen(QColor("#222222"))
                p.drawText(QRectF(2, y, LABEL_W - 6, ROW_H), Qt.AlignVCenter | Qt.AlignLeft,
                           f"{geo.FACE_NAMES[k]} {res}")
                p.setPen(QPen(QColor("#dddddd"), 1))
                p.drawLine(int(LABEL_W), int(y + ROW_H - 2), int(self.width() - 6), int(y + ROW_H - 2))
                top = max([1.0, ceil] + [float(x) for x in vals if x is not None])
                if ceil > 0:
                    yc = y + ROW_H - 2 - (ROW_H - 4) * ceil / top
                    p.setPen(QPen(QColor("#999999"), 1, Qt.DotLine))
                    p.drawLine(int(LABEL_W), int(yc), int(self.width() - 6), int(yc))
                p.setPen(QPen(QColor(geo.FACE_COLORS[k]), 1.5))
                pts = [(self._x(t), y + ROW_H - 2 - (ROW_H - 4) * float(x) / top) for t, x in zip(ts, vals)
                       if x is not None]
                for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
                    p.drawLine(int(xa), int(ya), int(xb), int(yb))
                y += ROW_H
        for e in prof.get("events", []):                        # flagged instants: a mark on the time axis
            if e.get("flagged"):
                p.setPen(QPen(QColor("#2b579a"), 1, Qt.DashLine))
                x = self._x(float(e["t_s"]))
                p.drawLine(int(x), 2, int(x), self.height() - 2)
        if self.cursor_t is not None:
            p.setPen(QPen(QColor("#000000"), 1))
            x = self._x(self.cursor_t)
            p.drawLine(int(x), 0, int(x), self.height())


class ProfileTab(QWidget):
    """The Profile mode (module docstring). source() -> (mission, link, mission path) of the mission preparation."""
    instant_picked = pyqtSignal(float)

    def __init__(self, source: Callable[[], tuple], parent=None):
        super().__init__(parent)
        self._source = source
        self.prof: Optional[dict] = None
        self._worker: Optional[ProfileWorker] = None
        self.b_compute = QPushButton("Compute profile")
        self.b_compute.setToolTip("The mission at the instants sampled along the linked scenario's flight plan, each "
                                  "with its own plan (fast estimate), and the exact computation where it flags a loss")
        self.b_compute.clicked.connect(self.compute)
        self.b_cancel = QPushButton("Cancel")
        self.b_cancel.clicked.connect(self.cancel)
        self.b_cancel.setVisible(False)
        self.bar = QProgressBar()
        self.bar.setVisible(False)
        self.status = QLabel("")
        head = QHBoxLayout()
        head.addWidget(self.b_compute)
        head.addWidget(self.b_cancel)
        head.addWidget(self.bar, 1)
        head.addWidget(self.status, 2)
        self.timeline = Timeline()
        self.timeline.picked.connect(self._pick)
        self.table_avail = QTableWidget(0, 8)
        self.table_avail.setHorizontalHeaderLabels(["Volume", "Held", "Degraded", "Lost", "Longest outage",
                                                    "Outage / RR", "Covered (mean)", "Covered (min)"])
        self.table_avail.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_avail.verticalHeader().setVisible(False)
        self.table_events = QTableWidget(0, 4)
        self.table_events.setHorizontalHeaderLabels(["t (s)", "Object", "Event", "Exact"])
        self.table_events.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_events.verticalHeader().setVisible(False)
        self.table_events.cellClicked.connect(self._on_event)
        lower = QSplitter(Qt.Horizontal)
        lower.addWidget(self.table_avail)
        lower.addWidget(self.table_events)
        split = QSplitter(Qt.Vertical)
        split.addWidget(self.timeline)
        split.addWidget(lower)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        lay = QVBoxLayout(self)
        lay.addLayout(head)
        lay.addWidget(split, 1)
        self.refresh_availability()

    # ------------------------------------------------------------------ computation
    def refresh_availability(self) -> None:
        """The compute button usable with a linked scenario and the profile computation in place; a profile kept
        beside the mission (P: '<mission>.profile.json') loaded when it matches."""
        mission, link, path = self._source()
        try:
            from core import time_profile  # noqa: F401
            ready = True
        except ImportError:
            ready = False
        self.b_compute.setEnabled(ready and link is not None and self._worker is None)
        if not ready:
            self.status.setText("Profile computation not available yet (pass T1).")
        elif link is None:
            self.status.setText("Link a computed scenario in the mission preparation to compute a profile.")
        elif self.prof is None:
            self._load_beside(mission, link, path)

    def _profile_path(self, path: Optional[str]) -> Optional[str]:
        return None if not path else os.path.splitext(path)[0] + ".profile.json"

    def _load_beside(self, mission: dict, link, path: Optional[str]) -> None:
        pp = self._profile_path(path)
        if not pp or not os.path.isfile(pp):
            return
        try:
            from core import time_profile
            with open(pp, encoding="utf-8") as fh:
                saved = json.load(fh)
            if saved.get("key") == time_profile.key(mission, link):
                self.show_profile(saved["profile"], f"profile loaded from {os.path.basename(pp)}")
        except (OSError, ValueError, KeyError, ImportError, AttributeError):
            pass

    def compute(self) -> None:
        mission, link, _path = self._source()
        if link is None or self._worker is not None:
            return
        self._worker = ProfileWorker(json.loads(json.dumps(mission, default=str)), link, self)
        self._worker.progress.connect(self._on_progress)
        self._worker.finished_ok.connect(self._on_done)
        self._worker.failed.connect(self._on_failed)
        self.b_compute.setEnabled(False)
        self.b_cancel.setVisible(True)
        self.bar.setVisible(True)
        self.bar.setRange(0, 0)
        self.status.setText("computing the profile...")
        self._worker.start()

    def cancel(self) -> None:
        if self._worker is not None:
            self._worker.cancelled = True
            self.status.setText("cancelling...")

    def _on_progress(self, done: int, total: int) -> None:
        self.bar.setRange(0, max(total, 1))
        self.bar.setValue(done)
        self.status.setText(f"instant {done} / {total}")

    def _finish(self) -> None:
        self._worker = None
        self.b_cancel.setVisible(False)
        self.bar.setVisible(False)
        self.refresh_availability()

    def _on_failed(self, msg: str) -> None:
        self._finish()
        self.status.setText(f"profile failed: {msg}")

    def _on_done(self, prof: dict) -> None:
        cancelled = bool(prof.get("cancelled"))
        self._finish()
        self.show_profile(prof, "profile cancelled (partial)" if cancelled else
                          f"profile: {len(prof.get('t_s', []))} instants")
        mission, link, path = self._source()
        pp = self._profile_path(path)
        if pp and not cancelled:
            try:
                from core import time_profile
                with open(pp, "w", encoding="utf-8") as fh:
                    json.dump({"key": time_profile.key(mission, link), "profile": prof}, fh)
            except (OSError, ImportError, AttributeError, TypeError, ValueError):
                pass

    # ------------------------------------------------------------------ display
    def show_profile(self, prof: Optional[dict], note: str = "") -> None:
        self.prof = prof
        self.timeline.set_profile(prof)
        self.table_avail.setRowCount(0)
        self.table_events.setRowCount(0)
        if prof is None:
            return
        av = prof.get("availability", {})
        self.table_avail.setRowCount(len(av))
        for i, (vid, a) in enumerate(av.items()):
            cells = [str(vid), f"{100 * a.get('held', 0.0):.1f} %", f"{100 * a.get('degraded', 0.0):.1f} %",
                     f"{100 * a.get('lost', 0.0):.1f} %", f"{a.get('longest_outage_s', 0.0):.0f} s",
                     f"{a.get('outage_over_rr', 0.0):.2g}", f"{100 * a.get('covered_mean', 1.0):.1f} %",
                     f"{100 * a.get('covered_min', 1.0):.1f} %"]
            for j, c in enumerate(cells):
                item = QTableWidgetItem(c)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                if j in (1, 2, 3):
                    item.setForeground(QColor(STATE_COLORS[("held", "degraded", "lost")[j - 1]]))
                self.table_avail.setItem(i, j, item)
        ev = prof.get("events", [])
        self.table_events.setRowCount(len(ev))
        for i, e in enumerate(ev):
            ex = (prof.get("exact") or {}).get(str(e.get("t_s")))
            cells = [f"{float(e['t_s']):.1f}", str(e.get("object", "")), str(e.get("what", "")),
                     (ex or {}).get("verdict", "flagged" if e.get("flagged") else "")]
            for j, c in enumerate(cells):
                item = QTableWidgetItem(c)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table_events.setItem(i, j, item)
        if note:
            self.status.setText(note)

    def set_cursor(self, t: Optional[float]) -> None:
        self.timeline.set_cursor(t)

    def _pick(self, t: float) -> None:
        self.timeline.set_cursor(t)
        self.instant_picked.emit(float(t))

    def _on_event(self, row: int, _col: int) -> None:
        ev = (self.prof or {}).get("events", [])
        if 0 <= row < len(ev):
            self._pick(float(ev[row]["t_s"]))

    def close_worker(self) -> None:
        if self._worker is not None:
            self._worker.cancelled = True
            self._worker.wait(5000)
