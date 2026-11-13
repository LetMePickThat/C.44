#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TIMELINE WINDOW (step 5.5, point B, framed with the user on 13 November 2026, B1-B6): the synthetic timeline the
scheduler built for an exact result (core/scheduler.replay: the same classes and draws, replayed on demand, nothing
stored in the result), one row per face and band (S, Ku), the transmissions coloured by priority (P0 calibration, P1
self-protection, P2 confirmation and initiation, P3 track updates and burn-through, P4 search, one hue per volume),
the recharge pauses hatched, the late track updates underlined in red; over it, per face and band, the scheduler's
figures (load by priority, longest wait per task, revisit spread, pauses). The model knows the transmit time only:
the receive windows of the echoes are not drawn (B5). Wheel: zoom about the cursor; drag: pan; hover: the task.
"""
from __future__ import annotations

import bisect
import colorsys
import math
from typing import Dict, List, Optional

from PyQt5.QtCore import QPointF, QRectF, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (QDialog, QHBoxLayout, QLabel, QPushButton, QToolTip, QTreeWidget, QTreeWidgetItem,
                             QVBoxLayout, QWidget, QSplitter)

from core.params import P

VIEW_S = float(P.get("tool.display.timeline_view_s"))
PRIO_COLORS = {"cal": "#8a8f99", "P1": "#d6453d", "confirm": "#e08a1e", "init": "#c9a227", "update": "#7b4fd6",
               "burn_through": "#a0522d"}
LATE = "#ff2d2d"
ROW_H, LABEL_W, AXIS_H = 34, 70, 22
NOTE = ("Transmit time only (the scheduler's model): the receive windows of the echoes are not drawn. Synthetic "
        "timeline of the exact result, replayed as the scheduler built it.")


def search_colors(labels: List[str]) -> Dict[str, str]:
    """One hue per search volume (P4), spread on the colour circle away from the other priorities' colours."""
    out = {}
    n = max(1, len(labels))
    for i, lab in enumerate(sorted(labels)):
        r, g, b = colorsys.hsv_to_rgb(0.30 + 0.38 * i / n, 0.55, 0.80)
        out[lab] = QColor(int(255 * r), int(255 * g), int(255 * b)).name()
    return out


def event_color(kind: str, prio, label: str, search: Dict[str, str]) -> str:
    if kind == "search":
        return PRIO_COLORS["P1"] if tuple(prio)[0] == 1 else search.get(label, "#3a8fd6")
    return PRIO_COLORS.get(kind, "#666666")


def pointing_text(e) -> str:
    """The transmission's off-boresight angles on the face's two axes (user, 13 November 2026): per pointing of the
    dwell, az = asin(u) along the face's horizontal axis, el = asin(v) along its vertical one, and the total off-
    boresight angle; tasks the scheduler draws statistically (confirmation, tracks) have no direction."""
    uv = e[7] if len(e) > 7 else None
    if not uv:
        return ("pointing: none (calibration)" if e[3] == "cal" else
                "pointing: not modelled (a task the scheduler draws statistically: its direction is the target's)")
    lines = []
    for u, v in uv:
        az = math.degrees(math.asin(max(-1.0, min(1.0, u))))
        el = math.degrees(math.asin(max(-1.0, min(1.0, v))))
        th = math.degrees(math.asin(min(1.0, math.hypot(u, v))))
        lines.append(f"az {az:+.2f} deg, el {el:+.2f} deg (off boresight {th:.2f} deg)")
    head = "pointing" if len(lines) == 1 else f"{len(lines)} pointings in this dwell"
    return head + ":\n  " + "\n  ".join(lines[:8]) + (f"\n  ... +{len(lines) - 8}" if len(lines) > 8 else "")


_CACHE: Dict[int, dict] = {}       # replays kept per result for the session (id of the result dict)


class ReplayWorker(QThread):
    """The replay in a worker process (user, 13 November 2026: ~1.5 min on MOONTEST_VELOCITY, the sky and the clutter
    of every volume computed again; the window stays fluid), kept per result for the session."""
    done = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, mission: dict, result: dict, parent=None):
        super().__init__(parent)
        self.mission, self.result = mission, result

    def run(self) -> None:
        hit = _CACHE.get(id(self.result))
        if hit is not None:
            self.done.emit(hit)
            return
        try:
            from concurrent.futures import ProcessPoolExecutor
            from core import optimizer as mo, scheduler as sch
            with ProcessPoolExecutor(max_workers=1, initializer=mo._mark_pool_worker) as ex:
                rp = ex.submit(sch.replay_task, self.mission, self.result).result()
            _CACHE[id(self.result)] = rp
            self.done.emit(rp)
        except Exception as exc:  # noqa: BLE001 -- shown in the window
            self.failed.emit(f"timeline not replayed: {exc}")


class Gantt(QWidget):
    """The rows (face x band), zoomable in time."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.rows: List[dict] = []            # {"name", "events" (sorted by start), "starts", "pauses", "focus"}
        self.colors: Dict[str, str] = {}
        self.t0, self.t1, self.t_end = 0.0, VIEW_S, VIEW_S
        self._drag: Optional[float] = None
        self.setMouseTracking(True)
        self.setMinimumHeight(AXIS_H + 8 * ROW_H + 8)

    def set_rows(self, rows: List[dict], colors: Dict[str, str], t_end: float) -> None:
        self.rows, self.colors = rows, colors
        self.t_end = max(t_end, 1e-6)
        self.t0, self.t1 = 0.0, min(VIEW_S, self.t_end)
        self.update()

    # --- coordinates
    def _x(self, t: float) -> float:
        w = max(1.0, self.width() - LABEL_W - 4)
        return LABEL_W + (t - self.t0) / max(self.t1 - self.t0, 1e-12) * w

    def _t(self, x: float) -> float:
        w = max(1.0, self.width() - LABEL_W - 4)
        return self.t0 + (x - LABEL_W) / w * (self.t1 - self.t0)

    def paintEvent(self, _ev) -> None:  # noqa: N802 -- Qt override
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#ffffff"))
        span = self.t1 - self.t0
        # time axis: a 1-2-5 step giving ~10 ticks
        import math
        raw = span / 10.0
        mag = 10.0 ** math.floor(math.log10(raw)) if raw > 0 else 1.0
        step = next(m * mag for m in (1, 2, 5, 10) if m * mag >= raw)
        p.setPen(QPen(QColor("#888888")))
        t = math.ceil(self.t0 / step) * step
        unit, k = ("s", 1.0) if step >= 0.1 else ("ms", 1e3) if step >= 1e-4 else ("us", 1e6)
        while t <= self.t1:
            x = self._x(t)
            p.drawLine(QPointF(x, AXIS_H - 6), QPointF(x, self.height()))
            p.drawText(QRectF(x - 40, 0, 80, AXIS_H - 6), Qt.AlignCenter, f"{t * k:.4g} {unit}")
            t += step
        for i, row in enumerate(self.rows):
            y = AXIS_H + i * ROW_H
            p.setPen(QPen(QColor("#000000")))
            f = p.font()
            f.setBold(bool(row.get("focus")))
            p.setFont(f)
            p.drawText(QRectF(2, y, LABEL_W - 4, ROW_H), Qt.AlignVCenter | Qt.AlignLeft, row["name"])
            p.setPen(QPen(QColor("#e0e0e0")))
            p.drawLine(QPointF(LABEL_W, y + ROW_H - 1), QPointF(self.width(), y + ROW_H - 1))
            p.setPen(Qt.NoPen)
            ev, starts = row["events"], row["starts"]
            j0 = max(0, bisect.bisect_left(starts, self.t0) - 1)
            j1 = bisect.bisect_right(starts, self.t1)
            last_px = -1e9
            for e in ev[j0:j1]:
                x0, x1 = self._x(e[0]), self._x(e[0] + e[1])
                if x1 < LABEL_W:
                    continue
                if x1 - x0 < 1.0 and x0 - last_px < 1.0:      # (sub-pixel crowds: one pixel each)
                    continue
                last_px = x0
                p.setBrush(QColor(event_color(e[3], e[4], e[2], self.colors)))
                p.drawRect(QRectF(max(x0, LABEL_W), y + 5, max(1.0, x1 - max(x0, LABEL_W)), ROW_H - 12))
                if e[6]:
                    p.fillRect(QRectF(max(x0, LABEL_W), y + ROW_H - 6, max(2.0, x1 - x0), 3), QColor(LATE))
            p.setBrush(QBrush(QColor("#555555"), Qt.BDiagPattern))
            for (ts, d) in row["pauses"]:
                if ts + d < self.t0 or ts > self.t1:
                    continue
                x0, x1 = self._x(ts), self._x(ts + d)
                p.drawRect(QRectF(max(x0, LABEL_W), y + 5, max(1.0, x1 - max(x0, LABEL_W)), ROW_H - 12))
        p.end()

    def wheelEvent(self, ev) -> None:  # noqa: N802
        t_c = self._t(ev.pos().x())
        f = 0.8 if ev.angleDelta().y() > 0 else 1.25
        span = min(max((self.t1 - self.t0) * f, 1e-7), self.t_end)
        a = (t_c - self.t0) / max(self.t1 - self.t0, 1e-12)
        self.t0 = min(max(0.0, t_c - a * span), max(0.0, self.t_end - span))
        self.t1 = self.t0 + span
        self.update()

    def mousePressEvent(self, ev) -> None:  # noqa: N802
        self._drag = ev.pos().x()

    def mouseReleaseEvent(self, _ev) -> None:  # noqa: N802
        self._drag = None

    def mouseMoveEvent(self, ev) -> None:  # noqa: N802
        if self._drag is not None:
            dt = (self._drag - ev.pos().x()) / max(1.0, self.width() - LABEL_W) * (self.t1 - self.t0)
            span = self.t1 - self.t0
            self.t0 = min(max(0.0, self.t0 + dt), max(0.0, self.t_end - span))
            self.t1 = self.t0 + span
            self._drag = ev.pos().x()
            self.update()
            return
        i = int((ev.pos().y() - AXIS_H) // ROW_H)
        if not 0 <= i < len(self.rows) or ev.pos().x() < LABEL_W:
            QToolTip.hideText()
            return
        t = self._t(ev.pos().x())
        row = self.rows[i]
        j = bisect.bisect_right(row["starts"], t) - 1
        tol = 3.0 * (self.t1 - self.t0) / max(1.0, self.width() - LABEL_W)
        if j >= 0:
            e = row["events"][j]
            if e[0] - tol <= t <= e[0] + e[1] + tol:
                QToolTip.showText(ev.globalPos(),
                                  f"{row['name']}: {e[2]}\nstart {e[0] * 1e3:.4g} ms, duration {e[1] * 1e6:.4g} us\n"
                                  f"ready at {e[5] * 1e3:.4g} ms, waited {(e[0] - e[5]) * 1e3:.4g} ms"
                                  + ("\nLATE track update" if e[6] else "") + "\n" + pointing_text(e), self)
                return
        for ts, d in row["pauses"]:
            if ts <= t <= ts + d:
                QToolTip.showText(ev.globalPos(), f"{row['name']}: recharge pause {d * 1e3:.4g} ms", self)
                return
        QToolTip.hideText()


class TimelineWindow(QDialog):
    def __init__(self, mission: dict, result: dict, focus_face: Optional[int] = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Timeline (exact result, synthetic scheduler timeline)")
        self.resize(1500, 760)
        self.focus_face = focus_face
        v = QVBoxLayout(self)
        self.lbl = QLabel("Replaying the scheduler's timeline (the sky and the clutter of every volume are computed "
                          "again: from a few seconds up to ~2 min near a body; then kept for this session)...")
        self.lbl.setWordWrap(True)
        v.addWidget(self.lbl)
        split = QSplitter(Qt.Vertical)
        self.summary = QTreeWidget()
        self.summary.setHeaderLabels(["Face / band", "Load", "Detail"])
        self.summary.setColumnWidth(0, 160)
        self.summary.setColumnWidth(1, 90)
        split.addWidget(self.summary)
        self.gantt = Gantt()
        split.addWidget(self.gantt)
        split.setSizes([200, 520])
        v.addWidget(split, 1)
        self.legend = QLabel("")
        self.legend.setWordWrap(True)
        v.addWidget(self.legend)
        h = QHBoxLayout()
        note = QLabel(NOTE)
        note.setWordWrap(True)
        note.setStyleSheet("color: #6b7280; font-size: 11px;")
        h.addWidget(note, 1)
        b_all = QPushButton("Whole horizon")
        b_all.clicked.connect(self._whole)
        h.addWidget(b_all)
        v.addLayout(h)
        self._worker = ReplayWorker(mission, result, self)
        self._worker.done.connect(self._show)
        self._worker.failed.connect(self.lbl.setText)
        self._worker.start()

    def set_focus_face(self, face) -> None:
        """The shared selection's face (5.5 E1): its rows in bold."""
        self.focus_face = face
        for r in self.gantt.rows:
            r["focus"] = face is not None and r["name"].startswith(f"F{face + 1} ")
        self.gantt.update()

    def _whole(self) -> None:
        self.gantt.t0, self.gantt.t1 = 0.0, self.gantt.t_end
        self.gantt.update()

    def _show(self, rp: dict) -> None:
        rows, labels, t_end = [], set(), 0.0
        for k in range(4):
            for b in ("S", "Ku"):
                r = rp.get(f"{k}{b}") or {}
                ev = sorted(r.get("events", []), key=lambda e: e[0])
                labels.update(e[2] for e in ev if e[3] == "search")
                t_end = max([t_end, float(r.get("horizon_s") or 0.0)] + [e[0] + e[1] for e in ev[-1:]])
                rows.append({"name": f"F{k + 1} {b}", "events": ev, "starts": [e[0] for e in ev],
                             "pauses": list(r.get("pauses", [])), "focus": self.focus_face == k})
                self._summary_item(k, b, r.get("summary") or {})
        colors = search_colors(sorted(labels))
        self.gantt.set_rows(rows, colors, t_end)
        self.lbl.setText(f"Horizon {t_end:.3g} s (the longest search revisit of a face, capped); shown first "
                         f"{min(VIEW_S, t_end):g} s. Wheel: zoom about the cursor; drag: pan; hover: the task.")
        parts = [f"<span style='color:{c}'>&#9632;</span> {n}" for n, c in
                 (("P0 calibration", PRIO_COLORS["cal"]), ("P1 self-protection", PRIO_COLORS["P1"]),
                  ("P2 confirmation", PRIO_COLORS["confirm"]), ("P2 initiation", PRIO_COLORS["init"]),
                  ("P3 track updates", PRIO_COLORS["update"]), ("P3 burn-through", PRIO_COLORS["burn_through"]))]
        parts += [f"<span style='color:{c}'>&#9632;</span> P4 {lab}" for lab, c in colors.items()]
        parts += [f"<span style='color:{LATE}'>&#9644;</span> late track update", "hatched: recharge pause"]
        self.legend.setText(" &nbsp; ".join(parts))

    def _summary_item(self, k: int, b: str, s: dict) -> None:
        tl = s.get("timeline") or {}
        load = float(s.get("load", 0.0))
        cap = float(s.get("capacity", 1.0) or 1.0)
        top = QTreeWidgetItem(self.summary, [f"F{k + 1} {b}", f"{100 * load:.2f} %",
                                             f"capacity {100 * cap:.0f} %" + ("" if s.get("feasible", True)
                                                                              else "; NOT FEASIBLE")
                                             + (f"; {tl.get('recharge_pauses', 0)} recharge pause(s), "
                                                f"{1e3 * float(tl.get('pause_time_s', 0.0)):.3g} ms"
                                                if tl.get("recharge_pauses") else "")
                                             + (f"; late track updates {100 * float(tl['late_update_frac']):.2f} %"
                                                if tl.get("late_update_frac") else "")])
        for pr, v in sorted((s.get("load_by_priority") or {}).items()):
            QTreeWidgetItem(top, [f"  {pr}", f"{100 * float(v):.3f} %", "load by priority"])
        md, sp = tl.get("max_delay_s") or {}, tl.get("revisit_spread_s") or {}
        for lab in sorted(md):
            QTreeWidgetItem(top, [f"  {lab}", "", f"longest wait {1e3 * float(md[lab]):.4g} ms"
                                  + (f"; revisit spread {float(sp[lab]):.3g} s" if lab in sp else "")])
        for lab, x in sorted((s.get("stretch") or {}).items()):
            QTreeWidgetItem(top, [f"  {lab}", "", f"stretched x{float(x):.3g} (priority shedding)"])
        top.setExpanded(self.focus_face == k)
