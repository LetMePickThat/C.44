#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The "Diagnosis" panel of the mission preparation (step 5.5, point A, user 13 November 2026, Q1-Q5): under the live
load, what limits the mission while it is edited (core/diagnosis.live on the fast estimate). One branch for the
search volumes (range and revisit served, what limits them), one per face and one for the ship: every resource with
its load in the nominal view (no target present) and at the peak (every track present), the worst first; under a
constraint, who weighs on it and the estimated levers (to verify by Calculate). A click on a line naming a volume
selects it (sector_selected).
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor
from PyQt5.QtWidgets import (QHBoxLayout, QLabel, QToolButton, QTreeWidget, QTreeWidgetItem, QVBoxLayout,
                             QWidget)

STATE_COLORS = {"OK": "#2fa35a", "ALERT": "#c8551e", "NOK": "#d6453d"}   # (the verdict's colours, profile_tab)
MUTED = "#8a8f98"
TREE_MIN_HEIGHT = 60          # (its height set with the mouse: the splitter above it, user 13 November 2026)
_ROLE_SID = Qt.UserRole


def _cell(load: Optional[float], state: Optional[str]) -> Tuple[str, Optional[str]]:
    if load is None:
        return "-", None
    pct = f"{100 * load:.0f}" if load >= 0.01 or load == 0 else f"{100 * load:.2g}"
    return f"{pct} %  {state}", STATE_COLORS.get(state)


class DiagnosisPanel(QWidget):
    sector_selected = pyqtSignal(str)
    face_selected = pyqtSignal(int)                # (5.5 E1: a face's branch clicked)
    folded = pyqtSignal(bool)                     # (the owner shrinks the part under its splitter)

    def __init__(self, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(4, 0, 4, 2)
        v.setSpacing(1)
        h = QHBoxLayout()
        self.btn = QToolButton()
        self.btn.setCheckable(True)
        self.btn.setChecked(True)
        self.btn.setArrowType(Qt.DownArrow)
        self.btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.btn.setText("Diagnosis")
        self.btn.setToolTip("What limits the mission, from the live estimate: per face every resource it holds (energy, "
                            "S time, Ku time, memory bandwidth, processing), nominal (no target) and at the peak (every "
                            "track present); who weighs on it; estimated levers, to verify by Calculate")
        self.btn.toggled.connect(self._toggle)
        h.addWidget(self.btn)
        self.lbl = QLabel("")
        self.lbl.setStyleSheet(f"color: {MUTED}; font-size: 11px;")
        h.addWidget(self.lbl, 1)
        v.addLayout(h)
        self.tree = QTreeWidget()
        self.tree.setColumnCount(4)
        self.tree.setHeaderLabels(["Constraint", "Nominal", "Peak", "Detail"])
        self.tree.setMinimumHeight(TREE_MIN_HEIGHT)
        self.tree.setColumnWidth(0, 230)
        self.tree.setColumnWidth(1, 95)
        self.tree.setColumnWidth(2, 95)
        self.tree.itemClicked.connect(self._clicked)
        v.addWidget(self.tree)
        self._header = h
        self._open: Dict[str, bool] = {}               # (the branches the operator opened stay open on a refresh)

    def _toggle(self, on: bool) -> None:
        self.btn.setArrowType(Qt.DownArrow if on else Qt.RightArrow)
        self.tree.setVisible(on)
        # folded: only its header line (the splitter gives the view the rest); unfolded: any height again
        self.setMaximumHeight(16777215 if on else self._header.sizeHint().height() + 4)
        self.folded.emit(not on)

    def _clicked(self, item: QTreeWidgetItem, _col: int) -> None:
        sid = item.data(0, _ROLE_SID)
        if sid:
            self.sector_selected.emit(str(sid))
            return
        top = item
        while top.parent() is not None:
            top = top.parent()
        name = top.text(0)
        if len(name) == 2 and name[0] == "F" and name[1] in "1234":
            self.face_selected.emit(int(name[1]) - 1)

    def set_selected(self, kind: str, ident: str) -> None:
        """The shared selection shown (5.5 E1): the volume's line, or the face's branch, made current."""
        self._sel = (kind, ident)
        target = None
        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            if kind == "face" and top.text(0) == f"F{int(ident) + 1}":
                target = top
            if kind == "sector" and top.text(0).startswith("Volumes"):
                for j in range(top.childCount()):
                    if top.child(j).data(0, _ROLE_SID) == ident:
                        target = top.child(j)
        if target is None:
            self.tree.clearSelection()
            return
        self.tree.setCurrentItem(target)
        self.tree.scrollToItem(target)

    def _item(self, parent, texts, colors=(), sid=None, tip: str = "") -> QTreeWidgetItem:
        it = QTreeWidgetItem(parent, [str(t) for t in texts])
        for j, c in enumerate(colors):
            if c:
                it.setForeground(j, QBrush(QColor(c)))
        if sid is not None:
            it.setData(0, _ROLE_SID, str(sid))
        if tip:
            for j in range(len(texts)):
                it.setToolTip(j, tip)
        return it

    def set_diagnosis(self, d: Optional[dict]) -> None:
        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            self._open[top.text(0).split("  ")[0]] = top.isExpanded()
        self.tree.clear()
        if not d:
            self.lbl.setText("no live estimate")
            return
        n_bad = sum(1 for r in d["rows"] if r["state"] != "OK") + sum(1 for v in d["volumes"] if v["state"] != "OK")
        self.lbl.setText(f"{d['state']}: {n_bad} constraint(s) or volume(s) in alert or failed; "
                         f"estimated (Calculate verifies); {d['not_covered']}")
        # --- the search volumes
        vols = d["volumes"]
        st_v = max([v["state"] for v in vols] + ["OK"], key=["OK", "ALERT", "NOK"].index)
        top = self._item(self.tree, [f"Volumes  ({len(vols)})", st_v, "", ""], (None, STATE_COLORS[st_v]))
        for v in vols:
            det = []
            if v["rho"] < 1.0 - 1e-6:
                det.append(f"range {v['range_held_km']:.0f} / {v['range_asked_km']:.0f} km")
            if v["rr_served_s"] > v["rr_asked_s"] * (1 + 1e-6):
                det.append(f"revisit {v['rr_served_s']:.3g} s (asked {v['rr_asked_s']:g} s)")
            elif v["rr_served_s"] < v["rr_asked_s"] * (1 - 1e-6):
                det.append(f"revisit {v['rr_served_s']:.3g} s (spare; asked {v['rr_asked_s']:g} s)")
            if v["limited_by"]:
                det.append("limited by " + ", ".join(v["limited_by"]))
            if v["out_of_reach_share"]:
                det.append(f"out of reach on {100 * v['out_of_reach_share']:.0f} %")
            if v["peak_shed_rho"] is not None:
                det.append(f"shed at the peak (range {100 * v['peak_shed_rho']:.0f} %)")
            if v["no_confirmation"]:
                det.append("its detections cannot be confirmed")
            self._item(top, [str(v["sector"]), v["state"], "", "; ".join(det) or "served as asked"],
                       (None, STATE_COLORS[v["state"]]), sid=v["sector"])
        top.setExpanded(self._open.get("Volumes", st_v != "OK"))
        # --- the constraints, one branch per face and the ship; nominal and peak side by side
        groups: Dict[str, Dict[str, Dict[str, dict]]] = {}
        for r in d["rows"]:
            groups.setdefault(r["where"], {}).setdefault(r["resource"], {})[r["view"]] = r
        order = sorted(groups, key=lambda w: (w == "ship", w))
        for where in order:
            res = groups[where]
            worst = max((r["state"] for vw in res.values() for r in vw.values()), key=["OK", "ALERT", "NOK"].index)
            top = self._item(self.tree, [where, worst, "", ""], (None, STATE_COLORS[worst]))
            for name in sorted(res, key=lambda n: (-max(["OK", "ALERT", "NOK"].index(r["state"])
                                                         for r in res[n].values()), n)):
                nom, pk = res[name].get("nominal"), res[name].get("peak")
                c1, k1 = _cell(nom["load"] if nom else None, nom["state"] if nom else None)
                c2, k2 = _cell(pk["load"] if pk else None, pk["state"] if pk else None)
                main = next(r for r in (pk if pk and pk["state"] != "OK" else None, nom, pk) if r)
                it = self._item(top, [name, c1, c2, main["note"]], (None, k1, k2),
                                tip=f"load / limit; the load ceiling is {100 * main['ceiling']:.0f} %")
                for w in main["who"][:6]:
                    self._item(it, [f"  {w['label']}", "", "", f"{100 * w['share']:.0f} % of it"
                                    + (f", served {100 * w['level']:.0f} %" if w.get("level", 1.0) < 1 - 1e-6 else "")],
                               (MUTED,), sid=w.get("sector"))
                for lv in main["levers"]:
                    self._item(it, ["  lever (estimated)", "", "", lv], (STATE_COLORS["ALERT"],),
                               sid=lv.split(":", 1)[0].split(" ", 1)[-1] if lv.startswith(("search ", "tracks "))
                               else None)
                it.setExpanded(main["state"] != "OK")
            top.setExpanded(self._open.get(where, worst != "OK"))
        if getattr(self, "_sel", None):
            self.set_selected(*self._sel)
