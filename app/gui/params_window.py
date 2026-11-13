#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The parameter file in the tool (step 5.5, user 13 November 2026): every entry of radar_system.toml with its value, unit,
status, source and date; a value edited in place (TOML syntax), checked, then saved on its own line (core/params_edit:
comments and order kept). A system entry edited gets status D (derived / proposed, to confirm) and the date of the edit.
The values are read once per process: the tool applies them when started again (said when saving).
"""
from __future__ import annotations

from datetime import date
from typing import Any, Dict, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor, QFont
from PyQt5.QtWidgets import (QDialog, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QTreeWidget,
                             QTreeWidgetItem, QVBoxLayout)

from core import params as pm
from core import params_edit as pe

COL_KEY, COL_VALUE, COL_UNIT, COL_STATUS, COL_SOURCE, COL_DATE = range(6)
EDITED = QColor("#2b579a")
READ_ONLY = QColor("#808080")
STATUS_COLORS = {"V": QColor("#2e7d32"), "D": QColor("#b26a00"), "?": QColor("#c62828")}


class ParamsWindow(QDialog):
    def __init__(self, parent=None, path: Optional[str] = None):
        super().__init__(parent)
        self.setWindowTitle("Parameters")
        self.resize(1300, 800)
        self.path = path or pm.P.path
        with open(self.path, encoding="utf-8", newline="") as fh:
            self.text = fh.read()
        self.params = pm.load(self.path)
        self.lines = pe.find_lines(self.text)
        self.edits: Dict[str, Any] = {}
        self._items: Dict[str, QTreeWidgetItem] = {}
        self._filling = False

        v = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel(f"File: {self.path}"))
        top.addStretch(1)
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("Filter (key, unit, status or source)")
        self.filter.setClearButtonEnabled(True)
        self.filter.setMinimumWidth(320)
        self.filter.textChanged.connect(self._apply_filter)
        top.addWidget(self.filter)
        v.addLayout(top)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Parameter", "Value", "Unit", "Status", "Source", "Date"])
        self.tree.setAlternatingRowColors(True)
        self.tree.setUniformRowHeights(True)
        self.tree.itemChanged.connect(self._on_changed)
        v.addWidget(self.tree, 1)
        self._fill()

        note = QLabel("Double-click a value to edit it (TOML syntax: 1.5, true, \"text\", [1, 2]). An edited system value "
                      "is marked D (to confirm) with today's date. Grey entries (sub-keys of a table) are edited in the "
                      "file. The tool applies the saved values when started again.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #606060")
        v.addWidget(note)
        bottom = QHBoxLayout()
        self.lbl_count = QLabel("")
        bottom.addWidget(self.lbl_count)
        bottom.addStretch(1)
        self.b_revert = QPushButton("Undo the edits")
        self.b_revert.clicked.connect(self._revert)
        self.b_save = QPushButton("Save")
        self.b_save.clicked.connect(self.save)
        b_close = QPushButton("Close")
        b_close.clicked.connect(self.close)
        for b in (self.b_revert, self.b_save, b_close):
            bottom.addWidget(b)
        v.addLayout(bottom)
        self._update_count()

    # ------------------------------------------------------------------ tree
    def _section_item(self, section: str, nodes: Dict[str, QTreeWidgetItem]) -> Optional[QTreeWidgetItem]:
        if not section:
            return None
        if section in nodes:
            return nodes[section]
        parent_name, _, name = section.rpartition(".")
        parent = self._section_item(parent_name, nodes)
        it = QTreeWidgetItem([name])
        f = it.font(COL_KEY)
        f.setBold(True)
        it.setFont(COL_KEY, f)
        it.setFlags(Qt.ItemIsEnabled)
        (parent.addChild(it) if parent is not None else self.tree.addTopLevelItem(it))
        nodes[section] = it
        return it

    def _fill(self) -> None:
        self._filling = True
        self.tree.clear()
        self._items.clear()
        nodes: Dict[str, QTreeWidgetItem] = {}
        for p in self.params:
            section, _, name = p.path.rpartition(".")
            parent = self._section_item(section, nodes)
            it = QTreeWidgetItem([name, pe.toml_literal(p.value), p.unit, p.status, p.source, p.date])
            it.setData(COL_KEY, Qt.UserRole, p.path)
            it.setToolTip(COL_KEY, p.path)
            it.setToolTip(COL_SOURCE, p.source)
            if p.status in STATUS_COLORS:
                it.setForeground(COL_STATUS, QBrush(STATUS_COLORS[p.status]))
            if p.path in self.lines:
                it.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable)
            else:
                it.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                for c in range(6):
                    it.setForeground(c, QBrush(READ_ONLY))
            (parent.addChild(it) if parent is not None else self.tree.addTopLevelItem(it))
            self._items[p.path] = it
        self.tree.expandToDepth(0)
        for c in (COL_KEY, COL_VALUE, COL_UNIT, COL_STATUS, COL_DATE):
            self.tree.resizeColumnToContents(c)
        self.tree.setColumnWidth(COL_VALUE, min(max(self.tree.columnWidth(COL_VALUE), 90), 260))
        self._filling = False

    def _apply_filter(self, text: str) -> None:
        t = text.strip().lower()

        def walk(item: QTreeWidgetItem) -> bool:
            shown = False
            for i in range(item.childCount()):
                shown |= walk(item.child(i))
            if item.childCount() == 0:
                key = str(item.data(COL_KEY, Qt.UserRole) or "")
                hay = " ".join([key] + [item.text(c) for c in (COL_UNIT, COL_STATUS, COL_SOURCE)]).lower()
                shown = not t or t in hay
            item.setHidden(not shown)
            if t and shown and item.childCount():
                item.setExpanded(True)
            return shown
        for i in range(self.tree.topLevelItemCount()):
            walk(self.tree.topLevelItem(i))

    # ------------------------------------------------------------------ edits
    def _on_changed(self, item: QTreeWidgetItem, column: int) -> None:
        if self._filling or column != COL_VALUE:
            return
        key = item.data(COL_KEY, Qt.UserRole)
        if not key:
            return
        old = self.params.get(key)
        self._filling = True
        try:
            try:
                new = pe.parse_value(item.text(COL_VALUE), old)
            except ValueError as exc:
                QMessageBox.warning(self, "Parameters", f"{key}: {exc}")
                item.setText(COL_VALUE, pe.toml_literal(self.edits.get(key, old)))
                return
            item.setText(COL_VALUE, pe.toml_literal(new))
            if new == old:
                self.edits.pop(key, None)
            else:
                self.edits[key] = new
            meta = self.params.meta(key)
            edited = key in self.edits
            f = QFont(item.font(COL_VALUE))
            f.setBold(edited)
            item.setFont(COL_VALUE, f)
            item.setForeground(COL_VALUE, QBrush(EDITED if edited else self.tree.palette().text().color()))
            if meta.status:                      # (a system entry: status D and today's date once saved)
                item.setText(COL_STATUS, "D" if edited else meta.status)
                item.setText(COL_DATE, date.today().isoformat() if edited else meta.date)
                st = item.text(COL_STATUS)
                item.setForeground(COL_STATUS, QBrush(STATUS_COLORS.get(st, self.tree.palette().text().color())))
        finally:
            self._filling = False
        self._update_count()

    def _update_count(self) -> None:
        n = len(self.edits)
        self.lbl_count.setText(f"{n} value(s) edited, not saved" if n else "No edit")
        self.b_save.setEnabled(n > 0)
        self.b_revert.setEnabled(n > 0)

    def _revert(self) -> None:
        self.edits.clear()
        self._fill()
        self._apply_filter(self.filter.text())
        self._update_count()

    def save(self) -> bool:
        if not self.edits:
            return True
        try:
            text = pe.apply(self.text, self.edits, date.today().isoformat())
            pe.check(text, self.edits, self.params)
        except (pm.ParamError, TypeError, ValueError) as exc:
            QMessageBox.critical(self, "Parameters", f"Not saved: {exc}")
            return False
        try:
            with open(self.path, "w", encoding="utf-8", newline="") as fh:
                fh.write(text)
        except OSError as exc:
            QMessageBox.critical(self, "Parameters", f"Not saved: {exc}")
            return False
        n = len(self.edits)
        self.text = text
        self.params = pm.load(self.path)
        self.lines = pe.find_lines(self.text)
        self.edits.clear()
        self._fill()
        self._apply_filter(self.filter.text())
        self._update_count()
        QMessageBox.information(self, "Parameters",
                                f"{n} value(s) saved in {self.path}.\n\nThe tool reads its parameters when it starts: "
                                "close it and start it again to compute with them (the caches built on the former "
                                "values are rebuilt then).")
        return True

    def closeEvent(self, event) -> None:
        if self.edits:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Question)
            box.setWindowTitle("Parameters")
            box.setText(f"{len(self.edits)} value(s) edited and not saved.")
            b_save = box.addButton("Save", QMessageBox.AcceptRole)
            b_drop = box.addButton("Discard", QMessageBox.DestructiveRole)
            box.addButton(QMessageBox.Cancel)
            box.exec_()
            clicked = box.clickedButton()
            if clicked is b_save:
                if not self.save():
                    event.ignore()
                    return
            elif clicked is not b_drop:
                event.ignore()
                return
        super().closeEvent(event)
