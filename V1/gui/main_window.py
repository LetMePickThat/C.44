#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN_WINDOW -- the application window (B14, user, 29 October 2026, replaces the ribbon B13): one header line, the
menu (the files of the mode shown), the choice of the working mode (the mission preparation, permanent; once a
calculation has run, the results, closable; the scenario is a separate tool since 29 October 2026,
gui/scenario_window.py) and the mode's main action on the right ("Calculate"); under it the working area of the
mode (its other commands where they act: over the 3D view, on the sector list), its state in the status bar. The mission is locked while the results are open, so the results always describe
the mission on screen; closing them (after an export prompt if the result was never exported) makes it editable
again. The exact calculation (core.optimizer.run) runs on a background thread with a progress dialog and a
cooperative cancel.
"""
from __future__ import annotations

from typing import List, Optional

from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (QApplication, QFrame, QHBoxLayout, QMainWindow, QMenu, QMessageBox, QProgressDialog,
                             QShortcut, QStackedWidget, QTabBar, QVBoxLayout, QWidget)

from core import geometry as geo
from core import mission as md
from core import optimizer as mo
from core.params import P
from gui import controls as uc
from gui.results_tab import ResultsTab
from gui.prep_tab import PrepTab

GRID_RES_DEG = P.get("tool.display.grid_res_deg")   # direction-sphere raster of the 3D views and the fast estimate (shared by both tabs)


class OptimizeWorker(QThread):
    """core.optimizer.run() on a background thread, with per-sector progress and a cooperative cancel (checked
    between sectors: the sector being solved when Cancel is pressed still finishes, there is no hard kill)."""
    progress = pyqtSignal(int, int, str)     # done, total, sector_id just finished
    finished_ok = pyqtSignal(dict)           # full run() result
    failed = pyqtSignal(str)                 # error message

    def __init__(self, mission: dict, parent=None):
        super().__init__(parent)
        self.mission = mission
        self._cancelled = False

    def cancel(self) -> None:
        self._cancelled = True

    def run(self) -> None:
        try:
            result = mo.run(self.mission, progress_cb=self._on_progress, cancel_cb=lambda: self._cancelled)
        except Exception as exc:  # noqa: BLE001 -- surface any model error to the UI instead of crashing
            self.failed.emit(f"calculation failed: {exc}")
            return
        self.finished_ok.emit(result)

    def _on_progress(self, done: int, total: int, sector_id: str) -> None:
        self.progress.emit(done, total, sector_id)


WINDOW_SIZE = tuple(int(x) for x in P.get("tool.display.window_size"))   # at start (user)
FRAME_MARGIN = (16, 48)                          # px kept for the window frame and its title bar
PREP_TAB, RESULTS_TAB = range(2)                 # the working modes, in order
MODE_STYLE = ("QTabBar {qproperty-drawBase: 0} QTabBar::tab {padding: 5px 14px; border: 1px solid #c8c8c8;"
              " background: #f6f6f6; margin-right: -1px}"
              " QTabBar::tab:selected {background: #2b579a; color: white; border-color: #2b579a}"
              " QTabBar::tab:hover:!selected {background: #e6e6e6}")
HEADER_STYLE = "#header {border-bottom: 1px solid #d0d0d0}"
PRIMARY_STYLE = ("QToolButton {background: #2b579a; color: white; border-radius: 3px; padding: 4px 12px;"
                 " font-weight: bold} QToolButton:disabled {background: #a0a8b8}")


class MainWindow(QMainWindow):
    def __init__(self, mission: Optional[dict] = None, path: Optional[str] = None):
        """mission: a mission to start on (with a new scenario); path: a file to open (sim/document.py)."""
        super().__init__()
        self.setWindowTitle("Radar mission preparation")
        self.resize(*WINDOW_SIZE)                 # fitted to the screen by show_default()
        self.grid = geo.SphereGrid(res_deg=GRID_RES_DEG)
        self.results: Optional[ResultsTab] = None
        self._worker: Optional[OptimizeWorker] = None
        self._progress_dlg: Optional[QProgressDialog] = None

        # (user, 29 October 2026) the scenario is a separate tool (scenario_tool.py, gui/scenario_window.py)
        self.prep = PrepTab(mission if mission is not None else md.empty_mission(), self.grid)
        self.prep.run_requested.connect(self._run)

        # B14: the header (menu, modes, main action), the working area of each mode, the status bar
        self.menu = QMenu(self)
        self.menu.aboutToShow.connect(self._fill_menu)
        self.modes = QTabBar()
        self.modes.setExpanding(False)
        self.modes.setStyleSheet(MODE_STYLE)
        self.work = QStackedWidget()
        self.actions_box = QHBoxLayout()
        self._modes: List[dict] = []            # per tab: its widget, status widgets, main action
        header = QFrame()
        header.setObjectName("header")
        header.setStyleSheet(HEADER_STYLE)
        hl = QHBoxLayout(header)
        hl.setContentsMargins(6, 4, 8, 4)
        hl.addWidget(uc.menu_button("File", self.menu, tip="Files of the mode shown (Ctrl+N / O / S)", arrow=False))
        hl.addSpacing(8)
        hl.addWidget(self.modes)
        hl.addStretch(1)
        hl.addLayout(self.actions_box)
        self._add_mode("Mission preparation", self.prep, [], self.prep.b_run)
        self.modes.setTabsClosable(True)
        for index in (PREP_TAB,):                           # only the results can be closed
            for side in (QTabBar.LeftSide, QTabBar.RightSide):
                self.modes.setTabButton(index, side, None)
        self.modes.tabCloseRequested.connect(self._on_tab_close_requested)
        self.modes.currentChanged.connect(self._on_mode_changed)
        central = QWidget()
        cl = QVBoxLayout(central)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        cl.addWidget(header)
        cl.addWidget(self.work, 1)
        self.setCentralWidget(central)
        # the usual shortcuts act on the document of the mode shown
        for key, cmd in ((QKeySequence.New, "new"), (QKeySequence.Open, "open"), (QKeySequence.Save, "save"),
                         (QKeySequence("Ctrl+Shift+S"), "save_as")):
            QShortcut(key, self, activated=lambda c=cmd: self._shortcut(c))
        self.modes.setCurrentIndex(PREP_TAB)
        self._on_mode_changed(PREP_TAB)
        if path:
            try:
                self.prep.set_mission(md.load_mission(path))
            except Exception as exc:  # noqa: BLE001 -- bad path / file: say so and stay on the mission shown
                QMessageBox.critical(self, "Open", f"Could not open {path}: {exc}")

    # ------------------------------------------------------------------ B14 header, modes, menu, status bar
    def _add_mode(self, title: str, widget: QWidget, status: List[QWidget], primary: Optional[QWidget]) -> int:
        """A tab: its widget, its status widgets, its main action."""
        index = self.modes.addTab(title)
        if self.work.indexOf(widget) < 0:
            self.work.addWidget(widget)
        for w in status:
            if w.parent() is None or w.parentWidget() is not self.statusBar():
                self.statusBar().addWidget(w)
            w.setVisible(False)
        holder = None
        if primary is not None:
            primary.setStyleSheet(PRIMARY_STYLE)
            holder = QWidget()                  # shown with its mode; the button's own visibility left to its tab
            h = QHBoxLayout(holder)
            h.setContentsMargins(0, 0, 0, 0)
            h.addWidget(primary)
            self.actions_box.addWidget(holder)
            holder.setVisible(False)
        self._modes.append({"widget": widget, "status": status, "holder": holder})
        return index

    def _on_mode_changed(self, index: int) -> None:
        """The working area of the mode chosen, its main action and its status shown."""
        if not 0 <= index < len(self._modes):
            return
        mode = self._modes[index]
        self.work.setCurrentWidget(mode["widget"])
        shown = set(map(id, mode["status"]))
        for m in self._modes:
            for w in m["status"]:
                w.setVisible(id(w) in shown)
            if m["holder"] is not None:
                m["holder"].setVisible(m is mode)

    def _commands(self) -> list:
        """(text, slot, icon, enabled) of the menu: the files of the mode shown."""
        current = self.work.currentWidget()
        if current is self.prep:
            return [(t, f, ic, not (e and self.prep.locked)) for t, f, ic, e in self.prep.file_commands()]
        if current is self.results:
            return [(t, f, ic, True) for t, f, ic in self.results.file_commands()]
        return []

    def _fill_menu(self) -> None:
        self.menu.clear()
        for text, slot, icon, enabled in self._commands():
            act = self.menu.addAction(text, slot)
            ic = uc.std_icon(self, icon)
            if ic is not None:
                act.setIcon(ic)
            act.setEnabled(enabled)
        self.menu.addSeparator()
        self.menu.addAction("About...", lambda: uc.show_about(self, "Radar mission preparation"))

    def _shortcut(self, cmd: str) -> None:
        """In the mission preparation Ctrl+O imports and Ctrl+S exports the mission (JSON); the results: Ctrl+S
        exports them."""
        current = self.work.currentWidget()
        if current is self.prep:
            if cmd == "open" and not self.prep.locked:
                self.prep._on_import()
            elif cmd in ("save", "save_as"):
                self.prep._on_export("json")
        elif current is self.results and cmd in ("save", "save_as"):
            self.results.export_json()

    def show_default(self) -> bool:
        """Show the window at WINDOW_SIZE centred on the available area of its screen (the task bar excluded),
        never beyond it: a size that does not fit in one direction is cut to that area (e.g. 2560 x 1321 on a
        5120 x 1440 screen); fitting in neither (e.g. 1920 x 1080) the window is maximised on it. Returns True when
        the default size fits as it is."""
        scr = self.screen() if self.screen() is not None else QApplication.primaryScreen()
        if scr is None:
            self.show()
            return True
        avail = scr.availableGeometry()
        aw, ah = avail.width() - FRAME_MARGIN[0], avail.height() - FRAME_MARGIN[1]
        w, h = WINDOW_SIZE
        if w > aw and h > ah:
            self.resize(max(aw, 640), max(ah, 480))
            self.move(avail.x(), avail.y())
            self.showMaximized()
            return False
        w, h = min(w, aw), min(h, ah)
        self.resize(w, h)
        self.move(avail.x() + (aw - w) // 2, avail.y() + (ah - h) // 2)
        self.show()
        return (w, h) == WINDOW_SIZE

    # ------------------------------------------------------------------ calculation
    def _run(self, mission: dict) -> None:
        """`mission` is the frozen copy the results tab will show."""
        self._progress_dlg = QProgressDialog("Calculating...", "Cancel", 0, len(mission["sectors"]), self)
        self._progress_dlg.setWindowModality(Qt.WindowModal)
        self._progress_dlg.setMinimumDuration(0)
        self._worker = OptimizeWorker(mission, self)
        self._worker.progress.connect(self._on_worker_progress)
        self._worker.finished_ok.connect(lambda result: self._on_worker_finished(mission, result))
        self._worker.failed.connect(self._on_worker_failed)
        self._progress_dlg.canceled.connect(self._worker.cancel)
        self._worker.start()
        self._progress_dlg.show()

    def _on_worker_progress(self, done: int, total: int, sector_id: str) -> None:
        if self._progress_dlg:
            self._progress_dlg.setLabelText(f"Sector {sector_id} done ({done}/{total})")
            self._progress_dlg.setValue(done)

    def _on_worker_finished(self, mission: dict, result: dict) -> None:
        if self._progress_dlg:
            self._progress_dlg.close()
        if result.get("cancelled"):
            n_done, n_total = len(result["sectors"]), len(mission["sectors"])
            if n_done == 0:
                QMessageBox.information(self, "Calculation", "Calculation cancelled before any sector was computed.")
                return
            answer = QMessageBox.question(
                self, "Calculation cancelled",
                f"Calculation cancelled after {n_done} of {n_total} sectors.\n"
                "Open the PARTIAL result (per-face totals will only include the computed sectors)?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self._open_results(mission, result)

    def _on_worker_failed(self, message: str) -> None:
        if self._progress_dlg:
            self._progress_dlg.close()
        QMessageBox.critical(self, "Calculation", message)

    # ------------------------------------------------------------------ results tab
    def _open_results(self, mission: dict, result: dict) -> None:
        self.results = ResultsTab(mission, result, self.grid)
        title = "Results" + (" (partial)" if self.results.partial else "")
        index = self._add_mode(title, self.results, [], None)
        self.prep.set_locked(True)
        self.modes.setCurrentIndex(index)

    def _on_tab_close_requested(self, index: int) -> None:
        tab = self._modes[index]["widget"] if 0 <= index < len(self._modes) else None
        if tab is None or tab is not self.results:
            return
        if not tab.exported:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Question)
            box.setWindowTitle("Close the results")
            box.setText("This result has not been exported. Closing the tab discards it and unlocks the mission.")
            b_export = box.addButton("Export JSON...", QMessageBox.AcceptRole)
            b_close = box.addButton("Close without export", QMessageBox.DestructiveRole)
            box.addButton(QMessageBox.Cancel)
            box.exec_()
            clicked = box.clickedButton()
            if clicked is b_export:
                tab.export_json()
                if not tab.exported:
                    return        # export dialog cancelled or failed: keep the tab
            elif clicked is not b_close:
                return
        self.modes.setCurrentIndex(PREP_TAB)
        self.work.removeWidget(tab)
        del self._modes[index]
        self.modes.removeTab(index)
        tab.view.plotter.close()
        tab.deleteLater()
        self.results = None
        self.prep.set_locked(False)

    def closeEvent(self, event) -> None:
        self.prep.view.plotter.close()
        if self.results is not None:
            self.results.view.plotter.close()
        super().closeEvent(event)
