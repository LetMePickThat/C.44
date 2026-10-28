#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN_WINDOW -- the application window: a tab widget holding the mission-preparation tab (permanent), the scenario
editor of the global simulator (permanent, phase 6) and, once a calculation has run, a closable results tab. The mission is locked while the results tab is open, so the results
always describe the mission on screen; closing the tab (after an export prompt if the result was never exported)
makes it editable again. The exact calculation (core.optimizer.run) runs on a background thread with a progress
dialog and a cooperative cancel.
"""
from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QProgressDialog, QTabBar, QTabWidget

from core import geometry as geo
from core import optimizer as mo
from core.params import P
from gui.prep_tab import PrepTab
from gui.results_tab import ResultsTab
from gui.scenario_tab import ScenarioTab

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


class MainWindow(QMainWindow):
    def __init__(self, mission: dict):
        super().__init__()
        self.setWindowTitle("Radar mission preparation")
        self.resize(*WINDOW_SIZE)                 # fitted to the screen by show_default()
        self.grid = geo.SphereGrid(res_deg=GRID_RES_DEG)
        self.results: Optional[ResultsTab] = None
        self._worker: Optional[OptimizeWorker] = None
        self._progress_dlg: Optional[QProgressDialog] = None

        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self._on_tab_close_requested)
        self.setCentralWidget(self.tabs)
        self.prep = PrepTab(mission, self.grid)
        self.prep.run_requested.connect(self._run)
        self.tabs.addTab(self.prep, "Mission preparation")
        self.scenario = ScenarioTab(self.grid)              # phase 6: scenario editor (passes S5-S7)
        self.tabs.addTab(self.scenario, "Scenario")
        for index in (0, 1):                                # the preparation and scenario tabs cannot be closed
            for side in (QTabBar.LeftSide, QTabBar.RightSide):
                self.tabs.tabBar().setTabButton(index, side, None)

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
        self.tabs.setCurrentIndex(self.tabs.addTab(self.results, title))
        self.prep.set_locked(True)

    def _on_tab_close_requested(self, index: int) -> None:
        tab = self.tabs.widget(index)
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
        self.tabs.removeTab(index)
        tab.view.plotter.close()
        tab.deleteLater()
        self.results = None
        self.prep.set_locked(False)
        self.tabs.setCurrentWidget(self.prep)

    def closeEvent(self, event) -> None:
        for view in [self.prep.view] + ([self.results.view] if self.results is not None else []):
            view.plotter.close()
        super().closeEvent(event)
