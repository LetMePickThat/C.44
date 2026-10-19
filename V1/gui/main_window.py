#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN_WINDOW -- the application window: a tab widget holding the mission-preparation tab (permanent) and, once a
calculation has run, a closable results tab. The mission is locked while the results tab is open, so the results
always describe the mission on screen; closing the tab (after an export prompt if the result was never exported)
makes it editable again. The exact calculation (core.optimizer.run) runs on a background thread with a progress
dialog and a cooperative cancel.
"""
from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtWidgets import QMainWindow, QMessageBox, QProgressDialog, QTabBar, QTabWidget

from core import geometry as geo
from core import optimizer as mo
from gui.prep_tab import PrepTab
from gui.results_tab import ResultsTab

GRID_RES_DEG = 0.5   # direction-sphere raster of the 3D views and the fast estimate (shared by both tabs)


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


class MainWindow(QMainWindow):
    def __init__(self, mission: dict):
        super().__init__()
        self.setWindowTitle("Radar mission preparation")
        self.resize(1400, 860)
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
        for side in (QTabBar.LeftSide, QTabBar.RightSide):   # the preparation tab cannot be closed
            self.tabs.tabBar().setTabButton(0, side, None)

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
