#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OPTIMIZE_WORKER -- runs mission_optimizer.run() on a background QThread so the mission-prep window stays
responsive, with per-sector progress and a cooperative cancel (checked between sectors: the sector
being solved when Cancel is pressed still finishes, there is no hard kill). Also runs the redundancy check
(redundancy.find_redundancies) off the UI thread.
"""
from __future__ import annotations

from PyQt5.QtCore import QThread, pyqtSignal

import mission_optimizer as mo
import redundancy


class OptimizeWorker(QThread):
    progress = pyqtSignal(int, int, str)     # done, total, sector_id just finished
    finished_ok = pyqtSignal(dict)           # full mission_optimizer.run() result
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


class RedundancyWorker(QThread):
    """Runs redundancy.find_redundancies() off the UI thread (it plans the earlier sectors with the optimiser).
    `token` is handed back untouched so the caller can tell whether the result is still current."""
    finished_ok = pyqtSignal(object, object)   # list of redundancy.Finding, token
    failed = pyqtSignal(str)

    def __init__(self, mission: dict, grid, token, parent=None):
        super().__init__(parent)
        self.mission, self.grid, self.token = mission, grid, token

    def run(self) -> None:
        try:
            findings = redundancy.find_redundancies(self.mission, self.grid)
        except Exception as exc:  # noqa: BLE001 -- a failed check must not take the window down
            self.failed.emit(f"redundancy check failed: {exc}")
            return
        self.finished_ok.emit(findings, self.token)
