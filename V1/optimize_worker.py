#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OPTIMIZE_WORKER -- runs mission_optimizer.run() on a background QThread so the mission-prep window stays
responsive, with per-sector progress and a cooperative cancel (checked between sectors: the sector
being solved when Cancel is pressed still finishes, there is no hard kill).
"""
from __future__ import annotations

from PyQt5.QtCore import QThread, pyqtSignal

import mission_optimizer as mo


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
