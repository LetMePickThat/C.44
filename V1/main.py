#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN -- entry point. Without arguments, opens the launch window (new mission, or open a JSON/CSV file through a file
dialog); with a mission file, or --new, goes straight to the mission-preparation window (window 1), which chains to the
results window (window 2) once a calculation finishes.
Usage: python main.py [mission.json | mission.csv | --new]
"""
from __future__ import annotations

import sys
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QFileDialog, QLabel, QMainWindow, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

import mission_data as md
from mission_prep_window import MissionPrepWindow


def load_mission_file(path: str) -> dict:
    mission = md.load_mission(path)
    mission.setdefault("global", dict(md.GLOBAL_DEFAULTS))
    return mission


class LauncherWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Space AESA radar -- Launcher")
        self.setFixedSize(440, 240)
        self.prep_window: Optional[MissionPrepWindow] = None

        central = QWidget()
        self.setCentralWidget(central)
        v = QVBoxLayout(central)
        v.setContentsMargins(28, 28, 28, 28)
        v.setSpacing(14)

        title = QLabel("Radar mission preparation")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        v.addWidget(title)

        subtitle = QLabel("Choose how to start.")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet("color: gray;")
        v.addWidget(subtitle)

        b_new = QPushButton("New mission")
        b_new.setMinimumHeight(42)
        b_new.clicked.connect(self._on_new)
        v.addWidget(b_new)

        b_open = QPushButton("Open a mission... (JSON/CSV)")
        b_open.setMinimumHeight(42)
        b_open.clicked.connect(self._on_open)
        v.addWidget(b_open)

        b_quit = QPushButton("Quit")
        b_quit.clicked.connect(QApplication.instance().quit)
        v.addWidget(b_quit)

    # ------------------------------------------------------------------ actions
    def _on_new(self) -> None:
        self._open_prep_window(md.empty_mission())

    def _on_open(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open a mission", "", "Mission (*.json *.csv)")
        if not path:
            return
        try:
            mission = load_mission_file(path)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Open", f"Could not open the file: {exc}")
            return
        self._open_prep_window(mission)

    def _open_prep_window(self, mission: dict) -> None:
        self.prep_window = MissionPrepWindow(mission)
        self.prep_window.show()
        self.close()


def main() -> int:
    app = QApplication(sys.argv)
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    win: QMainWindow
    if arg == "--new":
        win = MissionPrepWindow(md.empty_mission())
    elif arg:
        try:
            win = MissionPrepWindow(load_mission_file(arg))
        except Exception as exc:  # noqa: BLE001 -- bad path / file: say so and fall back to the launcher
            QMessageBox.critical(None, "Open", f"Could not open {arg}: {exc}")
            win = LauncherWindow()
    else:
        win = LauncherWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
