#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO_TOOL -- entry point of the scenario tool, separate from the mission preparation (user, 29 October 2026):
opens the scenario window (gui/scenario_window.py) on a new scenario, or on the scenario file given on the command
line.
Usage: python scenario_tool.py [scenario.json]
Without a terminal: double-click "Mission Scenario Editor.bat".
"""
from __future__ import annotations

import os
import sys

from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication

from main import install_error_dialog, prefer_high_performance_gpu
from gui.scenario_window import ScenarioWindow

ICON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "icons", "mission_scenario_editor.ico")
APP_ID = "TLW.MissionScenarioEditor"


def set_app_id() -> None:
    """The tool's own entry in the taskbar, with its icon (else Windows groups it under Python's). Best effort."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
    except Exception:  # noqa: BLE001 -- only the taskbar's icon is lost
        pass


def main() -> int:
    install_error_dialog("Mission Scenario Editor")
    prefer_high_performance_gpu()
    set_app_id()
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon(ICON))         # the window's title bar and the taskbar (user, 30 October 2026)
    win = ScenarioWindow(sys.argv[1] if len(sys.argv) > 1 else None)
    win.show_default()                     # maximised on its screen
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
