#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO_TOOL -- entry point of the scenario tool, separate from the mission preparation (user, 29 October 2026):
opens the scenario window (gui/scenario_window.py) on a new scenario, or on the scenario file given on the command
line.
Usage: python scenario_tool.py [scenario.json]
Without a terminal: double-click "Radar scenario.bat".
"""
from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication

from main import install_error_dialog, prefer_high_performance_gpu
from gui.scenario_window import ScenarioWindow


def main() -> int:
    install_error_dialog()
    prefer_high_performance_gpu()
    app = QApplication(sys.argv)
    win = ScenarioWindow(sys.argv[1] if len(sys.argv) > 1 else None)
    win.show_default()                     # maximised on its screen
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
