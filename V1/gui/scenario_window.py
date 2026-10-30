#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO_WINDOW -- the scenario tool, separate from the mission preparation (user, 29 October 2026: "sortir
complètement la partie scénario de l'outil Pmiss, et en faire un outil complètement séparé pour l'instant"): one
header line (the menu of the scenario's files, "Propagate" on the right when the scenario is too large for the
automatic propagation), the scenario editor and its 3D views with the playback (gui/scenario_tab.py,
gui/sim_view.py), the file name, the validation and the propagation's state in the status bar. P1 (user, 29 October
2026): the tool starts on the 3D view alone, its editor and controls shown once a file is created (its properties
asked in a dialog) or opened. The mission file the
scenario names is drawn in the ship view; "Export mission at t" writes the mission with the scenario's navigation
and sky at t, for the mission-preparation tool.
"""
from __future__ import annotations

from typing import Optional

from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QMainWindow, QMenu, QShortcut, QVBoxLayout, QWidget

from core import geometry as geo
from gui import controls as uc
from gui.main_window import GRID_RES_DEG, HEADER_STYLE, PRIMARY_STYLE
from gui.scenario_tab import ScenarioTab

TITLE = "Radar scenario"


class ScenarioWindow(QMainWindow):
    def __init__(self, path: Optional[str] = None):
        super().__init__()
        self.setWindowTitle(TITLE)
        self.grid = geo.SphereGrid(res_deg=GRID_RES_DEG)
        self.scenario = ScenarioTab(self.grid)
        self.menu = QMenu(self)
        self.menu.aboutToShow.connect(self._fill_menu)
        header = QFrame()
        header.setObjectName("header")
        header.setStyleSheet(HEADER_STYLE)
        hl = QHBoxLayout(header)
        hl.setContentsMargins(6, 4, 8, 4)
        hl.addWidget(uc.menu_button("File", self.menu, tip="The scenario's files (Ctrl+N / O / S)", arrow=False))
        hl.addStretch(1)
        prop = self.scenario.primary_action()
        if prop is not None:
            prop.setStyleSheet(PRIMARY_STYLE)
            hl.addWidget(prop)
        central = QWidget()
        cl = QVBoxLayout(central)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        cl.addWidget(header)
        cl.addWidget(self.scenario, 1)
        self.setCentralWidget(central)
        for w in self.scenario.status_widgets():
            self.statusBar().addWidget(w)
        for key, cmd in ((QKeySequence.New, "new"), (QKeySequence.Open, "open"), (QKeySequence.Save, "save"),
                         (QKeySequence("Ctrl+Shift+S"), "save_as")):
            QShortcut(key, self, activated=lambda c=cmd: getattr(self.scenario, c)())
        self.scenario.title_changed.connect(lambda name: (self.setWindowTitle(f"{TITLE} - {name}"),
                                                          self.statusBar().clearMessage()))
        self.statusBar().showMessage("Create a scenario (New...) or open one (Open...) from the File menu")
        if path:
            self.scenario.open_path(path)

    def _fill_menu(self) -> None:
        """The scenario's files; before a file is created or opened, only New and Open (P1)."""
        self.menu.clear()
        for text, slot, icon in self.scenario.file_commands():
            act = self.menu.addAction(text, slot)
            ic = uc.std_icon(self, icon)
            if ic is not None:
                act.setIcon(ic)
            act.setEnabled(self.scenario.loaded or text in ("New...", "Open..."))
        self.menu.addSeparator()
        self.menu.addAction("About...", lambda: uc.show_about(self, "Radar scenario"))

    def show_default(self) -> None:
        """At start, maximised on its screen (user, 29 October 2026): the whole screen but the task bar, the title
        bar kept."""
        self.showMaximized()

    def closeEvent(self, event) -> None:
        if hasattr(self.scenario.preview, "stop_worker"):
            self.scenario.preview.stop_worker()
        view = getattr(self.scenario.preview, "plotter", None)
        if view is not None:
            view.close()
        super().closeEvent(event)
