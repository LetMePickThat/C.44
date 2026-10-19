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


def prefer_high_performance_gpu() -> None:
    """Ask Windows to run this Python on the discrete GPU (Settings > Graphics > "High performance"), so the
    VTK OpenGL view does not land on the integrated GPU of a hybrid laptop. Must run before the first OpenGL
    context is created; takes effect for the current process. The preference is keyed by the application id
    for a Microsoft Store Python (the exe path is ignored for packaged apps) and by the exe path otherwise.
    An existing GpuPreference (user's explicit choice) is left alone. Best effort: never blocks the start."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        import winreg
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32")
        n = wintypes.UINT(1024)
        buf = ctypes.create_unicode_buffer(n.value)
        kernel32.GetCurrentApplicationUserModelId.argtypes = [ctypes.POINTER(wintypes.UINT), wintypes.LPWSTR]
        if kernel32.GetCurrentApplicationUserModelId(ctypes.byref(n), buf) == 0:
            app_key = buf.value                       # packaged (Store) Python
        else:
            n = wintypes.DWORD(32768)
            buf = ctypes.create_unicode_buffer(n.value)
            kernel32.GetCurrentProcess.restype = wintypes.HANDLE
            kernel32.QueryFullProcessImageNameW.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR,
                                                            ctypes.POINTER(wintypes.DWORD)]
            if not kernel32.QueryFullProcessImageNameW(kernel32.GetCurrentProcess(), 0, buf, ctypes.byref(n)):
                return
            app_key = buf.value                       # real image (python3.x.exe), not a launcher alias
        with winreg.CreateKey(winreg.HKEY_CURRENT_USER, r"Software\Microsoft\DirectX\UserGpuPreferences") as key:
            try:
                current = winreg.QueryValueEx(key, app_key)[0]
            except FileNotFoundError:
                current = ""
            if "GpuPreference=" not in current:
                winreg.SetValueEx(key, app_key, 0, winreg.REG_SZ, current + "GpuPreference=2;")
    except Exception:  # noqa: BLE001 -- a missing preference only costs speed
        pass


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
    prefer_high_performance_gpu()
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
