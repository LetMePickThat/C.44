#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN -- entry point: opens the main window (gui/main_window.py) on an empty mission, or on the mission file given on
the command line. The mission-preparation tab imports / exports missions itself; a results tab opens once a
calculation finishes.
Usage: python main.py [mission.json | mission.csv]   (--new, kept for old shortcuts, is the same as no argument)
"""
from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication, QMessageBox

from core import mission as md
from gui.main_window import MainWindow


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


def main() -> int:
    prefer_high_performance_gpu()
    app = QApplication(sys.argv)
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    mission = md.empty_mission()
    if arg and arg != "--new":
        try:
            mission = md.load_mission(arg)
        except Exception as exc:  # noqa: BLE001 -- bad path / file: say so and start on an empty mission
            QMessageBox.critical(None, "Open", f"Could not open {arg}: {exc}")
    win = MainWindow(mission)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
