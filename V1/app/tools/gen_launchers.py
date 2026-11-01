#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEN_LAUNCHERS -- builds the two launchers at the root of V1/ from tools/launcher.cs, with the C# compiler that comes with
Windows (.NET Framework 4). The icon is embedded in the .exe, so it needs no path and shows from the first clone on, on any
machine and in any folder. Each .exe starts the .bat of its own name in app/.
Run again whenever the icon or launcher.cs changes:
    python tools/gen_launchers.py
"""
from __future__ import annotations

import os
import subprocess
from typing import List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.dirname(HERE)
ROOT = os.path.dirname(APP)
SOURCE = os.path.join(HERE, "launcher.cs")
CSC = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "Microsoft.NET", "Framework64", "v4.0.30319", "csc.exe")
LAUNCHERS: Tuple[Tuple[str, Optional[str]], ...] = (
    ("Mission Scenario Editor", os.path.join(APP, "assets", "icons", "mission_scenario_editor.ico")),
    ("Radar mission prep", None),       # (no icon drawn for this tool yet)
)


def main() -> None:
    for name, icon in LAUNCHERS:
        out = os.path.join(ROOT, name + ".exe")
        cmd: List[str] = [CSC, "/nologo", "/target:winexe", "/optimize+", "/reference:System.Windows.Forms.dll",
                          "/reference:System.Core.dll", f"/out:{out}"]
        if icon:
            cmd.append(f"/win32icon:{icon}")
        subprocess.run(cmd + [SOURCE], check=True)
        print(out)


if __name__ == "__main__":
    main()
