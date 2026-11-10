#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO_LINK -- a scenario computed by the Scenario Editor as a use case of the mission preparation (pass S1, user
10 November 2026): the mission reads the scenario's run (its journal <stem>.npz, sim/run.py), never propagates it.

  - load(path): the scenario and its journal; the journal checked against the scenario file and the parameter file
    (stale: the reasons, the run can still be used, with an alert);
  - ScenarioLink.nav_sky(t): the navigation message (attitude, orbit, central body) and the sky (Sun, Earth, Moon, ship
    frame) of the scenario's ship at t, as sim/engine.mission_at writes them, and the alerts between the two;
  - compute(parent, path): the run computed (python -m sim.run, a separate process, as "Record run..." of the
    Scenario Editor) with a progress dialog.

The link is kept in the mission file under "scenario_link": {"path": the scenario's path, relative to the mission
file once saved, "t_s": the instant}; the optimiser ignores that key. A scenario is a use case, not the reference
threat: it gives the navigation and the sky, it never replaces the mission's settings or its target classes.
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field
from datetime import timedelta
from typing import List, Optional, Tuple

from PyQt5.QtCore import QEventLoop, QProcess
from PyQt5.QtWidgets import QProgressDialog, QWidget

from core import frames
from sim import engine
from sim import run as srun
from sim import scenario as scn

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIO_DIR = os.path.join(APP_DIR, "scenarios")
LINK_KEY = "scenario_link"


class LinkError(Exception):
    pass


@dataclass
class ScenarioLink:
    path: str                                  # the scenario file, absolute
    sc: dict                                   # the scenario as the run used it (from the journal)
    truth: object                              # sim.propagate.Truth
    g: object                                  # sim.geometry.Geometry at the recorded instants
    stale: List[str] = field(default_factory=list)   # why the journal may not match the files (empty: it does)

    @property
    def name(self) -> str:
        return str(self.sc["meta"].get("name") or os.path.splitext(os.path.basename(self.path))[0])

    @property
    def duration(self) -> float:
        return float(self.truth.t_s[-1]) if len(self.truth.t_s) else 0.0

    def utc(self, t: float) -> str:
        return (scn.epoch(self.sc) + timedelta(seconds=float(t))).strftime("%Y-%m-%d %H:%M:%S UTC")

    def nav_sky(self, t: float) -> Tuple[dict, dict, List[str]]:
        """(navigation message, sky, alerts) of the ship at t (sim/engine.mission_at without the mission)."""
        t = min(max(float(t), 0.0), self.duration)
        nav = engine.Navigation(self.sc, self.truth).message(t, label=f"{self.sc['meta']['name']} t = {t:g} s")
        sky = engine.sky_at(self.sc, self.truth, t)
        return nav, sky, list(frames.sky_nav_alerts(sky, frames.parse_nav([nav])))


def journal_path(path: str) -> str:
    """The journal of the scenario at path (next to it, sim/engine.journal_paths)."""
    return engine.journal_paths(path)[0]


def load(path: str) -> ScenarioLink:
    """The scenario at path with its run, read once; the run checked against the scenario file and the parameter
    file (ScenarioLink.stale). LinkError when the run is missing or unreadable."""
    path = os.path.abspath(path)
    npz = journal_path(path)
    if not os.path.exists(npz):
        raise LinkError(f"{os.path.basename(path)}: no run computed (no {os.path.basename(npz)})")
    try:
        sc_file = srun.load_scenario(path)
    except scn.ScenarioError as exc:
        raise LinkError(str(exc)) from None
    try:
        sc, truth, g, _nav, meta = engine.load_journal(npz)
    except (OSError, ValueError, KeyError) as exc:
        raise LinkError(f"{npz}: {exc}") from None
    why = []
    if json.dumps(sc, sort_keys=True, default=str) != json.dumps(sc_file, sort_keys=True, default=str):
        why.append("the scenario changed since the run")
    if meta.get("params_hash") != engine.params_hash():
        why.append("the run was computed with another parameter file")
    return ScenarioLink(path, sc, truth, g, why)


def compute(parent: QWidget, path: str) -> bool:
    """The scenario's run computed (python -m sim.run, as the Scenario Editor's "Record run..."), a progress dialog
    meanwhile; True when it succeeded."""
    dlg = QProgressDialog(f"Computing the run of {os.path.basename(path)}...", "Cancel", 0, 1000, parent)
    dlg.setWindowTitle("Scenario")
    dlg.setMinimumDuration(0)
    proc = QProcess(parent)
    proc.setProgram(sys.executable)
    proc.setArguments(["-u", "-m", "sim.run", path])
    proc.setWorkingDirectory(APP_DIR)
    proc.setProcessChannelMode(QProcess.MergedChannels)
    out: List[str] = []
    loop = QEventLoop()

    def _read() -> None:
        text = bytes(proc.readAllStandardOutput()).decode("utf-8", "replace")
        out.append(text)
        for part in text.replace("\n", "\r").split("\r"):        # "<stage> <percent> %" lines (sim/run.main)
            bits = part.split()
            if len(bits) >= 3 and bits[-1] == "%":
                try:
                    dlg.setValue(int(10 * float(bits[-2])))
                    dlg.setLabelText(f"Computing the run of {os.path.basename(path)}: {bits[0]}")
                except ValueError:
                    pass

    proc.readyReadStandardOutput.connect(_read)
    proc.finished.connect(lambda *_a: loop.quit())
    dlg.canceled.connect(proc.kill)
    proc.start()
    loop.exec_()
    cancelled = dlg.wasCanceled()                  # (before close: closing the dialog counts as a cancel)
    dlg.close()
    ok = proc.exitStatus() == QProcess.NormalExit and proc.exitCode() == 0
    if not ok and not cancelled:
        tail = "".join(out).strip().splitlines()
        raise LinkError("the run failed" + (f": {tail[-1]}" if tail else f" (exit code {proc.exitCode()})"))
    return ok


def relative(path: str, mission_path: Optional[str]) -> str:
    """The scenario's path as the mission file keeps it: relative to the mission file's folder (absolute before a
    first save, or on another drive)."""
    if not mission_path:
        return os.path.abspath(path)
    try:
        return os.path.relpath(os.path.abspath(path), os.path.dirname(os.path.abspath(mission_path))).replace("\\", "/")
    except ValueError:                                              # (another drive)
        return os.path.abspath(path)


def resolve(link_path: str, mission_path: Optional[str]) -> str:
    """The absolute path of a scenario link read from a mission file."""
    if os.path.isabs(link_path) or not mission_path:
        return os.path.abspath(link_path)
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(mission_path)), link_path))
