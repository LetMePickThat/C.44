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
LINK_KEY = "scenario_link"                       # {"t_s": the mission's instant} (+ "path" in the former format)
FLIGHT_PLAN_KEY = "flight_plan"                  # {"scenario": the scenario copied, "source": its file, "imported_utc"}
RESULTS_KEY = "results"                          # {"exact": the last exact result, "computed", "mission_sig"}
PROFILE_KEY = "profile"                          # {"key": its fingerprint, "profile": the profile along the plan}
DERIVED_KEYS = (RESULTS_KEY, PROFILE_KEY)        # what a calculation never reads
CACHE_DIR = os.path.join(APP_DIR, ".flight_plan_cache")   # the runs of the copied flight plans (recomputed if gone)


def embedded_path(sc: dict) -> str:
    """One file per mission (user, 11 November 2026): the mission keeps a COPY of its flight plan (the scenario); its
    run is a cache, recomputed when missing: the copy is written once into CACHE_DIR under a fingerprint of the
    scenario and of the parameter file, and its run computed next to it (the Scenario Editor's sim.run, unchanged)."""
    import hashlib
    key = hashlib.sha1((json.dumps(sc, sort_keys=True, default=str) + "|" + engine.params_hash())
                       .encode("utf-8")).hexdigest()[:16]
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, f"flight_plan_{key}.json")
    if not os.path.exists(path):
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(sc, fh, indent=1, ensure_ascii=False)
    return path


def copy_run(src_path: str, dst_path: str) -> None:
    """The run already computed next to a scenario file copied beside its cached copy (no recomputation)."""
    import shutil
    for a, b in zip(engine.journal_paths(src_path), engine.journal_paths(dst_path)):
        if os.path.exists(a) and not os.path.exists(b):
            shutil.copyfile(a, b)


def mission_signature(mission: dict) -> str:
    """The mission as a calculation reads it (the stored results and profile left out)."""
    import hashlib
    core = {k: v for k, v in mission.items() if k not in DERIVED_KEYS}
    return hashlib.sha1(json.dumps(core, sort_keys=True, default=str).encode("utf-8")).hexdigest()


def for_calculation(mission: dict) -> dict:
    """A copy of the mission without what a calculation never reads (results, profile: heavy)."""
    import copy
    return copy.deepcopy({k: v for k, v in mission.items() if k not in DERIVED_KEYS})


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


def check_scenario(path: str) -> None:
    """LinkError when the file is not a scenario (before offering to compute its run): a mission file is said as such
    (10 November 2026: a mission picked in "Load scenario..." offered a run that could only fail)."""
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, ValueError) as exc:
        raise LinkError(f"{os.path.basename(path)}: {exc}") from None
    if isinstance(data, dict) and "sectors" in data and "ship" not in data:
        raise LinkError(f"{os.path.basename(path)} is a mission file, not a scenario: open it with File > Open... "
                        "(its scenario, if it names one, is reopened with it)")
    try:
        srun.load_scenario(path)
    except scn.ScenarioError as exc:
        raise LinkError(f"{os.path.basename(path)} is not a valid scenario: {exc}") from None


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


# ----------------------------------------------------------------------------- the global view's source: the flight plan
# (user, 10 November 2026: the radar's use is planned on the ship's flight plan, its navigation, never on the
# scenario's truth, whose objects cannot be known: the global view shows the ship, the central body and the volumes)
def ship_only(link: ScenarioLink) -> Tuple[dict, object, object]:
    """(scenario, truth, geometry) of the linked run reduced to the ship: its states and its attitude as flown, no
    object; kept on the link (computed once)."""
    cached = getattr(link, "_ship_only", None)
    if cached is not None:
        return cached
    import copy
    import dataclasses
    from sim import geometry as sgeo
    tr = link.truth
    sc = copy.deepcopy(link.sc)
    sc["objects"], sc["groups"] = [], []
    ship = sc.setdefault("ship", {})
    ship["manoeuvres"] = [m for m in ship.get("manoeuvres", []) if m.get("type") in ("impulse", "burn")]
    ship["attitude"] = [w for w in ship.get("attitude", []) if w.get("attitude", {}).get("type") != "target"] or \
        ship.get("attitude", [])[:1]
    j = tr.att_ids.index("ship") if "ship" in tr.att_ids else None
    truth = dataclasses.replace(
        tr, ids=tr.ids[:1], classes=tr.classes[:1], r_km=tr.r_km[:, :1], v_kms=tr.v_kms[:, :1],
        t_appear_s=tr.t_appear_s[:1], t_impact_s=tr.t_impact_s[:1], t_pn_start_s=tr.t_pn_start_s[:1],
        miss_km=tr.miss_km[:1], group_of=tr.group_of[:1],
        att_ids=["ship"] if j is not None else [], att_q=tr.att_q[:, [j]] if j is not None else tr.att_q[:, :0],
        att_events=[e for e in tr.att_events if e.get("id", e.get("row")) in ("ship", 0, None)])
    out = (sc, truth, sgeo.geometry(sc, truth))
    link._ship_only = out
    return out


def nav_run(nav_msg: dict, name: str = "mission navigation") -> Tuple[dict, object, object]:
    """(scenario, truth, geometry) of the mission's navigation alone (no linked scenario): the ship on its orbit about
    the central body at the message's epoch, holding the message's attitude (fixed in the central body's equatorial
    axes), over one orbital period (its path drawn); computed by the editor's preview. ValueError without an orbit."""
    import math as _m
    import numpy as np
    from sim import attitude as att
    from sim import preview as pv_
    from sim import geometry as sgeo
    n = frames.Nav(nav_msg)
    if not n.has_orbit:
        raise ValueError("the navigation has no orbit (central body, r, v): no global view")
    eq = frames.equator_axes(n.body)
    r_eq, v_eq = eq.T @ n.r, eq.T @ n.v
    e = eq.T @ n.r_si                                           # ship -> the central body's equatorial axes
    z = e[:, 2]
    pitch = _m.degrees(_m.asin(max(-1.0, min(1.0, float(z[1])))))
    yaw = _m.degrees(_m.atan2(float(z[0]), float(z[2])))
    m = (att.rot_y(_m.radians(yaw)) @ att.rot_x(-_m.radians(pitch))).T @ e
    roll = -_m.degrees(_m.atan2(float(m[1, 0]), float(m[0, 0])))
    mu = float(scn.bodies()[n.body]["mu_km3_s2"])
    rn, vn = float(np.linalg.norm(r_eq)), float(np.linalg.norm(v_eq))
    a = 1.0 / max(2.0 / rn - vn * vn / mu, 1e-12)
    period = 2.0 * _m.pi * _m.sqrt(a ** 3 / mu) if a > 0 else 3600.0
    epoch = n.epoch or "2050-01-01T00:00:00Z"
    sc = {"meta": {"name": name, "epoch_utc": epoch, "duration_s": float(min(max(period, 60.0), 86400.0)), "seed": 1},
          "central_body": n.body,
          "ship": {"orbit": {"type": "state", "r_km": [float(x) for x in r_eq], "v_kms": [float(x) for x in v_eq]},
                   "attitude": [{"t_s": 0.0, "attitude": {"type": "fixed", "frame": "inertial", "yaw_deg": yaw,
                                                          "pitch_deg": pitch, "roll_deg": roll}}]},
          "objects": [], "groups": []}
    sc = scn.normalise(sc)
    truth = pv_.preview(sc)
    return sc, truth, sgeo.geometry(sc, truth)
