#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DOCUMENT -- the single mission file (roadmap phase 6, merge of the mission preparation and the scenario, step A;
framing validated by the user on 29 October 2026): one JSON with two sections,

  {"format_version": 1,
   "mission":  {"global": {...}, "sectors": [...],           the mission preparation (core/mission.py), as the
                "t_nav_s": instant of the scenario giving       optimiser reads it
                the navigation (default 0, step B)}
   "scenario": {"meta": ..., "central_body": ..., "ship": ..., "objects": [...], "groups": [...]}
                                                              the world (sim/scenario.py), optional: a mission alone
                                                              stays possible (static analysis)}

The scenario section has no "mission" key (the mission is the other section). The former files still open and are
converted (A2): a mission file alone (JSON or CSV: no scenario) and a scenario file (the mission file it names is read
and embedded; none named: an empty mission). Saving always writes the single format.

Step B (B1): with a scenario, the mission's navigation (attitude, orbit, central body) and sky are those of the
scenario's ship at t_nav_s (mission_for_calc); without one, the navigation typed in the mission stays.
"""
from __future__ import annotations

import copy
import json
import os
from dataclasses import dataclass, field
from typing import List, Optional

from core import mission as cm

from . import scenario as scn

FORMAT_VERSION = 1


class DocumentError(ValueError):
    """A file that is neither a mission file, nor a scenario, nor a valid document (the message says why)."""


@dataclass
class Document:
    """A mission (always) and its scenario (None: a mission alone); notes: what the reading converted."""
    mission: dict
    scenario: Optional[dict] = None
    notes: List[str] = field(default_factory=list)


def kind(data: object) -> str:
    """"document", "mission" or "scenario" for the JSON read from a file, "" when none of them."""
    if not isinstance(data, dict):
        return ""
    # a normalised scenario carries its own format_version (sim/scenario.py): a document is known by its mission
    # section, or a format_version without the scenario's "meta"
    if isinstance(data.get("mission"), dict) or ("format_version" in data and "meta" not in data):
        return "document"
    if "meta" in data:
        return "scenario"
    if "sectors" in data or "global" in data:
        return "mission"
    return ""


def load(path: str) -> Document:
    """Read a document, or a former mission file (JSON / CSV) or scenario file, converted (A2)."""
    if path.lower().endswith(".csv"):
        return Document(cm.load_mission(path), None, [f"{os.path.basename(path)}: mission (CSV), no scenario"])
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise DocumentError(f"{path}: {exc}") from None
    k = kind(data)
    name = os.path.basename(path)
    if k == "mission":
        return Document(cm.load_mission(path), None, [f"{name}: mission file, no scenario"])
    if k == "scenario":
        sc = _scenario(data, path)
        ref = sc.pop("mission", None)
        if not ref:
            return Document(cm.empty_mission(), sc, [f"{name}: scenario naming no mission file: empty mission"])
        try:
            mission = cm.load_mission(ref)
        except (OSError, ValueError) as exc:
            raise DocumentError(f"{path}: its mission file {ref}: {exc}") from None
        return Document(mission, sc, [f"{name}: scenario, its mission file {os.path.basename(ref)} embedded"])
    if k == "document":
        version = data.get("format_version", FORMAT_VERSION)
        if version != FORMAT_VERSION:
            raise DocumentError(f"{path}: format_version {version} (this tool reads {FORMAT_VERSION})")
        if not isinstance(data.get("mission"), dict):
            raise DocumentError(f"{path}: no \"mission\" section")
        mission = copy.deepcopy(data["mission"])
        mission.setdefault("global", dict(cm.GLOBAL_DEFAULTS))
        mission.setdefault("sectors", [])
        sc = data.get("scenario")
        return Document(mission, None if sc is None else _scenario(sc, path))
    raise DocumentError(f"{path}: neither a mission file, nor a scenario, nor a document")


def _scenario(data: dict, path: str) -> dict:
    """The scenario section validated and normalised (its mission path, if any, made absolute)."""
    err = scn.validate(data)
    if err:
        raise DocumentError(f"{path}: scenario:\n  " + "\n  ".join(err))
    s = scn.normalise(data)
    if s.get("mission"):
        s["mission"] = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(path)), s["mission"]))
    return s


# ----------------------------------------------------------------------------- B: the navigation drawn from the scenario
def t_nav(doc: Document) -> float:
    """B1 (user, 29 October 2026): the instant of the scenario (s from its epoch) whose ship gives the mission's
    navigation: the mission's "t_nav_s", 0 (the scenario's start) by default."""
    return float(doc.mission.get("t_nav_s", 0.0))


def mission_for_calc(doc: Document) -> tuple:
    """(mission, alerts) the optimiser computes: with a scenario, the mission with the navigation (attitude, orbit,
    central body) and the sky of the scenario's ship at t_nav_s (sim/engine.mission_at: the scenario propagated up to
    that instant); without one, the mission as it is (its navigation typed by hand)."""
    if doc.scenario is None:
        return {k: v for k, v in doc.mission.items() if k != "t_nav_s"}, []
    from . import engine, propagate                       # late: the propagation only when a scenario is there
    t = t_nav(doc)
    dur = float(doc.scenario["meta"]["duration_s"])
    if not 0.0 <= t <= dur:
        raise DocumentError(f"t_nav_s = {t:g} s outside the scenario (0 to {dur:g} s)")
    short = copy.deepcopy(doc.scenario)
    short["meta"]["duration_s"] = max(t, float(short["meta"]["step_s"]))
    mission = {k: v for k, v in doc.mission.items() if k != "t_nav_s"}
    return engine.mission_at(doc.scenario, propagate.propagate(short), t, mission=mission)


def to_json(doc: Document, check: bool = True) -> dict:
    """The JSON of a document (the scenario section validated unless check is False, without a "mission" key)."""
    out = {"format_version": FORMAT_VERSION, "mission": doc.mission}
    if doc.scenario is not None:
        sc = {k: v for k, v in doc.scenario.items() if k != "mission"}
        err = scn.validate(sc) if check else []
        if err:
            raise DocumentError("scenario:\n  " + "\n  ".join(err))
        out["scenario"] = sc
    return out


def save(path: str, doc: Document, check: bool = True) -> None:
    """Write a document in the single format (check False: a scenario with problems written as it is)."""
    data = to_json(doc, check)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(data, fh, indent=1, ensure_ascii=False)
