#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RUN -- one run of the global simulator (roadmap step 6.3, pass S4): load the scenario (a single mission file's
"scenario" section, or a former scenario file), propagate, compute the
geometry at the recorded instants, write the journal <stem>.npz and the summary <stem>.summary.json next to the
scenario (overwritten); the 100 Hz navigation is stored only with meta.log_nav.

  python -m sim.run scenario.json
"""
from __future__ import annotations

import json
import sys
import time
from typing import Callable, Optional

from . import engine
from . import geometry as geo
from . import propagate as prop
from . import scenario as scn


def load_scenario(path: str) -> dict:
    """The scenario of a single mission file (sim/document.py: its "scenario" section), or of a former scenario
    file."""
    from . import document
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise scn.ScenarioError(f"{path}: {exc}") from None
    if document.kind(data) != "document":
        return scn.load(path)
    try:
        doc = document.load(path)
    except document.DocumentError as exc:
        raise scn.ScenarioError(str(exc)) from None
    if doc.scenario is None:
        raise scn.ScenarioError(f"{path}: the mission file has no scenario")
    return doc.scenario


def run(path: str, progress_cb: Optional[Callable[[str, float], None]] = None) -> dict:
    """Run the scenario at path; returns the summary. progress_cb(stage, fraction)."""
    t0 = time.perf_counter()
    sc = load_scenario(path)
    cb = progress_cb or (lambda _s, _f: None)
    truth = prop.propagate(sc, progress_cb=lambda k, n: cb("propagation", k / max(n, 1)))
    cb("geometry", 0.0)
    g = geo.geometry(sc, truth)
    nav = None
    if sc["meta"].get("log_nav"):
        cb("navigation", 0.0)
        nav = engine.Navigation(sc, truth).series()
    cpu = time.perf_counter() - t0
    npz, js = engine.journal_paths(path)
    engine.save_journal(npz, sc, truth, g, nav, cpu)
    s = engine.summary(sc, truth, g, cpu)
    with open(js, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(s, fh, indent=1, ensure_ascii=False)
    cb("done", 1.0)
    return s


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    try:
        s = run(argv[0], lambda st, f: print(f"\r{st:12s} {100 * f:5.1f} %", end="", flush=True))
    except scn.ScenarioError as exc:
        print(f"\n{exc}")
        return 1
    npz, js = engine.journal_paths(argv[0])
    print(f"\n{s['name']}: {len(s['objects'])} objects, {s['cpu_s']:.1f} s -> {npz}, {js}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
