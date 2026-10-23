#!/usr/bin/env python3
"""Test protocol (23 October 2026): pick the test level from what changed, say why, run it.

  python tests/run_tests.py            automatic level (below)
  python tests/run_tests.py N1|N2|N3   forced level
  python tests/run_tests.py --dry      only say what would run

Levels:
  N1  model self-test + the tests whose last known duration is under FAST_S (tests/.test_durations.json, written by
      tests/conftest.py at the end of every run).
  N2  N1 + the tests pytest-testmon finds affected by the changed Python code + the tests depending on the modules
      that read a changed [tool.*] entry of radar_system.toml; never the snapshot tests (tests/test_snapshots.py),
      kept for N3 (user, option b: they run nearly all the code, so testmon would select them for any change).
  N3  the whole suite (tests/), then the other steps of a full pass are printed (cost table, impact report,
      snapshots, journal).
Automatic: N3 when there is no reference yet, or when a model entry (outside [tool.*]) of radar_system.toml changed
since the last successful run; else N2 when code, tests or [tool.*] changed; else N1. The parameter file of the last
successful run is kept in tests/.params_last_run.toml (local, not in the repository).
"""
from __future__ import annotations

import json
import os
import re
import shutil
import sqlite3
import subprocess
import sys
import time
import tomllib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS = os.path.join(ROOT, "tests")
PARAMS = os.path.join(ROOT, "radar_system.toml")
LAST = os.path.join(TESTS, ".params_last_run.toml")
DURATIONS = os.path.join(TESTS, ".test_durations.json")
TESTMON_DB = os.path.join(ROOT, ".testmondata")
FAST_S = 10.0                  # N1: tests faster than this (last known duration, s)
SNAPSHOT_TESTS = "tests/test_snapshots.py"   # N3 only (option b)


def _flat(d: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict) and not ("value" in v and "status" in v):
            out.update(_flat(v, key + "."))
        else:
            out[key] = v.get("value") if isinstance(v, dict) and "value" in v else v
    return out


def params_changes() -> tuple:
    """(model keys changed, [tool.*] sections changed) since the last successful run; None if no reference."""
    if not os.path.exists(LAST):
        return None
    with open(PARAMS, "rb") as fh:
        now = _flat(tomllib.load(fh))
    with open(LAST, "rb") as fh:
        old = _flat(tomllib.load(fh))
    changed = sorted(k for k in set(now) | set(old) if now.get(k) != old.get(k))
    model = [k for k in changed if not k.startswith("tool.")]
    tool = sorted({".".join(k.split(".")[:2]) for k in changed if k.startswith("tool.")})
    return model, tool


def modules_reading(sections) -> list:
    """Python modules of the tool whose source names one of the [tool.*] sections."""
    out = []
    for d in ("core", "gui", "tools"):
        for f in sorted(os.listdir(os.path.join(ROOT, d))):
            if not f.endswith(".py"):
                continue
            src = open(os.path.join(ROOT, d, f), encoding="utf-8").read()
            if any(re.search(re.escape(s) + r"[.\"']", src) for s in sections):
                out.append(f"{d}/{f}")
    return out


def tests_depending_on(modules) -> set:
    """Tests whose last testmon record executed one of the modules (empty without a testmon database)."""
    if not modules or not os.path.exists(TESTMON_DB):
        return set()
    con = sqlite3.connect(TESTMON_DB)
    try:                           # testmon 2.2 schema; file names relative to the root, with "/"
        rows = con.execute("SELECT DISTINCT te.test_name FROM test_execution te "
                           "JOIN test_execution_file_fp tf ON tf.test_execution_id = te.id "
                           "JOIN file_fp fp ON fp.id = tf.fingerprint_id WHERE fp.filename IN (%s)"
                           % ",".join("?" * len(modules)), modules).fetchall()
    except sqlite3.Error:
        return set()
    finally:
        con.close()
    return {r[0] for r in rows}


def fast_tests() -> set:
    try:
        with open(DURATIONS, encoding="utf-8") as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        return set()
    return {k for k, v in d.items() if v < FAST_S}


def _collect(*extra) -> set:
    r = subprocess.run([sys.executable, "-m", "pytest", "--collect-only", "-q", "-o", "addopts=", "-p", "no:xdist",
                        *extra], cwd=ROOT, capture_output=True, text=True)
    return {ln.strip() for ln in r.stdout.splitlines() if "::" in ln}


def _testmon_known() -> set:
    if not os.path.exists(TESTMON_DB):
        return set()
    con = sqlite3.connect(TESTMON_DB)
    try:
        return {r[0] for r in con.execute("SELECT test_name FROM test_execution")}
    except sqlite3.Error:
        return set()
    finally:
        con.close()


def testmon_selection() -> set:
    """Tests testmon selects for the changed code (collection only, nothing run), plus every test testmon has no
    record of yet (its database fills at the first run of each test: until then the test is always selected)."""
    return _collect("--testmon") | (_collect() - _testmon_known())


def choose(level: str):
    """(level, reason, node ids or None for the whole suite)."""
    ch = params_changes()
    if level == "auto":
        if ch is None:
            return "N3", "no reference run yet (tests/.params_last_run.toml)", None
        model, tool = ch
        if model:
            return "N3", f"model entries changed in radar_system.toml: {', '.join(model[:8])}" + \
                         (" ..." if len(model) > 8 else ""), None
    if level == "N3":
        return "N3", "forced", None
    fast = fast_tests()
    if level == "N1":
        fast = {i for i in fast if not i.startswith(SNAPSHOT_TESTS)}
        return "N1", f"forced: {len(fast)} tests under {FAST_S:g} s", sorted(fast)
    tool = ch[1] if ch else []
    mods = modules_reading(tool)
    by_tool = tests_depending_on(mods)
    by_code = testmon_selection()
    ids = {i for i in fast | by_tool | by_code if not i.startswith(SNAPSHOT_TESTS)}
    why = [f"{len(fast)} fast tests"]
    if by_code:
        why.append(f"{len(by_code)} affected by the code (testmon)")
    if tool:
        why.append(f"{len(by_tool)} depending on {', '.join(mods) or '-'} (changed {', '.join(tool)})")
    if level == "auto" and not by_code and not tool:
        return "N1", "nothing changed since the last run: " + why[0], sorted(fast)
    return "N2", "; ".join(why), sorted(ids)


def main(argv) -> int:
    dry = "--dry" in argv
    args = [a for a in argv if a != "--dry"]
    level = args[0] if args else "auto"
    if level not in ("auto", "N1", "N2", "N3"):
        print(__doc__)
        return 2
    lvl, why, ids = choose(level)
    n = "all" if ids is None else str(len(ids))
    print(f"[{lvl}] {why} -> {n} tests")
    if dry:
        return 0
    t0 = time.time()
    st = subprocess.run([sys.executable, os.path.join("core", "radar_model.py"), "--selftest"], cwd=ROOT).returncode
    cmd = [sys.executable, "-m", "pytest", "--testmon-noselect", "-rf"]
    if ids is not None:
        ids = sorted(set(ids) & _collect())   # durations may name tests renamed since: pytest refuses an unknown id
        n = str(len(ids))
        if not ids:
            cmd = None
        else:
            cmd.extend(ids)        # on the command line: an @file is lost by the xdist workers (23 October 2026)
    rc = subprocess.run(cmd, cwd=ROOT).returncode if cmd else 0
    el = time.time() - t0
    ok = st == 0 and rc == 0
    print(f"\n[{lvl}] {'PASSED' if ok else 'FAILED'}: self-test {'ok' if st == 0 else 'FAILED'}, {n} tests, "
          f"{el / 60:.1f} min")
    print(f"commit line: Tests: {lvl} ({n} tests, {el / 60:.1f} min){'' if ok else ' -- FAILURES'}")
    if ok:
        shutil.copyfile(PARAMS, LAST)
    if lvl == "N3":
        print("full pass: also python tools/gen_cost_table.py, python tests/impact_report.py, "
              "python tests/update_snapshots.py and the JOURNAL.md entry")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
