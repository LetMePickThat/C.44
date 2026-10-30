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
  N3  the full pass, with a global progress bar (25 October 2026): self-test, cost table, every test case computed
      once, impact report (tests/.impact_last.md), the whole suite; the refreeze (update_snapshots) stays a decision.
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


# ----------------------------------------------------------------------------- full pass N3 (25 October 2026)
N3_DURATIONS = os.path.join(TESTS, ".n3_durations.json")   # last duration of every N3 item (local)
N3_PROGRESS = os.path.join(TESTS, ".n3_progress.txt")       # the bar, readable while the pass runs (local)
IMPACT_OUT = os.path.join(TESTS, ".impact_last.md")         # impact report of the last N3 (local)
N3_DEFAULT_S = {"self-test": 20.0, "cost table": 150.0, "impact report": 10.0, "test suite": 600.0}
N3_DEFAULT_CASE_S = 300.0      # a case never timed yet: prudent


def _bar(frac: float, width: int = 30) -> str:
    n = int(round(max(0.0, min(1.0, frac)) * width))
    return "#" * n + "-" * (width - n)


class N3Progress:
    """Global progress bar of a full pass, weighted by the last known duration of each item (step or test case).
    Inside an item the bar advances with its elapsed time, held below its end until it finishes; inside the test
    suite it follows tests/.progress.txt (the weighted progress of conftest.py). Shown on one line in a terminal,
    a line every 30 s otherwise, and always written to tests/.n3_progress.txt."""

    def __init__(self, items):
        try:
            with open(N3_DURATIONS, encoding="utf-8") as fh:
                self.known = json.load(fh)
        except (OSError, ValueError):
            self.known = {}
        self.items = list(items)
        self.expected = {i: self.known.get(i, N3_DEFAULT_S.get(i, N3_DEFAULT_CASE_S)) for i in self.items}
        self.done, self.t0, self.cur, self.cur_t0 = {}, time.time(), None, None
        self.tty = sys.stdout.isatty()
        self.last_line = 0.0

    def start(self, item):
        self.cur, self.cur_t0 = item, time.time()
        self.show(force=True)

    def finish(self, item):
        self.done[item] = time.time() - self.cur_t0
        self.known[item] = self.done[item]
        self.cur = None
        try:
            with open(N3_DURATIONS, "w", encoding="utf-8") as fh:
                json.dump(self.known, fh, indent=1)
        except OSError:
            pass
        self.show(force=True)

    def _inner(self) -> float:
        """Share of the current item done."""
        if self.cur is None:
            return 0.0
        if self.cur == "test suite":
            try:
                with open(os.path.join(TESTS, ".progress.txt"), encoding="utf-8") as fh:
                    m = re.search(r"computation done: ([0-9.]+) %", fh.read())
                if m and os.path.getmtime(os.path.join(TESTS, ".progress.txt")) >= self.cur_t0:
                    return float(m.group(1)) / 100.0
            except OSError:
                pass
        return min(0.95, (time.time() - self.cur_t0) / max(self.expected[self.cur], 1.0))

    def show(self, force: bool = False):
        tot = sum(self.expected.values()) or 1.0
        done_w = sum(self.expected[i] for i in self.done)
        cur_w = self.expected[self.cur] * self._inner() if self.cur else 0.0
        frac = (done_w + cur_w) / tot
        el = time.time() - self.t0
        left = el * (1.0 - frac) / frac if frac > 0.02 else tot - el
        k = len(self.done) + (1 if self.cur else 0)
        cur = (f"{self.cur} {(time.time() - self.cur_t0) / 60:.1f} / ~{self.expected[self.cur] / 60:.1f} min"
               if self.cur else "-")
        line = (f"[N3] [{_bar(frac)}] {100 * frac:5.1f} % | item {k}/{len(self.items)}: {cur} | "
                f"elapsed {el / 60:.1f} min, left ~{max(left, 0.0) / 60:.1f} min")
        try:
            with open(N3_PROGRESS + ".tmp", "w", encoding="utf-8") as fh:
                fh.write(line + "\n" + "done: " + ", ".join(f"{i} {d / 60:.1f} min" for i, d in self.done.items())
                         + "\n")
            os.replace(N3_PROGRESS + ".tmp", N3_PROGRESS)
        except OSError:
            pass
        now = time.time()
        if self.tty:
            print("\r" + line[:200].ljust(200), end="", flush=True)
        elif force or now - self.last_line >= 30.0:
            print(line, flush=True)
            self.last_line = now


def _run_with_bar(prog: N3Progress, item: str, cmd, **kw) -> int:
    """One N3 item as a subprocess (its memory is freed when it ends), the bar refreshed every 2 s."""
    prog.start(item)
    log = open(os.path.join(TESTS, f".n3_{re.sub(r'[^a-z0-9]+', '_', item.lower())}.log"), "w", encoding="utf-8")
    p = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, **kw)
    while p.poll() is None:
        time.sleep(2.0)
        prog.show()
    log.close()
    prog.finish(item)
    return p.returncode


def full_pass() -> int:
    """N3: self-test, cost table, every test case computed once (kept by tests/testbench.py with the code
    fingerprint, T1), impact report against the frozen snapshots (tests/.impact_last.md), then the whole test suite
    (its snapshot tests read the cases already computed). Each item runs in its own process. The refreeze stays a
    decision: python tests/update_snapshots.py after reading the report (instant, from the computed cases)."""
    sys.path.insert(0, TESTS)
    import testbench as tb
    cases = tb.case_names()
    items = ["self-test", "cost table"] + [f"case {c}" for c in cases] + ["impact report", "test suite"]
    prog = N3Progress(items)
    py = sys.executable
    res = {"self-test": _run_with_bar(prog, "self-test", [py, os.path.join("core", "radar_model.py"), "--selftest"]),
           "cost table": _run_with_bar(prog, "cost table", [py, os.path.join("tools", "gen_cost_table.py")])}
    for c in cases:
        res[f"case {c}"] = _run_with_bar(prog, f"case {c}", [py, "-c", "import sys; sys.path.insert(0, 'tests'); "
                                                                  f"import testbench as tb; tb.compute_case({c!r})"])
    res["impact report"] = _run_with_bar(prog, "impact report", [py, os.path.join("tests", "impact_report.py"),
                                                                 "-o", IMPACT_OUT])
    res["test suite"] = _run_with_bar(prog, "test suite", [py, "-m", "pytest", "--testmon-noselect", "-rf"])
    if prog.tty:
        print()
    el = time.time() - prog.t0
    bad = [k for k, v in res.items() if v != 0]
    print(f"\n[N3] {'PASSED' if not bad else 'FAILED: ' + ', '.join(bad)} in {el / 60:.1f} min "
          f"(logs: tests/.n3_*.log; impact report: tests/.impact_last.md)")
    print(f"commit line: Tests: N3 full pass ({el / 60:.1f} min){'' if not bad else ' -- FAILURES'}")
    if not bad:
        shutil.copyfile(PARAMS, LAST)
    print("then, if the changes are intended: python tests/update_snapshots.py (from the computed cases) and the "
          "JOURNAL.md entry")
    return 0 if not bad else 1


def main(argv) -> int:
    dry = "--dry" in argv
    args = [a for a in argv if a != "--dry"]
    level = args[0] if args else "auto"
    if level not in ("auto", "N1", "N2", "N3"):
        print(__doc__)
        return 2
    lvl, why, ids = choose(level)
    n = "all" if ids is None else str(len(ids))
    print(f"[{lvl}] {why} -> {n} tests" + (" + cost table, every case, impact report (full pass)" if lvl == "N3" else ""))
    if dry:
        return 0
    if lvl == "N3":
        return full_pass()
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
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
