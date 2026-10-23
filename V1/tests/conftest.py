"""pytest configuration shared by the test files.

Parallel run (pytest-xdist, requirements-dev.txt; pytest.ini: -n 6 --dist loadgroup): every test is independent
(own inputs, tmp_path, parameter files through RADAR_PARAMS in subprocesses; the derived caches are written
atomically). The tests using the module fixture `runs` (every test mission through the optimiser) are kept in one
worker so the fixture is computed once. Sequential run: python -m pytest -n 0.

Progress (test protocol, 23 October 2026): the duration of every test is kept in tests/.test_durations.json (local,
not in the repository) at the end of each run; during a run, tests/.progress.txt gives the tests done / to do, the
share of the total computation done (weighted by the last known durations; a new test counts as the longest known
one, a prudent estimate), the elapsed time,
the estimated time left and the tests running. The longest tests are started first.
"""
import json
import os
import time

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
DURATIONS = os.path.join(_HERE, ".test_durations.json")
PROGRESS = os.path.join(_HERE, ".progress.txt")


def _load_durations() -> dict:
    try:
        with open(DURATIONS, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def _is_controller(config) -> bool:
    """The process that writes the progress: not an xdist worker, and not a collection-only run (run_tests.py
    collects to choose its tests: it must not overwrite the progress of a run going on)."""
    return not hasattr(config, "workerinput") and not config.getoption("collectonly", False)


def _share_threads():
    """Each parallel test process gets its share of the logical CPUs for the numba sums (core/sky.py)."""
    n = os.environ.get("PYTEST_XDIST_WORKER_COUNT")
    if not n:
        return
    try:
        import numba
        numba.set_num_threads(max(1, min(numba.config.NUMBA_NUM_THREADS, (os.cpu_count() or 1) // int(n))))
    except ImportError:
        pass


_share_threads()


@pytest.hookimpl(tryfirst=True)   # before xdist's own hook, which reads the groups
def pytest_collection_modifyitems(config, items):
    for item in items:
        if "runs" in getattr(item, "fixturenames", ()):
            item.add_marker(pytest.mark.xdist_group("runs"))
    known = _load_durations()
    if known:                      # longest first (new tests first), so that one long test does not finish alone at the end
        top = max(known.values())
        items.sort(key=lambda it: -known.get(it.nodeid, top))
    if _is_controller(config) and not _Progress.instance(config).ids:
        _Progress.instance(config).start([it.nodeid for it in items])


class _Progress:
    """Weighted progress of a run, written to PROGRESS by the controlling process."""

    def __init__(self):
        self.ids, self.expected, self.done, self.running, self.measured = [], {}, set(), set(), {}
        self.t0 = time.time()
        self.failed = 0
        self.failed_ids = []

    @staticmethod
    def instance(config) -> "_Progress":
        if not hasattr(config, "_radar_progress"):
            config._radar_progress = _Progress()
        return config._radar_progress

    def start(self, ids):
        known = _load_durations()
        top = max(known.values()) if known else 1.0          # a new test: as long as the longest known (prudent)
        self.ids = list(ids)
        self.expected = {i: known.get(i, top) for i in self.ids}
        self.write()

    def write(self, final: bool = False):
        if not self.ids:
            return
        tot = sum(self.expected.values()) or 1.0
        done_w = sum(self.expected.get(i, 0.0) for i in self.done)
        frac = done_w / tot
        el = time.time() - self.t0
        left = el * (1.0 - frac) / frac if frac > 0 else float("nan")
        lines = [f"{'finished' if final else 'running'} {time.strftime('%H:%M:%S')}",
                 f"tests: {len(self.done)} done ({self.failed} failed) / {len(self.ids) - len(self.done)} to do / "
                 f"{len(self.ids)} total",
                 f"computation done: {100.0 * frac:.1f} % (weighted by the last known durations)",
                 f"elapsed {el / 60:.1f} min, estimated left {left / 60:.1f} min",
                 "running: " + (", ".join(sorted(self.running)) or "-"),
                 "failed: " + (", ".join(self.failed_ids) or "-")]
        try:
            with open(PROGRESS + ".tmp", "w", encoding="utf-8") as fh:
                fh.write("\n".join(lines) + "\n")
            os.replace(PROGRESS + ".tmp", PROGRESS)
        except OSError:
            pass


@pytest.hookimpl(optionalhook=True)   # an xdist hook: absent when xdist is off
def pytest_xdist_node_collection_finished(node, ids):
    prog = _Progress.instance(node.config)
    if not prog.ids:
        prog.start(ids)


def pytest_configure(config):
    config.pluginmanager.register(_ProgressPlugin(config), "radar_progress")


class _ProgressPlugin:
    def __init__(self, config):
        self.config = config

    def pytest_runtest_logstart(self, nodeid, location):
        if _is_controller(self.config):
            p = _Progress.instance(self.config)
            p.running.add(nodeid)
            p.write()

    def pytest_runtest_logreport(self, report):
        if not _is_controller(self.config):
            return
        p = _Progress.instance(self.config)
        p.measured[report.nodeid] = p.measured.get(report.nodeid, 0.0) + report.duration
        if report.failed:
            p.failed += 1
            p.failed_ids.append(f"{report.nodeid} ({report.when})")
        if report.when == "teardown":
            p.running.discard(report.nodeid)
            p.done.add(report.nodeid)
            p.write()

    def pytest_sessionfinish(self, session, exitstatus):
        if not _is_controller(self.config):
            return
        p = _Progress.instance(self.config)
        p.write(final=True)
        if not p.measured:
            return
        known = _load_durations()
        known.update({k: round(v, 3) for k, v in p.measured.items() if k in p.done})
        try:
            with open(DURATIONS + ".tmp", "w", encoding="utf-8") as fh:
                json.dump(known, fh, indent=0, sort_keys=True)
            os.replace(DURATIONS + ".tmp", DURATIONS)
        except OSError:
            pass
