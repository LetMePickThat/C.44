"""pytest configuration shared by the test files.

Parallel run (pytest-xdist, requirements-dev.txt; pytest.ini: -n 6 --dist loadgroup): every test is independent
(own inputs, tmp_path, parameter files through RADAR_PARAMS in subprocesses; the derived caches are written
atomically). The tests using the module fixture `runs` (every test mission through the optimiser) are kept in one
worker so the fixture is computed once. Sequential run: python -m pytest -n 0.
"""
import pytest


@pytest.hookimpl(tryfirst=True)   # before xdist's own hook, which reads the groups
def pytest_collection_modifyitems(items):
    for item in items:
        if "runs" in getattr(item, "fixturenames", ()):
            item.add_marker(pytest.mark.xdist_group("runs"))
