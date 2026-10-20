"""Family 1 -- non-regression: every numeric output of every test case equals its frozen snapshot (relative 1e-9).

A failure lists the paths that changed. If the change is intended, read python tests/impact_report.py, then
refreeze with python tests/update_snapshots.py and commit the snapshots with the change. Test values only.
"""
import os

import pytest

import testbench as tb


@pytest.mark.parametrize("case", tb.case_names())
def test_outputs_match_snapshot(case):
    if not os.path.exists(tb.snapshot_path(case)):
        pytest.fail(f"no snapshot for '{case}': run python tests/update_snapshots.py {case}")
    diffs = tb.differences(tb.load_snapshot(case), tb.compute_case(case))
    assert not diffs, f"{len(diffs)} output value(s) of '{case}' changed:\n{tb.format_differences(diffs)}"


def test_every_test_mission_is_a_case():
    on_disk = {n for n in os.listdir(tb.MISSIONS_DIR) if n.endswith((".json", ".csv"))}
    assert on_disk == set(tb.MISSIONS), "add the new mission file to testbench.MISSIONS (or remove it)"


def test_difference_engine():
    """The comparison itself: within tolerance is equal, beyond is reported with its path."""
    assert tb.differences({"a": [1.0, "x"]}, {"a": [1.0 + 1e-12, "x"]}) == []
    d = tb.differences({"a": [1.0, "x"], "b": 2}, {"a": [1.0 + 1e-6, "y"], "c": 3})
    assert [p for p, _, _ in d] == ["/a[0]", "/a[1]", "/b", "/c"]
    assert tb.differences(float("nan"), float("nan")) == []
    assert tb.differences(True, 1) != []
