"""Freezes the current outputs of the tool as the test snapshots (tests/snapshots/*.json).

    python tests/update_snapshots.py            all cases
    python tests/update_snapshots.py model      one case (see testbench.case_names())

Run it ONLY when a change of the outputs is intended (a roadmap step that changes values), after reading the impact
report (python tests/impact_report.py); commit the new snapshots with that step. The snapshots are test values,
not radar performance.
"""
import sys

import testbench as tb


def main(argv) -> int:
    cases = argv or tb.case_names()
    unknown = [c for c in cases if c not in tb.case_names()]
    if unknown:
        print(f"unknown case(s) {unknown}; known: {tb.case_names()}")
        return 1
    for case in cases:
        tb.save_snapshot(case, tb.compute_case(case))
        print(f"frozen {tb.snapshot_path(case)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
