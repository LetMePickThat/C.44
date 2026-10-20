"""Impact report: key figures of the frozen snapshots (before) against the current code (after), as Markdown.

    python tests/impact_report.py               all cases, rows that changed only
    python tests/impact_report.py --all         every row
    python tests/impact_report.py -o FILE.md    also write the report to a file

Test missions and test values only: the figures show what a code change does, not what the radar achieves.
"""
import argparse
import os
import sys

import testbench as tb


def _fmt(v) -> str:
    if isinstance(v, float):
        return f"{v:.4g}" if abs(v) < 1e5 else f"{v:,.0f}"
    return "-" if v is None else str(v)


def _delta(a, b) -> str:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        if a == b:
            return ""
        return f"{100 * (b - a) / a:+.2f} %" if a else "new"
    return "" if a == b else "changed"


def report(show_all: bool) -> str:
    lines = ["# Impact report (test missions, test values only)", ""]
    total = 0
    for case in tb.case_names():
        if not os.path.exists(tb.snapshot_path(case)):
            lines += [f"## {case}", "", "_no frozen snapshot: run tests/update_snapshots.py_", ""]
            continue
        before, after = tb.load_snapshot(case), tb.compute_case(case)
        kb, ka = tb.key_figures(case, before), tb.key_figures(case, after)
        n_diff = len(tb.differences(before, after))
        total += n_diff
        rows = []
        for label in list(kb) + [x for x in ka if x not in kb]:
            a, b = kb.get(label), ka.get(label)
            d = _delta(a, b)
            if show_all or d:
                rows.append(f"| {label} | {_fmt(a)} | {_fmt(b)} | {d} |")
        lines += [f"## {case}", "", f"{n_diff} output value(s) differ from the snapshot (tolerance {tb.REL_TOL:g} relative).", ""]
        if rows:
            lines += ["| Figure | Before | After | Change |", "|---|---|---|---|", *rows]
        else:
            lines.append("_no key figure changed_")
        lines.append("")
    lines.insert(2, f"**{total} output value(s) differ in all.**\n")
    return "\n".join(lines)


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--all", action="store_true", help="show every key figure, not only the changed ones")
    ap.add_argument("-o", "--output", help="also write the report to this file")
    args = ap.parse_args(argv)
    text = report(args.all)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
