"""Impact report: key figures of the frozen snapshots (before) against the current code (after), as Markdown.

    python tests/impact_report.py               all cases, rows that changed only
    python tests/impact_report.py --all         every row
    python tests/impact_report.py -o FILE.md    also write the report to a file

Each mission case also lists the S search waveform sets and the confirmation modes that changed (what the snapshot
journal tests/snapshots/JOURNAL.md records for each refreeze).

Test missions and test values only: the figures show what a code change does, not what the radar achieves.
"""
import argparse
import os
import sys

import testbench as tb
from core import optimizer as mo  # noqa: E402  (path set by testbench)


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


def _wf_label(sector: dict) -> str:
    """S search waveform set of one optimised sector: "COHE/NONC tau x N, TX beams, dwell" per waveform."""
    return " | ".join(f"{w['integration'][:4]} {w['tau_us']:g}us x{w['n_pulses']} "
                      f"{('TX ' + mo.tx_label(w)) if 'tx_mode' in w else 'B' + str(w['b_tx'])} {w['dwell_ms']:g}ms"
                      for w in sector.get("wf_set", []))


def waveform_changes(before: dict, after: dict) -> list:
    """Lines describing the waveform sets and confirmation modes that differ between two mission-case outputs."""
    if "run" not in before or "run" not in after:
        return []
    out = []
    old = {s["id"]: s for s in before["run"]["sectors"]}
    for s in after["run"]["sectors"]:
        o = old.get(s["id"])
        if o is not None and _wf_label(o) != _wf_label(s):
            out += [f"- {s['id']}: before {_wf_label(o)}", f"  after  {_wf_label(s)}"]
    old_c = {s["id"]: s["confirm_mode"] for s in before["run"].get("tracking", {}).get("sectors", [])}
    for s in after["run"].get("tracking", {}).get("sectors", []):
        if s["id"] in old_c and old_c[s["id"]] != s["confirm_mode"]:
            out.append(f"- {s['id']}: confirmation mode {old_c[s['id']]} -> {s['confirm_mode']}")
    return out


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
        changes = waveform_changes(before, after)
        if changes:
            lines += ["", "Waveform sets / confirmation modes that changed:", "", *changes]
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
