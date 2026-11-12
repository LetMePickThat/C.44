#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EXACT COMPARE -- headless exact calculation of a scenario's mission, for the comparison with the fast estimate
(Documentation/Notes_de_travail/estimate-vs-exact.md, 12 November 2026).

    python tools/exact_compare.py [scenario.json] [result.pkl]          (run from app/)

Defaults: scenarios/MOONTEST_VELOCITY.json, tools/.exact_compare.pkl. The mission is built as the GUI builds it
(sim.document.mission_for_calc: the scenario's navigation at t_nav_s). A step counter is printed live (step, elapsed,
stage of optimizer.run) and written to <result>.progress.txt (user rule, 12 November 2026). At the end: the verdict,
the peak test (V6.44.0, just need) and, nominal then at the peak, per face the utilisation of every resource and the
memory traffic against the bandwidth, with each item's share of the traffic (search, confirmation, tracks per volume).
The result (mission and run) is pickled for later reading. Several tens of minutes on a large mission: run it on the
main PC (memory), on mains power (a worker killed on battery broke the pool: BrokenProcessPool).
"""
from __future__ import annotations

import json
import os
import pickle
import sys
import time

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)

from core import compute, margin                       # noqa: E402
from core import optimizer as mo                       # noqa: E402
from core import tracking as trk                       # noqa: E402
from sim import document as doc_m                      # noqa: E402


class Steps:
    """The step counter: step n, elapsed minutes, stage; printed (flushed) and appended to the progress file."""

    def __init__(self, path: str):
        self.t0, self.n, self.path = time.perf_counter(), 0, path
        open(path, "w", encoding="utf-8").close()

    def __call__(self, stage: str) -> None:
        self.n += 1
        line = f"[step {self.n:3d} | {(time.perf_counter() - self.t0) / 60:6.1f} min] {stage}"
        print(line, flush=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(line + "\n")


def traffic(mission: dict, res: dict) -> dict:
    """Per face, the memory traffic (TB/s) of each item (kind and volume), from compute's look streams."""
    st = compute._streams(mission, res, compute._nulled_faces(mission))
    out = {}
    for k in range(4):
        per: dict = {}
        for b in ("S", "Ku"):
            for s in st[k][b]:
                v = sum(rate * c["mem_bits"] for rate, c, _ in s["looks"]) / 8e12
                key = f"{s['kind']} {s['sector']}"
                per[key] = per.get(key, 0.0) + v
        out[k] = per
    return out


def report(mission: dict, res: dict, steps: Steps) -> None:
    vd = res.get("verdict") or {}
    print("verdict", vd.get("state"), vd.get("counts"), "limiting", vd.get("limiting"))
    pk = res.get("peak") or {}
    print("peak", pk.get("status"), "shed", json.dumps(pk.get("shed"), default=str), "short", pk.get("short"))
    print("peak targets", pk.get("targets"))
    print("peak flags", *pk.get("flags", []), sep="\n  ")
    print("warnings at the peak", *pk.get("warnings_at_peak", [])[:15], sep="\n  ")
    views = (("NOMINAL", mission, res), ("PEAK", trk.with_view(mission, trk.PEAK), mo.peak_out(res)))
    for name, m_, r_ in views:
        if r_ is None:
            continue
        steps(f"report: {name} utilisation and memory traffic")
        u_ship, u = margin.utilisation(r_)
        print(f"--- {name}: ship energy {100 * u_ship:.1f} %")
        tr = traffic(m_, r_)
        for k in range(4):
            mem = r_["compute"]["faces"][str(k)]["memory"]
            print(f"F{k + 1} " + " ".join(f"{n} {100 * v:.1f}%" for n, v in sorted(u[k].items()))
                  + f" | traffic {mem['traffic_TB_per_s']:.1f} / {mem['bandwidth_TB_per_s']:g} TB/s")
            tot = sum(tr[k].values()) or 1.0
            print("     traffic by item:", ", ".join(f"{i} {v:.1f} ({100 * v / tot:.0f}%)"
                                                     for i, v in sorted(tr[k].items(), key=lambda x: -x[1]) if v > 0))
    print("non-OK rows", [(r["object"], r["quantity"], r["state"], None if r["load"] is None else round(r["load"], 3))
                          for r in vd.get("constraints", []) if r["state"] != "OK"])
    mr = res.get("margin_reuse") or {}
    print("margin reuse", mr.get("realloc_mode"), json.dumps({s: (round(v.get("g"), 3), v.get("limited_by"))
                                                             for s, v in (mr.get("sectors") or {}).items()}))
    print("sectors", [(r["id"], r.get("status"), r.get("rr_target_s")) for r in res["sectors"]])


def main(argv: list) -> int:
    scen = argv[1] if len(argv) > 1 else os.path.join(APP, "scenarios", "MOONTEST_VELOCITY.json")
    dest = argv[2] if len(argv) > 2 else os.path.join(APP, "tools", ".exact_compare.pkl")
    steps = Steps(dest + ".progress.txt")
    steps(f"load {os.path.basename(scen)}")
    mission, _alerts = doc_m.mission_for_calc(doc_m.load(scen))
    res = mo.run(mission, stage_cb=steps)
    steps("exact calculation done: pickling the result")
    with open(dest, "wb") as f:
        pickle.dump({"mission": mission, "res": res}, f)
    report(mission, res, steps)
    steps(f"done ({dest})")
    return 0


if __name__ == "__main__":                               # mandatory on Windows: the optimiser uses a process pool
    sys.exit(main(sys.argv))
