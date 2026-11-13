#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESTIMATE COMPARE -- the fast estimate of a mission against its exact run (Documentation/Notes_de_travail/
estimate-vs-exact.md, 12 November 2026: "the estimate must match the exact calculation much better").

    python tools/estimate_compare.py [exact.pkl]          (run from app/; default tools/.exact_compare.pkl)

Reads the mission and the exact result pickled by tools/exact_compare.py. Then:
  1. calibration.measure (NOT recorded): per volume and per face, exact / estimate of the energy and of the S time,
     at the same request (range, revisit): the estimate's own error, revisit aside;
  2. the live load as the views compute it (cost_estimate.live_allocation: cost profiles and relief computed here,
     "wait"), at the same instant (the mission's navigation, its sky in the ship frame), in four variants:
       A  realloc PRIO_SEARCH, calibration of the cache   (what the window shows)
       B  realloc NONE,        no calibration              (raw estimate at the revisit asked)
       C  realloc NONE,        calibration measured in 1   (recalibrated at the revisit asked)
       D  realloc PRIO_SEARCH, calibration measured in 1
     per face: energy drawn (MW, always-on included), S time, Ku time, and per volume its revisit served;
  3. the exact figures beside them (power_allocation need_MW, S / Ku time utilisation, revisits).
A step counter is printed live and written to <exact.pkl>.estimate_progress.txt (user rule, 12 November 2026).
"""
from __future__ import annotations

import os
import pickle
import sys
import time

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)

from core import calibration, frames, margin                 # noqa: E402
from core import cost_estimate as ce                         # noqa: E402
from core import geometry as geo                             # noqa: E402
from core.params import P                                    # noqa: E402


class Steps:
    def __init__(self, path: str):
        self.t0, self.n, self.path = time.perf_counter(), 0, path
        open(path, "w", encoding="utf-8").close()

    def __call__(self, stage: str) -> None:
        self.n += 1
        line = f"[step {self.n:3d} | {(time.perf_counter() - self.t0) / 60:6.1f} min] {stage}"
        print(line, flush=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(line + "\n")


def faces_line(alloc: dict) -> list:
    out = []
    for k in range(4):
        f = alloc["faces"][str(k)]
        out.append(f"F{k + 1} energy {f['energy'] / 1e6:.3f} MW, S time {100 * f['S time']:.2f} %, "
                   f"Ku time {100 * f['Ku time']:.2f} %")
    return out


def main(argv: list) -> int:
    src = argv[1] if len(argv) > 1 else os.path.join(APP, "tools", ".exact_compare.pkl")
    steps = Steps(src + ".estimate_progress.txt")
    steps(f"load {os.path.basename(src)}")
    with open(src, "rb") as f:
        d = pickle.load(f)
    mission, res = d["mission"], d["res"]

    steps("1. calibration.measure: exact / estimate per volume and face, same request (not recorded)")
    meas = calibration.measure(mission, res)
    for sid, v in sorted(meas.items()):
        print(f"   {sid}: energy x " + " ".join(f"{x:.2f}" for x in v["e"]) + " | S time x "
              + " ".join(f"{x:.2f}" for x in v["t"]))
    cache = calibration.view_calibration(mission["sectors"], [s["id"] for s in mission["sectors"]])
    print("   calibration in the cache for:", sorted(k for k in cache if k != ce.ALWAYS_ON_KEY) or "none",
          "| always-on measured:", ce.ALWAYS_ON_KEY in cache)
    for sid in sorted(k for k in cache if k != ce.ALWAYS_ON_KEY):
        v = cache[sid]
        print(f"   cache {sid}: energy x " + " ".join(f"{x:.2f}" for x in v["e"]) + " | S time x "
              + " ".join(f"{x:.2f}" for x in v["t"]))

    resolved, _alerts = frames.resolve(mission)
    g = resolved["global"]
    secs = [dict(s) for s in resolved["sectors"]]
    sky = g.get("sky")
    e0, e1 = float(g.get("eps_min_deg", -90.0)), float(g.get("eps_max_deg", 90.0))
    grid = geo.SphereGrid(float(P.get("tool.display.grid_res_deg")))
    table = ce.cached_table()
    print("   cost table current:", table.is_current)
    on = calibration.always_on(mission["sectors"])
    meas_c = {sid: {"e": v["e"], "t": v["t"]} for sid, v in meas.items()}
    exact_on = [float(res["power_allocation"]["faces"][str(k)]["always_on_MW"]) * 1e6 for k in range(4)]
    meas_c[ce.ALWAYS_ON_KEY] = exact_on
    variants = (("A PRIO_SEARCH, cache calibration (the window)", "PRIO_SEARCH", cache),
                ("B NONE, no calibration", "NONE", None),
                ("C NONE, calibration measured in 1", "NONE", meas_c),
                ("D PRIO_SEARCH, calibration measured in 1", "PRIO_SEARCH", meas_c))
    for name, mode, calib in variants:
        steps(f"2. live estimate {name}")
        alloc = ce.live_allocation(table, secs, grid, e0, e1, sky, dict(g, realloc_mode=mode), calib=calib,
                                   profiles="wait", relief="wait")
        print(f"   status {alloc.get('status')}, relief {({s: round(v['share'], 2) for s, v in (alloc.get('relief') or {}).items()})}")
        for line in faces_line(alloc):
            print("   " + line)
        ol = alloc.get("other_loads") or {}
        print("   instant priced:", alloc.get("instant_priced"), "| estimating:", alloc.get("estimating"),
              "| time lower bound:", alloc.get("time_lower_bound"))
        print("   memory bandwidth / compute asked:", {n: round(100 * v, 1) for n, v in sorted((ol.get("asked") or {}).items())},
              "| reuse limited by:", ol.get("reuse_limited_by"))
        print("   confirmation:", [(d["label"], {k: round(v.get("energy", 0.0) / 1e3, 1) for k, v in d["faces"].items()})
                                   for d in alloc.get("demands", []) if str(d.get("label", "")).startswith("confirmation")],
              "(kW per face)")
        print("   revisit served:", {sid: round(float(r.get("rr_tenable_s", 0.0)), 2)
                                     for sid, r in sorted(alloc.get("by_sector", {}).items())},
              "| range held:", {sid: round(float(r.get("rho", 1.0)), 3) for sid, r in sorted(alloc.get("by_sector", {}).items())})
    steps("3. exact")
    _u_ship, u = margin.utilisation(res)
    for k in range(4):
        f = res["power_allocation"]["faces"][str(k)]
        print(f"   F{k + 1} energy {f['need_MW']:.3f} MW (always-on {f['always_on_MW']:.3f}), "
              f"S time {100 * u[k]['S time']:.2f} %, Ku time {100 * u[k]['Ku time']:.2f} %")
    print("   revisits:", {r["id"]: r.get("rr_target_s") for r in res["sectors"]},
          "| reuse g:", {s: round(v.get("g"), 3) for s, v in ((res.get("margin_reuse") or {}).get("sectors") or {}).items()})
    print("   always-on: exact", [round(x / 1e6, 3) for x in exact_on], "MW, cache",
          None if on is None else [round(x / 1e6, 3) for x in on], ", static", round(ce.always_on_static_w() / 1e6, 3))
    steps("done")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
