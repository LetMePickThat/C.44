#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EXACT COMPARE -- headless exact calculation of a scenario's mission, for the comparison with the fast estimate
(Documentation/Notes_de_travail/estimate-vs-exact.md, 12 November 2026).

    python tools/exact_compare.py [scenario.json] [result.pkl] [--workers N] [--cap-gb X] [--no-cache]   (from app/)

Defaults: scenarios/MOONTEST_VELOCITY.json, tools/.exact_compare.pkl. The mission is built as the GUI builds it
(sim.document.mission_for_calc: the scenario's navigation at t_nav_s). A step counter is printed live (step, elapsed,
stage of optimizer.run, RAM) and written to <result>.progress.txt (user rule, 12 November 2026). At the end: the
verdict, the peak test (V6.44.0, just need) and, nominal then at the peak, per face the utilisation of every resource
and the memory traffic against the bandwidth, with each item's share of the traffic (search, confirmation, tracks per
volume). The result (mission and run) is pickled for later reading.

Options (user, 12 November 2026: the first run on the main PC was stopped when the PC ran low on memory):
  --workers N   the optimiser's pool for THIS run only (optimizer.MAX_WORKERS; radar_system.toml unchanged): the
                frequencies of a volume are evaluated N at a time. Results identical to any other N (step 3.1, X3).
  --cap-gb X    hard ceiling of the PHYSICAL memory (working set) of every process of the run, the main one and each
                worker (Windows SetProcessWorkingSetSizeEx, hard max): above it Windows pages to disk, the run goes
                slower but goes on, the rest of the RAM stays free. Total = (1 + N) x X.
  --no-cache    do not use the resume cache.
Resume cache: each volume solve (optimizer._optimise_task, deterministic) is saved to tools/.exact_compare_cache/ as
soon as it is done, keyed by its inputs and by a fingerprint of core/, sim/ and the .toml files; a run cut short is
started again with the same command and re-reads the solves already done (identical result). A changed code or
parameter file gives other keys: the stale entries are never reused (delete the folder to free the disk).
RAM: every 30 s <result>.mem.txt gets the current and peak working set of the main process and of the workers and
the free RAM of the PC.
"""
from __future__ import annotations

import ctypes
import hashlib
import json
import os
import pickle
import sys
import threading
import time

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP)
GB = 1024 ** 3
_CAP_ENV = "EXACT_COMPARE_CAP_GB"                       # (inherited by the spawned workers)


def cap_this_process(cap_gb: float) -> str:
    """Hard maximum of this process's working set (Windows): pages above it go to the page file."""
    if os.name != "nt":
        return "memory cap: Windows only, not applied"
    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    k32.GetCurrentProcess.restype = ctypes.c_void_p
    k32.SetProcessWorkingSetSizeEx.argtypes = (ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_uint32)
    k32.SetProcessWorkingSetSizeEx.restype = ctypes.c_int
    QUOTA_LIMITS_HARDWS_MIN_DISABLE, QUOTA_LIMITS_HARDWS_MAX_ENABLE = 0x2, 0x4
    ok = k32.SetProcessWorkingSetSizeEx(k32.GetCurrentProcess(), 16 * 1024 ** 2, int(cap_gb * GB),
                                        QUOTA_LIMITS_HARDWS_MIN_DISABLE | QUOTA_LIMITS_HARDWS_MAX_ENABLE)
    if not ok:
        return f"memory cap {cap_gb:g} GB NOT applied (pid {os.getpid()}, Windows error {ctypes.get_last_error()})"
    return f"memory cap {cap_gb:g} GB applied (pid {os.getpid()})"


if __name__ != "__main__" and os.environ.get(_CAP_ENV):   # a spawned worker (module imported as __mp_main__)
    _msg = cap_this_process(float(os.environ[_CAP_ENV]))
    if "NOT" in _msg:
        print("worker: " + _msg, flush=True)

from core import compute, margin                       # noqa: E402
from core import optimizer as mo                       # noqa: E402
from core import tracking as trk                       # noqa: E402
from sim import document as doc_m                      # noqa: E402


def ram() -> dict:
    """Working sets (GB) of this process and of its child processes, free RAM of the PC."""
    import psutil
    me = psutil.Process()
    kids = []
    for c in me.children(recursive=True):
        try:
            kids.append(c.memory_info().rss / GB)
        except psutil.Error:
            pass
    return {"main": me.memory_info().rss / GB, "kids": kids, "free": psutil.virtual_memory().available / GB}


def ram_text(r: dict) -> str:
    k = r["kids"]
    kt = f"{len(k)} workers {sum(k):.1f} GB (max {max(k):.1f})" if k else "no worker"
    return f"RAM main {r['main']:.1f} GB, {kt}, PC free {r['free']:.1f} GB"


class Steps:
    """The step counter: step n, elapsed minutes, stage, RAM; printed (flushed) and appended to the progress file."""

    def __init__(self, path: str):
        self.t0, self.n, self.path = time.perf_counter(), 0, path
        open(path, "w", encoding="utf-8").close()

    def __call__(self, stage: str) -> None:
        self.n += 1
        try:
            mem = " | " + ram_text(ram())
        except Exception:  # noqa: BLE001 -- the counter never stops the run
            mem = ""
        line = f"[step {self.n:3d} | {(time.perf_counter() - self.t0) / 60:6.1f} min] {stage}{mem}"
        print(line, flush=True)
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(line + "\n")


def watch_ram(path: str, period_s: float = 30.0) -> None:
    """Every period_s, the current and peak RAM of the run to path (one line, overwritten); a daemon thread."""
    peak = {"main": 0.0, "kids": 0.0, "kid": 0.0, "min_free": float("inf")}
    t0 = time.perf_counter()

    def loop() -> None:
        while True:
            try:
                r = ram()
                peak["main"] = max(peak["main"], r["main"])
                peak["kids"] = max(peak["kids"], sum(r["kids"]))
                peak["kid"] = max(peak["kid"], max(r["kids"], default=0.0))
                peak["min_free"] = min(peak["min_free"], r["free"])
                with open(path, "w", encoding="utf-8") as f:
                    f.write(f"{(time.perf_counter() - t0) / 60:.1f} min | now: {ram_text(r)} | peak: main "
                            f"{peak['main']:.1f} GB, workers {peak['kids']:.1f} GB (one {peak['kid']:.1f}), "
                            f"PC free min {peak['min_free']:.1f} GB\n")
            except Exception:  # noqa: BLE001
                pass
            time.sleep(period_s)
    threading.Thread(target=loop, name="ram-watch", daemon=True).start()


# ---- resume cache over the volume solves --------------------------------------------------------------------------

def fingerprint() -> str:
    """Hash of the code and parameters the exact run depends on: core/ (calibration cache of the estimate left out),
    sim/, the .toml files of app/."""
    h = hashlib.sha256()
    files = []
    for sub in ("core", "sim"):
        for root, dirs, names in os.walk(os.path.join(APP, sub)):
            dirs[:] = sorted(d for d in dirs if d != "__pycache__")
            files += [os.path.join(root, n) for n in names if n != ".calibration_cache.json" and not n.endswith(".pyc")]
    files += [os.path.join(APP, n) for n in os.listdir(APP) if n.endswith(".toml")]
    for p in sorted(files):
        h.update(os.path.relpath(p, APP).encode())
        with open(p, "rb") as f:
            h.update(f.read())
    return h.hexdigest()


def _plain(x):
    import numpy as np
    if isinstance(x, np.ndarray):
        return ["ndarray", str(x.dtype), list(x.shape), x.tolist()]
    if isinstance(x, np.generic):
        return x.item()
    if isinstance(x, (set, frozenset)):
        return sorted(x, key=repr)
    if isinstance(x, tuple):
        return list(x)
    raise TypeError(type(x))


def install_cache(cache_dir: str, steps: Steps) -> None:
    """optimizer._optimise_task (one volume solve, called by name in the sequential run) behind a disk cache."""
    os.makedirs(cache_dir, exist_ok=True)
    fp = fingerprint()
    orig = mo._optimise_task

    def cached(sec, eps_min, eps_max, p_avg_max, sky=None):
        args = [sec, eps_min, eps_max, p_avg_max, sky]
        try:
            blob = json.dumps(args, sort_keys=True, default=_plain).encode()
        except (TypeError, ValueError):
            blob = pickle.dumps(args, protocol=4)
        key = hashlib.sha256(fp.encode() + blob).hexdigest()[:32]
        path = os.path.join(cache_dir, f"{sec.get('id', 'sector')}_{key}.pkl")
        if os.path.exists(path):
            try:
                with open(path, "rb") as f:
                    r = pickle.load(f)
                steps(f"resume cache: {sec.get('id')} read (solved by an earlier run)")
                return r
            except Exception:  # noqa: BLE001 -- a damaged entry: solved again
                pass
        t = time.perf_counter()
        r = orig(sec, eps_min, eps_max, p_avg_max, sky)
        try:
            with open(path + ".tmp", "wb") as f:
                pickle.dump(r, f, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(path + ".tmp", path)
            steps(f"resume cache: {sec.get('id')} solved in {(time.perf_counter() - t) / 60:.1f} min, saved")
        except Exception as exc:  # noqa: BLE001 -- not saved: the run goes on
            steps(f"resume cache: {sec.get('id')} solved, NOT saved ({exc})")
        return r
    mo._optimise_task = cached


# ---- report ---------------------------------------------------------------------------------------------------------

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
    pos, opt, i = [], {}, 1
    while i < len(argv):
        a = argv[i]
        if a in ("--workers", "--cap-gb"):
            opt[a] = argv[i + 1]
            i += 2
            continue
        if a == "--no-cache":
            opt[a] = True
        else:
            pos.append(a)
        i += 1
    scen = pos[0] if pos else os.path.join(APP, "scenarios", "MOONTEST_VELOCITY.json")
    dest = pos[1] if len(pos) > 1 else os.path.join(APP, "tools", ".exact_compare.pkl")
    steps = Steps(dest + ".progress.txt")
    if "--cap-gb" in opt:
        os.environ[_CAP_ENV] = opt["--cap-gb"]           # (before any worker is spawned: they inherit it)
        steps(cap_this_process(float(opt["--cap-gb"])))
    if "--workers" in opt:
        mo.MAX_WORKERS = int(opt["--workers"])
    steps(f"workers {mo.MAX_WORKERS} (this run), resume cache {'off' if '--no-cache' in opt else 'on'}")
    if "--no-cache" not in opt:
        install_cache(os.path.join(APP, "tools", ".exact_compare_cache"), steps)
    watch_ram(dest + ".mem.txt")
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
