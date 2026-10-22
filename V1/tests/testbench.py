"""
TESTBENCH -- shared by the test suite (pytest), tests/update_snapshots.py and tests/impact_report.py.

Computes the numeric outputs of the tool for the test cases, flattens them, compares them with the frozen snapshots
(tests/snapshots/*.json) and extracts the key figures of the impact report.

TEST VALUES ONLY: the missions of tests/missions/ are test missions, and the snapshots are the tool's own outputs
frozen at a given commit. They detect unintended changes of the code; they are NOT radar requirements or achieved
performance (the radar's capabilities are not established).
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys
from typing import Any, Callable, Dict, List, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from core import cost_estimate as ce  # noqa: E402
from core import detection  # noqa: E402
from core import frames  # noqa: E402
from core import geometry as geo  # noqa: E402
from core import isolation as iso  # noqa: E402
from core import mission as md  # noqa: E402
from core import optimizer as mo  # noqa: E402
from core import radar_model as rm  # noqa: E402
from core import redundancy as red  # noqa: E402
from core import sky  # noqa: E402
from core import tracking as trk  # noqa: E402

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
MISSIONS_DIR = os.path.join(TESTS_DIR, "missions")
SNAPSHOTS_DIR = os.path.join(TESTS_DIR, "snapshots")
REL_TOL = 1e-9          # [user 2026-10-20] relative tolerance of the snapshot comparison
ABS_TOL = 1e-15         # below this, two numbers are equal (zeros computed through different paths)
GRID_RES_DEG = 0.5      # raster of the redundancy check and the fast estimate (= tool.display.grid_res_deg default)

MISSIONS = ("mission_example.json", "mission_example.csv", "mission_veille500.json",
            "mission_redundancy_demo.json", "mission_synthetic_mix.json", "mission_sky_demo.json",
            "mission_target_library.json", "mission_environment.json", "mission_frames.json")
# Bodies on the face 0 boresight (phi 0, eps = tilt) for the sky-noise terms of the model case (step 1.5)
SKY_BODIES = {"sun": 0.267, "moon": 0.26, "earth": 60.0}

# Synthetic Ku waveforms of the model case (no Ku waveform in the model catalogue)
KU_TRACK = rm.Waveform(name="TEST-KU-TRK", role="TRACK", band="Ku", tau_s=40e-6, n=128, pri_s=500e-6,
                       sig_bw_mhz=75.0, integration="COHERENT", tracking_pack=rm.TrackingPack(0.05, 2.0, 5.0))
KU_CONFIRM = rm.Waveform(name="TEST-KU-CONF", role="CONFIRM", band="Ku", tau_s=100e-6, n=8, pri_s=1e-3,
                         sig_bw_mhz=200.0, integration="COHERENT")


def environment_terms() -> dict:
    """Step 2.5 terms of the model case: structure tables per face, CFAR variants, jammer classes, surfaces."""
    from core import environment as env
    t = env.tables()
    out = {"cfar_db": {f"{kind} {n}": fn(n, 1e-8) for n in (64, 1024)
                       for kind, fn in (("CA", rm.cfar_loss_db), ("OS", rm.os_cfar_loss_db), ("GO", rm.go_cfar_loss_db))},
           "jammer_classes": env.JAMMER_CLASSES, "surfaces": env.SURFACES,
           "faraday_deg": env.faraday_round_trip_deg(50.0, 20000.0, 2.8)}
    for k in range(4):
        out[f"face {k}"] = {"blocked_cells": int((t[f"f_{k}"] > 0).sum()), "blocked_sum": float(t[f"f_{k}"].sum()),
                            "structure_dirs": int(len(t[f"sdirs_{k}"])),
                            "structure_epst_sum": float(t[f"sepst_S_{k}"].sum())}
    return out


def to_plain(x: Any) -> Any:
    """JSON-ready copy: numpy and dataclasses converted, dict keys as strings, tuples as lists."""
    if isinstance(x, dict):
        return {str(k): to_plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return to_plain(x.tolist())
    if isinstance(x, np.bool_):
        return bool(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.floating):
        return float(x)
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return to_plain(dataclasses.asdict(x))
    if isinstance(x, (bool, int, float, str, type(None))):
        return x
    return repr(x)


def _try(fn: Callable[[], Any]) -> Any:
    try:
        return fn()
    except ValueError as exc:
        return f"ValueError: {exc}"


# ----------------------------------------------------------------------------- cases
def lin_to_db_gain(f: float, th: float) -> list:
    """TX and RX gain (dBi) of the band containing f at scan th."""
    b = rm.band_for_frequency(f)
    return [rm.lin_to_db(rm.gain_lin(b.eta_illum_tx, f, th)), rm.lin_to_db(rm.gain_lin(rm.rx_beam(b.name).eta, f, th))]


def sky_rise_terms() -> dict:
    """Antenna-temperature rise of a body centred on the face 0 boresight, beam offset in elevation (step 1.5)."""
    frame = mo.face_frame(0.0)
    out = {}
    for typ, rad in SKY_BODIES.items():
        s = sky.parse({"bodies": [{"id": typ, "type": typ, "phi_deg": 0.0, "eps_deg": rm.SECTOR_TILT_DEG,
                                   "angular_radius_deg": rad}]})
        offs = np.array([0.0, 0.5, 1.0, 2.0, 5.0, 20.0])
        dirs = sky.unit(np.zeros(len(offs)), rm.SECTOR_TILT_DEG + offs)
        for b, f in (("S", 2.8), ("Ku", 14.0)):
            r = sky.body_rise_k(s.bodies[0], b, f, -25.0, frame, dirs)
            out.update({f"{typ} | {b} | offset {o:g} deg": float(v) for o, v in zip(offs, r)})
    return out


def model_case() -> dict:
    """Radar model alone: solve() over a grid of frequencies / scan angles / targets, positions, loss terms."""
    out: Dict[str, Any] = {}
    sol = {}
    for f in (2.7, 2.75, 2.8, 2.88, 2.9):
        for th in (0.0, 30.0, 60.0, 70.0, 74.0):
            for name, tgt in rm.TARGETS.items():
                sol[f"S {f} GHz | {th} deg | {name}"] = _try(lambda: rm.solve(rm.WF_LRS, f, th, tgt))
    for w in (KU_TRACK, KU_CONFIRM):
        for f in (13.6, 14.0, 14.4):
            for th in (0.0, 50.0, 70.0):
                sol[f"{w.name} {f} GHz | {th} deg | KKV"] = _try(lambda: rm.solve(w, f, th, rm.TARGETS["KKV"]))
    for sll in rm.RX_SLL_OPTIONS_DB:                     # RX weighting options of a watch volume
        sol[f"S 2.8 GHz | 30.0 deg | ST | RX {sll:g} dB"] = _try(
            lambda: rm.solve(rm.WF_LRS, 2.8, 30.0, rm.TARGETS["ST"], rx_sll_db=sll))
        sol[f"{KU_TRACK.name} 14.0 GHz | 30.0 deg | KKV | RX {sll:g} dB"] = _try(
            lambda: rm.solve(KU_TRACK, 14.0, 30.0, rm.TARGETS["KKV"], rx_sll_db=sll))
    for r in sol.values():
        if isinstance(r, rm.Result):
            r.wf = r.wf.name       # the waveform is an input: keep its name only
            r.target = r.target.name
    out["solve"] = sol
    out["positions"] = {f"{f} GHz | eps {lo}..{hi}": rm.n_search_positions(f, lo, hi)
                        for f in (2.7, 2.8, 2.9) for lo, hi in ((-90.0, 90.0), (-20.0, 90.0), (-90.0, 30.0))}
    out["terms"] = {
        "grid_loss_db": rm.grid_loss_db(), "chain_eff": rm.CHAIN_EFF, "face_elec_avail_w_2MW": rm.face_elec_avail_w(2.0),
        "array_envelope_m": rm.ARRAY_ENVELOPE_M, "sector_area_uv": rm.sector_area_uv(),
        "hpbw_deg": {f"{f} GHz | {th} deg": rm.hpbw_deg(f, th) for f in (2.8, 14.0) for th in (0.0, 40.0, 70.0)},
        "patterns": rm.PATTERNS, "grid_rx_loss_db": rm.grid_rx_loss_db(),
        "pae": {f"{b} | tau {t * 1e6:g} us | d {d}": rm.pae(rm.BANDS[b], t, d)
                for b in ("S", "Ku") for t in (10e-6, 100e-6, 2e-3) for d in (None, 0.05, 0.3, 0.65)},
        "tau_max_s": {b: rm.tau_max_s(rm.BANDS[b]) for b in rm.BANDS},
        "active_mismatch": {f"{f} GHz | {th} deg": rm.active_mismatch(f, th)
                            for f in (2.7, 2.8, 2.85, 2.9, 13.5, 14.0, 14.5) for th in (0.0, 30.0, 45.0, 60.0, 70.0, 75.0)},
        "pol_purity": {f"{th} deg | phi {p}": rm.pol_purity(th, p) for th in (0.0, 45.0, 70.0) for p in (0.0, 22.5, 45.0)},
        "sidelobe_visible_db": {f"{f} GHz | {th} deg | RX {s:g}": rm.sidelobe_visible_db(f, th, "rx", s)
                                for f in (2.8, 2.9, 14.0, 14.5) for th in (0.0, 40.0, 70.0) for s in rm.RX_SLL_OPTIONS_DB},
        "rx_sll_held_up_to_deg": {f"{f} GHz | RX {s:g}": rm.rx_sll_held_up_to_deg(f, s, 75.0)
                                  for f in (2.8, 2.9, 14.0, 14.5) for s in rm.RX_SLL_OPTIONS_DB},
        "angle_k_tx": {b: rm.angle_k_tx(b) for b in rm.BANDS},
        "rx_beams_per_position": {f"{f} GHz | RX {s:g} dB": rm.rx_beams_per_position(f, s)
                                  for f in (2.8, 14.0) for s in rm.RX_SLL_OPTIONS_DB},
        "gain_dbi": {f"{f} GHz | {th} deg": lin_to_db_gain(f, th) for f in (2.8, 14.0) for th in (0.0, 30.0, 50.0, 70.0, 75.0)},
        "multibeam_tx_loss_db": {str(b): rm.multibeam_tx_loss_db(b) for b in (1, 2, 4, 8)},
        "coherence_loss_db": {str(t): rm.coherence_loss_db(14.0, t) for t in (1e-3, 0.064, 0.3)},
        "edge_loss_db": {f"{f} GHz | B {b:g} MHz": rm.BANDS[n].edge_loss_db(f, b)
                         for n, fs in (("S", (2.7, 2.72, 2.75, 2.8, 2.88, 2.9)), ("Ku", (13.5, 13.6, 14.0, 14.4, 14.5)))
                         for f in fs for b in (0.0, 10.0, 75.0)},
        "shnidman_db": {f"N {n} SW{s}": rm.shnidman_total_snr_db(rm.DETECT_PD, rm.DETECT_PFA, n, s)
                        for n in (1, 8, 24, 128) for s in (0, 1, 2, 3, 4)},
        "bands": {b.name: b for b in rm.BANDS.values()}, "targets": rm.TARGETS,
        "ship_array_radius": geo.RP,
        "environment": environment_terms(),
        "isolation": iso.matrix(), "scheduler_rules": iso.scheduler_rules(),
        "min_unsaturated_range_km": {f"{f} GHz | {n}": rm.min_unsaturated_range_km(f, 0.0, t.rcs_m2)
                                     for f in (2.8, 14.0) for n, t in rm.TARGETS.items()},
        "pol_loss_db": {f"{b} {role} n_nc {n} SW{sw}": rm.pol_recombination_loss_db(rm.BANDS[b], role, n, sw)
                        for b, role in (("S", "SEARCH"), ("Ku", "CONFIRM"), ("Ku", "TRACK"))
                        for n in (1, 8) for sw in (1, 3)},
        "sky_rise_k": sky_rise_terms(),
        "range_weighting": {f"{s:g} dB": rm.range_weighting(s) for s in rm.RX_SLL_OPTIONS_DB},
        "velocity_mismatch_loss_db": {f"n_c {n:g}": rm.velocity_mismatch_loss_db(n) for n in (0.25, 0.5, 1.0, 2.0, 10.0)},
        "exact_snr_req_db": {f"N {n} SW{sw} Pfa {pfa:g}": detection.snr_req_db(n, sw, pfa, rm.DETECT_PD)
                             for n in (1, 8) for sw in range(5) for pfa in (1e-8, 1e-12)},
        "cumulative_snr_req_db": {f"q {q:g}": detection.required_snr_db(1, 1, 1e-8, rm.DETECT_PD, q=q)
                                  for q in (0.5, 0.1, 0.02)},
        "modulation": {f"{f} GHz | B {b:g} | v {v:g}": rm.modulation_for(f, b, v)
                       for f in (2.8, 14.0) for b in (10.0, 75.0, 200.0) for v in (5000.0, 25000.0)},
    }
    out["confirmation_tables"] = {f"SW{sw}": trk.confirm_tables(sw, 2.8) for sw in (1, 3)}
    out["ku_look"] = {f"{th} deg | {r} km | {'track' if pack else 'detect'}":
                      trk.ku_look(th, rm.TARGETS["KKV"], r, rm.face_elec_avail_w(2.0), pack)
                      for th in (0.0, 60.0) for r in (300.0, 800.0)
                      for pack in (None, rm.TrackingPack(0.05, 5.0, 10.0))}
    for v in out["ku_look"].values():
        if v is not None:
            v["wf"] = trk._wf_label(v["wf"])
    return to_plain(out)


def run_mission(name: str) -> dict:
    """optimizer.run() of one test mission, sequential (module level: a process-pool task)."""
    return mo.run(md.load_mission(os.path.join(MISSIONS_DIR, name)), workers=1)


def mission_case(name: str, grid: "geo.SphereGrid", table) -> dict:
    """One test mission through every calculation of the tool."""
    m = md.load_mission(os.path.join(MISSIONS_DIR, name))
    g = {**md.GLOBAL_DEFAULTS, **m.get("global", {})}
    out: Dict[str, Any] = {"validation": md.validate_mission(m)}
    out["run"] = mo.run(m, workers=1)
    findings, notes = red.find_redundancies(m, grid)
    out["redundancy"] = {"findings": findings, "pair_notes": {f"{a} > {b}": t for (a, b), t in notes.items()}}
    out["fast_estimate"] = {s["id"]: ce.estimate_sector_faces(table, s, grid, g["eps_min_deg"], g["eps_max_deg"],
                                                              rm.face_elec_avail_w(g["face_elec_mw"]))
                            for s in frames.resolve(m)[0]["sectors"]}   # the GUI also estimates in the ship frame
    return to_plain(out)


def case_names() -> List[str]:
    return ["model"] + [os.path.splitext(n)[0] + ("_csv" if n.endswith(".csv") else "") for n in MISSIONS]


def compute_all() -> Dict[str, dict]:
    grid = geo.SphereGrid(res_deg=GRID_RES_DEG)
    table = ce.load_table()
    out = {"model": model_case()}
    for n, case in zip(MISSIONS, case_names()[1:]):
        out[case] = mission_case(n, grid, table)
    return out


def compute_case(case: str) -> dict:
    if case == "model":
        return model_case()
    n = MISSIONS[case_names().index(case) - 1]
    return mission_case(n, geo.SphereGrid(res_deg=GRID_RES_DEG), ce.load_table())


# ----------------------------------------------------------------------------- snapshots
def snapshot_path(case: str) -> str:
    return os.path.join(SNAPSHOTS_DIR, f"{case}.json")


def load_snapshot(case: str) -> dict:
    with open(snapshot_path(case), encoding="utf-8") as fh:
        return json.load(fh)


def save_snapshot(case: str, data: dict) -> None:
    os.makedirs(SNAPSHOTS_DIR, exist_ok=True)
    with open(snapshot_path(case), "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=1, sort_keys=True, ensure_ascii=False)
        fh.write("\n")


def _same(a: Any, b: Any) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b or a == b and type(a) is type(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if math.isnan(a) or math.isnan(b):
            return math.isnan(a) and math.isnan(b)
        return math.isclose(a, b, rel_tol=REL_TOL, abs_tol=ABS_TOL)
    return a == b


def differences(old: Any, new: Any, path: str = "") -> List[Tuple[str, Any, Any]]:
    """(path, frozen value, current value) of every difference beyond the tolerance."""
    if isinstance(old, dict) and isinstance(new, dict):
        out = []
        for k in list(old) + [k for k in new if k not in old]:
            p = f"{path}/{k}"
            if k not in new:
                out.append((p, old[k], "<missing>"))
            elif k not in old:
                out.append((p, "<missing>", new[k]))
            else:
                out.extend(differences(old[k], new[k], p))
        return out
    if isinstance(old, list) and isinstance(new, list):
        if len(old) != len(new):
            return [(path, f"<{len(old)} items>", f"<{len(new)} items>")]
        out = []
        for i, (a, b) in enumerate(zip(old, new)):
            out.extend(differences(a, b, f"{path}[{i}]"))
        return out
    return [] if _same(old, new) else [(path or "/", old, new)]


def format_differences(diffs: List[Tuple[str, Any, Any]], limit: int = 40) -> str:
    def short(v: Any) -> str:
        s = json.dumps(v, ensure_ascii=False) if not isinstance(v, str) else v
        return s if len(s) <= 120 else s[:117] + "..."
    lines = [f"{p} : {short(a)} -> {short(b)}" for p, a, b in diffs[:limit]]
    if len(diffs) > limit:
        lines.append(f"... and {len(diffs) - limit} more")
    return "\n".join(lines)


# ----------------------------------------------------------------------------- impact report key figures
def key_figures(case: str, data: dict) -> Dict[str, Any]:
    """The figures of the impact report for one case (flat, label -> value)."""
    k: Dict[str, Any] = {}
    if case == "model":
        for name in ("ST", "KKV", "SHIP"):
            for th in (0.0, 70.0):
                r = data["solve"].get(f"S 2.8 GHz | {th} deg | {name}")
                k[f"WF-LRS range {name} @ {th:g} deg (km)"] = r["range_km"] if isinstance(r, dict) else r
        for f in (2.7, 2.8, 2.9):
            k[f"positions per face @ {f} GHz"] = data["positions"][f"{f} GHz | eps -90.0..90.0"]
        return k
    run = data["run"]
    for s in run["sectors"]:
        sid = s["id"]
        k[f"{sid} status"] = s["status"]
        if s["status"] != "OK":
            continue
        k[f"{sid} RR est. (s)"] = s.get("rr_est_mission_s")
        k[f"{sid} range at worst scan (km)"] = s["range_km"]["at_worst_scan"]
        k[f"{sid} positions short of range (%)"] = s["range_km"]["positions_short_of_range_pct"]
        k[f"{sid} unscanned (%)"] = s["unscanned_pct"]
        if "sky" in s:
            k[f"{sid} sky budget T_sys S (K)"] = s["sky"]["t_sys_budget_k"]
            k[f"{sid} blinded S (%)"] = s["sky"]["blinded_pct"]
        k[f"{sid} flags"] = len(s.get("flags", []))
    trk_faces = run.get("tracking", {}).get("faces", {})
    for f, v in trk_faces.items():
        if v["load"] == 0.0 and v["total_energy"] == 0.0:
            continue
        tag = f"F{int(f) + 1}"
        k[f"{tag} energy"] = v["total_energy"]
        k[f"{tag} S time"] = v["total_s_time"]
        k[f"{tag} Ku time"] = v["total_ku_time"]
        k[f"{tag} load ({v['binding']})"] = v["load"]
    for s in run.get("tracking", {}).get("sectors", []):
        k[f"{s['id']} confirmation mode"] = s["confirm_mode"]
    return k
