"""N4 -- FLEXIBILITY BATTERY of the model (user, 14 November 2026, step 5.7 point R0): the model must follow a change of
its parameters without the physics breaking and without code to rewrite. E.g. the final design may have slightly larger
arrays: more TRSAs (the octagon one ring larger), or more TRMs on each TRSA; the ship geometry and the antennas'
placement are fixed (the centre of each array stays where it is), only the array's surface and outline change.

Each variant is a copy of radar_system.toml edited by core/params_edit (the operator's own edit path), run in its own
process (RADAR_PARAMS) with its own fast-estimate table (RADAR_COST_TABLE: never the repository's), and compared with
the default file run the same way:
  - consistency: TRSA and TRM counts, emissive area, peak power from the file;
  - physics: boresight gain follows the area (10 log of the ratio), the reach of a fixed waveform moves the right way;
  - the whole chain runs: optimiser, tracking, scheduler, verdict on a test mission; the fast-estimate table is built
    and the live allocation runs; the energy needed moves the right way.
On demand only (python tests/run_tests.py N4; RADAR_N4=1): every other pass leaves these tests out (tests/conftest.py).
The variants run one after the other (each cost table already uses every core: 58 processes at once when run in
parallel, 14 November 2026), the default file once, with the repository's current table.
"""
import json
import math
import os
import subprocess
import sys

import pytest

pytestmark = pytest.mark.n4

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

TRSA_SIDE_M = 0.26
VARIANTS = {
    "default": {},
    "grow_one_ring": {"aperture.trsa_rows": [6, 8, 10, 12, 14, 16, 18, 18, 18, 18, 18, 18, 16, 14, 12, 10, 8, 6]},
    "shrink_one_ring": {"aperture.trsa_rows": [6, 8, 10, 12, 14, 14, 14, 14, 14, 14, 12, 10, 8, 6]},
    "more_ku_trm_per_trsa": {"aperture.ku_trm_per_trsa_side": 30, "aperture.ku_trm_pitch_m": TRSA_SIDE_M / 30},
    "more_ku_and_s_trm_per_trsa": {"aperture.ku_trm_per_trsa_side": 30, "aperture.s_trm_per_trsa_side": 6,
                                   "aperture.ku_trm_pitch_m": TRSA_SIDE_M / 30},
}

PROBE = r'''
import json, math, os, sys
sys.path.insert(0, os.getcwd())
from core import radar_model as rm, optimizer as mo, mission as md
out = {"n_trsa": rm.N_TRSA, "n_trm": dict(rm.N_TRM), "area_m2": rm.PHYS_AREA_M2, "trsa_side_m": rm.TRSA_SIDE_M,
       "ku_side": rm.KU_TRM_PER_TRSA_SIDE, "s_side": rm.S_TRM_PER_TRSA_SIDE}
out["p_pk_w"] = {b: rm.BANDS[b].p_pk_total_w for b in ("S", "Ku")}
f = {"S": 2.8, "Ku": 14.0}
out["gain_db"] = {b: 10 * math.log10(rm.gain_lin(rm.BANDS[b].eta_illum_tx, f[b], 0.0)) for b in ("S", "Ku")}
w = mo.make_wf("probe", 100e-6, 16, 2e-3, 5.0, 1, "COHERENT")
tgt = rm.Target("probe", 1.0, 1, 1000.0)
res = rm.solve(w, 2.8, 0.0, tgt)
out["probe_range_km"] = float(res.range_km) if hasattr(res, "range_km") else float(res.r_km)
run = mo.run(md.load_mission(os.path.join("tests", "missions", "mission_example.csv")), workers=1)
vd = run["verdict"]
ship = next(r for r in vd["constraints"] if r["family"] == "ship" and r["quantity"] == "energy")
out["run"] = {"state": vd["state"], "n_constraints": len(vd["constraints"]), "ship_energy_mw": ship["value"],
              "statuses": {r["id"]: r["status"] for r in run["sectors"]},
              "worst_km": {r["id"]: (r.get("range_km") or {}).get("at_worst_scan") for r in run["sectors"]},
              "has": [k for k in ("tracking", "scheduler", "compute", "power_allocation") if run.get(k)]}
pa = (run.get("power_allocation") or {}).get("faces") or {}
out["run"]["always_on_mw"] = sum(float(f_.get("always_on_MW", 0.0)) for f_ in pa.values())
out["run"]["tasks_mw"] = sum(float(f_.get("alloc_MW", 0.0)) - float(f_.get("always_on_MW", 0.0)) for f_ in pa.values())
from core import cost_estimate as ce, geometry as geo
table = ce.load_table()
grid = geo.SphereGrid(2.0)
sec = dict(md.new_sector([]), id="A")
a = ce.live_allocation(table, [sec], grid, -90.0, 90.0, None, {"realloc_mode": "NONE"})
out["fast"] = {"table_ok": table is not None and table.is_current, "status": a["status"],
               "energy_w": max(f_["energy"] for f_ in a["by_sector"]["A"]["faces"].values())}
print("PROBE " + json.dumps(out))
'''


def _variant_file(name: str, tmp: str) -> str:
    from core import params_edit as pe
    text = open(os.path.join(ROOT, "radar_system.toml"), encoding="utf-8").read()
    if VARIANTS[name]:
        text = pe.apply(text, VARIANTS[name], "2026-11-14")
    path = os.path.join(tmp, f"radar_system_{name}.toml")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path


def _probe(name: str, tmp: str) -> dict:
    env = {k: v for k, v in os.environ.items() if k not in ("RADAR_PARAMS", "RADAR_COST_TABLE")}
    if VARIANTS[name]:                      # (the default file: the repository's own, current table, nothing rebuilt)
        params = _variant_file(name, tmp)
        table = os.path.join(tmp, f"cost_table_{name}.json")
        env.update(RADAR_PARAMS=params, RADAR_COST_TABLE=table)
        gen = subprocess.run([sys.executable, os.path.join("tools", "gen_cost_table.py")], cwd=ROOT, env=env,
                             capture_output=True, text=True, timeout=3600)
        assert gen.returncode == 0, f"{name}: cost table not built\n{gen.stderr[-3000:]}"
    res = subprocess.run([sys.executable, "-c", PROBE], cwd=ROOT, env=env, capture_output=True, text=True,
                         timeout=7200)
    assert res.returncode == 0, f"{name}: the model failed with this parameter file\n{res.stderr[-4000:]}"
    line = next(x for x in res.stdout.splitlines() if x.startswith("PROBE "))
    return json.loads(line[6:])


_CACHE = {}


def _get(name: str, tmp_path_factory) -> dict:
    if name not in _CACHE:
        _CACHE[name] = _probe(name, str(tmp_path_factory.mktemp(name)))
    return _CACHE[name]


@pytest.mark.parametrize("name", [n for n in VARIANTS if n != "default"])
def test_model_follows_the_array(name, tmp_path_factory):
    """A variant of the arrays: counts and area from the file, gain and reach move as the physics says, the whole chain
    runs (optimiser, tracking, scheduler, verdict, fast estimate)."""
    base, var = _get("default", tmp_path_factory), _get(name, tmp_path_factory)
    v = VARIANTS[name]
    # --- consistency with the file
    rows = v.get("aperture.trsa_rows")
    if rows is not None:
        assert var["n_trsa"] == sum(rows)
    assert var["n_trm"]["Ku"] == var["n_trsa"] * var["ku_side"] ** 2
    assert var["n_trm"]["S"] == var["n_trsa"] * var["s_side"] ** 2
    assert var["area_m2"] == pytest.approx(var["n_trsa"] * var["trsa_side_m"] ** 2, rel=1e-9)
    for b in ("S", "Ku"):
        assert var["p_pk_w"][b] / base["p_pk_w"][b] == pytest.approx(var["n_trm"][b] / base["n_trm"][b], rel=1e-9)
    # --- physics
    ratio = var["area_m2"] / base["area_m2"]
    for b in ("S", "Ku"):
        assert var["gain_db"][b] - base["gain_db"][b] == pytest.approx(10 * math.log10(ratio), abs=0.5)
    if ratio > 1.0 + 1e-9:
        assert var["probe_range_km"] > base["probe_range_km"]
    elif ratio < 1.0 - 1e-9:
        assert var["probe_range_km"] < base["probe_range_km"]
    else:                                   # (same aperture, more TRMs: more peak power, never less reach)
        assert var["probe_range_km"] >= base["probe_range_km"] * (1 - 1e-6)
    # --- the whole chain
    run = var["run"]
    assert run["n_constraints"] > 0 and set(run["has"]) == {"tracking", "scheduler", "compute", "power_allocation"}
    assert all(s in ("OK", "RELIEVED", "NO_PLAN", "UNREACHABLE", "PARTIAL") for s in run["statuses"].values()), run
    # (the ship's draw is mostly the always-on, ~71 kW a face on the test mission: the RX chains and ADCs of every
    # TRM, which grows with the TRM count; the tasks' share follows the aperture: 14 November 2026, first N4)
    n_ratio = sum(var["n_trm"].values()) / sum(base["n_trm"].values())
    if n_ratio > 1.0 + 1e-9:
        assert run["always_on_mw"] > base["run"]["always_on_mw"]
    elif n_ratio < 1.0 - 1e-9:
        assert run["always_on_mw"] < base["run"]["always_on_mw"]
    if ratio > 1.0 + 1e-9:
        assert run["tasks_mw"] <= base["run"]["tasks_mw"] * 1.02
    elif ratio < 1.0 - 1e-9:
        assert run["tasks_mw"] >= base["run"]["tasks_mw"] * 0.98
    fast = var["fast"]
    assert fast["table_ok"] and fast["status"] in ("OK", "DEGRADED")
    if ratio > 1.0 + 1e-9:
        assert fast["energy_w"] <= base["fast"]["energy_w"] * 1.02
