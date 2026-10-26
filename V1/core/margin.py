#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MARGIN REUSE -- the resources a mission leaves free are put to use (roadmap step 5.2, pass P4, rule of the user,
26 October 2026).

The revisit asked for each search volume is a minimum. When the faces have room, the radar shortens the revisits until
the utilisation reaches the mission's "auto_alloc_max" X (0 = off, the default): a face at 20 % with X = 80 % gives
60 % of itself back to the search. Rules (user):
  R1 the plan is kept (waveforms, lattice, pulse scale per bin); only the revisit is faster, RR_real = RR / g, g >= 1;
     the search loads go as g, the false alarms are frozen per second (fa_rate: the Pfa per cell goes as 1 / g), so
     the confirmation, the tracks and the always-on loads do not move;
  R2 utilisation = the dimensioning resource: the ship's energy (need / ship total, shared by the faces), and per face
     its energy (need / feeder or operator cap), S and Ku time, heat, compute, memory, memory bandwidth, beams;
  R3 one factor g for every search volume, the self-protection ones included, raised together (water-filling): the
     volumes of a face that reaches X, or a volume that reaches its rr_min_s, stop; the others go on;
  R4 the budgets are assembled again with the real revisits; a resource that does not go as g (scheduler, heat) is
     corrected by its secant and the filling redone, at most tool.global_opt.reuse_corrections times;
  R5 the verdict stays on the revisit asked; the reuse is the lowest tier of the power allocation (P5), the first
     given back on an overload, before any shedding (also in flight, phase 6);
  (a) the cumulative detection range at the real revisit is computed per bin (optimizer.ranges_at_rr).
The search loads per sector are apportioned from one probe (every search at g = 2): energy-like resources by the
sector's energy on the face, S time by its S time, the others by its pointing rate.
"""
from __future__ import annotations

import copy
import math
from typing import Callable, Dict, List, Optional, Tuple

from . import radar_model as rm
from .params import P

AUTO_MAX_DEFAULT = float(P.get("tool.global_opt.auto_alloc_max"))
CORRECTIONS = int(P.get("tool.global_opt.reuse_corrections"))
TOL = 0.005                      # utilisation tolerance of the check (absolute)
G_MAX = 1e4                      # guard: no resource bounds a volume (should not happen)
SHIP = "ship energy"


def auto_max(mission: dict) -> float:
    return float(mission.get("global", {}).get("auto_alloc_max", AUTO_MAX_DEFAULT) or 0.0)


def _ratio(a, b) -> float:
    a, b = float(a or 0.0), float(b or 0.0)
    return a / b if b > 0 else 0.0


def utilisation(out: dict) -> Tuple[float, Dict[int, Dict[str, float]]]:
    """(ship energy, per face {resource: utilisation}) of an assembled result (R2)."""
    pa = out.get("power_allocation", {})
    ship = _ratio(pa.get("need_MW"), pa.get("ship_MW"))
    faces = {}
    for k in range(4):
        sk = str(k)
        fa = pa.get("faces", {}).get(sk, {})
        cap = min(fa.get("cap_MW") or rm.FACE_FEEDER_MAX_MW, rm.FACE_FEEDER_MAX_MW)
        sch = out.get("scheduler", {}).get("faces", {}).get(sk, {})
        tr = out.get("tracking", {}).get("faces", {}).get(sk, {})
        cp = out.get("compute", {}).get("faces", {}).get(sk, {})
        mem = cp.get("memory", {})
        bands = cp.get("bands", {})
        faces[k] = {"energy": _ratio(fa.get("need_MW"), cap),
                    "S time": _ratio(sch.get("S", {}).get("load"), sch.get("S", {}).get("capacity", 1.0)),
                    "Ku time": _ratio(sch.get("Ku", {}).get("load"), sch.get("Ku", {}).get("capacity", 1.0)),
                    "heat": float(tr.get("heat_load", 0.0) or 0.0),
                    "compute": float(cp.get("load", 0.0) or 0.0),
                    "memory": _ratio(mem.get("buffers_peak_TB"), mem.get("capacity_TB")),
                    "memory bandwidth": _ratio(mem.get("traffic_TB_per_s"), mem.get("bandwidth_TB_per_s")),
                    **{f"{b} beams": _ratio(bands.get(b, {}).get("beams_peak"), bands.get(b, {}).get("beams_max"))
                       for b in ("S", "Ku")}}
    return ship, faces


def _face_max(ship: float, res: Dict[str, float]) -> Tuple[float, str]:
    name, v = max(res.items(), key=lambda x: x[1])
    return (ship, SHIP) if ship > v else (v, name)


def scaled(mission: dict, results: List[dict], g: Dict[str, float]) -> Tuple[dict, List[dict]]:
    """The mission and the search results with the revisits RR / g (R1)."""
    from . import optimizer as mo
    freeze = rm.FA_MODE == "fa_rate"
    secs = []
    for sec in mission["sectors"]:
        f = g.get(sec["id"], 1.0)
        if f == 1.0:
            secs.append(sec)
            continue
        s2 = dict(sec, rr_target_s=float(sec["rr_target_s"]) / f)
        if freeze:                                           # false alarms frozen per second (user, d)
            s2["fa_rate_per_s"] = mo.fa_per_rr(sec) / float(sec["rr_target_s"])
            s2.pop("fa_per_rr", None)
        secs.append(s2)
    res = []
    for r in results:
        f = g.get(r["id"], 1.0)
        if f == 1.0 or r.get("status") != "OK":
            res.append(r)
            continue
        r2 = copy.deepcopy(r)
        r2["rr_target_s"] = float(r["rr_target_s"]) / f
        r2["_reuse_g"] = f                                   # the P5 tier of the power allocation
        for fc in r2.get("faces", {}).values():
            for key in ("energy_fraction", "time_fraction", "avg_power_kW", "dissipation_kW", "dissipation_kW_m2"):
                if key in fc:
                    fc[key] *= f
        if freeze:
            r2["fa_per_rr"] = float(r.get("fa_per_rr", 0.0)) / f
            for d in [r2.get("wf", {}), *r2.get("wf_set", []), *(b.get("wf", {}) for b in r2.get("wf_by_bin", []))]:
                if "pfa" in d:
                    d["pfa"] = float(d["pfa"]) / f
        res.append(r2)
    return dict(mission, sectors=secs), res


def _weights(r: dict, k: int) -> Dict[str, float]:
    fc = r.get("faces", {}).get(str(k), {})
    e, t = float(fc.get("energy_fraction", 0.0)), float(fc.get("time_fraction", 0.0))
    rate = float(fc.get("positions", 0.0)) / float(r["rr_target_s"])
    return {"energy": e, "heat": e, "S time": t, "*": rate}


def _model(results: List[dict], ids: List[str], u0, u1) -> Dict[str, dict]:
    """Linear model of every constraint: {name: {"fixed": u at g = 0, "a": {sector: slope}}} from the nominal (u0)
    and the probe at g = 2 (u1)."""
    rs = {r["id"]: r for r in results}
    cons: Dict[str, dict] = {}
    slope = u1[0] - u0[0]
    e = {s: sum(float(f.get("energy_fraction", 0.0)) for f in rs[s].get("faces", {}).values()) for s in ids}
    tot = sum(e.values())
    cons[SHIP] = {"fixed": u0[0] - slope, "a": {s: slope * e[s] / tot if tot > 0 else 0.0 for s in ids},
                  "face": None}
    for k in range(4):
        w = {s: _weights(rs[s], k) for s in ids}
        for name in u0[1][k]:
            slope = u1[1][k][name] - u0[1][k][name]
            key = name if name in ("energy", "heat", "S time") else "*"
            tot = sum(w[s][key] for s in ids)
            if tot <= 0:
                continue
            a = {s: slope * w[s][key] / tot for s in ids if w[s][key] > 0}
            cons[f"F{k + 1} {name}"] = {"fixed": u0[1][k][name] - slope, "a": a, "face": k}
    return cons


def _value(c: dict, g: Dict[str, float]) -> float:
    return c["fixed"] + sum(a * g[s] for s, a in c["a"].items())


def _fill(cons: Dict[str, dict], ids: List[str], g_max: Dict[str, float], x: float,
          nominal: Dict[str, float]) -> Tuple[Dict[str, float], Dict[str, str]]:
    """Water-filling (R3): one factor raised for every free sector until a constraint reaches x or rr_min_s."""
    g = {s: 1.0 for s in ids}
    why: Dict[str, str] = {}
    for name, c in cons.items():                 # already at or above x with the revisits asked: nothing to give
        if nominal.get(name, _value(c, g)) >= x - 1e-12:
            for s, a in c["a"].items():
                if a > 0 and s not in why:
                    why[s] = f"{name} already at {100 * nominal.get(name, _value(c, g)):.0f} %"
    for s in ids:
        if s not in why and g_max[s] <= 1.0:
            why[s] = "rr_min_s"
    while True:
        free = [s for s in ids if s not in why]
        if not free:
            break
        step, hit = math.inf, None
        for name, c in cons.items():
            rate = sum(c["a"].get(s, 0.0) for s in free)
            if rate <= 1e-15:
                continue
            t = (x - _value(c, g)) / rate
            if t < step:
                step, hit = t, name
        t_min = min(g_max[s] - g[s] for s in free)
        if t_min <= step:
            step, hit = t_min, None
        if not math.isfinite(step):
            step = G_MAX - max(g[s] for s in free)
        step = max(step, 0.0)
        for s in free:
            g[s] += step
        if hit is None:
            done = [s for s in free if g[s] >= g_max[s] - 1e-12]
            for s in done:
                why[s] = "rr_min_s" if math.isfinite(g_max[s]) else "guard"
            if not done:
                for s in free:
                    why[s] = "guard"
        else:
            for s in free:
                if cons[hit]["a"].get(s, 0.0) > 0:
                    why[s] = hit
    return g, why


def reuse(mission: dict, out: dict, assemble: Callable[[dict, List[dict]], dict],
          ranges: Optional[Callable[[dict, dict], Optional[list]]] = None) -> Optional[dict]:
    """The margin reuse of an assembled mission (None when X = 0). assemble(mission, results) -> assembled result;
    ranges(sector, result) -> per wf_by_bin group the range at the sector's revisit (optimizer.ranges_at_rr)."""
    x = auto_max(mission)
    if x <= 0.0:
        return None
    results = out["sectors"]
    secs = {s["id"]: s for s in mission["sectors"]}
    ids = [r["id"] for r in results if r.get("status") == "OK" and any(
        float(f.get("positions", 0.0)) > 0 for f in r.get("faces", {}).values())]
    u0 = utilisation(out)
    ship0, faces0 = u0
    report = {"auto_alloc_max": x, "assemblies": 0, "fa_frozen_per_s": rm.FA_MODE == "fa_rate", "flags": []}
    if not ids:
        report["flags"].append("margin reuse: no search volume")
        return report
    g_max = {s: (float(secs[s]["rr_target_s"]) / float(secs[s]["rr_min_s"]) if secs[s].get("rr_min_s") else math.inf)
             for s in ids}
    g_probe = {s: 2.0 for s in ids}
    u1 = utilisation(assemble(*scaled(mission, results, g_probe)))
    report["assemblies"] += 1
    cons = _model(results, ids, u0, u1)

    def u_of(name: str, c: dict, u) -> float:
        return u[0] if c["face"] is None else u[1][c["face"]][name.split(" ", 1)[1]]
    nominal = {n: u_of(n, c, u0) for n, c in cons.items()}
    last = {n: (g_probe, u_of(n, c, u1)) for n, c in cons.items()}     # the previous point of each constraint
    g, why = _fill(cons, ids, g_max, x, nominal)
    real = out
    for n in range(CORRECTIONS + 1):
        if all(v == 1.0 for v in g.values()):
            real = out
            break
        real = assemble(*scaled(mission, results, g))
        report["assemblies"] += 1
        u = utilisation(real)
        over, off = [], False
        for name, c in cons.items():
            v = u_of(name, c, u)
            moved = any(g[s] > 1.0 for s in c["a"])
            over += [name] if v > x + TOL and moved else []
            off = off or (v > x + TOL and moved) or (v < x - TOL and name in why.values())
            g0, v0 = last[name]
            s0, s1 = sum(a * g0[s] for s, a in c["a"].items()), sum(a * g[s] for s, a in c["a"].items())
            if abs(s1 - s0) > 1e-12 and (v - v0) / (s1 - s0) > 0:     # secant through the last two points reached
                k = (v - v0) / (s1 - s0)
                c["a"] = {s: a * k for s, a in c["a"].items()}
                c["fixed"] = v - k * s1
            last[name] = (dict(g), v)
        if not off:
            break
        if n == CORRECTIONS:
            if over:
                report["flags"].append(f"margin reuse: {', '.join(over)} above {100 * x:.0f} % after "
                                       f"{CORRECTIONS} corrections (resource not linear in the revisit)")
            break
        g, why = _fill(cons, ids, g_max, x, nominal)
    ship_r, faces_r = utilisation(real)
    report["ship_energy"] = {"nominal": ship0, "real": ship_r}
    report["faces"] = {}
    for k in range(4):
        n0, b0 = _face_max(ship0, faces0[k])
        n1, b1 = _face_max(ship_r, faces_r[k])
        report["faces"][str(k)] = {"nominal": n0, "binding_nominal": b0, "real": n1, "binding_real": b1,
                                   "resources_nominal": faces0[k], "resources_real": faces_r[k]}
    rs = {r["id"]: r for r in results}
    real_rs = {r["id"]: r for r in real["sectors"]}
    report["sectors"] = {}
    for s in ids:
        rr = float(secs[s]["rr_target_s"])
        rec = {"g": g[s], "rr_target_s": rr, "rr_real_s": rr / g[s], "rr_min_s": secs[s].get("rr_min_s"),
               "limited_by": why.get(s, "guard"),
               "range_km_nominal": [b.get("range_km_at_bin_max") for b in rs[s].get("wf_by_bin", [])]}
        if g[s] > 1.0 + 1e-9 and ranges is not None:
            sec_r = next(x for x in scaled(mission, results, {s: g[s]})[0]["sectors"] if x["id"] == s)
            rec["range_km_at_rr_real"] = ranges(sec_r, real_rs[s])
        report["sectors"][s] = rec
        if why.get(s) == "guard":
            report["flags"].append(f"margin reuse: {s} revisit bounded by no resource (guard g = {G_MAX:g})")
    report["power_allocation"] = real.get("power_allocation")
    new = set(real.get("warnings", [])) - set(out.get("warnings", []))
    report["flags"].extend(sorted(f"at the real revisits: {w}" for w in new))
    return report
