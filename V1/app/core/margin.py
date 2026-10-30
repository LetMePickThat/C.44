#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MARGIN REUSE -- the resources a mission leaves free are put to use (roadmap step 5.2, pass P4, rule of the user,
26 October 2026).

The revisit asked for each search volume is a minimum. When the faces have room, the radar shortens the revisits until
the utilisation reaches the mission's "load_ceiling" X: a face at 20 % with X = 80 % gives 60 % of itself back to the
search. Step 5.2 P7 (user, 26 October 2026): the mission's "realloc_mode" says who gets the margin, NONE (the default:
the margin stays free), PRIO_SEARCH (the search, rules below) or PRIO_TRACK (the track cadence); the load ceiling is
always on, the part above it is a safety reserve never used (imperfect interleaving, step 5.4). Rules (user):
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

MODES = ("NONE", "PRIO_SEARCH", "PRIO_TRACK")     # step 5.2 P7
MODE_DEFAULT = str(P.get("tool.global_opt.realloc_mode"))
CEILING_DEFAULT = float(P.get("tool.global_opt.load_ceiling"))
if MODE_DEFAULT not in MODES or not 0.0 < CEILING_DEFAULT <= 1.0:
    raise ValueError(f"{P.path}: tool.global_opt.realloc_mode in {MODES} and 0 < tool.global_opt.load_ceiling <= 1")
CORRECTIONS = int(P.get("tool.global_opt.reuse_corrections"))
RR_MIN_SEARCH_S = float(P.get("tool.global_opt.rr_min_search_s"))   # step 5.2 P7: default floors (user)
RR_MIN_TRACK_S = float(P.get("tool.global_opt.rr_min_track_s"))
if RR_MIN_SEARCH_S <= 0 or RR_MIN_TRACK_S <= 0:
    raise ValueError(f"{P.path}: tool.global_opt.rr_min_search_s and rr_min_track_s must be > 0")
BISECT_STEPS = int(P.get("tool.global_opt.reuse_bisections"))
SAT_PROBE_FRAC = float(P.get("tool.global_opt.sat_probe_frac"))        # step 5.2 P7: saturation table
SAT_STRETCH_TRACK = [float(v) for v in P.get("tool.global_opt.sat_stretch_track")]
SAT_STRETCH_SEARCH = [float(v) for v in P.get("tool.global_opt.sat_stretch_search")]
SAT_STRETCH_STAGE = float(P.get("tool.global_opt.sat_stretch_stage"))
P_LOSS_MAX = float(P.get("tracking.p_loss_max"))
if SAT_STRETCH_TRACK[0] != 1.0 or sorted(SAT_STRETCH_TRACK) != SAT_STRETCH_TRACK or \
        sorted(SAT_STRETCH_SEARCH) != SAT_STRETCH_SEARCH or SAT_STRETCH_SEARCH[0] <= 1.0 or not 0.0 < P_LOSS_MAX < 1.0:
    raise ValueError(f"{P.path}: tool.global_opt.sat_stretch_track starts at 1 and rises, sat_stretch_search rises "
                     f"above 1, 0 < tracking.p_loss_max < 1")   # step 5.2 P7: cut-back of a non-linear resource
TOL = 0.005                      # utilisation tolerance of the check (absolute)
G_MAX = 1e4                      # guard: no resource bounds a volume (should not happen)
SHIP = "ship energy"


def realloc_mode(mission: dict) -> str:
    return str(mission.get("global", {}).get("realloc_mode", MODE_DEFAULT))


def load_ceiling(mission: dict) -> float:
    return float(mission.get("global", {}).get("load_ceiling", CEILING_DEFAULT))


def ceiling_flags(mission: dict, out: dict) -> List[str]:
    """Step 5.2 P7 (V4 c): a face whose binding resource is above the load ceiling but within 1 eats its safety
    reserve: alert (the verdict still fails above 1 only)."""
    x = load_ceiling(mission)
    ship, faces = utilisation(out)
    flags = []
    for k in range(4):
        v, name = _face_max(ship, faces[k])
        if x < v <= 1.0 + 1e-12:
            flags.append(f"face F{k + 1}: {name} at {100 * v:.0f} %, above the load ceiling {100 * x:.0f} % "
                         f"(safety reserve in use)")
    return flags


def _ratio(a, b) -> float:
    a, b = float(a or 0.0), float(b or 0.0)
    return a / b if b > 0 else 0.0


def utilisation(out: dict, beams: str = "beams_peak") -> Tuple[float, Dict[int, Dict[str, float]]]:
    """(ship energy, per face {resource: utilisation}) of an assembled result (R2). beams = "continuous": the
    simultaneous beams as open windows (mean, not rounded up) x the beams of the largest look, instead of the integer
    peak (step 5.2 P7: the saturation table, whose one-track probe cannot move an integer)."""
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
                    "memory": _ratio(float(mem.get("buffers_peak_TB") or 0.0) + float(mem.get("flight_tables_TB") or 0.0),
                                     mem.get("capacity_TB")),
                    "memory bandwidth": _ratio(mem.get("traffic_TB_per_s"), mem.get("bandwidth_TB_per_s")),
                    **{f"{b} beams": _ratio(_beams(bands.get(b, {}), beams), bands.get(b, {}).get("beams_max"))
                       for b in ("S", "Ku")}}
    return ship, faces


def _beams(band: dict, how: str) -> float:
    peak = float(band.get("beams_peak") or 0.0)
    if how == "beams_peak" or peak <= 0:
        return peak
    conc = float(band.get("open_windows_mean") or 0.0)
    return conc * peak / max(1, math.ceil(conc - 1e-12))


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


def _model(ids: List[str], weights: Callable[[str, int], Dict[str, float]], u0, u1) -> Dict[str, dict]:
    """Linear model of every constraint: {name: {"fixed": u at g = 0, "a": {item: slope}}} from the nominal (u0)
    and the probe at g = 2 (u1); the slope of a resource is apportioned between the items by weights(item, face):
    their energy for the energy and the heat, their own transmit time for S or Ku time, their pointing rate else."""
    cons: Dict[str, dict] = {}
    slope = u1[0] - u0[0]
    e = {s: sum(weights(s, k)["energy"] for k in range(4)) for s in ids}
    tot = sum(e.values())
    cons[SHIP] = {"fixed": u0[0] - slope, "a": {s: slope * e[s] / tot if tot > 0 else 0.0 for s in ids},
                  "face": None}
    for k in range(4):
        w = {s: weights(s, k) for s in ids}
        for name in u0[1][k]:
            slope = u1[1][k][name] - u0[1][k][name]
            tot = sum(w[s].get(name, w[s]["*"]) for s in ids)
            if tot <= 0:
                continue
            a = {s: slope * w[s].get(name, w[s]["*"]) / tot for s in ids if w[s].get(name, w[s]["*"]) > 0}
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


def _solve(out: dict, ids: List[str], g_max: Dict[str, float], x: float,
           weights: Callable[[str, int], Dict[str, float]], build: Callable[[Dict[str, float]], dict],
           report: dict) -> Tuple[Dict[str, float], Dict[str, str], dict]:
    """Probe, fill, assemble at the real factors and correct the non-linear resources (R3, R4), for search volumes
    (P4) or track brackets (P7) alike; build(g) -> assembled result with the factors g."""
    u0 = utilisation(out)
    g_probe = {s: 2.0 for s in ids}
    u1 = utilisation(build(g_probe))
    report["assemblies"] += 1
    cons = _model(ids, weights, u0, u1)

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
        real = build(g)
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
                g, real = _back_off(g, over, cons, x, build, u_of, report)
                real = out if real is None else real
                report["flags"].append(f"margin reuse: {', '.join(over)} above {100 * x:.0f} % after "
                                       f"{CORRECTIONS} corrections (resource not linear in the revisit): factors "
                                       f"cut back by bisection")
                why = {s: (why.get(s, "guard") if g[s] <= 1.0 else f"{over[0]} (bisection)") for s in ids}
            break
        g, why = _fill(cons, ids, g_max, x, nominal)
    ship0, faces0 = u0
    ship_r, faces_r = utilisation(real)
    report["ship_energy"] = {"nominal": ship0, "real": ship_r}
    report["faces"] = {}
    for k in range(4):
        n0, b0 = _face_max(ship0, faces0[k])
        n1, b1 = _face_max(ship_r, faces_r[k])
        report["faces"][str(k)] = {"nominal": n0, "binding_nominal": b0, "real": n1, "binding_real": b1,
                                   "resources_nominal": faces0[k], "resources_real": faces_r[k]}
    return g, why, real


def _back_off(g: Dict[str, float], over: List[str], cons: Dict[str, dict], x: float,
              build: Callable[[Dict[str, float]], dict], u_of, report: dict) -> Tuple[Dict[str, float], dict]:
    """Step 5.2 P7: a resource far from linear in the factor (a step or a threshold, e.g. the simultaneous Ku beams of
    the track updates) defeats the secant: the factors are cut back together, g' = 1 + t (g - 1), the largest t in
    [0, 1] keeping every constraint within x by bisection (BISECT_STEPS assemblies; t = 0 is the nominal result)."""
    lo, hi, best = 0.0, 1.0, None
    for _ in range(BISECT_STEPS):
        t = 0.5 * (lo + hi)
        gt = {s: 1.0 + t * (v - 1.0) for s, v in g.items()}
        res = build(gt)
        report["assemblies"] += 1
        u = utilisation(res)
        if all(u_of(n, c, u) <= x + TOL for n, c in cons.items() if any(gt[s] > 1.0 for s in c["a"])):
            lo, best = t, (gt, res)
        else:
            hi = t
    return best if best is not None else ({s: 1.0 for s in g}, None)


def _close(report: dict, out: dict, real: dict) -> dict:
    report["power_allocation"] = real.get("power_allocation")
    new = set(real.get("warnings", [])) - set(out.get("warnings", []))
    report["flags"].extend(sorted(f"at the real revisits: {w}" for w in new))
    return report


def reuse(mission: dict, out: dict, assemble: Callable[[dict, List[dict]], dict],
          ranges: Optional[Callable[[dict, dict], Optional[list]]] = None) -> Optional[dict]:
    """The margin reuse of an assembled mission (None unless realloc_mode is PRIO_SEARCH). assemble(mission, results)
    -> assembled result; ranges(sector, result) -> per wf_by_bin group the range at the sector's revisit
    (optimizer.ranges_at_rr)."""
    if realloc_mode(mission) != "PRIO_SEARCH":
        return None
    x = load_ceiling(mission)
    results = out["sectors"]
    secs = {s["id"]: s for s in mission["sectors"]}
    ids = [r["id"] for r in results if r.get("status") == "OK" and any(
        float(f.get("positions", 0.0)) > 0 for f in r.get("faces", {}).values())]
    report = {"realloc_mode": "PRIO_SEARCH", "load_ceiling": x, "assemblies": 0,
              "fa_frozen_per_s": rm.FA_MODE == "fa_rate", "flags": []}
    if not ids:
        report["flags"].append("margin reuse: no search volume")
        return report
    g_max = {s: max(1.0, float(secs[s]["rr_target_s"]) / float(secs[s].get("rr_min_s") or RR_MIN_SEARCH_S))
             for s in ids}
    rs = {r["id"]: r for r in results}
    g, why, real = _solve(out, ids, g_max, x, lambda s, k: _weights(rs[s], k),
                          lambda gg: assemble(*scaled(mission, results, gg)), report)
    real_rs = {r["id"]: r for r in real["sectors"]}
    report["sectors"] = {}
    for s in ids:
        rr = float(secs[s]["rr_target_s"])
        rec = {"g": g[s], "rr_target_s": rr, "rr_real_s": rr / g[s],
               "rr_min_s": secs[s].get("rr_min_s") or RR_MIN_SEARCH_S,
               "limited_by": why.get(s, "guard"),
               "range_km_nominal": [b.get("range_km_at_bin_max") for b in rs[s].get("wf_by_bin", [])]}
        if g[s] > 1.0 + 1e-9 and ranges is not None:
            sec_r = next(x for x in scaled(mission, results, {s: g[s]})[0]["sectors"] if x["id"] == s)
            rec["range_km_at_rr_real"] = ranges(sec_r, real_rs[s])
        report["sectors"][s] = rec
        if why.get(s) == "guard":
            report["flags"].append(f"margin reuse: {s} revisit bounded by no resource (guard g = {G_MAX:g})")
    return _close(report, out, real)


# ----------------------------------------------------------------------------- step 5.2 P7: PRIO_TRACK
def _brackets(mission: dict, out: dict) -> Dict[str, Tuple[dict, dict]]:
    """The track brackets that carry tracks: {"sector/i": (mission bracket, tracking output of the bracket)}."""
    secs = {s["id"]: s for s in mission["sectors"]}
    items = {}
    for t in out.get("tracking", {}).get("sectors", []):
        brs = secs.get(t["id"], {}).get("tracking", [])
        for i, a in enumerate(t.get("at", [])):
            if i < len(brs) and float(brs[i].get("n_targets", 0.0)) > 0 and any(
                    float(f.get("energy_fraction", 0.0)) > 0 or float(f.get("ku_time_fraction", 0.0)) > 0
                    for f in a.get("faces", {}).values()):
                items[f"{t['id']}/{i}"] = (brs[i], a)
    return items


def _track_weights(b: dict, a: dict, k: int) -> Dict[str, float]:
    fc = a.get("faces", {}).get(str(k), {})
    e = float(fc.get("energy_fraction", 0.0))
    tf = a.get("tracks_face") or [0.0] * 4
    return {"energy": e, "heat": e, "Ku time": float(fc.get("ku_time_fraction", 0.0)),
            "*": float(tf[k]) / float(b["rr_s"])}


def scaled_tracks(mission: dict, h: Dict[str, float], nominal: Dict[str, List[float]]) -> dict:
    """The mission with the track update periods rr_s / h of its brackets (keys "sector/i"); each scaled bracket keeps
    its factor and the energy fractions per face of its updates at the period asked (the P3 tier; the rest is P5)."""
    secs = []
    for sec in mission["sectors"]:
        brs = sec.get("tracking", [])
        if not any(h.get(f"{sec['id']}/{i}", 1.0) != 1.0 for i in range(len(brs))):
            secs.append(sec)
            continue
        new = []
        for i, b in enumerate(brs):
            f = h.get(f"{sec['id']}/{i}", 1.0)
            new.append(b if f == 1.0 else dict(b, rr_s=float(b["rr_s"]) / f, _reuse_h=f,
                                               _e_nominal=nominal[f"{sec['id']}/{i}"]))
        secs.append(dict(sec, tracking=new))
    return dict(mission, sectors=secs)


def reuse_tracks(mission: dict, out: dict, assemble: Callable[[dict, List[dict]], dict]) -> Optional[dict]:
    """Step 5.2 P7 (user, 26 October 2026): with realloc_mode PRIO_TRACK the margin speeds up the track cadence only
    (the search volumes stay at their revisit asked): one factor h for every track bracket, raised together up to the
    load ceiling, a bracket stopping at its own rr_min_s (optional, default rr_min_track_s 0.1 s); the plan, the search
    and the confirmations are kept; the track loads come from the tracking budget assembled again at the real periods
    (a shorter period also predicts better: secant corrections as for the search). None in the other modes."""
    if realloc_mode(mission) != "PRIO_TRACK":
        return None
    x = load_ceiling(mission)
    items = _brackets(mission, out)
    report = {"realloc_mode": "PRIO_TRACK", "load_ceiling": x, "assemblies": 0, "flags": []}
    if not items:
        report["flags"].append("margin reuse: no track bracket")
        return report
    ids = list(items)
    g_max = {s: max(1.0, float(b["rr_s"]) / float(b.get("rr_min_s") or RR_MIN_TRACK_S))
             for s, (b, _a) in items.items()}
    nominal = {s: [float(a.get("faces", {}).get(str(k), {}).get("energy_fraction", 0.0)) for k in range(4)]
               for s, (_b, a) in items.items()}
    h, why, real = _solve(out, ids, g_max, x, lambda s, k: _track_weights(*items[s], k),
                          lambda hh: assemble(scaled_tracks(mission, hh, nominal), out["sectors"]), report)
    report["brackets"] = {}
    for s, (b, _a) in items.items():
        rr = float(b["rr_s"])
        report["brackets"][s] = {"h": h[s], "rr_s": rr, "rr_real_s": rr / h[s],
                                 "rr_min_s": b.get("rr_min_s") or RR_MIN_TRACK_S,
                                 "limited_by": why.get(s, "guard")}
        if why.get(s) == "guard":
            report["flags"].append(f"margin reuse: track bracket {s} bounded by no resource (guard h = {G_MAX:g})")
    return _close(report, out, real)


# ----------------------------------------------------------------------------- step 5.2 P7: saturation table
def _all_brackets(mission: dict, out: dict) -> Dict[str, Tuple[dict, dict, dict]]:
    """Every track bracket of the computed sectors: {"sector/i": (mission sector, mission bracket, its output)}."""
    secs = {s["id"]: s for s in mission["sectors"]}
    items = {}
    for t in out.get("tracking", {}).get("sectors", []):
        sec = secs.get(t["id"], {})
        brs = sec.get("tracking", [])
        for i, a in enumerate(t.get("at", [])):
            if i < len(brs):
                items[f"{t['id']}/{i}"] = (sec, brs[i], a)
    return items


def with_targets(mission: dict, extra: Dict[str, float]) -> dict:
    """The mission with extra tracks in its brackets (keys "sector/i"): n_targets + extra."""
    secs = []
    for sec in mission["sectors"]:
        brs = sec.get("tracking", [])
        if not any(extra.get(f"{sec['id']}/{i}") for i in range(len(brs))):
            secs.append(sec)
            continue
        secs.append(dict(sec, tracking=[dict(b, n_targets=float(b["n_targets"]) + extra.get(f"{sec['id']}/{i}", 0.0))
                                        for i, b in enumerate(brs)]))
    return dict(mission, sectors=secs)


def saturation(mission: dict, out: dict, assemble: Callable[[dict, List[dict]], dict],
               ranges: Optional[Callable[[dict, dict], Optional[list]]] = None) -> Optional[dict]:
    """Step 5.2 P7 (user, 26 October 2026): the manageable saturation of every track bracket, from the plan at the
    specification (search at its revisit asked, tracks at their period, no margin reuse): extra tracks a = room left up
    to the load ceiling / cost of one more track, the smallest over every resource (shared ones and the band's own:
    the extra tracks also need Ku time and beams); the extra tracks keep their bracket's settings and spread over the
    faces like it. Cost of one track: one probe with max(1, sat_probe_frac x n) more tracks in every bracket, the rise
    of each resource apportioned between the brackets by their own rise (energy for the energy and the heat, Ku time
    for the Ku time, track rate else), linear in the number of tracks (the values per bracket alone are this linear
    extrapolation, flagged). Cumulative (user, Q18 a): every bracket overflows by the same fraction p of its n, the
    largest p that fits, the brackets served in priority order (class, then nearest) for the rounding; checked by one
    assembly at the cumulative tracks and cut back by bisection if a resource is above the ceiling (Q17 c). None when
    the mission has no track bracket."""
    items = _all_brackets(mission, out)
    if not items:
        return None
    x = load_ceiling(mission)
    delta = {s: float(max(1, round(SAT_PROBE_FRAC * float(b["n_targets"])))) for s, (_sec, b, _a) in items.items()}
    out1 = assemble(with_targets(mission, delta), out["sectors"])
    items1 = _all_brackets(with_targets(mission, delta), out1)
    u0, u1 = utilisation(out, "continuous"), utilisation(out1, "continuous")

    def rise(s: str, k: int) -> Dict[str, float]:
        (_s0, b0, a0), (_s1, b1, a1) = items[s], items1.get(s, items[s])
        w0, w1 = _track_weights(b0, a0, k), _track_weights(b1, a1, k)
        return {n: max(0.0, w1[n] - w0[n]) for n in w0}
    cost: Dict[str, Dict[str, float]] = {}
    room: Dict[str, float] = {}

    def apportion(name: str, du: float, w: Dict[str, float], nominal: float) -> None:
        tot = sum(w.values())
        if du <= 1e-12 or tot <= 0:
            return
        room[name] = x - nominal
        cost[name] = {s: du * w[s] / tot / delta[s] for s in w if w[s] > 0}
    apportion(SHIP, u1[0] - u0[0], {s: sum(rise(s, k)["energy"] for k in range(4)) for s in items}, u0[0])
    for k in range(4):
        r = {s: rise(s, k) for s in items}
        up = utilisation(out)[1][k]                     # the beams' room is taken from their integer peak
        for name in u0[1][k]:
            apportion(f"F{k + 1} {name}", u1[1][k][name] - u0[1][k][name],
                      {s: r[s].get(name, r[s]["*"]) for s in items}, up[name])
    report = {"load_ceiling": x, "probe": delta, "room": room, "brackets": {}, "extrapolated": True, "flags": []}
    for s, (sec, b, _a) in items.items():
        lim = {c: room[c] / v[s] for c, v in cost.items() if v.get(s, 0.0) > 1e-15}
        if not lim:
            report["flags"].append(f"saturation: bracket {s} costs no resource in the probe (out of Ku reach?)")
            continue
        c_bind = min(lim, key=lim.get)
        a = max(0, math.floor(lim[c_bind] + 1e-9))
        report["brackets"][s] = {"target_type": sec.get("target_type"), "r_min_km": float(b["r_min_km"]),
                                 "n_targets": float(b["n_targets"]), "extra_alone": a, "binding_alone": c_bind,
                                 "cost_per_track": {c: v[s] for c, v in cost.items() if v.get(s, 0.0) > 1e-15}}
        if a == 0:
            report["flags"].append(f"saturation: bracket {s} has no manageable saturation ({c_bind} at "
                                   f"{100 * (x - room[c_bind]):.0f} %, load ceiling {100 * x:.0f} %)")
    report["cumulative"] = _cumulative(mission, out, items, cost, room, x, assemble, report)
    report["critique"] = critique(mission, out, items, cost, room, x, assemble, ranges, report)
    return report


def _cumulative(mission: dict, out: dict, items: Dict[str, tuple], cost: Dict[str, Dict[str, float]],
                room: Dict[str, float], x: float, assemble: Callable[[dict, List[dict]], dict], report: dict) -> dict:
    """Step 5.2 P7 (Q18 a, Q17 c): the same overflow fraction p of n for every bracket, then the check."""
    from .allocation import CLASS_RANK
    n = {s: float(b["n_targets"]) for s, (_sec, b, _a) in items.items()}
    use = {c: sum(v.get(s, 0.0) * n[s] for s in n) for c, v in cost.items()}
    lim = {c: room[c] / u for c, u in use.items() if u > 1e-15}
    if not lim:
        return {"p": None, "extra": {}, "binding": None, "verified": False, "assemblies": 0}
    c_bind = min(lim, key=lim.get)
    p = max(0.0, lim[c_bind])
    order = sorted(n, key=lambda s: (CLASS_RANK.get(items[s][0].get("target_type") or "", len(CLASS_RANK)),
                                     float(items[s][1]["r_min_km"]), s))

    def extra_at(t: float) -> Dict[str, int]:
        a = {s: math.floor(t * n[s] + 1e-9) for s in n}
        left = {c: room[c] - sum(cost[c].get(s, 0.0) * a[s] for s in a) for c in cost}
        for s in order:                                  # the rounding, in priority order
            if n[s] > 0 and all(left[c] - cost[c].get(s, 0.0) >= -1e-12 for c in cost):
                a[s] += 1
                left = {c: left[c] - cost[c].get(s, 0.0) for c in cost}
        return a
    ship0, faces0 = utilisation(out)

    def over(res: dict) -> List[str]:
        ship, faces = utilisation(res)
        bad = [SHIP] if ship > x + TOL and ship > ship0 + 1e-12 else []
        return bad + [f"F{k + 1} {r}" for k in range(4) for r, v in faces[k].items()
                      if v > x + TOL and v > faces0[k][r] + 1e-12]
    rec = {"p": p, "extra": extra_at(p), "binding": c_bind, "verified": True, "bisected": False, "assemblies": 1}
    bad = over(assemble(with_targets(mission, rec["extra"]), out["sectors"]))
    if bad:
        lo, hi = 0.0, p
        for _ in range(BISECT_STEPS):
            t = 0.5 * (lo + hi)
            rec["assemblies"] += 1
            if over(assemble(with_targets(mission, extra_at(t)), out["sectors"])):
                hi = t
            else:
                lo = t
        rec.update({"p": lo, "extra": extra_at(lo) if lo > 0 else {s: 0 for s in n}, "binding": f"{bad[0]} (bisection)",
                    "bisected": True})
        report["flags"].append(f"saturation: cumulative overflow cut back by bisection ({', '.join(bad)} above "
                               f"{100 * x:.0f} % at the linear estimate)")
    return rec


# ----------------------------------------------------------------------------- step 5.2 P7: saturation critique
def _interp_cross(xs: List[float], ys: List[float], level: float) -> Optional[float]:
    """First x where y crosses level going up (log-log between the two points around it, linear in x when a y is 0);
    None when it never does."""
    for j in range(1, len(xs)):
        if ys[j] > level:
            x0, x1, y0, y1 = xs[j - 1], xs[j], ys[j - 1], ys[j]
            if y0 <= 0:
                return x0 + (x1 - x0) * (level - y0) / (y1 - y0)
            return math.exp(math.log(x0) + (math.log(x1) - math.log(x0)) * (math.log(level) - math.log(y0))
                            / (math.log(y1) - math.log(y0)))
    return None


def track_floors(mission: dict, out: dict, items: Dict[str, tuple], report: dict) -> Dict[str, dict]:
    """D1: t_max of every bracket, the longest update period whose loss probability per update stays within
    p_loss_max (each bracket at rr_s x sat_stretch_track in the tracking budget, log-log interpolation), capped by the
    sector's latency_max_s. Two budgets: the factors up to sat_stretch_stage, then the others only for the brackets
    still within p_loss_max (an untenable period is slow to cost)."""
    from . import tracking
    pl_of: Dict[str, Dict[float, float]] = {s: {} for s in items}
    stages = [[f for f in SAT_STRETCH_TRACK if f <= SAT_STRETCH_STAGE],
              [f for f in SAT_STRETCH_TRACK if f > SAT_STRETCH_STAGE]]
    for fs in stages:
        todo = [s for s in items if fs and all(v <= P_LOSS_MAX for v in pl_of[s].values())]
        if not todo:
            continue
        where: Dict[str, List[Tuple[str, float]]] = {}
        secs = []
        for sec in mission["sectors"]:
            keys = [(s, f) for s in todo if items[s][0]["id"] == sec["id"] for f in fs]
            if not keys:
                secs.append(sec if not sec.get("tracking") else dict(sec, tracking=[]))
                continue
            where[sec["id"]] = keys
            secs.append(dict(sec, tracking=[dict(items[s][1], rr_s=float(items[s][1]["rr_s"]) * f) for s, f in keys]))
        tb = tracking.budget(dict(mission, sectors=secs), out)
        report["tracking_budgets"] = report.get("tracking_budgets", 0) + 1
        for t in tb.get("sectors", []):
            for (s, f), a in zip(where.get(t["id"], []), t.get("at", [])):
                pl_of[s][f] = float(a.get("p_loss_per_update", 0.0)) if a.get("rr_tenable", True) else 1.0
    floors = {}
    for s, (sec, b, _a) in items.items():
        rr = float(b["rr_s"])
        if not pl_of[s]:
            continue
        f_ = sorted(pl_of[s])
        ts = [rr * f for f in f_]
        pl = [pl_of[s][f] for f in f_]
        if pl[0] > P_LOSS_MAX:
            t_max, by = rr, "loss already above p_loss_max at the period asked"
            report["flags"].append(f"saturation: bracket {s} loses {100 * pl[0]:.2g} % of its tracks per update at its "
                                   f"period asked (above p_loss_max {100 * P_LOSS_MAX:g} %): no stretching")
        else:
            t_max = _interp_cross(ts, pl, P_LOSS_MAX)
            by = "p_loss_max"
            if t_max is None:
                t_max, by = ts[-1], f"beyond the probe (x{f_[-1]:g})"
        lat = sec.get("latency_max_s")
        if lat and float(lat) < t_max:
            t_max, by = max(rr, float(lat)), "latency_max_s"
        floors[s] = {"t_max_s": t_max, "limited_by": by, "p_loss_per_update": dict(zip([round(t, 6) for t in ts], pl))}
    return floors


def search_floors(mission: dict, out: dict, ranges: Optional[Callable], report: dict) -> Dict[str, dict]:
    """D2: rr_max of every P4_n search volume (P1 untouchable): its rr_max_s, else the revisit where its worst
    cumulative range falls to its range_min_km (probe at y x sat_stretch_search, log interpolation of 1/range)."""
    from . import optimizer as mo
    floors = {}
    secs = {s["id"]: s for s in mission["sectors"]}
    for r in out["sectors"]:
        sec = secs.get(r["id"])
        if r.get("status") != "OK" or sec is None or sec.get("self_protection"):
            continue
        y = float(sec["rr_target_s"])
        if sec.get("rr_max_s"):
            floors[r["id"]] = {"rr_max_s": max(y, float(sec["rr_max_s"])), "limited_by": "rr_max_s"}
            continue
        r_min = float(sec.get("range_min_km", mo.RANGE_MIN_FRAC * float(sec["range_km"])))
        if ranges is None or r_min <= 0:
            floors[r["id"]] = {"rr_max_s": y * SAT_STRETCH_SEARCH[-1], "limited_by": "no range model"}
            continue
        xs, inv = [y], [1.0 / max(1e-9, min((v for v in (ranges(sec, r) or []) if v), default=math.inf))]
        for f in SAT_STRETCH_SEARCH:
            sec_f, res_f = scaled(mission, out["sectors"], {r["id"]: 1.0 / f})
            sec_f = next(x for x in sec_f["sectors"] if x["id"] == r["id"])
            res_f = next(x for x in res_f if x["id"] == r["id"])
            worst = min((v for v in (ranges(sec_f, res_f) or []) if v), default=math.inf)
            xs.append(y * f)
            inv.append(1.0 / max(1e-9, worst))
            if worst <= r_min:
                break
        rr_max = _interp_cross(xs, inv, 1.0 / r_min)
        if rr_max is None:
            floors[r["id"]] = {"rr_max_s": xs[-1], "limited_by": f"beyond the probe (x{SAT_STRETCH_SEARCH[-1]:g})"}
        else:
            floors[r["id"]] = {"rr_max_s": max(y, rr_max), "limited_by": "range_min_km"}
    return floors


def _ku_levers(out: dict) -> Dict[str, float]:
    """D3: room given back on each face by the Ku levers (user, Q16): the burn-through Ku time dropped (P3'), the Ku
    confirmations moved to S2 (their Ku time freed, the S2 S time taken), where S2 is admissible."""
    give: Dict[str, float] = {}
    for t in out.get("tracking", {}).get("sectors", []):
        for k, f in ((t.get("ku_burn_through") or {}).get("faces") or {}).items():
            name = f"F{int(k) + 1} Ku time"
            give[name] = give.get(name, 0.0) + float(f.get("ku_time_fraction", 0.0))
        s2 = (t.get("alternatives") or {}).get("S2") or {}
        if t.get("confirm_mode") not in (None, "S2") and s2.get("admissible") and s2.get("faces"):
            for k, f in ((t.get("confirmation") or {}).get("faces") or {}).items():
                name = f"F{int(k) + 1} Ku time"
                give[name] = give.get(name, 0.0) + float(f.get("ku_time_fraction", 0.0)) \
                    - float(s2["faces"].get(k, {}).get("ku_time_fraction", 0.0))
            for k, f in s2["faces"].items():
                name = f"F{int(k) + 1} S time"
                give[name] = give.get(name, 0.0) - float(f.get("s_time_fraction", 0.0)) + float(
                    ((t.get("confirmation") or {}).get("faces") or {}).get(k, {}).get("s_time_fraction", 0.0))
    return give


def critique(mission: dict, out: dict, items: Dict[str, tuple], cost: Dict[str, Dict[str, float]],
             room: Dict[str, float], x: float, assemble: Callable[[dict, List[dict]], dict],
             ranges: Optional[Callable], report: dict) -> dict:
    """Step 5.2 P7 (D1-D5, user 26 October 2026): where the saturation becomes critical, per bracket alone and
    cumulative (same overflow fraction p of n), in the mission's realloc_mode: PRIO_SEARCH and NONE keep the search at
    its revisit and stretch every track (the n asked too) to its floor t_max; PRIO_TRACK first gives back the search of
    the P4_n volumes down to their rr_max, then stretches the tracks. A stretched track costs its cost at the period
    asked x (rr_s / t_max) (D3: 1/T, optimistic: the wider prediction of a slower track is not recomputed). The Ku
    levers (P3' dropped, confirmations to S2) count when a Ku time binds. Cumulative checked by one assembly, cut back
    by bisection if a resource is above the ceiling."""
    from .allocation import CLASS_RANK
    mode = realloc_mode(mission)
    t_fl = track_floors(mission, out, items, report)
    s_fl = search_floors(mission, out, ranges, report) if mode == "PRIO_TRACK" else {}
    secs = {s["id"]: s for s in mission["sectors"]}
    freed: Dict[str, float] = {}
    g_deg: Dict[str, float] = {}
    if s_fl:                                             # D3: the search of the P4_n volumes at its rr_max
        g_deg = {sid: float(secs[sid]["rr_target_s"]) / f["rr_max_s"] for sid, f in s_fl.items()}
        u0, ud = utilisation(out), utilisation(assemble(*scaled(mission, out["sectors"], g_deg)))
        report["assemblies_critique"] = 1
        freed[SHIP] = max(0.0, u0[0] - ud[0])
        for k in range(4):
            for name, v in u0[1][k].items():
                freed[f"F{k + 1} {name}"] = max(0.0, v - ud[1][k][name])
    n = {s: float(b["n_targets"]) for s, (_sec, b, _a) in items.items()}
    rr = {s: float(b["rr_s"]) for s, (_sec, b, _a) in items.items()}
    t_max = {s: t_fl.get(s, {}).get("t_max_s", rr[s]) for s in items}
    stretched = {c: {s: v * rr[s] / t_max[s] for s, v in cv.items()} for c, cv in cost.items()}

    def solve(levers: Dict[str, float]):
        avail = {c: room[c] + freed.get(c, 0.0) + levers.get(c, 0.0) for c in cost}
        alone = {}
        for s in items:
            lim = {c: (avail[c] + cost[c].get(s, 0.0) * n[s]) / v[s] for c, v in stretched.items()
                   if v.get(s, 0.0) > 1e-15}
            if lim:
                c_b = min(lim, key=lim.get)
                alone[s] = {"extra_critique": max(0, math.floor(lim[c_b] + 1e-9 - n[s])), "binding_critique": c_b}
        use = {c: sum(v.get(s, 0.0) * n[s] for s in n) for c, v in stretched.items()}
        lim = {c: (avail[c] + sum(cost[c].get(s, 0.0) * n[s] for s in n)) / u - 1.0
               for c, u in use.items() if u > 1e-15}
        c_b = min(lim, key=lim.get) if lim else None
        return alone, (max(0.0, lim[c_b]) if c_b else None), c_b
    alone, p, c_b = solve({})
    levers_used = False
    if c_b and c_b.endswith("Ku time"):                  # D3: Ku levers when a Ku time binds
        lev = _ku_levers(out)
        if any(v > 0 for k, v in lev.items() if k.endswith("Ku time")):
            alone, p, c_b = solve(lev)
            levers_used = True
    rec = {"mode": mode, "track_floors": t_fl, "search_floors": s_fl, "brackets": alone, "ku_levers": levers_used,
           "cumulative": {"p": p, "binding": c_b, "verified": False, "bisected": False}}
    order = sorted(items, key=lambda s: (CLASS_RANK.get(items[s][0].get("target_type") or "", len(CLASS_RANK)),
                                         float(items[s][1]["r_min_km"])))
    if p is not None:
        ship0, faces0 = utilisation(out)

        def build(t: float) -> dict:
            mis = with_targets(mission, {s: math.floor(t * n[s] + 1e-9) for s in n})
            mis = dict(mis, sectors=[dict(sec, tracking=[dict(b, rr_s=t_max[f"{sec['id']}/{i}"])
                                                         if f"{sec['id']}/{i}" in t_max else b
                                                         for i, b in enumerate(sec["tracking"])])
                                     if sec.get("tracking") else sec for sec in mis["sectors"]])
            return assemble(*scaled(mis, out["sectors"], g_deg)) if g_deg else assemble(mis, out["sectors"])

        def over(res: dict) -> List[str]:
            ship, faces = utilisation(res)
            bad = [SHIP] if ship > x + TOL and ship > ship0 + 1e-12 else []
            return bad + [f"F{k + 1} {r_}" for k in range(4) for r_, v in faces[k].items()
                          if v > x + TOL and v > faces0[k][r_] + 1e-12 and not (levers_used and r_.endswith("Ku time"))]
        bad = over(build(p))
        rec["cumulative"].update({"verified": True, "assemblies": 1})
        if bad:
            lo, hi = 0.0, p
            for _ in range(BISECT_STEPS):
                t = 0.5 * (lo + hi)
                rec["cumulative"]["assemblies"] += 1
                if over(build(t)):
                    hi = t
                else:
                    lo = t
            rec["cumulative"].update({"p": lo, "binding": f"{bad[0]} (bisection)", "bisected": True})
            report["flags"].append(f"saturation critique: cumulative cut back by bisection ({', '.join(bad)} above "
                                   f"{100 * x:.0f} % at the linear estimate)")
        rec["cumulative"]["extra"] = {s: math.floor(rec["cumulative"]["p"] * n[s] + 1e-9) for s in order}
    # D4: what goes first once critical
    if mode == "PRIO_TRACK":
        p4 = [s for s in mission["sectors"] if not s.get("self_protection") and s["id"] in s_fl]
        if p4:
            last = max(p4, key=lambda s: (int(s.get("search_priority", 1)), s["id"]))
            rec["first_shed"] = f"search volume {last['id']} (P4_{int(last.get('search_priority', 1))}), whole volume"
    if "first_shed" not in rec and order:
        low = max(order, key=lambda s: (CLASS_RANK.get(items[s][0].get("target_type") or "", len(CLASS_RANK)),
                                        float(items[s][1]["r_max_km"])))
        rec["first_shed"] = (f"tracks of bracket {low} ({items[low][0].get('target_type') or 'other'}), the farthest "
                             f"first")
    return rec
