#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VALIDATION -- the verdict of a mission plan (roadmap step 5.3, pass P1, framed with the user on 27 October 2026).

Mission preparation only (Pmiss): the plan of optimizer.run(), at the navigation snapshot of the mission, at the
specification asked (the margin reuse of step 5.2 is reported apart, the verdict stays on the revisits asked). No
physics here: every figure is read from the outputs of the other modules and put in one table of constraints.

Every constraint is a load = need / limit and one of three states:
  resource   (a capacity: energy, transmit time, heat, compute, beams, memory, links, storage)
             OK up to the mission's load_ceiling (default 0.80), ALERT between the ceiling and 1 (the safety reserve is
             in use), KO above 1;
  spec       (a requirement of the operator: range, revisit, confirmation, latency, track period and reach,
             discrimination) OK up to 1, KO above; ALERT where the model allows a tolerance (positions short of range
             within the sector's short_tol, discrimination reached below the shortest range of interest);
  transient  (a passing overload: migration between faces while the ship turns, navigation pointing) OK up to 1,
             ALERT above.
The mission verdict is KO as soon as one constraint is KO (the operator's specification is not held), ALERT when one
is in alert, OK otherwise; every constraint carries its priority tier (P0 to P5, step 5.2 P1) so that a failure of
P0 to P2 shows at once. The limiting resource is the resource constraint with the highest load, per face and for the
mission. The saturation summary (step 5.2 P7) gives the state at the specification and the room beyond it.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from . import allocation
from . import compute
from . import margin
from . import optimizer as mo
from . import repoint

TOL = 1e-9
STATES = ("OK", "ALERT", "KO")
_FACE_RESOURCES = ("energy", "S time", "Ku time", "heat", "compute", "memory", "memory bandwidth", "S beams",
                   "Ku beams")


def _ratio(a, b) -> Optional[float]:
    """a / b; None when b is zero and a is not (an infinite load: no JSON infinity in the result)."""
    a, b = float(a or 0.0), float(b or 0.0)
    if b > 0:
        return a / b
    return 0.0 if a == 0 else None


def _state(load: Optional[float], kind: str, ceiling: float) -> str:
    if load is None or (isinstance(load, float) and math.isnan(load)):
        return "KO" if kind != "transient" else "ALERT"
    if kind == "resource":
        return "OK" if load <= ceiling + TOL else "ALERT" if load <= 1.0 + TOL else "KO"
    if kind == "transient":
        return "OK" if load <= 1.0 + TOL else "ALERT"
    return "OK" if load <= 1.0 + TOL else "KO"


class _Table:
    def __init__(self, ceiling: float):
        self.ceiling = ceiling
        self.rows: List[dict] = []

    def add(self, family: str, obj: str, quantity: str, kind: str, value, limit, load: Optional[float],
            priority: Optional[str], step: str, note: str = "", state: Optional[str] = None, unit: str = "") -> None:
        self.rows.append({"id": f"{obj}.{quantity}", "family": family, "object": obj, "quantity": quantity,
                          "kind": kind, "value": value, "limit": limit, "unit": unit,
                          "load": None if load is None else float(load),
                          "state": state or _state(load, kind, self.ceiling), "priority": priority, "step": step,
                          "note": note})


def _search_priority(sec: dict) -> str:
    """Tier of a search volume (step 5.2 P1): P1 for a self-protection volume, else P4_n."""
    return "P1" if sec.get("self_protection") else f"P4_{int(sec.get('search_priority', 1))}"


def _track_priority(sec: dict) -> str:
    """Tier of the tracks of a volume (step 5.2 P1): P3 by class, KKV > SHIP > DECOY > DEBRIS > other."""
    cls = sec.get("target_type")
    return f"P3 {cls if cls in allocation.CLASS_RANK else 'other'}"


def _sectors(t: _Table, out: dict, secs: Dict[str, dict]) -> None:
    for r in out.get("sectors", []):
        sid = r["id"]
        sec = secs.get(sid, {})
        pr = _search_priority(sec)
        if r.get("status") != "OK":
            t.add("sector", sid, "plan", "spec", r.get("status"), "OK", None, pr, "3.1", r.get("note", ""))
            continue
        t.add("sector", sid, "plan", "spec", "OK", "OK", 0.0, pr, "3.1")
        rg = r.get("range_km", {})
        short = float(rg.get("positions_short_of_range_pct", 0.0)) / 100.0
        tol = float(sec.get("short_tol", mo.SHORT_TOL))
        worst, want = rg.get("at_worst_scan"), float(rg.get("requested", sec.get("range_km", 0.0)))
        state = "OK" if short <= 0.0 else "ALERT" if short <= tol + TOL else "KO"
        t.add("sector", sid, "range", "spec", worst, want, _ratio(short, tol), pr, "3.1",
              (f"{100 * short:.2g} % of the positions short of range (tolerance {100 * tol:g} %; load = short "
               f"fraction / tolerance)") if short > 0 else "", state=state, unit="km")
        rr, rr_m = float(r["rr_target_s"]), float(r.get("rr_est_mission_s", r["rr_target_s"]))
        t.add("sector", sid, "revisit", "spec", rr_m, rr, _ratio(rr_m, rr), pr, "2.3",
              "revisit stretched in the mission (face over-subscribed)" if rr_m > rr * (1 + TOL) else "", unit="s")


def _tracking(t: _Table, out: dict, secs: Dict[str, dict]) -> None:
    trk = out.get("tracking") or {}
    lat = {s["id"]: s for s in (out.get("scheduler") or {}).get("sectors", [])}
    for s in trk.get("sectors", []):
        sid = s["id"]
        sec = secs.get(sid, {})
        conf = s.get("confirmation") or {}
        mode = s.get("confirm_mode")
        if mode is None:
            t.add("confirmation", sid, "confirmation mode", "spec", None, "admissible", None, "P2", "3.2",
                  s.get("reason", ""))
        else:
            ok = bool(conf.get("admissible", True))
            t.add("confirmation", sid, "confirmation mode", "spec", mode, "admissible", 0.0 if ok else None, "P2",
                  "3.2", "" if ok else f"forced mode breaks a constraint ({conf.get('why_not', '')})")
            p_min = float(s.get("confirm_p_min", trk.get("confirm_p_min", 0.98)))
            p = float(conf.get("success", 0.0))
            t.add("confirmation", sid, "P confirm", "spec", p, p_min, _ratio(1.0 - p, 1.0 - p_min), "P2", "3.2")
        if sid in lat:
            q, req = float(lat[sid]["latency_q95_s"]), float(lat[sid]["latency_required_s"])
            t.add("confirmation", sid, "confirmation latency", "spec", q, req, _ratio(q, req), "P2", "4.3",
                  "95 % quantile", unit="s")
        pr = _track_priority(sec)
        for i, b in enumerate(s.get("at", [])):
            obj = f"{sid}/{i} {b['r_min_km']:g}-{b['r_max_km']:g} km"
            if "rr_tenable" in b:
                ok = bool(b["rr_tenable"])
                note = "" if ok else (f"longest tenable period {b['rr_alert_s']:.3g} s" if b.get("rr_alert_s")
                                      else "no tenable period")
                t.add("track", obj, "track period", "spec", float(b["rr_s"]), float(b["rr_s"]), 0.0 if ok else None,
                      pr, "3.3", note, unit="s")
            n, lost = float(b.get("n_targets", 0.0)), float(b.get("unreachable_tracks", 0.0) or 0.0)
            t.add("track", obj, "track reach", "spec", n - lost, n, _ratio(n, n - lost) if lost > 0 else 0.0, pr,
                  "3.3", f"{lost:.3g} tracks beyond the Ku reach of the bracket" if lost > 0 else "")
        d = s.get("discrimination")
        if d:
            if d.get("looks") is None:
                t.add("track", sid, "discrimination", "spec", d.get("p_at_max_looks"), None, None, pr, "3.3",
                      f"against {d.get('against')}: not reached within the track looks allowed")
            else:
                r_min = float(mo.range_min_km(sec)) if sec else 0.0
                late = d.get("range_km") is not None and float(d["range_km"]) < r_min
                t.add("track", sid, "discrimination", "spec", d.get("range_km"), r_min, 0.0, pr, "3.3",
                      f"against {d.get('against')}: reached below the shortest range of interest" if late else
                      f"against {d.get('against')}: {d['looks']} looks", state="ALERT" if late else None, unit="km")


def _faces(t: _Table, out: dict) -> Dict[str, dict]:
    """Face resources; returns per face its binding resource and load."""
    _ship, util = margin.utilisation(out)
    trk = (out.get("tracking") or {}).get("faces", {})
    cmp_ = (out.get("compute") or {}).get("faces", {})
    sch = (out.get("scheduler") or {}).get("faces", {})
    binding = {}
    for k in range(4):
        sk, name = str(k), f"F{k + 1}"
        tf, cf, sf = trk.get(sk, {}), cmp_.get(sk, {}), sch.get(sk, {})
        loads = dict(util.get(k, {}))
        if "total_energy" in tf:                       # step 5.2 P6: need / allocation
            loads["energy"] = float(tf["total_energy"])
        for res in _FACE_RESOURCES:
            if res in loads:
                t.add("face", name, res, "resource", loads[res], 1.0, loads[res], None, _RES_STEP[res])
        for b in ("S", "Ku"):
            bd = cf.get("bands", {}).get(b, {})
            if bd:
                link = _ratio(float(bd.get("link_trsa_peak_tbps", 0.0)) * 1e12, compute.LINK_TRSA_BPS)
                inp = _ratio(float(bd.get("face_input_peak_tbps", 0.0)) * 1e12, compute.FACE_INPUT_BPS)
                t.add("face", name, f"{b} TRSA link", "resource", bd.get("link_trsa_peak_tbps"),
                      compute.LINK_TRSA_BPS / 1e12, link, None, "4.4", unit="Tbit/s")
                t.add("face", name, f"{b} processor input", "resource", bd.get("face_input_peak_tbps"),
                      compute.FACE_INPUT_BPS / 1e12, inp, None, "4.4", unit="Tbit/s")
                loads[f"{b} TRSA link"], loads[f"{b} processor input"] = link, inp
        pw = cf.get("power_kW", {})
        if pw:
            v = _ratio(pw.get("total"), pw.get("ceiling"))
            t.add("face", name, "processing power", "resource", pw.get("total"), pw.get("ceiling"), v, None, "4.4",
                  unit="kW")
            loads["processing power"] = v
        if "store_obs_deficit_MJ" in tf:
            v = _ratio(tf["store_obs_deficit_MJ"], tf.get("store_usable_MJ"))
            t.add("face", name, "storage (observation)", "resource", tf["store_obs_deficit_MJ"],
                  tf.get("store_usable_MJ"), v, None, "4.1", unit="MJ")
            loads["storage (observation)"] = v
        # a revisit packed together beyond the storage or the cooling is a rule of the scheduler (steps 4.1, 4.2), which
        # spreads the transmissions; its synthetic timeline (step 4.3) shows any pause or late update left
        if "store_burst_ok" in tf:
            ok = bool(tf["store_burst_ok"])
            t.add("face", name, "storage (burst)", "spec", tf.get("store_burst_deficit_MJ"), tf.get("store_usable_MJ"),
                  None, None, "4.1", "" if ok else "scheduler rule: a revisit packed together would exhaust the "
                  f"storage, spread (at most {tf.get('max_contiguous_tx_s', 0.0):.2g} s contiguous at peak power)",
                  state="OK", unit="MJ")
        if "heat_burst_ok" in tf:
            ok = bool(tf["heat_burst_ok"])
            t.add("face", name, "heat (burst)", "spec", tf.get("heat_burst_deficit_MJ"), 0.0, None, None, "4.2",
                  "" if ok else "scheduler rule: a burst beyond the cooling, spread", state="OK", unit="MJ")
        for b in ("S", "Ku"):
            tl = (sf.get(b) or {}).get("timeline") or {}
            late = float(tl.get("late_update_frac", 0.0) or 0.0)
            if tl.get("recharge_pauses") or late > 0:
                t.add("face", name, f"{b} timeline", "spec", late, 0.0, None, None, "4.3",
                      f"{tl.get('recharge_pauses', 0)} storage recharge pauses ({tl.get('pause_time_s', 0.0):.3g} s), "
                      f"{100 * late:.1f} % of the track updates late in the synthetic timeline", state="ALERT")
        if "peak_ok" in tf:
            ok = bool(tf["peak_ok"])
            t.add("face", name, "peak power", "spec", tf.get("peak_MW"), None, 0.0 if ok else None, None, "4.1",
                  "" if ok else "peak power of the face above its supply")
        for b in ("S", "Ku"):
            bs = sf.get(b)
            if not bs:
                continue
            if not bs.get("feasible", True):
                t.add("face", name, f"{b} scheduling", "spec", bs.get("load"), bs.get("capacity"), None, "P0-P2", "4.3",
                      "calibration, self-protection, confirmation and initiation alone exceed the transmit time")
            for lab, v in (bs.get("stretch") or {}).items():
                v = float(v) if v is not None and math.isfinite(float(v)) else None
                t.add("face", name, f"{b} shedding {lab}", "spec", v, 1.0, v, None, "4.3",
                      f"{lab} stretched x{v:.2f} by priority shedding" if v is not None else f"{lab} shed")
        res = {r_: v for r_, v in loads.items() if v is not None}
        worst = max(res, key=res.get) if res else None
        binding[sk] = {"resource": worst, "load": res.get(worst, 0.0) if worst else 0.0,
                       "state": _state(res.get(worst, 0.0) if worst else 0.0, "resource", t.ceiling)}
    return binding


_RES_STEP = {"energy": "5.2", "S time": "4.3", "Ku time": "4.3", "heat": "4.2", "compute": "4.4", "memory": "4.4",
             "memory bandwidth": "4.4", "S beams": "4.4", "Ku beams": "4.4"}


def _ship(t: _Table, out: dict) -> Optional[dict]:
    pa = out.get("power_allocation") or {}
    best = None
    if pa:
        v = _ratio(pa.get("need_MW"), pa.get("ship_MW"))
        t.add("ship", "ship", "energy", "resource", pa.get("need_MW"), pa.get("ship_MW"), v, None, "5.2", unit="MW")
        best = {"object": "ship", "resource": "energy", "load": v}
        for p_ in pa.get("partial", []):
            tier = p_["tier"]
            pr = tier.split(" ", 1)[0] if tier != "always-on" else "P0"
            note = (f"revisit tenable {p_['rr_tenable_s']:.3g} s" if "rr_tenable_s" in p_ else
                    f"{p_['tracks_tenable']:.3g} tracks tenable" if "tracks_tenable" in p_ else "")
            fr = float(p_["served_fraction"])
            t.add("ship", p_["label"], "ship power served", "spec", fr, 1.0, _ratio(1.0, fr) if fr > 0 else None,
                  pr, "5.2", (note + "; " if note else "") + ("never shed: mission not tenable"
                                                               if pr in ("P0", "P1", "P2") else ""))
    sp = (out.get("tracking") or {}).get("ship_power") or {}
    if "heat_load" in sp:
        t.add("ship", "ship", "heat rejection", "resource", sp.get("heat_MW"), sp.get("heat_rejection_MW"),
              float(sp["heat_load"]), None, "4.2", unit="MW")
    return best


def _flight(t: _Table, out: dict) -> None:
    rp = out.get("repointing") or {}
    for b, v in (rp.get("bands") or {}).items():
        e = float(v.get("error_hpbw", 0.0))
        t.add("flight", b, "navigation pointing", "transient", e, repoint.TOL_HPBW, _ratio(e, repoint.TOL_HPBW), None,
              "4.5", unit="HPBW")
        a, a_t = float(rp.get("accel_max_dps2", 0.0)), float(v.get("accel_tenable_dps2", 0.0))
        if a > repoint.ACCEL_MAX_DPS2 + TOL:
            t.add("flight", b, "angular acceleration", "transient", a, repoint.ACCEL_MAX_DPS2,
                  _ratio(a, repoint.ACCEL_MAX_DPS2), None, "4.5",
                  f"band loss sized for {repoint.ACCEL_MAX_DPS2:g} deg/s2: ranges optimistic (tenable {a_t:.3g})",
                  unit="deg/s2")
    for sid, s_ in (rp.get("sectors") or {}).items():
        for k, f in (s_.get("faces") or {}).items():
            v = float(f.get("neighbour_load_during_transit", 0.0))
            t.add("flight", f"{sid} F{int(k) + 1}", "migration", "transient", v, 1.0, v, None, "4.5",
                  f"neighbour face load while the volume crosses ({f.get('transit_s', 0.0):.2f} s)")


def _saturation(out: dict, state_at_spec: str) -> dict:
    """Step 5.2 P7 in the verdict: the state at the specification and the room beyond it per track bracket."""
    pa = out.get("power_allocation") or {}
    p2 = [p_["label"] for p_ in pa.get("partial", [])
          if p_["tier"] == "always-on" or p_["tier"].split(" ", 1)[0] in ("P1", "P2")]
    sat = out.get("saturation") or {}
    cr = sat.get("critique") or {}
    brackets = {}
    for s, b in (sat.get("brackets") or {}).items():
        brackets[s] = {"n_targets": b.get("n_targets"), "manageable_alone": b.get("extra_alone"),
                       "manageable_cumulative": (sat.get("cumulative") or {}).get("extra", {}).get(s),
                       "binding": b.get("binding_alone"),
                       "critical_alone": (cr.get("brackets") or {}).get(s, {}).get("extra_critique"),
                       "critical_cumulative": (cr.get("cumulative") or {}).get("extra", {}).get(s),
                       "t_max_s": (cr.get("track_floors") or {}).get(s, {}).get("t_max_s")}
    return {"state_at_spec": "P2 unreachable" if p2 else state_at_spec, "p2_unreachable": p2,
            "mode": cr.get("mode", margin.MODE_DEFAULT), "load_ceiling": sat.get("load_ceiling"),
            "brackets": brackets, "first_shed": cr.get("first_shed")}


def verdict(mission: dict, out: dict) -> dict:
    """The verdict of an assembled optimizer.run() result (see the module docstring)."""
    ceiling = margin.load_ceiling(mission)
    secs = {s["id"]: s for s in mission.get("sectors", [])}
    t = _Table(ceiling)
    _sectors(t, out, secs)
    _tracking(t, out, secs)
    faces = _faces(t, out)
    ship = _ship(t, out)
    _flight(t, out)
    counts = {s: sum(1 for r in t.rows if r["state"] == s) for s in STATES}
    state = "KO" if counts["KO"] else "ALERT" if counts["ALERT"] else "OK"
    if out.get("cancelled"):
        state = "INCOMPLETE"
    res = [r for r in t.rows if r["kind"] == "resource" and r["load"] is not None]
    worst = max(res, key=lambda r: r["load"]) if res else None
    limiting = ({"object": worst["object"], "resource": worst["quantity"], "load": worst["load"],
                 "state": worst["state"]} if worst else None)
    face_res = [r for r in res if r["family"] == "face"]
    sat_state = ("over-subscribed" if any(r["load"] > 1.0 + TOL for r in face_res + [r for r in res if r["family"] == "ship"])
                 else "reserve in use" if any(r["load"] > ceiling + TOL for r in face_res) else "nominal")
    failing_p = sorted({r["priority"] for r in t.rows if r["state"] == "KO" and r["priority"]})
    return {"state": state, "load_ceiling": ceiling, "counts": counts, "limiting": limiting, "faces": faces,
            "ship": ship, "ko_priorities": failing_p, "saturation": _saturation(out, sat_state),
            "constraints": t.rows}


def summary_lines(v: dict) -> List[str]:
    """The verdict as text: state, limiting resource, then every constraint in alert or failed."""
    lim = v.get("limiting")
    lines = [f"VERDICT: {v['state']} ({v['counts']['KO']} KO, {v['counts']['ALERT']} alert, {v['counts']['OK']} OK; "
             f"load ceiling {100 * v['load_ceiling']:.0f} %)"]
    if lim:
        lines.append(f"limiting resource: {lim['object']} {lim['resource']} at {100 * lim['load']:.1f} % ({lim['state']})")
    sat = v.get("saturation") or {}
    lines.append(f"saturation at the specification: {sat.get('state_at_spec')} (mode {sat.get('mode')})"
                 + (f", first shed beyond: {sat['first_shed']}" if sat.get("first_shed") else ""))
    for r in sorted((r for r in v["constraints"] if r["state"] != "OK"), key=lambda r: (r["state"] != "KO", r["id"])):
        load = "-" if r["load"] is None else f"{100 * r['load']:.1f} %"
        pr = f" [{r['priority']}]" if r["priority"] else ""
        lines.append(f"  {r['state']:5s} {r['object']} {r['quantity']}{pr}: load {load}"
                     + (f" -- {r['note']}" if r["note"] else ""))
    return lines
