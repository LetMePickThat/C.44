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
             in use), NOK above 1;
  spec       (a requirement of the operator: range, revisit, confirmation, latency, track period and reach,
             discrimination) OK up to 1, NOK above; ALERT where the model allows a tolerance (positions short of range
             within the sector's short_tol, discrimination reached below the shortest range of interest);
  transient  (a passing overload: migration between faces while the ship turns, navigation pointing) OK up to 1,
             ALERT above.
The mission verdict is NOK as soon as one constraint is NOK (the operator's specification is not held), ALERT when one
is in alert, OK otherwise; every constraint carries its priority tier (P0 to P5, step 5.2 P1) so that a failure of
P0 to P2 shows at once. The limiting resource is the resource constraint with the highest load, per face and for the
mission. The saturation summary (step 5.2 P7) gives the state at the specification and the room beyond it.

Pass P2 (validated by the user on 27 October 2026): the quantified relaxation advice (verdict["advice"]), for every
constraint in alert or failed. A resource of a face over its target (the load ceiling; the value at 1 as a second
figure) by dL is relieved by an item (search volume, confirmation, track bracket, burn-through) of share s > dL in its
load, the item keeping the fraction f = 1 - dL / s of its load, through the laws
    search:  load' / load = (R' / R)^4 SNR_req(q') / SNR_req(q) RR / RR', q = v_close_max RR / R (the range is the
             worst one: a target missed at the start of a cycle is still detected by R, the looks that count being
             spaced v RR beyond R; user, 27 October 2026), R' and RR' solved, no revisit lever when a longer revisit
             does not relieve; dimensioning RCS s' = s / f
    tracks:  number n' = n f | period T' = T / f | accuracy of the driver (ANG zone, RNG, DOP) a' = a / sqrt(f)
(the transmit time is taken energy-limited, so the same laws), the three largest items alone, and one uniform factor
on every item of the face (when none suffices alone); the confirmation alternatives (exact, step 3.2), the face cap
and the burn-through are given as they are. The energy of a face is need / allocation: with the ship's power enough
the allocation is pro rata of the needs, every face load is the ship's (need / 8 MW) and an item weighs its total
draw over the ship; a capped face weighs its own draw over its cap. Several resources over on one face: the advice
aims at the highest and checks the other linear ones by the same law (others_left). Resources in steps (beams,
memory, compute, links, storage) and the heat have no law: a uniform proportional factor, flagged approximate. A
failed requirement gets its direct value (range at the worst bin of the plan kept, confirmation probability or latency
reached, longest tenable track period, tenable revisit or tracks of a power tier short). Every figure is an estimate:
verify() recomputes the mission with the lever and bisects (tool.validation.verify_bisections steps), and only the
verified figure stands.
"""
from __future__ import annotations

import copy
import math
from typing import Callable, Dict, List, Optional, Tuple

from . import allocation
from . import compute
from . import margin
from . import optimizer as mo
from . import radar_model as rm
from . import repoint
from .params import P

VERIFY_BISECTIONS = int(P.get("tool.validation.verify_bisections"))
VERIFY_EXTENSIONS = 2          # the estimate doubled at most twice when it does not hold yet (method, P2)
TOL = 1e-9
STATES = ("OK", "ALERT", "NOK")
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
        return "NOK" if kind != "transient" else "ALERT"
    if kind == "resource":
        return "OK" if load <= ceiling + TOL else "ALERT" if load <= 1.0 + TOL else "NOK"
    if kind == "transient":
        return "OK" if load <= 1.0 + TOL else "ALERT"
    return "OK" if load <= 1.0 + TOL else "NOK"


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
        state = "OK" if short <= 0.0 else "ALERT" if short <= tol + TOL else "NOK"
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
    state = "NOK" if counts["NOK"] else "ALERT" if counts["ALERT"] else "OK"
    if out.get("cancelled"):
        state = "INCOMPLETE"
    res = [r for r in t.rows if r["kind"] == "resource" and r["load"] is not None]
    worst = max(res, key=lambda r: r["load"]) if res else None
    limiting = ({"object": worst["object"], "resource": worst["quantity"], "load": worst["load"],
                 "state": worst["state"]} if worst else None)
    face_res = [r for r in res if r["family"] == "face"]
    sat_state = ("over-subscribed" if any(r["load"] > 1.0 + TOL for r in face_res + [r for r in res if r["family"] == "ship"])
                 else "reserve in use" if any(r["load"] > ceiling + TOL for r in face_res) else "nominal")
    failing_p = sorted({r["priority"] for r in t.rows if r["state"] == "NOK" and r["priority"]})
    return {"state": state, "load_ceiling": ceiling, "counts": counts, "limiting": limiting, "faces": faces,
            "ship": ship, "nok_priorities": failing_p, "saturation": _saturation(out, sat_state),
            "constraints": t.rows, "advice": advice(mission, out, t.rows, ceiling)}


def summary_lines(v: dict) -> List[str]:
    """The verdict as text: state, limiting resource, then every constraint in alert or failed."""
    lim = v.get("limiting")
    lines = [f"VERDICT: {v['state']} ({v['counts']['NOK']} NOK, {v['counts']['ALERT']} alert, {v['counts']['OK']} OK; "
             f"load ceiling {100 * v['load_ceiling']:.0f} %)"]
    if lim:
        lines.append(f"limiting resource: {lim['object']} {lim['resource']} at {100 * lim['load']:.1f} % ({lim['state']})")
    sat = v.get("saturation") or {}
    lines.append(f"saturation at the specification: {sat.get('state_at_spec')} (mode {sat.get('mode')})"
                 + (f", first shed beyond: {sat['first_shed']}" if sat.get("first_shed") else ""))
    for r in sorted((r for r in v["constraints"] if r["state"] != "OK"), key=lambda r: (r["state"] != "NOK", r["id"])):
        load = "-" if r["load"] is None else f"{100 * r['load']:.1f} %"
        pr = f" [{r['priority']}]" if r["priority"] else ""
        lines.append(f"  {r['state']:5s} {r['object']} {r['quantity']}{pr}: load {load}"
                     + (f" -- {r['note']}" if r["note"] else ""))
    if v.get("advice"):
        lines.append("advice (estimates; python -m core.validation <mission> --verify <n> recomputes one):")
        lines.extend("  " + advice_text(a) for a in v["advice"])
    return lines


# ----------------------------------------------------------------------------- P2: quantified relaxation advice
_LIN = ("energy", "S time", "Ku time")
_ACC_KEY = {"ANG": "angle_zone_deg", "RNG": "range_acc_m", "DOP": "vr_acc_mps"}
_UNITS = {"range_km": "km", "rr_target_s": "s", "rcs_m2": "m2", "rr_s": "s", "angle_zone_deg": "deg",
          "range_acc_m": "m", "vr_acc_mps": "m/s", "latency_max_s": "s", "face_elec_mw": "MW"}


def _safe(x: float, up: bool, integer: bool = False) -> float:
    """x rounded to 3 significant digits towards the side that keeps the relief (up: never below x)."""
    if integer:
        return float(math.ceil(x - 1e-9) if up else math.floor(x + 1e-9))
    if x <= 0 or not math.isfinite(x):
        return x
    q = 10.0 ** (math.floor(math.log10(x)) - 2)
    return (math.ceil(x / q - 1e-6) if up else math.floor(x / q + 1e-6)) * q   # 1e-6: solver noise


def _capped(pa: dict, k: int) -> bool:
    """A face held at its cap (feeder or operator): its pro rata share of the ship total is above the cap."""
    f = pa.get("faces", {}).get(str(k), {})
    need_tot = float(pa.get("need_MW") or 0.0)
    cap, need = f.get("cap_MW"), float(f.get("need_MW", 0.0) or 0.0)
    if cap is None or need <= 0 or need_tot <= 0:
        return False
    return float(cap) < float(pa.get("ship_MW") or 8.0) * need / need_tot * (1.0 - 1e-6)


def _items(mission: dict, out: dict) -> Dict[tuple, dict]:
    """Every variable item of the plan with its share in the linear loads: {(kind, sector, bracket): {"share":
    {(face, resource): s}, "ship": s}}; the energy share follows the allocation (module docstring)."""
    pa = out.get("power_allocation") or {}
    faces = pa.get("faces", {})
    ship_w = float(pa.get("ship_MW") or 8.0) * 1e6
    capped = {k: _capped(pa, k) for k in range(4)}
    sch = (out.get("scheduler") or {}).get("faces", {})
    cap_t = {(k, b): float((sch.get(str(k), {}).get(b) or {}).get("capacity", 1.0) or 1.0)
             for k in range(4) for b in ("S", "Ku")}
    items: Dict[tuple, dict] = {}

    def item(key):
        return items.setdefault(key, {"share": {}, "ship": 0.0})

    def add_t(key, k, b, frac):
        if frac:
            it = item(key)
            res = f"{b} time"
            it["share"][(k, res)] = it["share"].get((k, res), 0.0) + float(frac) / cap_t[(k, b)]

    for it in allocation.items(mission, out):
        kind = it["kind"]
        if kind not in ("search", "confirm", "track", "burn_through"):
            continue
        br = int(it["label"].rsplit("/", 1)[1]) if kind == "track" else None
        x = item((kind, it["sector"], br))
        x["ship"] += sum(it["w"]) / ship_w
        for k in range(4):
            if it["w"][k] > 0:
                alloc = float(faces.get(str(k), {}).get("alloc_MW", 0.0)) * 1e6
                x["share"][(k, "energy")] = (it["w"][k] / alloc if capped[k] and alloc > 0
                                             else sum(it["w"]) / ship_w)
    for r in out.get("sectors", []):
        if r.get("status") == "OK":
            for kk, fc in r.get("faces", {}).items():
                add_t(("search", r["id"], None), int(kk), "S", fc.get("time_fraction"))
    for s in (out.get("tracking") or {}).get("sectors", []):
        for kk, fc in ((s.get("confirmation") or {}).get("faces") or {}).items():
            add_t(("confirm", s["id"], None), int(kk), "S", fc.get("s_time_fraction"))
            add_t(("confirm", s["id"], None), int(kk), "Ku", fc.get("ku_time_fraction"))
        for i, b in enumerate(s.get("at", [])):
            for kk, fc in (b.get("faces") or {}).items():
                add_t(("track", s["id"], i), int(kk), "Ku", fc.get("ku_time_fraction"))
        for kk, fc in ((s.get("ku_burn_through") or {}).get("faces") or {}).items():
            add_t(("burn_through", s["id"], None), int(kk), "Ku", fc.get("ku_time_fraction"))
    return items


class _SearchLaw:
    """Load of a search volume against its range and revisit (user, 27 October 2026): the range of a volume is the
    worst one, the detection being due by R even for a target missed at the start of a cycle, so the looks that count
    are spaced q = v_close_max RR / R beyond R (step 2.3, detection.required_snr_db), and
        load' / load = (R' / R)^4 x SNR_req(q') / SNR_req(q) x RR / RR'
    with the main waveform of the volume (pulses, Pfa), its fluctuation and its cumulative Pd. A longer revisit
    spaces the looks: it may raise the load (a fast closer), and then gives no lever."""

    def __init__(self, sec: dict, res: dict):
        from . import detection
        wf, tgt = res.get("wf") or {}, res.get("target") or {}
        self.r0, self.rr0 = float(sec.get("range_km", 0.0)), float(sec.get("rr_target_s", 0.0))
        self.v = float(sec.get("v_close_max_mps", 0.0) or 0.0)
        n, pfa = int(wf.get("n_pulses", 1) or 1), float(wf.get("pfa") or 1e-6)
        sw, pd = int(tgt.get("swerling", 1) or 1), float(res.get("pd_cumulative") or 0.9)
        self.snr = lambda q: detection.required_snr_db(n, sw, pfa, pd, q=q)
        self.snr0 = self.snr(self._q(self.r0, self.rr0)) if self.r0 > 0 else 0.0

    def _q(self, r_km: float, rr: float) -> float:
        return self.v * rr / (r_km * 1e3) if r_km > 0 else 0.0

    def ratio(self, r_km: float, rr: float) -> float:
        return ((r_km / self.r0) ** 4 * 10.0 ** ((self.snr(self._q(r_km, rr)) - self.snr0) / 10.0) * self.rr0 / rr)

    def range_at(self, f: float) -> Optional[float]:
        """R' keeping the fraction f of the load (bisection: the load rises with R'), None if none does."""
        lo, hi = self.r0 * 1e-3, self.r0
        if self.r0 <= 0 or self.ratio(lo, self.rr0) > f:
            return None
        for _ in range(40):
            mid = math.sqrt(lo * hi)
            lo, hi = (mid, hi) if self.ratio(mid, self.rr0) < f else (lo, mid)
        return lo

    def rr_at(self, f: float) -> Optional[float]:
        """RR' >= RR keeping the fraction f (first crossing on a geometric scan up to 100 RR, then bisection), None
        if no longer revisit does (its minimum load is in self.rr_min)."""
        if self.rr0 <= 0 or self.r0 <= 0:
            return None
        grid = [self.rr0 * 10.0 ** (i / 40.0) for i in range(1, 81)]
        vals = [self.ratio(self.r0, x) for x in grid]
        self.rr_min = grid[min(range(len(grid)), key=vals.__getitem__)]
        i = next((j for j, v in enumerate(vals) if v <= f), None)
        if i is None:
            return None
        lo, hi = (self.rr0 if i == 0 else grid[i - 1]), grid[i]
        for _ in range(30):
            mid = math.sqrt(lo * hi)
            lo, hi = (lo, mid) if self.ratio(self.r0, mid) <= f else (mid, hi)
        return hi


def _levers(key: tuple, mission: dict, out: dict) -> List[Tuple[dict, Callable[[float], float], bool, bool]]:
    """(change without its value, value at the kept fraction f, rounded up, integer) for each lever of an item."""
    kind, sid, br = key
    secs = {s["id"]: s for s in mission.get("sectors", [])}
    sec = secs.get(sid, {})
    res = {r["id"]: r for r in out.get("sectors", [])}.get(sid, {})
    out_l = []
    if kind == "search":
        rng, rr = float(sec.get("range_km", 0.0)), float(sec.get("rr_target_s", 0.0))
        rcs = float((res.get("target") or {}).get("rcs_m2") or sec.get("rcs_m2") or 0.0)
        law = _SearchLaw(sec, res)
        out_l += [({"sector": sid, "bracket": None, "key": "range_km", "current": rng}, law.range_at, False, False),
                  ({"sector": sid, "bracket": None, "key": "rr_target_s", "current": rr}, law.rr_at, True, False)]
        if rcs > 0:
            out_l.append(({"sector": sid, "bracket": None, "key": "rcs_m2", "current": rcs}, lambda f: rcs / f,
                          True, False))
    elif kind == "track":
        brs = sec.get("tracking", [])
        b = brs[br] if br is not None and br < len(brs) else {}
        at = next((s for s in (out.get("tracking") or {}).get("sectors", []) if s["id"] == sid), {}).get("at", [])
        drv = at[br].get("driver") if br is not None and br < len(at) else None
        n, period = float(b.get("n_targets", 0.0)), float(b.get("rr_s", 0.0))
        out_l += [({"sector": sid, "bracket": br, "key": "n_targets", "current": n}, lambda f: n * f, False, True),
                  ({"sector": sid, "bracket": br, "key": "rr_s", "current": period}, lambda f: period / f, True, False)]
        acc = _ACC_KEY.get(drv)
        if acc and b.get(acc):
            a0 = float(b[acc])
            out_l.append(({"sector": sid, "bracket": br, "key": acc, "current": a0}, lambda f: a0 / math.sqrt(f),
                          True, False))
    return out_l


def _change(c: dict, fn, up: bool, integer: bool, f: Optional[float]) -> Optional[dict]:
    if f is None or f <= 0:
        return None
    v = fn(f)
    return None if v is None else dict(c, proposed=_safe(v, up, integer))


def _label(key: tuple) -> str:
    kind, sid, br = key
    return {"search": f"search {sid}", "confirm": f"confirmation {sid}", "track": f"tracks {sid}/{br}",
            "burn_through": f"burn-through {sid}"}[kind]


def _uniform(items: Dict[tuple, dict], face: Optional[int], res: Optional[str], f: float, mission: dict,
             out: dict, pick=None) -> List[dict]:
    """The revisit lever on every search and track item weighing on a face (or the ship), at one kept fraction f
    (search RR / f, track period / f: the same law)."""
    ch = []
    for key, it in sorted(items.items(), key=lambda kv: str(kv[0])):
        if key[0] not in ("search", "track") or (pick and not pick(key)):
            continue
        w = it["ship"] if face is None else (sum(v for (k, _r), v in it["share"].items() if k == face) if res is None
                                             else it["share"].get((face, res), 0.0))
        if w <= 0:
            continue
        c, fn, up, integer = _levers(key, mission, out)[1]           # the revisit / period lever
        x = _change(c, fn, up, integer, f)
        if x:
            ch.append(x)
    return ch


def advice(mission: dict, out: dict, rows: List[dict], ceiling: float) -> List[dict]:
    """Quantified relaxation advice for every constraint in alert or failed (module docstring, P2)."""
    try:
        items = _items(mission, out)
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        items = {}
    by_id = {r["id"]: r for r in rows}
    nok_before = [r["id"] for r in rows if r["state"] == "NOK"]     # verify(): a relaxation may not add a failure
    adv: List[dict] = []

    def add(row, kind, changes, item=None, f=None, f1=None, law="", approx=False, others=None, note="",
            target=None):
        adv.append({"n": len(adv) + 1, "constraint": row["id"], "object": row["object"], "quantity": row["quantity"],
                    "state": row["state"], "load": row["load"], "target_load": ceiling if target is None else target,
                    "kind": kind, "item": item, "changes": changes, "kept_fraction": f, "kept_fraction_at_1": f1,
                    "law": law, "approximate": approx, "others_left": others or [], "note": note,
                    "nok_before": nok_before, "verified": None})

    for row in rows:
        if row["state"] == "OK":
            continue
        q, obj = row["quantity"], row["object"]
        if row["kind"] == "resource" and row["load"] is not None:
            face = int(obj[1:]) - 1 if row["family"] == "face" else None
            if row["family"] == "ship" and q != "energy":
                continue
            if q == "energy" and face is not None and not _capped(out.get("power_allocation") or {}, face):
                continue                      # its load is the ship's (need / 8 MW): advised once, on the ship
            dl, dl1 = row["load"] - ceiling, row["load"] - 1.0
            approx = (q not in _LIN) or (out.get("power_allocation") or {}).get("status") == "SHORT"
            n_before = len(adv)
            if q in _LIN:
                def share(it, r_=q):
                    return it["ship"] if face is None else it["share"].get((face, r_), 0.0)
                ranked = sorted(((share(it), key) for key, it in items.items() if share(it) > 0), reverse=True)
                for s, key in [x for x in ranked if x[0] > dl][:3]:
                    f = 1.0 - dl / s
                    f1 = 1.0 - dl1 / s if dl1 > 0 else None
                    others = []
                    if face is not None:
                        for r2 in _LIN:
                            o = by_id.get(f"F{face + 1}.{r2}")
                            if r2 == q or o is None or o["load"] is None or o["load"] <= ceiling + TOL:
                                continue
                            left = o["load"] - items[key]["share"].get((face, r2), 0.0) * (1.0 - f)
                            if left > ceiling + TOL:
                                others.append(f"{o['object']} {r2} still {100 * left:.1f} %")
                    for c, fn, up, integer in _levers(key, mission, out):
                        x = _change(c, fn, up, integer, f)
                        if x:
                            x1 = _change(c, fn, up, integer, f1)
                            x["proposed_at_1"] = x1["proposed"] if x1 else None
                            add(row, "alone", [x], _label(key), f, f1, _LAW[c["key"]], approx, others)
                        elif c["key"] == "rr_target_s":
                            rr_min = getattr(getattr(fn, "__self__", None), "rr_min", None)
                            add(row, "none", [], _label(key), f, f1, "no revisit lever", approx, others,
                                note="a longer revisit spaces the looks that count (v_close_max x RR beyond the "
                                     "range) and does not bring the load down enough"
                                     + (f" (lowest load near RR {rr_min:.3g} s)" if rr_min else ""))
                    if key[0] == "burn_through":
                        add(row, "alone", [{"sector": None, "bracket": None, "key": "burn_through_allowed",
                                            "current": True, "proposed": False}], _label(key), 0.0, None,
                            "burn-through look removed", approx, others)
                rev = [s for s, key in ranked if key[0] in ("search", "track")]
                total = sum(rev)
                if total > dl and len(rev) > 1:
                    f = 1.0 - dl / total
                    ch = _uniform(items, face, None if face is None else q, f, mission, out)
                    if ch:
                        add(row, "uniform", ch, "every search and track on " + (obj if face is not None else "the ship"),
                            f, 1.0 - dl1 / total if dl1 > 0 else None, "revisit and track period / f", approx)
                caps = mission.get("global", {}).get("face_elec_mw")
                cur = list(caps) if isinstance(caps, list) else [float(caps or 0.0)] * 4
                if q == "energy" and face is not None and cur[face] > 0:    # an operator cap holds the face
                    fa = (out.get("power_allocation") or {}).get("faces", {}).get(str(face), {})
                    new = list(cur)
                    new[face] = _safe(min(float(fa["need_MW"]) / ceiling, float(rm.FACE_FEEDER_MAX_MW)), True)
                    if new[face] > cur[face]:
                        add(row, "cap", [{"sector": None, "bracket": None, "key": "face_elec_mw", "current": cur,
                                          "proposed": new}], f"cap of {obj}", None, None,
                            "cap = need / load ceiling (within the feeder)", False)
                if face is not None:
                    for s in (out.get("tracking") or {}).get("sectors", []):
                        alts, mode = s.get("alternatives") or {}, s.get("confirm_mode")
                        cur = (alts.get(mode) or {}).get("mission_max_load")
                        if cur is None or str(face) not in ((s.get("confirmation") or {}).get("faces") or {}):
                            continue
                        for m_, a in alts.items():
                            if m_ != mode and a.get("admissible") and a.get("mission_max_load", cur) < cur - 1e-6:
                                add(row, "exact", [{"sector": s["id"], "bracket": None, "key": "confirm_mode",
                                                    "current": mode, "proposed": m_}], f"confirmation {s['id']}",
                                    None, None, f"mission's most loaded face {100 * cur:.1f} % -> "
                                    f"{100 * a['mission_max_load']:.1f} % (step 3.2)", False)
                if len(adv) == n_before:                  # nothing moves enough: say what the load is made of
                    var = sum(s for s, _k in ranked)
                    add(row, "none", [], None, law="no lever", approx=approx,
                        note=f"the variable items weigh {100 * var:.2g} % of the {100 * row['load']:.3g} %, the "
                             f"fixed part (always-on, calibration) {100 * (row['load'] - var):.3g} %, for a target "
                             f"{100 * ceiling:.0f} %")
            elif face is not None and (q.endswith("TRSA link") or q.endswith("processor input") or q == "memory"):
                add(row, "none", [], f"every look on {obj}", law="no revisit lever", approx=True,
                    note="a peak per look, set by the waveforms (bandwidth, beams per look), not by the revisits; "
                         "only a recomputation with other waveforms moves it")
            elif face is not None:                       # resources in steps and heat: proportional, approximate
                fixed = 0.0
                if q == "processing power":              # its static part does not move with the revisits
                    pw = ((out.get("compute") or {}).get("faces", {}).get(str(face), {}) or {}).get("power_kW", {})
                    fixed = float(_ratio(pw.get("static"), pw.get("ceiling")) or 0.0)
                if ceiling <= fixed + TOL:
                    add(row, "none", [], f"every item on {obj}", law="no lever", approx=True,
                        note=f"the static part alone is {100 * fixed:.0f} % of the capacity")
                    continue
                f = (ceiling - fixed) / (row["load"] - fixed)
                band = q.split(" ", 1)[0] if q.split(" ", 1)[0] in ("S", "Ku") else None
                pick = (None if band is None else
                        (lambda key: key[0] == "search") if band == "S" else (lambda key: key[0] == "track"))
                ch = _uniform(items, face, None, f, mission, out, pick)
                if ch:
                    add(row, "uniform", ch, f"every {'search' if band == 'S' else 'track' if band else 'item'} on "
                        f"{obj}", f, (1.0 - fixed) / (row["load"] - fixed) if row["load"] > 1 else None,
                        "load taken proportional to the revisit (approximate: resource in steps)", True)
            continue
        _direct(row, out, mission, add)
    return adv


_LAW = {"range_km": "load ~ R^4 SNR_req(v RR / R): R' solved", "rr_target_s": "load ~ SNR_req(v RR / R) / RR: RR' solved",
        "rcs_m2": "load ~ 1/RCS: RCS' = RCS / f (threat assumption)", "n_targets": "load ~ n: n' = n f",
        "rr_s": "load ~ 1/T: T' = T / f", "angle_zone_deg": "load ~ 1/zone^2: zone' = zone / sqrt(f)",
        "range_acc_m": "load ~ 1/acc^2: acc' = acc / sqrt(f)", "vr_acc_mps": "load ~ 1/acc^2: acc' = acc / sqrt(f)"}


def _direct(row: dict, out: dict, mission: dict, add) -> None:
    """A failed requirement: its direct value (module docstring)."""
    q, obj = row["quantity"], row["object"]
    secs = {s["id"]: s for s in mission.get("sectors", [])}
    if q == "range":
        sec = secs.get(obj, {})
        r = {x["id"]: x for x in out.get("sectors", [])}.get(obj, {})
        want = float(sec.get("range_km", row["limit"]))
        short = float((r.get("range_km") or {}).get("positions_short_of_range_pct", 0.0)) / 100.0
        add(row, "direct", [{"sector": obj, "bracket": None, "key": "short_tol",
                             "current": float(sec.get("short_tol", mo.SHORT_TOL)), "proposed": _safe(short, True)}],
            f"search {obj}", law="the shortfall accepted", target=1.0)
        bins = [b for b in r.get("wf_by_bin", []) if b.get("range_km_at_bin_max") and b["range_km_at_bin_max"] < want]
        if bins:
            add(row, "direct", [{"sector": obj, "bracket": None, "key": "range_km", "current": want,
                                 "proposed": _safe(min(b["range_km_at_bin_max"] for b in bins), False)}],
                f"search {obj}", law="range of the short scan-angle bins, plan kept",
                note="short bins " + ", ".join(f"{b['theta_lo_deg']:g}-{b['theta_hi_deg']:g} deg "
                                               f"({b['range_km_at_bin_max']:.0f} km)" for b in bins)
                + "; or more energy on their faces", target=1.0)
        else:
            add(row, "direct", [], f"search {obj}", law="no range lever", target=1.0,
                note="the short positions are scattered directions (sky, clutter), not a whole scan-angle bin")
    elif q == "P confirm" and row["value"] is not None:
        add(row, "direct", [{"sector": obj, "bracket": None, "key": "confirm_p_min", "current": row["limit"],
                             "proposed": math.floor(float(row["value"]) * 1000) / 1000}], f"confirmation {obj}",
            law="P reached", target=1.0, note="or another confirmation mode")
    elif q == "confirmation latency":
        add(row, "direct", [{"sector": obj, "bracket": None, "key": "latency_max_s", "current": row["limit"],
                             "proposed": _safe(float(row["value"]), True)}], f"confirmation {obj}",
            law="latency reached (95 %)", target=1.0)
    elif q in ("track period", "track reach"):
        sid, rest = obj.split("/", 1)
        br = int(rest.split(" ", 1)[0])
        if q == "track period":
            at = next((s for s in (out.get("tracking") or {}).get("sectors", []) if s["id"] == sid), {}).get("at", [])
            alert = at[br].get("rr_alert_s") if br < len(at) else None
            if alert:
                add(row, "direct", [{"sector": sid, "bracket": br, "key": "rr_s", "current": row["limit"],
                                     "proposed": _safe(float(alert), True)}], f"tracks {sid}/{br}",
                    law="longest tenable period (step 3.3)", target=1.0)
        else:
            add(row, "direct", [{"sector": sid, "bracket": br, "key": "n_targets", "current": row["limit"],
                                 "proposed": math.floor(float(row["value"]) + 1e-9)}], f"tracks {sid}/{br}",
                law="tracks within the Ku reach", target=1.0, note="or a shorter r_max_km")
    elif q == "ship power served":
        pa = {p_["label"]: p_ for p_ in (out.get("power_allocation") or {}).get("partial", [])}.get(obj, {})
        if obj.startswith("search ") and pa.get("rr_tenable_s"):
            sid = obj.split(" ", 1)[1]
            add(row, "direct", [{"sector": sid, "bracket": None, "key": "rr_target_s",
                                 "current": float(secs.get(sid, {}).get("rr_target_s", 0.0)),
                                 "proposed": _safe(float(pa["rr_tenable_s"]), True)}], obj,
                law="tenable revisit of the tier (step 5.2 P1)", target=1.0)
        elif obj.startswith("tracks ") and pa.get("tracks_tenable") is not None:
            sid, br = obj.split(" ", 1)[1].rsplit("/", 1)
            b = (secs.get(sid, {}).get("tracking") or [{}])[int(br)]
            add(row, "direct", [{"sector": sid, "bracket": int(br), "key": "n_targets",
                                 "current": float(b.get("n_targets", 0.0)),
                                 "proposed": math.floor(float(pa["tracks_tenable"]) + 1e-9)}], obj,
                law="tenable tracks of the tier (step 5.2 P1)", target=1.0)


def _val(v) -> str:
    if isinstance(v, list):
        return "[" + ", ".join(f"{x:g}" for x in v) + "]"
    return f"{v:g}" if isinstance(v, (int, float)) and not isinstance(v, bool) else str(v)


def changes_text(a: dict, most: int = 4) -> str:
    """The changes of an advice as text: 'S01 range_km 1500 -> 1380 (1310 at 100 %) km; ...'."""
    ch = "; ".join(f"{c['sector'] + ('/' + str(c['bracket']) if c['bracket'] is not None else '') + ' ' if c['sector'] else ''}"
                   f"{c['key']} {_val(c['current'])} -> {_val(c['proposed'])}"
                   + (f" ({_val(c['proposed_at_1'])} at 100 %)" if c.get("proposed_at_1") is not None else "")
                   + (f" {_UNITS[c['key']]}" if c["key"] in _UNITS else "") for c in a["changes"][:most])
    if len(a["changes"]) > most:
        ch += f"; ... ({len(a['changes'])} changes)"
    return ch


def advice_text(a: dict) -> str:
    ch = changes_text(a)
    tail = [x for x in (a["law"], "approximate" if a["approximate"] else "", a["note"],
                        ", ".join(a["others_left"])) if x]
    ver = "" if a["verified"] is None else (" VERIFIED" if a["verified"].get("holds") else " NOT VERIFIED")
    return (f"[{a['n']}] {a['object']} {a['quantity']} ({a['state']}, target {100 * a['target_load']:.0f} %): "
            f"{a['kind']} on {a['item'] or '-'}: {ch or '-'}" + (f" -- {'; '.join(tail)}" if tail else "") + ver)


# ----------------------------------------------------------------------------- P2: verified figure
def _value(c: dict, t: float):
    """The value of a change at the step t of the way from the current to the proposed value (geometric for the
    positive numbers, the proposed value for the others when t > 0)."""
    cur, new = c["current"], c["proposed"]
    if isinstance(cur, (int, float)) and isinstance(new, (int, float)) and not isinstance(cur, bool) \
            and cur > 0 and new > 0:
        v = cur * (new / cur) ** t
        if c["key"] == "n_targets":
            v = float(math.floor(v + 1e-9) if new < cur else math.ceil(v - 1e-9))
        return v
    if isinstance(cur, list) and isinstance(new, list):
        return [a * (b / a) ** t if a > 0 and b > 0 else (b if t > 0 else a) for a, b in zip(cur, new)]
    return new if t > 0 else cur


def apply_changes(mission: dict, changes: List[dict], t: float = 1.0) -> dict:
    """The mission with the changes applied at the step t (_value)."""
    m = copy.deepcopy(mission)
    secs = {s["id"]: s for s in m.get("sectors", [])}
    for c in changes:
        v = _value(c, t)
        if c["sector"] is None:
            m.setdefault("global", {})[c["key"]] = v
        elif c["bracket"] is None:
            secs[c["sector"]][c["key"]] = v
        else:
            secs[c["sector"]]["tracking"][c["bracket"]][c["key"]] = v
    return m


def verify(mission: dict, a: dict, run: Optional[Callable[[dict], dict]] = None,
           progress: Optional[Callable[[str], None]] = None) -> dict:
    """The verified figure of one advice: the mission recomputed with its changes; when the estimate holds, bisection
    between the current values and the estimate (VERIFY_BISECTIONS recomputations) for the smallest relaxation that
    holds; when it does not, the change doubled at most VERIFY_EXTENSIONS times first. Holds = the constraint of the
    advice at or under its target load (absent counts as holding)."""
    run = run or (lambda m: mo.run(m))
    numeric = all(isinstance(c["current"], (int, float, list)) and not isinstance(c["current"], bool)
                  for c in a["changes"])
    runs = []

    nok0 = set(a.get("nok_before") or [])

    def holds(t: float) -> bool:
        res = run(apply_changes(mission, a["changes"], t))
        rows = res["verdict"]["constraints"]
        row = next((r for r in rows if r["id"] == a["constraint"]), None)
        load = None if row is None else row["load"]
        new_nok = sorted(r["id"] for r in rows if r["state"] == "NOK" and r["id"] not in nok0 and r["id"] != a["constraint"])
        ok = (row is None or (load is not None and load <= a["target_load"] + TOL
                              and (row["kind"] != "spec" or row["state"] != "NOK"))) and not new_nok
        runs.append({"t": t, "load": load, "holds": ok, "mission_state": res["verdict"]["state"], "new_nok": new_nok})
        if progress:
            progress(f"run {len(runs)}: step {t:.3g}, load {'-' if load is None else f'{100 * load:.1f} %'}"
                     + (f", new NOK {', '.join(new_nok)}" if new_nok else "") + f" -> {'holds' if ok else 'fails'}")
        return ok

    hi, ok = 1.0, holds(1.0)
    lo = 0.0                                  # the last step that fails (0: the current values)
    if numeric:
        for _ in range(VERIFY_EXTENSIONS):
            if ok:
                break
            lo, hi = hi, 2.0 * hi
            ok = holds(hi)
        if ok:
            for _ in range(VERIFY_BISECTIONS):
                mid = 0.5 * (lo + hi)
                if holds(mid):
                    hi = mid
                else:
                    lo = mid
    verified = [dict(c, verified=_value(c, hi)) for c in a["changes"]] if ok else []
    return {"holds": ok, "step": hi if ok else None, "changes": verified, "runs": runs}


def main(argv: List[str]) -> int:
    import json
    import sys
    if not argv or argv[0].startswith("-"):
        print("usage: python -m core.validation mission.json|mission.csv [--verify N] [report.json]")
        return 1
    mission = mo.load_mission(argv[0])
    n_ver = int(argv[argv.index("--verify") + 1]) if "--verify" in argv else None
    res = mo.run(mission)
    v = res["verdict"]
    for line in summary_lines(v):
        print(line)
    if n_ver is not None:
        a = next((x for x in v["advice"] if x["n"] == n_ver), None)
        if a is None:
            print(f"no advice {n_ver}")
            return 1
        a["verified"] = verify(mission, a, progress=lambda s: print("  " + s, flush=True))
        print(advice_text(a))
        for c in a["verified"]["changes"]:
            print(f"  verified: {c['key']} {c['current']} -> {c['verified']}")
    rest = [x for x in argv[1:] if x != "--verify" and not (n_ver is not None and x == str(n_ver))]
    if rest:
        with open(rest[0], "w", encoding="utf-8") as fh:
            json.dump(res, fh, indent=2)
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv[1:]))
