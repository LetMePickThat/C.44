#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DIAGNOSIS PER CONSTRAINT OF THE FAST ESTIMATE (step 5.5, point A, framed with the user on 13 November 2026, Q1-Q6).

The mission preparation shows, while the operator edits, what limits the mission: per face, every resource the fast
estimate holds (energy, S time, Ku time from the final allocation rule; memory bandwidth and processing from the
margin-reuse loads) with its load against the load ceiling and its state; who weighs on it (each demand's share:
search, confirmation, tracks, always-on); per search volume what it is served (range held, revisit) and what limits
it; and, on a constraint in alert or failed, one estimated lever per item that would relieve it alone. No physics:
every figure is read from cost_estimate.live_allocation's result (nominal view: no target present; peak view: every
track present, the just need of 12 November 2026). Heat, beams, storage and links are the exact calculation's only.

States (as core/validation, step 5.3):
  energy, S time, Ku time  OK; ALERT when the rule holds a demand below its request on it (the volume degraded);
                           NOK when the demand held is never shed (P0 to P2: mission not tenable) or nothing serves it;
  memory bandwidth, compute  (at the revisits asked) OK up to the ceiling, ALERT up to 1, NOK above.
Levers (estimated, to be verified by Calculate): the deficit of a constraint is what the demands held on it miss
(their full need minus what they get) or, for the memory bandwidth and the processing, the load above the ceiling; an
item of share a relieves it alone when a > deficit, keeping the fraction f = 1 - deficit / a of its load, through the
rule's own laws: search range R' = R f^(1/4) below which the revisit grows (allocation.search_effect, rho_min), or revisit
RR' = RR / f; tracks n' = n f, or period T' = T / f; none alone: one factor on every search and track item of the
constraint. The memory bandwidth and the processing scale with the pointing rate: revisit only. Only a constraint at
the load ceiling holds a demand (the rule fills each one up to it).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

RESOURCES = ("energy", "S time", "Ku time")
OTHER = ("memory bandwidth", "compute")
STATE_RANK = {"OK": 0, "ALERT": 1, "NOK": 2}
_EPS = 1e-6


def _tier_never_shed(tier: str) -> bool:
    return tier[:2] in ("P0", "P1", "P2")


def _levers(rec: dict, sec: Optional[dict], f: float) -> List[str]:
    """The changes keeping the fraction f of one item's load (module docstring), as text."""
    if sec is None or not 0.0 < f < 1.0:
        return []
    out = []
    if rec.get("kind") == "search":
        from . import allocation as al
        r, rr = float(sec.get("range_km") or 0.0), float(sec.get("rr_target_s") or 0.0)
        if r > 0 and rr > 0:
            rho, rr_t = al.search_effect(f, rr)       # (the rule's own law: below rho_min^4 the revisit grows too)
            out.append(f"range {r:.0f} -> {r * rho:.0f} km" + (f" and revisit {rr:g} -> {rr_t:.3g} s"
                                                               if rr_t > rr * (1 + 1e-6) else ""))
        if rr > 0:
            out.append(f"revisit {rr:g} -> {rr / f:.3g} s")
    elif rec.get("kind") == "track":
        try:
            br = int(str(rec.get("label", "")).rsplit("/", 1)[1])
        except (IndexError, ValueError):
            br = None
        brs = sec.get("tracking") or []
        if br is not None and br < len(brs):
            b = brs[br]
            n, t = float(b.get("n_targets") or 0.0), float(b.get("rr_s") or 0.0)
            if n > 0:
                out.append(f"tracks {n:g} -> {math.floor(n * f + 1e-9):g}")
            if t > 0:
                out.append(f"track period {t:g} -> {t / f:.3g} s")
        else:                                     # (the fast estimate's track demand: every bracket of the volume)
            out.append(f"tracks x{f:.2f} (every bracket)")
    return out


def _rule_rows(rep: dict, view: str, ceiling: float, secs: Dict[str, dict]) -> List[dict]:
    """The constraints of one rule report (nominal or peak): per face and resource, and the ship energy."""
    demands = rep.get("demands") or []
    rows = []
    for k in range(4):
        f = (rep.get("faces") or {}).get(str(k)) or {}
        for res in RESOURCES:
            load = float(f.get(res + " load", 0.0) or 0.0)
            used = float(f.get(res, 0.0) or 0.0)
            rows.append(_row(f"F{k + 1}", k, res, view, load, used, ceiling, demands, secs,
                             lambda rec, k_=k, r_=res: float(((rec.get("faces") or {}).get(str(k_)) or {}).get(r_, 0.0))))
    ship = rep.get("ship") or {}
    if ship:
        rows.append(_row("ship", None, "energy", view, float(ship.get("energy load", 0.0)),
                         float(ship.get("energy", 0.0)), ceiling, demands, secs,
                         lambda rec: sum(float(v.get("energy", 0.0)) for v in (rec.get("faces") or {}).values())))
    return rows


def _row(where: str, k: Optional[int], res: str, view: str, load: float, used: float, ceiling: float,
         demands: List[dict], secs: Dict[str, dict], amount) -> dict:
    who, deficit, held, never = [], 0.0, [], False
    binding = load >= ceiling * (1.0 - 1e-4)          # (a demand is held by the constraints at the ceiling only)
    for rec in demands:
        a = amount(rec)
        if a <= 0:
            continue
        m = float(rec.get("level", 1.0))
        full = a / m if 0.0 < m < 1.0 - _EPS else a
        if m < 1.0 - _EPS and binding:
            deficit += full - a
            held.append(rec.get("label"))
            never = never or _tier_never_shed(str(rec.get("tier", "")))
        who.append({"label": rec.get("label"), "sector": rec.get("sector"), "kind": rec.get("kind"),
                    "tier": rec.get("tier"), "share": a / used if used > 0 else 0.0, "full": full,
                    "level": m, "_rec": rec})
    who.sort(key=lambda w: -w["share"])
    state = "NOK" if never else "ALERT" if held else "OK"
    levers = []
    if held and deficit > 0:
        for w in who[:3]:
            if w["full"] > deficit * (1.0 + 1e-9):
                f = 1.0 - deficit / w["full"]
                for t in _levers(w["_rec"], secs.get(str(w["sector"])), f):
                    levers.append(f"{w['label']}: {t}")
        var = [w for w in who if w["kind"] in ("search", "track")]
        tot = sum(w["full"] for w in var)
        if not levers and tot > deficit * (1.0 + 1e-9):   # (none alone: one factor on every search and track item)
            f = 1.0 - deficit / tot
            levers.append(f"every search and track on it: revisit / track period x{1.0 / f:.3g}")
    for w in who:
        w.pop("_rec", None)
    return {"where": where, "face": k, "resource": res, "view": view, "load": load, "ceiling": ceiling,
            "state": state, "held": held, "who": who, "levers": levers,
            "note": ("held: " + ", ".join(map(str, held))) if held else
                    ("at the load ceiling (the spare given out)" if load >= ceiling * (1.0 - 1e-4) else "")}


def _other_rows(ol: dict, ceiling: float, secs: Dict[str, dict], by_sector: Dict[str, dict]) -> List[dict]:
    rows = []
    shares = ol.get("shares") or {}
    for name, load in sorted((ol.get("asked") or {}).items()):
        where, res = name.split(" ", 1)
        a = shares.get(name) or {}
        state = "NOK" if load > 1.0 + 1e-9 else "ALERT" if load > ceiling + 1e-9 else "OK"
        who = sorted(({"label": f"search {s}", "sector": s, "kind": "search", "share": v / load if load else 0.0,
                       "full": v} for s, v in a.items()), key=lambda w: -w["share"])
        levers = []
        if state != "OK":
            deficit = load - ceiling
            for w in who[:3]:
                if w["full"] > deficit:
                    f = 1.0 - deficit / w["full"]
                    sec = secs.get(str(w["sector"]))
                    rr = float((sec or {}).get("rr_target_s") or 0.0)
                    if rr > 0:
                        levers.append(f"{w['label']}: revisit {rr:g} -> {rr / f:.3g} s")
        rows.append({"where": where, "face": int(where[1:]) - 1 if where[1:].isdigit() else None, "resource": res,
                     "view": "nominal", "load": float(load), "ceiling": ceiling, "state": state, "held": [],
                     "who": who, "levers": levers, "note": "at the revisits asked"})
    return rows


def live(alloc: Optional[dict], sectors: List[dict]) -> Optional[dict]:
    """{"rows": constraints (worst first), "volumes": per search volume, "state", "not_covered"} of a fast-estimate
    result (cost_estimate.live_allocation); None without one."""
    if not alloc or "faces" not in alloc:
        return None
    secs = {str(s.get("id")): s for s in sectors}
    ceiling = float(alloc.get("ceiling", 1.0) or 1.0)
    rows = _rule_rows(alloc, "nominal", ceiling, secs)
    pk = (alloc.get("peak") or {}).get("report")
    if pk:
        rows += _rule_rows(pk, "peak", ceiling, secs)
    if alloc.get("other_loads"):
        rows += _other_rows(alloc["other_loads"], ceiling, secs, alloc.get("by_sector") or {})
    rows.sort(key=lambda r: (-STATE_RANK[r["state"]], r["face"] if r["face"] is not None else -1,
                             r["view"] != "nominal", -r["load"]))
    limits: Dict[str, List[str]] = {}
    for r in rows:
        for h in r["held"]:
            limits.setdefault(str(h), []).append(f"{r['where']} {r['resource']}" + (" (peak)" if r["view"] == "peak"
                                                                                     else ""))
    vols = []
    shed = ((alloc.get("peak") or {}).get("shed") or {})
    for sid, rec in (alloc.get("by_sector") or {}).items():
        sec = secs.get(str(sid)) or {}
        r_asked, rr_asked = float(sec.get("range_km") or 0.0), float(sec.get("rr_target_s") or 0.0)
        rho, rr = float(rec.get("rho", 1.0)), float(rec.get("rr_tenable_s", rr_asked) or rr_asked)
        oor = (alloc.get("out_of_reach") or {}).get(sid)
        state = ("NOK" if str(sid) in (alloc.get("no_confirmation") or []) else
                 "ALERT" if rho < 1.0 - _EPS or rr > rr_asked * (1 + 1e-6) or oor or sid in shed else "OK")
        vols.append({"sector": sid, "state": state, "range_asked_km": r_asked, "range_held_km": r_asked * rho,
                     "rho": rho, "rr_asked_s": rr_asked, "rr_served_s": rr, "level": float(rec.get("level", 1.0)),
                     "limited_by": limits.get(str(rec.get("label")), []),
                     "out_of_reach_share": oor["share"] if oor else 0.0,
                     "peak_shed_rho": (shed.get(sid) or {}).get("rho"),
                     "no_confirmation": str(sid) in (alloc.get("no_confirmation") or [])})
    vols.sort(key=lambda v: (-STATE_RANK[v["state"]], str(v["sector"])))
    worst = max([r["state"] for r in rows] + [v["state"] for v in vols] + ["OK"], key=STATE_RANK.__getitem__)
    return {"rows": rows, "volumes": vols, "state": worst, "ceiling": ceiling,
            "not_covered": "heat, beams, storage and links: the exact calculation (Calculate) only"}
