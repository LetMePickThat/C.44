#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIVE -- the live simulator's edits (fork RADARv6_RS, live architecture step L3, user 8 November 2026): the first
instant an edit of the scenario touches, and the run of the edited scenario taken up from the last checkpoint
strictly before it (sim/propagate.LiveSim, L2), its past kept.

Rules (first_change): the earliest instant where an input of the propagation differs, None when none does (a name,
an identity, a jammer, an RCS offset, the mission file, the navigation settings: no effect on the paths), 0 when in
doubt:
  - central body, epoch, step, seed, any key not known here: 0;
  - duration: the shorter of the two (a longer run goes on from its end, a shorter one is cut);
  - the ship: orbit 0; attitude waypoints: the time of the first one that differs (the first waypoint: 0);
    manoeuvres: the time of each one added or removed (impulse, burn: t_s; pn: t_s, or 0 when started by its range,
    user 8 November 2026); singer added or removed: 0 (its draws run from t = 0);
  - an object or a group: its entry (0 for an absolute state, its appearance time for a relative state or a group,
    the earlier of the two times when it changes) for a change of state, class, overrides, n, reference or spread;
    attitude waypoints and manoeuvres as the ship's; added or removed: its entry (0 with singer).
"""
from __future__ import annotations

import json
from typing import Dict, List, Optional, Tuple

from . import propagate as prop

# keys with no effect on the propagation
_FREE_META = {"name", "nav_errors", "log_nav", "sun_type", "earth_type"}
_FREE_TOP = {"mission", "format_version"}
_FREE_BODY = {"name", "identity", "jammer", "rcs_offset_db"}


def _same(a, b) -> bool:
    return json.dumps(a, sort_keys=True, default=str) == json.dumps(b, sort_keys=True, default=str)


def _entry(o: dict) -> float:
    """When an object or a group begins to be propagated: 0 for an absolute state, else its appearance time."""
    if "n" not in o and o.get("state", {}).get("type") != "relative":
        return 0.0
    return float(o.get("t_appear_s", 0.0))


def _manoeuvre_time(m: dict) -> float:
    if m["type"] in ("impulse", "burn"):
        return float(m["t_s"])
    if m["type"] == "pn":
        return float(m["t_s"]) if "t_s" in m else 0.0           # started by its range: 0 (user)
    return 0.0                                                  # singer, unknown


def _list_change(old: List[dict], new: List[dict], when) -> Optional[float]:
    """The earliest time of the items of one list and not the other (by value)."""
    a = [json.dumps(x, sort_keys=True, default=str) for x in old]
    b = [json.dumps(x, sort_keys=True, default=str) for x in new]
    out = [when(x) for x, s in zip(old, a) if s not in b] + [when(x) for x, s in zip(new, b) if s not in a]
    return min(out) if out else None


def _waypoints_change(old: List[dict], new: List[dict], entry: float) -> Optional[float]:
    """The time of the first waypoint that differs (the first one: the entry)."""
    for k in range(max(len(old), len(new))):
        if k >= len(old) or k >= len(new) or not _same(old[k], new[k]):
            if k == 0:
                return entry
            ts = [float(w[k]["t_s"]) for w in (old, new) if k < len(w)]
            return min(ts)
    return None


def _body_change(o: Optional[dict], n: Optional[dict]) -> Optional[float]:
    """The first instant an object, a group or the ship differs (None: no physical change)."""
    if o is None and n is None:
        return None
    if o is None or n is None:
        x = n if o is None else o
        sing = any(m.get("type") == "singer" for m in x.get("manoeuvres", []))
        return 0.0 if sing else _entry(x)
    ts: List[float] = []
    e_old, e_new = _entry(o), _entry(n)
    entry = min(e_old, e_new)
    keys = (set(o) | set(n)) - _FREE_BODY
    for k in keys:
        a, b = o.get(k), n.get(k)
        if _same(a, b):
            continue
        if k == "attitude":
            t = _waypoints_change(a or [], b or [], entry)
            if t is not None:
                ts.append(t)
        elif k == "manoeuvres":
            sa = any(m.get("type") == "singer" for m in a or [])
            sb = any(m.get("type") == "singer" for m in b or [])
            if sa != sb:
                return 0.0                                  # Singer's draws run from t = 0
            t = _list_change(a or [], b or [], _manoeuvre_time)
            if t is not None:
                ts.append(t)
        elif k == "t_appear_s":
            ts.append(min(float(a or 0.0), float(b or 0.0)))
        elif k == "orbit":
            return 0.0
        else:                                               # state, class, overrides, n, around, spreads, unknown
            ts.append(entry)
    return min(ts) if ts else None


def first_change(old: dict, new: dict) -> Optional[float]:
    """The first instant where the normalised scenarios old and new differ for the propagation; None: nowhere."""
    if _same(old, new):
        return None
    ts: List[float] = []
    for k in (set(old) | set(new)) - _FREE_TOP - {"meta", "ship", "objects", "groups"}:
        if not _same(old.get(k), new.get(k)):
            return 0.0                                      # central body, unknown keys
    mo, mn = old.get("meta", {}), new.get("meta", {})
    for k in (set(mo) | set(mn)) - _FREE_META:
        if _same(mo.get(k), mn.get(k)):
            continue
        if k == "duration_s":
            ts.append(min(float(mo[k]), float(mn[k])))
        else:
            return 0.0                                      # epoch, step, seed, unknown
    t = _body_change(old.get("ship"), new.get("ship"))
    if t is not None:
        ts.append(t)
    for key in ("objects", "groups"):
        a: Dict[str, dict] = {x["id"]: x for x in old.get(key, [])}
        b: Dict[str, dict] = {x["id"]: x for x in new.get(key, [])}
        for oid in set(a) | set(b):
            t = _body_change(a.get(oid), b.get(oid))
            if t is not None:
                ts.append(t)
    return min(ts) if ts else None


def _upstream(sc: dict) -> Dict[tuple, set]:
    """What each row's path ("p", id) and attitude ("a", id) read (user, 8 November 2026): a path its PN target's
    path, its group's reference's path (around, at the appearance), the ship's path and attitude for a relative state
    (at the appearance); an attitude its own path and the paths of the targets it aims at (an attitude moves no path
    but a relative state's). Group members by their row ids "<group>#<k>"."""
    def edges(rid: str, spec: dict, relative: bool, around: Optional[str] = None) -> Dict[tuple, set]:
        path = {("p", m.get("target", "ship")) for m in spec.get("manoeuvres", []) if m.get("type") == "pn"}
        if around is not None:
            path.add(("p", around))
        if relative:
            path |= {("p", "ship"), ("a", "ship")}
        aim = {("p", w["attitude"]["id"]) for w in spec.get("attitude", []) or []
               if isinstance(w.get("attitude"), dict) and w["attitude"].get("type") == "target"}
        return {("p", rid): path, ("a", rid): aim | {("p", rid)}}
    up = edges("ship", sc.get("ship", {}), False)
    for o in sc.get("objects", []):
        up.update(edges(o["id"], o, o.get("state", {}).get("type") == "relative"))
    for g in sc.get("groups", []):
        for k in range(int(g.get("n", 0))):
            up.update(edges(f"{g['id']}#{k}", g, False, g.get("around", "ship")))
    return up


def affected_rows(old: dict, new: dict) -> Optional[Tuple[set, set]]:
    """(the ids of the rows whose path the edit old -> new may change, those whose attitude it may change), the
    normalised scenarios old and new; the rows read by them followed until none is left (user, 8 November 2026: the
    paths of the others stay as they were, up to the rounding of the integration steps, split at the instants of
    every row's events). None: every row (the central body, the epoch, the step, the seed, the duration or an unknown
    key changed)."""
    if _same(old, new):
        return set(), set()
    for k in (set(old) | set(new)) - _FREE_TOP - {"meta", "ship", "objects", "groups"}:
        if not _same(old.get(k), new.get(k)):
            return None
    mo, mn = old.get("meta", {}), new.get("meta", {})
    if any(not _same(mo.get(k), mn.get(k)) for k in (set(mo) | set(mn)) - _FREE_META):
        return None
    seeds = set()
    if _body_change(old.get("ship"), new.get("ship")) is not None:
        seeds.add("ship")
    for key in ("objects", "groups"):
        a = {x["id"]: x for x in old.get(key, [])}
        b = {x["id"]: x for x in new.get(key, [])}
        for oid in set(a) | set(b):
            if _body_change(a.get(oid), b.get(oid)) is None:
                continue
            if key == "objects":
                seeds.add(oid)
            else:
                n = max(int((a.get(oid) or {}).get("n", 0)), int((b.get(oid) or {}).get("n", 0)))
                seeds |= {f"{oid}#{k}" for k in range(n)}
    hit = {(kind, rid) for rid in seeds for kind in ("p", "a")}      # (an edited body: its path and its attitude)
    edges = _upstream(old)
    for node, ups in _upstream(new).items():            # (the edges of both: a dependency removed counts too)
        edges[node] = edges.get(node, set()) | ups
    grown = True
    while grown:
        grown = False
        for node, ups in edges.items():
            if node not in hit and ups & hit:
                hit.add(node)
                grown = True
    return {rid for kind, rid in hit if kind == "p"}, {rid for kind, rid in hit if kind == "a"}


def resume(sim: prop.LiveSim, sc: dict) -> prop.LiveSim:
    """The run of the edited scenario sc: the same run when nothing physical changed (its scenario replaced), a new
    run from the start when the edit touches t = 0, else a new run taken up from sim's last checkpoint strictly before
    the first change (its record and checkpoints before it kept)."""
    t = first_change(sim.sc, sc)
    if t is None:
        if [w.id for w in sim.rows] == [w.id for w in prop.LiveSim.row_specs(sc)]:
            sim.sc = sc
            return sim
        new = prop.LiveSim(sc)                              # the same run, its rows in the scenario's new order
        new.restore(sim.checkpoint(), source=sim)
        new.checkpoints = [c for c in sim.checkpoints]
        return new
    new = prop.LiveSim(sc)
    if t <= 0.0:
        return new
    ck = sim.checkpoint_before(t - 1e-6)                    # strictly before: a checkpoint at t has applied t's events
    if ck["t"] <= 0.0:
        return new                                          # (t = 0 is the start of a new run)
    new.restore(ck, source=sim)
    return new
