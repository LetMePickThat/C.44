#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO -- the format of a global scenario (roadmap step 6.1, pass S1; framing A1-E3 validated by the user on
28 October 2026). A scenario is a JSON file, validated on load with clear messages, then normalised (defaults filled).
It describes the WORLD only (the radar is the mission file it cites); values of the model live in [sim].

  {"meta": {"name": str, "epoch_utc": ISO 8601, "duration_s": > 0, "step_s": > 0 (default sim.step_s),
            "seed": int (default 1)},
   "central_body": "earth" | "moon" (a body of [sim.bodies]; default "earth"),
   "mission": path of the mission-preparation file (optional, relative to the scenario),
   "ship": {"orbit": ORBIT, "attitude": ATTITUDE (default inertial identity)},
   "objects": [OBJECT, ...], "groups": [GROUP, ...]}

  ORBIT     {"type": "state", "r_km": [x, y, z], "v_kms": [x, y, z]}   (ICRF axes, from the body centre)
          | {"type": "elements", "a_km", "e", "i_deg", "raan_deg", "argp_deg", "nu_deg"}
  ATTITUDE  {"mode": "inertial", "q": [w, x, y, z]}                      q ship -> ICRF (Hamilton, scalar first, step 3.0)
          | {"mode": "lvlh", "q": [w, x, y, z]}                          q ship -> local orbital frame (rtn of step 3.0)
          | {"mode": "waypoints", "points": [{"t_s", "q"}, ...]}         joined within repointing.rate_max_dps and
                                                                         repointing.accel_max_dps2 (step 4.5)
          | {"mode": "point", "axis": [x, y, z] (ship axis), "target": object id | {"ra_deg", "dec_deg"}}
  OBJECT    {"id": str, "class": a class of the target library, "overrides": {class keys} (optional),
             "t_appear_s": >= 0 (default 0), "jammer": a jammer class (optional),
             "state": ORBIT | RELATIVE, "manoeuvres": [MANOEUVRE, ...] (optional)}
  RELATIVE  {"type": "relative", "range_km": > 0, "phi_deg", "eps_deg" (direction from the ship, in "frame": "ship"
             (default) or "inertial"), "v_close_mps": closing speed, "v_transverse_mps": [a, b] (optional, along
             the two axes normal to the line of sight; default 0)}            resolved at t_appear_s by the propagation
  MANOEUVRE {"type": "impulse", "t_s", "dv_mps": [x, y, z], "frame": "rtn" | "inertial"}
          | {"type": "burn", "t_s", "duration_s", "accel_mps2": [x, y, z], "frame": "rtn" | "inertial"}
          | {"type": "singer"}            random manoeuvre of the class (accel_max, maneuver_tau), seeded
          | {"type": "pn", "n": navigation constant (default sim.pn_n), "start_range_km" | "t_s"}
                                          terminal guidance toward the ship, bounded by the class's acceleration
  GROUP     {"id", "class", "n": >= 1, "around": an object id, "pos_sigma_m", "vel_sigma_mps", "t_appear_s",
             "overrides", "manoeuvres"}   n objects drawn around the reference object at t_appear_s (decoys, debris)
"""
from __future__ import annotations

import copy
import json
import math
import os
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from core import radar_model as rm
from core.params import P

FORMAT_VERSION = 1
_ATT_MODES = ("inertial", "lvlh", "waypoints", "point")
_MAN_TYPES = ("impulse", "burn", "singer", "pn")
_FRAMES = ("rtn", "inertial")


def bodies() -> Dict[str, dict]:
    """Central bodies of the simulator ([sim.bodies]: mu, J2, equatorial radius, mean radius); empty when the section
    is missing from the parameter file (validate() says so)."""
    try:
        return {k: dict(v) for k, v in P.section("sim.bodies").items()}
    except KeyError:
        return {}


def jammer_classes() -> List[str]:
    return list(dict.fromkeys(p.path.split(".")[1] for p in P if p.path.startswith("jammers.")))


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def _vec(x, n: int = 3) -> bool:
    return isinstance(x, (list, tuple)) and len(x) == n and all(_num(v) for v in x)


def _quat(x) -> bool:
    return _vec(x, 4) and sum(float(v) ** 2 for v in x) > 1e-12


def _check_orbit(o, tag: str, err: List[str], relative_ok: bool) -> None:
    if not isinstance(o, dict) or "type" not in o:
        err.append(f"{tag}: a dict with a 'type' is needed")
        return
    t = o["type"]
    if t == "state":
        if not _vec(o.get("r_km")) or not _vec(o.get("v_kms")):
            err.append(f"{tag}: 'state' needs r_km and v_kms, 3 numbers each (km, km/s, ICRF axes)")
    elif t == "elements":
        miss = [k for k in ("a_km", "e", "i_deg", "raan_deg", "argp_deg", "nu_deg") if not _num(o.get(k))]
        if miss:
            err.append(f"{tag}: 'elements' needs numbers for {', '.join(miss)}")
        elif not (0.0 <= float(o["e"]) < 1.0) or float(o["a_km"]) <= 0.0:
            err.append(f"{tag}: elements need 0 <= e < 1 and a_km > 0 (closed orbits only)")
    elif t == "relative" and relative_ok:
        for k in ("range_km", "phi_deg", "eps_deg", "v_close_mps"):
            if not _num(o.get(k)):
                err.append(f"{tag}: 'relative' needs a number for {k}")
        if _num(o.get("range_km")) and float(o["range_km"]) <= 0.0:
            err.append(f"{tag}: range_km must be > 0")
        if o.get("frame", "ship") not in ("ship", "inertial"):
            err.append(f"{tag}: frame must be 'ship' or 'inertial'")
        if "v_transverse_mps" in o and not _vec(o["v_transverse_mps"], 2):
            err.append(f"{tag}: v_transverse_mps needs 2 numbers")
    else:
        allowed = "state, elements" + (", relative" if relative_ok else "")
        err.append(f"{tag}: type '{t}' unknown ({allowed})")


def _check_manoeuvres(ms, tag: str, err: List[str], duration: float) -> None:
    if not isinstance(ms, list):
        err.append(f"{tag}: manoeuvres must be a list")
        return
    for i, m in enumerate(ms):
        tg = f"{tag}.manoeuvres[{i}]"
        t = m.get("type") if isinstance(m, dict) else None
        if t not in _MAN_TYPES:
            err.append(f"{tg}: type must be one of {', '.join(_MAN_TYPES)}")
            continue
        if t in ("impulse", "burn"):
            if not _num(m.get("t_s")) or not (0.0 <= float(m["t_s"]) <= duration):
                err.append(f"{tg}: t_s must lie in [0, duration]")
            if m.get("frame", "rtn") not in _FRAMES:
                err.append(f"{tg}: frame must be 'rtn' or 'inertial'")
        if t == "impulse" and not _vec(m.get("dv_mps")):
            err.append(f"{tg}: dv_mps needs 3 numbers")
        if t == "burn":
            if not _num(m.get("duration_s")) or float(m["duration_s"]) <= 0.0:
                err.append(f"{tg}: duration_s must be > 0")
            if not _vec(m.get("accel_mps2")):
                err.append(f"{tg}: accel_mps2 needs 3 numbers")
        if t == "pn":
            if "n" in m and (not _num(m["n"]) or float(m["n"]) <= 0.0):
                err.append(f"{tg}: n must be > 0")
            if ("start_range_km" in m) == ("t_s" in m):
                err.append(f"{tg}: give either start_range_km or t_s")


def validate(sc: dict) -> List[str]:
    """Every problem of a scenario, as clear messages (empty list = valid)."""
    err: List[str] = []
    if not isinstance(sc, dict):
        return ["the scenario must be a JSON object"]
    meta = sc.get("meta")
    duration = 0.0
    if not isinstance(meta, dict):
        err.append("meta: a dict is needed (name, epoch_utc, duration_s)")
    else:
        try:
            datetime.fromisoformat(str(meta.get("epoch_utc", "")).replace("Z", "+00:00"))
        except ValueError:
            err.append("meta.epoch_utc: an ISO 8601 date is needed, e.g. 2050-01-01T00:00:00Z")
        if not _num(meta.get("duration_s")) or float(meta["duration_s"]) <= 0.0:
            err.append("meta.duration_s must be > 0")
        else:
            duration = float(meta["duration_s"])
        if "step_s" in meta and (not _num(meta["step_s"]) or float(meta["step_s"]) <= 0.0):
            err.append("meta.step_s must be > 0")
        if "seed" in meta and not isinstance(meta["seed"], int):
            err.append("meta.seed must be an integer")
    body = sc.get("central_body", "earth")
    if not bodies():
        err.append(f"[sim.bodies] missing from {P.path}: the central bodies (mu, J2, radii) are needed")
    elif body not in bodies():
        err.append(f"central_body '{body}' unknown ({', '.join(bodies())}, from [sim.bodies])")
    ship = sc.get("ship")
    if not isinstance(ship, dict):
        err.append("ship: a dict with an orbit is needed")
    else:
        _check_orbit(ship.get("orbit"), "ship.orbit", err, relative_ok=False)
        att = ship.get("attitude", {"mode": "inertial"})
        mode = att.get("mode") if isinstance(att, dict) else None
        if mode not in _ATT_MODES:
            err.append(f"ship.attitude.mode must be one of {', '.join(_ATT_MODES)}")
        elif mode in ("inertial", "lvlh") and "q" in att and not _quat(att["q"]):
            err.append(f"ship.attitude.q needs 4 numbers, not all zero")
        elif mode == "waypoints":
            pts = att.get("points")
            if not isinstance(pts, list) or len(pts) < 1:
                err.append("ship.attitude.points: at least one {t_s, q}")
            else:
                ts = [p.get("t_s") for p in pts if isinstance(p, dict)]
                if len(ts) != len(pts) or not all(_num(x) for x in ts) or any(b <= a for a, b in zip(ts, ts[1:])):
                    err.append("ship.attitude.points: t_s numbers, strictly increasing")
                if not all(isinstance(p, dict) and _quat(p.get("q")) for p in pts):
                    err.append("ship.attitude.points: each q needs 4 numbers")
        elif mode == "point":
            if not _vec(att.get("axis")):
                err.append("ship.attitude.axis: 3 numbers (the ship axis that points)")
            tg = att.get("target")
            if not (isinstance(tg, str) or (isinstance(tg, dict) and _num(tg.get("ra_deg")) and _num(tg.get("dec_deg")))):
                err.append("ship.attitude.target: an object id or {ra_deg, dec_deg}")
    ids = set()
    classes = set(rm.TARGETS)
    jams = set(jammer_classes())
    for i, o in enumerate(sc.get("objects", [])):
        tag = f"objects[{i}]" + (f" ({o.get('id')})" if isinstance(o, dict) and o.get("id") else "")
        if not isinstance(o, dict):
            err.append(f"{tag}: a dict is needed")
            continue
        oid = o.get("id")
        if not isinstance(oid, str) or not oid:
            err.append(f"{tag}: an id (text) is needed")
        elif oid in ids:
            err.append(f"{tag}: id '{oid}' used twice")
        ids.add(oid)
        if o.get("class") not in classes:
            err.append(f"{tag}: class must be one of {', '.join(sorted(classes))}")
        if "overrides" in o:
            bad = [k for k in o["overrides"] if k not in rm.SECTOR_TARGET_KEYS + ("v_close_max_mps",)]
            if bad:
                err.append(f"{tag}: overrides not known: {', '.join(bad)}")
        if "jammer" in o and o["jammer"] not in jams:
            err.append(f"{tag}: jammer must be one of {', '.join(sorted(jams))}")
        if not _num(o.get("t_appear_s", 0.0)) or not (0.0 <= float(o.get("t_appear_s", 0.0)) <= max(duration, 0.0)):
            err.append(f"{tag}: t_appear_s must lie in [0, duration]")
        _check_orbit(o.get("state"), f"{tag}.state", err, relative_ok=True)
        _check_manoeuvres(o.get("manoeuvres", []), tag, err, duration)
    for i, g in enumerate(sc.get("groups", [])):
        tag = f"groups[{i}]" + (f" ({g.get('id')})" if isinstance(g, dict) and g.get("id") else "")
        if not isinstance(g, dict):
            err.append(f"{tag}: a dict is needed")
            continue
        if not isinstance(g.get("id"), str) or g["id"] in ids:
            err.append(f"{tag}: a unique id is needed")
        ids.add(g.get("id"))
        if g.get("class") not in classes:
            err.append(f"{tag}: class must be one of {', '.join(sorted(classes))}")
        if not isinstance(g.get("n"), int) or g["n"] < 1:
            err.append(f"{tag}: n must be an integer >= 1")
        if g.get("around") not in {o.get("id") for o in sc.get("objects", []) if isinstance(o, dict)}:
            err.append(f"{tag}: around must name an object of 'objects'")
        for k in ("pos_sigma_m", "vel_sigma_mps"):
            if not _num(g.get(k, 0.0)) or float(g.get(k, 0.0)) < 0.0:
                err.append(f"{tag}: {k} must be >= 0")
        _check_manoeuvres(g.get("manoeuvres", []), tag, err, duration)
    if isinstance(ship, dict) and isinstance(ship.get("attitude"), dict) and ship["attitude"].get("mode") == "point":
        tg = ship["attitude"].get("target")
        if isinstance(tg, str) and tg not in ids:
            err.append(f"ship.attitude.target: object '{tg}' not found")
    return err


def normalise(sc: dict) -> dict:
    """A copy with the defaults filled (step, seed, body, attitude, appearance times, frames, PN constant)."""
    s = copy.deepcopy(sc)
    s["format_version"] = FORMAT_VERSION
    m = s["meta"]
    m.setdefault("name", "scenario")
    m.setdefault("step_s", float(P.get("sim.step_s")))
    m.setdefault("seed", 1)
    s.setdefault("central_body", "earth")
    s["ship"].setdefault("attitude", {"mode": "inertial", "q": [1.0, 0.0, 0.0, 0.0]})
    if s["ship"]["attitude"]["mode"] in ("inertial", "lvlh"):
        s["ship"]["attitude"].setdefault("q", [1.0, 0.0, 0.0, 0.0])
    for o in s.setdefault("objects", []) + s.setdefault("groups", []):
        o.setdefault("t_appear_s", 0.0)
        o.setdefault("overrides", {})
        o.setdefault("manoeuvres", [])
        for mn in o["manoeuvres"]:
            if mn["type"] in ("impulse", "burn"):
                mn.setdefault("frame", "rtn")
            if mn["type"] == "pn":
                mn.setdefault("n", float(P.get("sim.pn_n")))
        st = o.get("state")
        if isinstance(st, dict) and st.get("type") == "relative":
            st.setdefault("frame", "ship")
            st.setdefault("v_transverse_mps", [0.0, 0.0])
    for g in s["groups"]:
        g.setdefault("pos_sigma_m", 0.0)
        g.setdefault("vel_sigma_mps", 0.0)
    return s


class ScenarioError(ValueError):
    pass


def load(path: str) -> dict:
    """Read, validate and normalise a scenario; ScenarioError lists every problem. The mission path is made absolute."""
    try:
        with open(path, encoding="utf-8") as fh:
            sc = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise ScenarioError(f"{path}: {exc}") from None
    err = validate(sc)
    if err:
        raise ScenarioError(f"{path}:\n  " + "\n  ".join(err))
    s = normalise(sc)
    if s.get("mission"):
        s["mission"] = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(path)), s["mission"]))
    return s


def save(sc: dict, path: str) -> None:
    """Write a scenario (validated first)."""
    err = validate(sc)
    if err:
        raise ScenarioError("\n  ".join(err))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(sc, fh, indent=1, ensure_ascii=False)


def epoch(sc: dict) -> datetime:
    return datetime.fromisoformat(sc["meta"]["epoch_utc"].replace("Z", "+00:00"))


def object_ids(sc: dict) -> Tuple[str, ...]:
    return tuple(o["id"] for o in sc.get("objects", [])) + tuple(g["id"] for g in sc.get("groups", []))
