#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCENARIO -- the format of a global scenario (roadmap step 6.1, pass S1; framing A1-E3 validated by the user on
28 October 2026). A scenario is a JSON file, validated on load with clear messages, then normalised (defaults filled).
It describes the WORLD only (the radar is the mission file it cites); values of the model live in [sim].

  {"meta": {"name": str, "epoch_utc": ISO 8601, "duration_s": > 0, "step_s": > 0 (default sim.step_s),
            "seed": int (default 1), "nav_errors": bool (default false: the navigation is the truth),
            "log_nav": bool (default false: the 100 Hz navigation is not stored in the journal),
            "sun_type": a sun type of [sky.tb_k] (default "sun"), "earth_type": an earth type (default "earth")},
   "central_body": any body of bodies(): "earth" (default), "moon", "sun", a planet, a major moon,
   "mission": path of the mission-preparation file (optional, relative to the scenario),
   "ship": {"name": the ship's name in the views (default "OWNSHIP"; no effect on the physics), "orbit": ORBIT,
            "attitude": [WAYPOINT, ...] (sim/attitude.py; default prograde, roll 0)},
   "objects": [OBJECT, ...], "groups": [GROUP, ...]}

  ORBIT     {"type": "state", "r_km": [x, y, z], "v_kms": [x, y, z]}   (ICRF axes, from the body centre)
          | {"type": "elements", "a_km", "e", "i_deg", "raan_deg", "argp_deg", "nu_deg"}
  WAYPOINT  {"t_s", "rate_dps", "attitude": fixed in the orbital or inertial frame | prograde | retrograde | normal+ |
             normal- | radial_out | radial_in | toward a target}            (format and axes: sim/attitude.py, pass S3a)
  OBJECT    {"id": str, "class": a class of the target library, "identity": one of IDENTITIES (default "pending"),
             "overrides": {class keys} (optional),
             "attitude": [WAYPOINT, ...] (optional, classes SHIP and KKV; else the class's default attitude),
             "t_appear_s": >= 0 (default 0), "jammer": a jammer class (optional),
             "rcs_offset_db": the object's RCS against its class mean (optional; else drawn once from the class
             spread, sim/geometry.py G6),
             "state": ORBIT | RELATIVE, "manoeuvres": [MANOEUVRE, ...] (optional)}
  (the ship also takes "manoeuvres": [MANOEUVRE, ...] (optional, user 29 October 2026): its pn needs a target
   other than itself; pn and singer bounded by sim.ship_manoeuvre_class, Singer time sim.ship_maneuver_tau_s)
  RELATIVE  {"type": "relative", "range_km": > 0, "phi_deg", "eps_deg" (direction from the ship, in "frame": "ship"
             (default) or "inertial"), "v_close_mps": closing speed, "v_transverse_mps": [a, b] (optional, along
             the two axes normal to the line of sight; default 0)}            resolved at t_appear_s by the propagation
  MANOEUVRE {"type": "impulse", "t_s", "dv_mps": [x, y, z], "frame": "rtn" (default) | "pnr" | "inertial"}
          | {"type": "burn", "t_s", "duration_s", "accel_mps2": [x, y, z], "frame": "rtn" | "pnr" | "inertial"}
            (pnr: prograde, normal, radial, the manoeuvre node's axes, sim/propagate.py pnr_matrix)
          | {"type": "singer"}            random manoeuvre of the class (accel_max, maneuver_tau), seeded
          | {"type": "pn", "n": navigation constant (default sim.pn_n), "start_range_km" | "t_s",
             "target": "ship" (default) | an object id | a group member "<group>#<k>"}
                                          terminal guidance toward the target, bounded by the class's acceleration
  GROUP     {"id", "class", "identity" (its members'), "n": >= 1, "around": an object id or "ship", "pos_sigma_m",
             "vel_sigma_mps", "t_appear_s",
             "overrides", "manoeuvres", "rcs_offset_db"}   n objects drawn around the reference object at t_appear_s (decoys, debris)
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

from . import attitude as att

FORMAT_VERSION = 1
# the identities an object can be given (user, 29 October 2026; default "pending"). For later (radar in the loop,
# step 6.4): a "friendly" object is assumed cooperative and shared on the tactical data links, so OWNSHIP knows its
# position and kinematics; the others (pending, unknown, neutral, suspect, hostile) are not cooperative, share
# nothing and are known to OWNSHIP only through its own sensors (detection, tracking, classification)
IDENTITIES = ("pending", "unknown", "neutral", "friendly", "suspect", "hostile")
COOPERATIVE = ("friendly",)                       # shared on the tactical data links (step 6.4)
DEFAULT_IDENTITY = "pending"
DEFAULT_SHIP_NAME = "OWNSHIP"                     # the ship's name when the file gives none (user: renamable)
_MAN_TYPES = ("impulse", "burn", "singer", "pn")
_FRAMES = ("rtn", "pnr", "inertial")


_BODIES: Optional[Dict[str, dict]] = None


def bodies() -> Dict[str, dict]:
    """The bodies a scenario can have as its central body (user, 30 October 2026: any celestial body): the Earth and
    the Moon ([sim.bodies]), the Sun (sim.sun_*, its IAU pole), the planets ([sim.planets]) and the major moons
    ([sim.moons]: J2 0, their pole their orbit's normal in 2050, the moons being locked on their planets); each mu,
    J2, equatorial and mean radii, pole. Empty when [sim.bodies] is missing from the parameter file (validate()
    says so)."""
    global _BODIES
    if _BODIES is not None:
        return _BODIES
    try:
        out = {k: dict(v) for k, v in P.section("sim.bodies").items()}
    except KeyError:
        return {}
    from sim import ephemeris as eph
    rot = dict(P.get("sim.sun_rotation"))
    out["sun"] = {"mu_km3_s2": float(P.get("sim.sun_mu_km3_s2")), "j2": float(P.get("sim.sun_j2")),
                  "r_eq_km": float(P.get("sim.sun_radius_km")), "radius_km": float(P.get("sim.sun_radius_km")),
                  "pole_ra_deg": float(rot["pole_ra_deg"]), "pole_dec_deg": float(rot["pole_dec_deg"])}
    for k, v in P.section("sim.planets").items():
        v = dict(v)
        out[k] = {key: float(v[key]) for key in ("mu_km3_s2", "j2", "r_eq_km", "radius_km", "pole_ra_deg",
                                                  "pole_dec_deg")}
    T2050 = 0.5
    for k, v in eph.moons().items():
        pole = eph.orbit_pole(k, T2050)
        out[k] = {"mu_km3_s2": float(v["mu_km3_s2"]), "j2": 0.0, "r_eq_km": float(v["radius_km"]),
                  "radius_km": float(v["radius_km"]),
                  "pole_ra_deg": math.degrees(math.atan2(pole[1], pole[0])) % 360.0,
                  "pole_dec_deg": math.degrees(math.asin(max(-1.0, min(1.0, float(pole[2])))))}
    _BODIES = out
    return out


def sky_types(body: str) -> List[str]:
    """Body types of [sky.tb_k] for a body ("sun" -> sun, sun_quiet, sun_active)."""
    return [p.path.split(".")[2] for p in P if p.path.startswith("sky.tb_k.")
            and p.path.split(".")[2].split("_")[0] == body]


def jammer_classes() -> List[str]:
    return list(dict.fromkeys(p.path.split(".")[1] for p in P if p.path.startswith("jammers.")))


def _num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def _vec(x, n: int = 3) -> bool:
    return isinstance(x, (list, tuple)) and len(x) == n and all(_num(v) for v in x)


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
                err.append(f"{tg}: frame must be 'rtn', 'pnr' or 'inertial'")
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
        for k in ("nav_errors", "log_nav"):
            if k in meta and not isinstance(meta[k], bool):
                err.append(f"meta.{k} must be true or false")
        for k, b in (("sun_type", "sun"), ("earth_type", "earth")):
            if k in meta and meta[k] not in sky_types(b):
                err.append(f"meta.{k} must be one of {', '.join(sky_types(b))} ([sky.tb_k])")
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
        _check_manoeuvres(ship.get("manoeuvres", []), "ship", err, duration)
        name = ship.get("name", DEFAULT_SHIP_NAME)
        if not isinstance(name, str) or not name.strip():
            err.append("ship: its name must be a non-empty text")
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
        if isinstance(ship, dict) and oid == ship_name(sc):
            err.append(f"{tag}: id '{oid}' is the ship's name")
        if o.get("class") not in classes:
            err.append(f"{tag}: class must be one of {', '.join(sorted(classes))}")
        if o.get("identity", DEFAULT_IDENTITY) not in IDENTITIES:
            err.append(f"{tag}: identity must be one of {', '.join(IDENTITIES)}")
        if "overrides" in o:
            bad = [k for k in o["overrides"] if k not in rm.SECTOR_TARGET_KEYS + ("v_close_max_mps",)]
            if bad:
                err.append(f"{tag}: overrides not known: {', '.join(bad)}")
        if "rcs_offset_db" in o and not _num(o["rcs_offset_db"]):
            err.append(f"{tag}: rcs_offset_db must be a number (dB)")
        if "jammer" in o and o["jammer"] not in jams:
            err.append(f"{tag}: jammer must be one of {', '.join(sorted(jams))}")
        if not _num(o.get("t_appear_s", 0.0)) or not (0.0 <= float(o.get("t_appear_s", 0.0)) <= max(duration, 0.0)):
            err.append(f"{tag}: t_appear_s must lie in [0, duration]")
        _check_orbit(o.get("state"), f"{tag}.state", err, relative_ok=True)
        _check_manoeuvres(o.get("manoeuvres", []), tag, err, duration)
        if "attitude" in o and o.get("class") not in att.FLYING_CLASSES:
            err.append(f"{tag}: only the classes {', '.join(att.FLYING_CLASSES)} fly waypoints (attitude)")
    for i, g in enumerate(sc.get("groups", [])):
        tag = f"groups[{i}]" + (f" ({g.get('id')})" if isinstance(g, dict) and g.get("id") else "")
        if not isinstance(g, dict):
            err.append(f"{tag}: a dict is needed")
            continue
        if not isinstance(g.get("id"), str) or g["id"] in ids or (isinstance(ship, dict) and g.get("id") == ship_name(sc)):
            err.append(f"{tag}: a unique id is needed (not the ship's name either)")
        ids.add(g.get("id"))
        if g.get("class") not in classes:
            err.append(f"{tag}: class must be one of {', '.join(sorted(classes))}")
        if g.get("identity", DEFAULT_IDENTITY) not in IDENTITIES:
            err.append(f"{tag}: identity must be one of {', '.join(IDENTITIES)}")
        if not isinstance(g.get("n"), int) or g["n"] < 1:
            err.append(f"{tag}: n must be an integer >= 1")
        if g.get("around") not in {o.get("id") for o in sc.get("objects", []) if isinstance(o, dict)} | {"ship"}:
            err.append(f"{tag}: around must name an object of 'objects' or 'ship'")
        if "rcs_offset_db" in g and not _num(g["rcs_offset_db"]):
            err.append(f"{tag}: rcs_offset_db must be a number (dB)")
        for k in ("pos_sigma_m", "vel_sigma_mps"):
            if not _num(g.get(k, 0.0)) or float(g.get(k, 0.0)) < 0.0:
                err.append(f"{tag}: {k} must be >= 0")
        _check_manoeuvres(g.get("manoeuvres", []), tag, err, duration)
    # the guidance targets, once every id is known: the ship, an object or a group member, not the row itself
    members = {f"{g['id']}#{k}" for g in sc.get("groups", []) if isinstance(g, dict) and isinstance(g.get("id"), str)
               and isinstance(g.get("n"), int) for k in range(g["n"])}
    targets = {"ship"} | {o.get("id") for o in sc.get("objects", []) if isinstance(o, dict)} | members
    if isinstance(ship, dict):
        for mn in ship.get("manoeuvres", []):
            if isinstance(mn, dict) and mn.get("type") == "pn" and mn.get("target", "ship") not in targets - {"ship"}:
                err.append("ship: pn target must be an object or a group member '<group>#<k>' (not the ship)")
    for kind in ("objects", "groups"):
        for i, o in enumerate(sc.get(kind, [])):
            if not isinstance(o, dict):
                continue
            for mn in o.get("manoeuvres", []):
                if isinstance(mn, dict) and mn.get("type") == "pn" and "target" in mn:
                    tgt = mn["target"]
                    own = {o.get("id")} if kind == "objects" else {f"{o.get('id')}#{k}" for k in range(o.get("n", 0))
                                                                  if isinstance(o.get("n"), int)}
                    if tgt not in targets or tgt in own:
                        err.append(f"{kind}[{i}] ({o.get('id')}): pn target must be 'ship', an object or a group "
                                   f"member '<group>#<k>' other than itself")
    # waypoint attitudes, once every id is known (a target must exist at the waypoint's time)
    appear = {o["id"]: float(o.get("t_appear_s", 0.0)) for o in sc.get("objects", [])
              if isinstance(o, dict) and isinstance(o.get("id"), str) and _num(o.get("t_appear_s", 0.0))}
    if isinstance(ship, dict) and "attitude" in ship:
        att.validate(ship["attitude"], "ship.attitude", err, duration, appear, "ship")
    for i, o in enumerate(sc.get("objects", [])):
        if isinstance(o, dict) and "attitude" in o and o.get("class") in att.FLYING_CLASSES:
            att.validate(o["attitude"], f"objects[{i}] ({o.get('id')}).attitude", err, duration,
                         dict(appear, ship=0.0), o.get("id"))
    return err


def normalise(sc: dict) -> dict:
    """A copy with the defaults filled (step, seed, body, attitude, appearance times, frames, PN constant)."""
    s = copy.deepcopy(sc)
    s["format_version"] = FORMAT_VERSION
    m = s["meta"]
    m.setdefault("name", "scenario")
    m.setdefault("step_s", float(P.get("sim.step_s")))
    m.setdefault("seed", 1)
    m.setdefault("nav_errors", False)
    m.setdefault("log_nav", False)
    m.setdefault("sun_type", "sun")
    m.setdefault("earth_type", "earth")
    s.setdefault("central_body", "earth")
    s["ship"].setdefault("name", DEFAULT_SHIP_NAME)
    s["ship"].setdefault("attitude", copy.deepcopy(att.DEFAULT_SHIP))
    att.normalise(s["ship"]["attitude"])
    s["ship"].setdefault("manoeuvres", [])
    for o in [s["ship"]] + s.setdefault("objects", []) + s.setdefault("groups", []):
        if o is not s["ship"]:
            o.setdefault("t_appear_s", 0.0)
            o.setdefault("identity", DEFAULT_IDENTITY)
            o.setdefault("overrides", {})
            o.setdefault("manoeuvres", [])
            if "attitude" in o:
                att.normalise(o["attitude"])
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


def ship_name(sc: dict) -> str:
    """The ship's name in the views (DEFAULT_SHIP_NAME when not given)."""
    return (sc.get("ship") or {}).get("name") or DEFAULT_SHIP_NAME


def identity_of(sc: dict, oid: str, group: Optional[str] = None) -> str:
    """The identity of an object of the run (a group's member: its group's), the default when not given."""
    for o in sc.get("groups", []) if group else sc.get("objects", []):
        if o.get("id") == (group or oid):
            return o.get("identity", DEFAULT_IDENTITY)
    return DEFAULT_IDENTITY


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
