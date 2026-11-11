#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TIME PROFILE OF A MISSION ALONG ITS FLIGHT PLAN -- option A, pass T1 (framed with the user on 10 November 2026,
decisions P1 to P5; Choix_architecturaux, last entry of 10 November 2026).

The mission preparation judges a radar configuration along the flight plan of its linked scenario, each instant with
its own optimal plan (no worst-case aggregation: user), by the availability of each watched volume:
  P1 sampling: the instants of the scenario's journal kept at each attitude waypoint, then as soon as the attitude has
     turned by SAMPLE_TURN_DEG, or the central body seen from the ship has moved (direction or apparent radius) by
     SAMPLE_TURN_DEG (the ground moves with the orbit even without a manoeuvre), and at least every SAMPLE_MAX_S;
  P2 each volume at each instant, from the resources (the verdict's states): held (rho = 1, revisit held), degraded
     (rho_min <= rho < 1, revisit held), lost (revisit given way, or no face scans any of it); the coverage hole is a
     measure of its own (user, 10 November 2026, after the first profile: a hole of 0.6 % made a volume as lost as one
     entirely out of coverage): the share of the volume the faces scan at that attitude;
  P3 availability displayed: the shares of the time held / degraded / lost (each instant holding until the middle of
     the next interval), the longest outage (lost) and its ratio to the revisit asked, the covered share (mean over
     the time, and its minimum);
  P4 the instants where the fast estimate flags a loss (a state worse than at the previous instant, or rho down by more
     than FLAG_DROP_DB in energy) are flagged for the exact computation;
  P5 at most FLAG_MAX of them (the strongest losses first).
Each instant is evaluated by the fast estimate (core/cost_estimate.live_allocation: the final allocation rule over the
volumes' cost profiles, recalibrated by the last exact runs), the volumes resolved with the scenario's navigation and
sky at that instant (as the mission preparation's playback). The exact runs at the flagged instants are pass T3.
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import Callable, Dict, List, Optional

import numpy as np

from . import calibration
from . import cost_estimate as ce
from . import frames
from . import geometry as geo
from . import optimizer as mo
from . import radar_model as rm
from .allocation import RHO_MIN
from .params import P

SAMPLE_TURN_DEG = float(P.get("tool.profile.sample_turn_deg"))     # P1 (user, 10 November 2026)
SAMPLE_MAX_S = float(P.get("tool.profile.sample_max_s"))           # P1
FLAG_DROP_DB = float(P.get("tool.profile.flag_drop_db"))           # P4
FLAG_MAX = int(P.get("tool.profile.flag_max"))                     # P5
if SAMPLE_TURN_DEG <= 0 or SAMPLE_MAX_S <= 0 or FLAG_DROP_DB < 0 or FLAG_MAX < 0:
    raise ValueError(f"{P.path}: [tool.profile] sample_turn_deg, sample_max_s > 0, flag_drop_db, flag_max >= 0")
STATES = ("held", "degraded", "lost")
_RANK = {s: i for i, s in enumerate(STATES)}
_GRID = None


def _grid() -> geo.SphereGrid:
    global _GRID
    if _GRID is None:
        _GRID = geo.SphereGrid(float(P.get("tool.display.grid_res_deg")))
    return _GRID


def key(mission: dict, link) -> str:
    """Fingerprint of what a profile depends on: the mission, the scenario's journal, the parameter file and the model
    version (a profile kept beside the mission is reused only when it matches)."""
    h = hashlib.sha1(json.dumps(mission, sort_keys=True, default=str).encode("utf-8"))
    h.update(json.dumps(link.sc, sort_keys=True, default=str).encode("utf-8"))
    h.update(np.asarray(link.truth.t_s, float).tobytes())
    h.update(calibration._param_stamp().encode("utf-8"))
    return h.hexdigest()


def _waypoint_times(sc: dict) -> List[float]:
    return sorted(float(w.get("t_s", 0.0)) for w in (sc.get("ship", {}).get("attitude") or []) if isinstance(w, dict))


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(a @ b, -1.0, 1.0))))


def sample_instants(link) -> List[float]:
    """P1: the instants of the journal kept (module docstring)."""
    from sim import engine
    ts = [float(t) for t in link.truth.t_s]
    if not ts:
        return []
    nav = engine.Navigation(link.sc, link.truth)
    radius = float(link.truth.body.radius_km) if hasattr(link.truth.body, "radius_km") else None
    wps = _waypoint_times(link.sc)

    def view(t: float):
        m, _w, r, _v = nav.state(t)
        d = float(np.linalg.norm(r))
        u = m.T @ (-np.asarray(r, float) / d) if d > 0 else np.array([1.0, 0.0, 0.0])
        rad = math.degrees(math.asin(min(1.0, radius / d))) if radius and d > 0 else 0.0
        return m, u, rad

    kept = [ts[0]]
    m0, u0, r0 = view(ts[0])
    wp = [w for w in wps if w > ts[0]]
    for t in ts[1:]:
        m, u, rad = view(t)
        turn = math.degrees(float(np.linalg.norm(_rotvec(m0.T @ m))))
        due = (t - kept[-1] >= SAMPLE_MAX_S - 1e-9 or turn >= SAMPLE_TURN_DEG
               or _angle_deg(u0, u) >= SAMPLE_TURN_DEG or abs(rad - r0) >= SAMPLE_TURN_DEG)
        while wp and wp[0] <= t + 1e-9:                     # an attitude waypoint: the first instant at or after it
            wp.pop(0)
            due = True
        if due:
            kept.append(t)
            m0, u0, r0 = m, u, rad
    if kept[-1] != ts[-1]:
        kept.append(ts[-1])
    return kept


def _rotvec(m: np.ndarray) -> np.ndarray:
    """The rotation vector (rad) of a rotation matrix (its angle is the norm)."""
    c = float(np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0))
    ang = math.acos(c)
    if ang < 1e-12:
        return np.zeros(3)
    v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    n = float(np.linalg.norm(v))
    return v / n * ang if n > 1e-12 else np.array([ang, 0.0, 0.0])


def evaluate_instant(mission: dict, link, t: float, table, ids: List[str], calib: dict) -> dict:
    """The fast estimate at t: per volume its state (P2), rho, revisit tenable and coverage hole; per face its loads."""
    nav, sky, _alerts = link.nav_sky(t)
    g = dict(mission.get("global", {}), nav=[nav], sky=sky)
    eps_min, eps_max = float(g.get("eps_min_deg", -90.0)), float(g.get("eps_max_deg", 90.0))
    grid = _grid()
    ships, holes = [], {}
    for sid, sec in zip(ids, mission.get("sectors", [])):
        if frames.validate_sector(sec, g, ""):
            holes[sid] = 1.0
            continue
        try:
            ship = frames.resolve_sector(sec, g)
        except (KeyError, TypeError, ValueError):
            holes[sid] = 1.0
            continue
        split = mo.split_by_face(grid, ship, eps_min, eps_max)
        holes[sid] = float(split.get("hole", 0.0))
        ships.append({**ship, "id": sid})
    alloc = ce.live_allocation(table, ships, grid, eps_min, eps_max, sky, g, calib=calib, profiles="wait",
                               relief="wait")
    vols = {}
    for sid, sec in zip(ids, mission.get("sectors", [])):
        rec = alloc.get("by_sector", {}).get(sid)
        rr_asked = float(sec.get("rr_target_s", 0.0) or 0.0)
        hole = holes.get(sid, 1.0)
        if rec is None:
            vols[sid] = {"state": "lost", "rho": 0.0, "rr_s": None, "hole": hole}
            continue
        rho, rr = float(rec["rho"]), float(rec["rr_tenable_s"])
        if hole >= 1.0 - 1e-9 or (rr_asked > 0 and rr > rr_asked * (1.0 + 1e-9)) or rho < RHO_MIN * (1.0 - 1e-9):
            state = "lost"
        elif rho < 1.0 - 1e-6:
            state = "degraded"
        else:
            state = "held"
        vols[sid] = {"state": state, "rho": rho, "rr_s": rr, "hole": hole}
    faces = {str(k): {"energy": float(f["energy load"]), "S time": float(f["S time load"]),
                      "Ku time": float(f["Ku time load"])} for k, f in alloc.get("faces", {}).items()}
    return {"volumes": vols, "faces": faces, "ceiling": float(alloc.get("ceiling", 0.0))}


def _durations(ts: List[float], dur: float) -> np.ndarray:
    edges = [ts[0]] + [0.5 * (a + b) for a, b in zip(ts, ts[1:])] + [dur]
    return np.diff(np.asarray(edges, float))


def availability(ts: List[float], dur: float, states: List[str], rr_asked: float,
                 holes: Optional[List[float]] = None) -> dict:
    """P3: shares of the time held / degraded / lost, the longest outage and its ratio to the revisit asked, the
    covered share (1 - hole) as a time mean and its minimum."""
    w = _durations(ts, dur)
    tot = float(w.sum()) or 1.0
    cov = np.ones(len(states)) if holes is None else 1.0 - np.clip(np.asarray(holes, float), 0.0, 1.0)
    out = {s: float(w[[i for i, x in enumerate(states) if x == s]].sum()) / tot for s in STATES}
    longest, run = 0.0, 0.0
    for x, d in zip(states, w):
        run = run + float(d) if x == "lost" else 0.0
        longest = max(longest, run)
    out["longest_outage_s"] = longest
    out["outage_over_rr"] = longest / rr_asked if rr_asked > 0 else 0.0
    out["covered_mean"] = float((cov * w).sum()) / tot
    out["covered_min"] = float(cov.min()) if len(cov) else 1.0
    return out


def events(ts: List[float], vols: Dict[str, dict]) -> List[dict]:
    """State changes per volume, and P4 / P5: the instants flagged for the exact computation."""
    ev, cand = [], []
    for sid, v in vols.items():
        st, rho = v["state"], v["rho"]
        for i in range(1, len(ts)):
            if st[i] != st[i - 1]:
                ev.append({"t_s": ts[i], "object": sid, "what": f"{st[i - 1]} -> {st[i]}", "flagged": False})
            worse = _RANK[st[i]] > _RANK[st[i - 1]]
            drop = (40.0 * math.log10(rho[i - 1] / rho[i]) if rho[i] > 0 and rho[i - 1] > 0
                    else (math.inf if rho[i - 1] > 0 else 0.0))
            if worse or drop > FLAG_DROP_DB:
                cand.append(((_RANK[st[i]] - _RANK[st[i - 1]]), drop, ts[i], sid))
    flagged = set()
    for _w, _d, t, _sid in sorted(cand, key=lambda c: (-c[0], -c[1]))[:FLAG_MAX]:
        flagged.add(t)
    for e in ev:
        e["flagged"] = e["t_s"] in flagged
    have = {e["t_s"] for e in ev if e["flagged"]}
    for t in sorted(flagged - have):                        # a flagged drop without a state change
        sids = [c[3] for c in cand if c[2] == t]
        ev.append({"t_s": t, "object": ", ".join(sorted(set(sids))), "what": f"range down > {FLAG_DROP_DB:g} dB",
                   "flagged": True})
    return sorted(ev, key=lambda e: (e["t_s"], e["object"]))


def compute(mission: dict, link, progress_cb: Optional[Callable[[int, int], None]] = None,
            cancel_cb: Optional[Callable[[], bool]] = None) -> dict:
    """The profile (format in gui/profile_tab.py's docstring); "cancelled": True when stopped early (partial)."""
    ts_all = sample_instants(link)
    table = ce.cached_table()
    if table is None:
        raise ValueError("no fast-estimate table (core/cost_table.json)")
    secs = mission.get("sectors", [])
    ids = [str(s.get("id") or f"#{i + 1}") for i, s in enumerate(secs)]
    calib = calibration.view_calibration(secs, ids)
    ts, per = [], []
    cancelled = False
    for n, t in enumerate(ts_all):
        if cancel_cb is not None and cancel_cb():
            cancelled = True
            break
        per.append(evaluate_instant(mission, link, t, table, ids, calib))
        ts.append(t)
        if progress_cb is not None:
            progress_cb(n + 1, len(ts_all))
    dur = float(link.duration) if not cancelled else (ts[-1] if ts else 0.0)
    vols = {sid: {"state": [p_["volumes"][sid]["state"] for p_ in per],
                  "rho": [p_["volumes"][sid]["rho"] for p_ in per],
                  "rr_s": [p_["volumes"][sid]["rr_s"] for p_ in per],
                  "hole": [p_["volumes"][sid]["hole"] for p_ in per],
                  "rr_asked_s": float(sec.get("rr_target_s", 0.0) or 0.0)}
            for sid, sec in zip(ids, secs)}
    faces = {str(k): {r: [p_["faces"].get(str(k), {}).get(r) for p_ in per] for r in ("energy", "S time", "Ku time")}
             for k in range(4)}
    out = {"t_s": ts, "duration_s": dur, "load_ceiling": per[0]["ceiling"] if per else 0.0, "volumes": vols,
           "faces": faces, "cancelled": cancelled, "exact": {},
           "availability": {sid: availability(ts, dur, v["state"], v["rr_asked_s"], v["hole"])
                            for sid, v in vols.items()}
           if ts else {},
           "events": events(ts, vols) if ts else []}
    return out
