#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RECALIBRATION OF THE FAST ESTIMATE BY THE EXACT COMPUTATION -- final allocation rule, pass R3d (decisions M1 and M3,
user, 10 November 2026; decisions 3 and 4 of R3).

The fast estimate (core/cost_estimate.allocation_demands) costs a volume from a table: its energy within -1 % / +4 %
of the optimiser's, its S time only as a lower bound (energy x pae_sat / peak power, up to x27 per cell). After an
exact run, the ratio exact / estimate is measured per volume and per face, for the energy and for the S time (M1),
and the live load and the range gauge apply it to that volume. A volume the exact run re-planned at rho x R is
compared with the estimate of that same request (like for like).

The ratios are kept in a cache beside the code (decision 3: not in the mission file, which holds only what the
operator writes), under a fingerprint of the volume as the operator defines it (its own frame, geometry and
requirements, its name and display keys left aside) and of the parameter file and model version: the ratio stays
valid while none of them changes (M3), the attitude being free (the instant of the exact run is the reference). A
volume without a valid ratio is shown "not recalibrated". The cache is written by the tools after an exact run (the
mission preparation), never by the tests.
"""
from __future__ import annotations

import hashlib
import json
import os
from typing import Dict, List, Optional

import numpy as np

from . import radar_model as rm
from .params import P

CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".calibration_cache.json")
CACHE_MAX = 512                                  # volumes kept (the oldest dropped first)
_IGNORED = ("id", "name", "label", "colour", "color", "visible", "hidden", "note", "comment")
_cache: Optional[Dict[str, dict]] = None


def _param_stamp() -> str:
    try:
        with open(P.path, "rb") as fh:
            h = hashlib.sha1(fh.read()).hexdigest()
    except OSError:
        h = "?"
    return f"{rm.MODEL_VERSION}:{h}"


def fingerprint(sec: dict) -> str:
    """The volume as the operator defines it (not turned into the ship frame), with the parameter file and the model
    version (M3)."""
    keep = {k: v for k, v in sec.items() if k not in _IGNORED and not str(k).startswith("_")}
    txt = json.dumps(keep, sort_keys=True, default=str) + "|" + _param_stamp()
    return hashlib.sha1(txt.encode("utf-8")).hexdigest()


def _load() -> Dict[str, dict]:
    global _cache
    if _cache is None:
        try:
            with open(CACHE_PATH, encoding="utf-8") as fh:
                _cache = json.load(fh)
        except (OSError, ValueError):
            _cache = {}
    return _cache


def _save() -> None:
    c = _load()
    if len(c) > CACHE_MAX:
        for k in sorted(c, key=lambda k: c[k].get("order", 0))[:len(c) - CACHE_MAX]:
            del c[k]
    tmp = CACHE_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(c, fh, indent=1)
    os.replace(tmp, CACHE_PATH)


def _per_face(rows: np.ndarray) -> np.ndarray:
    """(4, 2) energy drawn and S time per face of the fast estimate's shares, each share on its cheapest face (the
    face of least scan, as the optimiser's owner)."""
    out = np.zeros((4, 2))
    if not len(rows):
        return out
    e = np.where(np.isfinite(rows[:, :, 0]), rows[:, :, 0], np.inf)
    k = np.argmin(e, axis=1)
    ok = np.isfinite(e[np.arange(len(rows)), k])
    np.add.at(out, k[ok], rows[ok, k[ok], :2])
    return out


def measure(mission: dict, result: dict) -> Dict[str, dict]:
    """{sector id: {"e": [4], "t": [4], "fp": fingerprint}} of an exact result: per face, exact / estimate for the
    energy drawn and for the S time (the plan's own face totals, before the moves of the rule); a face where only one
    of the two has load takes the volume's overall ratio."""
    from . import cost_estimate as ce
    from . import frames
    from . import geometry as geo
    table = ce.cached_table()
    if table is None or result.get("cancelled"):
        return {}
    resolved, _alerts = frames.resolve(mission)
    raw = {s["id"]: s for s in mission.get("sectors", [])}
    by_id = {r["id"]: r for r in result.get("sectors", []) if r.get("status") == "OK"}
    g = result["global"]
    k_w = float(g["face_elec_avail_kW"]) * 1e3 / rm.CHAIN_EFF
    grid = _grid(geo)
    out = {}
    for sec in resolved["sectors"]:
        r = by_id.get(sec["id"])
        if r is None or sec["id"] not in raw:
            continue
        ask = dict(sec)
        if "_rho" in r:                          # re-planned at rho x R: the estimate of that same request
            ask.update(range_km=float(r["_range_asked_km"]) * float(r["_rho"]), rr_target_s=float(r["_rr_tenable_s"]))
        try:
            demands, _b = ce.allocation_demands(table, [ask], grid, g["eps_min_deg"], g["eps_max_deg"], g.get("sky"),
                                                profiles="wait")          # against what the live load shows (c)
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            continue
        d = next((d for d in demands if d.kind == "search" and d.sector == str(sec["id"])), None)
        if d is None:
            continue
        fast = _per_face(np.asarray(d.rows, float))
        faces = r.get("_faces_planned") or r.get("faces", {})
        g_r = float(r.get("_reuse_g", 1.0))
        exact = np.zeros((4, 2))
        for k, fc in faces.items():
            exact[int(k)] = (float(fc.get("energy_fraction", 0.0)) * k_w / g_r, float(fc.get("time_fraction", 0.0)) / g_r)
        tot = [exact[:, j].sum() / fast[:, j].sum() if fast[:, j].sum() > 0 else None for j in range(2)]
        if tot[0] is None or tot[1] is None or tot[0] <= 0 or tot[1] <= 0:
            continue
        ratio = np.array([[exact[k, j] / fast[k, j] if exact[k, j] > 0 and fast[k, j] > 0 else tot[j]
                           for j in range(2)] for k in range(4)])
        out[str(sec["id"])] = {"e": [float(x) for x in ratio[:, 0]], "t": [float(x) for x in ratio[:, 1]],
                               "fp": fingerprint(raw[sec["id"]])}
    return out


_GRID = None


def _grid(geo):
    global _GRID
    if _GRID is None:
        _GRID = geo.SphereGrid(float(P.get("tool.display.grid_res_deg")))
    return _GRID


def record(mission: dict, result: dict) -> Dict[str, dict]:
    """Measure the ratios of an exact result and keep them (the mission preparation calls it after a run)."""
    got = measure(mission, result)
    if not got:
        return got
    c = _load()
    order = max((v.get("order", 0) for v in c.values()), default=0)
    for i, v in enumerate(got.values()):
        c[v["fp"]] = {"e": v["e"], "t": v["t"], "order": order + i + 1}
    faces = (result.get("power_allocation") or {}).get("faces", {})
    if all(str(k) in faces for k in range(4)):           # option (c): the always-on the exact run counted
        c[mission_key(mission.get("sectors", []))] = {
            "always_on_w": [float(faces[str(k)]["always_on_MW"]) * 1e6 for k in range(4)], "order": order + len(got) + 1}
    try:
        _save()
    except OSError:
        pass                                     # (a read-only install: the ratios stay for this session)
    return got


def lookup(sectors: List[dict]) -> Dict[str, dict]:
    """{sector id: {"e": [4], "t": [4]}} of the volumes (as the operator defines them) that have a valid ratio."""
    c = _load()
    out = {}
    for i, sec in enumerate(sectors):
        v = c.get(fingerprint(sec))
        if v is not None:
            out[str(sec.get("id", f"#{i + 1}"))] = {"e": v["e"], "t": v["t"]}
    return out


def mission_key(sectors: List[dict]) -> str:
    """The set of volumes of a mission (their fingerprints, in any order): the always-on of the faces depends on the
    processing load of all of them (step 4.4)."""
    return "mission:" + hashlib.sha1("|".join(sorted(fingerprint(s) for s in sectors)).encode("utf-8")).hexdigest()


def always_on(sectors: List[dict]) -> Optional[List[float]]:
    """Each face's always-on (W drawn from the bus) measured by the last exact run of this set of volumes (option (c),
    user 10 November 2026), None when there is none."""
    v = _load().get(mission_key(sectors))
    return None if v is None else list(v["always_on_w"])


def view_calibration(sectors: List[dict], ids: List[str]) -> dict:
    """The calib argument of the fast estimate for a mission's volumes (as the operator defines them) under the ids
    the view gives them: each volume's ratios, and the faces' always-on under cost_estimate.ALWAYS_ON_KEY."""
    from .cost_estimate import ALWAYS_ON_KEY
    out = {i: c for i, s in zip(ids, sectors) if (c := ratios(s)) is not None}
    on = always_on(sectors)
    if on is not None:
        out[ALWAYS_ON_KEY] = on
    return out


def ratios(sec: dict) -> Optional[dict]:
    """{"e": [4], "t": [4]} of one volume (as the operator defines it), None when not recalibrated."""
    v = _load().get(fingerprint(sec))
    return None if v is None else {"e": v["e"], "t": v["t"]}
