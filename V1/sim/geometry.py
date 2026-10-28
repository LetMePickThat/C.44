#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GEOMETRY -- what the ship sees of every object and body, in its own frame (roadmap step 6.3, pass S3b; framing G3-G9
validated by the user on 29 October 2026). It reads the truth (sim/propagate.py); the radar never computes where a
body is (the truth is the scenario's, the radar gets only its navigation and its own detections, pass S4 / step 6.4).

  G3 face and off-boresight: the direction of each object in the ship frame (the ship's flown attitude), its owner
     face by core.optimizer.face_of (the same rule as the sphere raster of the tool), theta from the face normal and
     (u, v) in the face frame; -1 = no face covers it;
  G4 any instant: states by a cubic Hermite on position and velocity between the recorded points (sim.log_step_s),
     attitudes by SLERP between the steps; a row that appears or ends inside an interval is extrapolated linearly
     from the recorded end it has;
  G5 default attitudes of the objects (nose = body Z), unless the object flies waypoints (SHIP, KKV): KKV and DECOY
     nose on their velocity relative to the ship, GENERIC and SHIP on their inertial velocity (central-body frame),
     DEBRIS tumbling at the class's rotation_dps about an axis drawn once (nose normal to it), ST toward the ship;
     a class not listed points at the ship;
  G6 aspect = angle between the nose and the direction toward the ship; RCS = the class's aspect table
     (Target.aspect_table_db: the class as it is, no sizing margin) at the centres of the usable S and Ku bands, plus
     an offset per object drawn once from the class spread (rcs_spread_db, truncated at sim.rcs_trunc_sigmas) or
     given by the scenario (rcs_offset_db); the Swerling fluctuation is NOT drawn here (radar in the loop, step 6.4);
  G7 occultation: the line of sight cut by the Earth or the Moon (spheres of mean radius); structure: the blocked
     fraction of the owner face's aperture tube toward the object (core.environment.blockage, geometric, the same in
     S and Ku), its loss per pass and the mask flag (above ship.block_mask_db);
  G8 Sun, Earth and Moon (sim/ephemeris.py): direction in the ship frame, distance, apparent radius, owner face and
     theta, the angle of every object from each of them (Sun in the beam, object against a disk), and the share of
     the solar disk hidden by the Earth or the Moon seen from the ship (umbra 1, penumbra between, flat disks).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from core import environment as env
from core import optimizer as opt
from core import radar_model as rm
from core.params import P

from . import attitude as att
from . import ephemeris as eph
from . import scenario as scn
from .propagate import Truth

BODIES = ("sun", "earth", "moon")
SUN_RADIUS_KM = float(P.get("sim.sun_radius_km"))
RCS_TRUNC_SIGMAS = float(P.get("sim.rcs_trunc_sigmas"))
F_S_GHZ = 0.5 * (rm.BAND_S.usable_lo_ghz + rm.BAND_S.usable_hi_ghz)
F_KU_GHZ = 0.5 * (rm.BAND_KU.usable_lo_ghz + rm.BAND_KU.usable_hi_ghz)
NOSE_RULES = {"KKV": "relative", "DECOY": "relative", "GENERIC": "inertial", "SHIP": "inertial",
              "DEBRIS": "tumble", "ST": "ship"}             # G5; any other class: "ship"
SEED_RCS, SEED_TUMBLE = 200000, 300000                     # generator streams per row (propagation: 0.., 100000..)


@dataclass
class Geometry:
    """Per instant t_s (n_t,) and object (ids, n_o: the truth rows without the ship), arrays (n_t, n_o[, 3]); NaN, -1 or
    False where the object does not exist. Bodies (BODIES) per instant (n_t, 3[, 3])."""
    t_s: np.ndarray
    ids: List[str]
    classes: List[str]
    f_s_ghz: float
    f_ku_ghz: float
    rcs_offset_db: np.ndarray                 # (n_o,)
    exists: np.ndarray
    range_km: np.ndarray
    range_rate_mps: np.ndarray                # > 0: opening
    dir_ship: np.ndarray                      # unit, ship frame
    face: np.ndarray                          # int8, -1 = no face
    theta_deg: np.ndarray
    u: np.ndarray
    v: np.ndarray
    aspect_deg: np.ndarray                    # 0 = nose on the ship
    nose_ship: np.ndarray                     # (n_t, n_o, 3) unit nose (body Z) of each object, ship frame
    rcs_s_dbsm: np.ndarray
    rcs_ku_dbsm: np.ndarray
    occulted_by: np.ndarray                   # int8 index into BODIES, -1 = clear
    block_f: np.ndarray                       # blocked aperture fraction of the owner face
    block_loss_db: np.ndarray                 # per pass
    masked: np.ndarray                        # loss per pass above ship.block_mask_db
    body_angle_deg: np.ndarray                # (n_t, n_o, 3) angle from each body's centre
    ship_m: np.ndarray                        # (n_t, 3, 3) ship body -> ICRF
    body_dir_ship: np.ndarray                 # (n_t, 3, 3)
    body_range_km: np.ndarray                 # (n_t, 3)
    body_radius_deg: np.ndarray               # (n_t, 3) apparent radius
    body_face: np.ndarray                     # (n_t, 3) int8
    body_theta_deg: np.ndarray                # (n_t, 3)
    ship_shadow: np.ndarray                   # (n_t,) share of the solar disk hidden
    nose_rule: List[str] = field(default_factory=list)


# ----------------------------------------------------------------------------- G4 states at any instant
def states_at(truth: Truth, t: float):
    """(r km, v km/s) of every row at t (n_rows, 3), NaN where the row does not exist."""
    tl = truth.t_s
    n = len(truth.ids)
    if len(tl) == 1:
        r, v = truth.r_km[0].copy(), truth.v_kms[0].copy()
    else:
        k = int(np.clip(np.searchsorted(tl, t, side="right") - 1, 0, len(tl) - 2))
        t0, t1 = tl[k], tl[k + 1]
        h = t1 - t0
        s = (t - t0) / h
        r0, r1, v0, v1 = truth.r_km[k], truth.r_km[k + 1], truth.v_kms[k], truth.v_kms[k + 1]
        h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
        d00, d10, d01, d11 = 6 * s ** 2 - 6 * s, 3 * s ** 2 - 4 * s + 1, -6 * s ** 2 + 6 * s, 3 * s ** 2 - 2 * s
        r = h00 * r0 + h10 * h * v0 + h01 * r1 + h11 * h * v1
        v = (d00 * r0 + d10 * h * v0 + d01 * r1 + d11 * h * v1) / h
        ok0, ok1 = np.isfinite(r0[:, 0]), np.isfinite(r1[:, 0])
        lo = ok0 & ~ok1
        hi = ok1 & ~ok0
        r[lo], v[lo] = r0[lo] + v0[lo] * (t - t0), v0[lo]
        r[hi], v[hi] = r1[hi] + v1[hi] * (t - t1), v1[hi]
    ex = (t >= truth.t_appear_s - 1e-9) & ~(t >= truth.t_impact_s - 1e-9)
    ex[0] = True
    r[~ex] = np.nan
    v[~ex] = np.nan
    return r.reshape(n, 3), v.reshape(n, 3)


# ----------------------------------------------------------------------------- G8 shadow
def _disk_overlap(rs: float, rb: float, d: float) -> float:
    """Area shared by two flat disks of radii rs, rb whose centres are d apart."""
    if d >= rs + rb:
        return 0.0
    if d <= abs(rb - rs):
        return math.pi * min(rs, rb) ** 2
    a1 = rs * rs * math.acos(max(-1.0, min(1.0, (d * d + rs * rs - rb * rb) / (2.0 * d * rs))))
    a2 = rb * rb * math.acos(max(-1.0, min(1.0, (d * d + rb * rb - rs * rs) / (2.0 * d * rb))))
    k = 0.5 * math.sqrt(max(0.0, (-d + rs + rb) * (d + rs - rb) * (d - rs + rb) * (d + rs + rb)))
    return a1 + a2 - k


def _unit(x: np.ndarray) -> np.ndarray:
    with np.errstate(invalid="ignore", divide="ignore"):
        return x / np.linalg.norm(x, axis=-1, keepdims=True)


def _angle_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.degrees(np.arccos(np.clip(np.einsum("...i,...i->...", a, b), -1.0, 1.0)))


# ----------------------------------------------------------------------------- the geometry
def _specs(sc: dict, truth: Truth) -> List[dict]:
    """The scenario entry behind each truth row (the group's for its members; {} for the ship)."""
    by_id = {o["id"]: o for o in sc.get("objects", [])}
    by_id.update({g["id"]: g for g in sc.get("groups", [])})
    return [{}] + [by_id[truth.group_of[i] or oid] for i, oid in enumerate(truth.ids) if i]


def geometry(sc: dict, truth: Truth, times: Optional[np.ndarray] = None) -> Geometry:
    """The geometry of a propagated scenario (sc: sim.scenario.load) at the instants times (default: the recorded
    ones)."""
    t_all = truth.t_s if times is None else np.asarray(times, float)
    seed = int(sc["meta"]["seed"])
    epoch = scn.epoch(sc)
    specs = _specs(sc, truth)
    rows = list(range(1, len(truth.ids)))
    n_t, n_o = len(t_all), len(rows)
    radius = {b: (SUN_RADIUS_KM if b == "sun" else float(scn.bodies()[b]["radius_km"])) for b in BODIES}

    # per object: class target, aspect tables, RCS offset, nose rule, tumble axis
    tabs, offs, rules, tumble = [], np.zeros(n_o), [], {}
    for j, i in enumerate(rows):
        cls, sp = truth.classes[i], specs[i]
        tgt = rm.target_for_sector({"id": truth.ids[i], "target_type": cls, **sp.get("overrides", {})})
        tabs.append((tgt.aspect_table_db(F_S_GHZ), tgt.aspect_table_db(F_KU_GHZ)))
        if "rcs_offset_db" in sp:
            offs[j] = float(sp["rcs_offset_db"])
        else:
            g = np.random.default_rng([seed, SEED_RCS + i])
            offs[j] = float(np.clip(g.standard_normal(), -RCS_TRUNC_SIGMAS, RCS_TRUNC_SIGMAS)) * tgt.rcs_spread_db
        flies = truth.ids[i] in truth.att_ids
        rule = "waypoints" if flies else NOSE_RULES.get(cls, "ship")
        rules.append(rule)
        if rule == "tumble":
            g = np.random.default_rng([seed, SEED_TUMBLE + i])
            a = _unit(g.standard_normal(3))
            n0 = g.standard_normal(3)
            n0 = _unit(n0 - (n0 @ a) * a)
            tumble[j] = (a, n0, math.radians(tgt.rotation_dps))
    grid = np.arange(181.0)

    shp = (n_t, n_o)
    out = {k: np.full(shp, np.nan) for k in ("range_km", "range_rate_mps", "theta_deg", "u", "v", "aspect_deg",
                                              "rcs_s_dbsm", "rcs_ku_dbsm", "block_f", "block_loss_db")}
    exists = np.zeros(shp, bool)
    dir_ship = np.full(shp + (3,), np.nan)
    nose_sh = np.full(shp + (3,), np.nan)
    face = -np.ones(shp, np.int8)
    occ = -np.ones(shp, np.int8)
    masked = np.zeros(shp, bool)
    b_ang = np.full(shp + (3,), np.nan)
    ship_m = np.full((n_t, 3, 3), np.nan)
    b_dir = np.full((n_t, 3, 3), np.nan)
    b_rng = np.full((n_t, 3), np.nan)
    b_rad = np.full((n_t, 3), np.nan)
    b_face = -np.ones((n_t, 3), np.int8)
    b_th = np.full((n_t, 3), np.nan)
    shadow = np.zeros(n_t)

    for it, t in enumerate(t_all):
        r, v = states_at(truth, float(t))
        rs, vs = r[0], v[0]
        m = truth.attitude("ship", float(t))
        ship_m[it] = m
        # bodies, ship-centred (ICRF axes)
        T = eph.centuries_tt(epoch, float(t))
        moon_geo, sun_geo = eph.moon_icrf_km(T), eph.sun_icrf_km(T)
        ship_geo = rs if truth.body.name == "earth" else moon_geo + rs
        bpos = np.array([sun_geo - ship_geo, -ship_geo, moon_geo - ship_geo])
        bd = np.linalg.norm(bpos, axis=1)
        bu = bpos / bd[:, None]
        b_rng[it] = bd
        b_rad[it] = np.degrees(np.arcsin(np.minimum(1.0, np.array([radius[b] for b in BODIES]) / bd)))
        b_dir[it] = bu @ m
        fo, w, _, _ = opt.face_of(b_dir[it, :, 0], b_dir[it, :, 1], b_dir[it, :, 2])
        b_face[it] = fo
        b_th[it] = np.where(fo >= 0, np.degrees(np.arccos(np.clip(w, 0.0, 1.0))), np.nan)
        rsun = math.radians(b_rad[it, 0])
        hid = sum(_disk_overlap(rsun, math.radians(b_rad[it, k]),
                                math.radians(float(_angle_deg(bu[0], bu[k])))) for k in (1, 2))
        shadow[it] = min(1.0, hid / (math.pi * rsun * rsun))

        ro, vo = r[rows], v[rows]
        ex = np.isfinite(ro[:, 0])
        exists[it] = ex
        if not ex.any():
            continue
        los = ro - rs                                             # ship -> object, ICRF
        rng_km = np.linalg.norm(los, axis=1)
        lu = los / rng_km[:, None]
        out["range_km"][it] = rng_km
        out["range_rate_mps"][it] = 1e3 * np.einsum("ij,ij->i", lu, vo - vs)
        ds = lu @ m                                              # ship frame
        dir_ship[it] = ds
        fo, w, uu, vv = opt.face_of(ds[:, 0], ds[:, 1], ds[:, 2])
        fo = np.where(ex, fo, -1).astype(np.int8)
        face[it] = fo
        out["theta_deg"][it] = np.where(fo >= 0, np.degrees(np.arccos(np.clip(w, 0.0, 1.0))), np.nan)
        out["u"][it], out["v"][it] = np.where(fo >= 0, uu, np.nan), np.where(fo >= 0, vv, np.nan)
        # G5 noses (ICRF) and G6 aspect / RCS
        nose = np.full((n_o, 3), np.nan)
        for j, i in enumerate(rows):
            if not ex[j]:
                continue
            rule = rules[j]
            if rule == "waypoints":
                mo = truth.attitude(truth.ids[i], float(t))
                nose[j] = mo[:, 2] if mo is not None else -lu[j]
            elif rule == "relative":
                nose[j] = vo[j] - vs
            elif rule == "inertial":
                nose[j] = vo[j]
            elif rule == "tumble":
                a, n0, om = tumble[j]
                nose[j] = att.expm(a * om * (float(t) - truth.t_appear_s[i])) @ n0
            else:
                nose[j] = -lu[j]
            if not np.linalg.norm(nose[j]) > 1e-12:              # no velocity: nose on the ship
                nose[j] = -lu[j]
        asp = _angle_deg(_unit(nose), -lu)
        nose_sh[it] = _unit(nose) @ m
        out["aspect_deg"][it] = asp
        for j in np.flatnonzero(ex):
            out["rcs_s_dbsm"][it, j] = np.interp(asp[j], grid, tabs[j][0]) + offs[j]
            out["rcs_ku_dbsm"][it, j] = np.interp(asp[j], grid, tabs[j][1]) + offs[j]
        # G7 occultation by the Earth and the Moon (segment ship -> object against each sphere)
        for k in (1, 2):
            c = bpos[k]
            s = np.clip((los @ c) / (rng_km * rng_km), 0.0, 1.0)
            dmin = np.linalg.norm(los * s[:, None] - c, axis=1)
            hit = ex & (dmin < radius[BODIES[k]]) & (occ[it] < 0)
            occ[it][hit] = k
        # G7 structure of the owner face
        for k in range(len(opt.FACE_PHI)):
            sel = np.flatnonzero(fo == k)
            if len(sel):
                f, _ = env.blockage(k, ds[sel], "S")
                out["block_f"][it, sel] = f
                loss = env.block_loss_db(f)
                out["block_loss_db"][it, sel] = loss
                masked[it, sel] = loss > env.BLOCK_MASK_DB
        b_ang[it] = _angle_deg(lu[:, None, :], bu[None, :, :])
        b_ang[it][~ex] = np.nan

    return Geometry(t_s=t_all, ids=[truth.ids[i] for i in rows], classes=[truth.classes[i] for i in rows],
                    f_s_ghz=F_S_GHZ, f_ku_ghz=F_KU_GHZ, rcs_offset_db=offs, exists=exists,
                    dir_ship=dir_ship, nose_ship=nose_sh, face=face, occulted_by=occ, masked=masked, body_angle_deg=b_ang,
                    ship_m=ship_m, body_dir_ship=b_dir, body_range_km=b_rng, body_radius_deg=b_rad, body_face=b_face,
                    body_theta_deg=b_th, ship_shadow=shadow, nose_rule=rules, **out)
