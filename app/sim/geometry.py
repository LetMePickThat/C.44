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
     Everything the radar sees is taken from the apertures, not from the ship's point, its centre of gravity (user,
     30 October 2026): array_state, r + m d and v + w x (m d), d = env.ARRAY_FROM_CG_M;
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

import json
import math
from functools import lru_cache
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

from core import environment as env
from core import optimizer as opt
from core import radar_model as rm
from core.params import P

from . import attitude as att
from . import ephemeris as eph
from . import scenario as scn
from .propagate import Truth, row_seed

BODIES = ("sun", "earth", "moon")                         # always; the central body (and its planet) added per run


def body_names(central: str) -> List[str]:
    """The bodies of a run's geometry (user, 30 October 2026: any central body): the Sun, the Earth, the Moon, and
    the central body and, for a moon, its planet when they are other ones (they hide the objects and the Sun; the
    radar sky knows the Sun, the Earth and the Moon only)."""
    out = list(BODIES)
    for b in (central, eph._MOONS[central]["planet"] if central in eph._MOONS else None):
        if b and b not in out:
            out.append(b)
    return out
SUN_RADIUS_KM = float(P.get("sim.sun_radius_km"))
RCS_TRUNC_SIGMAS = float(P.get("sim.rcs_trunc_sigmas"))
F_S_GHZ = 0.5 * (rm.BAND_S.usable_lo_ghz + rm.BAND_S.usable_hi_ghz)
F_KU_GHZ = 0.5 * (rm.BAND_KU.usable_lo_ghz + rm.BAND_KU.usable_hi_ghz)
NOSE_RULES = {"KKV": "relative", "DECOY": "relative", "GENERIC": "inertial", "SHIP": "inertial",
              "DEBRIS": "tumble", "ST": "ship"}             # G5; any other class: "ship"
SEED_RCS, SEED_TUMBLE = 200000, 300000                     # generator streams per row (propagation: 0.., 100000..)
ARRAY_KM = 1e-3 * env.ARRAY_FROM_CG_M                      # centre of gravity -> apertures, ship frame


@dataclass
class Geometry:
    """Per instant t_s (n_t,) and object (ids, n_o: the truth rows without the ship), arrays (n_t, n_o[, 3]); NaN, -1 or
    False where the object does not exist. Bodies (body_names: BODIES, then the central body and its planet when
    others) per instant (n_t, n_b[, 3])."""
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
    occulted_by: np.ndarray                   # int8 index into body_names, -1 = clear
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
    body_names: List[str] = field(default_factory=lambda: list(BODIES))


@lru_cache(maxsize=64)
def _aspect_tables(cls: str, overrides_json: str) -> Tuple[np.ndarray, np.ndarray]:
    """The class's aspect tables at S and Ku (computed once per class and overrides: 181 RCS each)."""
    tgt = rm.target_for_sector({"id": "_", "target_type": cls, **json.loads(overrides_json)})
    return tgt.aspect_table_db(F_S_GHZ), tgt.aspect_table_db(F_KU_GHZ)


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


def _ship_m(truth: Truth, t: float) -> np.ndarray:
    return truth.attitude("ship", min(max(t, float(truth.att_t_s[0])), float(truth.att_t_s[-1])))


def ship_rate_dps(truth: Truth, t: float) -> np.ndarray:
    """Angular rate of the ship (deg/s, ship frame), from the recorded attitude over the step around t."""
    ta = truth.att_t_s
    k = int(np.clip(np.searchsorted(ta, t, side="right") - 1, 0, len(ta) - 2))
    m0, m1 = _ship_m(truth, float(ta[k])), _ship_m(truth, float(ta[k + 1]))
    w_icrf = att.rotvec(m1 @ m0.T) / float(ta[k + 1] - ta[k])
    return np.degrees(_ship_m(truth, t).T @ w_icrf)


def array_state(truth: Truth, t: float, rs: np.ndarray, vs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """(r km, v km/s) of the apertures at t, from the ship's own (its centre of gravity)."""
    m = _ship_m(truth, t)
    return rs + m @ ARRAY_KM, vs + m @ np.cross(np.radians(ship_rate_dps(truth, t)), ARRAY_KM)


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
    names = body_names(truth.body.name)
    nb = len(names)
    radius = {b: (SUN_RADIUS_KM if b == "sun" else float(scn.bodies()[b]["radius_km"])) for b in names}

    # per object: class target, aspect tables, RCS offset, nose rule, tumble axis
    tabs, offs, rules, tumble = [], np.zeros(n_o), [], {}
    for j, i in enumerate(rows):
        cls, sp = truth.classes[i], specs[i]
        ov = json.dumps(sp.get("overrides", {}), sort_keys=True)
        tgt = rm.target_for_sector({"id": truth.ids[i], "target_type": cls, **sp.get("overrides", {})})
        tabs.append(_aspect_tables(cls, ov))
        if "rcs_offset_db" in sp:
            offs[j] = float(sp["rcs_offset_db"])
        else:
            g = np.random.default_rng(row_seed(seed, SEED_RCS, truth.ids[i]))     # by name (user, 8 Nov 2026)
            offs[j] = float(np.clip(g.standard_normal(), -RCS_TRUNC_SIGMAS, RCS_TRUNC_SIGMAS)) * tgt.rcs_spread_db
        flies = truth.ids[i] in truth.att_ids
        rule = "waypoints" if flies else NOSE_RULES.get(cls, "ship")
        rules.append(rule)
        if rule == "tumble":
            g = np.random.default_rng(row_seed(seed, SEED_TUMBLE, truth.ids[i]))
            a = _unit(g.standard_normal(3))
            n0 = g.standard_normal(3)
            n0 = _unit(n0 - (n0 @ a) * a)
            tumble[j] = (a, n0, math.radians(tgt.rotation_dps))
    grid = np.arange(181.0)
    # per rule, the objects (columns) concerned; tumble axes and rates as arrays (speed with 1000+ objects, user,
    # 29 October 2026: every instant handles all the objects at once)
    rule_cols = {r: np.array([j for j, x in enumerate(rules) if x == r], int)
                 for r in ("waypoints", "relative", "inertial", "tumble")}
    tum = rule_cols["tumble"]
    tum_a = np.array([tumble[j][0] for j in tum]).reshape(-1, 3)
    tum_n0 = np.array([tumble[j][1] for j in tum]).reshape(-1, 3)
    tum_b = np.cross(tum_a, tum_n0) if len(tum) else np.zeros((0, 3))     # n0 normal to a: Rodrigues in the plane
    tum_om = np.array([tumble[j][2] for j in tum])
    tum_t0 = np.array([truth.t_appear_s[rows[j]] for j in tum])
    tab_s = np.array([tb[0] for tb in tabs]).reshape(n_o, len(grid))      # (also with no object at all)
    tab_ku = np.array([tb[1] for tb in tabs]).reshape(n_o, len(grid))
    fly_ids = [truth.ids[rows[j]] for j in rule_cols["waypoints"]]

    shp = (n_t, n_o)
    out = {k: np.full(shp, np.nan) for k in ("range_km", "range_rate_mps", "theta_deg", "u", "v", "aspect_deg",
                                              "rcs_s_dbsm", "rcs_ku_dbsm", "block_f", "block_loss_db")}
    exists = np.zeros(shp, bool)
    dir_ship = np.full(shp + (3,), np.nan)
    nose_sh = np.full(shp + (3,), np.nan)
    face = -np.ones(shp, np.int8)
    occ = -np.ones(shp, np.int8)
    masked = np.zeros(shp, bool)
    b_ang = np.full(shp + (nb,), np.nan)
    ship_m = np.full((n_t, 3, 3), np.nan)
    b_dir = np.full((n_t, nb, 3), np.nan)
    b_rng = np.full((n_t, nb), np.nan)
    b_rad = np.full((n_t, nb), np.nan)
    b_face = -np.ones((n_t, nb), np.int8)
    b_th = np.full((n_t, nb), np.nan)
    shadow = np.zeros(n_t)

    for it, t in enumerate(t_all):
        r, v = states_at(truth, float(t))
        rs, vs = r[0], v[0]
        m = truth.attitude("ship", float(t))
        ship_m[it] = m
        ra, va = array_state(truth, float(t), rs, vs)
        # bodies, ship-centred (ICRF axes)
        T = eph.centuries_tt(epoch, float(t))
        geo_at = {b: eph.body_icrf_km(b, T) for b in names}
        ship_geo = geo_at[truth.body.name] + ra
        bpos = np.array([geo_at[b] - ship_geo for b in names])
        bd = np.linalg.norm(bpos, axis=1)
        bu = bpos / bd[:, None]
        b_rng[it] = bd
        b_rad[it] = np.degrees(np.arcsin(np.minimum(1.0, np.array([radius[b] for b in names]) / bd)))
        b_dir[it] = bu @ m
        fo, w, _, _ = opt.face_of(b_dir[it, :, 0], b_dir[it, :, 1], b_dir[it, :, 2])
        b_face[it] = fo
        b_th[it] = np.where(fo >= 0, np.degrees(np.arccos(np.clip(w, 0.0, 1.0))), np.nan)
        rsun = math.radians(b_rad[it, 0])
        hid = sum(_disk_overlap(rsun, math.radians(b_rad[it, k]),
                                math.radians(float(_angle_deg(bu[0], bu[k])))) for k in range(1, nb))
        shadow[it] = min(1.0, hid / (math.pi * rsun * rsun))

        ro, vo = r[rows], v[rows]
        ex = np.isfinite(ro[:, 0])
        exists[it] = ex
        if not ex.any():
            continue
        los = ro - ra                                             # apertures -> object, ICRF
        rng_km = np.linalg.norm(los, axis=1)
        lu = los / rng_km[:, None]
        out["range_km"][it] = rng_km
        out["range_rate_mps"][it] = 1e3 * np.einsum("ij,ij->i", lu, vo - va)
        ds = lu @ m                                              # ship frame
        dir_ship[it] = ds
        fo, w, uu, vv = opt.face_of(ds[:, 0], ds[:, 1], ds[:, 2])
        fo = np.where(ex, fo, -1).astype(np.int8)
        face[it] = fo
        out["theta_deg"][it] = np.where(fo >= 0, np.degrees(np.arccos(np.clip(w, 0.0, 1.0))), np.nan)
        out["u"][it], out["v"][it] = np.where(fo >= 0, uu, np.nan), np.where(fo >= 0, vv, np.nan)
        # G5 noses (ICRF) and G6 aspect / RCS, all the objects at once (by nose rule)
        nose = -lu.copy()                                      # default rule: nose on the ship
        cols = rule_cols["relative"]
        nose[cols] = vo[cols] - vs
        cols = rule_cols["inertial"]
        nose[cols] = vo[cols]
        if len(tum):
            ang = tum_om * (float(t) - tum_t0)
            nose[tum] = tum_n0 * np.cos(ang)[:, None] + tum_b * np.sin(ang)[:, None]
        for j, oid in zip(rule_cols["waypoints"], fly_ids):
            if ex[j]:
                mo = truth.attitude(oid, float(t))
                nose[j] = mo[:, 2] if mo is not None else -lu[j]
        with np.errstate(invalid="ignore"):
            weak = ~(np.linalg.norm(nose, axis=1) > 1e-12)       # no velocity: nose on the ship
        nose[weak] = -lu[weak]
        nose[~ex] = np.nan
        asp = _angle_deg(_unit(nose), -lu)
        nose_sh[it] = _unit(nose) @ m
        out["aspect_deg"][it] = asp
        # the tables on the 1-degree grid 0..180: linear interpolation as np.interp, every object at once
        e = np.flatnonzero(ex)
        a_e = np.clip(asp[e], 0.0, grid[-1])
        k0 = np.minimum(np.floor(a_e).astype(int), len(grid) - 2)
        fr = a_e - k0
        out["rcs_s_dbsm"][it, e] = tab_s[e, k0] + fr * (tab_s[e, k0 + 1] - tab_s[e, k0]) + offs[e]
        out["rcs_ku_dbsm"][it, e] = tab_ku[e, k0] + fr * (tab_ku[e, k0 + 1] - tab_ku[e, k0]) + offs[e]
        # G7 occultation by the bodies but the Sun (segment ship -> object against each sphere)
        for k in range(1, nb):
            c = bpos[k]
            s = np.clip((los @ c) / (rng_km * rng_km), 0.0, 1.0)
            dmin = np.linalg.norm(los * s[:, None] - c, axis=1)
            hit = ex & (dmin < radius[names[k]]) & (occ[it] < 0)
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
                    body_theta_deg=b_th, ship_shadow=shadow, nose_rule=rules, body_names=names, **out)


# ----------------------------------------------------------------------------- the scenario editor's geometry
SHADOW_SAMPLES = 120           # preview: the ship's shadow on at most this many instants, held between them (D)
_SHADOW: Dict[tuple, np.ndarray] = {}   # the last preview's shadow samples, by the ship's path


def _quats_to_matrices(q: np.ndarray) -> np.ndarray:
    """(n, 4) unit Hamilton quaternions [w, x, y, z] -> (n, 3, 3) (core/frames.quat_to_matrix)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


class _AtInstant:
    """A field of the preview's geometry computed at one recorded instant when read (g.rcs_s_dbsm[k, j]): the
    full geometry() of that instant, kept (LazyGeometry)."""

    def __init__(self, owner: "LazyGeometry", name: str):
        self.owner, self.name = owner, name

    def __getitem__(self, key):
        k = key[0] if isinstance(key, tuple) else key
        rest = key[1:] if isinstance(key, tuple) else ()
        if isinstance(k, (int, np.integer)):
            return getattr(self.owner.at(int(k)), self.name)[0][rest] if rest else getattr(self.owner.at(int(k)), self.name)[0]
        ks = np.arange(len(self.owner.t_s))[k]
        out = np.stack([getattr(self.owner.at(int(i)), self.name)[0] for i in np.atleast_1d(ks)])
        return out[(slice(None),) + rest] if rest else out

    @property
    def shape(self):
        return (len(self.owner.t_s), len(self.owner.ids))


@dataclass
class LazyGeometry(Geometry):
    """The geometry of a preview (sim/preview, the scenario editor, user 8 November 2026): the kinematics at every
    recorded instant at once (range, range rate, direction in the ship frame, existence, the ship's attitude; from the
    apertures as geometry(), their rotation speed left out), the radar's fields (faces, aspect, RCS, occultation,
    blockage, noses, the bodies) at one instant when read (_AtInstant), the ship's shadow on SHADOW_SAMPLES instants."""
    sc: dict = field(default_factory=dict)
    truth: Optional[Truth] = None
    _one: Dict[int, Geometry] = field(default_factory=dict)
    _shadow: Optional[np.ndarray] = None

    def at(self, k: int) -> Geometry:
        g = self._one.get(k)
        if g is None:
            if len(self._one) > 64:
                self._one.clear()
            g = self._one[k] = geometry(self.sc, self.truth, self.t_s[k:k + 1])
        return g

    def __getattribute__(self, name):
        if name == "ship_shadow":
            sh = object.__getattribute__(self, "_shadow")
            if sh is None:
                sh = object.__getattribute__(self, "_shadow_now")()
                object.__setattr__(self, "_shadow", sh)
            return sh
        return object.__getattribute__(self, name)

    def _shadow_now(self) -> np.ndarray:
        t = self.t_s
        if not len(t):
            return np.zeros(0)
        ks = np.unique(np.linspace(0, len(t) - 1, min(len(t), SHADOW_SAMPLES)).round().astype(int))
        tr = self.truth                                # kept while the ship's path is the same (an object's edit:
        key = (tr.body.name, str(self.sc["meta"].get("epoch_utc")),       # 46 ms per node drag event otherwise)
               t[ks].tobytes(), tr.r_km[ks, 0].tobytes())
        sub = _SHADOW.get(key)
        if sub is None:
            sub = geometry({**self.sc, "objects": [], "groups": []}, _ship_only(tr), t[ks]).ship_shadow
            _SHADOW.clear()
            _SHADOW[key] = sub
        return sub[np.clip(np.searchsorted(ks, np.arange(len(t)), side="right") - 1, 0, len(ks) - 1)]


def _ship_only(truth: Truth) -> Truth:
    """The truth reduced to the ship's row (the shadow reads only it)."""
    return replace(truth, ids=truth.ids[:1], classes=truth.classes[:1], r_km=truth.r_km[:, :1],
                   v_kms=truth.v_kms[:, :1], t_appear_s=truth.t_appear_s[:1], t_impact_s=truth.t_impact_s[:1],
                   t_pn_start_s=truth.t_pn_start_s[:1], miss_km=truth.miss_km[:1], group_of=truth.group_of[:1])


def lazy_geometry(sc: dict, truth: Truth) -> LazyGeometry:
    """The preview's geometry (LazyGeometry)."""
    t = truth.t_s
    n_t = len(t)
    rows = list(range(1, len(truth.ids)))
    ship_j = truth.att_ids.index("ship") if "ship" in truth.att_ids else None
    q = truth.att_q[:n_t, ship_j] if ship_j is not None else np.tile([1.0, 0.0, 0.0, 0.0], (n_t, 1))
    q = np.where(np.isfinite(q), q, [1.0, 0.0, 0.0, 0.0])
    m = _quats_to_matrices(q / np.linalg.norm(q, axis=1, keepdims=True))          # ship -> ICRF per instant
    rs, vs = truth.r_km[:, 0], truth.v_kms[:, 0]
    ra = rs + np.einsum("kij,j->ki", m, ARRAY_KM)
    ro, vo = truth.r_km[:, rows], truth.v_kms[:, rows]
    los = ro - ra[:, None]
    rng = np.linalg.norm(los, axis=2)
    with np.errstate(invalid="ignore", divide="ignore"):
        lu = los / rng[..., None]
        rate = 1e3 * np.einsum("kij,kij->ki", lu, vo - vs[:, None])
    dir_ship = np.einsum("kij,kjl->kil", lu, m)                                    # ship frame
    shp = (n_t, len(rows))
    nan = np.full(shp, np.nan)
    names = body_names(truth.body.name)
    g = LazyGeometry(t_s=t, ids=[truth.ids[i] for i in rows], classes=[truth.classes[i] for i in rows],
                     f_s_ghz=F_S_GHZ, f_ku_ghz=F_KU_GHZ, rcs_offset_db=np.zeros(len(rows)),
                     exists=np.isfinite(ro[..., 0]), range_km=rng, range_rate_mps=rate, dir_ship=dir_ship,
                     face=None, theta_deg=None, u=nan, v=nan, aspect_deg=None, nose_ship=None, rcs_s_dbsm=None,
                     rcs_ku_dbsm=None, occulted_by=None, block_f=None, block_loss_db=None, masked=None,
                     body_angle_deg=None, ship_m=m, body_dir_ship=None, body_range_km=None, body_radius_deg=None,
                     body_face=None, body_theta_deg=None, ship_shadow=None, body_names=names, sc=sc, truth=truth)
    for name in ("face", "theta_deg", "aspect_deg", "nose_ship", "rcs_s_dbsm", "rcs_ku_dbsm", "occulted_by",
                 "block_f", "block_loss_db", "masked", "body_angle_deg", "body_dir_ship", "body_range_km",
                 "body_radius_deg", "body_face", "body_theta_deg", "u", "v"):
        object.__setattr__(g, name, _AtInstant(g, name))
    return g
