#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PROPAGATE -- the truth of a global scenario (roadmap step 6.2, pass S2; framing P1-P12 validated by the user on
28 October 2026). Every body of the scenario, the ship included, is integrated together in the inertial frame of the
central body (ICRF axes, km, km/s, s):
  - forces: two bodies + J2 about the body's pole (Earth: the ICRF Z axis, precession left out; Moon: the IAU pole),
    no drag (C1); plus the attraction of the other bodies (user, 29 October 2026, O1-O5: every body orbits its
    reference and pulls the objects): the Sun and the Moon around the Earth, the Sun and the Earth around the Moon,
    the differential term mu_b [(r_b - r) / |r_b - r|^3 - r_b / |r_b|^3] of a frame centred on the central body
    (and the planets, Mercury to Pluto, with sim.planets_gravity, and the major moons, [sim.moons], with
    sim.moons_gravity, user 29 October 2026),
    body positions from sim/ephemeris.py on a THIRD_BODY_STEP_S grid, cubic between its points; off with
    sim.third_body = false;
  - RK4 at the fixed step meta.step_s, a step split only at the exact time of an event (impulse, burn start or end,
    appearance, PN start at a given time) (C2);
  - manoeuvres (C3-C5): impulse (dv at t_s), burn (constant acceleration over its duration), in the object's own rtn
    frame (X radial, Y along track, Z = r x v, step 3.0), its pnr frame (X prograde along v, Y normal along r x v,
    Z radial in the orbit's plane, X x Y; the manoeuvre node's axes, user 29 October 2026) or in inertial axes; Singer (Gauss-Markov acceleration per
    inertial axis, exact discretisation, the class's a_max and maneuver_tau with the Singer intensity of
    core/trackmodel without the filter's floor; seed meta.seed + row); proportional navigation toward the ship,
    a = N Vc (Omega x LOS), normal to the line of sight, bounded by the class's accel_max_mps2, from t_s or from the
    first step within start_range_km, until the range opens again (the miss distance is the closest approach over the
    step, relative motion linear within it), then ballistic;
  - an absolute state is given at the scenario epoch and propagated from t = 0, the object existing from t_appear_s;
    a relative state is resolved at t_appear_s from the ship (frame "ship": the ship's attitude at that step); a group
    draws its n objects around its reference at t_appear_s (isotropic Gaussian, seeded);
  - attitudes flown by waypoints (pass S3a, sim/attitude.py): the ownship and the SHIP / KKV objects that have an
    "attitude" list, advanced at every step and recorded at every step (attitude() interpolates by SLERP);
  - an object below the body's mean radius is marked impacted and stops; the ship below it (its own manoeuvres,
    user 29 October 2026: the radar is lost) ends the run there, its last record at the impact (Truth.t_s ends
    before meta.duration_s; nothing spawns after it).
The truth is recorded every sim.log_step_s (the playback interpolates with a cubic on position and velocity); the
.npz journal is pass S4 (D6).
"""
from __future__ import annotations

import copy
import math
import zlib
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from core import frames
from core import radar_model as rm
from core import trackmodel as tm
from core.params import P

from . import attitude as att
from . import fast
from . import ephemeris as eph
from . import scenario as scn

LOG_STEP_S = float(P.get("sim.log_step_s"))      # P4: recording interval of the truth
CHECKPOINT_S = float(P.get("sim.checkpoint_s"))  # L2: a full state of the fleet kept at every this (user)
SHIP_MANOEUVRE_CLASS = str(P.get("sim.ship_manoeuvre_class"))   # the ship's pn / singer: this class's bounds
SHIP_MANEUVER_TAU_S = float(P.get("sim.ship_maneuver_tau_s"))   # ... and this Singer time constant
THIRD_BODY = bool(P.get("sim.third_body"))       # O4: attraction of the other bodies
PLANETS_GRAVITY = bool(P.get("sim.planets_gravity"))   # ... the planets' too (user, 29 October 2026)
MOONS_GRAVITY = bool(P.get("sim.moons_gravity"))       # ... and the major moons' (user, 29 October 2026)
SUN_MU = float(P.get("sim.sun_mu_km3_s2"))       # O2
PARALLEL_MIN_ROWS = int(P.get("sim.parallel_min_rows"))   # C2: gravity on all the cores from this many rows
_NO_BODIES, _NO_MU = np.zeros((0, 3)), np.zeros(0)  # no other body (sim.third_body false)
_NO_GRID = np.zeros((4, 0, 3))                       # ... their grid (fast.coast_path)
BURN_SUBSTEPS = 20                                 # manoeuvre node preview: a burn in this many RK4 steps at least
SEED_SINGER, SEED_GROUP = 0, 1                     # L3: the generators named [seed, this, crc32(row id)]


def row_seed(seed: int, kind: int, oid: str) -> list:
    """The seed of a row's generator by its name, not its rank (fork RADARv6_RS, user 8 November 2026: adding or
    removing an object leaves the other rows' draws as they were)."""
    return [int(seed), int(kind), zlib.crc32(oid.encode("utf-8"))]


THIRD_BODY_STEP_S = 60.0                         # O3: grid of the body positions (cubic between: error << 1 m)


_VIEW_THIRD: Dict[tuple, "ThirdBodies"] = {}


def view_third_bodies(central: str, epoch, span_s: float) -> "ThirdBodies":
    """The other bodies' grid for the views (free falls, the manoeuvre node's path), kept per central body and epoch
    and grown only when a longer span is asked (fork RADARv6_RS, 8 November 2026: built again at each previewed run,
    0.25 s of a node drag's 0.4 s). Not for the propagation itself, whose grid follows its own duration."""
    key = (central, str(epoch))
    got = _VIEW_THIRD.get(key)
    if got is None or got.span_s < span_s:
        if len(_VIEW_THIRD) > 4:
            _VIEW_THIRD.clear()
        got = _VIEW_THIRD[key] = ThirdBodies(central, epoch, span_s)
    return got


class ThirdBodies:
    """The other bodies seen from the central body over a run: positions on a grid, cubic (4-point Lagrange)
    between its points, and their differential attraction on positions r (O1)."""

    def __init__(self, central: str, epoch, duration_s: float):
        # every body but the central one (user, 30 October 2026: any body can be the central one): the Earth, the
        # Moon, the Sun, then the planets (sim.planets_gravity) and the major moons (sim.moons_gravity)
        b = scn.bodies()
        names = [x for x in ("earth", "moon", "sun") if x != central]
        names += [x for x in eph.PLANETS if PLANETS_GRAVITY and x != central]
        names += [x for x in eph.MOON_NAMES if MOONS_GRAVITY and x != central]
        self.names = tuple(names)
        self.mu = np.array([SUN_MU if x == "sun" else float(b[x]["mu_km3_s2"]) for x in names])
        self.t0 = -THIRD_BODY_STEP_S
        n = int(math.ceil(duration_s / THIRD_BODY_STEP_S)) + 4
        ts = self.t0 + THIRD_BODY_STEP_S * np.arange(n)
        pos = np.empty((n, len(names), 3))
        for k, t in enumerate(ts):
            T = eph.centuries_tt(epoch, float(t))
            at = {"earth": np.zeros(3), "sun": eph.sun_icrf_km(T)}           # geocentric, then from the central body
            for x in list(names) + [central]:
                if x not in at:
                    if x in eph._MOONS and eph._MOONS[x]["planet"] not in at:
                        at[eph._MOONS[x]["planet"]] = eph.planet_icrf_km(eph._MOONS[x]["planet"], T)
                    at[x] = (at[eph._MOONS[x]["planet"]] + eph.satellite_planetocentric_km(x, T)
                             if x in eph._MOONS else eph.body_icrf_km(x, T))
            centre = at[central]
            pos[k] = np.array([at[x] for x in names]) - centre
        self.pos = pos
        self.span_s = float(duration_s)
        self._cache: Dict[float, np.ndarray] = {}          # the few instants of the current step (RK4 stages)

    def positions(self, t: float) -> np.ndarray:
        """(2, 3) positions (km) of the other bodies at t, from the central body (computed once per instant: the
        RK4 stages share t0 + h / 2, and t0 + h is the next step's t0)."""
        got = self._cache.get(t)
        if got is not None:
            return got
        if len(self._cache) > 8:
            self._cache.clear()
        x = (t - self.t0) / THIRD_BODY_STEP_S
        k = int(np.clip(math.floor(x), 1, len(self.pos) - 3))
        u = x - k
        w = np.array([-u * (u - 1.0) * (u - 2.0) / 6.0, (u + 1.0) * (u - 1.0) * (u - 2.0) / 2.0,
                      -(u + 1.0) * u * (u - 2.0) / 2.0, (u + 1.0) * u * (u - 1.0) / 6.0])
        out = np.einsum("k,kij->ij", w, self.pos[k - 1:k + 3])
        self._cache[t] = out
        return out

    def accel(self, r: np.ndarray, t: float) -> np.ndarray:
        """Differential attraction (km/s2) of the other bodies on positions r (n, 3) at t."""
        a = np.zeros_like(r)
        for mu, rb in zip(self.mu, self.positions(t)):
            d = rb - r
            dn = np.linalg.norm(d, axis=1)
            a += mu * (d / (dn ** 3)[:, None] - rb / float(np.linalg.norm(rb)) ** 3)
        return a

@dataclass
class Body:
    """The central body: mu (km3/s2), J2, equatorial and mean radii (km), unit pole (ICRF)."""
    name: str
    mu: float
    j2: float
    r_eq_km: float
    radius_km: float
    pole: np.ndarray

    @classmethod
    def of(cls, name: str) -> "Body":
        b = scn.bodies()[name]
        return cls(name, float(b["mu_km3_s2"]), float(b["j2"]), float(b["r_eq_km"]), float(b["radius_km"]),
                   frames.unit(float(b["pole_ra_deg"]), float(b["pole_dec_deg"])))

    def equator(self) -> np.ndarray:
        """Columns: its equatorial axes in ICRF (scn.equator_axes)."""
        return scn.equator_axes(self.name)

    def accel(self, r: np.ndarray) -> np.ndarray:
        """Two-body + J2 acceleration (km/s2) of positions r (n, 3)."""
        rn2 = np.einsum("ij,ij->i", r, r)
        rn = np.sqrt(rn2)
        a = -self.mu * r / (rn2 * rn)[:, None]
        if self.j2:
            z = r @ self.pole
            k = 1.5 * self.j2 * self.mu * self.r_eq_km ** 2 / (rn2 * rn2 * rn)
            a -= k[:, None] * ((1.0 - 5.0 * z * z / rn2)[:, None] * r + 2.0 * z[:, None] * self.pole)
        return a


def coast(body: Body, third: Optional[ThirdBodies], r: np.ndarray, v: np.ndarray, t0: float, duration_s: float,
          n_steps: int) -> np.ndarray:
    """Free fall of the states r, v (n, 3) at t0 over duration_s (user, 29 October 2026: the orbital velocity vector
    of the 3D views, the way the body would fly under the local gravity alone): the gravity of the run (two-body,
    J2, the other bodies when given), no burn, no guidance, no Singer acceleration; RK4 in n_steps equal steps.
    Positions (n_steps + 1, n, 3), NaN from where a path meets the central body's surface (and for NaN states)."""
    acc = _gravity(body, third)
    ok = np.isfinite(r[:, 0]) & np.isfinite(v[:, 0])
    out = np.full((n_steps + 1, len(r), 3), np.nan)
    if not ok.any():
        return out
    x, u = r[ok].astype(float), v[ok].astype(float)
    out[0, ok] = x
    h = duration_s / max(n_steps, 1)
    for k in range(n_steps):
        t = t0 + k * h
        k1v, k1a = u, acc(t, x)
        k2v, k2a = u + 0.5 * h * k1a, acc(t + 0.5 * h, x + 0.5 * h * k1v)
        k3v, k3a = u + 0.5 * h * k2a, acc(t + 0.5 * h, x + 0.5 * h * k2v)
        k4v, k4a = u + h * k3a, acc(t + h, x + h * k3v)
        x = x + h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)
        u = u + h / 6.0 * (k1a + 2.0 * k2a + 2.0 * k3a + k4a)
        out[k + 1, ok] = x
    below = np.linalg.norm(out, axis=2) < body.radius_km                  # under the surface (NaN: False)
    out[np.maximum.accumulate(below, axis=0)] = np.nan
    return out


def _gravity(body: Body, third: Optional[ThirdBodies]) -> Callable[[float, np.ndarray], np.ndarray]:
    """The run's gravity (two bodies, J2, the other bodies when given) on positions (n, 3) at t, km/s2."""
    def acc(t: float, rr: np.ndarray) -> np.ndarray:
        if fast.USE_NUMBA:
            tpos, tmu = (third.positions(t), third.mu) if third is not None else (_NO_BODIES, _NO_MU)
            return fast.gravity(np.ascontiguousarray(rr), np.zeros_like(rr), body.mu, float(body.j2), body.r_eq_km,
                                body.pole, tpos, tmu)
        a = body.accel(rr)
        return a + third.accel(rr, t) if third is not None else a
    return acc


def preview_path(body: Body, third: Optional[ThirdBodies], r0: np.ndarray, v0: np.ndarray, t0: float, t1: float,
                 manoeuvres: List[dict], step_s: float):
    """One body from its state r0, v0 (3,) at t0 to t1 under the run's gravity with its own impulses and burns (the
    manoeuvre node's real-time preview, user 29 October 2026: no PN, no Singer, no other body's influence; the full
    propagation follows): RK4 at step_s, split at every manoeuvre time, a burn in BURN_SUBSTEPS steps at least.
    Returns (t (k,), r (k, 3), v (k, 3), pre): pre[j] the state (r, v) just before manoeuvre j (impulse: at t_s,
    burn: at its start), None when it falls outside [t0, t1] or after the surface is met; the path stops at the
    central body's surface."""
    acc = _gravity(body, third)
    ms = [m for m in manoeuvres if m.get("type") in ("impulse", "burn")]
    mine = {id(m) for m in ms}                             # (by identity: two manoeuvres may be equal)
    burns = [(float(m["t_s"]), float(m["t_s"]) + float(m["duration_s"]), m) for m in ms if m["type"] == "burn"]
    ev = {t0, t1}
    for m in ms:
        ev.add(float(m["t_s"]))
        if m["type"] == "burn":
            ev.add(float(m["t_s"]) + float(m["duration_s"]))
    ev = sorted(t for t in ev if t0 <= t <= t1)
    pre: List[Optional[tuple]] = [None] * len(manoeuvres)
    x, u = np.asarray(r0, float).copy(), np.asarray(v0, float).copy()
    ts, rs, vs = [t0], [x.copy()], [u.copy()]
    axes = body.equator()

    def force(t: float, rr: np.ndarray, vv: np.ndarray, run) -> np.ndarray:
        a = acc(t, rr[None])[0]
        for m in run:
            a = a + frame_matrix(m["frame"], rr, vv, axes) @ (1e-3 * np.asarray(m["accel_mps2"], float))
        return a

    for a, b in zip(ev[:-1], ev[1:]):
        for j, m in enumerate(manoeuvres):                   # the manoeuvres at a: their state before, the impulses
            if id(m) in mine and abs(float(m["t_s"]) - a) < 1e-9:
                pre[j] = (x.copy(), u.copy())
                if m["type"] == "impulse":
                    u = u + frame_matrix(m["frame"], x, u, axes) @ (1e-3 * np.asarray(m["dv_mps"], float))
        running = [(b0, b1, m) for b0, b1, m in burns if b0 - 1e-9 <= a < b1 - 1e-9]
        run = [m for _b0, _b1, m in running]
        h = step_s if not run else min(step_s, min(b1 - b0 for b0, b1, _m in running) / BURN_SUBSTEPS)
        n = max(1, int(math.ceil((b - a) / max(h, 1e-9) - 1e-9)))
        h = (b - a) / n
        if fast.USE_NUMBA and not run:                       # a free fall: compiled (8 November 2026)
            o_r, o_v = np.empty((n, 3)), np.empty((n, 3))
            grid, g_t0, g_step, tmu = ((third.pos, third.t0, THIRD_BODY_STEP_S, third.mu) if third is not None
                                       else (_NO_GRID, 0.0, 1.0, _NO_MU))
            done = fast.coast_path(x, u, a, h, n, grid, g_t0, g_step, tmu, body.mu, float(body.j2), body.r_eq_km,
                                   body.pole, body.radius_km, o_r, o_v)
            ts.extend(a + h * np.arange(1, done + 1))
            rs.extend(o_r[:done])
            vs.extend(o_v[:done])
            x, u = o_r[done - 1].copy(), o_v[done - 1].copy()
            if done < n:                                     # met the surface: the path stops there
                return np.array(ts), np.array(rs), np.array(vs), pre
            continue
        for k in range(n):
            t = a + k * h
            k1v, k1a = u, force(t, x, u, run)
            k2v, k2a = u + 0.5 * h * k1a, force(t + 0.5 * h, x + 0.5 * h * k1v, u + 0.5 * h * k1a, run)
            k3v, k3a = u + 0.5 * h * k2a, force(t + 0.5 * h, x + 0.5 * h * k2v, u + 0.5 * h * k2a, run)
            k4v, k4a = u + h * k3a, force(t + h, x + h * k3v, u + h * k3a, run)
            x = x + h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)
            u = u + h / 6.0 * (k1a + 2.0 * k2a + 2.0 * k3a + k4a)
            ts.append(t + h)
            rs.append(x.copy())
            vs.append(u.copy())
            if float(np.linalg.norm(x)) < body.radius_km:   # met the surface: the path stops there
                return np.array(ts), np.array(rs), np.array(vs), pre
    for j, m in enumerate(manoeuvres):                       # a manoeuvre at t1 exactly
        if id(m) in mine and pre[j] is None and abs(float(m["t_s"]) - t1) < 1e-9:
            pre[j] = (x.copy(), u.copy())
    return np.array(ts), np.array(rs), np.array(vs), pre


def elements_to_state(o: dict, mu: float) -> Tuple[np.ndarray, np.ndarray]:
    """(r km, v km/s) of Keplerian elements a, e, i, raan, argp, nu (closed orbit), in the axes they are given in."""
    a, e = float(o["a_km"]), float(o["e"])
    i, W, w, nu = (math.radians(float(o[k])) for k in ("i_deg", "raan_deg", "argp_deg", "nu_deg"))
    p = a * (1.0 - e * e)
    r_pf = p / (1.0 + e * math.cos(nu)) * np.array([math.cos(nu), math.sin(nu), 0.0])
    v_pf = math.sqrt(mu / p) * np.array([-math.sin(nu), e + math.cos(nu), 0.0])
    cW, sW, ci, si, cw, sw = math.cos(W), math.sin(W), math.cos(i), math.sin(i), math.cos(w), math.sin(w)
    m = np.array([[cW * cw - sW * sw * ci, -cW * sw - sW * cw * ci, sW * si],
                  [sW * cw + cW * sw * ci, -sW * sw + cW * cw * ci, -cW * si],
                  [sw * si, cw * si, ci]])
    return m @ r_pf, m @ v_pf


def state_to_elements(r: np.ndarray, v: np.ndarray, mu: float) -> dict:
    """The Keplerian elements of a state, in the axes of r and v: elements_to_state's inverse, angles in [0, 360) deg. A
    circular orbit counts its anomaly from the node (argp 0), an equatorial one its node from +X (raan 0)."""
    r, v = np.asarray(r, float), np.asarray(v, float)
    rn, h = float(np.linalg.norm(r)), np.cross(r, v)
    hn = float(np.linalg.norm(h))
    ev = ((v @ v - mu / rn) * r - (r @ v) * v) / mu
    e = float(np.linalg.norm(ev))
    n = np.array([-h[1], h[0], 0.0])
    nn = float(np.linalg.norm(n))
    node = n / nn if nn > 1e-9 * hn else np.array([1.0, 0.0, 0.0])
    side = np.cross(h / hn, node)
    if e > 1e-9:
        w = math.atan2(float(ev @ side), float(ev @ node))
        nu = math.atan2(float(r @ np.cross(h / hn, ev / e)), float(r @ ev / e))
    else:
        w, nu = 0.0, math.atan2(float(r @ side), float(r @ node))
    return {"type": "elements", "a_km": 1.0 / (2.0 / rn - (v @ v) / mu), "e": e,
            "i_deg": math.degrees(math.atan2(math.hypot(h[0], h[1]), h[2])),
            "raan_deg": math.degrees(math.atan2(node[1], node[0])) % 360.0, "argp_deg": math.degrees(w) % 360.0,
            "nu_deg": math.degrees(nu) % 360.0}


def pnr_matrix(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Columns X prograde (along v), Y normal (r x v), Z = X x Y radial (in the orbit's plane, outward, normal to v):
    the manoeuvre node's axes (user, 29 October 2026); v_icrf = M v_pnr; r, v (3,) or (n, 3)."""
    x = v / np.linalg.norm(v, axis=-1, keepdims=True)
    y = np.cross(r, v)
    y = y / np.linalg.norm(y, axis=-1, keepdims=True)
    z = np.cross(x, y)
    return np.stack([x, y, z], axis=-1)


def frame_matrix(frame: str, r: np.ndarray, v: np.ndarray, axes: np.ndarray) -> np.ndarray:
    """The matrix of a manoeuvre frame ("rtn", "pnr", "inertial": the central body's equatorial axes, axes, Body.equator)
    at the state r, v (3,) or (n, 3): v_icrf = M v."""
    if frame == "rtn":
        return rtn_matrix(r, v)
    if frame == "pnr":
        return pnr_matrix(r, v)
    return np.broadcast_to(axes, np.shape(r)[:-1] + (3, 3)).copy()


def rtn_matrix(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Columns X radial, Y = Z x X, Z = r x v (step 3.0): v_icrf = M v_rtn; r, v (3,) or (n, 3) -> (3, 3) or (n, 3, 3)."""
    x = r / np.linalg.norm(r, axis=-1, keepdims=True)
    z = np.cross(r, v)
    z = z / np.linalg.norm(z, axis=-1, keepdims=True)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=-1)


@dataclass
class Truth:
    """The propagated scenario: per row (0 = ship, then the objects, then the group members in order) its id, class,
    recorded states (t_s (n_log,), r_km and v_kms (n_log, n_rows, 3), NaN when the row does not exist), its appearance
    time, impact time, PN start and miss distance (NaN when none). Attitudes flown by waypoints: att_ids (the rows),
    att_q (n_att_t, n_att, 4) body -> ICRF quaternions at the instants att_t_s (those of the record, sim.log_step_s,
    user 29 October 2026: memory with many objects flying waypoints; NaN before the body exists),
    att_events (slews not finished, holds lost, targets missing)."""
    body: Body
    ids: List[str]
    classes: List[str]
    t_s: np.ndarray
    r_km: np.ndarray
    v_kms: np.ndarray
    t_appear_s: np.ndarray
    t_impact_s: np.ndarray
    t_pn_start_s: np.ndarray
    miss_km: np.ndarray
    group_of: List[Optional[str]] = field(default_factory=list)
    att_ids: List[str] = field(default_factory=list)
    att_t_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    att_q: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 4)))
    att_events: List[dict] = field(default_factory=list)

    def attitude(self, oid: str, t_s: float) -> Optional[np.ndarray]:
        """Body -> ICRF matrix of a body flying waypoints at t_s (SLERP between steps); None before it exists."""
        j = self.att_ids.index(oid)
        t = self.att_t_s
        k = int(np.clip(np.searchsorted(t, t_s, side="right") - 1, 0, len(t) - 2))
        q0, q1 = self.att_q[k, j], self.att_q[k + 1, j]
        if not (np.isfinite(q0).all() and np.isfinite(q1).all()):
            return None
        u = float(np.clip((t_s - t[k]) / (t[k + 1] - t[k]), 0.0, 1.0))
        return frames.quat_to_matrix(att.slerp(q0, q1, u))

    def exists(self) -> np.ndarray:
        return np.isfinite(self.r_km[..., 0])

    def row(self, oid: str) -> int:
        return self.ids.index(oid)


class _Row:
    """Settings of one propagated row (manoeuvres, Singer, PN) and its spawn rule."""

    def __init__(self, oid: str, cls: str, spec: dict, group: Optional[str] = None, bound_cls: Optional[str] = None):
        # bound_cls: the class whose acceleration and Singer time bound the manoeuvres, when not the row's own (the
        # ship: no class of its own, sim.ship_manoeuvre_class)
        self.id, self.cls, self.spec, self.group = oid, cls, spec, group
        self.t_appear = float(spec.get("t_appear_s", 0.0))
        bound = bound_cls or cls
        tgt = rm.target_for_sector({"id": oid, "target_type": bound, **spec.get("overrides", {})}) if bound else None
        self.a_max_kms2 = (tgt.accel_max_mps2 * 1e-3) if tgt is not None else 0.0
        ms = spec.get("manoeuvres", [])
        self.impulses = [m for m in ms if m["type"] == "impulse"]
        self.burns = [m for m in ms if m["type"] == "burn"]
        self.pn = next((m for m in ms if m["type"] == "pn"), None)
        sing = any(m["type"] == "singer" for m in ms)
        self.singer: Optional[Tuple[float, float]] = None
        if sing and tgt is not None and tgt.accel_max_mps2 > 0.0 and tgt.maneuver_tau_s > 0.0:
            s2 = tgt.accel_max_mps2 ** 2 / 3.0 * (1.0 + 4.0 * tm.P_MAX - tm.P0)      # Singer intensity (A3), m/s2
            self.singer = (math.sqrt(s2) * 1e-3, float(tgt.maneuver_tau_s))
        self.relative = spec.get("state", {}).get("type") == "relative"


def _ship_row(sc: dict) -> _Row:
    """The ship's row: its manoeuvres (user, 29 October 2026), bounded by the class sim.ship_manoeuvre_class with
    the Singer time sim.ship_maneuver_tau_s; no class of its own."""
    spec = {"manoeuvres": sc["ship"].get("manoeuvres", []),
            "overrides": {"maneuver_tau_s": SHIP_MANEUVER_TAU_S}}
    return _Row("ship", "", spec, bound_cls=SHIP_MANOEUVRE_CLASS)


def _event_times(rows: List[_Row], groups: List[dict], duration: float) -> np.ndarray:
    ev = set()
    for w in rows:                                    # the ship's manoeuvres too (user, 29 October 2026)
        if w.relative or w.group:
            ev.add(w.t_appear)
        ev.update(float(m["t_s"]) for m in w.impulses)
        for m in w.burns:
            ev.update((float(m["t_s"]), float(m["t_s"]) + float(m["duration_s"])))
        if w.pn is not None and "t_s" in w.pn:
            ev.add(float(w.pn["t_s"]))
    ev.update(float(g["t_appear_s"]) for g in groups)
    return np.array(sorted(t for t in ev if 0.0 < t < duration))


class LiveSim:
    """The scenario simulated step by step (fork RADARv6_RS, live architecture, step L1, user 8 November 2026): the
    state of every row at the current time, advanced by step() / advance_to(), and its record (every sim.log_step_s)
    read at any moment by truth(). The same computation as the former one-block propagate, in the same order (the
    same results to the bit); propagate() is LiveSim run to the end. The run stops at the end date (meta.duration_s)
    or at the ship's impact."""

    def __init__(self, sc: dict):
        self.sc = sc
        self.tag = 0                                      # (L5: what a caller knows of the edit it takes in)
        body = self.body = Body.of(sc["central_body"])
        self.third = self._third_bodies(body.name, sc) if THIRD_BODY else None
        self.duration, self.dt = float(sc["meta"]["duration_s"]), float(sc["meta"]["step_s"])
        seed = self.seed = int(sc["meta"]["seed"])
        self.n_log_every = max(1, int(round(LOG_STEP_S / self.dt)))
        self.groups = sc.get("groups", [])
        rows = self.rows = self.row_specs(sc)
        n = self.n = len(rows)
        self.idx = {w.id: i for i, w in enumerate(rows)}
        self.x = np.full((n, 6), np.nan)
        self.active = np.zeros(n, bool)
        self.spawned = np.zeros(n, bool)
        self.t_appear = np.array([w.t_appear for w in rows])
        self.t_impact = np.full(n, np.nan)
        self.t_pn = np.full(n, np.nan)
        self.miss = np.full(n, np.nan)
        self.pn_on = np.zeros(n, bool)
        self.pn_done = np.zeros(n, bool)
        self.axes = axes = body.equator()
        x, active, spawned = self.x, self.active, self.spawned

        x[0] = self._absolute(sc["ship"]["orbit"])
        active[0] = spawned[0] = True
        for i, w in enumerate(rows[1:], 1):
            if not w.relative and not w.group:
                x[i] = self._absolute(w.spec["state"])
                active[i] = spawned[i] = True

        # Singer states (km/s2 per inertial axis), one generator per row
        self.sing = [i for i, w in enumerate(rows) if w.singer is not None]
        self.rngs = {i: np.random.default_rng(row_seed(seed, SEED_SINGER, rows[i].id)) for i in self.sing}
        self.a_sing = np.zeros((n, 3))
        for i in self.sing:
            self.a_sing[i] = rows[i].singer[0] * self.rngs[i].standard_normal(3)
        self.pn_n = np.array([float(w.pn["n"]) if w.pn is not None else 0.0 for w in rows])
        self.a_max = np.array([w.a_max_kms2 for w in rows])
        burns = [(i, m) for i, w in enumerate(rows) for m in w.burns]
        self.b_row = np.array([i for i, _m in burns], int)
        self.b_t0 = np.array([float(m["t_s"]) for _i, m in burns])
        self.b_t1 = self.b_t0 + np.array([float(m["duration_s"]) for _i, m in burns])
        self.b_acc = np.array([1e-3 * np.asarray(m["accel_mps2"], float) for _i, m in burns]).reshape(-1, 3)
        self.b_frame = np.array([m["frame"] for _i, m in burns], object)
        self.pn_rows = np.array([i for i, w in enumerate(rows) if w.pn is not None], int)
        self.pn_t = np.array([float(rows[i].pn["t_s"]) if "t_s" in rows[i].pn else np.nan for i in self.pn_rows])
        self.pn_r0 = np.array([float(rows[i].pn.get("start_range_km", np.nan)) for i in self.pn_rows])
        # each guided row's target (user, 29 October 2026: OWNSHIP's own KKV toward SHIP_1): "target" of its pn
        # manoeuvre, the ship (row 0) by default
        self.pn_tgt = np.zeros(n, int)
        for i in self.pn_rows:
            self.pn_tgt[i] = self.idx[rows[i].pn.get("target", "ship")]
        flyers = self.flyers = {0: att.Flyer(0, sc["ship"]["attitude"], axes)}
        flyers.update({i: att.Flyer(i, w.spec["attitude"], axes) for i, w in enumerate(rows)
                       if i and "attitude" in w.spec and not w.group})
        self.now = 0.0                                    # time of the states in x (the targets are extrapolated)
        # rows appearing later (relative states, group members), spawned in row order once their time has come
        self.pending = np.array([i for i, w in enumerate(rows) if (w.relative or w.group)], int)
        # impulses sorted by time (row order kept at equal times), found by bisection
        self.imp = sorted(((float(m["t_s"]), i, m) for i, w in enumerate(rows) for m in w.impulses), key=lambda e: e[0])
        self.imp_t = np.array([e[0] for e in self.imp])

        self.events = _event_times(rows, self.groups, self.duration)
        self.n_steps = int(math.ceil(self.duration / self.dt - 1e-9))
        n_log = self.n_steps // self.n_log_every + 1 + (1 if self.n_steps % self.n_log_every else 0)
        self.t_log = np.empty(n_log)
        self.r_log = np.full((n_log, n, 3), np.nan)
        self.v_log = np.full((n_log, n, 3), np.nan)
        self.att_rows = sorted(flyers)
        self.att_q = np.full((n_log, len(self.att_rows), 4), np.nan)    # at the recorded instants (sim.log_step_s)

        for i in self.att_rows:
            if spawned[i]:
                flyers[i].start(x[i, :3], x[i, 3:], self._tpos)
        self._spawn(0.0)
        for i in self.att_rows:                           # an initial attitude aimed at an object spawned at t = 0
            if flyers[i].events and flyers[i].events[-1]["kind"] == "target missing" and active[i]:
                flyers[i].events.pop()
                flyers[i].start(x[i, :3], x[i, 3:], self._tpos)
        self._impulses(0.0)
        self._pn_update(0.0, x.copy())
        self._record(0, 0.0)
        self.j_log, self.e_ptr = 1, 0
        self.k = 0                                        # the next step to take
        self.t = 0.0                                      # the time of the states
        self.done = self.n_steps == 0
        self.fleet = att.Fleet(flyers, self.idx) if fast.USE_NUMBA else None
        self.n_ck_every = max(1, int(round(CHECKPOINT_S / self.dt)))
        self.checkpoints: List[dict] = [self.checkpoint()]    # (L2) by time, the first one at t = 0

    @staticmethod
    def _third_bodies(central: str, sc: dict) -> ThirdBodies:
        """The other bodies' grid over the run (the scenario editor's preview keeps it from one run to the next)."""
        return ThirdBodies(central, scn.epoch(sc), float(sc["meta"]["duration_s"]))

    @staticmethod
    def row_specs(sc: dict) -> List[_Row]:
        """The rows of a scenario in order: the ship, the objects, then the group members."""
        rows = [_ship_row(sc)] + [_Row(o["id"], o["class"], o) for o in sc["objects"]]
        for g in sc.get("groups", []):
            rows += [_Row(f"{g['id']}#{k}", g["class"], g, group=g["id"]) for k in range(int(g["n"]))]
        return rows

    # ------------------------------------------------------------------ the scenario's states
    def _absolute(self, st: dict) -> np.ndarray:
        """A state of the scenario, in the central body's equatorial axes, in ICRF."""
        if st["type"] == "state":
            r, v = np.asarray(st["r_km"], float), np.asarray(st["v_kms"], float)
        else:
            r, v = elements_to_state(st, self.body.mu)
        return np.concatenate([self.axes @ r, self.axes @ v])

    def _tpos(self, oid: str, t: float) -> Optional[np.ndarray]:
        i = self.idx.get(oid)
        if i is None or not self.active[i]:
            return None
        return self.x[i, :3] + self.x[i, 3:] * (t - self.now)

    def _spawn(self, t: float) -> None:
        if not len(self.pending):
            return
        due = self.t_appear[self.pending] <= t + 1e-9
        if not due.any():
            return
        todo, self.pending = self.pending[due], self.pending[~due]
        x, axes = self.x, self.axes
        for i in todo.tolist():
            w = self.rows[i]
            if w.relative:
                st = w.spec["state"]
                d = frames.unit(float(st["phi_deg"]), float(st["eps_deg"]))
                rs, vs = x[0, :3], x[0, 3:]
                ship = st["frame"] == "ship"
                m = self.flyers[0].m if ship else axes
                los = m @ d
                ref = np.eye(3) if ship else axes
                ax1 = np.cross(los, ref[:, 2])
                if np.linalg.norm(ax1) < 1e-9:
                    ax1 = np.cross(los, ref[:, 0])
                ax1 /= np.linalg.norm(ax1)
                ax2 = np.cross(los, ax1)
                vt = st["v_transverse_mps"]
                x[i, :3] = rs + float(st["range_km"]) * los
                x[i, 3:] = vs + 1e-3 * (-float(st["v_close_mps"]) * los + vt[0] * ax1 + vt[1] * ax2)
            else:
                ref = self.idx[w.spec["around"]]
                if not self.active[ref]:
                    raise scn.ScenarioError(f"group '{w.group}': its reference '{w.spec['around']}' does not exist "
                                            f"at t = {t:g} s")
                g_rng = np.random.default_rng(row_seed(self.seed, SEED_GROUP, w.id))
                x[i, :3] = x[ref, :3] + 1e-3 * float(w.spec["pos_sigma_m"]) * g_rng.standard_normal(3)
                x[i, 3:] = x[ref, 3:] + 1e-3 * float(w.spec["vel_sigma_mps"]) * g_rng.standard_normal(3)
            self.active[i] = self.spawned[i] = True
            self.t_appear[i] = t
            if i in self.flyers:
                self.flyers[i].start(x[i, :3], x[i, 3:], self._tpos)

    def _impulses(self, t: float) -> None:
        lo, hi = np.searchsorted(self.imp_t, t - 1e-9), np.searchsorted(self.imp_t, t + 1e-9, side="right")
        for _t, i, m in self.imp[lo:hi]:
            if self.active[i]:
                dv = 1e-3 * np.asarray(m["dv_mps"], float)
                self.x[i, 3:] += frame_matrix(m["frame"], self.x[i, :3], self.x[i, 3:], self.axes) @ dv

    # ------------------------------------------------------------------ the forces and the integration
    def _accel(self, t0: float, tb: float, xs: np.ndarray, act: np.ndarray) -> np.ndarray:
        """d/dt of the active states xs (k, 6) at tb, during a sub-step that starts at t0 (burns decided at t0)."""
        body, third, n, axes = self.body, self.third, self.n, self.axes
        r, v = xs[:, :3], xs[:, 3:]
        if fast.USE_NUMBA:                            # compiled: gravity, J2, Singer and the other bodies at once
            tpos, tmu = (third.positions(tb), third.mu) if third is not None else (_NO_BODIES, _NO_MU)
            args = (np.ascontiguousarray(r), np.ascontiguousarray(self.a_sing[act]), body.mu, float(body.j2),
                    body.r_eq_km, body.pole, tpos, tmu)
            if len(r) >= PARALLEL_MIN_ROWS:           # C2: on all the cores, the same bits
                with fast.PAR_LOCK:
                    a = fast.gravity_par(*args)
            else:
                a = fast.gravity(*args)
        else:
            a = body.accel(r) + self.a_sing[act]
            if third is not None:
                a = a + third.accel(r, tb)
        rows_act = np.flatnonzero(act)
        b_row = self.b_row
        if len(b_row):                                # the burns running at t0, on the active rows
            pos = np.full(n, -1)
            pos[rows_act] = np.arange(len(rows_act))
            run = (self.b_t0 - 1e-9 <= t0) & (t0 < self.b_t1 - 1e-9) & (pos[b_row] >= 0)
            if run.any():
                ks = pos[b_row[run]]
                acc = self.b_acc[run].copy()
                fr = self.b_frame[run]
                for name in ("rtn", "pnr"):
                    rt = fr == name
                    if rt.any():
                        acc[rt] = np.einsum("nij,nj->ni", frame_matrix(name, r[ks[rt]], v[ks[rt]], axes), acc[rt])
                rt = fr == "inertial"
                acc[rt] = acc[rt] @ axes.T
                np.add.at(a, ks, acc)
        on = np.flatnonzero(self.pn_on[rows_act])
        if len(on):                                   # each guided row toward its target, when the target exists
            where = np.full(n, -1)
            where[rows_act] = np.arange(len(rows_act))
            tg = where[self.pn_tgt[rows_act[on]]]
            on, tg = on[tg >= 0], tg[tg >= 0]
        if len(on) and fast.USE_NUMBA:                # compiled PN command
            fast.pn_command(a, np.ascontiguousarray(r), np.ascontiguousarray(v), on, tg, self.pn_n[rows_act[on]],
                            self.a_max[rows_act[on]])
        elif len(on):
            los = r[tg] - r[on]                                       # object -> target
            vrel = v[tg] - v[on]
            rn2 = np.einsum("ij,ij->i", los, los)
            rn = np.sqrt(rn2)
            omega = np.cross(los, vrel) / rn2[:, None]
            vc = -np.einsum("ij,ij->i", los, vrel) / rn
            cmd = self.pn_n[rows_act[on]][:, None] * vc[:, None] * np.cross(omega, los / rn[:, None])
            nm = np.linalg.norm(cmd, axis=1)
            lim = self.a_max[rows_act[on]]
            cmd *= np.where(nm > lim, lim / np.maximum(nm, 1e-300), 1.0)[:, None]
            a[on] += cmd
        return np.concatenate([v, a], axis=1)

    def _rk4(self, t0: float, h: float) -> None:
        act = self.active.copy()
        if fast.USE_NUMBA and not self._burning(t0, act):     # C3: the whole step compiled, the same bits
            self._rk4_compiled(t0, h, act)
            return
        y = self.x[act]
        k1 = self._accel(t0, t0, y, act)
        k2 = self._accel(t0, t0 + 0.5 * h, y + 0.5 * h * k1, act)
        k3 = self._accel(t0, t0 + 0.5 * h, y + 0.5 * h * k2, act)
        k4 = self._accel(t0, t0 + h, y + h * k3, act)
        self.x[act] = y + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _burning(self, t0: float, act: np.ndarray) -> bool:
        """A burn of an active row runs at t0 (the step then goes the numpy way, _accel)."""
        b_row = self.b_row
        if not len(b_row):
            return False
        return bool(((self.b_t0 - 1e-9 <= t0) & (t0 < self.b_t1 - 1e-9) & act[b_row]).any())

    def _rk4_compiled(self, t0: float, h: float, act: np.ndarray) -> None:
        """_rk4 by fast.rk4_step (C3): the guided rows and their targets, the other bodies at the step's three
        instants and the Singer accelerations gathered once for the four stages (_accel does it at each)."""
        n, third = self.n, self.third
        rows_act = np.flatnonzero(act)
        on = np.flatnonzero(self.pn_on[rows_act])
        if len(on):
            where = np.full(n, -1)
            where[rows_act] = np.arange(len(rows_act))
            tg = where[self.pn_tgt[rows_act[on]]]
            on, tg = on[tg >= 0], tg[tg >= 0]
        else:
            tg = on
        if third is not None:
            tpos3 = np.stack([third.positions(t0), third.positions(t0 + 0.5 * h), third.positions(t0 + h)])
            tmu = third.mu
        else:
            tpos3, tmu = np.zeros((3, 0, 3)), _NO_MU
        b = self.body
        par = len(rows_act) >= PARALLEL_MIN_ROWS
        args = (self.x, rows_act, float(h), tpos3, tmu, b.mu, float(b.j2), b.r_eq_km, b.pole,
                np.ascontiguousarray(self.a_sing[act]), on.astype(np.int64), tg.astype(np.int64),
                np.ascontiguousarray(self.pn_n[rows_act[on]]), np.ascontiguousarray(self.a_max[rows_act[on]]), par)
        if par:
            with fast.PAR_LOCK:
                fast.rk4_step(*args)
        else:
            fast.rk4_step(*args)

    def _pn_update(self, t: float, prev: np.ndarray) -> None:
        """Start PN (time or range), end it when the range opens; miss = closest approach of the step (linear); all
        the guided rows at once."""
        pn_rows, x, active = self.pn_rows, self.x, self.active
        if not len(pn_rows):
            return
        sel = pn_rows[active[pn_rows] & ~self.pn_done[pn_rows] & active[self.pn_tgt[pn_rows]]]   # (the target exists)
        if not len(sel):
            return
        was_on = self.pn_on[sel]
        k_t = np.searchsorted(pn_rows, sel)
        tg = self.pn_tgt[sel]
        if fast.USE_NUMBA:                            # compiled: the same steps row by row
            fast.pn_step(x, prev, sel, tg, was_on, t, self.pn_t[k_t], self.pn_r0[k_t], self.pn_on, self.pn_done,
                         self.t_pn, self.miss)
            return
        los = x[tg, :3] - x[sel, :3]
        off = sel[~was_on]
        if len(off):                                  # not guided yet: start at its time or within its range
            ko = k_t[~was_on]
            rng_km = np.sqrt(np.einsum("ij,ij->i", los[~was_on], los[~was_on]))
            start = np.where(np.isfinite(self.pn_t[ko]), t >= self.pn_t[ko] - 1e-9, rng_km <= self.pn_r0[ko])
            self.pn_on[off[start]] = True
            self.t_pn[off[start]] = t
        g = sel[was_on]
        if len(g):                                    # guided: the step's closest approach, the end when it opens
            d0 = prev[self.pn_tgt[g], :3] - prev[g, :3]
            d1 = los[was_on]
            dd = d1 - d0
            u = np.clip(-np.einsum("ij,ij->i", d0, dd) / np.maximum(np.einsum("ij,ij->i", dd, dd), 1e-300), 0.0, 1.0)
            c = d0 + u[:, None] * dd
            closest = np.sqrt(np.einsum("ij,ij->i", c, c))
            self.miss[g] = np.where(np.isfinite(self.miss[g]), np.minimum(self.miss[g], closest), closest)
            opens = np.einsum("ij,ij->i", d1, x[self.pn_tgt[g], 3:] - x[g, 3:]) > 0.0   # range rate > 0: it opens
            self.pn_on[g[opens]] = False
            self.pn_done[g[opens]] = True

    def _impacts(self, t: float) -> bool:
        """The rows below the surface at t marked impacted; True when the ship is (the run ends: it stays
        active for its last record)."""
        r = np.linalg.norm(self.x[:, :3], axis=1)
        hit = self.active & (r < self.body.radius_km)
        if hit.any():
            self.t_impact[hit] = t
            self.active[hit & (np.arange(self.n) > 0)] = False
        return bool(hit[0])

    # ------------------------------------------------------------------ the record
    def _record(self, j: int, t: float) -> None:
        self.t_log[j] = t
        ex = self.active & (self.t_appear <= t + 1e-9)
        self.r_log[j, ex], self.v_log[j, ex] = self.x[ex, :3], self.x[ex, 3:]
        for jj, i in enumerate(self.att_rows):
            if self.flyers[i].m is not None and self.active[i]:
                self.att_q[j, jj] = self.flyers[i].quat()

    # ------------------------------------------------------------------ checkpoints (L2)
    _STATE = ("x", "active", "spawned", "t_appear", "t_impact", "t_pn", "miss", "pn_on", "pn_done", "a_sing")
    _FLEET = ("M", "W", "phi", "err", "has", "reached", "lost", "flags")

    def checkpoint(self) -> dict:
        """The full state of the fleet now (fork RADARv6_RS, live architecture L2): states, appearances, impacts,
        guidance, Singer and its generators, attitudes (waypoint, slew, events), the clock and the record's length.
        The event pointer, the rows still to appear and the current waypoints' arrays are not kept: restore() takes
        them again from the scenario (an edited one too, L3)."""
        ids = [w.id for w in self.rows]
        ck = {k: getattr(self, k).copy() for k in self._STATE}
        ck.update(ids=ids, t=self.t, k=self.k, now=self.now, j_log=self.j_log, done=self.done,
                  rngs={ids[i]: copy.deepcopy(g.bit_generator.state) for i, g in self.rngs.items()},
                  flyers={ids[i]: (f.k, copy.deepcopy(f.events)) for i, f in self.flyers.items()})
        if self.fleet is not None:
            ck["fleet_ids"] = [ids[f.row] for f in self.fleet.flyers]
            ck["fleet"] = {k: getattr(self.fleet, k).copy() for k in self._FLEET}
        else:
            ck["own"] = {ids[i]: (None if f.m is None else f.m.copy(), np.array(f.w, float).copy(), f.reached,
                                  f.lost, f._err_deg) for i, f in self.flyers.items()}
        return ck

    def checkpoint_before(self, t: float) -> dict:
        """The last checkpoint at or before t (the first one, t = 0, at least)."""
        ts = [c["t"] for c in self.checkpoints]
        return self.checkpoints[max(0, int(np.searchsorted(ts, t + 1e-9, side="right")) - 1)]

    def restore(self, ck: dict, source: Optional["LiveSim"] = None) -> None:
        """Back to the state of the checkpoint ck (one of this run's, or of a run of the same rows): the record and
        the checkpoints after it dropped, the run goes on from there with the same results as in one stretch.
        source: the run ck comes from, when it is another one (L3: the scenario edited after ck); its record and
        checkpoints up to ck are taken over (the past before an edit stays right)."""
        ids = [w.id for w in self.rows]
        here = {oid: i for i, oid in enumerate(ids)}
        old = {oid: i for i, oid in enumerate(ck["ids"])}
        new_rows = np.array([i for i, oid in enumerate(ids) if oid in old], int)     # the rows the checkpoint has
        old_rows = np.array([old[ids[i]] for i in new_rows], int)
        if source is not None and source is not self:
            j = ck["j_log"]
            src = {w.id: i for i, w in enumerate(source.rows)}
            cols = np.array([i for i, oid in enumerate(ids) if oid in src], int)
            scol = np.array([src[ids[i]] for i in cols], int)
            self.t_log[:j] = source.t_log[:j]
            self.r_log[:j] = np.nan
            self.v_log[:j] = np.nan
            self.r_log[:j, cols], self.v_log[:j, cols] = source.r_log[:j, scol], source.v_log[:j, scol]
            satt = {source.rows[i].id: jj for jj, i in enumerate(source.att_rows)}
            self.att_q[:j] = np.nan
            for jj, i in enumerate(self.att_rows):
                if ids[i] in satt:
                    self.att_q[:j, jj] = source.att_q[:j, satt[ids[i]]]
            self.checkpoints = [c for c in source.checkpoints if c["t"] <= ck["t"] + 1e-9]
        # the rows the checkpoint does not have appear after it (sim/live.first_change): as before they exist
        fresh = np.ones(self.n, bool)
        fresh[new_rows] = False
        self.x[fresh] = np.nan
        self.active[fresh] = self.spawned[fresh] = self.pn_on[fresh] = self.pn_done[fresh] = False
        self.t_appear[fresh] = [self.rows[i].t_appear for i in np.flatnonzero(fresh)]
        self.t_impact[fresh] = self.t_pn[fresh] = self.miss[fresh] = np.nan
        self.a_sing[fresh] = 0.0
        for k in self._STATE:
            getattr(self, k)[new_rows] = ck[k][old_rows]
        late = np.array([w.relative or bool(w.group) for w in self.rows], bool) & self.spawned
        spec_t = np.array([w.t_appear for w in self.rows])
        self.t_appear[~late] = spec_t[~late]              # (appearance times of the scenario given, edited ones too)
        self.t, self.k, self.now, self.j_log, self.done = ck["t"], ck["k"], ck["now"], ck["j_log"], ck["done"]
        for oid, state in ck["rngs"].items():
            if oid in here and here[oid] in self.rngs:
                self.rngs[here[oid]].bit_generator.state = copy.deepcopy(state)
        for oid, (k, events) in ck["flyers"].items():
            if oid in here and here[oid] in self.flyers:
                self.flyers[here[oid]].k, self.flyers[here[oid]].events = k, copy.deepcopy(events)
        if self.fleet is not None:
            mine = {ids[f.row]: j for j, f in enumerate(self.fleet.flyers)}
            for jo, oid in enumerate(ck.get("fleet_ids", [])):
                if oid in mine:
                    for k, a in ck["fleet"].items():
                        getattr(self.fleet, k)[mine[oid]] = a[jo]
            for j in range(len(self.fleet.flyers)):
                self.fleet._load(j)
        else:
            for oid, (m, w, reached, lost, err) in ck.get("own", {}).items():
                if oid in here and here[oid] in self.flyers:
                    f = self.flyers[here[oid]]
                    f.m, f.w = (None if m is None else m.copy()), w.copy()
                    f.reached, f.lost, f._err_deg = reached, lost, err
        relative = np.array([w.relative or bool(w.group) for w in self.rows], bool)
        self.pending = np.flatnonzero(relative & ~self.spawned)
        self.e_ptr = int(np.searchsorted(self.events, self.t - 1e-6))   # (the step skips those up to its start)
        self.r_log[self.j_log:] = np.nan                  # the record after the checkpoint dropped
        self.v_log[self.j_log:] = np.nan
        self.att_q[self.j_log:] = np.nan
        self.checkpoints = [c for c in self.checkpoints if c["t"] <= self.t + 1e-9]
        if not self.checkpoints or self.checkpoints[-1]["t"] < self.t - 1e-9:
            self.checkpoints.append(ck)

    # ------------------------------------------------------------------ advancing
    def step(self) -> bool:
        """One step (meta.step_s, split at the events inside it); False once the run is over (end date, ship lost)."""
        if self.done:
            return False
        k, dt, x, active = self.k, self.dt, self.x, self.active
        events = self.events
        t0, t1 = k * dt, min((k + 1) * dt, self.duration)
        prev = x.copy()
        self.now = t0
        if self.fleet is not None:                        # attitudes advanced on the states at the step start
            self.fleet.step(t0, t1 - t0, x, active)
        else:
            for i in self.att_rows:
                if active[i] and self.flyers[i].m is not None:
                    self.flyers[i].step(t0, t1 - t0, x[i, :3], x[i, 3:], self._tpos)
        t = t0
        while self.e_ptr < len(events) and events[self.e_ptr] <= t0 + 1e-9:
            self.e_ptr += 1
        while True:                                       # sub-steps split at the events inside the step
            t_end = events[self.e_ptr] if self.e_ptr < len(events) and events[self.e_ptr] < t1 - 1e-9 else t1
            if t_end > t + 1e-12:
                self._rk4(t, t_end - t)
            t = t_end
            if t >= t1 - 1e-12:
                break
            self.e_ptr += 1
            self._spawn(t)
            self._impulses(t)
            if self._impacts(t):
                break
        self.k = k + 1
        if np.isfinite(self.t_impact[0]):                 # the ship met the surface: the run ends there
            self._record(self.j_log, t)
            self.j_log += 1
            self.t, self.done = t, True
            return False
        self._spawn(t1)
        if self.e_ptr < len(events) and abs(events[self.e_ptr] - t1) < 1e-9:
            self._impulses(t1)
        for i in self.sing:                               # Singer: exact Gauss-Markov update over the step
            s, tau = self.rows[i].singer
            f = math.exp(-(t1 - t0) / tau)
            self.a_sing[i] = f * self.a_sing[i] + s * math.sqrt(1.0 - f * f) * self.rngs[i].standard_normal(3)
        ship_lost = self._impacts(t1)
        self._pn_update(t1, prev)
        if ship_lost or (k + 1) % self.n_log_every == 0 or k == self.n_steps - 1:
            self._record(self.j_log, t1)
            self.j_log += 1
        self.t = t1
        self.done = ship_lost or self.k >= self.n_steps
        if not self.done and self.k % self.n_ck_every == 0:
            self.checkpoints.append(self.checkpoint())
        return not self.done

    def advance_to(self, t: float, progress_cb=None) -> None:
        """Steps until the states reach t (or the run is over)."""
        while not self.done and self.t < t - 1e-9:
            self.step()
            if progress_cb and (self.k - 1) % 1000 == 0:
                progress_cb(self.k - 1, self.n_steps)

    def run(self, progress_cb=None) -> "LiveSim":
        """To the end date (or the ship's impact)."""
        self.advance_to(self.duration, progress_cb)
        return self

    def truth(self) -> Truth:
        """The record so far (views of the record arrays: they grow as the run goes on)."""
        j, rows = self.j_log, self.rows
        return Truth(body=self.body, ids=[w.id for w in rows], classes=[w.cls for w in rows], t_s=self.t_log[:j],
                     r_km=self.r_log[:j], v_kms=self.v_log[:j], t_appear_s=self.t_appear, t_impact_s=self.t_impact,
                     t_pn_start_s=self.t_pn, miss_km=self.miss, group_of=[w.group for w in rows],
                     att_ids=[rows[i].id for i in self.att_rows], att_t_s=self.t_log[:j].copy(),
                     att_q=self.att_q[:j],
                     att_events=[dict(e, id=rows[i].id) for i in self.att_rows for e in self.flyers[i].events])


def propagate(sc: dict, progress_cb=None) -> Truth:
    """The truth of a normalised scenario (sim.scenario.load): LiveSim run to the end."""
    return LiveSim(sc).run(progress_cb).truth()
