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

import math
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
SHIP_MANOEUVRE_CLASS = str(P.get("sim.ship_manoeuvre_class"))   # the ship's pn / singer: this class's bounds
SHIP_MANEUVER_TAU_S = float(P.get("sim.ship_maneuver_tau_s"))   # ... and this Singer time constant
THIRD_BODY = bool(P.get("sim.third_body"))       # O4: attraction of the other bodies
PLANETS_GRAVITY = bool(P.get("sim.planets_gravity"))   # ... the planets' too (user, 29 October 2026)
MOONS_GRAVITY = bool(P.get("sim.moons_gravity"))       # ... and the major moons' (user, 29 October 2026)
SUN_MU = float(P.get("sim.sun_mu_km3_s2"))       # O2
_NO_BODIES, _NO_MU = np.zeros((0, 3)), np.zeros(0)  # no other body (sim.third_body false)
BURN_SUBSTEPS = 20                                 # manoeuvre node preview: a burn in this many RK4 steps at least
THIRD_BODY_STEP_S = 60.0                         # O3: grid of the body positions (cubic between: error << 1 m)


class ThirdBodies:
    """The other bodies seen from the central body over a run: positions on a grid, cubic (4-point Lagrange)
    between its points, and their differential attraction on positions r (O1)."""

    def __init__(self, central: str, epoch, duration_s: float):
        b = scn.bodies()
        other = "moon" if central == "earth" else "earth"
        planets = list(eph.PLANETS) if PLANETS_GRAVITY else []
        moons = list(eph.MOON_NAMES) if MOONS_GRAVITY else []
        self.names = (other, "sun", *planets, *moons)
        self.mu = np.array([float(b[other]["mu_km3_s2"]), SUN_MU] +
                           [float(P.get(f"sim.planets.{p}")["mu_km3_s2"]) for p in planets] +
                           [float(eph._MOONS[m]["mu_km3_s2"]) for m in moons])
        self.t0 = -THIRD_BODY_STEP_S
        n = int(math.ceil(duration_s / THIRD_BODY_STEP_S)) + 4
        ts = self.t0 + THIRD_BODY_STEP_S * np.arange(n)
        pos = np.empty((n, 2 + len(planets) + len(moons), 3))
        for k, t in enumerate(ts):
            T = eph.centuries_tt(epoch, float(t))
            moon, sun = eph.moon_icrf_km(T), eph.sun_icrf_km(T)
            centre = np.zeros(3) if central == "earth" else moon
            pos[k, :2] = (moon - centre if other == "moon" else -centre), sun - centre
            at = {}
            for j, p in enumerate(planets):             # (geocentric, then from the central body)
                at[p] = eph.planet_icrf_km(p, T)
                pos[k, 2 + j] = at[p] - centre
            for j, m in enumerate(moons):
                pl = eph._MOONS[m]["planet"]
                if pl not in at:
                    at[pl] = eph.planet_icrf_km(pl, T)
                pos[k, 2 + len(planets) + j] = at[pl] + eph.satellite_planetocentric_km(m, T) - centre
        self.pos = pos
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

    def force(t: float, rr: np.ndarray, vv: np.ndarray, run) -> np.ndarray:
        a = acc(t, rr[None])[0]
        for m in run:
            a = a + frame_matrix(m["frame"], rr, vv) @ (1e-3 * np.asarray(m["accel_mps2"], float))
        return a

    for a, b in zip(ev[:-1], ev[1:]):
        for j, m in enumerate(manoeuvres):                   # the manoeuvres at a: their state before, the impulses
            if id(m) in mine and abs(float(m["t_s"]) - a) < 1e-9:
                pre[j] = (x.copy(), u.copy())
                if m["type"] == "impulse":
                    u = u + frame_matrix(m["frame"], x, u) @ (1e-3 * np.asarray(m["dv_mps"], float))
        running = [(b0, b1, m) for b0, b1, m in burns if b0 - 1e-9 <= a < b1 - 1e-9]
        run = [m for _b0, _b1, m in running]
        h = step_s if not run else min(step_s, min(b1 - b0 for b0, b1, _m in running) / BURN_SUBSTEPS)
        n = max(1, int(math.ceil((b - a) / max(h, 1e-9) - 1e-9)))
        h = (b - a) / n
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
    """(r km, v km/s) of Keplerian elements a, e, i, raan, argp, nu (closed orbit), ICRF axes."""
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


def pnr_matrix(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Columns X prograde (along v), Y normal (r x v), Z = X x Y radial (in the orbit's plane, outward, normal to v):
    the manoeuvre node's axes (user, 29 October 2026); v_icrf = M v_pnr; r, v (3,) or (n, 3)."""
    x = v / np.linalg.norm(v, axis=-1, keepdims=True)
    y = np.cross(r, v)
    y = y / np.linalg.norm(y, axis=-1, keepdims=True)
    z = np.cross(x, y)
    return np.stack([x, y, z], axis=-1)


def frame_matrix(frame: str, r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """The matrix of a manoeuvre frame ("rtn", "pnr", "inertial") at the state r, v (3,) or (n, 3): v_icrf = M v."""
    if frame == "rtn":
        return rtn_matrix(r, v)
    if frame == "pnr":
        return pnr_matrix(r, v)
    return np.broadcast_to(np.eye(3), np.shape(r)[:-1] + (3, 3)).copy()


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


def propagate(sc: dict, progress_cb=None) -> Truth:
    """The truth of a normalised scenario (sim.scenario.load)."""
    body = Body.of(sc["central_body"])
    third = ThirdBodies(body.name, scn.epoch(sc), float(sc["meta"]["duration_s"])) if THIRD_BODY else None
    duration, dt = float(sc["meta"]["duration_s"]), float(sc["meta"]["step_s"])
    seed = int(sc["meta"]["seed"])
    n_log_every = max(1, int(round(LOG_STEP_S / dt)))
    rows = [_ship_row(sc)] + [_Row(o["id"], o["class"], o) for o in sc["objects"]]
    groups = sc.get("groups", [])
    for g in groups:
        rows += [_Row(f"{g['id']}#{k}", g["class"], g, group=g["id"]) for k in range(int(g["n"]))]
    n = len(rows)
    idx = {w.id: i for i, w in enumerate(rows)}
    x = np.full((n, 6), np.nan)
    active = np.zeros(n, bool)
    spawned = np.zeros(n, bool)
    t_appear = np.array([w.t_appear for w in rows])
    t_impact = np.full(n, np.nan)
    t_pn = np.full(n, np.nan)
    miss = np.full(n, np.nan)
    pn_on = np.zeros(n, bool)
    pn_done = np.zeros(n, bool)

    def absolute(st: dict) -> np.ndarray:
        if st["type"] == "state":
            return np.concatenate([np.asarray(st["r_km"], float), np.asarray(st["v_kms"], float)])
        r, v = elements_to_state(st, body.mu)
        return np.concatenate([r, v])

    x[0] = absolute(sc["ship"]["orbit"])
    active[0] = spawned[0] = True
    for i, w in enumerate(rows[1:], 1):
        if not w.relative and not w.group:
            x[i] = absolute(w.spec["state"])
            active[i] = spawned[i] = True

    # Singer states (km/s2 per inertial axis), one generator per row
    sing = [i for i, w in enumerate(rows) if w.singer is not None]
    rngs = {i: np.random.default_rng([seed, i]) for i in sing}
    a_sing = np.zeros((n, 3))
    for i in sing:
        a_sing[i] = rows[i].singer[0] * rngs[i].standard_normal(3)
    pn_n = np.array([float(w.pn["n"]) if w.pn is not None else 0.0 for w in rows])
    a_max = np.array([w.a_max_kms2 for w in rows])
    burns = [(i, m) for i, w in enumerate(rows) for m in w.burns]
    b_row = np.array([i for i, _m in burns], int)
    b_t0 = np.array([float(m["t_s"]) for _i, m in burns])
    b_t1 = b_t0 + np.array([float(m["duration_s"]) for _i, m in burns])
    b_acc = np.array([1e-3 * np.asarray(m["accel_mps2"], float) for _i, m in burns]).reshape(-1, 3)
    b_frame = np.array([m["frame"] for _i, m in burns], object)
    pn_rows = np.array([i for i, w in enumerate(rows) if w.pn is not None], int)
    pn_t = np.array([float(rows[i].pn["t_s"]) if "t_s" in rows[i].pn else np.nan for i in pn_rows])
    pn_r0 = np.array([float(rows[i].pn.get("start_range_km", np.nan)) for i in pn_rows])
    # each guided row's target (user, 29 October 2026: OWNSHIP's own KKV toward SHIP_1): "target" of its pn
    # manoeuvre, the ship (row 0) by default
    pn_tgt = np.zeros(n, int)
    for i in pn_rows:
        pn_tgt[i] = idx[rows[i].pn.get("target", "ship")]
    flyers = {0: att.Flyer(0, sc["ship"]["attitude"])}
    flyers.update({i: att.Flyer(i, w.spec["attitude"]) for i, w in enumerate(rows) if i and "attitude" in w.spec
                   and not w.group})
    now = [0.0]                                           # time of the states in x (the targets are extrapolated)

    def tpos(oid: str, t: float) -> Optional[np.ndarray]:
        i = idx.get(oid)
        if i is None or not active[i]:
            return None
        return x[i, :3] + x[i, 3:] * (t - now[0])

    # rows appearing later (relative states, group members), spawned in row order once their time has come
    pending = np.array([i for i, w in enumerate(rows) if (w.relative or w.group)], int)

    def spawn(t: float) -> None:
        nonlocal pending
        if not len(pending):
            return
        due = t_appear[pending] <= t + 1e-9
        if not due.any():
            return
        todo, pending = pending[due], pending[~due]
        for i in todo.tolist():
            w = rows[i]
            if w.relative:
                st = w.spec["state"]
                d = frames.unit(float(st["phi_deg"]), float(st["eps_deg"]))
                rs, vs = x[0, :3], x[0, 3:]
                m = flyers[0].m if st["frame"] == "ship" else np.eye(3)
                los = m @ d
                ax1 = np.cross(los, [0.0, 0.0, 1.0])
                if np.linalg.norm(ax1) < 1e-9:
                    ax1 = np.cross(los, [1.0, 0.0, 0.0])
                ax1 /= np.linalg.norm(ax1)
                ax2 = np.cross(los, ax1)
                vt = st["v_transverse_mps"]
                x[i, :3] = rs + float(st["range_km"]) * los
                x[i, 3:] = vs + 1e-3 * (-float(st["v_close_mps"]) * los + vt[0] * ax1 + vt[1] * ax2)
            else:
                ref = idx[w.spec["around"]]
                if not active[ref]:
                    raise scn.ScenarioError(f"group '{w.group}': its reference '{w.spec['around']}' does not exist "
                                            f"at t = {t:g} s")
                g_rng = np.random.default_rng([seed, 100000 + i])
                x[i, :3] = x[ref, :3] + 1e-3 * float(w.spec["pos_sigma_m"]) * g_rng.standard_normal(3)
                x[i, 3:] = x[ref, 3:] + 1e-3 * float(w.spec["vel_sigma_mps"]) * g_rng.standard_normal(3)
            active[i] = spawned[i] = True
            t_appear[i] = t
            if i in flyers:
                flyers[i].start(x[i, :3], x[i, 3:], tpos)

    # impulses sorted by time (row order kept at equal times), found by bisection
    imp = sorted(((float(m["t_s"]), i, m) for i, w in enumerate(rows) for m in w.impulses), key=lambda e: e[0])
    imp_t = np.array([e[0] for e in imp])

    def impulses(t: float) -> None:
        lo, hi = np.searchsorted(imp_t, t - 1e-9), np.searchsorted(imp_t, t + 1e-9, side="right")
        for _t, i, m in imp[lo:hi]:
            if active[i]:
                dv = 1e-3 * np.asarray(m["dv_mps"], float)
                x[i, 3:] += frame_matrix(m["frame"], x[i, :3], x[i, 3:]) @ dv

    def accel(t0: float, tb: float, xs: np.ndarray, act: np.ndarray) -> np.ndarray:
        """d/dt of the active states xs (k, 6) at tb, during a sub-step that starts at t0 (burns decided at t0)."""
        r, v = xs[:, :3], xs[:, 3:]
        if fast.USE_NUMBA:                            # compiled: gravity, J2, Singer and the other bodies at once
            tpos, tmu = (third.positions(tb), third.mu) if third is not None else (_NO_BODIES, _NO_MU)
            a = fast.gravity(np.ascontiguousarray(r), np.ascontiguousarray(a_sing[act]), body.mu, float(body.j2),
                             body.r_eq_km, body.pole, tpos, tmu)
        else:
            a = body.accel(r) + a_sing[act]
            if third is not None:
                a = a + third.accel(r, tb)
        rows_act = np.flatnonzero(act)
        if len(b_row):                                # the burns running at t0, on the active rows
            pos = np.full(n, -1)
            pos[rows_act] = np.arange(len(rows_act))
            run = (b_t0 - 1e-9 <= t0) & (t0 < b_t1 - 1e-9) & (pos[b_row] >= 0)
            if run.any():
                ks = pos[b_row[run]]
                acc = b_acc[run].copy()
                fr = b_frame[run]
                for name in ("rtn", "pnr"):
                    rt = fr == name
                    if rt.any():
                        acc[rt] = np.einsum("nij,nj->ni", frame_matrix(name, r[ks[rt]], v[ks[rt]]), acc[rt])
                np.add.at(a, ks, acc)
        on = np.flatnonzero(pn_on[rows_act])
        if len(on):                                   # each guided row toward its target, when the target exists
            where = np.full(n, -1)
            where[rows_act] = np.arange(len(rows_act))
            tg = where[pn_tgt[rows_act[on]]]
            on, tg = on[tg >= 0], tg[tg >= 0]
        if len(on) and fast.USE_NUMBA:                # compiled PN command
            fast.pn_command(a, np.ascontiguousarray(r), np.ascontiguousarray(v), on, tg, pn_n[rows_act[on]],
                            a_max[rows_act[on]])
        elif len(on):
            los = r[tg] - r[on]                                       # object -> target
            vrel = v[tg] - v[on]
            rn2 = np.einsum("ij,ij->i", los, los)
            rn = np.sqrt(rn2)
            omega = np.cross(los, vrel) / rn2[:, None]
            vc = -np.einsum("ij,ij->i", los, vrel) / rn
            cmd = pn_n[rows_act[on]][:, None] * vc[:, None] * np.cross(omega, los / rn[:, None])
            nm = np.linalg.norm(cmd, axis=1)
            lim = a_max[rows_act[on]]
            cmd *= np.where(nm > lim, lim / np.maximum(nm, 1e-300), 1.0)[:, None]
            a[on] += cmd
        return np.concatenate([v, a], axis=1)

    def rk4(t0: float, h: float) -> None:
        act = active.copy()
        y = x[act]
        k1 = accel(t0, t0, y, act)
        k2 = accel(t0, t0 + 0.5 * h, y + 0.5 * h * k1, act)
        k3 = accel(t0, t0 + 0.5 * h, y + 0.5 * h * k2, act)
        k4 = accel(t0, t0 + h, y + h * k3, act)
        x[act] = y + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def pn_update(t: float, prev: np.ndarray) -> None:
        """Start PN (time or range), end it when the range opens; miss = closest approach of the step (linear); all
        the guided rows at once."""
        if not len(pn_rows):
            return
        sel = pn_rows[active[pn_rows] & ~pn_done[pn_rows] & active[pn_tgt[pn_rows]]]   # (the target exists)
        if not len(sel):
            return
        was_on = pn_on[sel]
        k_t = np.searchsorted(pn_rows, sel)
        tg = pn_tgt[sel]
        if fast.USE_NUMBA:                            # compiled: the same steps row by row
            fast.pn_step(x, prev, sel, tg, was_on, t, pn_t[k_t], pn_r0[k_t], pn_on, pn_done, t_pn, miss)
            return
        los = x[tg, :3] - x[sel, :3]
        off = sel[~was_on]
        if len(off):                                  # not guided yet: start at its time or within its range
            ko = k_t[~was_on]
            rng_km = np.sqrt(np.einsum("ij,ij->i", los[~was_on], los[~was_on]))
            start = np.where(np.isfinite(pn_t[ko]), t >= pn_t[ko] - 1e-9, rng_km <= pn_r0[ko])
            pn_on[off[start]] = True
            t_pn[off[start]] = t
        g = sel[was_on]
        if len(g):                                    # guided: the step's closest approach, the end when it opens
            d0 = prev[pn_tgt[g], :3] - prev[g, :3]
            d1 = los[was_on]
            dd = d1 - d0
            u = np.clip(-np.einsum("ij,ij->i", d0, dd) / np.maximum(np.einsum("ij,ij->i", dd, dd), 1e-300), 0.0, 1.0)
            c = d0 + u[:, None] * dd
            closest = np.sqrt(np.einsum("ij,ij->i", c, c))
            miss[g] = np.where(np.isfinite(miss[g]), np.minimum(miss[g], closest), closest)
            opens = np.einsum("ij,ij->i", d1, x[pn_tgt[g], 3:] - x[g, 3:]) > 0.0   # range rate > 0: it opens
            pn_on[g[opens]] = False
            pn_done[g[opens]] = True

    def impacts(t: float) -> bool:
        """The rows below the surface at t marked impacted; True when the ship is (the run ends: it stays
        active for its last record)."""
        r = np.linalg.norm(x[:, :3], axis=1)
        hit = active & (r < body.radius_km)
        if hit.any():
            t_impact[hit] = t
            active[hit & (np.arange(n) > 0)] = False
        return bool(hit[0])

    events = _event_times(rows, groups, duration)
    n_steps = int(math.ceil(duration / dt - 1e-9))
    n_log = n_steps // n_log_every + 1 + (1 if n_steps % n_log_every else 0)
    t_log = np.empty(n_log)
    r_log = np.full((n_log, n, 3), np.nan)
    v_log = np.full((n_log, n, 3), np.nan)

    att_rows = sorted(flyers)
    att_q = np.full((n_log, len(att_rows), 4), np.nan)    # at the recorded instants (sim.log_step_s)

    def record_att(j: int) -> None:
        for jj, i in enumerate(att_rows):
            if flyers[i].m is not None and active[i]:
                att_q[j, jj] = flyers[i].quat()

    def record(j: int, t: float) -> None:
        t_log[j] = t
        ex = active & (t_appear <= t + 1e-9)
        r_log[j, ex], v_log[j, ex] = x[ex, :3], x[ex, 3:]
        record_att(j)

    for i in att_rows:
        if spawned[i]:
            flyers[i].start(x[i, :3], x[i, 3:], tpos)
    spawn(0.0)
    for i in att_rows:                                    # an initial attitude aimed at an object spawned at t = 0
        if flyers[i].events and flyers[i].events[-1]["kind"] == "target missing" and active[i]:
            flyers[i].events.pop()
            flyers[i].start(x[i, :3], x[i, 3:], tpos)
    impulses(0.0)
    pn_update(0.0, x.copy())
    record(0, 0.0)
    j_log, e_ptr = 1, 0
    for k in range(n_steps):
        t0, t1 = k * dt, min((k + 1) * dt, duration)
        prev = x.copy()
        now[0] = t0
        for i in att_rows:                                # attitudes advanced on the states at the step start
            if active[i] and flyers[i].m is not None:
                flyers[i].step(t0, t1 - t0, x[i, :3], x[i, 3:], tpos)
        t = t0
        while e_ptr < len(events) and events[e_ptr] <= t0 + 1e-9:
            e_ptr += 1
        while True:                                       # sub-steps split at the events inside the step
            t_end = events[e_ptr] if e_ptr < len(events) and events[e_ptr] < t1 - 1e-9 else t1
            if t_end > t + 1e-12:
                rk4(t, t_end - t)
            t = t_end
            if t >= t1 - 1e-12:
                break
            e_ptr += 1
            spawn(t)
            impulses(t)
            if impacts(t):
                break
        if np.isfinite(t_impact[0]):                      # the ship met the surface: the run ends there
            record(j_log, t)
            j_log += 1
            break
        spawn(t1)
        if e_ptr < len(events) and abs(events[e_ptr] - t1) < 1e-9:
            impulses(t1)
        for i in sing:                                    # Singer: exact Gauss-Markov update over the step
            s, tau = rows[i].singer
            f = math.exp(-(t1 - t0) / tau)
            a_sing[i] = f * a_sing[i] + s * math.sqrt(1.0 - f * f) * rngs[i].standard_normal(3)
        ship_lost = impacts(t1)
        pn_update(t1, prev)
        if ship_lost or (k + 1) % n_log_every == 0 or k == n_steps - 1:
            record(j_log, t1)
            j_log += 1
        if ship_lost:
            break
        if progress_cb and k % 1000 == 0:
            progress_cb(k, n_steps)
    return Truth(body=body, ids=[w.id for w in rows], classes=[w.cls for w in rows], t_s=t_log[:j_log],
                 r_km=r_log[:j_log], v_kms=v_log[:j_log], t_appear_s=t_appear, t_impact_s=t_impact,
                 t_pn_start_s=t_pn, miss_km=miss, group_of=[w.group for w in rows],
                 att_ids=[rows[i].id for i in att_rows], att_t_s=t_log[:j_log].copy(),
                 att_q=att_q[:j_log], att_events=[dict(e, id=rows[i].id) for i in att_rows for e in flyers[i].events])
