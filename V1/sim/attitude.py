#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ATTITUDE -- attitudes flown by waypoints (roadmap step 6.3, pass S3a; framing G1-G2 validated by the user on
28 October 2026). The ownship, and the objects of classes SHIP and KKV, fly a list of waypoints:

  WAYPOINT  {"t_s": start of the slew (the first point: 0, the initial attitude, no slew),
             "rate_dps": slew rate (not on the first point; at most repointing.rate_max_dps),
             "attitude": ATTITUDE}
  ATTITUDE  {"type": "fixed", "frame": "orbital" | "inertial", "yaw_deg", "pitch_deg", "roll_deg"}
          | {"type": "prograde" | "retrograde" | "normal+" | "normal-" | "radial_out" | "radial_in", "roll_deg"}
          | {"type": "target", "id": object id ("ship" for an object aiming at the ownship), "roll_deg"}

Axes of a flying body (user): Z the roll axis (the nose; the four radar faces of the ownship lie around it, the ship
frame of step 3.0), the XZ plane its "horizontal" (roll 0 = wings flat), the attitude measured against the orbital
plane:
  - orbital reference (yaw = pitch = roll = 0): Z along track T = N x R, Y = the orbit normal N = r x v, X = -R
    (toward the central body); the XZ plane is the orbital plane;
  - angles in the aircraft order: yaw about Y (positive: nose toward +X), then pitch about the new X (positive: nose
    up, toward +Y), then roll about the new Z (positive: +X wing down); "inertial": the same angles from the ICRF axes;
  - a direction attitude (prograde = +v, normal+ = +N, radial_out = +R, target = line of sight, ...) puts Z along the
    direction and X in the orbital plane (X = N x Z; for a direction along the normal, X along the velocity), then
    roll_deg about Z.
Slews (G1): from t_s the body turns toward the waypoint's attitude, then holds it (follows it when it moves) until the
next point. One law at every step: a follower limited in angular acceleration (repointing.accel_max_dps2) and in rate
(the point's rate_dps until the attitude is reached, then repointing.rate_max_dps to follow it), with the attitude's
own rate fed forward: toward a fixed attitude it is the trapezoid, toward a moving one it follows and saturates by
itself. Logged events: a slew not finished when the next point starts, a held attitude lost by more than
HOLD_TOL_DEG (rate or acceleration limit), a target that does not exist. SHIP and KKV use the ownship's limits until
classes give their own (user, 28 October 2026).
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from core import frames
from core.params import P

TYPES = ("fixed", "prograde", "retrograde", "normal+", "normal-", "radial_out", "radial_in", "target")
FIXED_FRAMES = ("orbital", "inertial")
FLYING_CLASSES = ("SHIP", "KKV")                    # object classes that may fly waypoints (user)
RATE_MAX_DPS = float(P.get("repointing.rate_max_dps"))
ACCEL_MAX_DPS2 = float(P.get("repointing.accel_max_dps2"))
REACHED_TOL_DEG = 0.01                              # the slew is over below this error (then the rate limit is the max)
HOLD_TOL_DEG = 0.1                                  # a held attitude farther than this is logged


# ----------------------------------------------------------------------------- rotations
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def euler(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Body -> reference matrix of yaw (about Y, nose toward +X), pitch (about X, nose up toward +Y), roll (about Z,
    +X wing down), in that order."""
    return rot_y(math.radians(yaw_deg)) @ rot_x(-math.radians(pitch_deg)) @ rot_z(-math.radians(roll_deg))


def rotvec(m: np.ndarray) -> np.ndarray:
    """Rotation vector (axis x angle, rad) of a rotation matrix."""
    c = max(-1.0, min(1.0, (np.trace(m) - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return 0.5 * np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    if a > math.pi - 1e-6:                                   # near 180 deg: axis from the symmetric part
        b = (m + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(b)))
        ax = b[:, k] / math.sqrt(max(b[k, k], 1e-300))
        return ax * a
    return a / (2.0 * math.sin(a)) * np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])


def expm(w: np.ndarray) -> np.ndarray:
    """Rotation matrix of a rotation vector (Rodrigues)."""
    a = float(np.linalg.norm(w))
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * (kx @ kx)


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle between two attitudes (deg)."""
    return math.degrees(float(np.linalg.norm(rotvec(a @ b.T))))


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    d = float(q0 @ q1)
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1.0 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


# ----------------------------------------------------------------------------- reference and target attitudes
def orbital_axes(r: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(R, T, N) unit vectors: radial, along track N x R, orbit normal r x v."""
    rr = r / np.linalg.norm(r)
    n = np.cross(r, v)
    n = n / np.linalg.norm(n)
    return rr, np.cross(n, rr), n


def orbital_reference(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Body -> ICRF matrix of the orbital reference attitude: X = -R, Y = N, Z = T."""
    rr, t, n = orbital_axes(r, v)
    return np.stack([-rr, n, t], axis=1)


def pointing(d: np.ndarray, r: np.ndarray, v: np.ndarray, roll_deg: float) -> np.ndarray:
    """Body -> ICRF matrix with Z along d, X in the orbital plane (N x Z; along the velocity when d is along N), then
    roll_deg about Z."""
    rr, t, n = orbital_axes(r, v)
    z = d / np.linalg.norm(d)
    x = np.cross(n, z)
    if np.linalg.norm(x) < 1e-6:
        x = t - (t @ z) * z
    x = x / np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1) @ rot_z(-math.radians(roll_deg))


def target_attitude(att: dict, r: np.ndarray, v: np.ndarray, target_pos: Optional[np.ndarray]) -> Optional[np.ndarray]:
    """Body -> ICRF matrix of an attitude for a body at (r, v); target_pos: the target's position for "target"
    (None: the target does not exist, no attitude)."""
    t = att["type"]
    if t == "fixed":
        e = euler(att["yaw_deg"], att["pitch_deg"], att["roll_deg"])
        return orbital_reference(r, v) @ e if att["frame"] == "orbital" else e
    rr, _, n = orbital_axes(r, v)
    d = {"prograde": v, "retrograde": -v, "normal+": n, "normal-": -n, "radial_out": rr, "radial_in": -rr}.get(t)
    if t == "target":
        if target_pos is None:
            return None
        d = target_pos - r
    return pointing(d, r, v, att["roll_deg"])


# ----------------------------------------------------------------------------- format
def validate(wps, tag: str, err: List[str], duration: float, targets: Dict[str, float], self_id: str) -> None:
    """Problems of a waypoint list (targets: id -> appearance time of the objects that may be aimed at)."""
    if not isinstance(wps, list) or not wps:
        err.append(f"{tag}: a list of waypoints is needed ({{t_s, rate_dps, attitude}}, the first at t_s = 0)")
        return
    prev = -1.0
    for i, w in enumerate(wps):
        tg = f"{tag}[{i}]"
        if not isinstance(w, dict):
            err.append(f"{tg}: a dict is needed")
            continue
        t = w.get("t_s")
        if not isinstance(t, (int, float)) or isinstance(t, bool) or not (0.0 <= float(t) <= max(duration, 0.0)):
            err.append(f"{tg}: t_s must lie in [0, duration]")
            t = prev
        elif float(t) <= prev:
            err.append(f"{tg}: t_s must increase strictly")
        if i == 0 and t != 0.0 and isinstance(t, (int, float)):
            err.append(f"{tg}: the first waypoint is the initial attitude, at t_s = 0")
        if i > 0:
            rt = w.get("rate_dps")
            if not isinstance(rt, (int, float)) or isinstance(rt, bool) or not 0.0 < float(rt) <= RATE_MAX_DPS:
                err.append(f"{tg}: rate_dps must lie in ]0, {RATE_MAX_DPS:g}] (repointing.rate_max_dps)")
        prev = float(t) if isinstance(t, (int, float)) else prev
        a = w.get("attitude")
        if not isinstance(a, dict) or a.get("type") not in TYPES:
            err.append(f"{tg}.attitude: type must be one of {', '.join(TYPES)}")
            continue
        for k in ("yaw_deg", "pitch_deg", "roll_deg") if a["type"] == "fixed" else ("roll_deg",):
            if k in a and (not isinstance(a[k], (int, float)) or isinstance(a[k], bool) or not math.isfinite(a[k])):
                err.append(f"{tg}.attitude: {k} must be a number")
        if a["type"] == "fixed" and a.get("frame", "orbital") not in FIXED_FRAMES:
            err.append(f"{tg}.attitude: frame must be {' or '.join(FIXED_FRAMES)}")
        if a["type"] == "target":
            oid = a.get("id")
            if oid == self_id or oid not in targets:
                err.append(f"{tg}.attitude: target '{oid}' is not an object of the scenario"
                           + (" (itself)" if oid == self_id else ""))
            elif isinstance(t, (int, float)) and targets[oid] > float(t) + 1e-9:
                err.append(f"{tg}.attitude: target '{oid}' appears at {targets[oid]:g} s, after this waypoint")


def normalise(wps: List[dict]) -> None:
    for w in wps:
        a = w["attitude"]
        a.setdefault("roll_deg", 0.0)
        if a["type"] == "fixed":
            a.setdefault("frame", "orbital")
            a.setdefault("yaw_deg", 0.0)
            a.setdefault("pitch_deg", 0.0)


DEFAULT_SHIP = [{"t_s": 0.0, "attitude": {"type": "prograde", "roll_deg": 0.0}}]


# ----------------------------------------------------------------------------- the follower
class Flyer:
    """The attitude of one body along its waypoints, advanced step by step (step())."""

    def __init__(self, row: int, wps: List[dict]):
        self.row, self.wps = row, wps
        self.k = 0                                      # current waypoint
        self.m: Optional[np.ndarray] = None             # body -> ICRF
        self.w = np.zeros(3)                            # angular velocity (rad/s, ICRF)
        self.reached = True
        self.lost = False
        self._err_deg = 0.0
        self.events: List[dict] = []

    def _goal(self, k: int, r, v, tpos) -> Optional[np.ndarray]:
        return target_attitude(self.wps[k]["attitude"], r, v, tpos)

    def start(self, r: np.ndarray, v: np.ndarray, tpos: Callable[[str, float], Optional[np.ndarray]]) -> None:
        """The initial attitude (first waypoint) at t = 0."""
        g = self._goal(0, r, v, self._tp(0, tpos, 0.0))
        self.m = g if g is not None else np.eye(3)
        if g is None:
            self.events.append({"t_s": 0.0, "kind": "target missing", "waypoint": 0})

    def _tp(self, k: int, tpos, t: float):
        a = self.wps[k]["attitude"]
        return tpos(a["id"], t) if a["type"] == "target" else None

    def step(self, t0: float, dt: float, r: np.ndarray, v: np.ndarray, tpos) -> None:
        """Advance from t0 to t0 + dt; (r, v) the body's state at t0; tpos(id, t) a target's position (None: absent)."""
        nxt = self.k + 1
        if nxt < len(self.wps) and t0 >= float(self.wps[nxt]["t_s"]) - 1e-9:
            if not self.reached:
                self.events.append({"t_s": t0, "kind": "slew not finished", "waypoint": self.k,
                                    "error_deg": self._err_deg})
            self.k, self.reached, self.lost = nxt, False, False
        g0 = self._goal(self.k, r, v, self._tp(self.k, tpos, t0))
        g1 = self._goal(self.k, r + v * dt, v, self._tp(self.k, tpos, t0 + dt))
        if g0 is None or g1 is None:
            self.events.append({"t_s": t0, "kind": "target missing", "waypoint": self.k})
            self.w = np.zeros(3)
            return
        w_goal = rotvec(g1 @ g0.T) / dt                               # the attitude's own rate, fed forward
        e = rotvec(g0 @ self.m.T)
        phi = float(np.linalg.norm(e))
        self._err_deg = math.degrees(phi)
        a_max = math.radians(ACCEL_MAX_DPS2)
        cap = math.radians(RATE_MAX_DPS if self.reached or self.k == 0 else float(self.wps[self.k]["rate_dps"]))
        # the slew rate that still stops on the attitude after this step: w^2 = 2 a (phi - w dt / 2) (discrete time)
        h = 0.5 * a_max * dt
        sl = min(cap, -h + math.sqrt(h * h + 2.0 * a_max * phi), phi / dt) if phi > 0.0 else 0.0
        w_des = w_goal + (e / phi * sl if phi > 0.0 else 0.0)
        nd = float(np.linalg.norm(w_des))
        w_cap = math.radians(RATE_MAX_DPS)
        if nd > w_cap:
            w_des *= w_cap / nd
        dw = w_des - self.w
        ndw = float(np.linalg.norm(dw))
        if ndw > a_max * dt:
            dw *= a_max * dt / ndw
        self.w = self.w + dw
        self.m = expm(self.w * dt) @ self.m
        err = angle_deg(g1, self.m)
        if not self.reached and err < REACHED_TOL_DEG:
            self.reached = True
        elif self.reached and err > HOLD_TOL_DEG and not self.lost:
            self.lost = True
            self.events.append({"t_s": t0 + dt, "kind": "hold lost (rate or acceleration limit)", "waypoint": self.k,
                                "error_deg": err})
        elif self.lost and err < REACHED_TOL_DEG:
            self.lost = False
            self.events.append({"t_s": t0 + dt, "kind": "hold recovered", "waypoint": self.k})

    def quat(self) -> np.ndarray:
        return np.asarray(frames.matrix_to_quat(self.m))
