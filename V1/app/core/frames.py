#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FRAMES -- frames of the watch volumes and navigation message (roadmap step 3.0, RADAR COMPUTER layer).

Architecture (Documentation/Feuille_de_route.md, step 3.0): the radar computer knows only its navigation (ship attitude,
own orbit, central body) and its own detections, never the truth of the targets. It steers the volumes with the
navigation: every sector is turned into the SHIP frame here, once, before any computation; everything downstream
(optimizer, Ku, sky, structure, 3D views) stays in the ship frame. In mission prep the navigation is a snapshot typed
by the operator (later the global simulator's navigation message).

Navigation, global key "nav": a LIST of messages (one in mission preparation; the attitude envelope is phase 6):
  {"label": str, "epoch": ISO 8601 UTC (optional),
   "q_ship_to_icrf": [w, x, y, z]   Hamilton, scalar first: v_icrf = R(q) v_ship,
   "central_body": a type of [sky.surface], "r_km": [x, y, z], "v_kms": [x, y, z]
                    (optional orbit: ship position and velocity from the body centre, ICRF axes),
   "attitude_ref": "icrf" or "rtn" (optional, display only: the frame of the yaw / pitch / roll shown by the GUI)}
  No "nav" = identity attitude, no orbit.

Sector key "frame" (absent = "ship"); angles in every frame: phi about Z from +X toward +Y, eps from the XY plane
toward +Z; box and cap shapes are described in the sector's frame:
  ship     the ship frame (volumes tied to the ship, e.g. self-protection)
  inertial ICRF axes (X vernal equinox, Z celestial pole): phi = right ascension, eps = declination
  rtn      X radial outward, Z = r x v (orbit normal), Y = Z x X (along track); needs the orbit
  limb     Z local zenith, X = v projected on the local horizontal, Y = Z x X; box only, eps COUNTED FROM THE LIMB of
           the central body at altitude limb_alt_km (sector key, default tool.frames.limb_alt_km): the limb is
           acos((R + limb_alt) / |r|) below the local horizontal and the band follows the altitude; needs the orbit
  track    (step 3.3, F8) a cap around a track or an external designation: centre track_ra_deg / track_dec_deg (ICRF),
           radius tracking.track_frame_k x track_sigma_deg (the designation's angular sigma); the cap's own
           phi0_deg / eps0_deg / radius_deg are set from them
  two_dir  Z = primary direction, X in the plane of the two directions on the secondary's side; directions in ICRF
           (frame_p_ra_deg, frame_p_dec_deg, frame_s_ra_deg, frame_s_dec_deg)

Sky key "frame": "ship" (default) or "inertial": an inertial sky (bodies, galactic frame, stand-off jammers) is turned
into the ship frame with the same attitude.

resolve(mission) returns a copy in the ship frame: each non-ship sector gets "frame_m" (the 3x3 matrix ship -> sector
frame, 9 floats row by row) and "frame_eps_offset_deg" (limb), read by core.optimizer.Sphere; an identity matrix is
dropped (the sector is then masked exactly as a ship-frame one).
"""
from __future__ import annotations

import copy
import math
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .params import P

FRAMES = ("ship", "inertial", "rtn", "limb", "two_dir", "track")
TRACK_KEYS = ("track_ra_deg", "track_dec_deg", "track_sigma_deg")       # step 3.3, F8
SKY_FRAMES = ("ship", "inertial")
ORBIT_FRAMES = ("rtn", "limb")
ATTITUDE_REFS = ("icrf", "rtn")                  # frames of the yaw / pitch / roll shown by the GUI
TWO_DIR_KEYS = ("frame_p_ra_deg", "frame_p_dec_deg", "frame_s_ra_deg", "frame_s_dec_deg")
SECTOR_FRAME_FIELDS = ("frame", "limb_alt_km") + TWO_DIR_KEYS + TRACK_KEYS   # the operator's frame inputs of a sector
RESOLVED_FIELDS = ("frame_m", "frame_eps_offset_deg")              # added by resolve_sector()
_T = "tool.frames."
Q_NORM_TOL: float = P.get(_T + "q_norm_tol")                 # |q| - 1 beyond this: refused; within: normalised
TWO_DIR_MIN_SEP_DEG: float = P.get(_T + "two_dir_min_sep_deg")
TRACK_FRAME_K: float = P.get("tracking.track_frame_k")
SKY_NAV_TOL_DEG: float = P.get(_T + "sky_nav_tol_deg")       # sky central body vs navigation: alert beyond
LIMB_ALT_KM_DEFAULT: float = P.get(_T + "limb_alt_km")
BODY_RADIUS_KM: Dict[str, float] = {k: float(v["radius_km"]) for k, v in P.section("sky.surface").items()}
# the simulator's other bodies too (user, 30 October 2026: a scenario around any celestial body exports its mission):
# the Sun, the planets, the major moons; their radar surface (clutter) is not modelled yet, the sky.surface types are
BODY_RADIUS_KM.setdefault("sun", float(P.get("sim.sun_radius_km")))
for _sec in ("sim.planets", "sim.moons"):
    for _k, _v in P.section(_sec).items():
        BODY_RADIUS_KM.setdefault(_k, float(dict(_v)["radius_km"]))


# ----------------------------------------------------------------------------- rotations
def unit(phi_deg: float, eps_deg: float) -> np.ndarray:
    p, e = math.radians(phi_deg), math.radians(eps_deg)
    return np.array([math.cos(e) * math.cos(p), math.cos(e) * math.sin(p), math.sin(e)])


def angles(d: np.ndarray) -> Tuple[float, float]:
    """(phi, eps) in degrees of a unit vector, phi in [0, 360)."""
    phi = math.degrees(math.atan2(d[1], d[0])) % 360.0
    return (0.0 if phi >= 360.0 else phi), math.degrees(math.asin(max(-1.0, min(1.0, d[2]))))


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """Rotation matrix of a unit Hamilton quaternion [w, x, y, z]: v' = R v."""
    w, x, y, z = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(m: np.ndarray) -> List[float]:
    """Unit quaternion [w, x, y, z] (w >= 0) of a rotation matrix (Shepperd)."""
    t = np.trace(m)
    if t > 0:
        s = 2.0 * math.sqrt(1.0 + t)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    if q[0] < 0:
        q = [-c for c in q]
    return [float(c) for c in q]


def euler_321(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """R_ref<-ship of the 3-2-1 sequence (yaw about Z, then pitch about Y, then roll about X)."""
    a, b, c = (math.radians(v) for v in (yaw_deg, pitch_deg, roll_deg))
    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    rx = np.array([[1, 0, 0], [0, math.cos(c), -math.sin(c)], [0, math.sin(c), math.cos(c)]])
    return rz @ ry @ rx


def euler_321_of(m: np.ndarray) -> Tuple[float, float, float]:
    """(yaw, pitch, roll) in degrees of R_ref<-ship (inverse of euler_321; pitch in [-90, 90])."""
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -m[2, 0]))))
    return (math.degrees(math.atan2(m[1, 0], m[0, 0])), pitch, math.degrees(math.atan2(m[2, 1], m[2, 2])))


def _axes(z: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
    """Rows (X, Y, Z) of a frame: Z along z, X = x_hint made orthogonal to Z, Y = Z x X (matrix ICRF -> frame)."""
    z = z / np.linalg.norm(z)
    x = x_hint - (x_hint @ z) * z
    x = x / np.linalg.norm(x)
    return np.vstack([x, np.cross(z, x), z])


# ----------------------------------------------------------------------------- navigation
def _vec3(v) -> bool:
    return isinstance(v, (list, tuple)) and len(v) == 3 and all(isinstance(c, (int, float)) for c in v)


def validate_nav(nav) -> List[str]:
    """Operator-facing messages of the navigation message (empty = valid)."""
    if nav is None:
        return []
    if not isinstance(nav, list) or not nav:
        return ["nav must be a list of navigation messages (one)"]
    if len(nav) > 1:
        return [f"nav gives {len(nav)} navigation messages: one only in mission preparation (the attitude envelope "
                f"is for the scenario simulator, roadmap phase 6)"]
    n = nav[0]
    tag = "navigation" + (f" ({n.get('label')})" if isinstance(n, dict) and n.get("label") else "")
    if not isinstance(n, dict):
        return [f"{tag}: must be an object"]
    errs = []
    q = n.get("q_ship_to_icrf")
    if not (isinstance(q, (list, tuple)) and len(q) == 4 and all(isinstance(c, (int, float)) for c in q)):
        errs.append(f"{tag}: q_ship_to_icrf must be a quaternion [w, x, y, z]")
    elif abs(math.sqrt(sum(c * c for c in q)) - 1.0) > Q_NORM_TOL:
        errs.append(f"{tag}: |q_ship_to_icrf| = {math.sqrt(sum(c * c for c in q)):.6f}, not a unit quaternion "
                    f"(tolerance {Q_NORM_TOL:g})")
    if n.get("attitude_ref", "icrf") not in ATTITUDE_REFS:
        errs.append(f"{tag}: attitude_ref '{n['attitude_ref']}' unknown ({', '.join(ATTITUDE_REFS)})")
    if n.get("epoch") is not None:
        try:
            datetime.fromisoformat(str(n["epoch"]).replace("Z", "+00:00"))
        except ValueError:
            errs.append(f"{tag}: epoch '{n['epoch']}' is not an ISO 8601 date")
    orbit = [k for k in ("central_body", "r_km", "v_kms") if n.get(k) is not None]
    if orbit and len(orbit) < 3:
        errs.append(f"{tag}: the orbit needs central_body, r_km and v_kms together")
    elif not orbit and n.get("attitude_ref") == "rtn":
        errs.append(f"{tag}: attitude_ref 'rtn' needs the orbit (central_body, r_km, v_kms)")
    if orbit and len(orbit) == 3:
        if n["central_body"] not in BODY_RADIUS_KM:
            errs.append(f"{tag}: central_body '{n['central_body']}' is not a known body (sky.surface, sim.planets, "
                        f"sim.moons: {', '.join(BODY_RADIUS_KM)})")
        elif not (_vec3(n["r_km"]) and _vec3(n["v_kms"])):
            errs.append(f"{tag}: r_km and v_kms must be vectors [x, y, z]")
        else:
            r, v = np.array(n["r_km"], float), np.array(n["v_kms"], float)
            if np.linalg.norm(r) <= BODY_RADIUS_KM[n["central_body"]]:
                errs.append(f"{tag}: |r_km| ({np.linalg.norm(r):.1f} km) is inside the {n['central_body']} "
                            f"(radius {BODY_RADIUS_KM[n['central_body']]:g} km)")
            elif np.linalg.norm(v) == 0 or np.linalg.norm(np.cross(r, v)) <= 1e-9 * np.linalg.norm(r) * np.linalg.norm(v):
                errs.append(f"{tag}: v_kms is zero or along r_km: no orbit plane")
    return errs


class Nav:
    """Parsed navigation message: R (ship -> ICRF), orbit (r, v in km, km/s, body, body radius) or None."""

    def __init__(self, msg: Optional[dict] = None):
        msg = msg or {}
        self.label, self.epoch = msg.get("label"), msg.get("epoch")
        q = np.array(msg.get("q_ship_to_icrf", [1.0, 0.0, 0.0, 0.0]), float)
        self.q = q / np.linalg.norm(q)
        self.r_si = quat_to_matrix(self.q)
        self.body = msg.get("central_body")
        self.r = np.array(msg["r_km"], float) if self.body else None
        self.v = np.array(msg["v_kms"], float) if self.body else None
        self.body_radius_km = BODY_RADIUS_KM.get(self.body) if self.body else None

    @property
    def has_orbit(self) -> bool:
        return self.body is not None

    def rtn(self) -> np.ndarray:
        """Matrix ICRF -> RTN."""
        return _axes(np.cross(self.r, self.v), self.r)

    def limb_axes(self) -> np.ndarray:
        """Matrix ICRF -> local horizon frame (Z zenith, X along the horizontal velocity)."""
        return _axes(self.r, self.v)

    def limb_depression_deg(self, alt_km: float) -> float:
        return math.degrees(math.acos((self.body_radius_km + alt_km) / float(np.linalg.norm(self.r))))

    def summary(self) -> dict:
        out = {"label": self.label, "epoch": self.epoch, "q_ship_to_icrf": [round(float(c), 9) for c in self.q]}
        if self.has_orbit:
            out.update({"central_body": self.body,
                        "altitude_km": round(float(np.linalg.norm(self.r)) - self.body_radius_km, 3)})
        return out


def parse_nav(nav) -> Nav:
    errs = validate_nav(nav)
    if errs:
        raise ValueError("; ".join(errs))
    return Nav(nav[0] if nav else None)


# ----------------------------------------------------------------------------- sectors
def validate_sector(sec: dict, g: dict, tag: str) -> List[str]:
    """Frame messages of one sector, against the mission navigation (global 'nav')."""
    fr = sec.get("frame", "ship")
    if fr not in FRAMES:
        return [f"{tag}: unknown frame '{fr}' ({', '.join(FRAMES)})"]
    errs = []
    nav = g.get("nav")
    if fr in ORBIT_FRAMES:
        ok = not validate_nav(nav)
        if ok and not (nav and Nav(nav[0]).has_orbit):
            errs.append(f"{tag}: frame '{fr}' needs the ship orbit in the navigation (central_body, r_km, v_kms)")
    if fr == "limb":
        if sec.get("shape", "box") != "box":
            errs.append(f"{tag}: frame 'limb' takes a box only (eps band counted from the limb)")
        alt = sec.get("limb_alt_km", LIMB_ALT_KM_DEFAULT)
        if not isinstance(alt, (int, float)) or alt < 0:
            errs.append(f"{tag}: limb_alt_km must be >= 0")
        elif not errs and nav and not validate_nav(nav):
            n = Nav(nav[0])
            if n.has_orbit and np.linalg.norm(n.r) <= n.body_radius_km + alt:
                errs.append(f"{tag}: the ship is below the limb altitude ({alt:g} km above the {n.body})")
    if fr == "track":
        miss = [k for k in TRACK_KEYS if not isinstance(sec.get(k), (int, float))]
        if miss:
            errs.append(f"{tag}: frame 'track' needs {', '.join(miss)}")
        else:
            if not -90.0 <= sec["track_dec_deg"] <= 90.0:
                errs.append(f"{tag}: track_dec_deg ({sec['track_dec_deg']}) outside [-90, 90] deg")
            if sec["track_sigma_deg"] <= 0:
                errs.append(f"{tag}: track_sigma_deg must be > 0")
        if sec.get("shape", "box") != "cap":
            errs.append(f"{tag}: frame 'track' takes a cap (centred on the track, radius k sigma)")
    if fr == "two_dir":
        miss = [k for k in TWO_DIR_KEYS if not isinstance(sec.get(k), (int, float))]
        if miss:
            errs.append(f"{tag}: frame 'two_dir' needs {', '.join(miss)}")
        else:
            for k in ("frame_p_dec_deg", "frame_s_dec_deg"):
                if not -90.0 <= sec[k] <= 90.0:
                    errs.append(f"{tag}: {k} ({sec[k]}) outside [-90, 90] deg")
            p, s = _two_dir(sec)
            sep = math.degrees(math.acos(max(-1.0, min(1.0, float(p @ s)))))
            if min(sep, 180.0 - sep) < TWO_DIR_MIN_SEP_DEG:
                errs.append(f"{tag}: the two directions are {sep:.2f} deg apart: at least {TWO_DIR_MIN_SEP_DEG:g} deg "
                            f"from aligned needed")
    return errs


def _two_dir(sec: dict) -> Tuple[np.ndarray, np.ndarray]:
    return unit(sec["frame_p_ra_deg"], sec["frame_p_dec_deg"]), unit(sec["frame_s_ra_deg"], sec["frame_s_dec_deg"])


def sector_matrix(sec: dict, nav: Nav) -> Tuple[Optional[np.ndarray], float]:
    """(matrix ship -> sector frame or None for the ship frame, eps offset in deg of the sector's eps)."""
    fr = sec.get("frame", "ship")
    if fr == "ship":
        return None, 0.0
    if fr in ("inertial", "track"):
        a = np.eye(3)
    elif fr == "rtn":
        a = nav.rtn()
    elif fr == "limb":
        a = nav.limb_axes()
    else:
        p, s = _two_dir(sec)
        a = _axes(p, s)
    m = a @ nav.r_si                        # ship -> ICRF -> sector frame
    off = -nav.limb_depression_deg(sec.get("limb_alt_km", LIMB_ALT_KM_DEFAULT)) if fr == "limb" else 0.0
    return m, off


def resolve_sector(sec: dict, g: dict) -> dict:
    """A copy of the sector ready for the ship-frame raster (see the module docstring); a ship-frame sector unchanged."""
    if sec.get("frame", "ship") == "ship":
        return sec
    m, off = sector_matrix(sec, parse_nav(g.get("nav")))
    out = dict(sec)
    if sec.get("frame") == "track":                 # step 3.3, F8: the cap from the designation (ICRF)
        out.update(shape="cap", phi0_deg=float(sec["track_ra_deg"]) % 360.0, eps0_deg=float(sec["track_dec_deg"]),
                   radius_deg=TRACK_FRAME_K * float(sec["track_sigma_deg"]))
    if not np.array_equal(m, np.eye(3)) or off:
        out["frame_m"] = tuple(float(c) for c in m.ravel())
    if off:
        out["frame_eps_offset_deg"] = off
    return out


def frame_key(sec: dict) -> tuple:
    """Sectors with the same key share their frame (their boxes can be combined)."""
    fr = sec.get("frame", "ship")
    return (fr, sec.get("limb_alt_km", LIMB_ALT_KM_DEFAULT) if fr == "limb" else None,
            tuple(sec.get(k) for k in TWO_DIR_KEYS) if fr == "two_dir" else None)


def unresolved(sec: dict) -> dict:
    """The sector without the keys added by resolve_sector()."""
    return {k: v for k, v in sec.items() if k not in RESOLVED_FIELDS}


def sector_angles(sec: dict, phi: np.ndarray, eps: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """Ship-frame raster angles (deg) -> the sector frame's (phi in [0, 360), eps) and its eps offset."""
    m = sec.get("frame_m")
    if m is None:
        return phi, eps, 0.0
    m = np.array(m).reshape(3, 3)
    p, e = np.radians(phi), np.radians(eps)
    d = np.stack([np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)], axis=-1) @ m.T
    return (np.degrees(np.arctan2(d[..., 1], d[..., 0])) % 360.0,
            np.degrees(np.arcsin(np.clip(d[..., 2], -1.0, 1.0))), sec.get("frame_eps_offset_deg", 0.0))


# ----------------------------------------------------------------------------- sky
def _turn(item: dict, kp: str, ke: str, r_is: np.ndarray) -> None:
    item[kp], item[ke] = angles(r_is @ unit(item[kp], item[ke]))


def validate_sky_frame(sky: Optional[dict]) -> List[str]:
    if not isinstance(sky, dict) or sky.get("frame", "ship") in SKY_FRAMES:
        return []
    return [f"sky frame '{sky['frame']}' unknown ({', '.join(SKY_FRAMES)})"]


def sky_to_ship(sky: Optional[dict], nav: Nav) -> Optional[dict]:
    """The mission sky in the ship frame (an inertial sky turned with the attitude; key 'frame' removed)."""
    if not isinstance(sky, dict) or "frame" not in sky:
        return sky
    out = copy.deepcopy(sky)
    if out.pop("frame") == "inertial" and not np.array_equal(nav.r_si, np.eye(3)):
        r_is = nav.r_si.T
        for b in out.get("bodies", []):
            _turn(b, "phi_deg", "eps_deg", r_is)
        for j in out.get("jammers", []):
            _turn(j, "phi_deg", "eps_deg", r_is)
        gal = out.get("galactic")
        if isinstance(gal, dict):
            _turn(gal, "pole_phi_deg", "pole_eps_deg", r_is)
            _turn(gal, "centre_phi_deg", "centre_eps_deg", r_is)
    return out


def sky_nav_alerts(sky_ship: Optional[dict], nav: Nav) -> List[str]:
    """Central body of the sky (ship frame) against the navigation: direction and angular radius."""
    if not nav.has_orbit or not isinstance(sky_ship, dict):
        return []
    d = float(np.linalg.norm(nav.r))
    c_ship = nav.r_si.T @ (-nav.r / d)
    rad = math.degrees(math.asin(nav.body_radius_km / d))
    out = []
    for b in sky_ship.get("bodies", []):
        if str(b.get("type", "")).split("_")[0] != nav.body.split("_")[0]:
            continue
        off = math.degrees(math.acos(max(-1.0, min(1.0, float(unit(b["phi_deg"], b["eps_deg"]) @ c_ship)))))
        d_rad = b["angular_radius_deg"] - rad
        if off > SKY_NAV_TOL_DEG or abs(d_rad) > SKY_NAV_TOL_DEG:
            out.append(f"sky body {b.get('id')}: {off:.2f} deg from the {nav.body} direction of the navigation, "
                       f"angular radius {b['angular_radius_deg']:.2f} vs {rad:.2f} deg (tolerance {SKY_NAV_TOL_DEG:g} deg)")
    return out


# ----------------------------------------------------------------------------- mission
def validate(mission: dict) -> List[str]:
    """Navigation, sky frame and sector frames of a mission (operator-facing messages)."""
    g = mission.get("global", {})
    errs = validate_nav(g.get("nav")) + validate_sky_frame(g.get("sky"))
    for i, sec in enumerate(mission.get("sectors", [])):
        errs.extend(validate_sector(sec, g, f"sector #{i + 1} ({sec.get('id')})"))
    return errs


def uses_frames(mission: dict) -> bool:
    g = mission.get("global", {})
    return (g.get("nav") is not None or (isinstance(g.get("sky"), dict) and "frame" in g["sky"])
            or any(s.get("frame", "ship") != "ship" for s in mission.get("sectors", [])))


def resolve(mission: dict) -> Tuple[dict, List[str]]:
    """(mission in the ship frame, alerts). The input is not modified. A mission that uses no frame key is returned
    as is (ship frame, identity attitude). ValueError with the messages if the frames are invalid."""
    if not uses_frames(mission):
        return mission, []
    errs = validate(mission)
    if errs:
        raise ValueError("; ".join(errs))
    g = mission.get("global", {})
    nav = parse_nav(g.get("nav"))
    sky = sky_to_ship(g.get("sky"), nav)
    out = dict(mission)
    out["global"] = {**g, "sky": sky} if "sky" in g else dict(g)
    out["sectors"] = [resolve_sector(s, g) for s in mission.get("sectors", [])]
    return out, sky_nav_alerts(sky, nav)
