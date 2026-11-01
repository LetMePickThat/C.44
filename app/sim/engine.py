#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENGINE -- what the radar and the mission preparation read from a run of the global simulator, and its journal (roadmap
step 6.3, pass S4; framing N1-N14 validated by the user on 29 October 2026). The radar gets its navigation and the
sky, never the truth of the objects.

  N1-N3 navigation: messages at repointing.nav_rate_hz in the format of core/frames.py ("nav": epoch,
        q_ship_to_icrf, central_body, r_km, v_kms) plus w_ship_dps (angular rate, ship frame), computed on demand
        from the truth (Hermite states, SLERP attitude; the same seed gives the same stream). Errors only with
        meta.nav_errors: white noise per message on the attitude (repointing.attitude_sigma_arcsec per axis) and the
        rate (repointing.rate_sigma_dps), the message stamped t describes t - repointing.nav_latency_s, and constant
        offsets drawn once per run on the position and velocity (sim.nav_pos_sigma_m, sim.nav_vel_sigma_mps);
  N4-N8 sky at t, the mission "sky" format of core/sky.py in the ship frame: Sun (meta.sun_type), Earth
        (meta.earth_type) and Moon from the geometry, a body fully hidden behind a nearer one left out; the galactic
        frame (sim.galactic, ICRF) turned into the ship frame; the surface Doppler of the Earth and the Moon
        (closing_mps = speed toward the body centre, closing_spread_mps = horizontal speed x sin(angular radius));
        the stand-off jammers (objects with a SOJ jammer class) as "jammers" with their direction and range, unless
        occulted; self-protection jammers stay on their object (radar in the loop, step 6.4);
  N9    mission at t: a copy of the scenario's mission with global.nav (one message) and global.sky filled, checked
        by the mission-preparation validations (frames, sky, sky against navigation); a nav or sky already in the
        mission is replaced, with an alert;
  N10-N13 journal <stem>.npz (compressed: scenario JSON, format and model versions, parameter-file hash, truth in
        float64, attitudes, events, geometry in float32; the 100 Hz navigation only with meta.log_nav) and the
        summary <stem>.summary.json, next to the scenario, overwritten at each run; load_journal reads them back.
"""
from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import math
import os
from datetime import timedelta
from typing import Dict, List, Optional, Tuple

import numpy as np

from core import environment as env
from core import frames
from core import optimizer as opt
from core import radar_model as rm
from core import sky as skym
from core.params import P

from . import attitude as att
from . import ephemeris as eph
from . import geometry as geo
from . import scenario as scn
from .geometry import _ship_m, ship_rate_dps
from .propagate import Body, Truth

JOURNAL_VERSION = 1
NAV_RATE_HZ = float(P.get("repointing.nav_rate_hz"))
NAV_LATENCY_S = float(P.get("repointing.nav_latency_s"))
ATT_SIGMA_RAD = math.radians(float(P.get("repointing.attitude_sigma_arcsec")) / 3600.0)
RATE_SIGMA_DPS = float(P.get("repointing.rate_sigma_dps"))
POS_SIGMA_KM = float(P.get("sim.nav_pos_sigma_m")) * 1e-3
VEL_SIGMA_KMS = float(P.get("sim.nav_vel_sigma_mps")) * 1e-3
GALACTIC = P.get("sim.galactic")
SEED_NAV_BIAS, SEED_NAV_MSG = 400000, 500000           # generator streams (geometry: 200000.., 300000..)


# ----------------------------------------------------------------------------- N1-N3 navigation
class Navigation:
    """The navigation stream of one run (sc: sim.scenario.load, truth: sim.propagate.propagate)."""

    def __init__(self, sc: dict, truth: Truth):
        self.sc, self.truth = sc, truth
        self.seed = int(sc["meta"]["seed"])
        self.errors = bool(sc["meta"].get("nav_errors", False))
        self.epoch = scn.epoch(sc)
        g = np.random.default_rng([self.seed, SEED_NAV_BIAS])
        self.dr_km = POS_SIGMA_KM * g.standard_normal(3) if self.errors else np.zeros(3)
        self.dv_kms = VEL_SIGMA_KMS * g.standard_normal(3) if self.errors else np.zeros(3)

    def index(self, t: float) -> int:
        return int(round(t * NAV_RATE_HZ))

    def times(self, t0: float = 0.0, t1: Optional[float] = None) -> np.ndarray:
        t1 = min(float(self.sc["meta"]["duration_s"]), float(self.truth.t_s[-1])) if t1 is None else t1   # (ship lost)
        return np.arange(self.index(t0), self.index(t1) + 1) / NAV_RATE_HZ

    def state(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(m ship -> ICRF, w ship frame deg/s, r km, v km/s) of the message stamped t; r and v: the centre of gravity
        (the apertures follow from m and w)."""
        tt = max(0.0, t - NAV_LATENCY_S) if self.errors else t
        r, v = geo.states_at(self.truth, tt)
        m, w = _ship_m(self.truth, tt), ship_rate_dps(self.truth, tt)
        r, v = r[0] + self.dr_km, v[0] + self.dv_kms
        if self.errors:
            g = np.random.default_rng([self.seed, SEED_NAV_MSG, self.index(t)])
            m = m @ att.expm(ATT_SIGMA_RAD * g.standard_normal(3))
            w = w + RATE_SIGMA_DPS * g.standard_normal(3)
        return m, w, r, v

    def message(self, t: float, label: Optional[str] = None) -> dict:
        """A navigation message in the format of core/frames.py (+ w_ship_dps)."""
        m, w, r, v = self.state(t)
        ep = self.epoch + timedelta(seconds=float(t))
        return {"label": label or f"t = {t:g} s", "epoch": ep.isoformat().replace("+00:00", "Z"),
                "q_ship_to_icrf": [float(c) for c in frames.matrix_to_quat(m)],
                "central_body": self.truth.body.name, "r_km": [float(x) for x in r], "v_kms": [float(x) for x in v],
                "w_ship_dps": [float(x) for x in w]}

    def series(self, t0: float = 0.0, t1: Optional[float] = None) -> Dict[str, np.ndarray]:
        """The stream over [t0, t1] as arrays: t_s, q (n, 4), w_dps (n, 3), r_km, v_kms (n, 3)."""
        ts = self.times(t0, t1)
        q, w, r, v = (np.empty((len(ts), k)) for k in (4, 3, 3, 3))
        for i, t in enumerate(ts):
            m, w[i], r[i], v[i] = self.state(float(t))
            q[i] = frames.matrix_to_quat(m)
        return {"t_s": ts, "q": q, "w_dps": w, "r_km": r, "v_kms": v}


# ----------------------------------------------------------------------------- N4-N8 sky
def _phi_eps(d: np.ndarray) -> Tuple[float, float]:
    phi, eps = frames.angles(d / np.linalg.norm(d))
    return round(phi, 9), round(eps, 9)


def sky_at(sc: dict, truth: Truth, t: float, g: Optional[geo.Geometry] = None) -> dict:
    """The mission sky at t, ship frame (core/sky.py format). g: the geometry at [t] when already computed."""
    g = geo.geometry(sc, truth, np.array([float(t)])) if g is None else g
    meta = sc["meta"]
    # the radar sky knows the Sun, the Earth and the Moon (their types); another body (the central one, its planet:
    # user, 30 October 2026) only hides the others and the objects, no noise, no clutter of its own yet
    types = {"sun": meta.get("sun_type", "sun"), "earth": meta.get("earth_type", "earth"), "moon": "moon"}
    r, v = geo.states_at(truth, float(t))
    v_arr = geo.array_state(truth, float(t), r[0], v[0])[1]
    T = eph.centuries_tt(scn.epoch(sc), float(t))
    h = 30.0                                                      # s, central difference of the bodies' velocities
    dT = h / 3.15576e9
    central = truth.body.name
    v_body = {b: ((eph.body_icrf_km(b, T + dT) - eph.body_icrf_km(central, T + dT))
                  - (eph.body_icrf_km(b, T - dT) - eph.body_icrf_km(central, T - dT))) / (2.0 * h)
              for b in ("earth", "moon")}                         # (km/s, from the central body: as the ship's)
    m = g.ship_m[0]
    bodies = []
    names = g.body_names
    for k, b in enumerate(names):
        d, rad, rng = g.body_dir_ship[0, k], float(g.body_radius_deg[0, k]), float(g.body_range_km[0, k])
        hidden = any(float(g.body_range_km[0, j]) < rng
                     and math.degrees(math.acos(float(np.clip(d @ g.body_dir_ship[0, j], -1.0, 1.0)))) + rad
                     <= float(g.body_radius_deg[0, j]) for j in range(len(names)) if j != k)
        if hidden or b not in types:
            continue
        phi, eps = _phi_eps(d)
        item = {"id": b.upper(), "type": types[b], "phi_deg": phi, "eps_deg": eps,
                "angular_radius_deg": round(rad, 9)}
        if b in v_body:
            rel = 1e3 * (v_body[b] - v_arr)                      # m/s, body - apertures, ICRF
            u = m @ d
            item["closing_mps"] = round(-float(rel @ u), 6)      # body centre coming toward the ship: > 0
            item["closing_spread_mps"] = round(float(np.linalg.norm(rel - (rel @ u) * u))
                                               * math.sin(math.radians(rad)), 6)
        bodies.append(item)
    gal = {}
    for kk in ("pole", "centre"):
        d = m.T @ frames.unit(float(GALACTIC[f"{kk}_ra_deg"]), float(GALACTIC[f"{kk}_dec_deg"]))
        gal[f"{kk}_phi_deg"], gal[f"{kk}_eps_deg"] = _phi_eps(d)
    jams = []
    by_id = {o["id"]: o for o in sc.get("objects", [])}
    for j, oid in enumerate(g.ids):
        o = by_id.get(oid)
        if o is None or "jammer" not in o or env.JAMMER_CLASSES[o["jammer"]]["nature"] != "SOJ":
            continue
        if not g.exists[0, j] or g.occulted_by[0, j] >= 0:
            continue
        phi, eps = _phi_eps(g.dir_ship[0, j])
        jams.append({"id": oid, "class": o["jammer"], "phi_deg": phi, "eps_deg": eps,
                     "range_km": round(float(g.range_km[0, j]), 6)})
    out = {"bodies": bodies, "galactic": gal}
    if jams:
        out["jammers"] = jams
    return out


# ----------------------------------------------------------------------------- N9 mission at t
def mission_at(sc: dict, truth: Truth, t: float, mission: Optional[dict] = None) -> Tuple[dict, List[str]]:
    """(mission with global.nav and global.sky at t, alerts). mission: the scenario's mission file when None."""
    if mission is None:
        if not sc.get("mission"):
            raise scn.ScenarioError("the scenario names no mission file")
        from core import mission as cm
        mission = cm.load_mission(sc["mission"])
    m = copy.deepcopy(mission)
    gl = m.setdefault("global", {})
    alerts = [f"global.{k} of the mission replaced by the scenario's at t = {t:g} s" for k in ("nav", "sky") if k in gl]
    nav = Navigation(sc, truth).message(float(t), label=f"{sc['meta']['name']} t = {t:g} s")
    gl["nav"] = [nav]
    gl["sky"] = sky_at(sc, truth, float(t))
    errs = frames.validate(m) + skym.validate(gl["sky"])
    if errs:
        raise scn.ScenarioError("mission at t not valid:\n  " + "\n  ".join(errs))
    alerts += frames.sky_nav_alerts(gl["sky"], frames.parse_nav(gl["nav"]))
    return m, alerts


# ----------------------------------------------------------------------------- N10-N13 journal and summary
def params_hash() -> str:
    """Hash of every value of the parameter file (sim. included)."""
    vals = {p.path: p.value for p in P}
    return hashlib.sha256(json.dumps(vals, sort_keys=True, default=str).encode("utf-8")).hexdigest()[:16]


_TRUTH_ARRAYS = ("t_s", "r_km", "v_kms", "t_appear_s", "t_impact_s", "t_pn_start_s", "miss_km", "att_t_s", "att_q")
_TRUTH_LISTS = ("ids", "classes", "group_of", "att_ids", "att_events")


def journal_paths(scenario_path: str) -> Tuple[str, str]:
    stem = os.path.splitext(scenario_path)[0]
    return stem + ".npz", stem + ".summary.json"


def save_journal(path: str, sc: dict, truth: Truth, g: geo.Geometry, nav: Optional[Dict[str, np.ndarray]] = None,
                 cpu_s: float = 0.0) -> None:
    """The run's journal (.npz, compressed), written atomically."""
    z: Dict[str, np.ndarray] = {
        "journal_version": np.array(JOURNAL_VERSION), "model_version": np.array(rm.MODEL_VERSION),
        "params_hash": np.array(params_hash()), "scenario_json": np.array(json.dumps(sc, ensure_ascii=False)),
        "cpu_s": np.array(cpu_s), "truth.body": np.array(truth.body.name)}
    for k in _TRUTH_ARRAYS:
        z[f"truth.{k}"] = np.asarray(getattr(truth, k), np.float64)
    for k in _TRUTH_LISTS:
        z[f"truth.{k}"] = np.array(json.dumps(getattr(truth, k), default=float))
    for f in dataclasses.fields(geo.Geometry):
        val = getattr(g, f.name)
        if isinstance(val, list):
            z[f"geo.{f.name}"] = np.array(json.dumps(val))
        elif isinstance(val, np.ndarray) and val.dtype == np.float64 and f.name not in ("t_s", "rcs_offset_db"):
            z[f"geo.{f.name}"] = val.astype(np.float32)
        else:
            z[f"geo.{f.name}"] = np.asarray(val)
    if nav is not None:
        for k, a in nav.items():
            z[f"nav.{k}"] = a
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "wb") as fh:
        np.savez_compressed(fh, **z)
    os.replace(tmp, path)


def load_journal(path: str) -> Tuple[dict, Truth, geo.Geometry, Optional[Dict[str, np.ndarray]], dict]:
    """(scenario, truth, geometry, navigation or None, meta {versions, hash, cpu_s}) of a journal."""
    with np.load(path, allow_pickle=False) as z:
        if int(z["journal_version"]) != JOURNAL_VERSION:
            raise ValueError(f"{path}: journal version {int(z['journal_version'])}, this tool reads {JOURNAL_VERSION}")
        sc = json.loads(str(z["scenario_json"]))
        kw = {k: z[f"truth.{k}"] for k in _TRUTH_ARRAYS}
        kw.update({k: json.loads(str(z[f"truth.{k}"])) for k in _TRUTH_LISTS})
        truth = Truth(body=Body.of(str(z["truth.body"])), **kw)
        gk = {}
        for f in dataclasses.fields(geo.Geometry):
            if f"geo.{f.name}" not in z.files:              # (a field added since: its default)
                continue
            a = z[f"geo.{f.name}"]
            if a.dtype.kind == "U":
                gk[f.name] = json.loads(str(a))
            elif a.ndim == 0:
                gk[f.name] = a.item()
            else:
                gk[f.name] = a.astype(np.float64) if a.dtype == np.float32 else a
        nav = {k[4:]: z[k] for k in z.files if k.startswith("nav.")} or None
        meta = {"model_version": str(z["model_version"]), "params_hash": str(z["params_hash"]),
                "cpu_s": float(z["cpu_s"])}
    return sc, truth, geo.Geometry(**gk), nav, meta


def _weights(t: np.ndarray) -> np.ndarray:
    """Time each sample stands for (half the intervals on each side)."""
    if len(t) < 2:
        return np.zeros(len(t))
    d = np.diff(t)
    return np.concatenate([[d[0] / 2.0], (d[:-1] + d[1:]) / 2.0, [d[-1] / 2.0]])


def _periods(t: np.ndarray, on: np.ndarray) -> List[List[float]]:
    out, start = [], None
    for ti, x in zip(t, on):
        if x and start is None:
            start = float(ti)
        elif not x and start is not None:
            out.append([start, float(ti)])
            start = None
    if start is not None:
        out.append([start, float(t[-1])])
    return out


def summary(sc: dict, truth: Truth, g: geo.Geometry, cpu_s: float = 0.0) -> dict:
    """The run's summary (N12)."""
    w = _weights(g.t_s)
    per_class: Dict[str, int] = {}
    for c in g.classes:
        per_class[c] = per_class.get(c, 0) + 1
    objs = []
    for j, oid in enumerate(g.ids):
        i = truth.row(oid)
        ex = g.exists[:, j]
        rng = np.where(ex, g.range_km[:, j], np.inf)
        k = int(np.argmin(rng)) if ex.any() else None
        faces = {str(f): round(float(w[(g.face[:, j] == f) & ex].sum()), 3) for f in range(len(opt.FACE_PHI))}
        objs.append({"id": oid, "class": g.classes[j], "t_appear_s": float(truth.t_appear_s[i]),
                     "t_impact_s": None if not np.isfinite(truth.t_impact_s[i]) else float(truth.t_impact_s[i]),
                     "t_pn_start_s": None if not np.isfinite(truth.t_pn_start_s[i]) else float(truth.t_pn_start_s[i]),
                     "miss_m": None if not np.isfinite(truth.miss_km[i]) else round(1e3 * float(truth.miss_km[i]), 3),
                     "range_min_km": None if k is None else round(float(rng[k]), 3),
                     "t_range_min_s": None if k is None else float(g.t_s[k]),
                     "time_per_face_s": faces,
                     "time_no_face_s": round(float(w[(g.face[:, j] < 0) & ex].sum()), 3),
                     "time_occulted_s": round(float(w[(g.occulted_by[:, j] >= 0) & ex].sum()), 3),
                     "time_masked_s": round(float(w[g.masked[:, j] & ex].sum()), 3),
                     "rcs_offset_db": round(float(g.rcs_offset_db[j]), 3)})
    kinds: Dict[str, int] = {}
    for e in truth.att_events:
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    return {"name": sc["meta"]["name"], "epoch_utc": sc["meta"]["epoch_utc"],
            "duration_s": min(float(sc["meta"]["duration_s"]), float(truth.t_s[-1])), "central_body": truth.body.name,
            "model_version": rm.MODEL_VERSION, "params_hash": params_hash(), "cpu_s": round(cpu_s, 3),
            "objects_per_class": per_class, "impacts": [o["id"] for o in objs if o["t_impact_s"] is not None],
            "ship_shadow_periods_s": _periods(g.t_s, g.ship_shadow > 0.0),
            "ship_umbra_periods_s": _periods(g.t_s, g.ship_shadow >= 1.0 - 1e-9),
            "attitude_events": kinds, "attitude_event_list": truth.att_events, "objects": objs}
