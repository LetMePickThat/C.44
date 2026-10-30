#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RE-POINTING IN FLIGHT -- what keeps the pointings on their volumes and tracks while the ship turns and moves (roadmap
step 4.5, validated by the user on 25 October 2026).

Loop (B): the pointing lattice of a volume is fixed once per frame in the volume's own frame (inertial, orbital, limb,
...); each pointing is turned into the (u, v) of a face at the instant it is transmitted, with the latest navigation
message propagated by the measured rate, and the face that sees it then transmits it (B3); the receive beams are formed
with the attitude of the reception. Ship-frame volumes do not move; the limb offset follows the altitude at every
message (B2). The orbital motion of the rtn / limb frames comes from the navigation itself.

Errors (C): at a transmission the navigation is at most age = latency + 1 / cadence old; the pointing error is the
attitude knowledge sigma, the rate knowledge sigma x age (per axis) and the unmeasured angular acceleration
1/2 alpha_max age^2 (worst case, one direction): E[r^2] = 2 (sigma_att^2 + (sigma_rate age)^2) + (alpha age^2 / 2)^2.
Search: a mean two-way beam-shape loss 2 x 12.04 E[r^2] / HPBW^2 dB per band (at the top of the band, narrowest beam),
in the band losses ('nav_pointing'); the error against tolerance_hpbw x HPBW, else the largest tenable acceleration.
Tracking: the per-axis error sqrt(E[r^2] / 2) adds in quadrature to the filter's pointing error (step 3.3).
Lever arm (user, 30 October 2026): the ship turns about its centre of gravity, L = LEVER_M from the apertures, which
move at w L and accelerate at L sqrt(w^4 + alpha^2). The navigation gives w, so that motion is compensated in the
phase; the error left along the line of sight is e^2 = L^2 (sigma_att^2 + (sigma_rate age)^2) + (alpha L / 2)^2 Var[tau^2],
tau the age of the message used, uniform between the latency and age_s() (a constant error is no loss, only its
variation between pulses). Coherent loss 10 log10 exp(phi^2), phi = 4 pi e / lambda, at the top of the band ('nav_lever').
Migration (D): at rate_max_dps a sector that is not tied to the ship crosses a face boundary in (its angular extent on the face) / rate; while it
does, the neighbouring face must carry its load: face load + the sector's load on the face it leaves <= 1, else alert.
The check over the whole attitude envelope is for the scenario simulator (phase 6; no envelope in mission
preparation, user, 27 October 2026).
Compute (E): ops_per_pointing per pointing (rotation, (u, v), face choice), and per frame the mask of each volume that
is not tied to the ship on every raster direction: a function 'repointing' of the compute budget (step 4.4).
Every value is in radar_system.toml [repointing] (rate and acceleration editable there).
"""
from __future__ import annotations

import math
from typing import Dict, List

try:
    from .params import P
except ImportError:            # imported by core/radar_model.py run as a script
    from params import P

_R = "repointing."
RATE_MAX_DPS = float(P.get(_R + "rate_max_dps"))
ACCEL_MAX_DPS2 = float(P.get(_R + "accel_max_dps2"))
NAV_HZ = float(P.get(_R + "nav_rate_hz"))
NAV_LATENCY_S = float(P.get(_R + "nav_latency_s"))
ATT_SIGMA_DEG = float(P.get(_R + "attitude_sigma_arcsec")) / 3600.0
RATE_SIGMA_DPS = float(P.get(_R + "rate_sigma_dps"))
TOL_HPBW = float(P.get(_R + "tolerance_hpbw"))
OPS_PER_POINTING = float(P.get(_R + "ops_per_pointing"))
TX_LOSS_DB_PER_HPBW2 = 10.0 * math.log10(math.e) * 4.0 * math.log(2.0)      # 12.04 dB at 1 HPBW, Gaussian lobe
LEVER_M = abs(float(P.get("ship.model_array_station_z_m")) - float(P.get("ship.model_cg_z_m")))   # apertures - CG
C_MPS = 299792458.0


def age_s() -> float:
    """Oldest navigation used at a transmission: latency + one message interval."""
    return NAV_LATENCY_S + 1.0 / NAV_HZ


def r2_deg2(accel_dps2: float = ACCEL_MAX_DPS2) -> float:
    """Mean square pointing error (deg^2, both axes) from the navigation at a transmission."""
    a = age_s()
    return 2.0 * (ATT_SIGMA_DEG ** 2 + (RATE_SIGMA_DPS * a) ** 2) + (0.5 * accel_dps2 * a * a) ** 2


def axis_sigma_deg(accel_dps2: float = ACCEL_MAX_DPS2) -> float:
    """Per-axis pointing error (deg) the navigation adds to a track look."""
    return math.sqrt(r2_deg2(accel_dps2) / 2.0)


def loss_db(hpbw_deg: float, accel_dps2: float = ACCEL_MAX_DPS2) -> float:
    """Mean two-way beam-shape loss of the navigation pointing error for a beam of hpbw_deg."""
    return 2.0 * TX_LOSS_DB_PER_HPBW2 * r2_deg2(accel_dps2) / hpbw_deg ** 2


def accel_tenable_dps2(hpbw_deg: float) -> float:
    """Largest angular acceleration that keeps the rms error within tolerance_hpbw x HPBW (0 if the knowledge terms
    alone exceed it)."""
    a = age_s()
    left = (TOL_HPBW * hpbw_deg) ** 2 - 2.0 * (ATT_SIGMA_DEG ** 2 + (RATE_SIGMA_DPS * a) ** 2)
    return 2.0 * math.sqrt(left) / (a * a) if left > 0.0 else 0.0


def lever_motion(rate_dps: float = RATE_MAX_DPS, accel_dps2: float = ACCEL_MAX_DPS2) -> tuple:
    """(speed m/s, acceleration m/s2) of the apertures about the centre of gravity at these rate and acceleration."""
    w, a = math.radians(rate_dps), math.radians(accel_dps2)
    return w * LEVER_M, LEVER_M * math.hypot(w * w, a)


def lever_error_m(accel_dps2: float = ACCEL_MAX_DPS2) -> float:
    """Rms error (m) left along the line of sight after the navigation compensated the motion of the apertures."""
    lo, hi = NAV_LATENCY_S, age_s()
    m2, m4 = ((hi ** k - lo ** k) / (k * (hi - lo)) for k in (3, 5))     # means of tau^2 and tau^4, tau uniform
    known = math.radians(ATT_SIGMA_DEG) ** 2 + (math.radians(RATE_SIGMA_DPS) * hi) ** 2
    return LEVER_M * math.sqrt(known + (0.5 * math.radians(accel_dps2)) ** 2 * (m4 - m2 * m2))


def lever_loss_db(f_ghz: float, accel_dps2: float = ACCEL_MAX_DPS2) -> float:
    """Mean coherent loss of that error at f_ghz."""
    phi = 4.0 * math.pi * lever_error_m(accel_dps2) * f_ghz * 1e9 / C_MPS
    return 10.0 * math.log10(math.e) * phi * phi


def budget(mission: dict, res: dict) -> dict:
    """Navigation pointing per band, migration of the sectors between faces at the maximum rate (step 4.5). Step 5.1:
    the mission may set ship_rate_max_dps / ship_accel_max_dps2; the band loss stays sized by the file's values (it is
    in every range), the tolerance, the migration and the track pointing use the mission's, and a mission harsher than
    the file is flagged with the loss it misses."""
    from . import optimizer as mo
    from . import radar_model as rm
    g = mission.get("global", {})
    rate = float(g.get("ship_rate_max_dps", RATE_MAX_DPS))
    accel = float(g.get("ship_accel_max_dps2", ACCEL_MAX_DPS2))
    flags: List[str] = []
    bands = {}
    for name, b in rm.BANDS.items():
        hp = rm.hpbw_deg(b.usable_hi_ghz)
        r = math.sqrt(r2_deg2(accel))
        bands[name] = {"hpbw_deg": hp, "error_rms_deg": r, "error_hpbw": r / hp, "loss_db": loss_db(hp, accel),
                       "loss_db_sized": loss_db(hp), "accel_tenable_dps2": accel_tenable_dps2(hp),
                       "lever_loss_db": lever_loss_db(b.usable_hi_ghz, accel),
                       "lever_loss_db_sized": lever_loss_db(b.usable_hi_ghz)}
        if r > TOL_HPBW * hp:
            flags.append(f"{name}: navigation pointing error {r / hp:.3f} HPBW above the {TOL_HPBW:g} HPBW tolerance "
                         f"at {accel:g} deg/s2 (tenable up to {accel_tenable_dps2(hp):.3g} deg/s2)")
        if accel > ACCEL_MAX_DPS2:
            flags.append(f"{name}: mission angular acceleration {accel:g} deg/s2 above the {ACCEL_MAX_DPS2:g} deg/s2 the "
                         f"band loss is sized for: ranges optimistic by {loss_db(hp, accel) - loss_db(hp) + bands[name]['lever_loss_db'] - bands[name]['lever_loss_db_sized']:.2g} dB")
    # migration (D): sector extent on each face, transit time, load of the neighbouring face
    g = mission.get("global", {})
    sph = mo._sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    from . import frames
    secs = {s["id"]: s for s in frames.resolve(mission)[0]["sectors"]}     # the volumes in the ship frame
    face_load = {int(k): float(v.get("load", 0.0)) for k, v in res.get("tracking", {}).get("faces", {}).items()}
    sectors = {}
    for r in res.get("sectors", []):
        if r.get("status") != "OK" or r.get("frame", "ship") == "ship":
            continue                               # a ship-frame volume turns with the ship: it never migrates
        m = sph.mask(secs[r["id"]])
        out = {}
        for kk, f in r.get("faces", {}).items():
            k = int(kk)
            omega = float((sph.d_omega * (m & (sph.owner == k))).sum())
            extent = math.degrees(math.sqrt(omega))
            load_k = max(float(f.get("energy_fraction", 0.0)), float(f.get("time_fraction", 0.0)))
            worst = max(face_load.get(j, 0.0) + load_k for j in ((k + 1) % 4, (k - 1) % 4))
            out[kk] = {"extent_deg": extent, "transit_s": extent / rate if rate > 0 else math.inf,
                       "neighbour_load_during_transit": worst}
            if worst > 1.0:
                flags.append(f"sector {r['id']}: turning at {rate:g} deg/s moves its load of face F{k + 1} "
                             f"onto a neighbour face, which would carry {worst:.2f} of its resources")
        sectors[r["id"]] = {"frame": r.get("frame", "ship"), "faces": out}
    speed, acc = lever_motion(rate, accel)
    return {"rate_max_dps": rate, "accel_max_dps2": accel, "nav_age_s": age_s(), "bands": bands, "lever_m": LEVER_M,
            "lever_speed_mps": speed, "lever_accel_mps2": acc, "lever_error_m": lever_error_m(accel),
            "track_axis_sigma_deg": axis_sigma_deg(accel), "sectors": sectors, "flags": flags}
