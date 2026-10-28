#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EPHEMERIS -- analytical Sun and Moon (roadmap step 6.3, pass S3b, G8 validated by the user on 29 October 2026):
J. Meeus, Astronomical Algorithms (2nd ed.), geocentric, geometric (no aberration, no nutation, no light time):
  - Sun: chapter 25, low accuracy (~0.01 deg): mean longitude, equation of the centre, radius vector;
  - Moon: chapter 47, ELP-2000/82 truncated as in the book (tables 47.A and 47.B, the terms above ~0.001 deg kept);
    ~10 arcsec in longitude, ~0.1 km class in distance;
  - ecliptic and mean equinox OF DATE -> mean equator of date (mean obliquity, 22.2) -> J2000 / ICRF axes by the
    IAU 1976 precession (21.2-21.4); the frame bias ICRF - J2000 (~20 mas) is left out.
Time: the scenario epoch is UTC; TT = UTC + sim.tai_utc_s + 32.184 s (future leap seconds unknown: constant).
"""
from __future__ import annotations

import math
from datetime import datetime, timezone

import numpy as np

from core.params import P

AU_KM = 149597870.7
TAI_UTC_S = float(P.get("sim.tai_utc_s"))
TT_TAI_S = 32.184
_J2000 = datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc)     # J2000.0 as a TT instant, read on the same scale


def centuries_tt(epoch_utc: datetime, t_s: float) -> float:
    """Julian centuries of TT from J2000.0 at epoch_utc + t_s."""
    d = (epoch_utc - _J2000).total_seconds() + t_s + TAI_UTC_S + TT_TAI_S
    return d / (86400.0 * 36525.0)


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def obliquity_rad(T: float) -> float:
    """Mean obliquity of the ecliptic of date (22.2)."""
    return math.radians(23.0 + 26.0 / 60.0 + (21.448 - 46.8150 * T - 0.00059 * T * T + 0.001813 * T ** 3) / 3600.0)


def precession_to_j2000(T: float) -> np.ndarray:
    """Matrix mean equator of date -> J2000 (IAU 1976; 21.2-21.4, active form v_date = Rz(z) Ry(-theta) Rz(zeta) v0)."""
    a = math.radians(1.0 / 3600.0)
    zeta = (2306.2181 * T + 0.30188 * T * T + 0.017998 * T ** 3) * a
    z = (2306.2181 * T + 1.09468 * T * T + 0.018203 * T ** 3) * a
    theta = (2004.3109 * T - 0.42665 * T * T - 0.041833 * T ** 3) * a
    return _rz(-zeta) @ _ry(theta) @ _rz(-z)


def ecliptic_to_icrf(lon_deg: float, lat_deg: float, dist_km: float, T: float) -> np.ndarray:
    """ICRF-axes vector (km) of a point in ecliptic coordinates of date."""
    l, b = math.radians(lon_deg), math.radians(lat_deg)
    v = dist_km * np.array([math.cos(b) * math.cos(l), math.cos(b) * math.sin(l), math.sin(b)])
    return precession_to_j2000(T) @ (_rx(obliquity_rad(T)) @ v)


# ----------------------------------------------------------------------------- Sun (chapter 25)
def sun_ecliptic(T: float):
    """(true geometric longitude deg, latitude 0, distance km), mean equinox of date."""
    l0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T
    m = 357.52911 + 35999.05029 * T - 0.0001537 * T * T
    e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T
    mr = math.radians(m)
    c = ((1.914602 - 0.004817 * T - 0.000014 * T * T) * math.sin(mr) + (0.019993 - 0.000101 * T) * math.sin(2 * mr)
         + 0.000289 * math.sin(3 * mr))
    nu = math.radians(m + c)
    r_au = 1.000001018 * (1.0 - e * e) / (1.0 + e * math.cos(nu))
    return (l0 + c) % 360.0, 0.0, r_au * AU_KM


def sun_icrf_km(T: float) -> np.ndarray:
    """Geocentric position of the Sun (km, ICRF axes)."""
    return ecliptic_to_icrf(*sun_ecliptic(T), T)


# ----------------------------------------------------------------------------- Moon (chapter 47)
# Table 47.A: D, M, M', F, sum_l (1e-6 deg), sum_r (1e-3 km)
_LR = (
    (0, 0, 1, 0, 6288774, -20905355), (2, 0, -1, 0, 1274027, -3699111), (2, 0, 0, 0, 658314, -2955968),
    (0, 0, 2, 0, 213618, -569925), (0, 1, 0, 0, -185116, 48888), (0, 0, 0, 2, -114332, -3149),
    (2, 0, -2, 0, 58793, 246158), (2, -1, -1, 0, 57066, -152138), (2, 0, 1, 0, 53322, -170733),
    (2, -1, 0, 0, 45758, -204586), (0, 1, -1, 0, -40923, -129620), (1, 0, 0, 0, -34720, 108743),
    (0, 1, 1, 0, -30383, 104755), (2, 0, 0, -2, 15327, 10321), (0, 0, 1, 2, -12528, 0),
    (0, 0, 1, -2, 10980, 79661), (4, 0, -1, 0, 10675, -34782), (0, 0, 3, 0, 10034, -23210),
    (4, 0, -2, 0, 8548, -21636), (2, 1, -1, 0, -7888, 24208), (2, 1, 0, 0, -6766, 30824),
    (1, 0, -1, 0, -5163, -8379), (1, 1, 0, 0, 4987, -16675), (2, -1, 1, 0, 4036, -12831),
    (2, 0, 2, 0, 3994, -10445), (4, 0, 0, 0, 3861, -11650), (2, 0, -3, 0, 3665, 14403),
    (0, 1, -2, 0, -2689, -7003), (2, 0, -1, 2, -2602, 0), (2, -1, -2, 0, 2390, 10056),
    (1, 0, 1, 0, -2348, 6322), (2, -2, 0, 0, 2236, -9884), (0, 1, 2, 0, -2120, 5751),
    (0, 2, 0, 0, -2069, 0), (2, -2, -1, 0, 2048, -4950), (2, 0, 1, -2, -1773, 4130),
    (2, 0, 0, 2, -1595, 0), (4, -1, -1, 0, 1215, -3958), (0, 0, 2, 2, -1110, 0),
    (3, 0, -1, 0, -892, 3258), (2, 1, 1, 0, -810, 2616), (4, -1, -2, 0, 759, -1897),
    (0, 2, -1, 0, -713, -2117), (2, 2, -1, 0, -700, 2354), (2, 1, -2, 0, 691, 0),
    (2, -1, 0, -2, 596, 0), (4, 0, 1, 0, 549, -1423), (0, 0, 4, 0, 537, -1117),
    (4, -1, 0, 0, 520, -1571), (1, 0, -2, 0, -487, -1739), (2, 1, 0, -2, -399, 0),
    (0, 0, 2, -2, -381, -4421), (1, 1, 1, 0, 351, 0), (3, 0, -2, 0, -340, 0),
    (4, 0, -3, 0, 330, 0), (2, -1, 2, 0, 327, 0), (0, 2, 1, 0, -323, 1165),
    (1, 1, -1, 0, 299, 0), (2, 0, 3, 0, 294, 0), (2, 0, -1, -2, 0, 8752),
)
# Table 47.B: D, M, M', F, sum_b (1e-6 deg), terms down to ~0.001 deg
_B = (
    (0, 0, 0, 1, 5128122), (0, 0, 1, 1, 280602), (0, 0, 1, -1, 277693), (2, 0, 0, -1, 173237),
    (2, 0, -1, 1, 55413), (2, 0, -1, -1, 46271), (2, 0, 0, 1, 32573), (0, 0, 2, 1, 17198),
    (2, 0, 1, -1, 9266), (0, 0, 2, -1, 8822), (2, -1, 0, -1, 8216), (2, 0, -2, -1, 4324),
    (2, 0, 1, 1, 4200), (2, 1, 0, -1, -3359), (2, -1, -1, 1, 2463), (2, -1, 0, 1, 2211),
    (2, -1, -1, -1, 2065), (0, 1, -1, -1, -1870), (4, 0, -1, -1, 1828), (0, 1, 0, 1, -1794),
    (0, 0, 0, 3, -1749), (0, 1, -1, 1, -1565), (1, 0, 0, 1, -1491), (0, 1, 1, 1, -1475),
    (0, 1, 1, -1, -1410), (0, 1, 0, -1, -1344), (1, 0, 0, -1, -1335), (0, 0, 3, 1, 1107),
    (4, 0, 0, -1, 1021), (4, 0, -1, 1, 833),
)


def moon_ecliptic(T: float):
    """(geometric longitude deg, latitude deg, distance km of the centres), mean equinox of date (47.1-47.7)."""
    lp = 218.3164477 + 481267.88123421 * T - 0.0015786 * T * T + T ** 3 / 538841.0 - T ** 4 / 65194000.0
    d = 297.8501921 + 445267.1114034 * T - 0.0018819 * T * T + T ** 3 / 545868.0 - T ** 4 / 113065000.0
    m = 357.5291092 + 35999.0502909 * T - 0.0001536 * T * T + T ** 3 / 24490000.0
    mp = 134.9633964 + 477198.8675055 * T + 0.0087414 * T * T + T ** 3 / 69699.0 - T ** 4 / 14712000.0
    f = 93.2720950 + 483202.0175233 * T - 0.0036539 * T * T - T ** 3 / 3526000.0 + T ** 4 / 863310000.0
    a1, a2, a3 = (math.radians(x) for x in (119.75 + 131.849 * T, 53.09 + 479264.290 * T, 313.45 + 481266.484 * T))
    e = 1.0 - 0.002516 * T - 0.0000074 * T * T
    D, M, MP, F, LP = (math.radians(x % 360.0) for x in (d, m, mp, f, lp))
    sl = sr = sb = 0.0
    for cd, cm, cmp_, cf, kl, kr in _LR:
        arg = cd * D + cm * M + cmp_ * MP + cf * F
        w = e ** abs(cm)
        sl += kl * w * math.sin(arg)
        sr += kr * w * math.cos(arg)
    for cd, cm, cmp_, cf, kb in _B:
        sb += kb * e ** abs(cm) * math.sin(cd * D + cm * M + cmp_ * MP + cf * F)
    sl += 3958.0 * math.sin(a1) + 1962.0 * math.sin(LP - F) + 318.0 * math.sin(a2)
    sb += (-2235.0 * math.sin(LP) + 382.0 * math.sin(a3) + 175.0 * math.sin(a1 - F) + 175.0 * math.sin(a1 + F)
           + 127.0 * math.sin(LP - MP) - 115.0 * math.sin(LP + MP))
    return (lp + sl / 1e6) % 360.0, sb / 1e6, 385000.56 + sr / 1000.0


def moon_icrf_km(T: float) -> np.ndarray:
    """Geocentric position of the Moon (km, ICRF axes)."""
    return ecliptic_to_icrf(*moon_ecliptic(T), T)
