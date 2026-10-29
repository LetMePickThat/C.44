#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST -- compiled kernels of the global simulator (numba, user 29 October 2026: scenarios of 1000+ objects), used when
numba is installed and sim.use_numba is true; otherwise the numpy code of sim/propagate.py and sim/attitude.py runs
(same results to rounding). Compiled once and cached on disk (cache=True: the first run of a new version pays the
compilation, a few seconds).

  gravity   the acceleration of every row: two bodies + J2 about the pole, plus the differential attraction of the
            other bodies (O1), plus the rows' Singer accelerations
  pointing  body -> ICRF matrix with Z along d, X in the orbital plane, then the roll (sim/attitude.pointing)
  slew      one step of a flown attitude toward its goal (sim/attitude.Flyer.step): feed-forward of the goal's rate,
            the slew rate that still stops on it, the rate and acceleration bounds, the turn by expm
  pn_command  the proportional-navigation command of the guided rows (in the acceleration)
  pn_step     the per-step update of the guided rows: start, closest approach, end when the range opens
"""
from __future__ import annotations

import math

import numpy as np

from core.params import P

try:                                   # optional (requirements.txt): the numpy code runs without it
    import numba as _nb
except ImportError:                    # pragma: no cover
    _nb = None

USE_NUMBA: bool = bool(P.get("sim.use_numba")) and _nb is not None


def _jit(f):
    return _nb.njit(cache=True)(f) if USE_NUMBA else f


@_jit
def gravity(r, a_sing, mu, j2, r_eq_km, pole, tpos, tmu):
    """(n, 3) km/s2 at the positions r (n, 3): -mu r / |r|^3, J2 about the unit pole, + a_sing (n, 3), + the
    differential attraction mu_b [(r_b - r) / |r_b - r|^3 - r_b / |r_b|^3] of the bodies at tpos (k, 3)."""
    n = r.shape[0]
    nb = tpos.shape[0]
    a = np.empty((n, 3))
    ind = np.empty((nb, 3))
    for b in range(nb):
        rbn = math.sqrt(tpos[b, 0] * tpos[b, 0] + tpos[b, 1] * tpos[b, 1] + tpos[b, 2] * tpos[b, 2])
        rb3 = rbn ** 3
        for c in range(3):
            ind[b, c] = tpos[b, c] / rb3
    kj = 1.5 * j2 * mu * r_eq_km ** 2
    for i in range(n):
        x, y, z = r[i, 0], r[i, 1], r[i, 2]
        rn2 = x * x + y * y + z * z
        rn = math.sqrt(rn2)
        d3 = rn2 * rn
        ax, ay, az = -mu * x / d3, -mu * y / d3, -mu * z / d3
        if j2 != 0.0:
            zp = x * pole[0] + y * pole[1] + z * pole[2]
            k = kj / (rn2 * rn2 * rn)
            f = 1.0 - 5.0 * zp * zp / rn2
            ax -= k * (f * x + 2.0 * zp * pole[0])
            ay -= k * (f * y + 2.0 * zp * pole[1])
            az -= k * (f * z + 2.0 * zp * pole[2])
        ax += a_sing[i, 0]
        ay += a_sing[i, 1]
        az += a_sing[i, 2]
        tx, ty, tz = 0.0, 0.0, 0.0
        for b in range(nb):
            dx, dy, dz = tpos[b, 0] - x, tpos[b, 1] - y, tpos[b, 2] - z
            dn3 = math.sqrt(dx * dx + dy * dy + dz * dz) ** 3
            tx += tmu[b] * (dx / dn3 - ind[b, 0])
            ty += tmu[b] * (dy / dn3 - ind[b, 1])
            tz += tmu[b] * (dz / dn3 - ind[b, 2])
        a[i, 0] = ax + tx
        a[i, 1] = ay + ty
        a[i, 2] = az + tz
    return a


@_jit
def _cross(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


@_jit
def _norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


@_jit
def pointing(d, r, v, roll_rad):
    """Body -> ICRF matrix with Z along d, X = N x Z (along the track when d is along the orbit normal N), then the
    roll about Z (sim/attitude.pointing)."""
    rr = r / _norm(r)
    nn = _cross(r, v)
    nn = nn / _norm(nn)
    t = _cross(nn, rr)
    z = d / _norm(d)
    x = _cross(nn, z)
    if _norm(x) < 1e-6:
        x = t - (t[0] * z[0] + t[1] * z[1] + t[2] * z[2]) * z
    x = x / _norm(x)
    y = _cross(z, x)
    c, s = math.cos(-roll_rad), math.sin(-roll_rad)
    m = np.empty((3, 3))
    for i in range(3):                                   # [x y z] @ Rz(-roll)
        m[i, 0] = x[i] * c + y[i] * s
        m[i, 1] = -x[i] * s + y[i] * c
        m[i, 2] = z[i]
    return m


@_jit
def _rotvec(m):
    c = max(-1.0, min(1.0, (m[0, 0] + m[1, 1] + m[2, 2] - 1.0) / 2.0))
    a = math.acos(c)
    out = np.empty(3)
    if a < 1e-9:
        out[0], out[1], out[2] = 0.5 * (m[2, 1] - m[1, 2]), 0.5 * (m[0, 2] - m[2, 0]), 0.5 * (m[1, 0] - m[0, 1])
        return out
    if a > math.pi - 1e-6:                               # near 180 deg: the axis from the symmetric part
        b = (m + np.eye(3)) / 2.0
        k = 0
        for j in range(1, 3):
            if b[j, j] > b[k, k]:
                k = j
        s = math.sqrt(max(b[k, k], 1e-300))
        for j in range(3):
            out[j] = b[j, k] / s * a
        return out
    f = a / (2.0 * math.sin(a))
    out[0], out[1], out[2] = f * (m[2, 1] - m[1, 2]), f * (m[0, 2] - m[2, 0]), f * (m[1, 0] - m[0, 1])
    return out


@_jit
def _expm(w):
    a = _norm(w)
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1.0 - math.cos(a)) * (kx @ kx)


@_jit
def slew(g0, g1, m, w, dt, a_max, cap, w_cap):
    """(m, w, phi, err_deg) after one step dt of an attitude m turning at w (rad/s) toward the goal g0 (at the step
    start) then g1 (its end): phi the error at the start (rad), err_deg the error to g1 at the end (deg)."""
    w_goal = _rotvec(g1 @ g0.T) / dt                       # the attitude's own rate, fed forward
    e = _rotvec(g0 @ m.T)
    phi = _norm(e)
    h = 0.5 * a_max * dt
    if phi > 0.0:
        sl = min(cap, -h + math.sqrt(h * h + 2.0 * a_max * phi), phi / dt)
        w_des = w_goal + e / phi * sl
    else:
        w_des = w_goal.copy()
    nd = _norm(w_des)
    if nd > w_cap:
        w_des = w_des * (w_cap / nd)
    dw = w_des - w
    ndw = _norm(dw)
    if ndw > a_max * dt:
        dw = dw * (a_max * dt / ndw)
    w2 = w + dw
    m2 = _expm(w2 * dt) @ m
    err = math.degrees(_norm(_rotvec(g1 @ m2.T)))
    return m2, w2, phi, err


@_jit
def pn_command(a, r, v, on, gain, lim):
    """Adds, in place, the proportional-navigation command to the rows on (indices into r, v, a; the ship row 0):
    a = N Vc (Omega x LOS), normal to the line of sight, bounded by lim (km/s2) (sim/propagate)."""
    for q in range(on.shape[0]):
        i = on[q]
        lx, ly, lz = r[0, 0] - r[i, 0], r[0, 1] - r[i, 1], r[0, 2] - r[i, 2]          # object -> ship
        wx, wy, wz = v[0, 0] - v[i, 0], v[0, 1] - v[i, 1], v[0, 2] - v[i, 2]
        rn2 = lx * lx + ly * ly + lz * lz
        rn = math.sqrt(rn2)
        ox, oy, oz = (ly * wz - lz * wy) / rn2, (lz * wx - lx * wz) / rn2, (lx * wy - ly * wx) / rn2
        vc = -(lx * wx + ly * wy + lz * wz) / rn
        ux, uy, uz = lx / rn, ly / rn, lz / rn
        k = gain[q] * vc
        cx, cy, cz = k * (oy * uz - oz * uy), k * (oz * ux - ox * uz), k * (ox * uy - oy * ux)
        nm = math.sqrt(cx * cx + cy * cy + cz * cz)
        f = lim[q] / max(nm, 1e-300) if nm > lim[q] else 1.0
        a[i, 0] += cx * f
        a[i, 1] += cy * f
        a[i, 2] += cz * f


@_jit
def pn_step(x, prev, sel, was_on, t, pn_t, pn_r0, pn_on, pn_done, t_pn, miss):
    """The per-step update of the guided rows sel (row indices; was_on: guided before this step; pn_t / pn_r0 their
    start time or range, NaN when not used): start at the time or within the range; when guided, the step's
    closest approach (relative motion linear within it) and the end when the range opens (arrays updated in
    place)."""
    for q in range(sel.shape[0]):
        i = sel[q]
        lx, ly, lz = x[0, 0] - x[i, 0], x[0, 1] - x[i, 1], x[0, 2] - x[i, 2]
        if not was_on[q]:
            if not math.isnan(pn_t[q]):
                start = t >= pn_t[q] - 1e-9
            else:
                start = math.sqrt(lx * lx + ly * ly + lz * lz) <= pn_r0[q]
            if start:
                pn_on[i] = True
                t_pn[i] = t
            continue
        d0x, d0y, d0z = prev[0, 0] - prev[i, 0], prev[0, 1] - prev[i, 1], prev[0, 2] - prev[i, 2]
        ddx, ddy, ddz = lx - d0x, ly - d0y, lz - d0z
        u = -(d0x * ddx + d0y * ddy + d0z * ddz) / max(ddx * ddx + ddy * ddy + ddz * ddz, 1e-300)
        u = min(max(u, 0.0), 1.0)
        cx, cy, cz = d0x + u * ddx, d0y + u * ddy, d0z + u * ddz
        closest = math.sqrt(cx * cx + cy * cy + cz * cz)
        miss[i] = closest if math.isnan(miss[i]) else min(miss[i], closest)
        if lx * (x[0, 3] - x[i, 3]) + ly * (x[0, 4] - x[i, 4]) + lz * (x[0, 5] - x[i, 5]) > 0.0:
            pn_on[i] = False
            pn_done[i] = True
