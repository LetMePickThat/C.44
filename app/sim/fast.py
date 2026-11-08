#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST -- compiled kernels of the global simulator (numba, user 29 October 2026: scenarios of 1000+ objects), used when
numba is installed and sim.use_numba is true; otherwise the numpy code of sim/propagate.py and sim/attitude.py runs
(same results to rounding). Compiled once and cached on disk (cache=True: the first run of a new version pays the
compilation, a few seconds).

  gravity   the acceleration of every row: two bodies + J2 about the pole, plus the differential attraction of the
            other bodies (O1), plus the rows' Singer accelerations; gravity_par the same on all the cores (from
            sim.parallel_min_rows rows, the same result to the bit, user 8 November 2026)
  pointing  body -> ICRF matrix with Z along d, X in the orbital plane, then the roll (sim/attitude.pointing)
  slew      one step of a flown attitude toward its goal (sim/attitude.Flyer.step): feed-forward of the goal's rate,
            the slew rate that still stops on it, the rate and acceleration bounds, the turn by expm
  fleet_step  the same for every flyer of a run at once, with the goals of its attitude kinds (sim/attitude.Fleet)
  multitrack_labels  the groups of objects close on screen (gui/sim_view._multitracks)
  pn_command  the proportional-navigation command of the guided rows (in the acceleration)
  rk4_step    one RK4 step of the full computation's active rows in one call (no burn running), the same bits as
              the numpy step of sim/propagate (user 8 November 2026)
  pn_step     the per-step update of the guided rows: start, closest approach, end when the range opens
  fleet_step_all, pn_update_all, impacts, grid_at3  the rest of the full computation's step (P1-P3, user 8 November
              2026): the selections made in the kernels, the other bodies at a step's three instants in one call;
              the same bits as the numpy code
"""
from __future__ import annotations

import math
import threading

import numpy as np

from core.params import P

try:                                   # optional (requirements.txt): the numpy code runs without it
    import numba as _nb
except ImportError:                    # pragma: no cover
    _nb = None

USE_NUMBA: bool = bool(P.get("sim.use_numba")) and _nb is not None


def _jit(f):
    return _nb.njit(cache=True, nogil=True)(f) if USE_NUMBA else f   # (nogil: the editor's background preview
                                                                      # leaves the interface its turns, 8 Nov 2026)


def _jit_parallel(f):
    return _nb.njit(cache=True, nogil=True, parallel=True)(f) if USE_NUMBA else f


_prange = _nb.prange if _nb is not None else range
PAR_LOCK = threading.Lock()            # one parallel kernel at a time (numba's workqueue layer, used when neither
                                       # OpenMP nor TBB is there, aborts on concurrent calls from two threads)


@_jit
def _grid_at(grid, g_t0, g_step, t, out):
    """The other bodies at t from their grid (n, k, 3): the cubic through four points (propagate.ThirdBodies)."""
    x = (t - g_t0) / g_step
    k = int(math.floor(x))
    k = min(max(k, 1), grid.shape[0] - 3)
    u = x - k
    w0 = -u * (u - 1.0) * (u - 2.0) / 6.0
    w1 = (u + 1.0) * (u - 1.0) * (u - 2.0) / 2.0
    w2 = -(u + 1.0) * u * (u - 2.0) / 2.0
    w3 = (u + 1.0) * u * (u - 1.0) / 6.0
    for b in range(grid.shape[1]):
        for c in range(3):
            out[b, c] = (w0 * grid[k - 1, b, c] + w1 * grid[k, b, c] + w2 * grid[k + 1, b, c]
                         + w3 * grid[k + 2, b, c])


@_jit
def grid_at3(grid, g_t0, g_step, t0, h, out):
    """The other bodies at a step's start, middle and end (t0, t0 + h / 2, t0 + h) into out (3, n, 3) in one call
    (P1, user 8 November 2026: the same bits as propagate.ThirdBodies.positions, checked at 15 000 instants)."""
    _grid_at(grid, g_t0, g_step, t0, out[0])
    _grid_at(grid, g_t0, g_step, t0 + 0.5 * h, out[1])
    _grid_at(grid, g_t0, g_step, t0 + h, out[2])


@_jit
def coast_path(x, u, t0, h, n, grid, g_t0, g_step, tmu, mu, j2, r_eq_km, pole, radius_km, out_r, out_v):
    """The free fall of one state x, u (3,) from t0 in n RK4 steps of h (the manoeuvre node's path between its
    manoeuvres, propagate.preview_path; fork RADARv6_RS, 8 November 2026: 21 ms of a drag event in Python):
    out_r, out_v (n, 3) after each step. Returns the steps taken: fewer when the surface is met (the last one inside)."""
    r = np.empty((1, 3))
    no = np.zeros((1, 3))
    tp = np.empty((grid.shape[1], 3))
    xs, us = x.copy(), u.copy()
    kx = np.empty((4, 3))
    ka = np.empty((4, 3))
    for s in range(n):
        t = t0 + s * h
        for q in range(4):
            c = 0.0 if q == 0 else (0.5 if q < 3 else 1.0)
            tq = t + c * h
            for i in range(3):
                pv = ka[q - 1, i] if q > 0 else 0.0
                px = kx[q - 1, i] if q > 0 else 0.0
                kx[q, i] = us[i] + c * h * pv
                r[0, i] = xs[i] + c * h * px
            _grid_at(grid, g_t0, g_step, tq, tp)
            a = gravity(r, no, mu, j2, r_eq_km, pole, tp, tmu)
            for i in range(3):
                ka[q, i] = a[0, i]
        for i in range(3):
            xs[i] += h / 6.0 * (kx[0, i] + 2.0 * kx[1, i] + 2.0 * kx[2, i] + kx[3, i])
            us[i] += h / 6.0 * (ka[0, i] + 2.0 * ka[1, i] + 2.0 * ka[2, i] + ka[3, i])
            out_r[s, i] = xs[i]
            out_v[s, i] = us[i]
        if math.sqrt(xs[0] * xs[0] + xs[1] * xs[1] + xs[2] * xs[2]) < radius_km:
            return s + 1
    return n


@_jit
def _indirect(tpos):
    """r_b / |r_b|^3 of the bodies at tpos (k, 3): the indirect term of their differential attraction."""
    nb = tpos.shape[0]
    ind = np.empty((nb, 3))
    for b in range(nb):
        rbn = math.sqrt(tpos[b, 0] * tpos[b, 0] + tpos[b, 1] * tpos[b, 1] + tpos[b, 2] * tpos[b, 2])
        rb3 = rbn ** 3
        for c in range(3):
            ind[b, c] = tpos[b, c] / rb3
    return ind


@_jit
def _gravity_row(i, r, a_sing, mu, kj, j2, pole, tpos, tmu, ind, a):
    """a[i] of gravity (below): one row, on its own (the same operations in the same order, serial or parallel)."""
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
    for b in range(tpos.shape[0]):
        dx, dy, dz = tpos[b, 0] - x, tpos[b, 1] - y, tpos[b, 2] - z
        dn3 = math.sqrt(dx * dx + dy * dy + dz * dz) ** 3
        tx += tmu[b] * (dx / dn3 - ind[b, 0])
        ty += tmu[b] * (dy / dn3 - ind[b, 1])
        tz += tmu[b] * (dz / dn3 - ind[b, 2])
    a[i, 0] = ax + tx
    a[i, 1] = ay + ty
    a[i, 2] = az + tz


@_jit
def gravity(r, a_sing, mu, j2, r_eq_km, pole, tpos, tmu):
    """(n, 3) km/s2 at the positions r (n, 3): -mu r / |r|^3, J2 about the unit pole, + a_sing (n, 3), + the
    differential attraction mu_b [(r_b - r) / |r_b - r|^3 - r_b / |r_b|^3] of the bodies at tpos (k, 3)."""
    n = r.shape[0]
    a = np.empty((n, 3))
    ind = _indirect(tpos)
    kj = 1.5 * j2 * mu * r_eq_km ** 2
    for i in range(n):
        _gravity_row(i, r, a_sing, mu, kj, j2, pole, tpos, tmu, ind, a)
    return a


@_jit_parallel
def gravity_par(r, a_sing, mu, j2, r_eq_km, pole, tpos, tmu):
    """gravity on all the cores (C2, user 8 November 2026): the rows shared among the threads, each row computed as
    in gravity (the same result to the bit); worth it from sim.parallel_min_rows rows (the threads' start costs a
    few microseconds)."""
    n = r.shape[0]
    a = np.empty((n, 3))
    ind = _indirect(tpos)
    kj = 1.5 * j2 * mu * r_eq_km ** 2
    for i in _prange(n):
        _gravity_row(i, r, a_sing, mu, kj, j2, pole, tpos, tmu, ind, a)
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
def _goal(kind, roll_rad, fixed, r, v, d):
    """Body -> ICRF matrix of an attitude for a body at (r, v) (sim/attitude.target_attitude): kind as attitude.KINDS,
    fixed the matrix of the fixed kinds, d the direction of "target"."""
    if kind == 1:                                        # fixed, inertial
        return fixed
    rr = r / _norm(r)
    nn = _cross(r, v)
    nn = nn / _norm(nn)
    if kind == 0:                                        # fixed, orbital: X = -R, Y = N, Z = T, then fixed
        t = _cross(nn, rr)
        m = np.empty((3, 3))
        for i in range(3):
            m[i, 0], m[i, 1], m[i, 2] = -rr[i], nn[i], t[i]
        return m @ fixed
    if kind == 2:
        d = v
    elif kind == 3:
        d = -v
    elif kind == 4:
        d = nn
    elif kind == 5:
        d = -nn
    elif kind == 6:
        d = rr
    elif kind == 7:
        d = -rr
    return pointing(d, r, v, roll_rad)


@_jit
def fleet_step(sel, rows, kind, roll, fixed, target, rate, active, x, dt, a_max, w_cap, reached_deg, hold_deg, m, w,
               phi_deg, err_deg, reached, lost, flags):
    """One step dt of the attitudes sel (indices into the fleet's arrays, sim/attitude.Fleet) on the states x (n, 6) at
    the step start: their goals at both ends, slew, then the slew's state as Flyer._hold; flags: 0, 1 hold lost, 2 hold
    recovered, 3 target missing (no turn, no rate)."""
    for q in range(sel.shape[0]):
        _fly(sel[q], rows, kind, roll, fixed, target, rate, active, x, dt, a_max, w_cap, reached_deg, hold_deg, m, w,
             phi_deg, err_deg, reached, lost, flags)


@_jit
def fleet_step_all(rows, has, kind, roll, fixed, target, rate, active, x, dt, a_max, w_cap, reached_deg, hold_deg, m,
                   w, phi_deg, err_deg, reached, lost, flags, hit):
    """fleet_step on every attitude of an active row that has one (P2, user 8 November 2026: the selection made here,
    not in numpy at each step), in the same order; hit: the indices whose flags were set, their count returned."""
    nh = 0
    for j in range(rows.shape[0]):
        if not (active[rows[j]] and has[j]):
            continue
        _fly(j, rows, kind, roll, fixed, target, rate, active, x, dt, a_max, w_cap, reached_deg, hold_deg, m, w,
             phi_deg, err_deg, reached, lost, flags)
        if flags[j] != 0:
            hit[nh] = j
            nh += 1
    return nh


@_jit
def _fly(j, rows, kind, roll, fixed, target, rate, active, x, dt, a_max, w_cap, reached_deg, hold_deg, m, w, phi_deg,
         err_deg, reached, lost, flags):
    """One step of the attitude j (fleet_step)."""
    flags[j] = 0
    i = rows[j]
    r0, v0 = x[i, :3].copy(), x[i, 3:].copy()
    r1 = r0 + v0 * dt
    d0, d1 = np.zeros(3), np.zeros(3)
    if kind[j] == 8:
        s = target[j]
        if s < 0 or not active[s]:
            flags[j] = 3
            w[j, 0], w[j, 1], w[j, 2] = 0.0, 0.0, 0.0
            return
        d0 = x[s, :3] - r0
        d1 = x[s, :3] + x[s, 3:] * dt - r1
    g0 = _goal(kind[j], roll[j], fixed[j].copy(), r0, v0, d0)
    g1 = _goal(kind[j], roll[j], fixed[j].copy(), r1, v0, d1)
    m2, w2, phi, err = slew(g0, g1, m[j].copy(), w[j].copy(), dt, a_max, w_cap if reached[j] else rate[j], w_cap)
    m[j, :, :] = m2
    w[j, :] = w2
    phi_deg[j] = math.degrees(phi)
    err_deg[j] = err
    if not reached[j] and err < reached_deg:
        reached[j] = True
    elif reached[j] and err > hold_deg and not lost[j]:
        lost[j] = True
        flags[j] = 1
    elif lost[j] and err < reached_deg:
        lost[j] = False
        flags[j] = 2


@_jit
def _root(parent, a):
    while parent[a] != a:
        parent[a] = parent[parent[a]]
        a = parent[a]
    return a


@_jit
def multitrack_labels(xy, code, lim, bw, bh, near):
    """The groups of two or more points of one code linked by distances up to lim (neither of the two near), or whose
    boxes of bw x bh px (their data boxes, at the same offset from each point) would overlap: |dx| < bw and |dy| < bh,
    near or not (user, 8 November 2026) (one to the next: a chain is one group), code < 0 taking no part: each point's
    group (its lowest member), -1 when alone (user, 1 November 2026: 6 ms per frame at 1000 objects through a k-d tree
    and a graph). A sweep along x; a group's points all compared at worst, 1000 x 1000 simple tests."""
    n = xy.shape[0]
    order = np.argsort(xy[:, 0])
    parent = np.arange(n)
    lim2 = lim * lim
    reach = max(lim, bw)
    for p in range(n):
        a = order[p]
        if code[a] < 0:
            continue
        for s in range(p + 1, n):
            b = order[s]
            dx = xy[b, 0] - xy[a, 0]
            if dx > reach:
                break
            dy = xy[b, 1] - xy[a, 1]
            if code[b] == code[a] and ((dx * dx + dy * dy <= lim2 and not near[a] and not near[b])
                                       or (dx < bw and abs(dy) < bh)):
                ra, rb = _root(parent, a), _root(parent, b)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
    size = np.zeros(n, np.int64)
    for a in range(n):
        if code[a] >= 0:
            size[_root(parent, a)] += 1
    out = np.full(n, -1, np.int64)
    for a in range(n):
        if code[a] >= 0 and size[_root(parent, a)] >= 2:
            out[a] = _root(parent, a)
    return out


@_jit
def pn_command(a, r, v, on, tg, gain, lim):
    """Adds, in place, the proportional-navigation command to the rows on (indices into r, v, a) toward their
    targets tg (same indexing; the ship by default): a = N Vc (Omega x LOS), normal to the line of sight, bounded by
    lim (km/s2) (sim/propagate)."""
    for q in range(on.shape[0]):
        i = on[q]
        s = tg[q]
        lx, ly, lz = r[s, 0] - r[i, 0], r[s, 1] - r[i, 1], r[s, 2] - r[i, 2]          # object -> target
        wx, wy, wz = v[s, 0] - v[i, 0], v[s, 1] - v[i, 1], v[s, 2] - v[i, 2]
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
def rk4_step(x, act, h, tpos3, tmu, mu, j2, r_eq_km, pole, a_sing, on, tg, gain, lim, par):
    """One RK4 step h of the active rows act (indices into x, (n, 6), updated in place) in a single call (C3, user
    8 November 2026: the step in numpy cost more than the forces): the same operations in the same order as
    sim/propagate.LiveSim._rk4 and _accel without a burn running (gravity, J2, Singer a_sing (m, 3), the other
    bodies at tpos3 (3, nb, 3): the step's start, middle and end; proportional navigation of the rows on toward tg,
    indices into act), hence the same result to the bit; par: gravity_par instead of gravity."""
    m = act.shape[0]
    y = np.empty((m, 6))
    for i in range(m):
        for c in range(6):
            y[i, c] = x[act[i], c]
    ks = np.empty((4, m, 6))
    rr = np.empty((m, 3))
    vv = np.empty((m, 3))
    for q in range(4):
        if q == 0:
            for i in range(m):
                for c in range(3):
                    rr[i, c] = y[i, c]
                    vv[i, c] = y[i, c + 3]
        else:
            w = h if q == 3 else 0.5 * h            # (as y + 0.5 * h * k1 and y + h * k3 in numpy)
            for i in range(m):
                for c in range(3):
                    rr[i, c] = y[i, c] + w * ks[q - 1, i, c]
                    vv[i, c] = y[i, c + 3] + w * ks[q - 1, i, c + 3]
        tp = tpos3[0] if q == 0 else (tpos3[2] if q == 3 else tpos3[1])
        if par:
            a = gravity_par(rr, a_sing, mu, j2, r_eq_km, pole, tp, tmu)
        else:
            a = gravity(rr, a_sing, mu, j2, r_eq_km, pole, tp, tmu)
        if on.shape[0]:
            pn_command(a, rr, vv, on, tg, gain, lim)
        for i in range(m):
            for c in range(3):
                ks[q, i, c] = vv[i, c]
                ks[q, i, c + 3] = a[i, c]
    f = h / 6.0
    for i in range(m):
        for c in range(6):
            x[act[i], c] = y[i, c] + f * (ks[0, i, c] + 2.0 * ks[1, i, c] + 2.0 * ks[2, i, c] + ks[3, i, c])


@_jit
def pn_step(x, prev, sel, tg, was_on, t, pn_t, pn_r0, pn_on, pn_done, t_pn, miss):
    """The per-step update of the guided rows sel (row indices; tg: their targets' rows, the ship by default;
    was_on: guided before this step; pn_t / pn_r0 their start time or range, NaN when not used): start at the
    time or within the range of the target; when guided, the step's closest approach to it (relative motion
    linear within it) and the end when the range opens (arrays updated in place)."""
    for q in range(sel.shape[0]):
        _pn_one(x, prev, sel[q], tg[q], was_on[q], t, pn_t[q], pn_r0[q], pn_on, pn_done, t_pn, miss)


@_jit
def pn_update_all(x, prev, pn_rows, pn_tgt, active, t, pn_t, pn_r0, pn_on, pn_done, t_pn, miss):
    """pn_step on the guided rows pn_rows (pn_t / pn_r0 in their order) that exist, are not done and whose target
    exists (P3, user 8 November 2026: the selection made here, not in numpy at each step), in the same order."""
    for q in range(pn_rows.shape[0]):
        i = pn_rows[q]
        s = pn_tgt[i]
        if active[i] and not pn_done[i] and active[s]:
            _pn_one(x, prev, i, s, pn_on[i], t, pn_t[q], pn_r0[q], pn_on, pn_done, t_pn, miss)


@_jit
def _pn_one(x, prev, i, s, was_on, t, pt, pr0, pn_on, pn_done, t_pn, miss):
    """pn_step on the row i toward its target s."""
    lx, ly, lz = x[s, 0] - x[i, 0], x[s, 1] - x[i, 1], x[s, 2] - x[i, 2]
    if not was_on:
        if not math.isnan(pt):
            start = t >= pt - 1e-9
        else:
            start = math.sqrt(lx * lx + ly * ly + lz * lz) <= pr0
        if start:
            pn_on[i] = True
            t_pn[i] = t
        return
    d0x, d0y, d0z = prev[s, 0] - prev[i, 0], prev[s, 1] - prev[i, 1], prev[s, 2] - prev[i, 2]
    ddx, ddy, ddz = lx - d0x, ly - d0y, lz - d0z
    u = -(d0x * ddx + d0y * ddy + d0z * ddz) / max(ddx * ddx + ddy * ddy + ddz * ddz, 1e-300)
    u = min(max(u, 0.0), 1.0)
    cx, cy, cz = d0x + u * ddx, d0y + u * ddy, d0z + u * ddz
    closest = math.sqrt(cx * cx + cy * cy + cz * cz)
    miss[i] = closest if math.isnan(miss[i]) else min(miss[i], closest)
    if lx * (x[s, 3] - x[i, 3]) + ly * (x[s, 4] - x[i, 4]) + lz * (x[s, 5] - x[i, 5]) > 0.0:
        pn_on[i] = False
        pn_done[i] = True


@_jit
def impacts(x, active, radius_km, t, t_impact):
    """The active rows below the surface at t: their impact time set, the objects' rows (not the ship's, row 0)
    made inactive; True when the ship is below it (sim/propagate.LiveSim._impacts, the same norm to the bit: its
    squares summed from the first axis, as numpy)."""
    ship = False
    for i in range(x.shape[0]):
        if not active[i]:
            continue
        r = math.sqrt((x[i, 0] * x[i, 0] + x[i, 1] * x[i, 1]) + x[i, 2] * x[i, 2])
        if r < radius_km:
            t_impact[i] = t
            if i == 0:
                ship = True
            else:
                active[i] = False
    return ship
