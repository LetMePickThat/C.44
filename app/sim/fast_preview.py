#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FAST_PREVIEW -- the scenario editor's preview step compiled (sim/preview.py, fork RADARv6_RS, user 8 November 2026):
one RK4 step of the active rows in a single call (the central body with J2, the other bodies at the step's three
instants, the burns running at the step start in their frame at each stage, proportional navigation), the same
forces as sim/propagate.LiveSim._accel. Without numba (sim.use_numba false) the preview uses LiveSim's own step.
"""
from __future__ import annotations

import math

import numpy as np

from .fast import USE_NUMBA, _jit

FRAME_CODE = {"inertial": 0, "rtn": 1, "pnr": 2}


@_jit
def _deriv(y, tpos, tmu, mu, j2, r_eq_km, pole, axes, b_k, b_acc, b_frame, on_k, tg_k, gain, lim, out):
    """d/dt of the states y (m, 6) into out (m, 6); b_k, on_k, tg_k: indices into y."""
    m = y.shape[0]
    nb = tpos.shape[0]
    kj = 1.5 * j2 * mu * r_eq_km ** 2
    for i in range(m):
        x, yy, z = y[i, 0], y[i, 1], y[i, 2]
        rn2 = x * x + yy * yy + z * z
        rn = math.sqrt(rn2)
        d3 = rn2 * rn
        ax, ay, az = -mu * x / d3, -mu * yy / d3, -mu * z / d3
        if j2 != 0.0:
            zp = x * pole[0] + yy * pole[1] + z * pole[2]
            k = kj / (rn2 * rn2 * rn)
            f = 1.0 - 5.0 * zp * zp / rn2
            ax -= k * (f * x + 2.0 * zp * pole[0])
            ay -= k * (f * yy + 2.0 * zp * pole[1])
            az -= k * (f * z + 2.0 * zp * pole[2])
        for b in range(nb):
            px, py, pz = tpos[b, 0], tpos[b, 1], tpos[b, 2]
            rb3 = math.sqrt(px * px + py * py + pz * pz) ** 3
            dx, dy, dz = px - x, py - yy, pz - z
            dn3 = math.sqrt(dx * dx + dy * dy + dz * dz) ** 3
            ax += tmu[b] * (dx / dn3 - px / rb3)
            ay += tmu[b] * (dy / dn3 - py / rb3)
            az += tmu[b] * (dz / dn3 - pz / rb3)
        out[i, 0], out[i, 1], out[i, 2] = y[i, 3], y[i, 4], y[i, 5]
        out[i, 3], out[i, 4], out[i, 5] = ax, ay, az
    for q in range(b_k.shape[0]):                       # the burns, in their frame at this stage's state
        i = b_k[q]
        rx, ry, rz, vx, vy, vz = y[i, 0], y[i, 1], y[i, 2], y[i, 3], y[i, 4], y[i, 5]
        a0, a1, a2 = b_acc[q, 0], b_acc[q, 1], b_acc[q, 2]
        fr = b_frame[q]
        if fr == 0:
            gx = axes[0, 0] * a0 + axes[0, 1] * a1 + axes[0, 2] * a2
            gy = axes[1, 0] * a0 + axes[1, 1] * a1 + axes[1, 2] * a2
            gz = axes[2, 0] * a0 + axes[2, 1] * a1 + axes[2, 2] * a2
        else:
            hx, hy, hz = ry * vz - rz * vy, rz * vx - rx * vz, rx * vy - ry * vx      # r x v
            hn = math.sqrt(hx * hx + hy * hy + hz * hz)
            hx, hy, hz = hx / hn, hy / hn, hz / hn
            if fr == 1:                                 # rtn: X radial, Y = Z x X, Z = r x v
                rn = math.sqrt(rx * rx + ry * ry + rz * rz)
                ex, ey, ez = rx / rn, ry / rn, rz / rn
                fx, fy, fz = hy * ez - hz * ey, hz * ex - hx * ez, hx * ey - hy * ex
                gx = ex * a0 + fx * a1 + hx * a2
                gy = ey * a0 + fy * a1 + hy * a2
                gz = ez * a0 + fz * a1 + hz * a2
            else:                                       # pnr: X along v, Y = r x v, Z = X x Y
                vn = math.sqrt(vx * vx + vy * vy + vz * vz)
                ex, ey, ez = vx / vn, vy / vn, vz / vn
                fx, fy, fz = ey * hz - ez * hy, ez * hx - ex * hz, ex * hy - ey * hx
                gx = ex * a0 + hx * a1 + fx * a2
                gy = ey * a0 + hy * a1 + fy * a2
                gz = ez * a0 + hz * a1 + fz * a2
        out[i, 3] += gx
        out[i, 4] += gy
        out[i, 5] += gz
    for q in range(on_k.shape[0]):                      # proportional navigation (sim/fast.pn_command)
        i = on_k[q]
        s = tg_k[q]
        lx, ly, lz = y[s, 0] - y[i, 0], y[s, 1] - y[i, 1], y[s, 2] - y[i, 2]
        wx, wy, wz = y[s, 3] - y[i, 3], y[s, 4] - y[i, 4], y[s, 5] - y[i, 5]
        rn2 = lx * lx + ly * ly + lz * lz
        rn = math.sqrt(rn2)
        ox, oy, oz = (ly * wz - lz * wy) / rn2, (lz * wx - lx * wz) / rn2, (lx * wy - ly * wx) / rn2
        vc = -(lx * wx + ly * wy + lz * wz) / rn
        ux, uy, uz = lx / rn, ly / rn, lz / rn
        k = gain[q] * vc
        cx, cy, cz = k * (oy * uz - oz * uy), k * (oz * ux - ox * uz), k * (ox * uy - oy * ux)
        nm = math.sqrt(cx * cx + cy * cy + cz * cz)
        f = lim[q] / max(nm, 1e-300) if nm > lim[q] else 1.0
        out[i, 3] += cx * f
        out[i, 4] += cy * f
        out[i, 5] += cz * f


@_jit
def rk4(x, rows, h, tpos3, tmu, mu, j2, r_eq_km, pole, axes, b_k, b_acc, b_frame, on_k, tg_k, gain, lim, drv_k, drv):
    """One RK4 step h of the rows (indices into x, (n, 6), updated in place); tpos3 (3, nb, 3): the other bodies at
    the step's start, middle and end; b_k, on_k, tg_k: indices into rows. drv_k (indices into rows): rows driven,
    not integrated (a guided row's target interpolated, sim/preview), their states drv (3, nd, 6) at the step's
    start, middle and end; x left as it was for them."""
    m = rows.shape[0]
    y = np.empty((m, 6))
    for i in range(m):
        for c in range(6):
            y[i, c] = x[rows[i], c]
    nd = drv_k.shape[0]
    for q in range(nd):
        for c in range(6):
            y[drv_k[q], c] = drv[0, q, c]
    k1, k2, k3, k4 = np.empty((m, 6)), np.empty((m, 6)), np.empty((m, 6)), np.empty((m, 6))
    s = np.empty((m, 6))
    _deriv(y, tpos3[0], tmu, mu, j2, r_eq_km, pole, axes, b_k, b_acc, b_frame, on_k, tg_k, gain, lim, k1)
    for i in range(m):
        for c in range(6):
            s[i, c] = y[i, c] + 0.5 * h * k1[i, c]
    for q in range(nd):
        for c in range(6):
            s[drv_k[q], c] = drv[1, q, c]
    _deriv(s, tpos3[1], tmu, mu, j2, r_eq_km, pole, axes, b_k, b_acc, b_frame, on_k, tg_k, gain, lim, k2)
    for i in range(m):
        for c in range(6):
            s[i, c] = y[i, c] + 0.5 * h * k2[i, c]
    for q in range(nd):
        for c in range(6):
            s[drv_k[q], c] = drv[1, q, c]
    _deriv(s, tpos3[1], tmu, mu, j2, r_eq_km, pole, axes, b_k, b_acc, b_frame, on_k, tg_k, gain, lim, k3)
    for i in range(m):
        for c in range(6):
            s[i, c] = y[i, c] + h * k3[i, c]
    for q in range(nd):
        for c in range(6):
            s[drv_k[q], c] = drv[2, q, c]
    _deriv(s, tpos3[2], tmu, mu, j2, r_eq_km, pole, axes, b_k, b_acc, b_frame, on_k, tg_k, gain, lim, k4)
    driven = np.zeros(m, np.bool_)
    for q in range(nd):
        driven[drv_k[q]] = True
    for i in range(m):
        if driven[i]:
            continue
        for c in range(6):
            x[rows[i], c] = y[i, c] + h / 6.0 * (k1[i, c] + 2.0 * k2[i, c] + 2.0 * k3[i, c] + k4[i, c])


__all__ = ["USE_NUMBA", "FRAME_CODE", "rk4"]
