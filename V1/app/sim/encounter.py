#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENCOUNTER -- the geometry of a rendezvous between two paths (user, 1 November 2026): orbit planes, the passages of a path
through a plane (the nodes) and the closest approach, the two bodies at the same instant. Paths: arrays on one time grid,
(n_t, 3), km and km/s, NaN where the body is not there. No Qt, no drawing (gui/sim_view.py draws them).
"""
from __future__ import annotations

import math
from typing import List, NamedTuple, Optional

import numpy as np


class Crossing(NamedTuple):
    t_s: float                  # time of the passage
    r_km: np.ndarray            # the path's place there
    k: int                      # the record just before it
    ascending: bool             # from the negative side of the normal to the positive one


class Approach(NamedTuple):
    t_s: float                  # time of the closest approach
    range_km: float             # the separation then
    r_a_km: np.ndarray          # each body's place then
    r_b_km: np.ndarray
    v_rel_kms: float            # the relative speed then
    reached: bool               # False: still closing at the end of the paths (a bound, not a minimum)


def orbit_normal(r: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Unit normals of the orbit planes of the states r, v (..., 3)."""
    h = np.cross(r, v)
    return h / np.linalg.norm(h, axis=-1, keepdims=True)


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """The angle between two unit vectors: the inclination of one plane on another, from their normals."""
    return math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(a @ b)))


def crossings(ts: np.ndarray, r: np.ndarray, normal: np.ndarray) -> List[Crossing]:
    """The passages of the path r through the plane of normal (one (3,), or (n_t, 3): a plane that moves along the
    path, taken at the same instant), interpolated between the records, in time order."""
    s = np.einsum("...i,...i->...", r, normal)
    ks = np.flatnonzero(((s[:-1] <= 0.0) & (s[1:] > 0.0)) | ((s[:-1] >= 0.0) & (s[1:] < 0.0)))    # (NaN: never)
    out = []
    for k in ks:
        f = float(s[k] / (s[k] - s[k + 1]))
        out.append(Crossing(float(ts[k] + f * (ts[k + 1] - ts[k])), r[k] + f * (r[k + 1] - r[k]), int(k),
                            bool(s[k + 1] > s[k])))
    return out


def next_nodes(nodes: List[Crossing], t_s: float) -> List[Crossing]:
    """The first ascending and the first descending passages from t_s on."""
    out = []
    for asc in (True, False):
        n = next((c for c in nodes if c.ascending == asc and c.t_s >= t_s), None)
        if n is not None:
            out.append(n)
    return out


def closest_approach(ts: np.ndarray, r_a: np.ndarray, v_a: np.ndarray, r_b: np.ndarray, v_b: np.ndarray,
                     t_from: float) -> Optional[Approach]:
    """The smallest separation of the two bodies at the same instant, from t_from on: the record of the smallest
    range, refined by the relative motion at that record (a straight line, within its two neighbours). None: the two
    are never there together."""
    k0 = int(np.searchsorted(ts, t_from - 1e-9))
    rel, vrel = r_b - r_a, v_b - v_a
    d2 = np.einsum("ij,ij->i", rel, rel)
    fut = np.flatnonzero(np.isfinite(d2[k0:])) + k0
    if not len(fut):
        return None
    k = int(fut[np.argmin(d2[fut])])
    last = len(ts) - 1
    w2 = float(vrel[k] @ vrel[k])
    dt = -float(rel[k] @ vrel[k]) / w2 if w2 > 0.0 else 0.0
    lo = max(float(ts[max(k - 1, 0)]), t_from) - float(ts[k])
    hi = float(ts[min(k + 1, last)]) - float(ts[k])
    dt = min(max(dt, min(lo, 0.0)), hi)
    closing = float(rel[k] @ vrel[k]) < 0.0
    return Approach(float(ts[k]) + dt, float(np.linalg.norm(rel[k] + vrel[k] * dt)), r_a[k] + v_a[k] * dt,
                    r_b[k] + v_b[k] * dt, float(np.linalg.norm(vrel[k])), not (k == fut[-1] and closing))
