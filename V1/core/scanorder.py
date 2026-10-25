#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCAN ORDER -- the real search pointings of a volume on a face and the order they are served in (roadmap step 4.3 bis,
validated by the user on 25 October 2026).

  - Pointings: the hexagonal lattice of the sector's search plan in the face's direction cosines (u, v) (spacing
    beam_spacing_hpbw x the transmit beamwidth: the beam has a constant width in (u, v), its widening with the scan
    is exact there), kept where the sector, the face, the scan limit and the unblinded sky are.
  - Frame-to-frame shift of the lattice (step 2.3 grid dithering): a cycle of 3 shifts 0, (a + b)/3, 2(a + b)/3 of the
    lattice basis a, b, which fill the hexagonal cell.
  - Order: bit-reversed Morton (2D van der Corput): the first 4^k pointings fall one in each of the 4^k sub-squares of
    the volume, at every scale -- the coverage is uniform from the start of the frame, so a frame stretched or cut by
    the priority shedding degrades uniformly. Ties by pointing number. Changing the pointing costs nothing with the
    digital true time delay, so no locality is sought.
  - Each pointing lasts the transmit time of the waveform of its scan bin in the plan (shortened pulse); a multibeam
    dwell serves the next positions_per_dwell pointings of the order at once.
  - Outputs: pointings, frame duration, uniformity of the coverage at 10 / 25 / 50 % of the frame (share of the coarse
    5 x 5 cells of the volume already visited, against the ideal), pointing changes per second and mean step between
    two pointings (inputs of step 4.4).
The RADARv5 scan visualizer (PSCLB) was read for ideas only: its prefix-uniform principle is kept, not its grid nor
its locality bias.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import optimizer as mo
from . import radar_model as rm
from .params import P

_T = "tool.scanorder."
N_SHIFTS = int(P.get(_T + "n_shifts"))
COARSE_CELLS = int(P.get(_T + "coarse_cells"))
FRACTIONS = tuple(P.get(_T + "uniformity_fractions"))
MORTON_BITS = 16


def face_uv(sph: "mo.Sphere", k: int) -> Tuple[np.ndarray, np.ndarray]:
    """(u, v) of every raster cell in the frame of face k (direction = u e_u + v e_v + w n)."""
    n, eu, ev = mo.face_frame(mo.FACE_PHI[k])
    p, e = np.radians(sph.phi), np.radians(sph.eps)
    dx, dy, dz = np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)
    return dx * eu[0] + dy * eu[1], dx * ev[0] + dy * ev[1] + dz * ev[2]


def lattice(sel: np.ndarray, uu: np.ndarray, vv: np.ndarray, d: float, shift: int) -> np.ndarray:
    """Points (N, 2) of the hexagonal lattice of spacing d, shifted by shift/3 (a + b), inside the cells sel (an
    occupancy grid in (u, v) of step d/4 built from the raster cells)."""
    us, vs = uu[sel], vv[sel]
    if us.size == 0:
        return np.zeros((0, 2))
    step = d / 4.0
    occ = set(zip(np.floor(us / step).astype(int).tolist(), np.floor(vs / step).astype(int).tolist()))
    a, b = np.array([d, 0.0]), np.array([d / 2.0, d * math.sqrt(3.0) / 2.0])
    off = (shift % N_SHIFTS) / N_SHIFTS * (a + b)
    jmin, jmax = int(math.floor(vs.min() / b[1])) - 1, int(math.ceil(vs.max() / b[1])) + 1
    pts = []
    for j in range(jmin, jmax + 1):
        y = j * b[1] + off[1]
        x0 = j * b[0] + off[0]
        imin = int(math.floor((us.min() - x0) / d)) - 1
        imax = int(math.ceil((us.max() - x0) / d)) + 1
        for i in range(imin, imax + 1):
            x = x0 + i * d
            if (int(math.floor(x / step)), int(math.floor(y / step))) in occ:
                pts.append((x, y))
    return np.array(pts) if pts else np.zeros((0, 2))


def _spread(x: np.ndarray) -> np.ndarray:
    """Spread the MORTON_BITS low bits of x to the even bit positions."""
    x = x.astype(np.uint64) & np.uint64((1 << MORTON_BITS) - 1)
    for s, m in ((8, 0x00FF00FF), (4, 0x0F0F0F0F), (2, 0x33333333), (1, 0x55555555)):
        x = (x | (x << np.uint64(s))) & np.uint64(m)
    return x


def order_bitrev_morton(pts: np.ndarray, d: Optional[float] = None) -> np.ndarray:
    """Indices of the pointings in bit-reversed Morton order (ties by index). The points are binned on square cells
    of the volume's bounding square, 2^L per side, 2^L the largest power of two below the extent over the point
    spacing d (estimated as
    sqrt(area / N) when not given): the Morton code of the cell, bits reversed, puts the coarse halvings first."""
    n = len(pts)
    if n == 0:
        return np.zeros(0, dtype=int)
    lo = pts.min(axis=0)
    ext = np.ptp(pts, axis=0)
    span = max(float(ext.max()), 1e-12)
    if d is None:
        d = math.sqrt(max(float(ext[0]) * float(ext[1]), span * 1e-6) / n)
    # cells no finer than the spacing: every bit of the cell code then splits the points evenly
    lev = int(min(MORTON_BITS, max(1, math.floor(math.log2(span / max(d, 1e-12) + 1.0)))))
    side = 1 << lev
    q = np.minimum(((pts - lo) / span * side).astype(np.int64), side - 1)
    code = _spread(q[:, 0]) | (_spread(q[:, 1]) << np.uint64(1))
    rev = np.zeros(n, dtype=np.uint64)
    c = code.copy()
    for _ in range(2 * lev):
        rev = (rev << np.uint64(1)) | (c & np.uint64(1))
        c = c >> np.uint64(1)
    return np.lexsort((np.arange(n), rev))


def uniformity(pts: np.ndarray, order: np.ndarray) -> Dict[str, float]:
    """Share of the coarse cells (COARSE_CELLS^2 over the volume, the non-empty ones) visited after each fraction of
    the frame, over the ideal min(1, fraction x N / cells)."""
    n = len(pts)
    if n == 0:
        return {}
    lo, span = pts.min(axis=0), np.maximum(np.ptp(pts, axis=0), 1e-12)
    cell = np.minimum(((pts - lo) / span * COARSE_CELLS).astype(int), COARSE_CELLS - 1)
    cid = cell[:, 0] * COARSE_CELLS + cell[:, 1]
    n_cells = len(np.unique(cid))
    out = {}
    for f in FRACTIONS:
        m = max(1, int(round(f * n)))
        seen = len(np.unique(cid[order[:m]]))
        out[f"{int(round(100 * f))}%"] = seen / min(n_cells, m)
    return out


def mean_step_deg(pts: np.ndarray, order: np.ndarray) -> float:
    """Mean distance between two consecutive pointings, as an angle near boresight (input of step 4.4)."""
    if len(order) < 2:
        return 0.0
    d = np.hypot(*np.diff(pts[order], axis=0).T)
    return float(np.degrees(np.mean(d)))


def _bin_time(r: dict) -> List[Tuple[float, float, float, int]]:
    """(theta_lo, theta_hi, transmit time per position, positions per dwell) of the plan's scan bins."""
    out = []
    for b in r.get("wf_by_bin", []):
        w = b["wf"]
        cpd = max(1.0, float(w.get("positions_per_dwell", 1.0)))
        t = w["n_pulses"] * w["tau_us"] * 1e-6 * float(b.get("pulse_scale", 1.0)) / cpd
        out.append((b["theta_lo_deg"], b["theta_hi_deg"], t, int(round(cpd))))
    return out


def sector_scan(sec: dict, r: dict, sph: "mo.Sphere", sky: Optional[dict], shift: int = 0) -> Dict[int, dict]:
    """Pointings of an optimised sector per face (the frame of shift `shift`), in scan order, with their transmit times
    and the multibeam grouping, and the metrics of the order."""
    f = float(r["frequency"]["freq_ghz"])
    spacing = float(r.get("beam_spacing_hpbw", rm.BEAM_SPACING_HPBW))
    m = sph.mask(sec)
    _, keep = mo.sector_sky(sec, sph, m, f, "S", sky)
    lim = rm.band_for_frequency(f).scan_limit_deg(f)
    sel0 = m & sph.scan_zone & (sph.theta <= lim) & (np.asarray(keep) > 0.5)
    d = spacing * rm._pattern(f).tx.hpbw * rm.lambda_m(f) / rm.ARRAY_ENVELOPE_M
    bins = _bin_time(r)
    out = {}
    for k in range(4):
        if str(k) not in r.get("faces", {}):
            continue
        uu, vv = face_uv(sph, k)
        pts = lattice(sel0 & (sph.owner == k), uu, vv, d, shift)
        if len(pts) == 0:
            continue
        order = order_bitrev_morton(pts, d)
        theta = np.degrees(np.arcsin(np.clip(np.hypot(pts[:, 0], pts[:, 1]), 0.0, 1.0)))
        dur = np.zeros(len(pts))
        cpd = np.ones(len(pts), dtype=int)
        for lo_, hi_, t, c in bins:
            sel = (theta >= lo_) & (theta < hi_ + 1e-9)
            dur[sel], cpd[sel] = t, c
        if bins:
            last = max(bins, key=lambda x: x[1])
            sel = theta >= last[1]
            dur[sel], cpd[sel] = last[2], last[3]
        # dwells: the next positions_per_dwell pointings of the order served together
        dwells, i = [], 0
        while i < len(order):
            c = int(cpd[order[i]])
            grp = order[i:i + c]
            dwells.append(float(dur[grp].sum()))
            i += c
        rr = float(r["rr_target_s"])
        out[k] = {"pointings": len(pts), "plan_positions": float(r["faces"][str(k)]["positions"]),
                  "dwell_times_s": dwells, "frame_tx_s": float(sum(dwells)),
                  "uniformity": uniformity(pts, order), "mean_step_deg": mean_step_deg(pts, order),
                  "pointing_rate_hz": len(dwells) / rr}
    return out
