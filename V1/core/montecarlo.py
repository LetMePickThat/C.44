#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MONTE-CARLO -- the mission plan played by random draws, against the analytic model of the Pmiss (roadmap step 5.4,
blocks A-H validated by the user on 27 October 2026). On demand only (A2): `python -m core.montecarlo <mission>`; the
plan never changes (A3). The scene is the mission's navigation snapshot; the dense scene (masking, polluted CFAR) is
phase 6 and the reflections on the structure are out of the model (user, 27 October 2026).

Pass M1: the test targets of the search (blocks B and C) and the cumulative Pd of each search sector.
  - Plan: the waveform, Pfa, pulse scale and full-pulse range of each 1-deg scan bin, as the optimiser kept it
    (optimizer.run(keep_plans=True)); the SNR of a look is READ from the plan, not solved again (B3): mean SNR before
    the position losses at the requirement range = the required SNR of the bin (cumulative criterion, positions law)
    + 40 log10(reach / R), reach = full-pulse range x pulse_scale^(1/4) (sky, jammers and eclipse inside it, as the
    plan has them). Noise-limited scaling (R / r)^4 between the looks; a sector with a self-protection jammer is flagged
    (its burn-through goes as 1 / r^2).
  - Test target (C1-C6): one per draw, scan bin drawn in proportion to the bin's positions (masked and blinded ones are
    not in the plan), closing at v_close_max (or uniform between v_close_min and v_close_max, v_draw), looks at
    R_bin (1 + (k + phi) q), q = v RR / R_bin, phase phi uniform (the criterion of step 2.3); values of the sector, else
    of its class; frontal RCS. Every look whose mean Pd stays above pd_look_floor counts, without a time bound (as the
    analytic criterion; user, 27 October 2026).
  - Position of the target (B4), DRAWN instead of averaged: a point uniform in the TX lattice cell, FIXED for the life
    of the target, seen through the frame-to-frame lattice shifts of step 4.3 bis (cycle of tool.scanorder.n_shifts
    shifts (a + b) j / n): TX offset = the point reduced into the shifted TX cell, RX offset = that offset reduced into
    the RX lattice of the pointing; Doppler straddle and velocity-filter offset fixed per target (constant velocity);
    range straddle drawn per look (the target crosses many range cells between two revisits). A partition (sub-aperture)
    beam draws its TX loss per look from the analytic samples (flagged). The analytic model averages every position
    loss independently look by look: the difference is what this pass measures.
  - Fluctuation (B5): target power over a look Gamma(k) with the look's shape (Swerling case, class family and T0,
    agility, polarisation diversity); between looks the same draw if RR < T0, else a new one.
  - Decision (B6): the statistic Z = sum of the n noncoherent samples (noncentral chi-square, unit complex noise per
    sample) drawn exactly against the threshold of the bin's Pfa (homogeneous noise; the CFAR loss stays in the plan).
  - Statistics (G1-G3): seed per sector, batches of `batch` targets up to n_max, stop from n_min when the Wilson
    half-width at ci_level is below ci_tol.
  - State (A3, per sector, on the positions within range): NOK when the upper bound of the interval is below the
    sector's Pd; ALERTE when the analytic Pd is above the upper bound (model optimistic); else OK.
To come: M2 false alarms, confirmation, scheduler latencies, eclipse drawn (event file of the timeline); M3 tracks;
M4 verdict and load_ceiling; M5 tests at N3, snapshots, docs.
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import detection
from . import optimizer as mo
from . import radar_model as rm
from .params import P

_T = "tool.montecarlo."
SEED = int(P.get(_T + "seed"))
N_MIN = int(P.get(_T + "n_min"))
N_MAX = int(P.get(_T + "n_max"))
CI_TOL = float(P.get(_T + "ci_tol"))
CI_LEVEL = float(P.get(_T + "ci_level"))
HORIZON_MAX_S = float(P.get(_T + "horizon_max_s"))          # scheduler timeline of a draw (M2)
WARMUP_REVISITS = int(P.get(_T + "warmup_revisits"))        # revisits played before the counted time (M2)
V_DRAW = str(P.get(_T + "v_draw"))
BATCH = int(P.get(_T + "batch"))
PD_LOOK_FLOOR = float(P.get(_T + "pd_look_floor"))
N_SHIFTS = int(P.get("tool.scanorder.n_shifts"))
K_MAX_LOOKS = 20000                       # safety bound on the looks of one test target (flagged if it cuts)


# ----------------------------------------------------------------------------- statistics
def _z(level: float) -> float:
    """Two-sided normal quantile of a confidence level (bisection on erf)."""
    lo, hi, p = 0.0, 10.0, 0.5 + level / 2.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if 0.5 * (1.0 + math.erf(mid / math.sqrt(2.0))) < p else (lo, mid)
    return 0.5 * (lo + hi)


def wilson(k: int, n: int, level: float = CI_LEVEL) -> Tuple[float, float]:
    """Wilson interval of a proportion k / n (G3)."""
    if n <= 0:
        return 0.0, 1.0
    z = _z(level)
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2.0 * n)) / d
    h = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


# ----------------------------------------------------------------------------- lattices
def _basis(d: float) -> np.ndarray:
    """Basis (rows a, b) of the triangular lattice of step d, as radar_model._hex_cell_points."""
    return np.array([[d, 0.0], [d / 2.0, d * math.sqrt(3.0) / 2.0]])


def reduce_hex(p: np.ndarray, d: float) -> np.ndarray:
    """Offset of points p (N, 2) to their nearest site of the triangular lattice of step d (the hexagonal cell)."""
    b = _basis(d)
    c = p @ np.linalg.inv(b)
    f = np.floor(c)
    best, best_d = None, None
    for i in (0.0, 1.0):
        for j in (0.0, 1.0):
            site = (f + np.array([i, j])) @ b
            off = p - site
            dist = (off * off).sum(axis=-1)
            if best is None:
                best, best_d = off, dist
            else:
                sel = dist < best_d
                best = np.where(sel[..., None], off, best)
                best_d = np.where(sel, dist, best_d)
    return best


# ----------------------------------------------------------------------------- one scan bin of the plan
@dataclass
class BinModel:
    """What a look of one scan bin needs (B3-B6). Losses in dB; arrays of equally likely samples."""
    n: int                                  # noncoherent samples (coherent blocks)
    shape: float                            # Gamma shape of the target power over a look (inf: steady)
    threshold: float                        # of the statistic, from the bin's Pfa
    x_req_db: float                         # analytic required mean SNR at R_bin before the position losses
    x_avail_db: float                       # SNR the plan gives at R_bin before the position losses
    r_bin_km: float                         # requirement range of the bin
    short: bool                             # the plan does not reach the sector's range there
    pos_l: Tuple[float, ...]                # analytic positions law (losses, weights)
    pos_w: Tuple[float, ...]
    pfa: float
    tx_cell: float = 0.0                    # TX lattice step (lambda / D); 0: no lattice loss
    tx_beam: object = None
    rx_cell: float = 0.0
    rx_beam: object = None
    tx_samples: Optional[np.ndarray] = None  # partition beam: TX loss drawn per look from these
    range_samples: np.ndarray = field(default_factory=lambda: np.zeros(1))
    doppler_samples: np.ndarray = field(default_factory=lambda: np.zeros(1))
    velocity_samples: np.ndarray = field(default_factory=lambda: np.zeros(1))
    notes: Tuple[str, ...] = ()

    def curve(self) -> np.ndarray:
        return detection.mean_curve(self.n, detection.Fluct(self.shape), self.pfa, self.pos_l, self.pos_w)


def analytic_pcum(curve: np.ndarray, q: float, x_db: float, floor: float = rm.CUM_PD_FLOOR) -> float:
    """Cumulative Pd of the analytic model at mean SNR x_db at the requirement range (detection._required_snr_db)."""
    if q <= 0.0:
        return float(np.interp(x_db, detection.SNR_GRID_DB, curve))
    sh = detection._look_shifts(round(q, 9))
    pk = np.interp(x_db - sh, detection.SNR_GRID_DB, curve, left=float(curve[0]), right=float(curve[-1]))
    pk = np.where(pk < floor, 0.0, pk)
    return float(np.mean(1.0 - np.prod(1.0 - pk, axis=1)))


def bin_model(w, f: float, theta: float, tgt, sec: dict, pfa: float, spacing: Optional[float], reach_km: float,
              short: bool) -> BinModel:
    """The look of a planned waveform in one scan bin (same solve as the optimiser: optimizer.parts_at path)."""
    r_req = float(sec["range_km"])
    r_bin = r_req if (short or reach_km >= r_req * (1.0 - 1e-9)) else reach_km   # surface-limited bin: its distance
    cum = (float(sec["v_close_max_mps"]), float(sec["rr_target_s"]), r_bin) + mo.search_criterion(sec)[3:]
    wb = w if w.tx_mode != "superposition" else dataclasses.replace(w, b_tx=1, tx_mode="single")   # parts_at twin
    b = rm.solve_base(wb, f, theta, tgt, mo.rx_sll_db(sec), mo.velocity_span_mps(sec))
    k = rm.solve_look(b, pfa=pfa, cum=cum, grid_spacing_hpbw=spacing)
    x_req = k.snr_db + k.positions_db
    notes = []
    band = b.band.name
    pat = rm.PATTERNS[band]
    sp = rm._spacing(spacing)
    tx_cell = tx_beam = rx_cell = rx_beam = tx_samples = None
    if rm.ENABLE_GRID_LOSS:
        sub = rm.tx_subaperture(w)
        if sub is None:
            tx_cell, tx_beam = sp * pat.tx.hpbw, pat.tx
        else:
            tx_samples = rm.subaperture_grid_losses_db(band, sub, spacing)
            notes.append("partition beam: TX lattice loss drawn per look")
        rx_beam = rm.rx_beam(band, b.sll_key)
        rx_cell = rm.RX_BEAM_SPACING_HPBW * rx_beam.hpbw
    # the same samples as radar_model._fixed_position_law / position_losses (step 2.3), kept apart to be drawn
    u = (np.arange(401) + 0.5) / 401 - 0.5
    rng_s = -10.0 * np.log10(rm._range_response(b.psl)(u / rm.RANGE_GRID_OVERSAMPLING))
    dop_s = np.zeros(1)
    if b.ppb > 1:
        resp = (np.sinc(u / rm.DOPPLER_ZERO_PADDING) ** 2 if w.doppler_window_db is None
                else rm._range_response(w.doppler_window_db)(u / rm.DOPPLER_ZERO_PADDING))
        dop_s = -10.0 * np.log10(resp)
    vel_s = np.zeros(1)
    if b.bank_half > 0.0:
        vel_s = np.array([rm.bank_mismatch_loss_db(dv, w.tau_s, w.sig_bw_mhz, b.t_span, w.n, b.n_blocks == 1)
                          for dv in (np.arange(41) + 0.5) / 41 * b.bank_half])
    reach = reach_km if reach_km > 0 else r_bin
    return BinModel(n=b.n_blocks, shape=detection.shape_of(b.fl, b.n_blocks), threshold=detection.threshold(b.n_blocks, pfa),
                    x_req_db=x_req, x_avail_db=x_req + 40.0 * math.log10(reach / r_bin), r_bin_km=r_bin, short=short,
                    pos_l=tuple(k.pos_l), pos_w=tuple(k.pos_w), pfa=pfa, tx_cell=tx_cell or 0.0, tx_beam=tx_beam,
                    rx_cell=rx_cell or 0.0, rx_beam=rx_beam, tx_samples=tx_samples, range_samples=rng_s,
                    doppler_samples=dop_s, velocity_samples=vel_s, notes=tuple(notes))


# ----------------------------------------------------------------------------- the draws
def draw_targets(bm: BinModel, rng: np.random.Generator, n: int, rr_s: float, v_mps: np.ndarray,
                 t0_s: float, correlated: bool = True) -> Tuple[np.ndarray, bool]:
    """(detected (bool, n), horizon cut) for n test targets of one bin: any look at R_bin (1 + (k + phi) q) detects;
    cut = looks that could still detect were left out by the K_MAX_LOOKS safety bound. Every look whose mean Pd stays
    above pd_look_floor is drawn, without a time bound, as the analytic criterion counts them (user, 27 October 2026:
    horizon_max_s cut the worst-range window of slow far targets; it bounds the scheduler timeline of M2 only).
    correlated False: every position loss drawn anew at each look (the analytic assumption, for the tests)."""
    q = v_mps * rr_s / (bm.r_bin_km * 1e3)                           # (n,)
    curve = bm.curve()
    # looks that can still detect: mean Pd above the floor (at the smallest q, the most looks)
    q_min = float(q.min())
    k_cap = K_MAX_LOOKS
    k_need = 1
    if q_min > 0:
        k_try = np.arange(1, K_MAX_LOOKS + 2)
        pk = np.interp(bm.x_avail_db - 40.0 * np.log10(1.0 + (k_try - 1) * q_min), detection.SNR_GRID_DB, curve,
                       left=float(curve[0]), right=float(curve[-1]))
        alive = np.flatnonzero(pk >= PD_LOOK_FLOOR)
        k_need = int(alive.max()) + 1 if len(alive) else 1
    K = max(1, min(k_need, k_cap))
    phi = rng.random(n)
    kk = np.arange(K)[None, :] + phi[:, None]                           # (n, K)
    snr_db = bm.x_avail_db - 40.0 * np.log10(1.0 + kk * q[:, None])
    loss = np.zeros((n, K))
    # angular offsets: a point of the TX cell fixed per target, seen through the frame-to-frame lattice shifts
    if bm.tx_cell > 0.0 or bm.rx_cell > 0.0:
        basis = _basis(bm.tx_cell) if bm.tx_cell > 0.0 else _basis(1.0)
        if correlated:
            p0 = rng.random((n, 2)) @ basis                              # uniform over one cell (modulo the lattice)
            j0 = rng.integers(0, N_SHIFTS, n)
            j = (j0[:, None] + np.arange(K)[None, :]) % N_SHIFTS
            pts = p0[:, None, :] - (j[..., None] / N_SHIFTS) * (basis[0] + basis[1])[None, None, :]
        else:
            pts = rng.random((n, K, 2)) @ basis
        d_tx = reduce_hex(pts, bm.tx_cell) if bm.tx_cell > 0.0 else pts
        if bm.tx_cell > 0.0:
            loss += rm._beam_loss_db(bm.tx_beam, np.hypot(d_tx[..., 0], d_tx[..., 1]))
        if bm.rx_cell > 0.0:
            d_rx = reduce_hex(d_tx, bm.rx_cell)
            loss += rm._beam_loss_db(bm.rx_beam, np.hypot(d_rx[..., 0], d_rx[..., 1]))
    if bm.tx_samples is not None:
        loss += bm.tx_samples[rng.integers(0, len(bm.tx_samples), (n, K))]
    loss += bm.range_samples[rng.integers(0, len(bm.range_samples), (n, K))]
    shape_nk = (n, 1) if correlated else (n, K)
    loss += bm.doppler_samples[rng.integers(0, len(bm.doppler_samples), shape_nk)]
    loss += bm.velocity_samples[rng.integers(0, len(bm.velocity_samples), shape_nk)]
    lam = 10.0 ** ((snr_db - loss) / 10.0)                              # mean total SNR of each look
    if math.isfinite(bm.shape) and bm.shape <= 1e4:                    # as detection.pd_curve (_STEADY_SHAPE)
        same = correlated and rr_s < t0_s
        g = rng.gamma(bm.shape, 1.0 / bm.shape, (n, 1) if same else (n, K))
        lam = lam * g
    z = 0.5 * rng.noncentral_chisquare(2 * bm.n, np.maximum(2.0 * lam, 1e-300))
    return (z > bm.threshold).any(axis=1), k_need > k_cap


# ----------------------------------------------------------------------------- one sector
def sector_mc(sec: dict, plan: dict, index: int, seed: int = SEED, correlated: bool = True) -> dict:
    """Cumulative Pd of the test targets of one search sector, by draws, against the analytic model (M1)."""
    t_start = time.perf_counter()
    f = float(plan["freq_ghz"])
    nb = len(plan["wf"])
    theta = mo._theta_centres(f, nb)
    tgt = mo.sector_target(sec)
    weights = np.array([float(plan["positions"][i]) if plan["wf"][i] is not None else 0.0 for i in range(nb)])
    if weights.sum() <= 0.0:
        return {"id": sec["id"], "status": "NO_PLAN"}
    models: Dict[int, BinModel] = {}
    notes = set()
    for i in np.flatnonzero(weights > 0):
        reach = float(plan["range_km"][i]) * float(plan["pulse_scale"][i]) ** 0.25
        models[int(i)] = bin_model(plan["wf"][i], f, float(theta[i]), tgt, sec, float(plan["pfa"][i]),
                                   plan.get("beam_spacing_hpbw"), reach, bool(plan["short"][i]))
        notes.update(models[int(i)].notes)
    if mo.env.spj_density_w_hz(sec, f) is not None:
        notes.add("self-protection jammer: looks scaled as noise-limited (burn-through goes as 1/r^2)")
    rr = float(sec["rr_target_s"])
    vmax = float(sec["v_close_max_mps"])
    vmin = float(sec.get("v_close_min_mps", vmax))
    t0 = tgt.t0_s(f)
    pd_req = float(sec["pd"]) if sec.get("pd") is not None else rm.DETECT_PD
    # analytic expectation, weighted over the bins (the plan's law at the plan's SNR)
    p_w = weights / weights.sum()
    an = {}
    for i, bm in models.items():
        an[i] = analytic_pcum(bm.curve(), vmax * rr / (bm.r_bin_km * 1e3), bm.x_avail_db)
    in_range = np.array([0.0 if (i not in models or models[i].short) else weights[i] for i in range(nb)])
    pd_an = float(sum(p_w[i] * an[i] for i in models))
    pd_an_in = (float(sum(in_range[i] * an[i] for i in models) / in_range.sum()) if in_range.sum() > 0 else None)
    rng = np.random.default_rng([int(seed), int(index)])
    bins = np.array(sorted(models))
    probs = weights[bins] / weights[bins].sum()
    hits = tries = hits_in = tries_in = 0
    while tries < N_MAX:
        nb_draw = min(BATCH, N_MAX - tries)
        pick = bins[rng.choice(len(bins), nb_draw, p=probs)]
        for i in np.unique(pick):
            m = int((pick == i).sum())
            v = (np.full(m, vmax) if V_DRAW != "uniform" else rng.uniform(vmin, vmax, m))
            det, cut = draw_targets(models[int(i)], rng, m, rr, v, t0, correlated)
            if cut:
                notes.add(f"looks beyond {K_MAX_LOOKS} left out (pessimistic)")
            hits += int(det.sum())
            tries += m
            if not models[int(i)].short:
                hits_in += int(det.sum())
                tries_in += m
        lo, hi = wilson(hits_in, tries_in) if tries_in else wilson(hits, tries)
        if tries >= N_MIN and (hi - lo) / 2.0 < CI_TOL:
            break
    ci = wilson(hits, tries)
    ci_in = wilson(hits_in, tries_in) if tries_in else (None, None)
    pd_mc_in = hits_in / tries_in if tries_in else None
    if pd_mc_in is None:
        state, why = "NOK", "no position within range"
    elif ci_in[1] < pd_req:
        state, why = "NOK", f"Pd {pd_mc_in:.3f} (upper bound {ci_in[1]:.3f}) below the required {pd_req:g}"
    elif pd_an_in is not None and pd_an_in > ci_in[1]:
        state, why = "ALERTE", f"analytic Pd {pd_an_in:.3f} above the upper bound {ci_in[1]:.3f} (model optimistic)"
    else:
        state, why = "OK", ""
    return {"id": sec["id"], "status": "OK", "pd_required": pd_req, "targets": tries, "targets_in_range": tries_in,
            "pd_mc": hits / tries, "ci": list(ci), "pd_mc_in_range": pd_mc_in, "ci_in_range": list(ci_in),
            "pd_analytic": pd_an, "pd_analytic_in_range": pd_an_in,
            "short_share": float(1.0 - in_range.sum() / weights.sum()), "state": state, "why": why,
            "v_draw": V_DRAW, "correlated": correlated, "notes": sorted(notes),
            "runtime_s": time.perf_counter() - t_start}


# ----------------------------------------------------------------------------- mission
def run(mission: dict, out: Optional[dict] = None, seed: int = SEED, progress_cb=None) -> dict:
    """Monte-Carlo of a mission (M1: cumulative Pd of the search sectors). out: a run kept with its plans
    (optimizer.run(keep_plans=True)); None = computed here."""
    t_start = time.perf_counter()
    if out is None or not any(r.get("_plan") for r in out.get("sectors", [])):
        out = mo.run(mission, keep_plans=True)
    plans = {r["id"]: r.get("_plan") for r in out.get("sectors", [])}
    rows = []
    for n, sec in enumerate(mission["sectors"]):
        plan = plans.get(sec["id"])
        rows.append(sector_mc(sec, plan, n, seed) if plan else {"id": sec["id"], "status": "NO_PLAN"})
        if progress_cb:
            progress_cb(n + 1, len(mission["sectors"]), sec["id"])
    states = [r.get("state") for r in rows if r.get("status") == "OK"]
    state = "NOK" if "NOK" in states else ("ALERTE" if "ALERTE" in states else "OK")
    return {"pass": "M1", "seed": int(seed), "ci_level": CI_LEVEL, "state": state, "sectors": rows,
            "runtime_s": time.perf_counter() - t_start}


def report_lines(mc: dict) -> List[str]:
    """Text report of a Monte-Carlo run (H3)."""
    lines = [f"MONTE-CARLO (step 5.4, pass {mc['pass']}, seed {mc['seed']}, intervals {mc['ci_level']:.0%}): "
             f"{mc['state']} ({mc['runtime_s']:.1f} s)",
             f"  {'sector':<14}{'targets':>8}{'Pd req':>8}{'analytic':>10}{'Monte-Carlo':>13}{'interval':>17}  state"]
    for r in mc["sectors"]:
        if r.get("status") != "OK":
            lines.append(f"  {r['id']:<14}  no plan")
            continue
        a = r["pd_analytic_in_range"]
        m = r["pd_mc_in_range"]
        ci = r["ci_in_range"]
        lines.append(f"  {r['id']:<14}{r['targets']:>8}{r['pd_required']:>8.3f}"
                     f"{(f'{a:.3f}' if a is not None else '-'):>10}{(f'{m:.3f}' if m is not None else '-'):>13}"
                     f"{(f'[{ci[0]:.3f}, {ci[1]:.3f}]' if ci[0] is not None else '-'):>17}  {r['state']}"
                     + (f" -- {r['why']}" if r["why"] else ""))
        if r["short_share"] > 0:
            lines.append(f"  {'':<14}positions short of range {r['short_share']:.1%} (all positions: Pd "
                         f"{r['pd_mc']:.3f}, analytic {r['pd_analytic']:.3f})")
        for note in r["notes"]:
            lines.append(f"  {'':<14}note: {note}")
    return lines


def _clean(x):
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_clean(v) for v in x]
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return x


def main(argv: List[str]) -> int:
    if not argv:
        print("usage: python -m core.montecarlo <mission.json|.csv> [result.json]")
        return 2
    mission = mo.load_mission(argv[0])
    mc = run(mission, progress_cb=lambda d, n, s: print(f"  {d}/{n} {s}", flush=True))
    print("\n".join(report_lines(mc)))
    if len(argv) > 1:
        with open(argv[1], "w", encoding="utf-8") as fh:
            json.dump(_clean(mc), fh, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
