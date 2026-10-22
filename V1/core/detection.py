#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DETECTION -- exact detection laws and the detection criterion (roadmap step 2.3).

Square-law detector on n samples integrated noncoherently (the coherent blocks of radar_model), complex noise of unit
power per sample: the statistic Z = sum |y_i|^2 is Gamma(n, 1) under noise alone, so Pfa = Q(n, T) = P(Poisson(T)
<= n - 1). With a signal of total SNR L = sum |a_i|^2 (energy over the n samples, the convention of the
energy-form radar equation), Z is noncentral and Pd = sum_k Pois(k; L) Q(n + k, T) (steady target, Swerling 0).
The fluctuating cases are mixtures of that law over the target power, Gamma distributed with mean x:
  Swerling 1: one draw for the n samples, shape 1 (Gauss-Laguerre);   Swerling 3: shape 2 (Gauss-Laguerre, weight u)
  Swerling 2: one draw per sample, shape n -> closed form Pd = Q(n, T / (1 + x / n))
  Swerling 4: one draw per sample, shape 2n (equal-probability quantiles of the Gamma law)
(the noncentral sum only depends on the total noncentrality, so per-sample draws enter through their sum). Valid for
any Pfa (the Shnidman fit of radar_model holds for Pfa >= 1e-9 only; it stays as a cross-check).
Step 2.4: a look's fluctuation may also be given as Fluct(k), the Gamma shape of the target power over the look
(k = 1 SW1, 2 SW3, n SW2, 2n SW4, infinite = steady), so that partial decorrelation (1 + T_obs / T0 independent
samples) and the polarisation diversity of the two receive channels enter as a non-integer shape.

Criterion (step 2.3): the Pd of a look is AVERAGED over the random position of the target (search grid cell, range /
Doppler straddle, velocity-filter offset), given as a discrete distribution of losses; the search requirement is
the CUMULATIVE probability of detection over the looks of a target closing at v, spaced v x RR in range, averaged
over the phase of the looks, reached by the requirement range R: P_cum(R) = E_phi[1 - prod_k (1 - Pd(R + (k + phi)
v RR))], with Pd(R') = Pd(R) scaled by (R / R')^4 in SNR.
"""
from __future__ import annotations

import functools
import math
from dataclasses import dataclass
from typing import Sequence, Tuple, Union

import numpy as np

SNR_GRID_DB = np.arange(-40.0, 60.0 + 1e-9, 0.05)        # total SNR grid of the tabulated laws
_LAGUERRE = np.polynomial.laguerre.laggauss(64)
_N_GAMMA_Q = 96                                           # quantiles of the Swerling 4 mixture
_N_PHASES = 16                                            # phases of the looks (cumulative criterion)
_STEADY_SHAPE = 1e4                                       # Gamma shapes above this are taken as a steady target


@dataclass(frozen=True)
class Fluct:
    """Fluctuation of a look as the Gamma shape k of the target power over its n samples (step 2.4): math.inf =
    steady. A Swerling case is the special value k = 1 (SW1), 2 (SW3), n (SW2), 2n (SW4)."""
    k: float


Fluctuation = Union[int, Fluct]                            # a Swerling case 0..4, or a Gamma shape


def shape_of(fluct: Fluctuation, n: int) -> float:
    """Gamma shape of the target power over n samples (math.inf = steady)."""
    if isinstance(fluct, Fluct):
        return fluct.k
    k = {0: math.inf, 1: 1.0, 2: float(n), 3: 2.0, 4: 2.0 * n}.get(fluct)
    if k is None:
        raise ValueError(f"Swerling case {fluct} must be 0..4")
    return k


def _log_poisson_cdf(m_max: int, t: float) -> np.ndarray:
    """log P(Poisson(t) <= j) for j = 0 .. m_max (stable)."""
    j = np.arange(m_max + 1)
    logpmf = -t + j * math.log(max(t, 1e-300)) - np.array([math.lgamma(k + 1.0) for k in j])
    return np.logaddexp.accumulate(logpmf)


def threshold(n: int, pfa: float) -> float:
    """Threshold T of the noise-alone statistic Gamma(n, 1): Q(n, T) = P(Poisson(T) <= n - 1) = pfa."""
    lo, hi = 0.0, max(10.0, n + 20.0 * math.sqrt(n) - 2.0 * math.log(pfa) * 1.5)
    target = math.log(pfa)
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _log_poisson_cdf(n - 1, mid)[-1] > target else (lo, mid)
        if hi - lo < 1e-12 * max(1.0, hi):
            break
    return 0.5 * (lo + hi)


def _pd_steady(n: int, t: float, snr_lin: np.ndarray) -> np.ndarray:
    """Pd of a steady target, total SNR snr_lin (array): sum_k Pois(k; L) P(Poisson(T) <= n + k - 1). Evaluated
    up to L_cap = (sqrt(T) + 12)^2 + 4n, beyond which Pd = 1 to better than 1e-12 (vectorised over k)."""
    out = np.ones_like(snr_lin, dtype=float)
    lam_cap = (math.sqrt(t) + 12.0) ** 2 + 4.0 * n
    sel = snr_lin < lam_cap
    lam = snr_lin[sel]
    k_max = int(lam_cap + 14.0 * math.sqrt(lam_cap) + 40)
    k = np.arange(k_max + 1)
    log_fact = np.concatenate([[0.0], np.cumsum(np.log(np.arange(1, k_max + 1)))])
    log_cdf = _log_poisson_cdf(k_max + n, t)
    q = np.exp(log_cdf[n - 1 + k])                          # P(Poisson(T) <= n + k - 1)
    log_w = -lam[:, None] + k[None, :] * np.log(np.maximum(lam, 1e-300))[:, None] - log_fact[None, :]
    out[sel] = (np.exp(log_w) * q[None, :]).sum(axis=1)
    return np.clip(out, 0.0, 1.0)


def _mixture_db(pd0: np.ndarray, shape: float, pd_left: float) -> np.ndarray:
    """Pd of a target whose power is Gamma(shape) distributed with mean 1, from the steady law pd0 on SNR_GRID_DB:
    convolution over the grid with the law of 10 log10(u) (exact up to the 0.02 dB grid)."""
    step = SNR_GRID_DB[1] - SNR_GRID_DB[0]
    delta = np.arange(-80.0, 25.0 + 1e-9, step)              # dB offsets of the power draw
    u = 10.0 ** (delta / 10.0)
    log_f = (shape * math.log(shape) + (shape - 1.0) * np.log(u) - shape * u - math.lgamma(shape))
    w = np.exp(log_f) * u * math.log(10.0) / 10.0 * step      # density in dB x bin width
    w /= w.sum()
    n_left, n_right = int(round(-delta[0] / step)), int(round(delta[-1] / step))
    padded = np.concatenate([np.full(n_left, pd_left), pd0, np.ones(n_right)])
    # out[i] = sum_j w[j] pd0(x_i + delta_j)
    out = np.convolve(padded, w[::-1], mode="valid")
    return np.clip(out[:len(pd0)], 0.0, 1.0)


def _gamma_quantiles(shape: int, count: int) -> np.ndarray:
    """Equal-probability midpoints of Gamma(shape, 1) (integer shape: CDF = 1 - P(Poisson(v) <= shape - 1))."""
    out = []
    for u in (np.arange(count) + 0.5) / count:
        lo, hi = 0.0, shape + 20.0 * math.sqrt(shape) + 40.0
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            cdf = 1.0 - math.exp(_log_poisson_cdf(shape - 1, mid)[-1])
            lo, hi = (mid, hi) if cdf < u else (lo, mid)
        out.append(0.5 * (lo + hi))
    return np.array(out)


@functools.lru_cache(maxsize=256)
def _steady_curve(n: int, pfa: float) -> np.ndarray:
    """Steady-target Pd on SNR_GRID_DB (the base of every fluctuating law of the same n and Pfa)."""
    out = _pd_steady(n, threshold(n, pfa), 10.0 ** (SNR_GRID_DB / 10.0))
    out.setflags(write=False)
    return out


@functools.lru_cache(maxsize=1024)
def pd_curve(n: int, swerling: Fluctuation, pfa: float) -> np.ndarray:
    """Pd on SNR_GRID_DB (total SNR) for n noncoherent samples, Swerling case 0..4 or Fluct(k), false-alarm
    probability pfa."""
    t = threshold(n, pfa)
    snr = 10.0 ** (SNR_GRID_DB / 10.0)
    shape = shape_of(swerling, n)                           # power of the target, Gamma(shape) with mean x
    # SW2 (or a shape of exactly n): Z / (1 + x / n) ~ Gamma(n, 1): Pd = P(Poisson(T / (1 + x / n)) <= n - 1)
    if swerling == 2 or (isinstance(swerling, Fluct) and shape == float(n)):
        return np.clip(np.exp([_log_poisson_cdf(n - 1, t / (1.0 + x / n))[-1] for x in snr]), 0.0, 1.0)
    pd0 = _steady_curve(n, pfa)
    if shape > _STEADY_SHAPE:                              # spread of the power below the 0.05 dB grid: steady
        return pd0
    return _mixture_db(pd0, shape, pfa)


def pd_at(n: int, swerling: Fluctuation, pfa: float, snr_db) -> np.ndarray:
    """Pd at total SNR snr_db (scalar or array)."""
    return np.interp(snr_db, SNR_GRID_DB, pd_curve(n, swerling, pfa))


def _invert(curve: np.ndarray, p: float) -> float:
    """SNR (dB) where a monotone Pd curve on SNR_GRID_DB reaches p."""
    c = np.maximum.accumulate(curve)
    i = int(np.searchsorted(c, p))
    if i <= 0:
        return float(SNR_GRID_DB[0])
    if i >= len(c):
        return float(SNR_GRID_DB[-1])
    return float(np.interp(p, [c[i - 1], c[i]], [SNR_GRID_DB[i - 1], SNR_GRID_DB[i]]))


def snr_req_db(n: int, swerling: Fluctuation, pfa: float, pd: float) -> float:
    """Total SNR for Pd on one look, no position offset (exact law)."""
    return _invert(pd_curve(n, swerling, pfa), pd)


@functools.lru_cache(maxsize=4096)
def _mean_curve_cached(n: int, swerling: Fluctuation, pfa: float, losses_db: Tuple[float, ...],
                       weights: Tuple[float, ...]) -> np.ndarray:
    base = pd_curve(n, swerling, pfa)
    out = np.zeros_like(base)
    for l, w in zip(losses_db, weights):
        out += w * np.interp(SNR_GRID_DB - l, SNR_GRID_DB, base, left=float(base[0]), right=1.0)
    out.setflags(write=False)
    return out


def mean_curve(n: int, swerling: Fluctuation, pfa: float, losses_db: Sequence[float], weights: Sequence[float]) -> np.ndarray:
    """Pd averaged over the target position: sum_j w_j Pd(SNR - loss_j), on SNR_GRID_DB (cached)."""
    return _mean_curve_cached(n, swerling, pfa, tuple(float(x) for x in losses_db), tuple(float(x) for x in weights))


def quantise_pfa(pfa: float) -> float:
    """Pfa rounded to 1/20 of a decade (12 % steps, < 0.02 dB on the required SNR): bounded number of laws."""
    return float(10.0 ** (round(math.log10(pfa) * 20.0) / 20.0))


def cumulative_curve(curve: np.ndarray, q: float, floor: float = 1e-3) -> np.ndarray:
    """P_cum at the requirement range for a look Pd curve (vs SNR at that range): looks at R (1 + (k + phi) q),
    q = v RR / R, SNR - 40 log10(1 + (k + phi) q), phi averaged over _N_PHASES values; looks whose Pd stays below
    `floor` everywhere on the grid are dropped (they no longer count)."""
    if q <= 0.0:
        return curve
    out = np.zeros_like(curve)
    useful = curve < 1.0 - 1e-9                 # where P_cum is still to be decided
    phases = (np.arange(_N_PHASES) + 0.5) / _N_PHASES
    for phi in phases:
        miss = np.ones_like(curve)
        for k in range(5000):
            shift = 40.0 * math.log10(1.0 + (k + phi) * q)
            pk = np.interp(SNR_GRID_DB - shift, SNR_GRID_DB, curve, left=float(curve[0]), right=float(curve[-1]))
            miss *= 1.0 - pk
            if not np.any(useful) or float(np.max(pk[useful])) < floor or shift > 60.0:
                break                           # farther looks no longer count (Pd below the floor)
        out += 1.0 - miss
    return out / len(phases)


@functools.lru_cache(maxsize=1024)
def _look_shifts(q: float, floor_db_span: float = 60.0) -> np.ndarray:
    """SNR shifts 40 log10(1 + (k + phi) q) of the looks (rows: phases, columns: looks up to a 60 dB shift)."""
    phases = (np.arange(_N_PHASES) + 0.5) / _N_PHASES
    k_max = int(min(5000, math.ceil((10.0 ** (floor_db_span / 40.0) - 1.0) / q))) + 1
    k = np.arange(k_max)
    return 40.0 * np.log10(1.0 + (k[None, :] + phases[:, None]) * q)


def required_snr_db(n: int, swerling: Fluctuation, pfa: float, p_target: float, losses_db: Sequence[float] = (0.0,),
                    weights: Sequence[float] = (1.0,), q: float = 0.0, floor: float = 1e-3) -> float:
    """Mean total SNR (at the requirement range, before the position losses) meeting p_target: one look (q = 0) or
    cumulative over looks spaced q = v RR / R, Pd averaged over the position losses. The cumulative probability is
    solved by bisection on the SNR at R, all looks and phases at once; looks below `floor` do not count."""
    curve = mean_curve(n, swerling, pfa, losses_db, weights)
    if q <= 0.0:
        return _invert(curve, p_target)
    shifts = _look_shifts(round(q, 9))

    def p_cum(x: float) -> float:
        pk = np.interp(x - shifts, SNR_GRID_DB, curve, left=float(curve[0]), right=float(curve[-1]))
        pk = np.where(pk < floor, 0.0, pk)
        return float(np.mean(1.0 - np.prod(1.0 - pk, axis=1)))

    # upper bound: the first look, at R (1 + phi q) <= R (1 + q), alone meets the target there
    lo, hi = float(SNR_GRID_DB[0]), _invert(curve, p_target) + 40.0 * math.log10(1.0 + q)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if p_cum(mid) < p_target else (lo, mid)
        if hi - lo < 1e-4:
            break
    return hi


def quantiles(values: np.ndarray, count: int, weights: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray]:
    """`count` equal-probability representative values (mid quantiles) of a sample / weighted discrete law."""
    v = np.asarray(values, dtype=float)
    w = np.ones_like(v) if weights is None else np.asarray(weights, dtype=float)
    order = np.argsort(v)
    v, w = v[order], w[order] / w.sum()
    cum = np.cumsum(w) - 0.5 * w
    u = (np.arange(count) + 0.5) / count
    return np.interp(u, cum, v), np.full(count, 1.0 / count)


def combine(a: Tuple[np.ndarray, np.ndarray], b: Tuple[np.ndarray, np.ndarray], count: int) -> Tuple[np.ndarray, np.ndarray]:
    """Law of the sum of two independent discrete losses, reduced to `count` quantiles."""
    vals = (a[0][:, None] + b[0][None, :]).ravel()
    w = (a[1][:, None] * b[1][None, :]).ravel()
    return quantiles(vals, count, w)
