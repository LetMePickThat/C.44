#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PREVIEW -- the scenario editor's own model (fork RADARv6_RS, user 8 November 2026: create the scenarios live, the
trajectories shown at once as in KSP or Orbiter; approximations allowed while editing, the real computation, sim/run,
once the scenario is validated, its journal the entry point of the radar).

The same forces as the real computation (sim/propagate: the central body with J2, the other bodies, the burns, the
impulses, proportional navigation, the spawns of relative states and groups, the attitudes flown by waypoints, the
impacts), integrated by RK4 at two rates instead of 0.1 s everywhere (compiled, sim/fast_preview):
  - the free rows: one step of PREVIEW_STEP_S (15 s, user 8 November 2026: the 30 s segments showed on the paths;
    30 s was already < 1 m off the 0.1 s integration after an hour in low orbit, measured), each step ending at the next event (spawn, impulse, burn start or end, PN start time);
  - within it, the rows that need finer steps, at their own: a row burning (BURN_STEP_S), a guided row (a twentieth
    of the time to its target, PN_SHARE, down to FINE_STEP_S), a row closing on its PN start range or on the surface
    (half the time left, HALF); a guided row's target, when free, interpolated over the step (cubic on its states at
    both ends, as the playback);
  - the other bodies: only those whose tidal pull on the scenario's orbits moves a body more than TIDE_MIN_KM over the
    run (the planets hardly count around the Earth).
Approximations: no Singer manoeuvres (their mean path, none); the attitudes advanced at the free rows' steps (a slew
within a step); the bodies left out above.
The record: every free step, every RECORD_MIN_S at most within the fine ones (as the real computation's), and twice at
an impulse (just before it, PRE_IMPULSE_S earlier, then after it: the playback's cubic does not smooth the jump).
Truth as the real computation's; its instants are not evenly spaced.
"""
from __future__ import annotations

import copy
import math
from typing import Optional

import numpy as np

from . import fast
from . import fast_preview as fp
from . import propagate as prop

PREVIEW_STEP_S = 15.0          # a free row's step, s (user, 8 November 2026: 30 s segments visible)
BURN_STEP_S = 5.0              # a row burning, s (D)
FINE_STEP_S = 0.1              # the finest step: the real computation's default, s (D)
PN_SHARE = 0.05                # a guided row: this share of the time to its target per step (D)
HALF = 0.5                     # closing on a PN start range or on the surface: this share of the time left (D)
RECORD_MIN_S = 1.0             # within fine steps, a record every this at most, s (the real one's sim.log_step_s) (D)
PRE_IMPULSE_S = 1e-6           # the record just before an impulse, s before it (D)
TIDE_MIN_KM = 1e-4             # a body left out when its tidal pull moves the scenario's orbits less than this (D)

_THIRD: dict = {}              # the other bodies' grids by (central body, epoch, duration, orbit radius)
_NO_INT, _NO_F, _NO_VEC = np.zeros(0, np.int64), np.zeros(0), np.zeros((0, 3))
_NO_DRV = np.zeros((3, 0, 6))


def _preview_scenario(sc: dict) -> dict:
    """The scenario as the preview integrates it: without Singer manoeuvres, its step the free rows' one (the
    records' arrays are sized by it, grown as needed)."""
    c = copy.deepcopy(sc)
    for body in [c.get("ship", {})] + c.get("objects", []) + c.get("groups", []):
        if "manoeuvres" in body:
            body["manoeuvres"] = [m for m in body["manoeuvres"] if m.get("type") != "singer"]
    c["meta"] = dict(c["meta"], step_s=PREVIEW_STEP_S)
    return c


def _hermite(x0: np.ndarray, x1: np.ndarray, t0: float, t1: float, t: float) -> np.ndarray:
    """The states (n, 6) at t between x0 at t0 and x1 at t1: the cubic on positions and velocities (sgeo.states_at)."""
    h = t1 - t0
    if h <= 0.0:
        return x1.copy()
    s = (t - t0) / h
    h00, h10, h01, h11 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s, -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
    d00, d10, d01, d11 = 6 * s ** 2 - 6 * s, 3 * s ** 2 - 4 * s + 1, -6 * s ** 2 + 6 * s, 3 * s ** 2 - 2 * s
    r0, v0, r1, v1 = x0[:, :3], x0[:, 3:], x1[:, :3], x1[:, 3:]
    r = h00 * r0 + h10 * h * v0 + h01 * r1 + h11 * h * v1
    v = (d00 * r0 + d10 * h * v0 + d01 * r1 + d11 * h * v1) / h
    return np.concatenate([r, v], axis=1)


class PreviewSim(prop.LiveSim):
    """The scenario integrated at two rates (module docstring); run() to the end in one go."""

    def __init__(self, sc: dict):
        self.source_sc = sc
        super().__init__(_preview_scenario(sc))
        self.checkpoints = []                     # (none: the preview is computed again from the start)
        self.e_ptr = 0
        self.steps = self.fine_steps = 0
        self._t_rec = 0.0                         # the last record's time

    def _third_bodies(self, central: str, sc: dict) -> prop.ThirdBodies:
        """The other bodies' grid (kept from one preview to the next: 0.2 s for Jupiter's moons), without those whose
        tidal pull is negligible here (TIDE_MIN_KM over the run, at the largest orbit radius of the scenario)."""
        b = prop.Body.of(central)
        dur = float(sc["meta"]["duration_s"])
        r_orbit = max(2.0 * b.r_eq_km, *(_radius(sc, s) for s in _states(sc)))
        key = (central, sc["meta"].get("epoch_utc"), dur, round(r_orbit))
        got = _THIRD.get(key)
        if got is None:
            if len(_THIRD) > 4:
                _THIRD.clear()
            full = prop.ThirdBodies(central, prop.scn.epoch(sc), dur)
            d = np.linalg.norm(full.pos, axis=2).min(axis=0)            # each body's nearest over the run
            moved = 0.5 * 2.0 * full.mu * r_orbit / d ** 3 * dur ** 2    # its tide over the run, km
            keep = moved >= TIDE_MIN_KM
            got = copy.copy(full)
            got.names = tuple(n for n, k in zip(full.names, keep) if k)
            got.mu, got.pos, got._cache = full.mu[keep], np.ascontiguousarray(full.pos[:, keep]), {}
            _THIRD[key] = got
        return got

    # ------------------------------------------------------------------ the record, grown as needed
    def _grow(self) -> None:
        n = len(self.t_log)
        m = 2 * n + 16

        def bigger(a, fill):
            out = np.full((m,) + a.shape[1:], fill, a.dtype)
            out[:n] = a
            return out
        self.t_log = bigger(self.t_log, np.nan)
        self.r_log, self.v_log = bigger(self.r_log, np.nan), bigger(self.v_log, np.nan)
        self.att_q = bigger(self.att_q, np.nan)

    def _record(self, j: int, t: float) -> None:
        if j >= len(self.t_log):
            self._grow()
        super()._record(j, t)

    def _push(self, t: float, xs: Optional[np.ndarray] = None) -> None:
        """A record at t, of the states xs (default: the current ones)."""
        if xs is not None:
            keep, self.x = self.x, xs
        try:
            self._record(self.j_log, t)
        finally:
            if xs is not None:
                self.x = keep
        self.j_log += 1
        self._t_rec = t

    # ------------------------------------------------------------------ the forces, compiled (sim/fast_preview)
    def _rk4_rows(self, rows: np.ndarray, t0: float, h: float, drv_rows=_NO_INT, drv=_NO_DRV) -> None:
        """One RK4 step h of the rows (the others left as they are); drv_rows among them: driven by drv (3, nd, 6)."""
        if not len(rows):
            return
        act = self.active
        pos = np.full(self.n, -1)
        pos[rows] = np.arange(len(rows))
        mine = np.zeros(self.n, bool)
        mine[rows] = True
        if len(self.b_row):                              # the burns running at t0 (decided at the step start)
            run = (self.b_t0 - 1e-9 <= t0) & (t0 < self.b_t1 - 1e-9) & mine[self.b_row]
            b_k = pos[self.b_row[run]]
            b_acc = np.ascontiguousarray(self.b_acc[run])
            b_fr = np.array([fp.FRAME_CODE.get(f, 0) for f in self.b_frame[run]], np.int64)
        else:
            b_k, b_acc, b_fr = _NO_INT, _NO_VEC, _NO_INT
        on = np.flatnonzero(self.pn_on & mine)
        if len(on):
            tg = self.pn_tgt[on]
            ok = act[tg] & mine[tg]
            on = on[ok]
        on_k, tg_k = (pos[on], pos[self.pn_tgt[on]]) if len(on) else (_NO_INT, _NO_INT)
        gain = np.ascontiguousarray(self.pn_n[on]) if len(on) else _NO_F
        lim = np.ascontiguousarray(self.a_max[on]) if len(on) else _NO_F
        if self.third is not None and len(self.third.mu):
            tpos3 = np.stack([self.third.positions(t0), self.third.positions(t0 + 0.5 * h),
                              self.third.positions(t0 + h)])
            tmu = self.third.mu
        else:
            tpos3, tmu = np.zeros((3, 0, 3)), _NO_F
        b = self.body
        fp.rk4(self.x, np.ascontiguousarray(rows, np.int64), float(h), tpos3, tmu, b.mu, float(b.j2), b.r_eq_km,
               b.pole, self.axes, np.ascontiguousarray(b_k, np.int64), b_acc, b_fr,
               np.ascontiguousarray(on_k, np.int64), np.ascontiguousarray(tg_k, np.int64), gain, lim,
               np.ascontiguousarray(pos[drv_rows], np.int64), np.ascontiguousarray(drv))

    # ------------------------------------------------------------------ the steps each row needs
    def _needs(self, t: float, rows: np.ndarray) -> np.ndarray:
        """The step each of the rows needs from t (inf: free) (module docstring)."""
        x, act = self.x, self.active
        need = np.full(self.n, np.inf)
        if len(self.b_row):                              # burning
            run = (self.b_t0 - 1e-9 <= t) & (t < self.b_t1 - 1e-9)
            need[self.b_row[run]] = BURN_STEP_S
        pr = self.pn_rows
        if len(pr):
            tg = self.pn_tgt[pr]
            live = act[pr] & act[tg] & ~self.pn_done[pr]
            if live.any():
                i, j = pr[live], tg[live]
                rel = x[j, :3] - x[i, :3]
                vrel = x[j, 3:] - x[i, 3:]
                rng = np.linalg.norm(rel, axis=1)
                vc = -np.einsum("ij,ij->i", rel, vrel) / np.maximum(rng, 1e-12)        # closing speed
                closing = vc > 1e-9
                k_t = np.searchsorted(pr, i)
                r0 = self.pn_r0[k_t]
                on = self.pn_on[i]
                by_range = ~on & ~np.isfinite(self.pn_t[k_t]) & np.isfinite(r0) & closing
                with np.errstate(invalid="ignore", divide="ignore"):
                    d_start = np.where(by_range, HALF * (rng - r0) / vc, np.inf)
                    d_pn = np.where(on & closing, PN_SHARE * rng / vc, np.inf)
                need[i] = np.minimum(need[i], np.maximum(FINE_STEP_S, np.minimum(d_start, d_pn)))
        r = np.linalg.norm(x[:, :3], axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            vr = np.einsum("ij,ij->i", x[:, :3], x[:, 3:]) / r
            left = (r - self.body.radius_km) / -vr
            surf = act & (vr < 0.0) & (left < PREVIEW_STEP_S / HALF)
        need[surf] = np.minimum(need[surf], np.maximum(FINE_STEP_S, HALF * left[surf]))
        out = need[rows]
        out[~act[rows]] = np.inf
        return out

    # ------------------------------------------------------------------ the guided rows' bookkeeping
    def _pn_rows_update(self, t: float, prev: np.ndarray, mask: np.ndarray) -> None:
        """LiveSim._pn_update on the guided rows of mask only."""
        pr, x, act = self.pn_rows, self.x, self.active
        if not len(pr):
            return
        sel = pr[act[pr] & ~self.pn_done[pr] & act[self.pn_tgt[pr]] & mask[pr]]
        if not len(sel):
            return
        if fast.USE_NUMBA:
            k_t = np.searchsorted(pr, sel)
            fast.pn_step(x, prev, sel, self.pn_tgt[sel], self.pn_on[sel], t, self.pn_t[k_t], self.pn_r0[k_t],
                         self.pn_on, self.pn_done, self.t_pn, self.miss)
            return
        keep = self.pn_rows
        try:
            self.pn_rows = sel
            super()._pn_update(t, prev)
        finally:
            self.pn_rows = keep

    # ------------------------------------------------------------------ the step
    def step(self) -> bool:
        if self.done:
            return False
        x, act, events = self.x, self.active, self.events
        t0 = self.t
        t1 = min(t0 + PREVIEW_STEP_S, self.duration)
        while self.e_ptr < len(events) and events[self.e_ptr] <= t0 + 1e-9:
            self.e_ptr += 1
        at_event = False
        if self.e_ptr < len(events) and events[self.e_ptr] <= t1 + 1e-9:   # the step ends at the next event
            t1, at_event = float(events[self.e_ptr]), True
        if self.duration - t1 < 1e-9:
            t1 = self.duration
        h = t1 - t0
        rows = np.flatnonzero(act)
        is_fine = np.zeros(self.n, bool)
        is_fine[rows[self._needs(t0, rows) < h - 1e-9]] = True
        pr = self.pn_rows
        if len(pr) and is_fine.any():                     # a guided row whose target steps finely: with it
            guided = pr[act[pr] & self.pn_on[pr] & ~self.pn_done[pr]]
            grown = True
            while grown:
                pull = guided[is_fine[self.pn_tgt[guided]] & ~is_fine[guided]]
                grown = bool(len(pull))
                is_fine[pull] = True
        fine, free = rows[is_fine[rows]], rows[~is_fine[rows]]
        prev = x.copy()
        self.now = t0
        if self.fleet is not None:                        # attitudes at the free step
            self.fleet.step(t0, h, x, act)
        else:
            for i in self.att_rows:
                if act[i] and self.flyers[i].m is not None:
                    self.flyers[i].step(t0, h, x[i, :3], x[i, 3:], self._tpos)
        if h > 1e-12:
            self._rk4_rows(free, t0, h)
        self.k += 1
        self.steps += 1
        if len(fine) and h > 1e-12:
            if self._fine(t0, t1, fine, free, prev):
                return False                              # the ship met the surface
        is_free = np.zeros(self.n, bool)
        is_free[free] = True
        hit_ship = self._impacts_rows(t1, is_free)
        if hit_ship:
            self._push(t1)
            self.t, self.done = t1, True
            return False
        if at_event:
            self.e_ptr += 1
            lo = np.searchsorted(self.imp_t, t1 - 1e-9)
            if lo < len(self.imp_t) and self.imp_t[lo] <= t1 + 1e-9:   # an impulse: the state just before it
                self._push(t1 - PRE_IMPULSE_S)
            self._spawn(t1)
            self._impulses(t1)
        not_fine = np.ones(self.n, bool)
        not_fine[fine] = False
        self._pn_rows_update(t1, prev, not_fine)          # (the fine rows were updated at their own steps)
        self._push(t1)
        self.t = t1
        self.done = t1 >= self.duration - 1e-9
        return not self.done

    def _impacts_rows(self, t: float, mask: np.ndarray) -> bool:
        """LiveSim._impacts on the rows of mask only; True when the ship is among those below the surface."""
        r = np.linalg.norm(self.x[:, :3], axis=1)
        hit = self.active & mask & (r < self.body.radius_km)
        if hit.any():
            self.t_impact[hit] = t
            self.active[hit & (np.arange(self.n) > 0)] = False
        return bool(hit[0])

    def _fine(self, t0: float, t1: float, fine: np.ndarray, free: np.ndarray, prev: np.ndarray) -> bool:
        """The fine rows from t0 to t1 at their own steps, the free ones already at t1 (their targets interpolated
        between prev and now); records every RECORD_MIN_S at most. True when the ship met the surface."""
        x = self.x
        x0_free, x1_free = prev[free], x[free].copy()
        is_fine = np.zeros(self.n, bool)
        is_fine[fine] = True
        tg = self.pn_tgt[fine[np.isin(fine, self.pn_rows)]]
        drv_rows = np.unique(tg[~is_fine[tg] & self.active[tg]]) if len(tg) else _NO_INT
        where_free = np.full(self.n, -1)
        where_free[free] = np.arange(len(free))
        dk = where_free[drv_rows]
        drv_rows, dk = drv_rows[dk >= 0], dk[dk >= 0]
        if len(drv_rows):                                 # the targets at t0 for the first step's size
            x[drv_rows] = x0_free[dk]
        ta = t0
        while ta < t1 - 1e-12:
            live = fine[self.active[fine]]
            if not len(live):
                break
            hf = float(self._needs(ta, live).min())
            hf = t1 - ta if not np.isfinite(hf) else min(max(hf, FINE_STEP_S), t1 - ta)
            if t1 - (ta + hf) < 0.5 * FINE_STEP_S:          # (no sliver left before t1)
                hf = t1 - ta
            tb = ta + hf
            if len(drv_rows):
                drv = np.stack([_hermite(x0_free[dk], x1_free[dk], t0, t1, t) for t in (ta, ta + 0.5 * hf, tb)])
                x[drv_rows] = drv[0]
            else:
                drv = _NO_DRV
            prev_f = x.copy()
            self._rk4_rows(np.r_[live, drv_rows].astype(np.int64), ta, hf, drv_rows, drv)
            self.fine_steps += 1
            if len(drv_rows):
                x[drv_rows] = drv[2]                         # (the targets at tb for the guidance's checks)
            if self._impacts_rows(tb, is_fine):              # the ship below the surface: the run ends at tb
                xs = x.copy()
                xs[free] = _hermite(x0_free, x1_free, t0, t1, tb)
                self.x = xs
                self._push(tb)
                self.t, self.done = tb, True
                return True
            self._pn_rows_update(tb, prev_f, is_fine)
            if tb < t1 - 1e-9 and tb - self._t_rec >= RECORD_MIN_S - 1e-9:
                xs = x.copy()
                xs[free] = _hermite(x0_free, x1_free, t0, t1, tb)
                self._push(tb, xs)
            ta = tb
        x[free] = x1_free                                  # (the targets back at t1)
        return False


def _states(sc: dict):
    """The absolute states of the scenario (the ship's orbit, the objects' absolute states)."""
    yield sc["ship"]["orbit"]
    for o in sc.get("objects", []):
        if o.get("state", {}).get("type") in ("elements", "state"):
            yield o["state"]


def _radius(sc: dict, st: dict) -> float:
    """The largest distance from the central body of a state's orbit (apoapsis; |r| for a state), km."""
    if st.get("type") == "state":
        return float(np.linalg.norm(st["r_km"]))
    a, e = float(st.get("a_km", 0.0)), float(st.get("e", 0.0))
    return a * (1.0 + e) if e < 1.0 else a


def preview_sim(sc: dict) -> PreviewSim:
    """The preview run of the normalised scenario sc (module docstring)."""
    sim = PreviewSim(sc)
    sim.run()
    return sim


def preview(sc: dict) -> prop.Truth:
    """The previewed truth of the normalised scenario sc."""
    return preview_sim(sc).truth()
