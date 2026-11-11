#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COST PROFILE OF A VOLUME FOR THE FAST ESTIMATE (user, 10 November 2026, option (c), before pass R3e).

The fast estimate's table (core/cost_table.json, tools/gen_cost_table.py) prices a beam position for a generic
request: single-look detection, the default false-alarm probability, no revisit. The exact optimiser judges a search
volume by its cumulative criterion (Pd over the looks of the approach, v x RR), with the volume's own waveform
candidates and its Pfa from its false-alarm budget: on mission_example S01 (5000 km, 1000 m2, 5000 m/s, RR 20 s)
the table asked 8385 J per position at broadside, the optimiser's laws 68 J (the remaining gap, the optimiser's
choice of the lattice spacing, is the recalibration's, core/calibration.py).

The profile is, per volume, the energy of one position at each scan angle of the table (table.TH), at the frequency
of the table, with the optimiser's own laws: optimizer.candidate_waveforms(sec), candidate_pfas (positions of the
volume, by factors of 2), range_at with the volume's search_criterion, receive setting and velocity span; the
cheapest candidate reaching the range, its pulse shortened to just reach it (search_position_costs, as the table);
none: the longest-reach candidate at full pulse (unreachable). The sky noise per direction is left aside (as the
table). It does not depend on where the volume lies, so the attitude and the instant never recompute it.

Computed once per volume and kept in memory and on disk (.profile_cache.json beside this module) under a fingerprint
of the volume's requirements (its geometry, frame, name and display keys left aside), of its positions by factors of
2, of the parameter file and of the model version. The computation (3-5 s per volume) runs in a background process:
get() returns None until it is ready, and the fast estimate falls back to the table meanwhile ("estimating").
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import threading
from concurrent.futures import Future, ProcessPoolExecutor
from typing import Dict, List, Optional, Sequence, Tuple

CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".profile_cache.json")
CACHE_MAX = 1024
PROFILE_FORMAT = 3                                # 2: + the range reachable at full pulse; 3: + the confirmation
_GEOMETRY = ("id", "name", "label", "colour", "color", "visible", "hidden", "note", "comment", "shape",
             "phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg", "phi0_deg", "eps0_deg", "radius_deg",
             "half_width_deg", "half_height_deg", "rot_deg", "track_ra_deg", "track_dec_deg", "track_sigma_deg",
             "tracking")
_lock = threading.Lock()
_cache: Optional[Dict[str, dict]] = None
_jobs: Dict[str, Future] = {}
_pool: Optional[ProcessPoolExecutor] = None
_generation = 0                                   # +1 each time a profile becomes ready (the views refresh)


def _stamp() -> str:
    from . import radar_model as rm
    from .params import P
    try:
        with open(P.path, "rb") as fh:
            h = hashlib.sha1(fh.read()).hexdigest()
    except OSError:
        h = "?"
    return f"{rm.MODEL_VERSION}:{h}"


def positions_bucket(positions: float) -> float:
    """The positions that set the Pfa, by factors of 2 (the Pfa goes as log of it)."""
    return float(2.0 ** round(math.log2(max(positions, 1.0))))


def key(sec: dict, positions: float, th: Sequence[float], f_ghz: float) -> str:
    keep = {k: v for k, v in sec.items()
            if k not in _GEOMETRY and not str(k).startswith("_") and not str(k).startswith("frame")}
    txt = json.dumps([keep, positions_bucket(positions), list(th), f_ghz, PROFILE_FORMAT], sort_keys=True,
                     default=str) + "|" + _stamp()
    return hashlib.sha1(txt.encode("utf-8")).hexdigest()


def compute(sec: dict, positions: float, th: Sequence[float], f_ghz: float) -> Optional[dict]:
    """{"log_e": [per scan angle], "feasible": [...]} of a volume (module docstring); None for an incomplete one."""
    from . import optimizer as mo
    from . import radar_model as rm
    try:
        r_want = float(sec["range_km"])
        tgt = mo.sector_target(sec)
        cands = mo.candidate_waveforms(sec)
        pfas = mo.candidate_pfas(sec, cands, f_ghz, positions_bucket(positions))
        cum, sll, vs = mo.search_criterion(sec), mo.rx_sll_db(sec), mo.velocity_span_mps(sec)
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return None
    if r_want <= 0 or not cands:
        return None
    log_e, feas, r_reach = [], [], []
    for t in th:
        best, r_max, e_max = None, 0.0, None
        for w, pfa in zip(cands, pfas):
            try:
                r = mo.range_at(w, f_ghz, float(t), tgt, sll, vs, pfa, cum)
            except ValueError:
                continue
            if not r:
                continue
            if r > r_max:
                r_max, e_max = r, float(rm.search_position_costs(w)[0])
            if r >= r_want:
                e = float(rm.search_position_costs(w, (r_want / r) ** 4)[0])
                best = e if best is None or e < best else best
        e = best if best is not None else e_max
        if e is None or e <= 0:
            return None
        log_e.append(math.log(e))
        feas.append(best is not None)
        r_reach.append(float(r_max))
    # the confirmation of its detections, at every lattice spacing the optimiser may choose (user, 11 November 2026:
    # the live load said OK for a volume whose detections could never be confirmed)
    from . import tracking as trk
    from .params import P
    confirm_ok = []
    for sp in P.get("search_grid.beam_spacing_options_hpbw"):
        try:
            ok, _why = trk.confirmation_feasible(dict(sec, _grid_spacing=float(sp)), None, f_s_ghz=f_ghz)
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            ok = True                                  # (a volume mid-edit: not judged here)
        if ok:
            confirm_ok.append(float(sp))
    return {"log_e": log_e, "feasible": feas, "r_reach_km": r_reach, "confirm_ok_spacings": confirm_ok}


def _load() -> Dict[str, dict]:
    global _cache
    if _cache is None:
        try:
            with open(CACHE_PATH, encoding="utf-8") as fh:
                _cache = json.load(fh)
        except (OSError, ValueError):
            _cache = {}
    return _cache


def _save() -> None:
    c = _load()
    if len(c) > CACHE_MAX:
        for k in sorted(c, key=lambda k: c[k].get("order", 0))[:len(c) - CACHE_MAX]:
            del c[k]
    tmp = CACHE_PATH + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(c, fh)
        os.replace(tmp, CACHE_PATH)
    except OSError:
        pass                                      # (a read-only install: kept for this session)


def _done(k: str, fut: Future) -> None:
    global _generation
    try:
        res = fut.result()
    except Exception:  # noqa: BLE001 -- a failed profile: the table stays
        res = None
    with _lock:
        _jobs.pop(k, None)
        c = _load()
        c[k] = {"profile": res, "order": max((v.get("order", 0) for v in c.values()), default=0) + 1}
        _save()
        _generation += 1


def _executor() -> ProcessPoolExecutor:
    global _pool
    if _pool is None:
        import atexit
        from .optimizer import _mark_pool_worker
        _pool = ProcessPoolExecutor(max_workers=1, initializer=_mark_pool_worker)   # (stops when the tool is gone)
        atexit.register(shutdown)                 # (a profile still running at the exit: at most its 3-5 s)
    return _pool


def get(sec: dict, positions: float, th: Sequence[float], f_ghz: float, wait: bool = False) -> Optional[dict]:
    """The volume's profile, or None while it is being computed (started in the background on the first call);
    wait=True computes it here (tools, tests)."""
    k = key(sec, positions, th, f_ghz)
    with _lock:
        hit = _load().get(k)
        if hit is not None:
            return hit["profile"]
        if wait:
            pass
        elif k not in _jobs:
            try:
                fut = _executor().submit(compute, dict(sec), float(positions), list(th), float(f_ghz))
            except Exception:  # noqa: BLE001 -- no worker process (frozen install): computed here, once
                fut = None
            if fut is not None:
                _jobs[k] = fut
                fut.add_done_callback(lambda f_, k_=k: _done(k_, f_))
                return None
        else:
            return None
    res = compute(sec, positions, th, f_ghz)
    with _lock:
        c = _load()
        c[k] = {"profile": res, "order": max((v.get("order", 0) for v in c.values()), default=0) + 1}
        _save()
    return res


def pending() -> int:
    with _lock:
        return len(_jobs)


def generation() -> int:
    return _generation


def shutdown() -> None:
    global _pool
    if _pool is not None:
        _pool.shutdown(wait=False, cancel_futures=True)
        _pool = None
