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
PROFILE_FORMAT = 4                                # 2: + reach at full pulse; 3: + the confirmation; 4: + its range
_GEOMETRY = ("id", "name", "label", "colour", "color", "visible", "hidden", "note", "comment", "shape",
             "phi_min_deg", "phi_max_deg", "eps_min_deg", "eps_max_deg", "phi0_deg", "eps0_deg", "radius_deg",
             "half_width_deg", "half_height_deg", "rot_deg", "tracking")
_lock = threading.Lock()
_cache: Optional[Dict[str, dict]] = None
_jobs: Dict[str, Future] = {}
_pool: Optional[ProcessPoolExecutor] = None
_batch = {"total": 0, "done": 0, "last": 0.0}      # the profiles asked since the worker was last idle (progress)
_durations: List[float] = []                      # (the last profiles' durations: the running one's share)
_generation = 0                                   # +1 each time a profile becomes ready (the views refresh)


def _stamp() -> str:
    from . import radar_model as rm
    from .params import P, values_stamp
    return f"{rm.MODEL_VERSION}:{values_stamp(P)}"


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
    # none: the longest range at which the confirmation is still possible (user, 11 November 2026), by bisection
    # (the range only enters through the spacing of the approach looks, v RR / R: shorter, easier)
    r_conf = None
    if not confirm_ok:
        def ok_at(r_km: float) -> bool:
            for sp in P.get("search_grid.beam_spacing_options_hpbw"):
                try:
                    if trk.confirmation_feasible(dict(sec, range_km=r_km, _grid_spacing=float(sp)), None,
                                                 f_s_ghz=f_ghz)[0]:
                        return True
                except (KeyError, TypeError, ValueError, ZeroDivisionError):
                    return False
            return False
        lo, hi = 10.0, float(sec.get("range_km", 0.0) or 0.0)
        if hi > lo and ok_at(lo):
            for _ in range(10):                        # (log bisection: ~1 % of the ratio hi / lo)
                mid = math.sqrt(lo * hi)
                lo, hi = (mid, hi) if ok_at(mid) else (lo, mid)
            r_conf = lo
        else:
            r_conf = 0.0
    return {"log_e": log_e, "feasible": feas, "r_reach_km": r_reach, "confirm_ok_spacings": confirm_ok,
            "confirm_max_km": r_conf}


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
    import time
    with _lock:
        _jobs.pop(k, None)
        now = time.monotonic()
        _batch["done"] += 1
        _durations.append(now - _batch["last"])
        del _durations[:-8]
        _batch["last"] = now
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
                if not _jobs:                         # (a new batch: the progress from zero)
                    import time
                    _batch.update(total=0, done=0, last=time.monotonic())
                _batch["total"] += 1
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
    """Profiles being computed, and the reliefs (core/redundancy, A): the views poll both."""
    from . import redundancy, cost_estimate
    with _lock:
        n = len(_jobs) + len(_ijobs)
    return n + redundancy.relief_pending() + cost_estimate.async_pending()


def progress() -> Optional[Tuple[int, int, float]]:
    """(done, total, fraction) of the profiles being computed, None when none is; the running profile's share is
    its elapsed time over the mean duration of the last ones (an estimate, kept below its end)."""
    import time
    with _lock:
        if not _jobs:
            return None
        done, total = _batch["done"], max(_batch["total"], 1)
        mean = sum(_durations) / len(_durations) if _durations else 30.0
        run = min(0.95, (time.monotonic() - _batch["last"]) / max(mean, 1.0))
        return done, total, min(1.0, (done + run) / total)


def generation() -> int:
    from . import redundancy, background
    return _generation + redundancy.relief_generation() + background.generation()


def shutdown() -> None:
    global _pool, _ipool
    if _pool is not None:
        _pool.shutdown(wait=False, cancel_futures=True)
        _pool = None
    if _ipool is not None:
        _ipool.shutdown(wait=False, cancel_futures=True)
        _ipool = None


# ----------------------------------------------------------------------------- instant profile (b2-b4)
# User, 12 November 2026 (estimate-vs-exact, b2 b3 b4 (ii)): the profile above prices one position of a volume without
# the sky (noise rise, blinded directions, surface clutter and the waveforms the optimiser then takes) and at the
# default lattice spacing, while the exact optimiser picks its spacing among beam_spacing_options_hpbw: on
# MOONTEST_VELOCITY the volumes were off by x0.36-x4.5. The INSTANT profile is the exact optimiser's own plan of the
# volume (optimizer._best_plan, same code path) at ONE centre frequency (V1: the frequency the last exact run kept for
# the volume, else tool.cost_table.freq_ghz), every lattice spacing tried, with the sky of the instant in the ship
# frame. Per 1-deg scan bin: electrical energy and TX time of one position of the plan, positions per face; the
# spacing kept. It depends on where the volume and the bodies are: kept per volume with the geometry it was computed
# for, and computed again (in a background process) when a body, the galactic frame or the volume moved more than
# tool.cost_profile.instant_sky_move_deg, or a body's apparent radius or closing-speed spread changed by more than
# instant_sky_radius_rel (V2); meanwhile the last one of the volume is used and the volume is "estimating".
INSTANT_FORMAT = 4                                # 2: + memory traffic and processing energy per position (c);
                                                  # 3: + the confirmation of the false alarms (d);
                                                  # 4: + the blind range of each bin's waveform (5.5, played view)
ICACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".instant_profile_cache.json")
_IDISPLAY = ("id", "name", "label", "colour", "color", "visible", "hidden", "note", "comment", "tracking")
_icache: Optional[Dict[str, list]] = None
_ijobs: Dict[str, Future] = {}
_ipool: Optional[ProcessPoolExecutor] = None
_iorder = [0]


from .params import P as _P                       # (read at import: test_every_parameter_is_read_by_the_code)
INSTANT_MOVE_DEG = float(_P.get("tool.cost_profile.instant_sky_move_deg"))
INSTANT_RADIUS_REL = float(_P.get("tool.cost_profile.instant_sky_radius_rel"))
INSTANT_CACHE_MAX = int(_P.get("tool.cost_profile.instant_cache_max"))


def _itol() -> Tuple[float, float, int]:
    return INSTANT_MOVE_DEG, INSTANT_RADIUS_REL, INSTANT_CACHE_MAX


def instant_key(sec: dict, f_ghz: float) -> str:
    """The volume's requirements and shape in its own frame (its attitude in the ship frame, frame_*, compared apart),
    the frequency, the parameter file and model version."""
    keep = {k: v for k, v in sec.items()
            if k not in _IDISPLAY and not str(k).startswith("_") and not str(k).startswith("frame")}
    keep["frame"] = sec.get("frame")
    txt = json.dumps([keep, round(float(f_ghz), 6), INSTANT_FORMAT], sort_keys=True, default=str) + "|" + _stamp()
    return hashlib.sha1(txt.encode("utf-8")).hexdigest()


def _unit(phi_deg: float, eps_deg: float):
    p, e = math.radians(float(phi_deg)), math.radians(float(eps_deg))
    return (math.cos(e) * math.cos(p), math.cos(e) * math.sin(p), math.sin(e))


def _sep_deg(a, b) -> float:
    d = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(d))


def _rot_deg(a, b) -> float:
    """Angle of the rotation between two 3x3 matrices (deg)."""
    import numpy as np
    m = np.asarray(a, float).reshape(3, 3).T @ np.asarray(b, float).reshape(3, 3)   # (frame_m is stored flat, 9 values)
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(m)) - 1.0) / 2.0))))


def _rel(a, b) -> float:
    a, b = float(a or 0.0), float(b or 0.0)
    return abs(a - b) / max(abs(a), abs(b), 1e-12)


def _snapshot(sec: dict, sky: Optional[dict]) -> dict:
    return {"frame_m": sec.get("frame_m"), "frame_eps_offset_deg": sec.get("frame_eps_offset_deg"),
            "sky": json.loads(json.dumps(sky, default=str)) if sky is not None else None}


def close_enough(snap: dict, sec: dict, sky: Optional[dict]) -> bool:
    """The geometry of an instant profile still holds for (sec, sky) within the V2 tolerances."""
    move, rel, _ = _itol()
    fa, fb = snap.get("frame_m"), sec.get("frame_m")
    if (fa is None) != (fb is None) or (fa is not None and _rot_deg(fa, fb) > move):
        return False
    if abs(float(snap.get("frame_eps_offset_deg") or 0.0) - float(sec.get("frame_eps_offset_deg") or 0.0)) > move:
        return False
    sa, sb = snap.get("sky"), sky
    if (sa is None) != (sb is None):
        return False
    if sa is None:
        return True
    if set(sa) != set(sb):
        return False
    for k in sa:
        if k == "bodies":
            ba = {(b.get("id"), b.get("type")): b for b in sa["bodies"]}
            bb = {(b.get("id"), b.get("type")): b for b in sb["bodies"]}
            if set(ba) != set(bb):
                return False
            for idt, x in ba.items():
                y = bb[idt]
                if _sep_deg(_unit(x["phi_deg"], x["eps_deg"]), _unit(y["phi_deg"], y["eps_deg"])) > move:
                    return False
                if _rel(x.get("angular_radius_deg"), y.get("angular_radius_deg")) > rel:
                    return False
                sp = max(float(x.get("closing_spread_mps") or 0.0), float(y.get("closing_spread_mps") or 0.0))
                if _rel(x.get("closing_spread_mps"), y.get("closing_spread_mps")) > rel and sp > 0.0:
                    return False
                if abs(float(x.get("closing_mps") or 0.0) - float(y.get("closing_mps") or 0.0)) > rel * max(sp, 1.0):
                    return False
                if {q: v for q, v in x.items() if not isinstance(v, (int, float))} != \
                        {q: v for q, v in y.items() if not isinstance(v, (int, float))}:
                    return False
        elif k == "galactic" and isinstance(sa[k], dict) and isinstance(sb[k], dict):
            for p in ("pole", "centre"):
                if f"{p}_phi_deg" in sa[k]:
                    if _sep_deg(_unit(sa[k][f"{p}_phi_deg"], sa[k][f"{p}_eps_deg"]),
                                _unit(sb[k][f"{p}_phi_deg"], sb[k][f"{p}_eps_deg"])) > move:
                        return False
        elif json.dumps(sa[k], sort_keys=True, default=str) != json.dumps(sb[k], sort_keys=True, default=str):
            return False
    return True


def compute_instant(sec: dict, sky: Optional[dict], eps_min: float, eps_max: float, f_ghz: float) -> dict:
    """The exact optimiser's plan of the volume (ship frame, whole volume) at one centre, every lattice spacing."""
    import numpy as np
    from . import optimizer as mo
    from . import radar_model as rm
    s = {k: v for k, v in sec.items() if k not in ("_relief", "_force_plan")}
    s["freq_ghz_candidates"] = [float(f_ghz)]
    try:
        sph = mo._sphere(eps_min, eps_max)
        best = mo._best_plan(s, sph, sph.mask(s), rm.face_elec_avail_w(rm.FACE_ELEC_POWER_MW), sky)
    except (KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
        return {"status": "ERROR", "note": str(exc)}
    if isinstance(best, dict):
        return {"status": best.get("status", "NO_PLAN"), "note": best.get("note", "")}
    f, P, unscanned, plan, cands, _sky_s, _pfas, sp = best
    P = np.asarray(P, float)
    nb = P.shape[1]
    h = mo._scan_factor(float(f), nb)
    # c (user, 12 November 2026): per position of each bin's waveform, the memory traffic and the processing energy
    # (operations x e_op + transported bits x e_bit) of core/compute.look_cost, as the exact budget counts them (the
    # look listens to the volume's range): the resources that stop the margin reuse in the exact run
    from . import compute
    s_sp = dict(s, _grid_spacing=float(sp))
    mem_b, proc_b = np.zeros(nb), np.zeros(nb)
    blind = np.full(nb, np.nan)                 # (the blind range of each bin's waveform: the played view, 5.5)
    for b in range(nb):
        c = int(plan.choice[b])
        if c < 0:
            continue
        wd = mo._wf_dict("x", cands[c], float(f), s_sp)
        blind[b] = float(wd.get("blind_range_km", np.nan))
        ppd = max(1.0, float(wd.get("positions_per_dwell", 1.0)))
        beams = float(wd.get("rx_beams_per_position", 1.0)) * (float(wd.get("b_tx", 1)) if wd.get("tx_mode") == "partition"
                                                               else ppd)
        lc = compute.look_cost("S", wd, beams, 0, float(s.get("range_km") or 0.0) or None)
        mem_b[b] = lc["mem_bits"] / ppd
        proc_b[b] = (compute.E_OP_J * sum(lc["ops"].values()) + compute.E_BIT_J * lc["bits"]) / ppd
    return {"status": "OK", "f": float(f), "spacing": float(sp), "bin_deg": float(mo.THETA_BIN_DEG),
            "e": [float(x) for x in plan.e_bin], "t": [float(x) for x in plan.t_bin], "pos": P.tolist(),
            "populated": [bool(x) for x in ((P.sum(axis=0) > 0) & (np.asarray(plan.choice) >= 0))],
            "h": None if h is None else [float(x) for x in h], "unscanned_pct": float(unscanned),
            "mem_bits": [float(x) for x in mem_b], "proc_J": [float(x) for x in proc_b],
            "blind_km": [None if not np.isfinite(x) else float(x) for x in blind],
            "confirm": _instant_confirmation(s, sph, float(f), float(sp), sky, eps_min, eps_max)}


def _instant_confirmation(s: dict, sph, f: float, sp: float, sky: Optional[dict], eps_min: float,
                          eps_max: float) -> Optional[dict]:
    """d (user, 12 November 2026): the confirmation load of the volume's false alarms with no target present (the
    exact budget's nominal view, core/tracking.sector_budget) on the plan at (f, sp), at the Ku reference frequency:
    {"mode", "e" (per face, share of the face's electrical power for transmission), "ts", "tk" (per face, share of the
    S / Ku transmit time)}; the mode forced by the volume or the admissible one of least own load. None: no budget."""
    import numpy as np
    from . import optimizer as mo
    from . import radar_model as rm
    from . import tracking as trk
    try:
        p_avg = rm.face_elec_avail_w(rm.FACE_ELEC_POWER_MW)
        r = mo.optimise_sector(dict(s, _force_plan=(f, sp)), sph, sph.mask(s), p_avg, sky)
        if r.get("status") != "OK":
            return None
        sec_c = trk._with_mission_settings({k: v for k, v in s.items() if k != "freq_ghz_candidates"}, {})
        P, _, _, _ = mo.search_positions(dict(sec_c, _grid_spacing=sp), sph, sph.mask(sec_c), f, sky)
        Pm = np.array([P[k] for k in range(4)])
        sb = trk._ku_freq_task(sec_c, r, Pm, p_avg, eps_min, eps_max, sky, trk.KU_REF_GHZ)   # (its NOMINAL view kept)
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return None
    want = s.get("confirm_mode", "auto")
    if want in trk.CONFIRM_MODES and "e" in sb["modes"][want]:
        mode = want
    else:
        ok = [md for md in trk.CONFIRM_MODES if sb["modes"][md].get("admissible")]
        if not ok:
            return None
        mode = min(ok, key=lambda md: trk._own_load(sb, md))
    nom = sb["modes"][mode].get(trk.NOMINAL) or sb["modes"][mode]
    return {"mode": mode, **{k: [float(x) for x in np.asarray(nom[k], float)] for k in ("e", "ts", "tk")}}


def _iload() -> Dict[str, list]:
    global _icache
    if _icache is None:
        try:
            with open(ICACHE_PATH, encoding="utf-8") as fh:
                _icache = json.load(fh)
        except (OSError, ValueError):
            _icache = {}
        _iorder[0] = max((e.get("order", 0) for v in _icache.values() for e in v), default=0)
    return _icache


def _isave() -> None:
    c = _iload()
    _, _, cap = _itol()
    allx = sorted(((e.get("order", 0), k, i) for k, v in c.items() for i, e in enumerate(v)))
    drop = allx[:max(0, len(allx) - cap)]
    for _o, k, i in sorted(drop, key=lambda x: -x[2]):
        del c[k][i]
    for k in [k for k, v in c.items() if not v]:
        del c[k]
    tmp = ICACHE_PATH + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(c, fh)
        os.replace(tmp, ICACHE_PATH)
    except OSError:
        pass


def _istore(k: str, snap: dict, prof: Optional[dict]) -> None:
    global _generation
    c = _iload()
    _iorder[0] += 1
    c.setdefault(k, []).append(dict(snap, profile=prof, order=_iorder[0]))
    del c[k][:-8]                                      # (the last 8 geometries of a volume)
    _isave()
    _generation += 1


def _idone(k: str, snap: dict, fut: Future) -> None:
    try:
        res = fut.result()
    except Exception as exc:  # noqa: BLE001 -- a failed instant profile: the other pricing stays
        res = {"status": "ERROR", "note": str(exc)}
    with _lock:
        _ijobs.pop(k, None)
        _istore(k, snap, res)


def _iexecutor() -> ProcessPoolExecutor:
    global _ipool
    if _ipool is None:
        import atexit
        from .optimizer import _mark_pool_worker
        _ipool = ProcessPoolExecutor(max_workers=1, initializer=_mark_pool_worker)
        atexit.register(shutdown)
    return _ipool


def get_instant(sec: dict, sky: Optional[dict], eps_min: float, eps_max: float, f_ghz: float,
                wait: bool = False) -> Tuple[Optional[dict], bool]:
    """(the volume's instant profile, fresh): fresh when its geometry holds within V2; else the last one of the volume
    (or None) while the new one is computed in the background (wait=True: computed here)."""
    k = instant_key(sec, f_ghz)
    with _lock:
        entries = _iload().get(k, [])
        for e in reversed(entries):
            if close_enough(e, sec, sky):
                return e["profile"], True
        last = entries[-1]["profile"] if entries else None
        snap = _snapshot(sec, sky)
        fut = None
        if not wait:
            if k in _ijobs:
                return last, False
            try:
                fut = _iexecutor().submit(compute_instant, dict(sec), sky, float(eps_min), float(eps_max), float(f_ghz))
                _ijobs[k] = fut
            except Exception:  # noqa: BLE001 -- no worker process (frozen install): computed here
                fut = None
    if fut is not None:                               # (outside the lock: a future already done calls back at once)
        fut.add_done_callback(lambda f_, k_=k, s_=snap: _idone(k_, s_, f_))
        return last, False
    res = compute_instant(sec, sky, eps_min, eps_max, f_ghz)
    with _lock:
        _istore(k, snap, res)
    return res, True


def instant_pending() -> int:
    with _lock:
        return len(_ijobs)
