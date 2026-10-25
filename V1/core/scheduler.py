#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCHEDULER -- do the tasks of each face fit in TIME, with their priorities, and how long does a detection wait (roadmap
step 4.3, blocks A-G validated by the user on 25 October 2026). Runs after core.optimizer.run() and core.tracking.budget()
on their outputs; the result goes under the report key "scheduler".

  Tasks per face and per band (S and Ku have separate transmit timelines), with the user's priorities:
    P0  calibration (without it the radar does not work)
    P1  search of the volumes flagged self_protection (a checkbox per volume)
    P2  confirmation and track initiation (never lose a detection), the S touch of mode S2
    P3  track updates (a few misses are manageable); P3' Ku burn-through, only when the operator authorises it in
        flight (never in mission preparation: not scheduled here)
    P4  search, sub-priority per volume search_priority (P4_1 first)
  (a) Statistical budget: per band, a non-preemptive priority queue (Cobham): mean wait of class k
      W_k = W0 / ((1 - s_{k-1})(1 - s_k)), W0 = sum(rho_i S_i) / 2, s_k the load of the classes up to k; the 95 %
      quantile with an exponential tail beyond the busy probability. Overload sheds P4 (lowest sub-priority first:
      its revisit stretches), then P3 (update period stretches); P0-P2 are never shed (mission not feasible).
  (b) Synthetic timeline: one deterministic timeline per face and band over the longest revisit of the face (at most
      tool.scheduler.horizon_max_s), greedy by priority then deadline, no preemption, random arrivals drawn once with a
      fixed seed; the longest contiguous transmission of the face storage (step 4.1) and of the cold plate (step 4.2)
      inserts recharge pauses. It reports start delays, late track updates, the calibration delay and the pauses.
  Common cadence per band (step 1.5): the faces transmit a band in common slots, sized by the busiest face; every face
  is deaf in them, so the search echoes lose -10 log10(1 - rho_slots) (eclipse, reported, flagged above
  scheduler.eclipse_flag_db; the plan is not redone, step 5.2). S harmonic on the Ku channel: on a flagged face the Ku
  looks avoid the S slots. Latency of a confirmation: 95 % wait + service + round trips + processing, against the
  sector's latency_max_s or latency_frac x the time to impact at the detection range.
"""
from __future__ import annotations

import heapq
import math
import random
from typing import Dict, List, Optional, Tuple

from . import radar_model as rm
from .params import P

_S, _TS = "scheduler.", "tool.scheduler."
LATENCY_FRAC = P.get(_S + "latency_frac")
UPDATE_JITTER_FRAC = P.get(_S + "update_jitter_frac")
PROCESSING_S = P.get(_S + "processing_s")
CAL_MAX_DELAY_FRAC = P.get(_S + "calibration_max_delay_frac")
ECLIPSE_FLAG_DB = P.get(_S + "eclipse_flag_db")
HORIZON_MAX_S = P.get(_TS + "horizon_max_s")
REVISIT_FLAG_FRAC = P.get("tool.scanorder.revisit_flag_frac")
SEED = P.get(_TS + "seed")
C = rm.C_MPS


def priority_of(sec: dict) -> Tuple[int, int]:
    """(class, sub-priority) of a search volume: P1 when flagged self_protection, else P4_<search_priority>."""
    return (1, 0) if sec.get("self_protection") else (4, int(sec.get("search_priority", 1)))


def latency_requirement_s(sec: dict) -> float:
    """The sector's latency_max_s, else latency_frac x the time to impact at the detection range (E3)."""
    if sec.get("latency_max_s") is not None:
        return float(sec["latency_max_s"])
    return LATENCY_FRAC * float(sec["range_km"]) * 1e3 / float(sec["v_close_max_mps"])


class Klass:
    """One task class of a face and band: priority, load rho (share of the time), service time S, label."""

    def __init__(self, prio: Tuple[int, int], label: str, rho: float, service_s: float, kind: str,
                 sector: Optional[str] = None, period_s: Optional[float] = None, count: float = 0.0,
                 dwells: Optional[List[float]] = None):
        self.prio, self.label, self.rho, self.s, self.kind = prio, label, max(0.0, rho), max(service_s, 0.0), kind
        self.sector, self.period, self.count = sector, period_s, count
        self.dwells = dwells or []          # step 4.3 bis: the search dwells of a frame in scan order

    @property
    def rate(self) -> float:
        return self.rho / self.s if self.s > 0 else 0.0


def _classes(k: int, mission: dict, res: dict, scans: Optional[Dict[str, Dict[int, dict]]] = None) -> Dict[str, List[Klass]]:
    """Task classes of face k per band from the optimiser and the confirmation / tracking budget (A1-A6)."""
    secs = {s["id"]: s for s in mission["sectors"]}
    trk = res.get("tracking", {})
    tsec = {s["id"]: s for s in trk.get("sectors", [])}
    face = trk.get("faces", {}).get(str(k), {})
    out: Dict[str, List[Klass]] = {"S": [], "Ku": []}
    for b, key in (("S", "calibration_s_time"), ("Ku", "calibration_ku_time")):
        er = rm.BANDS[b].errors
        out[b].append(Klass((0, 0), f"calibration {b}", face.get(key, 0.0), er["cal_duration_s"], "cal",
                            period_s=er["cal_interval_s"]))
    for r in res.get("sectors", []):
        f = r.get("faces", {}).get(str(k))
        if r.get("status") != "OK" or not f or f["positions"] <= 0:
            continue
        sec = secs[r["id"]]
        rr = float(r["rr_target_s"])
        rho = float(f["time_fraction"])
        sc = (scans or {}).get(r["id"], {}).get(k)
        out["S"].append(Klass(priority_of(sec), f"search {r['id']}", rho, rho * rr / f["positions"], "search",
                              r["id"], rr, f["positions"], sc["dwell_times_s"] if sc else None))
        t = tsec.get(r["id"])
        if t is None:
            continue
        conf = t.get("confirmation", {}).get("faces", {}).get(str(k), {})
        n_new = float(sec.get("new_targets_per_rr", 1.0)) + float(sec.get("objects_per_rr", 0.0))
        rate = (n_new * t["p_share"][k] + t["false_alarms_per_rr_face"][k]) / rr
        for b, key in (("S", "s_time_fraction"), ("Ku", "ku_time_fraction")):
            rho_c = float(conf.get(key, 0.0))
            if rho_c > 0 and rate > 0:
                out[b].append(Klass((2, 0), f"confirmation {r['id']}", rho_c, rho_c / rate, "confirm", r["id"]))
        for i, at in enumerate(t.get("at", [])):
            tk = float(at.get("faces", {}).get(str(k), {}).get("ku_time_fraction", 0.0))
            init = float(at.get("init_tk", [0.0] * 4)[k])
            n_tr = float(at.get("tracks_face", [0.0] * 4)[k])
            if init > 0 and at.get("initiations_per_s"):
                r_i = at["initiations_per_s"] * t["p_share"][k]
                if r_i > 0:
                    out["Ku"].append(Klass((2, 0), f"initiation {r['id']}/{i}", init, init / r_i, "init", r["id"]))
            upd = max(0.0, tk - init)
            if upd > 0 and n_tr > 0:
                period = float(at["rr_s"])
                out["Ku"].append(Klass((3, 0), f"track updates {r['id']}/{i}", upd, upd * period / n_tr, "update",
                                       r["id"], period, n_tr))
    return out


def _cobham(classes: List[Klass]) -> Dict[str, float]:
    """Mean wait per class label of a non-preemptive priority queue, and the probability the server is busy."""
    order = sorted(classes, key=lambda c: c.prio)
    w0 = sum(c.rho * c.s for c in order) / 2.0
    out, sigma, prev = {}, 0.0, 0.0
    groups: Dict[Tuple[int, int], float] = {}
    for c in order:
        groups[c.prio] = groups.get(c.prio, 0.0) + c.rho
    cum: Dict[Tuple[int, int], Tuple[float, float]] = {}
    for p_ in sorted(groups):
        prev, sigma = sigma, sigma + groups[p_]
        cum[p_] = (prev, sigma)
    for c in order:
        a, b = cum[c.prio]
        out[c.label] = w0 / ((1.0 - a) * (1.0 - b)) if b < 1.0 else math.inf
    out["_busy"] = min(1.0, sum(c.rho for c in order))
    return out


def _q95(mean_wait: float, busy: float) -> float:
    """95 % quantile of the wait: zero below 5 % busy, else an exponential tail of mean W / busy."""
    if not math.isfinite(mean_wait):
        return math.inf
    if busy <= 0.05:
        return 0.0
    return mean_wait / busy * math.log(busy / 0.05)


def _shed(classes: List[Klass], capacity: float) -> Tuple[Dict[str, float], bool]:
    """Stretch factors per label so that the load fits the capacity: P4 from the lowest sub-priority, then P3 (F1);
    infeasible when P0-P2 alone exceed it."""
    stretch = {c.label: 1.0 for c in classes}
    hard = sum(c.rho for c in classes if c.prio[0] <= 2)
    if hard > capacity:
        return stretch, False
    left = capacity - hard
    soft = sorted([c for c in classes if c.prio[0] > 2], key=lambda c: c.prio)       # most important first
    for c in soft:
        if c.rho <= left:
            left -= c.rho
        else:
            stretch[c.label] = c.rho / left if left > 1e-12 else math.inf
            left = 0.0
    return stretch, True


def _timeline(classes: List[Klass], horizon: float, rng: random.Random, max_contiguous_s: float,
              pause_ratio: float) -> Dict[str, float]:
    """Greedy timeline of one face and band (b): releases of the periodic tasks spread over their period, Poisson
    arrivals for confirmations and initiations, one calibration released at t = 0; no preemption; a recharge pause of
    busy x pause_ratio after max_contiguous_s of contiguous transmission."""
    tasks = []                                      # (release, prio, deadline, duration, label, kind)
    for c in classes:
        if c.s <= 0 or c.rho <= 0:
            continue
        if c.kind == "cal":
            tasks.append((0.0, c.prio, c.period * CAL_MAX_DELAY_FRAC, c.s, c.label, c.kind))
        elif c.kind == "search" and c.dwells:
            # step 4.3 bis: the real dwells of the frame in scan order, spread over the revisit, frame after frame
            step = c.period / len(c.dwells)
            t, j = rng.random() * step, 0
            while t < horizon:
                tasks.append((t, c.prio, t + c.period, c.dwells[j % len(c.dwells)], c.label, c.kind))
                t += step
                j += 1
        elif c.kind in ("search", "update"):
            n_per = c.count
            if n_per <= 0:
                continue
            step = c.period / n_per
            t = rng.random() * step
            while t < horizon:
                dl = t + (c.period if c.kind == "search" else UPDATE_JITTER_FRAC * c.period)
                tasks.append((t, c.prio, dl, c.s, c.label, c.kind))
                t += step
        else:
            t = rng.expovariate(c.rate) if c.rate > 0 else math.inf
            while t < horizon:
                tasks.append((t, c.prio, t + 1e9, c.s, c.label, c.kind))
                t += rng.expovariate(c.rate)
    tasks.sort()
    ready: List[tuple] = []
    i, t, busy_run = 0, 0.0, 0.0
    delays: Dict[str, List[float]] = {}
    late_updates = n_updates = 0
    cal_delay, pauses, pause_time = 0.0, 0, 0.0
    while i < len(tasks) or ready:
        while i < len(tasks) and tasks[i][0] <= t:
            rel, prio, dl, dur, lab, kind = tasks[i]
            heapq.heappush(ready, (prio, dl, rel, dur, lab, kind))
            i += 1
        if not ready:
            t = tasks[i][0]
            busy_run = 0.0
            continue
        prio, dl, rel, dur, lab, kind = heapq.heappop(ready)
        if busy_run + dur > max_contiguous_s and busy_run > 0:
            pause = busy_run * pause_ratio
            t += pause
            pauses += 1
            pause_time += pause
            busy_run = 0.0
        delays.setdefault(lab, []).append(t - rel)
        if kind == "update":
            n_updates += 1
            late_updates += (t > dl)
        if kind == "cal":
            cal_delay = max(cal_delay, t - rel)
        t += dur
        busy_run += dur
    return {"max_delay_s": {lab: max(v) for lab, v in delays.items()},
            # step 4.3 bis (V5): spread of the start delays of a search volume = spread of the revisit of its directions
            "revisit_spread_s": {c.label: max(delays[c.label]) - min(delays[c.label]) for c in classes
                                 if c.kind == "search" and delays.get(c.label)},
            "late_update_frac": late_updates / n_updates if n_updates else 0.0,
            "calibration_delay_s": cal_delay, "recharge_pauses": pauses, "pause_time_s": pause_time,
            "end_s": t}


def schedule(mission: dict, res: dict) -> dict:
    """Scheduler verdict of a mission (step 4.3): per face and band the loads by priority, the waits, the shedding,
    the eclipse of the common cadence, the synthetic timeline; per sector the confirmation latency against its
    requirement; flags."""
    secs = {s["id"]: s for s in mission["sectors"]}
    trk = res.get("tracking", {})
    tsec = {s["id"]: s for s in trk.get("sectors", [])}
    rng = random.Random(SEED)
    # step 4.3 bis: the real search pointings of each volume per face, in scan order
    from . import optimizer as mo
    from . import scanorder as so
    g = res.get("global", {})
    sph = mo._sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    sky = mission.get("global", {}).get("sky")
    scans = {r["id"]: so.sector_scan(secs[r["id"]], r, sph, sky) for r in res.get("sectors", [])
             if r.get("status") == "OK"}
    per_face = {k: _classes(k, mission, res, scans) for k in range(4)}
    # common cadence per band (D1): slots sized by the busiest face; S harmonic on the Ku channel (D2)
    slots = {b: max(sum(c.rho for c in per_face[k][b]) for k in range(4)) for b in ("S", "Ku")}
    eclipse = {b: -10.0 * math.log10(max(1e-9, 1.0 - min(slots[b], 0.999999))) for b in slots}
    harm_faces = set()
    for sid, t in tsec.items():
        if any("harmonics" in f_ for f_ in t.get("flags", [])):
            harm_faces.update(k for k, v in enumerate(t.get("p_share", [])) if v > 0)
    flags: List[str] = []
    faces: Dict[str, dict] = {}
    waits: Dict[Tuple[int, str], Dict[str, float]] = {}
    for k in range(4):
        fk = {}
        for b in ("S", "Ku"):
            cl = per_face[k][b]
            cap = 1.0 - (slots["S"] if (b == "Ku" and k in harm_faces) else 0.0)
            stretch, ok = _shed(cl, cap)
            w = _cobham(cl)
            waits[(k, b)] = w
            # one horizon per face: its longest search revisit (the Ku timeline follows the same window)
            r_max = max([c.period for c in per_face[k]["S"] if c.kind == "search" and c.period] + [1.0])
            horizon = min(r_max, HORIZON_MAX_S)
            lim, ratio = _contiguous(k, b, trk)
            tl = _timeline(cl, horizon, rng, lim, ratio) if cl else {}
            load = sum(c.rho for c in cl)
            fk[b] = {"load": load, "capacity": cap, "feasible": ok,
                     "load_by_priority": _by_prio(cl), "stretch": {lab: v for lab, v in stretch.items() if v > 1.0},
                     "mean_wait_s": {lab: v for lab, v in w.items() if not lab.startswith("_")},
                     "timeline": tl}
            if not ok:
                flags.append(f"face F{k + 1} {b}: calibration, self-protection, confirmation and initiation alone "
                             f"exceed the transmit time: mission not feasible (step 4.3)")
            for lab, v in stretch.items():
                if v > 1.0:
                    flags.append(f"face F{k + 1} {b}: {lab} stretched x{v:.2f} (priority shedding, step 4.3)")
            for c in cl:
                sp = (tl or {}).get("revisit_spread_s", {}).get(c.label)
                if c.kind == "search" and sp is not None and c.period and sp > REVISIT_FLAG_FRAC * c.period:
                    flags.append(f"face F{k + 1}: {c.label}: revisit of its directions spread by {sp:.2f} s "
                                 f"({100 * sp / c.period:.0f} % of the {c.period:g} s revisit, step 4.3 bis)")
            if tl and tl["late_update_frac"] > 0.0:
                flags.append(f"face F{k + 1} {b}: {100 * tl['late_update_frac']:.1f} % of the track updates start "
                             f"later than {100 * UPDATE_JITTER_FRAC:g} % of their period in the synthetic timeline")
            if tl and tl["calibration_delay_s"] > 0 and any(c.kind == "cal" for c in cl):
                cal = next(c for c in cl if c.kind == "cal")
                if tl["calibration_delay_s"] > cal.period * CAL_MAX_DELAY_FRAC:
                    flags.append(f"face F{k + 1} {b}: calibration delayed {tl['calibration_delay_s']:.1f} s")
        faces[str(k)] = fk
    for b, v in eclipse.items():
        if v > ECLIPSE_FLAG_DB:
            flags.append(f"{b}: common transmit slots take {100 * slots[b]:.1f} % of the time: echoes eclipsed, "
                         f"{v:.2f} dB (step 4.3; not in the plan yet, step 5.2)")
    # latency of the confirmations per sector (E1-E3)
    sectors = []
    for r in res.get("sectors", []):
        t = tsec.get(r["id"])
        if r.get("status") != "OK" or t is None:
            continue
        sec = secs[r["id"]]
        req = latency_requirement_s(sec)
        mode = t.get("confirm_mode")
        rtt = 2.0 * float(sec["range_km"]) * 1e3 / C
        trips = float(t.get("confirmation", {}).get("round_trips", 1) or 1)
        lat = 0.0
        for k in (int(x) for x in t.get("confirmation", {}).get("faces", {}).keys()):
            w = waits[(k, "Ku")]
            lab = f"confirmation {r['id']}"
            cls = next((c for c in per_face[k]["Ku"] if c.label == lab), None)
            if cls is None:
                continue
            lat = max(lat, _q95(w.get(lab, 0.0), w["_busy"]) + cls.s + trips * rtt + PROCESSING_S)
        ok = lat <= req
        sectors.append({"id": r["id"], "priority": "P1" if sec.get("self_protection") else
                        f"P4_{int(sec.get('search_priority', 1))}", "confirm_mode": mode,
                        "latency_q95_s": lat, "latency_required_s": req, "latency_ok": ok,
                        "go_cfar_advised": bool(r.get("sky", {}).get("limbs")),
                        # step 4.3 bis: the scan of the volume per face
                        "scan": {str(k): {x: v[x] for x in ("pointings", "plan_positions", "frame_tx_s", "uniformity",
                                                             "mean_step_deg", "pointing_rate_hz")}
                                 for k, v in scans.get(r["id"], {}).items()}})
        if not ok:
            flags.append(f"sector {r['id']}: confirmation latency {lat:.3g} s (95 %) above the {req:.3g} s required")
    return {"faces": faces, "slots": slots, "eclipse_db": eclipse, "sectors": sectors, "flags": flags,
            "harmonic_faces": sorted(harm_faces), "frame_interleaved_checked": True}


def _by_prio(classes: List[Klass]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for c in classes:
        key = f"P{c.prio[0]}" + (f"_{c.prio[1]}" if c.prio[0] == 4 else "")
        out[key] = out.get(key, 0.0) + c.rho
    return out


def _contiguous(k: int, band: str, trk: dict) -> Tuple[float, float]:
    """Longest contiguous transmission of face k (the stricter of the storage and the cold plate, steps 4.1-4.2) and
    the recharge pause per second of transmission."""
    f = trk.get("faces", {}).get(str(k), {})
    lim = min(f.get("max_contiguous_tx_s", math.inf), f.get("max_contiguous_tx_thermal_s", math.inf))
    peak = rm.BANDS[band].p_pk_total_w / rm.BANDS[band].pae_sat
    cap = f.get("cap_MW", rm.FACE_FEEDER_MAX_MW) * 1e6
    rech = max(1.0, cap * rm.CHAIN_EFF - rm.non_tx_power_w())
    return lim, max(0.0, (peak - rech) / rech)
