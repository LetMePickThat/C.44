#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
POWER ALLOCATION -- the ship's electrical power shared by the four faces (roadmap step 5.2, pass P1, validated by the
user on 25 October 2026).

Need of a face = what it draws from the ship bus: its always-on consumption (receive chains, converters, control, the
processing as computed by step 4.4, not its ceiling) and pumps, plus the transmissions of every task, all through the
power supply and distribution (CHAIN_EFF). The tasks are sorted in priority tiers (the scheduler's priorities, step 4.3,
and the target class for the tracks):
  0  always-on and calibration          1  search of the self-protection volumes (P1)
  2  confirmations and initiations      3  track updates, by class: KKV > SHIP > DECOY > DEBRIS > other
  3' Ku burn-through (allowed)          4  search of the other volumes, P4_1, P4_2, ...
Enough power (sum of the needs within the ship total, each face within its feeder and the operator's cap): every face
gets a share of the ship total proportional to its need (water-filling under the caps), so the energy load of a face
is need / allocation. Shortfall: the tiers are served in order over all the faces (a P1 volume on one face before a P3
track on another), each tier pro rata of its needs within the room left on each face; the last tier served is only in
part: its search volumes get a tenable revisit (RR / served share, the energy goes with the revisit), its tracks a
tenable number; a tier P0 to P2 not served in full makes the mission not tenable. Spare power is shared pro rata.
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

from . import radar_model as rm
from .params import P

SHIP_W = float(P.get("power.ship_elec_total_mw")) * 1e6
CLASS_RANK = {"KKV": 0, "SHIP": 1, "DECOY": 2, "DEBRIS": 3}         # the rest (GENERIC, ST, ...) after
TIER_NAMES = {(0, 0): "always-on", (1, 0): "P1 self-protection search", (2, 0): "P2 confirmation / initiation",
              (3, 9): "P3' burn-through"}


def tier_name(t: Tuple[int, int]) -> str:
    if t in TIER_NAMES:
        return TIER_NAMES[t]
    if t[0] == 3:
        return "P3 tracks " + next((c for c, r in CLASS_RANK.items() if r == t[1]), "other")
    return f"P4_{t[1]} search"


def _caps(res: dict) -> List[float]:
    cap_in = res.get("global", {}).get("face_elec_cap_mw")
    caps = ([float(cap_in)] * 4 if isinstance(cap_in, (int, float)) and cap_in else
            [float(c) for c in cap_in] if isinstance(cap_in, list) else [math.inf] * 4)
    return [min(rm.FACE_FEEDER_MAX_MW, c) * 1e6 for c in caps]


def items(mission: dict, res: dict) -> List[dict]:
    """Every need of the faces: (label, sector, kind, tier, W drawn per face)."""
    secs = {s["id"]: s for s in mission.get("sectors", [])}
    p_ref = float(res["global"]["face_elec_avail_kW"]) * 1e3
    k_w = p_ref / rm.CHAIN_EFF
    out = []
    comp = res.get("compute", {}).get("faces", {})
    trk = res.get("tracking", {})
    tf = trk.get("faces", {})
    always = [0.0] * 4
    for k in range(4):
        f = tf.get(str(k), {})
        non_tx = comp.get(str(k), {}).get("non_tx_kW_actual")
        non_tx = (rm.non_tx_power_w() / 1e3) if non_tx is None else non_tx
        always[k] = (non_tx + f.get("pumps_kW", 0.0)) * 1e3 / rm.CHAIN_EFF
    out.append({"label": "always-on", "sector": None, "kind": "always", "tier": (0, 0), "w": always})
    for r in res.get("sectors", []):
        if r.get("status") != "OK":
            continue
        sec = secs.get(r["id"], {})
        tier = (1, 0) if sec.get("self_protection") else (4, int(sec.get("search_priority", 1)))
        w = [float(r.get("faces", {}).get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
        out.append({"label": f"search {r['id']}", "sector": r["id"], "kind": "search", "tier": tier, "w": w,
                    "rr_s": float(r["rr_target_s"])})
    for t in trk.get("sectors", []):
        sec = secs.get(t["id"], {})
        cf = (t.get("confirmation") or {}).get("faces", {})
        w = [float(cf.get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
        if any(w):
            out.append({"label": f"confirmation {t['id']}", "sector": t["id"], "kind": "confirm", "tier": (2, 0),
                        "w": w})
        rank = CLASS_RANK.get(sec.get("target_type") or "", len(CLASS_RANK))
        for i, a in enumerate(t.get("at", [])):
            w = [float(a.get("faces", {}).get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
            if any(w):
                out.append({"label": f"tracks {t['id']}/{i}", "sector": t["id"], "kind": "track", "tier": (3, rank),
                            "w": w, "n_targets": float(a.get("n_targets", 0.0))})
        bt = t.get("ku_burn_through") or {}
        w = [float(bt.get("faces", {}).get(str(k), {}).get("energy_fraction", 0.0)) * k_w for k in range(4)]
        if any(w):
            out.append({"label": f"burn-through {t['id']}", "sector": t["id"], "kind": "burn_through",
                        "tier": (3, 9), "w": w})
    return out


def _water_fill(need: List[float], caps: List[float], total: float) -> List[float]:
    """Share total pro rata of need, no face above its cap (the excess goes to the others)."""
    alloc = [0.0] * 4
    free = [k for k in range(4) if need[k] > 0]
    left = total
    while free and left > 1e-9:
        s = sum(need[k] for k in free)
        scale = left / s
        capped = [k for k in free if need[k] * scale >= caps[k] - alloc[k]]
        if not capped:
            for k in free:
                alloc[k] += need[k] * scale
            break
        for k in capped:
            left -= caps[k] - alloc[k]
            alloc[k] = caps[k]
            free.remove(k)
    return alloc


def allocate(mission: dict, res: dict) -> dict:
    its = items(mission, res)
    caps = _caps(res)
    need = [sum(it["w"][k] for it in its) for k in range(4)]
    flags: List[str] = []
    short = sum(need) > SHIP_W or any(need[k] > caps[k] for k in range(4))
    served = {id(it): [1.0] * 4 for it in its}
    tiers_out = []
    if not short:
        alloc = _water_fill(need, caps, SHIP_W)
        for t in sorted({it["tier"] for it in its}):
            w = sum(sum(it["w"]) for it in its if it["tier"] == t)
            tiers_out.append({"tier": tier_name(t), "need_MW": w / 1e6, "served_MW": w / 1e6, "fraction": 1.0})
    else:
        alloc = [0.0] * 4
        left = SHIP_W
        for t in sorted({it["tier"] for it in its}):
            members = [it for it in its if it["tier"] == t]
            req = [sum(it["w"][k] for it in members) for k in range(4)]
            room = [min(req[k], max(0.0, caps[k] - alloc[k])) for k in range(4)]
            tot = sum(room)
            share = min(1.0, left / tot) if tot > 0 else 0.0
            got = [room[k] * share for k in range(4)]
            for k in range(4):
                alloc[k] += got[k]
                for it in members:
                    served[id(it)][k] = got[k] / req[k] if req[k] > 0 else 1.0
            left -= sum(got)
            tiers_out.append({"tier": tier_name(t), "need_MW": sum(req) / 1e6, "served_MW": sum(got) / 1e6,
                              "fraction": sum(got) / sum(req) if sum(req) > 0 else 1.0})
        if left > 1e-6:                                     # spare power pro rata of the needs, under the caps
            extra = _water_fill([need[k] if caps[k] - alloc[k] > 0 else 0.0 for k in range(4)],
                                [caps[k] - alloc[k] for k in range(4)], left)
            alloc = [alloc[k] + extra[k] for k in range(4)]
    partial = []
    for it in its:
        fr = min((served[id(it)][k] for k in range(4) if it["w"][k] > 0), default=1.0)
        if fr >= 1.0 - 1e-9:
            continue
        rec = {"label": it["label"], "tier": tier_name(it["tier"]), "served_fraction": fr}
        if it["kind"] == "search":
            rec["rr_tenable_s"] = it["rr_s"] / fr if fr > 0 else math.inf
            flags.append(f"{it['label']}: ship power short, revisit tenable {rec['rr_tenable_s']:.3g} s "
                         f"(wanted {it['rr_s']:g} s)")
        elif it["kind"] == "track":
            rec["tracks_tenable"] = it["n_targets"] * fr
            flags.append(f"{it['label']}: ship power short, {rec['tracks_tenable']:.3g} tracks tenable "
                         f"of {it['n_targets']:g}")
        else:
            flags.append(f"{it['label']}: ship power short, only {100 * fr:.0f} % served")
        if it["tier"][0] <= 2:
            flags.append(f"mission not tenable: {it['label']} ({tier_name(it['tier'])}) is never shed")
        partial.append(rec)
    faces = {str(k): {"need_MW": need[k] / 1e6, "cap_MW": caps[k] / 1e6 if math.isfinite(caps[k]) else None,
                      "alloc_MW": alloc[k] / 1e6, "always_on_MW": its[0]["w"][k] / 1e6,
                      "load": need[k] / alloc[k] if alloc[k] > 0 else (0.0 if need[k] == 0 else math.inf)}
             for k in range(4)}
    return {"ship_MW": SHIP_W / 1e6, "need_MW": sum(need) / 1e6, "status": "SHORT" if short else "OK",
            "faces": faces, "tiers": tiers_out, "partial": partial, "flags": flags}
