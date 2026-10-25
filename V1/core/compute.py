#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COMPUTE AND DATA -- the digital chain of each face (roadmap step 4.4, validated by the user on 25 October 2026).

Architecture (A): hierarchical. Every TRSA brings its channels down to the signal bandwidth (digital down-conversion:
complex samples at adc_oversampling x the signal bandwidth, I and Q on compute.iq_bits). Per band, compute.transport_<b>
says what the TRSA sends to the face processor: 'raw' (its channels; S: 50 channels, fewer than the beams of a dwell)
or 'subarray_beams' (partial beams; Ku: 1250 channels for a few beams). The face processor forms the beams, compresses,
runs the velocity / acceleration banks and the CFAR, and nulls the sidelobe jammers on nulling_dof degrees of freedom;
the central computer keeps the tracks. The Ku data are processed during the Ku looks only (A5).

Load (B), per look of a face (S search dwell, Ku confirmation, Ku track update), counted from the look itself:
  fs = adc_oversampling x B (complex), listening time T = N x PRI, samples per pulse n_p = fs x PRI, range cells of the
  detection grid c_r = range_grid_oversampling x B x PRI, hypotheses h = velocity x acceleration hypotheses,
  Doppler bins M = doppler_zero_padding x pulses per coherent block (1 for a single pulse), beams b (RX beams of the dwell):
  beamforming    ops_per_cmac x channels x b x fs x T            (+ the TRSA partial beams summed: 2 N_TRSA x b x fs x T)
  compression    fft_ops_factor x 2 pol x b x N x n_p log2 n_p
  banks          ops_per_cmac x 2 pol x b x h x c_r x N           (+ Doppler FFT: fft_ops_factor x 2 b h c_r x blocks x M log2 M)
  CFAR           cfar_ops_per_cell x b x h x c_r x M
  nulling        ops_per_cmac x (d^2 x snapshots + d^3 / 3 + d x b x fs x T), d = degrees of freedom, on a face a
                 stand-off jammer lights (up to jamming.max_nulled)
  tracks         track_update_ops per update
Means over the mission = sum of rate x cost. The looks of a face and band follow one another, their listening windows
overlap: open windows ~ sum of rate x T (Little), so the simultaneous beams are ceil(that) x the beams of the largest
dwell (an upper bound) and the mean beams sum rate x T x beams. RX beams of a dwell = RX beams per search position x
positions per dwell (a partition's sub-aperture beam already counts the positions it covers). Transport = the bits the
TRSAs send during T; buffers = beams x 2 pol x N x n_p complex words.
Memory: the coherent buffers of the open looks (S + Ku) against compute.memory_tb; the memory traffic against compute.memory_bw_tbps: writes of
the received samples and of the compressed samples, ONE read of the compressed samples per dwell (the banks work in the
processor's on-chip memory, user decision after step 4.4; their re-reads are in e_op), words of 2 x iq_bits.
Power (C): the processing power ceiling power.dbf_kw sets the capacity: (ceiling x (1 - static_frac) - transport power)
/ e_op; the consumption is static_frac x ceiling + e_op x ops + e_bit x bits. The power and heat budgets that size the
plan keep the ceiling (conservative); the consumption and the corrected draw / heat are reported beside them.
Latency (D): the data stream to the face processor during the listening; after the end of the dwell remains the backlog
of a link slower than the stream (bits / face input - T, if positive), then the processing at the capacity left by the
mean load. It replaces the provisional scheduler.processing_s.
In flight (E1): re-selecting a precomputed waveform when the measured spectrum changes costs reselect_ops_per_entry per
(waveform, scan bin) entry of the sector (the full re-optimisation stays in mission prep or step 5.2).
Every value is read from radar_system.toml [compute] and [power], so the code follows the architecture as it is refined.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from . import environment as env
from . import radar_model as rm
from .params import P

_C = "compute."
TRANSPORT = {b: P.get(_C + f"transport_{b}") for b in ("S", "Ku")}
IQ_BITS = int(P.get(_C + "iq_bits"))
BEAMS_MAX = {b: int(P.get(_C + f"beams_max_{b}")) for b in ("S", "Ku")}
NULLING_DOF = P.get(_C + "nulling_dof")
OPS_CMAC = float(P.get(_C + "ops_per_cmac"))
FFT_K = float(P.get(_C + "fft_ops_factor"))
CFAR_OPS = float(P.get(_C + "cfar_ops_per_cell"))
NULL_SNAP = float(P.get(_C + "nulling_snapshots_per_dof"))
TRACK_OPS = float(P.get(_C + "track_update_ops"))
E_OP_J = float(P.get(_C + "e_op_pj")) * 1e-12
E_BIT_J = float(P.get(_C + "e_bit_pj")) * 1e-12
STATIC_FRAC = float(P.get(_C + "static_frac"))
LINK_TRSA_BPS = float(P.get(_C + "link_trsa_tbps")) * 1e12
FACE_INPUT_BPS = float(P.get(_C + "face_input_tbps")) * 1e12
MEMORY_B = float(P.get(_C + "memory_tb")) * 1e12
MEMORY_BW_BPS = float(P.get(_C + "memory_bw_tbps")) * 1e12 * 8      # bit/s
RESELECT_OPS = float(P.get(_C + "reselect_ops_per_entry"))
CEILING_W = float(P.get("power.dbf_kw")) * 1e3
ADC_OVERSAMPLING = {b: float(P.get(f"band.{b}.rx.adc_oversampling")) for b in ("S", "Ku")}
RANGE_OSR = float(P.get("processing.range_grid_oversampling"))
DOPPLER_ZP = float(P.get("processing.doppler_zero_padding"))
for _b, _t in TRANSPORT.items():
    if _t not in ("raw", "subarray_beams"):
        raise ValueError(f"{P.path}: compute.transport_{_b} '{_t}' must be 'raw' or 'subarray_beams'")
if NULLING_DOF not in ("trsa", "element"):
    raise ValueError(f"{P.path}: compute.nulling_dof '{NULLING_DOF}' must be 'trsa' or 'element'")
FUNCTIONS = ("beamforming", "compression", "banks", "cfar", "nulling", "tracks")


def _nlogn(n: float) -> float:
    n = max(2.0, float(n))
    return n * math.log2(n)


def channels(band: str) -> int:
    """Receive channels of a face (two polarisations per TRM)."""
    return 2 * rm.N_TRM[band]


def dof(band: str) -> int:
    return rm.N_TRSA if NULLING_DOF == "trsa" else channels(band)


def look_cost(band: str, wf: dict, beams: float, nulled: int = 0) -> Dict[str, float]:
    """Operations per function, transported bits, buffer bytes and listening time of ONE look (dwell) of a face.
    wf: n_pulses, pri_ms, sig_bw_mhz, velocity_hypotheses, acceleration_hypotheses, coherent_blocks, pulses_per_block."""
    n = max(1, int(wf["n_pulses"]))
    pri = float(wf["pri_ms"]) * 1e-3
    bw = float(wf["sig_bw_mhz"]) * 1e6
    fs = ADC_OVERSAMPLING[band] * bw
    t = n * pri
    n_p = fs * pri
    c_r = RANGE_OSR * bw * pri
    h = max(1, int(wf.get("velocity_hypotheses", 1))) * max(1, int(wf.get("acceleration_hypotheses", 1)))
    ppb = max(1, int(wf.get("pulses_per_block", n)))
    blocks = max(1, int(wf.get("coherent_blocks", 1)))
    m = DOPPLER_ZP * ppb if ppb > 1 else 1.0
    ch = channels(band)
    sub = TRANSPORT[band] == "subarray_beams"
    ops = {"beamforming": OPS_CMAC * (ch + (2 * rm.N_TRSA if sub else 0)) * beams * fs * t,
           "compression": FFT_K * 2 * beams * n * _nlogn(n_p),
           "banks": OPS_CMAC * 2 * beams * h * c_r * n + (FFT_K * 2 * beams * h * c_r * blocks * _nlogn(m) if m > 1 else 0.0),
           "cfar": CFAR_OPS * beams * h * c_r * m,
           "nulling": 0.0, "tracks": 0.0}
    if nulled > 0:
        d = dof(band)
        ops["nulling"] = OPS_CMAC * (d * d * NULL_SNAP * d + d ** 3 / 3.0 + d * beams * fs * t)
    words = (2 * rm.N_TRSA * beams) if sub else ch          # complex words per sample instant the TRSAs send
    word = 2 * IQ_BITS
    mem_bits = (words * fs * t + 2 * beams * 2 * n * n_p) * word      # writes + one read per dwell (banks on chip)
    return {"ops": ops, "bits": words * fs * t * word, "buffer_B": beams * 2 * n * n_p * word / 8, "mem_bits": mem_bits,
            "listen_s": t, "fs": fs, "words": words}


def _cluster_beams(name: Optional[str]) -> int:
    """RX beams of a Ku confirmation cluster: 'superposition x7' / 'partition x3' -> 7 / 3, 'sequential' -> 1."""
    if not name or "x" not in name:
        return 1
    try:
        return max(1, int(str(name).rsplit("x", 1)[1]))
    except ValueError:
        return 1


def _nulled_faces(mission: dict) -> List[int]:
    """Jammers each face nulls: the stand-off jammers in front of it, at most jamming.max_nulled (step 2.5)."""
    jams = env.jammers(mission.get("global", {}).get("sky"))
    out = []
    for k in range(4):
        n_ = np.asarray(env.face_frame(k)[0])
        out.append(min(env.JAM_MAX_NULLED, sum(1 for j in jams if float(np.dot(j.c, n_)) > 0.0)))
    return out


def _streams(mission: dict, res: dict, nulled: List[int]) -> Dict[int, Dict[str, list]]:
    """Per face and band, the streams of looks: (label, sector, kind, [(rate Hz, look cost, beams)], track updates /s)."""
    out = {k: {"S": [], "Ku": []} for k in range(4)}
    for r in res.get("sectors", []):
        if r.get("status") != "OK":
            continue
        tot = float(r.get("positions_total") or 0.0)
        rr = float(r["rr_target_s"])
        for kk, f in r.get("faces", {}).items():
            k = int(kk)
            share = float(f["positions"]) / tot if tot > 0 else 0.0
            looks = []
            for b in r.get("wf_by_bin", []):
                w = b["wf"]
                ppd = max(1.0, float(w.get("positions_per_dwell", 1.0)))
                # RX beams of the dwell: the partition's field is per TX beam (sub-aperture), the others' per position
                beams = float(w.get("rx_beams_per_position", 1.0)) * (
                    float(w.get("b_tx", 1)) if w.get("tx_mode") == "partition" else ppd)
                rate = float(b["positions"]) * share / ppd / rr
                if rate > 0:
                    looks.append((rate, look_cost("S", w, beams, nulled[k]), beams))
            if looks:
                out[k]["S"].append({"label": f"search {r['id']}", "sector": r["id"], "kind": "search",
                                    "looks": looks, "updates": 0.0})
    for t in res.get("tracking", {}).get("sectors", []):
        conf = t.get("confirmation") or {}
        cw = t.get("ku_confirm_wf")
        if cw:
            beams = _cluster_beams(conf.get("cluster_tx"))
            for kk, f in conf.get("faces", {}).items():
                k = int(kk)
                tx = cw["n_pulses"] * cw["tau_us"] * 1e-6
                rate = float(f.get("ku_time_fraction", 0.0)) / tx if tx > 0 else 0.0
                if rate > 0:
                    out[k]["Ku"].append({"label": f"confirmation {t['id']}", "sector": t["id"], "kind": "confirm",
                                         "looks": [(rate, look_cost("Ku", cw, beams, nulled[k]), beams)],
                                         "updates": 0.0})
        for i, a in enumerate(t.get("at", [])):
            w = a.get("wf_params")
            if not w:
                continue
            tx = w["n_pulses"] * w["tau_us"] * 1e-6
            for kk, f in a.get("faces", {}).items():
                k = int(kk)
                rate = float(f.get("ku_time_fraction", 0.0)) / tx if tx > 0 else 0.0
                trk = float((a.get("tracks_face") or [0.0] * 4)[k])
                upd = trk / float(a["rr_s"]) if a.get("rr_s") else 0.0
                if rate > 0 or upd > 0:
                    out[k]["Ku"].append({"label": f"tracks {t['id']}/{i}", "sector": t["id"], "kind": "track",
                                         "looks": [(rate, look_cost("Ku", w, 1.0, nulled[k]), 1.0)] if rate > 0 else [],
                                         "updates": upd})
    return out


def budget(mission: dict, res: dict) -> dict:
    """Compute and data budget of every face (step 4.4): loads, resources against capacities, power, latency."""
    nulled = _nulled_faces(mission)
    streams = _streams(mission, res, nulled)
    trk_faces = res.get("tracking", {}).get("faces", {})
    faces, flags, lat_sector = {}, [], {}
    for k in range(4):
        fk = {"jammers_nulled": nulled[k]}
        ops_tot, bits_tot, buf_tot, mem_tot = 0.0, 0.0, 0.0, 0.0
        per_band = {}
        for b in ("S", "Ku"):
            ops = dict.fromkeys(FUNCTIONS, 0.0)
            looks = [lk for s in streams[k][b] for lk in s["looks"]]
            bits = 0.0
            conc = sum(rate * c["listen_s"] for rate, c, _ in looks)          # open listening windows (Little)
            n_open = math.ceil(conc - 1e-12) if looks else 0
            beams_mean = sum(rate * c["listen_s"] * bm for rate, c, bm in looks)
            beams_peak = n_open * max((bm for _, _, bm in looks), default=0.0)
            buf_peak = n_open * max((c["buffer_B"] for _, c, _ in looks), default=0.0)
            fs_max = max((c["fs"] for _, c, _ in looks), default=0.0)
            link_peak = ((channels(b) / rm.N_TRSA) if TRANSPORT[b] == "raw" else 2 * beams_peak) * fs_max * 2 * IQ_BITS
            for s in streams[k][b]:
                for rate, c, _ in s["looks"]:
                    for fn, v in c["ops"].items():
                        ops[fn] += rate * v
                    bits += rate * c["bits"]
                    mem_tot += rate * c["mem_bits"]
                ops["tracks"] += s["updates"] * TRACK_OPS
            tot = sum(ops.values())
            per_band[b] = {"ops_per_s": ops, "ops_total_per_s": tot, "bits_per_s": bits,
                           "open_windows_mean": conc, "beams_mean": beams_mean, "beams_peak": beams_peak, "beams_max": BEAMS_MAX[b],
                           "buffer_peak_GB": buf_peak / 1e9, "link_trsa_peak_tbps": link_peak / 1e12,
                           "face_input_peak_tbps": link_peak * rm.N_TRSA / 1e12, "transport": TRANSPORT[b]}
            ops_tot += tot
            bits_tot += bits
            buf_tot += buf_peak
            if beams_peak > BEAMS_MAX[b]:
                flags.append(f"face F{k + 1} {b}: {beams_peak:.0f} simultaneous RX beams needed, the beamformer "
                             f"forms {BEAMS_MAX[b]}")
            if link_peak > LINK_TRSA_BPS:
                flags.append(f"face F{k + 1} {b}: TRSA link {link_peak / 1e12:.2f} Tbit/s needed, capacity "
                             f"{LINK_TRSA_BPS / 1e12:g}")
            if link_peak * rm.N_TRSA > FACE_INPUT_BPS:
                flags.append(f"face F{k + 1} {b}: face processor input {link_peak * rm.N_TRSA / 1e12:.1f} Tbit/s "
                             f"needed, capacity {FACE_INPUT_BPS / 1e12:g}")
        if buf_tot > MEMORY_B:
            flags.append(f"face F{k + 1}: coherent buffers {buf_tot / 1e12:.2f} TB, memory {MEMORY_B / 1e12:g} TB")
        if mem_tot > MEMORY_BW_BPS:
            flags.append(f"face F{k + 1}: memory traffic {mem_tot / 8e12:.1f} TB/s, bandwidth {MEMORY_BW_BPS / 8e12:g} TB/s")
        p_static = STATIC_FRAC * CEILING_W
        p_link = E_BIT_J * bits_tot
        capacity = max(0.0, (CEILING_W - p_static - p_link) / E_OP_J)
        p_compute = p_static + E_OP_J * ops_tot + p_link
        load = ops_tot / capacity if capacity > 0 else math.inf
        if load > 1.0:
            flags.append(f"face F{k + 1}: processing load {100 * load:.0f} % of the capacity "
                         f"({ops_tot:.3g} op/s against {capacity:.3g}; ceiling power.dbf_kw)")
        left = max(capacity - ops_tot, 1e-9 * max(capacity, 1.0))
        # latency of a search dwell: its data reach the face processor, then are processed with the capacity left
        for s in streams[k]["S"]:
            for rate, c, _ in s["looks"]:
                lat = max(0.0, c["bits"] / FACE_INPUT_BPS - c["listen_s"]) + sum(c["ops"].values()) / left
                lat_sector[s["sector"]] = max(lat_sector.get(s["sector"], 0.0), lat)
        tf = trk_faces.get(str(k), {})
        delta = CEILING_W - p_compute
        fk.update({"memory": {"buffers_peak_TB": buf_tot / 1e12, "capacity_TB": MEMORY_B / 1e12,
                              "traffic_TB_per_s": mem_tot / 8e12, "bandwidth_TB_per_s": MEMORY_BW_BPS / 8e12},
                   "bands": per_band, "ops_total_per_s": ops_tot, "capacity_ops_per_s": capacity,
                   "load": load, "bits_per_s": bits_tot,
                   "power_kW": {"ceiling": CEILING_W / 1e3, "static": p_static / 1e3, "ops": E_OP_J * ops_tot / 1e3,
                                "transport": p_link / 1e3, "total": p_compute / 1e3},
                   # the budgets that size the plan keep the ceiling; the consumption corrects them here
                   "non_tx_kW_actual": tf["non_tx_kW"] - delta / 1e3 if "non_tx_kW" in tf else None,
                   "bus_draw_MW_actual": tf["bus_draw_MW"] - delta / rm.CHAIN_EFF / 1e6 if "bus_draw_MW" in tf else None,
                   "face_heat_kW_actual": tf["face_heat_kW"] - delta / 1e3 if "face_heat_kW" in tf else None})
        faces[str(k)] = fk
    reselect = {r["id"]: RESELECT_OPS * len(r.get("wf_set") or [r.get("wf")]) * len(r.get("wf_by_bin", []))
                for r in res.get("sectors", []) if r.get("status") == "OK"}
    return {"faces": faces, "sectors": {sid: {"processing_s": v, "reselect_ops": reselect.get(sid)}
                                        for sid, v in lat_sector.items()},
            "reselect_ops": reselect, "flags": flags}


def processing_s(res: dict, sector_id: str) -> float:
    """Processing latency of a sector's detections (step 4.4), 0 when the budget has not run."""
    return float(((res.get("compute") or {}).get("sectors", {}).get(sector_id) or {}).get("processing_s", 0.0))
