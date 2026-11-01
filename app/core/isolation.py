#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ISOLATION -- leaks between the bands and the faces (roadmap step 1.5).

For every pair (transmitted band -> received band, faces: same / adjacent / opposite), the leak reaching ONE receive
element (antenna port, the reference of T_sys and of the clipping level) and three checks:
  LNA blocking      : leak before the LNA (after the preselection filter if out of band) <= LNA P1dB - lna_block_backoff_db
  ADC clipping      : leak left at the ADC (after preselection and post-LNA rejection if out of band) <= clipping level
  desensitisation   : a leak IN the received band <= noise - desens_margin_db
Leaks: same face, cross band = one S TRM to the neighbouring Ku elements, or the Ku TRMs of one S cell to the S element
(isolation.intra_trsa_*); other faces = the face's total peak power minus coupling_adjacent_db / coupling_opposite_db.
The S PA harmonic that falls in the Ku band (orders derived from the two bands, 5 today) is in band for the Ku
receiver: its level is the S leak + harmonic_in_ku_dbc - harmonic_filter_rej_in_ku_db, the same coupling assumed at the
harmonic. Verdicts: "allowed", "forbidden", "frequency_rule" (allowed only if the S harmonic misses the Ku channel),
"eclipsing" (same band, same face: step 2.1). With interface_mode "sync" the forbidden in-band pairs become a rule of
the scheduler (common TX / RX cadence of all faces per band, step 4.3); this module checks nothing at run time.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from . import radar_model as rm
from .params import P

INTERFACE_MODES = ("sync",)          # 'cancel' (active cancellation) to revisit with the scheduler (4.3)
INTERFACE_MODE: str = P.get("isolation.interface_mode")
if INTERFACE_MODE not in INTERFACE_MODES:
    raise ValueError(f"{P.path}: isolation.interface_mode '{INTERFACE_MODE}' is not implemented (one of "
                     f"{INTERFACE_MODES}; active cancellation is to be revisited with the scheduler, step 4.3)")
COUPLING_DB: Dict[str, float] = {"adjacent": P.get("isolation.coupling_adjacent_db"),
                                 "opposite": P.get("isolation.coupling_opposite_db")}
INTRA_S_TO_KU_DB: float = P.get("isolation.intra_trsa_s_to_ku_db")
INTRA_KU_TO_S_DB: float = P.get("isolation.intra_trsa_ku_to_s_db")
DESENS_MARGIN_DB: float = P.get("isolation.desens_margin_db")
LNA_BLOCK_BACKOFF_DB: float = P.get("isolation.lna_block_backoff_db")
HARMONIC_DBC: float = P.get("band.S.tx.harmonic_in_ku_dbc")
HARMONIC_REJ_DB: float = P.get("band.S.tx.harmonic_filter_rej_in_ku_db")
RELATIONS = ("same", "adjacent", "opposite")


def harmonic_orders() -> Tuple[int, ...]:
    """Orders n >= 2 whose S harmonic band n x [S usable] overlaps the Ku usable band (5 today)."""
    s, k = rm.BAND_S, rm.BAND_KU
    return tuple(n for n in range(2, int(math.ceil(k.usable_hi_ghz / s.usable_lo_ghz)) + 1)
                 if n * s.usable_lo_ghz <= k.usable_hi_ghz and n * s.usable_hi_ghz >= k.usable_lo_ghz)


HARMONIC_ORDERS: Tuple[int, ...] = harmonic_orders()


def _dbm(w: float) -> float:
    return 10.0 * math.log10(w * 1e3)


def pair(tx: str, rx: str, relation: str) -> dict:
    """Leak and checks of one pair (bands "S" / "Ku", relation of the faces)."""
    btx, brx = rm.BANDS[tx], rm.BANDS[rx]
    out: Dict[str, object] = {"tx": tx, "rx": rx, "relation": relation}
    if tx == rx and relation == "same":
        return {**out, "verdict": "eclipsing", "note": "same band, same face: the receiver is deaf during its own "
                                                       "pulses (eclipsing, step 2.1)"}
    if relation == "same":
        if tx == "S":
            leak = _dbm(btx.p_pk_trm_w) + INTRA_S_TO_KU_DB
        else:
            n_ku = (rm.KU_TRM_PER_TRSA_SIDE // rm.S_TRM_PER_TRSA_SIDE) ** 2      # Ku TRMs of one S cell
            leak = _dbm(btx.p_pk_trm_w * n_ku) + INTRA_KU_TO_S_DB
    else:
        leak = _dbm(btx.p_pk_total_w) - COUPLING_DB[relation]
    in_band = tx == rx
    lna_level = leak - (0.0 if in_band else brx.preselect_rej_db)
    adc_level = leak - (0.0 if in_band else brx.preselect_rej_db + brx.post_rej_db)
    desens_limit = brx.noise_ant_dbm - DESENS_MARGIN_DB
    out.update({"leak_dbm": leak, "lna_margin_db": brx.lna_p1db_ant_dbm - LNA_BLOCK_BACKOFF_DB - lna_level,
                "adc_margin_db": brx.clip_ant_dbm - adc_level})
    if in_band:
        out["desens_margin_db"] = desens_limit - leak
    harm: Optional[dict] = None
    if tx == "S" and rx == "Ku" and HARMONIC_ORDERS:
        level = leak + HARMONIC_DBC - HARMONIC_REJ_DB                # in the Ku band: not rejected by the Ku filters
        harm = {"orders": list(HARMONIC_ORDERS), "level_dbm": level, "desens_margin_db": desens_limit - level,
                "adc_margin_db": brx.clip_ant_dbm - level}
    out["harmonic"] = harm
    margins = [out["lna_margin_db"], out["adc_margin_db"]] + ([harm["adc_margin_db"]] if harm else [])
    if in_band:
        verdict = "allowed" if min(margins + [out["desens_margin_db"]]) >= 0.0 else "forbidden"
    elif min(margins) < 0.0:
        verdict = "forbidden"
    else:
        verdict = "frequency_rule" if harm and harm["desens_margin_db"] < 0.0 else "allowed"
    out["verdict"] = verdict
    return out


def matrix() -> List[dict]:
    """Every pair, in the order S -> S, S -> Ku, Ku -> S, Ku -> Ku x same / adjacent / opposite."""
    return [pair(tx, rx, rel) for tx in ("S", "Ku") for rx in ("S", "Ku") for rel in RELATIONS]


def harmonic_bands_ghz(f_ghz: float, sig_bw_mhz: float) -> List[Tuple[float, float]]:
    """Spectrum of the S harmonics that fall in the Ku band, for an S signal f +/- B/2."""
    half = sig_bw_mhz * 1e-3 / 2.0
    return [(n * (f_ghz - half), n * (f_ghz + half)) for n in HARMONIC_ORDERS]


def harmonic_conflict(f_s_ghz: float, bw_s_mhz: float, f_ku_ghz: float, bw_ku_mhz: float) -> bool:
    """True if an S harmonic of the signal f_s +/- B_s/2 overlaps the Ku channel f_ku +/- B_ku/2."""
    lo, hi = f_ku_ghz - bw_ku_mhz * 1e-3 / 2.0, f_ku_ghz + bw_ku_mhz * 1e-3 / 2.0
    return any(a <= hi and b >= lo for a, b in harmonic_bands_ghz(f_s_ghz, bw_s_mhz))


def same_face_s_to_ku_verdict() -> str:
    return pair("S", "Ku", "same")["verdict"]


def scheduler_rules() -> List[str]:
    """Rules for the scheduler (step 4.3) that follow from the matrix and interface_mode."""
    rules = []
    for b in ("S", "Ku"):
        bad = [p["relation"] for p in (pair(b, b, r) for r in ("adjacent", "opposite")) if p["verdict"] == "forbidden"]
        if bad and INTERFACE_MODE == "sync":
            rules.append(f"{b} band: common TX / RX cadence of all faces (interface_mode sync): no face receives {b} "
                         f"while another transmits {b} ({' and '.join(bad)} faces fail the isolation)")
    for tx, rx in (("S", "Ku"), ("Ku", "S")):
        for r in RELATIONS:
            p = pair(tx, rx, r)
            if p["verdict"] == "forbidden":
                rules.append(f"{tx} transmission forbids {rx} reception on the {r} face(s)")
            elif p["verdict"] == "frequency_rule":
                rules.append(f"{tx} transmission and {rx} reception on the {r} face(s) only if the S harmonic "
                             f"(order {', '.join(map(str, HARMONIC_ORDERS))}) misses the {rx} channel")
    return rules
