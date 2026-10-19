#!/usr/bin/env python3
"""
REFERENCE PROTOTYPE (not production) -- per-scan-angle-bin waveform selection with a joint energy/time objective.
Spec: SPEC_optimiseur_v2.md. Run from V1/: python -m AUDIT.proto_per_bin_reference mission.json [sector_id ...]
Representative face only (face with most positions); production code must do every face.
"""
import json, math, sys
import numpy as np
from core import optimizer as mo

rm, C = mo.rm, mo.C
BIN_DEG = 10.0                      # production: 5.0 (with caching of solve())
PPK = rm.BAND_S.p_pk_total_w
P_AVG = 2e6 * rm.CHAIN_EFF * rm.PAE


def candidates(sec):
    r = sec["range_km"]; pri_u = 2 * r * 1e3 / C
    res = sec.get("res_max_m", mo.RES_MAX_M_DEFAULT)
    bws = [b for b in mo.SIGBW_CAND if C / (2 * b * 1e6) <= res + 1e-9] or [75.0]
    for integ in ("COHERENT", "NONCOHERENT"):
        for b in mo.B_CAND:
            for bw in bws:
                for n in mo.N_CAND:
                    for tau in mo.TAU_CAND:
                        pri = max(tau, 1e-3) if n == 1 else max(tau, pri_u)
                        try:
                            yield mo.make_wf("c", tau, n, pri, bw, b, integ)
                        except ValueError:
                            pass


def design_sector(sph, sec, f_list=(2.7, 2.8), alphas=np.linspace(0, 1, 21)):
    tgt = rm.Target(sec["id"], sec["rcs_m2"], int(sec.get("swerling", 1)), sec["v_close_max_mps"])
    R, rr = sec["range_km"], sec["rr_target_s"]
    mask = sph.mask(sec); cl = list(candidates(sec)); best = None
    for f in f_list:                                   # f_list must keep scan limit >= 75 deg (f <= 2.85)
        pos, unscanned = mo.positions_table(sph, mask, f)
        face = max(range(4), key=lambda k: pos[k].sum()); cnt = pos[face]
        nb = int(math.ceil(len(cnt) / BIN_DEG))
        P = [cnt[int(i * BIN_DEG):int((i + 1) * BIN_DEG)].sum() for i in range(nb)]
        table = []
        for i in range(nb):
            if P[i] <= 0:
                table.append(None); continue
            th = min((i + 1) * BIN_DEG, rm.SECTOR_RADIUS_DEG)        # worst angle of the bin
            rows = []
            for w in cl:
                try:
                    r = rm.solve(w, f, th, tgt, 2.0).range_km
                except ValueError:
                    continue
                s = min(1.0, (R / r) ** 4)                           # power scale that just reaches R
                rows.append((r >= R, PPK * w.n * w.tau_s / w.b_tx * s,   # J per position
                             w.n * w.tau_s / w.b_tx, w, r))            # s of TX time per position
            table.append(rows)
        for a in alphas:                                             # scalarisation a*E + (1-a)*T
            fe = ft = short = 0.0; ch = []
            for i, rows in enumerate(table):
                if rows is None:
                    ch.append(None); continue
                ok = [x for x in rows if x[0]]
                if ok:
                    pick = min(ok, key=lambda x: a * x[1] / P_AVG + (1 - a) * x[2])
                else:
                    pick = max(rows, key=lambda x: x[4]); short += P[i]
                ch.append(pick); fe += P[i] * pick[1] / (P_AVG * rr); ft += P[i] * pick[2] / rr
            key = (short / sum(P), max(fe, ft))
            if best is None or key < best[0]:
                best = (key, f, a, fe, ft, ch, P, unscanned)
    return best


if __name__ == "__main__":
    m = json.load(open(sys.argv[1])); ids = sys.argv[2:] or [s["id"] for s in m["sectors"]]
    g = m.get("global", {}); sph = mo.Sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    for sec in m["sectors"]:
        if sec["id"] not in ids:
            continue
        key, f, a, fe, ft, ch, P, uns = design_sector(sph, sec)
        print(f"{sec['id']}: f={f} alpha={a:.2f} E={100*fe:.1f}% T={100*ft:.1f}% short={100*key[0]:.2f}% unscanned={100*uns:.1f}%")
        for i, c in enumerate(ch):
            if c:
                w = c[3]
                print(f"  theta {i*BIN_DEG:.0f}-{(i+1)*BIN_DEG:.0f}: P={P[i]:.0f} {w.integration[:4]} N{w.n} tau{w.tau_s*1e6:.0f}us "
                      f"b{w.b_tx} B{w.sig_bw_mhz:.0f} R={c[4]:.0f}km s={min(1,(sec['range_km']/c[4])**4):.2f}")
