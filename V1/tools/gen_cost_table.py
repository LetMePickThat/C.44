"""Builds core/cost_table.json, the table behind the fast cost estimate (core/cost_estimate.py).
Run from anywhere: python tools/gen_cost_table.py"""
import os, sys, json, itertools
from multiprocessing import Pool
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_OUT = os.path.join(_ROOT, "core", "cost_table.json")
sys.path.insert(0, _ROOT)
from core import optimizer as mo  # noqa: E402
from core import radar_model as rm  # noqa: E402
from core import params  # noqa: E402
from core.params import P  # noqa: E402
PPK = rm.BAND_S.p_pk_total_w; F = P.get("tool.cost_table.freq_ghz"); ELEC = P.get("power.face_elec_mw")
# Grid in radar_system.toml [tool.cost_table]: dense enough that interpolation between rows stays within the
# E ~ R^4 law's comfort zone (the old 7000 -> 10000 gap made an 8000 km request look unreachable at every scan
# angle); 70 deg added because linear interpolation under-caught the scan-loss curvature (power 3) near the limit.
RS = P.get("tool.cost_table.range_km")
RCS = P.get("tool.cost_table.rcs_m2")
VS = P.get("tool.cost_table.v_mps")
TH = P.get("tool.cost_table.theta_deg")
def one(args):
    """Cheapest waveform of the optimiser's search space (default 2 m resolution -> 75 MHz) reaching R."""
    R, rcs, v, th = args
    tgt = rm.Target("x", rcs, 1, v)
    best = None; rmax = 0; e_rmax = None
    for w in mo.candidate_waveforms({"range_km": R}):
        try:
            r = rm.solve(w, F, th, tgt, ELEC).range_km
        except ValueError:
            continue
        n, tau, b = w.n, w.tau_s, w.b_tx
        if r > rmax:
            rmax, e_rmax = r, PPK * n * tau / b      # longest-reach waveform, at full power
        if r < R: continue
        e = PPK * n * tau / b * (R / r) ** 4
        if best is None or e < best[0]:
            best = (e, w.integration[0], n, tau * 1e6, w.pri_s * 1e3, b)
    return [args, best, rmax, e_rmax]
if __name__ == "__main__":
    jobs = list(itertools.product(RS, RCS, VS, TH))
    with Pool() as p: out = p.map(one, jobs, chunksize=8)
    tab, reach = {}, {}
    for (R, rcs, v, th), best, rmax, e_rmax in out:
        key = f"{R}|{rcs}|{v}|{th}"
        tab[key] = None if best is None else [best[0], best[1], best[2], round(best[3], 1), round(best[4], 2), best[5]]
        reach[key] = [round(rmax, 1), e_rmax]     # max range of the cell (full power) and that waveform's energy (J)
    json.dump({"R": RS, "RCS": RCS, "V": VS, "TH": TH, "ppk": PPK, "pavg": rm.face_avg_rf_w(ELEC), "t": tab,
               "reach": reach, "params_fingerprint": params.fingerprint(P), "rx_sll_db": rm.RX_SLL_DEFAULT_DB},
              open(_OUT, "w"))
    print(len(tab), sum(1 for x in tab.values() if x is None), "infeasible")
