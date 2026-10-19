"""Builds core/cost_table.json, the table behind the fast cost estimate (core/cost_estimate.py).
Run from anywhere: python tools/gen_cost_table.py"""
import os, sys, json, itertools
from multiprocessing import Pool
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_OUT = os.path.join(_ROOT, "core", "cost_table.json")
sys.path.insert(0, _ROOT)
from core import optimizer as mo  # noqa: E402
from core import radar_model as rm  # noqa: E402
PPK = rm.BAND_S.p_pk_total_w; F = 2.8
# Dense enough that interpolation between rows stays within the E ~ R^4 law's comfort zone (the old 7000 -> 10000 gap
# made an 8000 km request look unreachable at every scan angle).
RS = [100, 150, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 12000, 15000, 20000]
RCS = [0.1, 1, 10, 100, 1000, 10000]
VS = [1000, 5000, 12000, 25000]
TH = [0, 20, 40, 55, 65, 70, 75]   # 70 added: linear interpolation under-caught the scan-loss curvature (power 3) near the limit
def one(args):
    """Cheapest waveform of the optimiser's search space (default 2 m resolution -> 75 MHz) reaching R."""
    R, rcs, v, th = args
    tgt = rm.Target("x", rcs, 1, v)
    best = None; rmax = 0; e_rmax = None
    for w in mo.candidate_waveforms({"range_km": R}):
        try:
            r = rm.solve(w, F, th, tgt, 2.0).range_km
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
    json.dump({"R": RS, "RCS": RCS, "V": VS, "TH": TH, "ppk": PPK, "pavg": rm.face_avg_rf_w(2.0), "t": tab,
               "reach": reach}, open(_OUT, "w"))
    print(len(tab), sum(1 for x in tab.values() if x is None), "infeasible")
