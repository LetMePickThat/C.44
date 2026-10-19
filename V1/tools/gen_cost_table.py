"""Builds core/cost_table.json, the table behind the fast cost estimate (core/cost_estimate.py).
Run from anywhere: python tools/gen_cost_table.py"""
import os, sys, json, itertools
from multiprocessing import Pool
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_OUT = os.path.join(_ROOT, "core", "cost_table.json")
sys.path.insert(0, _ROOT)
from core import radar_model as rm  # noqa: E402
C = rm.C_MPS; PPK = rm.BAND_S.p_pk_total_w; F = 2.8
# Dense enough that interpolation between rows stays within the E ~ R^4 law's comfort zone (the old 7000 -> 10000 gap
# made an 8000 km request look unreachable at every scan angle).
RS = [100, 150, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 12000, 15000, 20000]
RCS = [0.1, 1, 10, 100, 1000, 10000]
VS = [1000, 5000, 12000, 25000]
TH = [0, 20, 40, 55, 65, 75]
TAU = (20e-6, 50e-6, 100e-6, 200e-6, 400e-6, 600e-6, 800e-6, 937.5e-6)
NN = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32)
def one(args):
    R, rcs, v, th = args
    tgt = rm.Target("x", rcs, 1, v)
    pri_u = 2 * R * 1e3 / C
    best = None; rmax = 0; e_rmax = None
    for integ in ("COHERENT", "NONCOHERENT"):
        for b in (1, 2, 4, 8):
            for n in NN:
                for tau in TAU:
                    pri = max(tau, 1e-3) if n == 1 else max(tau, pri_u)
                    try:
                        w = rm.Waveform("c", "SEARCH", "S", tau, n, pri, 75.0, b, integ)
                        r = rm.solve(w, F, th, tgt, 2.0).range_km
                    except ValueError:
                        continue
                    if r > rmax:
                        rmax, e_rmax = r, PPK * n * tau / b      # longest-reach waveform, at full power
                    if r < R: continue
                    e = PPK * n * tau / b * (R / r) ** 4
                    if best is None or e < best[0]:
                        best = (e, integ[0], n, tau * 1e6, pri * 1e3, b)
    return [args, best, rmax, e_rmax]
if __name__ == "__main__":
    jobs = list(itertools.product(RS, RCS, VS, TH))
    with Pool() as p: out = p.map(one, jobs, chunksize=8)
    tab, reach = {}, {}
    for (R, rcs, v, th), best, rmax, e_rmax in out:
        key = f"{R}|{rcs}|{v}|{th}"
        tab[key] = None if best is None else [best[0], best[1], best[2], round(best[3], 1), round(best[4], 2), best[5]]
        reach[key] = [round(rmax, 1), e_rmax]     # max range of the cell (full power) and that waveform's energy (J)
    json.dump({"R": RS, "RCS": RCS, "V": VS, "TH": TH, "ppk": PPK, "pavg": 2e6 * rm.CHAIN_EFF * rm.PAE, "t": tab,
               "reach": reach}, open(_OUT, "w"))
    print(len(tab), sum(1 for x in tab.values() if x is None), "infeasible")
