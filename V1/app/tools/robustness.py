"""Robustness test of the chain search > confirmation > track (user, 28 October 2026): the plan is sized on the class
means, as the Pmiss does; the true targets depart from them (RCS spread and aspect, closing speed, rotation,
fluctuation case, manoeuvre: core/montecarlo.draw_truth). One evaluation mission, never used to tune the model: one
search sector per class of the library, drawn once from the ranges of [tool.robustness] with its own seed and frozen
in tools/robustness_mission.json. The plan is cached with the fingerprint of the code (local, tools/.robustness_cache/);
the test lasts at most tool.robustness.budget_s, the sizes being cut from the stage times of the last run.
On demand only: python tools/robustness.py   (report in tools/.robustness_last.md and .json, local)"""
import glob
import hashlib
import json
import math
import os
import pickle
import sys
import time
from typing import List

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from core import montecarlo as mcm  # noqa: E402
from core import optimizer as mo  # noqa: E402
from core import radar_model as rm  # noqa: E402
from core.params import P  # noqa: E402

_T = "tool.robustness."
MISSION_PATH = os.path.join(_ROOT, "tools", "robustness_mission.json")
CACHE_DIR = os.path.join(_ROOT, "tools", ".robustness_cache")
REPORT = os.path.join(_ROOT, "tools", ".robustness_last")


def load_cfg() -> dict:
    """[tool.robustness], every entry read (the test bench checks it)."""
    c = {k: P.get(_T + k) for k in ("seed", "budget_s", "n_targets", "tracks_per_bracket", "n_rep", "rcs_trunc_sigmas",
                                    "frontal_cone_deg", "rot_factor", "fluct_alt_p", "classes", "v_close_min_frac",
                                    "box_az_deg", "box_el_deg", "el_span_deg", "angle_zone_deg", "range_acc_m",
                                    "vr_acc_mps", "tracks_n")}
    for k in ("range_km", "rr_s", "track_rr_s"):
        c[k] = P.section(_T + k)
    return c


CFG = load_cfg()


def make_mission(cfg: dict) -> dict:
    """The evaluation mission: one box sector per class, placed and sized by draws within the configured ranges."""
    rng = np.random.default_rng(int(cfg["seed"]))
    u = lambda lo_hi: float(rng.uniform(*lo_hi))                       # noqa: E731
    secs = []
    for cls in cfg["classes"]:
        tgt = rm.TARGETS[cls]
        w, h = u(cfg["box_az_deg"]), u(cfg["box_el_deg"])
        c = u([0.0, 360.0])
        e_lo, e_hi = cfg["el_span_deg"]
        ec = u([e_lo + h / 2.0, e_hi - h / 2.0])
        r_km = round(u(cfg["range_km"][cls]), -1)
        sec = {"id": f"R_{cls}", "shape": "box", "target_type": cls,
               "phi_min_deg": round((c - w / 2.0) % 360.0, 1), "phi_max_deg": round((c + w / 2.0) % 360.0, 1),
               "eps_min_deg": round(ec - h / 2.0, 1), "eps_max_deg": round(ec + h / 2.0, 1),
               "range_km": r_km, "rr_target_s": round(u(cfg["rr_s"][cls]), 1),
               "v_close_min_mps": round(cfg["v_close_min_frac"] * tgt.v_close_mps, 1),
               "v_close_max_mps": tgt.v_close_mps, "confirm_mode": "auto", "new_targets_per_rr": 1.0, "tracking": []}
        if cls == "GENERIC":
            sec["rcs_m2"] = tgt.rcs_m2
        if cls in cfg["track_rr_s"]:                                   # one bracket 0-R per tracked class (decision 3)
            sec["tracking"].append({"r_min_km": 0.0, "r_max_km": r_km,
                                    "n_targets": int(rng.integers(cfg["tracks_n"][0], cfg["tracks_n"][1] + 1)),
                                    "rr_s": round(u(cfg["track_rr_s"][cls]), 2),
                                    "angle_zone_deg": round(u(cfg["angle_zone_deg"]), 2),
                                    "range_acc_m": round(u(cfg["range_acc_m"]), 1),
                                    "vr_acc_mps": round(u(cfg["vr_acc_mps"]), 1)})
        secs.append(sec)
    return {"global": {"eps_min_deg": -40.0, "eps_max_deg": 90.0, "face_elec_mw": 2.0}, "sectors": secs}


def fingerprint() -> str:
    """Hash of everything the plan depends on: the code, the parameter values (not the simulator's [sim] nor this
    test's own settings, which do not change the plan), the cost table, the mission."""
    h = hashlib.sha256()
    files = sorted(glob.glob(os.path.join(_ROOT, "core", "*.py"))) + [
        os.path.join(_ROOT, "core", "cost_table.json"), MISSION_PATH]
    for f in files:
        with open(f, "rb") as fh:
            h.update(fh.read().replace(bytes([13, 10]), bytes([10])))
    vals = {p.path: p.value for p in P if not p.path.startswith(("sim.", _T))}
    h.update(json.dumps(vals, sort_keys=True, default=str).encode())
    h.update(f"{sys.version}|{np.__version__}|{os.environ.get('RADAR_PARAMS', '')}".encode())
    return h.hexdigest()[:20]


def plan(mission: dict) -> tuple:
    """(run kept with its plans, seconds spent, from cache): the optimiser's run, cached with the fingerprint."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, "plan.pkl")
    fp = fingerprint()
    if os.path.exists(path):
        try:
            with open(path, "rb") as fh:
                got = pickle.load(fh)
            if got.get("fingerprint") == fp:
                return got["out"], 0.0, True
        except (OSError, pickle.UnpicklingError, EOFError):
            pass
    t = time.perf_counter()
    out = mo.run(mission, keep_plans=True)
    dt = time.perf_counter() - t
    with open(path, "wb") as fh:
        pickle.dump({"fingerprint": fp, "out": out}, fh)
    return out, dt, False


def sizes(cfg: dict, plan_s: float) -> tuple:
    """(targets, tracks, timelines, note): the configured sizes, cut in proportion when the stage times of the last run
    say the test would last more than the budget."""
    n_t, n_k, n_r = int(cfg["n_targets"]), int(cfg["tracks_per_bracket"]), int(cfg["n_rep"])
    last = os.path.join(CACHE_DIR, "stages.json")
    if not os.path.exists(last):
        return n_t, n_k, n_r, ""
    with open(last, encoding="utf-8") as fh:
        st = json.load(fh)
    left = float(cfg["budget_s"]) - plan_s - 30.0
    per = {k: st[k] / max(st[f"n_{k}"], 1) for k in ("tracks", "search_confirmation", "timelines")}
    est = per["tracks"] * n_k + per["search_confirmation"] * n_t + per["timelines"] * n_r
    if est <= left or est <= 0:
        return n_t, n_k, n_r, ""
    f = max(left, 60.0) / est
    k2 = max(100, int(n_k * min(1.0, f * 0.8)))                       # the tracks first (the largest share)
    rest = left - per["tracks"] * k2
    t2 = n_t if per["search_confirmation"] * n_t + per["timelines"] * n_r <= rest else \
        max(500, int(n_t * max(rest, 30.0) / (per["search_confirmation"] * n_t + per["timelines"] * n_r)))
    return t2, k2, n_r, f"sizes cut to hold the {cfg['budget_s']:.0f} s budget: {t2} targets, {k2} tracks per bracket"


def _bins(x: np.ndarray, y: np.ndarray, edges: List = None, q: int = 4) -> List[tuple]:
    """(label, count, share of failures) of y (failures, bool) in quantile bins of x (or given edges)."""
    e = np.unique(np.quantile(x, np.linspace(0, 1, q + 1))) if edges is None else np.asarray(edges, dtype=float)
    out = []
    for a, b in zip(e[:-1], e[1:]):
        m = (x >= a) & ((x <= b) if b == e[-1] else (x < b))
        if m.sum():
            out.append((f"{a:.3g}..{b:.3g}", int(m.sum()), float(y[m].mean())))
    return out


def sensitivity(tr: dict, fail: np.ndarray) -> dict:
    """Failure share per bin of each drawn parameter (the parameters that make the chain fail stand out)."""
    out = {"RCS offset S (dB)": _bins(np.asarray(tr["off_s_db"]), fail),
           "aspect from nose (deg)": _bins(np.asarray(tr["aspect_deg"]), fail),
           "closing speed (m/s)": _bins(np.asarray(tr["v_mps"]), fail),
           "coherence-time factor": _bins(np.asarray(tr["t0_factor"]), fail)}
    sf = np.asarray(tr["shape_factor"])
    out["other Swerling case"] = [(lab, int(m.sum()), float(fail[m].mean())) for lab, m in
                                  (("class case", sf == 1.0), ("other case", sf != 1.0)) if m.sum()]
    return out


def report(mission: dict, mc: dict, meta: dict) -> List[str]:
    """Text report: per class the chain with true targets, the analytic value of the plan, the sensitivities."""
    cls = {s["id"]: s.get("target_type", "") for s in mission["sectors"]}
    L = [f"ROBUSTNESS TEST (true targets around the class means, seed {meta['seed']}): {mc['state']}",
         f"  total {meta['total_s']:.0f} s (plan {meta['plan_s']:.0f} s{' cached' if meta['cached'] else ''}; "
         + ", ".join(f"{k} {v:.0f} s" for k, v in mc["stage_s"].items() if k != "plan")
         + f"); budget {meta['budget_s']:.0f} s" + (" -- EXCEEDED" if meta["total_s"] > meta["budget_s"] else ""),
         f"  sizes: {meta['n_targets']} targets per sector, {meta['tracks']} tracks per bracket, {meta['n_rep']} "
         f"timelines per face" + (f" ({meta['note']})" if meta["note"] else ""), "",
         "  Search and confirmation (positions within range)",
         f"  {'class':<9}{'R km':>7}{'RR s':>6}{'Pd req':>8}{'plan':>7}{'true':>7}{'interval':>16}  "
         f"{'P confirm':>10}{'interval':>16}  state"]
    for r in mc["sectors"]:
        if r.get("status") != "OK":
            L.append(f"  {cls[r['id']]:<9} no plan")
            continue
        s = next(x for x in mission["sectors"] if x["id"] == r["id"])
        c = r.get("confirmation") or {}
        ci, cc = r["ci_in_range"], c.get("ci", [None, None])
        L.append(f"  {cls[r['id']]:<9}{s['range_km']:>7.0f}{s['rr_target_s']:>6.0f}{r['pd_required']:>8.2f}"
                 f"{(r['pd_analytic_in_range'] or 0):>7.3f}{(r['pd_mc_in_range'] or 0):>7.3f}"
                 f"{(f'[{ci[0]:.3f}, {ci[1]:.3f}]' if ci[0] is not None else '-'):>16}  "
                 f"{(format(c['p_mc'], '.4f') if c else '-'):>10}"
                 f"{(f'[{cc[0]:.4f}, {cc[1]:.4f}]' if c else '-'):>16}  {r['state']}"
                 + (f" / conf {c.get('state')}" if c else ""))
    if mc.get("tracks"):
        L += ["", "  Tracks (true targets; steady: loss per update, RMS against the requirements; initiation)"]
        for t in mc["tracks"]:
            if t.get("status") != "OK":
                L.append(f"  {t['id']:<12} no update look")
                continue
            sid = t["id"].split("/")[0]
            rq = t["required"]
            L.append(f"  {cls.get(sid, sid):<9}{t['id'].split('/')[1]:>2}  loss {t['p_loss_mc']:.2e} "
                     f"[{t['ci'][0]:.1e}, {t['ci'][1]:.1e}] (analytic {t['p_loss_analytic']:.1e}, max {t['p_loss_max']:g})"
                     f"  RMS " + ", ".join(f"{k} {v / rq[k]:.2f}x" for k, v in t["rms"].items() if rq.get(k))
                     + f" of required; init {100 * t['init_success']:.2f} % in {t['init_time_mean_s'] or 0:.2f} s  "
                     f"{t['state']}")
    if mc.get("chain"):
        L += ["", "  Chain first detection > confirmation > established track (requirement "
                  f"{mc['chain'][0]['required']:g})"]
        for c in mc["chain"]:
            L.append(f"  {cls.get(c['id'], c['id']):<9} failure {c['failure_mc']:.2e} [{c['ci'][0]:.1e}, "
                     f"{c['ci'][1]:.1e}]  {c['state']}")
    L += ["", "  Sensitivity: share of the true targets in range NOT confirmed (missed or unconfirmed), per bin"]
    for r in mc["sectors"]:
        tr = r.get("truth")
        if not tr:
            continue
        fail = ~np.asarray(tr.get("confirmed", tr["detected"]), dtype=bool)
        L.append(f"  {cls[r['id']]} ({len(fail)} targets, {100 * fail.mean():.1f} % not confirmed)")
        for name, rows in sensitivity(tr, fail).items():
            L.append(f"    {name:<24}" + "  ".join(f"{lab}: {100 * p:.1f} % ({n})" for lab, n, p in rows))
    tracks_tr = [t for t in mc.get("tracks", []) if t.get("truth")]
    if tracks_tr:
        L += ["", "  Sensitivity of the steady tracks: loss per update per bin"]
        for t in tracks_tr:
            tr = t["truth"]
            los, up = np.asarray(tr["losses"]), np.maximum(np.asarray(tr["updates"]), 1.0)
            L.append(f"  {t['id']}")
            for name, key in (("Ku RCS offset (dB)", "off_ku_db"), ("manoeuvre scale", "accel_scale"),
                              ("coherence-time factor", "t0_factor")):
                x = np.asarray(tr[key])
                e = np.unique(np.quantile(x, np.linspace(0, 1, 5)))
                cells = []
                for a, b in zip(e[:-1], e[1:]):
                    m = (x >= a) & ((x <= b) if b == e[-1] else (x < b))
                    if m.sum():
                        cells.append(f"{a:.3g}..{b:.3g}: {los[m].sum() / up[m].sum():.1e}")
                L.append(f"    {name:<24}" + "  ".join(cells))
    return L


def main() -> int:
    t_start = time.perf_counter()
    cfg = CFG
    if not os.path.exists(MISSION_PATH):
        with open(MISSION_PATH, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(make_mission(cfg), fh, indent=1)
    mission = mo.load_mission(MISSION_PATH)
    out, plan_s, cached = plan(mission)
    n_t, n_k, n_r, note = sizes(cfg, plan_s if not cached else 0.0)
    mcm.N_MAX, mcm.N_MIN, mcm.TRACKS_PER_BRACKET, mcm.N_REP = n_t, min(mcm.N_MIN, n_t), n_k, n_r
    mcm.CI_TOL = 0.0                                     # the configured sizes, no early stop
    truth_cfg = {k: cfg[k] for k in ("rcs_trunc_sigmas", "frontal_cone_deg", "rot_factor", "fluct_alt_p")}
    mc = mcm.run(mission, out, seed=int(cfg["seed"]), truth_cfg=truth_cfg,
                 progress_cb=lambda d, n, s: print(f"  {d}/{n} {s}", flush=True))
    st = dict(mc["stage_s"], n_tracks=n_k, n_search_confirmation=n_t, n_timelines=n_r)
    with open(os.path.join(CACHE_DIR, "stages.json"), "w", encoding="utf-8") as fh:
        json.dump(st, fh)
    meta = {"seed": cfg["seed"], "total_s": time.perf_counter() - t_start, "plan_s": plan_s, "cached": cached,
            "budget_s": float(cfg["budget_s"]), "n_targets": n_t, "tracks": n_k, "n_rep": n_r, "note": note}
    lines = report(mission, mc, meta)
    print("\n".join(lines))
    with open(REPORT + ".md", "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    with open(REPORT + ".json", "w", encoding="utf-8") as fh:
        json.dump(mcm._clean({"meta": meta, "mc": mc}), fh, indent=1, default=float)
    return 0


if __name__ == "__main__":
    sys.exit(main())
