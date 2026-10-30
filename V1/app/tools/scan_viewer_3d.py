#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time 3D view of the search pointings of a mission around the low-poly ship model (step 4.3 bis).

    python tools/scan_viewer_3d.py                          scenario tools/scenarios/scan3d_shell_lp.json
    python tools/scan_viewer_3d.py mission_veille500        a test mission (tests/missions), or any mission path
    python tools/scan_viewer_3d.py --view full              start in the 1:1 view of the whole scene (real time)
    python tools/scan_viewer_3d.py --snapshot out.png       render one instant off screen and exit (check)

The whole scene is at true scale, in km: the ship model, each transmit beam drawn as its real beam out to the sector's
range (a 500 km search -> 500 km long): from the emitting part of the plate (the envelope disc, or a half / quadrant
of it for a partitioned aperture), collimated over its near field D_eff^2 / lambda, then opening along its -3 dB
contour, built in the face's (u, v) so that its widening with the scan angle is exact; the dwells follow the TX
multibeam mode of their waveform (single, superposition, time multiplex, partition: see pointing_events), and the sky bodies of the mission with a known radius (Moon,
Earth) as spheres at the distance their angular radius implies (radius / sin(angular radius)), in their direction.
Two cameras: 'ship' (close to the hull) and 'full' (1:1, the whole scene); a scale bar follows the zoom.
Each watch volume is outlined in its colour at its range: its boundary on the sphere of radius range_km (traced on a
0.5 deg raster of its mask, so within 0.25 deg), a faint surface, and the radial edges from the ship to its corners.
The play lasts --duration (default: the longest revisit), each volume looping on its own revisit (a 20 s volume
loops 6 times in 120 s). The pointings are those of core/scanorder.py: the hexagonal lattice of each volume's plan in the face's (u, v),
shifted frame to frame, in bit-reversed Morton order; pointing i of a frame is drawn at i x RR / N (the release times
of the scheduler's synthetic timeline). Three timings (key m, option --timing): 'trace' draws each beam for a share of
its volume's revisit (--fade, a visual trace, NOT a physical duration); 'physical' draws it for its real duration, the
transmission of its dwell plus the echo window 2 R / c, and at least one image; 'slow' is the physical timing in slow
motion (--slow). The mission result is cached in tools/.viewer_cache
(keyed by the code, the parameter file and the mission), else computed (a few minutes). Test values only.
Keys: space = pause, Home = restart from the beginning, v = ship / 1:1 view, m = timing (trace / physical / slow),
q = quit.
"""
import argparse
import glob
import hashlib
import json
import math
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import pyvista as pv  # noqa: E402
from scipy.constants import c as C_LIGHT  # noqa: E402
import vtk  # noqa: E402

from core import environment as env  # noqa: E402
from core import frames  # noqa: E402
from core import geometry as geo  # noqa: E402
from core import optimizer as mo  # noqa: E402
from core import radar_model as rm  # noqa: E402
from core import scanorder as so  # noqa: E402

PALETTE = ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8", "#fff176", "#4db6ac", "#f06292", "#aed581", "#90a4ae"]
KM = 1e-3                                                   # scene unit: km (the model is in metres)
BODY_RADIUS_KM = {"moon": geo.MOON_RADIUS_KM, "earth": 6371.0}
CACHE_DIR = os.path.join(ROOT, "tools", ".viewer_cache")
DEFAULT_MISSION = os.path.join(ROOT, "tools", "scenarios", "scan3d_multi.json")
SHIP_CAMERA = [(0.23, -0.16, 0.04), (0.0, 0.0, -0.025), (0.0, 0.0, 1.0)]
TIMINGS = ("trace", "physical", "slow")
CONE_SIDES = 12                                             # facets of a beam frustum
BEAM_OPACITY = 0.25


def mission_path(arg: str) -> str:
    for p in (arg, os.path.join(ROOT, "tests", "missions", f"{arg}.json"),
              os.path.join(ROOT, "tools", "scenarios", f"{arg}.json")):
        if os.path.isfile(p):
            return os.path.abspath(p)
    raise SystemExit(f"mission not found: {arg}")


def _fingerprint(path: str) -> str:
    h = hashlib.sha256()
    for f in (sorted(glob.glob(os.path.join(ROOT, "core", "*.py"))) + [os.path.join(ROOT, "radar_system.toml"),
              os.path.join(ROOT, "core", "cost_table.json"), path]):
        with open(f, "rb") as fh:
            h.update(fh.read().replace(b"\r\n", b"\n"))
    return h.hexdigest()[:20]


def _plain(o):
    return o.tolist() if hasattr(o, "tolist") else str(o)


def mission_result(path: str) -> dict:
    """optimizer.run() of the mission, from the viewer cache when the code, the parameters and the mission match."""
    fp = _fingerprint(path)
    cache = os.path.join(CACHE_DIR, os.path.splitext(os.path.basename(path))[0] + ".json")
    if os.path.exists(cache):
        with open(cache, encoding="utf-8") as fh:
            d = json.load(fh)
        if d.get("fingerprint") == fp:
            return d["run"]
    print("optimising the mission (cached afterwards) ...")
    res = json.loads(json.dumps(mo.run(mo.load_mission(path)), default=_plain))
    os.makedirs(CACHE_DIR, exist_ok=True)
    with open(cache, "w", encoding="utf-8") as fh:
        json.dump({"fingerprint": fp, "run": res}, fh)
    return res


def direction(phi_deg: float, eps_deg: float) -> np.ndarray:
    p, e = math.radians(phi_deg), math.radians(eps_deg)
    return np.array([math.cos(e) * math.cos(p), math.cos(e) * math.sin(p), math.sin(e)])


def aperture_centre(k: int) -> np.ndarray:
    """Centre of the array plate of face k in the ship frame (km), just in front of it."""
    return (env.array_centre_m(k) + 0.05 * np.asarray(env.face_frame(k)[0])) * KM


_FRAMES = [tuple(np.asarray(x) for x in env.face_frame(k)) for k in range(4)]


BASE_KINDS = ("full", "half_y+", "half_y-", "half_x+", "half_x-", "quad++", "quad-+", "quad--", "quad+-")
PARTITION_BASES = {("h", 2): (1, 2), ("v", 2): (3, 4), (None, 2): (1, 2), ("h", 4): (5, 6, 7, 8),
                   ("v", 4): (5, 6, 7, 8), (None, 4): (5, 6, 7, 8)}
SUBAPERTURE_OF_BASE = {1: "half_h", 2: "half_h", 3: "half_v", 4: "half_v", 5: "quarter", 6: "quarter", 7: "quarter",
                       8: "quarter"}


def pointing_events(path: str, duration: float = None) -> dict:
    """Every transmit beam of each volume and face over duration s (default: the longest revisit), each volume looping
    on its own revisit. The pointings of a frame (core/scanorder.py) are grouped into dwells as the scan order serves
    them (the next positions_per_dwell pointings of the order), the dwells spread over the revisit; per dwell, by the TX
    mode of its scan bin's waveform (step 3.1, block C):
      single         one beam of the full aperture;
      superposition  the dwell's beams together, each of the full aperture;
      tdm            the dwell's beams one after the other within the dwell, each of the full aperture;
      partition      b_tx beams together, each from its sub-aperture (halves cut 'h' / 'v', or quadrants), toward the
                     mean of its share of the dwell's pointings, with the sub-aperture's -3 dB widths along u and v.
    A beam lasts (physical timing) its share of the dwell's transmission plus the echo window 2 R / c."""
    mission, _ = frames.resolve(mo.load_mission(path))
    res = mission_result(path)
    g = mission.get("global", {})
    sph = mo._sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    sky = g.get("sky")
    secs = {s["id"]: s for s in mission["sectors"]}
    ok = [r for r in res["sectors"] if r.get("status") == "OK"]
    if duration is None:
        duration = max(float(r["rr_target_s"]) for r in ok)
    cols = {k: [] for k in ("t", "face", "u", "v", "hu", "hv", "base", "len", "sidx", "on", "lnf", "sub")}
    ids, rrs = [], []
    for r in ok:
        sec = secs[r["id"]]
        si = len(ids)
        ids.append(r["id"])
        f = float(r["frequency"]["freq_ghz"])
        rr = float(r["rr_target_s"])
        rrs.append(rr)
        n_frames = max(1, math.ceil(duration / rr - 1e-9))
        reach = float(sec["range_km"])
        lam_m = rm.lambda_m(f)
        lam_d = lam_m / rm.ARRAY_ENVELOPE_M
        hw_full = rm._pattern(f).tx.hpbw * lam_d / 2.0                  # -3 dB TX half-width in (u, v)
        band = rm.band_for_frequency(f).name
        echo_s = 2.0 * reach * 1e3 / C_LIGHT
        lnf_full = rm.ARRAY_ENVELOPE_M ** 2 / lam_m * KM                 # near field of the full aperture (km)
        # scan bins: (theta lo, its waveform, transmit time of a dwell)
        bins = []
        for b in r.get("wf_by_bin", []):
            w = b["wf"]
            cpd = max(1.0, float(w.get("positions_per_dwell", 1.0)))
            t_dwell = w["n_pulses"] * w["tau_us"] * 1e-6 * float(b.get("pulse_scale", 1.0))
            bins.append((float(b["theta_lo_deg"]), w, cpd, t_dwell))
        bins.sort(key=lambda x: x[0])
        spacing = float(r.get("beam_spacing_hpbw", rm.BEAM_SPACING_HPBW))
        m = sph.mask(sec)
        _, keep = mo.sector_sky(sec, sph, m, f, "S", sky)
        lim = rm.band_for_frequency(f).scan_limit_deg(f)
        sel0 = m & sph.scan_zone & (sph.theta <= lim) & (np.asarray(keep) > 0.5)
        d = spacing * rm._pattern(f).tx.hpbw * lam_d
        for k in range(4):
            if str(k) not in r.get("faces", {}):
                continue
            uu, vv = so.face_uv(sph, k)
            for fr in range(n_frames):
                pts = so.lattice(sel0 & (sph.owner == k), uu, vv, d, fr)
                if len(pts) == 0:
                    continue
                order = so.order_bitrev_morton(pts, d)
                u, v = pts[order, 0], pts[order, 1]
                theta = np.degrees(np.arcsin(np.clip(np.hypot(u, v), 0.0, 1.0)))
                bin_of = np.zeros(len(order), dtype=int)
                for j, (lo, _, _, _) in enumerate(bins):
                    bin_of[theta >= lo - 1e-9] = j
                dwells, i = [], 0
                while i < len(order):
                    j = bin_of[i]
                    c = max(1, int(round(bins[j][2])))
                    dwells.append((j, np.arange(i, min(i + c, len(order)))))
                    i += c
                for jd, (j, grp) in enumerate(dwells):
                    t0 = fr * rr + jd * rr / len(dwells)
                    _, w, _, t_dwell = bins[j]
                    mode, b_tx = w.get("tx_mode", "single"), int(w.get("b_tx", 1))

                    def add(t, uc, vc, hu, hv, base, on, lnf, sub=0):
                        for key, val in (("t", t), ("face", k), ("u", uc), ("v", vc), ("hu", hu), ("hv", hv),
                                         ("base", base), ("len", reach), ("sidx", si), ("on", on), ("lnf", lnf),
                                         ("sub", sub)):
                            cols[key].append(val)

                    if mode == "partition" and b_tx in (2, 4):
                        kinds = PARTITION_BASES[(w.get("tx_cut"), b_tx)]
                        for q, part in enumerate(np.array_split(grp, b_tx)):
                            if len(part) == 0:
                                continue
                            sa = rm.subaperture(band, SUBAPERTURE_OF_BASE[kinds[q]])
                            add(t0, float(u[part].mean()), float(v[part].mean()), sa.hpbw_u * lam_d / 2.0,
                                sa.hpbw_v * lam_d / 2.0, kinds[q], t_dwell + echo_s,
                                lnf_full * sa.n_elements / sa.n_total, q)
                    elif mode == "tdm":
                        for q, p_ in enumerate(grp):
                            add(t0 + q * t_dwell / len(grp), float(u[p_]), float(v[p_]), hw_full, hw_full, 0,
                                t_dwell / len(grp) + echo_s, lnf_full)
                    else:                       # single, superposition: every beam of the dwell at once, full face
                        for p_ in grp:
                            add(t0, float(u[p_]), float(v[p_]), hw_full, hw_full, 0, t_dwell + echo_s, lnf_full)
    ev = {k: np.asarray(v_) for k, v_ in cols.items()}
    idx = np.argsort(ev["t"], kind="stable")
    idx = idx[ev["t"][idx] < duration]
    ev = {k: v_[idx] for k, v_ in ev.items()}
    nn = np.array([f_[0] for f_ in _FRAMES])[ev["face"]]
    eu = np.array([f_[1] for f_ in _FRAMES])[ev["face"]]
    evv = np.array([f_[2] for f_ in _FRAMES])[ev["face"]]
    w_ = np.sqrt(np.clip(1.0 - ev["u"] ** 2 - ev["v"] ** 2, 0.0, 1.0))
    ev["dir"] = ev["u"][:, None] * eu + ev["v"][:, None] * evv + w_[:, None] * nn
    ev["org"] = np.array([aperture_centre(k) for k in range(4)])[ev["face"]]
    ev.update(ids=ids, rrs=rrs, mission=mission, duration=duration,
              wf_lines={r["id"]: waveform_lines(r) for r in ok})
    return ev


def waveform_lines(r: dict) -> list:
    """The waveforms the optimiser plays in a volume, one line each: integration, nominal pulse and the pulses
    actually sent over its scan bins (shortened, no excess energy), pulses x PRI, bandwidth, TX mode, scan span."""
    by = {}
    for b in r.get("wf_by_bin", []):
        by.setdefault(b["wf"]["name"], []).append(b)
    out = []
    for name, bs in by.items():
        w = bs[0]["wf"]
        sent = [w["tau_us"] * float(b.get("pulse_scale", 1.0)) for b in bs]
        lo, hi = min(b["theta_lo_deg"] for b in bs), max(b["theta_hi_deg"] for b in bs)
        tx = w.get("tx_mode", "single") + (f" x{w['b_tx']}" if int(w.get("b_tx", 1)) > 1 else "")
        span = f"{min(sent):.3g}-{max(sent):.3g}" if max(sent) > 1.001 * min(sent) else f"{max(sent):.3g}"
        out.append(f"{name}: {w['integration'][:4].lower()} {w['tau_us']:g} us (sent {span} us) x {w['n_pulses']} "
                   f"@ {w['pri_ms']:g} ms, {w['sig_bw_mhz']:g} MHz, {tx}, scan {lo:.0f}-{hi:.0f} deg")
    return out


def _base_outline(kind: int, sides: int) -> np.ndarray:
    """Outline (sides, 2) of the emitting part of the plate in the face plane (x along e_u, y along e_v, km),
    counter-clockwise from about angle 0: the envelope disc, a half or a quadrant of it."""
    rad = rm.ARRAY_ENVELOPE_M / 2.0 * KM
    name = BASE_KINDS[kind]
    if name == "full":
        a = 2.0 * np.pi * np.arange(sides) / sides
        return np.c_[rad * np.cos(a), rad * np.sin(a)]
    if name.startswith("half"):
        arc = np.linspace(0.0, np.pi, 65)
        poly = np.r_[np.c_[np.cos(arc), np.sin(arc)], [[0.0, 0.0]]] * rad
    else:
        arc = np.linspace(0.0, np.pi / 2.0, 33)
        poly = np.r_[np.c_[np.cos(arc), np.sin(arc)], [[0.0, 0.0]]] * rad
    sx, sy = 1.0, 1.0
    if name in ("half_y-",):
        sy = -1.0
    if name in ("half_x+", "half_x-"):
        poly = poly[:, ::-1] * np.array([1.0, -1.0])      # the half y>0 turned to x>0
        poly = poly[::-1]
        sx = -1.0 if name == "half_x-" else 1.0
    if name.startswith("quad"):
        sx = 1.0 if name[4] == "+" else -1.0
        sy = 1.0 if name[5] == "+" else -1.0
    poly = poly * np.array([sx, sy])
    if sx * sy < 0:
        poly = poly[::-1]                                 # keep the outline counter-clockwise
    closed = np.r_[poly, poly[:1]]
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    t = np.linspace(0.0, s[-1], sides, endpoint=False)
    out = np.c_[np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])]
    c = out.mean(axis=0)
    start = int(np.argmin(np.abs(np.arctan2(out[:, 1] - c[1], out[:, 0] - c[0]))))
    return np.roll(out, -start, axis=0)


_OUTLINES: dict = {}
SUB_SHADES = (1.0, 0.72, 0.5, 0.86)          # sub-beam q of a partition: a shade of its volume's colour


def beam_rgb(ev: dict, idx: np.ndarray) -> np.ndarray:
    """Colour (n, 3) uint8 of beams idx: their volume's colour, shaded per sub-aperture of a partition, so that a
    sub-aperture and the beam it transmits share a colour and the sub-beams of a dwell are told apart."""
    pal = np.array([[int(c[i:i + 2], 16) for i in (1, 3, 5)] for c in PALETTE], dtype=float)
    rgb = pal[ev["sidx"][idx] % len(pal)] * np.array(SUB_SHADES)[ev["sub"][idx] % len(SUB_SHADES)][:, None]
    return np.clip(rgb, 0, 255).astype(np.uint8)


def base_patches(ev: dict, sel: np.ndarray, sides: int = CONE_SIDES) -> pv.PolyData:
    """The emitting part of each drawn beam's plate (full disc, half or quadrant), filled in the beam's colour, just in
    front of the yellow envelope disc: shows which (sub-)aperture transmits."""
    idx = np.flatnonzero(sel)
    if len(idx) == 0:
        empty = pv.PolyData(np.zeros((3, 3)), faces=np.array([3, 0, 1, 2]))
        empty["rgb"] = np.zeros((3, 3), dtype=np.uint8)
        return empty
    face, base = ev["face"][idx], ev["base"][idx]
    nn = np.array([f_[0] for f_ in _FRAMES])[face]
    eu = np.array([f_[1] for f_ in _FRAMES])[face]
    evv = np.array([f_[2] for f_ in _FRAMES])[face]
    centre = np.array([aperture_centre(k) for k in range(4)])[face] + 0.02 * KM * nn
    for kb in np.unique(base):
        if (kb, sides) not in _OUTLINES:
            _OUTLINES[(kb, sides)] = _base_outline(int(kb), sides)
    xy = np.array([_OUTLINES[(kb, sides)] for kb in base])
    pts = (centre[:, None, :] + xy[..., 0:1] * eu[:, None, :] + xy[..., 1:2] * evv[:, None, :]).reshape(-1, 3)
    faces = np.c_[np.full(len(idx), sides), (np.arange(len(idx)) * sides)[:, None] + np.arange(sides)[None, :]]
    poly = pv.PolyData(pts, faces=faces.ravel())
    poly["rgb"] = np.repeat(beam_rgb(ev, idx), sides, axis=0)
    return poly


def beam_cones(ev: dict, sel: np.ndarray, sides: int = CONE_SIDES) -> pv.PolyData:
    """Transmit beams as solids: from the emitting part of the plate (full disc, half or quadrant), collimated over the
    near field D_eff^2 / lambda, then opening along the -3 dB contour (half-widths hu, hv along u and v around the
    pointing in the face's direction cosines: exact widening with the scan), out to the volume's range."""
    idx = np.flatnonzero(sel)
    n_b = len(idx)
    if n_b == 0:
        empty = pv.PolyData(np.zeros((3, 3)), faces=np.array([3, 0, 1, 2]))
        empty["rgb"] = np.zeros((3, 3), dtype=np.uint8)
        return empty
    face, base = ev["face"][idx], ev["base"][idx]
    a = 2.0 * np.pi * np.arange(sides) / sides
    ca, sa = np.cos(a), np.sin(a)
    nn = np.array([f_[0] for f_ in _FRAMES])[face]
    eu = np.array([f_[1] for f_ in _FRAMES])[face]
    evv = np.array([f_[2] for f_ in _FRAMES])[face]
    centre = np.array([aperture_centre(k) for k in range(4)])[face]
    u = ev["u"][idx, None] + ev["hu"][idx, None] * ca
    v = ev["v"][idx, None] + ev["hv"][idx, None] * sa
    w = np.sqrt(np.clip(1.0 - u * u - v * v, 0.0, 1.0))
    d = u[..., None] * eu[:, None, :] + v[..., None] * evv[:, None, :] + w[..., None] * nn[:, None, :]
    d /= np.linalg.norm(d, axis=2, keepdims=True)
    for kb in np.unique(base):
        if (kb, sides) not in _OUTLINES:
            _OUTLINES[(kb, sides)] = _base_outline(int(kb), sides)
    xy = np.array([_OUTLINES[(kb, sides)] for kb in base])                 # (n_b, sides, 2)
    start = centre[:, None, :] + xy[..., 0:1] * eu[:, None, :] + xy[..., 1:2] * evv[:, None, :]
    lens = ev["len"][idx]
    lnf = np.minimum(ev["lnf"][idx], lens)
    mid = start + lnf[:, None, None] * ev["dir"][idx][:, None, :]
    end = mid + (lens - lnf)[:, None, None] * d
    pts = np.concatenate([start, mid, end], axis=1).reshape(-1, 3)       # per beam: start, mid, end rings
    b0 = (np.arange(n_b) * 3 * sides)[:, None]
    j = np.arange(sides)[None, :]
    j1 = (j + 1) % sides
    q1 = np.stack([np.full((n_b, sides), 4), b0 + j, b0 + j1, b0 + sides + j1, b0 + sides + j], axis=2)
    q2 = np.stack([np.full((n_b, sides), 4), b0 + sides + j, b0 + sides + j1, b0 + 2 * sides + j1,
                   b0 + 2 * sides + j], axis=2)
    poly = pv.PolyData(pts, faces=np.concatenate([q1, q2], axis=1).ravel())
    poly["rgb"] = np.repeat(beam_rgb(ev, idx), 3 * sides, axis=0)
    return poly


def _ship_dirs(sec: dict, phi_deg: np.ndarray, eps_deg: np.ndarray) -> np.ndarray:
    """Directions (N, 3) given in the sector's frame, in the ship frame (frame_m turns ship -> sector)."""
    p, e = np.radians(phi_deg), np.radians(eps_deg)
    d = np.stack([np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e)], axis=-1)
    m = sec.get("frame_m")
    return d if m is None else d @ np.array(m).reshape(3, 3)


def volume_outline(sec: dict, res_deg: float = 0.5):
    """(boundary lines, faint surface, radial edges) of a watch volume at its range (km): the sector's mask
    (core.optimizer.Sphere._mask, so every frame of step 3.0 is honoured) on a raster of directions on the sphere of
    radius range_km."""
    r = float(sec["range_km"])
    phi = np.arange(0.0, 360.0 + res_deg / 2, res_deg)
    eps = np.arange(-90.0, 90.0 + res_deg / 2, res_deg)
    pp, ee = np.meshgrid(phi, eps, indexing="ij")
    inside = mo.Sphere._mask(SimpleNamespace(phi=pp, eps=ee), sec).astype(float)
    p, e = np.radians(pp), np.radians(ee)
    grid = pv.StructuredGrid((r * np.cos(e) * np.cos(p))[..., None], (r * np.cos(e) * np.sin(p))[..., None],
                             (r * np.sin(e))[..., None])
    grid["m"] = inside.ravel(order="F")
    lines = grid.contour([0.5], scalars="m") if 0.0 < inside.mean() < 1.0 else None
    surface = grid.threshold(0.5, scalars="m").extract_surface(algorithm="dataset_surface")
    off = sec.get("frame_eps_offset_deg", 0.0)
    if sec.get("shape") == "cap":
        rad = math.radians(sec["radius_deg"])
        c = direction(sec["phi0_deg"], sec["eps0_deg"])
        u = np.cross([0.0, 0.0, 1.0], c)
        u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-9 else np.array([1.0, 0.0, 0.0])
        v = np.cross(c, u)
        corners = np.array([math.cos(rad) * c + math.sin(rad) * (math.cos(t) * u + math.sin(t) * v)
                            for t in np.radians(np.arange(0.0, 360.0, 90.0))])
        m = sec.get("frame_m")
        corners = corners if m is None else corners @ np.array(m).reshape(3, 3)
    elif sec["phi_max_deg"] - sec["phi_min_deg"] < 360.0:
        cp = np.array([sec["phi_min_deg"], sec["phi_min_deg"], sec["phi_max_deg"], sec["phi_max_deg"]])
        ce = np.clip(np.array([sec["eps_min_deg"], sec["eps_max_deg"], sec["eps_min_deg"], sec["eps_max_deg"]]) + off,
                     -90.0, 90.0)
        corners = _ship_dirs(sec, cp, ce)
    else:
        corners = np.zeros((0, 3))
    radials = None
    if len(corners):
        pts = np.vstack([np.zeros((len(corners), 3)), corners * r])
        radials = pv.PolyData(pts, lines=np.ravel([[2, i, i + len(corners)] for i in range(len(corners))]))
    return lines, surface, radials


def ship_mesh() -> pv.PolyData:
    faces = np.hstack([np.full((len(env.MODEL_FACES), 1), 3), env.MODEL_FACES]).ravel()
    return pv.PolyData(env.MODEL_VERTICES * KM, faces)


def aperture_discs() -> pv.PolyData:
    """Disc of the array envelope of each face, in the face plane."""
    out = pv.PolyData()
    for k in range(4):
        out = out.merge(pv.Disc(center=aperture_centre(k), inner=0.0, outer=rm.ARRAY_ENVELOPE_M / 2.0 * KM,
                                normal=np.asarray(env.face_frame(k)[0]), c_res=48))
    return out


def body_spheres(mission: dict):
    """(id, sphere km, centre km, kind) of the sky bodies with a known radius, at the distance their angular radius
    gives."""
    out = []
    for b in (mission.get("global", {}).get("sky") or {}).get("bodies", []):
        kind = "earth" if str(b["type"]).startswith("earth") else str(b["type"])
        if kind not in BODY_RADIUS_KM:
            continue
        rad = BODY_RADIUS_KM[kind]
        c = direction(b["phi_deg"], b["eps_deg"]) * rad / math.sin(math.radians(b["angular_radius_deg"]))
        out.append((b["id"], pv.Sphere(radius=rad, center=c, theta_resolution=128, phi_resolution=128,
                                       texture_coordinates=True), c, kind))
    return out


class ScaleBar:
    """Scale bar at the bottom right of the window: a round length (1, 2 or 5 x 10^n km) about 200 px long, exact in
    the orthographic 1:1 view, at the focal distance in the perspective ship view."""

    def __init__(self, pl: pv.Plotter):
        self.pl = pl
        self.pts = vtk.vtkPoints()
        self.pts.SetNumberOfPoints(6)
        lines = vtk.vtkCellArray()
        for i, j in ((0, 1), (2, 3), (4, 5)):
            lines.InsertNextCell(2)
            lines.InsertCellPoint(i)
            lines.InsertCellPoint(j)
        poly = vtk.vtkPolyData()
        poly.SetPoints(self.pts)
        poly.SetLines(lines)
        mapper = vtk.vtkPolyDataMapper2D()
        mapper.SetInputData(poly)
        self.actor = vtk.vtkActor2D()
        self.actor.SetMapper(mapper)
        self.actor.GetProperty().SetColor(0.81, 0.85, 0.86)
        self.actor.GetProperty().SetLineWidth(2)
        pl.renderer.AddActor(self.actor)
        self.poly = poly
        self.text = pl.add_text(" ", position=(0, 0), font_size=9, color="#cfd8dc", name="scale_text")
        self.text.GetTextProperty().SetJustificationToRight()

    def update(self) -> None:
        cam = self.pl.camera
        w, h = self.pl.window_size
        if cam.parallel_projection:
            world_h, where = 2.0 * cam.parallel_scale, ""
        else:
            dist = float(np.linalg.norm(np.subtract(cam.position, cam.focal_point)))
            world_h, where = 2.0 * dist * math.tan(math.radians(cam.view_angle) / 2.0), " (at the ship)"
        km_px = world_h / h
        raw = 200.0 * km_px
        e = 10.0 ** math.floor(math.log10(raw))
        length = max(m * e for m in (1.0, 2.0, 5.0) if m * e <= raw)
        px = length / km_px
        x1, y = w - 30.0, 50.0
        x0 = x1 - px
        for i, (x, yy) in enumerate(((x0, y), (x1, y), (x0, y - 6), (x0, y + 6), (x1, y - 6), (x1, y + 6))):
            self.pts.SetPoint(i, x, yy, 0.0)
        self.pts.Modified()
        self.poly.Modified()
        label = f"{length * 1e3:g} m" if length < 1.0 else f"{length:,.0f} km".replace(",", " ")
        self.text.SetInput(label + where)
        self.text.SetPosition(x1, y + 10)


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("mission", nargs="?", default=DEFAULT_MISSION,
                    help="mission path, or a name in tests/missions or tools/scenarios (default: scan3d_shell_lp)")
    ap.add_argument("--speed", type=float, default=1.0, help="simulated seconds per second (default 1: real time)")
    ap.add_argument("--duration", type=float, default=None, help="simulated seconds played (default: the longest "
                                                                    "revisit; each volume loops on its own)")
    ap.add_argument("--fade", type=float, default=0.02, help="a beam stays drawn this fraction of its sector's "
                                                               "revisit (default 0.02)")
    ap.add_argument("--timing", choices=TIMINGS, default="trace", help="trace (default), physical or slow (m cycles)")
    ap.add_argument("--slow", type=float, default=0.01, help="simulated seconds per second of the slow timing")
    ap.add_argument("--view", choices=("ship", "full"), default="ship", help="starting camera (v toggles)")
    ap.add_argument("--snapshot", help="render one instant (40 %% of the play) to this PNG and exit")
    a = ap.parse_args(argv)
    path = mission_path(a.mission)
    print(f"computing the pointings of {os.path.basename(path)} ...")
    ev = pointing_events(path, a.duration)
    ts, org, dirs, lens, sidx, on_s = ev["t"], ev["org"], ev["dir"], ev["len"], ev["sidx"], ev["on"]
    ids, rrs, mission, duration = ev["ids"], ev["rrs"], ev["mission"], ev["duration"]
    rr_max = max(rrs)
    loops = [max(1, math.ceil(duration / rr - 1e-9)) for rr in rrs]
    speed = {"trace": a.speed, "physical": a.speed, "slow": a.slow}
    fade = a.fade * np.asarray(rrs)[sidx]                     # per beam (s)
    print(f"{len(ts)} transmit beams over {duration:g} s; sectors {', '.join(f'{i} (RR {r:g} s)' for i, r in zip(ids, rrs))}; "
          f"physical durations {1e3 * on_s.min():.3g} to {1e3 * on_s.max():.3g} ms")

    pl = pv.Plotter(off_screen=bool(a.snapshot), window_size=(1400, 900))
    pl.set_background("#0c0c10")
    pl.add_mesh(ship_mesh(), color="#9aa4b1", smooth_shading=False, show_edges=False)
    pl.add_mesh(aperture_discs(), color="#ffd54f")
    pl.add_mesh(pv.PolyData(np.zeros((1, 3))), color="white", point_size=6, render_points_as_spheres=True)
    bodies = body_spheres(mission)
    for _, sphere, _, kind in bodies:
        if kind == "moon":
            pl.add_mesh(sphere, texture=pv.Texture(geo.moon_texture()), culling="back", smooth_shading=True)
        else:
            pl.add_mesh(sphere, color="#3b6fb6", culling="back", smooth_shading=True)
    cmap = PALETTE[:max(1, len(ids))]
    beams = beam_cones(ev, np.arange(len(ts)) == 0)
    bases = base_patches(ev, np.arange(len(ts)) == 0)
    pl.add_mesh(bases, scalars="rgb", rgb=True, lighting=False, name="bases")
    pl.add_mesh(beams, scalars="rgb", rgb=True, opacity=BEAM_OPACITY, lighting=False, name="beams")
    secs = {s["id"]: s for s in mission["sectors"]}
    for i, sid in enumerate(ids):
        lines, surface, radials = volume_outline(secs[sid])
        col = cmap[i % len(cmap)]
        if lines is not None and lines.n_points:
            pl.add_mesh(lines, color=col, line_width=2, opacity=0.9)
        if surface.n_points:
            pl.add_mesh(surface, color=col, opacity=0.06, show_edges=False, lighting=False)
        if radials is not None:
            pl.add_mesh(radials, color=col, line_width=1, opacity=0.7)
    # left column: each volume, then the waveforms the optimiser plays in it; then the sky bodies (dark backing so
    # that the lines stay readable over the volumes)
    def label(txt, y_, size, col):
        a_ = pl.add_text(txt, position=(10, y_), font_size=size, color=col)
        tp = a_.GetTextProperty()
        tp.SetBackgroundColor(0.05, 0.05, 0.07)
        tp.SetBackgroundOpacity(0.7)
    y = 865
    for i, sid in enumerate(ids):
        label(f"{sid}  {secs[sid]['range_km']:g} km  RR {rrs[i]:g} s", y, 9, cmap[i % len(cmap)])
        y -= 19
        for line in ev["wf_lines"].get(sid, []):
            label("   " + line, y, 7, cmap[i % len(cmap)])
            y -= 15
        y -= 4
    for bid, _, c, kind in bodies:
        dist = float(np.linalg.norm(c))
        label(f"{bid}  {dist:,.0f} km to its centre, {dist - BODY_RADIUS_KM[kind]:,.0f} km to its surface", y, 9,
              "#cfd8dc")
        y -= 19
    hud = pl.add_text(" ", position=(10, 10), font_size=10, color="white", name="hud")
    scale = ScaleBar(pl)

    # the scene spans 1e-3 to 1e4 km: the clipping planes are set at every image, the near one just in front of the
    # ship, so that the depth buffer resolves the hull (a near plane at 1 m with a far one at 3e4 km let beams of the
    # hidden plates show through it), the far one behind the farthest object
    scene = np.vstack([np.zeros((1, 3)), org + dirs * lens[:, None]] + [np.array([c]) for _, _, c, _ in bodies])
    lo, hi = scene.min(axis=0), scene.max(axis=0)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    ship_r = float(np.linalg.norm(env.MODEL_VERTICES, axis=1).max()) * KM
    for _, _, c, kind in bodies:
        corners = np.vstack([corners, c + BODY_RADIUS_KM[kind] * np.eye(3), c - BODY_RADIUS_KM[kind] * np.eye(3)])
    state = {"paused": False, "t": 0.0, "last": None, "ticks": 0, "view": a.view, "timing": a.timing, "t_prev": 0.0}
    pl.iren.interactor.GetInteractorStyle().SetAutoAdjustCameraClippingRange(False)

    def set_clipping() -> None:
        cam = np.asarray(pl.camera.position)
        look = np.asarray(pl.camera.focal_point) - cam
        look /= np.linalg.norm(look)
        depth = (corners - cam) @ look
        far = 1.05 * float(depth.max()) + 1.0
        if pl.camera.parallel_projection:
            near = max(1e-6, float(depth.min()) - 0.05 * float(np.ptp(depth)))
        else:
            near = max(1e-4, 0.5 * (float(np.linalg.norm(cam)) - ship_r))
        pl.camera.clipping_range = (near, max(far, 2.0 * near))

    def set_view(view: str) -> None:
        state["view"] = view
        if view == "ship":
            pl.camera.parallel_projection = False
            pl.camera_position = SHIP_CAMERA
        else:                                   # orthographic: the scale bar holds over the whole scene
            pl.camera.parallel_projection = True
            pl.camera_position = [(0.6, -1.0, 0.45), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)]
            pl.renderer.ResetCamera(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        set_clipping()
        scale.update()

    def draw(t_sim: float) -> None:
        tm = state["timing"]
        if tm == "trace":
            sel = (ts <= t_sim) & (ts > t_sim - fade)
            what = f"trace: {100 * a.fade:g} % of each revisit, not a physical duration"
        else:                   # real duration, and at least one image: the beams on air since the previous image
            sel = (ts <= t_sim) & (ts + on_s > min(state["t_prev"], t_sim - 1e-12))
            what = "physical: dwell + echo window" + (f", slow motion" if tm == "slow" else "")
        state["t_prev"] = t_sim
        beams.copy_from(beam_cones(ev, sel))
        bases.copy_from(base_patches(ev, sel))
        lp = "  ".join(f"{sid} {min(n, int(t_sim // rr) + 1)}/{n}" for sid, rr, n in zip(ids, rrs, loops))
        hud.SetInput(f"{int(sel.sum())} beams  |  {what} (m)  |  x{speed[tm]:g}  |  view {state['view']} (v)"
                     + chr(10) + f"t = {t_sim:8.3f} / {duration:g} s  |  loop {lp}")
        set_clipping()
        scale.update()

    set_view(a.view)
    if a.snapshot:
        state["t_prev"] = 0.4 * duration - 1.0 / 30.0 * speed[a.timing]
        draw(0.4 * duration)
        pl.screenshot(a.snapshot)
        print(f"wrote {a.snapshot}")
        return 0

    # animation driven by a timer of the VTK event loop: closing the window ends the loop cleanly (a hand-made render
    # loop kept drawing into the destroyed OpenGL context). On Windows a VTK timer needs the window to exist: open it
    # first (non-blocking show), then create the timer and run the event loop.
    t_end = duration

    def tick(_step) -> None:
        now = time.perf_counter()
        if state["last"] is not None and not state["paused"]:
            state["t"] = min(t_end, state["t"] + (now - state["last"]) * speed[state["timing"]])
        state["last"] = now
        state["ticks"] += 1
        draw(state["t"])

    pl.add_key_event("space", lambda: state.update(paused=not state["paused"]))
    # replay from the beginning ('r' is VTK's camera reset)
    pl.add_key_event("Home", lambda: (state.update(t=0.0, t_prev=0.0, last=None), print("restart", flush=True)))
    pl.add_key_event("v", lambda: set_view("full" if state["view"] == "ship" else "ship"))
    pl.add_key_event("m", lambda: state.update(timing=TIMINGS[(TIMINGS.index(state["timing"]) + 1) % len(TIMINGS)]))
    draw(0.0)
    pl.show(interactive_update=True)
    set_view(state["view"])
    pl.add_timer_event(max_steps=10 ** 9, duration=33, callback=tick)

    # while the mouse moves the view (button held), Windows delivers mouse messages instead of the timer's: the play
    # would freeze; the interaction events advance it too (at most one image every 30 ms, rendered by the interaction)
    def on_interaction(_obj, _event) -> None:
        if state["last"] is None or time.perf_counter() - state["last"] >= 0.03:
            tick(0)

    for ev_name in ("InteractionEvent", "MouseMoveEvent", "MouseWheelForwardEvent", "MouseWheelBackwardEvent"):
        pl.iren.interactor.AddObserver(ev_name, on_interaction)
    pl.iren.interactor.Start()
    print(f"played {state['t']:.1f} s of mission time ({state['ticks']} images)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
