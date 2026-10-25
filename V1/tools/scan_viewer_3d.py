#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time 3D view of the search pointings of a mission around the low-poly ship model (step 4.3 bis).

    python tools/scan_viewer_3d.py                          scenario tools/scenarios/scan3d_shell_lp.json
    python tools/scan_viewer_3d.py mission_veille500        a test mission (tests/missions), or any mission path
    python tools/scan_viewer_3d.py --view full --speed 10   start in the 1:1 view of the whole scene
    python tools/scan_viewer_3d.py --snapshot out.png       render one instant off screen and exit (check)

The whole scene is at true scale, in km: the ship model, each beam drawn from its face's array centre out to the
sector's range (a 500 km search -> a 500 km line), and the sky bodies of the mission with a known radius (Moon,
Earth) as spheres at the distance their angular radius implies (radius / sin(angular radius)), in their direction.
Two cameras: 'ship' (close to the hull) and 'full' (1:1, the whole scene); a scale bar follows the zoom.
The pointings are those of core/scanorder.py: the hexagonal lattice of each volume's plan in the face's (u, v),
shifted frame to frame, in bit-reversed Morton order; pointing i of a frame is drawn at i x RR / N (the release times
of the scheduler's synthetic timeline); recent beams fade out. The mission result is cached in tools/.viewer_cache
(keyed by the code, the parameter file and the mission), else computed (a few minutes). Test values only.
Keys: space = pause, v = ship / 1:1 view, q = quit.
"""
import argparse
import glob
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import pyvista as pv  # noqa: E402
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
DEFAULT_MISSION = os.path.join(ROOT, "tools", "scenarios", "scan3d_shell_lp.json")
SHIP_CAMERA = [(0.23, -0.16, 0.04), (0.0, 0.0, -0.025), (0.0, 0.0, 1.0)]
CLIP = {"ship": (1e-3, 3e4), "full": (1.0, 1e5)}              # near / far planes (km) of each view


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


def pointing_events(path: str, n_frames: int):
    """(times s, origins (N, 3) km, unit directions (N, 3), lengths km (N,), sector index (N,), sector ids, revisits,
    mission) of every pointing over n_frames frames of each volume and face."""
    mission, _ = frames.resolve(mo.load_mission(path))
    res = mission_result(path)
    g = mission.get("global", {})
    sph = mo._sphere(g.get("eps_min_deg", -90.0), g.get("eps_max_deg", 90.0))
    sky = g.get("sky")
    secs = {s["id"]: s for s in mission["sectors"]}
    t_all, o_all, d_all, l_all, s_all, ids, rrs = [], [], [], [], [], [], []
    for r in res["sectors"]:
        if r.get("status") != "OK":
            continue
        sec = secs[r["id"]]
        si = len(ids)
        ids.append(r["id"])
        f = float(r["frequency"]["freq_ghz"])
        rr = float(r["rr_target_s"])
        rrs.append(rr)
        reach = float(sec["range_km"])
        spacing = float(r.get("beam_spacing_hpbw", rm.BEAM_SPACING_HPBW))
        m = sph.mask(sec)
        _, keep = mo.sector_sky(sec, sph, m, f, "S", sky)
        lim = rm.band_for_frequency(f).scan_limit_deg(f)
        sel0 = m & sph.scan_zone & (sph.theta <= lim) & (np.asarray(keep) > 0.5)
        d = spacing * rm._pattern(f).tx.hpbw * rm.lambda_m(f) / rm.ARRAY_ENVELOPE_M
        for k in range(4):
            if str(k) not in r.get("faces", {}):
                continue
            uu, vv = so.face_uv(sph, k)
            n_, eu, ev = (np.array(x) for x in env.face_frame(k))
            origin = aperture_centre(k)
            for fr in range(n_frames):
                pts = so.lattice(sel0 & (sph.owner == k), uu, vv, d, fr)
                if len(pts) == 0:
                    continue
                order = so.order_bitrev_morton(pts, d)
                u, v = pts[order, 0], pts[order, 1]
                w = np.sqrt(np.clip(1.0 - u * u - v * v, 0.0, 1.0))
                t_all.append(fr * rr + np.arange(len(order)) * rr / len(order))
                o_all.append(np.repeat(origin[None, :], len(order), axis=0))
                d_all.append(u[:, None] * eu + v[:, None] * ev + w[:, None] * n_)
                l_all.append(np.full(len(order), reach))
                s_all.append(np.full(len(order), si))
    ts = np.concatenate(t_all)
    idx = np.argsort(ts, kind="stable")
    return (ts[idx], np.concatenate(o_all)[idx], np.concatenate(d_all)[idx], np.concatenate(l_all)[idx],
            np.concatenate(s_all)[idx], ids, rrs, mission)


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


def beams_poly(origins: np.ndarray, dirs: np.ndarray, lengths: np.ndarray, scalars: np.ndarray) -> pv.PolyData:
    if len(origins) == 0:
        return pv.PolyData(np.zeros((2, 3)), lines=np.array([2, 0, 1]))
    pts = np.empty((2 * len(origins), 3))
    pts[0::2] = origins
    pts[1::2] = origins + dirs * lengths[:, None]
    lines = np.c_[np.full(len(origins), 2), np.arange(0, 2 * len(origins), 2), np.arange(1, 2 * len(origins), 2)]
    poly = pv.PolyData(pts, lines=lines.ravel())
    poly["c"] = np.repeat(scalars, 2)
    return poly


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
    ap.add_argument("--speed", type=float, default=None, help="simulated seconds per second (default: a frame of the "
                                                                "longest revisit in ~20 s)")
    ap.add_argument("--frames", type=int, default=3, help="frames played (lattice shifts cycle every 3)")
    ap.add_argument("--fade", type=float, default=0.02, help="a beam stays drawn this fraction of its sector's "
                                                               "revisit (default 0.02)")
    ap.add_argument("--view", choices=("ship", "full"), default="ship", help="starting camera (v toggles)")
    ap.add_argument("--snapshot", help="render one instant (40 %% of the first frame) to this PNG and exit")
    a = ap.parse_args(argv)
    path = mission_path(a.mission)
    print(f"computing the pointings of {os.path.basename(path)} ...")
    ts, org, dirs, lens, sidx, ids, rrs, mission = pointing_events(path, a.frames)
    rr_max = max(rrs)
    speed = a.speed or rr_max / 20.0
    fade = a.fade * np.asarray(rrs)[sidx]                     # per beam (s)
    print(f"{len(ts)} pointings over {a.frames} frames; sectors {', '.join(f'{i} (RR {r:g} s)' for i, r in zip(ids, rrs))}; "
          f"speed x{speed:g}")

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
    beams = beams_poly(org[:1], dirs[:1], lens[:1], sidx[:1].astype(float))
    pl.add_mesh(beams, scalars="c", cmap=cmap, clim=(0, max(1, len(ids) - 1)), line_width=2, show_scalar_bar=False,
                name="beams")
    secs = {s["id"]: s for s in mission["sectors"]}
    for i, sid in enumerate(ids):
        pl.add_text(f"{sid}  {secs[sid]['range_km']:g} km  RR {rrs[i]:g} s", position=(10, 860 - 22 * i), font_size=9,
                    color=cmap[i % len(cmap)])
    for j, (bid, _, c, _) in enumerate(bodies):
        pl.add_text(f"{bid}  {np.linalg.norm(c):,.0f} km to its centre", position=(10, 860 - 22 * (len(ids) + j)),
                    font_size=9, color="#cfd8dc")
    hud = pl.add_text(" ", position=(10, 10), font_size=10, color="white", name="hud")
    scale = ScaleBar(pl)

    # the scene spans 1e-3 to 1e4 km: fixed clipping planes per view (the automatic ones would cut the hull away)
    scene = np.vstack([np.zeros((1, 3)), org + dirs * lens[:, None]] + [np.array([c]) for _, _, c, _ in bodies])
    lo, hi = scene.min(axis=0), scene.max(axis=0)
    state = {"paused": False, "t": 0.0, "last": None, "ticks": 0, "view": a.view}
    pl.iren.interactor.GetInteractorStyle().SetAutoAdjustCameraClippingRange(False)

    def set_view(view: str) -> None:
        state["view"] = view
        if view == "ship":
            pl.camera.parallel_projection = False
            pl.camera_position = SHIP_CAMERA
        else:                                   # orthographic: the scale bar holds over the whole scene
            pl.camera.parallel_projection = True
            pl.camera_position = [(0.6, -1.0, 0.45), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)]
            pl.renderer.ResetCamera(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        pl.camera.clipping_range = CLIP[view]
        scale.update()

    def draw(t_sim: float) -> None:
        sel = (ts <= t_sim) & (ts > t_sim - fade)
        beams.copy_from(beams_poly(org[sel], dirs[sel], lens[sel], sidx[sel].astype(float)))
        hud.SetInput(f"t = {t_sim:7.2f} s  |  frame {int(t_sim // rr_max) + 1}/{a.frames} of the longest revisit  |  "
                     f"{int(sel.sum())} beams drawn ({100 * a.fade:g} % of their revisit)  |  x{speed:g}  |  view {state['view']} (v)")
        pl.camera.clipping_range = CLIP[state["view"]]
        scale.update()

    set_view(a.view)
    if a.snapshot:
        draw(0.4 * rr_max)
        pl.screenshot(a.snapshot)
        print(f"wrote {a.snapshot}")
        return 0

    # animation driven by a timer of the VTK event loop: closing the window ends the loop cleanly (a hand-made render
    # loop kept drawing into the destroyed OpenGL context). On Windows a VTK timer needs the window to exist: open it
    # first (non-blocking show), then create the timer and run the event loop.
    t_end = float(ts[-1])

    def tick(_step) -> None:
        now = time.perf_counter()
        if state["last"] is not None and not state["paused"]:
            state["t"] = min(t_end, state["t"] + (now - state["last"]) * speed)
        state["last"] = now
        state["ticks"] += 1
        draw(state["t"])

    pl.add_key_event("space", lambda: state.update(paused=not state["paused"]))
    pl.add_key_event("v", lambda: set_view("full" if state["view"] == "ship" else "ship"))
    draw(0.0)
    pl.show(interactive_update=True)
    set_view(state["view"])
    pl.add_timer_event(max_steps=10 ** 9, duration=33, callback=tick)
    pl.iren.interactor.Start()
    print(f"played {state['t']:.1f} s of mission time ({state['ticks']} images)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
