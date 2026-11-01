#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MANEUVER_NODE -- the manoeuvre node of the simulator's 3D views (user, 29 October 2026, after the space-flight
games' node): the impulse or burn chosen in the editor's manoeuvre list sits as a node on its body's path, at its time.
  - six handles, + and - along the pnr axes (prograde, normal, radial, sim/propagate.py pnr_matrix) of the body's
    state at the node, true 3D segments (their length on screen shrinks as their axis turns toward the camera), in
    the ship axes' colours (X, Y, Z): a handle pulled away from its rest place changes the
    delta-v (impulse) or the acceleration (burn) along its axis at a rate that grows as the square of the pull, up to
    sim_node_dv_rate (m/s per s) or sim_node_accel_rate (m/s2 per s) at its stop; pushed toward the centre, it takes
    away; the value is kept in the manoeuvre's own frame (pnr, rtn or inertial);
  - the node's ring dragged along the path chooses the manoeuvre's time (from the playback time on);
  - the editor's box and the node show the same values at all times, each way;
  - the body's planned trajectory follows in real time (prop.preview_path: the body alone under the run's gravity
    with its own impulses and burns, from its last record before the earliest of its manoeuvres; no PN, no Singer);
    the full propagation replaces it once the edit is over (the editor's validation, then the view's automatic
    propagation);
  - global view and ship view (there the path relative to the ship in its frame at each instant; no node for the
    ship's own manoeuvres, the ship being the view's centre; none in the full range projection either).
The node keeps NODE_PX on screen (x the UI size) whatever the zoom.
"""
from __future__ import annotations

import math
import time
from typing import Callable, List, Optional, Tuple

import numpy as np
from PyQt5.QtCore import QTimer

from core import geometry as cgeo
from core.params import P
from sim import geometry as sgeo
from sim import propagate as prop
from sim import scenario as scn

NODE_PX = float(P.get("tool.display.sim_node_px"))                # px on screen, handles included (user)
DV_RATE = float(P.get("tool.display.sim_node_dv_rate"))           # impulse: m/s per s at the handle's stop (user)
ACCEL_RATE = float(P.get("tool.display.sim_node_accel_rate"))     # burn: m/s2 per s at the handle's stop (user)
PREVIEW_STEP_S = float(P.get("tool.display.sim_node_preview_step_s"))   # RK4 step of the real-time path (D)
RING_F, HANDLE_F, MARK_F = 0.15, 0.40, 0.08      # ring radius, handle's end, its marker's radius: x NODE_PX (D)
PULL_F = 0.35                                    # a handle's stop: this x NODE_PX from its rest place (D)
PICK_PAD_PX = 5.0                                # a press this near a marker or the ring takes it
BACK_COS, BACK_DIM, BACK_SIZE = 0.2, 0.55, 0.7   # a handle pointing away from the camera (cos > this): darker, smaller
TICK_MS = 30                                     # a held handle changes the value at this period
RING_POINTS = 48
AXIS_COLORS = tuple(cgeo.COLORS[k] for k in ("axis_x", "axis_y", "axis_z"))   # prograde, normal, radial (user)
RING_COLOR = "#6a8a92"
LABEL_COLOR = "#e6e6e6"
HANDLE_TEXT = (("pro", "retro"), ("nrm+", "nrm-"), ("rad+", "rad-"))
NAME = "dyn_node_"                               # its actors (the view's dynamic prefix: hidden when not drawn)


def _rgb(hex_color: str) -> Tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


class NodeTool:
    """The manoeuvre node of one SimView (view: its plotter, truth, scenario, time and drawing helpers)."""

    def __init__(self, view):
        self.v = view
        self.target: Optional[dict] = None       # row_id, owner (the editor's dict), m (the manoeuvre), on_edit
        self._cache: Optional[dict] = None       # the preview of the current values
        self._third = None                       # (run key, ThirdBodies)
        self._drag: Optional[dict] = None
        self._screen: Optional[dict] = None      # the glyph as drawn: centre, ring radius, handle ends (display px)
        self._glyph: Optional[tuple] = None      # (q world, axes (3, 3) in the view's axes)
        self._timer = QTimer(view)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------ the manoeuvre edited
    def set_target(self, row_id: Optional[str], owner: Optional[dict] = None, m: Optional[dict] = None,
                   on_edit: Optional[Callable[[bool], None]] = None,
                   on_release: Optional[Callable[[], None]] = None) -> None:
        """The manoeuvre m (a dict of the editor's scenario, edited in place) of the body row_id ("ship", an object
        id, "<group>#0"), owner its scenario entry; on_edit(final) after each change by the node (final: the drag is
        over); on_release() when the view lets the manoeuvre go (a click on another body or on nothing). None: no
        node."""
        self._stop_drag()
        self._glyph = self._screen = None                # (a camera change must not draw the former node)
        self.target = (dict(row_id=row_id, owner=owner, m=m, on_edit=on_edit, on_release=on_release)
                       if row_id is not None and m is not None and m.get("type") in ("impulse", "burn") else None)
        self.invalidate()

    def invalidate(self) -> None:
        """The values or the run changed: the preview is computed again at the next frame."""
        self._cache = None

    def row(self) -> Optional[int]:
        """The truth row of the body whose path the node redraws, None when no node."""
        tr = self.v.truth
        if self.target is None or tr is None or self.target["row_id"] not in tr.ids:
            return None
        return tr.row(self.target["row_id"])

    def dragging(self) -> bool:
        return self._drag is not None

    def owns(self, oid: Optional[str]) -> bool:
        """Whether the body oid picked in the view (OWNSHIP's own name: "ship") is the node's (a group: any of its
        members)."""
        if self.target is None or oid is None:
            return False
        rid = self.target["row_id"]
        if "#" in rid:
            return oid.rsplit("#", 1)[0] == rid.rsplit("#", 1)[0]
        return oid == rid

    def release(self) -> None:
        """The manoeuvre let go (user, 29 October 2026: a click on another body or on nothing): no node, the
        editor's list without a manoeuvre chosen."""
        if self.target is None:
            return
        cb = self.target.get("on_release")
        self.set_target(None)
        if cb is not None:
            cb()

    # ------------------------------------------------------------------ the real-time path
    def _bodies(self):
        tr, sc = self.v.truth, self.v.sc
        if not prop.THIRD_BODY:
            return None
        key = (id(tr), self.v.scenario_duration())
        if self._third is None or self._third[0] != key:
            self._third = (key, prop.ThirdBodies(tr.body.name, scn.epoch(sc), self.v.scenario_duration() + 10.0))
        return self._third[1]

    def _run_owner(self) -> Optional[dict]:
        """The body's scenario entry in the run shown (the manoeuvres the truth was propagated with)."""
        sc, rid = self.v.sc, self.target["row_id"]
        if rid == "ship":
            return sc.get("ship")
        for o in sc.get("objects", []):
            if o.get("id") == rid:
                return o
        gid = rid.rsplit("#", 1)[0]
        return next((g for g in sc.get("groups", []) if g.get("id") == gid), None)

    def _start(self, i: int, t_from: float) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
        """The body's last recorded state before t_from (its first one when it appears later)."""
        tr = self.v.truth
        ok = np.isfinite(tr.r_km[:, i, 0])
        if not ok.any():
            return None
        ks = np.flatnonzero(ok & (tr.t_s < t_from - 1e-9))
        k = ks[-1] if len(ks) else np.flatnonzero(ok)[0]
        return float(tr.t_s[k]), tr.r_km[k, i].copy(), tr.v_kms[k, i].copy()

    def _t_from(self, ms: List[dict]) -> float:
        """The earliest time any manoeuvre of the body changes, now or in the run shown."""
        old = (self._run_owner() or {}).get("manoeuvres", [])
        ts = [float(m["t_s"]) for m in list(ms) + list(old) if m.get("type") in ("impulse", "burn")]
        return min(ts) if ts else self.v.scenario_duration()

    def _preview(self) -> Optional[dict]:
        if self._cache is not None:
            return self._cache
        i = self.row()
        if i is None:
            return None
        tr, m = self.v.truth, self.target["m"]
        ms = self.target["owner"].get("manoeuvres", [])
        st = self._start(i, self._t_from(ms))
        if st is None:
            return None
        t0, r0, v0 = st
        ts, r, v, pre = prop.preview_path(tr.body, self._bodies(), r0, v0, t0, self.v.scenario_duration(), ms,
                                             PREVIEW_STEP_S)
        j = next((k for k, x in enumerate(ms) if x is m), None)
        self._cache = dict(t0=t0, ts=ts, r=r, v=v, node=pre[j] if j is not None else None, view=None)
        return self._cache

    def _to_view(self, ts: np.ndarray, r: np.ndarray) -> np.ndarray:
        """Positions at the times ts as the view draws them: global view, from the central body; ship view,
        relative to the ship in its frame at each instant."""
        if self.v._global():
            return r
        tr, dur = self.v.truth, self.v.duration()
        out = np.empty_like(r)
        for k, t in enumerate(ts):
            rs = sgeo.states_at(tr, float(t))[0][0]
            out[k] = (r[k] - rs) @ self.v._view_axes(float(min(t, dur)))
        return out

    def _view_path(self, c: dict) -> np.ndarray:
        if c["view"] is None or c["view"][0] != self.v.view_mode:
            c["view"] = (self.v.view_mode, self._to_view(c["ts"], c["r"]))
        return c["view"][1]

    # ------------------------------------------------------------------ drawing
    def shown(self) -> bool:
        i = self.row()
        return i is not None and (self.v._global() or i != 0)

    def draw(self) -> None:
        """At each frame: the body's planned trajectory from t (its records up to the preview's start, then the
        preview) and the node at the manoeuvre's time."""
        self._glyph = self._screen = None
        i = self.row()
        if i is None:
            return
        c = self._preview()
        if c is None:
            return
        v, t = self.v, self.v.t
        style = dict(line_width=3 if i == 0 else 2)
        rgb = v._row_rgb[i]
        if not v._global() and i == 0:
            return                                       # the ship view's centre: nothing to draw for its own path
        if t < c["t0"]:
            if v._global():
                v._truth_lines(f"{NAME}path_rec", np.array([i]), rgb[None], t, c["t0"], **style)
            else:
                g = v.g
                v._ship_lines(f"{NAME}path_rec", np.array([i - 1]), rgb[None], (g.t_s >= t) & (g.t_s <= c["t0"]),
                              **style)
        pts = self._view_path(c)
        keep = c["ts"] >= t - 1e-9
        pts = pts[keep]
        if len(pts) >= 2:
            if v._full_projection():
                pts = v._to_plane(pts)
            v._poly(f"{NAME}path", pts, lines=np.r_[len(pts), np.arange(len(pts))], color=_hex(rgb), **style)
        if c["node"] is None or v._full_projection():
            return
        rn, vn = c["node"]
        axes = prop.pnr_matrix(rn, vn).T                 # rows: prograde, normal, radial (ICRF)
        tn = float(self.target["m"]["t_s"])
        if v._global():
            q = rn
        else:
            tr = v.truth
            mm = v._view_axes(float(min(tn, v.duration())))
            q = (rn - sgeo.states_at(tr, tn)[0][0]) @ mm
            axes = axes @ mm
        self._glyph = (np.asarray(q, float), axes)
        self.place()

    def place(self) -> None:
        """The node's glyph at NODE_PX on screen, facing the camera (each frame and each camera change)."""
        if self._glyph is None or self.target is None:
            return
        v = self.v
        q, axes = self._glyph
        cam = v.plotter.camera
        pos = np.asarray(cam.GetPosition())
        fwd = np.asarray(cam.GetDirectionOfProjection())
        up = np.asarray(cam.GetViewUp(), float)
        up = up - (up @ fwd) * fwd
        up /= np.linalg.norm(up)
        right = np.cross(fwd, up)
        h_px = max(v.plotter.ren_win.GetSize()[1], 1)
        depth = max(float((q - pos) @ fwd), 1e-12)
        wpp = depth * 2.0 * math.tan(math.radians(0.5 * cam.GetViewAngle())) / h_px   # world size of one pixel
        px = NODE_PX * v.ui_scale
        ring_px, end_px, mark_px = RING_F * px, HANDLE_F * px, MARK_F * px
        ang = np.linspace(0.0, 2.0 * math.pi, RING_POINTS)
        circle = np.cos(ang)[:, None] * right + np.sin(ang)[:, None] * up
        pts = [q + ring_px * wpp * circle]
        lines = [np.r_[RING_POINTS, np.arange(RING_POINTS)]]
        cells = [_rgb(RING_COLOR)]
        marks = {True: ([], []), False: ([], [])}        # front / back: points, colours
        lab_p, lab_t, ends = [], [], []
        cx, cy = self._display(q)
        for a in range(3):
            for s_i, sign in enumerate((1.0, -1.0)):
                # a true 3D segment along its axis (user, 29 October 2026, as in the games: never two handles on one
                # another): its length on screen shrinks as the axis turns toward the camera
                d = sign * axes[a]
                sx, sy = float(d @ right), float(d @ up)
                n = math.hypot(sx, sy)                   # the axis's share in the screen's plane
                if n < 1e-6:                             # along the line of sight: pulled along a fixed direction
                    ux, uy = math.cos(a * 2.0 * math.pi / 3.0) * sign, math.sin(a * 2.0 * math.pi / 3.0) * sign
                else:
                    ux, uy = sx / n, sy / n
                away = float(d @ fwd)                    # > 0: the axis points away from the camera
                front = away <= BACK_COS
                col = _rgb(AXIS_COLORS[a]) if front else tuple(int(BACK_DIM * c) for c in _rgb(AXIS_COLORS[a]))
                mk = mark_px if front else BACK_SIZE * mark_px   # behind the node: smaller and darker (depth cue)
                e_px = end_px
                dr = self._drag
                if dr is not None and dr["kind"] == "handle" and dr["axis"] == a and dr["sign"] == sign:
                    e_px = end_px + dr["pull"] / max(n, 0.2)    # the pulled marker follows the mouse on screen
                end = q + e_px * wpp * d
                if e_px * n > ring_px + mk:              # the spoke from the ring's edge on screen to the marker
                    w = ux * right + uy * up
                    off = len(np.concatenate(pts))
                    pts.append(np.array([q + ring_px * wpp * w, end - mk * wpp * w]))
                    lines.append(np.array([2, off, off + 1]))
                    cells.append(col)
                if sign > 0:                             # + : a filled marker; - : a hollow one
                    marks[front][0].append(end)
                    marks[front][1].append(col)
                else:
                    off = len(np.concatenate(pts))
                    pts.append(end + mk * wpp * circle)
                    lines.append(np.r_[RING_POINTS, off + np.arange(RING_POINTS)])
                    cells.append(col)
                lab_p.append(end + 1.6 * mark_px * wpp * (ux * right + uy * up))
                lab_t.append(HANDLE_TEXT[a][s_i])
                ex, ey = self._display(q + end_px * wpp * d)      # the marker at rest, where a press takes it
                ends.append(dict(axis=a, sign=sign, x=ex, y=ey, sx=ux, sy=uy, away=away))
        v._poly(f"{NAME}glyph", np.concatenate(pts), lines=np.concatenate(lines),
                cell_rgb=np.array(cells, np.uint8), line_width=4)
        for front, name, size in ((True, "marks", mark_px), (False, "marks_back", BACK_SIZE * mark_px)):
            p_, c_ = marks[front]
            v._poly(f"{NAME}{name}", np.array(p_).reshape(-1, 3), rgb=np.array(c_, np.uint8).reshape(-1, 3),
                    point_size=2.0 * size, render_points_as_spheres=True)
            e = v._dyn.get(f"{NAME}{name}")
            if e is not None:
                e[2].GetProperty().SetPointSize(2.0 * size)
        v._labels(f"{NAME}handle_lbl", np.array(lab_p), lab_t, LABEL_COLOR, 12)
        v._labels(f"{NAME}value_lbl", (q - (ring_px + 0.3 * px) * wpp * up)[None], [self._value_text()], LABEL_COLOR, 14)
        self._screen = dict(cx=cx, cy=cy, ring=ring_px, mark=mark_px, ends=ends)

    def _value_text(self) -> str:
        m = self.target["m"]
        if m["type"] == "impulse":
            return f"dv {np.linalg.norm(m.get('dv_mps', [0, 0, 0])):.1f} m/s  t {float(m['t_s']):.1f} s"
        return (f"a {np.linalg.norm(m.get('accel_mps2', [0, 0, 0])):.2f} m/s2 x {float(m.get('duration_s', 0)):g} s  "
                f"t {float(m['t_s']):.1f} s")

    def _display(self, w: np.ndarray) -> Tuple[float, float]:
        ren = self.v.plotter.renderer
        ren.SetWorldPoint(float(w[0]), float(w[1]), float(w[2]), 1.0)
        ren.WorldToDisplay()
        x, y, _z = ren.GetDisplayPoint()
        return x, y

    # ------------------------------------------------------------------ the mouse
    def install(self, iren) -> None:
        """Observers before the view's own (its click) and the camera style's: a press on the node is the node's."""
        for ev, fn in (("LeftButtonPressEvent", self._press), ("MouseMoveEvent", self._move),
                       ("LeftButtonReleaseEvent", self._release)):
            tag = [None]

            def cb(obj, _ev, fn=fn, tag=tag):
                if fn(*obj.GetEventPosition()):
                    obj.GetCommand(tag[0]).SetAbortFlag(1)
            tag[0] = iren.AddObserver(ev, cb, 2.0)

    def _press(self, x: float, y: float) -> bool:
        s = self._screen
        if s is None or self.target is None:
            return False
        hit = [e for e in s["ends"] if math.hypot(x - e["x"], y - e["y"]) <= s["mark"] + PICK_PAD_PX]
        if hit:                                          # markers on one another: the one in front of the node
            near = min(hit, key=lambda e: (e["away"], math.hypot(x - e["x"], y - e["y"])))
            self._drag = dict(kind="handle", axis=near["axis"], sign=near["sign"], end=near, mouse=(x, y), pull=0.0,
                              last=time.perf_counter())
            self._timer.start()
            return True
        if math.hypot(x - s["cx"], y - s["cy"]) <= s["ring"] + PICK_PAD_PX:
            self._drag = dict(kind="time", base=self._base_path())
            return True
        return False

    def _move(self, x: float, y: float) -> bool:
        dr = self._drag
        if dr is None:
            return False
        if dr["kind"] == "handle":
            dr["mouse"] = (x, y)
            e = dr["end"]
            stop = PULL_F * NODE_PX * self.v.ui_scale
            dr["pull"] = float(np.clip((x - e["x"]) * e["sx"] + (y - e["y"]) * e["sy"], -stop, stop))
            self.place()
            self.v._render()
            return True
        t = self._time_at(dr["base"], x, y)
        if t is not None and abs(t - float(self.target["m"]["t_s"])) > 1e-6:
            self.target["m"]["t_s"] = round(t, 3)
            self._edited(False)
        return True

    def _release(self, _x: float, _y: float) -> bool:
        if self._drag is None:
            return False
        self._stop_drag()
        self._edited(True)
        return True

    def _stop_drag(self) -> None:
        self._timer.stop()
        self._drag = None

    def _tick(self) -> None:
        """A held handle: its pull changes the value along its axis (square law, signed)."""
        dr = self._drag
        if dr is None or dr["kind"] != "handle" or self.target is None:
            self._timer.stop()
            return
        now = time.perf_counter()
        dt, dr["last"] = now - dr["last"], now
        stop = PULL_F * NODE_PX * self.v.ui_scale
        frac = dr["pull"] / stop if stop > 0 else 0.0
        if abs(frac) < 1e-6:
            return
        c = self._preview()
        if c is None or c["node"] is None:
            return
        m = self.target["m"]
        key, rate = ("dv_mps", DV_RATE) if m["type"] == "impulse" else ("accel_mps2", ACCEL_RATE)
        delta = dr["sign"] * rate * frac * abs(frac) * dt               # along the handle's pnr axis
        rn, vn = c["node"]
        d_icrf = prop.pnr_matrix(rn, vn)[:, dr["axis"]] * delta
        d_own = prop.frame_matrix(m.get("frame", "rtn"), rn, vn).T @ d_icrf   # in the manoeuvre's own frame
        m[key] = [round(float(x), 4) for x in np.asarray(m.get(key, [0.0, 0.0, 0.0]), float) + d_own]
        self._edited(False)

    def _edited(self, final: bool) -> None:
        self.invalidate()
        cb = self.target["on_edit"] if self.target else None
        if cb is not None:
            cb(final)
        self.v._draw_frame()

    # ------------------------------------------------------------------ the node along its path
    def _base_path(self) -> Optional[dict]:
        """The places the node can take: the body's path without this manoeuvre (its records before any change,
        then the preview of its other manoeuvres), from the playback time on, as the view draws it."""
        i = self.row()
        if i is None:
            return None
        tr, m = self.v.truth, self.target["m"]
        ms = self.target["owner"].get("manoeuvres", [])
        others = [x for x in ms if x is not m]
        st = self._start(i, self._t_from(ms))
        if st is None:
            return None
        t0, r0, v0 = st
        ts, r, _v, _pre = prop.preview_path(tr.body, self._bodies(), r0, v0, t0, self.v.scenario_duration(), others,
                                            PREVIEW_STEP_S)
        lo = max(self.v.t, float(tr.t_appear_s[i]))
        ks = np.flatnonzero(np.isfinite(tr.r_km[:, i, 0]) & (tr.t_s >= lo) & (tr.t_s < t0))
        rec = self._to_view(tr.t_s[ks], tr.r_km[ks, i]) if len(ks) else np.zeros((0, 3))
        keep = ts >= lo
        pts = np.concatenate([rec, self._to_view(ts[keep], r[keep])])
        times = np.concatenate([tr.t_s[ks], ts[keep]])
        if self.v._full_projection():
            pts = self.v._to_plane(pts)
        return dict(t=times, pts=pts)

    def _time_at(self, base: Optional[dict], x: float, y: float) -> Optional[float]:
        """The time of the path's point nearest to the display point (x, y) (on its segments)."""
        if base is None or len(base["t"]) < 2:
            return None
        d = np.array([self._display(p) for p in base["pts"]])
        a, b = d[:-1], d[1:]
        ab = b - a
        L2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12)
        u = np.clip(np.einsum("ij,ij->i", np.array([x, y]) - a, ab) / L2, 0.0, 1.0)
        dist = np.linalg.norm(a + u[:, None] * ab - np.array([x, y]), axis=1)
        k = int(np.argmin(dist))
        t = base["t"][k] + u[k] * (base["t"][k + 1] - base["t"][k])
        return float(np.clip(t, 0.0, self.v.scenario_duration()))


def _hex(rgb) -> str:
    r, g, b = (int(x) for x in rgb[:3])
    return f"#{r:02x}{g:02x}{b:02x}"
