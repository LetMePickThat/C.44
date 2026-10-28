"""Tests of the global scenario simulator (roadmap phase 6): scenario format and loading (pass S1), the scenario
editor's form logic (pass S5, without a window)."""
import copy
import json
import os

import pytest

from core.params import P
from sim import scenario as scn


def _base() -> dict:
    return {"meta": {"name": "t", "epoch_utc": "2050-01-01T00:00:00Z", "duration_s": 600.0},
            "ship": {"orbit": {"type": "elements", "a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 0.0,
                               "argp_deg": 0.0, "nu_deg": 0.0}},
            "objects": [{"id": "K1", "class": "KKV",
                         "state": {"type": "relative", "range_km": 1000.0, "phi_deg": 0.0, "eps_deg": 0.0,
                                   "v_close_mps": 5000.0},
                         "manoeuvres": [{"type": "pn", "start_range_km": 100.0}]}],
            "groups": [{"id": "G1", "class": "DECOY", "n": 5, "around": "K1", "pos_sigma_m": 500.0,
                        "vel_sigma_mps": 5.0}]}


def test_valid_scenario_and_defaults():
    """S1: a complete scenario is valid; the defaults come from [sim] (step, PN constant) and the format."""
    sc = _base()
    assert scn.validate(sc) == []
    n = scn.normalise(sc)
    assert n["meta"]["step_s"] == P.get("sim.step_s") and n["meta"]["seed"] == 1
    assert n["central_body"] == "earth" and n["ship"]["attitude"] == [{"t_s": 0.0, "attitude": {"type": "prograde",
                                                                                                 "roll_deg": 0.0}}]
    assert n["objects"][0]["manoeuvres"][0]["n"] == P.get("sim.pn_n")
    assert n["objects"][0]["state"]["frame"] == "ship" and n["objects"][0]["state"]["v_transverse_mps"] == [0.0, 0.0]
    assert n["groups"][0]["t_appear_s"] == 0.0
    assert sc["meta"].get("step_s") is None                 # normalise works on a copy
    b = scn.bodies()["earth"]
    assert b["mu_km3_s2"] == pytest.approx(398600.4418) and b["r_eq_km"] > b["radius_km"] > 0 and b["j2"] > 0


@pytest.mark.parametrize("edit, words", [
    (lambda s: s["meta"].update(epoch_utc="yesterday"), "ISO 8601"),
    (lambda s: s.update(central_body="mars"), "unknown"),
    (lambda s: s["objects"].append(copy.deepcopy(s["objects"][0])), "used twice"),
    (lambda s: s["objects"][0].update({"class": "XX"}), "class must be"),
    (lambda s: s["objects"][0]["state"].update(range_km=-1.0), "range_km must be > 0"),
    (lambda s: s["objects"][0]["manoeuvres"].append({"type": "pn"}), "either start_range_km or t_s"),
    (lambda s: s["objects"][0]["manoeuvres"].append({"type": "impulse", "t_s": 1e6, "dv_mps": [0, 0, 1]}), "t_s must lie"),
    (lambda s: s["groups"][0].update(n=0), "n must be an integer"),
    (lambda s: s["groups"][0].update(around="nobody"), "around must name"),
    (lambda s: s["ship"].update(attitude=[{"t_s": 0.0, "attitude": {"type": "target", "id": "ghost"}}]),
     "not an object"),
    (lambda s: s["ship"].update(orbit={"type": "elements", "a_km": 7000.0, "e": 1.2, "i_deg": 0, "raan_deg": 0,
                                       "argp_deg": 0, "nu_deg": 0}), "closed orbits"),
    (lambda s: s["ship"].update(attitude=[{"t_s": 0.0, "attitude": {"type": "prograde"}},
                                          {"t_s": 5.0, "rate_dps": 2.0, "attitude": {"type": "radial_in"}},
                                          {"t_s": 1.0, "rate_dps": 2.0, "attitude": {"type": "normal+"}}]),
     "increase strictly"),
    (lambda s: s["ship"].update(attitude=[{"t_s": 0.0, "attitude": {"type": "prograde"}},
                                          {"t_s": 5.0, "rate_dps": 90.0, "attitude": {"type": "radial_in"}}]),
     "rate_dps must lie"),
    (lambda s: s["ship"].update(attitude=[{"t_s": 3.0, "attitude": {"type": "prograde"}}]), "at t_s = 0"),
    (lambda s: s["objects"].append(
        {"id": "D1", "class": "DEBRIS", "attitude": [{"t_s": 0.0, "attitude": {"type": "prograde"}}],
         "state": {"type": "relative", "range_km": 10.0, "phi_deg": 0.0, "eps_deg": 0.0, "v_close_mps": 0.0}}),
     "fly waypoints"),
])
def test_invalid_scenarios_are_explained(edit, words):
    """S1: each problem gives a clear message naming the element."""
    sc = _base()
    edit(sc)
    err = scn.validate(sc)
    assert any(words in e for e in err), err


def test_load_save_roundtrip(tmp_path):
    """S1: save then load gives the normalised scenario; the mission path becomes absolute; a bad file is refused
    with every problem listed."""
    sc = _base()
    sc["mission"] = "mission.json"
    p = tmp_path / "s.json"
    scn.save(sc, str(p))
    got = scn.load(str(p))
    assert got["mission"] == os.path.normpath(str(tmp_path / "mission.json"))
    assert got["objects"][0]["id"] == "K1" and got["format_version"] == scn.FORMAT_VERSION
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"meta": {}}), encoding="utf-8")
    with pytest.raises(scn.ScenarioError) as exc:
        scn.load(str(bad))
    assert "duration_s" in str(exc.value) and "ship" in str(exc.value)


def test_scenario_editor_forms(tmp_path):
    """S5: the editor builds, adds and removes elements and manoeuvres, switches an orbit type, keeps one form per
    page (no stale widgets) and writes a JSON that reads back identical."""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication, QComboBox
    app = QApplication.instance() or QApplication([])
    from gui.scenario_tab import FormWidget, ManoeuvreList, ScenarioTab
    tab = ScenarioTab()
    assert tab.messages.item(0).text() == "Scenario valid"
    tab.add_object()
    tab.add_group()
    assert [o["id"] for o in tab.sc["objects"]] == ["KKV-1", "OBJ-1"] and tab.sc["groups"][0]["id"] == "GRP-1"
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    ml = tab.form_holder.findChild(ManoeuvreList)
    for t in ("impulse", "burn", "singer", "pn"):
        ml._add(t)
    ml.list.setCurrentRow(0)
    ml._remove()
    assert [m["type"] for m in tab.sc["objects"][0]["manoeuvres"]] == ["burn", "singer", "pn"]
    tab.tree.setCurrentItem(tab.n_ship)
    tab.form_holder.findChildren(QComboBox)[0].setCurrentText("state")
    assert tab.sc["ship"]["orbit"]["type"] == "state"
    assert len(tab.form_holder.findChildren(FormWidget)) == 2
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    tab.delete()
    assert [o["id"] for o in tab.sc["objects"]] == ["OBJ-1"]
    tab.sc["groups"][0]["around"] = "OBJ-1"
    p = str(tmp_path / "e.json")
    tab._write(p)
    with open(p, encoding="utf-8") as fh:
        assert json.load(fh) == tab.sc
    assert not tab.modified
    app.processEvents()


# ----------------------------------------------------------------------------- propagation (pass S2)
import dataclasses  # noqa: E402
import math  # noqa: E402

import numpy as np  # noqa: E402

from core import radar_model as rm  # noqa: E402
from sim import propagate as prop  # noqa: E402

MU = 398600.4418


def _orbit_sc(duration: float, objects=None, groups=None, step: float = 1.0) -> dict:
    sc = {"meta": {"name": "p", "epoch_utc": "2050-01-01T00:00:00Z", "duration_s": duration, "step_s": step},
          "ship": {"orbit": {"type": "elements", "a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 30.0,
                             "argp_deg": 0.0, "nu_deg": 0.0}},
          "objects": objects or [], "groups": groups or []}
    assert scn.validate(sc) == []
    return scn.normalise(sc)


def _no_j2(monkeypatch):
    orig = prop.Body.of
    monkeypatch.setattr(prop.Body, "of", classmethod(lambda cls, n: dataclasses.replace(orig(n), j2=0.0)))


def test_two_body_period_and_energy(monkeypatch):
    """C1-C2: without J2 a circular orbit closes after 2 pi sqrt(a^3 / mu) (RK4 at 1 s: within 1 m) and the energy
    holds; elements -> state gives |r| = a, |v| = sqrt(mu / a)."""
    _no_j2(monkeypatch)
    T = 2 * math.pi * math.sqrt(7000.0 ** 3 / MU)
    sc = _orbit_sc(T)
    sc["meta"]["step_s"] = T / math.ceil(T)
    tr = prop.propagate(sc)
    r0, r1 = tr.r_km[0, 0], tr.r_km[-1, 0]
    assert np.linalg.norm(r0) == pytest.approx(7000.0) and np.linalg.norm(tr.v_kms[0, 0]) == pytest.approx(
        math.sqrt(MU / 7000.0))
    assert np.linalg.norm(r1 - r0) < 1e-3
    e = 0.5 * np.einsum("ij,ij->i", tr.v_kms[:, 0], tr.v_kms[:, 0]) - MU / np.linalg.norm(tr.r_km[:, 0], axis=1)
    assert np.ptp(e) / abs(e[0]) < 1e-10


def test_j2_nodal_regression():
    """C1: the node regresses at -3/2 n J2 (Re / p)^2 cos i (secular rate of the Earth's J2)."""
    b = prop.Body.of("earth")
    tr = prop.propagate(_orbit_sc(6 * 3600.0, step=2.0))

    def node(r, v):
        h = np.cross(r, v)
        return math.atan2(h[0], -h[1])

    w0, w1 = node(tr.r_km[0, 0], tr.v_kms[0, 0]), node(tr.r_km[-1, 0], tr.v_kms[-1, 0])
    n = math.sqrt(MU / 7000.0 ** 3)
    rate = -1.5 * n * b.j2 * (b.r_eq_km / 7000.0) ** 2 * math.cos(math.radians(51.6))
    assert (w1 - w0) / (6 * 3600.0) == pytest.approx(rate, rel=0.02)


def test_impulse_and_burn_change_the_velocity(monkeypatch):
    """C3: an impulse adds its dv at t_s (rtn: along track); a burn adds a x duration, the step split at its edges."""
    _no_j2(monkeypatch)
    o = lambda m: {"id": "A", "class": "GENERIC", "manoeuvres": [m],                  # noqa: E731
                   "state": {"type": "elements", "a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 30.0,
                             "argp_deg": 0.0, "nu_deg": 0.0}}
    tr = prop.propagate(_orbit_sc(10.0, [o({"type": "impulse", "t_s": 3.3, "dv_mps": [0.0, 10.0, 0.0]})], step=1.0))
    dv = np.linalg.norm(tr.v_kms[-1, 1]) - np.linalg.norm(tr.v_kms[-1, 0])
    assert dv * 1e3 == pytest.approx(10.0, rel=1e-3)
    tr = prop.propagate(_orbit_sc(10.0, [o({"type": "burn", "t_s": 2.5, "duration_s": 4.25, "accel_mps2": [0, 0, 2.0],
                                             "frame": "inertial"})], step=1.0))
    # the burning object drifts ~20 m from the ship: the gravity gradient adds ~2e-4 m/s
    assert (tr.v_kms[-1, 1, 2] - tr.v_kms[-1, 0, 2]) * 1e3 == pytest.approx(8.5, abs=5e-4)


def test_relative_state_groups_and_impact():
    """P6, P10, P11: a relative object starts range_km from the ship along its direction, closing at v_close; a group
    draws its members around the reference (seeded, same draw twice); an object aimed at the Earth impacts."""
    rel = {"id": "R", "class": "KKV", "t_appear_s": 5.0,
           "state": {"type": "relative", "range_km": 500.0, "phi_deg": 0.0, "eps_deg": 0.0, "v_close_mps": 3000.0,
                     "frame": "inertial"}}
    grp = {"id": "G", "class": "DECOY", "n": 50, "around": "R", "t_appear_s": 5.0, "pos_sigma_m": 1000.0,
           "vel_sigma_mps": 10.0}
    down = {"id": "D", "class": "DEBRIS", "state": {"type": "state", "r_km": [6400.0, 0.0, 0.0], "v_kms": [-5.0, 0.0, 0.0]}}
    sc = _orbit_sc(20.0, [rel, down], [grp])
    tr = prop.propagate(sc)
    j = int(np.flatnonzero(tr.t_s == 5.0)[0])
    i = tr.row("R")
    assert not tr.exists()[j - 1, i] and tr.exists()[j, i]
    d = tr.r_km[j, i] - tr.r_km[j, 0]
    assert np.linalg.norm(d) == pytest.approx(500.0, rel=1e-6) and d[0] / 500.0 == pytest.approx(1.0, abs=1e-6)
    assert ((tr.v_kms[j, i] - tr.v_kms[j, 0]) @ d / 500.0) * 1e3 == pytest.approx(-3000.0, rel=1e-6)
    m = [k for k, g in enumerate(tr.group_of) if g == "G"]
    off = (tr.r_km[j, m] - tr.r_km[j, i]) * 1e3
    assert off.std() == pytest.approx(1000.0, rel=0.25)
    assert np.array_equal(prop.propagate(sc).r_km, tr.r_km, equal_nan=True)
    k = tr.row("D")
    assert 0.0 < tr.t_impact_s[k] < 20.0 and not tr.exists()[-1, k]


def test_singer_statistics():
    """C4: the drawn Singer acceleration has the class's intensity (the velocity spread grows accordingly)."""
    o = [{"id": f"S{k}", "class": "KKV", "manoeuvres": [{"type": "singer"}],
          "state": {"type": "elements", "a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 30.0, "argp_deg": 0.0,
                    "nu_deg": 0.0}} for k in range(200)]
    tr = prop.propagate(_orbit_sc(2.0, o, step=0.1))
    t = rm.TARGETS["KKV"]
    s = math.sqrt(t.accel_max_mps2 ** 2 / 3.0 * (1.0 + 4.0 * prop.tm.P_MAX - prop.tm.P0))
    dv = (tr.v_kms[-1, 1:] - tr.v_kms[-1, 0]) * 1e3                   # ~ a x 2 s while tau >> 2 s
    assert dv.std() / 2.0 == pytest.approx(s, rel=0.15)


def test_pn_intercepts_the_ship():
    """C5: a KKV closing from 300 km with a 200 m/s transverse error and PN from 200 km passes within a few metres;
    without guidance it misses by ~ 200 m/s x range / v_close."""
    def sc(pn: bool) -> dict:
        o = {"id": "K", "class": "KKV", "manoeuvres": [{"type": "pn", "start_range_km": 200.0}] if pn else [],
             "state": {"type": "relative", "range_km": 300.0, "phi_deg": 40.0, "eps_deg": 10.0, "v_close_mps": 8000.0,
                       "v_transverse_mps": [200.0, 0.0], "frame": "inertial"}}
        return _orbit_sc(60.0, [o], step=0.01)
    tr = prop.propagate(sc(True))
    k = tr.row("K")
    assert tr.t_pn_start_s[k] == pytest.approx(12.5, abs=0.5) and tr.miss_km[k] < 0.01
    free = prop.propagate(sc(False))
    d = np.linalg.norm(free.r_km[:, k] - free.r_km[:, 0], axis=1)
    assert np.nanmin(d) > 5.0


# ----------------------------------------------------------------------------- attitudes by waypoints (pass S3a)
from sim import attitude as att  # noqa: E402


def _att_sc(duration: float, wps: list, objects=None, step: float = 0.1) -> dict:
    sc = _orbit_sc(duration, objects, step=step)
    sc["ship"]["attitude"] = wps
    assert scn.validate(sc) == []
    return scn.normalise(sc)


def _nose_off_deg(m: np.ndarray, los: np.ndarray) -> float:
    """Angle (deg) between the nose (Z) of an attitude and a direction."""
    return math.degrees(math.acos(float(np.clip(m[:, 2] @ los / np.linalg.norm(los), -1.0, 1.0))))


def _fixed(yaw: float = 0.0, frame: str = "inertial") -> dict:
    return {"type": "fixed", "frame": frame, "yaw_deg": yaw, "pitch_deg": 0.0, "roll_deg": 0.0}


def test_attitude_conventions():
    """Axes of the user: Z roll axis, XZ the horizontal; yaw about Y (nose toward +X), pitch about X (nose up toward
    +Y), roll about Z (+X wing down); the orbital reference X = -R, Y = N, Z = T; prograde with roll 0 is the orbital
    reference on a circular orbit, X in the orbital plane."""
    z, x = np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0])
    assert np.allclose(att.euler(90.0, 0.0, 0.0) @ z, [1.0, 0.0, 0.0])
    assert np.allclose(att.euler(0.0, 90.0, 0.0) @ z, [0.0, 1.0, 0.0])
    assert np.allclose(att.euler(0.0, 0.0, 90.0) @ x, [0.0, -1.0, 0.0])
    r, v = prop.elements_to_state({"a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 30.0, "argp_deg": 10.0,
                                   "nu_deg": 40.0}, MU)
    ref = att.orbital_reference(r, v)
    rr, t, n = att.orbital_axes(r, v)
    assert np.allclose(ref[:, 0], -rr) and np.allclose(ref[:, 1], n) and np.allclose(ref[:, 2], t)
    assert np.allclose(att.target_attitude({"type": "prograde", "roll_deg": 0.0}, r, v, None), ref, atol=1e-9)
    assert np.allclose(att.target_attitude(_fixed(0.0, "orbital"), r, v, None), ref)
    nrm = att.target_attitude({"type": "normal+", "roll_deg": 0.0}, r, v, None)
    assert np.allclose(nrm[:, 2], n) and abs(nrm[:, 0] @ n) < 1e-12 and np.allclose(nrm[:, 0], t)


def test_slew_is_the_trapezoid():
    """G1: a 60 deg slew at 10 deg/s with 15 deg/s2 lasts 60 / 10 + 10 / 15 = 6.67 s from t_s, the rate stays at most
    rate_dps, then the attitude is held."""
    tr = prop.propagate(_att_sc(12.0, [{"t_s": 0.0, "attitude": _fixed()},
                                       {"t_s": 1.0, "rate_dps": 10.0, "attitude": _fixed(60.0)}]))
    goal = att.euler(60.0, 0.0, 0.0)
    ts = np.arange(0.0, 12.0, 0.1)
    err = np.array([att.angle_deg(goal, tr.attitude("ship", t)) for t in ts])
    t_done = ts[np.argmax(err < 0.05)]
    assert t_done == pytest.approx(1.0 + 60.0 / 10.0 + 10.0 / 15.0, abs=0.3)
    rate = -np.diff(err) / 0.1
    assert rate.max() <= 10.0 + 1e-6 and abs(err[ts <= 1.0]).max() == pytest.approx(60.0, abs=1e-6)
    assert err[-1] < att.REACHED_TOL_DEG and not tr.att_events


def test_target_tracking_unfinished_slew_and_ship_frame_state():
    """G1-G2: the ship turns its nose toward a target and follows it; a slew still running when the next waypoint
    starts is logged; a relative state in the ship frame is resolved with the ship's attitude."""
    obj = {"id": "T", "class": "SHIP", "state": {"type": "relative", "range_km": 200.0, "phi_deg": 0.0, "eps_deg": 0.0,
                                                  "v_close_mps": 0.0, "v_transverse_mps": [300.0, 0.0],
                                                  "frame": "inertial"}}
    tr = prop.propagate(_att_sc(30.0, [{"t_s": 0.0, "attitude": _fixed()},
                                       {"t_s": 1.0, "rate_dps": 20.0, "attitude": {"type": "target", "id": "T"}}],
                                [obj]))
    i = tr.row("T")
    for t in (20.0, 29.0):
        k = int(np.flatnonzero(tr.t_s == t)[0])
        los = tr.r_km[k, i] - tr.r_km[k, 0]
        assert _nose_off_deg(tr.attitude("ship", t), los) < att.HOLD_TOL_DEG
    tr = prop.propagate(_att_sc(10.0, [{"t_s": 0.0, "attitude": _fixed()},
                                       {"t_s": 1.0, "rate_dps": 1.0, "attitude": _fixed(170.0)},
                                       {"t_s": 5.0, "rate_dps": 1.0, "attitude": _fixed(0.0)}]))
    ev = [e for e in tr.att_events if e["kind"] == "slew not finished"]
    assert len(ev) == 1 and ev[0]["t_s"] == pytest.approx(5.0) and ev[0]["error_deg"] > 160.0
    rel = {"id": "R", "class": "DEBRIS", "t_appear_s": 2.0,
           "state": {"type": "relative", "range_km": 100.0, "phi_deg": 0.0, "eps_deg": 90.0, "v_close_mps": 0.0}}
    tr = prop.propagate(_att_sc(3.0, [{"t_s": 0.0, "attitude": _fixed(90.0)}], [rel]))
    k = int(np.flatnonzero(tr.t_s == 2.0)[0])
    d = tr.r_km[k, tr.row("R")] - tr.r_km[k, 0]
    assert np.allclose(d / 100.0, [1.0, 0.0, 0.0], atol=1e-6)        # ship +Z = ICRF +X after a 90 deg yaw


def test_object_flies_waypoints_toward_the_ship():
    """G2 bis: a KKV flies waypoints too (the ownship's limits); aimed at "ship", its nose follows the ship."""
    k = {"id": "K", "class": "KKV",
         "state": {"type": "relative", "range_km": 300.0, "phi_deg": 30.0, "eps_deg": 10.0, "v_close_mps": 5000.0,
                   "frame": "inertial"},
         "attitude": [{"t_s": 0.0, "attitude": _fixed()},
                      {"t_s": 0.5, "rate_dps": 30.0, "attitude": {"type": "target", "id": "ship"}}]}
    tr = prop.propagate(_att_sc(20.0, [{"t_s": 0.0, "attitude": {"type": "prograde"}}], [k]))
    j = tr.row("K")
    kk = int(np.flatnonzero(tr.t_s == 15.0)[0])
    los = tr.r_km[kk, 0] - tr.r_km[kk, j]
    assert "K" in tr.att_ids and _nose_off_deg(tr.attitude("K", 15.0), los) < att.HOLD_TOL_DEG


# ----------------------------------------------------------------------------- geometry (pass S3b)
from core import environment as env  # noqa: E402
from core import optimizer as opt  # noqa: E402
from core import frames  # noqa: E402
from sim import ephemeris as eph  # noqa: E402
from sim import geometry as geo  # noqa: E402


def test_face_of_rule():
    """G3: a face normal is owned by its face at theta 0; (u, v, w) rebuild the direction in the owner's frame; the
    sphere raster of the tool uses the same rule."""
    for k, pk in enumerate(opt.FACE_PHI):
        n = np.array(opt.face_frame(pk)[0])
        o, w, u, v = opt.face_of(n[0], n[1], n[2])
        assert int(o) == k and float(w) == pytest.approx(1.0) and abs(float(u)) < 1e-12 and abs(float(v)) < 1e-12
    d = np.random.default_rng(3).standard_normal((2000, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    o, w, u, v = opt.face_of(d[:, 0], d[:, 1], d[:, 2])
    for k, pk in enumerate(opt.FACE_PHI):
        n, eu, ev = (np.array(a) for a in opt.face_frame(pk))
        s = o == k
        assert np.allclose(u[s, None] * eu + v[s, None] * ev + w[s, None] * n, d[s], atol=1e-12)
    assert np.isnan(u[o < 0]).all() and (w[o < 0] == 0.0).all()
    sp = opt.Sphere(res_deg=3.0)
    p, e = np.radians(sp.phi), np.radians(sp.eps)
    assert np.array_equal(sp.owner, opt.face_of(np.cos(e) * np.cos(p), np.cos(e) * np.sin(p), np.sin(e))[0])


def test_hermite_states_between_records(monkeypatch):
    """G4: the cubic Hermite between the 1-s records stays within 1 m of the truth at every 0.1-s step (ship in orbit,
    KKV under proportional navigation)."""
    o = {"id": "K", "class": "KKV", "manoeuvres": [{"type": "pn", "start_range_km": 200.0}],
         "state": {"type": "relative", "range_km": 300.0, "phi_deg": 40.0, "eps_deg": 10.0, "v_close_mps": 8000.0,
                   "v_transverse_mps": [200.0, 0.0], "frame": "inertial"}}
    sc = _orbit_sc(40.0, [o], step=0.1)
    tr = prop.propagate(sc)
    monkeypatch.setattr(prop, "LOG_STEP_S", 0.1)
    ref = prop.propagate(sc)
    err = 0.0
    for k, t in enumerate(ref.t_s):
        r, _ = geo.states_at(tr, float(t))
        ok = np.isfinite(ref.r_km[k, :, 0])
        err = max(err, float(np.nanmax(np.linalg.norm(r[ok] - ref.r_km[k, ok], axis=1))))
    assert err * 1e3 < 1.0


def test_aspect_rcs_direction_and_structure():
    """G5-G7: a KKV closing on the ship (nose on its relative velocity) is seen nose on, its RCS the class table at 0
    deg plus its offset; its direction in the ship frame is the one it was placed at; the structure loss is the face's
    blockage table; a GENERIC points its nose along its inertial velocity; a DEBRIS tumbles at its rotation rate; drawn
    offsets are reproducible and within the truncation."""
    objs = [{"id": "K", "class": "KKV", "rcs_offset_db": 1.5,
             "state": {"type": "relative", "range_km": 500.0, "phi_deg": 45.0, "eps_deg": 5.0, "v_close_mps": 3000.0}},
            {"id": "G", "class": "GENERIC",
             "state": {"type": "relative", "range_km": 800.0, "phi_deg": 200.0, "eps_deg": 20.0, "v_close_mps": 500.0,
                       "v_transverse_mps": [300.0, -100.0]}},
            {"id": "D", "class": "DEBRIS",
             "state": {"type": "relative", "range_km": 100.0, "phi_deg": 120.0, "eps_deg": -10.0, "v_close_mps": 0.0}}]
    sc = _orbit_sc(40.0, objs, step=0.1)
    tr = prop.propagate(sc)
    g = geo.geometry(sc, tr, np.array([0.0, 4.0, 36.0]))
    k, gi, d = (g.ids.index(x) for x in ("K", "G", "D"))
    assert g.aspect_deg[0, k] < 1e-6 and g.range_rate_mps[0, k] == pytest.approx(-3000.0, abs=1.0)
    tgt = rm.target_for_sector({"id": "K", "target_type": "KKV"})
    assert g.rcs_s_dbsm[0, k] == pytest.approx(tgt.aspect_table_db(g.f_s_ghz)[0] + 1.5)
    assert g.rcs_ku_dbsm[0, k] == pytest.approx(tgt.aspect_table_db(g.f_ku_ghz)[0] + 1.5)
    assert np.allclose(g.dir_ship[0, k], frames.unit(45.0, 5.0), atol=1e-9) and g.range_km[0, k] == pytest.approx(500.0)
    for j in (k, gi, d):
        if g.face[0, j] >= 0:
            f, _ = env.blockage(int(g.face[0, j]), g.dir_ship[0, j][None, :], "S")
            assert g.block_f[0, j] == pytest.approx(float(f[0]))
            assert g.masked[0, j] == (g.block_loss_db[0, j] > env.BLOCK_MASK_DB)
    r, v = geo.states_at(tr, 4.0)
    j = tr.row("G")
    want = math.degrees(math.acos(float(v[j] @ (r[0] - r[j]) / np.linalg.norm(v[j]) / np.linalg.norm(r[0] - r[j]))))
    assert g.aspect_deg[1, gi] == pytest.approx(want, abs=1e-6)
    assert rm.TARGETS["DEBRIS"].rotation_dps == pytest.approx(10.0)
    assert g.aspect_deg[2, d] == pytest.approx(g.aspect_deg[0, d], abs=0.5)     # one turn in 36 s
    lim = geo.RCS_TRUNC_SIGMAS * np.array([rm.target_for_sector({"id": x, "target_type": c}).rcs_spread_db
                                           for x, c in zip(g.ids, g.classes)])
    assert g.rcs_offset_db[k] == 1.5 and (np.abs(g.rcs_offset_db) <= lim + 1e-12).all()
    assert np.array_equal(geo.geometry(sc, tr, np.array([0.0])).rcs_offset_db, g.rcs_offset_db)


def test_occultation_and_shadow():
    """G7-G8: an object on the far side of the Earth is occulted by it, a near one is not; a ship behind the Earth
    from the Sun is in the umbra (the whole solar disk hidden), on the Sun side it is lit."""
    r0, v0 = prop.elements_to_state({"a_km": 7000.0, "e": 0.0, "i_deg": 51.6, "raan_deg": 30.0, "argp_deg": 0.0,
                                     "nu_deg": 0.0}, MU)
    objs = [{"id": "FAR", "class": "ST", "state": {"type": "state", "r_km": list(-r0), "v_kms": list(-v0)}},
            {"id": "NEAR", "class": "ST", "state": {"type": "relative", "range_km": 100.0, "phi_deg": 0.0,
                                                    "eps_deg": 0.0, "v_close_mps": 0.0}}]
    sc = _orbit_sc(2.0, objs)
    g = geo.geometry(sc, prop.propagate(sc))
    assert g.occulted_by[0, g.ids.index("FAR")] == geo.BODIES.index("earth")
    assert g.occulted_by[0, g.ids.index("NEAR")] == -1
    T = eph.centuries_tt(scn.epoch(sc), 0.0)
    su = eph.sun_icrf_km(T)
    su /= np.linalg.norm(su)
    side = np.cross(su, [0.0, 0.0, 1.0])
    side /= np.linalg.norm(side)
    vc = math.sqrt(MU / 7000.0)
    for sign, want in ((-1.0, 1.0), (1.0, 0.0)):
        s = _orbit_sc(2.0)
        s["ship"]["orbit"] = {"type": "state", "r_km": list(sign * 7000.0 * su), "v_kms": list(vc * side)}
        gg = geo.geometry(s, prop.propagate(s))
        assert gg.ship_shadow[0] == pytest.approx(want)
        assert gg.body_radius_deg[0, 0] == pytest.approx(0.27, abs=0.01)


def test_ephemerides_against_meeus():
    """G8: Meeus, examples 25.a (Sun, 1992 October 13.0 TD: longitude 199.90988 deg, R 0.99766 AU) and 47.a (Moon,
    1992 April 12.0 TD: longitude 133.162655 deg, latitude -3.229126 deg, distance 368409.7 km); the precession
    matrix is the identity at J2000 and a rotation."""
    T = (2448908.5 - 2451545.0) / 36525.0
    lon, lat, d = eph.sun_ecliptic(T)
    assert lon == pytest.approx(199.90988, abs=2e-5) and d / eph.AU_KM == pytest.approx(0.99766, abs=1e-5)
    T = (2448724.5 - 2451545.0) / 36525.0
    lon, lat, d = eph.moon_ecliptic(T)
    assert lon == pytest.approx(133.162655, abs=1e-4) and lat == pytest.approx(-3.229126, abs=3e-3)
    assert d == pytest.approx(368409.7, abs=0.5)
    assert np.allclose(eph.precession_to_j2000(0.0), np.eye(3))
    m = eph.precession_to_j2000(0.5)
    assert np.allclose(m @ m.T, np.eye(3), atol=1e-12)
