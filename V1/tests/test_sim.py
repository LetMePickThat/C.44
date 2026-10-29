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
    assert n["objects"][0]["identity"] == n["groups"][0]["identity"] == "pending"   # default identity
    n["groups"][0]["identity"] = "hostile"
    assert scn.identity_of(n, "x#0", n["groups"][0]["id"]) == "hostile"            # a member: its group's
    assert scn.identity_of(n, n["objects"][0]["id"]) == "pending"
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
    (lambda s: s["objects"][0].update(identity="foe"), "identity must be"),
    (lambda s: s["groups"][0].update(identity="ally"), "identity must be"),
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
    tab = ScenarioTab(view=False)
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
    assert len(tab.form_holder.findChildren(FormWidget)) == 3      # orbit, waypoint time, waypoint attitude
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    tab.delete()
    assert [o["id"] for o in tab.sc["objects"]] == ["OBJ-1"]
    tab.sc["groups"][0]["around"] = "OBJ-1"
    p = str(tmp_path / "e.json")
    scn.save(tab.sc, p)                               # (the window writes it in the single mission file)
    with open(p, encoding="utf-8") as fh:
        assert json.load(fh) == tab.sc
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
    """Pure two-body motion: no J2 and no attraction of the other bodies."""
    orig = prop.Body.of
    monkeypatch.setattr(prop.Body, "of", classmethod(lambda cls, n: dataclasses.replace(orig(n), j2=0.0)))
    monkeypatch.setattr(prop, "THIRD_BODY", False)


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


def test_slew_is_the_trapezoid(monkeypatch):
    """G1: a 60 deg slew at 10 deg/s with 15 deg/s2 lasts 60 / 10 + 10 / 15 = 6.67 s from t_s, the rate stays at most
    rate_dps, then the attitude is held (the attitudes recorded every 0.1 s here: the record step, 1 s by default,
    would hide the slew's timing)."""
    monkeypatch.setattr(prop, "LOG_STEP_S", 0.1)
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


# ----------------------------------------------------------------------------- navigation, sky, journal (pass S4)
from core import sky as skym  # noqa: E402
from sim import engine  # noqa: E402
from sim import run as simrun  # noqa: E402

MISSION = os.path.join(os.path.dirname(__file__), "missions", "mission_example.json")


def test_navigation_without_errors_is_the_truth():
    """N1: without errors the message at t is the truth (attitude, state), every 1 / nav_rate_hz; the angular rate is
    the slew's (10 deg/s cruise about the ship's Y axis)."""
    sc = _att_sc(10.0, [{"t_s": 0.0, "attitude": _fixed()},
                        {"t_s": 1.0, "rate_dps": 10.0, "attitude": _fixed(60.0)}])
    tr = prop.propagate(sc)
    nav = engine.Navigation(sc, tr)
    ts = nav.times(0.0, 1.0)
    assert len(ts) == int(engine.NAV_RATE_HZ) + 1 and ts[1] == pytest.approx(1.0 / engine.NAV_RATE_HZ)
    msg = nav.message(4.23)
    assert frames.validate_nav([msg]) == []
    m = frames.quat_to_matrix(msg["q_ship_to_icrf"])
    assert att.angle_deg(m, tr.attitude("ship", 4.23)) < 1e-9
    r, v = geo.states_at(tr, 4.23)
    assert np.allclose(msg["r_km"], r[0], atol=1e-12) and np.allclose(msg["v_kms"], v[0], atol=1e-15)
    w = np.array(msg["w_ship_dps"])
    assert np.linalg.norm(w) == pytest.approx(10.0, rel=1e-3) and abs(abs(w[1]) - 10.0) < 0.01


def test_navigation_errors_statistics_and_latency():
    """N2-N3: with meta.nav_errors the attitude and rate errors are white with the sigmas of [repointing], the
    position and velocity offsets constant over the run, the message stamped t describes t - nav_latency_s; the same
    seed gives the same stream."""
    sc = _att_sc(30.0, [{"t_s": 0.0, "attitude": {"type": "prograde"}}])
    sc["meta"]["nav_errors"] = True
    tr = prop.propagate(sc)
    nav = engine.Navigation(sc, tr)
    ts = nav.times(1.0, 21.0)
    da, dw, dr = [], [], []
    for t in ts:
        m, w, r, _ = nav.state(float(t))
        tt = float(t) - engine.NAV_LATENCY_S
        da.append(att.rotvec(tr.attitude("ship", tt).T @ m))
        dw.append(w - engine.ship_rate_dps(tr, tt))
        dr.append(r - geo.states_at(tr, tt)[0][0])
    da, dw, dr = np.array(da), np.array(dw), np.array(dr)
    assert da.std(axis=0) == pytest.approx([engine.ATT_SIGMA_RAD] * 3, rel=0.1)
    assert dw.std(axis=0) == pytest.approx([engine.RATE_SIGMA_DPS] * 3, rel=0.1)
    assert np.allclose(dr, dr[0], atol=1e-12) and np.allclose(dr[0], nav.dr_km)
    assert 0.0 < np.linalg.norm(nav.dr_km) * 1e3 < 5.0 * engine.POS_SIGMA_KM * 1e3
    assert engine.Navigation(sc, tr).message(5.0) == nav.message(5.0)


def test_sky_and_mission_at_t():
    """N4-N9: the sky at t is a valid mission sky (Sun, Earth, Moon, galactic frame, the stand-off jammer at its range),
    the Earth's angular radius and surface Doppler of a circular orbit; the mission at t passes the mission-preparation
    checks with no sky / navigation alert."""
    sh = {"id": "EW", "class": "SHIP", "jammer": "SOJ_SHIP",
          "state": {"type": "relative", "range_km": 900.0, "phi_deg": 30.0, "eps_deg": 40.0, "v_close_mps": 0.0}}
    sc = _orbit_sc(5.0, [sh])
    sc["mission"] = MISSION
    tr = prop.propagate(sc)
    sky = engine.sky_at(sc, tr, 2.0)
    assert skym.validate(sky) == []
    b = {x["id"]: x for x in sky["bodies"]}
    assert {"SUN", "EARTH"} <= set(b) and b["SUN"]["type"] == "sun" and b["EARTH"]["type"] == "earth"
    assert b["EARTH"]["angular_radius_deg"] == pytest.approx(math.degrees(math.asin(6371.0 / 7000.0)), abs=0.05)
    assert abs(b["EARTH"]["closing_mps"]) < 20.0
    assert b["EARTH"]["closing_spread_mps"] == pytest.approx(7546.0 * 6371.0 / 7000.0, rel=0.01)
    assert sky["jammers"][0]["id"] == "EW" and sky["jammers"][0]["range_km"] == pytest.approx(900.0, rel=1e-3)
    m, alerts = engine.mission_at(sc, tr, 2.0)
    assert frames.validate(m) == [] and not [a for a in alerts if "deg from" in a]
    assert m["global"]["nav"][0]["central_body"] == "earth" and m["global"]["sky"] == sky


def test_journal_roundtrip_and_summary(tmp_path):
    """N10-N13: a run writes the journal and the summary next to the scenario; the journal reads back the scenario,
    the truth (exactly), the geometry (float32) and the 100 Hz navigation; the summary agrees with the geometry."""
    o = {"id": "K", "class": "KKV", "state": {"type": "relative", "range_km": 50.0, "phi_deg": 45.0, "eps_deg": 5.0,
                                              "v_close_mps": 3000.0}}
    sc = _orbit_sc(20.0, [o], step=0.1)
    sc["meta"]["log_nav"] = True
    path = str(tmp_path / "run1.json")
    scn.save(sc, path)
    s = simrun.run(path)
    npz, js = engine.journal_paths(path)
    assert os.path.exists(npz) and json.load(open(js, encoding="utf-8"))["name"] == s["name"]
    sc2, tr, g, nav, meta = engine.load_journal(npz)
    ref = prop.propagate(scn.load(path))
    assert sc2["meta"]["name"] == sc["meta"]["name"] and meta["params_hash"] == engine.params_hash()
    assert np.array_equal(tr.r_km, ref.r_km, equal_nan=True) and tr.ids == ref.ids and tr.att_ids == ref.att_ids
    g_ref = geo.geometry(scn.load(path), ref)
    assert np.allclose(g.range_km, g_ref.range_km, rtol=1e-6, equal_nan=True)
    assert np.array_equal(g.face, g_ref.face) and g.ids == g_ref.ids
    assert len(nav["t_s"]) == int(20.0 * engine.NAV_RATE_HZ) + 1
    assert np.allclose(nav["r_km"][-1], ref.r_km[-1, 0])
    k = g.ids.index("K")
    ok = s["objects"][k]
    assert ok["range_min_km"] == pytest.approx(float(np.nanmin(g.range_km[:, k])), abs=1e-3)   # summary in m
    w = engine._weights(g.t_s)
    assert sum(ok["time_per_face_s"].values()) + ok["time_no_face_s"] == pytest.approx(float(w[g.exists[:, k]].sum()))


# ----------------------------------------------------------------------------- waypoint editor (W), 3D views and playback (S6, S7)
def _qt():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_waypoint_editor():
    """W1-W3: the ship's waypoints are a list kept sorted by time (the first without a slew rate); an object of class
    KKV flies waypoints on demand (toward the ship by default) and goes back to its class attitude without them."""
    app = _qt()
    from PyQt5.QtWidgets import QCheckBox
    from gui.scenario_tab import FormWidget, ScenarioTab, WaypointList
    tab = ScenarioTab(view=False)
    tab.tree.setCurrentItem(tab.n_ship)
    wl = tab.form_holder.findChild(WaypointList)
    wl._add()
    wl._add()
    assert [w["t_s"] for w in tab.sc["ship"]["attitude"]] == [0.0, 10.0, 20.0]
    form = wl.form_area.itemAt(0).widget()
    assert isinstance(form, FormWidget)
    form.widgets["t_s"][1].setValue(5.0)                 # the last point moved before the second
    ts = [w["t_s"] for w in tab.sc["ship"]["attitude"]]
    assert ts == [0.0, 5.0, 10.0] and "rate_dps" not in tab.sc["ship"]["attitude"][0]
    assert all("rate_dps" in w for w in tab.sc["ship"]["attitude"][1:])
    assert scn.validate(tab.sc) == []
    tab.add_object()                                    # KKV-1
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    cb = [c for c in tab.form_holder.findChildren(QCheckBox) if "waypoints" in c.text()][0]
    cb.setChecked(True)
    o = tab.sc["objects"][0]
    assert o["attitude"][0]["attitude"] == {"type": "target", "id": "ship", "roll_deg": 0.0}
    assert scn.validate(tab.sc) == []
    cb.setChecked(False)
    assert "attitude" not in o
    app.processEvents()


def test_sim_view_offscreen(tmp_path):
    """S6-S7: the view shows a propagated run in both views (Earth, Moon and Sun at their places, trajectories,
    objects, lines of sight, the linked mission's volumes), centres on a body (OWNSHIP or an object) and zooms,
    lists the events, plays and steps, informs on an object and the ship, computes in the background and replays a
    journal."""
    app = _qt()
    from gui.sim_view import CENTRAL, DYN, OWNSHIP, VOL, SimView
    objs = [{"id": "K", "class": "KKV", "t_appear_s": 2.0, "manoeuvres": [{"type": "pn", "start_range_km": 30.0}],
             "state": {"type": "relative", "range_km": 60.0, "phi_deg": 45.0, "eps_deg": 5.0, "v_close_mps": 4000.0}},
            {"id": "D", "class": "DEBRIS", "state": {"type": "relative", "range_km": 300.0, "phi_deg": 200.0,
                                                     "eps_deg": -10.0, "v_close_mps": 0.0}}]
    sc = _orbit_sc(20.0, objs, step=0.1)
    tr = prop.propagate(sc)
    g = geo.geometry(sc, tr)
    v = SimView()
    from core import mission as cm
    v.set_mission(cm.load_mission(MISSION))           # D: the mission being edited, drawn in the ship view
    v.show_run(sc, tr, g, "test")
    names = list(v.plotter.actors)
    assert any(n.startswith("traj_") for n in names) and DYN + "pts" in names
    assert {"body_earth", "body_earth_atm", "body_moon", "body_sun", "moon_orbit"} <= set(names)
    assert DYN + "sun" not in names                                    # no Sun arrow (the Sun is drawn)
    atm = v.plotter.actors["body_earth_atm"].user_matrix                 # the halo on the Earth, a bit larger
    assert np.allclose(atm[:3, 3], v.plotter.actors["body_earth"].user_matrix[:3, 3])
    u = v.plotter.actors["body_moon"].user_matrix
    assert np.linalg.norm(u[:3, 3]) == pytest.approx(np.linalg.norm(eph.moon_icrf_km(
        eph.centuries_tt(scn.epoch(sc), v.t))), rel=1e-9)                # the Moon at its true place
    T = eph.centuries_tt(scn.epoch(sc), v.t)
    assert np.allclose(v.plotter.actors["body_sun"].user_matrix[:3, 3], eph.sun_icrf_km(T), rtol=1e-9)   # 1 AU
    v.set_centre("Sun")
    assert np.allclose(v.plotter.camera.GetFocalPoint(), eph.sun_icrf_km(T), rtol=1e-9)
    v.set_centre(OWNSHIP)
    r0 = geo.states_at(tr, v.t)[0][0]
    assert np.allclose(v.plotter.camera.GetFocalPoint(), r0, atol=1e-3)
    d0 = v.plotter.camera.GetDistance()
    v.zoom(2.0)
    assert v.plotter.camera.GetDistance() == pytest.approx(d0 / 2.0, rel=1e-6)
    v.set_centre(CENTRAL)
    assert np.allclose(v.plotter.camera.GetFocalPoint(), 0.0, atol=1e-6)
    v._select(OWNSHIP)
    assert "altitude" in v.info.toPlainText()
    texts = [x for _t, x in v.events]
    assert any("K appears" in x for x in texts) and any("terminal guidance" in x for x in texts)
    assert len(v.slider._marks) == len(v.events) and v.slider._clusters()        # L4: marks on the slider
    v.tg_view.setChecked(True)                           # ship view
    v.set_time(10.0)
    names = list(v.plotter.actors)
    assert DYN + "los" in names and any(n.startswith(VOL) for n in names)
    v.cb_vectors.setChecked(True)
    assert DYN + "nose" in v.plotter.actors
    v.set_centre("K")                     # ship view centred on an object
    assert np.allclose(v.plotter.camera.GetFocalPoint(), v._screen_pts["K"], atol=1e-9)
    v.set_centre(CENTRAL)
    # range rings of about constant size on screen, their values following the zoom
    rings = v._lab[DYN + "rings_lbl"]
    before = list(rings._last)
    assert before and all(t.endswith(" km") for t in before)
    from gui import sim_view as sv_
    # log scale, range zoom: the camera keeps its distance, the scale follows the zoom (values, objects)
    d_log = v.plotter.camera.GetDistance()
    k_pos = np.array(v._screen_pts["K"])
    v.zoom(4.0)
    app.processEvents()
    assert v.plotter.camera.GetDistance() == pytest.approx(d_log, rel=1e-9) and rings._last != before
    assert np.linalg.norm(v._screen_pts["K"]) > np.linalg.norm(k_pos) * 1.05      # K drawn farther (fewer decades)
    n_many = 0
    for z in (1.0, 1.7, 0.45, 2.9, 3.3, 0.2):           # the rings equally spaced at any zoom, filling the view
        v.zoom(z)
        pts = v._dyn[DYN + "rings"][1].points
        rad = np.linalg.norm(pts[::sv_.RING_POINTS], axis=1)
        n_many += len(rad) >= 3
        assert len(rad) >= 3 and np.allclose(np.diff(rad), np.diff(rad)[0], rtol=1e-6)
        assert min(float(t.split()[0]) for t in rings._last) >= sv_.MIN_RING_KM
    v.zoom(1e-6)                                         # far zoomed out: the view's farthest ring at MAX_VIEW_KM
    assert v._ship_range(1e6) is None
    half, _, _, cover = v._ring_frame()
    assert v._ship_range(cover) == pytest.approx(sv_.MAX_VIEW_KM, rel=1e-6)
    assert max(float(t.split()[0]) for t in rings._last) <= sv_.MAX_VIEW_KM * (1 + 1e-9)
    assert len(v._dyn[DYN + "rings"][1].points) >= 3 * sv_.RING_POINTS
    v.zoom(1e9)                                          # far zoomed in: the half-height at the smallest span
    assert v._log_span == pytest.approx(sv_.LOG_SPAN_MIN)
    assert min(float(t.split()[0]) for t in rings._last) >= sv_.MIN_RING_KM
    assert v.plotter.camera.GetDistance() == pytest.approx(d_log, rel=1e-9)
    # 1:1 scale: an object at its true range (km) from the ship
    v.cb_true.setChecked(True)
    v.set_time(10.0)
    r, _ = geo.states_at(tr, 10.0)
    assert np.linalg.norm(v._screen_pts["K"]) == pytest.approx(np.linalg.norm(r[tr.row("K")] - r[0]), rel=1e-9)
    assert any(n.startswith(VOL) for n in v.plotter.actors) and DYN + "ship_axes" in v.plotter.actors
    v.cb_bodies.setChecked(True)                         # celestial objects at 1:1: the Moon at its true range
    u = v.plotter.actors[DYN + "sbody_moon"].user_matrix
    T = eph.centuries_tt(scn.epoch(sc), 10.0)
    assert np.linalg.norm(u[:3, 3]) == pytest.approx(np.linalg.norm(eph.moon_icrf_km(T) - r[0]), rel=1e-6)
    v.cb_true.setChecked(False)
    assert v.plotter.actors[DYN + "sbody_sun"].GetVisibility()
    v.cb_bodies.setChecked(False)
    assert not v.plotter.actors[DYN + "sbody_sun"].GetVisibility()
    # mouse rotation by the virtual trackball: a vertical drag right of the centre (outside its circle) turns the view
    # about the line of sight; a horizontal drag through the centre turns the scene about the screen's vertical
    iren = v.plotter.iren.interactor
    v.plotter.ren_win.SetSize(900, 700)                 # offscreen: the window has no size of its own
    w, h = v.plotter.ren_win.GetSize()
    assert w > 0 and h > 0
    cam = v.plotter.camera

    def drag(p0, p1):
        iren.SetEventInformation(int(p0[0]), int(p0[1]), 0, 0)
        iren.InvokeEvent("LeftButtonPressEvent")
        for a in np.linspace(0.0, 1.0, 6)[1:]:
            iren.SetEventInformation(int(p0[0] + a * (p1[0] - p0[0])), int(p0[1] + a * (p1[1] - p0[1])), 0, 0)
            iren.InvokeEvent("MouseMoveEvent")
        iren.InvokeEvent("LeftButtonReleaseEvent")
    d0, up0 = np.array(cam.GetDirectionOfProjection()), np.array(cam.GetViewUp())
    drag((0.97 * w, 0.4 * h), (0.97 * w, 0.6 * h))      # far right of the centre: mostly a roll
    d1, up1 = np.array(cam.GetDirectionOfProjection()), np.array(cam.GetViewUp())
    tilt = math.degrees(math.acos(np.clip(d0 @ d1, -1, 1)))
    roll = math.degrees(math.acos(np.clip(up0 @ up1, -1, 1)))
    assert roll > 5.0 and roll > 3.0 * tilt
    # azimuth spokes every 45 deg in the rings' plane, from the ship's centre, labelled 0..315 deg
    v._draw_frame()
    spk = v._dyn[DYN + "spokes"]
    assert spk[2].GetVisibility() and len(spk[1].points) == 16
    ends = spk[1].points[8:]
    assert np.allclose(ends[:, 2], 0.0) and np.allclose(spk[1].points[:8], 0.0)
    ang = np.degrees(np.arctan2(ends[:, 1], ends[:, 0])) % 360.0
    assert np.allclose(ang, np.arange(0.0, 360.0, 45.0), atol=1e-6)
    assert v._lab[DYN + "spokes_lbl"]._last == [f"{a}°" for a in range(0, 360, 45)]
    # the rotations turn about the ship's centre, the centre of the rings; "Recentre" undoes a pan
    assert np.allclose(cam.GetFocalPoint(), 0.0, atol=1e-9)
    ring = v._dyn[DYN + "rings"][1].points
    assert np.allclose(ring[:sv_.RING_POINTS - 1].mean(axis=0), 0.0, atol=1e-6 * np.abs(ring).max())
    pan = np.array([0.3, -0.2, 0.1])                    # a pan: the camera and its focal point moved together
    cam.SetFocalPoint(*(np.array(cam.GetFocalPoint()) + pan))
    cam.SetPosition(*(np.array(cam.GetPosition()) + pan))
    v.recentre()
    assert np.allclose(cam.GetFocalPoint(), 0.0, atol=1e-9) and np.allclose(v.plotter._pivot, 0.0)
    d1, up1 = np.array(cam.GetDirectionOfProjection()), np.array(cam.GetViewUp())
    # Bell's trackball: no step from the ball to the roll (the mapped point moves little for one pixel anywhere)
    from gui import scene_view as scv
    st = scv._ARC[id(v.plotter.iren.style)]
    pts = np.array([scv._arc_ball(st, x, 0.5 * h) for x in np.arange(0.5 * w, w, 1.0)])
    assert np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", pts[1:], pts[:-1]), -1, 1))).max() < 0.5
    drag((0.5 * w - 0.1 * h, 0.5 * h), (0.5 * w + 0.1 * h, 0.5 * h))
    d2, up2 = np.array(cam.GetDirectionOfProjection()), np.array(cam.GetViewUp())
    roll = math.degrees(math.acos(np.clip(up1 @ up2, -1, 1)))      # (pixel rounding of the events: ~0.03 deg)
    assert math.degrees(math.acos(np.clip(d1 @ d2, -1, 1))) > 5.0 and roll < 0.5
    v._select("K")
    assert "range" in v.info.toPlainText() and "RCS S" in v.info.toPlainText()
    v.step(1)
    assert v.t == pytest.approx(11.0)
    v.toggle_play()
    v._wall -= 0.1
    v._tick()
    v.toggle_play()
    assert v.t > 11.0 and not v.timer.isActive()
    v.tg_view.setChecked(False)
    v.pending = scn.normalise(sc)
    v.propagate()
    v.worker.wait()
    app.processEvents()
    assert v.status.text().startswith("propagation")
    npz = str(tmp_path / "j.npz")
    engine.save_journal(npz, sc, tr, g)
    sc2, tr2, g2, _n, _m = engine.load_journal(npz)
    v.show_run(sc2, tr2, g2, "journal")
    assert v.g.ids == g.ids
    app.processEvents()


# ----------------------------------------------------------------------------- the other bodies (O1-O5)
def test_third_body_attraction(monkeypatch):
    """O1-O3: the Moon and the Sun pull a low-orbit object by ~1e-6 m/s2 (tidal 2 mu r / d^3 along their line), the
    differential term vanishes at the centre, the gridded positions match the ephemeris within 1 m; a high orbit
    drifts by kilometres in a day under them, and the propagation without them is the old one."""
    from datetime import datetime, timezone
    ep = datetime(2050, 1, 1, tzinfo=timezone.utc)
    tb = prop.ThirdBodies("earth", ep, 3600.0)
    for t in (0.0, 1234.5, 3599.0):
        T = eph.centuries_tt(ep, t)
        assert np.allclose(tb.positions(t), [eph.moon_icrf_km(T), eph.sun_icrf_km(T)], atol=1e-3)
    assert np.allclose(tb.accel(np.zeros((1, 3)), 0.0), 0.0, atol=1e-20)
    rm_ = tb.positions(0.0)[0]
    u = rm_ / np.linalg.norm(rm_)
    moon_only = prop.ThirdBodies("earth", ep, 3600.0)
    moon_only.mu = np.array([tb.mu[0], 0.0])
    a_moon = float(np.linalg.norm(moon_only.accel(7000.0 * u[None, :], 0.0)[0]))
    assert a_moon == pytest.approx(2.0 * tb.mu[0] * 7000.0 / np.linalg.norm(rm_) ** 3, rel=0.03)
    a_all = 1e3 * float(np.linalg.norm(tb.accel(7000.0 * u[None, :], 0.0)[0]))
    assert 1e-7 < a_all < 1e-5
    geo_orbit = {"type": "elements", "a_km": 42164.0, "e": 0.0, "i_deg": 0.1, "raan_deg": 0.0, "argp_deg": 0.0,
                 "nu_deg": 0.0}
    sc = _orbit_sc(86400.0, step=10.0)
    sc["ship"]["orbit"] = geo_orbit
    with_tb = prop.propagate(sc)
    monkeypatch.setattr(prop, "THIRD_BODY", False)
    without = prop.propagate(sc)
    drift = float(np.linalg.norm(with_tb.r_km[-1, 0] - without.r_km[-1, 0]))
    assert 1.0 < drift < 200.0


def test_identity_colours_match_the_symbology():
    """The objects' colours by classification are the identity frames' colours of the symbology."""
    import importlib.util
    from gui.sim_view import IDENTITY_COLORS
    spec = importlib.util.spec_from_file_location("gen_symbols", os.path.join(ROOT, "tools", "gen_symbols.py"))
    sym = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sym)
    assert set(IDENTITY_COLORS) == set(scn.IDENTITIES)
    assert all(IDENTITY_COLORS[k].lower() == sym.IDENTITIES[k][1].lower() for k in IDENTITY_COLORS)


def test_ship_name():
    """The ship is named (default OWNSHIP), renamable without propagating; no object may take its name."""
    from gui.sim_view import _renaming
    sc = scn.normalise(_base())
    assert sc["ship"]["name"] == scn.ship_name(sc) == "OWNSHIP"
    new = copy.deepcopy(sc)
    new["ship"]["name"] = "ALPHA"
    assert scn.validate(new) == [] and _renaming(sc, new) == {}
    new["objects"][0]["id"] = "ALPHA"
    assert any("the ship's name" in e for e in scn.validate(new))
    new["ship"]["name"] = " "
    assert any("name must be" in e for e in scn.validate(new))


def test_no_propagation_for_non_physical_changes():
    """A renaming, a classification (identity) or the scenario's name: no propagation; a characterisation (class),
    a value of the dynamics: propagation."""
    from gui.sim_view import _renaming
    old = scn.normalise(_base())
    new = copy.deepcopy(old)
    new["objects"][0]["identity"] = "hostile"
    new["groups"][0]["identity"] = "friendly"
    new["meta"]["name"] = "other"
    assert _renaming(old, new) == {}
    new["objects"][0]["id"] = "RENAMED"
    assert _renaming(old, new) == {old["objects"][0]["id"]: "RENAMED"}
    cls = copy.deepcopy(old)
    cls["objects"][0]["class"] = "DECOY" if old["objects"][0]["class"] != "DECOY" else "DEBRIS"
    assert _renaming(old, cls) is None
    dyn = copy.deepcopy(old)
    dyn["objects"][0]["t_appear_s"] = float(dyn["objects"][0]["t_appear_s"]) + 1.0
    assert _renaming(old, dyn) is None


def test_lunar_interception_scenario_loads():
    """The lunar scenario (scenarios/lune_interception.json, user 29 October 2026): OWNSHIP in a low circular orbit,
    SHIP_1 on an eccentric orbit whose periapsis is OWNSHIP's altitude, 20 KKV fired 600 s before the interception
    with a 5000 m/s impulse and terminal guidance."""
    sc = scn.load(os.path.join(os.path.dirname(__file__), "..", "scenarios", "lune_interception.json"))
    assert sc["central_body"] == "moon" and sc["ship"]["orbit"]["e"] == 0.0
    ship1 = sc["objects"][0]["state"]
    assert ship1["a_km"] * (1.0 - ship1["e"]) == pytest.approx(sc["ship"]["orbit"]["a_km"], abs=0.01)
    g = sc["groups"][0]
    imp = next(m for m in g["manoeuvres"] if m["type"] == "impulse")
    assert g["n"] == 20 and g["class"] == "KKV" and g["t_appear_s"] == 300.0
    assert np.linalg.norm(imp["dv_mps"]) == pytest.approx(5000.0, abs=0.01)


def test_main_window_fits_the_screen(monkeypatch):
    """The main window starts at tool.display.window_size (2560 x 1440) on a screen large enough, cut to the available
    area in a direction that does not fit (5120 x 1440 with its task bar), maximised when neither fits (1920 x 1080);
    never larger than the screen."""
    app = _qt()
    from PyQt5.QtCore import QRect, Qt
    from core import mission as md
    from gui import main_window as mw

    class _Screen:
        def __init__(self, r):
            self.r = r

        def availableGeometry(self):
            return self.r
    win = mw.MainWindow(md.empty_mission())
    for rect, fits, size, maxi in ((QRect(0, 0, 3840, 2100), True, mw.WINDOW_SIZE, False),
                                   (QRect(0, 0, 5120, 1369), False, (2560, 1369 - mw.FRAME_MARGIN[1]), False),
                                   (QRect(0, 0, 1920, 1040), False, None, True)):
        win.showNormal()
        monkeypatch.setattr(win, "screen", lambda r=rect: _Screen(r))
        assert win.show_default() == fits
        app.processEvents()
        assert bool(win.windowState() & Qt.WindowMaximized) == maxi
        if size is not None:
            assert (win.width(), win.height()) == size
        assert win.width() <= rect.width() and win.height() <= rect.height()
    win.close()


def test_scenario_tab_layout():
    """R1-R6, B14: the Scenario tab: the options column (the form under the tree), the column of the information and
    the 3D view; the view's controls in the playback bar, the events as marks on its slider; its files, main action and status handed to the window (menu, header,
    status bar)."""
    app = _qt()
    from gui.scenario_tab import ScenarioTab
    from gui.scenario_tab import new_scenario as scn_tab_new
    from gui.sim_view import OWNSHIP
    tab = ScenarioTab()
    assert not tab.loaded and tab.left_col.isHidden() and tab.preview.view_panel.isHidden()   # P1: the view alone
    tab._set_scenario(scn_tab_new(), None)                                    # (New: after its properties dialog)
    assert tab.loaded and not tab.left_col.isHidden() and not tab.preview.view_panel.isHidden()
    assert [tab.tree.topLevelItem(i).text(0) for i in range(tab.tree.topLevelItemCount())] == [
        "OWNSHIP", "Objects", "Groups"]                                        # the ship by its name (default)
    assert tab._split.count() == 3 and tab._split.widget(0) is tab.left_col
    assert tab._split.widget(1) is tab.preview.side_panel                         # the information: own column
    assert tab.preview.side_panel.isAncestorOf(tab.preview.info)
    assert tab.preview.view_panel.isAncestorOf(tab.preview.tg_view)            # the view's options under the Info
    assert tab.preview.side_panel.isAncestorOf(tab.preview.view_panel)
    assert tab.primary_action() is tab.preview.bt_prop and tab.preview.bt_prop.isHidden()   # small: automatic
    assert [c[0] for c in tab.file_commands()] == ["New...", "Open...", "Save", "Save as...", "Scenario properties...",
                                                   "Export mission at t...", "Record run...", "Open run..."]
    assert tab.status in tab.status_widgets() and tab.preview.progress in tab.status_widgets()
    assert not tab.preview.cb_true.isEnabled() and tab.preview.cb_true.isHidden()   # ship view only
    assert tab.preview.view_panel.isAncestorOf(tab.preview.cb_labels)          # layers as switches, no menu
    tab.preview.tg_view.setChecked(True)
    assert tab.preview.cb_true.isEnabled() and not tab.preview.cb_true.isHidden()
    tab.preview.tg_view.setChecked(False)
    assert "valid" in tab.status.text()
    tab.sc["objects"][0]["class"] = "NOPE"
    tab._validate()
    assert "problem" in tab.status.text() and tab.messages.count() >= 1
    tab.sc["objects"][0]["class"] = "KKV"
    tab._validate()
    sc = scn.normalise(tab.sc)
    tr = prop.propagate(sc)
    tab.preview.show_run(sc, tr, geo.geometry(sc, tr), "test")
    tab.preview._select(OWNSHIP)
    assert "altitude" in tab.preview.info.toPlainText()
    tab.preview._select("")
    assert not tab.preview.info.toPlainText()
    app.processEvents()


def test_document_formats(tmp_path):
    """Phase 6 merge, step A: the single mission file (sections mission and scenario); a mission alone (JSON, CSV) and
    a scenario naming its mission file open converted; saving writes the single format, read back identical."""
    from sim import document as doc
    d = doc.load(MISSION)                                                  # a mission alone
    assert d.scenario is None and d.mission["sectors"] and doc.kind(json.load(open(MISSION, encoding="utf-8"))) == "mission"
    csv_path = MISSION.replace(".json", ".csv")
    assert doc.load(csv_path).scenario is None
    sc = _orbit_sc(20.0)
    sc_file = tmp_path / "s.json"
    raw = {k: v for k, v in sc.items()}
    raw["mission"] = os.path.relpath(MISSION, tmp_path)
    sc_file.write_text(json.dumps(raw), encoding="utf-8")
    d2 = doc.load(str(sc_file))                                            # a scenario: its mission embedded
    assert d2.scenario is not None and "mission" not in d2.scenario
    assert [s["id"] for s in d2.mission["sectors"]] == [s["id"] for s in d.mission["sectors"]]
    out = tmp_path / "m.json"
    doc.save(str(out), d2)
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["format_version"] == doc.FORMAT_VERSION and set(data) == {"format_version", "mission", "scenario"}
    d3 = doc.load(str(out))
    assert d3.mission == d2.mission and d3.scenario == d2.scenario
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"format_version": 99, "mission": {}}), encoding="utf-8")
    with pytest.raises(doc.DocumentError):
        doc.load(str(bad))


def test_document_navigation_from_the_scenario():
    """Step B: with a scenario, the mission computed carries the navigation of the scenario's ship at t_nav_s (its
    orbit state at that instant); without one, the mission as typed."""
    from sim import document as doc
    from core import mission as cm
    mission = cm.load_mission(MISSION)
    m0, _ = doc.mission_for_calc(doc.Document(dict(mission, t_nav_s=5.0)))
    assert "t_nav_s" not in m0 and m0.get("global", {}).get("nav") == mission.get("global", {}).get("nav")
    sc = _orbit_sc(20.0)
    d = doc.Document(dict(mission, t_nav_s=10.0), sc)
    m, _alerts = doc.mission_for_calc(d)
    nav = m["global"]["nav"][0]
    tr = prop.propagate(sc)
    r10 = geo.states_at(tr, 10.0)[0][0]
    assert np.allclose(nav["r_km"], r10, atol=1e-3) and "t_nav_s" not in m
    with pytest.raises(doc.DocumentError):
        doc.mission_for_calc(doc.Document(dict(mission, t_nav_s=50.0), sc))
