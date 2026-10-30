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
    (lambda s: s.update(central_body="vulcan"), "unknown"),   # (any body of the simulator is valid, Mars too)
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
    from gui.scenario_tab import ActionList, FormWidget, ScenarioTab
    tab = ScenarioTab(view=False)
    assert tab.messages.item(0).text() == "Scenario valid"
    tab.add_object()
    tab.add_group()
    assert [o["id"] for o in tab.sc["objects"]] == ["KKV-1", "OBJ-1"] and tab.sc["groups"][0]["id"] == "GRP-1"
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    ml = tab.form_holder.findChild(ActionList)
    for t in ("impulse", "burn", "singer", "pn"):
        ml._add_manoeuvre(t)
    ml.select(1)                                      # (the random manoeuvre first, the impulse next)
    ml._remove()
    assert [m["type"] for m in tab.sc["objects"][0]["manoeuvres"]] == ["burn", "singer", "pn"]
    tab.tree.setCurrentItem(tab.n_ship)
    tab.form_holder.findChildren(QComboBox)[0].setCurrentText("state")
    assert tab.sc["ship"]["orbit"]["type"] == "state"
    assert len(tab.form_holder.findChildren(FormWidget)) == 4      # name, orbit, waypoint time, waypoint attitude
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


def test_pnr_frame_and_node_preview():
    """The manoeuvre node (user, 29 October 2026): the pnr frame (X along v, Y along r x v, Z = X x Y) and the
    one-body preview of an impulse and a burn, which keeps within a metre of the full propagation over 900 s."""
    r, v = np.array([7000.0, 0.0, 0.0]), np.array([0.3, 7.5, 0.4])
    m = prop.pnr_matrix(r, v)
    assert np.allclose(m.T @ m, np.eye(3)) and np.allclose(m[:, 0], v / np.linalg.norm(v))
    assert np.allclose(m[:, 1], np.cross(r, v) / np.linalg.norm(np.cross(r, v))) and m[:, 2] @ r > 0.0
    man = [{"type": "impulse", "t_s": 100.0, "dv_mps": [50.0, 20.0, -30.0], "frame": "pnr"},
           {"type": "burn", "t_s": 300.0, "duration_s": 40.0, "accel_mps2": [0.5, 0.0, 0.2], "frame": "pnr"}]
    sc = _orbit_sc(900.0, step=0.1)
    sc["ship"]["orbit"]["e"] = 0.05
    sc["ship"]["manoeuvres"] = man
    assert scn.validate(sc) == []
    sc = scn.normalise(sc)
    tr = prop.propagate(sc)
    third = prop.ThirdBodies(tr.body.name, scn.epoch(sc), 900.0) if prop.THIRD_BODY else None
    k = int(np.searchsorted(tr.t_s, 50.0))
    ts, rr, _vv, pre = prop.preview_path(tr.body, third, tr.r_km[k, 0], tr.v_kms[k, 0], float(tr.t_s[k]), 900.0,
                                         sc["ship"]["manoeuvres"], 5.0)
    assert ts[-1] == pytest.approx(900.0) and np.linalg.norm(rr[-1] - tr.r_km[-1, 0]) < 1e-3
    assert all(p is not None for p in pre)
    r100 = geo.states_at(tr, 100.0 - 1e-6)[0][0]
    assert np.linalg.norm(pre[0][0] - r100) < 1e-3       # the state just before the impulse


def test_ship_manoeuvres():
    """The ship takes manoeuvres as any object (user, 29 October 2026): an impulse changes its velocity from its
    time on (before it, the path is the free one); its pn needs a target other than itself, bounded by the class
    sim.ship_manoeuvre_class (1 g), and nulls the line of sight's turn."""
    o = {"id": "T", "class": "SHIP", "state": {"type": "relative", "range_km": 50.0, "phi_deg": 0.0, "eps_deg": 90.0,
                                               "v_close_mps": 800.0, "v_transverse_mps": [50.0, 0.0]}}

    def run(man):
        sc = _orbit_sc(60.0, [copy.deepcopy(o)], step=0.1)
        sc["ship"]["manoeuvres"] = man
        assert scn.validate(sc) == []
        return prop.propagate(scn.normalise(sc))
    free = run([])
    imp = run([{"type": "impulse", "t_s": 10.0, "dv_mps": [0.0, 10.0, 0.0], "frame": "rtn"}])
    k = np.searchsorted(imp.t_s, 9.95)
    assert np.allclose(imp.r_km[:k, 0], free.r_km[:k, 0], atol=1e-9)
    assert np.linalg.norm(imp.v_kms[-1, 0] - free.v_kms[-1, 0]) * 1e3 == pytest.approx(10.0, rel=0.01)
    pn = run([{"type": "pn", "t_s": 0.0, "target": "T"}])

    def turn(t):
        u = (t.r_km[:, 1] - t.r_km[:, 0]) / np.linalg.norm(t.r_km[:, 1] - t.r_km[:, 0], axis=1)[:, None]
        return np.degrees(np.arccos(np.clip(u[0] @ u[-1], -1.0, 1.0)))
    assert turn(pn) < 0.1 * turn(free)
    acc = np.linalg.norm(np.diff(pn.v_kms[:, 0] - free.v_kms[:, 0], axis=0), axis=1) / np.diff(pn.t_s) * 1e3
    assert 0.0 < acc.max() <= 9.80665 * (1 + 1e-6)
    # crashed by its own manoeuvre (user, 29 October 2026): the run ends at the ship's impact, nothing spawns after
    crash = _orbit_sc(600.0, [dict(copy.deepcopy(o), id="LATE", t_appear_s=500.0)])
    crash["ship"]["manoeuvres"] = [{"type": "impulse", "t_s": 10.0, "dv_mps": [-3000.0, 0.0, 0.0], "frame": "pnr"}]
    tr = prop.propagate(scn.normalise(crash))
    assert np.isfinite(tr.t_impact_s[0]) and tr.t_s[-1] == pytest.approx(tr.t_impact_s[0]) and tr.t_s[-1] < 500.0
    assert np.isfinite(tr.r_km[-1, 0]).all() and not np.isfinite(tr.r_km[:, tr.row("LATE")]).any()
    geo.geometry(scn.normalise(crash), tr)              # (the geometry of a run cut short)
    sc = _orbit_sc(60.0, [copy.deepcopy(o)])
    for bad in ({"type": "pn", "t_s": 0.0}, {"type": "pn", "t_s": 0.0, "target": "ship"}):
        sc["ship"]["manoeuvres"] = [bad]
        assert any("pn target" in e for e in scn.validate(sc))


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
    assert g.aspect_deg[0, k] < 0.01 and g.range_rate_mps[0, k] == pytest.approx(-3000.0, abs=1.0)
    tgt = rm.target_for_sector({"id": "K", "target_type": "KKV"})
    # (a relative object starts from the ship's point, the apertures are 59 m from it: nose on within 0.01 deg)
    assert g.rcs_s_dbsm[0, k] == pytest.approx(tgt.aspect_table_db(g.f_s_ghz)[0] + 1.5, abs=1e-3)
    assert g.rcs_ku_dbsm[0, k] == pytest.approx(tgt.aspect_table_db(g.f_ku_ghz)[0] + 1.5, abs=1e-3)
    assert np.allclose(g.dir_ship[0, k], frames.unit(45.0, 5.0), atol=2e-4)
    assert g.range_km[0, k] == pytest.approx(500.0, abs=0.07)
    for j in (k, gi, d):
        if g.face[0, j] >= 0:
            f, _ = env.blockage(int(g.face[0, j]), g.dir_ship[0, j][None, :], "S")
            assert g.block_f[0, j] == pytest.approx(float(f[0]))
            assert g.masked[0, j] == (g.block_loss_db[0, j] > env.BLOCK_MASK_DB)
    r, v = geo.states_at(tr, 4.0)
    j = tr.row("G")
    want = math.degrees(math.acos(float(v[j] @ (r[0] - r[j]) / np.linalg.norm(v[j]) / np.linalg.norm(r[0] - r[j]))))
    assert g.aspect_deg[1, gi] == pytest.approx(want, abs=0.01)         # (seen from the apertures)
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
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


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


def test_manoeuvre_node():
    """The manoeuvre node (user, 29 October 2026): the impulse chosen in the box shows its node on its body's path;
    a handle held pulled changes the delta-v along its axis, the box shows it; a time typed in the box moves the node;
    the node dragged along the path changes the time; the body's planned trajectory follows at once."""
    app = _qt()
    from gui.scenario_tab import ActionList, ScenarioTab
    from gui import maneuver_node as mn
    tab = ScenarioTab()
    k = {"id": "K", "class": "KKV", "manoeuvres": [{"type": "impulse", "t_s": 20.0, "dv_mps": [0.0, 0.0, 0.0],
                                                    "frame": "pnr"}],
         "state": {"type": "relative", "range_km": 300.0, "phi_deg": 40.0, "eps_deg": 10.0, "v_close_mps": 100.0}}
    sc = _orbit_sc(60.0, [k], step=0.1)
    tab._set_scenario(sc, None)
    v = tab.preview
    v.plotter.ren_win.SetSize(900, 700)
    v.show_run(sc, prop.propagate(sc), geo.geometry(sc, prop.propagate(sc)), "test")
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    ml = tab.form_holder.findChild(ActionList)
    ml.select(0)
    node, m = v.node, tab.sc["objects"][0]["manoeuvres"][0]
    assert node.target["m"] is m and v.truth.ids[node.row()] == "K" and node._screen is not None
    path0 = v._dyn[mn.NAME + "path"][1].points.copy()
    e = next(x for x in node._screen["ends"] if x["axis"] == 0 and x["sign"] > 0)      # prograde +, to its stop
    assert node._press(e["x"], e["y"])
    stop = mn.PULL_F * mn.NODE_PX * v.ui_scale
    node._move(e["x"] + e["sx"] * stop, e["y"] + e["sy"] * stop)
    node._drag["last"] -= 0.5                           # held 0.5 s
    node._tick()
    node._release(0.0, 0.0)
    assert m["dv_mps"][0] == pytest.approx(0.5 * mn.DV_RATE, rel=0.05) and abs(m["dv_mps"][1]) < 1e-9
    assert np.allclose([b.value() for b in ml.form.widgets["dv_mps"][1].boxes], m["dv_mps"], atol=1e-3)
    assert not np.allclose(v._dyn[mn.NAME + "path"][1].points[-1], path0[-1])
    ml.form.widgets["t_s"][1].setValue(30.0)            # the box -> the node
    c = node._preview()
    assert m["t_s"] == 30.0 and c["node"] is not None
    assert np.linalg.norm(c["node"][0] - geo.states_at(v.truth, 30.0)[0][v.truth.row("K")]) < 0.05
    s = node._screen                                    # the node dragged along its path
    assert node._press(s["cx"], s["cy"])
    base = node._drag["base"]
    j = len(base["t"]) // 2
    node._move(*node._display(base["pts"][j]))
    node._release(0.0, 0.0)
    assert m["t_s"] == pytest.approx(float(base["t"][j]), abs=0.5)
    assert scn.validate(tab.sc) == []

    class _Click:                                       # a click in the view, far from every body: on nothing
        def GetEventPosition(self):
            return (1, 1)

        def GetRepeatCount(self):
            return 0
    v._on_click(_Click(), None)
    assert node.target is None and ml.table.currentRow() == -1 and ml.form is None
    v.plotter.camera.Azimuth(10.0)                      # a camera change after: no former node drawn (no error)
    v._on_camera()
    ml.select(0)                                        # chosen again, then another element of the tree
    assert node.target is not None
    tab.tree.setCurrentItem(tab.n_ship)
    assert node.target is None
    v.plotter.close()
    app.processEvents()


def test_multitracks():
    """Multitracks (user, 29 October 2026): objects of one identity and class within MULTITRACK_PX of one another on
    screen are drawn as one symbol with a doubled outline, their number under it, one data box; another identity
    stays apart; a zoom in splits them; a click on one shows its members in the Info box."""
    app = _qt()
    from gui import sim_view as sv
    objs = [{"id": f"K{k}", "class": "KKV", "identity": "hostile",
             "state": {"type": "relative", "range_km": 500.0 + 0.2 * k, "phi_deg": 30.0, "eps_deg": 10.0,
                       "v_close_mps": 0.0}} for k in range(4)]
    objs.append({"id": "F", "class": "KKV", "identity": "friendly",
                 "state": {"type": "relative", "range_km": 500.3, "phi_deg": 30.0, "eps_deg": 10.0, "v_close_mps": 0.0}})
    sc = _orbit_sc(20.0, objs, step=0.1)
    tr = prop.propagate(sc)
    v = sv.SimView()
    v.plotter.ren_win.SetSize(900, 700)
    v.show_run(sc, tr, geo.geometry(sc, tr), "test")
    v.set_time(5.0)
    cl = v._multitracks(*v._mt_input)
    assert [(c["ident"], c["cls"], len(c["k"])) for c in cl] == [("hostile", "KKV", 4)]
    assert sv.DYN + "sym_multi_hostile_KKV.png" in v._dyn and v._lab[sv.DYN + "mt_n_hostile"]._last == ["4"]
    assert any(b.startswith("4 x KKV") for b in v._lab[sv.DYN + "labels"]._last)
    assert len(v._screen_src[sv.DYN + "planned"][1]) == 2   # one path for the multitrack, one for F (not one per
    # member; the path's counts: it is drawn dashed)
    v.set_view("ship")                          # ship view: one line of sight per multitrack
    v.cb_los.setChecked(True)                   # (off by default, user 30 October 2026)
    v.set_time(5.0)
    if v._mt_click:
        assert v._dyn[sv.DYN + "los"][1].n_lines == 2
    v.set_view("global")
    v.set_time(5.0)
    key = next(iter(v._mt_click))
    v.select(key)
    assert v.info.toPlainText().startswith("name	4 x KKV") and "K3	" in v.info.toPlainText()
    v.select("K1")                                      # a member selected: the square around its multitrack
    assert np.allclose(v._sel_pt, v._mt_of["K1"])
    c = cl[0]["q"]
    cam = v.plotter.camera                              # zoomed in under MULTITRACK_OFF_KM: no multitrack, even with
    cam.SetFocalPoint(*c)                               # the objects still within MULTITRACK_PX on screen
    u = (np.asarray(cam.GetPosition()) - c) / np.linalg.norm(np.asarray(cam.GetPosition()) - c)
    d = 0.4 * sv.MULTITRACK_OFF_KM / (2.0 * np.tan(np.radians(0.5 * cam.GetViewAngle())))
    cam.SetPosition(*(c + d * u))
    v.plotter.renderer.ResetCameraClippingRange()
    xy, _ok = v._display_xy(np.array([geo.states_at(tr, 5.0)[0][tr.row(f"K{k}")] for k in range(2)]))
    assert np.linalg.norm(xy[0] - xy[1]) < sv.MULTITRACK_PX and v._multitracks(*v._mt_input) == []
    c = cl[0]["q"]                                      # zoomed in close: the multitrack splits
    cam.SetFocalPoint(*c)
    cam.SetPosition(*(c + 0.05 * (np.asarray(cam.GetPosition()) - c) / np.linalg.norm(np.asarray(cam.GetPosition()) - c)))
    v.plotter.renderer.ResetCameraClippingRange()
    v._draw_frame()
    assert v._multitracks(*v._mt_input) == [] and not v._mt_click
    v.plotter.close()
    app.processEvents()


def test_tree_and_view_selection():
    """The editor's tree and the 3D view choose the same body (user, 29 October 2026): a click on an object, on a
    group's member or on OWNSHIP chooses it (its group) in the tree, the view keeping the member clicked; a choice
    in the tree selects the body in the view."""
    app = _qt()
    from gui.scenario_tab import ScenarioTab
    from gui.sim_view import OWNSHIP
    tab = ScenarioTab()
    objs = [{"id": "A", "class": "KKV", "state": {"type": "relative", "range_km": 300.0, "phi_deg": 30.0,
                                                   "eps_deg": 10.0, "v_close_mps": 0.0}}]
    grp = [{"id": "G", "class": "DECOY", "n": 3, "around": "A", "pos_sigma_m": 50000.0, "vel_sigma_mps": 0.0}]
    sc = _orbit_sc(20.0, objs, grp, step=0.1)
    tab._set_scenario(sc, None)
    v = tab.preview
    v.plotter.ren_win.SetSize(900, 700)
    tr = prop.propagate(sc)
    v.show_run(sc, tr, geo.geometry(sc, tr), "test")
    v.cb_symbols.setChecked(False)                      # (no multitrack: every body its own target)
    v.set_time(5.0)
    r = geo.states_at(tr, 5.0)[0]

    def click(oid):
        xy = v._display_xy(r[tr.row(oid)][None])[0][0]

        class _C:
            def GetEventPosition(self):
                return tuple(xy)

            def GetRepeatCount(self):
                return 0
        v._on_click(_C(), None)
    click("A")
    assert tab.tree.currentItem() is tab.n_obj.child(0) and v.selected == "A"
    click("G#2")
    assert tab.tree.currentItem() is tab.n_grp.child(0) and v.selected == "G#2"   # the member clicked kept
    click("ship")
    assert tab.tree.currentItem() is tab.n_ship and v.selected == OWNSHIP
    tab.tree.setCurrentItem(tab.n_obj.child(0))         # the tree -> the view
    assert v.selected == "A"
    v.plotter.close()
    app.processEvents()


def test_event_marks_follow_the_classification():
    """The events' marks on the paths are in their body's colour (its identity) and follow a change of
    classification, which needs no propagation (user, 29 October 2026)."""
    app = _qt()
    from vtkmodules.util.numpy_support import vtk_to_numpy
    from gui import sim_view as sv
    o = {"id": "K", "class": "KKV", "identity": "hostile", "t_appear_s": 5.0,
         "manoeuvres": [{"type": "impulse", "t_s": 10.0, "dv_mps": [1.0, 0.0, 0.0], "frame": "pnr"}],
         "state": {"type": "relative", "range_km": 300.0, "phi_deg": 30.0, "eps_deg": 10.0, "v_close_mps": 0.0}}
    sc = _orbit_sc(20.0, [o], step=0.1)
    tr = prop.propagate(sc)
    v = sv.SimView()
    v.show_run(sc, tr, geo.geometry(sc, tr), "test")
    def marks():
        a = v._dyn[sv.DYN + "markers"][2]
        if not a.GetVisibility():
            return []
        a.GetMapper().Update()                          # the data as drawn (the pipeline runs at the render)
        return vtk_to_numpy(a.GetMapper().GetInput().GetPointData().GetArray("rgb")).tolist()
    assert marks() == [list(sv._rgb(sv.IDENTITY_COLORS["hostile"]))] * 2      # appears, impulse
    new = copy.deepcopy(sc)
    new["objects"][0]["identity"] = "friendly"
    v.set_scenario(new)
    assert not v.debounce.isActive() and marks() == [list(sv._rgb(sv.IDENTITY_COLORS["friendly"]))] * 2
    v.set_time(7.0)                                     # a past event's mark goes (user): the impulse's stays
    assert len(marks()) == 1 and v._lab[sv.DYN + "markers_lbl"]._last == ["K impulse"]
    v.set_time(12.0)
    assert marks() == []
    v.plotter.close()
    app.processEvents()


def test_equatorial_orbit():
    """A new central body puts the ship on a circular orbit in its equatorial plane, at 1.5 equatorial radii."""
    for name, b in scn.bodies().items():
        o = scn.equatorial_orbit(name)
        r, v = prop.elements_to_state(o, b["mu_km3_s2"])
        ra, dec = np.radians(b["pole_ra_deg"]), np.radians(b["pole_dec_deg"])
        pole = np.array([np.cos(dec) * np.cos(ra), np.cos(dec) * np.sin(ra), np.sin(dec)])
        assert np.linalg.norm(r) == pytest.approx(1.5 * b["r_eq_km"])
        assert np.cross(r, v) / np.linalg.norm(np.cross(r, v)) == pytest.approx(pole, abs=1e-9)


def test_action_list():
    """W1-W3: the attitude waypoints and the manoeuvres read as one chronological list (time, type, parameters), the
    waypoints sorted by time in the scenario (the first without a slew rate); an object of class KKV flies waypoints
    on demand (toward the ship by default) and goes back to its class attitude without them."""
    app = _qt()
    from gui.scenario_tab import ActionList, FormWidget, ScenarioTab
    tab = ScenarioTab(view=False)
    tab.tree.setCurrentItem(tab.n_ship)
    wl = tab.form_holder.findChild(ActionList)
    wl._add_attitude()
    wl._add_attitude()
    assert [w["t_s"] for w in tab.sc["ship"]["attitude"]] == [0.0, 10.0, 20.0]
    form = wl.form_area.itemAt(0).widget()
    assert isinstance(form, FormWidget)
    form.widgets["t_s"][1].setValue(5.0)                 # the last point moved before the second
    ts = [w["t_s"] for w in tab.sc["ship"]["attitude"]]
    assert ts == [0.0, 5.0, 10.0] and "rate_dps" not in tab.sc["ship"]["attitude"][0]
    assert all("rate_dps" in w for w in tab.sc["ship"]["attitude"][1:])
    assert scn.validate(tab.sc) == []
    wl._add_manoeuvre("impulse")                         # at the playback time, 0 here
    wl.form.widgets["t_s"][1].setValue(7.0)              # placed between the waypoints, still selected
    wl._add_manoeuvre("singer")
    wl._add_manoeuvre("pn")
    cell = lambda r, c: wl.table.item(r, c).text()       # noqa: E731
    assert [cell(r, 0) for r in range(wl.table.rowCount())] == ["-", "0", "5", "7", "10", "-"]
    assert [cell(r, 1) for r in range(wl.table.rowCount())] == ["Random (Singer)", "Attitude", "Attitude", "Impulse",
                                                                "Attitude", "Guidance (PN)"]
    assert cell(2, 2) == "prograde, 5 deg/s" and cell(5, 2) == "below 100 km, toward ship"
    wl.select(3)
    wl.form.widgets["t_s"][1].setValue(20.0)             # the line follows its time, the form stays
    assert wl.table.currentRow() == 4 and wl.current["type"] == "impulse" and wl.form is not None
    for r in (5, 4, 3, 2, 1, 0, 0):                      # all removed but the ship's initial attitude
        wl.select(r)
        wl._remove()
    assert [w["t_s"] for w in tab.sc["ship"]["attitude"]] == [0.0] and tab.sc["ship"]["manoeuvres"] == []
    tab.add_object()                                    # KKV-1
    tab.tree.setCurrentItem(tab.n_obj.child(0))
    wl = tab.form_holder.findChild(ActionList)
    o = tab.sc["objects"][0]
    wl._add_attitude()
    assert o["attitude"][0] == {"t_s": 0.0, "attitude": {"type": "target", "id": "ship", "roll_deg": 0.0}}
    assert scn.validate(tab.sc) == []
    wl._remove()
    assert "attitude" not in o and scn.validate(tab.sc) == []
    app.processEvents()


def test_sim_view_offscreen(tmp_path):
    """S6-S7: the view shows a propagated run in both views (Earth, Moon and Sun at their places, trajectories,
    objects, lines of sight, the linked mission's volumes), centres on a body (OWNSHIP or an object) and zooms,
    lists the events, plays and steps, informs on an object and the ship, computes in the background and replays a
    journal."""
    app = _qt()
    from gui.sim_view import CENTRAL, DYN, OWNSHIP, VIEW_COLORS, VIEW_PROFILES, VOL, SimView
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
    assert DYN + "planned" in names and DYN + "planned_ship" in names
    import gc                             # UI size: the events' texts follow it, their dots keep their size
    gc.collect()
    tp = v._lab[DYN + "markers_lbl"].mapper.GetLabelTextProperty()
    f0, p0 = tp.GetFontSize(), v._dyn[DYN + "markers"][2].GetProperty().GetPointSize()
    v.set_ui_scale(1)
    assert tp.GetFontSize() > f0 and v._dyn[DYN + "markers"][2].GetProperty().GetPointSize() == p0
    v.set_ui_scale(-1)
    v.cb_events.setChecked(False)                       # "events" off: neither the marks nor their texts
    assert not v._dyn[DYN + "markers"][2].GetVisibility() and not v._lab[DYN + "markers_lbl"].actor.GetVisibility()
    v.cb_events.setChecked(True)
    assert v._dyn[DYN + "markers"][2].GetVisibility() and v._lab[DYN + "markers_lbl"].actor.GetVisibility()
    # the star background under the scene (its own renderer, layer 0), the constellations on demand
    assert v.sky.ren.GetLayer() == 0 and v.plotter.renderer.GetLayer() == 1 and v.sky.star_actors
    assert v.sky.star_actors[0].GetVisibility() and not v.sky.line_actor.GetVisibility()   # the global profile (user)
    v.cb_const.setChecked(True)
    assert v.sky.line_actor.GetVisibility()
    v.tg_sky.setChecked(False)                                         # plain background
    assert not v.sky.star_actors[0].GetVisibility() and not v.sky.line_actor.GetVisibility()
    v.cb_const.setChecked(False)
    assert any(n.startswith(DYN + "sym_") for n in names)              # the objects drawn by their symbols
    # the planned trajectories from t to the run's end, the trails of their chosen duration, the orbital vectors
    v.set_time(10.0)
    pl = v._dyn[DYN + "planned_ship"][1]
    assert np.allclose(pl.points[0], geo.states_at(tr, 10.0)[0][0]) and np.allclose(
        pl.points[-1], geo.states_at(tr, v.duration())[0][0])
    v.cb_trails.setChecked(True)                        # (off in the global profile, user 30 October 2026)
    v.sp_trail.setValue(5.0)
    tl = v._dyn[DYN + "trails"][1]
    assert np.allclose(tl.points[0], geo.states_at(tr, 5.0)[0][0])     # OWNSHIP's trail starts 5 s back
    v.cb_planned.setChecked(False)
    assert not v.plotter.actors[DYN + "planned"].GetVisibility()
    v.cb_orbit.setChecked(True)
    v.sp_orbit.setValue(5.0)
    vel = v._dyn[DYN + "vel"][1].points                 # OWNSHIP's free fall (no manoeuvre here): its run path
    assert np.allclose(vel[0], geo.states_at(tr, 10.0)[0][0])
    assert np.linalg.norm(vel[v._dyn[DYN + "vel"][1].lines[0] - 1] - geo.states_at(tr, 15.0)[0][0]) < 1e-3
    v.cb_planned.setChecked(True)
    v.cb_orbit.setChecked(False)
    v.sp_trail.setValue(60.0)
    v.set_time(0.0)
    assert {"body_earth", "body_earth_atm", "body_moon", "body_sun"} <= set(names)
    # a celestial body selected: its orbit and those of the bodies orbiting it (user, 29 October 2026)
    from gui.sim_view import BODY_SEL
    def orbits():                                       # the orbits drawn: one actor each (its part near t)
        return sorted(n[len(DYN) + 6:-5] for n, e in v._dyn.items()
                      if n.startswith(DYN + "orbit_") and n.endswith("_near") and e[2].GetVisibility())
    v.cb_all_orbits.setChecked(False)                   # (on in the global profile, user 30 October 2026)
    for name, n_orbits in (("Sun", 9), ("Earth", 2), ("Moon", 2)):   # (+ the central body's, the Earth's, always;
        #                                                                the Sun: the Earth and the eight planets)
        v.select(BODY_SEL + name)
        assert len(orbits()) == n_orbits and "earth" in orbits()
    v.select(BODY_SEL + "Sun")                          # the Earth's orbit around the Sun passes through the Earth
    near = v._dyn[DYN + "orbit_earth_near"][1].points
    assert np.linalg.norm(near - v._bodies_at()["earth"], axis=1).min() < 1.0     # (km: its part near t)
    v.select(None)                                      # nothing selected: the central body's orbit alone
    assert orbits() == ["earth"]
    from gui.sim_view import BODY_PARENT
    v.cb_all_orbits.setChecked(True)                    # "all orbits": every body's, whatever the selection
    assert len(orbits()) == len(BODY_PARENT)
    v.cb_all_orbits.setChecked(False)
    assert orbits() == ["earth"]
    v.set_centre("Jupiter")                             # a click on a moon beside its planet: the planet (user)
    cam = v.plotter.camera
    f = np.asarray(cam.GetFocalPoint())
    u = np.asarray(cam.GetPosition()) - f
    cam.SetPosition(*(f + 5e8 * u / np.linalg.norm(u)))    # (far: Io a few px from Jupiter)
    v.plotter.renderer.ResetCameraClippingRange()
    io_xy = v.node._display(v._bodies_at()["io"])
    assert v._body_under(*io_xy) == "Jupiter"
    v.set_centre(CENTRAL)
    assert v._sun_light.positional and np.allclose(v._sun_light.position, v._bodies_at()["sun"])   # the Sun's light
    assert v._actor("body_jupiter").GetProperty().GetLighting() and not v._actor("body_sun").GetProperty().GetLighting()
    body_names = v._lab.get(DYN + "planet_lbl")         # the bodies' names follow "celestial labels", their dots stay
    v.cb_celestial_labels.setChecked(False)
    assert body_names is None or not body_names.actor.GetVisibility()
    v.cb_celestial_labels.setChecked(True)
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
    assert not v.cb_los.isChecked()              # array lines off by default (user, 30 October 2026)
    v.set_view("ship")                           # ship view
    assert v.tg_sky.isChecked() and v.cb_bodies.isChecked()     # the ship profile (user, 30 October 2026)
    v.cb_los.setChecked(True)
    v.cb_axes.setChecked(True)
    v.set_view("orbital")                                # each view its own settings (user, 30 October 2026)
    assert not v.cb_los.isChecked() and v.sp_orbit.isEnabled()
    assert v.cb_los._on_color.name() == VIEW_COLORS["orbital"]
    v.sp_trail.setValue(30.0)
    v.set_view("ship")
    assert v.cb_los.isChecked() and v.cb_axes.isChecked() and v.sp_trail.value() == VIEW_PROFILES["ship"]["trail_s"]
    v.set_view("orbital")
    assert v.sp_trail.value() == 30.0 and not v.cb_los.isChecked()
    v.set_time(10.0)                                     # Recenter, orbital view: the ship, from the zenith, 0 deg up
    v.set_centre("K")
    v.plotter.camera.Roll(30.0)
    d_o = v.plotter.camera.GetDistance()
    v.recentre()
    assert v.centre == CENTRAL and np.allclose(v.plotter.camera.GetFocalPoint(), 0.0, atol=1e-9)
    assert np.allclose(v.plotter.camera.GetPosition(), [0.0, 0.0, d_o], rtol=1e-9)
    assert np.allclose(v.plotter.camera.GetViewUp(), [1.0, 0.0, 0.0], atol=1e-9)
    v.set_view("ship")
    v.set_time(10.0)
    names = list(v.plotter.actors)
    assert DYN + "los" in names and any(n.startswith(VOL) for n in names)
    v.cb_vectors.setChecked(True)
    assert DYN + "nose" in v.plotter.actors
    box = v._lab[DYN + "labels"]._last    # the data boxes: name, altitude, orbital speed (+ range, Vc, CPA)
    kb = [b for b in box if b.startswith("K\n")]
    assert kb and kb[0].startswith("K\nalt ") and "\nVc " in kb[0] and "\ndCPA " in kb[0]
    v.tg_object_full.setChecked(False)                  # partial: the id, the range and the closing speed only
    kb = [b for b in v._lab[DYN + "labels"]._last if b.startswith("K\n")]
    assert kb and kb[0].split("\n")[1].startswith("rng ") and kb[0].split("\n")[2].startswith("Vc ")
    assert kb[0].count("\n") == 2
    v.tg_object_full.setChecked(True)
    t_c, d_c, _ok = v._cpa_all()          # the boxes' closest approach agrees with the Info box's finer one
    fine = v._cpa(tr.row("K"), v.g.ids.index("K"))
    assert t_c[v.g.ids.index("K")] == pytest.approx(fine[0], abs=0.05)
    k_part = np.array(v._screen_pts["K"])  # range projection "full": the copies only, in the rings' plane
    v.tg_proj_full.setChecked(True)
    k_full = np.array(v._screen_pts["K"])
    assert abs(k_full[2]) < 1e-9 and np.linalg.norm(k_full) == pytest.approx(np.linalg.norm(k_part), rel=1e-9)
    assert not v.plotter.actors[DYN + "proj_arcs"].GetVisibility()
    v.tg_proj_full.setChecked(False)
    assert v.plotter.actors[DYN + "proj_arcs"].GetVisibility()
    v.select("K")                         # as the scenario tool's tree does: the square around it, its Info
    sq, cam = v._dyn[DYN + "select"][1].points.mean(axis=0), np.asarray(v.plotter.camera.GetPosition())
    q = v._screen_pts["K"]                    # on its line of sight (drawn nearer by the model's radius, 30 Oct 2026)
    assert np.allclose((sq - cam) / np.linalg.norm(sq - cam), (q - cam) / np.linalg.norm(q - cam), atol=1e-9)
    assert abs(np.linalg.norm(q - sq) - v._model_r_km) < 1e-6
    assert v.info.toPlainText().startswith("name	K")
    v.select(None)
    assert not v.plotter.actors[DYN + "select"].GetVisibility()
    cam = v.plotter.camera                # the axes keep their size on screen through a zoom (no new frame)
    if v.cb_axes.isChecked():
        L0 = np.linalg.norm(v._dyn[DYN + "ship_axes"][1].points[3])
        v.zoom(2.0)
        assert np.linalg.norm(v._dyn[DYN + "ship_axes"][1].points[3]) == pytest.approx(L0 / 2.0, rel=1e-6)
    v.set_centre("K")                     # Recentre, ship view: back to the view at start, zoom and roll undone
    v.zoom(2.0)
    cam.Roll(30.0)
    v.recentre()
    assert v.centre == CENTRAL and np.allclose(cam.GetFocalPoint(), 0.0, atol=1e-9)
    p_home, up_home = np.array(cam.GetPosition()), np.array(cam.GetViewUp())
    assert np.allclose(up_home, [0.0, 0.0, 1.0], atol=1e-9)
    v._ship_home()
    assert np.allclose(cam.GetPosition(), p_home, rtol=1e-9)
    v.set_centre("K")                   # ship view centred on an object
    assert np.allclose(v.plotter.camera.GetFocalPoint(), v._screen_pts["K"], atol=1e-9)
    v.set_centre(CENTRAL)
    from gui import sim_view as sv_
    # 1:1 scale (the only one, user 29 October 2026): an object at its true range (km) from the ship
    v.set_time(10.0)
    r, _ = geo.states_at(tr, 10.0)
    assert np.linalg.norm(v._screen_pts["K"]) == pytest.approx(np.linalg.norm(r[tr.row("K")] - r[0]), rel=1e-9)
    assert any(n.startswith(VOL) for n in v.plotter.actors) and DYN + "ship_axes" in v.plotter.actors
    v.cb_bodies.setChecked(True)                         # celestial objects at 1:1: the Moon at its true range
    u = v.plotter.actors[DYN + "sbody_moon"].user_matrix
    T = eph.centuries_tt(scn.epoch(sc), 10.0)
    assert np.linalg.norm(u[:3, 3]) == pytest.approx(np.linalg.norm(eph.moon_icrf_km(T) - r[0]), rel=1e-6)
    sun_dot = v._lab.get(DYN + "planet_lbl")             # its sphere, or its dot and name when under 3 px (user)
    assert v.plotter.actors[DYN + "sbody_sun"].GetVisibility() or (
        sun_dot is not None and sun_dot.actor.GetVisibility() and "Sun" in sun_dot._last)
    v.cb_bodies.setChecked(False)
    assert not v.plotter.actors[DYN + "sbody_sun"].GetVisibility()
    # mouse rotation as a turntable: azimuth about the ship's Z, elevation about the screen's horizontal, one rate and
    # no roll wherever the drag starts
    iren =v.plotter.iren.interactor
    v.plotter.ren_win.SetSize(900, 700)                 # offscreen: the window has no size of its own
    w, h = v.plotter.ren_win.GetSize()
    assert w > 0 and h > 0
    cam = v.plotter.camera

    def drag(p0, p1, button="Right"):                   # the scenario tool turns the view by the right button
        iren.SetEventInformation(int(p0[0]), int(p0[1]), 0, 0)
        iren.InvokeEvent(f"{button}ButtonPressEvent")
        for a in np.linspace(0.0, 1.0, 6)[1:]:
            iren.SetEventInformation(int(p0[0] + a * (p1[0] - p0[0])), int(p0[1] + a * (p1[1] - p0[1])), 0, 0)
            iren.InvokeEvent("MouseMoveEvent")
        iren.InvokeEvent(f"{button}ButtonReleaseEvent")
    p_left = np.array(cam.GetPosition())
    drag((0.3 * w, 0.5 * h), (0.6 * w, 0.5 * h), "Left")    # a plain left drag leaves the camera alone
    assert np.allclose(cam.GetPosition(), p_left)
    def look():                                         # the line of sight, the screen's horizontal
        fwd = np.array(cam.GetDirectionOfProjection())
        s = np.cross(fwd, cam.GetViewUp())               # (the view up may not be normal to fwd at the start)
        return fwd, s / np.linalg.norm(s)

    def angle(a, b):
        return math.degrees(math.acos(np.clip(a @ b, -1, 1)))
    d0, s0 = look()
    drag((0.97 * w, 0.4 * h), (0.97 * w, 0.6 * h))      # at the edge of the view: elevation only, the usual rate
    d1, s1 = look()
    px = int(0.6 * h) - int(0.4 * h)
    assert angle(d0, d1) == pytest.approx(sv_.TURN_DEG_PX * px, abs=0.3)        # (a pixel of rounding)
    assert angle(s0, s1) < 1e-6 and abs(s1[2]) < 1e-9
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
    lbl = v._lab[DYN + "rings_lbl"]                     # each ring's value at the azimuths 0 and 180
    assert np.allclose(lbl._pts[0::2, 0], -lbl._pts[1::2, 0]) and (lbl._pts[0::2, 0] > 0.0).all()
    assert lbl._last[0::2] == lbl._last[1::2]
    c_h, r_h = lbl._pts[0] * [1.0, 0.0, 0.0], 0.2 * lbl._pts[0, 0]     # a body cutting a ring: its hidden part dotted
    v._planets_drawn = {"test": (c_h, r_h)}
    v._update_rings()
    arcs = v._screen_src[DYN + "rings_hidden"][0]
    assert len(arcs) and (np.linalg.norm(arcs - c_h, axis=1) > 0.999 * r_h).all()     # in front of the body
    assert v._dyn[DYN + "rings_hidden"][2].GetVisibility()
    in_body = v._in_front(c_h[None])                    # a projection inside a body: shown in front of it
    assert np.linalg.norm(in_body[0] - c_h) > 0.999 * r_h
    v._planets_drawn = {}
    v._update_rings()
    assert not len(v._screen_src[DYN + "rings_hidden"][0])
    pan = np.array([0.3, -0.2, 0.1])                    # a pan: the camera and its focal point moved together
    cam.SetFocalPoint(*(np.array(cam.GetFocalPoint()) + pan))
    cam.SetPosition(*(np.array(cam.GetPosition()) + pan))
    v.recentre()
    assert np.allclose(cam.GetFocalPoint(), 0.0, atol=1e-9) and np.allclose(v.plotter._pivot, 0.0)
    d1, s1 = look()
    drag((0.97 * w, 0.9 * h), (0.97 * w - 200, 0.9 * h))   # in a corner: azimuth only, the usual rate
    d2, s2 = look()
    assert angle(s1, s2) == pytest.approx(sv_.TURN_DEG_PX * 200.0, abs=0.5) and abs(s2[2]) < 1e-9
    assert d2[2] == pytest.approx(d1[2], abs=1e-9)
    drag((0.5 * w, 0.95 * h), (0.5 * w, 0.05 * h))         # far: stopped at the pole, the view not turned over
    drag((0.5 * w, 0.95 * h), (0.5 * w, 0.05 * h))
    assert np.allclose(look()[0], [0.0, 0.0, -1.0], atol=1e-6) and abs(look()[1][2]) < 1e-6
    v.recentre()
    v._select("K")
    assert "range" in v.info.toPlainText() and "RCS S" in v.info.toPlainText()
    v.step(1)
    assert v.t == pytest.approx(11.0)
    v.toggle_play()
    v._wall -= 0.1
    v._tick()
    v.toggle_play()
    assert v.t > 11.0 and not v.timer.isActive()
    v.set_view("global")
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
        assert np.allclose(tb.positions(t), [eph.moon_icrf_km(T), eph.sun_icrf_km(T)] +
                           ([eph.planet_icrf_km(p, T) for p in eph.PLANETS] if prop.PLANETS_GRAVITY else []) +
                           ([eph.satellite_icrf_km(m, T) for m in eph.MOON_NAMES] if prop.MOONS_GRAVITY else []),
                           atol=1e-3)
    assert np.allclose(tb.accel(np.zeros((1, 3)), 0.0), 0.0, atol=1e-20)
    rm_ = tb.positions(0.0)[0]
    u = rm_ / np.linalg.norm(rm_)
    moon_only = prop.ThirdBodies("earth", ep, 3600.0)
    moon_only.mu = np.r_[tb.mu[0], np.zeros(len(tb.mu) - 1)]
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


def test_planets():
    """The planets (user, 29 October 2026): JPL approximate elements within 0.2 % of the known distances at three
    close approaches; each one pulls the objects (sim.planets_gravity), by far less than the Sun."""
    from datetime import datetime, timezone

    def T(y, m, d):
        return eph.centuries_tt(datetime(y, m, d, tzinfo=timezone.utc), 0.0)
    for name, date, km in (("mars", (2003, 8, 27), 55.76e6), ("mars", (2018, 7, 31), 57.59e6),
                           ("jupiter", (2022, 9, 26), 591.0e6)):
        assert np.linalg.norm(eph.planet_icrf_km(name, T(*date))) == pytest.approx(km, rel=2e-3)
    for p in eph.PLANETS:                                # within their perihelion and aphelion
        a, e = eph.PLANET_ELEMENTS[p][0][:2]
        r = np.linalg.norm(eph.planet_helio_icrf_km(p, 0.5)) / eph.AU_KM
        assert a * (1 - e) * 0.99 < r < a * (1 + e) * 1.01
    ep = datetime(2050, 1, 1, tzinfo=timezone.utc)
    tb = prop.ThirdBodies("earth", ep, 600.0)
    if prop.PLANETS_GRAVITY:
        assert tb.names[2:2 + len(eph.PLANETS)] == eph.PLANETS
        x = np.array([[7000.0, 0.0, 0.0]])
        sun_only = prop.ThirdBodies("earth", ep, 600.0)
        sun_only.mu = np.r_[0.0, tb.mu[1], np.zeros(len(tb.mu) - 2)]
        planets = prop.ThirdBodies("earth", ep, 600.0)
        planets.mu = np.r_[0.0, 0.0, tb.mu[2:]]               # (and the moons, sim.moons_gravity)
        assert 0.0 < np.linalg.norm(planets.accel(x, 0.0)) < 1e-4 * np.linalg.norm(sun_only.accel(x, 0.0))


def test_major_moons():
    """The major moons (user, 29 October 2026): JPL mean elements; their mean longitude rates read the table's period
    right (Jupiter's: the mean anomaly's, the others: sidereal; Phobos, Deimos, Triton from their IAU synchronous
    rotation) within 1e-3 deg/day of the known mean motions; the Laplace resonance of Io, Europa and Ganymede holds
    near 180 deg; Phobos' shape model in its body frame (its long axis X), its X toward Mars; they pull the objects
    (sim.moons_gravity)."""
    known = {"io": 203.4890, "europa": 101.3747, "ganymede": 50.3176, "callisto": 21.5711, "mimas": 381.9945,
             "enceladus": 262.7319, "titan": 22.5770, "phobos": 1128.8446, "triton": 61.2573, "ariel": 142.8356}
    for n, lam in known.items():
        assert sum(eph._precession(n)) == pytest.approx(lam, abs=1e-3)
    from datetime import datetime, timezone
    T = eph.centuries_tt(datetime(2050, 1, 1, tzinfo=timezone.utc), 0.0)
    frame = eph._plane(eph._radec(268.1, 64.5))

    def lon(n, t):
        x = frame.T @ eph.satellite_planetocentric_km(n, t)
        return np.radians(np.degrees(np.arctan2(x[1], x[0])))
    z = np.mean([np.exp(1j * (lon("io", T + d) - 3 * lon("europa", T + d) + 2 * lon("ganymede", T + d)))
                 for d in np.linspace(0, 7.155588 / 36525, 60)])
    assert abs(np.degrees(np.angle(z)) % 360 - 180.0) < 20.0
    for n in eph.MOON_NAMES:                             # within their periapsis and apoapsis
        el = eph._MOONS[n]
        r = np.linalg.norm(eph.satellite_planetocentric_km(n, T))
        assert el["a_km"] * (1 - el["e"]) * 0.999 < r < el["a_km"] * (1 + el["e"]) * 1.001
    from gui import sim_view as sv
    mesh = sv._phobos_mesh()
    if mesh is not None:
        ext = np.ptp(mesh.points, axis=0)
        assert ext[0] > ext[1] > ext[2] and 20.0 < ext[0] < 30.0      # long axis X, short Z (its pole)
    if prop.MOONS_GRAVITY:
        tb = prop.ThirdBodies("earth", datetime(2050, 1, 1, tzinfo=timezone.utc), 600.0)
        assert tb.names[-len(eph.MOON_NAMES):] == eph.MOON_NAMES


def test_any_central_body():
    """Any celestial body as the scenario's central body (user, 30 October 2026): the others pull (the Earth among
    them), a low orbit around Europa stays near circular under Jupiter's tide, the geometry adds the central body
    and its planet as occulters, the mission exports with its navigation around it."""
    for central, alt in (("europa", 100.0), ("jupiter", 2.0e5), ("sun", 1.0e8)):
        R = scn.bodies()[central]["radius_km"]
        sc = {"meta": {"name": central, "epoch_utc": "2050-01-01T00:00:00Z", "duration_s": 1200.0, "step_s": 1.0},
              "central_body": central, "mission": os.path.abspath(MISSION),
              "ship": {"orbit": {"type": "elements", "a_km": R + alt, "e": 0.0, "i_deg": 30.0, "raan_deg": 0.0,
                                 "argp_deg": 0.0, "nu_deg": 0.0}}, "objects": [], "groups": []}
        assert scn.validate(sc) == []
        sc = scn.normalise(sc)
        tr = prop.propagate(sc)
        assert tr.body.name == central and "earth" in prop.ThirdBodies(central, scn.epoch(sc), 60.0).names
        r = np.linalg.norm(tr.r_km[:, 0], axis=1)
        assert np.all(np.abs(r - (R + alt)) < 1e-3 * (R + alt))
        g = geo.geometry(sc, tr)
        assert g.body_names[:3] == ["sun", "earth", "moon"] and (central == "sun" or central in g.body_names)
        if central == "europa":
            assert "jupiter" in g.body_names
        m, _alerts = engine.mission_at(sc, tr, 600.0)
        nav = m["global"]["nav"]
        assert nav[0]["central_body"] == central and frames.validate_nav(nav) == []


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
    new["groups"][0]["around"] = "RENAMED"              # (the group drawn around it follows: a valid scenario)
    assert _renaming(old, new) == {old["objects"][0]["id"]: "RENAMED"}
    cls = copy.deepcopy(old)
    cls["objects"][0]["class"] = "DECOY" if old["objects"][0]["class"] != "DECOY" else "DEBRIS"
    assert _renaming(old, cls) is None
    dyn = copy.deepcopy(old)
    dyn["objects"][0]["t_appear_s"] = float(dyn["objects"][0]["t_appear_s"]) + 1.0
    assert _renaming(old, dyn) is None


def test_geometry_seen_from_the_apertures():
    """The ship turns about its centre of gravity: range and range rate are the apertures', 59.3 m ahead on +Z."""
    objs = [{"id": "T", "class": "ST", "state": {"type": "relative", "range_km": 100.0, "phi_deg": 0.0, "eps_deg": 0.0,
                                                 "v_close_mps": 0.0}}]
    sc = _orbit_sc(20.0, objs, step=0.1)
    tr = prop.propagate(sc)
    t = 10.0
    r, v = geo.states_at(tr, t)
    m, w = tr.attitude("ship", t), np.radians(geo.ship_rate_dps(tr, t))
    d = np.array([0.0, 0.0, 0.059298])
    ra, va = geo.array_state(tr, t, r[0], v[0])
    assert np.allclose(ra, r[0] + m @ d) and np.allclose(va, v[0] + m @ np.cross(w, d))
    g = geo.geometry(sc, tr, np.array([t]))
    los = r[1] - ra
    assert g.range_km[0, 0] == pytest.approx(np.linalg.norm(los))
    assert g.range_rate_mps[0, 0] == pytest.approx(1e3 * (los / np.linalg.norm(los)) @ (v[1] - va))


def test_lunar_interception_scenario_loads():
    """The lunar scenario (scenarios/moon_interception.json, user 29 October 2026): OWNSHIP in a low circular orbit;
    SHIP_1 on an orbit at 20 deg of OWNSHIP's plane, passing it near 900 s; 20 KKV fired by SHIP_1 at 300 s with a
    5000 m/s impulse and terminal guidance on OWNSHIP; OWNSHIP's answer 15 s later, 15 friendly KKV launched around
    it at 6500 m/s with terminal guidance on SHIP_1 (pn target). Every KKV reaches its target (miss < 1 m)."""
    sc = scn.load(os.path.join(os.path.dirname(__file__), "..", "scenarios", "moon_interception.json"))
    assert sc["central_body"] == "moon" and sc["ship"]["orbit"]["e"] == 0.0
    tr = prop.propagate(sc)
    i1 = tr.row("SHIP_1")
    h0, h1 = np.cross(tr.r_km[0, 0], tr.v_kms[0, 0]), np.cross(tr.r_km[0, i1], tr.v_kms[0, i1])
    assert math.degrees(math.acos(h0 @ h1 / np.linalg.norm(h0) / np.linalg.norm(h1))) == pytest.approx(20.0, abs=0.01)
    assert np.nanmin(np.linalg.norm(tr.r_km[:, i1] - tr.r_km[:, 0], axis=1)) < 0.1     # the pass near 900 s
    g1, g2 = sc["groups"]
    imp1 = next(m for m in g1["manoeuvres"] if m["type"] == "impulse")
    imp2 = next(m for m in g2["manoeuvres"] if m["type"] == "impulse")
    assert g1["n"] == 20 and g1["class"] == "KKV" and g1["t_appear_s"] == 300.0
    assert np.linalg.norm(imp1["dv_mps"]) == pytest.approx(5000.0, abs=0.01)
    assert g2["n"] == 15 and g2["around"] == "ship" and g2["identity"] == "friendly" and g2["t_appear_s"] == 315.0
    assert np.linalg.norm(imp2["dv_mps"]) == pytest.approx(6500.0, abs=0.01)
    assert next(m for m in g2["manoeuvres"] if m["type"] == "pn")["target"] == "SHIP_1"
    for grp in ("KKV#", "KKV_OS#"):
        rows = [i for i, x in enumerate(tr.ids) if x.startswith(grp)]
        assert np.isfinite(tr.miss_km[rows]).all() and tr.miss_km[rows].max() < 1e-3
    bad = copy.deepcopy(sc)                              # a pn target must exist and not be the row itself
    next(m for m in bad["groups"][1]["manoeuvres"] if m["type"] == "pn")["target"] = "KKV_OS#3"
    assert any("pn target" in e for e in scn.validate(bad))


def test_load_propagates_and_problems_block_play():
    """A scenario created or opened propagates at once whatever its size, not with problems; while the validation
    fails Play is greyed and Space does nothing; a closing window stops the propagation under way (user, 30 October
    2026)."""
    app = _qt()
    from gui.scenario_tab import ScenarioTab
    from gui.scenario_tab import new_scenario as scn_tab_new
    tab = ScenarioTab()
    v = tab.preview
    tab._set_scenario(scn_tab_new(), None)
    assert v.load_timer.isActive() and not v.debounce.isActive()
    app.processEvents()
    v.stop_background()
    assert not v.load_timer.isActive() and not v.worker.isRunning()
    tab.sc["objects"].append({"id": "BAD", "class": "NOPE"})
    tab._validate()
    assert not v._valid and not v.bt_play.isEnabled()
    v.sc = tab.sc
    v.toggle_play()
    assert not v._playing
    tab._set_scenario(tab.sc, None)
    assert not v.load_timer.isActive()


def test_window_closes_during_the_playback():
    """No frame after the window's close: the playback's timer would draw without a render window (user, 30 October
    2026)."""
    app = _qt()
    from gui.scenario_window import ScenarioWindow
    from gui.scenario_tab import new_scenario as scn_tab_new
    w = ScenarioWindow()
    v = w.scenario.preview
    w.scenario._set_scenario(scn_tab_new(), None)
    app.processEvents()
    while v.worker is None or v.worker.isRunning() or v.truth is None:
        app.processEvents()
    v.toggle_play()
    assert v._playing and v.timer.isActive()
    w.close()
    assert not v._playing and not v.timer.isActive()
    app.processEvents()


def test_labels_in_double_precision():
    """The labels' points in double precision: in single, 31 m steps at the Moon's range shook them when zoomed in."""
    app = _qt()
    from gui import sim_view as sv
    from vtkmodules.vtkCommonCore import VTK_DOUBLE
    v = sv.SimView()
    lab = sv._Labels(v.plotter, "#ffffff", 15)
    lab.set(np.array([[384400.001, 0.0, 0.0]]), ["x"])
    assert lab.points.GetDataType() == VTK_DOUBLE and lab.points.GetPoint(0)[0] == 384400.001
    v.plotter.close()


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
    assert tab.preview.view_panel.isAncestorOf(tab.preview.view_buttons["ship"])            # the view's options under the Info
    assert tab.preview.side_panel.isAncestorOf(tab.preview.view_panel)
    assert tab.primary_action() is tab.preview.bt_prop and not tab.preview.bt_prop.isEnabled()   # small: automatic
    from gui import sim_view as sv_b                    # large: active after a change, greyed once propagated
    keep = sv_b.AUTO_MAX_CELLS
    sv_b.AUTO_MAX_CELLS = 0.0
    tab.preview.pending, tab.preview._launched = scn.normalise(tab.sc), None
    tab.preview._valid = True
    tab.preview._update_prop(busy=False)
    assert tab.preview.bt_prop.isEnabled()
    tab.preview._launched = tab.preview.pending
    tab.preview._update_prop(busy=False)
    assert not tab.preview.bt_prop.isEnabled()
    sv_b.AUTO_MAX_CELLS = keep
    assert [c[0] for c in tab.file_commands()] == ["New...", "Open...", "Save", "Save as...", "Scenario properties...",
                                                   "Export mission at t...", "Record run...", "Open run..."]
    assert tab.status in tab.status_widgets() and tab.preview.progress in tab.status_widgets()
    assert not tab.preview.cb_vectors.isEnabled() and not tab.preview.cb_vectors.isVisibleTo(tab.preview.view_panel)   # ship view only
    assert tab.preview.view_panel.isAncestorOf(tab.preview.cb_object_labels)          # layers as switches, no menu
    tab.preview.set_view("ship")
    assert tab.preview.cb_vectors.isEnabled() and tab.preview.cb_vectors.isVisibleTo(tab.preview.view_panel)
    tab.preview.set_view("global")
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
