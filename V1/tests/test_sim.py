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
    assert n["central_body"] == "earth" and n["ship"]["attitude"]["mode"] == "inertial"
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
    (lambda s: s["ship"].update(attitude={"mode": "point", "axis": [0, 0, 1], "target": "ghost"}), "not found"),
    (lambda s: s["ship"].update(orbit={"type": "elements", "a_km": 7000.0, "e": 1.2, "i_deg": 0, "raan_deg": 0,
                                       "argp_deg": 0, "nu_deg": 0}), "closed orbits"),
    (lambda s: s["ship"].update(attitude={"mode": "waypoints", "points": [{"t_s": 5.0, "q": [1, 0, 0, 0]},
                                                                          {"t_s": 1.0, "q": [1, 0, 0, 0]}]}),
     "strictly increasing"),
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
