"""Family 2 -- physics and consistency: laws and invariants that must hold whatever the parameter values.

Uses the test missions of tests/missions/ (test values only, not radar performance).
"""
import contextlib
import importlib.util
import io
import json
import math
import os
import subprocess
import sys

import pytest

import testbench as tb
from core import cost_estimate as ce
from core import mission as md
from core import optimizer as mo
from core import params
from core import radar_model as rm
from core import tracking as trk

PYTHON = sys.executable


def _run_with_params(toml_text: str, code: str, tmp_path) -> str:
    """Runs code in a fresh process reading another parameter file (RADAR_PARAMS); returns its stdout."""
    path = tmp_path / "radar_system.toml"
    path.write_text(toml_text, encoding="utf-8")
    env = {**os.environ, params.PARAMS_ENV: str(path)}
    res = subprocess.run([PYTHON, "-c", code], cwd=tb.ROOT, env=env, capture_output=True, text=True, timeout=300)
    assert res.returncode == 0, res.stderr
    return res.stdout


def _default_toml() -> str:
    with open(params.DEFAULT_PATH, encoding="utf-8") as fh:
        return fh.read()


def _replace_value(text: str, section: str, line_start: str, old: str, new: str) -> str:
    """Text of the parameter file with one entry's value changed (entry line_start of [section])."""
    lines = text.splitlines(keepends=True)
    start = lines.index(f"[{section}]\n") + 1
    end = next((i for i in range(start, len(lines)) if lines[i].startswith("[")), len(lines))
    hits = [i for i in range(start, end) if lines[i].startswith(line_start)]
    assert len(hits) == 1, line_start
    assert f"value = {old}," in lines[hits[0]], lines[hits[0]]
    lines[hits[0]] = lines[hits[0]].replace(f"value = {old},", f"value = {new},")
    return "".join(lines)


# ----------------------------------------------------------------------------- model self-test
def test_model_selftest_passes():
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        rc = rm.selftest()
    assert rc == 0, out.getvalue()


def test_model_cli_selftest():
    res = subprocess.run([PYTHON, os.path.join("core", "radar_model.py"), "--selftest"], cwd=tb.ROOT,
                         capture_output=True, text=True, timeout=300)
    assert res.returncode == 0 and "ALL TESTS PASSED" in res.stdout, res.stdout + res.stderr


# ----------------------------------------------------------------------------- radar equation laws
@pytest.mark.parametrize("wf,f,th", [(rm.WF_LRS, 2.8, 0.0), (rm.WF_LRS, 2.75, 60.0), (tb.KU_TRACK, 14.0, 30.0),
                                     (tb.KU_CONFIRM, 13.8, 0.0)])
def test_range_scales_as_rcs_quarter_power(wf, f, th):
    r1 = rm.solve(wf, f, th, rm.Target("a", 1.0, 1, 5000.0)).range_km
    r2 = rm.solve(wf, f, th, rm.Target("b", 81.0, 1, 5000.0)).range_km
    assert r2 / r1 == pytest.approx(3.0, rel=1e-12)


def test_range_scales_as_peak_power_quarter_power(tmp_path):
    """16x the S TRM peak power -> 2x the range (R ~ P^(1/4)), and the change reaches solve() from the parameter file.
    The TRM buffer is scaled with it, so the buffer droop (pulse energy / buffer) stays the same."""
    text = _replace_value(_default_toml(), "band.S", "p_pk_trm_w ", "2000.0", "32000.0")
    text = _replace_value(text, "band.S", "e_buf_trm_j ", "5.0", "80.0")
    out = _run_with_params(text, "from core import radar_model as rm\n"
                                 "print(rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS['ST']).range_km)", tmp_path)
    assert float(out) / rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"]).range_km == pytest.approx(2.0, rel=1e-12)


def test_range_independent_of_face_electrical_power():
    """Electrical power sets the average-power budget, not the peak power of a look."""
    a = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], elec_mw=0.5)
    b = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], elec_mw=4.0)
    assert a.range_km == b.range_km and b.p_avg_max_w == pytest.approx(8 * a.p_avg_max_w)


def test_range_decreases_with_scan_angle():
    r = [rm.solve(rm.WF_LRS, 2.8, th, rm.TARGETS["ST"]).range_km for th in range(0, 76, 5)]
    assert all(b < a for a, b in zip(r, r[1:]))


def test_total_losses_are_the_sum_of_the_terms():
    r = rm.solve(rm.WF_LRS, 2.8, 40.0, rm.TARGETS["KKV"])
    assert r.l_tot_db == pytest.approx(sum(r.losses.values()), rel=1e-12)
    assert set(rm.BAND_S.losses_db) <= set(r.losses)


def test_coherent_requirement_independent_of_pulse_count():
    assert rm.detect_snr_req_db("COHERENT", 1, 1) == rm.detect_snr_req_db("COHERENT", 256, 1)


def test_nothing_playable_outside_the_bands_or_beyond_scan_limit():
    assert rm.playable(5.0, 0.0) == [] and rm.playable(2.8, rm.BAND_S.scan_limit_deg(2.8) + 0.1) == []
    for band in rm.BANDS.values():
        assert band.edge_loss_db(band.usable_lo_ghz - 1e-6) >= 100.0


# ----------------------------------------------------------------------------- aperture build-up
def test_aperture_derived_from_trsa_build_up():
    """196 bi-band TRSAs (25 x 25 Ku, 5 x 5 S) in an octagon, 2 mm dead gaps (array design note, review 0.3)."""
    assert rm.N_TRSA == 196 and rm.N_TRSA_GAPS == 2 * 180
    assert rm.BAND_S.n_trm == 4900 and rm.BAND_KU.n_trm == 122500
    assert rm.TRSA_SIDE_M == pytest.approx(0.260) and rm.S_TRM_PITCH_M == pytest.approx(0.052)
    assert rm.PHYS_AREA_M2 == pytest.approx(13.2496, rel=1e-9)
    assert rm.TOTAL_AREA_M2 == pytest.approx(13.4368, rel=1e-9)
    assert rm.FILL_FACTOR == pytest.approx(0.9861, abs=5e-5)
    assert rm.ARRAY_ENVELOPE_M == pytest.approx(4.190, rel=1e-9)


def test_s_trm_block_must_divide_the_ku_block(tmp_path):
    text = _replace_value(_default_toml(), "aperture", "s_trm_per_trsa_side ", "5", "4")
    path = tmp_path / "radar_system.toml"
    path.write_text(text, encoding="utf-8")
    res = subprocess.run([PYTHON, "-c", "import core.radar_model"], cwd=tb.ROOT, capture_output=True, text=True,
                         env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=120)
    assert res.returncode != 0 and "must divide" in res.stderr


def test_t_sys_from_the_receive_chain(tmp_path):
    """T_sys is derived from [band.<b>.rx] (Friis) in both bands: a 1 dB worse S LNA must raise it accordingly."""
    assert rm.BAND_S.t_sys_k == pytest.approx(154.3, abs=0.1) and rm.BAND_KU.t_sys_k == pytest.approx(238.7, abs=0.1)
    assert "pre_LNA" not in rm.BAND_S.losses_db and "lumped_impl" not in rm.BAND_KU.losses_db
    assert set(rm.BAND_S.losses_db) - {"Ku_blockage"} == set(rm.BAND_KU.losses_db) - {"S_blockage"}   # same posts
    text = _replace_value(_default_toml(), "band.S.rx", "lna_nf_db ", "0.75", "1.75")
    out = _run_with_params(text, "from core import radar_model as rm\nprint(rm.BAND_S.t_sys_k)", tmp_path)
    l_pre = 10 ** (0.85 / 10)
    assert float(out) - rm.BAND_S.t_sys_k == pytest.approx(l_pre * 290 * (10 ** 0.175 - 10 ** 0.075) * 10 ** 0.005, rel=1e-6)


def test_tx_chain_from_its_components():
    """PA-output convention: TX_chain = harmonic filter + circulator (shared with RX) + coupler + line."""
    assert rm.BAND_S.losses_db["TX_chain"] == pytest.approx(0.15 + 0.3 + 0.05 + 0.1)
    assert rm.BAND_KU.losses_db["TX_chain"] == pytest.approx(0.2 + 0.45 + 0.1 + 0.2)


def test_fingerprint_follows_system_values_only(tmp_path):
    base = params.fingerprint(params.P)
    p1 = tmp_path / "a.toml"
    p1.write_text(_replace_value(_default_toml(), "band.S.rx", "lna_nf_db ", "0.75", "0.8"), encoding="utf-8")
    assert params.fingerprint(params.load(str(p1))) != base
    p2 = tmp_path / "b.toml"
    p2.write_text(_default_toml().replace("grid_res_deg = 0.5", "grid_res_deg = 1.0"), encoding="utf-8")
    assert params.fingerprint(params.load(str(p2))) == base     # a display setting does not stale the table


# ----------------------------------------------------------------------------- beam grid / geometry
def test_positions_times_cell_area_is_sector_area():
    for f in (2.7, 2.8, 2.9):
        assert rm.n_search_positions(f) * rm.beam_cell_area_uv(f) == pytest.approx(rm.sector_area_uv(), rel=1e-12)


def test_eps_window_only_removes_positions():
    full = rm.n_search_positions(2.8)
    assert rm.n_search_positions(2.8, -20.0, 90.0) < full and rm.n_search_positions(2.8, -90.0, 30.0) < full
    assert rm.n_search_positions(2.8, -90.0, 90.0) == full


def test_eps_window_is_not_written_into_the_model():
    """Mission inputs are passed explicitly; running a mission leaves the model's defaults untouched."""
    before = (rm.SECTOR_EPS_MIN_DEG, rm.SECTOR_EPS_MAX_DEG)
    mo.run(md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_synthetic_mix.json")), workers=1)
    assert (rm.SECTOR_EPS_MIN_DEG, rm.SECTOR_EPS_MAX_DEG) == before


def test_face_sector_radius_follows_band_scan_limits(tmp_path):
    """The face sector radius is derived from the band scan limits (review 0.3), not a separate figure."""
    text = _replace_value(_default_toml(), "band.S", "scan_max_deg ", "75.0", "78.0")
    out = _run_with_params(text, "from core import radar_model as rm\nprint(rm.SECTOR_RADIUS_DEG)", tmp_path)
    assert float(out) == 78.0 and rm.SECTOR_RADIUS_DEG == max(rm.BAND_S.scan_max_deg, rm.BAND_KU.scan_max_deg)


def test_faces_partition_the_scanned_sphere():
    sph = mo.Sphere(res_deg=1.0)
    owned = sph.owner >= 0
    assert set(sph.owner[owned].tolist()) == set(range(len(mo.FACE_PHI)))
    assert (sph.theta[owned] <= rm.SECTOR_RADIUS_DEG + 1e-9).all()


def test_tilt_change_propagates_to_model_optimiser_and_ship(tmp_path):
    text = _replace_value(_default_toml(), "geometry", "tilt_deg ", "25.0", "20.0")
    out = _run_with_params(text, "import json\nfrom core import radar_model as rm, optimizer as mo, geometry as geo\n"
                                 "print(json.dumps([rm.SECTOR_TILT_DEG, rm.n_search_positions(2.8), "
                                 "mo.face_frame(0.0)[0][2], geo.RP]))", tmp_path)
    tilt, pos, nz, rp = json.loads(out)
    assert tilt == 20.0 and nz == pytest.approx(math.sin(math.radians(20.0)))
    assert abs(pos - 4308) < 30          # value of the model history at 20 deg (V6.0.10)
    from core import geometry as geo
    assert rp < geo.RP                  # arrays sit lower on the cone at a smaller tilt


# ----------------------------------------------------------------------------- optimiser invariants
@pytest.fixture(scope="module")
def runs():
    return {n: mo.run(md.load_mission(os.path.join(tb.MISSIONS_DIR, n)), workers=1) for n in tb.MISSIONS}


def test_time_on_air_at_least_energy_over_peak_power(runs):
    """f_t >= f_e x P_avg / P_pk: a face cannot radiate its energy faster than at full peak power."""
    for n, res in runs.items():
        p_avg = res["global"]["face_avg_rf_kW"] * 1e3
        for k, f in res["faces_total"].items():
            assert f["time_fraction"] >= f["energy_fraction"] * p_avg / rm.BAND_S.p_pk_total_w * (1 - 1e-9), (n, k)


def test_longest_pulse_follows_the_trm_buffer(tmp_path):
    """The longest S candidate pulse is derived from the buffer (frac x E x PAE / P_pk), and solve() accepts it."""
    assert max(mo.TAU_CAND) == pytest.approx(0.5 * 5.0 * 0.75 / 2000.0) == rm.tau_max_s(rm.BAND_S)
    rm.solve(mo.make_wf("t", max(mo.TAU_CAND), 1, 1e-3, 75.0, 1, "NONCOHERENT"), 2.8, 0.0, rm.TARGETS["ST"])
    text = _replace_value(_default_toml(), "band.S", "e_buf_trm_j ", "5.0", "2.0")
    out = _run_with_params(text, "from core import optimizer as mo\nprint(max(mo.TAU_CAND))", tmp_path)
    assert float(out) == pytest.approx(0.5 * 2.0 * 0.75 / 2000.0)


def test_no_coherent_dwell_beyond_the_coherence_floor():
    """INTERIM until step 2.2: past COH_T_MAX_S the eta_min floor would credit N tau without limit."""
    assert rm.COH_T_MAX_S == pytest.approx(rm.COH_T0_S * (-math.log(rm.COH_ETA_MIN)) ** (1 / rm.COH_P))
    assert rm.coherence_loss_db(2.8, rm.COH_T_MAX_S * 0.999) < rm.coherence_loss_db(2.8, rm.COH_T_MAX_S * 1.5)
    for w in mo.candidate_waveforms({"range_km": 8000.0}) + mo.candidate_waveforms({"range_km": 500.0}):
        assert w.integration != "COHERENT" or w.dwell_s <= rm.COH_T_MAX_S
    for det, _ in trk._ku_candidates("COHERENT", None):
        assert det.dwell_s <= rm.COH_T_MAX_S


def test_estimator_constants_share_one_margin_over_the_bounds():
    """k_theta, k_f, k_r = Cramer-Rao bound x the same 1.13 margin (review 0.3)."""
    margin = rm.K_THETA / 0.44
    assert rm.K_F == pytest.approx(0.551 * margin, abs=0.03) and rm.RANGE_CRLB_KR == pytest.approx(0.551 * margin, abs=0.03)
    assert rm.K_F > 0.551 and rm.RANGE_CRLB_KR > 0.551           # never below the bound


def test_power_scale_never_above_full_power(runs):
    """power_scale is reported rounded to 3 decimals, so a strong back-off ((20/228)^4) shows as 0.0."""
    for res in runs.values():
        for s in res["sectors"]:
            for b in s.get("wf_by_bin", []):
                assert 0.0 <= b["power_scale"] <= 1.0


def test_no_frequency_above_scan_break_when_it_leaves_sky_unscanned(runs):
    """Optimiser rule (SPEC v2 defect 3): a frequency that leaves more of the sector unscanned never wins."""
    for s in runs["mission_veille500.json"]["sectors"]:
        assert s["wf"]["freq_ghz"] <= rm.BAND_S.scan_break_ghz, s["id"]


def test_tracking_loads_add_up(runs):
    for res in runs.values():
        for k, f in res["tracking"]["faces"].items():
            assert f["total_energy"] == pytest.approx(f["search_energy"] + f["confirm_energy"] + f["at_energy"], abs=1e-12)
            assert f["total_ku_time"] == pytest.approx(f["confirm_ku_time"] + f["at_ku_time"], abs=1e-12)
            assert f["load"] == pytest.approx(max(f["total_energy"], f["total_s_time"], f["total_ku_time"]), abs=1e-12)


def test_parallel_run_equals_sequential():
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_redundancy_demo.json"))
    assert json.dumps(mo.run(m, workers=1), sort_keys=True) == json.dumps(mo.run(m, workers=2), sort_keys=True)


# ----------------------------------------------------------------------------- confirmation
def test_confirmation_probability_grows_with_margin():
    for sw in (1, 3):
        succ, _ = trk.confirm_tables(sw, 2.8)["KU1+7"]
        assert all(b >= a - 1e-12 for a, b in zip(succ, succ[1:]))


# ----------------------------------------------------------------------------- parameter file
def test_parameter_file_entries_are_complete():
    n, counts = params.summary(params.P)
    assert n > 0 and sum(counts.values()) == n
    for p in params.P:
        if not p.path.startswith("tool."):
            assert p.status in params.STATUSES and p.source, p.path


def test_every_parameter_is_read_by_the_code():
    """An entry nothing reads is a dead value (renamed or forgotten in the code)."""
    import gui.main_window  # noqa: F401  (GUI defaults)
    spec = importlib.util.spec_from_file_location("gen_cost_table", os.path.join(tb.ROOT, "tools", "gen_cost_table.py"))
    spec.loader.exec_module(importlib.util.module_from_spec(spec))
    unread = sorted(p.path for p in params.P if p.path not in params.P.read_keys and p.path != "meta.file_version")
    assert not unread, f"parameters read by no module: {unread}"


@pytest.mark.parametrize("text,msg", [
    ('[detection]\npd = 0.9\n', "bare value"),
    ('[detection]\npd = { value = 0.9, unit = "-", status = "X", source = "s", date = "" }\n', "status 'X'"),
    ('[detection]\npd = { value = 0.9, unit = "-", status = "V", source = "s" }\n', "misses ['date']"),
    ('[detection]\npd = { value = 0.9, unit = "-", status = "V", source = "s", date = "", typo = 1 }\n', "unknown keys"),
    ('[detection\n', "radar_system.toml"),
])
def test_invalid_parameter_file_is_refused(tmp_path, text, msg):
    path = tmp_path / "radar_system.toml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(params.ParamError, match=msg.replace("[", r"\[").replace("]", r"\]")):
        params.load(str(path))


def test_missing_parameter_file_is_refused(tmp_path):
    with pytest.raises(params.ParamError, match="not found"):
        params.load(str(tmp_path / "none.toml"))


def test_missing_parameter_names_the_file():
    with pytest.raises(KeyError, match="radar_system.toml"):
        params.P.get("detection.no_such_value")


# ----------------------------------------------------------------------------- derived files
def test_cost_table_matches_the_parameter_file():
    """cost_table.json is built from the parameter file: same grid and powers (else rerun tools/gen_cost_table.py)."""
    t = ce.load_table()
    g = "tool.cost_table."
    assert (t.R, t.RCS, t.V, t.TH) == (params.P.get(g + "range_km"), params.P.get(g + "rcs_m2"),
                                       params.P.get(g + "v_mps"), params.P.get(g + "theta_deg"))
    assert t.is_current, "radar_system.toml changed since cost_table.json was built: python tools/gen_cost_table.py"
    assert t.ppk == rm.BAND_S.p_pk_total_w
    assert t.pavg_ref_w == pytest.approx(rm.face_avg_rf_w(params.P.get("power.face_elec_mw")), rel=1e-12)


# ----------------------------------------------------------------------------- mission validation
def test_default_sectors_and_brackets_are_valid():
    m = md.empty_mission()
    m["sectors"] = [md.new_sector([], "box"), md.new_sector(["S01"], "cap")]
    m["sectors"][0]["tracking"] = [md.new_bracket(m["sectors"][0])]
    assert md.validate_mission(m) == []


def test_test_missions_are_valid():
    for n in tb.MISSIONS:
        assert md.validate_mission(md.load_mission(os.path.join(tb.MISSIONS_DIR, n))) == [], n


@pytest.mark.parametrize("change,msg", [
    ({"phi_max_deg": 0.0, "phi_min_deg": 0.0}, "zero-width"),
    ({"v_close_max_mps": 0.0, "v_close_min_mps": 0.0}, "v_close_max_mps must be > 0"),
    ({"eps_min_deg": 50.0, "eps_max_deg": 10.0}, "empty sector"),
    ({"swerling": 7}, "swerling"),
    ({"tracking": [{**md.BRACKET_DEFAULTS, "r_max_km": 900.0}, {**md.BRACKET_DEFAULTS, "r_min_km": 500.0}]}, "overlap"),
    ({"tracking": [{**md.BRACKET_DEFAULTS, "r_max_km": 5000.0}]}, "beyond the required detection range"),
])
def test_invalid_sector_is_refused(change, msg):
    m = md.empty_mission()
    m["sectors"] = [{**md.new_sector([], "box"), **change}]
    errs = md.validate_mission(m)
    assert any(msg in e for e in errs), errs


def test_face_power_above_ship_total_is_refused():
    errs = md.validate_global({**md.GLOBAL_DEFAULTS, "face_elec_mw": md.SHIP_ELEC_TOTAL_MW + 0.1})
    assert any("ship total" in e for e in errs)


def test_gui_modules_import():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import gui.main_window  # noqa: F401
    import gui.prep_tab  # noqa: F401
    import gui.results_tab  # noqa: F401
