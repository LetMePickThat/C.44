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

import numpy as np
import pytest

import testbench as tb
from core import antenna
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
    text = _replace_value(text, "band.S", "e_buf_trm_j ", "25.0", "400.0")
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
    """196 bi-band TRSAs (25 x 25 Ku, 5 x 5 S) in an octagon, dead gaps of 0.5 mm (2 mm in the design note, review 0.3;
    0.5 mm since step 1.2)."""
    assert rm.N_TRSA == 196 and rm.N_TRSA_GAPS == 2 * 180
    assert rm.BAND_S.n_trm == 4900 and rm.BAND_KU.n_trm == 122500
    assert rm.TRSA_SIDE_M == pytest.approx(0.260) and rm.S_TRM_PITCH_M == pytest.approx(0.052)
    assert rm.PHYS_AREA_M2 == pytest.approx(13.2496, rel=1e-9)
    assert rm.TRSA_GAP_M == pytest.approx(0.0005)
    assert rm.TOTAL_AREA_M2 == pytest.approx(13.2496 + 360 * 0.0005 * 0.26, rel=1e-9)
    assert rm.FILL_FACTOR == pytest.approx(0.99648, abs=5e-5)
    assert rm.ARRAY_ENVELOPE_M == pytest.approx(16 * 0.26 + 15 * 0.0005, rel=1e-9)


# ----------------------------------------------------------------------------- radiation pattern (step 1.1)
def test_special_functions_of_the_taylor_law():
    assert float(antenna.bessel_j(0, 0.0)) == pytest.approx(1.0, abs=1e-14)
    assert antenna.j1_zeros(3) == pytest.approx([3.831705970, 7.015586670, 10.173468135], abs=1e-8)
    assert float(antenna.bessel_j(1, 2.0)) == pytest.approx(0.576724808, abs=1e-9)


def test_tx_beam_of_the_uniform_octagon():
    """Uniform octagon: eta 1, HPBW ~1.04 lambda / D_env (0.886 was the rectangular value), nearly circular, first
    sidelobe ~-15.6 dB; S and Ku sample the same aperture so they agree within 0.1 %."""
    for b in ("S", "Ku"):
        tx = rm.PATTERNS[b].tx
        assert tx.eta == pytest.approx(1.0, abs=1e-12) and tx.design_sll_db is None
        assert 1.03 < tx.hpbw <= tx.hpbw_x <= tx.hpbw_max < 1.05 and tx.hpbw_max / tx.hpbw < 1.01
        assert -16.5 < tx.sll_db < -15.0
    assert rm.PATTERNS["S"].tx.hpbw == pytest.approx(rm.PATTERNS["Ku"].tx.hpbw, rel=1e-3)
    assert rm.PATTERNS["S"].n_elements == rm.BAND_S.n_trm and rm.PATTERNS["Ku"].n_elements == rm.BAND_KU.n_trm


def test_rx_weighting_reaches_each_sidelobe_option():
    """Achieved peak sidelobe level within [target - tol, target]; lower level -> lower eta, wider beam."""
    for b in ("S", "Ku"):
        beams = [rm.rx_beam(b, s) for s in sorted(rm.RX_SLL_OPTIONS_DB, reverse=True)]
        for s, beam in zip(sorted(rm.RX_SLL_OPTIONS_DB, reverse=True), beams):
            assert s - antenna.FIT_TOL_DB - 1e-9 <= beam.sll_db <= s + 1e-9, (b, s, beam.sll_db)
            assert beam.design_sll_db > -s                          # the octagon needs a deeper design level
        assert all(x.eta > y.eta and x.hpbw < y.hpbw for x, y in zip(beams, beams[1:]))
        assert all(rm.PATTERNS[b].tx.hpbw < x.hpbw for x in beams)
    with pytest.raises(ValueError):
        rm.rx_beam("S", -28.0)


def test_pattern_cache_is_current():
    """core/antenna_cache.json matches the parameter file (else the tool recomputes it at every start)."""
    _, key = antenna.input_key(rm.TRSA_ROWS, rm.TRSA_SIDE_M, rm.TRSA_GAP_M,
                               {"S": rm.S_TRM_PER_TRSA_SIDE, "Ku": rm.KU_TRM_PER_TRSA_SIDE}, rm.ARRAY_ENVELOPE_M,
                               rm.RX_SLL_OPTIONS_DB, params.P.get("aperture.rx_taper_nbar"),
                               params.P.get("aperture.rx_taper_edge"), {b: rm._visible_r_lam_d(b) for b in ("S", "Ku")})
    assert antenna._read_cache(key) is not None


def test_pattern_follows_the_build_up_without_touching_the_cache(tmp_path):
    """A smaller octagon -> wider beam in degrees, about the same in lambda / D_env (same shape); a modified copy of the
    parameter file never rewrites the committed cache."""
    before = open(antenna.CACHE_PATH, encoding="utf-8").read()
    text = _replace_value(_default_toml(), "aperture", "trsa_rows ", "[6, 8, 10, 12, 14, 16, 16, 16, 16, 16, 16, 14, 12, 10, 8, 6]",
                          "[4, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6, 4]")
    out = _run_with_params(text, "from core import radar_model as rm; print(rm.PATTERNS['S'].tx.hpbw, rm.N_TRSA, "
                                 "rm.hpbw_deg(2.8))", tmp_path)
    hpbw, n, deg = out.split()
    assert int(n) == 104 and float(deg) > rm.hpbw_deg(2.8) * 1.2
    assert float(hpbw) == pytest.approx(rm.PATTERNS["S"].tx.hpbw, rel=0.03)
    assert open(antenna.CACHE_PATH, encoding="utf-8").read() == before


def test_angle_accuracy_is_the_cramer_rao_bound_times_the_margin():
    """sigma_u = margin x lambda / (2 pi x_rms sqrt(2 SNR)), independent of the RX weighting; x_rms of a filled
    octagon ~ D / 4 (disc: R / 2)."""
    p = rm.PATTERNS["S"]
    assert p.x_rms_m / rm.ARRAY_ENVELOPE_M == pytest.approx(0.248, abs=0.002)
    assert rm.ANGLE_CRLB_MARGIN == pytest.approx(1.13)
    assert rm.angle_k_tx("S") == pytest.approx(1.13 * 0.436, abs=0.003)
    pack = rm.TrackingPack(0.01, 1e6)
    for f, th in ((14.0, 0.0), (14.0, 60.0), (2.8, 30.0)):
        snr, dom = rm.tracking_snr_req(pack, f, th, 1e-3, 75.0)
        sigma_u = math.radians(0.01) * math.cos(math.radians(th))
        x = rm.PATTERNS[rm.band_for_frequency(f).name].x_rms_m
        assert dom == "ANG" and snr == pytest.approx(0.5 * (1.13 * rm.lambda_m(f) / (2 * math.pi * x * sigma_u)) ** 2)


def test_rx_weighting_only_scales_the_rx_gain():
    """Range ratio between two options = (eta ratio)^(1/4), in search (S) and tracking (Ku); positions unchanged."""
    for wf, f, tgt in ((rm.WF_LRS, 2.8, "ST"), (tb.KU_TRACK, 14.0, "KKV")):
        r25 = rm.solve(wf, f, 30.0, rm.TARGETS[tgt], rx_sll_db=-25.0).range_km
        r35 = rm.solve(wf, f, 30.0, rm.TARGETS[tgt], rx_sll_db=-35.0).range_km
        band = rm.band_for_frequency(f).name
        assert r35 / r25 == pytest.approx((rm.rx_beam(band, -35.0).eta / rm.rx_beam(band, -25.0).eta) ** 0.25, rel=1e-12)
    assert rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"]).range_km == \
        rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], rx_sll_db=rm.RX_SLL_DEFAULT_DB).range_km


def test_search_has_the_rx_grid_loss_and_counts_rx_beams():
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"])
    assert r.losses["grid_rx"] == pytest.approx(1.672 * 0.25, abs=1e-3)
    n_rx = r.frame["rx_beams_per_position"]
    assert n_rx == pytest.approx((1.04 * rm.PATTERNS["S"].tx.hpbw / (0.5 * rm.rx_beam("S").hpbw)) ** 2)
    assert 3.0 < n_rx < 4.0 and rm.rx_beams_per_position(2.8, -35.0) < n_rx


def test_mission_rx_sidelobe_setting(tmp_path):
    """Per-sector option: default when absent, validated, kept by the CSV round trip, used by the optimiser."""
    m = md.empty_mission()
    m["sectors"].append(md.new_sector([]))
    assert m["sectors"][0]["rx_sll_db"] == rm.RX_SLL_DEFAULT_DB == -25.0
    m["sectors"][0]["rx_sll_db"] = -28.0
    assert any("rx_sll_db" in e for e in md.validate_mission(m))
    m["sectors"][0]["rx_sll_db"] = -35.0
    assert md.validate_mission(m) == []
    path = str(tmp_path / "m.csv")
    md.save_mission_csv(path, m)
    assert md.load_mission(path)["sectors"][0]["rx_sll_db"] == -35.0
    sec = dict(m["sectors"][0], range_km=1500.0)
    r35 = mo.run({"global": m["global"], "sectors": [sec]}, workers=1)["sectors"][0]
    r25 = mo.run({"global": m["global"], "sectors": [dict(sec, rx_sll_db=-25.0)]}, workers=1)["sectors"][0]
    assert r35["rx_sll_db"] == -35.0 and r25["rx_sll_db"] == -25.0
    e = lambda r: sum(f["energy_J_per_refresh"] for f in r["faces"].values())   # noqa: E731
    assert e(r35) > e(r25)                                    # less RX gain -> more energy for the same range


def test_fast_estimate_follows_the_rx_setting():
    """Equivalent-range scaling of the cost table: -35 dB costs more energy than the table's -25 dB."""
    table = ce.load_table()
    from core import geometry as geo
    grid = geo.SphereGrid(res_deg=tb.GRID_RES_DEG)
    sec = md.new_sector([])
    p = rm.face_elec_avail_w(2.0)
    a = ce.estimate_sector_faces(table, sec, grid, -90.0, 90.0, p)
    b = ce.estimate_sector_faces(table, dict(sec, rx_sll_db=-35.0), grid, -90.0, 90.0, p)
    ea, eb = (sum(f.energy_j_per_refresh for f in x.values()) for x in (a, b))
    assert table.rx_sll_db == rm.RX_SLL_DEFAULT_DB and eb > ea


# ----------------------------------------------------------------------------- active mismatch, far lobes (step 1.2)
def test_active_mismatch_current_sheet_with_waim():
    """|G_waim| = |cos t - cos t_m| / (cos t + cos t_m), t_m = 45 deg in both bands; one model for S and Ku."""
    for f in (2.8, 14.0):
        assert rm.band_for_frequency(f).waim_match_deg == 45.0
        for th in (0.0, 20.0, 45.0, 60.0, 70.0, 75.0):
            c, cm = math.cos(math.radians(th)), math.cos(math.radians(45.0))
            assert rm.active_mismatch(f, th)["gamma_waim"] == pytest.approx(abs(c - cm) / (c + cm), abs=1e-12)
    assert rm.active_mismatch(2.8, 45.0)["loss_db_per_pass"] == pytest.approx(0.0, abs=1e-9)
    assert rm.active_mismatch(2.8, 70.0)["loss_db_per_pass"] == pytest.approx(0.56, abs=0.01)
    # S and Ku share the lattice in lambda units: same mismatch at the same relative frequency
    assert rm.active_mismatch(2.9, 70.0)["loss_db_per_pass"] == pytest.approx(rm.active_mismatch(14.5, 70.0)["loss_db_per_pass"])


def test_grating_lobe_term_explains_the_70_deg_limit():
    """Mean pitch (TRSA pitch / TRMs per side) sets the grating lobes; inside the locked scan limits the margin stays
    positive and the mismatch below the 3 dB alert; at 2.9 GHz / 75 deg (outside the limit) the grating-lobe term
    adds to the WAIM one."""
    p = rm.TRSA_SIDE_M + rm.TRSA_GAP_M
    assert rm.PATTERNS["S"].d_mean_m == pytest.approx(p / 5) and rm.PATTERNS["Ku"].d_mean_m == pytest.approx(p / 25)
    assert rm.grating_lobe_margin(2.9, 75.0) == pytest.approx(rm.lambda_m(2.9) * 5 / p - math.sin(math.radians(75)) - 1)
    am = rm.active_mismatch(2.9, 75.0)
    assert am["gamma_gl"] == pytest.approx(math.exp(-am["margin_g"] / 0.02))
    assert am["loss_db_per_pass"] > rm.active_mismatch(2.85, 75.0)["loss_db_per_pass"]   # grating lobe approaching
    for band in rm.BANDS.values():
        for i in range(21):
            f = band.usable_lo_ghz + i * (band.usable_hi_ghz - band.usable_lo_ghz) / 20
            lim = band.scan_limit_deg(f)
            assert rm.grating_lobe_margin(f, lim) > 0
            assert rm.active_mismatch(f, lim)["loss_db_per_pass"] < rm.MISMATCH_ALERT_DB


def test_solve_applies_the_mismatch_on_tx_and_rx():
    for f, th in ((2.8, 0.0), (2.85, 75.0), (14.0, 60.0)):
        wf = rm.WF_LRS if f < 3 else tb.KU_TRACK
        r = rm.solve(wf, f, th, rm.TARGETS["KKV"])
        assert r.losses["scan_mismatch"] == pytest.approx(2 * rm.active_mismatch(f, th)["loss_db_per_pass"])


def test_gap_lobes_next_to_the_grating_lobe_match_the_subarray_factor():
    """TRSA pitch P = side + gap: lobes at k lambda / P weighted by the factor of the n uniform TRMs of one TRSA. Where
    they dominate (Ku, 2 mm gaps: the step-1.2 finding) the one next to the grating lobe (k = n - 1) is the analytic
    -16.7 dB; with the 0.5 mm gaps it is far weaker, and the far scan of the model sees at least that lobe."""
    from core import antenna

    def lobe_db(gap: float) -> float:
        xs, ys, mult, present, _ = antenna._quadrant(rm.TRSA_ROWS, rm.TRSA_SIDE_M, gap, 25)
        lam, wm, p = rm.lambda_m(14.0), (mult * present).sum(axis=0), rm.TRSA_SIDE_M + gap
        u = 24 * lam / p + np.linspace(-1.0, 1.0, 2001) * lam / (16 * rm.TRSA_SIDE_M)
        return 20 * math.log10((np.abs(np.cos(2 * math.pi * np.outer(u, xs) / lam) @ wm) / wm.sum()).max())

    d, p = rm.TRSA_SIDE_M / 25, rm.TRSA_SIDE_M + 0.002
    x = math.pi * d * 24 / p
    assert lobe_db(0.002) == pytest.approx(20 * math.log10(abs(math.sin(25 * x) / (25 * math.sin(x)))), abs=0.3)
    now = lobe_db(rm.TRSA_GAP_M)
    assert now < lobe_db(0.002) - 6.0 and rm.PATTERNS["Ku"].tx.far_sll_db >= now - 0.3
    prof = rm.PATTERNS["Ku"].tx.far_profile_db
    assert all(a <= b for a, b in zip(prof, prof[1:]))           # cumulative profile


def test_visible_sidelobes_and_held_scan_angle():
    """Near the axis the visible sidelobe is the near one; the far gap lobes enter with the scan (worst azimuth)."""
    for b, f in (("S", 2.8), ("Ku", 14.0)):
        assert rm.sidelobe_visible_db(f, 0.0, "tx") == pytest.approx(rm.PATTERNS[b].tx.sll_db)
        for s in rm.RX_SLL_OPTIONS_DB:
            vis = [rm.sidelobe_visible_db(f, th, "rx", s) for th in range(0, 76, 5)]
            assert all(a <= b_ for a, b_ in zip(vis, vis[1:]))
    for s in rm.RX_SLL_OPTIONS_DB:                                     # 0.5 mm gaps: every S option over the sector
        assert rm.rx_sll_held_up_to_deg(2.8, s, 75.0) == 75.0
    assert rm.rx_sll_held_up_to_deg(14.0, -25.0, 75.0) == 75.0
    held = rm.rx_sll_held_up_to_deg(14.0, -35.0, 75.0)
    assert 55.0 < held < 70.0 and rm.sidelobe_visible_db(14.0, held + 0.5, "rx", -35.0) > -35.0 + antenna.FIT_TOL_DB


def test_tx_polarisation_purity_ludwig3():
    """Linear along e_v: no cross-pol in the principal planes, (1 + cos t) / (1 - cos t) at 45 deg azimuth."""
    assert rm.TX_POLARISATION == "linear_ev"
    assert rm.pol_purity(70.0, 0.0)["xpd_db"] == math.inf and rm.pol_purity(70.0, 90.0)["xpd_db"] > 200
    for th in (30.0, 60.0):
        c = math.cos(math.radians(th))
        assert rm.pol_purity(th, 45.0)["xpd_db"] == pytest.approx(20 * math.log10((1 + c) / (1 - c)))


def test_sector_alerts_of_step_1_2(runs):
    """Default -25 dB held everywhere (no flag); a -35 dB volume reaching large Ku scans gets the Ku flag."""
    res = runs["mission_example.json"]
    s03 = next(s for s in res["sectors"] if s["id"] == "S03")
    assert s03["rx_sll_held_up_to_deg"] == pytest.approx(min(75.0, max(f["theta_max_deg"] for f in s03["faces"].values())))
    assert s03["mismatch_db_per_pass_max"] < rm.MISMATCH_ALERT_DB
    assert not any("held only" in w for w in res["warnings"])
    sec = dict(md.new_sector([]), rx_sll_db=-35.0, eps_min_deg=-60.0, eps_max_deg=60.0, phi_min_deg=0.0,
               phi_max_deg=90.0, range_km=500.0)
    r = mo.run({"global": md.empty_mission()["global"], "sectors": [sec]}, workers=1)
    assert any("Ku RX sidelobes -35 dB held only" in w for w in r["warnings"])


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
    # 4308 at 20 deg in the model history (V6.0.10, HPBW 0.886 lambda / D, D = 4.19 m), x (0.886 / HPBW_TX)^2 since
    # step 1.1, x (D_env / 4.19)^2 since the 0.5 mm gaps (step 1.2)
    assert abs(pos - 4308 * (0.886 / rm.PATTERNS["S"].tx.hpbw) ** 2 * (rm.ARRAY_ENVELOPE_M / 4.19) ** 2) < 25
    from core import geometry as geo
    assert rp < geo.RP                  # arrays sit lower on the cone at a smaller tilt


# ----------------------------------------------------------------------------- optimiser invariants
@pytest.fixture(scope="module")
def runs():
    return {n: mo.run(md.load_mission(os.path.join(tb.MISSIONS_DIR, n)), workers=1) for n in tb.MISSIONS}


def test_time_on_air_at_least_energy_over_peak_power(runs):
    """f_t >= f_e x P_avg / P_pk: a face cannot radiate its energy faster than at full peak power."""
    for n, res in runs.items():
        p_avg = res["global"]["face_elec_avail_kW"] * 1e3
        for k, f in res["faces_total"].items():
            # electrical budget (step 1.3): RF energy >= electrical x the lowest PAE of the S search space
            pae_min = rm.pae(rm.BAND_S, min(mo.TAU_CAND), rm.BAND_S.duty_max)
            assert f["time_fraction"] >= f["energy_fraction"] * p_avg * pae_min / rm.BAND_S.p_pk_total_w * (1 - 1e-9), (n, k)


def test_longest_pulse_follows_the_trm_buffer(tmp_path):
    """The longest S candidate pulse is derived from the buffer (frac x E x pae_sat / P_pk - t_edge), and solve()
    accepts it."""
    assert max(mo.TAU_CAND) == pytest.approx(0.2 * 25.0 * 0.8 / 2000.0 - 0.2e-6) == rm.tau_max_s(rm.BAND_S)
    rm.solve(mo.make_wf("t", max(mo.TAU_CAND), 1, 3e-3, 75.0, 1, "NONCOHERENT"), 2.8, 0.0, rm.TARGETS["ST"])
    text = _replace_value(_default_toml(), "band.S", "e_buf_trm_j ", "25.0", "10.0")
    out = _run_with_params(text, "from core import optimizer as mo\nprint(max(mo.TAU_CAND))", tmp_path)
    assert float(out) == pytest.approx(0.2 * 10.0 * 0.8 / 2000.0 - 0.2e-6)


# ----------------------------------------------------------------------------- TRM and transmit chain (step 1.3)
def test_pae_law_per_band():
    """pae_sat x tau / (tau + t_edge) x min(1, 1 - a (d - d_ref)); single pulse at d_ref; capped below d_ref."""
    for b, sat, edge, a in ((rm.BAND_S, 0.80, 0.2e-6, 0.05), (rm.BAND_KU, 0.65, 0.1e-6, 0.08)):
        assert (b.pae_sat, b.pae_t_edge_s, b.pae_duty_slope) == (sat, edge, a)
        for tau in (10e-6, 1e-3):
            for d in (0.3, 0.65):
                assert rm.pae(b, tau, d) == pytest.approx(sat * tau / (tau + edge) * (1 - a * (d - 0.10)))
            assert rm.pae(b, tau, 0.01) == rm.pae(b, tau) == pytest.approx(sat * tau / (tau + edge))
    w = rm.Waveform("t", "SEARCH", "S", 100e-6, 8, 400e-6, 10.0)
    assert rm.pae_wf(w) == rm.pae(rm.BAND_S, 100e-6, 0.25)
    assert rm.pae_wf(rm.Waveform("t", "SEARCH", "S", 100e-6, 1, 1e-3, 10.0)) == rm.pae(rm.BAND_S, 100e-6)


def test_duty_ceiling_on_trains_only():
    assert rm.BAND_S.duty_max == rm.BAND_KU.duty_max == 0.65
    rm.Waveform("ok", "SEARCH", "S", 650e-6, 4, 1e-3, 10.0)
    with pytest.raises(ValueError, match="duty"):
        rm.Waveform("hot", "SEARCH", "S", 700e-6, 4, 1e-3, 10.0)
    rm.Waveform("single", "SEARCH", "S", 900e-6, 1, 1e-3, 10.0)     # one pulse: no train, no duty limit
    assert all(w.n == 1 or w.duty <= 0.65 + 1e-12 for w in mo.candidate_waveforms({"range_km": 150.0}))


def test_buffer_droop_and_tau_max_from_the_buffer():
    """x = P_pk tau / PAE / E_buf; -10 log(1 - x / 2); the longest pulse sits on buffer_max_fraction = 0.2."""
    w = rm.WF_LRS
    x = 2000.0 * w.tau_s / rm.pae_wf(w) / 25.0
    assert rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"]).losses["buffer_droop"] == pytest.approx(-10 * math.log10(1 - x / 2))
    for b in (rm.BAND_S, rm.BAND_KU):
        tmax = rm.tau_max_s(b)
        e = b.p_pk_trm_w * tmax / rm.pae(b, tmax)
        assert e / b.e_buf_trm_j == pytest.approx(0.2)
    assert rm.tau_max_s(rm.BAND_S) == pytest.approx(2.0e-3, abs=1e-6) and rm.tau_max_s(rm.BAND_KU) == pytest.approx(2.08e-3, abs=1e-6)


def test_electrical_budget_and_dissipation(runs):
    """The face has P_elec x CHAIN_EFF; each look costs RF / PAE; the heat is electrical - RF (> 0, < electrical)."""
    assert rm.face_elec_avail_w(2.0) == pytest.approx(2e6 * rm.CHAIN_EFF)
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"])
    assert r.p_avg_used_w == pytest.approx(rm.BAND_S.p_pk_total_w * rm.WF_LRS.duty / r.pae)
    assert r.p_diss_trm_w == pytest.approx(2000.0 * rm.WF_LRS.duty * (1 / r.pae - 1))
    res = runs["mission_example.json"]
    for k, fc in res["tracking"]["faces"].items():
        if res["faces_total"][k]["positions"] > 0:
            elec_kw = fc["total_energy"] * res["global"]["face_elec_avail_kW"]
            assert 0.0 < fc["dissipation_kW"] < elec_kw
            assert fc["dissipation_kW_m2"] == pytest.approx(fc["dissipation_kW"] / rm.TOTAL_AREA_M2)
    look = trk.ku_look(30.0, rm.TARGETS["KKV"], 500.0, rm.face_elec_avail_w(2.0))
    assert look["energy_J"] == pytest.approx(look["energy_rf_J"] / look["pae"]) and look["pae"] < rm.BAND_KU.pae_sat + 1e-12



def test_no_coherent_dwell_beyond_the_coherence_floor():
    """INTERIM until step 2.2: past COH_T_MAX_S the eta_min floor would credit N tau without limit."""
    assert rm.COH_T_MAX_S == pytest.approx(rm.COH_T0_S * (-math.log(rm.COH_ETA_MIN)) ** (1 / rm.COH_P))
    assert rm.coherence_loss_db(2.8, rm.COH_T_MAX_S * 0.999) < rm.coherence_loss_db(2.8, rm.COH_T_MAX_S * 1.5)
    for w in mo.candidate_waveforms({"range_km": 8000.0}) + mo.candidate_waveforms({"range_km": 500.0}):
        assert w.integration != "COHERENT" or w.dwell_s <= rm.COH_T_MAX_S
    for det, _ in trk._ku_candidates("COHERENT", None):
        assert det.dwell_s <= rm.COH_T_MAX_S


def test_estimator_constants_share_one_margin_over_the_bounds():
    """Angle, k_f, k_r = Cramer-Rao bound x the same 1.13 margin (review 0.3, step 1.1)."""
    margin = rm.ANGLE_CRLB_MARGIN
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
    assert t.pavg_ref_w == pytest.approx(rm.face_elec_avail_w(params.P.get("power.face_elec_mw")), rel=1e-12)


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
