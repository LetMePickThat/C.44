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


# ----------------------------------------------------------------------------- the band (step 1.4)
def test_edge_law_from_the_band_entries():
    """Raised cosine between the core and usable edges (no loss in the core), max edge_derating_db at the edges,
    averaged over the occupied spectrum (exact integral = numerical mean)."""
    for b in rm.BANDS.values():
        assert b.usable_lo_ghz < b.core_lo_ghz < b.core_hi_ghz < b.usable_hi_ghz
        mid = 0.5 * (b.core_lo_ghz + b.core_hi_ghz)
        assert b.edge_loss_db(mid) == 0.0
        assert b.edge_loss_db(b.usable_lo_ghz) == pytest.approx(b.edge_derating_db)
        assert b.edge_loss_db(b.usable_hi_ghz) == pytest.approx(b.edge_derating_db)
        f = 0.5 * (b.usable_lo_ghz + b.core_lo_ghz)
        assert b.edge_loss_db(f) == pytest.approx(b.edge_derating_db / 2)
        for f0, bw in ((f, 20.0), (b.core_hi_ghz, 40.0), (mid, 50.0)):
            pts = [f0 + (k + 0.5 - 2000) * bw * 1e-3 / 4000 for k in range(4000)]
            assert b.edge_loss_db(f0, bw) == pytest.approx(sum(b.edge_loss_db(p) for p in pts) / 4000, abs=1e-6)


def test_signal_must_fit_in_the_usable_band():
    s = rm.BAND_S
    assert s.fits(s.usable_lo_ghz + 0.005, 10.0) and not s.fits(s.usable_lo_ghz + 0.004, 10.0)
    assert s.fits(0.5 * (s.usable_lo_ghz + s.usable_hi_ghz), s.bi_max_mhz)
    w = rm.Waveform("w", "SEARCH", "S", 100e-6, 1, 1e-3, 75.0)
    with pytest.raises(ValueError, match="leaves the usable"):
        rm.solve(w, s.usable_lo_ghz, 0.0, rm.TARGETS["ST"])
    rm.solve(w, s.usable_lo_ghz + 0.0375, 0.0, rm.TARGETS["ST"])
    assert rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"]).losses["edge"] == 0.0


def test_true_time_delay_entry_and_phase_steering_limit(tmp_path):
    """Digital TTD: no bandwidth limit with scan; 'none' limits B to c / (D_env sin theta)."""
    assert rm.BAND_S.true_time_delay == rm.BAND_KU.true_time_delay == "digital_per_trm"
    text = _replace_value(_default_toml(), "band.S", "true_time_delay ", '"digital_per_trm"', '"none"')
    code = ("from core import radar_model as rm\n"
            "w = rm.Waveform('w', 'SEARCH', 'S', 100e-6, 1, 1e-3, 75.0)\n"
            "rm.solve(w, 2.8, 60.0, rm.TARGETS['ST'])\n"
            "try:\n    rm.solve(w, 2.8, 75.0, rm.TARGETS['ST']); print('accepted')\n"
            "except ValueError as e:\n    print('refused', 'aperture-fill' in str(e))")
    assert _run_with_params(text, code, tmp_path).split() == ["refused", "True"]


def test_band_can_be_redefined_in_the_file(tmp_path):
    """A wider S band (better TRMs) in the parameter file alone: the model, the optimiser and the search run, and an
    inconsistent band is refused at load time with a clear message."""
    text = _replace_value(_default_toml(), "band.S", "usable_lo_ghz ", "2.7", "2.6")
    text = _replace_value(text, "band.S", "usable_hi_ghz ", "2.9", "3.0")
    code = ("from core import radar_model as rm, optimizer as mo, mission as md\n"
            "print(rm.BAND_S.fits(2.7, 75.0), rm.BAND_S.edge_loss_db(2.6))\n"
            "m = md.empty_mission(); m['sectors'].append(md.new_sector([]))\n"
            "r = mo.run(m, workers=1); print(r['sectors'][0]['status'])")
    assert _run_with_params(text, code, tmp_path).split() == ["True", "0.5", "OK"]
    bad = _replace_value(_default_toml(), "band.S", "core_lo_ghz ", "2.74", "2.65")
    path = tmp_path / "bad.toml"
    path.write_text(bad, encoding="utf-8")
    res = subprocess.run([PYTHON, "-c", "import core.radar_model"], cwd=tb.ROOT, capture_output=True, text=True,
                         env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=120)
    assert res.returncode != 0 and "core band" in res.stderr


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
        a = rm.solve(wf, f, 30.0, rm.TARGETS[tgt], rx_sll_db=-25.0)
        b = rm.solve(wf, f, 30.0, rm.TARGETS[tgt], rx_sll_db=-35.0)
        band = rm.band_for_frequency(f).name
        # since step 2.1 the option also sets the range sidelobes: an LFM look pays its weighting (loss, k_r)
        extra_db = (b.l_tot_db - a.l_tot_db) + (b.snr_req_db - a.snr_req_db)
        if a.pulse["modulation"] == "nlfm":    # NLFM: only the range straddle follows the sidelobe level (step 2.2)
            assert extra_db == pytest.approx(rm.straddle_range_db(-35.0) - rm.straddle_range_db(-25.0), abs=1e-12)
        assert b.range_km / a.range_km == pytest.approx(
            (rm.rx_beam(band, -35.0).eta / rm.rx_beam(band, -25.0).eta * 10 ** (-extra_db / 10)) ** 0.25, rel=1e-12)
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
    """T_sys is derived from [band.<b>.rx] (Friis, cold sky since step 1.5) in both bands: a 1 dB worse S LNA must
    raise it accordingly."""
    assert rm.BAND_S.t_sys_k == pytest.approx(144.5, abs=0.1) and rm.BAND_KU.t_sys_k == pytest.approx(230.8, abs=0.1)
    assert "pre_LNA" not in rm.BAND_S.losses_db and "lumped_impl" not in rm.BAND_KU.losses_db
    assert set(rm.BAND_S.losses_db) - {"Ku_blockage"} == set(rm.BAND_KU.losses_db) - {"S_blockage"}   # same posts
    text = _replace_value(_default_toml(), "band.S.rx", "lna_nf_db ", "0.75", "1.75")
    out = _run_with_params(text, "from core import radar_model as rm\nprint(rm.BAND_S.t_sys_k)", tmp_path)
    l_pre = 10 ** (0.85 / 10)
    assert float(out) - rm.BAND_S.t_sys_k == pytest.approx(l_pre * 290 * (10 ** 0.175 - 10 ** 0.075) * 10 ** 0.005, rel=1e-6)


# ----------------------------------------------------------------------------- parametric waveforms (step 2.1)
def test_range_weighting_follows_the_volume_sidelobes():
    """Taylor nbar 5 weighting at the volume's RX option: loss, broadening and RMS bandwidth grow with the level;
    the achieved range sidelobes meet the option; k_r = RANGE_CRLB_KR / RMS ratio; NLFM: no weighting loss."""
    w = [rm.range_weighting(s) for s in rm.RX_SLL_OPTIONS_DB]
    assert all(a["loss_db"] < b["loss_db"] and a["broadening"] < b["broadening"] and a["rms_ratio"] > b["rms_ratio"]
               for a, b in zip(w, w[1:]))
    assert all(abs(x["psl_db"] - s) < 0.4 for x, s in zip(w, rm.RX_SLL_OPTIONS_DB))
    r = rm.solve(tb.KU_TRACK, 14.0, 0.0, rm.TARGETS["ST"], rx_sll_db=-35.0)
    assert r.pulse["range_psl_db"] == -35.0 and r.pulse["k_r"] == pytest.approx(0.6 / w[2]["rms_ratio"])
    assert r.pulse["range_resolution_m"] == pytest.approx(w[2]["broadening"] * rm.C_MPS / (2 * 75e6))


def test_modulation_choice_and_compression_loss():
    """NLFM while f_d,max / B <= 2 % (mismatch 0.1 dB), else LFM: weighting + Doppler loss on the ridge."""
    f_d = rm.doppler_max_hz(2.8, 25000.0)
    b_lim = f_d / rm.NLFM_DOPPLER_MAX_FRAC / 1e6
    assert rm.modulation_for(2.8, b_lim * 1.01, 25000.0) == "nlfm" and rm.modulation_for(2.8, b_lim * 0.99, 25000.0) == "lfm"
    assert rm.compression_loss_db("nlfm", 2.8, 75.0, 25000.0, -25.0) == rm.NLFM_MISMATCH_LOSS_DB
    lfm = rm.compression_loss_db("lfm", 2.8, 10.0, 25000.0, -25.0)
    assert lfm == pytest.approx(rm.range_weighting(-25.0)["loss_db"] - 20 * math.log10(1 - f_d / 10e6))
    w = rm.Waveform("w", "SEARCH", "S", 100e-6, 1, 1e-3, 10.0)
    r = rm.solve(w, 2.8, 0.0, rm.TARGETS["KKV"])
    assert r.pulse["modulation"] == "lfm" and r.losses["compression"] == pytest.approx(lfm)
    assert "DSP" not in rm.BAND_S.losses_db and rm.BAND_S.losses_db["finite_precision"] == 0.3


def test_intra_pulse_velocity_mismatch_law():
    """Quadratic-phase loss of a filter matched to the wrong velocity grows with n_c = 2 dv tau B / c; the velocity
    filters for 1 dB scale with the span and with tau B."""
    loss = [rm.velocity_mismatch_loss_db(n) for n in (0.0, 0.25, 0.5, 1.0, 2.0)]
    assert loss[0] == pytest.approx(0.0, abs=1e-12) and all(b > a for a, b in zip(loss, loss[1:]))
    assert rm.cells_in_pulse(25000.0, 2e-3, 75.0) == pytest.approx(25.0, rel=1e-3)
    n1 = rm.velocity_hypotheses(20000.0, 2e-3, 75.0)
    assert rm.velocity_hypotheses(40000.0, 2e-3, 75.0) in (2 * n1 - 1, 2 * n1, 2 * n1 + 1)
    assert rm.velocity_hypotheses(20000.0, 1e-3, 75.0) <= (n1 + 1) // 2 + 1


def test_blind_range_eclipsing_and_unambiguous_pri():
    """Blind range c (tau + recovery) / 2; an unambiguous train has PRI >= 2 R / c + tau + recovery (no echo out to R
    falls on its own pulses); a staggered ambiguous train pays -10 log(1 - (tau + rec) / PRI)."""
    s = rm.BAND_S
    assert rm.blind_range_km(s, 2e-3) == pytest.approx(rm.C_MPS * (2e-3 + 50e-9) / 2e3)
    assert rm.unambiguous_pri_s(s, 1000.0, 1e-4) == pytest.approx(2e6 / rm.C_MPS + 1e-4 + 50e-9)
    w = rm.Waveform("w", "SEARCH", "S", 2e-4, 8, 1e-3, 75.0, staggered=True)
    r = rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"])
    assert r.losses["eclipsing"] == pytest.approx(-10 * math.log10(1 - (2e-4 + 50e-9) / 1e-3))
    sec = dict(md.new_sector([]), range_km=2000.0)
    cands = mo.candidate_waveforms(sec)
    assert all(rm.blind_range_km(s, w.tau_s) <= mo.range_min_km(sec) for w in cands)
    assert all(w.n == 1 or w.pri_s >= rm.unambiguous_pri_s(s, 2000.0, w.tau_s) - 1e-15 for w in cands)
    assert not any(w.staggered for w in cands)
    amb = mo.candidate_waveforms(dict(sec, allow_ambiguous=True))
    assert any(w.staggered for w in amb) and all(w.staggered == (w.n > 1) for w in amb if w.pri_s == mo.PRI_MIN_S)


def test_sector_range_min_and_resolution_flags():
    sec = dict(md.new_sector([]), range_km=2000.0, range_min_km=40.0)
    default = {k: v for k, v in sec.items() if k != "range_min_km"}
    assert mo.range_min_km(sec) == 40.0 and mo.range_min_km(default) == pytest.approx(mo.RANGE_MIN_FRAC * 2000.0)
    assert max(w.tau_s for w in mo.candidate_waveforms(sec)) <= 2 * 40e3 / rm.C_MPS
    m = md.empty_mission()
    m["sectors"] = [dict(sec, range_min_km=2500.0)]
    assert any("range_min_km" in e for e in md.validate_mission(m))
    res = mo.run({"global": md.empty_mission()["global"], "sectors": [dict(sec, res_max_m=2.0)]}, workers=1)
    assert any("range resolution 2 m not met" in f for f in res["sectors"][0]["flags"])
    res = mo.run({"global": md.empty_mission()["global"], "sectors": [dict(sec, res_max_m=3.0)]}, workers=1)
    assert not any("range resolution" in f for f in res["sectors"][0]["flags"])


# ----------------------------------------------------------------------------- processing (step 2.2)
def test_oscillator_phase_noise_through_the_round_trip(tmp_path):
    """One LO for TX and RX: the round trip filters its noise (nothing at tau_rt = 0); with the photonic-class
    table the loss is negligible; a 40 dB worse LO raises the variance 10^4 times; the Doppler floor is an output."""
    assert rm.oscillator_phase_var(14.0, 0.1, 0.0, 75.0) == pytest.approx(
        (4 * math.pi * rm.PLATFORM_MOTION_RMS_M / rm.lambda_m(14.0)) ** 2, rel=1e-9)
    v = rm.oscillator_phase_var(14.0, 0.1, 8000.0, 75.0) - rm.oscillator_phase_var(14.0, 0.1, 0.0, 75.0)
    assert 0.0 < v < 1e-6
    assert rm.oscillator_phase_var(14.0, 0.1, 8000.0, 75.0) > rm.oscillator_phase_var(2.8, 0.1, 8000.0, 75.0)
    floor = rm.doppler_floor_dbc(2.8, 0.05, 8000.0)
    assert set(floor) == {"100 Hz", "1000 Hz", "10000 Hz"} and all(x < -100.0 for x in floor.values())
    text = _default_toml().replace("value = [-100.0, -120.0, -140.0, -150.0, -160.0, -170.0, -175.0]",
                                   "value = [-60.0, -80.0, -100.0, -110.0, -120.0, -130.0, -135.0]")
    out = _run_with_params(text, "from core import radar_model as rm\n"
                                 "print(rm.oscillator_phase_var(14.0, 0.1, 8000.0, 75.0) - rm.oscillator_phase_var(14.0, 0.1, 0.0, 75.0))",
                           tmp_path)
    assert float(out) == pytest.approx(v * 1e4, rel=1e-6)


def test_velocity_bank_loss_and_hypotheses():
    """Bank step: 1 dB at half a step; the mean over the step is below it; more span -> more hypotheses; a TRACK look
    has one hypothesis; a single pulse only sees the intra-pulse term; a long noncoherent dwell needs a dense bank."""
    n1, mean1 = rm.velocity_bank(5000.0, 2e-4, 75.0, 0.6, 24, False)
    n2, _ = rm.velocity_bank(10000.0, 2e-4, 75.0, 0.6, 24, False)
    assert 0.0 < mean1 < rm.BANK_MAX_LOSS_DB and n2 in (2 * n1 - 1, 2 * n1, 2 * n1 + 1) and n1 > 100
    assert rm.velocity_bank(5000.0, 2e-4, 75.0, 2e-4, 1, True)[0] == 1
    assert rm.walk_loss_db(0.0, True) == 0.0 and rm.walk_loss_db(1.0, False) < rm.walk_loss_db(1.0, True)
    trk_wf = rm.Waveform("t", "TRACK", "Ku", 40e-6, 16, 5e-4, 75.0, integration="COHERENT",
                         tracking_pack=rm.TrackingPack(0.05, 2.0))
    r = rm.solve(trk_wf, 14.0, 0.0, rm.TARGETS["KKV"])
    assert r.pulse["velocity_hypotheses"] == 1 and r.pulse["velocity_span_mps"] == 0.0
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], v_span_mps=4000.0)
    assert r.losses["migration"] == pytest.approx(rm.velocity_bank(4000.0, 2e-4, 75.0, rm.WF_LRS.dwell_s, 24, False)[1]
                                                  + rm.MIGRATION_RESIDUAL_DB)


def test_straddle_and_cfar_follow_the_processing_entries(tmp_path):
    """Straddle shrinks with the grid factors, the CA-CFAR loss with the reference cells; the lumps are gone."""
    assert "straddle" not in rm.BAND_S.losses_db and "CFAR" not in rm.BAND_S.losses_db
    assert rm.cfar_loss_db(64) == pytest.approx(0.64, abs=0.01) and rm.cfar_loss_db(1024) < rm.cfar_loss_db(256)
    assert rm.straddle_range_db(-35.0) < rm.straddle_range_db(-25.0)
    text = _replace_value(_default_toml(), "processing", "range_grid_oversampling", "4.0", "2.0")
    text = _replace_value(text, "processing", "doppler_zero_padding", "4.0", "1.0")
    out = _run_with_params(text, "from core import radar_model as rm\n"
                                 "print(rm.straddle_range_db(-25.0), rm.straddle_doppler_db())", tmp_path).split()
    assert float(out[0]) == pytest.approx(0.215, abs=0.01) and float(out[1]) == pytest.approx(1.12, abs=0.02)


def test_false_alarms_count_independent_cells(runs):
    """Confirmation budget: false alarms = positions x independent cells x Pfa, cells = range from range_min x
    Doppler cells of a block x velocity hypotheses x angle cells."""
    assert rm.independent_cells(100.0, 1100.0, 75.0, 4, 3, 0.5) == pytest.approx(1e6 / 2.0 * 4 * 3 * 0.5, rel=1e-3)
    res = runs["mission_example.json"]
    s = next(x for x in res["sectors"] if x["id"] == "S02")
    w = s["wf"]
    assert {"coherent_blocks", "pulses_per_block", "velocity_hypotheses", "block_ms"} <= set(w)
    trk_s = next(x for x in res["tracking"]["sectors"] if x["id"] == "S02")
    sec = next(x for x in md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))["sectors"] if x["id"] == "S02")
    angle = rm.rx_beams_per_position(w["freq_ghz"], mo.rx_sll_db(sec)) * rm.RX_BEAM_SPACING_HPBW ** 2
    cells = rm.independent_cells(mo.range_min_km(sec), sec["range_km"], w["sig_bw_mhz"], w["pulses_per_block"],
                                 w["velocity_hypotheses"], angle)
    assert trk_s["false_alarms_per_rr"] == pytest.approx(round(s["positions_total"] * cells * rm.DETECT_PFA, 1), abs=0.11)


# ----------------------------------------------------------------------------- the receive chain (step 1.5)
def test_adc_noise_figure_from_the_converter(tmp_path):
    """NF = P_FS - (6.02 ENOB + 1.76) - 10 log10(k T0 fs): one more bit = -6.02 dB, +6 dB of full scale = +6 dB; the
    clipping level follows the full scale and the gain; an analog chain compressing before the ADC is refused."""
    s = rm.BAND_S
    assert s.adc_nf_db == pytest.approx(10.0 - 80.02 - 10 * math.log10(rm.K_BOLTZ * 290 * 250e6 * 1e3), abs=1e-9)
    assert s.adc_fs_hz == pytest.approx(250e6) and s.clip_ant_dbm == pytest.approx(10.0 - 3.0 - s.gain_to_adc_db)
    assert s.gain_to_adc_db == pytest.approx(-0.85 + 27.5 + 20.0)
    text = _replace_value(_default_toml(), "band.S.rx", "adc_enob ", "13.0", "14.0")
    out = _run_with_params(text, "from core import radar_model as rm\nprint(rm.BAND_S.adc_nf_db)", tmp_path)
    assert float(out) == pytest.approx(s.adc_nf_db - 6.02, abs=1e-9)
    bad = _replace_value(_default_toml(), "band.Ku.rx", "post_lna_op1db_dbm ", "15.0", "12.0")
    path = tmp_path / "bad.toml"
    path.write_text(bad, encoding="utf-8")
    res = subprocess.run([PYTHON, "-c", "import core.radar_model"], cwd=tb.ROOT, capture_output=True, text=True,
                         env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=120)
    assert res.returncode != 0 and "compress before" in res.stderr


def test_saturation_range_scales_as_rcs_quarter_power():
    """The echo per element grows as sigma / R^4: the shortest unsaturated range scales as sigma^(1/4), and at that
    range the echo sits exactly on the clipping level."""
    for f in (2.8, 14.0):
        r1, r16 = rm.min_unsaturated_range_km(f, 0.0, 1.0), rm.min_unsaturated_range_km(f, 0.0, 16.0)
        assert r16 / r1 == pytest.approx(2.0, rel=1e-12)
        assert rm.echo_peak_dbm_per_element(f, 0.0, 1.0, r1) == pytest.approx(rm.band_for_frequency(f).clip_ant_dbm)
    assert rm.rx_deaf_after_pulse_s(rm.BAND_S, 1e-3) == pytest.approx(1e-3 + 50e-9)


def test_polarisation_glrt_replaces_the_placeholder():
    """Dual-pol detection looks: one more unknown (the polarisation) = SNR_req(n + 1) - SNR_req(n), n = 1 for a
    coherent dwell; smaller for long noncoherent trains; TRACK looks: the estimation residual; solve() carries it."""
    s = rm.BAND_S
    g1 = rm.pol_recombination_loss_db(s, "SEARCH", 1, 1)
    assert g1 == pytest.approx(rm.shnidman_total_snr_db(rm.DETECT_PD, rm.DETECT_PFA, 2, 1)
                               - rm.shnidman_total_snr_db(rm.DETECT_PD, rm.DETECT_PFA, 1, 1))
    assert 0.4 < g1 < 0.6 and rm.pol_recombination_loss_db(s, "SEARCH", 24, 1) < 0.1
    assert rm.pol_recombination_loss_db(rm.BAND_KU, "TRACK", 1, 1) == 0.05
    assert rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"]).losses["polarisation"] == pytest.approx(
        rm.pol_recombination_loss_db(s, "SEARCH", 24, 1))
    assert "pol_loss_dual_db" not in {p.path.split(".")[-1] for p in params.P}


def test_solve_scales_with_the_system_temperature():
    r0 = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"])
    r1 = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], t_sys_k=16 * rm.BAND_S.t_sys_k)
    assert r0.t_sys_k == rm.BAND_S.t_sys_k and r1.range_km / r0.range_km == pytest.approx(0.5, rel=1e-12)


def _boresight_body(typ: str, radius_deg: float, **kw) -> "sky.Body":
    from core import sky
    return sky.parse({"bodies": [{"id": typ, "type": typ, "phi_deg": 0.0, "eps_deg": rm.SECTOR_TILT_DEG,
                                  "angular_radius_deg": radius_deg, **kw}]}).bodies[0]


def test_sky_body_filling_the_front_half_space_adds_its_temperature():
    """A body covering the whole front half-space adds T_b (Parseval normalisation of the mean pattern, 2 %)."""
    from core import sky
    d = sky.unit(np.array([0.0]), np.array([rm.SECTOR_TILT_DEG]))
    for b, f in (("S", 2.8), ("Ku", 14.0)):
        for sll in rm.RX_SLL_OPTIONS_DB:
            r = sky.body_rise_k(_boresight_body("earth", 90.0, tb_k=100.0), b, f, sll, mo.face_frame(0.0), d)
            assert r[0] == pytest.approx(100.0, rel=0.02)


def test_sun_in_the_beam_matches_the_hand_calculation():
    """S: the Sun (0.267 deg) is small against the beam: T_b Omega_sun G0 / 4 pi x the mean of a Gaussian main lobe
    over the disk (within 1 %); Ku: the Sun is wider than the -3 dB beam: most of T_b. Both blind the position (rise >
    sky.blind_rise_db). Behind the face: nothing."""
    from core import sky
    d = sky.unit(np.array([0.0]), np.array([rm.SECTOR_TILT_DEG]))
    sun = _boresight_body("sun", 0.267)
    rs = sky.body_rise_k(sun, "S", 2.8, -25.0, mo.face_frame(0.0), d)[0]
    a_r2 = 4 * math.log(2) * (math.radians(0.267) * rm.ARRAY_ENVELOPE_M / rm.lambda_m(2.8) / rm.rx_beam("S").hpbw) ** 2
    hand = (sun.tb_k[0] * math.pi * math.radians(0.267) ** 2 * rm.gain_lin(rm.rx_beam("S").eta, 2.8, 0.0) / (4 * math.pi)
            * (1 - math.exp(-a_r2)) / a_r2)
    assert rs == pytest.approx(hand, rel=0.01)
    rk = sky.body_rise_k(sun, "Ku", 14.0, -25.0, mo.face_frame(0.0), d)[0]
    assert 0.75 * sun.tb_k[1] < rk < sun.tb_k[1]          # disk wider than the -3 dB beam: most of the main lobe
    for b, r in (("S", rs), ("Ku", rk)):
        band = rm.BANDS[b]
        assert rm.lin_to_db(rm.t_sys_with_rise(band, r) / band.t_sys_k) > sky.BLIND_RISE_DB
    assert sky.body_rise_k(sun, "S", 2.8, -25.0, mo.face_frame(180.0), d)[0] == 0.0


def test_galactic_rise_profile_and_worst_case():
    from core import sky
    d = sky.unit(np.array([180.0, 0.0]), np.array([30.0, 60.0]))      # galactic centre, galactic pole
    s = sky.parse({"galactic": {"pole_phi_deg": 0.0, "pole_eps_deg": 60.0, "centre_phi_deg": 180.0, "centre_eps_deg": 30.0}})
    r = sky.galactic_rise_k(s, d, sky.GAL_REF_GHZ)
    assert r[0] == pytest.approx(sky.GAL_T_CENTRE_K - sky.GAL_T_HIGH_K) and r[1] == pytest.approx(0.0, abs=1e-9)
    w = sky.galactic_rise_k(sky.parse(None), d, 14.0)
    assert np.allclose(w, (sky.GAL_T_CENTRE_K - sky.GAL_T_HIGH_K) * (14.0 / sky.GAL_REF_GHZ) ** -sky.GAL_INDEX)
    assert sky.parse(None).galactic.startswith("worst case")


def test_sky_sector_budget_blinds_and_takes_the_quantile(runs):
    """Sky test mission: the Sun blinds part of its sector (those positions are not scanned), the budget T_sys is the
    (1 - short_tol) position quantile of the others, ranges follow it; a sector without bodies only gets the galaxy."""
    res = runs["mission_sky_demo.json"]
    by = {s["id"]: s for s in res["sectors"]}
    sun = by["SUN"]["sky"]
    assert sun["blinded_pct"] > 0 and sun["blinded_by"] == ["SUN"]
    assert by["SUN"]["sky"]["t_sys_budget_k"] > rm.BAND_S.t_sys_k
    earth = by["EARTH"]["sky"]
    assert earth["blinded_pct"] == 0 and 1.5 < earth["rise_db"] < 4.5
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_sky_demo.json"))
    sec = next(s for s in m["sectors"] if s["id"] == "SUN")
    sph = mo._sphere(-90.0, 90.0)
    pos_sky, _, _, _ = mo.search_positions(sec, sph, sph.mask(sec), 2.8, m["global"]["sky"])
    pos_cold, _ = mo.positions_table(sph, sph.mask(sec), 2.8)
    blinded = 1.0 - sum(v.sum() for v in pos_sky.values()) / sum(v.sum() for v in pos_cold.values())
    assert blinded == pytest.approx(sun["blinded_pct"] / 100.0, abs=2e-3)
    trk_sun = next(s for s in res["tracking"]["sectors"] if s["id"] == "SUN")
    assert trk_sun["ku_sky"]["band"] == "Ku" and trk_sun["ku_sky"]["blinded_by"] == ["SUN"]


def test_invalid_sky_is_refused():
    bad = {"bodies": [{"id": "X", "type": "comet", "phi_deg": 0.0, "eps_deg": 0.0, "angular_radius_deg": 1.0},
                      {"id": "Y", "type": "sun", "phi_deg": 0.0, "eps_deg": 95.0, "angular_radius_deg": 0.3}],
           "galactic": {"pole_phi_deg": 0.0, "pole_eps_deg": 90.0, "centre_phi_deg": 0.0, "centre_eps_deg": 45.0}}
    errs = md.validate_global({**md.GLOBAL_DEFAULTS, "sky": bad})
    assert any("give tb_k" in e for e in errs) and any("outside [-90, 90]" in e for e in errs)
    assert any("90 expected" in e for e in errs)
    with pytest.raises(ValueError):
        mo.run({"global": {**md.GLOBAL_DEFAULTS, "sky": bad}, "sectors": [md.new_sector([])]}, workers=1)


def test_isolation_matrix_and_scheduler_rules(tmp_path):
    """Same band, other faces: forbidden (sync rule); S harmonic in Ku on the same face: frequency rule; Ku -> S:
    allowed; the harmonic order follows the bands; 'cancel' is not implemented."""
    from core import isolation as iso
    v = {(p["tx"], p["rx"], p["relation"]): p["verdict"] for p in iso.matrix()}
    assert v[("S", "S", "same")] == v[("Ku", "Ku", "same")] == "eclipsing"
    assert all(v[(b, b, r)] == "forbidden" for b in ("S", "Ku") for r in ("adjacent", "opposite"))
    assert v[("S", "Ku", "same")] == "frequency_rule" and v[("S", "Ku", "adjacent")] == "allowed"
    assert all(v[("Ku", "S", r)] == "allowed" for r in iso.RELATIONS)
    assert iso.HARMONIC_ORDERS == (5,)
    assert iso.harmonic_conflict(2.8, 75.0, 14.0, 50.0) and not iso.harmonic_conflict(2.75, 10.0, 14.2, 50.0)
    assert any("common TX / RX cadence" in r for r in iso.scheduler_rules())
    text = _replace_value(_default_toml(), "isolation", "interface_mode ", '"sync"', '"cancel"')
    path = tmp_path / "bad.toml"
    path.write_text(text, encoding="utf-8")
    res = subprocess.run([PYTHON, "-c", "import core.isolation"], cwd=tb.ROOT, capture_output=True, text=True,
                         env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=120)
    assert res.returncode != 0 and "not implemented" in res.stderr


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



def test_coherent_blocks_replace_the_coherence_floor():
    """Step 2.2: no cap any more; a long coherent train is cut into the blocks minimising SNR_req(M) + block
    coherence loss; noncoherent and fast-fluctuating targets integrate pulse by pulse; the loss keeps growing."""
    assert not hasattr(rm, "COH_T_MAX_S")
    assert rm.coherence_loss_db(2.8, 0.15) < rm.coherence_loss_db(2.8, 0.3) < rm.coherence_loss_db(2.8, 1.0)
    m_short, _ = rm.coherent_blocks(4, 2e-3, 1e-4, "COHERENT", 1)
    m_long, t_b = rm.coherent_blocks(32, 60e-3, 1e-4, "COHERENT", 1)
    assert m_short == 1 and m_long > 1 and 32 % m_long == 0 and t_b == pytest.approx((32 // m_long - 1) * 60e-3 + 1e-4)
    assert rm.coherent_blocks(8, 1e-3, 1e-4, "NONCOHERENT", 1) == (8, 1e-4)
    assert rm.coherent_blocks(8, 1e-3, 1e-4, "COHERENT", 2)[0] == 8
    w = rm.Waveform("w", "SEARCH", "S", 1e-3, 32, 60e-3, 75.0, integration="COHERENT")
    r = rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"])
    assert r.pulse["coherent_blocks"] == m_long or r.pulse["coherent_blocks"] > 1
    assert r.snr_req_db == pytest.approx(rm.shnidman_total_snr_db(rm.DETECT_PD, rm.DETECT_PFA, r.pulse["coherent_blocks"], 1))


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
