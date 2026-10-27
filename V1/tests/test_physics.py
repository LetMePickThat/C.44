"""Family 2 -- physics and consistency: laws and invariants that must hold whatever the parameter values.

Uses the test missions of tests/missions/ (test values only, not radar performance).
"""
import contextlib
import dataclasses
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
    assert a.range_km == b.range_km and b.p_avg_max_w - a.p_avg_max_w == pytest.approx(3.5e6 * rm.CHAIN_EFF)   # 4.1


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
    text = _replace_value(_default_toml(), "band.S", "usable_lo_ghz ", "2.6", "2.5")
    text = _replace_value(text, "band.S", "usable_hi_ghz ", "2.93", "3.0")
    code = ("from core import radar_model as rm, optimizer as mo, mission as md\n"
            "print(rm.BAND_S.fits(2.6, 75.0), rm.BAND_S.edge_loss_db(2.5))\n"
            "m = md.empty_mission(); m['sectors'].append(md.new_sector([]))\n"
            "r = mo.run(m, workers=1); print(r['sectors'][0]['status'])")
    assert _run_with_params(text, code, tmp_path).split() == ["True", str(rm.BAND_S.edge_derating_db), "OK"]
    bad = _replace_value(_default_toml(), "band.S", "core_lo_ghz ", "2.66", "2.55")
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
        assert b.range_km / a.range_km == pytest.approx(
            (rm.rx_beam(band, -35.0).eta / rm.rx_beam(band, -25.0).eta * 10 ** (-extra_db / 10)) ** 0.25, rel=1e-12)
    assert rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"]).range_km == \
        rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], rx_sll_db=rm.RX_SLL_DEFAULT_DB).range_km


def test_search_has_the_rx_grid_loss_and_counts_rx_beams():
    """Step 2.3: the search lattice cell and the RX beam lattice enter the averaged Pd ('positions'); their mean
    losses are given (real patterns, close to the Gaussian 1.672 s^2 dB); RX beams per position at 1.4 HPBW."""
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"])
    parts = r.pulse["position_losses_mean_db"]
    assert parts["grid_rx"] == pytest.approx(1.672 * 0.25, abs=0.1) and parts["grid_tx"] == pytest.approx(1.672 * 1.96, abs=0.4)
    assert r.losses["positions"] > sum(parts.values()) - 1e-9          # averaging Pd costs more than averaging dB here
    n_rx = r.frame["rx_beams_per_position"]
    assert n_rx == pytest.approx((1.4 * rm.PATTERNS["S"].tx.hpbw / (0.5 * rm.rx_beam("S").hpbw)) ** 2)
    assert 5.5 < n_rx < 7.0 and rm.rx_beams_per_position(2.8, -35.0) < n_rx


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
    res = mo.run({"global": md.empty_mission()["global"], "sectors": [dict(sec, res_max_m=0.3)]}, workers=1)
    assert any("range resolution 0.3 m not met" in f for f in res["sectors"][0]["flags"])   # > 600 MHz needed
    res = mo.run({"global": md.empty_mission()["global"], "sectors": [dict(sec, res_max_m=2.0)]}, workers=1)
    assert not any("range resolution" in f for f in res["sectors"][0]["flags"])            # 100 MHz since step 3.1


# ----------------------------------------------------------------------------- detection criterion (step 2.3)
def test_exact_laws_match_the_closed_forms_and_shnidman():
    """SW1 on one sample = 10 log(ln Pfa / ln Pd - 1) at any Pfa; SW1 = SW2 and SW3 = SW4 on one sample; Shnidman
    within 0.5 dB where it is valid; Pd grows with SNR and N."""
    from core import detection as d
    for pfa in (1e-6, 1e-8, 1e-12, 1e-14):
        assert d.snr_req_db(1, 1, pfa, 0.9) == pytest.approx(10 * math.log10(math.log(pfa) / math.log(0.9) - 1), abs=0.01)
    assert d.snr_req_db(1, 2, 1e-8, 0.9) == pytest.approx(d.snr_req_db(1, 1, 1e-8, 0.9), abs=0.01)
    assert d.snr_req_db(1, 4, 1e-8, 0.9) == pytest.approx(d.snr_req_db(1, 3, 1e-8, 0.9), abs=0.01)
    for sw in range(5):
        for n in (1, 8, 24):
            assert d.snr_req_db(n, sw, 1e-8, 0.9) == pytest.approx(rm.shnidman_total_snr_db(0.9, 1e-8, n, sw), abs=0.5)
    c = d.pd_curve(4, 1, 1e-10)
    assert np.all(np.diff(c) >= -1e-12) and c[0] < 1e-8 and c[-1] > 0.999


def test_cumulative_criterion_and_averaged_pd():
    """Denser looks (smaller q = v RR / R) need less SNR at R; the averaged Pd costs more than a perfectly centred
    target; solve() with cum meets the cumulative target exactly at the requirement range."""
    from core import detection as d
    reqs = [d.required_snr_db(1, 1, 1e-8, 0.9, q=q) for q in (0.5, 0.2, 0.05, 0.01)]
    assert all(b < a for a, b in zip(reqs, reqs[1:]))
    law = rm.position_losses("S", -25.0, -25.0, True, False, (0.0, 1e-4, 75.0, 1e-4, 1, True))
    assert d.required_snr_db(1, 1, 1e-8, 0.9, law[0], law[1]) > d.required_snr_db(1, 1, 1e-8, 0.9)
    w = rm.Waveform("w", "SEARCH", "S", 1e-4, 1, 1e-3, 75.0)
    r = rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"], cum=(5000.0, 60.0, 3000.0))
    assert r.pulse["criterion"] == "cumulative" and r.pulse["look_spacing_q"] == pytest.approx(0.1)
    assert r.snr_req_db < rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"]).snr_req_db
    assert 0.0 < r.pulse["pd_per_look_at_range"] < rm.DETECT_PD


def test_false_alarm_policy(tmp_path):
    """'fa_rate': Pfa per cell = rate / (positions x cells), bounded, on 1/20 decade; 'pfa_cell': detection.pfa;
    a fixed search lattice (no dithering) is refused."""
    assert rm.search_pfa(10.0, 1000.0, 1e7) == pytest.approx(1e-9, rel=0.13)
    assert rm.search_pfa(10.0, 1.0, 1.0) == pytest.approx(rm.PFA_MAX, rel=0.13)
    assert rm.search_pfa(1e-30, 1e9, 1e9) == pytest.approx(rm.PFA_MIN, rel=0.13)
    text = _replace_value(_default_toml(), "detection", "fa_mode ", '"fa_rate"', '"pfa_cell"')
    out = _run_with_params(text, "from core import radar_model as rm\nprint(rm.search_pfa(10.0, 1000.0, 1e7))", tmp_path)
    assert float(out) == rm.DETECT_PFA
    bad = _replace_value(_default_toml(), "search_grid", "grid_dithering ", "true", "false")
    path = tmp_path / "bad.toml"
    path.write_text(bad, encoding="utf-8")
    res = subprocess.run([PYTHON, "-c", "import core.radar_model"], cwd=tb.ROOT, capture_output=True, text=True,
                         env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=120)
    assert res.returncode != 0 and "not implemented" in res.stderr


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
    n1, mean1, _ = rm.velocity_bank(5000.0, 2e-4, 75.0, 0.6, 24, False)
    n2, _, _ = rm.velocity_bank(10000.0, 2e-4, 75.0, 0.6, 24, False)
    assert 0.0 < mean1 < rm.BANK_MAX_LOSS_DB and n2 in (2 * n1 - 1, 2 * n1, 2 * n1 + 1) and n1 > 100
    assert rm.velocity_bank(5000.0, 2e-4, 75.0, 2e-4, 1, True)[0] == 1
    assert rm.walk_loss_db(0.0, True) == 0.0 and rm.walk_loss_db(1.0, False) < rm.walk_loss_db(1.0, True)
    trk_wf = rm.Waveform("t", "TRACK", "Ku", 40e-6, 16, 5e-4, 75.0, integration="COHERENT",
                         tracking_pack=rm.TrackingPack(0.05, 2.0))
    r = rm.solve(trk_wf, 14.0, 0.0, rm.TARGETS["KKV"])
    assert r.pulse["velocity_hypotheses"] == 1 and r.pulse["velocity_span_mps"] == 0.0
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"], v_span_mps=4000.0)
    assert r.losses["migration"] == rm.MIGRATION_RESIDUAL_DB      # the filter offset is in 'positions' (step 2.3)
    assert r.pulse["position_losses_mean_db"]["velocity_filter"] == pytest.approx(
        rm.velocity_bank(4000.0, 2e-4, 75.0, rm.WF_LRS.dwell_s, 24, False)[1], abs=0.02)


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
    angle = (rm.rx_beams_per_position(w["freq_ghz"], mo.rx_sll_db(sec), s["beam_spacing_hpbw"])   # the sector's
             * rm.RX_BEAM_SPACING_HPBW ** 2)                                                         # spacing (P5)
    cells = rm.independent_cells(mo.range_min_km(sec), sec["range_km"], w["sig_bw_mhz"], w["pulses_per_block"],
                                 w["velocity_hypotheses"], angle)
    assert trk_s["false_alarms_per_rr"] == pytest.approx(round(s["positions_total"] * cells * w["pfa"], 1), abs=0.11)
    if rm.FA_MODE == "fa_rate":         # step 2.3: the tolerated rate (Pfa rounded to 1/20 decade)
        assert trk_s["false_alarms_per_rr"] == pytest.approx(mo.fa_per_rr(sec), rel=0.13)


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
    from core import detection
    g1 = rm.pol_recombination_loss_db(s, "SEARCH", 1, 1)
    assert g1 == pytest.approx(detection.snr_req_db(2, 1, rm.DETECT_PFA, rm.DETECT_PD)
                               - detection.snr_req_db(1, 1, rm.DETECT_PFA, rm.DETECT_PD))
    assert 0.4 < g1 < 0.6 and rm.pol_recombination_loss_db(s, "SEARCH", 24, 1) < 0.1
    assert rm.pol_recombination_loss_db(rm.BAND_KU, "TRACK", 1, 1) == 0.05
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"])          # ST: its Gamma shape since step 2.4
    assert r.losses["polarisation"] == pytest.approx(
        rm.pol_recombination_loss_db(s, "SEARCH", 24, detection.Fluct(r.pulse["target"]["gamma_shape"])))
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
    earth = by["EARTH"]["sky"]           # step 2.5: the Earth ring at the sector range adds sidelobe clutter
    assert earth["clutter_limited_pct"] > 0 and "clutter" in earth["blinded_by"] and earth["rise_db"] > 1.5
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


def _one_frequency(m: dict, f_ghz: float = 2.8) -> dict:
    """The mission with one forced S frequency per sector (test protocol, step 4: a test that only needs a run, not
    the frequency search, skips it; a sector that already forces its frequency keeps it)."""
    for sec in m["sectors"]:
        sec.setdefault("freq_ghz_candidates", [f_ghz])
    return m


def test_eps_window_is_not_written_into_the_model():
    """Mission inputs are passed explicitly; running a mission leaves the model's defaults untouched (one frequency
    per sector: the run, not the search, is tested)."""
    before = (rm.SECTOR_EPS_MIN_DEG, rm.SECTOR_EPS_MAX_DEG)
    mo.run(_one_frequency(md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_synthetic_mix.json"))), workers=1)
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


def test_tilt_change_propagates_to_the_model(tmp_path):
    """The model follows geometry.tilt_deg; the whole tool also needs the 3D model's panels at that tilt, else it
    refuses the file (test_ship_model_matches_the_locked_dimensions_and_its_placement_is_checked)."""
    text = _replace_value(_default_toml(), "geometry", "tilt_deg ", "25.0", "20.0")
    out = _run_with_params(text, "import json\nfrom core import radar_model as rm\n"
                                 "print(json.dumps([rm.SECTOR_TILT_DEG, rm.n_search_positions(2.8)]))", tmp_path)
    tilt, pos = json.loads(out)
    assert tilt == 20.0
    # 4308 at 20 deg in the model history (V6.0.10, HPBW 0.886 lambda / D, D = 4.19 m), x (0.886 / HPBW_TX)^2 since
    # step 1.1, x (D_env / 4.19)^2 since the 0.5 mm gaps (step 1.2)
    # and x (1.04 / 1.4)^2 since the search lattice of step 2.3
    assert abs(pos - 4308 * (0.886 / rm.PATTERNS["S"].tx.hpbw) ** 2 * (rm.ARRAY_ENVELOPE_M / 4.19) ** 2
               * (1.04 / rm.BEAM_SPACING_HPBW) ** 2) < 20


# ----------------------------------------------------------------------------- optimiser invariants
@pytest.fixture(scope="module")
def runs():
    """optimizer.run() of every test mission, read from the frozen snapshots (step 3.1, X5): tests/test_snapshots.py
    already recomputes each mission and fails on any difference, so the invariants below are checked on outputs equal
    to the current ones (to 1e-9) without computing them twice."""
    return {n: tb.load_snapshot(c)["run"] for n, c in zip(tb.MISSIONS, tb.case_names()[1:])}


def test_time_on_air_at_least_energy_over_peak_power(runs):
    """f_t >= f_e x P_avg / P_pk: a face cannot radiate its energy faster than at full peak power."""
    for n, res in runs.items():
        p_avg = res["global"]["face_elec_avail_kW"] * 1e3
        for k, f in res["faces_total"].items():
            # electrical budget (step 1.3): RF energy >= electrical x the lowest PAE of the S search space, i.e. of the
            # shortest pulse a "no excess energy" shortening can reach (tau B = pulse_tb_min, step 3.1 C); the face
            # totals are published to 4 decimals (5e-5 each way)
            pae_min = rm.pae(rm.BAND_S, rm.PULSE_TB_MIN / (max(mo.SIGBW_CAND) * 1e6), rm.BAND_S.duty_max)
            assert f["time_fraction"] + 5e-5 >= ((f["energy_fraction"] - 5e-5) * p_avg * pae_min
                                                 / rm.BAND_S.p_pk_total_w * (1 - 1e-9)), (n, k)


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
    assert rm.face_elec_avail_w(2.0) == pytest.approx(2e6 * rm.CHAIN_EFF - rm.non_tx_power_w())   # step 4.1
    r = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.TARGETS["ST"])
    assert r.p_avg_used_w == pytest.approx(rm.BAND_S.p_pk_total_w * rm.WF_LRS.duty / r.pae)
    assert r.p_diss_trm_w == pytest.approx(2000.0 * rm.WF_LRS.duty * (1 / r.pae - 1))
    res = runs["mission_example.json"]
    for k, fc in res["tracking"]["faces"].items():
        if res["faces_total"][k]["positions"] > 0:
            elec_kw = fc["bus_draw_MW"] * 1e3                     # step 5.2 (P6): energy load = share of the ship
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
    from core import detection
    fl = detection.Fluct(r.pulse["target"]["gamma_shape"])        # ST: SW1/2 with its T0 and diversity (step 2.4)
    assert r.snr_req_db == pytest.approx(detection.snr_req_db(r.pulse["coherent_blocks"], fl, rm.DETECT_PFA, rm.DETECT_PD))


def test_estimator_constants_share_one_margin_over_the_bounds():
    """Angle, k_f, k_r = Cramer-Rao bound x the same 1.13 margin (review 0.3, step 1.1)."""
    margin = rm.ANGLE_CRLB_MARGIN
    assert rm.K_F == pytest.approx(0.551 * margin, abs=0.03) and rm.RANGE_CRLB_KR == pytest.approx(0.551 * margin, abs=0.03)
    assert rm.K_F > 0.551 and rm.RANGE_CRLB_KR > 0.551           # never below the bound


def test_pulse_scale_never_above_full_pulse(runs):
    """pulse_scale is reported rounded to 3 decimals, so a strong shortening ((20/228)^4) shows as 0.0."""
    for res in runs.values():
        for s in res["sectors"]:
            for b in s.get("wf_by_bin", []):
                assert 0.0 <= b["pulse_scale"] <= 1.0


def test_no_frequency_above_scan_break_when_it_leaves_sky_unscanned(runs):
    """Optimiser rule (SPEC v2 defect 3): a frequency that leaves more of the sector unscanned never wins."""
    for s in runs["mission_veille500.json"]["sectors"]:
        assert s["wf"]["freq_ghz"] <= rm.BAND_S.scan_break_ghz, s["id"]


def test_tracking_loads_add_up(runs):
    for res in runs.values():
        for k, f in res["tracking"]["faces"].items():
            # step 5.2 (P6): the energy load is the face's need over the share of the ship's power it is allocated;
            # step 3.4: the calibration time adds to the S and Ku times
            a = res["power_allocation"]["faces"][k]
            assert f["total_energy"] == pytest.approx(a["need_MW"] / a["alloc_MW"], rel=1e-12)
            assert f["alloc_MW"] == pytest.approx(a["alloc_MW"], rel=1e-12)
            assert f["total_ku_time"] == pytest.approx(f["confirm_ku_time"] + f["at_ku_time"] + f["calibration_ku_time"],
                                                       abs=1e-12)
            assert f["total_s_time"] == pytest.approx(f["search_s_time"] + f["confirm_s_time"] + f["calibration_s_time"],
                                                      abs=1e-12)
            assert f["load"] == pytest.approx(max(f["total_energy"], f["total_s_time"], f["total_ku_time"],
                                                  f.get("heat_load", 0.0)), abs=1e-12)        # step 4.2: heat


def test_parallel_run_equals_sequential():
    """Ten sectors (enough for the parallel run), one frequency each: the parallel and sequential runs agree."""
    m = _one_frequency(md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_redundancy_demo.json")))
    assert json.dumps(mo.run(m, workers=1), sort_keys=True) == json.dumps(mo.run(m, workers=2), sort_keys=True)


# ----------------------------------------------------------------------------- target models (step 2.4)
def _target_toml(name: str, extra: str = "") -> str:
    """A target class appended to the default parameter file (the library is extended in the file only)."""
    band = ('rcs_m2 = { value = 2.0, unit = "m2", status = "D", source = "t", date = "" }\n'
            'fluctuation = { value = "SW1/2", unit = "", status = "D", source = "t", date = "" }\n'
            'depol_db = { value = -10.0, unit = "dB", status = "D", source = "t", date = "" }\n')
    return (_default_toml() + f"\n[targets.{name}]\n"
            'v_close_mps = { value = 7000.0, unit = "m/s", status = "D", source = "t", date = "" }\n'
            'length_m = { value = 2.0, unit = "m", status = "D", source = "t", date = "" }\n'
            'rotation_dps = { value = 2.0, unit = "deg/s", status = "D", source = "t", date = "" }\n'
            f'{extra}accel_max_mps2 = {{ value = 0.0, unit = "m/s2", status = "D", source = "t", date = "" }}\n'
            f"[targets.{name}.band.S]\n{band}[targets.{name}.band.Ku]\n{band}")


def test_target_library_classes_and_derived_values():
    """Every class of the file is loaded; T0 = lambda / (2 L omega); the KKV seeker optics add max(pi (2f)^2,
    eta_r 4 pi A^2 / lambda^2) to the frontal RCS (about 1.3 m2 in S, 14 m2 in Ku with the class means)."""
    assert {"ST", "GENERIC", "KKV", "SHIP", "DEBRIS", "DECOY"} <= set(rm.TARGETS) and rm.DEFAULT_TARGET_TYPE == "GENERIC"
    st = rm.TARGETS["ST"]
    assert st.t0_s(2.8) == pytest.approx(rm.lambda_m(2.8) / (2 * 10.0 * math.radians(0.1)), rel=1e-12)
    assert st.t0_s(14.0) == pytest.approx(st.t0_s(2.8) * 2.8 / 14.0, rel=1e-12)
    kkv = rm.TARGETS["KKV"]
    a = math.pi * 0.3 ** 2 / 4
    assert kkv.optics_rcs_m2(2.8) == pytest.approx(max(math.pi * 0.6 ** 2, 0.1 * 4 * math.pi * a * a / rm.lambda_m(2.8) ** 2))
    assert kkv.rcs_at(2.8) == pytest.approx(1.27, abs=0.01) and kkv.rcs_at(14.0) == pytest.approx(13.8, abs=0.1)
    assert rm.TARGETS["DECOY"].optics_rcs_m2(14.0) == 0.0 and rm.TARGETS["SHIP"].rcs_at(2.8) == 300.0
    assert rm.Target("x", 1.0, 1, 5000.0).target_type == rm.DEFAULT_TARGET_TYPE      # former form: default class


def test_target_class_added_in_the_file_only(tmp_path):
    """A new class needs no code change; an incomplete class is refused at load time with a clear message."""
    code = ("from core import radar_model as rm\nt = rm.target_for_sector({'target_type': 'TESTCLASS', 'v_close_max_mps': 3000.0})\n"
            "print(t.rcs_at(2.8), t.v_close_mps, round(t.t0_s(2.8), 4))")
    out = _run_with_params(_target_toml("TESTCLASS"), code, tmp_path).split()
    assert out == ["2.0", "3000.0", str(round(rm.lambda_m(2.8) / (2 * 2.0 * math.radians(2.0)), 4))]
    bad = _target_toml("BAD")
    bad = bad[:bad.rindex("depol_db")]                                        # last band entry missing
    path = tmp_path / "bad.toml"
    path.write_text(bad, encoding="utf-8")
    res = subprocess.run([PYTHON, "-c", "import core.radar_model"], cwd=tb.ROOT, capture_output=True, text=True,
                         env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=120)
    assert res.returncode != 0 and "target class 'BAD' misses" in res.stderr


def test_sector_target_resolution():
    """No class: the sector's RCS and Swerling (default 1) with the default class for the rest (former behaviour);
    a class: its values, each overridable by the sector."""
    t = rm.target_for_sector({"id": "A", "rcs_m2": 50.0, "v_close_max_mps": 4000.0})
    assert (t.rcs_m2, t.rcs_ku_m2, t.swerling, t.v_close_mps, t.target_type) == (50.0, None, 1, 4000.0, "GENERIC")
    k = rm.target_for_sector({"id": "B", "target_type": "KKV", "v_close_max_mps": 20000.0})
    assert k.swerling is None and k.rcs_at(14.0) == rm.TARGETS["KKV"].rcs_at(14.0) and k.v_close_mps == 20000.0
    o = rm.target_for_sector({"id": "C", "target_type": "KKV", "v_close_max_mps": 2e4, "rcs_m2": 0.5, "swerling": 2,
                              "optics_retro_eff": 1.0, "depol_db": -6.0})
    a = math.pi * 0.3 ** 2 / 4
    assert o.rcs_at(2.8) == pytest.approx(0.5 + max(math.pi * 0.6 ** 2, 4 * math.pi * a * a / rm.lambda_m(2.8) ** 2))
    assert o.swerling == 2 and o.optics_retro_eff == 1.0 and o.depol_ku_db == -6.0 and o.rcs_ku_m2 is None
    with pytest.raises(ValueError, match="unknown target_type"):
        rm.target_for_sector({"target_type": "NOPE", "v_close_max_mps": 1.0})


def test_fluctuation_shape_of_a_look():
    """Declared case: k = 1, 2, n, 2n (inf for SW0); class: family x min(n, 1 + T_obs / T0); both x the
    polarisation diversity (1 + rho)^2 / (1 + rho^2); the Gamma-shape laws reproduce the Swerling cases exactly."""
    from core import detection
    for sw in range(5):
        assert detection.snr_req_db(6, detection.Fluct(detection.shape_of(sw, 6)), 1e-8, 0.9) == \
            detection.snr_req_db(6, sw, 1e-8, 0.9)
    g = rm.TARGETS["GENERIC"]
    div = (1.1 ** 2) / 1.01
    assert rm.diversity_factor(g, 2.8) == pytest.approx(div)
    assert rm.look_fluctuation(g, 2.8, 2, 8, 0.1).k == pytest.approx(8 * div, rel=0.03)
    t0 = g.t0_s(2.8)
    assert rm.look_fluctuation(g, 2.8, None, 8, t0).k == pytest.approx(2.0 * div, rel=0.03)      # 1 + T / T0 = 2
    assert rm.look_fluctuation(g, 2.8, None, 8, 100 * t0).k == pytest.approx(8.0 * div, rel=0.03)  # capped at n
    assert math.isinf(rm.look_fluctuation(rm.TARGETS["DECOY"], 2.8, None, 8, 1.0).k)
    assert rm.look_fluctuation(rm.TARGETS["KKV"], 14.0, None, 1, 0.0).k == pytest.approx(
        2.0 * rm.diversity_factor(rm.TARGETS["KKV"], 14.0), rel=0.03)


def test_target_coherence_time_drives_the_coherence_loss():
    fast = rm.Target("f", 1.0, 1, 5000.0, length_m=50.0, rotation_dps=5.0)
    assert rm.coherence_loss_db(2.8, 0.05, target=fast) > rm.coherence_loss_db(2.8, 0.05)
    assert rm.coherence_loss_db(2.8, 0.05, target=rm.Target("s", 1.0, 1, 5000.0, rotation_dps=0.0)) < 1e-3
    assert rm.coherence_loss_db(14.0, 0.05) > rm.coherence_loss_db(2.8, 0.05)                 # T0 ~ lambda


def test_depolarisation_gain_and_acceleration_bank():
    """The dual-pol receiver collects 1 + rho of the copolar energy; the acceleration bank grows with a_max and a
    TRACK look knows the acceleration."""
    a = rm.solve(rm.WF_LRS, 2.8, 0.0, rm.Target("a", 10.0, 1, 5000.0, depol_s_db=-3.0, accel_max_mps2=0.0))
    assert a.losses["target_depolarisation"] == pytest.approx(-10 * math.log10(1 + 10 ** -0.3))
    assert a.losses["acceleration"] == 0.0 and a.pulse["acceleration_hypotheses"] == 1
    n1, l1 = rm.acceleration_bank(10.0, 2.8, 0.0, 0.6, 75.0, False)
    n2, l2 = rm.acceleration_bank(1000.0, 2.8, 0.0, 0.6, 75.0, False)
    assert n1 <= n2 and n2 > 1 and 0.0 < l2 <= rm.BANK_MAX_LOSS_DB
    assert rm.acceleration_loss_db(9.8, 2.8, 0.1, 0.1, 75.0, True) == pytest.approx(
        rm.velocity_mismatch_loss_db(round(9.8 * 0.01 / rm.lambda_m(2.8), 6))
        + rm.walk_loss_db(9.8 * 0.01 / (8 * rm.C_MPS / 150e6), True), abs=1e-3)
    r = rm.solve(tb.KU_TRACK, 14.0, 0.0, dataclasses.replace(rm.TARGETS["KKV"], accel_max_mps2=0.0))
    assert r.pulse["acceleration_max_mps2"] == 0.0 and r.losses["acceleration"] == 0.0   # step 3.3 (E1): the track
    # model passes +/- k sigma of the predicted acceleration as the target's accel_max_mps2 (a known one: 0)


def test_os_cfar_and_masking_outputs():
    """OS-CFAR costs more than CA-CFAR on homogeneous noise and less with fewer cells; masking envelopes: unweighted
    Doppler -13.26 dB beyond one cell, range at the volume's sidelobe level far out."""
    assert rm.os_cfar_loss_db(1024, 1e-8) > rm.cfar_loss_db(1024, 1e-8) > 0.0
    assert rm.os_cfar_loss_db(64, 1e-8) > rm.os_cfar_loss_db(1024, 1e-8)
    lv = rm.masking_levels_db(-30.0, True)
    assert lv["doppler"]["1 cells"] == pytest.approx(-13.26, abs=0.02)
    assert lv["range"]["3 cells"] == pytest.approx(-30.0, abs=1.5) and "doppler" not in rm.masking_levels_db(-30.0, False)


def test_sector_with_a_target_class_runs_and_is_valid():
    sec = {**md.new_sector([], "box"), "target_type": "KKV", "range_km": 500.0, "rr_target_s": 20.0,
           "v_close_max_mps": 25000.0}
    sec.pop("rcs_m2")
    sec.pop("swerling")
    m = md.empty_mission()
    m["sectors"] = [sec]
    assert md.validate_mission(m) == []
    r = mo.run(m, workers=1)["sectors"][0]
    assert r["status"] == "OK" and r["target"]["type"] == "KKV" and r["target"]["fluctuation"] == "SW3/4"
    assert r["target"]["rcs_m2"] == pytest.approx(rm.TARGETS["KKV"].rcs_at(r["wf"]["freq_ghz"]))
    m["sectors"][0]["target_type"] = "NOPE"
    assert any("unknown target_type" in e for e in md.validate_mission(m))


# ----------------------------------------------------------------------------- environment (step 2.5)
from core import environment as env  # noqa: E402


def test_structure_cache_is_current():
    """core/structure_cache.npz matches the [ship] geometry (else the tool recomputes it at every start)."""
    assert env.cache_is_current()


def test_ship_model_matches_the_locked_dimensions_and_its_placement_is_checked(tmp_path):
    """The 3D model gives the locked length and width; a dimension, an aperture station or a tilt that no longer match
    the model are refused at load time with a clear message; the 3D view shows the model's own triangles."""
    assert np.ptp(env.MODEL_VERTICES[:, 2]) == pytest.approx(params.P.get("ship.length_m"), rel=0.01)
    assert 2 * np.hypot(env.MODEL_VERTICES[:, 0], env.MODEL_VERTICES[:, 1]).max() == pytest.approx(
        params.P.get("ship.width_m"), rel=0.01)
    cases = ((_replace_value(_default_toml(), "ship", "width_m ", "74.68", "80.0"), "width_m"),
             (_replace_value(_default_toml(), "ship", "model_array_station_z_m ", "-59.298", "-58.298"),
              "not on a panel front face"),
             (_replace_value(_default_toml(), "geometry", "tilt_deg ", "25.0", "20.0"), "deg away from the face normal"))
    for i, (text, msg) in enumerate(cases):
        path = tmp_path / f"bad{i}.toml"
        path.write_text(text, encoding="utf-8")
        res = subprocess.run([PYTHON, "-c", "import core.environment"], cwd=tb.ROOT, capture_output=True, text=True,
                             env={**os.environ, params.PARAMS_ENV: str(path)}, timeout=300)
        assert res.returncode != 0 and msg in res.stderr, (msg, res.stderr[-500:])
    from core import geometry as geo
    import pyvista as pv
    b = pv.MultiBlock([h["mesh"] for h in geo.build_ship()["hull"]]).bounds
    assert b[5] / geo._M - b[4] / geo._M == pytest.approx(np.ptp(env.MODEL_VERTICES[:, 2]), abs=1e-6)


def test_static_3d_scene_builds():
    """Every builder of the static 3D scene runs (the GUI opens on it; a stale helper once broke the start-up)."""
    from core import geometry as geo
    assert geo.build_axes() and geo.build_face_normals() and geo.build_ship()["hull"]


def test_ray_casting_on_the_model():
    """The exact cone test equals a brute-force Moller-Trumbore intersection on random rays; a ray toward the axis at
    the widest radiator tier meets it at its tip (radiator material); a ray ahead of the nose meets nothing."""
    rng = np.random.default_rng(1)
    o = rng.uniform([-40.0, -40.0, -140.0], [40.0, 40.0, 20.0], size=(40, 3))
    d = rng.normal(size=(40, 3))
    d /= np.linalg.norm(d, axis=1)[:, None]
    t, mat = env.cast(o, d)
    tri = env.TRIANGLES
    for i in range(len(o)):
        e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
        p = np.cross(d[i], e2)
        det = np.einsum("ij,ij->i", e1, p)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        s = o[i] - tri[:, 0]
        u = np.einsum("ij,ij->i", s, p) * inv
        q = np.cross(s, e1)
        v = (q @ d[i]) * inv
        tt = np.einsum("ij,ij->i", e2, q) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (tt > 1e-9)
        ref = tt[hit].min() if hit.any() else np.inf
        assert (t[i] == pytest.approx(ref, rel=1e-9)) if np.isfinite(ref) else not np.isfinite(t[i]), i
    t, mat = env.cast(np.array([[60.0, 0.0, -85.0], [0.0, 60.0, 30.0]]), np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]))
    assert t[0] == pytest.approx(60.0 - 37.34, abs=0.05) and env.MATERIAL_NAMES[mat[0]] == "radiator"
    assert not np.isfinite(t[1]) and mat[1] == -1


def test_structure_masks_no_owned_direction_and_the_radiators_are_hidden():
    """With the 25 deg forward tilt, the structure blocks only directions owned by another face (theta 71 to 90 deg
    from this face's normal) and the faces never see the radiator wings."""
    sph = mo.Sphere(res_deg=0.5)
    for k in range(len(mo.FACE_PHI)):
        sel = (sph.owner == k) & (sph.theta <= rm.SECTOR_RADIUS_DEG)
        f, _ = env.blockage(k, np.stack([np.cos(np.radians(sph.eps[sel])) * np.cos(np.radians(sph.phi[sel])),
                                             np.cos(np.radians(sph.eps[sel])) * np.sin(np.radians(sph.phi[sel])),
                                             np.sin(np.radians(sph.eps[sel]))], axis=1), "S")
        assert f.max() == 0.0
        t = env.tables()
        assert (t[f"f_{k}"] > 0).sum() > 0 and not (t[f"sepst_S_{k}"] > 30.0).any()   # hull only (eps T 2.9 K)
    assert env.block_loss_db(np.array([0.0, 0.293]))[1] == pytest.approx(3.0, abs=0.02)


def test_surface_shadow_and_clutter_geometry():
    """Distance to a surface along its centre direction = D - R; the clutter needs the iso-range ring (d_min < r <
    horizon); the clutter-as-noise factor 1 / (1 - x) is infinite (clutter-limited) at x >= 1."""
    sky = {"bodies": [{"id": "E", "type": "earth", "phi_deg": 0.0, "eps_deg": -35.0, "angular_radius_deg": 13.97}]}
    s = env.surfaces(sky)[0]
    c = np.array([s.c])
    assert env.surface_distance_km(s, c)[0] == pytest.approx(s.dist_km - s.radius_km)
    assert s.d_min_km == pytest.approx(6371 / math.sin(math.radians(13.97)) - 6371)
    dirs = np.array([mo.face_frame(0.0)[0]])
    assert env.clutter_over_signal(s, 0, dirs, "S", 2.8, -25.0, s.d_min_km * 0.9, 2.0, 1.0)[0] == 0.0
    assert env.clutter_over_signal(s, 0, dirs, "S", 2.8, -25.0, (s.d_min_km + s.d_horizon_km) / 2, 2.0, 1.0)[0] > 0.0
    assert list(env.clutter_noise_factor(np.array([0.0, 0.5, 1.0]))) == [1.0, 2.0, math.inf]
    assert env.doppler_rejected(env.Surface("m", (0, 0, 1), 1.0, 10.0, (0.1, 0.1), 0.0, 500.0), 1000.0, 5000.0)
    assert not env.doppler_rejected(s, 1000.0, 5000.0)                   # no closing speed given: prudent


def test_jammers_main_lobe_sidelobes_and_nulling():
    """A SOJ_SHIP at 10 000 km: J/N about +57 dB in the main lobe (S), its sidelobe level nulled by 40 dB."""
    n = np.array(mo.face_frame(0.0)[0])
    sky = {"jammers": [{"id": "J", "class": "SOJ_SHIP", "phi_deg": 0.0, "eps_deg": rm.SECTOR_TILT_DEG,
                        "range_km": 10000.0}]}
    j = env.jammers(sky)
    main = env.jammer_rise_k(j, 0, n[None, :], "S", 2.8, -25.0)["J"][0]
    assert rm.lin_to_db(main / rm.BAND_S.t_sys_k) == pytest.approx(57.0, abs=1.5)
    off = np.array([mo.face_frame(0.0)[1]]) * math.sin(math.radians(30)) + n * math.cos(math.radians(30))
    one = env.jammer_rise_k(j, 0, off, "S", 2.8, -25.0)["J"][0]
    import core.environment as e2
    saved = e2.JAM_MAX_NULLED
    try:
        e2.JAM_MAX_NULLED = 0
        raw = env.jammer_rise_k(j, 0, off, "S", 2.8, -25.0)["J"][0]
    finally:
        e2.JAM_MAX_NULLED = saved
    assert one == pytest.approx(raw * 10 ** (-env.JAM_NULL_DB / 10), rel=1e-9)
    assert any("unknown class" in m for m in env.validate_jammers({"jammers": [{**sky["jammers"][0], "class": "X"}]}))


def test_burn_through_scales_and_cfar_variants():
    """R_bt ~ sqrt(energy / EIRP density); GO-CFAR exact loss between CA and OS at 1024 cells (0.051 dB), 0.76 dB at 64."""
    r1 = env.burn_through_km(1e6, 1e4, 1.0, 1e-3, 13.0, 3.0, 10.0)
    assert env.burn_through_km(1e6, 1e4, 1.0, 4e-3, 13.0, 3.0, 10.0) == pytest.approx(2 * r1)
    assert env.burn_through_km(1e6, 1e4, 1.0, 1e-3, 13.0, 3.0, 30.0) == pytest.approx(r1 / 10)
    assert rm.go_cfar_loss_db(1024, 1e-8) == pytest.approx(0.051, abs=0.002)
    assert rm.go_cfar_loss_db(64, 1e-8) == pytest.approx(0.76, abs=0.01)
    assert rm.cfar_loss_db(1024, 1e-8) < rm.go_cfar_loss_db(1024, 1e-8) < rm.os_cfar_loss_db(1024, 1e-8)
    assert env.faraday_round_trip_deg(50.0, 20000.0, 2.8) == pytest.approx(
        math.degrees(2 * 2.36e4 * 2e-5 * 5e17 / 2.8e9 ** 2))


def test_environment_mission_outputs(runs):
    """mission_environment (test values only): Earth clutter and shadow, a self-protection jammer below the required
    range, stand-off jammers blinding a few positions, objects added to the confirmation, Faraday output."""
    res = runs["mission_environment.json"]
    by = {s["id"]: s for s in res["sectors"]}
    e = by["E_EARTH"]
    assert e["sky"]["clutter_limited_pct"] > 0 and e["sky"]["surfaces"]["EARTH"]["shadowed_pct"] > 0
    assert e["environment"]["faraday_deg"]["S"] > e["environment"]["faraday_deg"]["Ku"] > 0
    k = by["E_KKV_SPJ"]["environment"]
    assert k["burn_through_best_km"] >= k["burn_through_km"] and k["burn_through_best_km"] < 500.0
    assert any(b.startswith("jammer") for b in by["E_JAMMED"]["sky"]["blinded_by"])
    assert any("burn-through" in f for f in by["E_KKV_SPJ"]["flags"])
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_environment.json"))
    m["sectors"] = m["sectors"][1:2]                                   # E_KKV_SPJ alone, with and without objects
    m["sectors"][0]["objects_per_rr"] = 0.0
    t0 = mo.run(m, workers=1)["tracking"]["faces"]["1"]
    m["sectors"][0]["objects_per_rr"] = 5.0
    t5 = mo.run(m, workers=1)["tracking"]["faces"]["1"]
    assert t5["confirm_energy"] > t0["confirm_energy"]


# ----------------------------------------------------------------------------- S frequency grid (step 3.1, A)
def test_frequency_grid_is_derived_from_the_band_and_the_exclusions():
    grid = mo.centre_grid()
    assert grid[0] == rm.BAND_S.usable_lo_ghz and grid[-1] == pytest.approx(rm.BAND_S.usable_hi_ghz)
    assert len(grid) == round((rm.BAND_S.usable_hi_ghz - rm.BAND_S.usable_lo_ghz) / mo.FREQ_STEP_GHZ) + 1
    lo = rm.BAND_S.usable_lo_ghz
    assert mo.signal_fits(lo + 0.1, 200.0, ()) and not mo.signal_fits(lo + 0.095, 200.0, ())
    assert not mo.signal_fits(2.8, 20.0, ((2.79, 2.795),)) and mo.signal_fits(2.8, 10.0, ((2.79, 2.795),))
    # every free fragment keeps a centre in the coarse pass, even one narrower than the coarse step
    excl = ((rm.BAND_S.usable_lo_ghz, 2.795), (2.805, rm.BAND_S.usable_hi_ghz))
    assert mo.coarse_centres(10.0, excl) == [2.8]
    assert all(mo.signal_fits(f, 10.0, excl) for f in mo.valid_centres(10.0, excl))


def test_two_pass_frequency_search_matches_the_exhaustive_one(monkeypatch):
    """The two-pass frequency search finds the exhaustive optimum (one lattice spacing: the frequency search is
    tested, not the spacing choice, which test_optimiser_keeps_the_cheapest_spacing covers; the quiet-spectrum
    shortcut A6 is switched off here and tested on its own)."""
    monkeypatch.setattr(rm, "BEAM_SPACING_OPTIONS", (rm.BEAM_SPACING_HPBW,))
    monkeypatch.setattr(mo, "FREQ_QUIET_CENTRES", 0)
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    sec = m["sectors"][0]
    sph = mo._sphere(-90.0, 90.0)
    p_avg = rm.face_elec_avail_w(m["global"].get("face_elec_mw", rm.FACE_ELEC_POWER_MW))
    two = mo._best_plan(sec, sph, sph.mask(sec), p_avg)
    cands = mo.candidate_waveforms(sec)
    allf = mo.valid_centres(min(w.sig_bw_mhz for w in cands), ())
    full = mo._best_plan(dict(sec, freq_ghz_candidates=allf), sph, sph.mask(sec), p_avg)
    assert two[3].objective == pytest.approx(full[3].objective, rel=1e-2)
    assert two[3].short_frac == full[3].short_frac


def test_quiet_spectrum_evaluates_few_centres_and_stays_near_the_full_search(monkeypatch):
    """A6 (25 October 2026): with no exclusion and no jammer, FREQ_QUIET_CENTRES centres spread over the core band;
    the plan stays within 3 % of the full two-pass search (one lattice spacing). An exclusion restores the search."""
    monkeypatch.setattr(rm, "BEAM_SPACING_OPTIONS", (rm.BEAM_SPACING_HPBW,))
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    sec = m["sectors"][0]
    assert mo.quiet_spectrum(sec, None) and not mo.quiet_spectrum(dict(sec, spectrum_exclusions_ghz=[[2.7, 2.72]]), None)
    fs = mo.quiet_centres(mo.valid_centres(10.0, ()), 3, rm.BAND_S)
    assert len(fs) == 3 and all(rm.BAND_S.core_lo_ghz <= f <= rm.BAND_S.core_hi_ghz for f in fs)
    sph = mo._sphere(-90.0, 90.0)
    p_avg = rm.face_elec_avail_w(m["global"].get("face_elec_mw", rm.FACE_ELEC_POWER_MW))
    quiet = mo._best_plan(sec, sph, sph.mask(sec), p_avg)
    assert quiet[0] in fs
    monkeypatch.setattr(mo, "FREQ_QUIET_CENTRES", 0)
    full = mo._best_plan(sec, sph, sph.mask(sec), p_avg)
    assert quiet[3].short_frac == full[3].short_frac
    assert full[3].objective <= quiet[3].objective * (1 + 1e-9) and quiet[3].objective == pytest.approx(
        full[3].objective, rel=3e-2)


def test_ties_are_broken_deterministically():
    assert mo.snap(1.0 + 1e-13) == mo.snap(1.0) and mo.snap(1.0 + 1e-6) > mo.snap(1.0)
    assert np.array_equal(mo.snap(np.array([3.0, 3.0 * (1 + 1e-14)])), np.array([3.0, 3.0]))


@pytest.mark.parametrize("glob,sec,msg", [
    ({"spectrum_exclusions_ghz": {"S": [[2.8, 2.7]]}}, {}, "lo < hi"),
    ({"spectrum_exclusions_ghz": {"X": []}}, {}, "spectrum_exclusions_ghz must be"),
    ({}, {"freq_ghz_candidates": [3.5]}, "outside the S band"),
    ({}, {"freq_ghz_candidates": [rm.BAND_S.usable_lo_ghz]}, "no signal of the search space fits"),
    ({"spectrum_exclusions_ghz": {"S": [[2.7, 2.9]]}}, {"freq_ghz_candidates": [2.8]}, "no signal of the search space fits"),
])
def test_invalid_spectrum_inputs_are_refused(glob, sec, msg):
    m = md.empty_mission()
    m["global"].update(glob)
    m["sectors"] = [{**md.new_sector([], "box"), **sec}]
    assert any(msg in e for e in md.validate_mission(m)), md.validate_mission(m)


def test_exclusions_move_the_frequency_and_are_flagged():
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    m["sectors"] = m["sectors"][:1]
    m["global"]["spectrum_exclusions_ghz"] = {"S": [[2.70, 2.80]]}
    s = mo.run(m, workers=1)["sectors"][0]
    lo, hi = s["frequency"]["occupied_ghz"]
    assert s["status"] == "OK" and (hi <= 2.70 or lo >= 2.80)
    assert s["frequency"]["centres_excluded"] > 0 and any("spectrum exclusions remove" in f for f in s["flags"])
    m["global"]["spectrum_exclusions_ghz"] = {"S": [[2.64, 2.88]]}     # free fragments of 40 and 50 MHz: no 75 MHz signal
    assert mo.run(m, workers=1)["sectors"][0]["status"] == "NO_WAVEFORM"


# ----------------------------------------------------------------------------- S waveform space (step 3.1, B)
def test_search_space_holds_the_short_and_long_pulses_and_wide_bands():
    assert {5e-6, 10e-6, 1.0e-3, 1.2e-3, 1.5e-3} <= set(mo.TAU_CAND) and max(mo.TAU_CAND) == rm.tau_max_s(rm.BAND_S)
    assert {100.0, 150.0, 200.0} <= set(mo.SIGBW_CAND)
    assert rm.blind_range_km(rm.BAND_S, 10e-6) < 2.0 < rm.blind_range_km(rm.BAND_S, 20e-6)   # 2 km self-defence (B1)


def test_frequency_agility_decorrelates_the_target():
    """Draws = floor(df / (c / 2L)) + 1; a declared SW1 tends to SW2 (Gamma shape n) when every pulse is a draw."""
    st = rm.TARGETS["ST"]                                    # L = 10 m: c / 2L = 15 MHz
    assert rm.frequency_draws(st, 0.0) == 1 and rm.frequency_draws(st, 14.9) == 1 and rm.frequency_draws(st, 15.0) == 2
    assert rm.look_fluctuation(st, 2.8, 1, 8, 1e-3, 7 * 15.0).k == pytest.approx(
        rm.look_fluctuation(st, 2.8, 2, 8, 1e-3).k)
    with pytest.raises(ValueError):
        mo.make_wf("x", 50e-6, 4, 1e-3, 10.0, 1, "COHERENT", agility=30.0)
    w0 = mo.make_wf("x", 50e-6, 8, 2e-3, 10.0, 1, "NONCOHERENT")
    w1 = mo.make_wf("x", 50e-6, 8, 2e-3, 10.0, 1, "NONCOHERENT", agility=105.0)
    sw1 = rm.Target("t", 10.0, 1, 3000.0, length_m=10.0)
    assert w1.occupied_mhz == 115.0
    assert rm.solve(w1, 2.77, 0.0, sw1).range_km > rm.solve(w0, 2.77, 0.0, sw1).range_km * 1.05


def test_agile_variants_fit_the_band():
    sec = dict(md.new_sector([]), res_max_m=20.0)
    ag = [w for w in mo.candidate_waveforms(sec) if w.agility_mhz > 0]
    width = (rm.BAND_S.usable_hi_ghz - rm.BAND_S.usable_lo_ghz) * 1e3
    assert ag and all(w.integration == "NONCOHERENT" and w.n >= 2 and w.occupied_mhz <= width + 1e-9 for w in ag)


def test_pruning_of_dominated_candidates_is_exact():
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    sec = m["sectors"][0]
    sph = mo._sphere(-90.0, 90.0)
    pos, _, _, factor = mo.search_positions(sec, sph, sph.mask(sec), 2.8, None)
    P = np.array([pos[k] for k in range(4)])
    cands = mo.candidate_waveforms(sec)
    r = mo._range_matrix(cands, 2.8, mo.sector_target(sec), P.shape[1], mo.rx_sll_db(sec), mo.velocity_span_mps(sec),
                         mo.candidate_pfas(sec, cands, 2.8, float(P.sum())), mo.search_criterion(sec)) * factor
    costs = mo.PositionCosts(cands)
    p_avg = rm.face_elec_avail_w(2.0)
    keep = mo.prune_dominated(r, costs.keys)
    assert len(keep) < len(cands)
    full = mo._plan_waveforms(P, r, costs, sec["range_km"], sec["rr_target_s"], p_avg, 4, 0.01)
    part = mo._plan_waveforms(P, r[keep], costs.subset(keep), sec["range_km"], sec["rr_target_s"], p_avg, 4, 0.01)
    assert part.objective == pytest.approx(full.objective, rel=1e-9) and part.short_frac == full.short_frac


# ----------------------------------------------------------------------------- TX multibeam (step 3.1, C)
def _wf(**kw) -> "rm.Waveform":
    return mo.make_wf("t", kw.pop("tau", 50e-6), kw.pop("n", 4), kw.pop("pri", 4e-3), 100.0, kw.pop("b", 1),
                      "COHERENT", **kw)


def test_multibeam_values_and_modes():
    assert rm.MB_SUP_LOSS_DB == {2: 0.4, 4: 0.6, 8: 0.8} and rm.MB_PART_BEAMS == (2, 4) and rm.MB_TDM_BEAMS == (2, 4, 8)
    assert [rm.multibeam_tx_loss_db(_wf(b=b, mode=m)) for b, m in ((1, "single"), (4, "superposition"),
                                                                    (4, "partition"), (4, "tdm"))] == [0.0, 0.6, 0.0, 0.0]
    for bad in (dict(b=3, mode="superposition"), dict(b=8, mode="partition"), dict(b=2, mode="partition"),
                dict(b=2, mode="partition", cut="x"), dict(b=4, mode="partition", cut="h"), dict(b=2, mode="beam")):
        with pytest.raises(ValueError):
            _wf(**bad)
    assert "tdm" not in mo.TX_MODES and [v[0] for v in mo.tx_variants()].count("superposition") == 3


def test_superposition_range_is_the_scaled_one_beam_range():
    """The optimiser derives a superposition's range from its one-beam twin (P / B, table loss): same as solve()."""
    tgt = rm.TARGETS["ST"]
    for b in (2, 4, 8):
        w = _wf(b=b, mode="superposition")
        direct = rm.solve(w, 2.8, 30.0, tgt).range_km
        assert mo.range_at(w, 2.8, 30.0, tgt) == pytest.approx(direct, rel=1e-12)
        assert direct == pytest.approx(rm.solve(_wf(), 2.8, 30.0, tgt).range_km
                                       * (b * rm.db_to_lin(rm.MB_SUP_LOSS_DB[b])) ** -0.25, rel=1e-12)


def test_partition_beam_gain_and_lattice():
    """Half: 2450 TRMs, P / 2 and G / 2, cell x1.85 (widened beam along the cut); quadrant 1225, x3.43; both halves the
    same beam rotated by 90 deg, so one cut is searched; positions per dwell = B x the cell ratio."""
    h, v, q = (_wf(b=2, mode="partition", cut="h"), _wf(b=2, mode="partition", cut="v"), _wf(b=4, mode="partition"))
    assert rm.tx_gain_ratio(h) == 0.5 and rm.tx_gain_ratio(q) == 0.25
    assert rm.tx_beam_power_w(h) == rm.BAND_S.p_pk_total_w / 2 and rm.tx_beam_power_w(q) == rm.BAND_S.p_pk_total_w / 4
    sh, sq = rm.subaperture("S", "half_h"), rm.subaperture("S", "quarter")
    assert sh.hpbw_u == pytest.approx(sh.full_hpbw_u, rel=1e-6) and sh.hpbw_v / sh.full_hpbw_v == pytest.approx(1.852, abs=0.002)
    assert rm.subaperture_cell_ratio("S", "quarter") == pytest.approx(3.430, abs=0.002)
    assert rm.cells_per_dwell(h) == pytest.approx(2 * 1.852, abs=0.005) and rm.partition_cuts() == ("h",)
    tgt = rm.TARGETS["ST"]
    assert rm.solve(h, 2.8, 0.0, tgt).range_km == pytest.approx(rm.solve(v, 2.8, 0.0, tgt).range_km, rel=1e-12)
    # grid loss of the widened beam on its own lattice: within 0.1 dB of the full face's (same spacing in beamwidths)
    full = rm.position_losses("S", -25.0, -25.0, True, False, (0.0, 1e-4, 100.0, 1e-4, 1, True))[2]["grid_tx"]
    half = rm.position_losses("S", -25.0, -25.0, True, False, (0.0, 1e-4, 100.0, 1e-4, 1, True), "half_h")[2]["grid_tx"]
    assert abs(half - full) < 0.1
    fr = rm.solve(h, 2.8, 0.0, tgt).frame
    assert fr["t_frame_s"] == pytest.approx(fr["positions"] * fr["t_pos_s"] / rm.cells_per_dwell(h), rel=1e-12)


def test_time_multiplex_blocks():
    """Q1: B full-power pulses back to back: blind range c B tau / 2, duty and PAE on B tau, one beam's energy per
    position, B positions per dwell; refused when B tau / PRI exceeds the duty ceiling."""
    w = _wf(b=4, mode="tdm", tau=100e-6, pri=1e-3)
    assert w.tx_block_s == 400e-6 and w.trm_duty == pytest.approx(0.4) and rm.cells_per_dwell(w) == 4
    assert rm.pae_wf(w) == rm.pae(rm.BAND_S, 400e-6, 0.4)
    assert rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"]).pulse["blind_range_km"] == rm.blind_range_km(rm.BAND_S, 400e-6)
    e, t, _, _ = rm.search_position_costs(w)
    assert t == pytest.approx(4 * 100e-6) and e == pytest.approx(rm.BAND_S.p_pk_total_w * t / rm.pae_wf(w))
    with pytest.raises(ValueError):
        _wf(b=8, mode="tdm", tau=100e-6, pri=1e-3)               # duty 0.8 > 0.65


def test_pulse_shortening_costs():
    """'No excess energy' (Q4, Q5 a): the pulse is cut to s tau, PAE of the shortened pulse, floor tau B >= 1."""
    w = _wf(tau=10e-6, n=1)
    e1, t1, s1, eta1 = rm.search_position_costs(w)
    e, t, s, eta = rm.search_position_costs(w, 0.25)
    assert s1 == 1.0 and s == 0.25 and t == pytest.approx(t1 / 4) and eta == pytest.approx(rm.pae(rm.BAND_S, 2.5e-6))
    assert e == pytest.approx(rm.BAND_S.p_pk_total_w * 2.5e-6 / eta) and e > e1 / 4    # the edge time weighs more
    _, _, s_floor, _ = rm.search_position_costs(w, 1e-6)
    assert s_floor == pytest.approx(1.0 / (100e6 * 10e-6))                              # 10 ns at 100 MHz
    c = mo.PositionCosts([w, _wf(b=4, mode="superposition", tau=10e-6, n=1)])
    ec, tc, _, _ = c(np.array([[0.25], [0.25]]))
    assert ec[0, 0] == pytest.approx(e) and ec[1, 0] == pytest.approx(e / 4) and tc[1, 0] == pytest.approx(t / 4)


# ----------------------------------------------------------------------------- environment per waveform (step 3.1, P4)
def test_doppler_weighting_option():
    """Q5: Taylor -30 / -40 dB Doppler weighting on coherent trains only, its loss in the budget."""
    assert rm.DOPPLER_WINDOW_OPTIONS_DB == (-30.0, -40.0)
    w = mo.make_wf("t", 50e-6, 8, 2e-3, 100.0, 1, "COHERENT", window=-40.0)
    assert rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"]).losses["doppler_weighting"] == rm.range_weighting(-40.0)["loss_db"]
    for bad in (dict(integ="NONCOHERENT", n=8, win=-40.0), dict(integ="COHERENT", n=1, win=-40.0),
                dict(integ="COHERENT", n=8, win=-35.0)):
        with pytest.raises(ValueError):
            mo.make_wf("t", 50e-6, bad["n"], 2e-3, 100.0, 1, bad["integ"], window=bad["win"])
    assert mo.doppler_windows({"range_km": 1000.0}, None) == ()


def test_self_protection_jammer_in_the_range():
    """Q11: S = SNR (N + J), J ~ 1 / R^2: 1 / R^4 = 1 / R_n^4 + 1 / (R_bt^2 R^2), below both ranges; the superposition
    shortcut keeps it exact (R_bt x (1 / (B L))^(1/2))."""
    t, w = rm.TARGETS["KKV"], rm.WF_LRS
    d = 10.0 ** (5.0 / 10.0) / 1e6
    r = rm.solve(w, 2.8, 0.0, t, spj_density_w_hz=d)
    assert r.range_noise_km == pytest.approx(rm.solve(w, 2.8, 0.0, t).range_km, rel=1e-12)
    assert r.range_km ** -4 == pytest.approx(r.range_noise_km ** -4 + 1.0 / (r.burn_through_km ** 2 * r.range_km ** 2),
                                             rel=1e-9)
    assert r.range_km < min(r.range_noise_km, r.burn_through_km)
    assert rm.jammed_range_km(1000.0, math.inf) == 1000.0
    sup = mo.make_wf("t", 100e-6, 4, 4e-3, 100.0, 4, "COHERENT", mode="superposition")
    p = mo.parts_at(sup, 2.8, 0.0, t, None, None, None, None, d)
    direct = rm.solve(sup, 2.8, 0.0, t, spj_density_w_hz=d)
    assert p[0] == pytest.approx(direct.range_noise_km, rel=1e-12) and p[1] == pytest.approx(direct.burn_through_km, rel=1e-12)
    sec = {"jammer_type": "SPJ_KKV"}
    assert env.spj_density_w_hz(sec, 2.8) == pytest.approx(10.0 ** (env._erp("SPJ_KKV", None)[0] / 10.0) / 1e6)
    assert env.spj_density_w_hz({}, 2.8) is None


def test_doppler_residual_of_the_clutter():
    """Q4-Q8: no rejection without the scenario's closing speed or without a coherent block; a clutter spectrum filling
    the PRF leaves the filter's noise bandwidth; a narrow one far from the target is rejected, more with a weighting;
    the residual phase sets a floor."""
    args = (1000.0, 5000.0, 2.8, 1e-3)
    assert set(env.doppler_residual(None, None, *args, 16, None, 0.0)) == {1.0}
    assert set(env.doppler_residual(0.0, 1.0, *args, 1, None, 0.0)) == {1.0}
    wide = env.doppler_residual(0.0, 5000.0, *args, 16, None, 0.0)
    assert max(abs(x - 1.0 / 16) for x in wide) < 1e-6
    far0 = env.doppler_residual(0.0, 1.0, *args, 16, None, 0.0)
    far40 = env.doppler_residual(0.0, 1.0, *args, 16, -40.0, 0.0)
    assert min(far0) < 1e-2 and np.median(far40) < np.median(far0)
    floor = env.doppler_residual(0.0, 1.0, *args, 16, None, 1e-2)
    assert min(floor) >= 1e-2 / 16 * (1 - 1e-9)
    assert len(env.target_velocities(1000.0, 5000.0)) == env.CLUTTER_V_SAMPLES


def test_subaperture_clutter_pattern_tables():
    """Q3: the tabulated 2D pattern of a sub-aperture matches the direct sum (main lobe, sidelobes, both half planes)."""
    du, dv = np.array([0.0, 0.5, 0.0, 10.0, -3.0, 40.0]), np.array([0.0, 0.0, 0.9, 3.0, -0.4, -20.0])
    direct = rm.subaperture("S", "half_h").power(du, dv)
    assert np.allclose(env.subaperture_gain("S", "half_h", du, dv), direct, rtol=0.05, atol=1e-6)


def test_clutter_gain_restores_the_look_pd():
    """R1 b1: G = 1 without clutter, grows with the clutter, inf beyond what any power beats; the velocity-averaged Pd
    at s* G with the clutter equals the look's Pd without it."""
    from core import detection
    law = (1, detection.Fluct(1.0), 1e-6, (0.0,), (1.0,))
    lid = mo._law_id(law)
    curves = mo._curves(np.array([lid]))
    sdb = np.array([15.0, 15.0, 15.0])
    pstar = mo._pd_at(curves, np.array([lid] * 3), sdb)
    a = np.array([[1e-9] * 3, [1e-3] * 3, [1.0] * 3])
    g = mo._clutter_gain(curves, np.array([lid] * 3), sdb, pstar, a)
    assert g[0] == pytest.approx(1.0, abs=1e-6) and 1.0 < g[1] < math.inf and g[2] == math.inf
    back = mo._mean_pd_clutter(curves, np.array([lid]), sdb[1:2], a[1:2], np.log10(g[1:2]))
    assert back[0] == pytest.approx(pstar[1], abs=1e-6)


def test_requirement_brought_back_to_the_surface():
    """Q9, R2 c2: d - guard (guard = max(surface_guard_km, resolution)), rounded UP on 5 % of the range; <= 0: no
    volume; one class per pointing."""
    sec = {"range_km": 1000.0, "res_max_m": 2.0, "v_close_max_mps": 5000.0, "rcs_m2": 1.0}
    assert mo.surface_guard_km(sec) == 10.0 and mo.surface_guard_km(dict(sec, surface_guard_km=0.0)) > 0.0
    s = env.Surface("S1", (0.0, 0.0, 1.0), 100.0, 600.0, (0.1, 0.1), None, None)
    dirs = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    se = mo._surface_env_s(sec, 2.8, -25.0, dirs, np.zeros(3, dtype=int), (s,), mo.sector_target(sec))
    d = 600.0 - 100.0                                   # straight at the surface
    assert se["classes"][0] == 1000.0 and se["k_idx"][1] == 0
    k = se["k_idx"][0]
    assert se["classes"][k] == pytest.approx(math.ceil((d - 10.0) / 50.0) * 50.0) and se["classes"][k] >= d - 10.0
    near = env.Surface("S2", (0.0, 0.0, 1.0), 100.0, 105.0, (0.1, 0.1), None, None)
    se2 = mo._surface_env_s(sec, 2.8, -25.0, dirs, np.zeros(3, dtype=int), (near,), mo.sector_target(sec))
    assert se2["no_volume"][0] and not se2["no_volume"][1]


def test_environment_per_waveform_outputs(runs):
    """mission_environment: the self-protection jammer puts E_KKV_SPJ's 500 km out of the search's reach (best
    burn-through below it), so the sector is planned against the noise alone and flagged for a Ku burn-through search
    (R5 b); the Earth sector keeps clutter-limited positions."""
    by = {s["id"]: s for s in runs["mission_environment.json"]["sectors"]}
    k = by["E_KKV_SPJ"]
    assert k["environment"]["burn_through_best_km"] < k["range_km"]["requested"]
    assert k["sky"].get("spj_out_of_reach") and any("Ku burn-through" in x for x in k["flags"])
    assert k["range_km"]["at_worst_scan"] >= k["range_km"]["requested"]
    assert by["E_EARTH"]["sky"]["clutter_limited_pct"] > 0.0


# ----------------------------------------------------------------------------- sky refinement and lattice spacing (3.1, P5)
def _sun_sector(dphi: float = 0.0):
    sec = dict(md.new_sector([]), id="SUNX", shape="box", phi_min_deg=5.0 + dphi, phi_max_deg=35.0 + dphi,
               eps_min_deg=10.0, eps_max_deg=30.0, range_km=1500.0)
    sec.pop("frame", None)
    sky = {"bodies": [{"id": "SUN", "type": "sun", "phi_deg": 20.0 + dphi, "eps_deg": 20.0,
                       "angular_radius_deg": 0.267}]}
    return sec, sky


def test_sky_refinement_is_invariant_under_rotation():
    """S1-S4: cells near a hot source are evaluated one by one, split into refine_subsamples^2 points (option 3);
    turning the sky and the sector together about the ship axis by angles off the raster changes the budget T_sys and
    the blinded fraction by less than 1 % (D7; 0.81 % and 0.40 % at most, 1.05 % and 1.41 % with per-cell values)."""
    sph = mo._sphere(-90.0, 90.0)
    ref = None
    for dphi in (0.0, 0.37, 1.13):
        sec, sky = _sun_sector(dphi)
        s, _ = mo.sector_sky(sec, sph, sph.mask(sec), 2.8, "S", sky)
        if ref is None:
            ref = s
            assert s["blinded_pct"] > 0.0
            continue
        assert s["t_sys_budget_k"] == pytest.approx(ref["t_sys_budget_k"], rel=0.01)
        assert s["blinded_pct"] == pytest.approx(ref["blinded_pct"], rel=0.01)
    sec, sky = _sun_sector()
    m = sph.mask(sec)
    ii, jj = np.nonzero(m & sph.scan_zone)
    fine = mo._refine_cells(sky_mod_parse(sky), sky, rm.BAND_S, "S", 2.8, -25.0, sph, ii, jj, ii, jj,
                            sph.owner[ii, jj].astype(np.int64))
    assert 0 < fine.sum() < len(fine)


def test_sky_rotation_drift_is_measured():
    """Option 3, point 6: over larger rotations the budget T_sys drifts steadily (about 1.4 % at 2.9 deg), believed
    physical (the faces stay fixed, so the scan angle and the pattern change) and open until 5.6. No criterion: the
    test prints the drift and checks only that the blinded fraction stays within 1 %."""
    sph = mo._sphere(-90.0, 90.0)
    out = []
    for dphi in (0.0, 1.57, 2.9):
        sec, sky = _sun_sector(dphi)
        s, _ = mo.sector_sky(sec, sph, sph.mask(sec), 2.8, "S", sky)
        out.append((dphi, s["t_sys_budget_k"], s["blinded_pct"]))
    print("rotation drift (deg, budget %, blinded %):",
          [(d, round(100 * (b / out[0][1] - 1), 2), round(100 * (p / out[0][2] - 1), 2)) for d, b, p in out[1:]])
    assert all(np.isfinite(b) for _, b, _ in out)
    assert all(p == pytest.approx(out[0][2], rel=0.01) for _, _, p in out)


def test_refined_cells_are_shared_point_by_point():
    """Option 3: a refined cell on the edge of the region blinded around the Sun is scanned for a share between 0 and
    1, and the positions count that share."""
    sph = mo._sphere(-90.0, 90.0)
    sec, sky = _sun_sector()
    m = sph.mask(sec)
    _, keep = mo.sector_sky(sec, sph, m, 2.8, "S", sky)
    k = keep[m & sph.scan_zone]
    assert (k == 0.0).any() and ((k > 0.0) & (k < 1.0)).any()
    pos, _, _, _ = mo.search_positions(sec, sph, m, 2.8, sky)
    pos_bin, _ = mo.positions_table(sph, m & (keep == 1.0), 2.8, mo.grid_spacing(sec))
    pos_all, _ = mo.positions_table(sph, m & (keep > 0.0), 2.8, mo.grid_spacing(sec))
    tot = lambda p: sum(float(v.sum()) for v in p.values())    # noqa: E731
    assert tot(pos_bin) < tot(pos) < tot(pos_all)


def test_numba_sky_sums_equal_the_numpy_ones():
    """N4: the compiled sums of body_rise_k (small body, near zone, far lobes) equal the numpy path to 1e-12: a small
    body (Sun) and a large one (Earth, 60 deg), beam axes around each."""
    from core import sky as skym
    if skym._nb is None:
        pytest.skip("numba not installed")
    sky = skym.parse({"bodies": [{"id": "SUN", "type": "sun", "phi_deg": 20.0, "eps_deg": 20.0,
                                  "angular_radius_deg": 0.267},
                                 {"id": "E", "type": "earth_ocean", "phi_deg": 200.0, "eps_deg": -60.0,
                                  "angular_radius_deg": 60.0}]})
    rng = np.random.default_rng(3)
    frames = [mo.face_frame(p) for p in mo.FACE_PHI]
    seen = 0
    for b in sky.bodies:
        c = np.array(b.c)
        fr = frames[int(np.argmax([np.dot(f[0], c) for f in frames]))]
        v = rng.normal(size=(3000, 3))
        v -= (v @ c)[:, None] * c
        v /= np.linalg.norm(v, axis=1)[:, None]
        a = np.radians(rng.uniform(0.0, math.degrees(b.radius_rad) + 6.0, 3000))
        d = np.cos(a)[:, None] * c + np.sin(a)[:, None] * v
        d = d[d @ fr[0] > 0.1]
        saved = skym.USE_NUMBA
        try:
            skym.USE_NUMBA = False
            ref = skym.body_rise_k(b, "S", 2.8, -25.0, fr, d)
            skym.USE_NUMBA = True
            got = skym.body_rise_k(b, "S", 2.8, -25.0, fr, d)
        finally:
            skym.USE_NUMBA = saved
        assert np.allclose(got, ref, rtol=1e-12, atol=0.0)
        seen += int((ref > 0).sum())
    assert seen > 1000


def sky_mod_parse(sky):
    from core import sky as skym
    return skym.parse(sky)


def test_lattice_spacing_threads_through_the_model():
    """E7: positions go as 1 / s^2, RX beams per position as s^2, the TX grid loss grows with s, solve() and the
    sector positions follow the spacing given."""
    full = rm.n_search_positions(2.8)
    assert rm.BEAM_SPACING_OPTIONS == (1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0) and rm.BEAM_SPACING_HPBW == 1.4
    for s_ in rm.BEAM_SPACING_OPTIONS:
        assert rm.n_search_positions(2.8, spacing_hpbw=s_) == pytest.approx(full * (1.4 / s_) ** 2, rel=1e-12)
        assert rm.rx_beams_per_position(2.8, -25.0, s_) == pytest.approx(rm.rx_beams_per_position(2.8) * (s_ / 1.4) ** 2)
    g = [rm.position_losses("S", -25.0, -25.0, True, False, (0.0, 1e-4, 100.0, 1e-4, 1, True), None, None, s_)[2]["grid_tx"]
         for s_ in rm.BEAM_SPACING_OPTIONS]
    assert g == sorted(g) and g[1] == pytest.approx(
        rm.position_losses("S", -25.0, -25.0, True, False, (0.0, 1e-4, 100.0, 1e-4, 1, True))[2]["grid_tx"])
    w = rm.WF_LRS
    a, b = rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"], grid_spacing_hpbw=1.3), rm.solve(w, 2.8, 0.0, rm.TARGETS["ST"],
                                                                                    grid_spacing_hpbw=1.6)
    assert a.range_km > b.range_km and a.frame["positions"] > b.frame["positions"]
    sec = dict(md.new_sector([]), id="T1")
    sph = mo._sphere(-90.0, 90.0)
    p13 = mo.search_positions(dict(sec, _grid_spacing=1.3), sph, sph.mask(sec), 2.8, None)[0]
    p16 = mo.search_positions(dict(sec, _grid_spacing=1.6), sph, sph.mask(sec), 2.8, None)[0]
    assert sum(p13[k].sum() for k in p13) == pytest.approx(sum(p16[k].sum() for k in p16) * (1.6 / 1.3) ** 2, rel=1e-9)


def test_optimiser_keeps_the_cheapest_spacing(monkeypatch):
    """E7 a: the plan of a sector is the best over every spacing (here at one forced centre)."""
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    sec = dict(next(s for s in m["sectors"] if s["id"] == "S02"), freq_ghz_candidates=[2.8])
    sph = mo._sphere(-90.0, 90.0)
    best = mo._best_plan(sec, sph, sph.mask(sec), rm.face_elec_avail_w(2.0))
    objs = {}
    for s_ in rm.BEAM_SPACING_OPTIONS:
        monkeypatch.setattr(rm, "BEAM_SPACING_OPTIONS", (s_,))
        objs[s_] = mo._best_plan(sec, sph, sph.mask(sec), rm.face_elec_avail_w(2.0))[3].objective
    assert best[7] in objs and best[3].objective == pytest.approx(min(objs.values()), rel=1e-9)


def test_surface_data_survive_a_full_sky_cache():
    """The S surface data are kept with their sky summary even when the summary cache is cleared on insertion (a
    long process running every mission, as tests/update_snapshots.py)."""
    from core import frames
    m, _ = frames.resolve(md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_environment.json")))
    sec = next(s for s in m["sectors"] if s["id"] == "E_EARTH")
    sky = m["global"].get("sky")
    sph = mo._sphere(-90.0, 90.0)
    saved = dict(mo._SKY_CACHE), dict(mo._S_ENV)
    try:
        mo._SKY_CACHE.clear()
        mo._S_ENV.clear()
        for i in range(mo.MASK_CACHE_SIZE):
            mo._SKY_CACHE[("filler", i)] = None
        assert mo.sector_env_s(sec, sph, sph.mask(sec), 2.8, sky) is not None
    finally:
        mo._SKY_CACHE.clear()
        mo._S_ENV.clear()
        mo._SKY_CACHE.update(saved[0])
        mo._S_ENV.update(saved[1])


# ----------------------------------------------------------------------------- frames and navigation (step 3.0)
def _quat_z(deg: float) -> list:
    h = math.radians(deg) / 2.0
    return [math.cos(h), 0.0, 0.0, math.sin(h)]


def _framed(m: dict, nav=None) -> dict:
    """The mission with every sector and the sky in ICRF axes (same angles), and the given navigation."""
    m = json.loads(json.dumps(m))
    for sec in m["sectors"]:
        sec["frame"] = "inertial"
    if m["global"].get("sky") is not None:
        m["global"]["sky"]["frame"] = "inertial"
    if nav is not None:
        m["global"]["nav"] = nav
    return m


def _strip_frames(r: dict) -> dict:
    r = json.loads(json.dumps(tb.to_plain(r)))
    r["global"].pop("nav", None)
    for s in r["sectors"]:
        s.pop("frame", None)
    return r


def test_rotations_round_trip():
    from core import frames
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.normal(size=4)
        q = list(q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1))
        m = frames.quat_to_matrix(q)
        assert np.allclose(m @ m.T, np.eye(3), atol=1e-12) and np.linalg.det(m) == pytest.approx(1.0)
        assert np.allclose(frames.matrix_to_quat(m), q, atol=1e-12)
        y, p_, r_ = frames.euler_321_of(m)
        assert np.allclose(frames.euler_321(y, p_, r_), m, atol=1e-12)
    assert np.allclose(frames.quat_to_matrix(_quat_z(90.0)) @ [1, 0, 0], [0, 1, 0], atol=1e-15)


def test_rtn_limb_and_two_dir_axes():
    from core import frames
    r_e = frames.BODY_RADIUS_KM["earth"]
    a, i = r_e + 400.0, math.radians(30.0)
    nav = frames.Nav({"q_ship_to_icrf": [1, 0, 0, 0], "central_body": "earth", "r_km": [0.0, a, 0.0],
                      "v_kms": [-7.67 * math.cos(i), 0.0, 7.67 * math.sin(i)]})
    rtn = nav.rtn()
    assert np.allclose(rtn[0], [0, 1, 0]) and np.allclose(rtn[1], [-math.cos(i), 0, math.sin(i)])
    assert np.allclose(rtn[2], np.cross(rtn[0], rtn[1]))
    assert nav.limb_depression_deg(0.0) == pytest.approx(math.degrees(math.acos(r_e / a)), rel=1e-12)
    assert nav.limb_depression_deg(100.0) < nav.limb_depression_deg(0.0)
    sec = {"frame": "two_dir", "frame_p_ra_deg": 30.0, "frame_p_dec_deg": 10.0, "frame_s_ra_deg": 120.0,
           "frame_s_dec_deg": 0.0}
    m, off = frames.sector_matrix(sec, frames.Nav())
    assert off == 0.0 and np.allclose(m @ frames.unit(30.0, 10.0), [0, 0, 1])
    s_in = m @ frames.unit(120.0, 0.0)
    assert abs(s_in[1]) < 1e-12 and s_in[0] > 0


def test_limb_band_follows_the_altitude():
    """A limb box is an eps band below the local horizontal, acos((R + h) / |r|) lower; ship pointed with +Z zenith."""
    from core import frames
    r_e = frames.BODY_RADIUS_KM["earth"]
    sph = mo.Sphere(res_deg=1.0)
    for alt in (400.0, 2000.0):
        nav = [{"q_ship_to_icrf": [1, 0, 0, 0], "central_body": "earth", "r_km": [0.0, 0.0, r_e + alt],
                "v_kms": [7.0, 0.0, 0.0]}]
        sec = {"id": "H", "frame": "limb", "shape": "box", "phi_min_deg": 0.0, "phi_max_deg": 360.0,
               "eps_min_deg": 0.0, "eps_max_deg": 5.0}
        mk = sph.mask(frames.resolve_sector(sec, {"nav": nav}))
        dep = math.degrees(math.acos(r_e / (r_e + alt)))
        e = sph.eps[mk]
        assert mk.sum() > 0 and e.min() >= -dep - 1e-9 and e.max() <= -dep + 5.0 + 1e-9


def test_inertial_frame_with_identity_attitude_is_the_ship_frame():
    """Sectors and sky in ICRF axes with the identity attitude give the ship-frame run (the Sun and Moon sectors,
    one frequency: the frames are tested, the full mission is in the snapshots)."""
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_sky_demo.json"))
    m["sectors"] = [s for s in m["sectors"] if s["id"] in ("SUN", "MOON")]
    _one_frequency(m)
    ref = _strip_frames(mo.run(m, workers=1))
    got = _strip_frames(mo.run(_framed(m, [{"q_ship_to_icrf": [1.0, 0.0, 0.0, 0.0]}]), workers=1))
    # step 4.5: an ICRF volume re-points in flight whatever the attitude of the moment (its mask follows the ship's
    # rotation, and it migrates between faces): only that re-pointing work and its totals may differ
    repoint_paths = ("ops_per_s/repointing", "ops_total_per_s", "/load", "power_kW/ops")
    for r_ in (ref, got):                 # step 5.3: the same in the verdict (migration rows, compute loads)
        vd = r_["verdict"]
        vd["constraints"] = [c for c in vd["constraints"]
                             if c["quantity"] not in ("migration", "compute", "processing power")]
        vd.pop("counts")
    diffs = [d for d in tb.differences(ref, got)
             if not d[0].startswith("/repointing/sectors/")
             and not (d[0].startswith("/compute/faces/") and d[0].endswith(repoint_paths))]
    assert diffs == []


def test_turning_the_ship_turns_the_faces():
    """Volumes and sky fixed in ICRF, ship turned 90 deg about its axis: face k now does what face k+1 did (within
    the small asymmetry of the structure: radiator wings on +/-X, and the sampling of the sky noise, whose blocks
    are not invariant by rotation: the Sun, a hot point source, is left out, the Moon moves the budget by < 1 %)."""
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_sky_demo.json"))
    m["sectors"] = [s for s in m["sectors"] if s["id"] in ("SUN", "MOON")]
    m["global"]["sky"]["bodies"] = [b for b in m["global"]["sky"]["bodies"] if b["id"] == "MOON"]
    for s in m["sectors"]:          # one frequency: the optimum is flat in frequency (step 3.1); the faces are tested
        s["freq_ghz_candidates"] = [2.8]
    a = mo.run(_framed(m), workers=1)
    b = mo.run(_framed(m, [{"q_ship_to_icrf": _quat_z(90.0)}]), workers=1)
    for sa, sb in zip(a["sectors"], b["sectors"]):
        assert sa["status"] == sb["status"] == "OK"
        assert sa["wf"]["freq_ghz"] == sb["wf"]["freq_ghz"]
        for k, fa in sa["faces"].items():
            fb = sb["faces"][str((int(k) - 1) % 4)]
            assert fb["positions"] == pytest.approx(fa["positions"], rel=1e-3)
            assert fb["energy_fraction"] == pytest.approx(fa["energy_fraction"], rel=1e-2)


@pytest.mark.parametrize("glob,sec,msg", [
    ({"nav": [{"q_ship_to_icrf": [1, 0, 0, 0]}, {"q_ship_to_icrf": [1, 0, 0, 0]}]}, {}, "one only"),
    ({"nav": [{"q_ship_to_icrf": [1.01, 0, 0, 0]}]}, {}, "not a unit quaternion"),
    ({"nav": [{"q_ship_to_icrf": [1, 0, 0, 0], "epoch": "tomorrow"}]}, {}, "ISO 8601"),
    ({"nav": [{"q_ship_to_icrf": [1, 0, 0, 0], "central_body": "earth", "r_km": [7000.0, 0, 0]}]}, {}, "together"),
    ({"nav": [{"q_ship_to_icrf": [1, 0, 0, 0], "central_body": "earth", "r_km": [7000.0, 0, 0],
               "v_kms": [7.0, 0, 0]}]}, {}, "no orbit plane"),
    ({}, {"frame": "rtn"}, "needs the ship orbit"),
    ({}, {"frame": "limb"}, "needs the ship orbit"),
    ({}, {"frame": "galactic"}, "unknown frame"),
    ({}, {"frame": "two_dir", "frame_p_ra_deg": 10.0, "frame_p_dec_deg": 0.0, "frame_s_ra_deg": 10.5,
          "frame_s_dec_deg": 0.0}, "from aligned"),
    ({}, {"frame": "two_dir"}, "needs frame_p_ra_deg"),
])
def test_invalid_frames_are_refused(glob, sec, msg):
    m = md.empty_mission()
    m["global"].update(glob)
    m["sectors"] = [{**md.new_sector([], "box"), **sec}]
    errs = md.validate_mission(m)
    assert any(msg in e for e in errs), errs
    with pytest.raises(ValueError):
        mo.run(m, workers=1)


def test_sky_inconsistent_with_the_navigation_is_flagged():
    from core import frames
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_frames.json"))
    assert frames.resolve(m)[1] == []
    m["global"]["sky"]["bodies"][0]["phi_deg"] = 170.0
    assert any("EARTH" in a for a in frames.resolve(m)[1])


# ----------------------------------------------------------------------------- confirmation
def test_confirmation_probability_grows_with_margin():
    for sw in (1, 3):
        succ, _ = trk.confirm_tables(sw, 2.8)["KU1+7"]
        assert all(b >= a - 1e-12 for a, b in zip(succ, succ[1:]))


# ----------------------------------------------------------------------------- Ku confirmation rebuilt (step 3.2)
def test_ku_centres_follow_the_band_and_the_exclusions():
    """A2, A5-A7: centres usable_lo + k x 10 MHz with the narrowest Ku signal inside; the coarse 50 MHz step plus the
    centre-most centre of each free fragment; forced centres checked."""
    grid = trk.ku_centre_grid()
    assert grid[0] == rm.BAND_KU.usable_lo_ghz and all(abs((b - a) - 0.010) < 1e-9 for a, b in zip(grid, grid[1:]))
    free = trk.ku_centres({})
    assert free and all(mo.signal_fits(f, min(trk.KU_BW), (), rm.BAND_KU) for f in free)
    # A8 (25 October 2026): a quiet Ku spectrum evaluates ku_freq_quiet_centres centres in the core band
    assert len(free) == trk.KU_FREQ_QUIET_CENTRES and all(rm.BAND_KU.core_lo_ghz <= f <= rm.BAND_KU.core_hi_ghz
                                                          for f in free)
    cut = trk.ku_centres({"ku_spectrum_exclusions_ghz": [[13.8, 14.2]]})
    assert all(not (13.8 - min(trk.KU_BW) / 2e3 < f < 14.2 + min(trk.KU_BW) / 2e3) for f in cut)
    assert any(f < 13.8 for f in cut) and any(f > 14.2 for f in cut)
    assert trk.ku_centres({"ku_freq_ghz_candidates": [14.1]}) == [14.1]
    assert trk.validate_ku_frequencies({"ku_freq_ghz_candidates": [2.8]})
    assert not trk.validate_ku_frequencies({"ku_freq_ghz_candidates": [14.0]})


def test_ku_cluster_superposition_loss_is_interpolated():
    """C2: 7 beams between the 4- and 8-beam entries of [multibeam], in log2 of the count (0.76 dB); the search table
    keeps its own entries."""
    assert trk.superposition_loss_db(7) == pytest.approx(0.6 + 0.2 * (math.log2(7) - 2.0), abs=1e-12)
    assert 7 not in rm.MB_SUP_LOSS_DB
    names = [c.name for c in trk.cluster_variants()]
    assert names[0] == "sequential" and any(n.startswith("superposition") for n in names) \
        and any(n.startswith("partition") for n in names)
    seq = trk.cluster_variants()[0]
    assert seq.looks == trk.CLUSTER_N and all(c.looks == 1.0 for c in trk.cluster_variants()[1:])


def test_ku_scan_factor_is_common_to_the_waveforms():
    """The Ku range at a scan angle is the broadside range x h(f, theta), the same for every waveform (exact)."""
    h = trk._ku_scan(14.0, 40.0)
    assert h is not None and 0.0 < h[0] < 1.0 and 0.0 < h[1] < 1.0
    w = rm.Waveform("x", "CONFIRM", "Ku", 50e-6, 4, 1e-3, 50.0, 1, "COHERENT")
    tgt = rm.TARGETS["KKV"]
    assert rm.solve(w, 14.0, 40.0, tgt).range_km == pytest.approx(rm.solve(w, 14.0, 0.0, tgt).range_km * h[0], rel=1e-9)


def test_ku_look_is_shortened_to_its_need():
    """B5: the chosen look just reaches the range: pulse shortened at saturated PAs (scale <= 1, floor tau B >= 1),
    its energy = P_pk x n x tau' / PAE of the shortened pulse; its reach is at least the range asked."""
    look = trk.ku_look(30.0, rm.TARGETS["KKV"], 500.0, rm.face_elec_avail_w(2.0), f_ghz=14.0)
    assert look is not None and look["reach_km"] >= 500.0 and 0.0 < look["scale"] <= 1.0
    w = look["wf"]
    tau = w.tau_s * max(look["scale"], min(1.0, rm.PULSE_TB_MIN / (w.sig_bw_mhz * 1e6) / w.tau_s))
    assert look["time_s"] == pytest.approx(w.n * tau, rel=1e-12)
    e2, t2, _ = trk.look_cost(look, 2.0)
    assert e2 >= look["energy_J"] and t2 >= look["time_s"]


def test_confirmation_margin_is_solved_continuously():
    """B6: the margin found is the smallest reaching p_min within the tolerance (success grows with the margin)."""
    tgt = rm.TARGETS["KKV"]
    fl = rm.look_fluctuation(tgt, 14.0, tgt.swerling, 1, 0.0)
    st = trk.ConfirmStats(fl, fl, 2.8, 14.0, None, trk.cluster_variants()[0])
    m = st.min_margin(lambda x: st.ku("KU1+7", x)[0])
    assert m is not None and st.ku("KU1+7", m)[0] >= trk.CONFIRM_P_MIN
    if m > 0:
        assert st.ku("KU1+7", max(0.0, m - 2 * trk.MARGIN_TOL_DB))[0] < trk.CONFIRM_P_MIN


def test_single_look_pd_uses_the_exact_law():
    """E4: the confirmation statistics use the exact detection laws; SW1 matches its closed form."""
    t = -math.log(rm.DETECT_PFA)
    snr = np.array([1.0, 10.0, 100.0])
    assert np.allclose(trk.pd_single(snr, 1), np.exp(-t / (1.0 + snr)), atol=5e-3)
    q = trk.detected_snr_quantiles(10.0, 1)
    u = (np.arange(trk.N_SNR_Q) + 0.5) / trk.N_SNR_Q
    assert np.allclose(q, -10.0 * np.log(rm.DETECT_PD * (1.0 - u)), rtol=2e-3)     # SW1 closed form (exponential)
    assert np.all(np.diff(q) > 0)


def test_s2_touch_is_an_s_confirm_waveform_and_promote_is_gone():
    """D1-D2: CONFIRM is allowed in S (the S2 touch); PROMOTE is not a role any more."""
    rm.Waveform("t", "CONFIRM", "S", 100e-6, 1, 1e-3, 75.0, 1, "COHERENT")
    with pytest.raises(ValueError):
        rm.Waveform("p", "PROMOTE", "Ku", 100e-6, 1, 1e-3, 50.0, 1, "COHERENT")
    assert "PROMOTE" not in rm.DETECT_ROLES


def test_harmonic_cover_of_the_ku_channel():
    """A3: the share of the Ku channel the S harmonics cover is reported (a scheduler time rule, not a frequency rule)."""
    r = {"wf": {"name": "W", "freq_ghz": 2.8, "sig_bw_mhz": 100.0}}
    cov = trk._harmonic_cover(r, 14.0, 50.0)
    assert cov and cov[0][1] == pytest.approx(1.0)
    assert not trk._harmonic_cover(r, 13.55, 50.0)


# ----------------------------------------------------------------------------- exact speed-ups (step 3.2, Q6)
def test_optimiser_solve_path_is_bit_identical_to_solve():
    """E1: solve_base (cached) + solve_look, without the diagnostics, gives the optimiser exactly what solve() gives,
    over pulse trains, integrations, TX modes, Doppler windows, Pfas, criteria, jammers, lattices and the Ku looks."""
    tgt = dataclasses.replace(rm.TARGETS["KKV"], name="cache")
    wfs = [(mo.make_wf("a", 100e-6, 1, 1e-3, 50.0, 1, "NONCOHERENT"), 2.8),
           (mo.make_wf("b", 200e-6, 8, 4e-3, 100.0, 1, "COHERENT", window=-30.0), 2.75),
           (mo.make_wf("c", 50e-6, 6, 2e-3, 25.0, 1, "NONCOHERENT", agility=60.0), 2.8),
           (mo.make_wf("d", 100e-6, 8, 0.5e-3, 50.0, 1, "COHERENT", staggered=True), 2.8),
           (tb.KU_TRACK, 14.0), (rm.Waveform("k", "CONFIRM", "Ku", 50e-6, 4, 1e-3, 50.0, 1, "COHERENT"), 14.1)]
    wfs += [(mo.make_wf("m", 100e-6, 4, 2e-3, 50.0, b, "COHERENT", mode=mode, cut=cut), 2.8)
            for mode, b, cut in mo.tx_variants() if mode == "partition"]
    looks = [(None, None, None, None, None, None), (-30.0, 1e-7, (5000.0, 10.0, 800.0), None, 1.6, 3000.0),
             (-35.0, 3e-8, (3000.0, 5.0, 400.0), 1e-9, 1.3, 0.0)]
    for w, f in wfs:
        for sll, pfa, cum, spj, sp, v_span in looks:
            sll = rm.RX_SLL_DEFAULT_DB if sll is None else sll
            p = mo._parts_cached(w, f, 0.0, tgt, sll, v_span, pfa, cum, spj, sp)
            if p is None:
                with pytest.raises(ValueError):
                    rm.solve(w, f, 0.0, tgt, rm.FACE_ELEC_POWER_MW, sll, v_span_mps=v_span)
                continue
            r = rm.solve(w, f, 0.0, tgt, rm.FACE_ELEC_POWER_MW, sll, v_span_mps=v_span, pfa=pfa, cum=cum,
                         spj_density_w_hz=spj, grid_spacing_hpbw=sp)
            assert p[:4] == (r.range_noise_km, r.burn_through_km, r.snr_req_db + r.losses["positions"], r.l_tot_db)
            assert mo._LAW_LIST[p[4]] == r.look_law
            assert list(r.losses) == list(mo._base_cached(w, f, 0.0, tgt, sll, v_span).losses)


def test_s2_gains_kept_per_touch_grid_are_exact():
    """E2: s2_vec with the beam gains kept per touch grid equals the direct computation (bit-identical), on a cache
    miss and on a hit."""
    tgt = rm.TARGETS["KKV"]
    fl = rm.look_fluctuation(tgt, 14.0, tgt.swerling, 1, 0.0)
    for cl in trk.cluster_variants()[:2]:
        st = trk.ConfirmStats(fl, fl, 2.8, 14.0, 1.6, cl)
        mt = np.array([0.0, 1.5, 4.0])
        for mk in (np.array([0.5, 2.0, 3.0]), np.array([6.0, 0.0, 1.0])):     # second pass: a hit
            got = st.s2_vec(mt, mk)
            pt, _, _ = st._memo[("touch", tuple(np.round(mt, 6)))]
            sig2 = trk.K_ML_S * st.q / np.sqrt(st.x_det[:, None] + (st.touch_snr0 * 10.0 ** (mt / 10.0))[None, :])
            snr = (st.snr_ku * 10.0 ** (mk / 10.0))[None, :, None, None]
            ex = st.bias_ku + sig2[..., None, None] * trk.GH_NODES[:, None]
            ey = st.bias_ku + sig2[..., None, None] * trk.GH_NODES[None, :]
            p1 = trk.pd_single(trk._tx_gain(ex, ey) * snr, fl)
            miss = 1.0
            for cx, cy, ku, kv in cl.beams:
                miss = miss * (1.0 - trk.pd_single(trk._tx_gain(ex - cx, ey - cy, ku, kv) * cl.eirp * snr, fl))
            w = trk.GH_WEIGHTS[:, None] * trk.GH_WEIGHTS[None, :]
            s17 = np.tensordot(p1 + (1.0 - p1) * (1.0 - miss), w, axes=([2, 3], [0, 1]))
            l17 = np.tensordot(1.0 + (1.0 - p1) * cl.looks, w, axes=([2, 3], [0, 1]))
            assert np.array_equal(got[0], (pt * s17).mean(axis=0)) and np.array_equal(got[1], (pt * l17).mean(axis=0))


def test_single_look_pd_by_index_matches_interp():
    """E3: pd_single interpolates by index on the regular grid: np.interp to rounding, flat beyond the grid ends."""
    for fl in (1, 3, rm.look_fluctuation(rm.TARGETS["KKV"], 14.0, None, 1, 0.0)):
        x = np.concatenate([np.linspace(-45.0, 65.0, 20001), rm.detection.SNR_GRID_DB])
        ref = rm.detection.pd_at(1, fl, rm.DETECT_PFA, x)
        assert np.max(np.abs(trk.pd_single(10.0 ** (x / 10.0), fl) - ref)) < 1e-12


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
    import core.montecarlo  # noqa: F401  (step 5.4, command line only until M4)
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


def test_face_power_above_the_feeder_is_refused():
    """Step 4.1: a face cap above the face feeder is refused (the ship total is shared, judged in the budget)."""
    errs = md.validate_global({**md.GLOBAL_DEFAULTS, "face_elec_mw": md.FACE_FEEDER_MAX_MW + 0.1})
    assert any("feeder" in e for e in errs)


def test_gui_modules_import():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import gui.main_window  # noqa: F401
    import gui.prep_tab  # noqa: F401
    import gui.results_tab  # noqa: F401


# ----------------------------------------------------------------------------- track model (step 3.3)
from core import frames as fr_  # noqa: E402
from core import trackmodel as tm  # noqa: E402


def test_singer_model_is_consistent():
    """A1: the discretised Singer model: F of a constant-acceleration model for a long time constant, Q symmetric and
    positive, the steady filtered covariance below the predicted one."""
    f, q = tm.singer(2.0, 10.0, 1e6)
    assert np.allclose(f, [[1, 2, 2], [0, 1, 2], [0, 0, 1]], atol=1e-5)
    assert np.allclose(q, q.T) and np.all(np.linalg.eigvalsh(q) > -1e-12)
    p, pf = tm.steady(f, q, np.array([[1.0, 0.0, 0.0]]), np.array([[25.0]]))
    assert np.all(np.diag(pf) <= np.diag(p) + 1e-12) and pf[0, 0] < 25.0


def test_allowed_measurement_meets_the_requirement_and_shrinks_with_the_period():
    """A7: the largest measurement error meets the predicted-angle requirement exactly; a longer period or a
    manoeuvring class allows less."""
    sm, tau = tm.manoeuvre(rm.TARGETS["KKV"])
    zone = math.radians(0.2 / 4.0)
    a1 = tm.allowed_measurement(1.0, sm, tau, 5e5, zone, 10.0, 5.0)
    a2 = tm.allowed_measurement(2.0, sm, tau, 5e5, zone, 10.0, 5.0)
    assert 0 < a2["theta"] < a1["theta"] and 0 < a2["vr"] < a1["vr"]
    f, q = tm.singer(2.0, sm, tau)
    p, _ = tm.steady(f, q, np.array([[1.0, 0.0, 0.0]]), np.array([[(a2["theta"] * 5e5) ** 2]]))
    assert math.sqrt(p[0, 0]) == pytest.approx(zone * 5e5, rel=1e-3)
    calm = tm.allowed_measurement(2.0, *tm.manoeuvre(rm.TARGETS["DEBRIS"]), 5e5, zone, 10.0, 5.0)
    assert calm["theta"] > a2["theta"]


def test_track_state_floors_and_feasibility():
    """E4-E6: a requirement tighter than the measurement floor cannot be held; a shorter period helps."""
    t = rm.TARGETS["KKV"]
    ok = tm.TrackState(t, 2.0, 500.0, 0.2, 10.0, 5.0, 14.0, 0.0)
    assert ok.feasible and ok.pack().sigma_theta_deg > 0 and 0 < ok.pointing_hpbw < 1
    assert ok.a_bank_mps2 <= t.accel_max_mps2 + 1e-12 and ok.v_span_mps > 0
    tight = tm.TrackState(t, 2.0, 500.0, 1e-5, 10.0, 5.0, 14.0, 0.0)
    assert not tight.feasible
    assert tm.TrackState(t, 0.05, 500.0, 0.02, 10.0, 5.0, 14.0, 0.0).allowed["theta"] > \
        tm.TrackState(t, 2.0, 500.0, 0.02, 10.0, 5.0, 14.0, 0.0).allowed["theta"]
    assert [ok.coast_pointing(k) for k in range(3)] == sorted(ok.coast_pointing(k) for k in range(3))


def test_pointing_law_and_cluster_cell():
    """B1 / B4: the pointing error costs more as it grows; the cluster cell law is a fixed loss."""
    w = rm.Waveform("k", "CONFIRM", "Ku", 50e-6, 4, 1e-3, 50.0, 1, "COHERENT")
    r = [rm.solve(w, 14.0, 0.0, rm.TARGETS["KKV"], pointing=pt).range_km
         for pt in (None, ("gauss", 0.05), ("gauss", 0.2), ("gauss", 0.4))]
    assert r == sorted(r, reverse=True)
    c = rm.solve(w, 14.0, 0.0, rm.TARGETS["KKV"], pointing=("cluster", rm.BEAM_SPACING_HPBW))
    assert "pointing_tx" in c.pulse["position_losses_mean_db"] and "grid_rx" in c.pulse["position_losses_mean_db"]
    assert rm.solve(w, 14.0, 0.0, rm.TARGETS["KKV"], pd=0.5).range_km > r[0]


def test_track_look_banks_and_coupling_residual():
    """B3 / E1 / E2: a TRACK look covers the velocity span and the acceleration it is given; the range accuracy pays
    the range-Doppler coupling residual of long pulses."""
    pk = rm.TrackingPack(0.05, 5.0, 1.0)
    w = rm.Waveform("t", "TRACK", "Ku", 100e-6, 8, 1e-3, 50.0, 1, "COHERENT", tracking_pack=pk)
    calm = dataclasses.replace(rm.TARGETS["KKV"], accel_max_mps2=0.0)
    r0 = rm.solve(w, 14.0, 0.0, calm).pulse
    r1 = rm.solve(w, 14.0, 0.0, calm, v_span_mps=500.0).pulse
    assert r0["velocity_hypotheses"] <= r1["velocity_hypotheses"]
    a1 = rm.solve(w, 14.0, 0.0, rm.TARGETS["KKV"]).pulse
    assert a1["acceleration_max_mps2"] == rm.TARGETS["KKV"].accel_max_mps2
    s0, _ = rm.tracking_snr_req(pk, 14.0, 0.0, 8e-3, 50.0, rm.RANGE_CRLB_KR, 0.0)
    s1, _ = rm.tracking_snr_req(pk, 14.0, 0.0, 8e-3, 50.0, rm.RANGE_CRLB_KR, 1e-3)
    assert s1 >= s0


def test_attenuator_steps():
    """E3: attenuation on its steps, None beyond the largest; the attenuated unsaturated range."""
    r_sat = rm.min_unsaturated_range_km(14.0, 0.0, 1.0)
    assert rm.attenuation_needed_db(14.0, 0.0, 1.0, 2 * r_sat) == 0.0
    assert rm.attenuation_needed_db(14.0, 0.0, 1.0, r_sat / 1.5) == rm.ATTENUATOR_STEP_DB["Ku"]
    assert rm.attenuation_needed_db(14.0, 0.0, 1.0, r_sat / 100.0) is None
    assert rm.min_unsaturated_range_attenuated_km(14.0, 0.0, 1.0) == pytest.approx(
        r_sat / 10 ** (rm.ATTENUATOR_MAX_DB["Ku"] / 40.0))


def test_track_loss_chain():
    """B7: no loss with perfect looks; with a constant Pd the loss per update follows the renewal chain."""
    assert tm.loss_per_update([1.0, 1.0, 1.0])[0] == 0.0
    pd = 0.9
    p, _ = tm.loss_per_update([pd] * 3)
    pi = np.array([1.0, 1 - pd, (1 - pd) ** 2])
    assert p == pytest.approx(pi[-1] / pi.sum() * (1 - pd))
    assert tm.loss_per_update([0.9, 0.7, 0.5])[0] > p


def test_initiation_converges():
    """C1-C3: a track settles in a finite number of looks, more for a manoeuvring class at a tight zone."""
    t = rm.TARGETS["KKV"]
    st = tm.TrackState(t, 2.0, 500.0, 0.2, 10.0, 5.0, 14.0, 0.0)
    n, sig = tm.initiation(st, t, 1e-4, 10.0, 5.0, 0.2, 10.0, 5.0)
    assert 1 <= n < 200 and sig > 0


def test_discrimination_indices():
    """F1-F5: KKV and decoy are told apart; more looks never hurt; Faraday uncertainty weakens the depolarisation
    index unless the TX polarisation is switched."""
    a = rm.TARGETS["KKV"]
    b = dataclasses.replace(rm.TARGETS["DECOY"], depol_ku_db=rm.TARGETS["KKV"].depol_ku_db + 6.0)
    d = tm.discrimination(a, b, 14.0, 20.0, 0.0, 5.0)
    assert d["looks"] is not None and d["looks"] >= 1
    p = [tm.p_correct(tm.indices(a, b, 14.0, 20.0, n, 0.0, False, 1e9)) for n in (1, 5, 20)]
    assert p == sorted(p)
    plain = tm.indices(a, b, 14.0, 20.0, 10, 0.0, False, 1e9)["depolarisation"]
    far = tm.indices(a, b, 14.0, 20.0, 10, math.radians(90.0), False, 1e9)["depolarisation"]
    sw = tm.indices(a, b, 14.0, 20.0, 10, math.radians(90.0), True, 1e9)["depolarisation"]
    assert far < plain and sw > far
    sep = tm.separability(14.0, 20.0)
    assert all(0.5 <= v <= 1.0 for row in sep.values() for v in row.values())


def test_aspect_table_and_track_frame():
    """F6: an RCS(aspect) table scales the RCS at the class offset; F8: a 'track' sector becomes a cap of k sigma
    around its designation."""
    # step 5.1: the table is 7 elevations x 12 azimuths; frontal policy at the offset (az 45 deg, el 0: between the
    # 30 and 60 deg columns of the el 0 row, linear in dB), the optics outside their cone are not seen
    row = [0.0, -6.0, -12.0] + [-12.0] * 9
    tab = tuple(tuple(row) if e == 0 else (0.0,) * 12 for e in rm.ASPECT_EL_DEG)
    t = dataclasses.replace(rm.TARGETS["KKV"], aspect_rcs_db=tab, aspect_offset_deg=45.0, aspect_policy="frontal")
    assert t.aspect_factor() == pytest.approx(10 ** -0.9)
    assert t.optics_rcs_m2(14.0) == 0.0 and t.rcs_at(14.0) < rm.TARGETS["KKV"].rcs_at(14.0)
    m = md.empty_mission()
    m["sectors"] = [{**md.new_sector([], "cap"), "frame": "track", "track_ra_deg": 40.0, "track_dec_deg": 10.0,
                     "track_sigma_deg": 0.5}]
    assert md.validate_mission(m) == []
    s = fr_.resolve(m)[0]["sectors"][0]
    assert (s["phi0_deg"], s["eps0_deg"]) == (40.0, 10.0) and s["radius_deg"] == pytest.approx(tm.TRACK_FRAME_K * 0.5)
    m["sectors"][0]["shape"] = "box"
    assert any("takes a cap" in e for e in md.validate_mission(m))


def test_track_model_mission_keys():
    """Sector keys of step 3.3 are checked."""
    m = md.empty_mission()
    m["sectors"] = [{**md.new_sector([]), "discriminate_against": "NOPE", "jammer_deceptive": "yes",
                     "maneuver_tau_s": -1}]
    errs = md.validate_mission(m)
    assert any("discriminate_against" in e for e in errs) and any("jammer_deceptive" in e for e in errs) \
        and any("maneuver_tau_s" in e for e in errs)


def test_start_bracket():
    """C6: tracks start in the bracket covering the confirmation range, else in the farthest one."""
    bs = [{"r_min_km": 0.0, "r_max_km": 300.0}, {"r_min_km": 300.0, "r_max_km": 800.0}]
    assert trk._start_bracket(bs, 500.0) == 1 and trk._start_bracket(bs, 2000.0) == 1
    assert trk._start_bracket([], 500.0) is None


# ----------------------------------------------------------------------------- array errors and calibration (step 3.4)
def test_array_error_loss_replaces_the_lumped_calibration():
    """B4: the 'calibration' loss of each band is the Ruze loss of the residual errors, TX and RX; no error, no loss."""
    zero = {k: 0.0 for k in ("trm_phase_deg", "trm_amp_db", "trsa_phase_deg", "trsa_amp_db", "drift_phase_deg_min",
                             "drift_amp_db_min")}
    zero.update(cal_interval_s=300.0, cal_duration_s=0.01)
    assert rm.array_error_loss_db(zero) == 0.0
    ph = dict(zero, trm_phase_deg=10.0)
    assert rm.array_error_loss_db(ph) == pytest.approx(10 * math.log10(math.e) * math.radians(10.0) ** 2)
    for b in ("S", "Ku"):
        band = rm.BANDS[b]
        assert band.losses_db["calibration"] == pytest.approx(2.0 * rm.array_error_loss_db(band.errors))
        assert 0.0 < band.losses_db["calibration"] < 0.1
    longer = dict(rm.BAND_S.errors, cal_interval_s=3000.0)
    assert rm.array_error_loss_db(longer) > rm.array_error_loss_db(rm.BAND_S.errors)


def test_array_error_figures_and_checks(monkeypatch):
    """B5-B6: sidelobe floor well under the RX options, boresight under the angle floor; the checks fire otherwise."""
    for b in ("S", "Ku"):
        a = rm.array_errors(b)
        assert a["sidelobe_floor_db"] < min(rm.RX_SLL_OPTIONS_DB) - 10.0
        assert a["boresight_mrad"] < params.P.get("tracking.angle_floor_mrad")
        assert a["cal_time_frac"] == pytest.approx(rm.BANDS[b].errors["cal_duration_s"] / rm.BANDS[b].errors["cal_interval_s"])
    assert rm.calibration_checks() == []
    bad = {"loss_db": 0.0, "sidelobe_floor_db": -30.0, "boresight_mrad": 2.0, "cal_time_frac": 0.0}
    monkeypatch.setattr(rm, "array_errors", lambda b: bad)
    msgs = rm.calibration_checks()
    assert any("sidelobe floor" in m for m in msgs) and any("angle_floor" in m for m in msgs) \
        and any("bias_mrad" in m for m in msgs)


# ----------------------------------------------------------------------------- electrical power (step 4.1)
def test_always_on_consumption_and_bus_draw():
    """B1-B6: two ADCs and two receive chains per TRM in both bands, beamforming and control; the bus draw adds the
    pumps and goes through the PSU and the distribution."""
    n = 2 * (rm.N_TRM["S"] * (0.1 + 0.5) + rm.N_TRM["Ku"] * (0.1 + 0.1))
    assert rm.non_tx_power_w() == pytest.approx(n + 52e3)
    assert rm.face_bus_draw_w(1e6, 2e5) == pytest.approx((1e6 + rm.non_tx_power_w() + rm.PUMP_FRAC * 2e5) / rm.CHAIN_EFF)


def test_trm_supply_removes_waveforms():
    """C1-C2: a train beyond the TRM supply (mean power or buffer recharge) is refused, a single pulse never."""
    w = rm.Waveform("k", "CONFIRM", "Ku", 1e-3, 8, 2e-3, 50.0, 1, "COHERENT")
    rm.trm_supply_check(w, rm.BAND_KU)
    weak = dataclasses.replace(rm.BAND_KU, trm_supply_w=10.0)
    with pytest.raises(ValueError):
        rm.trm_supply_check(w, weak)
    rm.trm_supply_check(dataclasses.replace(w, n=1), weak)


def test_face_power_caps_in_the_mission():
    """A: face_elec_mw is an optional cap per face (number, list of 4, 0 = none), each at most the feeder."""
    for cap, ok in ((0.0, True), (3.0, True), ([1.0, 2.0, 3.0, 4.0], True), ([1.0, 2.0], False),
                    (md.FACE_FEEDER_MAX_MW + 1.0, False), (-1.0, False)):
        errs = md.validate_global({**md.GLOBAL_DEFAULTS, "face_elec_mw": cap})
        assert (not any("face_elec_mw" in e for e in errs)) == ok, (cap, errs)


# ----------------------------------------------------------------------------- thermal (step 4.2)
def test_cooling_figures_derive_from_the_coolant():
    """C1-C3: flux = h x area factor x dT, capacity = flux x emissive area, flow for the rise, inertia and buffer."""
    c = rm.face_cooling()
    flux = rm.TH_H * rm.TH_AREA_FACTOR * rm.TH_DT_MAX
    assert c["flux_max_w_cm2"] == pytest.approx(flux / 1e4) and c["capacity_w"] == pytest.approx(flux * rm.PHYS_AREA_M2)
    assert c["flow_l_s"] == pytest.approx(c["capacity_w"] / (rm.TH_CP * rm.TH_RISE) / rm.TH_RHO)
    assert c["buffer_j"] == pytest.approx(c["inertia_j_k"] * params.P.get("thermal.plate_rise_max_k"))
    assert rm.r_conv_trm("S") == pytest.approx(1 / (rm.TH_H * rm.TH_AREA_FACTOR * rm.PHYS_AREA_M2 / rm.N_TRM["S"]))


def test_junction_temperature_model():
    """B1-B6: the junction warms with the duty and the train, stays under Tj max over the search spaces (measured:
    the check is off, duty_max kept); with a weak cooling the check removes the waveform."""
    a = rm.Waveform("a", "SEARCH", "S", 1e-4, 1, 1e-3, 10.0, 1, "COHERENT")
    b = rm.Waveform("b", "SEARCH", "S", 2e-3, 32, 2e-3 / 0.65, 10.0, 1, "COHERENT")
    assert rm.TH_INLET_C + rm.TH_RISE < rm.junction_temp_c(a) < rm.junction_temp_c(b) < rm.TJ_MAX_C
    hot = dataclasses.replace(rm.BAND_S, rth=((2.0, 1e-5), (3.0, 1e-3)))
    with pytest.raises(ValueError):
        rm.junction_check(b, hot)
    assert params.P.get("tool.thermal.junction_check") is False


# ----------------------------------------------------------------------------- scheduler (step 4.3)
from core import scheduler as sch  # noqa: E402


def test_priorities_and_latency_requirement():
    """Priorities of the search volumes (P1 when self-protection, else P4_n) and the latency requirement."""
    assert sch.priority_of({"self_protection": True, "search_priority": 3}) == (1, 0)
    assert sch.priority_of({"search_priority": 3}) == (4, 3) and sch.priority_of({}) == (4, 1)
    sec = {"range_km": 1000.0, "v_close_max_mps": 10000.0}
    assert sch.latency_requirement_s(sec) == pytest.approx(sch.LATENCY_FRAC * 100.0)
    assert sch.latency_requirement_s(dict(sec, latency_max_s=0.5)) == 0.5


def test_priority_queue_and_shedding():
    """Cobham waits grow with the priority rank; shedding stretches P4 (lowest sub-priority first), then P3, and
    declares the mission infeasible when P0-P2 alone exceed the time."""
    cl = [sch.Klass((0, 0), "cal", 0.01, 0.01, "cal"), sch.Klass((2, 0), "conf", 0.2, 0.002, "confirm"),
          sch.Klass((3, 0), "upd", 0.3, 0.002, "update"), sch.Klass((4, 1), "s1", 0.3, 0.001, "search"),
          sch.Klass((4, 2), "s2", 0.4, 0.001, "search")]
    w = sch._cobham(cl)
    assert w["cal"] < w["conf"] < w["upd"] < w["s1"] < w["s2"]
    st, ok = sch._shed(cl, 1.0)
    assert ok and st["s2"] > 1.0 and st["s1"] == 1.0 and st["upd"] == 1.0
    _, ok2 = sch._shed([sch.Klass((2, 0), "c", 1.2, 0.01, "confirm")], 1.0)
    assert not ok2
    assert sch._q95(0.0, 0.01) == 0.0 and sch._q95(0.1, 0.5) > 0.1


def test_synthetic_timeline_serves_priorities():
    """The greedy timeline serves the calibration at once and keeps the updates on time when the face is light."""
    import random
    cl = [sch.Klass((0, 0), "cal", 1e-4, 0.01, "cal", period_s=300.0),
          sch.Klass((3, 0), "upd", 0.01, 0.001, "update", "A", 1.0, 10.0),
          sch.Klass((4, 1), "s", 0.2, 0.002, "search", "A", 10.0, 1000.0)]
    tl = sch._timeline(cl, 10.0, random.Random(1), math.inf, 0.0)
    assert tl["calibration_delay_s"] <= 0.002 + 1e-12 and tl["late_update_frac"] == 0.0 and tl["recharge_pauses"] == 0
    tl2 = sch._timeline(cl, 10.0, random.Random(1), 0.005, 1.0)
    assert tl2["recharge_pauses"] > 0


def test_scheduler_mission_keys():
    m = md.empty_mission()
    m["sectors"] = [{**md.new_sector([]), "self_protection": "yes", "search_priority": 0, "latency_max_s": -1}]
    errs = md.validate_mission(m)
    assert any("self_protection" in e for e in errs) and any("search_priority" in e for e in errs)         and any("latency_max_s" in e for e in errs)


# ----------------------------------------------------------------------------- scan order (step 4.3 bis)
from core import scanorder as so  # noqa: E402


def test_bit_reversed_morton_is_prefix_uniform():
    """V1: on a square grid of 32 x 32 points, the first 4^k pointings fall one in each of the 4^k sub-squares."""
    g = np.array([(i, j) for i in range(32) for j in range(32)], dtype=float)
    order = so.order_bitrev_morton(g)
    assert sorted(order.tolist()) == list(range(len(g)))
    for k in (1, 2, 3):
        n = 4 ** k
        cells = {(int(g[i, 0] * 2 ** k // 32), int(g[i, 1] * 2 ** k // 32)) for i in order[:n]}
        assert len(cells) == n
    u = so.uniformity(g, order)
    assert all(v == pytest.approx(1.0) for v in u.values())


def test_lattice_shifts_and_counts():
    """V2: the shifted lattices of a uniform region have about the same number of points, area / cell area."""
    uu, vv = np.meshgrid(np.linspace(-0.3, 0.3, 301), np.linspace(-0.3, 0.3, 301), indexing="ij")
    sel = np.ones(uu.shape, bool)
    d = 0.02
    n = [len(so.lattice(sel, uu, vv, d, s_)) for s_ in range(so.N_SHIFTS)]
    expect = 0.6 * 0.6 / (math.sqrt(3) / 2 * d * d)
    assert all(abs(x - expect) / expect < 0.05 for x in n)
    p0, p1 = so.lattice(sel, uu, vv, d, 0), so.lattice(sel, uu, vv, d, 1)
    assert not np.allclose(p0[:5], p1[:5])



# ----------------------------------------------------------------------------- compute and data (step 4.4)
from core import compute as cmp  # noqa: E402


def test_compute_look_cost_by_hand():
    """B1-B4: one S pulse of 1 ms PRI at 100 MHz, one beam, no bank: each function counted from the file's values."""
    wf = {"n_pulses": 1, "pri_ms": 1.0, "sig_bw_mhz": 100.0, "velocity_hypotheses": 1, "acceleration_hypotheses": 1,
          "coherent_blocks": 1, "pulses_per_block": 1}
    c = cmp.look_cost("S", wf, 1.0)
    fs = cmp.ADC_OVERSAMPLING["S"] * 100e6
    ch = 2 * rm.N_TRM["S"]
    assert c["listen_s"] == pytest.approx(1e-3)
    sub = 2 * rm.N_TRSA if cmp.TRANSPORT["S"] == "subarray_beams" else 0
    assert c["ops"]["beamforming"] == pytest.approx(cmp.OPS_CMAC * (ch + sub) * fs * 1e-3)
    n_p = fs * 1e-3
    assert c["ops"]["compression"] == pytest.approx(cmp.FFT_K * 2 * n_p * math.log2(n_p))
    c_r = cmp.RANGE_OSR * 100e6 * 1e-3
    assert c["ops"]["banks"] == pytest.approx(cmp.OPS_CMAC * 2 * c_r)
    assert c["ops"]["cfar"] == pytest.approx(cmp.CFAR_OPS * c_r)
    assert c["ops"]["nulling"] == 0.0
    words = 2 * rm.N_TRSA if sub else ch
    assert c["bits"] == pytest.approx(words * fs * 1e-3 * 2 * cmp.IQ_BITS)
    # the banks scale with the hypotheses, the nulling appears with a jammer
    c2 = cmp.look_cost("S", {**wf, "velocity_hypotheses": 10, "acceleration_hypotheses": 3}, 1.0, nulled=1)
    assert c2["ops"]["banks"] == pytest.approx(30 * c["ops"]["banks"])
    assert c2["ops"]["nulling"] > 0.0


def test_compute_budget_invariants(runs):
    """C4, D1, F2: power = static + operations + transport, capacity from the ceiling, the scheduler's latency holds
    the processing latency; peak beams >= mean beams."""
    for n, res in runs.items():
        c = res["compute"]
        for k, f in c["faces"].items():
            p = f["power_kW"]
            assert p["total"] == pytest.approx(p["static"] + p["ops"] + p["transport"])
            assert p["static"] == pytest.approx(cmp.STATIC_FRAC * cmp.CEILING_W / 1e3)
            assert p["ops"] == pytest.approx(cmp.E_OP_J * f["ops_total_per_s"] / 1e3)
            assert f["capacity_ops_per_s"] == pytest.approx(
                (cmp.CEILING_W * (1 - cmp.STATIC_FRAC) - 1e3 * p["transport"]) / cmp.E_OP_J)
            for b, v in f["bands"].items():
                assert v["beams_peak"] + 1e-9 >= v["beams_mean"]
                assert v["ops_total_per_s"] == pytest.approx(sum(v["ops_per_s"].values()))
        lat = {s["id"]: s["latency_q95_s"] for s in res["scheduler"]["sectors"]}
        for sid, s in c["sectors"].items():
            if sid in lat and lat[sid] > 0:
                assert lat[sid] + 1e-12 >= s["processing_s"], (n, sid)


def test_compute_partition_dwell_beams():
    """A3: a partition's RX beams field is per TX beam (sub-aperture): the dwell forms field x b_tx beams; the other
    modes field x positions per dwell."""
    res = {"sectors": [{"id": "X", "status": "OK", "positions_total": 10.0, "rr_target_s": 1.0,
                        "faces": {"0": {"positions": 10.0}},
                        "wf_by_bin": [{"positions": 10.0, "wf": {"n_pulses": 1, "pri_ms": 1.0, "sig_bw_mhz": 10.0,
                                                                   "positions_per_dwell": 6.0, "b_tx": 2,
                                                                   "tx_mode": "partition", "rx_beams_per_position": 9.0}}]}]}
    st = cmp._streams({"global": {}}, res, [0, 0, 0, 0])
    (rate, _, beams), = st[0]["S"][0]["looks"]
    assert beams == pytest.approx(18.0) and rate == pytest.approx(10.0 / 6.0)
    res["sectors"][0]["wf_by_bin"][0]["wf"]["tx_mode"] = "superposition"
    (rate, _, beams), = cmp._streams({"global": {}}, res, [0, 0, 0, 0])[0]["S"][0]["looks"]
    assert beams == pytest.approx(54.0)


def test_compute_memory_counts(runs):
    """Memory: per face, buffers = the sum of the bands' peaks, against the file's capacity and bandwidth."""
    for n, res in runs.items():
        for k, f in res["compute"]["faces"].items():
            m = f["memory"]
            assert m["buffers_peak_TB"] == pytest.approx(sum(b["buffer_peak_GB"] for b in f["bands"].values()) / 1e3)
            assert m["capacity_TB"] == pytest.approx(cmp.MEMORY_B / 1e12)
            assert m["bandwidth_TB_per_s"] == pytest.approx(cmp.MEMORY_BW_BPS / 8e12)
            assert m["traffic_TB_per_s"] >= 0.0


# ----------------------------------------------------------------------------- re-pointing in flight (step 4.5)
from core import repoint as rpt  # noqa: E402


def test_repoint_navigation_error_by_hand():
    """C1-C2: error of a transmission from the file's navigation values, its two-way loss in every band's losses, the
    acceleration that reaches the tolerance gives exactly the tolerance."""
    age = rpt.NAV_LATENCY_S + 1.0 / rpt.NAV_HZ
    att = rpt.ATT_SIGMA_DEG
    r2 = 2 * (att ** 2 + (rpt.RATE_SIGMA_DPS * age) ** 2) + (0.5 * rpt.ACCEL_MAX_DPS2 * age ** 2) ** 2
    assert rpt.r2_deg2() == pytest.approx(r2)
    for b in rm.BANDS.values():
        hp = rm.hpbw_deg(b.usable_hi_ghz)
        assert b.losses_db["nav_pointing"] == pytest.approx(2 * 12.0412 * r2 / hp ** 2, rel=1e-4)
        a_t = rpt.accel_tenable_dps2(hp)
        assert math.sqrt(rpt.r2_deg2(a_t)) == pytest.approx(rpt.TOL_HPBW * hp, rel=1e-9)
    assert rpt.axis_sigma_deg() == pytest.approx(math.sqrt(r2 / 2))


def test_repoint_migration_skips_ship_frame_and_flags_overload():
    """D1-D2: a ship-frame volume never migrates; a volume whose face load would overload its neighbour is flagged."""
    mission = {"global": {"eps_min_deg": -90.0, "eps_max_deg": 90.0},
               "sectors": [{"id": "A", "shape": "cap", "phi0_deg": 45.0, "eps0_deg": 20.0, "radius_deg": 10.0},
                           {"id": "B", "shape": "cap", "phi0_deg": 45.0, "eps0_deg": 20.0, "radius_deg": 10.0}]}
    res = {"tracking": {"faces": {"0": {"load": 0.9}, "1": {"load": 0.9}, "2": {"load": 0.0}, "3": {"load": 0.0}}},
           "sectors": [{"id": "A", "status": "OK", "frame": "ship", "faces": {"0": {"energy_fraction": 0.5}}},
                       {"id": "B", "status": "OK", "frame": "inertial",
                        "faces": {"0": {"energy_fraction": 0.3, "time_fraction": 0.05}}}]}
    out = rpt.budget(mission, res)
    assert "A" not in out["sectors"] and "B" in out["sectors"]
    f = out["sectors"]["B"]["faces"]["0"]
    assert f["neighbour_load_during_transit"] == pytest.approx(1.2)
    assert f["transit_s"] == pytest.approx(f["extent_deg"] / rpt.RATE_MAX_DPS)
    assert any("sector B" in x for x in out["flags"])


# ----------------------------------------------------------------------------- enriched mission model (step 5.1)
def test_aspect_policies_and_spreads():
    """B: worst = least total RCS over every presentation, mean = solid-angle mean, frontal = at the offset; the
    library keeps its detection RCS (worst is the frontal value of the tables, the KKV is frontal); the spreads lower
    the RCS and depolarisation discrimination indices."""
    for name, t in rm.TARGETS.items():
        assert len(t.aspect_rcs_db) in (0, len(rm.ASPECT_EL_DEG))
    k = rm.TARGETS["KKV"]
    assert k.policy() == "frontal" and k.optics_cone_deg == 5.0
    assert k.rcs_at(2.8) == pytest.approx(1.27, abs=0.01) and k.rcs_at(14.0) == pytest.approx(13.8, abs=0.1)
    assert rm.TARGETS["SHIP"].rcs_at(2.8) == pytest.approx(300.0) and rm.TARGETS["SHIP"].policy() == "worst"
    worst = dataclasses.replace(k, aspect_policy="worst")
    assert worst.rcs_at(2.8) == pytest.approx(k._frontal_body_m2(2.8))           # the tail, outside the optics cone
    mean = dataclasses.replace(rm.TARGETS["SHIP"], aspect_policy="mean")
    assert 10 * math.log10(mean.rcs_at(2.8) / 300.0) == pytest.approx(7.1, abs=0.2)
    assert rm.TARGETS["SHIP"].aspect_spread_db() > 2.0 and k.aspect_spread_db() == 0.0
    d = rm.TARGETS["DECOY"]
    no_spread = [dataclasses.replace(x, rcs_spread_db=0.0, depol_spread_db=0.0) for x in (k, d)]
    a = tm.indices(k, d, 14.0, 100.0, 5, 0.0, False, 1.0)
    b = tm.indices(*no_spread, 14.0, 100.0, 5, 0.0, False, 1.0)
    assert a["rcs"] < b["rcs"]
    # depolarisation: a pair of classes whose depolarisations differ
    x, y = next((p, q) for p in rm.TARGETS.values() for q in rm.TARGETS.values()
                if p.depol_lin(14.0) != q.depol_lin(14.0) and p.depol_spread_db > 0)
    a = tm.indices(x, y, 14.0, 100.0, 5, 0.0, False, 1.0)
    b = tm.indices(*[dataclasses.replace(z, depol_spread_db=0.0) for z in (x, y)], 14.0, 100.0, 5, 0.0, False, 1.0)
    assert a["depolarisation"] < b["depolarisation"]


def test_mission_settings_keys():
    """A: mission and volume settings are checked and reach the criteria."""
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    s = m["sectors"][0]
    m["global"].update(confirm_p_min=0.9, ship_rate_max_dps=20.0, ship_accel_max_dps2=10.0)
    s.update(pd=0.95, fa_rate_per_s=0.5)
    assert md.validate_mission(m) == []
    assert mo.fa_per_rr(s) == pytest.approx(0.5 * s["rr_target_s"])
    assert mo.search_criterion(s)[3] == 0.95 and len(mo.search_criterion(m["sectors"][1])) == 3
    s["fa_per_rr"] = 3.0
    m["global"]["confirm_p_min"] = 1.0
    errs = md.validate_mission(m)
    assert any("not both" in e for e in errs) and any("confirm_p_min" in e for e in errs)


# ----------------------------------------------------------------------------- global optimiser (step 5.2)
from core import allocation as alloc_  # noqa: E402


def test_power_allocation_pro_rata_and_priority_tiers(monkeypatch):
    """P1: enough power -> each face gets a share pro rata of its need (under its cap); shortfall -> the tiers in order
    over all the faces (a P1 volume before a P3 track on another face), the last one in part with a tenable revisit."""
    mission = {"sectors": [{"id": "SP", "self_protection": True}, {"id": "S4", "search_priority": 1},
                           {"id": "T", "target_type": "KKV"}]}
    k = 1.0 / rm.CHAIN_EFF
    res = {"global": {"face_elec_avail_kW": 1000.0, "face_elec_cap_mw": None},
           "compute": {"faces": {str(f): {"non_tx_kW_actual": 0.0} for f in range(4)}},
           "sectors": [{"id": "SP", "status": "OK", "rr_target_s": 20.0, "faces": {"1": {"energy_fraction": 1.0}}},
                       {"id": "S4", "status": "OK", "rr_target_s": 60.0, "faces": {"2": {"energy_fraction": 2.0}}}],
           "tracking": {"faces": {str(f): {"pumps_kW": 0.0} for f in range(4)},
                        "sectors": [{"id": "T", "at": [{"n_targets": 4.0, "faces": {"2": {"energy_fraction": 1.0}}}]}]}}
    a = alloc_.allocate(mission, res)
    need = [0.0, 1.0 * k, 3.0 * k, 0.0]
    assert a["status"] == "OK" and a["faces"]["1"]["alloc_MW"] / a["faces"]["2"]["alloc_MW"] == pytest.approx(1 / 3)
    assert sum(v["alloc_MW"] for v in a["faces"].values()) == pytest.approx(alloc_.SHIP_W / 1e6)
    monkeypatch.setattr(alloc_, "SHIP_W", 2.5e6 * k)          # P1 (1) and the KKV tracks (1) served, search in part
    b = alloc_.allocate(mission, res)
    part = {p["label"]: p for p in b["partial"]}
    assert b["status"] == "SHORT" and set(part) == {"search S4"}
    assert part["search S4"]["served_fraction"] == pytest.approx(0.25)
    assert part["search S4"]["rr_tenable_s"] == pytest.approx(240.0)
    assert sum(need) > 2.5 * k


def test_mission_choice_relieves_the_most_loaded_face():
    """B: a sector moves to an alternative plan when it lowers the most loaded face of the mission, even at a higher
    cost of its own; no alternative better for the mission -> no change."""
    def sec(sid, fe, ft, alts):
        return {"id": sid, "status": "OK", "faces": {str(k): {"energy_fraction": fe[k], "time_fraction": ft[k]}
                                                     for k in range(4)}, "_alternatives": alts}
    alt = {"freq_ghz": 2.8, "spacing": 1.6, "fe": [0.1, 0.5, 0.0, 0.0], "ft": [0.0] * 4, "objective": 0.5}
    out = {"tracking": {"faces": {}},
           "sectors": [sec("A", [0.4, 0.1, 0, 0], [0] * 4, [alt]), sec("B", [0.5, 0, 0, 0], [0] * 4, [])]}
    assert mo._mission_choice(out) == {"A": (2.8, 1.6)}              # face 0: 0.9 -> 0.6, face 1: 0.1 -> 0.5
    out["sectors"][1]["faces"]["0"]["energy_fraction"] = 0.0         # nothing saturates: keep the cheaper own plan
    assert mo._mission_choice(out) == {}


def test_couplings_feed_back_only_what_moves_a_choice(monkeypatch):
    """Couplings: the eclipse beyond scheduler.eclipse_flag_db only; the false-alarm cost only when the false alarms
    depend on the waveform (fa_mode pfa_cell)."""
    from core import scheduler as sch
    out = {"global": {"face_elec_avail_kW": 1000.0}, "scheduler": {"eclipse_db": {"S": sch.ECLIPSE_FLAG_DB / 2}},
           "sectors": [{"id": "A", "status": "OK", "rr_target_s": 10.0}],
           "tracking": {"sectors": [{"id": "A", "false_alarms_per_rr": 9.0,
                                     "confirmation": {"faces": {"0": {"energy_fraction": 0.001}}}}]}}
    assert mo._couplings({}, out) == {"A": {}}
    out["scheduler"]["eclipse_db"]["S"] = 2 * sch.ECLIPSE_FLAG_DB
    monkeypatch.setattr(rm, "FA_MODE", "pfa_cell")
    k = mo._couplings({}, out)["A"]
    assert k["_eclipse_db"] == pytest.approx(2 * sch.ECLIPSE_FLAG_DB)
    assert k["_fa_energy_J"] == pytest.approx(0.001 * 1e6 * 10.0 / 10.0)


# ----------------------------------------------------------------------------- margin reuse (step 5.2, P4)
from core import margin as margin_  # noqa: E402


def test_margin_fill_one_factor_until_a_face_or_rr_min():
    """R3: one factor raised for every search volume; the volumes of the face that reaches X stop, the others go on
    until their own face reaches X or their rr_min_s."""
    cons = {"F1 energy": {"fixed": 0.1, "a": {"A": 0.1, "B": 0.1}, "face": 0},
            "F2 energy": {"fixed": 0.2, "a": {"B": 0.05, "C": 0.05}, "face": 1}}
    ids = ["A", "B", "C"]
    nominal = {n: margin_._value(c, {s: 1.0 for s in ids}) for n, c in cons.items()}
    g, why = margin_._fill(cons, ids, {s: math.inf for s in ids}, 0.8, nominal)
    assert g["A"] == pytest.approx(3.5) and g["B"] == pytest.approx(3.5) and g["C"] == pytest.approx(8.5)
    assert why == {"A": "F1 energy", "B": "F1 energy", "C": "F2 energy"}
    g, why = margin_._fill(cons, ids, {"A": math.inf, "B": math.inf, "C": 6.0}, 0.8, nominal)
    assert g["C"] == pytest.approx(6.0) and why["C"] == "rr_min_s"
    nominal["F1 energy"] = 0.9                                        # a face above X: its volumes keep their RR
    g, why = margin_._fill(cons, ids, {s: math.inf for s in ids}, 0.8, nominal)
    assert g["A"] == g["B"] == 1.0 and g["C"] > 1.0


def test_margin_reuse_corrects_a_non_linear_resource(monkeypatch):
    """R4: a resource that grows faster than the revisit (here as g^2) is corrected by its secant and the filling
    redone: the real utilisation ends within the tolerance of X; realloc_mode NONE or PRIO_TRACK -> no search reuse."""
    mission = {"global": {"realloc_mode": "PRIO_SEARCH", "load_ceiling": 0.6},
               "sectors": [{"id": "A", "rr_target_s": 10.0, "fa_per_rr": 5.0}]}
    res = {"sectors": [{"id": "A", "status": "OK", "rr_target_s": 10.0, "fa_per_rr": 5.0,
                        "wf": {"pfa": 1e-6}, "wf_set": [{"pfa": 1e-6}], "wf_by_bin": [{"wf": {"pfa": 1e-6}}],
                        "faces": {"0": {"positions": 100.0, "energy_fraction": 0.1, "time_fraction": 0.01}}}],
           "warnings": []}

    def util(out):
        g = 10.0 / out["sectors"][0]["rr_target_s"]
        faces = {k: {"heat": 0.0} for k in range(4)}
        faces[0]["heat"] = 0.1 * g * g
        return 0.0, faces
    monkeypatch.setattr(margin_, "utilisation", util)
    seen = []

    def assemble(mis, results):
        seen.append((mis["sectors"][0], results[0]))
        return {"sectors": results, "warnings": []}
    mr = margin_.reuse(mission, res, assemble)
    g = mr["sectors"]["A"]["g"]
    assert 0.1 * g * g == pytest.approx(0.6, abs=margin_.TOL) and mr["sectors"]["A"]["limited_by"] == "F1 heat"
    sec, r = seen[-1]
    assert sec["rr_target_s"] == pytest.approx(10.0 / g) and "fa_per_rr" not in sec
    if rm.FA_MODE == "fa_rate":                                       # false alarms frozen per second
        assert sec["fa_rate_per_s"] == pytest.approx(0.5) and r["wf"]["pfa"] == pytest.approx(1e-6 / g)
    assert r["faces"]["0"]["energy_fraction"] == pytest.approx(0.1 * g) and r["_reuse_g"] == g
    assert res["sectors"][0]["rr_target_s"] == 10.0                  # the nominal result is untouched
    for mode in ("NONE", "PRIO_TRACK"):
        mission["global"]["realloc_mode"] = mode
        assert margin_.reuse(mission, res, assemble) is None


def test_margin_reuse_is_the_last_power_tier():
    """R5: the part of a search drawn by its shortened revisit is tier P5, after every search P4_n."""
    res = {"global": {"face_elec_avail_kW": 1000.0, "face_elec_cap_mw": None},
           "compute": {"faces": {str(f): {"non_tx_kW_actual": 0.0} for f in range(4)}},
           "sectors": [{"id": "A", "status": "OK", "rr_target_s": 2.0, "_reuse_g": 4.0,
                        "faces": {"0": {"energy_fraction": 0.8}}}],
           "tracking": {"faces": {str(f): {"pumps_kW": 0.0} for f in range(4)}, "sectors": []}}
    its = {it["label"]: it for it in alloc_.items({"sectors": [{"id": "A"}]}, res)}
    k = 1e6 / rm.CHAIN_EFF
    assert its["search A"]["w"][0] == pytest.approx(0.2 * k) and its["search A"]["rr_s"] == pytest.approx(8.0)
    assert its["margin reuse A"]["w"][0] == pytest.approx(0.6 * k) and its["margin reuse A"]["tier"] == (5, 0)
    assert alloc_.tier_name((5, 0)) == "P5 margin reuse"


def test_margin_reuse_mission_keys_validated():
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    m["global"].update({"auto_alloc_max": 0.8, "realloc_mode": "PRIO_ALL", "load_ceiling": 1.5})
    m["sectors"][0]["rr_min_s"] = m["sectors"][0]["rr_target_s"] * 2
    errs = md.validate_mission(m)
    assert any("auto_alloc_max was replaced" in e for e in errs) and any("rr_min_s" in e for e in errs)
    assert any("realloc_mode" in e and "PRIO_ALL" in e for e in errs) and any("load_ceiling" in e for e in errs)
    m["global"].pop("auto_alloc_max")
    m["global"].update({"realloc_mode": "PRIO_TRACK", "load_ceiling": 1.0})
    m["sectors"][0].pop("rr_min_s")
    assert not [e for e in md.validate_mission(m) if "realloc" in e or "ceiling" in e or "auto_alloc" in e]


def test_load_ceiling_alert_between_ceiling_and_one(monkeypatch):
    """Step 5.2 P7 (V4 c): a face whose binding resource lies above the load ceiling and within 1 is flagged (safety
    reserve in use); above 1 it is the verdict's fault, not this alert."""
    faces = {k: {"S time": 0.5} for k in range(4)}
    faces[1]["S time"], faces[2]["S time"] = 0.9, 1.2
    monkeypatch.setattr(margin_, "utilisation", lambda out: (0.1, faces))
    flags = margin_.ceiling_flags({"global": {}}, {})
    assert margin_.load_ceiling({"global": {}}) == margin_.CEILING_DEFAULT
    assert len(flags) == 1 and flags[0].startswith("face F2: S time at 90 %")
    assert margin_.ceiling_flags({"global": {"load_ceiling": 0.95}}, {}) == []


def _track_case(rr_min=None):
    """A mission with one sector and two track brackets on face F1, and an assembly whose Ku time goes as the update
    rate of the brackets (0.05 per bracket at its period asked)."""
    b0 = {"r_min_km": 0, "r_max_km": 100, "n_targets": 4, "rr_s": 2.0}
    b1 = {"r_min_km": 100, "r_max_km": 200, "n_targets": 2, "rr_s": 4.0}
    if rr_min is not None:
        b1["rr_min_s"] = rr_min
    mission = {"global": {"realloc_mode": "PRIO_TRACK", "load_ceiling": 0.6},
               "sectors": [{"id": "A", "rr_target_s": 10.0, "tracking": [b0, b1]}]}
    at = [{"faces": {"0": {"energy_fraction": 0.01, "ku_time_fraction": 0.05}}, "tracks_face": [4, 0, 0, 0]},
          {"faces": {"0": {"energy_fraction": 0.01, "ku_time_fraction": 0.05}}, "tracks_face": [2, 0, 0, 0]}]
    rr0 = {0: 2.0, 1: 4.0}

    def assemble(mis, results):
        return {"sectors": results, "warnings": [], "_mis": mis,
                "tracking": {"sectors": [{"id": "A", "at": at}]}}
    out = assemble(mission, [{"id": "A", "status": "OK"}])

    def util(o):
        brs = o["_mis"]["sectors"][0]["tracking"]
        faces = {k: {"Ku time": 0.0} for k in range(4)}
        faces[0]["Ku time"] = 0.1 + sum(0.05 * rr0[i] / float(b["rr_s"]) for i, b in enumerate(brs))
        return 0.0, faces
    return mission, out, assemble, util


def test_prio_track_speeds_up_the_track_cadence_only(monkeypatch):
    """Step 5.2 P7: PRIO_TRACK -> one factor h on every track bracket up to the load ceiling (0.1 + 0.1 h = 0.6,
    h = 5), the search untouched; a bracket stops at its rr_min_s; NONE / PRIO_SEARCH -> no track reuse."""
    mission, out, assemble, util = _track_case()
    monkeypatch.setattr(margin_, "utilisation", util)
    mr = margin_.reuse_tracks(mission, out, assemble)
    br = mr["brackets"]
    assert br["A/0"]["h"] == pytest.approx(5.0, rel=1e-6) and br["A/1"]["h"] == pytest.approx(5.0, rel=1e-6)
    assert br["A/0"]["rr_real_s"] == pytest.approx(0.4) and br["A/0"]["limited_by"] == "F1 Ku time"
    assert br["A/0"]["rr_min_s"] == margin_.RR_MIN_TRACK_S
    mission, out, assemble, util = _track_case(rr_min=2.0)          # bracket 1 stops at h = 2, bracket 0 goes on
    mr = margin_.reuse_tracks(mission, out, assemble)
    br = mr["brackets"]
    assert br["A/1"]["h"] == pytest.approx(2.0) and br["A/1"]["limited_by"] == "rr_min_s"
    assert 0.1 + 0.05 * br["A/0"]["h"] + 0.05 * 2.0 == pytest.approx(0.6, abs=margin_.TOL)
    for mode in ("NONE", "PRIO_SEARCH"):
        mission["global"]["realloc_mode"] = mode
        assert margin_.reuse_tracks(mission, out, assemble) is None


def test_reuse_default_floors(monkeypatch):
    """Step 5.2 P7 (user): without rr_min_s the reuse stops at rr_min_track_s (0.1 s) for the tracks and
    rr_min_search_s (1 s) for the search; a period already at or below the floor is not sped up."""
    assert margin_.RR_MIN_TRACK_S == 0.1 and margin_.RR_MIN_SEARCH_S == 1.0
    mission, out, assemble, util = _track_case()
    mission["global"]["load_ceiling"] = 1.0
    monkeypatch.setattr(margin_, "utilisation", lambda o: (0.0, {k: {"Ku time": 0.0} for k in range(4)}))
    br = margin_.reuse_tracks(mission, out, assemble)["brackets"]
    assert br["A/0"]["rr_real_s"] == pytest.approx(0.1) and br["A/1"]["rr_real_s"] == pytest.approx(0.1)
    assert {b["limited_by"] for b in br.values()} == {"rr_min_s"}
    mission["sectors"][0]["tracking"][0]["rr_s"] = 0.05
    assert margin_.reuse_tracks(mission, out, assemble)["brackets"]["A/0"]["h"] == 1.0


def test_reuse_bisection_on_a_step_resource(monkeypatch):
    """Step 5.2 P7: a resource that jumps (a step, as the simultaneous Ku beams) defeats the secant: the factors are cut
    back together by bisection and the result stays within the load ceiling."""
    mission, out, assemble, util = _track_case()

    def step_util(o):
        brs = o["_mis"]["sectors"][0]["tracking"]
        h = 2.0 / float(brs[0]["rr_s"])
        faces = {k: {"Ku beams": 0.0} for k in range(4)}
        faces[0]["Ku beams"] = 0.2 if h < 3.0 else 0.9               # flat, then a jump above the ceiling
        return 0.0, faces
    monkeypatch.setattr(margin_, "utilisation", step_util)
    mr = margin_.reuse_tracks(mission, out, assemble)
    h = mr["brackets"]["A/0"]["h"]
    assert 1.0 <= h < 3.0 and "bisection" in mr["brackets"]["A/0"]["limited_by"]
    assert any("cut back by bisection" in f for f in mr["flags"])


def test_track_reuse_is_the_last_power_tier():
    """Step 5.2 P7: the energy of a sped-up track bracket beyond its updates at the period asked is tier P5."""
    res = {"global": {"face_elec_avail_kW": 1000.0, "face_elec_cap_mw": None},
           "compute": {"faces": {str(f): {"non_tx_kW_actual": 0.0} for f in range(4)}},
           "sectors": [],
           "tracking": {"faces": {str(f): {"pumps_kW": 0.0} for f in range(4)},
                        "sectors": [{"id": "A", "at": [{"n_targets": 4,
                                                        "faces": {"0": {"energy_fraction": 0.5}}}]}]}}
    mission = {"sectors": [{"id": "A", "target_type": "KKV",
                            "tracking": [{"rr_s": 0.5, "_reuse_h": 4.0, "_e_nominal": [0.2, 0, 0, 0]}]}]}
    its = {it["label"]: it for it in alloc_.items(mission, res)}
    k = 1e6 / rm.CHAIN_EFF
    assert its["tracks A/0"]["w"][0] == pytest.approx(0.2 * k) and its["tracks A/0"]["tier"] == (3, 0)
    assert its["margin reuse A/0"]["w"][0] == pytest.approx(0.3 * k) and its["margin reuse A/0"]["tier"] == (5, 0)


def test_saturation_table_extra_tracks_alone(monkeypatch):
    """Step 5.2 P7: manageable saturation of a bracket = room up to the load ceiling / cost of one more track, the
    smallest over the resources; the cost comes from one probe of max(1, 10 % n) more tracks per bracket."""
    mission, out, assemble, _util = _track_case()
    mission["global"]["load_ceiling"] = 0.8
    cost = {0: {"Ku time": 0.01, "energy": 0.002}, 1: {"Ku time": 0.002, "energy": 0.02}}   # per track, face F1

    def util(o, beams="beams_peak"):
        brs = o["_mis"]["sectors"][0]["tracking"]
        faces = {k: {"Ku time": 0.0, "energy": 0.0} for k in range(4)}
        for i, b in enumerate(brs):
            for name, c in cost[i].items():
                faces[0][name] += c * float(b["n_targets"])
        return 0.0, faces

    def assemble2(mis, results):
        o = assemble(mis, results)
        brs = mis["sectors"][0]["tracking"]
        o["tracking"]["sectors"][0]["at"] = [
            {"faces": {"0": {"energy_fraction": cost[i]["energy"] * float(b["n_targets"]),
                             "ku_time_fraction": cost[i]["Ku time"] * float(b["n_targets"])}},
             "tracks_face": [float(b["n_targets"]), 0, 0, 0]} for i, b in enumerate(brs)]
        return o
    monkeypatch.setattr(margin_, "utilisation", util)
    monkeypatch.setattr(margin_, "critique", lambda *a, **k: None)     # tested on its own
    sat = margin_.saturation(mission, assemble2(mission, out["sectors"]), assemble2)
    assert sat["probe"] == {"A/0": 1.0, "A/1": 1.0}
    u_t, u_e = 0.01 * 4 + 0.002 * 2, 0.002 * 4 + 0.02 * 2                # nominal Ku time 0.044, energy 0.048
    b0, b1 = sat["brackets"]["A/0"], sat["brackets"]["A/1"]
    assert b0["binding_alone"] == "F1 Ku time" and b0["extra_alone"] == math.floor((0.8 - u_t) / 0.01 + 1e-9)
    assert b1["binding_alone"] == "F1 energy" and b1["extra_alone"] == math.floor((0.8 - u_e) / 0.02 + 1e-9)
    assert b0["cost_per_track"]["F1 Ku time"] == pytest.approx(0.01)
    cum = sat["cumulative"]                       # same fraction p of n: Ku time 0.044 (1 + p) <= 0.8, energy 0.048 ...
    p = min((0.8 - u_t) / u_t, (0.8 - u_e) / u_e)
    assert cum["p"] == pytest.approx(p) and cum["verified"] and not cum["bisected"]
    a0, a1 = cum["extra"]["A/0"], cum["extra"]["A/1"]
    assert a0 >= math.floor(p * 4) and a1 >= math.floor(p * 2)
    assert 0.01 * (4 + a0) + 0.002 * (2 + a1) <= 0.8 + 1e-9 and 0.002 * (4 + a0) + 0.02 * (2 + a1) <= 0.8 + 1e-9

    def step(o, beams="beams_peak"):                  # a step above the ceiling beyond 20 tracks in bracket 0
        ship, faces = util(o, beams)
        if float(o["_mis"]["sectors"][0]["tracking"][0]["n_targets"]) > 20:
            faces[0]["memory"] = 0.95
        else:
            faces[0]["memory"] = 0.1
        return ship, faces
    monkeypatch.setattr(margin_, "utilisation", step)
    cum = margin_.saturation(mission, assemble2(mission, out["sectors"]), assemble2)["cumulative"]
    assert cum["bisected"] and 4 + cum["extra"]["A/0"] <= 20 and "(bisection)" in cum["binding"]
    monkeypatch.setattr(margin_, "utilisation", util)
    mission["global"]["load_ceiling"] = 0.04                            # below the nominal load: nothing manageable
    sat = margin_.saturation(mission, assemble2(mission, out["sectors"]), assemble2)
    assert sat["brackets"]["A/0"]["extra_alone"] == 0 and any("no manageable saturation" in f for f in sat["flags"])
    assert margin_.saturation({"global": {}, "sectors": []}, {"tracking": {"sectors": []}}, assemble2) is None


def test_track_floor_from_the_loss_chain(monkeypatch):
    """Step 5.2 P7 (D1): t_max = the period where the loss per update crosses p_loss_max (log-log), the factors above
    sat_stretch_stage only for a bracket still within it, capped by latency_max_s."""
    calls = []

    def budget(mission, out):
        brs = mission["sectors"][0]["tracking"]
        calls.append([b["rr_s"] for b in brs])
        loss = lambda T: 1e-3 * (T / 2.0) ** 2              # 1 % at T = 2 sqrt(10) = 6.32 s
        return {"sectors": [{"id": "A", "at": [{"p_loss_per_update": loss(b["rr_s"]), "rr_tenable": True}
                                               for b in brs]}]}
    monkeypatch.setattr(trk, "budget", budget)
    sec = {"id": "A", "tracking": [{"r_min_km": 0, "r_max_km": 100, "n_targets": 4, "rr_s": 2.0},
                                   {"r_min_km": 100, "r_max_km": 200, "n_targets": 2, "rr_s": 0.5}]}
    mission = {"global": {}, "sectors": [sec]}
    items = {"A/0": (sec, sec["tracking"][0], {}), "A/1": (sec, sec["tracking"][1], {})}
    rep = {"flags": []}
    fl = margin_.track_floors(mission, {}, items, rep)
    assert fl["A/0"]["t_max_s"] == pytest.approx(2.0 * math.sqrt(10.0), rel=1e-9)
    assert fl["A/1"]["t_max_s"] == pytest.approx(2.0 * math.sqrt(10.0), rel=1e-9)   # found in the second stage
    assert len(calls) == 2 and calls[1] == [0.5 * 8.0, 0.5 * 16.0]                   # only bracket A/1 (x8, x16)
    sec["latency_max_s"] = 3.0
    f0 = margin_.track_floors(mission, {}, items, rep)["A/0"]
    assert f0["t_max_s"] == 3.0 and f0["limited_by"] == "latency_max_s"


def test_critique_threshold_per_mode(monkeypatch):
    """Step 5.2 P7 (D3, D4): PRIO_SEARCH / NONE stretch every track (n too) to t_max with the search kept: N x cost x
    rr / t_max <= room + n x cost; PRIO_TRACK adds the room the P4_n search gives back at its rr_max first; the first
    task shed follows the mode."""
    sec = {"id": "A", "rr_target_s": 10.0, "search_priority": 2,
           "tracking": [{"r_min_km": 0, "r_max_km": 100, "n_targets": 10, "rr_s": 1.0}]}
    mission = {"global": {"realloc_mode": "NONE"}, "sectors": [sec, {"id": "B", "rr_target_s": 5.0}]}
    items = {"A/0": (sec, sec["tracking"][0], {})}
    cost = {"F1 energy": {"A/0": 0.01}}
    room = {"F1 energy": 0.5}                                           # 50 more tracks at 1 s
    monkeypatch.setattr(margin_, "track_floors", lambda *a: {"A/0": {"t_max_s": 4.0}})
    monkeypatch.setattr(margin_, "utilisation",
                        lambda o, beams="beams_peak": (0.0, {k: {"energy": o.get("_u", 0.3) if k == 0 else 0.0}
                                                             for k in range(4)}))
    assemble = lambda mis, res: {"sectors": res, "_u": 0.3}
    out = {"sectors": [{"id": "A", "status": "OK", "rr_target_s": 10.0, "faces": {}},
                       {"id": "B", "status": "OK", "rr_target_s": 5.0, "faces": {}}]}
    rep = {"flags": []}
    c = margin_.critique(mission, out, items, cost, room, 0.8, assemble, None, rep)
    # N x 0.01 x 1/4 <= 0.5 + 0.1 -> N = 240, extra 230
    assert c["brackets"]["A/0"]["extra_critique"] == 230 and c["mode"] == "NONE"
    assert c["cumulative"]["p"] == pytest.approx(23.0) and c["cumulative"]["verified"]
    assert c["first_shed"].startswith("tracks of bracket A/0")
    mission["global"]["realloc_mode"] = "PRIO_TRACK"
    monkeypatch.setattr(margin_, "search_floors", lambda *a: {"A": {"rr_max_s": 40.0}, "B": {"rr_max_s": 20.0}})
    assemble2 = lambda mis, res: {"sectors": res, "_u": 0.3 if all(r.get("_reuse_g", 1.0) == 1.0 for r in res) else 0.1}
    c = margin_.critique(mission, out, items, cost, room, 0.8, assemble2, None, rep)
    # the search gives back 0.2: N x 0.0025 <= 0.5 + 0.2 + 0.1 -> N = 320
    assert c["brackets"]["A/0"]["extra_critique"] == 310
    assert c["first_shed"] == "search volume A (P4_2), whole volume"


def test_ku_levers_give_back_burn_through_and_confirmations():
    """Step 5.2 P7 (Q16): P3' Ku time dropped, Ku confirmations moved to S2 (Ku freed, S2 S time taken)."""
    out = {"tracking": {"sectors": [{
        "id": "A", "confirm_mode": "KU1+7",
        "confirmation": {"faces": {"0": {"ku_time_fraction": 0.05, "s_time_fraction": 0.0}}},
        "alternatives": {"S2": {"admissible": True, "faces": {"0": {"ku_time_fraction": 0.01, "s_time_fraction": 0.03}}}},
        "ku_burn_through": {"faces": {"0": {"ku_time_fraction": 0.02}}}}]}}
    lev = margin_._ku_levers(out)
    assert lev["F1 Ku time"] == pytest.approx(0.02 + 0.05 - 0.01) and lev["F1 S time"] == pytest.approx(-0.03)


def test_search_rr_max_validated():
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_example.json"))
    m["sectors"][0]["rr_max_s"] = m["sectors"][0]["rr_target_s"] / 2
    assert any("rr_max_s" in e and "below rr_target_s" in e for e in md.validate_mission(m))
    m["sectors"][0]["rr_max_s"] = m["sectors"][0]["rr_target_s"] * 4
    assert not [e for e in md.validate_mission(m) if "rr_max_s" in e]
    assert margin_._interp_cross([1.0, 2.0, 4.0], [0.0, 0.5, 2.0], 1.0) == pytest.approx(2.0 * 2 ** 0.5)
    assert margin_._interp_cross([1.0, 2.0], [0.1, 0.2], 1.0) is None


def test_track_rr_min_validated():
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_synthetic_mix.json"))
    sec = next(s for s in m["sectors"] if s.get("tracking"))
    sec["tracking"][0]["rr_min_s"] = sec["tracking"][0]["rr_s"] * 2
    assert any("rr_min_s" in e and "update period" in e for e in md.validate_mission(m))
    sec["tracking"][0]["rr_min_s"] = -1
    assert any("rr_min_s must be > 0" in e for e in md.validate_mission(m))
    sec["tracking"][0]["rr_min_s"] = sec["tracking"][0]["rr_s"] / 2
    assert not [e for e in md.validate_mission(m) if "rr_min_s" in e]


# ----------------------------------------------------------------------------- flight tables (step 5.2, P5)
def test_flight_table_spreads_over_the_band_and_flags_degraded(monkeypatch):
    """P5: the chosen plan first, then plans spread over the band (farthest frequency first), at most
    flight_table_entries_max; a plan with more of the sector unscanned or more positions short is kept, degraded."""
    from types import SimpleNamespace as NS
    monkeypatch.setattr(mo, "FT_ENTRIES_MAX", 3)

    def x(f, uns=0.0, short=0.0):
        plan = NS(choice=np.array([0, 0, -1]), r_bin=np.array([500.0, 480.0, np.nan]), short_frac=short,
                  fe=np.array([0.1, 0, 0, 0]), ft=np.array([0.01, 0, 0, 0]))
        return (f, None, uns, plan, None, None, None, np.array([0, 1, 2]), None, 1.4)
    best = x(2.70)
    ev = {(f, 1.4): x(f, short=0.05 if f == 2.90 else 0.0) for f in (2.66, 2.70, 2.72, 2.80, 2.90)}
    ev[(2.70, 1.4)] = best
    cands = [NS(occupied_mhz=100.0)]
    ft = mo._flight_table(ev, best, cands, 0.01, lambda x, m: (round(x[2] - m, 3), x[3].short_frac, x[0]))
    fs = [e["freq_ghz"] for e in ft["entries"]]
    assert fs == [2.70, 2.80, 2.90] and ft["centres_evaluated"] == 5          # 2.90 farthest, then 2.80
    assert [e["chosen"] for e in ft["entries"]] == [True, False, False]
    assert [e["degraded"] for e in ft["entries"]] == [False, False, True] and ft["degraded"] == 1
    assert ft["entries"][0]["occupied_ghz"] == [2.65, 2.75] and ft["entries"][0]["range_worst_km"] == 480
    assert ft["entry_groups"] == 3 and ft["bytes"] == pytest.approx(3 * mo.FT_BYTES)


def test_flight_table_keeps_the_search_after_a_forced_plan():
    """P5: a sector re-planned on one forced centre keeps the table of its search, the forced plan in place of the
    entry nearest in frequency."""
    def e(f, chosen=False, degraded=False):
        return {"freq_ghz": f, "spacing": 1.4, "chosen": chosen, "degraded": degraded, "short_pct": 0.0,
                "unscanned_pct": 0.0, "groups": 2}
    old = mo._table_summary([e(2.66), e(2.72, chosen=True), e(2.84, degraded=True)], 12)
    new = mo._table_summary([dict(e(2.83, chosen=True), spacing=1.6)], 1)
    ft = mo._merge_flight_table(old, new)
    assert [(x["freq_ghz"], x["chosen"]) for x in ft["entries"]] == [(2.66, False), (2.72, False), (2.83, True)]
    assert ft["centres_evaluated"] == 12 and ft["degraded"] == 0 and ft["entry_groups"] == 6


# ----------------------------------------------------------------------------- step 5.3: verdict
from core import validation as val_  # noqa: E402


def test_verdict_states_by_kind():
    """Step 5.3 P1: resource OK up to the load ceiling, ALERT up to 1, NOK above; spec OK up to 1, NOK above; transient
    ALERT above 1; an infinite load (None) fails a resource or a spec and alerts a transient."""
    assert [val_._state(x, "resource", 0.8) for x in (0.8, 0.9, 1.0, 1.01, None)] == ["OK", "ALERT", "ALERT", "NOK", "NOK"]
    assert [val_._state(x, "spec", 0.8) for x in (0.9, 1.0, 1.01, None)] == ["OK", "OK", "NOK", "NOK"]
    assert [val_._state(x, "transient", 0.8) for x in (1.0, 1.5, None)] == ["OK", "ALERT", "ALERT"]


def test_verdict_ko_alert_priorities_and_limiting():
    """Step 5.3 P1: a face in its safety reserve alerts, a tier P2 not served in full fails the mission (P2
    unreachable), a revisit stretched fails its sector; the limiting resource is the highest resource load."""
    out = {"sectors": [{"id": "A", "status": "OK", "rr_target_s": 10.0, "rr_est_mission_s": 12.0,
                        "range_km": {"requested": 1000.0, "at_worst_scan": 1000, "positions_short_of_range_pct": 0.5}}],
           "tracking": {"faces": {"0": {"total_energy": 0.9, "heat_load": 0.1}}},
           "power_allocation": {"need_MW": 4.0, "ship_MW": 8.0, "faces": {},
                                "partial": [{"label": "confirmation A", "tier": "P2 confirmation / initiation",
                                             "served_fraction": 0.5}]}}
    v = val_.verdict({"global": {}, "sectors": [{"id": "A", "range_km": 1000.0, "short_tol": 0.01}]}, out)
    by = {r["id"]: r for r in v["constraints"]}
    assert by["F1.energy"]["state"] == "ALERT" and by["F1.heat"]["state"] == "OK"
    assert by["A.revisit"]["state"] == "NOK" and abs(by["A.revisit"]["load"] - 1.2) < 1e-12
    assert by["A.range"]["state"] == "ALERT" and abs(by["A.range"]["load"] - 0.5) < 1e-12
    assert by["confirmation A.ship power served"]["state"] == "NOK"
    assert v["state"] == "NOK" and v["nok_priorities"] == ["P2", "P4_1"]
    assert v["saturation"]["state_at_spec"] == "P2 unreachable"
    assert v["limiting"] == {"object": "F1", "resource": "energy", "load": 0.9, "state": "ALERT"}
    assert v["counts"]["NOK"] == 2 and v["faces"]["0"]["resource"] == "energy"


def test_verdict_on_the_frozen_missions(runs):
    """Step 5.3 P1: the verdict frozen with each mission is the one recomputed from its outputs, without NaN or
    infinity, and the mission state is the worst constraint state."""
    for n, res in runs.items():
        v = val_.verdict(md.load_mission(os.path.join(tb.MISSIONS_DIR, n)), res)
        json.dumps(v, allow_nan=False)
        states = {r["state"] for r in v["constraints"]}
        assert states <= set(val_.STATES), n
        assert v["state"] == ("NOK" if "NOK" in states else "ALERT" if "ALERT" in states else "OK"), n
        if "verdict" in res:                 # frozen from the refreeze of step 5.3 on
            # the frozen outputs keep their keys sorted (the re-pointing bands Ku before S): the order of the rows,
            # and so the numbers of the advice, is presentation only
            def plain(x):
                x = tb.to_plain(x)
                x["constraints"] = {r["id"]: r for r in x["constraints"]}
                x["advice"] = sorted((dict(a, n=None) for a in x["advice"]), key=lambda a: json.dumps(a, sort_keys=True))
                return x
            # keys added by a later pass (P3: redundancy, overlap) are compared once refrozen
            assert tb.differences(plain(res["verdict"]), plain({k: v[k] for k in res["verdict"]})) == [], n


def test_advice_laws_alone_and_uniform(monkeypatch):
    """Step 5.3 P2: a face 0.4 above the ceiling (S time 1.2, ceiling 0.8), a search of share 0.6 alone keeps
    f = 1/3 of its load: range x f^(1/4), RR / f, RCS / f, rounded to 3 digits on the relieving side; the track
    bracket (0.3) cannot alone; the uniform factor over both is f = 1 - 0.4 / 0.9 on the revisits."""
    items = {("search", "A", None): {"share": {(0, "S time"): 0.6}, "ship": 0.1},
             ("track", "A", 0): {"share": {(0, "S time"): 0.3}, "ship": 0.05}}
    monkeypatch.setattr(val_, "_items", lambda m, o: items)
    mission = {"global": {}, "sectors": [{"id": "A", "range_km": 1000.0, "rr_target_s": 10.0,
                                          "tracking": [{"n_targets": 10, "rr_s": 1.0, "range_acc_m": 10.0}]}]}
    out = {"sectors": [{"id": "A", "status": "OK", "target": {"rcs_m2": 1.0}}],
           "tracking": {"sectors": [{"id": "A", "at": [{"driver": "RNG"}]}]}}
    row = {"id": "F1.S time", "family": "face", "object": "F1", "quantity": "S time", "kind": "resource",
           "load": 1.2, "state": "NOK", "limit": 1.0, "value": 1.2}
    adv = val_.advice(mission, out, [row], 0.8)
    alone = {a["changes"][0]["key"]: a["changes"][0] for a in adv if a["kind"] == "alone"}
    assert set(alone) == {"range_km", "rr_target_s", "rcs_m2"}
    assert alone["range_km"]["proposed"] == 759.0 and alone["range_km"]["proposed_at_1"] == 903.0
    assert alone["rr_target_s"]["proposed"] == 30.0 and alone["rcs_m2"]["proposed"] == 3.0
    uni = [a for a in adv if a["kind"] == "uniform"][0]
    assert {c["key"]: c["proposed"] for c in uni["changes"]} == {"rr_target_s": 18.0, "rr_s": 1.8}
    assert val_._safe(1234.5, True) == 1240.0 and val_._safe(1234.5, False) == 1230.0
    assert val_._safe(2.3, False, integer=True) == 2.0


def test_verify_bisects_to_the_smallest_relaxation():
    """Step 5.3 P2: a mission whose load goes as (R / 1000)^4 x 1.2 holds at the ceiling 0.8 for R <= 903.6 km; from
    the estimate 759 km the verification keeps the largest range that holds, within the 6 bisection steps."""
    a = {"constraint": "F1.S time", "target_load": 0.8,
         "changes": [{"sector": "A", "bracket": None, "key": "range_km", "current": 1000.0, "proposed": 759.0}]}
    mission = {"global": {}, "sectors": [{"id": "A", "range_km": 1000.0}]}

    def run(m):
        load = 1.2 * (m["sectors"][0]["range_km"] / 1000.0) ** 4
        return {"verdict": {"state": "NOK", "constraints": [{"id": "F1.S time", "load": load, "kind": "resource",
                                                            "state": "NOK" if load > 1 else "OK"}]}}
    res = val_.verify(mission, a, run=run)
    r = res["changes"][0]["verified"]
    assert res["holds"] and len(res["runs"]) == 1 + val_.VERIFY_BISECTIONS
    assert 1000.0 * (0.8 / 1.2) ** 0.25 * (759.0 / 1000.0) ** (1 / 2 ** val_.VERIFY_BISECTIONS) <= r
    assert r <= 1000.0 * (0.8 / 1.2) ** 0.25 + 1e-9
    a["changes"][0]["proposed"] = 950.0                   # an estimate that does not hold: doubled, then bisected
    res = val_.verify(mission, a, run=run)
    assert res["holds"] and res["runs"][0]["holds"] is False

def test_search_law_counts_the_look_spacing():
    """Step 5.3 P2 (user, 27 October 2026): the range of a volume is the worst one, the looks that count being spaced
    v RR beyond it. Without closing speed the law is R^4 / RR; for a fast closer (25 km/s, RR 20 s, 500 km: q = 1) a
    longer revisit raises the load, and a shorter range relieves less than R^4 (R' below R f^(1/4))."""
    res = {"wf": {"n_pulses": 4, "pfa": 6.3e-11}, "target": {"swerling": 1}, "pd_cumulative": 0.9}
    still = val_._SearchLaw({"range_km": 1000.0, "rr_target_s": 10.0, "v_close_max_mps": 0.0}, res)
    assert abs(still.ratio(500.0, 20.0) - 0.5 ** 4 / 2) < 1e-12
    assert abs(still.range_at(0.5) - 1000.0 * 0.5 ** 0.25) < 1e-6 and abs(still.rr_at(0.5) - 20.0) < 1e-6
    fast = val_._SearchLaw({"range_km": 500.0, "rr_target_s": 20.0, "v_close_max_mps": 25000.0}, res)
    assert fast.ratio(500.0, 33.8) > 1.0 and fast.rr_at(0.64) is None
    assert fast.range_at(0.64) < 500.0 * 0.64 ** 0.25


def test_overlap_takeable_within_the_neighbour_margin(monkeypatch):
    """Step 5.3 P3: face F1 could hand over to F2 a zone that costs F2 15 % of S time; F2 is at 74 % for a ceiling of
    80 %, so 0.06 / 0.15 = 40 % of the zone is takeable, relieving F1 by 40 % of its 10 %; the ship energy (faces
    pro rata, not capped) rises by the cost difference. On F1 S time in alert, one overlap line, no change."""
    monkeypatch.setattr(val_, "_items", lambda m, o: {})
    out = {"sectors": [{"id": "A", "face_overlap": {"0>1": {
        "sr": 1.0, "sr_reach": 0.5, "fe_own": 0.1, "ft_own": 0.2, "fe_own_reach": 0.05, "ft_own_reach": 0.1,
        "fe_other": 0.08, "ft_other": 0.15}}}],
        "power_allocation": {"ship_MW": 8.0, "need_MW": 4.0, "faces": {str(k): {"need_MW": 1.0} for k in range(4)}}}
    rows = [{"id": "F1.S time", "family": "face", "object": "F1", "quantity": "S time", "kind": "resource",
             "load": 0.85, "state": "ALERT", "limit": 1.0, "value": 0.85},
            {"id": "F2.S time", "family": "face", "object": "F2", "quantity": "S time", "kind": "resource",
             "load": 0.74, "state": "OK", "limit": 1.0, "value": 0.74}]
    ov = val_._overlap(out, rows, 0.8)
    assert len(ov) == 1 and ov[0]["face"] == "F1" and ov[0]["neighbour"] == "F2"
    o = ov[0]
    assert abs(o["takeable"] - 0.4) < 1e-12 and abs(o["relief_s_time"] - 0.04) < 1e-12
    p_ref = rm.face_elec_avail_w(rm.FACE_ELEC_POWER_MW) / 1e6
    assert o["energy_on"] == "ship" and abs(o["relief_energy"] - 0.4 * (0.05 - 0.08) * p_ref / 8.0) < 1e-12
    adv = val_.advice({"global": {}, "sectors": []}, out, rows, 0.8, ov)
    lines = [a for a in adv if a["kind"] == "overlap"]
    assert len(lines) == 1 and lines[0]["changes"] == [] and "takeable 40 %" in lines[0]["note"]


def test_redundancy_advice_comes_first_and_applies(monkeypatch):
    """Step 5.3 P3: a fully redundant volume weighing 0.3 on an S time in alert is advised first (drop), a partly
    redundant one is trimmed to its pieces; apply_changes drops or replaces the volume."""
    monkeypatch.setattr(val_, "_items", lambda m, o: {("search", "B", None): {"share": {(0, "S time"): 0.3}, "ship": 0.1},
                                                     ("search", "C", None): {"share": {(0, "S time"): 0.2}, "ship": 0.1}})
    pieces = [{"id": "C", "phi_min_deg": 0.0, "phi_max_deg": 10.0, "eps_min_deg": 0.0, "eps_max_deg": 5.0},
              {"id": "C_2", "phi_min_deg": 20.0, "phi_max_deg": 30.0, "eps_min_deg": 0.0, "eps_max_deg": 5.0}]
    out = {"sectors": [], "redundancy": [
        {"sector": "B", "by": ["A"], "fraction": 1.0, "full": True, "pieces": None, "note": "", "caveat": ""},
        {"sector": "C", "by": ["A"], "fraction": 0.5, "full": False, "pieces": pieces, "note": "", "caveat": ""}]}
    row = {"id": "F1.S time", "family": "face", "object": "F1", "quantity": "S time", "kind": "resource",
           "load": 0.9, "state": "ALERT", "limit": 1.0, "value": 0.9}
    mission = {"global": {}, "sectors": [{"id": "A"}, {"id": "B"}, {"id": "C"}]}
    adv = val_.advice(mission, out, [row], 0.8)
    assert [a["kind"] for a in adv[:2]] == ["redundancy", "redundancy"]
    assert adv[0]["changes"][0]["key"] == "removed" and "relief 30.00 %" in adv[0]["note"]
    assert adv[1]["changes"][0]["key"] == "pieces" and adv[1]["approximate"]
    assert [s["id"] for s in val_.apply_changes(mission, adv[0]["changes"])["sectors"]] == ["A", "C"]
    assert [s["id"] for s in val_.apply_changes(mission, adv[1]["changes"])["sectors"]] == ["A", "B", "C", "C_2"]
    assert val_.apply_changes(mission, adv[0]["changes"], 0.0)["sectors"] == mission["sectors"]
    assert "B removed" in val_.changes_text(adv[0]) and "C_2" in val_.changes_text(adv[1])


def test_redundancy_uses_the_plans_given(monkeypatch):
    """Step 5.3 P3: with the plans of a run, find_redundancies plans no sector itself (a sector without a plan meets
    nobody's need: no finding)."""
    from core import geometry as geo
    from core import redundancy as red

    def no_plan(*a, **k):
        raise AssertionError("planned again")
    monkeypatch.setattr(red, "_plan", no_plan)
    m = md.load_mission(os.path.join(tb.MISSIONS_DIR, "mission_redundancy_demo.json"))
    findings, notes = red.find_redundancies(m, geo.SphereGrid(res_deg=2.0), {s["id"]: None for s in m["sectors"]})
    assert findings == [] and notes

def _mc_bin(n=4, shape=1.0, pfa=1e-6, x_db=None, r_km=500.0, short=False, lattice=False, q=0.0):
    """A scan bin of the Monte-Carlo (step 5.4) built without the optimiser: a range straddle of +/- 0.5 dB and, with
    lattice, the S TX and RX lattices of the search; its analytic law from the same samples; x_db None = the SNR the
    criterion requires (cumulative over looks spaced q)."""
    from core import detection as det
    from core import montecarlo as mc
    rng_s = np.linspace(0.0, 1.0, 401)
    tx_cell = tx_beam = rx_cell = rx_beam = None
    law = (np.array(det.quantiles(rng_s, rm.OFFSET_QUANTILES)[0]), np.full(rm.OFFSET_QUANTILES, 1.0 / rm.OFFSET_QUANTILES))
    if lattice:
        pat = rm.PATTERNS["S"]
        tx_cell, tx_beam = rm.BEAM_SPACING_HPBW * pat.tx.hpbw, pat.tx
        rx_beam = rm.rx_beam("S", None)
        rx_cell = rm.RX_BEAM_SPACING_HPBW * rx_beam.hpbw
        tx = rm._hex_cell_points(tx_cell)
        rx = rm._hex_cell_points(rx_cell)
        for v in (rm._beam_loss_db(tx_beam, np.hypot(tx[:, 0], tx[:, 1])), rm._beam_loss_db(rx_beam, np.hypot(rx[:, 0], rx[:, 1]))):
            law = det.combine(law, det.quantiles(v, rm.OFFSET_QUANTILES), 2 * rm.OFFSET_QUANTILES)
    fl = det.Fluct(shape)
    x_req = det.required_snr_db(n, fl, pfa, 0.9, law[0], law[1], q, rm.CUM_PD_FLOOR)
    return mc.BinModel(n=n, shape=shape, threshold=det.threshold(n, pfa), x_req_db=x_req,
                       x_avail_db=x_req if x_db is None else x_db, r_bin_km=r_km, short=short,
                       pos_l=tuple(law[0]), pos_w=tuple(law[1]), pfa=pfa, tx_cell=tx_cell or 0.0, tx_beam=tx_beam,
                       rx_cell=rx_cell or 0.0, rx_beam=rx_beam, range_samples=rng_s)


def test_mc_hex_reduction():
    """Step 5.4 M1: a point reduced into the hexagonal cell is its offset to the nearest lattice site (inside the
    cell of radar_model._hex_cell_points, same for a point moved by a lattice vector)."""
    from core import montecarlo as mc
    d = 1.7
    rng = np.random.default_rng(3)
    p = rng.uniform(-5, 5, (2000, 2))
    off = mc.reduce_hex(p, d)
    b = mc._basis(d)
    sites = np.array([i * b[0] + j * b[1] for i in range(-1, 2) for j in range(-1, 2)])
    dist0 = (off ** 2).sum(1)
    assert np.all(dist0[:, None] <= ((off[:, None, :] - sites[None]) ** 2).sum(2) + 1e-9)
    assert np.allclose(mc.reduce_hex(p + 2 * b[0] - 3 * b[1], d), off)


def test_mc_wilson_interval():
    """Step 5.4 G3: Wilson interval at 95 % (reference values) and its limits."""
    from core import montecarlo as mc
    lo, hi = mc.wilson(90, 100, 0.95)
    assert abs(lo - 0.8256) < 2e-4 and abs(hi - 0.9448) < 2e-4
    assert mc.wilson(0, 50)[0] == 0.0 and mc.wilson(50, 50)[1] == 1.0


def test_mc_single_look_matches_the_exact_law():
    """Step 5.4 B6: one look (q = 0 through a revisit of 0 s) of a steady and a Swerling 1 target, no position loss:
    the drawn Pd falls in the 99.9 % interval of the exact law of detection.pd_at."""
    from core import detection as det
    from core import montecarlo as mc
    for shape, sw in ((math.inf, 0), (1.0, 1)):
        bm = _mc_bin(n=4, shape=shape, x_db=13.0)
        bm.range_samples = np.zeros(1)
        rng = np.random.default_rng(7)
        hit, _ = mc.draw_targets(bm, rng, 20000, 1e9, np.zeros(20000), 0.0)
        lo, hi = mc.wilson(int(hit.sum()), len(hit), 0.999)
        assert lo <= float(det.pd_at(4, sw, 1e-6, 13.0)) <= hi


def test_mc_reproducible_and_matches_the_analytic_criterion():
    """Step 5.4 G1 + B4: the same seed gives the same draws; with every position loss drawn anew at each look (the
    analytic assumption) the cumulative Pd of targets at the required SNR is the criterion's 0.9 within its 99.9 %
    interval; with the lattice point fixed per target the draws differ (the effect M1 measures)."""
    from core import montecarlo as mc
    bm = _mc_bin(lattice=True, q=1.0)
    v = np.full(6000, 25000.0)
    a = mc.draw_targets(bm, np.random.default_rng(11), 6000, 20.0, v, 0.0, correlated=False)[0]
    b = mc.draw_targets(bm, np.random.default_rng(11), 6000, 20.0, v, 0.0, correlated=False)[0]
    assert np.array_equal(a, b)
    lo, hi = mc.wilson(int(a.sum()), len(a), 0.999)
    assert lo <= 0.9 <= hi
    an = mc.analytic_pcum(bm.curve(), 25000.0 * 20.0 / 500e3, bm.x_avail_db)
    assert abs(an - 0.9) < 0.003
    c = mc.draw_targets(bm, np.random.default_rng(11), 6000, 20.0, v, 0.0, correlated=True)[0]
    assert not np.array_equal(a, c)


def test_mc_poisson_and_quantile_intervals():
    """Step 5.4 M2: Poisson interval of a count (Byar, reference values at 95 %) and the order-statistic interval of a
    quantile (contains the true 95 % quantile of an exponential sample)."""
    from core import montecarlo as mc
    lo, hi = mc.poisson_ci(10, 0.95)
    assert abs(lo - 4.80) < 0.03 and abs(hi - 18.39) < 0.05
    assert mc.poisson_ci(0)[0] == 0.0
    x = np.random.default_rng(5).exponential(1.0, 4000)
    q, qlo, qhi = mc.quantile_ci(x, 0.95)
    assert qlo <= math.log(20.0) <= qhi and qlo <= q <= qhi


def test_mc_greedy_matches_the_scheduler_rule():
    """Step 5.4 M2-1: the drawn timeline serves by priority then deadline without preemption, as scheduler._timeline:
    a P2 task released during a long P4 dwell waits for its end, then goes before the next P4."""
    from core import montecarlo as mc
    tasks = [(0.0, (4, 1), 10.0, 1.0), (0.5, (2, 0), 1e9, 0.1), (0.2, (4, 1), 10.0, 1.0)]
    d, pauses = mc._greedy(tasks, math.inf, 0.0)
    assert np.allclose(d, [0.0, 0.5, 0.9], atol=1e-12) and pauses == 0
    d, pauses = mc._greedy([(0.0, (4, 1), 9.0, 1.0), (0.0, (4, 1), 9.0, 1.0)], 1.5, 0.5)
    assert pauses == 1 and abs(d[1] - 1.5) < 1e-12


def test_mc_confirmation_matches_the_analytic_success():
    """Step 5.4 M2-4: fed with the analytic detected-SNR quantiles and independent looks (T0 = 0), the drawn KU1+7 and
    KU7 successes fall in the 99.9 % interval of tracking.ConfirmStats (the same Gaussian errors, averaged)."""
    from core import montecarlo as mc
    from core import tracking as trk_
    cl = trk_.cluster_variants()[0]
    st = trk_.ConfirmStats(1, 1, 2.8, 14.0, None, cl)
    snr = np.tile(np.asarray(st.x_det), 3000)
    for mode, m_db in (("KU1+7", 3.0), ("KU7", 3.0)):
        spec = mc.ConfirmSpec(sid="T", mode=mode, margin_db=m_db, success_analytic=0.0, p_min=0.0, st=st,
                              t0_s_s=0.0, t0_ku_s=0.0, trip_s=1.0)
        r = spec.draw(snr, np.random.default_rng(2))
        lo, hi = mc.wilson(int(round(r["p_mc"] * r["detections"])), r["detections"], 0.999)
        assert lo <= st.ku(mode, m_db)[0] <= hi, (mode, r["p_mc"], st.ku(mode, m_db)[0])


def test_mc_batch_kalman_reaches_the_steady_covariance():
    """Step 5.4 M3: the batch Kalman of the Monte-Carlo, fed every period with the allowed angle measurement of a
    bracket, settles to the steady predicted cross-range error of trackmodel.TrackState (RMS of the drawn errors within
    5 %)."""
    from core import montecarlo as mc
    from core import trackmodel as tm
    tgt = mo.sector_target({"id": "T", "rcs_m2": 1.0, "swerling": 1})
    st = tm.TrackState(tgt, 1.0, 500.0, 0.2, None, 5.0, 14.0, 0.0)
    rng = np.random.default_rng(4)
    n, f, q = 4000, st.f, st.q
    sq = mc._sqrtm(q)
    s_c = st.allowed["theta"] * st.r_m
    truth = rng.standard_normal((n, 3)) * [s_c, 1.0, 0.1]
    est, cov = np.zeros((n, 3)), np.tile(np.diag([s_c ** 2, 1.0, 0.01]), (n, 1, 1))
    err = []
    for k in range(300):
        truth = truth @ f.T + rng.standard_normal((n, 3)) @ sq.T
        est, cov = est @ f.T, f @ cov @ f.T + q
        if k >= 200:
            err.append(truth[:, 0] - est[:, 0])
        z = truth[:, :1] + s_c * rng.standard_normal((n, 1))
        est, cov = mc._kf_update(est, cov, z, np.full((n, 1), s_c ** 2), np.array([[1.0, 0.0, 0.0]]))
    rms = float(np.sqrt(np.mean(np.square(err))))
    assert abs(rms / math.sqrt(st.p_cross[0, 0]) - 1.0) < 0.05
