---
name: radar-roadmap
description: "Agreed step-by-step roadmap (phases 0-5) to rebuild the radar model into a full mission-prep + system-validation tool; per-step status, decisions summary and renvois; STATUS line says the next step"
metadata:
  type: project
---

Final goal (user, 2026-10-20): a tool where the operator prepares a mission, validated against ALL radar system
constraints and resources (RF, scheduler, power, thermal, compute...), not just an RF budget.
Roadmap agreed by the user on 2026-10-20: follow it in order, one step at a time, one commit per step (only when
asked). Method: [[explain-before-coding]], [[ask-for-every-variable]], [[group-by-topic]], [[global-parameter-file]],
[[test-values-not-reference]]; workflow and git: [[session-workflow]].

AUTHORITATIVE DETAIL lives in the repo (French, kept up to date at every step):
- V1/Documentation/Choix_architecturaux.md: every variable (definition, value, status, rationale), formulas,
  provisional measures, open points, decision history per step (question tables).
- V1/Documentation/Feuille_de_route.md: this roadmap in French with per-step decisions and renvois.
- V1/tests/snapshots/JOURNAL.md: every refreeze of the test snapshots and its causes.
- V1/radar_system.toml (source text of each entry) and the change log at the top of V1/core/radar_model.py.
This memory is the index; if it and the repo disagree, the repo wins (then fix this file).

**Why:** the user wants the rebuild driven step by step from this list, session after session, with no loss of info.
**How to apply:** at session start read the STATUS below, then the step's section of Feuille_de_route.md; resume at
the next step; after each step update STATUS here AND the status table of Feuille_de_route.md.

## Steps (done = committed and pushed on origin/main)
Phase 0 - Foundations
- 0.1 DONE 90a5fc4: single parameter file radar_system.toml (TOML, one file + [tool.*] block, entry =
  value/unit/status V|D|?/source/date) + loader core/params.py (RADAR_PARAMS env var), model V6.0.13.
- 0.2 DONE c776688: pytest bench (frozen snapshots rel 1e-9 + physics invariants + selftest), TEST missions,
  impact report, snapshot journal.
- 0.3 DONE f205a23 + 7f6bd35: review of every value in 10 batches + [tool.*] (details: Choix_architecturaux.md).
Phase 1 - Antenna & RF
- 1.1 DONE 5b3df06 (model V6.1.0): real pattern from the TRSA build-up (core/antenna.py, cache antenna_cache.json);
  TX uniform 1.035 lambda/D_env; RX circular Taylor nbar 5 fitted per volume option (mission key rx_sll_db:
  -25/-30/-35 dB, default -25, S and Ku); eta_illum_rx derived; RX beam lattice 0.5 HPBW_RX (grid_rx 0.42 dB);
  angle = CRLB of the aperture x 1.13 (k_theta removed); Ku aperture shared. Documentation/ created edfa726.
- 1.2 DONE e6ce750 (model V6.2.0): one active-mismatch model S + Ku on TX and RX (current sheet + WAIM 45 deg,
  grating-lobe term exp(-g/0.02), worst azimuth, real mean pitch); scan limits kept locked, alert > 3 dB per pass;
  far sidelobes over the visible region (TRSA gap lobes) with per-sector "RX option held up to" flags; TX pol
  linear along e_v (D); pol purity output only; pol_loss_dual 0.5 dB kept as recombination placeholder until 1.5.
  Gap-lobe finding -> USER set trsa_gap_m 2 -> 0.5 mm.
- 1.3 DONE e242519 (model V6.3.0): GaN-on-diamond TRMs + microchannel cooling, optimistic 2050-60 values;
  ELECTRICAL face budget (P_elec x CHAIN_EFF, each look RF / PAE); PAE = pae_sat x tau/(tau+t_edge) x
  min(1, 1 - a (d - 0.10)), S 0.80 / 0.2 us / 0.05, Ku 0.65 / 0.1 us / 0.08; duty_max 0.65 on trains;
  buffer S 25 J / Ku 1.2 J, fraction 0.2 -> tau_max 2.0 / 2.08 ms (user wants very long pulses for very long range);
  dissipation output (TRM, face kW, kW/m2).
- 1.4 DONE 1feda84 (model V6.4.0, 85 tests): edge law = raised cosine with core band in GHz per band
  (S 2.74-2.86, Ku 13.7-14.3), 0.5 dB total at the edges, averaged over f +/- B/2 (exact integral); HARD rule: the
  signal must fit in the usable band (2.7 / 2.9 GHz candidates now host no signal; optimiser skips out-of-band
  frequencies); S bi_max 200 MHz V; band.<b>.true_time_delay = digital_per_trm (V; 'none' -> B <= c/(D sin theta));
  scan limit 70 deg above the break KEPT (grating-lobe skirt at the horizon, element pattern not modelled); load-time
  band consistency checks; USER: values may change later (e.g. wider band) -> code must absorb it (test widens S to
  2.6-3.0 GHz). Test sector mission_redundancy_demo/A3_HIBAND (forced 2.9 GHz) is now NO_WAVEFORM, kept as is.
- 1.5 DONE d8245d1 (model V6.5.0, 96 tests; optimistic end of every range, user): LO phase noise MOVED
  WHOLE to 2.2; sky in the SHIP frame from the mission key "sky" (later the global scenario sim,
  [[global-scenario-simulator]]): cold sky t_ant S 3.2 / Ku 2.8 K + galactic profile (worst case 8 K at 2.8 GHz
  without frame) + bodies (T_b table [sky.tb_k], scenario override) integrated on the azimuthal-mean RX pattern
  (core/sky.py, antenna.py ALGO 5); positions blinded above +10 dB; sector budget T_sys = (1 - short_tol) quantile
  of the others (user OK after the "worst point" flaw was shown); ADC 13 ENOB +10 dBm 250 Msps -> NF DERIVED 20 dB
  (I had wrongly announced 8 dB, corrected in doc 10.3 n), T_sys S 144.5 / Ku 230.8 K; dynamic range, min
  unsaturated range, AT alert; limiter 50/25 ns; isolation matrix core/isolation.py: interface_mode SYNC per band
  (cancel to revisit at 4.3), S 5th harmonic in Ku = frequency rule same face; pol GLRT loss SNR(n+1)-SNR(n), track
  0.05 dB (pol_loss_dual_db removed). Ship-structure noise -> 2.5 with masking.
Phase 2 - Signal & detection
- 2.1 Parametric waveforms: modulation, compression, range sidelobes, Doppler tolerance, range-Doppler coupling,
  eclipsing, range + Doppler ambiguity, staggered PRF. RENVOIS: (1.3) BLIND RANGE c tau / 2 judged against the
  SECTOR need (2 ms -> 300 km, a non-event at 10000 km), ECLIPSING at duty up to 0.65, intra-pulse migration (50 m
  at 25 km/s over 2 ms) and Doppler, intra-pulse phase / amplitude drift; (0.3) Taylor weighting for k_r; signal
  bandwidths above 75 MHz; TX multibeam time / frequency multiplex.
- 2.2 Processing: velocity-hypothesis banks (migration loss vs cells crossed, not flat 1 dB), straddle, CFAR, real
  cell count. MUST fix the COHERENCE model: eta_min floor 0.70 reached at 149 ms -> INTERIM radar_model.COH_T_MAX_S =
  149 ms drops longer coherent dwells (remove when coherent blocks + noncoherent are modelled); t0 0.25 s and p 2
  have NO known source; LO phase noise over the round trip (53 ms at 8000 km).
- 2.3 Detection criterion: mean vs worst-point Pd on the grid cell, cumulative Pd for a closing target (D).
- 2.4 Target models: RCS vs aspect / band, Swerling per band, manoeuvre (D); coherence time T0 PER TARGET TYPE;
  POLARISATION: scattering matrix per target type, dual-pol diversity vs Swerling.
- 2.5 Environment: background; SHIP MASKING of the fields of view from [ship] (today only eps floor ~ -55 deg);
  (D) debris / clutter / jamming; FARADAY rotation kept (only lines of sight crossing ionosphere / plasmasphere).
Phase 3 - Radar functions
- 3.1 S search rebuilt (optimizer.py). TX MULTIBEAM: every mode possible, optimiser chooses (superposition with PA
  back-off, aperture partition, time / frequency multiplex) -> replaces multibeam.l8_db / p (status ?). RENVOIS: (1.3)
  PAE vs PA BACK-OFF law (today the PAE is kept under power back-off / power scale: optimistic) and S candidate pulses
  between 0.8 and 2 ms; (0.3) fine S frequency grid 5 or 10 MHz restricted to the usable spectrum; bandwidths > 75 MHz;
  optimiser rules bin_deg / max_wf / short_tol / n_alphas / short_penalty; exact-tie defect (also 5.2); RX beam
  spacing 0.5 tunable.
- 3.2 Ku confirmation rebuilt (tracking.py), S -> Ku handover, S/Ku bias 1.1 mrad, calibration. The optimiser MUST
  choose the Ku frequency (confirmation.ku_freq_ghz 14.0 is INTERIM); REAL-TIME spectrum awareness (jammed parts of S
  and Ku bands). Widen the Ku look space (2 bandwidths, 3 PRIs, pulses only up to 400 us while tau_max Ku = 2.08 ms),
  revisit margin_db_step 0.5 dB. NOTE: the S2 "touch" is an S confirmation look outside the S role policy; TWS /
  PROMOTE roles unused.
- 3.3 Tracking: Kalman / IMM, update rate, initiation / maintenance / loss. REMIND THE USER (decided at 1.2): TX
  polarisation linear along e_v; per-dwell e_u / e_v SWITCHING (single PA + switch, ~0.1-0.2 dB more TX_chain) is an
  OPTION to evaluate for polarimetry. POLARISATION: polarimetric confirmation / decoy discrimination / classification.
- 3.4 Ancillary: in-flight calibration, TRM health, degraded mode (D).
Phase 4 - System resources
- 4.1 Electrical power: 8 MW bus, storage, transients (D). FROM 1.3 (option A, whole topic here): in-train recharge of
  the TRM buffer, TRM DC feed capacity (proposals were 1700 W S / 80 W Ku for duty 0.65), INSTANTANEOUS face draw
  during a dwell (all 4900 S TRMs at once ~8.2 MW at duty 0.65 vs 2 MW allocation; today only the RR average is
  checked), face storage. The 8 MW ship total is DISTRIBUTABLE ON DEMAND among faces (radar software manages it, also
  5.2); today one face_elec_mw for all faces and only face <= 8 MW is checked (temporary, user). Replace overhead_frac
  3 % by a real non-TX budget (digital RX / ADC ~130 kW per face, control, cooling) with 4.4.
- 4.2 Thermal (D): dissipation (computed since 1.3: ~370 W per S TRM at duty 0.65), microchannel capacity, time
  constants; CTE-matched materials required by the 0.5 mm TRSA gaps; power density ~104 kW/m2 RF average.
- 4.3 Scheduler (D): task model, priorities, RTT interleaving, S and Ku timelines, overload policy; LATENCY FROM
  SCRATCH (latency_max_frac deleted at 0.3); check frame_interleaved (D). Most structural step.
- 4.4 Compute & data (D): ADC rates, digital RX beams (~3.4 per search position since 1.1), processing load; their
  power feeds 4.1.
Phase 5 - Mission & validation
- 5.1 Enriched mission model, THREAT LIBRARY (target type per sector), confirmation.p_min as a mission setting
  (per-volume option to study).
- 5.2 Global optimiser over all resources and faces incl. the on-demand split of the 8 MW.
- 5.3 Validation engine (verdict per constraint, binding resource, relax advice); track_margin_warn.
- 5.4 Discrete-event Monte-Carlo verification.
- 5.5 Operator GUI incl. parameter-edit tab, fast estimate recalibrated (cost table at 2.8 GHz only), power_scale
  rounding (3 decimals).
- 5.6 V&V + docs.
Milestones: A = phases 0-2, B = 3, C = 4.1-4.3, D = 5.

## Open points not tied to a step (also in Choix_architecturaux.md section 10.3)
optimiser exact ties broken by float noise (3.1 / 5.2); 4 faces hard-coded in the code; rx_taper_edge "inscribed"
(D) to confirm; TRM buffer MASS ~270 kg per face at 1 J/g (no mass budget in the roadmap); element pattern isotropic
in the array factor; tile-edge mutual coupling not modelled.

## STATUS (2026-10-22)
HISTORY REWRITTEN on 2026-10-22 (all hashes above in this file are the OLD ones; the current hashes are in
Feuille_de_route.md / Choix_architecturaux §11). Steps 2.4 (008d583, V6.9.0, target library [targets.*] class means,
T0 per class, Gamma-shape fluctuation, optics, acceleration bank, OS-CFAR) and 2.5 (baeadcf, V6.10.0,
core/environment.py: ship primitives in [ship], masking, structure noise, surface clutter, jammers, Faraday, GO-CFAR;
126 tests) were done on ANOTHER machine (laptop). Rules added there: threats / environment = MEDIAN or class mean
(optimistic end only for our technology); V values never change without explicit user OK; repo neutrality
([[session-workflow]]). Repo §1.5 lists pending off-roadmap items, §1.6 working knowledge and reference timings.
2026-10-22 (this machine): ship geometry = the user's 3D model V1/assets/models/LP_NISO.obj (reference for ALL masking /
element positions), model V6.10.1: length 149.46, width 74.68, hull diam 17.22, arrays at r 5.248 (panel front faces,
tilt 25 deg confirmed by the ship designer); [ship.hull]/[ship.parts] removed; load-time checks of dims + apertures vs
tilt; radiator material region V (user). COMMITTED 9ea4985 (pushed).
STEP 3.0 (mission frames + navigation, model V6.11.0) CODED, 144 tests pass (6 min 26 on the desktop), docs updated
(Choix_architecturaux 7.14 + history, Feuille_de_route 3.0 done, JOURNAL), COMMITTED 22114fd (pushed); then 146cae9 (pushed):
parallel test run (pytest-xdist -n 6, ~2 min), atomic cache writes, step 3.0 hash in the docs. Step 3.0 content:
core/frames.py (computer layer): global "nav" = list of ONE message (q_ship_to_icrf Hamilton [w,x,y,z] ship->ICRF,
epoch, optional orbit central_body/r_km/v_kms, attitude_ref icrf|rtn for GUI angles); sector "frame" ship (absent) /
inertial / rtn / limb (limb_alt_km default 0, box only, eps from the limb) / two_dir (frame_p/s_ra/dec_deg); sky
"frame" ship|inertial; resolve() turns the mission into the ship frame once (frame_m on the sector, read by
Sphere._mask); [tool.frames] outside the fingerprint. GUI: Navigation group, sector frame, 3D view ship/ICRF.
Architecture D1-D6 in [[global-scenario-simulator]].
Sky-noise block sampling artefact (1-4 % with a hot point body vs attitude) -> 3.1, option 1 (local refinement)
validated. STEP 3.1 IN PROGRESS (framing A-E validated 2026-10-22): BLOCK A VALIDATED: 5 MHz centre grid DERIVED from
the band (f = usable_lo + k step, [f +/- B/2] inside the band and outside exclusions; freq_candidates_ghz removed);
mission key spectrum_exclusions_ghz {"S": [[lo,hi]...], "Ku": [...]} (Ku used at 3.2), guard 0 MHz; two-pass search
(20 MHz coarse over valid centres, then 5 MHz over +/-20 MHz around the best; test vs exhaustive); every free
fragment keeps its centre-most valid centre in the coarse pass; one frequency per sector; agility -> block B; sector
freq_ghz_candidates kept as forcing, checked vs band + exclusions; outputs f, occupied band, centres excluded, flag
when exclusions remove > half the valid centres. Measured: ~3 s per frequency on mission_example (3 sectors).
BLOCK B in progress: user design assumption (2026-10-22): self-defence volume = last protection vs small objects
(micrometeorites, KKV, microdebris) of ~0.01-0.1 m2, minimum range of interest 2 km (shortest real one; no real
volume has a 20 km range, B2_AUTOPROT is a test sector); ADC saturation acceptable if the unsaturated range for
0.01-0.1 m2 is ~ a few km (it is 0.7-1.2 km) -> B1 = add 5 and 10 us pulses (tau <= 13.3 us for 2 km), no peak-power
reduction rule (B10 dropped). BLOCK B VALIDATED: tau + 5, 10 us and 1.0, 1.2, 1.5 ms (2.0 ms buffer limit auto); bandwidths + 100, 150,
200 MHz, none below 10; n_pulses unchanged; modulation rule kept (NLFM if fd <= 2 % B); frequency AGILITY added for
noncoherent trains n >= 2: independent draws = min(n, floor(df / (c / 2L)) + 1), df <= usable band - B outside
exclusions, occupied band = excursion + B (A2 rule), Gamma-shape fluctuation; PRI rule kept. Dominated-candidate
pruning + runtime budget -> block E. Flag in impact report: default res_max_m 2 m pushes sectors to >= 100 MHz.
BLOCK C VALIDATED (S TX only; RX multibeam already digital, free in loss, cost = compute 4.4): modes (a2) phase-only
superposition B 2/4/8 loss 0.4/0.6/0.8 dB (one shared table in [multibeam], band-generic code), (b) aperture
partition B 2/4 (beam sqrt(B) wider per axis, RX beams x B), (c) time multiplex in the train B 2/4/8 no loss (blind
range of the whole composite pulse); NOT offered: (a1) linear superposition, (c) frequency multiplex; l8_db / p
removed; optimiser picks the mode; "no excess energy" power scale = SHORTER PULSE at saturated PA (tau' = s tau),
PAE back-off law deferred to the first step needing it. Ku multibeam -> 3.2 (written in Feuille_de_route).
Constat: with the TX-time budget + frame_interleaved, B beams ~ one beam with a B x shorter pulse: expect B = 1
mostly until 4.3. BLOCK D VALIDATED: D1 clutter factor per signal bandwidth x position; D2 progressive Doppler rejection
(Gaussian clutter Doppler closing_mps / spread vs the target's velocity-filter span, + floor doppler_floor_dbc;
coherent only); D3 required range -> surface distance d on positions hitting a surface (still flagged); D4 SPJ inside
the range: 1/R^2 = [SNR J1 + sqrt((SNR J1)^2 + 4 K E SNR N0)] / (2 K E), flag + burn-through if none holds;
D5 hot sources = sky bodies with main-lobe T_sys rise > 0.5 dB + every stand-off jammer; D6 refine blocks within
3 TX beamwidths (+ apparent radius) at every 0.2 deg raster cell; D7 rotation test with the Sun, 1 % on energy;
D5/D6 thresholds in [tool.sky]. BLOCK E VALIDATED: bin_deg 5, max_wf 4, short_tol 0.01, n_alphas 21, short_penalty 1e6 kept; tie = objectives
within 1e-9 rel -> lower energy, lower time, fewer WFs, simpler mode (B=1, no agility), narrower band, f nearest
core centre (deterministic); search-grid spacing per sector in {1.3,1.4,1.5,1.6} at the chosen f after the fine
pass; RX beam spacing 0.5 fixed until 4.4; exact dominated-candidate pruning (+ test); runtime budget
mission_example <= 60 s sequential on the desktop, test suite <= 3 min parallel (else profile/accelerate, report
before any simplification).
3.1 CODED IN 6 PASSES with a pause after each (user: tokens), NO commit before P6, snapshots refrozen only at P6,
physics tests only between passes (python -m pytest tests/test_physics.py):
P1 DONE, COMMITTED 90555d8 (user asked c,p per pass; model V6.11.1, 153 tests 1 min 49): optimizer.py grid helpers (snap, exclusions_s, with_spectrum, signal_fits, centre_grid,
free_fragments, valid/coarse/fine_centres, validate_frequencies), _best_plan two-pass evaluate()/best_of() with E6
secondary key, DP costs snapped; outputs "frequency" + flag; mission.validate_spectrum; redundancy PLAN_FIELDS +
signature; toml tool.optimizer freq_step_mhz 5 / freq_coarse_mhz 20 / freq_fine_span_mhz 20 / excluded_flag_frac
0.5 / tie_rel_tol 1e-9 (freq_candidates_ghz removed); detection.required_snr_db now lru-cached (exact; 25880 calls,
508 distinct): mission_example 91 s -> 18 s; test sector A3_HIBAND forced 2.9 -> 2.86 GHz (JOURNAL at P6); 5 new
tests; only failure = cost table stale (regenerate at P6).
S BAND WIDENED, COMMITTED b4be91e (V6.11.2, W1-W7 validated): usable 2.60-2.93 GHz (D), core 2.66-2.88, edge
1.0 dB, scan limits / bi_max 200 / pae_sat kept; top set by the 52.1 mm pitch (70 deg mismatch 0.97 dB/pass at 2.93,
2.35 at 2.95, grating visible at 3.0); 67 centres; 153 tests 2 min 19 s.
P2 CODED (uncommitted, physics tests pass except cost table): toml tau_s + 5, 10 us, 1.0/1.2/1.5 ms, sig_bw + 100/150/
200; Waveform.agility_mhz + occupied_mhz (fits / edge loss on occupied), rm.frequency_draws, look_fluctuation(agility)
(declared SW1/3 -> x min(n, draws)); optimizer agility_for (excursion (n-1) c/2L on the 5 MHz step, within band - B),
agile variant for NONCOHERENT n >= 2, prune_dominated (exact, groups of equal e/t), _wf_dict agility outputs;
lru_cache on rm.walk_loss_db / oscillator_phase_var; 4368 candidates per sector; mission_example 52 s (budget 60),
physics tests alone 3 min 07 in parallel (suite budget 3 min EXCEEDED); 4 new tests; 3 tests adapted (resolution
flag now 0.3 m, exclusion [2.70,2.80], partial sector -> no agility). Results: many sectors pick 5 us x1 at 100 MHz.
RUNTIME DECISION (user): apply X1 (drop out-of-band candidates BEFORE solve), X2 (closed-form walk loss, Si),
X3 (parallel frequencies within a sector, identical results), X5 (lighten the test suite) BEFORE P3; NOT X4; budget to
re-express in normal tool use (parallel included). P2 COMMITTED cd97216 (V6.11.3, 157 tests, 7 min 06 parallel). X1/X2/X3/X5 DONE (V6.11.4; suite ~4 min 30; mission_example 47 s seq / 42 s with X3). USER RULE: no runtime
target (3 min removed), seek gains, NEVER reduce model accuracy for speed. NEXT: P3 (multibeam C).
NEXT (after the decision): P3 (multibeam C + pruning E9 + runtime), then P3 multibeam, P4 environment D1-D4, P5 sky refine +
grid spacing E7, P6 impact / snapshots / docs / c,p, then C, D, E, then code. Earlier plan: step 3.1 (framing proposed: A freq grid, B waveform space, C multibeam, D environment per
waveform, E optimiser rules), not yet validated.
UPDATE 2026-10-23 (laptop work pulled on the desktop, repo wins over the lines above): P3 b0dbc09 (V6.11.5), P4 f8fa433
(V6.11.6), English design reference Documentation/design/radar_design.tex+pdf a431848, P5 INTERMEDIATE d19829e
(V6.11.7: sky refinement S1-S4, lattice spacing per sector 1.3-2.0 step 0.1 exact search). Test rhythm: full suite /
snapshots / impact report every SECOND pass (snapshot tests fail until P6). NEXT: user decision on D7 (Sun rotation
test 1 % not met: 2.9 % budget / 2.0 % blinded on the 0.2 deg raster; 0.1 deg -> 1.7 / 1.3 %; test xfail), then P6
(impact, snapshots, full suite, docs incl. radar_design.tex, "c, p").
D7 SETTLED 2026-10-23 (user, option 3): every refined cell split into 8 x 8 points (tool.sky.refine_subsamples = 8,
new meaning: points counted separately in blinding AND budget quantile; edge-only split tried first: fixed the blinded
share, not the budget); sector_sky now returns a float "keep" share per raster cell, positions_table(keep=...).
Test 0.81 % / 0.40 % (xfail removed); drift 1.4 % at 2.9 deg = open point (o), check at 5.6. Renvoi added at 4.3/4.4/5.2:
the optimiser runs only in Pmiss today; decide what the radar computer recomputes in flight. Uncommitted. NEXT: P6.
2026-10-23 evening (desktop, uncommitted): TEST PROTOCOL validated by the user (Choix_architecturaux 1.3 "Protocole de
test"): tests/run_tests.py picks N1 (self-test + tests < 10 s) / N2 (+ testmon-affected + tests of modules reading a
changed [tool.*]) / N3 (full + cost table, impact, snapshots, journal; forced by any model value, V value, end of step,
every second pass); progress in tests/.progress.txt weighted by tests/.test_durations.json (conftest.py). Exact
speed-ups A1-A5 (caches, look cut-off, position-law cache, mean-curve 32768, per-frequency worker) mission_example
1727 -> 188 s seq; Numba sums in sky.body_rise_k (tool.sky.use_numba, numpy fallback, test N4). P5 option 3 made big
bodies (Earth limb) expensive: mission_sky_demo 1198 s seq before numba. NEXT: measure numba gain, then step 4 of the
protocol (lighten slow invariant tests, needs user OK), then P6.
2026-10-23 late: STEP 3.1 DONE (P6 coded, docs done, NOT yet committed at time of writing): snapshots refrozen
(JOURNAL), cost table regenerated, radar_design.tex/pdf updated (tectonic in the session scratchpad; download
tectonic 0.15.0 again on a new session). N3 interrupted on test_snapshots[mission_frames] (>45 min sequential under
pytest); 11/12 snapshots verified; test_false_alarms_count_independent_cells fixed (sector spacing). Before next N3:
make snapshot tests cheaper (frequency parallelism). NEXT STEP: 3.2 Ku confirmation (start with framing).
P6 COMMITTED 4bf7d64 (pushed) 2026-10-23. Step 3.1 DONE. NEXT: 3.2 framing.
STEP 3.2 framing + blocks A-E ALL VALIDATED 2026-10-23 (detail in Choix_architecturaux 11, step 3.2). Coding passes Q1 (Ku freq) .. Q6 (impact/snapshots at user's N3). NEXT: Q1 on user's go.
3.2 Q1-Q5 CODED 2026-10-23 night (V6.12.0, core/tracking.py rebuilt, uncommitted, NOT run yet). Next: ask user to run self-test + new 3.2 unit tests (N1-type), fix, then Q6 at user's N3 slot.
3.2 Q1-Q5 TESTED 2026-10-24 (171 passed, cost-table test fails as expected, snapshots not run) and COMMITTED as intermediate. RESUME AT Q6: first (a) profile one mission for exact speed-ups (ask before running), then cost table, impact, snapshots refreeze, docs at the user's N3.
2026-10-24 STOP POINT: Q6 (a) profiling DONE (mission_example 533 s profiled: S search 361 s = solve x985k; Ku 172 s = s2_vec 99 + _LookSpace 52). Exact speed-ups E1-E4 PROPOSED, none coded; detail in Choix_architecturaux 11 step 3.2 'Profilage'. RESUME: ask the user which of E1-E4 to code (docs edits uncommitted until user says c, p).
2026-10-25: user kept the roadmap order (single attitude until 5.3) after asking whether results hold for one ship
orientation only (yes). E1-E4 ALL CODED (V6.12.1, uncommitted): rm.solve_base + solve_look (solve = both +
diagnostics), optimizer._base_cached (tool.optimizer.solve_base_cache_size 16384); s2_vec gains per touch grid;
tracking._interp_grid for pd_single; tracking._ku_freq_task parallel Ku budget. 3 new tests in test_physics.py; one-off
old-vs-new compare script dump_solve.py in the session scratchpad (git archive HEAD into scratch to run the old tree).
NOTHING RUN YET. RESUME: ask the user to run the targeted tests + compare + re-profile, then Q6 N3 and docs.

2026-10-25 later: E1-E4 verified (bit-identical vs HEAD, 533 -> 329 s profiled); N3 killed: 64 GB RAM full (persistent
pool workers + unbounded caches). User REMOVED the absolute-precision rule; chose and I coded (V6.13.0, uncommitted,
NOT tested): A1 position-law rounding 0.02 dB (Pfa was already quantised 1/20 decade; the position laws were the
19k-laws driver), A2 cum tol 1e-3, A4 pol-loss cache, A5 spacing after frequency, A6/A8 3 quiet centres S/Ku,
A9 S2 tol 0.05 / 16 q / 8 GH, T1 tests/.case_cache (code fingerprint), T3 bounded caches + mo.release_memory().
Not chosen yet: A3, A7, A10. RESUME: ask to run targeted tests, measure effect (mission_example, mission_sky_demo
vs V6.12.1 outputs), then N3 (doc Choix_architecturaux 11 step 3.2).
STEP 3.3 (tracking) FRAMING OPENED 2026-10-25 while N3 runs: blocks A filter+cadence, B pointing+maintenance, C
initiation, D targets within bracket, E optimistic assumptions (unknown accel, range-Doppler coupling, attenuator),
F ALL in 3.3 (polarimetry/discrimination/Faraday/e_u-e_v switching, RCS aspect, deceptive jamming, track-linked
volume). Update period: OPERATOR FIXES it; if not achievable the tool gives the computed value as an alert and the
operator decides (accept it or move other levers). Next: block A questions (Choix_architecturaux 11, step 3.3).
3.3 block A VALIDATED 2026-10-25 (all proposals A1-A8; detail Choix_architecturaux 11 step 3.3). Next: block B.
3.3 block B VALIDATED 2026-10-25 (B1-B9 all proposals). Next: block C. Pending: user OK to measure A9 bias on S2 (S2->KU1+7 in 7 sectors at N3), then refreeze + journal + Q6 docs + commit 3.2. N3 2026-10-25: 18.7 min, 178 passed, 10 snapshot tests fail as expected.
3.3 block C VALIDATED (C1-C6). A9 S2 check: tol 0.01 still KU1+7; modes tie at 1e-4 mission load -> not a bias. Next: refreeze + journal when user OKs; block D questions.
STEP 3.2 DONE: COMMITTED aa8d27d (pushed) 2026-10-25, V6.13.0, snapshots refrozen, design tex/pdf updated (tectonic at scratchpad of session 517be718.../tectonic/tectonic.exe). To revisit: VEILLE_KV veille500 +9 % energy, probably A6 (5 centres or fine pass around best of 3). NEXT: 3.3 block D questions (already posed: D1 cost averaged over 8 ranges, D2 feasibility at worst range, D3 v_close_max, D4/D5 unchanged).
3.3 block D VALIDATED (D1 cost mean over 8 ranges, D2 feasibility at worst range, D3 v_close_max, D4/D5 unchanged). Next: block E.
3.3 block E VALIDATED (E1 accel bank +/-3 sigma pred, E2 coupling residual, E3 attenuator 30 dB / 10 dB steps after LNA, floors 0.05 mrad / 0.05 m / 0.01 m/s -- NOT the optimistic ends, flagged to user). Next: block F.
3.3 block F VALIDATED (F1-F8 all proposals); floors kept at validated values. FRAMING COMPLETE. Coding passes proposed R1 model A, R2 B+E1, R3 C+D+E2-E6, R4 F1-F5, R5 F6-F8, R6 impact/snapshots/docs. Waiting for user's go.
3.3 CODED 2026-10-25 (R1-R6, V6.14.0, uncommitted): core/trackmodel.py (Singer Kalman steady state, decoupled
requirements -> allowed meas, floors, Markov loss, initiation (criterion = operator accuracy x1.1, init meas =
min(allowed, hand-over)), discrimination 4 indices + Faraday + pol switching, separability), solve(pointing, pd),
TRACK looks bank v_span/accel given by caller, coupling residual, attenuator, aspect table, frame 'track' (+GUI),
AT budget rewritten in tracking.py (8 ranges, single vs 7-seq cluster, rr alert by bisection). 12 new tests. D
values to confirm: faraday_uncert_frac 0.2, deception_verify_rate 1, track_frame_k 3. Finding: separability ~1 for
all pairs (class means without spread). N3 running; then refreeze + journal; commit only on user's c,p.
3.3 N3 done 2026-10-25 (21 min, 190 passed, 416 values changed, refrozen, journal). Waiting for user's c,p.
STEP 3.3 COMMITTED 5f51498 (pushed) 2026-10-25, V6.14.0. NEXT (user): class RCS variability: aspect span + RCS(aspect) tables + between-object spread (rcs/depol) per class, into cost and discrimination; questions per class being posed. Still to confirm: D values faraday_uncert_frac, deception_verify_rate, track_frame_k.
Class RCS variability (aspect tables/span/spread) DEFERRED by user 2026-10-25: renvoi at end of Feuille_de_route open points. User wants architecture work first. Next: step 3.4 or phase 4 (ask).
STEP 3.4 framing 2026-10-25: scope = A calibration + B TRM errors (option c hybrid: gain loss & sidelobe floor derived; angle floor & S/Ku bias kept as inputs with consistency alert; calibration loss 0.5/0.8 dB REPLACED by derived value) + BIT cost + E integration. Degraded modes & TRM failures DEFERRED to end of development (MVP nominal). Next: block questions.
3.4 CODED 2026-10-25 (V6.15.0, uncommitted, NOT tested): [band.*.errors], calibration loss derived (~0.006 dB), array_errors, calibration_checks, cal time in budget, 2 tests. Next: ask user for tests/N3, then refreeze, c,p.
STEP 4.1 framing 2026-10-25: blocks A (8 MW split, sum check), B (non-TX consumption), C (TRM supply: waveform REMOVED if exceeded), D (face storage: per-observation AND worst burst per revisit), E (transients output). Next: block questions. (3.4 refreeze running, then c,p pending)
4.1 variables VALIDATED 2026-10-25: POOLED 8 MW (sum of face draws <= ship total), face_feeder_max_mw 8, face_elec_mw = optional operator cap (number or list of 4), optimizer keeps 2 MW reference per face; B non-TX bill (ADC 0.1 W, RX chain S 0.5/Ku 0.1 W, DBF 50 kW, control 2 kW, pumps 2% heat), C TRM supply S1700/Ku80 W (waveform removed), D storage 5 MJ/15 MW/DoD 0.8 per-obs + burst, E transient output. Next: code 4.1.
STEP 3.4 COMMITTED (see git log) 2026-10-25, V6.15.0. Next: code 4.1.
4.1 CODED 2026-10-25 (V6.16.0, uncommitted): pooled power, non-TX 106.9 kW, TRM supply check, storage checks (burst always fails -> scheduler rule max contiguous TX 0.29 s), caps in mission/GUI, 3 new tests + 2 adapted; targeted tests pass. Next: N3 (ask) then refreeze, journal, tex, c,p.
4.1 N3 done 2026-10-25, refrozen, journal written; LaTeX updated (glossary + effects table, V6.16.0). Remaining: French doc note of N3 result, commit 4.1 on user's c,p.
STEP 4.1 COMMITTED 2026-10-25 (V6.16.0). Burst storage rule kept as alert + scheduler rule (user: do not change). NEXT: 4.2 thermal framing.
STEP 4.2 framing 2026-10-25: blocks A heat balance, B junction Z_th (waveforms removed BUT quantify impact; fall back to fixed duty_max if too much), C cold plate + thermal mass + burst, D ship heat rejection pooled; E couplings DEFERRED to end of development with degraded modes. Next: block questions.
4.2 coolant (user): submer SmartCoolant derivative; 2050 values VALIDATED: cp 2.5 kJ/kgK, h 3000 W/m2K, rho 0.8, microchannel area x10, plate-fluid dT 40 K, fluid rise 10 K; C1 capacity (~1.6 MW/face) and C2 flux (~12 W/cm2) DERIVED; C3 ~190 kJ/K. Code must absorb later spec changes. Pending answers: A1-A4, B1-B6, C4-C5, D1-D2.
4.2 all variables VALIDATED 2026-10-25 (A1-A4, B1-B6, C4-C5, D1 6 MW D, D2 pooled). Coding: junction check behind a tool switch, OFF until user decides after B6 impact measurement.
4.2 CORRECTION: flux 120 W/cm2 (not 12), face cooling capacity ~15.9 MW (not 1.6) -> never limiting. B6 measured: junction Tj max ~59 C (S) / 57 C (Ku) over all candidates, ~62 C even CW -> 0 waveforms removed, no thermal duty limit. Only thermal constraint left: ship rejection 6 MW (D). Code 4.2 written (V not bumped yet), junction_check false. Asked user: keep duty_max fixed + junction check off (Tj reported)?
4.2 CODED V6.17.0 2026-10-25 (uncommitted): junction check off (user), Tj output, face heat/capacity, ship rejection 6 MW D, burst rule; docs done (tex pending). Launching N3.
STEP 4.3 framing 2026-10-25: approach (a) statistical + (b) deterministic synthetic timeline; latency req = sector key else derived from time to impact; priorities P0 calibration, P1 confirmation+initiation+self-protection volumes (checkbox flag), P2 track updates, P3_x search with per-volume sub-priority set by operator. Next: block questions.
4.2 N3 done (197 passed, 746 new-output values), refrozen, journal; ready for c,p. 4.3 block questions pending user answers.
4.3 PRIORITIES CORRECTED by user: P0 calibration, P1 self-protection volumes (checkbox), P2 confirmation+initiation, P3 track updates, P4_x search (per-volume sub-priority).
4.3 burn-through: FORBIDDEN by default, only authorised LIVE by the operator (never in Pmiss); then priority P3' (tail of P3, before P4). In Pmiss the 3.2 burn-through look stays an information output, out of budget and timeline.
STEP 4.2 COMMITTED 2026-10-25 (V6.17.0). 4.3 variables A-G ALL VALIDATED (with corrected priorities). Next: code 4.3 (core/scheduler.py).
4.3 CODED V6.18.0 2026-10-25 (uncommitted): core/scheduler.py, GUI checkbox+priority, 4 tests, docs+tex. Deviations flagged: G4 GO-CFAR advised only, G5 TDM gain not computed (nil under interleaving). Next: N3 then c,p.
4.3 bis: option (2) low-discrepancy order on the u,v lattice VALIDATED (PSCLB analysed: keep prefix-uniform principle, drop locality and its grid).
4.3 N3 done 2026-10-25 (200 passed, 103 values); fixed import-time reads (power, thermal inertia); refreeze running in background; then c,p 4.3 on user's word, then code 4.3 bis (option 2 low-discrepancy).
STEP 4.3 COMMITTED 2026-10-25 (V6.18.0). Now coding 4.3 bis.
4.3 bis CODED V6.19.0 (uncommitted): core/scanorder.py, scheduler uses real dwells, 2 tests, docs+tex. Next: N3 then c,p, then 4.4.
STEP 4.3 COMMITTED c46f965 (V6.18.0). STEP 4.3 bis (scan order: core/scanorder.py, hex lattice in (u,v), 3 shifts, bit-reversed Morton) COMMITTED 8d0353c 2026-10-25 (V6.19.0), N3 21.6 min 203 passed + refreeze. Also tools/scan_viewer_3d.py (true-scale 3D viewer, km; ship/1:1 views, key v; scale bar; Moon from sky body) + tools/scenarios/scan3d_shell_lp.json (viewer scenario, NOT a test case: keep viewer scenarios out of tests/missions, which is in the case-cache fingerprint and adds N3 time). NEXT: step 4.4 (compute & data) framing; scan outputs pointing_rate_hz / mean_step_deg are its inputs.
FdR 2026-10-25 (uncommitted): added 4.5 "Re-pointage en vol" (post 4.4: nav cadence, re-rotation of volumes / limb, rotation within a frame, sector migration between faces, track compensation, compute cost; groups the attitude renvois); added 5.7 "Fin de développement de la Pmiss" (degraded modes, TRM failures, thermal couplings, remaining open points); added Phase 6 "Simulateur global de scénario". USER DECISION: Pmiss must be FINISHED (all phase 5 incl. 5.7) BEFORE starting phase 6, so the whole Pmiss code is available to build the global sim. Jalons: D = 5.1-5.6, "Pmiss finie" = 5.7, 6 = phase 6.
STEP 4.4 framing A1-F3 ALL VALIDATED 2026-10-25 (C values must stay editable: all in [compute] of radar_system.toml). CODED V6.20.0 (uncommitted): core/compute.py (called after tracking.budget, before scheduler; scheduler latency uses compute.processing_s; scheduler.processing_s key removed; power.dbf_kw = processing power CEILING); tracking exports ku_confirm_wf / at wf_params; 3 tests; docs + tex + pdf done. Deviation flagged: sizing budgets keep the 50 kW ceiling (actual ~10 kW reported; loop -> 5.2). reselect_ops_per_entry 1e3 = D, value NOT asked yet -> ask user. Viewer default speed now 1 (real time). Next: ask before N3, then refreeze + JOURNAL, then c,p (also FdR 4.5/5.7/phase 6 + viewer changes).
STEP 4.4 COMMITTED 06d2c94 2026-10-25 (V6.20.0; with FdR 4.5 / 5.7 / phase 6 and viewer cones + per-image clipping). N3 20.7 min, 206 passed + refreeze. User: F1 accepted (sizing keeps 50 kW ceiling, loop in 5.2); reselect_ops_per_entry 1e3 kept, revisit at 5.7. OPEN: new alert, S partition waveforms need 155-171 simultaneous RX beams vs 64 (optimizer ignores beamformer capacity) -> ask user whether to constrain the optimizer now or in 5.2. Memory capacity per face not set (output only). NEXT: step 4.5 framing (re-pointing in flight).
4.4 capacities COMMITTED 7dd1577: beams S 512 / Ku 32, memory 4 TB + bandwidth 50 TB/s per face (one read per dwell), beam capacity in optimiser -> 5.2. N3 20.5 min 207 passed. STEP 4.4 DONE. NEXT: step 4.5 framing presented (A1 1 deg/s, A2 0.1 deg/s2, A3 100 Hz, A4 5 ms, A5 1 arcsec, A6 1e-4 deg/s, C2 0.05 HPBW tolerance; blocks B-F) -> waiting user answers.
STEP 4.5 values: rate 30 deg/s, accel 15 deg/s2 (user; editable in [repointing] toml, NOT mission keys -> flagged), rest as proposed. CODED V6.21.0 (uncommitted): core/repoint.py, band loss 'nav_pointing' (added at end of radar_model), trackmodel pointing += nav axis sigma, compute function 'repointing', report section, 2 tests, docs + tex + pdf. Next: ask before N3 (all snapshots change: tiny loss in every range), then c,p.
STEP 4.5 COMMITTED 1013500 (V6.21.0; N3 23.8 min + fixes: script-mode import of repoint, identity-frame test allows repointing work). Viewer (uncommitted, awaiting c,p): TX modes drawn per dwell (single / superposition / tdm / partition with sub-aperture bases + sub-aperture HPBW u/v, near field D^2/lambda), plate patches coloured per sub-beam shade, timings trace/physical/slow (key m), scenario tools/scenarios/scan3d_multi.json (5 volumes RR 20-120 s; partition 4 on DEBRIS_BELT, partition 2 + superposition 8 on SHIP_WATCH). tests/.refreeze.log was committed by mistake in 1013500: removed from index (git rm --cached) + excluded, goes in the next commit. NEXT roadmap step: 5.1.
Viewer + scan3d_multi COMMITTED 7962a3d (refreeze.log removed). Phase 4 (4.1-4.5) DONE. NEXT: step 5.1 framing (enriched mission model).
STEP 5.1 framing answers 2026-10-25: A validated; B = RCS aspect TABLE per class, 12 az (0-330) x 7 el (-90..90) = 84 values, every 30 deg (values, B2-B4 keep? and table values still to confirm); C = single phase only + mission toggle allow/forbid jammer burn-through (interpretation P3' in Pmiss to confirm); D none; E none; F ok (class choice + new keys in 5.1, rest 5.5). Viewer: waveform lines per volume + Moon surface distance (uncommitted). User set viewer default to scan3d_multi.
5.1 answers: table values from known values (D); ALL aspects possible for ALL targets (no per-class aspect span; search uses table mean over all aspects?); burn-through allowed -> Ku look budgeted + scheduled at P3' in Pmiss. User asked the workflow to reach the phase 6 global scenario sim (answered: finish 5.1-5.7 then phase 6 sub-steps 6.1-6.7; suggested 5.4 Monte-Carlo as the bridge).
5.1 P1 COMMITTED bb0e060 (intermediate, tests not run on user request; snapshots to refreeze at end of 5.1). Viewer 992f1ab (wf lines, Moon surface, Home restart, drag keeps play running). USER DECISION P2: aspect policy WORST for detection (table mean would give KKV x1.9 / SHIP x1.5 range); full 12x7 table used for discrimination spread (+ object spread, optics cone) and phase 6. Tables: KKV 0/2/5/10/15 dB at 0/15/30/60/90 deg off-nose (mirrored rear), SHIP 0->10 dB, others flat (status D). NEXT: code P2.
5.1 P2 coded (aspect tables 12x7, policy worst default / KKV frontal, spreads in discrimination). P3 coded (burn_through_allowed mission key -> budget + P3' scheduler class). USER: the Pmiss GUI is TEMPORARY; a full HMI replaces it in the global-sim phase -> keep GUI additions minimal and simple.
5.1 P4 (interim GUI) + P5 docs/tex/pdf done, model V6.22.0 (uncommitted). Next: ask before N3, then refreeze + JOURNAL + c,p.
STEP 5.1 COMMITTED a163e80 2026-10-25 (V6.22.0), N3 22.2 min 211 passed + refreeze (291 values, no range change). NEXT: step 5.2 framing (global optimiser: joint search/confirm/track, 8 MW split, FA cost in waveform choice, eclipse feedback, compute power loop, beam capacity).
STEP 5.2 framing VALIDATED: A fixed-point loop on existing optimisers, B min max face load then sum, C 8 MW pro rata of need; shortfall/tie -> tiers P0 > P1 > P2 > P3 (tracks KKV>SHIP>DECOY>DEBRIS>other) > P3' > P4_n over all faces; points 1,2,3,4,5,7,8 in 5.2, point 6 (imperfect interleaving, GO-CFAR) -> 5.4; N=3 iterations, runtime +30% max. Passes P1 alloc / P2 coupled costs / P3 loop / P4 flight tables / P5 tests+N3. P1 CODED (core/allocation.py, report key power_allocation, 1 test) uncommitted.
5.2 P2 coded (couplings hooks _eclipse_db, _fa_energy_J, beam filter; no-key outputs identical to snapshots). P3 coded: run() -> _assemble + fixed-point loop (_couplings: eclipse only above scheduler.eclipse_flag_db, FA cost only in pfa_cell; _mission_choice min max face load among 6 alternatives, _force_plan re-plan); [tool.global_opt] max_iterations 3, alternatives 6; test missions: no change, no runtime overhead. 3 tests. Uncommitted. Next P4 (flight tables output + docs) then P5 N3.
RESUME POINT (2026-10-25 end of day): step 5.2 IN PROGRESS, intermediate commit 2db355b (P1 allocation, P2 couplings, P3 fixed-point loop; tests NOT run, snapshots will fail on new keys power_allocation / global_opt until refreeze). Next: P4 = flight tables output (per sector: size of the reselection tables = candidate waveforms x scan bins, reselect cost from compute.reselect_ops_per_entry) + docs; P5 = make the budget's face energy load use the P1 allocation (need/alloc) instead of draw/8 MW feeder, LaTeX section 'Global optimiser', ask before N3, refreeze, JOURNAL, version bump V6.23.0, commit. Model version still V6.22.0. Viewer default scenario scan3d_multi; GUI is interim.
FdR 2026-10-25: added to phase 6 the requirement 'Pmiss navigation derived from the scenario' + proposed 6.1-6.7 split, and a 'Workflow de l'utilisateur' section (system -> Pmiss -> scenario -> simulation -> analysis). Uncommitted (awaiting c,p).
USER RULE (phase 6, in-flight overload): before any shedding, identify the binding resource of the face in limit; if POWER and spare exists elsewhere -> reallocate power to the limited face(s) (ship pool, feeder/operator caps); time/heat/compute are per face -> waveform reselection (energy vs time) or neighbour face in the overlap; shedding by 4.3 priorities only as last resort. Also: Pmiss hypotheses vs scenario reality (margin reuse policy to decide; report planned vs realised). Written in FdR phase 6 (uncommitted).
FdR workflow + phase 6 requirements + overload rule COMMITTED 2fa4d18. Repo clean. Resume point unchanged: step 5.2 P4 then P5.
OPEN POINT (user, later): face responsibility zones are indicative; use the face-overlap zones (two faces within scan limit) to let a neighbour face take part of a saturated face's search/track, in Pmiss (global optimiser direction assignment) and in flight (overload response before shedding). Added to FdR open points (uncommitted).
Overlap open point COMMITTED 4414a9e. Repo clean. Resume: step 5.2 P4 then P5.
STEP 5.2 DONE on the OTHER machine 2026-10-26 (V6.23.0): 1ace8da P4 margin reuse (core/margin.py, auto_alloc_max, rr_min_s) + load control section + French design doc (Documentation/design/radar_design_fr.tex/.pdf), 87d0fcd P5 flight tables (8 plans per sector), 78ba4e1 P6 verdict energy load = need/allocation, refreeze 311 values. Pulled on the desktop 2026-10-26. NEXT: step 5.3 validation engine framing (verdict per constraint, limiting resource, quantified relax advice, redundancy/overlaps; attitude envelope worst case per face; track_margin_warn).
USER RULE 2026-10-26 (margin reuse amended, not coded yet): spare goes to search by default (P4 rule), BUT on TRACK saturation (more tracks than n_targets) the extra tracks are taken if the binding resource is SHARED S/Ku (ship/face energy, heat, compute, memory, memory bw) by giving back search reuse, never below the search spec (g >= 1). N_extra = available resource / cost of one track (worst case: search exactly at spec, all spare to tracks). Extra tracks inherit their bracket's params; search margin given back with ONE common g (R3 reversed); extra tracks served by track priority KKV>SHIP>DECOY>DEBRIS>other then nearest. "Saturation gerable" = extra tracks w/o search degradation; "saturation non gerable" = beyond (not tracked, alert) OR binding resource band-specific (S/Ku time, beams) for now; out-of-spec band modes later. Open: ceiling x when auto_alloc_max = 0, per-bracket vs joint output, placement (5.2 P7 vs 5.3).
Same rule, answers: Q7 auto_alloc_max = 0 -> no reallocation at all (no manageable saturation); Q8 Pmiss output per track bracket = extra tracks alone + cumulative (all brackets saturating, priority order); Q9 computation = step 5.2 pass P7 in core/margin.py, "saturation gerable / non gerable" labels in the 5.3 verdict, flight rule written in FdR phase 6.
REALLOC RULE REDEFINED by user 2026-10-26 (supersedes the P4+ ladder idea; not coded): mission flag REALLOC PRIO SEARCH | PRIO TRACK. Both: no realloc allowed -> spec (n tracks at x, volume at y) + unused margin; manageable saturation (also allowed when realloc forbidden: uses free resource) = n+a tracks at x, volume at y, a = available/cost per extra track. PRIO SEARCH: spare -> search RR z<y; non-manageable -> search held at y, tracks n+a stretched t = x*cost(n+a at x)/remaining after search. PRIO TRACK: spare -> track cadence only (b<x), volumes stay at their spec RR; non-manageable -> tracks at x, search degraded c = y*cost(v at y)/(face resource - cost(n+a at x) - always-on, P1, P2). Search degraded in P4_n ORDER; P0 and P1 UNTOUCHABLE (operator not declaring P1 = allows degradation). Tracks stretched by CLASS ORDER (DEBRIS, DECOY, SHIP, KKV last), also in TRACK once no search is left to take. Formulas validated. "Saturation critique" (floors: track loss / latency_max_s, max search RR; shedding by priority) to define right after the rest. Open: Q12 pros/cons class order, Q14 ceiling (x vs 100%) per saturation type, Q16 droppable Ku tasks.
Answers 2026-10-26: Q12 STRICT class order for track stretching (DEBRIS to its floor, then DECOY, SHIP, KKV last); Q14 (a) the part above the ceiling x is a SAFETY RESERVE never consumed (imperfect interleaving, 5.4) -> needs a ceiling when realloc is off; Q16 Ku levers in order: drop P3' burn-through, move confirmations Ku -> S2 (S time taken from search in P4_n order under PRIO TRACK), stretch tracks by class; beyond = saturation critique. NEXT: define saturation critique.
SATURATION CRITIQUE VALIDATED 2026-10-26: mission keys realloc_mode NONE (default) | PRIO_SEARCH | PRIO_TRACK and load_ceiling (replaces auto_alloc_max, always on, default 0.80 D, recal at 5.4; above it = safety reserve). Critique = every lever exhausted (margin, search to floor, P3' dropped, confirmations to S2, tracks to floor). Floors: track t_max per class = RR where loss prob per update > p_loss_max 1 % (D) (trackmodel loss chain, n_miss_lost), capped by latency_max_s; search: rr_max_s key if given else derived (cumulative range falls to the volume's min range of interest). Shedding: PRIO_TRACK volumes P4_n (lowest sub-priority first, whole volume) -> tracks by class DEBRIS, DECOY, SHIP, KKV (farthest first) -> P2; PRIO_SEARCH tracks first then volumes then P2. P2 unreachable -> MAJOR ALERT, continue P0/P1 only, no track creation. Pmiss table per bracket adds the critique-entry track count. Open: behaviour under NONE beyond manageable.
NONE validated 2026-10-26: behaves as PRIO_SEARCH without margin reuse (non-manageable: search at y, tracks stretched by class within tracking + free resource; critique: tracks, then volumes, then P2). Realloc rule FULLY DEFINED; diagram (private, outside the repo). Not yet written in FdR / Choix_architecturaux.
DOCS UPDATED 2026-10-26 (uncommitted): Choix_architecturaux §7.15 point 10 (full realloc/saturation rule) + P5 row + point 7 note + §9.1 realloc_mode/load_ceiling + §11 5.2 P7 question table; Feuille_de_route status 5.2 (P7 added, resume 5.2 P7 then 5.3), 5.2 P7 paragraph, 5.3 renvoi (saturation state in verdict), phase 6 flight saturation rule. NEXT: step 5.2 P7 framing (code).
5.2 P7 FRAMING VALIDATED 2026-10-26: passes P7a keys (realloc_mode, load_ceiling; auto_alloc_max REMOVED, mission with it refused with a message), P7b PRIO_TRACK nominal (one factor on track cadences, per-bracket optional rr_min_s, NO default), P7c marginal track cost probe n+max(1,10% n) + manageable table (alone/cumulative), P7d floors + critique + Ku levers, P7e report/docs/tex V6.24.0 then ask N3. V4 (c): alert between load_ceiling and 1, fault above 1. Runtime <= +30 % mission_example sequential. Extra tracks spread over faces like their bracket. Pause after each pass; physics tests only between passes.
5.2 P7a CODED 2026-10-26 (uncommitted, model version NOT bumped: V6.24.0 + change log at P7e): toml [tool.global_opt] realloc_mode "NONE" + load_ceiling 0.80 (auto_alloc_max removed); margin.py MODES / MODE_DEFAULT / CEILING_DEFAULT, realloc_mode(), load_ceiling(), ceiling_flags() (V4 c alert), reuse only in PRIO_SEARCH, report keys realloc_mode/load_ceiling; mission.py validation (auto_alloc_max refused with message); optimizer adds ceiling flags to warnings (snapshots will change: new warnings) and calls reuse in PRIO_SEARCH; GUI combo Reallocation + Load ceiling; tests adapted + test_load_ceiling_alert_between_ceiling_and_one. Physics tests: only failure = stale cost table (regen at P7e). NEXT: P7b PRIO_TRACK nominal.
5.2 P7b CODED, uncommitted (2026-10-26): margin.py refactor (_model(ids, weights), _solve, _close, _back_off bisection when secant fails, toml reuse_bisections 8), reuse_tracks/scaled_tracks/_brackets (bracket keys "sector/i", _reuse_h, _e_nominal -> allocation P3/P5 split), optimizer calls reuse_tracks in PRIO_TRACK, bracket rr_min_s validation, GUI AT column "RR min"; tracking.py exact memo of (sector, Ku freq) budgets (_KU_MEMO, toml ku_memo_size 256) - gave NO gain. Trial mission_synthetic_mix PRIO_TRACK: without rr_min_s h ~950 (track period 1 ms, absurd but operator floor), Ku beams step-like -> bisection keeps 0.66 < 0.8; runtime 302 s vs 80 s NONE (12 assemblies x ~18 s, cost = cold caches in pool workers for the KV Ku jobs, 1.25 s warm sequential). Was about to test RADAR_FREQ_PARALLEL=0 when user interrupted. No tests written yet for P7b.
USER 2026-10-26: DEFAULT FLOORS for margin reuse: rr_min_search_s 1 s, rr_min_track_s 0.1 s ([tool.global_opt], operator rr_min_s overrides). Ku memo REMOVED (no gain). P7b DONE (uncommitted): synthetic_mix PRIO_TRACK 2 assemblies, 104 s vs 80 s NONE (+30 %), tracks stop at 0.1 s; 5 new tests (prio_track, default floors, bisection, track P5 tier, bracket rr_min_s); physics tests: only failure = stale cost table. NEXT: P7c marginal track cost probe + manageable saturation table. Docs (Choix §7.15 pt 7/10, §11) must mention the floors at P7e.
P7c PARTLY CODED (uncommitted): margin.saturation() (probe n+max(1, sat_probe_frac 0.1 x n) all brackets, cost per track per resource apportioned by bracket rise, extra_alone = floor(min room/cost), room from load_ceiling minus nominal at spec; beams continuous (utilisation(out, "beams_mean")) because integer peak does not move with +1 track, room from integer peak); optimizer out["saturation"] + flags (all modes, before reuse); toml sat_probe_frac; test_saturation_table_extra_tracks_alone. Trial synthetic_mix: KV/0 +37825 (F1 Ku beams), KV/1 +41477 (F1 energy), 88.6 s vs 80 (+11 %). OPEN QUESTIONS to user: cumulative definition (strict greedy priority degenerates), verification assembly at the extrapolated value (linear over 4 orders of magnitude).
Q17 (c): alone values = linear extrapolation, flagged; only the cumulative is verified (1 assembly + bisection if over). Q18 (a): cumulative = same overflow fraction p of n for every bracket (max p that fits), priority order (class then nearest) only for the rounding.
P7c DONE (uncommitted): cumulative (_cumulative: same p for all brackets, rounding by priority, 1 verification assembly, bisection if over); beams proxy = continuous peak (open_windows_mean x beams of the largest look, margin._beams; the mean under-estimated the 7-beam cluster peak -> 9 assemblies before). Trial synthetic_mix: KV/0 +5403, KV/1 +10807 alone (F1 Ku beams), cumulative p = 831 (KV/0 +4157, KV/1 +2493), verified 1st time; 122.5 s vs 80 s NONE (+53 %; probe + check = 2 assemblies; mission_example has no bracket -> 0 %). Tests: saturation test incl. cumulative + bisection; physics only failure = cost table. NEXT: P7d floors + critique + Ku levers.
P7d DONE (uncommitted, D1-D5 validated): margin.track_floors (tracking.p_loss_max 0.01 D entry; staged probe sat_stretch_track [1,2,4,8,16] with sat_stretch_stage 4: x8/x16 only for brackets not crossed, untenable periods cost 115-163 s), search_floors (rr_max_s key validated >= rr_target, else range at range_min_km via ranges_at_rr at y x sat_stretch_search [2,4,8,16]), _ku_levers (P3' + confirmations to S2, tracking alternatives now carry per-face loads "faces"), critique() (alone + cumulative p, 1/T cost approx, PRIO_TRACK freed search by one degraded assembly, verification + bisection, first_shed by mode), saturation(..., ranges). Trial synthetic_mix: t_max KV/0 3.37 s, KV/1 2.48 s; critique extra alone 18229 / 13401, cumulative p 2008 (verified), first shed: NONE tracks KV/1 farthest, PRIO_TRACK search SHIPW; search floors beyond probe x16 (all). Runtime NONE 192 s vs 80 (+140 %), PRIO_TRACK 264 s: BUDGET EXCEEDED, reported to user. 4 new tests; physics: only cost table fails. NEXT: P7e (report, docs, tex, V6.24.0, cost table regen, ask N3).
P7e DONE 2026-10-26 (uncommitted): console report sections MARGIN REUSE (PRIO_TRACK) + TRACK SATURATION; MODEL_VERSION 6.24.0 + change log; selftest check of realloc defaults (script-mode import fallback); cost table regenerated; Choix §7.15 pts 3/7/10 (implementation, approximations, runtime accepted), §8 tool.global_opt row, §9.1/9.2 keys, §11 P7 table + implementation; FdR 5.2 status (resume 5.3); radar_design_fr.tex AND radar_design.tex (EN) updated V6.24.0 incl. "Saturation de la poursuite" subsection, 241 tests; NO TeX engine on the desktop -> PDFs NOT rebuilt (laptop). Physics tests all pass. NEXT: ask user before N3, then update_snapshots + JOURNAL + c,p.
2026-10-26: MiKTeX installed, both design PDFs rebuilt at V6.24.0 (3 passes, no errors). N3 launched in background (log in scratchpad n3.log).
P7 COMMITTED db1af20 (pushed, 2026-10-26, intermediate: code + docs + PDFs, snapshots NOT refrozen; N3 running). After N3: update_snapshots + JOURNAL entry + Choix/FdR N3 note -> second commit on c,p.
N3 2026-10-26 34.6 min: 232 passed, only 9 snapshot tests failed (121 values, all additions: alternatives faces, saturation, 1 sky_demo warning). Snapshots REFROZEN, JOURNAL entry + Choix/FdR N3 notes written (uncommitted). Waiting for user "c, p" -> then STEP 5.2 fully closed, NEXT 5.3 validation engine.
STEP 5.2 FULLY CLOSED 2026-10-26: refreeze + JOURNAL + docs (N3 timing 34.6 min in Choix timings table and both tex) COMMITTED bb2e170 (pushed); repo clean. NEXT: step 5.3 validation engine framing (verdict per constraint, limiting resource, quantified relax advice, redundancy/overlaps; renvois: saturation state in verdict (5.2 P7), attitude envelope worst case per face, track_margin_warn, threat-value sensitivity (2.4)).
End of day 2026-10-26: last commit 1b7b0b5 (status line + PDF build note), pushed, repo clean. Resume (any machine): git pull, then step 5.3 framing.
2026-10-27 (laptop work pulled on the desktop, 951acdb): step 5.3 framing validated (Pmiss only, attitude envelope moved to phase 6); P1 verdict 37bcd0c (V6.25.0, core/validation.py, key "verdict", states OK/ALERTE/NOK, "KO" renamed "NOK"), P2 relax advice 8565545 (V6.26.0, verdict.advice + validation.verify, N3 + refreeze), P3 redundancies/overlaps ed15e4a (V6.27.0, N2 only, snapshots NOT refrozen). NEXT: 5.3 P4 threat sensitivity (Q8: RCS two figures frozen plan / reoptimised plan; speed and Swerling frozen plan only), then N3 + refreeze (covers P3 + P4 keys), docs, c,p -> step 5.3 closed.
2026-10-27 desktop: 5.3 P4 REMOVED (user: verdict uses given values, sector else class; operator edits the class; threat sensitivity -> "Développements futurs potentiels" list in Feuille_de_route). 5.4 framing VALIDATED (all proposals A-H, detail Choix §11 "Étape 5.4"): structure reflections OUT of model (future dev list), dense scene -> phase 6. Passes M1-M5. N3 for 5.3 closing launched (log in session scratchpad n3.log); after N3: refreeze + JOURNAL + docs + PDFs + c,p (user pre-authorised if no unexpected error), then code M1.
STEP 5.3 CLOSED 2026-10-27: 57ffac7 (pushed): refreeze after N3 (34.3 min desktop, 241 passed, 66 additions, laptop/desktop noise <= 5.7e-11), JOURNAL, docs, both PDFs (MiKTeX pdflatex x3, output dir in scratchpad). NEXT: 5.4 pass M1 (core/montecarlo.py event engine + test targets, cumulative Pd), N2 at end of M1 on user OK.
5.4 M1 CODED 2026-10-27 (V6.28.0, uncommitted, NOT run): core/montecarlo.py (BinModel from solve_base/solve_look, draw_targets vectorised, lattice point fixed per target through n_shifts lattice shifts, Wilson stop, state NOK/ALERTE/OK per sector), optimizer.run(keep_plans=True) + _plan range_km/positions, [tool.montecarlo], 4 tests at end of test_physics.py, docs Choix §11 M1 + FdR. Deviation from B1 flagged: looks periodic/vectorised in M1, event queue with M2. NEXT: ask user to run the 4 MC tests + one mission (mission_veille500), then N2.
5.4 M1 TESTED 2026-10-27: horizon decision (user): test targets count every look above pd_look_floor (no time bound; horizon_max_s for M2 timeline only). veille500 trial all OK (CONE 0.92-0.93 vs analytic 0.90: analytic prudent). N2 12.3 min 240 passed, 2 fixed (cost table regen fingerprint-only; param test imports core.montecarlo). Uncommitted; next: user c,p (intermediate) then M2.
5.4 M2 CODED + 7 MC tests pass (uncommitted, docs Choix/FdR NOT yet updated for M2). Trials veille500 (54 s) / synthetic_mix (201 s): P confirm MC 0.89-0.95 vs analytic 0.98 (causes: Ku looks within T0 share one power draw 0.98->0.95; cumulative detection at lower S SNR than design 20 dB, single-pulse CONE 15 dB -> 0.89); eclipse whole-echo drops Pd VEILLE_KV 0.907->0.873, KV 0.886 (plan holds 0 dB below eclipse flag); FA MC = Poisson of analytic (analytic 10.1-10.6 > fa_per_rr 10, Pfa quantisation?); class-wait ALERTE at microsecond level (noise). Decisions pending with user (FA state, wait state, acceptance of confirmation NOK).
M2 decisions (user 2026-10-27): FA per RR and per-class mean waits = INFO (no state); confirmation + eclipse optimism accepted as MC NOK findings, renvoi "retour sur la 3.2" (+5.2 eclipse) written in FdR 3.2 section + open points (also FA analytic 10.1-10.6 > fa_per_rr). Model V6.29.0. Docs done. N2 deferred to next code pass (user). Uncommitted. NEXT: M3 tracks (framing questions first).
5.4 M3 CODED (V6.30.0, uncommitted, docs not written): Kalman drawn per bracket; 8 MC tests pass; synthetic_mix 254 s. Finding: initiation infeasible as modelled (first Ku look after confirmation points 9.5 HPBW off: transverse prior v_close/3 = 8.3 km/s x T 1 s; analytic init_looks 1 assumes hits) -> 98.5 % of initiations lost, few steady updates (KV/0 1693, KV/1 49); survivors: loss KV/0 1.2e-2 vs analytic 1.2e-3, RMS 20-30 % above requirements. Proposed to user: renvoi to 3.3 + steady-state start option to measure steady loss/RMS. Waiting decision.
M3 decisions (user 2026-10-27): initiation chain must be as fast as possible -> MC init intervals longest keeping predicted pointing <= init_pointing_max_hpbw 0.3 (D), between RTT+processing and period, init look = budget look sized for that pointing; two populations (steady-start / hand-over); renvoi to 3.3 written (init at period infeasible, cluster law ignores pointing error, accuracy at fluctuating SNR). Code + docs done, NOT run. Next: ask to run MC tests + synthetic_mix trial, then N2, then c,p.
M3 trial 2 (fast init + steady pop, 2026-10-27, uncommitted; diag outputs pd_update_mc / pointing_rms_hpbw + snr_scaled arg added): steady KV/0 loss 1.4e-2 vs analytic 1.2e-3 (Pd 0.86, pointing RMS 0.208 vs 0.162); with design meas errors (no SNR scaling) loss 4.8e-3, Pd 0.888, pointing 0.175, angle RMS +9 % (analytic steady ignores missed updates + correlated misses); SNR-scaled accuracy explains the rest. Initiation still only 20 % (KV/0) / 51 % (KV/1) established, 22-44 looks, 3-7 s; 44 % even without SNR scaling: cause NOT identified yet (suspects: init look Pd at 0.3-lobe pointing, establish criterion unreachable when steady accuracy above requirement). Bilan given to user.
M3 init diagnosis (2026-10-27): init look exists (single, +3.5 dB); init looks Pd 0.80-0.83 at 0.30-0.33 lobe; intervals 1 ms-0.65 s (median 30 ms) << T0 Ku 0.2 s -> fades shared by consecutive looks -> 3 misses in ms -> lost; establish criterion (zone at period <= 1.1 req) unmet in 96-99 % of track-looks since steady accuracy itself above req. Options put to user: (A) establish criterion vs filter's own steady value, (B) loss rule / decorrelation (freq hop) for fast init. Instrumentation counters (i_looks, i_hits, i_fail, i_mode) added to simulate_bracket.
RETURN ON 3.3 CODED 2026-10-27 (V6.31.0, uncommitted, NOT run): H1-H11 (fast init, Ku freq hopping c/2L core band no S-harmonic, fallback wait T0, init_chain Markov w/ retries, outputs init_time_s/init_loss_prob/init_freqs/init_mode, alerts vs latency req & p_loss_max, init_pointing_max_hpbw moved to tracking.*), MC M3 hops + criterion A(b); 3 new tests; docs done. NEXT: ask to run tests + trial, then N3 + refreeze (tracking outputs change), c,p.
3.3-return trial (2026-10-27): 12 targeted tests pass; bug fixed (sector_budget param P shadows params P -> INIT_LOSS_MAX constant). synthetic_mix KV: S 5th harmonic (13.24-14.46 GHz) covers whole Ku core -> N_f = 1 -> fallback wait T0 everywhere; analytic init KV/0 4.0 looks 0.87 s loss/attempt 20 %, KV/1 2 looks 0.43 s 0.08 %; MC 29 % / 52 % established, 21 / 40 looks, 5.9 / 2.3 s, loss/attempt 0.71 / 0.48. Found: 3.2 treats the harmonic as a TIMING rule (Ku reception avoids S slots on the face), so excluding harmonic bands from hop freqs (H1 addition) is inconsistent and kills hopping -> proposed to user: drop that exclusion. Steady results unchanged. Uncommitted.
Harmonic exclusion REMOVED from init_hop_freqs (user; harmonic is a timing rule as in 3.2). Trial 3 (247 s): KV 9 / 6 hop freqs; MC established per attempt 50.9 % / 60.8 % (was 29 / 52 with wait), looks 21.6 / 36.5, 6.0 / 2.2 s; analytic 3.2 / 2 looks, 0.69 / 0.43 s, loss 1.5e-3 / 1e-9 -> analytic still far optimistic; fluctuation correlation not the dominant cause; init looks Pd ~0.8 vs 0.9 design + accuracy at fluctuating SNR suspected. User then asked HOW to make robust initiations: levers proposed A (multi-pulse agile init looks), B (pd_init 0.99 + accuracy quantile sizing), C (class prior on transverse velocity, KKV homing small), D (M-of-N + local re-search on uncertainty ellipse), E (widened TX sub-aperture first looks), F (S-band velocity first). Recommended A+B, C, D; waiting user choice + KKV prior value. Uncommitted.
Levers coded 2026-10-27 (uncommitted, V6.31.0): B tracking.pd_init 0.99 (D, my proposal for test) via tracking._init_state -> A emerges (agile trains); C class/sector key init_vt_sigma_mps, KKV 1000 m/s (user value for test), trackmodel.init_vt_sigma. Diagnosis: init look Pd drawn 0.874 vs analytic 0.900 at 0.3 lobe (quantile law compresses tail) + fragile single pulse SW~1. Trials KV established/attempt: hop only 51/61 %, A+B 63/95 %, A+B+C (scratch mission copy) 76/97.5 %; times ~6/2.3 s (criterion A(b) convergence). D (M-of-N + local re-search) and B accuracy-quantile part NOT coded. 13 targeted tests pass. Docs updated.
USER REQUIREMENT 2026-10-27: chain first detection > confirmation > established track fails <= 1e-3 (tracking.p_track_fail_max, V). Lever D coded (D1-D9 proposals: research after 1st miss, 3 sigma, <=37 beams, 2 attempts; trackmodel.research_cluster, init_chain weights/visits, tracking._research_look; MC research beams _HEX37; verdict row "track chain failure"; MC track_chain). MC fix: per-look fluctuation shape (init agile trains were drawn with update-look shape). pd_init sweep: 0.99 recommended (energy negligible ~3 W). Trial D (vt1000 copy): KV/0 established 94.5 %, KV/1 93.9 % (n_f=2), chain failure 0.109 (conf 0.949 x init 0.939) vs analytic 0.020 -> far from 1e-3; confirmation dominates. Uncommitted, docs for D not yet written.
RETURN ON 3.2 (K1-K8) CODED 2026-10-27 (uncommitted): CONFIRM_P_MIN = 1 - chain_split(0.5) x p_track_fail_max = 0.9995 (confirmation.p_min 0.98 V entry REPLACED by confirm_attempts 2 D + tracking.chain_split D); first_detection_snr_quantiles (K2); attempts in _ku_after_s / s2_vec (same angle error); FA looks x attempts (K4); round_trips doubled if first-attempt failure > 5 % (K6); MC ConfirmSpec hops over n_freqs + attempts. 25 targeted tests pass. Trial (vt1000 copy, 311 s): KV KU1+7 margin 8.5 dB, P MC 1.000 [0.9989,1] vs 0.9995; SHIPW forced KU7 impossible -> S2, MC 0.991 vs analytic 0.9995 NOK; FWD (8000 km, q 0.075, first-detection SNR ~12 dB) NO admissible mode even at 12 dB margin. Chain KV 5.7e-2 (init 0.943). Next: user decisions on FWD/SHIPW, then initiation levers.
K9 (a)+(b) + init tuning CODED 2026-10-27 (uncommitted): confirm_attempts adaptive 2..confirm_attempts_max 3, margin_db_max 12->15; init_research_attempts 2->4, pd_research 0.999, narrower init look for more hop freqs (tracking.init_look, research_state, ku_look bw_max_mhz). 25 tests pass. Trial: KV conf 1.000 [0.999,1]; SHIPW S2 now 1.000 [0.999,1] (K10 gap gone at this sample size); FWD still NO admissible confirmation; KV/0 init 99.8 %, KV/1 98.8 % (5 hop freqs); chain KV 1.23e-2 [1.07,1.51]e-2 vs req 1e-3, analytic 5e-4. Next: FWD needs K9 (c) angle-refinement look; diagnose KV/1 residual 1.2 % init loss; 1e5-sample validation.
2026-10-27 late (uncommitted): KV/1 init loss diag = target on hex cell vertex (s/sqrt3 = 0.81 lobe) at every re-search (re-centred same prediction) -> MC re-search lattice shifted (a+b)j/3 per attempt (163 -> 39 losses). K9 (c) found physically unsound (a Ku look cannot refine angle without detecting) -> user validated K9 (c'): sequential clusters x19, x37 in cluster_variants (tracking.hex_rings_xy), and K11: cluster shifted per confirmation attempt (attempt_shift), centre look kept on estimate (first version shifted centre -> KV margin 12.2 dB, fixed). Trial: FWD now KU1+7 x37 (margin 3.3 dB, FA 76 looks each!), KV 5.4 dB, SHIPW KU7 5.0 dB; MC conf 1.000 all (~3000 samples); init KV/0 99.7 %, KV/1 99.6 %; chain KV 3.6e-3 [2.8,5.7]e-3 vs req 1e-3. Next: init residual factor ~4, FWD FA Ku cost check, 1e5 validation.
END OF DAY 2026-10-27 (desktop): model V6.32.0, commit of the day = intermediate (see git log). Last fixes: research lattice shift 0,+1/3,-1/3 + beam kept on the estimate (tracking.attempt_shift; MC); diag after: KV/0 0 lost /5877, KV/1 1 /16870. Sequential cluster beams counted 1 (compute._cluster_beams). NOT done: full MC chain trial after last fix, 1e5 validation, N3 + refreeze (snapshot tests WILL FAIL), .tex/PDF since V6.27.0. Test mission copy for KV prior: tests/missions/mission_synthetic_mix.json with KV "init_vt_sigma_mps": 1000 (scratch only, recreate). RESUME (any machine): git pull, then the "Prochaine étape" row of Choix_architecturaux (5 items).
2026-10-29: phase 6 S3b (geometry) CODED, uncommitted, tests NOT run yet: G3-G9 validated by the user ("validé, go"). sim/geometry.py (states_at Hermite, Geometry: dir in ship frame, face_of face/theta/u,v, aspect, RCS S/Ku at usable band centres + per-object offset, occultation Earth/Moon, structure blockage, Sun/Earth/Moon dirs, ship_shadow), sim/ephemeris.py (Meeus 25 + 47 truncated, IAU 1976 precession), core/optimizer.face_of (Sphere uses it), Target.aspect_table_db (montecarlo delegates), [sim] tai_utc_s 37 / sun_radius_km 695700 / rcs_trunc_sigmas 2 (D), scenario key rcs_offset_db; 5 tests in tests/test_sim.py; no MODEL_VERSION bump (code moves only). Docs: Choix_architecturaux §11 point 7, Feuille_de_route. test_sim.py 33 passed (face_of scalar fix). COMMITTED 43db981 (pushed). NEXT: S4 (nav 100 Hz, sky, mission link, .npz log), then S6, S7, then 5.4 M4 / M5.
2026-10-29 (later): S4 CODED, uncommitted, tests NOT run yet: framing N1-N14 validated. sim/engine.py (Navigation 100 Hz on demand + w_ship_dps, errors only with meta.nav_errors; sky_at: Sun/Earth/Moon, galactic sim.galactic, surface Doppler, SOJ jammers; mission_at + GUI "Export mission at t..."; save/load_journal .npz, summary), sim/run.py (python -m sim.run sc.json -> <stem>.npz + <stem>.summary.json), [sim] nav_pos_sigma_m 1, nav_vel_sigma_mps 1e-3, galactic (D); meta keys nav_errors, log_nav, sun_type, earth_type; 4 tests. Docs §11 point 8. test_sim.py 37 passed. COMMITTED 0705e2d (pushed). NEXT: waypoint editor form (S5 gap) + S6 3D views + S7 playback, framing to propose.
2026-10-29 (evening): waypoint editor (gui/scenario_tab.WaypointList), S6 3D views + S7 playback (gui/sim_view.py SimView, in ScenarioTab(grid, view)), Geometry.nose_ship, [tool.display] sim_trail_s 60, sim_fps 20, sim_ship_view_r [1.2,3], sim_auto_max_cells 2e6 (D); 2 GUI tests. CODED, uncommitted; test_sim.py 39 passed, real app launched OK (labels always_visible fix after). Docs §11 point 9. COMMITTED 0f9533e (pushed). NEXT: back to 5.4 M4/M5; open UX items: zoom on ship in global view, seed field shows 0 when empty, bool fields typed as JSON (phase 6.1-6.3 complete).
2026-10-29 late: user asks done in gui/sim_view.py (uncommitted, tests not run): Earth+Moon always in global view (true places, Moon orbit, display only: propagation still 2-body+J2), zoom during playback (single-shot timer leaving 15 ms idle, Zoom +/- dolly, 'follow the ship'), Earth texture assets/textures/earth.jpg (untracked, 4.5 MB, to commit) on a UV sphere turned by GMST; Moon near side to Earth.
2026-10-29 late+: Sun textured (assets/textures/sun.jpg, untracked) at its TRUE place 1 AU (NEAR_TOL 1e-9, back-face culling; centre on Earth/Moon/Sun), lights the scene; 'centre on' combo + double-click centres on any body (OWNSHIP incl.), click = info; dark background #0b1020. Uncommitted, tests not run. Then (user O1-O5 validated): third-body gravity in sim/propagate.py (ThirdBodies, 60 s grid, sim.third_body true, sim.sun_mu_km3_s2), playback up to x100000; uncommitted.
2026-10-29 end: bodies / third-body gravity / centring / zoom / textures / Sun at 1 AU COMMITTED 504b2d1 (pushed), test_sim.py 40 passed. Commit hashes 0f9533e, 504b2d1 not yet written in Choix_architecturaux / Feuille_de_route. NEXT: 5.4 M4/M5 (or more simulator UX on request: earth_type by geography, seed field 0, bool fields).
2026-10-29 night: 5.4 M4 CODED (V6.35.0, uncommitted, tests NOT run): Q1-Q8 validated. validation.montecarlo_rows/merge_montecarlo (source MC, state recomputed), MC states ALERT (was ALERTE), montecarlo.propose_ceiling (timelines x g, bisection 6, latency upper bound + late_update_max 1 %, transmit time only, never applied), results tab Monte-Carlo button + report, tool.montecarlo. out of the cost-table fingerprint (cost table STALE: gen_cost_table needed), 2 tests in test_physics.py. KV/0 kept ALERT. Docs §11 point 10. Trials: 13 targeted tests passed, cost table regenerated (fingerprint only), MC veille500 OK 112 s, ceiling 100 % (x19, no tracks; finite-window caveat). NEXT: c,p; trial on a mission with tracks (synthetic_mix ~20 min) on request; then M5 (impact, N3, refreeze).
2026-10-29 night: MC synthetic_mix 542 s ALERT (KV/0 only), ceiling 100 % but S fills first (x37.9) while Ku at 1.5 %: common g never loads Ku (latency/updates untested). QUESTION to user: one factor per band.
2026-10-29 late night: scenarios/lune_interception.json written (OWNSHIP 100 km circ, SHIP_1 rp 100 km ra 3000 km, intercept at 900 s; 20 KKV at 300 s, dv 5000 m/s inertial + PN, hit at ~336 s); per-band load factor applied in core/montecarlo.py (propose_ceiling bands S/Ku, lowest proposed). Uncommitted, tests NOT run.
2026-10-29 late night: 3D perf done (GPU already RTX 4080; in-place actors, _Labels, one render, volumes turned not rebuilt, aspect cache): frames 22->5.6/9.1 ms, 95->9.2 ms with volumes x100. Uncommitted; test_sim / mc tests NOT run since. NEXT: tests, then MC synthetic_mix re-run with per-band ceiling, c,p.
2026-10-29 late night: ship view 1:1 option + zoom-following 1-2-5 rings (log mode bounded to [R0,R1]); test_sim 41 passed, checked on screen. MC per-band: both bands 100 % (Ku x2500): no criterion on late SEARCH dwells -> QUESTION to user: add late_search criterion + threshold. Still uncommitted: lunar scenario, per-band, M4, 3D perf, ship view.
2026-10-29 end of day: arcball also in Pmiss SceneView (built on pyvista's trackball style), celestial objects in ship view, rings equidistant/full-view/zoom-bounded (sim_max_view_km 2e13), layout of Scenario tab; test_sim 41 + 16 physics passed; ALL COMMITTED (see git log). REMIND THE USER LATER (user: 'pas décidé, rappelle plus tard'): late-search criterion for the load ceiling + its threshold. NEXT: 5.4 M5 (impact, N3, refreeze).
