#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# Radar Performance Model - V6.32.0
#
# Port of V5.4.0 to the current space AESA (4 faces, S-band surveillance +
# Ku-band confirmation/tracking, fully digital TRM architecture).
#
# Tags used in comments:  [V] locked by user | [D] derived / proposed, to confirm
#                         [?] known conflict between sources, to settle
# Every value, with its tag and source, lives in radar_system.toml (repository root).
#
# Changes V6.31.0 -> V6.32.0 (requirement of the user: the chain first detection > confirmation > established track
#   fails at most tracking.p_track_fail_max = 1e-3; return on 3.2 and 3.3 from the Monte-Carlo of 5.4, 27 Oct 2026)
#   - confirmation (3.2): CONFIRM_P_MIN = 1 - chain_split x p_track_fail_max (0.9995; confirmation.p_min removed);
#     first_detection_snr_quantiles (the SNR of the first detection of the cumulative criterion, K2); attempts
#     confirm_attempts 2 .. confirm_attempts_max 3 with the same angle error, the cluster shifted 0, +1/3, -1/3 of
#     (a + b) per attempt, the centre look on the estimate (K3, K9, K11); false alarms pay every attempt (K4); round
#     trips of the 95 % latency (K6); sequential clusters of 19 and 37 beams (K9 c'); margin_db_max 15 dB;
#     compute._cluster_beams: a sequential cluster forms one RX beam at a time.
#   - initiation (3.3): Pd tracking.pd_init 0.99 (lever B; agile trains follow, lever A); class / sector key
#     init_vt_sigma_mps (KKV 1000 m/s, lever C); local re-search after a miss (lever D: research_cluster, 3 sigma,
#     <= 37 beams, pd_research 0.999, init_research_attempts 4, lattice shifted per attempt with a beam kept on the
#     estimate); narrower initiation look for more hop frequencies (tracking.init_look); init_chain weighted.
#   - verdict: requirement row 'track chain failure'; montecarlo: track_chain, confirmation hopping and attempts,
#     per-look fluctuation shape, re-search beams; diagnostics counters.
# Changes V6.30.0 -> V6.31.0 (return on roadmap step 3.3 from the Monte-Carlo of 5.4: fast initiation, H1-H11, user,
#   27 October 2026)
#   - trackmodel.fast_initiation: each interval the longest keeping the predicted pointing error under
#     tracking.init_pointing_max_hpbw (0.3 Ku beamwidth, D), from the round trip up to the period; coast_from,
#     init_chain: misses degrade the pointing, n_miss_lost misses in a row lose an attempt, lost attempts repeat.
#   - tracking: the initiation look sized for that pointing (no 7-beam cluster), Ku frequency hopping from look to
#     look (init_hop_freqs: spacing c / 2L in the core band, outside the exclusions and the S harmonic bands), a
#     wait of T0 after a miss when one frequency only; new outputs init_time_s, init_loss_prob, init_freqs,
#     init_mode; init_looks is now the expected number with the misses and the repeated attempts; alerts when the
#     time to the track exceeds the latency requirement or the loss exceeds p_loss_max. _initiation_cost removed.
#   - montecarlo (M3): the initiation looks hop (one target power per frequency) or wait T0; established when the
#     covariance is within init_conv_frac of the accuracies the filter really holds (steady-start population).
# Changes V6.29.0 -> V6.30.0 (roadmap step 5.4, pass M3: Monte-Carlo of the tracks, Kalman filter drawn, user, 27 October 2026)
#   - core/montecarlo.py: per active-tracking bracket, tracks drawn with the Singer Kalman running (truth on three axes,
#     hand-over from the confirming Ku look of M2, updates decided at the real pointing error with the drawn SNR,
#     loss after n_miss_lost misses then a new initiation): loss per update, initiation looks, RMS accuracies against
#     the analytic chain and the requirements; [tool.montecarlo] tracks_per_bracket, updates_per_track.
#   - tracking.KEEP_MC_OBJECTS: sector_budget keeps the states and update looks of each bracket ("_mc", on demand).
# Changes V6.28.0 -> V6.29.0 (roadmap step 5.4, pass M2: Monte-Carlo of false alarms, confirmation, timelines, eclipse)
#   - core/montecarlo.py: eclipse drawn whole per echo (the plan's mean loss given back), confirmation of every detected
#     test target drawn from the SNR of the look that detected (exact Ku decisions, one power draw within T0), drawn
#     timelines per face and band (scheduler classes and rule, confirmations from drawn false alarms and detections,
#     n_rep per face): confirmation latency (95 %) and late updates against the analytic scheduler; false alarms per
#     revisit and mean waits as information; [tool.montecarlo] n_rep.
# Changes V6.27.0 -> V6.28.0 (roadmap step 5.4, pass M1: Monte-Carlo of the search test targets, user, 27 October 2026)
#   - core/montecarlo.py (on demand, python -m core.montecarlo <mission>): cumulative Pd of each search sector by
#     draws of test targets on the plan kept by the run (SNR read from the plan, position point fixed per target through
#     the lattice shifts, range straddle per look, Gamma fluctuation, exact noncentral chi-square decision), against the
#     analytic expectation; state NOK / ALERTE (model optimistic) / OK per sector; [tool.montecarlo].
#   - optimizer.run(keep_plans=True) keeps "_plan" (with the full-pulse range and the positions of each bin); the
#     outputs of a normal run do not change.
# Changes V6.26.0 -> V6.27.0 (roadmap step 5.3, pass P3: redundancies and overlaps, user, 27 October 2026)
#   - out["redundancy"] (core/redundancy.in_plan): "the plan of A holds the need of B" tested with the plans the run
#     kept (worst-range rule: A's revisit, B's target and range), with the cost of the redundant volume per face;
#     verdict["redundancy"] repeats it and a face resource in alert or failed (energy, S time) is first advised to drop
#     or trim it (kind "redundancy"; verify() applies the drop or the pieces).
#   - sectors[...]["face_overlap"] (optimizer.face_overlap): per face and neighbour, the S search a neighbour also sees
#     under its scan limit, its load on the owner and its cost on the neighbour (the plan's own cost per scan bin,
#     beyond it range_at() scaling and PositionCosts); verdict["overlap"]: the margin of the neighbour to the load
#     ceiling, the part takeable and the relief (information, plan unchanged: kind "overlap" in the advice).
#   - redundancy.find_redundancies takes the plans of a run (plans=) and any optimizer.Sphere (grid.mask).
# Changes V6.25.0 -> V6.26.0 (roadmap step 5.3, pass P2: quantified relaxation advice, user, 27 October 2026)
#   - verdict["advice"]: for every constraint in alert or failed, the lever and its value: a face resource over its
#     target (load ceiling; the value at 1 second) relieved by an item of share s > dL keeping f = 1 - dL / s of its
#     load (search range x f^(1/4), RR / f, RCS / f; tracks n f, period / f, accuracy of the driver / sqrt(f)), the
#     three largest items alone, a uniform revisit factor on the face, the confirmation alternatives (exact), the
#     operator's face cap; the energy of an uncapped face is the ship's (advised once); resources in steps and heat
#     proportional (approximate; the static part of the compute power left out); a failed requirement gets its direct
#     value (short_tol accepted or range of the short bins, P confirm or latency reached, longest tenable period,
#     tenable revisit or tracks of a power tier short).
#   - core/validation.verify(): the mission recomputed with one advice, bisection (tool.validation.verify_bisections
#     6, D) for the smallest relaxation that holds, the change doubled at most twice first; python -m core.validation
#     <mission> --verify <n>, and a "Verify" button in the results tab. No physics changed.
#
# Changes V6.24.0 -> V6.25.0 (roadmap step 5.3, pass P1: verdict, framed with the user on 27 October 2026)
#   - core/validation.py: out["verdict"], one table of constraints (sector, confirmation, track, face, ship, flight),
#     each a load = need / limit in three states: resource OK up to load_ceiling, ALERT up to 1, NOK above; spec OK up
#     to 1, NOK above (ALERT for positions short of range within short_tol and a late discrimination); transient OK up
#     to 1, ALERT above. Mission NOK as soon as one constraint is NOK; priority tier per constraint; limiting resource
#     per face and for the mission; saturation state at the specification (step 5.2 P7). No physics changed.
#   - tool.optimizer.track_margin_warn removed (its alert is the load ceiling of the verdict).
#   - Pmiss only: one navigation snapshot per mission (no attitude envelope, user, 27 October 2026; phase 6).
#
# Changes V6.23.0 -> V6.24.0 (roadmap step 5.2, pass P7: reallocation and saturation, user rule of 26 October 2026)
#   - Mission keys realloc_mode (NONE default, PRIO_SEARCH, PRIO_TRACK) and load_ceiling (default 0.80, D; above it a
#     safety reserve, alert when a face uses it) replace auto_alloc_max (refused with a message).
#   - core/margin.py: one filling for the search (PRIO_SEARCH) and the track cadence (PRIO_TRACK, volumes at their
#     revisit asked), default floors rr_min_search_s 1 s / rr_min_track_s 0.1 s (rr_min_s per volume or AT bracket
#     overrides), factors cut back by bisection when a resource is far from linear (Ku beams); the sped-up part of a
#     track bracket is tier P5 of the power allocation.
#   - Saturation table (out["saturation"]): per AT bracket the extra tracks of the manageable saturation (alone: linear
#     estimate from one probe; cumulative: same overflow fraction for every bracket, checked by one assembly), the track
#     floor t_max (loss per update at tracking.p_loss_max 1 %, D, or latency_max_s), the search floor (rr_max_s or the
#     revisit where the range falls to range_min_km, PRIO_TRACK), the entry of the critical saturation per mode (Ku
#     levers: burn-through dropped, confirmations to S2), the first task shed. Ku confirmation alternatives carry their
#     per-face loads.
#
# Changes V6.22.0 -> V6.23.0 (roadmap step 5.2: global optimiser, validated by the user on 25-26 October 2026)
#   - core/allocation.py: the ship's 8 MW shared by the faces pro rata of their needs, by priority tier on a shortfall
#     (P0 always-on ... P4_n search, P5 margin reuse); the energy load of a face in the verdict is need / allocation.
#   - Global loop in optimizer.run (at most 3 passes): eclipse and false-alarm couplings fed back into the search
#     plans, waveforms beyond the beamformer removed, mission-level choice among each sector's alternatives.
#   - core/margin.py: the free margin shortens the revisits up to the mission's auto_alloc_max (0 = off), one factor
#     for every volume, rr_min_s per volume, plan kept, false alarms frozen per second, range at the real revisit.
#   - Flight tables: up to 8 precomputed search plans per sector spread over the band (degraded ones flagged),
#     their memory and re-selection cost in the compute budget.
#
# Changes V6.21.0 -> V6.22.0 (roadmap step 5.1: enriched mission model, validated by the user on 25 October 2026)
#   - Mission settings (each defaulting to the file): global confirm_p_min, ship_rate_max_dps, ship_accel_max_dps2,
#     burn_through_allowed; per sector confirm_p_min, pd (cumulative search Pd, carried by the search criterion),
#     fa_rate_per_s; per AT bracket pd_update, p_discrimination.
#   - Target library: RCS vs presentation table per class (7 elevations x 12 azimuths, dB vs frontal), aspect policy
#     of detection (tool.targets.aspect_policy = worst; the KKV frontal: it aims at the ship), object-to-object RCS and
#     depolarisation spreads and the seeker optics cone, systematic terms of the discrimination indices.
#   - Ku burn-through in a self-protection jammer's axis budgeted (one look per revisit) and scheduled at P3' when the
#     mission allows it. Interim interface: target class, mission and volume settings.
#
# Changes V6.20.0 -> V6.21.0 (roadmap step 4.5: re-pointing in flight, validated by the user on 25 October 2026)
#   - core/repoint.py: each pointing turned into its face's (u, v) at its transmission with the latest navigation
#     (age = latency + 1 / cadence); pointing error from the attitude and rate knowledge and the unmeasured angular
#     acceleration; band loss 'nav_pointing' (two-way beam shape, at the top of the band); the per-axis error adds to
#     the filter's pointing error of a track look; sectors migrating between faces at the maximum rate (transit
#     time, load of the neighbouring face); compute function 'repointing' (per pointing and per frame mask).
#
# Changes V6.19.0 -> V6.20.0 (roadmap step 4.4: compute and data, validated by the user on 25 October 2026)
#   - core/compute.py: per face and band, the looks (S search dwells, Ku confirmations and track updates) costed in
#     operations per function (beamforming, compression, velocity / acceleration banks and Doppler, CFAR, jammer
#     nulling, tracks), transported bits (TRSA channels in S, TRSA partial beams in Ku), simultaneous RX beams and
#     buffers (open listening windows, Little), against the beamformer, link and face-input capacities; power of the
#     processing = static share of the ceiling power.dbf_kw + energy per operation and per bit; capacity from the
#     ceiling; processing latency of a detection (replaces scheduler.processing_s); cost of an in-flight waveform
#     reselection. The budgets that size the plan keep the ceiling; the consumption and the corrected draw / heat are
#     reported. Ku looks of the tracking budget now carry their numbers (ku_confirm_wf, wf_params).
#
# Changes V6.18.0 -> V6.19.0 (roadmap step 4.3 bis: scan order inside a volume, validated by the user on 25 October 2026)
#   - core/scanorder.py: the real search pointings of a volume per face on the (u, v) hexagonal lattice of its plan,
#     shifted frame to frame (3 shifts, grid dithering), ordered by bit-reversed Morton (prefix-uniform coverage at
#     every scale), each lasting the transmit time of its scan bin's waveform, grouped by positions_per_dwell.
#   - The scheduler's synthetic timeline serves these dwells in scan order (was identical mean tasks); outputs per
#     volume and face: pointings, frame transmit time, coverage uniformity at 10/25/50 % of the frame, mean step,
#     pointing rate; spread of the revisit of the directions (flag above tool.scanorder.revisit_flag_frac).
#
# Changes V6.17.0 -> V6.18.0 (roadmap step 4.3: scheduler, validated by the user on 25 October 2026)
#   - core/scheduler.py: task classes per face and band (P0 calibration, P1 self-protection volumes, P2 confirmation
#     and initiation, P3 track updates, P4_n search volumes by search_priority; P3' burn-through only in flight);
#     statistical budget (non-preemptive priority queues: mean and 95 % waits, shedding P4 then P3, P0-P2 never shed);
#     synthetic timeline per face (greedy, no preemption, fixed seed, recharge pauses of the storage and cold plate);
#     common transmit slots per band and the eclipse they cause (reported); S-harmonic rule on the Ku time;
#     confirmation latency against latency_max_s or latency_frac x the time to impact.
#   - Sector keys self_protection, search_priority, latency_max_s (interface: checkbox and spin box).
#
# Changes V6.16.0 -> V6.17.0 (roadmap step 4.2: thermal, validated by the user on 25 October 2026)
#   - [thermal]: coolant (SmartCoolant derivative, 2050: cp 2.5 kJ/kg K, h 3000 W/m2 K, 0.8 kg/L), micro-channel area
#     x10, plate-fluid dT 40 K, rise 10 K, inlet 20 C, inertia 50 L + 90 kJ/K, buffer 10 K, Tj max 225 C, ship heat
#     rejection 6 MW (D). DERIVED: admissible flux 120 W/cm2, cooling capacity ~15.9 MW per face, flow, convection
#     resistance per TRM, buffer 1.9 MJ.
#   - Junction of a TRM PA: 2-pole Foster network per band + convection (junction_temp_c, output 'junction_c'); the
#     removal of waveforms above Tj max is behind tool.thermal.junction_check (false: measured impact nil, Tj <= 63 C
#     even CW; duty_max 0.65 kept, user decision).
#   - Budget: face loop heat (transmissions + always-on electronics + pumps) against the cooling capacity (load 'heat'),
#     heat burst against the cold-plate buffer with a scheduler rule, ship heat (faces + conversion losses) against the
#     rejection reserved for the radar.
#
# Changes V6.15.0 -> V6.16.0 (roadmap step 4.1: electrical power, validated by the user on 25 October 2026)
#   - The faces SHARE the ship total (8 MW): the sum of their mean bus draws (transmission + always-on consumption +
#     pumps, through PSU and distribution) is checked against it; a face may exceed 2 MW when the others draw less,
#     up to its feeder (power.face_feeder_max_mw 8 MW) and the operator's optional cap (mission face_elec_mw: a number,
#     a list of 4, or 0 = none). The search optimiser keeps the reference 2 MW per face to weigh energy against time.
#   - overhead_frac (3 %) replaced by the always-on consumption: 2 ADCs (0.1 W) and 2 receive chains (S 0.5 W, Ku
#     0.1 W) per TRM, beamforming 50 kW, control 2 kW (106.9 kW per face), pumps 2 % of the heat; CHAIN_EFF 0.945.
#   - TRM supply (S 1700 W, Ku 80 W): a train whose mean TRM power or buffer recharge exceeds it is removed.
#   - Face storage 5 MJ (80 % usable, 15 MW discharge): costliest observation, worst burst of a revisit, peak power;
#     longest contiguous transmission given as a scheduler rule; transients (peak MW and MW/us) as outputs.
#
# Changes V6.14.0 -> V6.15.0 (roadmap step 3.4: array errors and in-flight calibration, validated by the user on
# 25 October 2026; degraded modes and TRM failures deferred to the end of development, MVP nominal)
#   - [band.*.errors]: random errors per TRM (1 deg, 0.1 dB), correlated per TRSA (0.5 deg, 0.05 dB), thermal drift per
#     TRSA (0.1 deg/min, 0.01 dB/min), calibration every 300 s lasting 10 ms (S) / 50 ms (Ku) per face.
#   - The lumped 'calibration' loss (0.5 dB S, 0.8 dB Ku) is REPLACED by the Ruze loss of the residual errors at the end
#     of a calibration interval, TX and RX (about 0.006 dB); mean sidelobe floor (about -59 dB) and boresight error
#     derived; the angle floor and the S/Ku bias are checked against them (calibration_checks).
#   - The calibration time is taken from the S and Ku transmit time of every face in the mission budget.
#
# Changes V6.13.0 -> V6.14.0 (roadmap step 3.3: active tracking, blocks A-F validated by the user on 25 October 2026)
#   - core/trackmodel.py: steady-state Kalman per axis (Singer: sigma_m^2 = a_max^2 / 3 (1 + 4 p_max - p0) + floor,
#     class time constant [targets.*].maneuver_tau_s), exact discretisation, Riccati; requirements: angular zone on the
#     predicted angle, range and radial velocity on the filtered track, turned into the largest measurement errors
#     (decoupled, prudent), floors 0.05 mrad / 0.05 m / 0.01 m/s in quadrature; the operator's period is checked at
#     every range of the bracket, the longest tenable one given as an alert.
#   - solve(): pointing law of a track look (Gaussian error on the TX and RX beams, or the cell of a 7-beam cluster),
#     Pd of the look (tracking.pd_update); a TRACK look banks over the velocity span and the acceleration its caller
#     gives (+/- 3 sigma of the predicted values; was one hypothesis and a known acceleration); range accuracy pays the
#     range-Doppler coupling residual c tau sigma_fd / 2B.
#   - tracking.py AT budget: cost averaged over 8 ranges of the bracket, single look or sequential cluster (cheaper),
#     Markov chain of the misses (track lost after 3), initiation looks from the confirmation hand-over (C1-C3) in the
#     start bracket and after each loss, attenuator (30 dB, 10 dB steps) moves the ADC saturation flag, false tracks,
#     KKV / decoy discrimination on the track looks (4 indices, Faraday, TX polarisation switching option),
#     class separability matrix, deceptive-jammer verification looks.
#   - Target RCS(aspect) table (optional) and aspect offset per class; sector frame 'track' (cap of k sigma around a
#     designation); new sector keys maneuver_tau_s, aspect_offset_deg, discriminate_against, jammer_deceptive.
#
# Changes V6.12.1 -> V6.13.0 (25 October 2026: approximations chosen by the user, the absolute-precision rule removed;
# every one is a [tool.*] setting whose exact value stays available)
#   - A1 tool.detection.pos_law_step_db 0.02: position-loss laws rounded (distinct laws / 6, required SNR within 0.003 dB
#     measured). The Pfa was already on a 1/20-decade grid (detection.quantise_pfa).
#   - A2 tool.detection.cum_snr_tol_db 1e-3 (was 1e-4): bisection of the cumulative criterion.
#   - A4 polarisation GLRT loss cached per (samples, fluctuation, Pfa): exact.
#   - A5 tool.optimizer.spacing_after_frequency: frequency searched at the default spacing, every spacing at the
#     frequency kept. A6 tool.optimizer.freq_quiet_centres 3: no S exclusion and no jammer -> 3 centres over the core
#     band. A8 tool.confirmation.ku_freq_quiet_centres 3: the same for the Ku frequencies.
#   - A9 tool.confirmation: margin_db_tol 0.05 (0.01), n_snr_quantiles 16 (32), gh_nodes 8 (12).
#   - T3 bounded caches (tool.detection.snr_cache_size / mean_curve_cache_size / position_cache_size) and
#     optimizer.release_memory(); T1 tests/testbench.py keeps each case output with the code fingerprint.
#
# Changes V6.12.0 -> V6.12.1 (roadmap step 3.2, pass Q6: exact speed-ups E1-E4 chosen by the user; no model change)
#   - E1: solve() = solve_base (waveform, frequency, scan, target, RX option, velocity span) + solve_look (Pfa,
#     criterion, jammer, lattice, T_sys) + diagnostics; the optimiser caches solve_base (tool.optimizer.
#     solve_base_cache_size) and skips the diagnostics: same operations in the same order (bit-identical, tested).
#   - E2: tracking.ConfirmStats.s2_vec keeps the TX gains of the centre look and the cluster beams per touch grid
#     (independent of the Ku margin of the bisection; bit-identical).
#   - E3: tracking.pd_single interpolates by index on the regular SNR grid instead of np.interp (rounding only, ~1e-16).
#   - E4: the Ku budget of every (sector, Ku frequency) runs in parallel processes when allowed (identical results).
#
# Changes V6.11.7 -> V6.12.0 (roadmap step 3.2: Ku confirmation rebuilt, blocks A-E, all validated by the user)
#   - Ku frequency per sector (A): centres usable_lo + k x 10 MHz, signal inside the band and outside the mission's
#     Ku exclusions, the valid ones every 50 MHz plus the centre-most of each free fragment; the (frequency, mode) pair
#     minimising the mission's most loaded face is kept (ties: nearest the core-band centre); sector key
#     ku_freq_ghz_candidates forces; confirmation.ku_freq_ghz removed. S harmonics on the Ku channel flagged with the
#     covered share (time rule for 4.3, not a frequency rule).
#   - Ku look space (B): 5 us to 2.08 ms pulses, PRI 0.5-5 ms, bands 25-200 MHz, both integrations (E5), agile variant
#     of noncoherent trains; the look is shortened at saturated PAs (no excess energy, as the search); range at a scan
#     angle = broadside range x h(f, theta) common to every waveform (exact, checked); margins solved continuously.
#   - Cluster (C): sequential, phase-only superposition of the 7 beams (loss 0.76 dB, log2 interpolation of the
#     [multibeam] table), or partition into 2 / 4 Ku sub-apertures; the cheapest kept; round trips published.
#   - S2 touch = an S waveform of role CONFIRM (S allowed_roles + CONFIRM); role PROMOTE removed (D).
#   - Confirmation statistics with the target's Gamma law at each band (E4); self-protection jammer in every Ku look,
#     Ku burn-through look in the jamming axis (E1-E2); sidelobe clutter per Ku look with the P4 model, out of the Ku
#     T_sys budget (E3).
#
# Changes V6.11.6 -> V6.11.7 (roadmap step 3.1, pass P5, IN PROGRESS: frozen outputs and impact report not refreshed)
#   - Sky refinement near hot sources (S1-S4): the raster cells of a block whose sky rise exceeds 0.5 dB, and those
#     within 3 TX beamwidths of a stand-off jammer, of a small body's centre or of a large body's limb, are evaluated
#     one by one (optimizer._refine_cells), then split into tool.sky.refine_subsamples^2 = 8 x 8 points of equal
#     weight (option 3, user), each blinded or not on its own rise and counted on its own in the budget quantile; the
#     cell is scanned for the share left (sector_sky returns that share per raster cell, positions_table counts it).
#     Rotation test (Sun, 1 %) met: 0.81 % on the budget T_sys, 0.40 % on the blinded fraction (1.05 % and 1.41 %
#     with per-cell values); a steady drift at larger angles (1.4 % at 2.9 deg) is measured, not judged (open point).
#   - Runtime, exact (outputs of mission_example identical to the character): A1 unbounded cache of the cumulative-Pd
#     inversion (detection._required_snr_db, ~1e5 laws per mission, the 8 spacings emptied the 8192 entries); A2 looks
#     whose Pd is below the floor left out before the interpolation (bit-identical); A3 unbounded caches of
#     position_losses / _fixed_position_law; A4 mean-curve cache 4096 -> 32768 (~0.5 GB); A5 the frequency workers take
#     one centre at all its spacings (shared sky and caches). mission_example 1727 s -> 188 s sequential, 146 s
#     parallel. Numba sums of sky.body_rise_k (tool.sky.use_numba; numpy path kept, ~1e-14 relative).
#   - Search lattice spacing per sector (E7 a): every spacing of search_grid.beam_spacing_options_hpbw (1.3 to 2.0 by
#     0.1) is tried at every centre frequency, the cheapest kept; solve(grid_spacing_hpbw), positions, grid losses,
#     RX beams per position, Pfa cells, the Ku budget and the redundancy follow the sector's spacing.
#
# Changes V6.11.5 -> V6.11.6 (roadmap step 3.1, pass P4: environment per waveform; user Q1-Q13, R1-R3)
#   - S sidelobe clutter per candidate and per 1-deg bin (out of the T_sys budget): ratio sigma_c / sigma_t of the
#     candidate's TX beam (full face or the sub-aperture's 2D pattern, Q3) for its range cell c / 2B, over its
#     coherent blocks, through its Doppler filter (Gaussian clutter spectrum, 1 sigma spread, folded over the PRF;
#     residual phase floor; none without coherent blocks, Q4-Q8), at the (1 - short_tol) quantile of the bin (Q2);
#     the SNR increase restoring the look's Pd averaged over the target velocities (R1 b1). A pointing no candidate can
#     clear is blinded (R3). Ku keeps the step-2.5 noise factor (Ku at 3.2).
#   - Waveform.doppler_window_db: Taylor -30 / -40 dB Doppler weighting (processing.doppler_window_options_db),
#     offered on coherent trains where a surface with a known closing speed can clutter the sector (Q5).
#   - Requirement brought back to d - guard on the pointings that meet a surface (sector key surface_guard_km, 10 km,
#     at least the resolution; Q9), in classes rounded UP on 5 % of the range (R2 c2), the cumulative criterion at
#     each class range (Q10); <= 0: no volume, not scanned.
#   - Self-protection jammer in solve() (spj_density_w_hz): S = SNR (N + J), exact quadratic in 1 / R^2
#     (jammed_range_km, burn_through_range_km; Q11-Q13, barrage, not nulled).
#   - Plan rules: with no candidate reaching the range, the longest reach wins before the cost (R4 b); a sector whose
#     requirement the self-protection jammer puts out of the search's reach is planned against the noise alone and
#     flagged for a Ku burn-through search pointed by the scheduler in the jamming axis (R5 b, step 4.3).
#
# Changes V6.11.4 -> V6.11.5 (roadmap step 3.1, pass P3: S TX multibeam; user Q1-Q5)
#   - [multibeam] l8_db / p (V5.6.0 law, definition unknown) replaced by three modes (Waveform.tx_mode, b_tx, tx_cut):
#     phase-only superposition (B 2 / 4 / 8, loss 0.4 / 0.6 / 0.8 dB, P / B per beam, PAs saturated); aperture
#     partition (halves cut 'h' or 'v', quadrants: P / B and G N_sub / N per beam, the sub-aperture's computed beam on
#     its own lattice = the full one stretched by the beamwidth ratios, x1.85 / x3.43, its grid losses from the 2D
#     pattern, core/antenna.subaperture); time multiplex (B 2 / 4 / 8, no loss: B pulses back to back, blind range,
#     buffer, duty and PAE on B tau), modelled but not searched (tool.optimizer.tx_modes, Q1: never better).
#   - Positions per dwell (cells_per_dwell) set the frame time and the cost of a position (search_position_costs).
#   - "No excess energy" by SHORTENING THE PULSE (PAs saturated): energy and TX time scale with s = (R_want / R)^4,
#     PAE of the shortened pulse exact, floor tau B >= waveform.pulse_tb_min (1, Q5 a), losses of the full pulse kept
#     (Q4, prudent); was a power scale on the energy only. Reported as "pulse_scale" (was "power_scale").
#   - Exact speed-ups: a superposition's range = its one-beam twin's x (1 / (B L))^(1/4) (no solve of its own); the
#     velocity-independent part of the position-loss law cached apart; Pfa per pulse train (not per TX mode); pruning
#     groups the candidates by cost function, the scale floor compared.
#
# Changes V6.11.3 -> V6.11.4 (roadmap step 3.1: runtime, user decision X1, X2, X3, X5; no accuracy given up)
#   - walk_loss_db in closed form (sine integral, |error| < 5e-7; the former 401-point mean differed by < 2e-5 dB).
#   - optimizer: only the candidates whose occupied band fits at a centre are solved (X1); the centres of a sector are
#     evaluated in parallel processes from the main process (X3, identical results; off inside sector workers, under
#     pytest-xdist, or with RADAR_FREQ_PARALLEL=0).
#
# Changes V6.11.2 -> V6.11.3 (roadmap step 3.1, pass P2: S waveform space)
#   - Pulses + 5, 10 us (2 km self-defence minimum range) and 1.0, 1.2, 1.5 ms; bandwidths + 100, 150, 200 MHz.
#   - Waveform.agility_mhz: carrier excursion over a noncoherent train; occupied band = B + excursion (band fit, edge
#     loss); frequency_draws = floor(df / (c / 2L)) + 1 multiply the independent samples (declared SW1/3 -> SW2/4).
#   - Exact pruning of dominated candidates (optimizer.prune_dominated); walk_loss_db / oscillator_phase_var cached.
#
# Changes V6.11.1 -> V6.11.2 (roadmap step 3.1: S band widened, values only; user W1-W7)
#   - band.S usable 2.70-2.90 -> 2.60-2.93 GHz (D): no grating-lobe limit downward; upward the 52.1 mm mean pitch sets
#     the wall (active mismatch at 70 deg: 0.97 dB per pass at 2.93 GHz = the 75 deg value accepted at 2.7 GHz,
#     2.35 dB at 2.95 GHz, grating lobe visible at 3.0 GHz). Core band 2.74-2.86 -> 2.66-2.88 GHz, edge loss 0.5 ->
#     1.0 dB (12 % relative band); scan limits, instantaneous 200 MHz and pae_sat 0.80 kept (more optimistic).
#     The 5 MHz grid has 67 centres; the self-test edge check now reads the band from the file.
#
# Changes V6.11.0 -> V6.11.1 (roadmap step 3.1, pass P1: S frequency grid; core/optimizer.py)
#   - The S centre frequencies are DERIVED from the band: usable_lo + k x tool.optimizer.freq_step_mhz (5 MHz), kept
#     where the signal [f - B/2, f + B/2] fits inside the usable band and touches none of the mission's
#     spectrum_exclusions_ghz["S"] (new global key); tool.optimizer.freq_candidates_ghz removed. Two-pass search:
#     the valid centres on freq_coarse_mhz (20 MHz) + the centre-most valid one of every free fragment, then the step
#     within +/- freq_fine_span_mhz of the best. Forced centres (sector freq_ghz_candidates) are checked at load.
#   - Ties (costs within tool.optimizer.tie_rel_tol, relative) are broken deterministically: the DP and the plan
#     keys compare snapped costs (first = simplest candidate wins), the frequency key adds energy, time, number of
#     waveforms and the distance to the core-band centre.
#   - detection.required_snr_db is cached (exact: the same few hundred laws are asked tens of thousands of times).
#
# Changes V6.10.1 -> V6.11.0 (roadmap step 3.0: frames of the volumes and navigation message; core/frames.py)
#   - Architecture truth / knowledge (Documentation/Feuille_de_route.md, step 3.0): this tool is the RADAR COMPUTER in
#     mission prep; it knows its navigation (attitude, own orbit, central body), never the truth of the targets.
#   - Mission key "nav": a list of navigation messages (one until step 5.3): q_ship_to_icrf [w, x, y, z] (Hamilton,
#     |q| - 1 within tool.frames.q_norm_tol normalised, refused beyond), optional epoch, optional orbit (central_body of
#     [sky.surface], r_km, v_kms in ICRF axes). No nav = identity attitude.
#   - Sector key "frame": ship (default when absent), inertial (ICRF), rtn, limb (box, eps counted from the limb at
#     limb_alt_km, acos((R + h_limb) / |r|) below the local horizontal), two_dir (two ICRF directions); sky key
#     "frame": ship or inertial. core/frames.resolve() turns the mission into the ship frame once, before any
#     computation; Sphere._mask tests box / cap in the sector's frame; downstream unchanged. A ship-frame mission
#     gives exactly the former results.
#   - Alert when the sky's central body differs from the navigation by more than tool.frames.sky_nav_tol_deg.
#
# Changes V6.10.0 -> V6.10.1 (ship geometry: the 3D model)
#   - The low-poly 3D model assets/models/LP_NISO.obj (ship.model_path) replaces the [ship] primitives as the reference
#     geometry of the masking, the structure noise, the element positions and the 3D view (user): exact ray-triangle
#     casting on its triangles, materials by region (radiator wings), placement in the ship frame by
#     ship.model_nose_axis and ship.model_array_station_z_m. Locked dimensions from the model: length 149.46 m, width
#     74.68 m, hull diameter 17.22 m; aperture centres at 5.248 m from the axis (front face of the radar panels).
#     Load-time checks: length and width, and each aperture on a panel front face of the model whose normal is the face
#     normal at geometry.tilt_deg (a tilt that no longer matches the model is refused). Same conclusions as 2.5: no owned
#     direction blocked (blocked directions from 71.7 deg off the face normal), radiator wings never seen.
#
# Changes V6.9.0 -> V6.10.0 (roadmap step 2.5: environment; core/environment.py, wired in core/optimizer.sector_sky)
#   - Ship structure as primitives in [ship] (hull of revolution + parts, materials), the same ones for the masking,
#     the structure noise and the 3D view. Masking: blocked fraction f of the aperture tube (near field), loss
#     -20 log(1 - f) per pass, masked above ship.block_mask_db (3 dB); with the 25 deg tilt no owned direction is
#     blocked and the radiators are never seen (hull noise only: emissivity 0.01 x 290 K).
#   - Surfaces ([sky.surface], constant gamma): shadow beyond the surface, sidelobe clutter on the iso-range ring at
#     the target range as noise (factor 1 / (1 - (C/S) SNR_req), clutter-limited at >= 1), Doppler rejection when the
#     scenario gives the surface closing band, main-lobe ADC clipping alert, range-ambiguous trains left out when a
#     surface is in the main lobe.
#   - Jammers ([jammers.*] class means): stand-off ones in the mission sky as noise through the RX pattern, sidelobe
#     nulling 40 dB (8 per face); self-protection ones (sector jammer_type): burn-through range.
#   - Non-threatening objects confirmed like targets (objects_per_rr); Faraday rotation output (no loss, dual-pol);
#     GO-CFAR option (exact Hansen-Sawyers loss) and clutter-edge flag.
#
# Changes V6.8.0 -> V6.9.0 (roadmap step 2.4: target models)
#   - Target library: one class per [targets.<NAME>] of the file (CLASS MEANS, the adversary's hardware being unknown),
#     schema checked at load, new classes in the file only: ST (reference), GENERIC (default of a sector naming no
#     class), KKV, SHIP, DEBRIS, DECOY. Per band: copolar frontal RCS, fluctuation family (SW0 / SW1/2 / SW3/4),
#     depolarisation; common: closing speed, length L, rotation omega, line-of-sight acceleration, optional seeker
#     optics. A sector names a class (target_type) and may override any value (target_for_sector); a sector without
#     class keeps its rcs_m2 / swerling (default SW1) with the GENERIC values for the rest.
#   - Coherence time per class and band, T0 = t0_k x lambda / (2 L omega), in the coherence loss exp(-(T / T0)^2) of
#     the coherent blocks (replaces the provisional t0 0.25 s of step 2.2).
#   - Fluctuation of a look as a Gamma shape (core/detection.Fluct): a declared case (k = 1, 2, n, 2n, SW0 steady), else
#     the family x min(blocks, 1 + T_obs / T0) independent samples; times the polarisation diversity of the two RX
#     channels (1 + rho)^2 / (1 + rho^2). The dual-pol receiver also collects the cross-polar energy: loss
#     "target_depolarisation" = -10 log(1 + rho).
#   - Seeker optics (KKV): frontal RCS + max(pi (2 f)^2, eta_r 4 pi A^2 / lambda^2) (direct reflection on the primary,
#     cat's-eye retro-reflection): KKV class 1.27 m2 in S, 13.8 m2 in Ku (body 0.135 m2).
#   - Acceleration: quadratic phase over a coherent block (law of the velocity mismatch, a T_b^2 / lambda) + range
#     curvature over the dwell (a T^2 / 8); mean loss "acceleration" while a_max costs <= 1 dB, else a bank of
#     acceleration hypotheses (1 dB at half a step) that multiplies the velocity hypotheses (false alarms, compute load).
#     TRACK looks: acceleration known from the track (optimistic until 3.3).
#   - Outputs: masking envelopes by a strong neighbour (range: the look's weighting; Doppler: unweighted transform,
#     -13.3 dB beyond one cell); OS-CFAR option (processing.cfar_type 'OS', rank 3/4, exact Swerling 1 loss).
#
# Changes V6.7.0 -> V6.8.0 (roadmap step 2.3: detection criterion)
#   - Exact detection laws, Swerling 0 to 4, any Pfa (core/detection.py); Shnidman kept as a cross-check.
#   - Pd of a look AVERAGED over the random position of the target (search lattice cell with the real TX / RX
#     patterns, range / Doppler straddle, offset to the velocity filter): one loss 'positions' (the SNR the average
#     costs) replaces grid_tx, grid_rx, straddle_range, straddle_doppler and the bank's mean loss.
#   - Search criterion (solve(cum=...)): detection.pd 0.9 is the CUMULATIVE probability reached at the requirement
#     range over the looks of a target closing at v_close_max, spaced v x RR, phase averaged; Ku looks: one look.
#   - False-alarm policy (detection.fa_mode): 'fa_rate' = fa_per_rr false alarms per sector and revisit, Pfa per cell
#     = rate / (positions x independent cells) in [1e-14, 1e-4]; 'pfa_cell' = detection.pfa. solve(pfa=...).
#   - Search lattice 1.04 -> 1.4 TX beamwidths (optimum of the averaged criterion; lattice shifted every frame).
#
# Changes V6.6.0 -> V6.7.0 (roadmap step 2.2: processing; optimistic end of every range, user)
#   - Coherent blocks: a COHERENT train is cut into M blocks (divisor of N) minimising SNR_req(M noncoherent blocks)
#     + the block's coherence loss; the eta_min floor and the 149 ms cap (COH_T_MAX_S) are removed. Block loss =
#     provisional target law exp(-(T / t0)^2), t0 0.25 s (per target type at 2.4) + LO phase noise (one LO, round
#     trip 4 sin^2(pi f tau_rt), photonic-class table, negligible) + residual platform motion 10 um. Doppler floor out.
#   - Migration: velocity-hypothesis bank over the sector's velocity span, step for 1 dB at half a step (intra-pulse
#     law of 2.1 + walk over the dwell), mean loss to the budget + compensation residual 0.1 dB (was 1 dB flat);
#     TRACK looks: one hypothesis. UNCOMPENSATED mode kept.
#   - Straddle computed (range grid x4: 0.05 dB at -25 dB; Doppler padding x4: 0.07 dB) and CA-CFAR loss computed
#     (1024 reference cells: 0.04 dB): the straddle 0.7 dB and CFAR 0.5 dB entries are removed.
#   - Independent cells per look (range from range_min x Doppler cells of a block x velocity hypotheses x angle) for
#     the false-alarm count of the confirmation budget (core/tracking.py). solve() takes v_span_mps.
#
# Changes V6.5.0 -> V6.6.0 (roadmap step 2.1: parametric waveforms; optimistic end of every range, user)
#   - Modulation per look ([waveform]): NLFM while f_d,max / B <= 2 % (mismatch 0.1 dB), else LFM with a Taylor nbar 5
#     receive weighting; range sidelobes = the volume's RX option. New loss "compression" (weighting or mismatch +
#     LFM Doppler loss -20 log(1 - f_d / B)) replaces the weighting part of the former DSP 2.0 dB, now split into
#     finite_precision 0.3 dB + straddle 0.7 dB (lump until 2.2). k_r = range_crlb_kr / RMS-bandwidth ratio of the
#     weighted spectrum (0.70 at -25 dB). Physical range resolution = broadening x c / 2B (x1.18 at -25 dB).
#   - Intra-pulse effects, outputs: cells crossed in the pulse n_c = 2 v tau B / c, velocity-mismatch loss (exact
#     quadratic-phase law, for the velocity bank of 2.2), velocity hypotheses for 1 dB, range-Doppler coupling bias,
#     AM/PM phase drift. No loss added (velocity bank assumed, migration lump kept until 2.2).
#   - Dwell timeline: blind range c (tau + limiter recovery) / 2; staggered (range-ambiguous) trains pay the mean
#     eclipsing loss -10 log(1 - (tau + recovery) / PRI); Doppler ambiguity output. The optimiser: blind range <=
#     the sector's range_min_km, unambiguous PRI >= 2 R / c + tau + recovery (no self-eclipsing).
#
# Changes V6.4.0 -> V6.5.0 (roadmap step 1.5: the receive chain; optimistic end of every range, user)
#   - T_sys by direction: band.<b>.rx.t_ant_k is the COLD sky (S 10 -> 3.2 K, Ku 5 -> 2.8 K); core/sky.py adds the
#     galactic rise (profile in galactic latitude / longitude, worst case 8 K at 2.8 GHz without a galactic frame) and
#     every body of the mission sky (ship frame): T_b x (A eta / D^2) x integral of the azimuthal-mean RX pattern
#     (core/antenna.py) over the body in (u, v). solve() takes t_sys_k (None = the cold-sky band value). Positions
#     above sky.blind_rise_db (10 dB) are blinded; the sector budget uses the (1 - short_tol) quantile (optimizer).
#   - ADC noise figure DERIVED from the converter (13 ENOB, +10 dBm full scale, fs = 1.25 x 200 MHz): 20.0 dB, was
#     27.5 dB entered (+6 dB of full scale cost what the 13th bit gains); T_sys cold sky S 154.3 -> 144.5 K, Ku 238.7 -> 230.8 K. Dynamic range of the chain (clipping
#     level at the antenna port, noise in dBFS, LNA compression) and the shortest unsaturated range of a target
#     (min_unsaturated_range_km); limiter recovery (receiver deaf for tau + recovery, used at step 2.1).
#   - Polarisation: the 0.5 dB placeholder is replaced by the GLRT on the unknown polarisation of the echo (both RX
#     channels, polarisation estimated over the dwell): SNR_req(n + 1) - SNR_req(n), n = noncoherent samples (0.54 dB
#     at n = 1 with the Shnidman fit, the exact value is 0.42 dB); TRACK looks: pol_track_est_loss_db 0.05 dB.
#   - Isolation between bands and faces: core/isolation.py (matrix, sync rule, S harmonic in the Ku channel).
#
# Changes V6.3.0 -> V6.4.0 (roadmap step 1.4: the band)
#   - Edge derating per band: raised cosine between the core band (core_lo_ghz / core_hi_ghz, in GHz: S 2.74-2.86,
#     Ku 13.7-14.3) and the usable edges, edge_derating_db 0.5 dB (was 1.5 dB with a core = band minus 1/6, copied
#     from the former Ku band); total (round-trip) loss, AVERAGED over the occupied spectrum [f - B/2, f + B/2]
#     (exact integral of the law).
#   - Spectrum occupancy: solve() refuses a signal that does not fit in the usable band (f +/- B/2): an S signal can
#     no longer sit at 2.7 or 2.9 GHz; the optimiser skips frequencies outside the band.
#   - True-time delay is an explicit entry (digital per TRM, S and Ku): no squint; 'none' would limit the signal
#     bandwidth to c / (D_env sin theta). S instantaneous bandwidth 200 MHz confirmed. Scan limits kept.
#   - Load-time consistency checks of the band entries (core inside usable, break inside the band, ...), so the band
#     can be widened or moved in radar_system.toml alone.
#
# Changes V6.2.0 -> V6.3.0 (roadmap step 1.3: TRM and transmit chain, GaN-on-diamond with microchannel cooling)
#   - ELECTRICAL face budget: a face has face_elec_mw x CHAIN_EFF for transmission (face_elec_avail_w, was the RF
#     power x one PAE 0.75 shared by S and Ku) and every look costs its RF energy / PAE. solve() reports the PAE, the
#     electrical average power of the dwell and the TRM dissipation; the optimiser and tracking add the heat per face.
#   - Variable PAE per band: pae_sat x tau / (tau + t_edge) x min(1, 1 - a (d - d_ref)), d = tau / PRI of a train,
#     d_ref 0.10: S 0.80 / 0.2 us / a 0.05, Ku 0.65 / 0.1 us / a 0.08 (optimistic 2050-60).
#   - duty_max 0.65 (S, Ku) checked on trains (Waveform); TRM buffer S 5 -> 25 J, Ku 0 -> 1.2 J, max fraction
#     0.5 -> 0.2: tau_max = frac E PAE_sat / P_pk - t_edge = S 2.0 ms, Ku 2.08 ms. The pulse electrical energy
#     includes the edges. In-train recharge / TRM feed moved to step 4.1 with the face bus and storage.
#   - Power back-off ("no excess energy" power scale, Ku look back-off) keeps the PAE: optimistic until the PAE vs
#     back-off law of step 3.1.
#
# Changes V6.1.0 -> V6.2.0 (roadmap step 1.2: active mismatch, grating lobes, far sidelobes, polarisation)
#   - ONE active-mismatch model for S and Ku, on TX and on RX (loss "scan_mismatch" = 2 x the per-pass loss), separate
#     from the band-edge derating: current sheet matched at band.<b>.waim_match_deg = 45 deg by a WAIM,
#     |G_waim| = |cos t - cos t_m| / (cos t + cos t_m), combined in power with the grating-lobe term
#     |G_gl| = exp(-g / aperture.grating_lobe_g0), g = lambda / d_mean - sin t - 1 (worst azimuth), g0 = 0.02.
#     Round trip: 0.26 dB at broadside, 1.12 dB at 70 deg, 2.24 dB at 75 deg / 2.85 GHz; ends the optimism left by
#     review 0.3 (gain = cos(theta) projection only). Scan limits stay locked; the margin and the loss are reported,
#     and the optimiser warns above tool.alerts.mismatch_db_per_pass (3 dB) per pass.
#   - Real mean element pitch for the grating lobes (TRSA pitch / TRMs per side): with 2 mm gaps S 52.4 mm, margin at
#     2.9 GHz / 75 deg 0.35 % (1.1 % with the nominal 52 mm).
#   - TRSA gap 2 -> 0.5 mm (user, step 1.2): the pitch break between TRSAs made gap lobes next to the grating lobe
#     (Ku -16.6 dB, S -27 dB) that broke the RX options at large scan; with 0.5 mm the S options hold over the whole
#     sector and Ku -25 dB everywhere (-30 / -35 dB to 68.5 / 58.5 deg at 14.25 GHz). Total area 13.2964 m2, envelope
#     4.1675 m, mean pitch S 52.1 / Ku 10.42 mm; positions -1.1 %, ranges unchanged (emissive area unchanged).
#   - Far sidelobes over the visible region (core/antenna.py, gap lobes of the TRSA lattice included): warning when
#     an RX option is not held there.
#   - TX polarisation linear along e_v (D); pol_purity(): Ludwig-3 cross-polar discrimination of the element, output
#     only (dual-pol RX: no SNR cost; pol_loss_dual_db 0.5 dB kept as the recombination placeholder until step 1.5).
#
# Changes V6.0.13 -> V6.1.0 (roadmap step 1.1: real radiation pattern of the octagonal TRSA array)
#   - The pattern is COMPUTED from the array build-up (core/antenna.py, cached in core/antenna_cache.json) instead of
#     HPBW = 0.886 lambda / D_env (rectangular aperture). TX uniform: HPBW 1.035 lambda / D_env (smallest cut; x 1.041,
#     diagonal 1.035), first sidelobe -15.6 dB. Search lattice on the real TX beam: positions -27 %.
#   - RX weighting = circular Taylor nbar 5 fitted on the real octagon so that the ACHIEVED peak sidelobe level is the
#     watch volume's option (aperture.rx_sll_options_db, mission key "rx_sll_db", default -25 dB, one setting for S and
#     Ku): eta_illum_rx is derived (-25: 0.873, -30: 0.773, -35: 0.693; was 0.88 entered, i.e. only -24 dB achieved).
#   - Digital RX beams formed in each TX position on a lattice of rx_beam_spacing_hpbw = 0.5 RX beamwidth: residual
#     RX grid loss 0.42 dB on search looks (was 0) and the RX beam count per position (compute load, step 4.4).
#   - Angle accuracy = Cramer-Rao bound of the full digital aperture (x_rms from the build-up) x angle_crlb_margin
#     1.13, independent of the RX weighting; replaces K_THETA 0.5 x HPBW(0.886): SNR need of an angle-driven track
#     update +1.27 dB.
#   - solve() takes rx_sll_db (None = the default option).
#
# Changes V6.0.12 -> V6.0.13 (roadmap step 0.1: single parameter file, no value changed)
#   - All constants moved to radar_system.toml (loaded by core/params.py); module constant names kept. Bands and
#     targets are built from the file. The literals left in formulas (0.886 beamwidth factor, 0.5 buffer ceiling)
#     became the parameters aperture.hpbw_factor and band.<b>.buffer_max_fraction.
#   - The mission scan window is passed explicitly (sector_area_uv / n_search_positions take eps_min_deg,
#     eps_max_deg) instead of callers overwriting SECTOR_EPS_MIN_DEG / SECTOR_EPS_MAX_DEG in this module.
#   - Review 0.3: SECTOR_RADIUS_DEG (face sector radius) is no longer an entry; it is derived as the widest band scan
#     limit (75 deg today, unchanged), so it cannot drift from band.<b>.scan_max_deg.
#   - Review 0.3: the aperture is derived from the array build-up -- bi-band TRSA tiles of 25 x 25 Ku TRMs (10.4 mm)
#     and 5 x 5 S TRMs, 196 TRSAs in an approximate octagon (rows 6..16..6), 2 mm dead gaps between TRSAs. Emissive
#     area 13.2496 m2 (was the rounded 13.25 entered by hand: ranges -0.0008 %), n_trm S 4900 / Ku 122500, envelope
#     4.190 m (unchanged), fill factor 98.61 %. The envelope stays the beamwidth width (octagon taper: step 1.1).
#   - Review 0.3: eta_ohmic 0.96 -> 1.0 (no feed in the digital V6; TX_chain / pre_LNA hold the local paths):
#     ranges +2.1 %. hpbw_factor removed from the parameter file (0.886 kept in code until step 1.1).
#   - Review 0.3: scan dependence of the gain = geometric projection cos(theta) only. Removed as a probable double
#     count: element exponent n = 1.1 (cos^0.1 = active mismatch, V5.6.0) and the V5.4.0 scan-loss law (0.6 dB at
#     70 deg). TEMPORARY and OPTIMISTIC at large scan (1.53 dB round trip at 70 deg: range +9.2 %) until roadmap
#     step 1.2 brings a single active-reflection model |Gamma(theta, f)| (WAIM, grating-lobe approach).
#   - Review 0.3: clock-jitter term of the coherence loss removed (sigma_t 20 fs: 1e-7 at 2.8 GHz, negligible).
#   - Review 0.3: K_F 0.5 -> 0.6 and RANGE_CRLB_KR 1.3 -> 0.6 (Cramer-Rao bound 0.551 x the 1.13 margin of K_THETA).
#   - Review 0.3, INTERIM: COH_T_MAX_S = t0 (-ln eta_min)^(1/p) = 149 ms; coherent dwells beyond it are no longer
#     generated (the eta_min floor credited them N tau without limit and made exact energy ties). Until step 2.2.
#   - Review 0.3: S-band T_sys derived from its receive chain ([band.S.rx], Friis at the antenna port): optimised
#     front end with median estimates -> 154.3 K (was 346.5 K entered as a block, a classic T/R module). The
#     pre-LNA loss left the signal-loss budget (it was counted twice: as a loss and inside T_sys).
#   - Review 0.3: Ku the same way -- T_sys from [band.Ku.rx] (medians, 238.7 K, was 400 K) and the V5.6.0 lumped
#     8 dB (processing only) replaced by the S posts with Ku values: DSP 2.0, CFAR 0.5, calibration 0.8,
#     S_blockage 0.3, TX_chain 0.5 dB, polarisation 0.5 dB (was 0): fixed losses 8.0 -> 4.6 dB.
#   - Review 0.3: TRM peak power specified at the PA OUTPUT (where the PAE is defined). TX_chain derived per
#     component ([band.<b>.tx] + the circulator shared with the receive chain): S 0.3 -> 0.6 dB, Ku 0.5 -> 0.95 dB.
#
# Changes V6.0.11 -> V6.0.12 (array tilt)
#   - SECTOR_TILT_DEG 20 -> 25 deg [V]: updated figure for all four arrays (ship axis vertical, lighthouse
#     model). Sector area in (u, v), the owner masks, every eps/theta formula derived from the tilt, and
#     the 3D ship model's radar-mount cone (core/geometry.py, now reads this constant instead of a
#     separate hardcoded angle) all follow automatically; only the self-test's locked numeric checks and
#     mission_detection_envelope.csv needed recomputing.
#
# Changes V6.0.10 -> V6.0.11 (confirmation and active tracking budget, core/tracking.py)
#   - DETECT_PFA 1e-6 -> 1e-8 per range-Doppler cell [V]: every detection is confirmed in Ku, and at 1e-6 the
#     ~21 500 false alarms per face and refresh cost twice the search itself (search +33 %, total / 2.3).
#   - K_THETA 0.7 -> 0.5 [V]: ML angle estimator on the full digital aperture (Cramer-Rao ~0.44 plus margin).
#   - Tracking Doppler term: observation time = the dwell (N - 1) PRI + tau, not N x PRI (a single 10 us pulse was
#     credited with a whole PRI of Doppler observation).
#
# Changes V6.0.9 -> V6.0.10
#   - Mission inputs SECTOR_EPS_MIN_DEG / SECTOR_EPS_MAX_DEG (rear/front limits of the scan zone, eps from the XY plane):
#     adjustable per mission preparation and sent to the scheduler, independent of tilt and scan limit. Default -90/+90.
#   - Working geometry: ALL arrays tilted 20 deg toward the bow (SECTOR_TILT_DEG, estimated ~22 deg on the drawing).
#     Ship axis = vertical (lighthouse model: az swept about the axis, bow = zenith). Face normals at el = +20 deg.
#     Face boundaries stay the az = +/-45 deg meridian planes; in the face frame they become |u| <= w cos t - v sin t.
#     Sector area 2.198 -> 2.069, positions per face at 2.8 GHz 4577 -> ~4308 (total 4 faces ~17 230).
#   - Beam spacing 1.1 -> 1.04 HPBW [V] (worst-case optimum): closes coverage holes at hex vertices
#     (worst point ~-4.3 dB vs -4.9 dB; mean TX grid loss 1.81 dB vs 2.02 dB).
#   - SECTOR FIX: face boundary = the planes az = +/-45 deg between adjacent faces (4 faces x 90 deg), i.e. in (u, v)
#     |u| <= sin45*sqrt(1-v^2) (2u^2+v^2<=1), intersected with the scan-limit disc. The previous straight cut |u|<=sin45
#     overshot the plane at elevation ("diabolo" in angle space): area 2.463 -> 2.198 (-10.8 %), positions 5130 -> ~4577.
#
# Changes V6.0.8 -> V6.0.9
#   - Beam spacing locked [V] at 1.1 HPBW (was 1.4): consistent with geom_v1 (5250 positions at 3.06 GHz / 1.211 deg
#     <=> ~1.1 HPBW on the truncated disc) and near the worst-case optimum (~1.04). At 2.8 GHz: ~4590 positions,
#     mean TX grid loss 2.02 dB, crossover -3.6 dB, worst point -4.9 dB (Gaussian lobe).
#
# Changes V6.0.7 -> V6.0.8
#   - Sector shape corrected: a circle truncated by two straight cuts (not an oval). In (u, v) the sector is the
#     disk u^2 + v^2 <= sin^2(75 deg) cut by |u| <= sin(45 deg) (azimuth taken as asin(u), direction-cosine
#     convention, straight cut lines). V6.0.7 used |atan2(u, w)| <= 45 deg, whose cut lines are curved.
#     Area 2.461 (was 2.193): ~2835 positions at 2.8 GHz / 1.4 HPBW (was 2526).
#
# Changes V6.0.6 -> V6.0.7
#   - Search positions counted in direction-cosine space (u, v): a planar array scans with a beam of constant
#     width in (u, v) (angular width grows as 1/cos(theta)). Sector = {theta <= SECTOR_RADIUS, |az| <= SECTOR_AZ_HALF}
#     mapped to (u, v); cell = sqrt(3)/2 x (BEAM_SPACING_HPBW x lambda*0.886/D)^2.
#     At 2.8 GHz and 1.4 HPBW: ~2520 positions (was 4425 counted in angle space, which ignored beam broadening).
#
# Changes V6.0.5 -> V6.0.6
#   - KKV closing speed locked [V] at 25 km/s (was 30): 15 km/s max LEO crossing orbital speed + 7 km/s KKV
#     terminal speed (universe: 6-7 km/s at impact, 10+ with booster) + margin. Conservative design bound.
#
# Changes V6.0.4 -> V6.0.5
#   - Ku instantaneous bandwidth locked [V] at 200 MHz (hardware ceiling, BandConfig.bi_max_mhz); Waveform
#     refuses sig_bw_mhz above it. Needs per-TRM digital true-time delay at scan. S band: not defined (0 = no limit).
#   - Waveforms stay open (future input: dynamically designed by the central computer, no fixed catalogue).
#
# Changes V6.0.3 -> V6.0.4 : LOCKED [V] by the user (no value change, tags only)
#   - Ku band 13.5-14.5 GHz (scan 75 deg up to 14.25 GHz, 70 deg above), Ku illumination 0.88/0.88.
#   - Beam spacing 1.4 HPBW (grid loss 3.28 dB mean, ~4425 positions at 2.8 GHz).
#   - All loss terms: S: DSP 2.0, CFAR 0.5, calibration 0.5, pre-LNA 0.3, Ku blockage 0.2, TX chain 0.3,
#     polarisation 0.5 (dual-pol RX), buffer 5 J/TRM (droop 0.24 dB on WF-LRS), multibeam 0.8 dB @ 8 beams,
#     grid (spacing above), migration 1.0 dB compensated; Ku: 8 dB lumped.
#
# Changes V6.0.2 -> V6.0.3 (S band; Ku keeps its lumped 8 dB)
#   - TX_chain = 0.3 dB: PA output -> radiating element inside the integrated TRM (T/R switch or circulator,
#     output match, feed). No distribution network exists (power-only + optical command in), so only this
#     local path remains; by reciprocity it equals the pre-LNA path (0.3 dB). [?] 0 dB if the 2 kW is
#     specified at the radiating port. Array active-VSWR at scan is already in the scan loss.
#   - Polarisation loss 0.5 dB [?]: single TX polarisation, dual-polarised digital RX recombined (per-TRM ADC);
#     covers axial-ratio / cross-pol degradation with scan. POL_RX_DUAL=False gives 3.0 dB (single-pol RX on a
#     depolarising target).
#   - Energy-buffer droop: per-TRM buffer E_BUF_TRM_J = 5 J [?]. Pulse electrical energy = P_pk/PAE x tau,
#     taken from the buffer (recharge during the pulse neglected: conservative), P_RF proportional to stored
#     energy. x = E_pulse/E_buf; mean power droop x/2 -> loss = -10 log10(1 - x/2). WF-LRS: x = 0.107, 0.24 dB.
#     solve() refuses x > 0.5 (buffer undersized for the pulse). Inter-pulse recharge checked by the
#     average-power budget.
#
# Changes V6.0.1 -> V6.0.2
#   - Processing losses set to 2050 / deep-space values (user decision): DSP 3.0 -> 2.0 dB
#     (weighting/mismatch ~1.0 + range/Doppler straddle ~0.7 + finite precision ~0.3),
#     CFAR 1.5 -> 0.5 dB (homogeneous, known noise floor, no clutter, large reference window).
#   - Kept as is by decision: grid loss (mean over the cell, 1.4 HPBW), migration 1 dB (velocity hypotheses
#     calibrated on orbital mechanics), T_sys 346.5 K (deep-space value).
#
# Changes V6.0.0 -> V6.0.1 (consistency pass)
#   - Beam grid: ONE spacing parameter (BEAM_SPACING_HPBW, hexagonal lattice) now drives both the number of
#     search positions and the mean TX grid loss (closed form 1.672*spacing^2 dB, Gaussian main lobe).
#     Grid loss is ON by default: V6.0.0 was optimistic by ~3.3 dB (range x0.83) at the default spacing.
#     Spacing 0.92 reproduces the 1.41 dB quoted in the notes, but implies ~10200 positions.
#   - Time bases separated: dwell = (N-1)*PRI + tau (frame time, migration), coherent time (tau for a single
#     pulse, dwell for a coherent train; coherence loss). V6.0.0 used N*PRI for all three.
#   - Frame time optionally includes the echo wait (FRAME_INTERLEAVED = False).
#   - solve() now refuses a frequency outside the waveform band or a scan angle above the band limit.
#   - TX_chain loss placeholder (0 dB) in S band: reference plane of the 2 kW is still to be settled [?].
#   - NOT changed (needs a decision): scan broadening in the position count, migration model, T_sys/pre-LNA
#     definition, Ku loss decomposition, element factor exponent for S.
#   - Round-trip-time dependence of the coherence loss is deliberately NOT modelled: it only affects
#     pulse-to-pulse coherence and is bounded by the coherent span already used.
#
# Changes vs V5.4.0 (V6.0.0)
#  FIXES
#   - Detection SNR: Albersheim was mis-implemented (log10 argument used as dB) and the
#     integration gain was counted twice (-10log10(N) on the SNR AND N*tau in the energy
#     term). Replaced by Shnidman's TOTAL-ENERGY SNR requirement (C*X_inf), consistent
#     with the energy-form radar equation. Albersheim kept as a cross-check only.
#   - Explicit COHERENT / NONCOHERENT integration mode (V5.4.0 mixed both through N_eff).
#   - Signal bandwidth (range resolution/accuracy) separated from RX channel bandwidth.
#   - Scan loss no longer clamped at 70 deg (extrapolated up to the band scan limit).
#  NEW
#   - Two bands (S, Ku) with their own TRM count, peak power, losses, T_sys, scan limit.
#   - Only waveforms PLAYABLE at the chosen frequency are evaluated.
#     Role policy: S = SEARCH/TWS only (no tracking), Ku = CONFIRM/TRACK only (no search; PROMOTE removed, 3.2).
#   - TX and RX gains separated (TX illumination 1.0, RX 0.88 in S) [V].
#   - Average-power / duty check against the electrical budget (PAE 75 % [V]).
#   - Range-migration loss (target closing speed) and range-ambiguity flag.
#   - Search-frame feasibility (positions x dwell vs refresh time).
#   - Multi-target table (ST / KKV / SHIP).
#  REMOVED
#   - All Ku-era waveforms (WF-C, ETHR, ET, PS, PA, PRD, PRA, LRV, AT), RX channel
#     accounting (obsolete with one ADC per TRM), cued-measurement driver.
#   - Only WF-LRS (S band) kept, as a code benchmark.
# ============================================================

from __future__ import annotations

import argparse
import dataclasses
import functools
import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from . import antenna
    from . import detection
    from .params import P
except ImportError:            # run as a script: python core/radar_model.py
    import antenna
    import detection
    from params import P

MODEL_VERSION = "6.32.0"

# =============================================================================
# PARAMETERS -- all values come from radar_system.toml (core/params.py): edit them there, not here.
# Names kept as module constants so callers (optimizer, tracking, GUI, ...) are unchanged.
# =============================================================================
# --- Scenario (CLI defaults) ---
FREQUENCY_GHZ: float = P.get("tool.cli.frequency_ghz")     # selects the band and playable waveforms
SCAN_ANGLE_DEG: float = P.get("tool.cli.scan_angle_deg")
TARGET_NAME: str = P.get("tool.cli.target")                # a class of [targets.*] (ST = reference target)
FACE_ELEC_POWER_MW: float = P.get("power.face_elec_mw")    # electrical power allocated to one face

# --- Detection ---
DETECT_PD: float = P.get("detection.pd")
DETECT_PFA: float = P.get("detection.pfa")                 # per independent cell ('pfa_cell' mode) and Ku looks
FA_MODE: str = P.get("detection.fa_mode")                  # 'fa_rate' | 'pfa_cell' (step 2.3)
PFA_MIN: float = P.get("detection.pfa_min")
PFA_MAX: float = P.get("detection.pfa_max")
CUM_PD_FLOOR: float = P.get("detection.cum_pd_floor")
OFFSET_QUANTILES: int = P.get("tool.detection.offset_quantiles")
if FA_MODE not in ("fa_rate", "pfa_cell"):
    raise ValueError(f"{P.path}: detection.fa_mode '{FA_MODE}' must be 'fa_rate' or 'pfa_cell'")

# --- Power chain (electrical budget since step 1.3; the PAE is per band and per waveform: pae()) ---
PAE_DUTY_REF: float = P.get("power.pae_duty_ref")
TRM_TECHNOLOGY: str = P.get("power.trm_technology")
CHAIN_EFF: float = P.get("power.psu_eff") * P.get("power.distribution_eff")   # step 4.1: overhead_frac -> NON_TX
FACE_FEEDER_MAX_MW: float = P.get("power.face_feeder_max_mw")      # step 4.1 (A)
FACE_STORE_J: float = P.get("power.face_store_mj") * 1e6            # step 4.1 (D)
FACE_STORE_MAX_W: float = P.get("power.face_store_max_mw") * 1e6
STORE_DOD: float = P.get("power.store_dod")
PUMP_FRAC: float = P.get("power.pump_frac")
ADC_W: float = P.get("power.adc_w")                              # step 4.1 (B), read at import
DBF_CONTROL_W: float = 1e3 * (P.get("power.dbf_kw") + P.get("power.control_kw"))

# --- Range migration ---
MIGRATION_MODE: str = P.get("migration.mode")              # "COMPENSATED" (residual loss) | "UNCOMPENSATED"
MIGRATION_RESIDUAL_DB: float = P.get("migration.residual_db")
BANK_MAX_LOSS_DB: float = P.get("migration.bank_max_loss_db")

# --- Search beam grid ---
# Hexagonal lattice of TX beam positions, spacing = BEAM_SPACING_HPBW x HPBW_TX. The SAME spacing drives the number
# of positions (frame time) and the mean TX grid loss. In each position the digital RX forms a lattice of RX beams
# (spacing RX_BEAM_SPACING_HPBW x HPBW_RX): residual RX grid loss, RX beam count (step 1.1).
# Spacing history (position counts of V6.0.9 / V6.0.10, with the former 0.886 lambda / D beam and a 20 deg tilt; 3016
# positions at 1.04 since step 1.2):
#   1.1  -> ~4590 positions, mean loss ~2.0 dB, worst -4.9 dB (V6.0.9) ; 1.04 -> ~5130, mean 1.81 dB, worst -4.3 dB (locked, V6.0.10)
#   1.4  -> ~2830 positions, mean loss ~3.3 dB, worst point -7.9 dB (best mean range per energy)
#   0.92 -> ~10200 positions, mean loss 1.41 dB (the value quoted in the notes)
BEAM_SPACING_HPBW: float = P.get("search_grid.beam_spacing_hpbw")
# spacings the search optimiser tries for each sector (step 3.1, P5 E7: exact search, every spacing at every centre)
BEAM_SPACING_OPTIONS: Tuple[float, ...] = tuple(float(x) for x in P.get("search_grid.beam_spacing_options_hpbw"))
RX_BEAM_SPACING_HPBW: float = P.get("search_grid.rx_beam_spacing_hpbw")
ENABLE_GRID_LOSS: bool = P.get("search_grid.enable_grid_loss")
FRAME_INTERLEAVED: bool = P.get("search_grid.frame_interleaved")   # else the frame adds the echo wait 2R/c
GRID_DITHERING: bool = P.get("search_grid.grid_dithering")         # lattice shifted every frame (step 2.3)
if not GRID_DITHERING:
    raise ValueError(f"{P.path}: search_grid.grid_dithering = false (fixed lattice, positions correlated between "
                     f"revisits) is not implemented: the step-2.3 criterion draws the position anew at each revisit")

# --- Multibeam (TX, step 3.1 block C); RX loss = 0 (digital beamforming) ---
TX_MODES: Tuple[str, ...] = ("single", "superposition", "partition", "tdm")
MB_SUP_LOSS_DB: Dict[int, float] = dict(zip((int(b) for b in P.get("multibeam.superposition_beams")),
                                            (float(x) for x in P.get("multibeam.superposition_loss_db"))))
if len(MB_SUP_LOSS_DB) != len(P.get("multibeam.superposition_loss_db")):
    raise ValueError(f"{P.path}: multibeam.superposition_beams and superposition_loss_db must pair one to one")
MB_PART_BEAMS: Tuple[int, ...] = tuple(int(b) for b in P.get("multibeam.partition_beams"))
if not set(MB_PART_BEAMS) <= {2, 4}:
    raise ValueError(f"{P.path}: multibeam.partition_beams: only 2 (halves) and 4 (quadrants) are modelled")
MB_TDM_BEAMS: Tuple[int, ...] = tuple(int(b) for b in P.get("multibeam.tdm_beams"))
PULSE_TB_MIN: float = P.get("waveform.pulse_tb_min")   # shortest pulse = this / B ("no excess energy" floor)

# --- S-band energy buffer: legacy S-only alias (V6.0.x); the model uses the BandConfig values ---
E_BUF_TRM_J: float = P.get("band.S.e_buf_trm_j")

# --- Coherence (step 2.2): coherent blocks of optimal length, noncoherent beyond; target law exp(-(T / T0)^p) with
# --- T0 = t0_k x lambda / (2 L omega) of the target class (step 2.4)
# exp(-(T / t0)^p) (per target type at 2.4) + LO phase noise through the round trip + residual platform motion ---
ENABLE_COH_LOSS: bool = P.get("coherence.enable")
COH_T0_K: float = P.get("coherence.t0_k")
COH_P: float = P.get("coherence.p")
LO_REF_GHZ: float = P.get("coherence.lo_ref_ghz")
LO_OFFSETS_HZ: Tuple[float, ...] = tuple(P.get("coherence.lo_offsets_hz"))
LO_SSB_DBC_HZ: Tuple[float, ...] = tuple(P.get("coherence.lo_ssb_dbc_hz"))
PLATFORM_MOTION_RMS_M: float = P.get("coherence.platform_motion_rms_m")
if len(LO_OFFSETS_HZ) != len(LO_SSB_DBC_HZ) or list(LO_OFFSETS_HZ) != sorted(LO_OFFSETS_HZ) or LO_OFFSETS_HZ[0] <= 0:
    raise ValueError(f"{P.path}: coherence.lo_offsets_hz must be increasing, > 0, as long as lo_ssb_dbc_hz")

# --- Processing grid, CFAR, velocity bank (step 2.2) ---
RANGE_GRID_OVERSAMPLING: float = P.get("processing.range_grid_oversampling")
DOPPLER_ZERO_PADDING: float = P.get("processing.doppler_zero_padding")
DOPPLER_WINDOW: str = P.get("processing.doppler_window")
DOPPLER_WINDOW_OPTIONS_DB: Tuple[float, ...] = tuple(float(x) for x in P.get("processing.doppler_window_options_db"))
CFAR_TYPE: str = P.get("processing.cfar_type")
CFAR_REF_CELLS: int = P.get("processing.cfar_ref_cells")
CFAR_OS_RANK_FRAC: float = P.get("processing.cfar_os_rank_frac")
MASKING_OFFSETS_CELLS: Tuple[int, ...] = tuple(P.get("tool.detection.masking_offsets_cells"))
if DOPPLER_WINDOW != "none" or CFAR_TYPE not in ("CA", "OS", "GO"):
    raise ValueError(f"{P.path}: [processing] only doppler_window 'none' and cfar_type 'CA' / 'OS' / 'GO' are implemented")
if RANGE_GRID_OVERSAMPLING < 1.0 or DOPPLER_ZERO_PADDING < 1.0 or CFAR_REF_CELLS < 2:
    raise ValueError(f"{P.path}: [processing] grid factors must be >= 1 and cfar_ref_cells >= 2")
if not 0.0 < CFAR_OS_RANK_FRAC <= 1.0:
    raise ValueError(f"{P.path}: processing.cfar_os_rank_frac must be in ]0, 1]")

# --- Aperture physics: derived from the array build-up (bi-band TRSA tiles in an approximate octagon) ---
KU_TRM_PITCH_M: float = P.get("aperture.ku_trm_pitch_m")
KU_TRM_PER_TRSA_SIDE: int = P.get("aperture.ku_trm_per_trsa_side")
S_TRM_PER_TRSA_SIDE: int = P.get("aperture.s_trm_per_trsa_side")
TRSA_ROWS: Tuple[int, ...] = tuple(P.get("aperture.trsa_rows"))
TRSA_GAP_M: float = P.get("aperture.trsa_gap_m")
if KU_TRM_PER_TRSA_SIDE % S_TRM_PER_TRSA_SIDE:
    raise ValueError(f"{P.path}: aperture.s_trm_per_trsa_side ({S_TRM_PER_TRSA_SIDE}) must divide "
                     f"ku_trm_per_trsa_side ({KU_TRM_PER_TRSA_SIDE}): each S TRM covers a whole block of Ku TRMs")
TRSA_SIDE_M: float = KU_TRM_PER_TRSA_SIDE * KU_TRM_PITCH_M
S_TRM_PITCH_M: float = TRSA_SIDE_M / S_TRM_PER_TRSA_SIDE
N_TRSA: int = sum(TRSA_ROWS)
# Gap segments between adjacent TRSAs (rows centred): within the rows, and between consecutive rows
N_TRSA_GAPS: int = sum(n - 1 for n in TRSA_ROWS) + sum(min(a, b) for a, b in zip(TRSA_ROWS, TRSA_ROWS[1:]))
PHYS_AREA_M2: float = N_TRSA * TRSA_SIDE_M ** 2            # emissive area per face (the gaps are dead area)
TOTAL_AREA_M2: float = PHYS_AREA_M2 + N_TRSA_GAPS * TRSA_GAP_M * TRSA_SIDE_M
FILL_FACTOR: float = PHYS_AREA_M2 / TOTAL_AREA_M2
ARRAY_ENVELOPE_M: float = max(TRSA_ROWS) * TRSA_SIDE_M + (max(TRSA_ROWS) - 1) * TRSA_GAP_M   # widest row, beamwidth only
N_TRM: Dict[str, int] = {"S": N_TRSA * S_TRM_PER_TRSA_SIDE ** 2, "Ku": N_TRSA * KU_TRM_PER_TRSA_SIDE ** 2}
ETA_OHMIC: float = P.get("aperture.eta_ohmic")
# Radiation pattern of each band computed from the build-up (core/antenna.py): TX uniform beam, RX weighting per
# sidelobe option (eta_illum_rx, RX beamwidth), RMS extent x_rms (angle accuracy). Beamwidths in lambda / D_env.
RX_SLL_OPTIONS_DB: Tuple[float, ...] = tuple(float(s) for s in P.get("aperture.rx_sll_options_db"))
RX_SLL_DEFAULT_DB: float = float(P.get("tool.mission_defaults.sector.rx_sll_db"))
if RX_SLL_DEFAULT_DB not in RX_SLL_OPTIONS_DB:
    raise ValueError(f"{P.path}: tool.mission_defaults.sector.rx_sll_db ({RX_SLL_DEFAULT_DB:g}) is not one of "
                     f"aperture.rx_sll_options_db {list(RX_SLL_OPTIONS_DB)}")


def _visible_r_lam_d(band_name: str) -> float:
    """Radius of the visible region seen by the scanned beam, (1 + sin scan limit) D_env / lambda, at the top of the
    band (largest in lambda / D_env): the region of the far-sidelobe scan (step 1.2)."""
    b = f"band.{band_name}."
    hi = P.get(b + "usable_hi_ghz")
    lim = P.get(b + "scan_max_deg") if hi <= P.get(b + "scan_break_ghz") else P.get(b + "scan_max_hi_deg")
    return (1.0 + math.sin(math.radians(lim))) * ARRAY_ENVELOPE_M * hi * 1e9 / 299_792_458.0


PATTERNS: Dict[str, "antenna.BandPattern"] = antenna.face_patterns(
    TRSA_ROWS, TRSA_SIDE_M, TRSA_GAP_M, {"S": S_TRM_PER_TRSA_SIDE, "Ku": KU_TRM_PER_TRSA_SIDE}, ARRAY_ENVELOPE_M,
    RX_SLL_OPTIONS_DB, P.get("aperture.rx_taper_nbar"), P.get("aperture.rx_taper_edge"),
    {b: _visible_r_lam_d(b) for b in ("S", "Ku")})
# --- Active mismatch and polarisation (step 1.2) ---
GRATING_LOBE_G0: float = P.get("aperture.grating_lobe_g0")
TX_POLARISATION: str = P.get("aperture.tx_polarisation")
MISMATCH_ALERT_DB: float = P.get("tool.alerts.mismatch_db_per_pass")

# --- Surveillance sector (per face); its radius SECTOR_RADIUS_DEG is derived from the band scan limits below ---
REFRESH_S: float = P.get("tool.cli.refresh_s")   # frame check of the CLI report only (missions: sector RR)

# --- Tracking estimator constants (used by Ku TRACK waveforms, core/tracking.py) ---
ANGLE_CRLB_MARGIN: float = P.get("estimation.angle_crlb_margin")
K_F: float = P.get("estimation.k_f")
RANGE_CRLB_KR: float = P.get("estimation.range_crlb_kr")

# --- Waveforms: modulation, compression, intra-pulse effects, dwell timeline (step 2.1) ---
MODULATIONS: Tuple[str, ...] = tuple(P.get("waveform.modulations"))
NLFM_DOPPLER_MAX_FRAC: float = P.get("waveform.nlfm_doppler_max_frac")
NLFM_MISMATCH_LOSS_DB: float = P.get("waveform.nlfm_mismatch_loss_db")
RANGE_PSL = P.get("waveform.range_psl")                    # "volume" (= rx_sll_db) or a level in dB
RANGE_TAYLOR_NBAR: int = P.get("waveform.range_taylor_nbar")
PA_AM_PM_DEG_PER_DB: float = P.get("waveform.pa_am_pm_deg_per_db")
STAGGER_N_PRI: int = P.get("waveform.stagger_n_pri")
STAGGER_SPREAD: float = P.get("waveform.stagger_spread")
if not set(MODULATIONS) <= {"lfm", "nlfm"} or "lfm" not in MODULATIONS:
    raise ValueError(f"{P.path}: waveform.modulations {list(MODULATIONS)} must be within ['lfm', 'nlfm'] and hold 'lfm'")
if not (RANGE_PSL == "volume" or (isinstance(RANGE_PSL, (int, float)) and RANGE_PSL < 0)):
    raise ValueError(f"{P.path}: waveform.range_psl must be 'volume' or a negative level in dB")
# =============================================================================

C_MPS = 299_792_458.0
K_BOLTZ = 1.380_649e-23


def db_to_lin(x_db: float) -> float:
    return 10.0 ** (x_db / 10.0)


def lin_to_db(x: float) -> float:
    return 10.0 * math.log10(x) if x > 0 else -999.0


def lambda_m(freq_ghz: float) -> float:
    return C_MPS / (freq_ghz * 1e9)


# -----------------------------------------------------------------------------
# Bands
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class BandConfig:
    name: str
    usable_lo_ghz: float
    usable_hi_ghz: float
    core_lo_ghz: float           # core band: no edge loss inside (step 1.4)
    core_hi_ghz: float
    edge_derating_db: float      # total (round-trip) loss at the usable edges, raised cosine from the core edges
    scan_max_deg: float          # scan limit up to scan_break_ghz
    scan_break_ghz: float
    scan_max_hi_deg: float       # scan limit above scan_break_ghz
    waim_match_deg: float        # scan angle where the WAIM matches the array (active-mismatch model, step 1.2)
    n_trm: int
    p_pk_trm_w: float
    pae_sat: float               # saturated PAE at PAE_DUTY_REF (step 1.3)
    pae_t_edge_s: float          # DC-on time per pulse without full RF (edges, bias / phase settling)
    pae_duty_slope: float        # relative PAE loss per unit duty above PAE_DUTY_REF (channel heating)
    eta_illum_tx: float          # (eta_illum_rx is derived from the volume's RX weighting: rx_beam())
    t_sys_k: float               # cold sky (the sky in front of the face is added by direction: core/sky.py)
    losses_db: Dict[str, float]  # fixed implementation losses (summed)
    allowed_roles: Tuple[str, ...]
    pol_rx_dual: bool = True      # two RX polarisation channels per TRM (GLRT recombination, step 1.5)
    pol_loss_single_db: float = 0.0   # loss of a single-pol receiver (pol_rx_dual false)
    pol_track_est_loss_db: float = 0.0   # TRACK looks with dual pol: polarisation known, estimation residual
    e_buf_trm_j: float = 0.0      # per-TRM energy buffer, J (0 = droop not modelled)
    buffer_max_fraction: float = 0.5   # solve() refuses a pulse drawing more than this fraction of the buffer
    bi_max_mhz: float = 0.0       # instantaneous bandwidth ceiling, MHz (0 = not defined)
    true_time_delay: str = "digital_per_trm"   # or "none": phase steering, bandwidth limited by the aperture fill time
    duty_max: float = 0.0         # TRM duty ceiling on pulse trains (0 = not defined), checked by Waveform
    # receive chain dynamic range (step 1.5), levels referred to the antenna port (element output, as T_sys)
    digital_noise_db: float = 0.0     # factor on any antenna-temperature rise (T_sys = (...) x 10^(digital/10))
    adc_nf_db: float = 0.0            # derived from ENOB, full scale and sample rate
    adc_fs_hz: float = 0.0            # complex sample rate
    gain_to_adc_db: float = 0.0       # antenna port -> ADC input: -L_pre + G_LNA + G_post
    clip_ant_dbm: float = 0.0         # clipping level (P_FS - clip_backoff) referred to the antenna port
    lna_p1db_ant_dbm: float = 0.0     # LNA input P1dB referred to the antenna port
    preselect_rej_db: float = 0.0     # rejection of the other band before the LNA
    post_rej_db: float = 0.0          # rejection of the other band before the ADC
    limiter_recovery_s: float = 0.0   # receiver deaf for tau + this after each pulse (eclipsing, step 2.1)
    errors: Optional[Dict[str, float]] = None   # residual array errors and calibration cadence (step 3.4)
    trm_supply_w: float = 0.0         # continuous supply capacity of one TRM, W (step 4.1, C; 0 = not checked)
    rth: Tuple[Tuple[float, float], ...] = ()   # junction-to-plate Foster poles (K/W, s) of a TRM PA (step 4.2)

    @property
    def noise_ant_dbm(self) -> float:
        """Cold-sky noise power at the antenna port over the ADC band, k T_sys fs (dBm)."""
        return 10.0 * math.log10(K_BOLTZ * self.t_sys_k * self.adc_fs_hz * 1e3)

    @property
    def clip_headroom_db(self) -> float:
        """Largest echo per element and sample above the noise before the ADC clips (dB)."""
        return self.clip_ant_dbm - self.noise_ant_dbm

    def contains(self, f_ghz: float) -> bool:
        return self.usable_lo_ghz <= f_ghz <= self.usable_hi_ghz

    def scan_limit_deg(self, f_ghz: float) -> float:
        return self.scan_max_deg if f_ghz <= self.scan_break_ghz else self.scan_max_hi_deg

    def fits(self, f_ghz: float, sig_bw_mhz: float = 0.0) -> bool:
        """The occupied spectrum [f - B/2, f + B/2] lies inside the usable band (step 1.4)."""
        half = sig_bw_mhz * 1e-3 / 2.0
        return self.usable_lo_ghz - 1e-12 <= f_ghz - half and f_ghz + half <= self.usable_hi_ghz + 1e-12

    def _edge_integral(self, f_ghz: float) -> float:
        """Integral of the edge law from usable_lo_ghz to f (dB x GHz), exact for the raised cosine."""
        w1, w2 = self.core_lo_ghz - self.usable_lo_ghz, self.usable_hi_ghz - self.core_hi_ghz
        x = min(max((f_ghz - self.usable_lo_ghz) / w1, 0.0), 1.0) if w1 > 0 else 1.0
        y = min(max((f_ghz - self.core_hi_ghz) / w2, 0.0), 1.0) if w2 > 0 else 0.0
        low = 0.5 * w1 * (x + math.sin(math.pi * x) / math.pi)
        high = 0.5 * w2 * (y - math.sin(math.pi * y) / math.pi)
        return self.edge_derating_db * (low + high)

    def edge_loss_db(self, f_ghz: float, sig_bw_mhz: float = 0.0) -> float:
        """Raised-cosine derating outside the core band, max edge_derating_db at the usable edges, averaged over the
        occupied spectrum [f - B/2, f + B/2] (B = 0: at f). 100 dB when the spectrum leaves the usable band."""
        if not self.fits(f_ghz, sig_bw_mhz):
            return 100.0
        half = sig_bw_mhz * 1e-3 / 2.0
        if half <= 0.0:
            if self.core_lo_ghz <= f_ghz <= self.core_hi_ghz:
                return 0.0
            lmax = self.edge_derating_db
            if f_ghz < self.core_lo_ghz:
                x = (f_ghz - self.usable_lo_ghz) / (self.core_lo_ghz - self.usable_lo_ghz)
                return lmax * 0.5 * (1.0 + math.cos(math.pi * x))
            x = (f_ghz - self.core_hi_ghz) / (self.usable_hi_ghz - self.core_hi_ghz)
            return lmax * 0.5 * (1.0 - math.cos(math.pi * x))
        return (self._edge_integral(f_ghz + half) - self._edge_integral(f_ghz - half)) / (2.0 * half)

    @property
    def p_pk_total_w(self) -> float:
        return self.n_trm * self.p_pk_trm_w

    @property
    def l_fixed_db(self) -> float:
        return sum(self.losses_db.values())


T0_REF_K = 290.0   # IEEE reference temperature of a noise figure


def pre_lna_loss_db(rx: Dict[str, float]) -> float:
    """Circulator + limiter + line before the LNA (attenuates the signal and adds noise, inside T_sys)."""
    return rx["circulator_loss_db"] + rx["limiter_loss_db"] + rx["pre_lna_line_db"]


def adc_noise_figure_db(rx: Dict[str, float], bi_max_mhz: float) -> float:
    """ADC noise figure from the converter (step 1.5): quantisation noise P_FS - (6.02 ENOB + 1.76) spread over the
    complex sample rate fs = adc_oversampling x bi_max, against k T0 fs: NF = P_FS - SNR_FS - 10 log10(k T0 fs 1e3)."""
    fs = rx["adc_oversampling"] * bi_max_mhz * 1e6
    snr_fs = 6.02 * rx["adc_enob"] + 1.76
    return rx["adc_fullscale_dbm"] - snr_fs - 10.0 * math.log10(K_BOLTZ * T0_REF_K * fs * 1e3)


def t_sys_cascade(rx: Dict[str, float], bi_max_mhz: float) -> float:
    """System noise temperature at the antenna port (Friis), cold sky, from the receive-chain contributors of
    [band.<b>.rx]: the losses before the LNA attenuate the signal and add (L - 1) T_phys, the later stages count divided
    by the gain ahead of them (the ADC with its derived noise figure), and the digital processing raises the total by
    digital_noise_db."""
    l_pre = db_to_lin(pre_lna_loss_db(rx))
    t_nf = lambda nf_db: T0_REF_K * (db_to_lin(nf_db) - 1.0)          # noqa: E731
    g_lna, g_post = db_to_lin(rx["lna_gain_db"]), db_to_lin(rx["post_lna_gain_db"])
    t_adc = t_nf(adc_noise_figure_db(rx, bi_max_mhz))
    t_rx = t_nf(rx["lna_nf_db"]) + t_nf(rx["post_lna_nf_db"]) / g_lna + t_adc / (g_lna * g_post)
    t = rx["t_ant_k"] + (l_pre - 1.0) * rx["t_phys_frontend_k"] + l_pre * t_rx
    return t * db_to_lin(rx["digital_noise_db"])



def tx_chain_loss_db(tx: Dict[str, float], rx: Dict[str, float]) -> float:
    """PA output -> radiating element: harmonic filter, circulator (TX pass of the component of the receive chain),
    calibration coupler, line. The TRM peak power is specified at the PA output (where the PAE is defined)."""
    return tx["harmonic_filter_db"] + rx["circulator_loss_db"] + tx["cal_coupler_db"] + tx["line_to_element_db"]


TTD_MODES = ("digital_per_trm", "none")


def _error_variances(er: Dict[str, float]) -> Tuple[float, float]:
    """(random per TRM, correlated per TRSA) error variances, rad^2, at the END of a calibration interval (step 3.4):
    phase sigma^2 + relative amplitude sigma^2, the drift (per TRSA) adding to the TRSA residual in quadrature."""
    amp = lambda db: (10.0 ** (db / 20.0) - 1.0)                       # noqa: E731
    t_min = er["cal_interval_s"] / 60.0
    rnd = math.radians(er["trm_phase_deg"]) ** 2 + amp(er["trm_amp_db"]) ** 2
    cor = (math.radians(er["trsa_phase_deg"]) ** 2 + math.radians(er["drift_phase_deg_min"] * t_min) ** 2
           + amp(er["trsa_amp_db"]) ** 2 + amp(er["drift_amp_db_min"] * t_min) ** 2)
    return rnd, cor


def array_error_loss_db(er: Dict[str, float]) -> float:
    """Gain loss of ONE pass (TX or RX) of the residual array errors (Ruze: phase exp(-sigma^2), amplitude
    1 / (1 + sigma^2), random and correlated alike), step 3.4 B4."""
    amp = lambda db: (10.0 ** (db / 20.0) - 1.0)                       # noqa: E731
    t_min = er["cal_interval_s"] / 60.0
    ph = (math.radians(er["trm_phase_deg"]) ** 2 + math.radians(er["trsa_phase_deg"]) ** 2
          + math.radians(er["drift_phase_deg_min"] * t_min) ** 2)
    am = amp(er["trm_amp_db"]) ** 2 + amp(er["trsa_amp_db"]) ** 2 + amp(er["drift_amp_db_min"] * t_min) ** 2
    return -10.0 * math.log10(math.exp(-ph) / (1.0 + am))


def _band(name: str) -> BandConfig:
    """Band [band.<name>] of radar_system.toml, with load-time consistency checks so that the band (usable, core,
    break frequency) can be changed in the file alone (step 1.4)."""
    b = f"band.{name}."
    lo, hi = P.get(b + "usable_lo_ghz"), P.get(b + "usable_hi_ghz")
    clo, chi, brk = P.get(b + "core_lo_ghz"), P.get(b + "core_hi_ghz"), P.get(b + "scan_break_ghz")
    ttd = P.get(b + "true_time_delay")
    errs = []
    if not lo < hi:
        errs.append(f"usable_lo_ghz ({lo}) must be below usable_hi_ghz ({hi})")
    if not lo <= clo <= chi <= hi:
        errs.append(f"core band {clo}-{chi} GHz must lie inside the usable band {lo}-{hi} GHz")
    if not lo <= brk <= hi:
        errs.append(f"scan_break_ghz ({brk}) must lie inside the usable band {lo}-{hi} GHz")
    if ttd not in TTD_MODES:
        errs.append(f"true_time_delay '{ttd}' must be one of {TTD_MODES}")
    rx = P.section(b + "rx")
    if rx["post_lna_op1db_dbm"] < rx["adc_fullscale_dbm"] + rx["clip_backoff_db"]:
        errs.append(f"rx.post_lna_op1db_dbm ({rx['post_lna_op1db_dbm']:g}) must reach adc_fullscale_dbm + clip_backoff_db "
                    f"({rx['adc_fullscale_dbm'] + rx['clip_backoff_db']:g} dBm): the analog chain would compress before "
                    f"the ADC clips")
    if errs:
        raise ValueError(f"{P.path}: [band.{name}] " + "; ".join(errs))
    bi = P.get(b + "bi_max_mhz")
    er = P.section(b + "errors")
    l_pre = pre_lna_loss_db(rx)
    gain = -l_pre + rx["lna_gain_db"] + rx["post_lna_gain_db"]
    return BandConfig(
        name=name, usable_lo_ghz=lo, usable_hi_ghz=hi, core_lo_ghz=clo, core_hi_ghz=chi,
        edge_derating_db=P.get(b + "edge_derating_db"),
        scan_max_deg=P.get(b + "scan_max_deg"), scan_break_ghz=brk,
        scan_max_hi_deg=P.get(b + "scan_max_hi_deg"), waim_match_deg=P.get(b + "waim_match_deg"),
        n_trm=N_TRM[name], p_pk_trm_w=P.get(b + "p_pk_trm_w"), pae_sat=P.get(b + "pae_sat"),
        pae_t_edge_s=P.get(b + "pae_t_edge_s"), pae_duty_slope=P.get(b + "pae_duty_slope"),
        eta_illum_tx=P.get(b + "eta_illum_tx"),
        t_sys_k=t_sys_cascade(rx, bi),
        # step 3.4: 'calibration' = gain loss of the residual array errors, TX and RX (was a lumped 0.5 / 0.8 dB)
        losses_db={**P.section(b + "losses_db"), "calibration": 2.0 * array_error_loss_db(er),
                   "TX_chain": tx_chain_loss_db(P.section(b + "tx"), rx)},
        errors=er, trm_supply_w=P.get(b + "trm_supply_w"),
        rth=((P.get(b + "rth_fast_k_w"), P.get(b + "tau_fast_s")), (P.get(b + "rth_slow_k_w"), P.get(b + "tau_slow_s"))),
        allowed_roles=tuple(P.get(b + "allowed_roles")),
        pol_rx_dual=P.get(b + "pol_rx_dual"), pol_loss_single_db=P.get(b + "pol_loss_single_db"),
        pol_track_est_loss_db=P.get(b + "pol_track_est_loss_db"),
        e_buf_trm_j=P.get(b + "e_buf_trm_j"), buffer_max_fraction=P.get(b + "buffer_max_fraction"),
        bi_max_mhz=bi, duty_max=P.get(b + "duty_max"), true_time_delay=ttd,
        digital_noise_db=rx["digital_noise_db"], adc_nf_db=adc_noise_figure_db(rx, bi),
        adc_fs_hz=rx["adc_oversampling"] * bi * 1e6, gain_to_adc_db=gain,
        clip_ant_dbm=rx["adc_fullscale_dbm"] - rx["clip_backoff_db"] - gain,
        lna_p1db_ant_dbm=rx["lna_p1db_in_dbm"] + l_pre, preselect_rej_db=rx["preselect_rej_db"],
        post_rej_db=rx["post_rej_db"], limiter_recovery_s=rx["limiter_recovery_s"],
    )


BAND_S = _band("S")
BAND_KU = _band("Ku")
_S_LO, _S_HI = BAND_S.usable_lo_ghz, BAND_S.usable_hi_ghz
_KU_LO, _KU_HI = BAND_KU.usable_lo_ghz, BAND_KU.usable_hi_ghz
BANDS: Dict[str, BandConfig] = {"S": BAND_S, "Ku": BAND_KU}
# Face surveillance sector radius = the widest scan limit of any band (derived, no entry of its own: review 0.3)
SECTOR_RADIUS_DEG: float = max(max(b.scan_max_deg, b.scan_max_hi_deg) for b in BANDS.values())


def pae(band: BandConfig, tau_s: float, duty: Optional[float] = None) -> float:
    """PAE of a pulse of the band (step 1.3): pae_sat x tau / (tau + t_edge) x min(1, 1 - a (d - d_ref)). duty = tau / PRI
    of a train; None (single pulse, no train) = PAE_DUTY_REF. Capped at pae_sat below d_ref (prudent)."""
    d = PAE_DUTY_REF if duty is None else duty
    derate = min(1.0, 1.0 - band.pae_duty_slope * (d - PAE_DUTY_REF))
    return band.pae_sat * tau_s / (tau_s + band.pae_t_edge_s) * derate


def pae_wf(wf: "Waveform") -> float:
    """PAE of a waveform: its train duty if N > 1, the reference duty for a single pulse. A time-multiplexed dwell
    sends its B pulses back to back: one TRM pulse of B tau, duty B tau / PRI (step 3.1, C)."""
    return pae(BANDS[wf.band], wf.tx_block_s, wf.trm_duty if wf.n > 1 else None)


def pulse_elec_energy_trm_j(wf: "Waveform") -> float:
    """Electrical energy one TRM draws from its buffer for one pulse (edges included): P_pk x tau / PAE (B tau for a
    time-multiplexed dwell)."""
    return BANDS[wf.band].p_pk_trm_w * wf.tx_block_s / pae_wf(wf)


def trm_supply_check(wf: "Waveform", band: "BandConfig") -> None:
    """Supply of a TRM over a pulse train (step 4.1, C1-C2), ValueError if not held (the waveform is removed from the
    search spaces): the mean electrical power within the train must stay within the continuous supply capacity, and
    the buffer, recharged at that capacity between pulses, must not lose more than buffer_max_fraction over the train
    (net loss of the n - 1 first pulses plus the last pulse)."""
    if band.trm_supply_w <= 0.0 or wf.n < 2:
        return
    e_p = pulse_elec_energy_trm_j(wf)
    p_train = e_p / wf.pri_s
    if p_train > band.trm_supply_w * (1.0 + 1e-12):
        raise ValueError(f"{wf.name}: {p_train:.0f} W per TRM over the train exceeds its supply "
                         f"{band.trm_supply_w:g} W")
    drawn = (wf.n - 1) * max(0.0, e_p - band.trm_supply_w * wf.pri_s) + e_p
    if band.e_buf_trm_j > 0 and drawn > band.buffer_max_fraction * band.e_buf_trm_j * (1.0 + 1e-12):
        raise ValueError(f"{wf.name}: the train draws {drawn:.2f} J of the TRM buffer (max "
                         f"{band.buffer_max_fraction * band.e_buf_trm_j:.2f} J with the {band.trm_supply_w:g} W supply)")


# ----------------------------------------------------------------------------- thermal (step 4.2)
_TH = "thermal."
TH_CP = P.get(_TH + "coolant_cp_kj_kgk") * 1e3
TH_H = P.get(_TH + "coolant_h_w_m2k")
TH_RHO = P.get(_TH + "coolant_density_kg_l")
TH_AREA_FACTOR = P.get(_TH + "microchannel_area_factor")
TH_DT_MAX = P.get(_TH + "plate_fluid_dt_max_k")
TH_RISE = P.get(_TH + "coolant_rise_k")
TH_INLET_C = P.get(_TH + "coolant_inlet_c")
TJ_MAX_C = P.get(_TH + "tj_max_c")
TH_FLUID_L = P.get(_TH + "fluid_volume_l")                 # read at import: the face inertia (C3)
TH_STRUCT_J_K = P.get(_TH + "structure_heat_capacity_kj_k") * 1e3
TH_PLATE_RISE = P.get(_TH + "plate_rise_max_k")
SHIP_HEAT_REJECTION_W = P.get(_TH + "ship_heat_rejection_mw") * 1e6
JUNCTION_CHECK = P.get("tool.thermal.junction_check")
for _k, _v in (("coolant_cp_kj_kgk", TH_CP), ("coolant_h_w_m2k", TH_H), ("coolant_density_kg_l", TH_RHO),
               ("microchannel_area_factor", TH_AREA_FACTOR), ("plate_fluid_dt_max_k", TH_DT_MAX),
               ("coolant_rise_k", TH_RISE)):
    if _v <= 0:
        raise ValueError(f"{P.path}: thermal.{_k} must be > 0")
if TJ_MAX_C <= TH_INLET_C + TH_RISE:
    raise ValueError(f"{P.path}: thermal.tj_max_c must be above the coolant outlet temperature")


def face_cooling() -> Dict[str, float]:
    """Derived cooling figures of one face (step 4.2, C1-C3): admissible flux h x area factor x dT (W/m2) over the
    emissive area = capacity; flow for the coolant rise; thermal inertia of fluid + structure and the buffer of the
    admissible plate rise."""
    flux = TH_H * TH_AREA_FACTOR * TH_DT_MAX
    cap = flux * PHYS_AREA_M2
    inertia = TH_FLUID_L * TH_RHO * TH_CP + TH_STRUCT_J_K
    return {"flux_max_w_cm2": flux / 1e4, "capacity_w": cap, "flow_l_s": cap / (TH_CP * TH_RISE) / TH_RHO,
            "inertia_j_k": inertia, "buffer_j": inertia * TH_PLATE_RISE}


def r_conv_trm(band_name: str) -> float:
    """Plate-to-fluid convection resistance of one TRM footprint (K/W): 1 / (h x area factor x footprint)."""
    return 1.0 / (TH_H * TH_AREA_FACTOR * PHYS_AREA_M2 / N_TRM[band_name])


def junction_temp_c(wf: "Waveform", band: Optional["BandConfig"] = None) -> float:
    """Junction temperature of a TRM PA at the end of the train (step 4.2, B1): coolant outlet + convection at the
    mean heat of the dwell + the Foster poles driven by the pulses, P_d (1 - e^(-tau/tau_i)) sum_k e^(-k PRI/tau_i)."""
    band = band or BANDS[wf.band]
    eta = pae_wf(wf)
    p_d = band.p_pk_trm_w * (1.0 / eta - 1.0)                         # heat while the pulse is on
    t_on, pri = wf.tx_block_s, wf.pri_s
    t = TH_INLET_C + TH_RISE + r_conv_trm(band.name) * trm_dissipation_w(wf)
    for r_i, tau_i in band.rth:
        acc = sum(math.exp(-k * pri / tau_i) for k in range(wf.n))
        t += r_i * p_d * (1.0 - math.exp(-t_on / tau_i)) * acc
    return t


def junction_check(wf: "Waveform", band: "BandConfig") -> None:
    """ValueError when the junction exceeds tj_max_c (step 4.2, B; active with tool.thermal.junction_check)."""
    tj = junction_temp_c(wf, band)
    if tj > TJ_MAX_C:
        raise ValueError(f"{wf.name}: junction {tj:.0f} C above {TJ_MAX_C:g} C")


def search_position_costs(wf: "Waveform", scale=1.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Cost of ONE position of the full-aperture search lattice (step 3.1, C), with the pulse shortened by `scale`
    ("no excess energy": PAs saturated, the pulse is cut to scale x tau, floored at PULSE_TB_MIN / B; the energy is
    then kept). Returns (electrical J, s of TX time, effective scale, PAE of the shortened pulse); scale may be an
    array. A dwell sends trains_per_dwell trains at the face peak power and covers cells_per_dwell positions."""
    band = BANDS[wf.band]
    s = np.maximum(np.minimum(np.asarray(scale, dtype=float), 1.0), min(1.0, PULSE_TB_MIN / (wf.sig_bw_mhz * 1e6) / wf.tau_s))
    tau = wf.tau_s * s
    block = tau * wf.trains_per_dwell if wf.tx_mode == "tdm" else tau
    d = block / wf.pri_s if wf.n > 1 else PAE_DUTY_REF
    eta = band.pae_sat * block / (block + band.pae_t_edge_s) * np.minimum(1.0, 1.0 - band.pae_duty_slope * (d - PAE_DUTY_REF))
    share = wf.trains_per_dwell / cells_per_dwell(wf)
    t = wf.n * tau * share
    return band.p_pk_total_w * t / eta, t, s, eta


def tau_max_s(band: BandConfig) -> float:
    """Longest pulse the TRM energy buffer allows (solve() refuses more): buffer_max_fraction x E_buf x pae_sat / P_pk
    minus the edge time, per TRM (exact for a single pulse or a train at d <= d_ref). Infinite when the buffer is not
    modelled (e_buf_trm_j = 0). Derived, so waveform spaces follow the architecture."""
    if band.e_buf_trm_j <= 0.0:
        return math.inf
    return band.buffer_max_fraction * band.e_buf_trm_j * band.pae_sat / band.p_pk_trm_w - band.pae_t_edge_s


def non_tx_power_w() -> float:
    """Always-on consumption of one face behind the PSU (W, step 4.1 B1-B4): two ADCs and two receive chains per TRM in
    both bands, digital beamforming and processing, control. The pumps (a share of the heat) are added by the budget."""
    return (sum(2 * N_TRM[b] * (ADC_W + P.get(f"band.{b}.rx.rx_chain_w")) for b in ("S", "Ku"))
            + DBF_CONTROL_W)


def face_bus_draw_w(tx_elec_w: float, heat_w: float) -> float:
    """Mean power one face draws from the ship bus (W, step 4.1): transmission (TRM supply), always-on consumption and
    pumps, through the PSU and the distribution."""
    return (tx_elec_w + non_tx_power_w() + PUMP_FRAC * heat_w) / CHAIN_EFF


def face_elec_avail_w(elec_mw: float) -> float:
    """Electrical power a face can spend on transmission (W) from an allocation (MW): PSU and distribution, minus the
    always-on consumption (step 4.1; was a lumped 3 % overhead).
    Every look costs its RF energy / PAE (electrical budget, step 1.3)."""
    return elec_mw * 1e6 * CHAIN_EFF - non_tx_power_w()


def trm_dissipation_w(wf: "Waveform") -> float:
    """Heat one TRM dissipates on average during the dwell (W): P_pk x duty x (1 / PAE - 1). Output for step 4.2."""
    return BANDS[wf.band].p_pk_trm_w * wf.trm_duty * (1.0 / pae_wf(wf) - 1.0)


def band_for_frequency(f_ghz: float) -> Optional[BandConfig]:
    for b in BANDS.values():
        if b.contains(f_ghz):
            return b
    return None


# -----------------------------------------------------------------------------
# Targets
# -----------------------------------------------------------------------------
# Target library (step 2.4): one class per [targets.<NAME>] table of radar_system.toml (CLASS MEANS, not one weapon's
# specification); a class is added or changed in the file only. A sector names a class ("target_type") and may override
# any value (target_for_sector). Fluctuation families: SW0 steady, SW1/2 many scatterers, SW3/4 one dominant scatterer.
FLUCTUATIONS: Dict[str, float] = {"SW0": math.inf, "SW1/2": 1.0, "SW3/4": 2.0}   # Gamma shape per independent sample
_TARGET_KEYS = ("v_close_mps", "length_m", "rotation_dps", "accel_max_mps2")
_TARGET_BAND_KEYS = ("rcs_m2", "fluctuation", "depol_db")
_TARGET_OPTICS = {"optics_diameter_m": 0.0, "optics_f_ratio": 1.0, "optics_retro_eff": 0.0,   # absent = no optics
                  "maneuver_tau_s": 0.0,            # step 3.3 (A5): Singer time constant, 0 = no manoeuvre
                  "aspect_offset_deg": 0.0,         # step 3.3 (F6): aspect of a closing target off its nose
                  # step 5.1 (B): RCS vs presentation, rows elevation ASPECT_EL_DEG x columns azimuth ASPECT_AZ_DEG, dB vs
                  # the frontal RCS (absent = flat); spreads from one object to another (1 sigma); half-angle of the cone
                  # where the seeker optics return; the class's own aspect policy ("" = tool.targets.aspect_policy)
                  "aspect_rcs_db": (), "rcs_spread_db": 0.0, "depol_spread_db": 0.0, "optics_cone_deg": 180.0,
                  "aspect_policy": "",
                  # return on 3.3 from step 5.4 (lever C): 1-sigma transverse velocity at the track birth, per axis;
                  # 0 = tracking.init_vel_sigma_frac x v_close (a homing class aims at the ship: small)
                  "init_vt_sigma_mps": 0.0}
ASPECT_AZ_DEG = tuple(range(0, 360, 30))
ASPECT_EL_DEG = tuple(range(-90, 91, 30))
ASPECT_POLICIES = ("worst", "frontal", "mean")
ASPECT_POLICY: str = P.get("tool.targets.aspect_policy")
if ASPECT_POLICY not in ASPECT_POLICIES:
    raise ValueError(f"{P.path}: tool.targets.aspect_policy '{ASPECT_POLICY}' must be one of {', '.join(ASPECT_POLICIES)}")
DEFAULT_TARGET_TYPE: str = P.get("tool.targets.default_type")
TARGET_TYPES: Tuple[str, ...] = tuple(dict.fromkeys(p.path.split(".")[1] for p in P if p.path.startswith("targets.")))


def _target_class_values(name: str) -> Dict[str, object]:
    """The values of one class, checked against the schema (an incomplete or inconsistent class is refused)."""
    pre = f"targets.{name}"
    keys = {p.path[len(pre) + 1:] for p in P if p.path.startswith(pre + ".")}
    need = set(_TARGET_KEYS) | {f"band.{b}.{k}" for b in BANDS for k in _TARGET_BAND_KEYS}
    missing, extra = sorted(need - keys), sorted(keys - need - set(_TARGET_OPTICS))
    if missing or extra:
        raise ValueError(f"{P.path}: target class '{name}' " + (f"misses {missing}" if missing else f"has unknown keys {extra}")
                         + f" (schema: {', '.join(_TARGET_KEYS)}, optional {', '.join(_TARGET_OPTICS)}, and per band "
                         f"[{pre}.band.<S|Ku>] {', '.join(_TARGET_BAND_KEYS)})")
    v: Dict[str, object] = {k: P.get(f"{pre}.{k}") for k in _TARGET_KEYS}
    v.update({k: P.get(f"{pre}.{k}") if k in keys else d for k, d in _TARGET_OPTICS.items()})
    for b in BANDS:
        fl = P.get(f"{pre}.band.{b}.fluctuation")
        if fl not in FLUCTUATIONS:
            raise ValueError(f"{P.path}: {pre}.band.{b}.fluctuation '{fl}' must be one of {', '.join(FLUCTUATIONS)}")
        if P.get(f"{pre}.band.{b}.rcs_m2") <= 0:
            raise ValueError(f"{P.path}: {pre}.band.{b}.rcs_m2 must be > 0")
        v[f"rcs_{b.lower()}"] = float(P.get(f"{pre}.band.{b}.rcs_m2"))
        v[f"fluct_{b.lower()}"] = fl
        v[f"depol_{b.lower()}_db"] = float(P.get(f"{pre}.band.{b}.depol_db"))
    if v["length_m"] <= 0 or v["rotation_dps"] < 0 or v["accel_max_mps2"] < 0 or v["v_close_mps"] <= 0:
        raise ValueError(f"{P.path}: target class '{name}': length_m and v_close_mps must be > 0, rotation_dps and "
                         f"accel_max_mps2 >= 0")
    tab = v["aspect_rcs_db"]
    if tab and (len(tab) != len(ASPECT_EL_DEG) or any(len(r) != len(ASPECT_AZ_DEG) for r in tab)):
        raise ValueError(f"{P.path}: target class '{name}': aspect_rcs_db must be {len(ASPECT_EL_DEG)} rows (elevation "
                         f"{ASPECT_EL_DEG[0]}..{ASPECT_EL_DEG[-1]} deg) of {len(ASPECT_AZ_DEG)} values (azimuth 0..330 deg)")
    if (v["maneuver_tau_s"] < 0 or v["rcs_spread_db"] < 0 or v["depol_spread_db"] < 0 or v["init_vt_sigma_mps"] < 0
            or not 0.0 < v["optics_cone_deg"] <= 180.0 or v["aspect_policy"] not in ("",) + ASPECT_POLICIES):
        raise ValueError(f"{P.path}: target class '{name}': maneuver_tau_s and spreads >= 0, optics_cone_deg in ]0, 180], "
                         f"aspect_policy one of {', '.join(ASPECT_POLICIES)}")
    v["aspect_rcs_db"] = tuple(tuple(float(x) for x in r) for r in tab)
    if v["optics_diameter_m"] < 0 or v["optics_f_ratio"] <= 0 or not 0.0 <= v["optics_retro_eff"] <= 1.0:
        raise ValueError(f"{P.path}: target class '{name}': optics_diameter_m >= 0, optics_f_ratio > 0, "
                         f"optics_retro_eff in [0, 1]")
    return v


if DEFAULT_TARGET_TYPE not in TARGET_TYPES:
    raise ValueError(f"{P.path}: tool.targets.default_type '{DEFAULT_TARGET_TYPE}' is not a class of [targets.*] "
                     f"({', '.join(TARGET_TYPES)})")
_DEF = _target_class_values(DEFAULT_TARGET_TYPE)


# step 5.1 (B): presentations on a 2 deg raster of nodes (the nose az 0, el 0 among them, so that the frontal value is
# reached exactly), solid-angle weights cos(el), off-nose angle of each
_FINE_AZ, _FINE_EL = np.meshgrid(np.arange(0.0, 360.0, 2.0), np.arange(-90.0, 90.1, 2.0))
_FINE_W = np.cos(np.radians(_FINE_EL)) / np.cos(np.radians(_FINE_EL)).sum()
_FINE_NOSE = np.degrees(np.arccos(np.clip(np.cos(np.radians(_FINE_EL)) * np.cos(np.radians(_FINE_AZ)), -1.0, 1.0)))


def _table_db(table: Tuple[Tuple[float, ...], ...], az: np.ndarray, el: np.ndarray) -> np.ndarray:
    """The class table (dB) at (az, el), bilinear (azimuth wraps at 360 deg); 0 dB without a table."""
    if not table:
        return np.zeros(np.shape(az))
    t = np.asarray(table)
    a = np.mod(np.asarray(az, float), 360.0) / 30.0
    e = (np.clip(np.asarray(el, float), -90.0, 90.0) + 90.0) / 30.0
    i0, j0 = np.minimum(np.floor(e).astype(int), len(ASPECT_EL_DEG) - 2), np.floor(a).astype(int) % len(ASPECT_AZ_DEG)
    fe, fa = e - i0, a - np.floor(a)
    j1 = (j0 + 1) % len(ASPECT_AZ_DEG)
    return ((1 - fe) * ((1 - fa) * t[i0, j0] + fa * t[i0, j1]) + fe * ((1 - fa) * t[i0 + 1, j0] + fa * t[i0 + 1, j1]))


@functools.lru_cache(maxsize=64)
def _aspect_stats(table: Tuple[Tuple[float, ...], ...]) -> Tuple[np.ndarray, float]:
    """(table on the raster in dB, solid-angle weighted spread in dB)."""
    g = _table_db(table, _FINE_AZ, _FINE_EL)
    mu = float((_FINE_W * g).sum())
    return g, math.sqrt(float((_FINE_W * (g - mu) ** 2).sum()))


@functools.lru_cache(maxsize=512)
def _presentation(table: Tuple[Tuple[float, ...], ...], cone_deg: float, offset_deg: float, policy: str,
                  optics_ratio: float) -> Tuple[float, float]:
    """(body factor, optics factor) of the presentation of detection (step 5.1): 'frontal' at offset_deg off the nose
    (step 3.3 F6), 'worst' the presentation of least total RCS (body x table + optics within its cone), 'mean' the
    solid-angle means."""
    if policy == "frontal":
        return (db_to_lin(float(_table_db(table, np.array(offset_deg), np.array(0.0)))),
                1.0 if offset_deg <= cone_deg else 0.0)
    g = 10.0 ** (_aspect_stats(table)[0] / 10.0)
    vis = (_FINE_NOSE <= cone_deg).astype(float)
    if policy == "mean":
        return float((_FINE_W * g).sum()), float((_FINE_W * vis).sum())
    k = int(np.argmin(g + optics_ratio * vis))
    return float(g.flat[k]), float(vis.flat[k])


@dataclass(frozen=True)
class Target:
    """A target as seen by one look. The first four fields keep the former form Target(name, rcs, swerling, v): the
    other values then come from the default class (tool.targets.default_type). rcs_m2 is the copolar frontal RCS of the
    body in S (and in Ku when rcs_ku_m2 is None); swerling = a declared case 0..4, or None = derived from the band's
    fluctuation family and the coherence time T0 (step 2.4)."""
    name: str
    rcs_m2: float
    swerling: Optional[int]
    v_close_mps: float
    rcs_ku_m2: Optional[float] = None
    fluct_s: str = _DEF["fluct_s"]
    fluct_ku: str = _DEF["fluct_ku"]
    depol_s_db: float = _DEF["depol_s_db"]
    depol_ku_db: float = _DEF["depol_ku_db"]
    length_m: float = _DEF["length_m"]
    rotation_dps: float = _DEF["rotation_dps"]
    accel_max_mps2: float = _DEF["accel_max_mps2"]
    optics_diameter_m: float = _DEF["optics_diameter_m"]
    optics_f_ratio: float = _DEF["optics_f_ratio"]
    optics_retro_eff: float = _DEF["optics_retro_eff"]
    target_type: str = DEFAULT_TARGET_TYPE
    maneuver_tau_s: float = _DEF["maneuver_tau_s"]           # step 3.3 (A5)
    aspect_offset_deg: float = _DEF["aspect_offset_deg"]     # step 3.3 (F6)
    aspect_rcs_db: Tuple[Tuple[float, ...], ...] = _DEF["aspect_rcs_db"]   # step 5.1 (B)
    rcs_spread_db: float = _DEF["rcs_spread_db"]
    depol_spread_db: float = _DEF["depol_spread_db"]
    optics_cone_deg: float = _DEF["optics_cone_deg"]
    aspect_policy: str = _DEF["aspect_policy"]
    init_vt_sigma_mps: float = _DEF["init_vt_sigma_mps"]     # return on 3.3 from step 5.4 (lever C)

    def _ku(self, f_ghz: float) -> bool:
        band = band_for_frequency(f_ghz)
        return band is not None and band.name == "Ku"

    def policy(self) -> str:
        """Presentation of detection (step 5.1): the class's own policy, else tool.targets.aspect_policy."""
        return self.aspect_policy or ASPECT_POLICY

    def _frontal_body_m2(self, f_ghz: float) -> float:
        return (self.rcs_m2 if self.rcs_ku_m2 is None else self.rcs_ku_m2) if self._ku(f_ghz) else self.rcs_m2

    def _frontal_optics_m2(self, f_ghz: float) -> float:
        """Return of a focusing optic facing the radar (step 2.4, K): the larger of the direct reflection on the concave
        primary pi (2 f)^2 (geometrical optics) and the cat's-eye retro-reflection eta_r 4 pi A^2 / lambda^2."""
        if self.optics_diameter_m <= 0.0:
            return 0.0
        d = self.optics_diameter_m
        a = math.pi * d * d / 4.0
        return max(math.pi * (2.0 * self.optics_f_ratio * d) ** 2,
                   self.optics_retro_eff * 4.0 * math.pi * a * a / lambda_m(f_ghz) ** 2)

    def _factors(self, f_ghz: float) -> Tuple[float, float]:
        b0 = self._frontal_body_m2(f_ghz)
        ratio = round(self._frontal_optics_m2(f_ghz) / b0, 9) if b0 > 0 else 0.0
        return _presentation(self.aspect_rcs_db, float(self.optics_cone_deg), round(abs(self.aspect_offset_deg), 6),
                             self.policy(), ratio)

    def aspect_factor(self, f_ghz: float = 2.8) -> float:
        """Factor of the body RCS at the presentation of detection (step 3.3 F6, step 5.1 B)."""
        return self._factors(f_ghz)[0]

    def body_rcs_m2(self, f_ghz: float) -> float:
        return self._frontal_body_m2(f_ghz) * self._factors(f_ghz)[0]

    def optics_rcs_m2(self, f_ghz: float) -> float:
        """The seeker optics' return at the presentation of detection: full within optics_cone_deg of the nose, none
        outside (step 5.1, B4)."""
        return self._frontal_optics_m2(f_ghz) * self._factors(f_ghz)[1]

    def rcs_at(self, f_ghz: float) -> float:
        """Copolar RCS at f of the presentation of detection: body + seeker optics."""
        return self.body_rcs_m2(f_ghz) + self.optics_rcs_m2(f_ghz)

    def aspect_spread_db(self) -> float:
        """Spread (1 sigma, dB) of the body RCS over every presentation, solid-angle weighted (step 5.1, B): 0 for a
        class seen frontally (it aims at the ship) or without a table."""
        return 0.0 if self.policy() == "frontal" else _aspect_stats(self.aspect_rcs_db)[1]

    def t0_s(self, f_ghz: float) -> float:
        """Coherence time T0 = t0_k x lambda / (2 L omega) (step 2.4, B1); infinite for a non-rotating target."""
        w = math.radians(self.rotation_dps)
        return math.inf if w <= 0.0 else COH_T0_K * lambda_m(f_ghz) / (2.0 * self.length_m * w)

    def fluctuation(self, f_ghz: float) -> str:
        return self.fluct_ku if self._ku(f_ghz) else self.fluct_s

    def depol_lin(self, f_ghz: float) -> float:
        """Cross-polar / copolar RCS ratio rho at f."""
        return db_to_lin(self.depol_ku_db if self._ku(f_ghz) else self.depol_s_db)

    def swerling_case(self, f_ghz: float) -> int:
        """Nearest Swerling case of one short look (the declared case, else the family: SW0 0, SW1/2 1, SW3/4 3), for
        the closed-form confirmation statistics of core/tracking.py."""
        if self.swerling is not None:
            return self.swerling
        return {"SW0": 0, "SW1/2": 1, "SW3/4": 3}[self.fluctuation(f_ghz)]


def _target_of_class(name: str) -> Target:
    v = _target_class_values(name)
    return Target(name, v["rcs_s"], None, v["v_close_mps"], rcs_ku_m2=v["rcs_ku"], fluct_s=v["fluct_s"],
                  fluct_ku=v["fluct_ku"], depol_s_db=v["depol_s_db"], depol_ku_db=v["depol_ku_db"],
                  length_m=v["length_m"], rotation_dps=v["rotation_dps"], accel_max_mps2=v["accel_max_mps2"],
                  optics_diameter_m=v["optics_diameter_m"], optics_f_ratio=v["optics_f_ratio"],
                  optics_retro_eff=v["optics_retro_eff"], target_type=name, maneuver_tau_s=v["maneuver_tau_s"],
                  aspect_offset_deg=v["aspect_offset_deg"], aspect_rcs_db=v["aspect_rcs_db"],
                  rcs_spread_db=v["rcs_spread_db"], depol_spread_db=v["depol_spread_db"],
                  optics_cone_deg=v["optics_cone_deg"], aspect_policy=v["aspect_policy"],
                  init_vt_sigma_mps=v["init_vt_sigma_mps"])


TARGETS: Dict[str, Target] = {name: _target_of_class(name) for name in TARGET_TYPES}

# Sector keys overriding a class value (step 2.4, A2); rcs_m2, depol_db and fluctuation apply to both bands.
SECTOR_TARGET_KEYS = ("target_type", "rcs_m2", "swerling", "fluctuation", "depol_db", "length_m", "rotation_dps",
                      "accel_max_mps2", "optics_diameter_m", "optics_f_ratio", "optics_retro_eff", "maneuver_tau_s",
                      "aspect_offset_deg", "init_vt_sigma_mps")


def target_for_sector(sec: dict) -> Target:
    """The target of a sector (step 2.4, A2-A3): its class "target_type" (else the default class, keeping the
    sector's own rcs_m2 and swerling, default SW1: the former behaviour), then every value the sector gives. The
    closing speed is the sector's v_close_max_mps."""
    name = sec.get("target_type")
    if name is not None and name not in TARGETS:
        raise ValueError(f"unknown target_type '{name}' (classes: {', '.join(TARGETS)})")
    base = TARGETS[name or DEFAULT_TARGET_TYPE]
    kw: Dict[str, object] = {"name": str(sec.get("id", base.name)),
                             "v_close_mps": float(sec.get("v_close_max_mps", base.v_close_mps))}
    if name is None:
        kw.update(rcs_m2=float(sec["rcs_m2"]), rcs_ku_m2=None, swerling=int(sec.get("swerling", 1)))
    else:
        if sec.get("rcs_m2") is not None:
            kw.update(rcs_m2=float(sec["rcs_m2"]), rcs_ku_m2=None)
        if sec.get("swerling") is not None:
            kw["swerling"] = int(sec["swerling"])
    if "fluctuation" in sec:
        kw.update(fluct_s=sec["fluctuation"], fluct_ku=sec["fluctuation"])
    if "depol_db" in sec:
        kw.update(depol_s_db=float(sec["depol_db"]), depol_ku_db=float(sec["depol_db"]))
    for k in ("length_m", "rotation_dps", "accel_max_mps2", "optics_diameter_m", "optics_f_ratio", "optics_retro_eff",
              "maneuver_tau_s", "aspect_offset_deg", "init_vt_sigma_mps"):
        if k in sec:
            kw[k] = float(sec[k])
    return dataclasses.replace(base, **kw)


# -----------------------------------------------------------------------------
# Gain / geometry
# -----------------------------------------------------------------------------
def aperture_m2(eta_illum: float, theta_deg: float) -> float:
    # Geometric projection; the active mismatch is the loss "scan_mismatch" of solve() (active_mismatch, step 1.2).
    ef = max(math.cos(math.radians(theta_deg)), 1e-6)
    return PHYS_AREA_M2 * eta_illum * ETA_OHMIC * ef


def gain_lin(eta_illum: float, f_ghz: float, theta_deg: float) -> float:
    return 4.0 * math.pi * aperture_m2(eta_illum, theta_deg) / lambda_m(f_ghz) ** 2


def rx_beam(band_name: str, rx_sll_db: Optional[float] = None) -> "antenna.Beam":
    """RX beam of a band for a sidelobe option (None = the default option): eta_illum_rx, beamwidth, achieved level."""
    s = RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    try:
        return PATTERNS[band_name].rx[s]
    except KeyError:
        raise ValueError(f"RX sidelobe level {s:g} dB is not an option of aperture.rx_sll_options_db "
                         f"{list(RX_SLL_OPTIONS_DB)}") from None


def _pattern(f_ghz: float) -> "antenna.BandPattern":
    band = band_for_frequency(f_ghz)
    if band is None:
        raise ValueError(f"{f_ghz} GHz is outside every band")
    return PATTERNS[band.name]


def hpbw_deg(f_ghz: float, theta_deg: float = 0.0, beam: str = "tx", rx_sll_db: Optional[float] = None) -> float:
    """-3 dB beamwidth (deg) of the TX beam (uniform) or of the RX beam (weighting rx_sll_db), smallest over the
    cuts; constant in (u, v), so the angular width grows as 1/cos(theta) in the scan plane (the worst axis)."""
    fac = _pattern(f_ghz).tx.hpbw if beam == "tx" else rx_beam(band_for_frequency(f_ghz).name, rx_sll_db).hpbw
    d_eff = ARRAY_ENVELOPE_M * max(math.cos(math.radians(theta_deg)), 1e-6)
    return math.degrees(fac * lambda_m(f_ghz) / d_eff)


def grating_lobe_margin(f_ghz: float, theta_deg: float) -> float:
    """Distance (in u) of the first grating lobe to the visible region, worst azimuth (scan along a lattice axis):
    lambda / d_mean - sin(theta) - 1, d_mean = mean element pitch of the band. <= 0: a grating lobe is visible."""
    return lambda_m(f_ghz) / _pattern(f_ghz).d_mean_m - math.sin(math.radians(abs(theta_deg))) - 1.0


def active_mismatch(f_ghz: float, theta_deg: float) -> Dict[str, float]:
    """Active reflection of the array at scan theta (one model for S and Ku, step 1.2): current sheet matched at
    waim_match_deg by a WAIM, |G_waim| = |cos t - cos t_m| / (cos t + cos t_m) (same in the E and H planes), and the
    rise near a grating lobe, |G_gl| = exp(-g / GRATING_LOBE_G0) (worst azimuth), combined in power.
    Returns |G_waim|, |G_gl|, |Gamma|, the grating-lobe margin g and the loss per pass (dB; TX or RX)."""
    band = band_for_frequency(f_ghz)
    c, cm = max(math.cos(math.radians(theta_deg)), 1e-6), math.cos(math.radians(band.waim_match_deg))
    g_w = abs(c - cm) / (c + cm)
    g = grating_lobe_margin(f_ghz, theta_deg)
    g_gl = min(1.0, math.exp(-g / GRATING_LOBE_G0))
    trans = max((1.0 - g_w * g_w) * (1.0 - g_gl * g_gl), 1e-12)
    return {"gamma_waim": g_w, "gamma_gl": g_gl, "gamma": math.sqrt(1.0 - trans), "margin_g": g,
            "loss_db_per_pass": -10.0 * math.log10(trans)}


def pol_purity(theta_deg: float, phi_deg: float) -> Dict[str, float]:
    """Polarisation purity of the TX element (linear along e_v, TX_POLARISATION), output only (step 1.2): current
    element over the array, E_theta = cos t sin p, E_phi = cos p (p from e_u). Ludwig-3 co / cross components, the
    cross-polar discrimination (dB, infinite in the principal planes, (1 + cos t) / (1 - cos t) at 45 deg) and the
    tilt of the radiated linear polarisation. With the dual-pol RX this costs no SNR (both channels captured)."""
    t, p = math.radians(theta_deg), math.radians(phi_deg)
    co = math.cos(t) * math.sin(p) ** 2 + math.cos(p) ** 2
    cross = math.sin(p) * math.cos(p) * (math.cos(t) - 1.0)
    xpd = math.inf if abs(cross) < 1e-15 else 20.0 * math.log10(abs(co) / abs(cross))
    return {"co": co, "cross": cross, "xpd_db": xpd, "tilt_deg": math.degrees(math.atan2(cross, co))}


def sidelobe_visible_db(f_ghz: float, theta_deg: float, beam: str = "rx", rx_sll_db: Optional[float] = None) -> float:
    """Worst sidelobe (dB) visible when the beam is scanned to theta at f: near sidelobes, and the far lobes at a
    distance r lambda / D_env <= 1 + sin(theta) of the axis (worst azimuth; gap lobes of the TRSA lattice, step 1.2)."""
    pat = _pattern(f_ghz)
    b = pat.tx if beam == "tx" else rx_beam(band_for_frequency(f_ghz).name, rx_sll_db)
    r = (1.0 + math.sin(math.radians(abs(theta_deg)))) * ARRAY_ENVELOPE_M / lambda_m(f_ghz)
    prof = b.far_profile_db or ()
    far = prof[min(int(r / antenna.FAR_PROFILE_STEP), len(prof) - 1)] if prof else -math.inf
    return max(b.sll_db, far)


def rx_sll_held_up_to_deg(f_ghz: float, rx_sll_db: Optional[float] = None, theta_max_deg: float = 90.0) -> float:
    """Largest scan angle (0.5 deg steps, up to theta_max_deg) at which the RX option is held over the visible
    region (sidelobe_visible_db <= option + fit tolerance). An input of the per-sector alert (step 1.2)."""
    s = RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    held = 0.0
    th = 0.0
    while th <= theta_max_deg + 1e-9:
        if sidelobe_visible_db(f_ghz, th, "rx", s) > s + antenna.FIT_TOL_DB:
            break
        held = th
        th += 0.5
    return held


def angle_k_tx(band_name: str) -> float:
    """Angle accuracy per axis x sqrt(SNR), in TX beamwidths: Cramer-Rao bound of the full digital aperture,
    sigma_u = lambda / (2 pi x_rms sqrt(2 SNR)), x ANGLE_CRLB_MARGIN, divided by the TX beamwidth in (u, v).
    The same in every direction and at every frequency of the band (both scale with lambda / cos(theta))."""
    p = PATTERNS[band_name]
    return ANGLE_CRLB_MARGIN * ARRAY_ENVELOPE_M / (2.0 * math.pi * math.sqrt(2.0) * p.x_rms_m * p.tx.hpbw)


SECTOR_TILT_DEG: float = P.get("geometry.tilt_deg")   # all four arrays tilted toward +Z (nose) (ship axis vertical, lighthouse model)
# Default of the mission-preparation scan window (a mission input sent to the scheduler, independent of tilt and scan
# limit; passed explicitly by the callers that have a mission):
SECTOR_EPS_MIN_DEG: float = P.get("tool.mission_defaults.eps_min_deg")   # rear limit (-90 = no limit; physical floor ~ -55 deg)
SECTOR_EPS_MAX_DEG: float = P.get("tool.mission_defaults.eps_max_deg")   # front limit (90 = no limit)


_SECTOR_AREA_CACHE: Dict[Tuple[float, float], float] = {}


def sector_area_uv(eps_min_deg: Optional[float] = None, eps_max_deg: Optional[float] = None) -> float:
    """Cached per eps window (solve() calls it for every SEARCH waveform); None = the default window."""
    key = (SECTOR_EPS_MIN_DEG if eps_min_deg is None else eps_min_deg,
           SECTOR_EPS_MAX_DEG if eps_max_deg is None else eps_max_deg)
    if key not in _SECTOR_AREA_CACHE:
        _SECTOR_AREA_CACHE[key] = _sector_area_uv(*key)
    return _SECTOR_AREA_CACHE[key]


def _sector_area_uv(eps_min_deg: float, eps_max_deg: float) -> float:
    """Area of the face sector in direction cosines (u, v) of the (tilted) face frame: disk of radius sin(R) (scan
    limit) cut by (i) the two PLANES phi = +/-45 deg between adjacent faces, (ii) the mission elevation window
    [SECTOR_EPS_MIN_DEG, SECTOR_EPS_MAX_DEG] (eps measured from the plane XY toward +Z).
    Face frame: direction = u e_u + v e_v + w n, n tilted by t toward +Z => eps = asin(v cos t + w sin t).
    The planes cut |u| <= w cos t - v sin t, i.e. |u| <= (-v sin t + cos t sqrt(1 + cos^2 t - 2 v^2)) / (1 + cos^2 t)
    (t = 0 gives sin45 sqrt(1 - v^2)). For a given v, eps decreases with |u|, so the eps window is an interval
    [u_lo, u_hi] on |u|."""
    r = math.sin(math.radians(SECTOR_RADIUS_DEG))
    c, sn = math.cos(math.radians(SECTOR_TILT_DEG)), math.sin(math.radians(SECTOR_TILT_DEG))
    zmin, zmax = math.sin(math.radians(eps_min_deg)), math.sin(math.radians(eps_max_deg))
    n = 20000
    tot = 0.0
    dv = 2.0 * r / n
    for i in range(n):
        v = -r + (i + 0.5) * dv
        disc = 1.0 + c * c - 2.0 * v * v
        hi = (-sn * v + c * math.sqrt(disc)) / (1.0 + c * c) if disc > 0 else 0.0
        hi = min(max(hi, 0.0), math.sqrt(max(r * r - v * v, 0.0)))
        lo = 0.0
        if sn > 1e-9:
            wmin = (zmin - v * c) / sn          # eps >= eps_min  <=>  w >= wmin
            if wmin > 0.0:
                hi = min(hi, math.sqrt(max(1.0 - v * v - wmin * wmin, 0.0)) if 1.0 - v * v - wmin * wmin > 0.0 else 0.0)
            wmax = (zmax - v * c) / sn          # eps <= eps_max  <=>  w <= wmax
            if wmax < 0.0:
                hi = 0.0
            elif wmax * wmax < 1.0 - v * v:
                lo = math.sqrt(1.0 - v * v - wmax * wmax)
        elif not (zmin <= v <= zmax):
            hi = 0.0
        if hi > lo:
            tot += 2.0 * (hi - lo) * dv
    return tot


def _spacing(spacing_hpbw: Optional[float]) -> float:
    return BEAM_SPACING_HPBW if spacing_hpbw is None else float(spacing_hpbw)


def beam_cell_area_uv(f_ghz: float, spacing_hpbw: Optional[float] = None) -> float:
    """Area of one hexagonal cell of the TX beam lattice in (u, v): spacing = spacing_hpbw (None: BEAM_SPACING_HPBW)
    x HPBW_TX (real uniform-illumination pattern, smallest cut)."""
    h = _pattern(f_ghz).tx.hpbw * lambda_m(f_ghz) / ARRAY_ENVELOPE_M
    return (math.sqrt(3.0) / 2.0) * (_spacing(spacing_hpbw) * h) ** 2


def rx_beams_per_position(f_ghz: float, rx_sll_db: Optional[float] = None, spacing_hpbw: Optional[float] = None) -> float:
    """Digital RX beams formed per TX search position: ratio of the TX and RX hexagonal lattice cells (RX spacing
    RX_BEAM_SPACING_HPBW x HPBW_RX). An input of the compute and data budget (roadmap step 4.4)."""
    p = _pattern(f_ghz)
    rx = rx_beam(band_for_frequency(f_ghz).name, rx_sll_db)
    return (_spacing(spacing_hpbw) * p.tx.hpbw / (RX_BEAM_SPACING_HPBW * rx.hpbw)) ** 2


def n_search_positions(f_ghz: float, eps_min_deg: Optional[float] = None, eps_max_deg: Optional[float] = None,
                       spacing_hpbw: Optional[float] = None) -> float:
    """Positions per face in the surveillance sector, counted in (u, v) (beam broadening with scan included)."""
    return sector_area_uv(eps_min_deg, eps_max_deg) / beam_cell_area_uv(f_ghz, spacing_hpbw)


def grid_loss_db(spacing_hpbw: Optional[float] = None) -> float:
    """Mean TX loss over a regular hexagonal cell (Gaussian main lobe): L(r) = 12.04 (r/HPBW)^2 dB,
    mean r^2 over the cell = 5 s^2 / 36  ->  L = 1.672 s^2 dB. Corner (worst case) loss is 12.04 s^2 / 3."""
    s = BEAM_SPACING_HPBW if spacing_hpbw is None else spacing_hpbw
    return 10.0 * math.log10(math.e) * 4.0 * math.log(2.0) * (5.0 / 36.0) * s * s


def grid_rx_loss_db() -> float:
    """Mean RX loss of a target between the digital RX beams of a position (same Gaussian law, RX lattice spacing
    RX_BEAM_SPACING_HPBW in RX beamwidths): 0.42 dB at 0.5."""
    return grid_loss_db(RX_BEAM_SPACING_HPBW)


# -----------------------------------------------------------------------------
# Loss terms
# -----------------------------------------------------------------------------
def multibeam_tx_loss_db(wf: "Waveform") -> float:
    """TX multibeam loss (step 3.1, C): the phase-only superposition's table; partition and time multiplex lose
    nothing (their cost is in the power, gain and positions of each beam)."""
    return MB_SUP_LOSS_DB[wf.b_tx] if wf.tx_mode == "superposition" else 0.0


# --- TX aperture partition (step 3.1, C): sub-aperture beams on their own search lattice
@functools.lru_cache(maxsize=8)
def subaperture(band_name: str, kind: str) -> "antenna.SubAperture":
    n_sub = {"S": S_TRM_PER_TRSA_SIDE, "Ku": KU_TRM_PER_TRSA_SIDE}[band_name]
    return antenna.subaperture(TRSA_ROWS, TRSA_SIDE_M, TRSA_GAP_M, n_sub, ARRAY_ENVELOPE_M, kind)


def subaperture_cell_ratio(band_name: str, kind: str) -> float:
    """Area of a lattice cell of the sub-aperture beam / that of the full face: the full-face lattice stretched by the
    beamwidth ratios along u and v (1.85 for a half, 3.43 for a quadrant of the current octagon)."""
    sa = subaperture(band_name, kind)
    return (sa.hpbw_u / sa.full_hpbw_u) * (sa.hpbw_v / sa.full_hpbw_v)


@functools.lru_cache(maxsize=32)
def subaperture_grid_losses_db(band_name: str, kind: str, spacing_hpbw: Optional[float] = None) -> np.ndarray:
    """TX loss (dB) at uniform points of a lattice cell of the sub-aperture beam: the hexagonal cell of the full face
    (BEAM_SPACING_HPBW x its HPBW) stretched by the beamwidth ratios, its first lattice axis along the NARROW beam
    axis (so the two halves, rotated by 90 deg, see the same cell), real 2D pattern of the sub-aperture."""
    sa = subaperture(band_name, kind)
    p = _hex_cell_points(_spacing(spacing_hpbw) * PATTERNS[band_name].tx.hpbw)
    ku, kv = sa.hpbw_u / sa.full_hpbw_u, sa.hpbw_v / sa.full_hpbw_v
    u, v = (p[:, 0] * ku, p[:, 1] * kv) if sa.hpbw_u <= sa.hpbw_v else (p[:, 1] * ku, p[:, 0] * kv)
    return -10.0 * np.log10(np.maximum(sa.power(u, v), 1e-12))


def tx_subaperture(wf: "Waveform") -> Optional[str]:
    """Sub-aperture of a partition waveform ('half_h', 'half_v', 'quarter'), None otherwise."""
    if wf.tx_mode != "partition":
        return None
    return "quarter" if wf.b_tx == 4 else f"half_{wf.tx_cut}"


def tx_gain_ratio(wf: "Waveform") -> float:
    """TX gain of one beam / that of the full face: N_sub / N for a partition (uniform), 1 otherwise."""
    kind = tx_subaperture(wf)
    if kind is None:
        return 1.0
    sa = subaperture(wf.band, kind)
    return sa.n_elements / sa.n_total


def tx_beam_power_w(wf: "Waveform") -> float:
    """Peak power of one beam: the face peak power shared by the B beams (superposition: every TRM, partition: the
    sub-aperture's TRMs), or all of it for one beam and for each pulse of the time multiplex."""
    share = wf.b_tx if wf.tx_mode in ("superposition", "partition") else 1
    return BANDS[wf.band].p_pk_total_w / share


def cells_per_dwell(wf: "Waveform") -> float:
    """Positions of the full-face search lattice one dwell covers: B beams, each covering one cell, or the
    sub-aperture's larger cell for a partition."""
    kind = tx_subaperture(wf)
    return wf.b_tx * (1.0 if kind is None else subaperture_cell_ratio(wf.band, kind))


def partition_cuts() -> Tuple[str, ...]:
    """Cuts of the 2-beam partition the optimiser evaluates: 'h' and 'v', or 'h' alone when the two halves are the
    same beam rotated by 90 deg (x <-> y symmetric face: same gain, cell and grid losses, so the same cost; exact)."""
    a, b = subaperture("S", "half_h"), subaperture("S", "half_v")
    same = (a.n_elements == b.n_elements and math.isclose(a.hpbw_u, b.hpbw_v, rel_tol=1e-9)
            and math.isclose(a.hpbw_v, b.hpbw_u, rel_tol=1e-9)
            and np.allclose(subaperture_grid_losses_db("S", "half_h"), subaperture_grid_losses_db("S", "half_v"),
                            rtol=0.0, atol=1e-9))
    return ("h",) if same else ("h", "v")


def target_coherence_loss_db(t_block_s: float, t0_s: float) -> float:
    """Target decorrelation over a coherent block: -10 log exp(-(T / T0)^p), T0 of the target class (step 2.4)."""
    if not ENABLE_COH_LOSS or t_block_s <= 0.0 or math.isinf(t0_s):
        return 0.0
    return 10.0 * math.log10(math.e) * (t_block_s / t0_s) ** COH_P


def lo_ssb_dbc_hz(f_hz, carrier_ghz: float):
    """LO single-sideband phase noise at offset f (Hz), scaled to the carrier: +20 log(f_c / ref)."""
    lf = np.interp(np.log10(np.asarray(f_hz, dtype=float)), np.log10(LO_OFFSETS_HZ), LO_SSB_DBC_HZ)
    return lf + 20.0 * math.log10(carrier_ghz / LO_REF_GHZ)


@functools.lru_cache(maxsize=1 << 16)   # pure (step 3.1)
def oscillator_phase_var(f_ghz: float, t_block_s: float, r_km: float, sig_bw_mhz: float) -> float:
    """Residual phase variance (rad^2) of a coherent block (step 2.2, A2-A3): one LO for TX and RX, its phase noise
    S = 2 x 10^(L/10) filtered by the round trip 4 sin^2(pi f tau_rt), from 1 / T_block (slower: absorbed by the
    Doppler estimate) to B / 2; plus the residual platform motion (4 pi delta / lambda)^2."""
    tau_rt = 2.0 * r_km * 1e3 / C_MPS
    f_lo, f_hi = 1.0 / max(t_block_s, 1e-9), max(sig_bw_mhz * 1e6 / 2.0, 2.0 / max(t_block_s, 1e-9))
    f = np.logspace(math.log10(f_lo), math.log10(f_hi), 2000)
    s = 2.0 * 10.0 ** (lo_ssb_dbc_hz(f, f_ghz) / 10.0) * 4.0 * np.sin(math.pi * f * tau_rt) ** 2
    lo = float(np.sum(0.5 * (s[1:] + s[:-1]) * np.diff(f)))
    return lo + (4.0 * math.pi * PLATFORM_MOTION_RMS_M / lambda_m(f_ghz)) ** 2


def coherence_loss_db(f_ghz: float, t_block_s: float, r_km: float = 0.0, sig_bw_mhz: float = 0.0,
                      target: Optional[Target] = None) -> float:
    """Loss of a coherent block: target decorrelation (T0 of the target at f; None = the default class) + oscillator
    and platform residual phase (4.34 sigma^2)."""
    if not ENABLE_COH_LOSS or t_block_s <= 0.0:
        return 0.0
    t0 = (target or TARGETS[DEFAULT_TARGET_TYPE]).t0_s(f_ghz)
    return (target_coherence_loss_db(t_block_s, t0)
            + 10.0 * math.log10(math.e) * oscillator_phase_var(f_ghz, t_block_s, r_km, sig_bw_mhz))


def doppler_floor_dbc(f_ghz: float, t_block_s: float, r_km: float, offsets_hz=(100.0, 1e3, 1e4)) -> Dict[str, float]:
    """Phase-noise floor in one Doppler cell (1 / T_block wide) at offsets from a target, relative to the target's
    integrated peak (dBc): L(f) + 10 log(4 sin^2(pi f tau_rt)) + 10 log(2 / T_block) (output, step 2.2 A4)."""
    tau_rt = 2.0 * r_km * 1e3 / C_MPS
    out = {}
    for fo in offsets_hz:
        rt = 4.0 * math.sin(math.pi * fo * tau_rt) ** 2
        out[f"{fo:g} Hz"] = (float(lo_ssb_dbc_hz(fo, f_ghz)) + lin_to_db(max(rt, 1e-30))
                             + 10.0 * math.log10(2.0 / max(t_block_s, 1e-9)))
    return out


def migration_loss_db(v_mps: float, t_span_s: float, sig_bw_mhz: float, n_pulses: int) -> Tuple[float, float]:
    """UNCOMPENSATED processing (migration.mode): energy spread over the cells crossed, 10 log(min(cells, N)).
    Returns (loss_db, n_range_cells_crossed). The COMPENSATED case is the velocity bank (velocity_bank)."""
    dr = C_MPS / (2.0 * sig_bw_mhz * 1e6)
    n_cells = 1.0 + v_mps * t_span_s / dr
    return 10.0 * math.log10(max(1.0, min(n_cells, float(n_pulses)))), n_cells


# -----------------------------------------------------------------------------
# Processing (step 2.2): coherent blocks, velocity bank, straddle, CFAR, independent cells
# -----------------------------------------------------------------------------
def sine_integral(x: float) -> float:
    """Si(x) = int_0^x sin(t) / t dt, x >= 0: power series up to 4, beyond pi/2 - f cos x - g sin x with the rational
    auxiliary functions of Abramowitz & Stegun 5.2.38-39 (|error| < 5e-7)."""
    if x <= 4.0:
        term, total, k, x2 = x, x, 0, x * x
        while abs(term) > 1e-17 * max(1.0, abs(total)):
            term *= -x2 * (2 * k + 1) / ((2 * k + 2) * (2 * k + 3) ** 2)
            total += term
            k += 1
        return total
    x2 = x * x
    f = ((((x2 + 38.027264) * x2 + 265.187033) * x2 + 335.677320) * x2 + 38.102495) / \
        (x * ((((x2 + 40.021433) * x2 + 322.624911) * x2 + 570.236280) * x2 + 157.105423))
    g = ((((x2 + 42.242855) * x2 + 302.757865) * x2 + 352.018498) * x2 + 21.821899) / \
        (x2 * ((((x2 + 48.196927) * x2 + 482.485984) * x2 + 1114.978885) * x2 + 449.690326))
    return math.pi / 2.0 - f * math.cos(x) - g * math.sin(x)


@functools.lru_cache(maxsize=1 << 16)   # pure; asked ~10^5 times per sector (step 3.1)
def walk_loss_db(m_cells: float, coherent: bool) -> float:
    """Loss of a target walking m cells (c / 2B) across a dwell, relative to the bank filter: the compressed range
    response sinc(m x), x in [-1/2, 1/2], summed coherently (-20 log mean sinc) or in power (-10 log mean sinc^2).
    Closed forms (step 3.1, X2; were a 401-point mean): mean sinc = 2 Si(pi m / 2) / (pi m), mean sinc^2 =
    2 [Si(pi m) - sin^2(pi m / 2) / (pi m / 2)] / (pi m)."""
    if m_cells <= 0.0:
        return 0.0
    a = math.pi * m_cells
    if coherent:
        mean = 2.0 * sine_integral(a / 2.0) / a
        return -20.0 * math.log10(max(abs(mean), 1e-12))
    mean = 2.0 * (sine_integral(a) - math.sin(a / 2.0) ** 2 / (a / 2.0)) / a
    return -10.0 * math.log10(max(mean, 1e-12))


def bank_mismatch_loss_db(dv_mps: float, tau_s: float, sig_bw_mhz: float, t_obs_s: float, n: int,
                          coherent: bool) -> float:
    """Loss of a velocity filter dv off the target (step 2.2, B1): intra-pulse (2.1) + walk over the dwell (N > 1)."""
    dr = C_MPS / (2.0 * sig_bw_mhz * 1e6)
    loss = velocity_mismatch_loss_db(round(cells_in_pulse(dv_mps, tau_s, sig_bw_mhz), 6))
    if n > 1:
        loss += walk_loss_db(abs(dv_mps) * t_obs_s / dr, coherent)
    return loss


@functools.lru_cache(maxsize=1 << 14)
def velocity_bank(v_span_mps: float, tau_s: float, sig_bw_mhz: float, t_obs_s: float, n: int,
                  coherent: bool) -> Tuple[int, float, float]:
    """Velocity-hypothesis bank over v_span (step 2.2, B2-B3): step such that a target half a step off loses
    BANK_MAX_LOSS_DB; returns (hypotheses, mean loss over a uniform offset in +/- half a step, half step in m/s)."""
    f = lambda dv: bank_mismatch_loss_db(dv, tau_s, sig_bw_mhz, t_obs_s, n, coherent)   # noqa: E731
    lo, hi = 0.0, max(v_span_mps, 1.0)
    if f(hi) <= BANK_MAX_LOSS_DB:
        half = hi
    else:
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if f(mid) <= BANK_MAX_LOSS_DB else (lo, mid)
        half = lo
    n_hyp = max(1, int(math.ceil(v_span_mps / (2.0 * half)))) if v_span_mps > 0 else 1
    half_eff = min(half, v_span_mps / 2.0) if v_span_mps > 0 else 0.0
    mean = float(np.mean([f(d) for d in np.linspace(0.0, half_eff, 21)])) if half_eff > 0 else 0.0
    return n_hyp, mean, half_eff


def _quantise_shape(k: float) -> float:
    """Gamma shape rounded to 1/100 of a decade (2.3 % steps, a few 0.01 dB): bounded number of detection laws."""
    return k if math.isinf(k) or k <= 0.0 else float(10.0 ** (round(math.log10(k) * 100.0) / 100.0))


def diversity_factor(target: Target, f_ghz: float) -> float:
    """Polarisation diversity of the two receive channels (step 2.4, E2): the copolar and cross-polar returns fluctuate
    independently, so the degrees of freedom grow by (1 + rho)^2 / (1 + rho^2), rho = cross / copolar RCS. 1 with a
    single-polarisation receiver."""
    band = band_for_frequency(f_ghz)
    if band is None or not band.pol_rx_dual:
        return 1.0
    rho = target.depol_lin(f_ghz)
    return (1.0 + rho) ** 2 / (1.0 + rho * rho)


def frequency_draws(target: Target, agility_mhz: float) -> int:
    """Independent target draws of a frequency excursion (step 3.1, B7): floor(df / (c / 2L)) + 1, L the target
    length (decorrelation frequency c / 2L: 15 MHz at 10 m, 100 MHz at 1.5 m)."""
    if agility_mhz <= 0 or not target.length_m or target.length_m <= 0:
        return 1
    return int(math.floor(agility_mhz * 1e6 / (C_MPS / (2.0 * target.length_m)) + 1e-9)) + 1


def look_fluctuation(target: Target, f_ghz: float, swerling: Optional[int], n_samples: int,
                     t_obs_s: float, agility_mhz: float = 0.0) -> detection.Fluct:
    """Fluctuation of a look of n_samples noncoherent samples spread over t_obs_s (step 2.4, B3 + E2), as the Gamma
    shape of the target power: a declared Swerling case (k = 1, 2, n or 2n, infinite for SW0), else the band's family
    x the independent samples min(n, 1 + T_obs / T0) (SW1/2: 1 per sample, SW3/4: 2, SW0: steady); times the
    polarisation diversity. Frequency agility (step 3.1, B7) multiplies the independent samples by its draws (capped
    at n): a declared SW1 / SW3 then tends to SW2 / SW4."""
    draws = frequency_draws(target, agility_mhz)
    if swerling is not None:
        if swerling in (1, 3) and draws > 1:
            k = detection.shape_of(swerling, 1) * min(float(n_samples), float(draws))
        else:
            k = detection.shape_of(swerling, n_samples)
    else:
        base = FLUCTUATIONS[target.fluctuation(f_ghz)]
        t0 = target.t0_s(f_ghz)
        k = base * min(float(n_samples), (1.0 + (0.0 if math.isinf(t0) else t_obs_s / t0)) * draws)
    return detection.Fluct(_quantise_shape(k * diversity_factor(target, f_ghz)))


def coherent_blocks(n: int, pri_s: float, tau_s: float, integration: str, swerling: Optional[int],
                    target: Optional[Target] = None, f_ghz: Optional[float] = None) -> Tuple[int, float]:
    """Number of coherent blocks M (a divisor of N) and the block duration (step 2.2, A7): NONCOHERENT or a declared
    fast-fluctuating case (SW2 / SW4) = pulse by pulse; COHERENT = the M minimising SNR_req(M noncoherent blocks, with
    the look's fluctuation over M blocks, step 2.4) + target coherence loss of one block (T0 of the target at f; the
    oscillator term is negligible and left out of the choice). target None = the default class, f None = the CLI
    frequency."""
    if integration != "COHERENT" or n == 1 or swerling in (2, 4):
        return n, tau_s
    tgt = target or TARGETS[DEFAULT_TARGET_TYPE]
    f = FREQUENCY_GHZ if f_ghz is None else f_ghz
    t0 = tgt.t0_s(f)
    t_span = (n - 1) * pri_s + tau_s
    best = None
    for m in (d for d in range(1, n + 1) if n % d == 0):
        t_b = (n // m - 1) * pri_s + tau_s
        cost = (shnidman_total_snr_db(DETECT_PD, DETECT_PFA, m, look_fluctuation(tgt, f, swerling, m, t_span))
                + target_coherence_loss_db(t_b, t0))
        if best is None or cost < best[0] - 1e-12:
            best = (cost, m, t_b)
    return best[1], best[2]


def acceleration_loss_db(a_mps2: float, f_ghz: float, t_block_s: float, t_span_s: float, sig_bw_mhz: float,
                         coherent: bool) -> float:
    """Loss of a filter matched to the wrong line-of-sight acceleration a (step 2.4, C1): quadratic phase over a
    coherent block, 2 pi (a T_b^2 / lambda) x^2 (same law as the velocity mismatch, 2.1), plus the range curvature
    over the dwell left by the linear (velocity) compensation, a T^2 / 8 in cells c / 2B (walk law of 2.2)."""
    if a_mps2 <= 0.0:
        return 0.0
    dr = C_MPS / (2.0 * sig_bw_mhz * 1e6)
    phase = _quadratic_phase_loss_db(a_mps2 * t_block_s ** 2 / lambda_m(f_ghz)) if t_block_s > 0 else 0.0
    return phase + walk_loss_db(a_mps2 * t_span_s ** 2 / (8.0 * dr), coherent)


@functools.lru_cache(maxsize=1)
def _quadratic_phase_table() -> Tuple[np.ndarray, np.ndarray]:
    n = np.linspace(0.0, 16.0, 3201)
    x = (np.arange(4001) + 0.5) / 4001 - 0.5              # same sampling as velocity_mismatch_loss_db
    loss = np.concatenate([-20.0 * np.log10(np.abs(np.exp(2j * math.pi * c[:, None] * x * x).mean(axis=1)))
                           for c in np.array_split(n, 16)])
    return n, loss


def _quadratic_phase_loss_db(n_c: float) -> float:
    """velocity_mismatch_loss_db interpolated on a 0.005 grid up to n_c = 16 (exact beyond): the acceleration bank
    evaluates it for continuous values (step 2.4)."""
    grid, loss = _quadratic_phase_table()
    return float(np.interp(n_c, grid, loss)) if n_c <= grid[-1] else velocity_mismatch_loss_db(round(n_c, 6))


@functools.lru_cache(maxsize=1 << 14)
def acceleration_bank(a_max_mps2: float, f_ghz: float, t_block_s: float, t_span_s: float, sig_bw_mhz: float,
                      coherent: bool) -> Tuple[int, float]:
    """Acceleration hypotheses over +/- a_max (step 2.4, C2): none while a_max itself costs at most BANK_MAX_LOSS_DB
    (mean loss over a uniform in [0, a_max] to the budget), else a bank whose step leaves BANK_MAX_LOSS_DB at half a
    step (mean loss over the half step). Returns (hypotheses, mean loss dB); they multiply the velocity hypotheses."""
    if a_max_mps2 <= 0.0:
        return 1, 0.0
    f = lambda a: acceleration_loss_db(a, f_ghz, t_block_s, t_span_s, sig_bw_mhz, coherent)   # noqa: E731
    half = a_max_mps2
    if f(a_max_mps2) > BANK_MAX_LOSS_DB:
        lo, hi = 0.0, a_max_mps2
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if f(mid) <= BANK_MAX_LOSS_DB else (lo, mid)
        half = max(lo, 1e-12)
    n_hyp = max(1, int(math.ceil(a_max_mps2 / half - 1e-9)))
    return n_hyp, float(np.mean([f(a) for a in np.linspace(0.0, half, 21)]))


def _sampled_mean_loss_db(response, half: float) -> float:
    x = np.linspace(-half, half, 801)
    return -10.0 * math.log10(float(np.mean(response(x))))


@functools.lru_cache(maxsize=32)
def _range_response(psl_db: float):
    """Compressed range response (power, peak 1) of the look's weighting (Taylor at psl; same main lobe for NLFM),
    as a function of the offset in cells c / 2B."""
    n, nbar = 2048, RANGE_TAYLOR_NBAR
    a = math.acosh(10.0 ** (abs(psl_db) / 20.0)) / math.pi
    s2 = nbar ** 2 / (a ** 2 + (nbar - 0.5) ** 2)
    ms = range(1, nbar)
    x = (np.arange(n) - (n - 1) / 2.0) / n
    w = np.ones(n)
    for m in ms:
        num = np.prod([1.0 - m ** 2 / s2 / (a ** 2 + (k - 0.5) ** 2) for k in ms])
        den = np.prod([1.0 - m ** 2 / k ** 2 for k in ms if k != m])
        w += 2.0 * (-1) ** (m + 1) * num / (2.0 * den) * np.cos(2.0 * math.pi * m * x)
    return lambda u: (np.cos(2.0 * math.pi * np.outer(np.atleast_1d(u), x)) @ w / w.sum()) ** 2


@functools.lru_cache(maxsize=32)
def straddle_range_db(psl_db: float) -> float:
    """Mean range straddle on the detection grid (RANGE_GRID_OVERSAMPLING samples per c / 2B), with the compressed
    response of the look's weighting, target uniform between samples (output since step 2.3)."""
    return _sampled_mean_loss_db(_range_response(psl_db), 0.5 / RANGE_GRID_OVERSAMPLING)


@functools.lru_cache(maxsize=4)
def straddle_doppler_db() -> float:
    """Mean Doppler straddle of an unweighted transform zero-padded DOPPLER_ZERO_PADDING times (sinc^2 response)."""
    return _sampled_mean_loss_db(lambda u: np.sinc(u) ** 2, 0.5 / DOPPLER_ZERO_PADDING)


@functools.lru_cache(maxsize=64)
def cfar_loss_db(n_ref: int = CFAR_REF_CELLS, pfa: Optional[float] = None) -> float:
    """CA-CFAR loss, exact for Swerling 1 on one sample at the model Pd and the look's Pfa (step 2.2, D3): threshold
    alpha = N (Pfa^(-1/N) - 1), Pd = (1 + alpha / (N (1 + S)))^(-N), against the known-noise SNR ln Pfa / ln Pd - 1."""
    pfa = DETECT_PFA if pfa is None else pfa
    ideal = math.log(pfa) / math.log(DETECT_PD) - 1.0
    if CFAR_TYPE == "OS":
        return os_cfar_loss_db(n_ref, pfa)
    if CFAR_TYPE == "GO":
        return go_cfar_loss_db(n_ref, pfa)
    alpha = n_ref * (pfa ** (-1.0 / n_ref) - 1.0)
    s = alpha / (n_ref * (DETECT_PD ** (-1.0 / n_ref) - 1.0)) - 1.0
    return 10.0 * math.log10(s / ideal)


@functools.lru_cache(maxsize=64)
def os_cfar_loss_db(n_ref: int = CFAR_REF_CELLS, pfa: Optional[float] = None) -> float:
    """OS-CFAR loss (step 2.4, D3), exact for Swerling 1 on one sample: the threshold is T x the k-th smallest of the
    N reference cells, k = CFAR_OS_RANK_FRAC x N; Pfa(T) = prod_{i<k} (N - i) / (N - i + T) and Pd = Pfa(T / (1 + S)),
    against the known-noise SNR ln Pfa / ln Pd - 1. Robust to neighbouring targets in the reference window."""
    pfa = DETECT_PFA if pfa is None else pfa
    k = min(n_ref, max(1, int(round(CFAR_OS_RANK_FRAC * n_ref))))
    i = np.arange(k)

    def log_p(t: float) -> float:
        return float(np.sum(np.log((n_ref - i) / (n_ref - i + t))))

    def solve_t(target_log: float) -> float:
        lo, hi = 0.0, 1.0
        while log_p(hi) > target_log:
            hi *= 2.0
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if log_p(mid) > target_log else (lo, mid)
        return 0.5 * (lo + hi)

    t = solve_t(math.log(pfa))
    s = t / solve_t(math.log(DETECT_PD)) - 1.0
    return 10.0 * math.log10(s / (math.log(pfa) / math.log(DETECT_PD) - 1.0))


def _go_log_pfa(t: float, n: int) -> float:
    """log P(false alarm) of a GO-CFAR on Swerling 1, one sample, two halves of n cells, threshold t x max(half
    means) (Hansen and Sawyers): 2 [(1 + t/n)^-n - (2 + t/n)^-n sum_k C(n - 1 + k, k) (2 + t/n)^-k]."""
    a, b = 1.0 + t / n, 2.0 + t / n
    k = np.arange(n)
    log_terms = (np.array([math.lgamma(n + x) - math.lgamma(x + 1.0) - math.lgamma(n) for x in k]) - k * math.log(b))
    m = log_terms.max()
    log_sum = m + math.log(float(np.exp(log_terms - m).sum()))
    first, second = -n * math.log(a), -n * math.log(b) + log_sum
    return math.log(2.0) + first + math.log1p(-math.exp(second - first))


@functools.lru_cache(maxsize=64)
def go_cfar_loss_db(n_ref: int = CFAR_REF_CELLS, pfa: Optional[float] = None) -> float:
    """GO-CFAR loss (step 2.5, G1), exact for Swerling 1 on one sample: the larger of the two half-window means
    (N / 2 cells each) sets the threshold, so the Pfa holds at a clutter edge; Pd = Pfa(T / (1 + S))."""
    pfa = DETECT_PFA if pfa is None else pfa
    n = max(1, n_ref // 2)

    def solve_t(target_log: float) -> float:
        lo, hi = 0.0, 1.0
        while _go_log_pfa(hi, n) > target_log:
            hi *= 2.0
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if _go_log_pfa(mid, n) > target_log else (lo, mid)
        return 0.5 * (lo + hi)

    s = solve_t(math.log(pfa)) / solve_t(math.log(DETECT_PD)) - 1.0
    return 10.0 * math.log10(s / (math.log(pfa) / math.log(DETECT_PD) - 1.0))


@functools.lru_cache(maxsize=32)
def masking_levels_db(psl_db: float, doppler: bool) -> Dict[str, Dict[str, float]]:
    """Masking by a strong neighbour (step 2.4, D1-D2, output only: the scene is not known): envelope of the strong
    target's response (dB under its peak) at MASKING_OFFSETS_CELLS cells or more, in range (compressed pulse with the
    look's weighting) and in Doppler (unweighted transform of a coherent block, doppler_window 'none'). A target
    fainter than the neighbour by more than this, that far or farther, may be hidden."""
    u = np.arange(0.0, 12.0, 0.005)
    rng = 10.0 * np.log10(np.maximum(_range_response(psl_db)(u), 1e-30))
    out = {"range": {f"{k} cells": float(rng[u >= k].max()) for k in MASKING_OFFSETS_CELLS}}
    if doppler:
        dop = 10.0 * np.log10(np.maximum(np.sinc(u) ** 2, 1e-30))
        out["doppler"] = {f"{k} cells": float(dop[u >= k].max()) for k in MASKING_OFFSETS_CELLS}
    return out


# -----------------------------------------------------------------------------
# Detection criterion (step 2.3): random position of the target, cumulative probability, false-alarm policy
# -----------------------------------------------------------------------------
def _hex_cell_points(spacing: float, n_side: int = 40) -> np.ndarray:
    """Uniform points of the Voronoi cell (hexagon) of a triangular lattice of step `spacing` (deterministic)."""
    g = (np.arange(-n_side, n_side + 1) + 0.5) / n_side * spacing
    xx, yy = np.meshgrid(g, g)
    p = np.c_[xx.ravel(), yy.ravel()]
    a1, a2 = np.array([spacing, 0.0]), np.array([spacing / 2.0, spacing * math.sqrt(3.0) / 2.0])
    sites = np.array([i * a1 + j * a2 for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j])
    d0 = (p * p).sum(1)
    keep = np.all(d0[:, None] <= ((p[:, None, :] - sites[None]) ** 2).sum(2) + 1e-12, axis=1)
    return p[keep]


def _beam_loss_db(beam: "antenna.Beam", r_lam_d: np.ndarray) -> np.ndarray:
    """Gain loss (dB) at a distance r (lambda / D_env) from the beam axis: azimuthal mean of the computed pattern."""
    near = np.array(beam.mean_near)
    g = np.interp(r_lam_d, np.arange(len(near)) * antenna.R_STEP, near)
    return -10.0 * np.log10(np.maximum(g, 1e-12))


POS_LAW_STEP_DB = P.get("tool.detection.pos_law_step_db")   # 0 = exact laws (A1, 25 October 2026)

@functools.lru_cache(maxsize=P.get("tool.detection.position_cache_size"))   # exact (step 3.1 A3); bounded at T3
def position_losses(band_name: str, rx_sll_db: float, psl_db: float, search: bool, doppler: bool,
                    bank: Tuple[float, float, float, float, int, bool],
                    tx_sub: Optional[str] = None, doppler_window_db: Optional[float] = None,
                    spacing_hpbw: Optional[float] = None,
                    pointing: Optional[Tuple[str, float]] = None) -> Tuple[Tuple[float, ...], Tuple[float, ...], Dict[str, float]]:
    """Discrete law of the losses due to the random position of the target (step 2.3, B1): search TX lattice (hexagon
    of BEAM_SPACING_HPBW TX beamwidths, real pattern; tx_sub = the sub-aperture of a partition, its own stretched
    lattice and 2D pattern, step 3.1 C), RX beam lattice, range straddle on the detection grid, Doppler
    straddle (coherent blocks of more than one pulse), offset to the nearest velocity filter (bank = (half step,
    tau, B, T_obs, N, coherent); half step 0 = none). Each to OFFSET_QUANTILES quantiles, summed as independent.
    pointing (step 3.3, B1 / B4): ("gauss", sigma) = a track look pointed at the predicted direction, error sigma per
    axis in TX beamwidths (Rayleigh offsets on the real TX pattern); ("cluster", s) = the cell of a hexagonal cluster
    of spacing s TX beamwidths. RX: a gauss look forms its RX beam on the predicted direction (same offset), a cluster
    uses the digital RX lattice of the search (grid_rx).
    Returns (losses dB, weights, mean of each component)."""
    law, means = _fixed_position_law(band_name, rx_sll_db, psl_db, search, doppler, tx_sub, doppler_window_db,
                                     _spacing(spacing_hpbw), pointing)
    means = dict(means)
    half, tau_s, bw, t_obs, n, coh = bank
    if half > 0.0:
        v = np.array([bank_mismatch_loss_db(dv, tau_s, bw, t_obs, n, coh) for dv in (np.arange(41) + 0.5) / 41 * half])
        law = detection.combine((np.array(law[0]), np.array(law[1])), detection.quantiles(v, OFFSET_QUANTILES),
                                2 * OFFSET_QUANTILES)
        means["velocity_filter"] = float(np.mean(v))
    if POS_LAW_STEP_DB > 0.0:
        # losses on a POS_LAW_STEP_DB grid (25 October 2026, A1): the velocity-filter part made ~14 000 distinct laws
        # per sector and frequency, ~3 000 once rounded to 0.02 dB; required SNR within 0.003 dB (measured)
        losses = tuple(float(x) for x in np.round(np.asarray(law[0]) / POS_LAW_STEP_DB) * POS_LAW_STEP_DB)
        return losses, tuple(law[1]), means
    return tuple(law[0]), tuple(law[1]), means


@functools.lru_cache(maxsize=None)
def _fixed_position_law(band_name: str, rx_sll_db: float, psl_db: float, search: bool, doppler: bool,
                        tx_sub: Optional[str], doppler_window_db: Optional[float] = None,
                        spacing_hpbw: float = BEAM_SPACING_HPBW,
                        pointing: Optional[Tuple[str, float]] = None) -> Tuple[Tuple[Tuple[float, ...], Tuple[float, ...]], Tuple]:
    """The part of position_losses that does not depend on the velocity bank (grids, straddles), combined in the same
    order; cached apart (a few dozen keys) since the bank changes with every waveform (exact)."""
    q = OFFSET_QUANTILES
    parts: Dict[str, np.ndarray] = {}
    pat = PATTERNS[band_name]
    if search and ENABLE_GRID_LOSS:
        if tx_sub is None:
            tx = _hex_cell_points(spacing_hpbw * pat.tx.hpbw)
            parts["grid_tx"] = _beam_loss_db(pat.tx, np.hypot(tx[:, 0], tx[:, 1]))
        else:
            parts["grid_tx"] = subaperture_grid_losses_db(band_name, tx_sub, spacing_hpbw)
        rxb = rx_beam(band_name, rx_sll_db)
        rx = _hex_cell_points(RX_BEAM_SPACING_HPBW * rxb.hpbw)
        parts["grid_rx"] = _beam_loss_db(rxb, np.hypot(rx[:, 0], rx[:, 1]))
    if pointing is not None:
        kind, val = pointing
        if kind == "gauss":
            w = (np.arange(401) + 0.5) / 401
            r = float(val) * np.sqrt(-2.0 * np.log(1.0 - w))          # Rayleigh quantiles, sigma per axis
        else:
            c = _hex_cell_points(float(val))
            r = np.hypot(c[:, 0], c[:, 1])
        parts["pointing_tx"] = _beam_loss_db(pat.tx, r * pat.tx.hpbw)
        rxb = rx_beam(band_name, rx_sll_db)
        if kind == "gauss":                     # RX beam formed on the predicted direction: the same offset
            parts["pointing_rx"] = _beam_loss_db(rxb, r * pat.tx.hpbw)
        elif "grid_rx" not in parts:            # cluster: the digital RX lattice of each beam
            rx = _hex_cell_points(RX_BEAM_SPACING_HPBW * rxb.hpbw)
            parts["grid_rx"] = _beam_loss_db(rxb, np.hypot(rx[:, 0], rx[:, 1]))
    u = (np.arange(401) + 0.5) / 401 - 0.5
    parts["straddle_range"] = -10.0 * np.log10(_range_response(psl_db)(u / RANGE_GRID_OVERSAMPLING))
    if doppler:
        # unweighted transform: sinc^2; Taylor Doppler weighting (step 3.1, P4 Q5): its filter response, the same
        # 1D Taylor law as the range weighting
        resp = (np.sinc(u / DOPPLER_ZERO_PADDING) ** 2 if doppler_window_db is None
                else _range_response(doppler_window_db)(u / DOPPLER_ZERO_PADDING))
        parts["straddle_doppler"] = -10.0 * np.log10(resp)
    law = (np.zeros(1), np.ones(1))
    for v in parts.values():
        law = detection.combine(law, detection.quantiles(v, q), 2 * q)
    return (tuple(law[0]), tuple(law[1])), tuple((k, float(np.mean(v))) for k, v in parts.items())


@functools.lru_cache(maxsize=4)
def array_errors(band_name: str) -> Dict[str, float]:
    """Figures of the residual array errors of a band (step 3.4, B4-B6): gain loss TX + RX, mean sidelobe floor
    relative to the peak (random: sigma^2 / (eta N); per TRSA: sigma_c^2 / (eta N_TRSA)), boresight error per axis
    from the TRSA-correlated phase (sigma_c lambda / (2 pi x_rms sqrt(N_TRSA))), calibration share of the face time."""
    band = BANDS[band_name]
    er = band.errors
    rnd, cor = _error_variances(er)
    f = 0.5 * (band.core_lo_ghz + band.core_hi_ghz)
    eta = rx_beam(band_name).eta
    floor = rnd / (eta * band.n_trm) + cor / (eta * N_TRSA)
    t_min = er["cal_interval_s"] / 60.0
    ph_c = math.sqrt(math.radians(er["trsa_phase_deg"]) ** 2 + math.radians(er["drift_phase_deg_min"] * t_min) ** 2)
    bore = ph_c * lambda_m(f) / (2.0 * math.pi * _pattern(f).x_rms_m * math.sqrt(N_TRSA))
    return {"loss_db": band.losses_db["calibration"], "sidelobe_floor_db": lin_to_db(floor),
            "boresight_mrad": bore * 1e3, "cal_time_frac": er["cal_duration_s"] / er["cal_interval_s"]}


def calibration_checks() -> List[str]:
    """Consistency of the inputs with the array errors (step 3.4, B5-B6): the mean sidelobe floor must stay 10 dB
    under the strictest RX option, the angle floor and the S/Ku bias must not be finer than the boresight errors."""
    out = []
    for name in BANDS:
        a = array_errors(name)
        if a["sidelobe_floor_db"] > min(RX_SLL_OPTIONS_DB) - 10.0:
            out.append(f"{name}: mean sidelobe floor of the array errors {a['sidelobe_floor_db']:.1f} dB is within 10 dB "
                       f"of the {min(RX_SLL_OPTIONS_DB):g} dB RX option")
        if P.get("tracking.angle_floor_mrad") < a["boresight_mrad"]:
            out.append(f"{name}: tracking.angle_floor_mrad {P.get('tracking.angle_floor_mrad'):g} is finer than the "
                       f"boresight error of the array errors {a['boresight_mrad']:.3f} mrad")
    bs = math.hypot(array_errors("S")["boresight_mrad"], array_errors("Ku")["boresight_mrad"])
    if P.get("confirmation.bias_mrad") < bs:
        out.append(f"confirmation.bias_mrad {P.get('confirmation.bias_mrad'):g} is finer than the combined S / Ku "
                   f"boresight error {bs:.3f} mrad")
    return out


def search_pfa(fa_per_rr: float, positions: float, cells_per_position: float) -> float:
    """Pfa per independent cell of a search waveform (step 2.3, C1-C4): FA_MODE 'fa_rate' = fa_per_rr false alarms per
    sector and revisit over positions x cells, bounded to [PFA_MIN, PFA_MAX]; 'pfa_cell' = DETECT_PFA."""
    if FA_MODE != "fa_rate" or positions <= 0.0 or cells_per_position <= 0.0:
        return DETECT_PFA
    return detection.quantise_pfa(min(max(fa_per_rr / (positions * cells_per_position), PFA_MIN), PFA_MAX))


def independent_cells(range_min_km: float, range_max_km: float, sig_bw_mhz: float, doppler_cells: int,
                      velocity_hypotheses: int, angle_cells: float = 1.0) -> float:
    """Independent resolution cells tested per look (step 2.2, E1-E5): range (R_max - R_min) / (c / 2B) x Doppler x
    velocity hypotheses x angle. The oversampled grids are not counted (correlated samples)."""
    n_r = max(range_max_km - range_min_km, 0.0) * 1e3 / (C_MPS / (2.0 * sig_bw_mhz * 1e6))
    return n_r * doppler_cells * velocity_hypotheses * angle_cells


# -----------------------------------------------------------------------------
# Waveforms: compression and intra-pulse effects (step 2.1)
# -----------------------------------------------------------------------------
@functools.lru_cache(maxsize=32)
def range_weighting(psl_db: float) -> Dict[str, float]:
    """1D Taylor weighting (nbar = RANGE_TAYLOR_NBAR) of a compressed pulse at the range sidelobe level psl_db,
    evaluated numerically on 4096 spectral samples: SNR loss (dB), -3 dB main-lobe broadening against the
    unweighted pulse, RMS-bandwidth ratio (k_r = RANGE_CRLB_KR / ratio) and the achieved sidelobe level."""
    n, nbar = 4096, RANGE_TAYLOR_NBAR
    a = math.acosh(10.0 ** (abs(psl_db) / 20.0)) / math.pi
    s2 = nbar ** 2 / (a ** 2 + (nbar - 0.5) ** 2)
    ms = range(1, nbar)
    x = (np.arange(n) - (n - 1) / 2.0) / n
    w = np.ones(n)
    for m in ms:
        num = np.prod([1.0 - m ** 2 / s2 / (a ** 2 + (k - 0.5) ** 2) for k in ms])
        den = np.prod([1.0 - m ** 2 / k ** 2 for k in ms if k != m])
        w += 2.0 * (-1) ** (m + 1) * num / (2.0 * den) * np.cos(2.0 * math.pi * m * x)
    u = np.arange(0.0, 1.5, 1e-4)                     # main lobe, in resolution cells (1 / pulse spectrum width)

    def half_power(win: np.ndarray) -> float:
        # row blocks of cos(2 pi u x) @ win: same values, without a 15000 x 4096 matrix in memory
        g = np.concatenate([np.cos(2.0 * math.pi * np.outer(c, x)) @ win for c in np.array_split(u, 30)])
        g = (g / win.sum()) ** 2
        i = int(np.argmax(g < 0.5))
        return float(np.interp(0.5, [g[i], g[i - 1]], [u[i], u[i - 1]]))

    pw = np.abs(np.fft.rfft(w, 16 * n)) ** 2
    pw /= pw[0]
    k = int(np.argmax(pw < 0.5))
    while pw[k + 1] < pw[k]:
        k += 1
    return {"loss_db": -10.0 * math.log10(w.sum() ** 2 / (n * (w * w).sum())),
            "broadening": half_power(w) / half_power(np.ones(n)),
            "rms_ratio": math.sqrt((w * x * x).sum() / w.sum()) / math.sqrt((x * x).mean()),
            "psl_db": 10.0 * math.log10(pw[k:].max())}


def range_psl_db(rx_sll_db: Optional[float] = None) -> float:
    """Range sidelobe level of a look: the volume's RX option (waveform.range_psl = 'volume') or the fixed level."""
    if RANGE_PSL == "volume":
        return RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    return float(RANGE_PSL)


def doppler_max_hz(f_ghz: float, v_mps: float) -> float:
    return 2.0 * abs(v_mps) / lambda_m(f_ghz)


def modulation_for(f_ghz: float, sig_bw_mhz: float, v_mps: float) -> str:
    """Family of a look (step 2.1, A2): NLFM while f_d,max / B <= NLFM_DOPPLER_MAX_FRAC (and modelled), else LFM."""
    if "nlfm" in MODULATIONS and doppler_max_hz(f_ghz, v_mps) <= NLFM_DOPPLER_MAX_FRAC * sig_bw_mhz * 1e6:
        return "nlfm"
    return "lfm"


def compression_loss_db(modulation: str, f_ghz: float, sig_bw_mhz: float, v_mps: float, psl_db: float) -> float:
    """Pulse-compression loss: LFM = Taylor weighting loss + Doppler loss on the ridge -20 log(1 - f_d / B);
    NLFM = its mismatch loss (no weighting, Doppler within its tolerance)."""
    if modulation == "nlfm":
        return NLFM_MISMATCH_LOSS_DB
    frac = min(doppler_max_hz(f_ghz, v_mps) / (sig_bw_mhz * 1e6), 0.999)
    return range_weighting(psl_db)["loss_db"] - 20.0 * math.log10(1.0 - frac)


def range_resolution_m(sig_bw_mhz: float, psl_db: float) -> float:
    """Physical range resolution: -3 dB width of the compressed pulse = broadening x c / 2B (same for LFM and NLFM)."""
    return range_weighting(psl_db)["broadening"] * C_MPS / (2.0 * sig_bw_mhz * 1e6)


def cells_in_pulse(v_mps: float, tau_s: float, sig_bw_mhz: float) -> float:
    """Range cells (c / 2B) a target crosses during one pulse: n_c = 2 v tau B / c (step 2.1, B1)."""
    return 2.0 * abs(v_mps) * tau_s * sig_bw_mhz * 1e6 / C_MPS


@functools.lru_cache(maxsize=4096)
def velocity_mismatch_loss_db(n_c: float) -> float:
    """Loss of a filter matched to the wrong velocity, n_c = 2 |dv| tau B / c (B1): the echo time compression is a
    quadratic phase error 2 pi n_c x^2 over the pulse (x in [-1/2, 1/2]): -20 log |mean exp(j 2 pi n_c x^2)|."""
    x = (np.arange(4001) + 0.5) / 4001 - 0.5
    return -20.0 * math.log10(abs(np.exp(2j * math.pi * n_c * x * x).mean()))


@functools.lru_cache(maxsize=1)
def _n_c_for_db(loss_db: float = 1.0) -> float:
    lo, hi = 0.0, 4.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if velocity_mismatch_loss_db(mid) < loss_db else (lo, mid)
    return lo


def velocity_hypotheses(v_span_mps: float, tau_s: float, sig_bw_mhz: float, loss_db: float = 1.0) -> int:
    """Velocity filters needed over v_span so that the worst mismatch (half a step) costs at most loss_db (B2,
    output for the velocity bank of 2.2 and the compute load of 4.4)."""
    if v_span_mps <= 0.0:
        return 1
    dv_half = _n_c_for_db(loss_db) * C_MPS / (2.0 * tau_s * sig_bw_mhz * 1e6)
    return max(1, int(math.ceil(v_span_mps / (2.0 * dv_half))))


def am_pm_drift_deg(x_buf: float) -> float:
    """Phase drift over the pulse (B5): AM/PM x the end-of-pulse power drop -10 log(1 - x) of the TRM buffer."""
    return PA_AM_PM_DEG_PER_DB * (-10.0 * math.log10(max(1.0 - x_buf, 1e-12)))


def blind_range_km(band: BandConfig, tau_s: float) -> float:
    """Range hidden by the receiver deaf time (pulse + limiter recovery): c (tau + recovery) / 2 (step 2.1, C2)."""
    return C_MPS * rx_deaf_after_pulse_s(band, tau_s) / 2.0 / 1000.0


def eclipsing_loss_db(band: BandConfig, tau_s: float, pri_s: float) -> float:
    """Mean eclipsing loss of a staggered range-ambiguous train (C4): -10 log(1 - (tau + recovery) / PRI)."""
    return -10.0 * math.log10(max(1.0 - rx_deaf_after_pulse_s(band, tau_s) / pri_s, 1e-6))


def unambiguous_pri_s(band: BandConfig, r_km: float, tau_s: float) -> float:
    """Shortest PRI of a train with no ambiguity and no self-eclipsing out to r_km (C3): 2 R / c + tau + recovery."""
    return 2.0 * r_km * 1e3 / C_MPS + rx_deaf_after_pulse_s(band, tau_s)


# -----------------------------------------------------------------------------
# Detection SNR requirement (TOTAL integrated energy SNR, for the energy-form radar equation)
# -----------------------------------------------------------------------------
def albersheim_total_snr_db(pd: float, pfa: float, n: int) -> float:
    """Steady target, noncoherent integration of n pulses. Cross-check only."""
    a = math.log(0.62 / pfa)
    b = math.log(pd / (1.0 - pd))
    per_pulse_db = -5.0 * math.log10(n) + (6.2 + 4.54 / math.sqrt(n + 0.44)) * math.log10(a + 0.12 * a * b + 1.7 * b)
    return per_pulse_db + 10.0 * math.log10(n)


def shnidman_total_snr_db(pd: float, pfa: float, n: int, swerling: "detection.Fluctuation") -> float:
    """Shnidman empirical fit; returns the SNR required on the TOTAL energy of n noncoherently
    integrated pulses (= n x per-pulse SNR). Cases 0..4 (0 = steady), or a Gamma shape detection.Fluct(k) (step 2.4:
    the fit is written in k)."""
    if not (0.1 <= pd <= 0.99) or not (1e-9 <= pfa <= 1e-2) or n < 1:
        raise ValueError("Shnidman fit outside validity range")
    eta = math.sqrt(-0.8 * math.log(4.0 * pfa * (1.0 - pfa))) \
        + (1.0 if pd > 0.5 else -1.0) * math.sqrt(-0.8 * math.log(4.0 * pd * (1.0 - pd)))
    alpha = 0.25 if n > 40 else 0.0
    x_inf = eta * (eta + 2.0 * math.sqrt(n / 2.0 + (alpha - 0.25)))
    k = detection.shape_of(swerling, n)
    c_db = 0.0
    if math.isfinite(k):
        c1 = (((17.7006 * pd - 18.4496) * pd + 14.5339) * pd - 3.525) / k
        c2 = (math.exp(27.31 * pd - 25.14)
              + (pd - 0.8) * (0.7 * math.log(1e-5 / pfa) + (2.0 * n - 20.0) / 80.0)) / k
        c_db = c1 + (c2 if pd > 0.872 else 0.0)
    return 10.0 * math.log10(x_inf) + c_db


def detect_snr_req_db(integration: str, n: int, swerling: int, blocks: Optional[int] = None) -> float:
    """SNR on the total energy: COHERENT = `blocks` coherent blocks integrated noncoherently (1 = the whole train,
    the default), NONCOHERENT = n pulses."""
    if integration == "COHERENT":
        if swerling not in (0, 1, 3):
            raise ValueError("COHERENT integration needs a slow-fluctuation case (SW0/SW1/SW3)")
        return shnidman_total_snr_db(DETECT_PD, DETECT_PFA, blocks or 1, swerling)
    return shnidman_total_snr_db(DETECT_PD, DETECT_PFA, n, swerling)


def pol_recombination_loss_db(band: BandConfig, role: str, n_nc: int, swerling: "detection.Fluctuation",
                              pfa: Optional[float] = None) -> float:
    """Polarisation loss of a look (step 1.5; swerling = a case 0..4 or the look's Gamma shape since step 2.4). Dual-pol RX: a detection look recombines the two channels by the GLRT
    on the unknown polarisation of the echo, estimated over the whole dwell (one more unknown complex amplitude):
    SNR_req(n_nc + 1) - SNR_req(n_nc), n_nc = samples integrated noncoherently (the coherent blocks since step 2.2;
    exact laws at the look's Pfa since step 2.3, core/detection.py); a TRACK look knows the polarisation from the
    previous looks: pol_track_est_loss_db. Single-pol RX: pol_loss_single_db."""
    if not band.pol_rx_dual:
        return band.pol_loss_single_db
    if role == "TRACK":
        return band.pol_track_est_loss_db
    return _pol_glrt_loss_db(n_nc, swerling, DETECT_PFA if pfa is None else float(pfa))


@functools.lru_cache(maxsize=1 << 14)   # few keys since the Pfa grid (25 October 2026, A1/A4); exact
def _pol_glrt_loss_db(n_nc: int, swerling: "detection.Fluctuation", pfa: float) -> float:
    return (detection.snr_req_db(n_nc + 1, swerling, pfa, DETECT_PD)
            - detection.snr_req_db(n_nc, swerling, pfa, DETECT_PD))


# -----------------------------------------------------------------------------
# Receive chain: dynamic range (step 1.5)
# -----------------------------------------------------------------------------
ELEMENT_PITCH_M: Dict[str, float] = {"S": S_TRM_PITCH_M, "Ku": KU_TRM_PITCH_M}


def t_sys_with_rise(band: BandConfig, d_t_ant_k: float) -> float:
    """T_sys at the antenna port when the antenna temperature rises by d_t_ant_k over the cold sky (core/sky.py)."""
    return band.t_sys_k + d_t_ant_k * db_to_lin(band.digital_noise_db)


def echo_peak_dbm_per_element(f_ghz: float, theta_deg: float, rcs_m2: float, r_km: float) -> float:
    """Peak echo power at ONE element (antenna port) of a target at r_km, the whole face transmitting one beam at full
    peak power, before any loss (upper bound): P G_tx sigma A_el / ((4 pi)^2 R^4), A_el = pitch^2 cos(theta)."""
    band = band_for_frequency(f_ghz)
    a_el = ELEMENT_PITCH_M[band.name] ** 2 * max(math.cos(math.radians(theta_deg)), 1e-6) * ETA_OHMIC
    p = band.p_pk_total_w * gain_lin(band.eta_illum_tx, f_ghz, theta_deg) * rcs_m2 * a_el \
        / ((4.0 * math.pi) ** 2 * (r_km * 1e3) ** 4)
    return 10.0 * math.log10(p * 1e3)


def min_unsaturated_range_km(f_ghz: float, theta_deg: float, rcs_m2: float) -> float:
    """Shortest range at which the echo of a target stays below the ADC clipping level (echo_peak_dbm_per_element =
    clip_ant_dbm); closer, the per-element ADCs clip before the digital beamforming (fixed gain, step 1.5)."""
    band = band_for_frequency(f_ghz)
    r1 = echo_peak_dbm_per_element(f_ghz, theta_deg, rcs_m2, 1.0)
    return 10.0 ** ((r1 - band.clip_ant_dbm) / 40.0)


ATTENUATOR_MAX_DB: Dict[str, float] = {b: P.get(f"band.{b}.rx.attenuator_max_db") for b in ("S", "Ku")}   # 3.3 E3
ATTENUATOR_STEP_DB: Dict[str, float] = {b: P.get(f"band.{b}.rx.attenuator_step_db") for b in ("S", "Ku")}


def min_unsaturated_range_attenuated_km(f_ghz: float, theta_deg: float, rcs_m2: float) -> float:
    """min_unsaturated_range_km with the switchable post-LNA attenuator at its largest step (step 3.3, E3): the echo
    is lowered by A dB, so the range by 10^(A / 40)."""
    band = band_for_frequency(f_ghz)
    return min_unsaturated_range_km(f_ghz, theta_deg, rcs_m2) / 10.0 ** (ATTENUATOR_MAX_DB[band.name] / 40.0)


def attenuation_needed_db(f_ghz: float, theta_deg: float, rcs_m2: float, r_km: float) -> Optional[float]:
    """Attenuator setting (dB, on its steps) that keeps the echo at r_km below the ADC clipping level; 0 when none is
    needed, None when even the largest step does not suffice (step 3.3, E3)."""
    band = band_for_frequency(f_ghz)
    need = 40.0 * math.log10(max(min_unsaturated_range_km(f_ghz, theta_deg, rcs_m2) / r_km, 1.0))
    step = ATTENUATOR_STEP_DB[band.name]
    a = math.ceil(need / step - 1e-9) * step if need > 0 else 0.0
    return a if a <= ATTENUATOR_MAX_DB[band.name] + 1e-9 else None


def rx_deaf_after_pulse_s(band: BandConfig, tau_s: float) -> float:
    """Time the receiver is deaf from the start of a pulse: the pulse itself plus the limiter recovery (step 2.1)."""
    return tau_s + band.limiter_recovery_s


# -----------------------------------------------------------------------------
# Tracking requirement (Ku TRACK waveforms; formulas from V5.4.0)
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class TrackingPack:
    sigma_theta_deg: float
    sigma_vr_mps: float
    sigma_r_m: Optional[float] = None


def tracking_snr_req(pack: TrackingPack, f_ghz: float, theta_deg: float, t_obs_s: float,
                     sig_bw_mhz: float, k_r: float = RANGE_CRLB_KR, tau_s: float = 0.0) -> Tuple[float, str]:
    """SNR (linear) a track update needs for the pack's accuracies, and the dominant term (ANG / DOP / RNG).
    t_obs_s = coherent observation time of the Doppler measurement (the dwell, (N - 1) PRI + tau). k_r: range
    estimator constant of the look's spectrum (step 2.1: RANGE_CRLB_KR / RMS-bandwidth ratio of the weighting).
    Angle: Cramer-Rao bound of the full digital aperture x ANGLE_CRLB_MARGIN, sigma_u = lambda / (2 pi x_rms
    sqrt(2 SNR)); in the scan plane (worst axis) an angle error is d_u / cos(theta). Range (step 3.3, E2): the residual
    of the range-Doppler coupling c tau sigma_fd / 2B (the mean bias is removed with the measured Doppler) adds in
    quadrature, sigma_fd = K_F / (t_obs sqrt(2 SNR)); tau_s = the pulse (0 = none)."""
    lam = lambda_m(f_ghz)
    sigma_u = math.radians(pack.sigma_theta_deg) * max(math.cos(math.radians(theta_deg)), 1e-6)
    ang = 0.5 * (ANGLE_CRLB_MARGIN * lam / (2.0 * math.pi * _pattern(f_ghz).x_rms_m * sigma_u)) ** 2
    sigma_f = 2.0 * pack.sigma_vr_mps / lam
    dop = 0.5 * (K_F / (t_obs_s * sigma_f)) ** 2
    rng = 0.0
    if pack.sigma_r_m is not None:
        b2 = 2.0 * sig_bw_mhz * 1e6
        rng = 0.5 * (((k_r * C_MPS) / b2) ** 2 + (C_MPS * tau_s * K_F / (b2 * t_obs_s)) ** 2) / pack.sigma_r_m ** 2
    comps = {"ANG": ang, "DOP": dop, "RNG": rng}
    dom = max(comps, key=comps.get)
    return comps[dom], dom


# -----------------------------------------------------------------------------
# Waveforms
# -----------------------------------------------------------------------------
DETECT_ROLES = ("SEARCH", "TWS", "CONFIRM")


@dataclass(frozen=True)
class Waveform:
    name: str
    role: str                 # SEARCH | TWS (reserved, 3.3) | CONFIRM | TRACK
    band: str                 # "S" | "Ku"
    tau_s: float
    n: int                    # pulses per dwell
    pri_s: float
    sig_bw_mhz: float         # signal bandwidth (range resolution / accuracy)
    b_tx: int = 1
    integration: str = "NONCOHERENT"   # "COHERENT" | "NONCOHERENT"
    swerling: Optional[int] = None     # None -> target's case
    tracking_pack: Optional[TrackingPack] = None
    modulation: Optional[str] = None   # "lfm" | "nlfm" | None = chosen per look (modulation_for, step 2.1)
    staggered: bool = False            # range-ambiguous train staggered over STAGGER_N_PRI PRIs (eclipsing loss)
    agility_mhz: float = 0.0           # carrier excursion over a noncoherent train, pulse to pulse (step 3.1, B7)
    tx_mode: Optional[str] = None      # TX multibeam mode (step 3.1, C): single | superposition | partition | tdm;
                                       # None = single if b_tx is 1, else superposition
    tx_cut: Optional[str] = None       # 2-beam partition: 'h' (horizontal cut) or 'v'
    doppler_window_db: Optional[float] = None   # Taylor Doppler weighting of the coherent blocks (step 3.1, P4 Q5), one of
                                                # processing.doppler_window_options_db; None = doppler_window ("none")

    def __post_init__(self) -> None:
        if self.tx_mode is None:
            object.__setattr__(self, "tx_mode", "single" if self.b_tx == 1 else "superposition")
        beams = {"single": (1,), "superposition": tuple(MB_SUP_LOSS_DB), "partition": MB_PART_BEAMS,
                 "tdm": MB_TDM_BEAMS}.get(self.tx_mode)
        if beams is None:
            raise ValueError(f"{self.name}: TX mode '{self.tx_mode}' is not one of {TX_MODES}")
        if self.b_tx not in beams:
            raise ValueError(f"{self.name}: {self.b_tx} beams is not offered in TX mode {self.tx_mode} {beams}")
        if self.tx_mode != "single" and self.band != "S":
            raise ValueError(f"{self.name}: TX multibeam is modelled in S only (Ku multibeam: step 3.2)")
        if (self.tx_cut is not None or (self.tx_mode == "partition" and self.b_tx == 2)) and not (
                self.tx_mode == "partition" and self.b_tx == 2 and self.tx_cut in ("h", "v")):
            raise ValueError(f"{self.name}: tx_cut 'h' or 'v' is for (and needed by) the 2-beam partition only")
        if self.modulation is not None and self.modulation not in MODULATIONS:
            raise ValueError(f"{self.name}: modulation '{self.modulation}' is not one of {MODULATIONS}")
        if self.band not in BANDS:
            raise ValueError(f"{self.name}: unknown band {self.band}")
        if self.role not in BANDS[self.band].allowed_roles:
            raise ValueError(f"{self.name}: role {self.role} is not allowed in {self.band} band "
                             f"(allowed: {BANDS[self.band].allowed_roles})")
        dmax = BANDS[self.band].duty_max
        if dmax > 0 and self.n > 1 and self.trm_duty > dmax * (1.0 + 1e-12):
            raise ValueError(f"{self.name}: duty {self.trm_duty:.3f} exceeds the {self.band} TRM ceiling {dmax:g}")
        bi = BANDS[self.band].bi_max_mhz
        if bi > 0 and self.sig_bw_mhz > bi:
            raise ValueError(f"{self.name}: signal bandwidth {self.sig_bw_mhz} MHz exceeds the {self.band} "
                             f"instantaneous bandwidth {bi} MHz")
        if self.role == "TRACK" and self.tracking_pack is None:
            raise ValueError(f"{self.name}: TRACK waveform needs a tracking_pack")
        if self.integration not in ("COHERENT", "NONCOHERENT"):
            raise ValueError(f"{self.name}: bad integration mode")
        if not (0.0 < self.tx_block_s <= self.pri_s) or self.n < 1 or self.b_tx < 1:
            raise ValueError(f"{self.name}: inconsistent timing")
        if self.doppler_window_db is not None and (self.doppler_window_db not in DOPPLER_WINDOW_OPTIONS_DB
                                                   or self.integration != "COHERENT" or self.n < 2):
            raise ValueError(f"{self.name}: a Doppler weighting is one of {DOPPLER_WINDOW_OPTIONS_DB} dB, on a coherent "
                             f"train of 2 pulses or more")
        if self.agility_mhz < 0 or (self.agility_mhz > 0 and (self.integration != "NONCOHERENT" or self.n < 2)):
            raise ValueError(f"{self.name}: frequency agility needs a noncoherent train of 2 pulses or more")

    @property
    def occupied_mhz(self) -> float:
        """Spectrum the train occupies: the signal bandwidth plus the agility excursion (step 3.1, B8)."""
        return self.sig_bw_mhz + self.agility_mhz

    @property
    def duty(self) -> float:
        return self.tau_s / self.pri_s

    @property
    def trains_per_dwell(self) -> int:
        """Pulse trains a dwell sends at the face peak power: B for the time multiplex, 1 otherwise (step 3.1, C)."""
        return self.b_tx if self.tx_mode == "tdm" else 1

    @property
    def tx_block_s(self) -> float:
        """TRM pulse: tau, or the B pulses sent back to back by the time multiplex (blind range, buffer, PAE)."""
        return self.tau_s * self.trains_per_dwell

    @property
    def trm_duty(self) -> float:
        """TRM duty of the train: tx_block / PRI."""
        return self.tx_block_s / self.pri_s

    @property
    def dwell_s(self) -> float:
        """Time on target: first pulse start to last pulse end."""
        return (self.n - 1) * self.pri_s + self.tau_s

    @property
    def coherent_time_s(self) -> float:
        """Span over which phase must stay coherent: the pulse, or the whole train when integrated coherently."""
        return self.dwell_s if (self.integration == "COHERENT" or self.role == "TRACK") else self.tau_s


# --- Catalogue: WF-LRS (S) only, kept as code benchmark. Timing values come from the V5.4.0 Ku-era
# --- definition (S-band waveform design is abandoned/to be redefined [V]); integration set to
# --- NONCOHERENT because a 648 ms dwell cannot be integrated coherently on a moving target [D].
WF_LRS = Waveform(
    name="WF-LRS", role="SEARCH", band="S",
    tau_s=200e-6, n=24, pri_s=27e-3,          # PRI staggered 20/27/34 ms nominal in V5.4.0
    sig_bw_mhz=75.0, b_tx=4,
    integration="NONCOHERENT", swerling=None,
)
CATALOGUE: List[Waveform] = [WF_LRS]


def playable(f_ghz: float, theta_deg: float, catalogue: Optional[List[Waveform]] = None) -> List[Waveform]:
    band = band_for_frequency(f_ghz)
    if band is None or abs(theta_deg) > band.scan_limit_deg(f_ghz):
        return []
    return [w for w in (catalogue if catalogue is not None else CATALOGUE)
            if w.band == band.name and w.role in band.allowed_roles and band.fits(f_ghz, w.sig_bw_mhz)]


# -----------------------------------------------------------------------------
# Solver
# -----------------------------------------------------------------------------
@dataclass
class Result:
    wf: Waveform
    target: Target
    snr_req_db: float
    driver: str
    range_km: float
    losses: Dict[str, float]
    l_tot_db: float
    p_avg_used_w: float          # electrical average power of the dwell, whole face (step 1.3)
    p_avg_max_w: float           # electrical power available for transmission (face_elec_avail_w)
    r_unamb_km: float
    n_cells: float
    frame: Optional[Dict[str, float]]
    pae: float = 0.0             # PAE of the waveform
    p_diss_trm_w: float = 0.0    # heat per TRM during the dwell
    t_sys_k: float = 0.0         # system noise temperature used
    pulse: Optional[Dict[str, float]] = None   # step 2.1: modulation and intra-pulse / timeline outputs
    range_noise_km: float = 0.0  # range against the noise alone (= range_km without a self-protection jammer)
    burn_through_km: float = math.inf   # self-protection jammer (step 3.1, P4 Q11): S / J0 = SNR_req at this range
    look_law: Optional[tuple] = None    # (coherent blocks, fluctuation, Pfa, position losses, weights): Pd of one look


class SolveBase:
    """The part of solve() common to every Pfa, detection criterion, self-protection jammer, search lattice and system
    temperature (step 3.2, Q6 E1): computed once per (waveform, frequency, scan, target, RX option, velocity span) and
    shared by the optimiser's lattice spacings and requirement classes. The losses that depend on the look (polarisation,
    positions, CFAR) hold their place in the dict so that the total is summed in the same order as before (bit-identical)."""
    __slots__ = ("wf", "f_ghz", "theta_deg", "target", "rx_sll_db", "band", "sw", "psl", "mod", "wgt", "n_blocks",
                 "t_block", "ppb", "t_span", "fl", "span", "n_hyp", "bank_half", "a_max", "n_acc", "search", "sll_key",
                 "track_snr", "losses", "x_buf", "r_listen_km", "num", "p_g_tx_w", "sigma_energy")


def solve_base(wf: Waveform, f_ghz: float, theta_deg: float, target: Target, rx_sll_db: Optional[float] = None,
               v_span_mps: Optional[float] = None) -> SolveBase:
    """First part of solve() (same arguments); raises ValueError when the waveform cannot play there."""
    band = band_for_frequency(f_ghz)
    if band is None or band.name != wf.band:
        raise ValueError(f"{wf.name}: {f_ghz} GHz is not in its {wf.band} band")
    if abs(theta_deg) > band.scan_limit_deg(f_ghz):
        raise ValueError(f"{wf.name}: scan {theta_deg} deg exceeds the {band.scan_limit_deg(f_ghz):.0f} deg "
                         f"limit at {f_ghz} GHz")
    if not band.fits(f_ghz, wf.occupied_mhz):
        raise ValueError(f"{wf.name}: {wf.occupied_mhz:g} MHz at {f_ghz} GHz leaves the usable {band.name} band "
                         f"{band.usable_lo_ghz}-{band.usable_hi_ghz} GHz")
    if band.true_time_delay == "none" and abs(theta_deg) > 0:
        b_fill = C_MPS / (ARRAY_ENVELOPE_M * math.sin(math.radians(abs(theta_deg)))) / 1e6
        if wf.sig_bw_mhz > b_fill:
            raise ValueError(f"{wf.name}: {wf.sig_bw_mhz:g} MHz exceeds the aperture-fill bandwidth {b_fill:.0f} MHz "
                             f"at {theta_deg} deg (no true-time delay)")
    b = SolveBase()
    b.wf, b.f_ghz, b.theta_deg, b.target, b.rx_sll_db, b.band = wf, f_ghz, theta_deg, target, rx_sll_db, band
    sw = b.sw = wf.swerling if wf.swerling is not None else target.swerling     # None = derived from the class (2.4)
    psl = b.psl = range_psl_db(rx_sll_db)
    mod = b.mod = wf.modulation or modulation_for(f_ghz, wf.sig_bw_mhz, target.v_close_mps)
    wgt = b.wgt = range_weighting(psl)

    n_blocks, t_block = coherent_blocks(wf.n, wf.pri_s, wf.tau_s, wf.integration, sw, target, f_ghz)
    ppb = wf.n // n_blocks                                     # pulses per coherent block (Doppler cells)
    t_span = wf.dwell_s
    fl = look_fluctuation(target, f_ghz, sw, n_blocks, t_span, wf.agility_mhz)   # Gamma shape (2.4 B3 + E2, 3.1 B7)
    b.n_blocks, b.t_block, b.ppb, b.t_span, b.fl = n_blocks, t_block, ppb, t_span, fl
    # velocity bank (COMPENSATED migration): over the span, one hypothesis on a TRACK look (step 2.2)
    # a TRACK look covers the span its caller gives (step 3.3, B3: +/- k sigma of the predicted radial velocity), else one
    span = ((target.v_close_mps if v_span_mps is None else float(v_span_mps)) if wf.role != "TRACK"
            else (0.0 if v_span_mps is None else float(v_span_mps)))
    n_hyp, bank_half = 1, 0.0
    if MIGRATION_MODE.upper() == "COMPENSATED":
        n_hyp, _, bank_half = velocity_bank(round(span, 6), wf.tau_s, wf.sig_bw_mhz, t_span, wf.n, n_blocks == 1)
    b.span, b.n_hyp, b.bank_half = span, n_hyp, bank_half
    # acceleration (step 2.4, C1-C2): bank over +/- a_max; a TRACK look knows it from the track (optimistic until 3.3)
    # step 3.3 (E1): a TRACK look banks over the target's accel_max_mps2 too; the track model passes +/- k sigma of the
    # predicted acceleration as that value (was 0: acceleration assumed known)
    a_max = b.a_max = target.accel_max_mps2
    b.n_acc, acc_loss = acceleration_bank(a_max, f_ghz, t_block if ppb > 1 else 0.0, t_span, wf.sig_bw_mhz,
                                          n_blocks == 1)
    b.search = wf.role in ("SEARCH", "TWS")
    b.sll_key = RX_SLL_DEFAULT_DB if rx_sll_db is None else float(rx_sll_db)
    b.track_snr = None
    if wf.role not in DETECT_ROLES:
        b.track_snr = tracking_snr_req(wf.tracking_pack, f_ghz, theta_deg, wf.dwell_s, wf.sig_bw_mhz,
                                       RANGE_CRLB_KR / wgt["rms_ratio"], wf.tau_s)
    losses: Dict[str, float] = dict(band.losses_db)
    losses["edge"] = band.edge_loss_db(f_ghz, wf.occupied_mhz)       # averaged over the hopped spectrum (3.1, B8)
    losses["compression"] = compression_loss_db(mod, f_ghz, wf.sig_bw_mhz, target.v_close_mps, psl)
    if wf.staggered and wf.n > 1:
        losses["eclipsing"] = eclipsing_loss_db(band, wf.tau_s, wf.pri_s)
    losses["polarisation"] = 0.0                                     # per Pfa: solve_look
    # the dual-pol receiver also collects the cross-polar return of the target: x (1 + rho) (step 2.4, E1)
    rho = target.depol_lin(f_ghz) if band.pol_rx_dual else 0.0
    losses["target_depolarisation"] = -10.0 * math.log10(1.0 + rho)
    x_buf = 0.0
    if band.e_buf_trm_j > 0:
        x_buf = pulse_elec_energy_trm_j(wf) / band.e_buf_trm_j
        if x_buf > band.buffer_max_fraction * (1.0 + 1e-12):     # tolerance: tau_max_s() sits exactly on the limit
            raise ValueError(f"{wf.name}: pulse needs {x_buf:.2f} of the TRM buffer (max {band.buffer_max_fraction:g}): "
                             f"droop too large")
        losses["buffer_droop"] = -10.0 * math.log10(1.0 - x_buf / 2.0)
        trm_supply_check(wf, band)
    if JUNCTION_CHECK:
        junction_check(wf, band)
    b.x_buf = x_buf
    losses["multibeam_tx"] = multibeam_tx_loss_db(wf)
    losses["scan_mismatch"] = 2.0 * active_mismatch(f_ghz, theta_deg)["loss_db_per_pass"]   # TX and RX
    # random position of the target (step 2.3): search grid, range / Doppler straddle, velocity-filter offset
    losses["positions"] = 0.0                                        # per lattice, Pfa and criterion: solve_look
    losses["cfar"] = 0.0                                             # per Pfa: solve_look
    r_listen_km = b.r_listen_km = C_MPS * wf.pri_s / 2.0 / 1000.0    # oscillator round trip at the listening range
    if ppb > 1:
        losses["coherence"] = coherence_loss_db(f_ghz, t_block, r_listen_km, wf.sig_bw_mhz, target)
        if wf.doppler_window_db is not None:
            losses["doppler_weighting"] = range_weighting(wf.doppler_window_db)["loss_db"]   # step 3.1, P4 Q5
    losses["acceleration"] = acc_loss
    # migration: compensation residual (the velocity-filter offset is in 'positions'), or the uncompensated law
    dr = C_MPS / (2.0 * wf.sig_bw_mhz * 1e6)
    n_cells = 1.0 + target.v_close_mps * t_span / dr
    if MIGRATION_MODE.upper() == "COMPENSATED":
        losses["migration"] = MIGRATION_RESIDUAL_DB if n_cells > 1.5 else 0.0
    else:
        losses["migration"], _ = migration_loss_db(target.v_close_mps, t_span, wf.sig_bw_mhz, wf.n)
    b.losses = losses

    g_tx = gain_lin(band.eta_illum_tx, f_ghz, theta_deg) * tx_gain_ratio(wf)
    g_rx = gain_lin(rx_beam(band.name, rx_sll_db).eta, f_ghz, theta_deg)
    p_beam = tx_beam_power_w(wf)
    b.num = p_beam * g_tx * g_rx * lambda_m(f_ghz) ** 2 * target.rcs_at(f_ghz) * wf.tau_s * wf.n
    b.p_g_tx_w, b.sigma_energy = p_beam * g_tx, target.rcs_at(f_ghz) * wf.tau_s * wf.n
    return b


class SolveLook:
    """The look-dependent part of solve() on top of a SolveBase (step 3.2, Q6 E1)."""
    __slots__ = ("pfa", "q", "pos_l", "pos_w", "pos_parts", "snr_db", "positions_db", "driver", "losses", "l_tot",
                 "t_sys", "r_noise_km", "r_bt_km", "r_km")


def solve_look(b: SolveBase, t_sys_k: Optional[float] = None, pfa: Optional[float] = None,
               cum: Optional[Tuple[float, float, float]] = None, spj_density_w_hz: Optional[float] = None,
               grid_spacing_hpbw: Optional[float] = None, pointing: Optional[Tuple[str, float]] = None,
               pd: Optional[float] = None) -> SolveLook:
    """Second part of solve() (same arguments): required SNR, position, polarisation and CFAR losses, ranges."""
    wf, band = b.wf, b.band
    k = SolveLook()
    pfa_l = k.pfa = DETECT_PFA if pfa is None else float(pfa)
    pos_l, pos_w, pos_parts = position_losses(band.name, b.sll_key, b.psl, b.search, b.ppb > 1,
                                              (round(b.bank_half, 9), wf.tau_s, wf.sig_bw_mhz, b.t_span, wf.n,
                                               b.n_blocks == 1),
                                              tx_subaperture(wf), wf.doppler_window_db if b.ppb > 1 else None,
                                              grid_spacing_hpbw, pointing)
    k.pos_l, k.pos_w, k.pos_parts = pos_l, pos_w, pos_parts
    if pd is None and cum is not None and len(cum) > 3:
        pd = cum[3]                                 # step 5.1: the sector's own cumulative Pd (search_criterion)
    pd_l = DETECT_PD if pd is None else float(pd)
    q = k.q = cum[0] * cum[1] / (cum[2] * 1e3) if (cum is not None and b.search) else 0.0
    if b.track_snr is None:
        # exact law (step 2.3), n = coherent blocks integrated noncoherently, Pd averaged over the position
        snr_db = detection.required_snr_db(b.n_blocks, b.fl, pfa_l, pd_l, q=q, floor=CUM_PD_FLOOR)
        positions_db = detection.required_snr_db(b.n_blocks, b.fl, pfa_l, pd_l, pos_l, pos_w, q,
                                                 CUM_PD_FLOOR) - snr_db
        driver = ("Detect-CUM-" if q > 0 else "Detect-") + wf.integration[:4]
    else:
        snr_lin, dom = b.track_snr
        snr_db, driver = lin_to_db(snr_lin), "Track-" + dom
        positions_db = sum(pos_parts.values())               # accuracy-driven: mean position losses
    k.snr_db, k.positions_db, k.driver = snr_db, positions_db, driver
    losses = dict(b.losses)
    losses["polarisation"] = pol_recombination_loss_db(band, wf.role, b.n_blocks, b.fl, pfa_l)
    losses["positions"] = positions_db
    losses["cfar"] = cfar_loss_db(CFAR_REF_CELLS, pfa_l)
    k.losses = losses
    l_tot = k.l_tot = sum(losses.values())
    t_sys = k.t_sys = band.t_sys_k if t_sys_k is None else float(t_sys_k)
    den = (4.0 * math.pi) ** 3 * K_BOLTZ * t_sys * db_to_lin(l_tot) * db_to_lin(snr_db)
    k.r_noise_km = (b.num / den) ** 0.25 / 1000.0
    k.r_bt_km = math.inf
    if spj_density_w_hz:
        k.r_bt_km = burn_through_range_km(b.p_g_tx_w, b.sigma_energy, snr_db, l_tot, spj_density_w_hz)
    k.r_km = jammed_range_km(k.r_noise_km, k.r_bt_km)
    return k


def solve(wf: Waveform, f_ghz: float, theta_deg: float, target: Target,
          elec_mw: float = FACE_ELEC_POWER_MW, rx_sll_db: Optional[float] = None,
          t_sys_k: Optional[float] = None, v_span_mps: Optional[float] = None, pfa: Optional[float] = None,
          cum: Optional[Tuple[float, float, float]] = None, spj_density_w_hz: Optional[float] = None,
          grid_spacing_hpbw: Optional[float] = None, pointing: Optional[Tuple[str, float]] = None,
          pd: Optional[float] = None) -> Result:
    """rx_sll_db: RX peak sidelobe level of the watch volume (an option of RX_SLL_OPTIONS_DB, None = default).
    spj_density_w_hz: EIRP density of a self-protection jammer carried by the target (step 3.1, P4 Q11-Q13: barrage
    noise in the main lobe, not nulled): S = SNR_req (N + J), J ~ 1 / R^2, solved exactly (jammed_range_km).
    grid_spacing_hpbw: spacing of the search TX lattice of the volume (step 3.1, P5 E7; None = BEAM_SPACING_HPBW).
    t_sys_k: system noise temperature of the look direction (sky noise, core/sky.py); None = the band's cold sky.
    v_span_mps: velocity span the bank covers (sector v_max - v_min); None = 0 to the target's speed. TRACK looks:
    the span given (step 3.3, B3), one hypothesis when None. pointing: pointing-error law of a track look (step 3.3,
    position_losses); pd: Pd of the look (None = DETECT_PD; track updates: tracking.pd_update).
    Detection criterion (step 2.3): pfa = false-alarm probability per cell (None = DETECT_PFA); cum = (v_close m/s,
    revisit s, requirement range km) for a SEARCH look = DETECT_PD reached CUMULATIVELY over the looks spaced v x RR
    (q = v RR / R), None = one look. The Pd of a look is averaged over the random position of the target (loss
    'positions' = the SNR this average costs); the returned range is where the criterion is met (exact at R).
    Computed as solve_base + solve_look (the optimiser's path, without the diagnostics) + the diagnostics."""
    b = solve_base(wf, f_ghz, theta_deg, target, rx_sll_db, v_span_mps)
    k = solve_look(b, t_sys_k, pfa, cum, spj_density_w_hz, grid_spacing_hpbw, pointing, pd)
    band, sw, psl, wgt, n_blocks, ppb, fl = b.band, b.sw, b.psl, b.wgt, b.n_blocks, b.ppb, b.fl
    snr_db, positions_db, pfa_l, q, r_km = k.snr_db, k.positions_db, k.pfa, k.q, k.r_km

    eta = pae_wf(wf)
    p_avg_used = band.p_pk_total_w * wf.trm_duty / eta
    p_avg_max = face_elec_avail_w(elec_mw)
    dr = C_MPS / (2.0 * wf.sig_bw_mhz * 1e6)
    n_cells = 1.0 + target.v_close_mps * b.t_span / dr

    frame = None
    if wf.role == "SEARCH":
        pos = n_search_positions(f_ghz, spacing_hpbw=grid_spacing_hpbw)
        t_pos = wf.dwell_s + (0.0 if FRAME_INTERLEAVED else 2.0 * r_km * 1000.0 / C_MPS)
        cpd = cells_per_dwell(wf)
        t_frame = pos * t_pos / cpd
        sub = tx_subaperture(wf)
        frame = {"positions": pos, "dwell_s": wf.dwell_s, "t_pos_s": t_pos, "t_frame_s": t_frame,
                 "ok": 1.0 if t_frame <= REFRESH_S else 0.0, "positions_per_dwell": cpd,
                 "rx_beams_per_position": rx_beams_per_position(f_ghz, rx_sll_db, grid_spacing_hpbw)
                 * (1.0 if sub is None else subaperture_cell_ratio(band.name, sub))}

    f_d = doppler_max_hz(f_ghz, target.v_close_mps)
    n_c = cells_in_pulse(target.v_close_mps, wf.tau_s, wf.sig_bw_mhz)
    pulse = {"modulation": b.mod, "range_psl_db": psl, "range_resolution_m": range_resolution_m(wf.sig_bw_mhz, psl),
             "k_r": RANGE_CRLB_KR / wgt["rms_ratio"], "doppler_hz": f_d,
             "coupling_bias_m": C_MPS * f_d * wf.tau_s / (2.0 * wf.sig_bw_mhz * 1e6),
             "cells_in_pulse": n_c, "zero_velocity_filter_loss_db": velocity_mismatch_loss_db(n_c),
             "velocity_hypotheses_1db": velocity_hypotheses(target.v_close_mps, wf.tau_s, wf.sig_bw_mhz),
             "am_pm_drift_deg": am_pm_drift_deg(b.x_buf), "blind_range_km": blind_range_km(band, wf.tx_block_s),
             "doppler_ambiguity": f_d * wf.pri_s if wf.n > 1 else 0.0,
             # processing (step 2.2)
             "coherent_blocks": n_blocks, "pulses_per_block": ppb, "block_s": b.t_block,
             "velocity_hypotheses": b.n_hyp, "velocity_span_mps": b.span,
             "doppler_floor_dbc": doppler_floor_dbc(f_ghz, b.t_block, b.r_listen_km) if ppb > 1 else {},
             # detection criterion (step 2.3)
             "pfa": pfa_l, "criterion": "cumulative" if q > 0 else "single look", "look_spacing_q": q,
             "position_losses_mean_db": k.pos_parts,
             "pd_per_look_at_range": float(np.interp(snr_db + positions_db, detection.SNR_GRID_DB,
                                                     detection.mean_curve(n_blocks, fl, pfa_l, k.pos_l, k.pos_w)))
             if wf.role in DETECT_ROLES else None,
             # target model (step 2.4)
             "target": {"type": target.target_type, "rcs_m2": target.rcs_at(f_ghz),
                        "optics_rcs_m2": target.optics_rcs_m2(f_ghz), "t0_s": target.t0_s(f_ghz),
                        "swerling": sw, "fluctuation": target.fluctuation(f_ghz) if sw is None else f"SW{sw}",
                        "gamma_shape": fl.k, "depol_db": lin_to_db(target.depol_lin(f_ghz)),
                        "diversity": diversity_factor(target, f_ghz)},
             "acceleration_max_mps2": b.a_max, "acceleration_hypotheses": b.n_acc,
             "masking_db": masking_levels_db(psl, ppb > 1)}
    return Result(wf, target, snr_db, k.driver, r_km, k.losses, k.l_tot, p_avg_used, p_avg_max,
                  C_MPS * wf.pri_s / 2.0 / 1000.0, n_cells, frame, eta, trm_dissipation_w(wf), k.t_sys, pulse,
                  k.r_noise_km, k.r_bt_km, (n_blocks, fl, pfa_l, tuple(k.pos_l), tuple(k.pos_w)))


def burn_through_range_km(p_g_tx_w: float, sigma_energy: float, snr_req_db: float, l_tot_db: float,
                          density_w_hz: float) -> float:
    """Range where the echo over the self-protection jammer noise reaches the required SNR (energy form, J0 =
    EIRP density x G_rx lambda^2 / ((4 pi)^2 R^2), losses on the signal as in the radar equation):
    R_bt = sqrt(P G_t sigma tau N / (4 pi J0 SNR_req L)); sigma_energy = sigma x tau N."""
    return math.sqrt(p_g_tx_w * sigma_energy / (4.0 * math.pi * density_w_hz * db_to_lin(snr_req_db)
                                                * db_to_lin(l_tot_db))) / 1e3


def jammed_range_km(r_noise_km, r_bt_km):
    """Range with noise and a self-protection jammer (step 3.1, P4 Q11): a / R^4 = SNR (N0 + b / R^2) is quadratic in
    u = 1 / R^2: u = (1 / R_bt^2 + sqrt(1 / R_bt^4 + 4 / R_n^4)) / 2 (R_n: noise alone, R_bt: jammer alone). Arrays
    accepted; R_bt = inf gives R_n."""
    rn, rb = np.asarray(r_noise_km, dtype=float), np.asarray(r_bt_km, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        ib = np.where(np.isinf(rb), 0.0, 1.0 / rb ** 2)
        u = 0.5 * (ib + np.sqrt(ib * ib + 4.0 / rn ** 4))
        out = np.where(np.isinf(rb), rn, 1.0 / np.sqrt(u))
    return float(out) if out.ndim == 0 else out


# -----------------------------------------------------------------------------
# Output
# -----------------------------------------------------------------------------
def report(f_ghz: float, theta_deg: float, target_name: str, elec_mw: float) -> None:
    band = band_for_frequency(f_ghz)
    tgt = TARGETS[target_name]
    print("=" * 100)
    print(f" RADAR PERFORMANCE MODEL V{MODEL_VERSION}")
    print("=" * 100)
    print(f"Frequency {f_ghz} GHz | scan {theta_deg} deg | target {tgt.name} | "
          f"Pd {DETECT_PD}, Pfa {DETECT_PFA:g} | face electrical power {elec_mw} MW")
    if band is not None:
        print("Target classes (class means; frontal copolar RCS, fluctuation, T0, depolarisation, LOS acceleration): "
              + " | ".join(f"{n} {t.rcs_at(f_ghz):.4g} m2 {t.fluctuation(f_ghz)} T0 {t.t0_s(f_ghz):.2f} s "
                           f"{lin_to_db(t.depol_lin(f_ghz)):g} dB {t.accel_max_mps2:g} m/s2" for n, t in TARGETS.items()))
    if band is None:
        print(f"-> {f_ghz} GHz is outside every band (S {_S_LO}-{_S_HI}, Ku {_KU_LO}-{_KU_HI} GHz). Nothing playable.")
        return
    lim = band.scan_limit_deg(f_ghz)
    print(f"Aperture: {N_TRSA} TRSA ({KU_TRM_PER_TRSA_SIDE}x{KU_TRM_PER_TRSA_SIDE} Ku, {S_TRM_PER_TRSA_SIDE}x"
          f"{S_TRM_PER_TRSA_SIDE} S TRM) | emissive {PHYS_AREA_M2:.4f} m2, total {TOTAL_AREA_M2:.4f} m2, fill "
          f"{100 * FILL_FACTOR:.2f} % | envelope {ARRAY_ENVELOPE_M:.3f} m | pitch S {1e3 * S_TRM_PITCH_M:.1f} mm, "
          f"Ku {1e3 * KU_TRM_PITCH_M:.1f} mm")
    print(f"TRM: {TRM_TECHNOLOGY} | PAE sat {band.pae_sat:g} at duty {PAE_DUTY_REF:g} | buffer {band.e_buf_trm_j:g} J "
          f"(max {band.buffer_max_fraction:g} per pulse -> tau max {1e3 * tau_max_s(band):.2f} ms) | duty max {band.duty_max:g}")
    print(f"Band {band.name}: {band.n_trm} TRM x {band.p_pk_trm_w:.0f} W = {band.p_pk_total_w/1e6:.2f} MW peak/face | "
          f"scan limit {lim:.0f} deg | T_sys {band.t_sys_k:.1f} K (cold sky) | fixed losses {band.l_fixed_db:.2f} dB")
    print(f"RX chain: ADC NF {band.adc_nf_db:.1f} dB at {band.adc_fs_hz / 1e6:.0f} Msps | noise "
          f"{band.noise_ant_dbm + band.gain_to_adc_db - P.get(f'band.{band.name}.rx.adc_fullscale_dbm'):.1f} dBFS | "
          f"clipping {band.clip_ant_dbm:.1f} dBm at the element, {band.clip_headroom_db:.1f} dB above the noise | LNA P1dB "
          f"{band.lna_p1db_ant_dbm:.1f} dBm | limiter recovery {band.limiter_recovery_s * 1e9:.0f} ns | unsaturated beyond "
          + ", ".join(f"{n} {min_unsaturated_range_km(f_ghz, 0.0, t.rcs_at(f_ghz)):.1f} km" for n, t in TARGETS.items()))
    rx = rx_beam(band.name)
    print(f"Gain TX {lin_to_db(gain_lin(band.eta_illum_tx, f_ghz, theta_deg)):.2f} dBi | "
          f"RX {lin_to_db(gain_lin(rx.eta, f_ghz, theta_deg)):.2f} dBi (sidelobes {RX_SLL_DEFAULT_DB:g} dB, eta "
          f"{rx.eta:.3f}) | HPBW TX {hpbw_deg(f_ghz, theta_deg):.2f} / RX {hpbw_deg(f_ghz, theta_deg, 'rx'):.2f} deg | "
          f"electrical available for TX {face_elec_avail_w(elec_mw)/1e6:.3f} MW")
    if abs(theta_deg) <= lim:
        am = active_mismatch(f_ghz, theta_deg)
        pp = pol_purity(theta_deg, 45.0)
        print(f"Active mismatch at {theta_deg:g} deg: |Gamma| {am['gamma']:.3f} (WAIM {am['gamma_waim']:.3f}, grating "
              f"lobe {am['gamma_gl']:.3f}, margin {am['margin_g']:+.4f} in u) -> {am['loss_db_per_pass']:.2f} dB per pass"
              f" | TX pol {TX_POLARISATION}, cross-pol discrimination at 45 deg azimuth {pp['xpd_db']:.1f} dB")
        print(f"Worst visible sidelobe at {theta_deg:g} deg: TX {sidelobe_visible_db(f_ghz, theta_deg, 'tx'):.1f} dB, RX "
              f"{sidelobe_visible_db(f_ghz, theta_deg):.1f} dB (option {RX_SLL_DEFAULT_DB:g} dB held up to "
              f"{rx_sll_held_up_to_deg(f_ghz, RX_SLL_DEFAULT_DB, lim):g} deg)")
    try:
        from . import isolation
    except ImportError:            # run as a script: import the package (core.isolation uses relative imports)
        import os
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from core import isolation
    print("Isolation (leak to one RX element; margins LNA / ADC / desensitisation, dB): " + " | ".join(
        f"{p['tx']}->{p['rx']} {p['relation']} {p['verdict']}" for p in isolation.matrix()))
    for rule in isolation.scheduler_rules():
        print(f"  scheduler rule (4.3): {rule}")
    if abs(theta_deg) > lim:
        print(f"-> scan angle {theta_deg} deg exceeds the {lim:.0f} deg limit at {f_ghz} GHz. Nothing playable.")
        return
    wfs = playable(f_ghz, theta_deg)
    if not wfs:
        print(f"-> No waveform in the catalogue is playable in {band.name} band at {f_ghz} GHz "
              f"(roles allowed: {', '.join(band.allowed_roles)}; the signal f +/- B/2 must fit in "
              f"{band.usable_lo_ghz}-{band.usable_hi_ghz} GHz).")
        return

    print("-" * 100)
    for w in wfs:
        r = solve(w, f_ghz, theta_deg, tgt, elec_mw)
        tg = r.pulse["target"]
        print(f"{w.name} [{w.role}, {w.band}, {w.integration}, {tg['fluctuation']} -> Gamma shape {tg['gamma_shape']:.3g}] "
              f"tau {w.tau_s*1e6:.0f} us, N {w.n}, PRI {w.pri_s*1e3:.1f} ms, B {w.sig_bw_mhz:.0f} MHz, "
              f"TX {w.tx_mode} x{w.b_tx}")
        print(f"  Required SNR (total energy): {r.snr_req_db:.2f} dB   driver: {r.driver}")
        print(f"  Range on {tgt.name}: {r.range_km:,.0f} km")
        ls = ", ".join(f"{k} {v:.2f}" for k, v in r.losses.items() if v)
        print(f"  Losses (dB): {ls} | total {r.l_tot_db:.2f}")
        print(f"  Migration: {r.n_cells:.0f} range cells crossed over the dwell ({MIGRATION_MODE.lower()})")
        pu = r.pulse
        print(f"  Pulse: {pu['modulation'].upper()}, range sidelobes {pu['range_psl_db']:g} dB, resolution "
              f"{pu['range_resolution_m']:.2f} m, k_r {pu['k_r']:.3f} | blind range {pu['blind_range_km']:.1f} km | "
              f"{pu['cells_in_pulse']:.1f} cells crossed in the pulse -> {pu['velocity_hypotheses_1db']} velocity filters "
              f"for 1 dB | range-Doppler coupling {pu['coupling_bias_m']:.0f} m (corrected) | AM/PM drift "
              f"{pu['am_pm_drift_deg']:.2f} deg")
        print(f"  Processing: {pu['coherent_blocks']} coherent block(s) of {pu['pulses_per_block']} pulse(s) "
              f"({pu['block_s'] * 1e3:.1f} ms) | velocity bank {pu['velocity_hypotheses']} hypotheses over "
              f"{pu['velocity_span_mps']:.0f} m/s | Doppler floor "
              + (", ".join(f"{k} {v:.0f} dBc" for k, v in pu['doppler_floor_dbc'].items()) or "-"))
        print(f"  Target: T0 {tg['t0_s']:.2f} s | diversity x{tg['diversity']:.3f} | acceleration {pu['acceleration_max_mps2']:g} "
              f"m/s2 -> {pu['acceleration_hypotheses']} hypotheses | masking by a strong neighbour (dB under its peak): "
              + " ; ".join(f"{dim} " + ", ".join(f"{k} {v:.1f}" for k, v in lv.items())
                           for dim, lv in pu["masking_db"].items()))
        print(f"  Electrical power: {r.p_avg_used_w/1e3:.1f} kW used of {r.p_avg_max_w/1e3:.0f} kW available "
              f"({100*r.p_avg_used_w/r.p_avg_max_w:.1f} %) | duty {100*w.duty:.2f} % | PAE {r.pae:.3f} | "
              f"TRM heat {r.p_diss_trm_w:.1f} W during the dwell")
        flag = "  <-- RANGE AMBIGUOUS (needs staggered PRI / ambiguity resolution)" if r.range_km > r.r_unamb_km else ""
        print(f"  Unambiguous range (nominal PRI): {r.r_unamb_km:,.0f} km{flag}")
        if r.frame:
            fr = r.frame
            ok = "OK" if fr["ok"] else "NOT FEASIBLE"
            print(f"  Search frame: {fr['positions']:.0f} positions, dwell {fr['dwell_s']*1e3:.0f} ms, "
                  f"frame time {fr['t_frame_s']:.0f} s vs refresh {REFRESH_S:.0f} s -> {ok}")
        print("  Ranges by target: " + " | ".join(
            f"{n} {solve(w, f_ghz, theta_deg, t, elec_mw).range_km:,.0f} km" for n, t in TARGETS.items()))
    print("=" * 100)


# -----------------------------------------------------------------------------
# Self-tests (regression / benchmark guards)
# -----------------------------------------------------------------------------
def selftest() -> int:
    fails = 0

    def check(label: str, cond: bool, detail: str = "") -> None:
        nonlocal fails
        print(f"[{'PASS' if cond else 'FAIL'}] {label} {detail}")
        fails += 0 if cond else 1

    pd, pfa = DETECT_PD, 1e-6   # reference values of the formula checks below (published at Pfa 1e-6)
    alb1 = albersheim_total_snr_db(pd, pfa, 1)
    check("Albersheim steady N=1 = 13.11 dB", abs(alb1 - 13.11) < 0.05, f"({alb1:.2f})")
    sh0 = shnidman_total_snr_db(pd, pfa, 1, 0)
    check("Shnidman SW0 N=1 within 0.3 dB of Albersheim", abs(sh0 - alb1) < 0.3, f"({sh0:.2f})")
    exact_sw1 = 10 * math.log10(math.log(pfa) / math.log(pd) - 1.0)
    sh1 = shnidman_total_snr_db(pd, pfa, 1, 1)
    check("Shnidman SW1 N=1 within 0.5 dB of exact 21.14 dB", abs(sh1 - exact_sw1) < 0.5, f"({sh1:.2f} vs {exact_sw1:.2f})")
    for n in (8, 24, 128):
        a, s = albersheim_total_snr_db(pd, pfa, n), shnidman_total_snr_db(pd, pfa, n, 0)
        check(f"Shnidman steady N={n} within 0.5 dB of Albersheim (total energy)", abs(a - s) < 0.5, f"({s:.2f} vs {a:.2f})")
    check("Coherent SW1 requirement independent of N (regression of V5.4.0 double count)",
          abs(detect_snr_req_db("COHERENT", 128, 1) - shnidman_total_snr_db(DETECT_PD, DETECT_PFA, 1, 1)) < 1e-9)
    check("SW1 penalty about 8 dB over steady at N=1", abs((sh1 - sh0) - 8.0) < 0.5, f"({sh1 - sh0:.2f})")

    check("S scan limit 75 deg at 2.8 GHz, 70 deg at 2.88 GHz",
          BAND_S.scan_limit_deg(2.8) == 75.0 and BAND_S.scan_limit_deg(2.88) == 70.0)
    _lo, _hi, _ed = BAND_S.usable_lo_ghz, BAND_S.usable_hi_ghz, BAND_S.edge_derating_db
    check(f"S core band has no edge loss; edges {_ed:g} dB; a 10 MHz signal averages the law; nothing outside the band",
          BAND_S.edge_loss_db((BAND_S.core_lo_ghz + BAND_S.core_hi_ghz) / 2) == 0.0
          and abs(BAND_S.edge_loss_db(_lo) - _ed) < 1e-9 and abs(BAND_S.edge_loss_db(_hi) - _ed) < 1e-9
          and 0.0 < BAND_S.edge_loss_db(_lo + 0.01, 10.0) < _ed and BAND_S.edge_loss_db(_lo, 10.0) == 100.0
          and BAND_S.fits(_lo + 0.1, 200.0) and not BAND_S.fits(_lo + 0.1, 201.0))
    check("Search positions at 2.8 GHz in (u, v), spacing 1.4 HPBW_TX (real pattern), about 1664 (3016 at 1.04)",
          abs(n_search_positions(2.8) - 1664) < 25, f"({n_search_positions(2.8):.0f})")
    check("Exact detection laws: SW1 one sample = closed form at Pfa 1e-6 and 1e-12; SW1 = SW2 on one sample",
          abs(detection.snr_req_db(1, 1, 1e-6, 0.9) - 10 * math.log10(math.log(1e-6) / math.log(0.9) - 1)) < 0.01
          and abs(detection.snr_req_db(1, 1, 1e-12, 0.9) - 10 * math.log10(math.log(1e-12) / math.log(0.9) - 1)) < 0.01
          and abs(detection.snr_req_db(1, 2, 1e-8, 0.9) - detection.snr_req_db(1, 1, 1e-8, 0.9)) < 0.01)
    check("TX beam of the uniform octagon 1.035 lambda / D_env (rectangular 0.886 replaced, step 1.1)",
          abs(PATTERNS["S"].tx.hpbw - 1.035) < 0.003, f"({PATTERNS['S'].tx.hpbw:.4f})")
    check("RX weighting reaches each sidelobe option on the real octagon",
          all(s - 0.06 <= rx_beam(b, s).sll_db <= s + 1e-9 for b in BANDS for s in RX_SLL_OPTIONS_DB))
    check("Active mismatch: 0 at the WAIM match angle, 0.13 dB per pass at broadside, grating-lobe term 0.76 dB at "
          "2.9 GHz / 75 deg with 0.5 mm gaps (1.8 dB with the WAIM term)",
          active_mismatch(2.8, 45.0)["loss_db_per_pass"] < 1e-9
          and abs(active_mismatch(2.8, 0.0)["loss_db_per_pass"] - 0.1298) < 1e-3
          and abs(-10 * math.log10(1 - active_mismatch(2.9, 75.0)["gamma_gl"] ** 2) - 0.76) < 0.05
          and abs(active_mismatch(2.9, 75.0)["loss_db_per_pass"] - 1.82) < 0.05)
    check("Angle CRLB x margin = 1.13 x 0.436 TX beamwidths / sqrt(SNR)", abs(angle_k_tx("S") - 0.4927) < 0.003,
          f"({angle_k_tx('S'):.4f})")
    check("Chain efficiency 0.945 and 1.784 MW electrical for TX at 2 MW (always-on 106.9 kW, step 4.1)",
          abs(CHAIN_EFF - 0.94525) < 1e-6 and abs(non_tx_power_w() - 106880.0) < 1.0
          and abs(face_elec_avail_w(2.0) - 1.78362e6) < 1e2)
    check("PAE: S 0.80 at d_ref, edges -1 % on 20 us, duty derating -2.75 % at d = 0.65; Ku 0.65",
          abs(pae(BAND_S, 1e-3) - 0.8 * 1e-3 / (1e-3 + 0.2e-6)) < 1e-12
          and abs(pae(BAND_S, 20e-6) / 0.8 - 20 / 20.2) < 1e-12
          and abs(pae(BAND_S, 1e-3, 0.65) / pae(BAND_S, 1e-3) - (1 - 0.05 * 0.55)) < 1e-12
          and abs(pae(BAND_KU, 1e-3) - 0.65 * 1e-3 / (1e-3 + 0.1e-6)) < 1e-12)
    check("tau_max from the buffer: S 2.0 ms, Ku 2.08 ms", abs(tau_max_s(BAND_S) - (2.0e-3 - 0.2e-6)) < 1e-12
          and abs(tau_max_s(BAND_KU) - (2.08e-3 - 0.1e-6)) < 1e-12)

    check("S 2.8 GHz: WF-LRS playable", [w.name for w in playable(2.8, 0.0)] == ["WF-LRS"])
    check("Ku 14.0 GHz: nothing playable (no Ku waveform in catalogue, no search in Ku)", playable(14.0, 0.0) == [])
    check("5.0 GHz (out of band): nothing playable", playable(5.0, 0.0) == [])
    check("S 2.8 GHz at 80 deg scan: nothing playable", playable(2.8, 80.0) == [])
    for label, kwargs in (("SEARCH in Ku", dict(role="SEARCH", band="Ku")),
                          ("TRACK in S", dict(role="TRACK", band="S", tracking_pack=TrackingPack(0.05, 2.0)))):
        try:
            Waveform(name="X", tau_s=1e-5, n=1, pri_s=1e-3, sig_bw_mhz=10.0, **kwargs)
            check(f"Policy rejects {label}", False)
        except ValueError:
            check(f"Policy rejects {label}", True)

    lrs = solve(WF_LRS, 2.8, 0.0, TARGETS["ST"])
    check("RCS scaling R ~ sigma^0.25 (same speed)",
          abs(solve(WF_LRS, 2.8, 0.0, Target("a", 1000.0, 1, 5000.0)).range_km
              / solve(WF_LRS, 2.8, 0.0, Target("b", 10.0, 1, 5000.0)).range_km - 100 ** 0.25) < 1e-6)
    # Independent re-derivation of the radar equation for the WF-LRS benchmark (no solve() internals)
    lam = C_MPS / 2.8e9
    gt = 4 * math.pi * PHYS_AREA_M2 * 1.0 * ETA_OHMIC / lam ** 2
    gr = 4 * math.pi * PHYS_AREA_M2 * PATTERNS["S"].rx[-25.0].eta * ETA_OHMIC / lam ** 2
    # step 2.4: ST (L 10 m, 0.1 deg/s, SW1/2, depol -10 dB): T0 = lambda / (2 L omega), 1 + dwell / T0 independent
    # samples over the 24 pulses, x the polarisation diversity (1 + rho)^2 / (1 + rho^2); cross-polar energy 1 + rho
    rho = 0.1
    lam_st = C_MPS / 2.8e9
    t0_st = lam_st / (2.0 * 10.0 * math.radians(0.1))
    fl_st = detection.Fluct(_quantise_shape(min(24.0, 1.0 + WF_LRS.dwell_s / t0_st) * (1 + rho) ** 2 / (1 + rho ** 2)))
    pol_glrt = detection.snr_req_db(25, fl_st, DETECT_PFA, DETECT_PD) - detection.snr_req_db(24, fl_st, DETECT_PFA, DETECT_PD)
    # finite precision, calibration, Ku_blockage, TX_chain, pol, compression (NLFM: f_d 93 kHz << 2 % of 75 MHz); the
    # pre-LNA path is in T_sys; CA-CFAR; compensation residual. Step 2.3: exact law, Pd averaged over the position of
    # the target (search grid, range straddle, velocity filter over 0-5 km/s; checked on their own in the tests)
    # array errors (step 3.4): Ruze over the residual errors at the end of a calibration interval, TX and RX
    er, t_min = BAND_S.errors, BAND_S.errors["cal_interval_s"] / 60.0
    ph2 = sum(math.radians(x) ** 2 for x in (er["trm_phase_deg"], er["trsa_phase_deg"], er["drift_phase_deg_min"] * t_min))
    am2 = sum((10 ** (x / 20) - 1) ** 2 for x in (er["trm_amp_db"], er["trsa_amp_db"], er["drift_amp_db_min"] * t_min))
    cal_db = 2 * 10 * math.log10((1 + am2) / math.exp(-ph2))
    l_db = (0.3 + cal_db + 0.2 + 0.6 + pol_glrt + 0.1) + cfar_loss_db() + MIGRATION_RESIDUAL_DB - 10 * math.log10(1 + rho)
    l_db += -20 * math.log10(1 - ((1 - math.cos(math.radians(45))) / (1 + math.cos(math.radians(45)))) ** 2)   # WAIM
    l_db += -10 * math.log10(1 - (2000 * 200.2e-6 / 0.8 / 25.0) / 2) + MB_SUP_LOSS_DB[4]   # 4-beam superposition
    half = velocity_bank(5000.0, 200e-6, 75.0, WF_LRS.dwell_s, 24, False)[2]
    law = position_losses("S", -25.0, -25.0, True, False, (round(half, 9), 200e-6, 75.0, WF_LRS.dwell_s, 24, False))
    snr_mean_db = detection.required_snr_db(24, fl_st, DETECT_PFA, DETECT_PD, law[0], law[1])
    check("Step 2.2 processing terms: CA-CFAR 0.04 dB (1024 cells), range straddle 0.05 dB (x4 grid, -25 dB), Doppler "
          "straddle 0.07 dB (x4 padding)", abs(cfar_loss_db() - 0.039) < 0.002 and abs(straddle_range_db(-25.0) - 0.054)
          < 0.005 and abs(straddle_doppler_db() - 0.07) < 0.005,
          f"({cfar_loss_db():.3f}, {straddle_range_db(-25.0):.3f}, {straddle_doppler_db():.3f})")
    snr = db_to_lin(snr_mean_db)
    r_hand = ((9.8e6 / 4) * gt * gr * lam ** 2 * 1000.0 * 200e-6 * 24
              / ((4 * math.pi) ** 3 * K_BOLTZ * BAND_S.t_sys_k * db_to_lin(l_db) * snr)) ** 0.25 / 1000.0
    l_pre = 10 ** ((0.3 + 0.35 + 0.2) / 10)
    nf_adc = 10.0 - (6.02 * 13 + 1.76) - 10 * math.log10(1.380649e-23 * 290 * 250e6 * 1e3)   # 13 bits, +10 dBm, 250 Msps
    check("ADC noise figure derived from the converter: 20.0 dB (13 ENOB, +10 dBm, 250 Msps)",
          abs(BAND_S.adc_nf_db - nf_adc) < 1e-9 and abs(nf_adc - 20.0) < 0.05, f"({BAND_S.adc_nf_db:.2f} dB)")
    t_hand = (3.2 + (l_pre - 1) * 320.0 + l_pre * (290 * (10 ** 0.075 - 1) + 290 * (10 ** 0.8 - 1) / 10 ** 2.75
                                                   + 290 * (10 ** (nf_adc / 10) - 1) / 10 ** 4.75)) * 10 ** 0.005
    check("S-band T_sys from the receive-chain cascade, hand calculation (0.1 %), cold sky 144.5 K",
          abs(BAND_S.t_sys_k / t_hand - 1) < 1e-3 and abs(BAND_S.t_sys_k - 144.5) < 0.1,
          f"({BAND_S.t_sys_k:.1f} vs {t_hand:.1f} K)")
    check("Range Taylor weighting nbar 5: -25 / -30 / -35 dB -> loss 0.41 / 0.68 / 0.93 dB, broadening 1.18 / 1.27 / 1.34",
          all(abs(range_weighting(s)["loss_db"] - lo) < 0.01 and abs(range_weighting(s)["broadening"] - br) < 0.005
              and abs(range_weighting(s)["psl_db"] - s) < 0.4
              for s, lo, br in ((-25.0, 0.407, 1.184), (-30.0, 0.679, 1.267), (-35.0, 0.926, 1.340))))
    check("Velocity-mismatch loss: 0 at n_c = 0, about 10 log(2 n_c) for large n_c; WF-LRS is NLFM",
          velocity_mismatch_loss_db(0.0) < 1e-9 and abs(velocity_mismatch_loss_db(50.0) - 10 * math.log10(100.0)) < 0.5
          and lrs.pulse["modulation"] == "nlfm" and lrs.losses["compression"] == 0.1)
    check("Dual-pol GLRT loss 0.47 dB on one coherent sample (exact law, SW1, Pfa 1e-8), track looks 0.05 dB",
          abs(pol_recombination_loss_db(BAND_S, "SEARCH", 1, 1) - 0.474) < 0.005
          and pol_recombination_loss_db(BAND_KU, "TRACK", 1, 1) == 0.05)
    check("ADC noise at about -56.4 dBFS, clipping about 53.4 dB above the noise (S)",
          abs(BAND_S.noise_ant_dbm + BAND_S.gain_to_adc_db - 10.0 + 56.37) < 0.05 and abs(BAND_S.clip_headroom_db - 53.37) < 0.05,
          f"({BAND_S.noise_ant_dbm + BAND_S.gain_to_adc_db - 10.0:.1f} dBFS, headroom {BAND_S.clip_headroom_db:.1f} dB)")
    check("Radar equation matches an independent hand derivation (0.5 %)", abs(lrs.range_km / r_hand - 1) < 0.005,
          f"({lrs.range_km:.1f} vs {r_hand:.1f} km)")
    # coarse plausibility bracket (test value): 5682 km with T_sys 346.5 K, 7099 km after review 0.3, 6916 km at 1.1,
    # 6814 km at 1.2 (active mismatch 0.26 dB round trip at broadside), 6890 km at 1.3 (buffer droop 0.24 -> 0.04 dB)
    check("Benchmark WF-LRS on ST at 2.8 GHz in [5000, 9000] km", 5000 < lrs.range_km < 9000, f"({lrs.range_km:.0f} km)")
    check("Grid loss at spacing 0.92 reproduces the 1.41 dB of the notes", abs(grid_loss_db(0.92) - 1.41) < 0.02,
          f"({grid_loss_db(0.92):.3f} dB)")
    import random
    random.seed(7)
    s_, acc, cnt = 1.4, 0.0, 0
    a1, a2 = (s_, 0.0), (s_ / 2.0, s_ * math.sqrt(3.0) / 2.0)
    sites = [(i * a1[0] + j * a2[0], i * a1[1] + j * a2[1]) for i in range(-2, 3) for j in range(-2, 3)]
    while cnt < 60000:
        x, y = random.uniform(-s_, s_), random.uniform(-s_, s_)
        if all(x * x + y * y <= (x - cx) ** 2 + (y - cy) ** 2 for cx, cy in sites):
            acc += 12.0412 * (x * x + y * y)
            cnt += 1
    check("Closed-form grid loss matches Monte-Carlo over the hexagonal cell (0.1 dB)",
          abs(acc / cnt - grid_loss_db(1.4)) < 0.1, f"({acc / cnt:.2f} vs {grid_loss_db(1.4):.2f} dB)")
    check("Positions x cell area = sector area (u, v)", abs(n_search_positions(2.8) * beam_cell_area_uv(2.8) - sector_area_uv()) < 1e-9)
    check("Sector uv area = 2.0006 (disc R=sin75 cut by the az=+/-45 planes, array tilt 25 deg)", abs(sector_area_uv() - 2.0006) < 0.002, f"({sector_area_uv():.4f})")
    _n_lim = n_search_positions(2.8, eps_min_deg=-20.0)
    check("Mission rear limit eps_min = -20 deg: ~1439 positions at 1.4 HPBW (2607 at 1.04)",
          abs(_n_lim - 1439) < 20, f"({_n_lim:.0f})")
    check("Default eps window (-90, +90) leaves the sector unchanged", abs(sector_area_uv() - 2.0006) < 0.002)
    check("uv count at small scan ~ angular count (cone 15 deg, 1 %)",
          abs(math.pi * math.sin(math.radians(15)) ** 2 / beam_cell_area_uv(2.8) /
              (math.pi * math.radians(15) ** 2 / ((math.sqrt(3) / 2) * (BEAM_SPACING_HPBW * math.radians(hpbw_deg(2.8))) ** 2)) - 1) < 0.03)
    check("Single-pulse coherent span is tau, not PRI",
          abs(Waveform(name="T", role="SEARCH", band="S", tau_s=4e-3, n=1, pri_s=27e-3, sig_bw_mhz=75.0,
                       integration="COHERENT").coherent_time_s - 4e-3) < 1e-12)
    check("WF-LRS dwell = (N-1)*PRI + tau", abs(WF_LRS.dwell_s - (23 * 27e-3 + 200e-6)) < 1e-12)
    # step 3.1, C: TX multibeam modes
    one = dataclasses.replace(WF_LRS, b_tx=1, tx_mode="single")
    tdm = dataclasses.replace(WF_LRS, b_tx=2, tx_mode="tdm")
    half = dataclasses.replace(WF_LRS, b_tx=2, tx_mode="partition", tx_cut="h")
    r1 = solve(one, 2.8, 0.0, TARGETS["ST"])
    check("Partition sub-apertures: 2450 / 1225 S TRMs, lattice cell x1.85 (half) / x3.43 (quadrant)",
          subaperture("S", "half_h").n_elements == 2450 and subaperture("S", "quarter").n_elements == 1225
          and abs(subaperture_cell_ratio("S", "half_h") - 1.85) < 0.02
          and abs(subaperture_cell_ratio("S", "quarter") - 3.43) < 0.03,
          f"({subaperture_cell_ratio('S', 'half_h'):.3f}, {subaperture_cell_ratio('S', 'quarter'):.3f})")
    check("The two halves are the same beam rotated by 90 deg (octagon x <-> y symmetric): one cut evaluated",
          partition_cuts() == ("h",))
    r_t = solve(tdm, 2.8, 0.0, TARGETS["ST"])
    check("Time multiplex: one beam's range but for the buffer droop of 2 tau back to back, 2 positions per dwell",
          abs((r_t.range_km / r1.range_km) ** 4 * db_to_lin(r_t.l_tot_db - r1.l_tot_db) - 1) < 1e-6
          and r_t.losses["buffer_droop"] > r1.losses["buffer_droop"]
          and cells_per_dwell(tdm) == 2 and tdm.tx_block_s == 2 * WF_LRS.tau_s)
    r_h = solve(half, 2.8, 0.0, TARGETS["ST"])
    check("Half aperture: P / 2 and G / 2 per beam (R^4 / 4 but for the grid loss of the widened beam)",
          abs((r_h.range_km / r1.range_km) ** 4 * 4.0 * db_to_lin(r_h.l_tot_db - r1.l_tot_db) - 1) < 1e-6,
          f"(range ratio {r_h.range_km / r1.range_km:.4f})")
    try:
        Waveform(name="WIDE", role="CONFIRM", band="Ku", tau_s=1e-4, n=1, pri_s=1e-3, sig_bw_mhz=250.0,
                 integration="COHERENT")
        check("Ku waveform above 200 MHz instantaneous bandwidth is refused", False)
    except ValueError:
        check("Ku waveform above 200 MHz instantaneous bandwidth is refused", True)
    try:
        solve(Waveform(name="LONG", role="SEARCH", band="S", tau_s=4e-3, n=4, pri_s=40e-3, sig_bw_mhz=75.0,
                       b_tx=1, integration="NONCOHERENT"), 2.8, 0.0, TARGETS["ST"])
        check("solve() rejects a pulse exceeding the TRM buffer", False)
    except ValueError:
        check("solve() rejects a pulse exceeding the TRM buffer", True)
    for label, args in (("scan above limit", (WF_LRS, 2.8, 80.0, TARGETS["ST"])),
                        ("S waveform at a Ku frequency", (WF_LRS, 14.0, 0.0, TARGETS["ST"]))):
        try:
            solve(*args)
            check(f"solve() rejects {label}", False)
        except ValueError:
            check(f"solve() rejects {label}", True)

    ku_trk = Waveform(name="TEST-KU-TRK", role="TRACK", band="Ku", tau_s=40e-6, n=128, pri_s=500e-6,
                      sig_bw_mhz=75.0, integration="COHERENT", tracking_pack=TrackingPack(0.05, 2.0))
    check("Synthetic Ku TRACK waveform playable at 14.0 GHz",
          [w.name for w in playable(14.0, 0.0, [ku_trk])] == ["TEST-KU-TRK"])
    check("Synthetic Ku TRACK waveform not playable at 80 deg", playable(14.0, 80.0, [ku_trk]) == [])
    rk = solve(ku_trk, 14.0, 0.0, TARGETS["KKV"])
    check("Synthetic Ku TRACK solve returns a finite positive range", 0 < rk.range_km < 1e6, f"({rk.range_km:.0f} km)")
    try:                                                                       # step 5.2 (P7)
        from . import margin
    except ImportError:            # run as a script: import the package (core.margin uses relative imports)
        import os
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from core import margin
    check("Reallocation defaults: mode in NONE / PRIO_SEARCH / PRIO_TRACK, 0 < load ceiling <= 1, floors > 0",
          margin.MODE_DEFAULT in margin.MODES and 0.0 < margin.CEILING_DEFAULT <= 1.0
          and margin.RR_MIN_SEARCH_S > 0 and margin.RR_MIN_TRACK_S > 0 and 0.0 < margin.P_LOSS_MAX < 1.0)

    print(f"\n{'ALL TESTS PASSED' if fails == 0 else str(fails) + ' TEST(S) FAILED'}")
    return 1 if fails else 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=f"Radar Performance Model V{MODEL_VERSION}")
    ap.add_argument("--freq", type=float, default=FREQUENCY_GHZ, help="frequency in GHz")
    ap.add_argument("--theta", type=float, default=SCAN_ANGLE_DEG, help="scan angle in deg")
    ap.add_argument("--target", default=TARGET_NAME, choices=sorted(TARGETS), help="target for the main range line")
    ap.add_argument("--elec-mw", type=float, default=FACE_ELEC_POWER_MW, help="electrical power of the face (MW)")
    ap.add_argument("--selftest", action="store_true", help="run regression tests")
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    report(args.freq, args.theta, args.target, args.elec_mw)
    return 0


# step 4.5: mean beam-shape loss of the navigation pointing error, in every band's losses (at the top of the band, where
# the beam is narrowest); set here because it needs the computed beamwidth (hpbw_deg)
try:
    from . import repoint as _repoint  # noqa: E402
except ImportError:            # run as a script: python core/radar_model.py
    import repoint as _repoint  # noqa: E402
for _b in BANDS.values():
    _b.losses_db["nav_pointing"] = _repoint.loss_db(hpbw_deg(_b.usable_hi_ghz))

if __name__ == "__main__":
    sys.exit(main())
