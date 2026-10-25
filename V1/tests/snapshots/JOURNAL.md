# Snapshot journal

One entry per intended refreeze of the test snapshots (`python tests/update_snapshots.py`): what changed in the
model, and the waveform / mode changes it caused in the test missions. Test missions and test values only: these are
not radar requirements or achieved performance.

## 2026-10-20 -- review 0.3, batches 1-5

Model changes (radar_model V6.0.13, see its header):
- Aperture derived from the TRSA build-up: emissive area 13.25 -> 13.2496 m2 (ranges -0.0008 %). Exact energy ties
  of the optimiser flipped two waveforms (S03, FWD): N = 24 <-> 32 at 200 us, both at the coherence floor.
- eta_ohmic 0.96 -> 1.0: ranges +2.06 %, search energy about -8 %.
- Gain scan dependence = cos(theta) projection only (exponent 1.1 and V5.4.0 scan-loss law removed): ranges at
  70 deg +9.2 %, at 75 deg +12.2 % -- OPTIMISTIC until step 1.2.
- Clock-jitter term removed (largest change 1e-5 dB).
- K_F 0.5 -> 0.6, RANGE_CRLB_KR 1.3 -> 0.6 (bound x 1.13): no effect on the test missions (detection- or
  angle-driven tracks).
- INTERIM coherent-dwell cap COH_T_MAX_S = 149 ms in the waveform generators (until step 2.2). Waveform sets that
  changed (S search, per sector; "COHE/NONC tau x N, B_tx, dwell"):
  - mission_example / S03:
    before COHE 600us x2 B1 54ms | COHE 800us x2 B1 54.2ms | COHE 200us x24 B1 1227.7ms | COHE 600us x24 B1 1228.1ms
    after  COHE 800us x2 B1 54.2ms | COHE 937.5us x3 B1 107.7ms | NONC 800us x8 B1 374.4ms | NONC 800us x32 B1 1655.3ms
  - mission_veille500 / CONE_D1..D4 (identical):
    before COHE 400us x24 B1 1227.9ms | COHE 400us x32 B1 1654.9ms | COHE 600us x32 B1 1655.1ms
    after  NONC 800us x16 B1 801.4ms | NONC 800us x24 B1 1228.3ms | NONC 937.5us x32 B1 1655.4ms
    confirmation mode KU7 -> KU1+7 (fewer false alarms: no Doppler cells per pulse)
  - mission_synthetic_mix / FWD:
    before COHE 600us x3 B1 80.7ms | COHE 800us x3 B1 80.9ms | COHE 100us x32 B1 1241ms | COHE 200us x24 B1 920.8ms
    after  COHE 600us x3 B1 80.7ms | COHE 800us x3 B1 80.9ms | COHE 937.5us x4 B1 121ms | NONC 800us x8 B1 281ms
  Effect: S search energy 0 to +12 %, Ku time -25 to -88 % (fewer false alarms to confirm).
- core/cost_table.json regenerated (same waveform space as the optimiser).

## 2026-10-20 -- review 0.3, batch 6 (S-band receive chain)

- S-band T_sys derived from its receive chain ([band.S.rx], Friis at the antenna port), "optimised front end" with
  the median estimates: 154.3 K (was the block value 346.5 K). The pre-LNA 0.3 dB left the signal-loss budget (now
  counted once, inside T_sys). Together +3.81 dB: every S range +24.55 %, S search energy about -60 % on the test
  missions, S time about -60 %, Ku time -8 to -36 %.
- Confirmation mode changes: mission_redundancy_demo / A1_TRIM and B4_FINE KU1+7 -> S2.
- Model self-test plausibility bracket of WF-LRS on ST widened to [5000, 9000] km (now 7223 km).
- core/cost_table.json regenerated; it now stores the parameter fingerprint (stale-table alert in the GUI).

## 2026-10-20 -- review 0.3, Ku receive chain and losses

- Ku T_sys derived from [band.Ku.rx] (same Friis formula as S, median of the Ku ranges): 238.7 K (was 400 K).
- Ku V5.6.0 lumped 8 dB (processing only, sec. 5.2.1) replaced by the S posts with Ku values (status D): DSP 2.0,
  CFAR 0.5, calibration 0.8, S_blockage 0.3, TX_chain 0.5 dB, plus polarisation 0.5 dB (was 0): 8.0 -> 4.6 dB.
- Together +5.64 dB: every Ku range +38.4 % (synthetic Ku track 4670 -> 6462 km); Ku time on air -52 to -77 % on the
  test missions, total energy -0.3 to -1.2 %.
- Confirmation mode change: mission_example (json and csv) / S02 KU1+7 -> KU7.
- core/cost_table.json regenerated (fingerprint changed; S content unchanged).

## 2026-10-20 -- review 0.3, TX chain (PA-output convention)

- TRM peak power specified at the PA output (where the PAE is defined). TX_chain derived per component
  ([band.<b>.tx]: harmonic filter, calibration coupler, line; + the circulator shared with the receive chain):
  S 0.3 -> 0.6 dB, Ku 0.5 -> 0.95 dB. Ranges S -1.71 %, Ku -2.56 %.
- Waveform sets changed (discrete choices) in mission_example / S03, mission_veille500 / CONE_D1-D4 and VEILLE_KV,
  mission_redundancy_demo / A4_SEARCH and A5_SHORT. Face energy +7 to +11 %, S time +7 to +22 %, Ku time +8 to +22 %
  on the test missions (E ~ L: +0.3 dB alone is +7 %).
- core/cost_table.json regenerated.

## 2026-10-20 -- review 0.3, S instantaneous bandwidth

- band.S.bi_max_mhz 0 (undefined) -> 200 MHz (digital per-TRM true-time delay in S as in Ku). No mission output
  changed (the S waveform space stops at 75 MHz); only the model snapshot record of the value. cost_table.json rebuilt
  for its fingerprint (content unchanged).

## 2026-10-20 -- review 0.3, Ku TX illumination

- band.Ku.eta_illum_tx 0.88 -> 1.0 (uniform TX, PAs at saturation, as in S; the V5.6.0 taper cut the gain but not
  the radiated power). Ku ranges +3.25 %; Ku time on air -5 to -17 % on the test missions, energy about -0.03 %.
  No confirmation mode change. cost_table.json rebuilt for its fingerprint.

## 2026-10-20 -- review 0.3, S/Ku alignment bias

- confirmation.bias_s_hpbw 0.05 S beamwidth -> confirmation.bias_mrad 1.1 mrad per axis (absolute angle, constant in
  direction cosines; was 1.13 mrad at 2.8 GHz and varied with the S frequency). Confirmation energy -0.14 % at most,
  no mode change. cost_table.json rebuilt for its fingerprint.

## 2026-10-20 -- roadmap step 1.1 (real radiation pattern of the octagonal TRSA array)

Model changes (radar_model V6.1.0, see its header; pattern computed by core/antenna.py, cached in
core/antenna_cache.json):
- TX beam of the uniform octagon 1.035 lambda / D_env (smallest cut) instead of 0.886 (rectangular aperture): search
  positions per face -26.8 % at every frequency (2.8 GHz: 4167 -> 3050).
- RX weighting = circular Taylor nbar 5 fitted on the octagon, per watch volume (rx_sll_db, default -25 dB): eta_rx
  0.88 -> 0.873 at the default (-0.035 dB; design level 29.5 dB, achieved -25.004 dB). Test missions have no rx_sll_db: all use the default.
- RX grid loss 0.42 dB on search looks (digital RX beams every 0.5 RX beamwidth; was 0).
- Together: WF-LRS ranges -2.58 % at every scan angle (ST at 0 deg: 7099 -> 6916 km); energy per position about +12 %
  but 27 % fewer positions: S search energy about -18 % on the test missions, S time -14 to -27 %, Ku time -8 to -40 %
  (fewer false alarms to confirm).
- Angle accuracy = CRLB of the aperture x 1.13 (was 0.5 x HPBW(0.886)): the test missions have no angle-driven track
  (model case only: synthetic Ku TRACK).
- Waveform sets that changed (S search, per sector; "COHE/NONC tau x N, B_tx, dwell"):
  - mission_example (and _csv) / S01: before NONC 100us x1 | COHE 100us x2 | COHE 200us x2;
    after COHE 100us x2 | COHE 200us x2
  - mission_example / S03: before COHE 400us x2 | COHE 800us x2 | NONC 937.5us x4 | NONC 937.5us x8;
    after COHE 400us x2 | COHE 800us x2 | COHE 800us x3 | NONC 800us x12
  - mission_veille500 / CONE_D1..D4: before NONC 800us x6 | NONC 800us x8 | NONC 800us x12;
    after NONC 800us x6 | NONC 600us x12 | NONC 937.5us x12
  - mission_veille500 / VEILLE_KV: before COHE 20us x6 | 50us x6 | 100us x8 | 200us x8;
    after COHE 50us x4 | 50us x6 | 100us x6 | 200us x8
  - mission_synthetic_mix / FWD: before COHE 400us x2 | 600us x2 | 600us x3; after COHE 400us x2 | 600us x2 | 800us x2 |
    800us x3
  - mission_synthetic_mix / KV: before COHE 20us x6 | 50us x4 | 50us x8 | 100us x8; after COHE 50us x4 | 50us x6 |
    100us x6 | 100us x12
  - confirmation mode KU7 -> KU1+7: mission_redundancy_demo / A2_VEILLE, mission_synthetic_mix / FWD
- New snapshot entries: model terms "patterns", "grid_rx_loss_db", "angle_k_tx", "rx_beams_per_position", solve() at
  each RX option.
- core/cost_table.json regenerated (now records the RX option it was built with).

## 2026-10-20 -- roadmap step 1.2 (active mismatch, grating lobes, far sidelobes, polarisation)

Model changes (radar_model V6.2.0, see its header):
- Active mismatch on TX and RX (loss "scan_mismatch" = 2 x per pass): current sheet + WAIM matched at 45 deg (S and
  Ku), combined with the grating-lobe term exp(-g / 0.02), g from the real mean pitch (S 52.4 mm, Ku 10.48 mm).
  Round trip 0.26 dB at broadside, 1.12 dB at 70 deg, 2.11 dB at 75 deg (2.8 GHz); 2.24 dB at 2.85 GHz / 75 deg.
  WF-LRS ranges -1.48 % at broadside, -6.25 % at 70 deg (ST: 6916 -> 6813 km at 0 deg, 4045 -> 3792 km at 70 deg).
- Test missions: energy per face +0.2 to +9.5 %, S time 0 to +19 %, Ku time -6 to +61 % (mission_veille500: longer
  coherent VEILLE_KV dwells -> more Doppler cells -> more false alarms to confirm). No confirmation-mode change.
- Waveform sets that changed (S search, "COHE/NONC tau x N"):
  - mission_example / S03: COHE 400us x2 | 800us x2 | COHE 800us x3 | NONC 800us x12
    -> COHE 400us x2 | 800us x2 | NONC 800us x8 | NONC 800us x24
  - mission_veille500 / CONE_D1..D4: third WF NONC 937.5us x12 -> NONC 800us x16
  - mission_veille500 / VEILLE_KV: COHE 50us x4 | 50us x6 | 100us x6 | 200us x8 -> 50us x4 | 50us x8 | 100us x12 |
    200us x12
  - mission_redundancy_demo / A3_HIBAND: + COHE 100us x3 (4th WF); B3_EDGE: NONC 50us x1 -> 100us x1;
    A5_SHORT: COHE 200us x8 dropped (3 WFs)
  - mission_synthetic_mix / KV: COHE 50us x4 | 50us x6 | 100us x6 | 100us x12 -> 50us x4 | 50us x8 | 100us x8 | 100us x12
- New outputs: far sidelobes / visible sidelobe vs scan (TRSA gap lobes), RX option held up to (deg), max mismatch per
  sector, step-1.2 flags (mission_example S03: Ku RX -25 dB held only up to 61 deg at 14.0 GHz), pol_purity.
- core/cost_table.json regenerated.
- Same step, user decision after the gap-lobe finding: aperture.trsa_gap_m 2 mm -> 0.5 mm. Everything derived follows
  (total area 13.4368 -> 13.2964 m2, fill 99.65 %, envelope 4.190 -> 4.1675 m, mean pitch S 52.1 / Ku 10.42 mm,
  pattern cache recomputed): positions -1.14 %, ranges unchanged (emissive area unchanged; -0.002 %), energy per face
  0 to -4.9 %, S time 0 to -1.4 %, Ku time -0.1 to -1.1 %. No waveform-set or confirmation-mode change. Gap lobes
  Ku -16.6 -> -25.3 dB: S options held over the whole sector, Ku -25 dB everywhere (no more step-1.2 flag in the
  test missions); Ku -30 / -35 dB held to 68.5 / 58.5 deg at 14.25 GHz. Mismatch at 2.9 GHz / 75 deg 4.1 -> 1.8 dB.

## 2026-10-20 -- roadmap step 1.3 (TRM and transmit chain, GaN-on-diamond with microchannel cooling)

Model changes (radar_model V6.3.0, see its header):
- ELECTRICAL face budget: P_elec x CHAIN_EFF for transmission, each look costs RF / PAE(band, pulse, duty);
  PAE S 0.80 / Ku 0.65 at duty 0.10 (was 0.75 shared), edges 0.2 / 0.1 us, duty derating 0.05 / 0.08.
- TRM buffer S 5 -> 25 J, Ku 0 -> 1.2 J, max fraction 0.5 -> 0.2: tau_max S 937.5 us -> 2.0 ms, Ku 2.08 ms;
  WF-LRS buffer droop 0.24 -> 0.04 dB: WF-LRS ranges +1.12 % (ST 6813 -> 6890 km at 0 deg). duty_max 0.65 on trains.
- Test missions: energy per face -5 to -26 % (PAE S, lower droop, 2 ms pulses), S time -20 to +66 %, Ku time -48 to
  +80 % (Ku PAE 0.65, duty ceiling 0.25 -> 0.65 opens denser Ku looks). Confirmation mode KU7 -> KU1+7:
  mission_example (and _csv) / S02, mission_synthetic_mix / KV.
- Waveform sets that changed (S search, "COHE/NONC tau x N"): the new 2 ms pulse (1999.8 us) is used by
  mission_example / S03 (NONC 800us x1 | NONC 2ms x1 | COHE 2ms x2 | NONC 2ms x6), mission_veille500 / CONE_D1..D4
  (COHE 2ms x2 | COHE 2ms x3 | NONC 2ms x4, were NONC 600-800us x6..x16) and mission_synthetic_mix / FWD; single
  pulses for mission_example / S01 (NONC 200us x1 | 400us x1, were COHE x2); shorter coherent trains for S02,
  VEILLE_KV, A5_SHORT, KV; A2_VEILLE, B5_LONG and A3_HIBAND drop one waveform.
- New outputs: PAE per waveform, electrical average power, TRM heat in solve(); face heat (kW, kW/m2) in the
  optimiser (search) and the tracking budget (search + confirmation + AT).
- Optimistic until 3.1: the PAE is kept under power back-off (power scale, Ku look back-off). Until 2.1: blind range
  of the 2 ms pulses (300 km) and eclipsing at high duty not checked.
- core/cost_table.json regenerated (energies now electrical).

## 2026-10-20 -- roadmap step 1.4 (the band)

Model changes (radar_model V6.4.0, see its header):
- Edge derating per band: core band in GHz (S 2.74-2.86, Ku 13.7-14.3), raised cosine to the usable edges, 0.5 dB
  at the edges (was 1.5 dB, core = band minus 1/6), total loss averaged over the occupied spectrum f +/- B/2.
- Spectrum occupancy: a signal must fit in the usable band; frequencies outside the band are skipped by the
  optimiser. True-time delay explicit (digital per TRM); S instantaneous bandwidth 200 MHz confirmed; scan limits kept.
- Test missions: the optimiser keeps 2.8 GHz (core band, no edge loss), so ranges and waveforms barely move: energy
  per face 0 to -1.5 %, S time -2.0 to +0.5 %, Ku time 0 to -7 %.
- mission_redundancy_demo / A3_HIBAND forces freq_ghz_candidates [2.9] (the upper band edge): no signal fits there any
  more, the sector is now NO_WAVEFORM ("no playable waveform in the search space"). Kept as is (it now exercises that
  path); move it inside the band (e.g. 2.86 GHz) if a playable high-band sector is wanted.
- core/cost_table.json regenerated.

## 2026-10-21 -- roadmap step 1.5 (the receive chain)

Model changes (radar_model V6.5.0, see its header; optimistic end of every range, user):
- Cold-sky T_sys: t_ant_k S 10 -> 3.2 K, Ku 5 -> 2.8 K; ADC noise figure derived from the converter (13 ENOB,
  +10 dBm, 250 Msps: 20.0 dB, was 27.5 dB entered). T_sys S 154.3 -> 144.5 K, Ku 238.7 -> 230.8 K.
- Polarisation: the 0.5 dB placeholder replaced by the GLRT loss SNR_req(n + 1) - SNR_req(n) (0.54 dB on one
  coherent sample, 0.06 dB on the 24 noncoherent pulses of WF-LRS), 0.05 dB on TRACK looks.
- Together WF-LRS ranges +4.28 % (ST 6890 -> 7185 km at 0 deg).
- Missions: sky noise by direction (core/sky.py). Without a galactic frame (every test mission but the new one) the
  worst-case galactic rise is applied everywhere: +7.5 K in S, budget T_sys 152.1 K (+0.22 dB over the cold sky);
  so the mission ranges move only +0.1 to +2.4 %, face energy -0.6 to -1.1 %, S time 0 to -11 %, Ku time 0 to -9 %.
- Waveform sets that changed: mission_veille500 / CONE_D1..D4, second waveform COHE 2ms x3 -> NONC 2ms x3 (the GLRT
  loss is smaller on noncoherent trains).
- New test mission mission_sky_demo.json (Sun, Moon, Earth limb, galactic frame): blinded positions, budget quantile,
  Ku sky, AT bracket below the Ku saturation range.
- New outputs: sector "sky" (S) and tracking "ku_sky" summaries, ADC saturation ranges, rx_deaf_us per waveform, the
  isolation matrix and the scheduler rules (sync per band, S harmonic in the Ku channel).
- core/antenna_cache.json rebuilt (ALGO_VERSION 5: azimuthal-mean pattern profiles); core/cost_table.json regenerated
  (fast estimate: cold sky, the sky noise is not in it).

## 2026-10-21 -- roadmap step 2.1 (parametric waveforms)

Model changes (radar_model V6.6.0, see its header; optimistic end of every range, user):
- Former DSP 2.0 dB split: finite_precision 0.3 + straddle 0.7 (lump until 2.2) + a COMPUTED loss "compression":
  NLFM 0.1 dB while f_d,max / B <= 2 %, else LFM with a Taylor nbar 5 receive weighting at the volume's sidelobe
  option (0.41 / 0.68 / 0.93 dB at -25 / -30 / -35 dB) + Doppler loss on the ridge. In S at 75 MHz every look is NLFM:
  -0.9 dB of losses, WF-LRS ranges +5.32 % (ST 7185 -> 7567 km).
- k_r from the weighted spectrum (0.70 at -25 dB); physical range resolution (x1.18 at -25 dB): the default 2 m
  needs 89 MHz, so every sector asking 2 m is flagged "range resolution not met" (2.37 m at 75 MHz) until 3.1.
- Blind range c (tau + recovery) / 2 <= range_min_km (default 10 % of range_km), unambiguous PRI >= 2R/c + tau +
  recovery (no self-eclipsing), staggered ambiguous trains (Ku looks at long range) with the mean eclipsing loss.
- Test missions: energy per face -22 to +6 % (losses -0.9 dB; +6 % on one face of mission_sky_demo), S time -7 to -50 %, Ku time -42 to +53 % (eclipsing of
  the ambiguous Ku trains vs the lower losses). Short-range sectors lose their long pulses (S02 100 -> 50 us, B3_EDGE,
  S01 adds a 100 us waveform); mission_redundancy_demo / B2_AUTOPROT (20 km) is now NO_WAVEFORM: the shortest pulse of
  the search space (20 us) leaves 3 km blind, above its default range_min_km of 2 km (kept: exercises the path;
  shorter pulses belong to the search space of step 3.1).
- New outputs per waveform / look: modulation, resolution, k_r, blind range, staggering, cells crossed in the pulse,
  velocity filters for 1 dB, range-Doppler coupling, AM/PM drift, Doppler ambiguity.
- core/cost_table.json regenerated.

## 2026-10-21 -- roadmap step 2.2 (processing)

Model changes (radar_model V6.7.0, see its header; optimistic end of every range, user):
- Lumps replaced by computed losses: straddle 0.7 dB -> range 0.05 dB (x4 grid) + Doppler 0.07 dB (x4 padding, coherent
  blocks only); CFAR 0.5 dB -> CA-CFAR 0.04 dB (1024 cells); migration 1.0 dB -> mean velocity-bank loss (1 dB at half
  a step) + 0.1 dB residual. WF-LRS: losses 7.00 -> 5.34 dB, ranges +10.0 % (ST 7567 -> 8325 km).
- Coherent blocks of optimal length (no cap, no eta_min floor); provisional target t0 0.25 s; LO phase noise and
  platform motion (negligible). Optimiser and Ku look generators keep their former waveform spaces without the cap.
- False alarms on independent cells: range from range_min x Doppler cells of a block x velocity hypotheses x angle
  cells (0.85 per position). Trains now need velocity banks of 50 to 535 hypotheses: their false alarms, hence the Ku
  confirmation looks, grow accordingly; the search optimiser does not see that cost when it picks its waveforms.
- Test missions: S energy per face -20 to -36 %, S time -14 to -50 %, but Ku time -53 % to x300 (velocity banks of the
  trains: mission_veille500 x30, mission_sky_demo up to x300 on one face, 0.25 of Ku time); most sectors switch to
  single pulses (labelled COHE, equivalent for N = 1); confirmation modes KU1+7 -> KU7 on several sectors.
- New outputs: coherent blocks, pulses per block, velocity hypotheses per waveform; Doppler floor.
- core/cost_table.json regenerated.

## 2026-10-21 -- roadmap step 2.3 (detection criterion)

Model changes (radar_model V6.8.0, see its header):
- Exact detection laws, Swerling 0-4, any Pfa (core/detection.py); Shnidman kept as a cross-check (within 0.3 dB).
- Pd of a look averaged over the random position of the target (search lattice cell and RX lattice with the real
  patterns, range / Doppler straddle, velocity-filter offset): one loss 'positions' replaces grid_tx, grid_rx,
  straddle_range, straddle_doppler and the bank's mean loss in the budget.
- Search criterion: detection.pd 0.9 = CUMULATIVE probability reached at range_km over the looks of a target closing
  at v_close_max, spaced v x RR (q = v RR / R), phase averaged, no cap on the looks counted (Pd > 0.001).
- False alarms: 'fa_rate' mode, 10 per sector and revisit (Pfa per cell 1e-11 to 1e-7 on the test missions).
- Search lattice 1.04 -> 1.4 TX beamwidths (lattice shifted every frame: scheduler rule).
- WF-LRS benchmark (CLI, single look, no criterion parameters): ranges -10.2 % (positions loss 4.6 dB with the
  wider lattice at Pd 0.9 on one look; ST 8325 -> 7479 km).
- Test missions, two regimes: sectors with q << 1 (long range, e.g. 8000 km / 120 s / 5 km/s: q = 0.075, tens of
  looks) need far less energy: -69 to -96 % per face; the anti-KKV sectors (500 km, RR 20 s, 25 km/s: q = 1, the
  target covers the whole required range between two looks) need more: +280 to +305 % on the faces they dominate
  (mission_veille500, mission_synthetic_mix). Ku time mostly -80 to -99 % (10 false alarms per revisit instead of
  hundreds), +1000 % on one face of mission_example_csv. Many confirmation modes change (KU7 -> KU1+7 or S2).
- core/cost_table.json regenerated (fast estimate: still single look at detection.pfa, see the renvoi to 5.5).

## 2026-10-22 -- roadmap step 2.4 (target models)

Model changes (radar_model V6.9.0, see its header; target classes are CLASS MEANS, user):
- Target library [targets.*] (schema checked at load, classes added in the file only): ST (reference), GENERIC
  (completes a sector naming no class), KKV, SHIP, DEBRIS, DECOY; sector key target_type with overrides.
- T0 = lambda / (2 L omega) per class and band in the coherence loss (was the provisional t0 0.25 s); fluctuation as a
  Gamma shape (declared case, or family x min(n, 1 + T_obs / T0)) x the polarisation diversity (1 + rho)^2 / (1 + rho^2);
  cross-polar energy collected: loss target_depolarisation -10 log(1 + rho); acceleration loss / bank (multiplies the
  velocity hypotheses); KKV seeker optics (frontal RCS 1.27 m2 in S, 13.8 m2 in Ku); SHIP frontal RCS 300 m2 (was the
  lateral 3000 m2). Outputs: masking envelopes, OS-CFAR option, alert below 3 T0.
- WF-LRS benchmark (CLI, reference classes): ST 7479 -> 8743 km (+16.9 %: SW1/2 with 1.2 independent samples over the
  dwell, diversity x1.2, cross-polar energy -0.41 dB); KKV 806 -> 1801 km (+123 %: optics); SHIP 9843 -> 8089 km (-17.8 %:
  frontal RCS).
- Test missions (sectors without class: their RCS / Swerling, GENERIC for the rest, i.e. -10 dB depolarisation, T0
  1.0 s in S / 0.2 s in Ku, 1 g): ranges +0.4 to +5 % at the same power, face energy -5 to -24 % (mission_synthetic_mix
  F3 -50 %), S time 0 to -28 %, Ku time +3 to -35 %. No confirmation-mode change.
- Waveform sets that changed: mission_example / S03 (third WF 600 -> 400 us), mission_veille500 / VEILLE_KV and
  mission_synthetic_mix / KV (shorter coherent trains added), mission_redundancy_demo / B5_LONG (+ a 20 us x 8 beams WF),
  mission_sky_demo / SUN (drops its 600 us x2 train) and EARTH (third WF 800 -> 600 us x6).
- New test mission mission_target_library.json (KKV, KKV with eta_r 1 override, SHIP, DEBRIS, DECOY sectors).
- core/cost_table.json regenerated (GENERIC SW1, see the renvoi to 5.5).

## 2026-10-22 -- roadmap step 2.5 (environment)

Model changes (radar_model V6.10.0, core/environment.py wired in optimizer.sector_sky; environment values are medians):
- Ship structure as primitives in [ship] (also the 3D view): masking by the blocked fraction of the aperture tube
  (none of the owned directions is blocked with the 25 deg tilt; the radiator wings are never seen), hull noise
  through the far sidelobes (negligible: test missions move below 1e-6 relative).
- Surfaces ([sky.surface], constant gamma): shadow, sidelobe clutter on the iso-range ring as noise (clutter-limited
  positions blinded), Doppler rejection when the sky gives the surface's closing band, main-lobe ADC alert.
- Jammers ([jammers.*] class means): stand-off ones in the mission sky (noise through the RX pattern, 40 dB sidelobe
  nulling x 8 per face), self-protection ones by sector (burn-through output). Objects to confirm (objects_per_rr),
  Faraday rotation output, GO-CFAR option and limb flag.
- mission_sky_demo / EARTH (Earth seen from low orbit, angular radius 60 deg): sidelobe clutter of the Earth ring at
  the sector range: budget T_sys 255 -> 998 K, 19.8 % of the positions clutter-limited (blinded), faces F3 / F4 energy
  +168 / +160 %, S time +140 / +137 %, Ku time +57 %; waveform set: longer coherent trains (COHE 800us x4 / x8 / x12 /
  x24). Every other test mission: no key figure changed.
- New test mission mission_environment.json (Earth and Moon surfaces, two stand-off jammers, a KKV with SPJ_KKV,
  objects and Faraday inputs). New derived file core/structure_cache.npz.
- core/cost_table.json regenerated (fingerprint: new parameters; the fast estimate has no environment, see 5.5).

## 2026-10-22 -- ship geometry: the 3D model (radar_model V6.10.1)

Model changes (see the header of core/radar_model.py):
- The low-poly 3D model assets/models/LP_NISO.obj replaces the [ship] primitives (masking, structure noise, element
  positions, 3D view; user). Length 149.46 m (was 136.4), width 74.68 m (was 76.5), hull diameter 17.22 m (was 16.8),
  aperture centres 5.248 m from the axis (was 4.926), all taken from the model; load-time checks of the dimensions and
  of the four apertures (on the panel front faces, normal at the 25 deg tilt).
- Model case: blocked cells per face 735 -> 621, structure directions seen from the arrays about 310 -> 335 (hull only,
  e T 2.9 K); still no owned direction blocked and the radiator wings never seen.
- Test missions: structure noise through the far sidelobes only, every key figure within 0.01 %; no waveform or
  confirmation-mode change. core/structure_cache.npz rebuilt; core/cost_table.json regenerated (parameter fingerprint).

## 2026-10-22 -- frames of the volumes and navigation message (radar_model V6.11.0, roadmap step 3.0)

Model changes (see the header of core/radar_model.py): core/frames.py turns the volumes and the sky into the ship
frame with the mission navigation (global key nav, sector key frame, sky key frame) before any computation.
- Existing test missions: no frame key, so no change; their snapshots were NOT refrozen (pytest confirms them).
- New test mission mission_frames.json (LEO 400 km, ship pointed nadir, one sector per frame, inertial sky):
  snapshot frozen alone (python tests/update_snapshots.py mission_frames).
- core/cost_table.json unchanged (tool.frames is outside the fingerprint of the derived files).

## 2026-10-22 -- step 3.1 pass P1: S frequency grid (radar_model V6.11.1)

Model changes (see the header of core/radar_model.py): S centre frequencies derived from the band (5 MHz step, the
signal inside the band and outside spectrum_exclusions_ghz), two-pass search (20 MHz then 5 MHz), deterministic tie
breaking (snapped costs, secondary key), exact cache of detection.required_snr_db.
- Test mission change: mission_redundancy_demo / A3_HIBAND forced 2.9 -> 2.86 GHz (no signal fits at 2.9 GHz; the
  forced frequency is now checked at load). It is now OK (3 waveforms, 15 % unscanned at the 70 deg limit).
- Frequencies leave 2.80 GHz (e.g. mission_example 2.825 / 2.785 / 2.825 GHz): ranges and face loads within about
  +/-2 %; mission_sky_demo face loads -11 to -14 % (frequency away from the Sun's worst rise), mission_frames up to
  -8 %. Waveform sets changed: VEILLE_KV (pulse counts), B5_LONG, SUN, EARTH, HORIZON, AXIS (one pulse length).
- core/cost_table.json regenerated (tool.optimizer settings changed: fingerprint). Model case unchanged.

## 2026-10-22 -- step 3.1: S band widened to 2.60-2.93 GHz (radar_model V6.11.2)

Values only (user W1-W7): usable 2.70-2.90 -> 2.60-2.93 GHz, core 2.74-2.86 -> 2.66-2.88 GHz, edge loss 0.5 -> 1.0 dB.
- Model case: 204 values (edge-loss checks, the far-sidelobe visible region follows the band top, antenna cache rebuilt).
- Test missions: face loads within about +/-1 % except mission_sky_demo faces 3-4 (-13 to -14 %, EARTH sector:
  frequency farther from the worst rise) and mission_frames face 4 (-2.7 %). Waveform sets changed: S03, A5_SHORT,
  EARTH, ORBIT (one pulse length or count).
- core/cost_table.json regenerated (band values changed). 153 tests pass (2 min 19 s in parallel).

## 2026-10-22 -- step 3.1 pass P2: S waveform space (radar_model V6.11.3)

Pulses 5 us to 2.0 ms (13 values), bandwidths 10-200 MHz, frequency agility on noncoherent trains, exact pruning of
dominated candidates. Most test sectors now pick one 5 us pulse at 100 MHz (default 2 m resolution); B2_AUTOPROT
(test sector, 20 km) gets a waveform; agility not chosen on the test missions. Impact report not run (token budget);
snapshots refrozen, core/cost_table.json regenerated. 157 tests pass, 7 min 06 s in parallel (budget 3 min exceeded:
accelerations X1, X2, X3, X5 decided for the next pass).

## 2026-10-22 -- step 3.1: runtime X1, X2, X3, X5 (radar_model V6.11.4)

walk_loss_db in closed form (sine integral): values move by < 2e-5 dB, so the snapshots of the model case and of the
missions with migration losses were refrozen; no waveform or decision is meant to change. X1, X3, X5 change no value.
core/cost_table.json regenerated.

## 2026-10-23 -- step 3.1 pass P3: S TX multibeam (radar_model V6.11.5)

Multibeam modes (phase-only superposition 0.4 / 0.6 / 0.8 dB, aperture partition halves / quadrants on their own
lattice, time multiplex modelled but not searched) replace the l8_db / p law; "no excess energy" now shortens the
pulse (PAE of the shortened pulse, floor tau B >= 1) instead of scaling the power. Impact report (5467 values):
- Model case: WF-LRS (4-beam superposition) range -0.50 % (loss 0.6 dB, was 0.51 dB with the old law).
- Test missions: S time on air -13 to -99 % per face (the shortened pulse saves time too), face energy 0 to +3.8 %
  (lower PAE of the shortened pulse; mission_example_csv F1 +3.8 %, mission_example F1 +1.2 %, others < 0.4 %).
  Range at worst scan (full pulse) -10 to -28 % on several sectors: shorter pulses now suffice, still above the
  requirement. The partition is chosen on the short-range test sectors (A1/B1_TRIM, B2_AUTOPROT, A3_HIBAND,
  B3_EDGE, A4_SEARCH, B4_FINE, B5_LONG, SHIPW: PARTx2h / PARTx4): it spreads the fixed cost of a pulse (edge time,
  1 / B floor) over 3.7 or 13.7 positions. Superposition never chosen; the other sectors keep one beam, some with a
  different pulse length (S01, A2_VEILLE, PROT, T_SHIP).
- Output key power_scale renamed pulse_scale; wf dicts gain tx_mode, tx_cut, positions_per_dwell,
  rx_beams_per_position. core/cost_table.json regenerated (infeasible cells unchanged, 1000; median energy +0.4 %).

## 2026-10-23 -- step 3.1 pass P4: environment per waveform (radar_model V6.11.6)

S sidelobe clutter per candidate and per 1-deg bin (TX beam incl. partition sub-apertures, range cell c / 2B, coherent
blocks, progressive Doppler rejection, Pd averaged over the target velocities), out of the S T_sys budget; Taylor
-30 / -40 dB Doppler weightings offered where a surface with a known closing speed clutters a sector; requirement
brought back to d - 10 km in front of a surface (classes rounded up on 5 % of the range); self-protection jammer in
the range (exact quadratic). Plan rule R4 b: with no candidate reaching the range, the longest reach wins before the
cost. Impact report run before R4; it ran every mission in one process and so carried the cache defect below for
mission_sky_demo and mission_frames: the figures here are read from the corrected snapshots (key figures, P3 -> P4).
- Missions without surfaces or self-protection jammer: no key figure changes (new output key only: doppler_window_db
  of the waveforms).
- Sectors that see a surface: the S T_sys budget loses the clutter (mission_sky_demo EARTH 686 -> 255 K,
  mission_frames ORBIT 244 -> 148 K, HORIZON 434 -> 149 K, mission_environment E_EARTH 488 -> 440 K); the clutter is
  now paid per waveform and per bin, at the bin quantile (stricter near the limb than the former sector quantile).
  mission_frames faces -7 to -48 %; mission_sky_demo face 3 -9 %, face 4 +45 %; mission_environment face 1 x5.9
  (E_EARTH: long noncoherent trains near the limb; some bins switch to noncoherent trains, the clutter over M blocks).
- E_KKV_SPJ (SPJ_KKV in the range): no candidate reaches 500 km (104 km at best, broadside). Before R4 the plan took
  the cheapest all-short candidate (about 1 km); with R4 b alone the longest reach, coherent 200 us x 32, 68 km, at
  full power on 582 positions (face 2 at 124 % of its energy). Final rule R5 b (user): a sector whose requirement the
  self-protection jammer puts out of the search's reach is planned against the noise alone and flagged (the search
  gives the jamming axis, the scheduler points a Ku burn-through search, step 4.3): E_KKV_SPJ back to 20 / 50 us,
  511 to 542 km against the noise, its P3 face cost.
- Cache defect found while refreezing: in a long process the S surface data could be dropped when the sky-summary
  cache was cleared on insertion (clutter then ignored for that sector); fixed before the final snapshots
  (test_surface_data_survive_a_full_sky_cache).
- core/cost_table.json regenerated (parameter fingerprint; no surface in its grid).

## 2026-10-23 -- step 3.1 pass P5, intermediate state (radar_model V6.11.7): snapshots NOT refrozen

Sky refinement near hot sources and the search lattice spacing per sector (1.3 to 2.0, exact search). By the test
rhythm decided on 23 October 2026 (full suite, impact report and snapshots every second pass), the snapshots of P4
are kept: test_snapshots fails until P6 refreezes them. Spot checks (not frozen): mission_example sectors choose
1.6 / 1.5 / 1.6 with the 1.3-1.6 range (the range was then widened to 2.0); mission_example runs in about 2 min 20 in
parallel on the laptop (71 s before P5).

Same day, still P5 (V6.11.7, snapshots NOT refrozen): rotation test D7 settled by the user (option 3): every
refined cell split into 8 x 8 points (tool.sky.refine_subsamples = 8), each blinded or not and counted on its own
in the budget quantile; cells scanned for the share left (positions count it). Measured at the test angles 0.37 /
1.13 deg: budget 0.15 / 0.81 %, blinded 0.16 / 0.40 % (per-cell values: 1.05 / 0.92 % and 0.70 / 1.41 %); drift
1.4 % at 2.9 deg (open point o). Snapshots will move at P6 near hot sources (sky_demo, frames, environment).

## 2026-10-23 -- step 3.1 pass P6 (radar_model V6.11.7): snapshots refrozen (P5 + P6)

Causes: P5 sky refinement (refined cells evaluated one by one, then split into 8 x 8 points, each blinded or not and
counted on its own in the budget quantile; option 3, user) and the search lattice spacing chosen per sector among
1.3 to 2.0 (E7 a). The runtime changes of the same day (caches A1-A5, look cut-off, Numba sums of the body noise) do
not move any output (mission_example identical to the character; mission_sky_demo Numba vs NumPy within 1e-9).
cost_table.json regenerated (parameter fingerprint). 4966 output values differ; the model case does not change
at the 1e-9 tolerance (model.json is rewritten with last-digit, ~1e-16, changes of the confirmation tables: the
looks left out before the interpolation (A2) change the grouping of the product, not its value).
Test values only, not radar performance. Main moves:
- mission_sky_demo EARTH: range at worst scan 1748 -> 1018 km (-42 %), positions short of range 0.3 -> 0.8 % (still
  under short_tol 1 %), blinded 14.48 -> 15.59 %, F3 energy +17 %: the 8 x 8 split along the Earth limb with the
  per-sector spacing; the plan now accepts a few more short positions at the worst scan (believed, not verified).
  SUN budget T_sys 379.5 -> 370.8 K, blinded 1.094 -> 0.978 %. MOON confirmation mode KU1+7 -> S2 (F2 Ku time -75 %).
- Waveform sets move in every mission with the per-sector spacing (e.g. mission_example S03 400/1000 us -> 600/1500 us;
  mission_redundancy_demo B5_LONG three waveforms -> one; mission_synthetic_mix SHIPW partition x2h -> x4). Face
  energy and time change by -98 % (an almost idle face of mission_redundancy_demo) to +17 %.
- Ranges at worst scan move by -13 % to +22 % outside EARTH (the cheapest plan per spacing, not a longer reach).
Case times (P6 script, one computation for the report and the freeze, frequencies in parallel): model 4 s,
mission_example 245 s, _csv 51 s, veille500 155 s, redundancy_demo 589 s, synthetic_mix 259 s, sky_demo 278 s,
target_library 251 s, environment 285 s, frames 835 s (49 min in all).

## 2026-10-25 -- step 3.2 closed (passes Q1-Q6): Ku confirmation rebuilt + speed-ups (radar_model V6.13.0)

Causes, cumulated since the P6 refreeze of 2026-10-23: step 3.2 Q1-Q5 (V6.12.0: Ku frequency per sector, Ku look
space, cluster modes, S2 touch as an S CONFIRM waveform, Gamma statistics, jammer and clutter per Ku look); Q6 exact
speed-ups E1-E4 (V6.12.1, solve() split, bit-identical on 129 024 calls of mission_example against V6.12.0); the
approximations chosen by the user once the absolute-precision rule was removed (V6.13.0): A1 position-loss laws
rounded to 0.02 dB, A2 cumulative inversion to 1e-3 dB, A4 cached pol loss (exact), A5 lattice spacing searched at the
frequency kept, A6 / A8 three S / Ku centres in a quiet spectrum, A9 S2 margins 0.05 dB / 16 quantiles / 8 GH nodes.
cost_table.json regenerated. 13 178 output values differ. Test values only, not radar performance. Main moves:
- Ku time per face -75 to -100 % in most missions (the 3.2 look space: shortened, cheaper Ku looks).
- Confirmation mode S2 -> KU1+7 in S01 (mission_example and _csv), A2_VEILLE, A5_SHORT, MOON, T_KKV_RETRO;
  T_DEBRIS KU7 -> KU1+7; EARTH KU1+7 -> none admissible. Checked on mission_example: the three modes load the
  mission equally to 1e-4 (0.0119); with the S2 tolerance back at 0.01 dB S01 stays KU1+7: ties, not a bias.
- mission_veille500 VEILLE_KV: S energy and time of every face +9.1 %, 2.765 -> 2.77 GHz, positions short of range
  0.3 -> 0.6 % (same short_tol tier), waveforms NONC x8/x16 -> COHE x12/x24. Probably A6 (2.765 GHz no longer
  evaluated in a quiet spectrum, not verified against V6.12): the frequency is not flat for this sector. To revisit.
- mission_sky_demo EARTH: 2.76 -> 2.695 GHz, energy -5.1 % (the two-pass search had missed it), range at worst scan
  1018 -> 1334 km; mission_redundancy_demo A1_TRIM range at worst scan +17.5 %.
- Model case: WF-LRS ranges +0.01 % (A1 / A2 order); fast estimate 0.01-0.1 %.
Case times (first N3 with the progress bar, each case once in its own process, at most 8 workers): model 0.2 min,
mission_example 0.9, _csv 0.6, veille500 0.7, redundancy_demo 1.8, synthetic_mix 1.0, sky_demo 1.3, target_library
1.2, environment 3.6, frames 2.7; cost table 1.9; test suite 2.7 (178 passed); whole N3 18.7 min (was 49 min of
cases + over an hour of tests).

## 2026-10-25 -- step 3.3 coded: active-tracking model (radar_model V6.14.0)

Causes: core/trackmodel.py (steady-state Singer Kalman per axis, requirements turned into measurement errors, floors,
Markov loss chain, initiation, discrimination); track looks pointed at the prediction (pointing loss law), Pd
tracking.pd_update, velocity / acceleration banks of the predicted errors, range-Doppler coupling residual; AT cost
averaged over 8 ranges of each bracket, single look or 7-beam sequential cluster, initiation and re-initiation
costs; attenuator; new outputs per bracket (period held / alert, predicted and filtered accuracies, pointing, loss,
initiation), discrimination per sector, false tracks, class separability. cost_table.json regenerated. 416 output
values differ. Test values only, not radar performance. Main moves:
- No S search output changes. Model case: TRACK looks now bank over the class acceleration (104 values).
- mission_synthetic_mix: Ku time of the faces x3 to x3.8 (AT brackets KV 0-250 km and 250-500 km: pointing loss 0.16
  and 0.40 HPBW, velocity banks, initiation), face energy +0.03 %; both periods held; loss per update 0.0014 / 0.0072.
- mission_target_library T_DECOY 200-600 km: Ku time +7 to +9 %, 32 initiation looks (unpowered target, low process
  noise, transverse velocity unknown at hand-over), period held.
- mission_sky_demo MOON 5-1000 km: F2 energy +0.12 %, period held.
- Separability of every class pair ~1 (class means without RCS spread: optimistic, renvoi 5.1).
Case times: model 0.2 min, mission_example 1.0, _csv 0.6, veille500 0.7, redundancy_demo 1.9, synthetic_mix 1.4,
sky_demo 1.4, target_library 1.3, environment 3.7, frames ~2.7; cost table 1.9; test suite 3.9 (190 passed); N3 21.0 min.

## 2026-10-25 -- step 3.4: array errors and in-flight calibration (radar_model V6.15.0)

Causes: the lumped 'calibration' loss (0.5 dB S, 0.8 dB Ku) is replaced by the Ruze loss of the residual array errors
at the end of a calibration interval, TX and RX (about 0.006 dB in both bands: random 1 deg / 0.1 dB per TRM,
0.5 deg / 0.05 dB per TRSA, drift 0.1 deg/min / 0.01 dB/min over 300 s); every face spends 3.3e-5 of its S time and
1.7e-4 of its Ku time on calibration (10 ms / 50 ms every 300 s); new outputs array_errors, calibration_*_time.
cost_table.json regenerated. 5225 output values differ. Test values only, not radar performance. Main moves:
- Model case: WF-LRS ranges +2.89 % (0.5 dB). S search energy and time of the faces -7 to -14 % in the missions;
  ranges at worst scan about +2.9 % where the waveform is unchanged.
- Ku time of the faces: the calibration (1.7e-4) now dominates the small confirmation / tracking Ku times of the test
  missions (x40 to x4000); idle faces show the calibration load only (binding resource: Ku time on air).
- mission_redundancy_demo: A1_TRIM, B1_TRIM, A2_VEILLE, B3_EDGE drop their longest pulse (the budget no longer needs
  it): range at worst scan -13 to -18 % (still above the requirement), energy down.
- mission_veille500 VEILLE_KV: positions short of range 0.6 -> 0.3 %, energy -10.7 %.
The self-test hand derivation now computes the Ruze loss from the file instead of the former 0.5 dB.
N3: 20.5 min, 190 passed; the snapshot tests and the self-test (hand derivation) failed before the fix and the
intended refreeze.

## 2026-10-25 -- step 4.1: electrical power (radar_model V6.16.0)

Causes: faces share the 8 MW ship budget (energy load = bus draw / cap); always-on consumption 106.9 kW per face
replaces overhead_frac 3 % (CHAIN_EFF 0.945, reference TX power 1.784 MW instead of 1.834); TRM supply check (68 Ku
looks removed, no S waveform); face storage checks and contiguous-transmission scheduler rule. 1361 output values
differ; no S waveform set changes. Face energy loads rise (always-on ~5.7 % of a 2 MW cap; idle faces now show it).
N3 22.2 min: 194 passed; snapshot tests and test_tracking_loads_add_up (calibration time, new energy load) failed
before the fix and the refreeze.

## 2026-10-25 -- step 4.2: thermal (radar_model V6.17.0)

Causes: new outputs only -- junction temperature of every search waveform (junction_c), face heat (transmissions +
always-on electronics + pumps) against the derived cooling capacity (~15.9 MW per face, load 'heat' in the face
verdict), heat burst against the cold-plate buffer, ship heat (faces + conversion losses) against the 6 MW rejection
reserved for the radar. The junction check is off (measured: Tj <= 63 C even CW, user decision), so no waveform is
removed. 746 output values differ (new keys); no key figure, waveform set or confirmation mode changes.
N3 20.7 min: 197 passed; only the snapshot tests failed before the refreeze.

## 2026-10-25 -- step 4.3: scheduler (radar_model V6.18.0)

Causes: new outputs only -- report key "scheduler" (task loads per face, band and priority, mean waits, shedding,
synthetic timeline, common transmit slots and their eclipse, confirmation latency per sector against its
requirement); tracking outputs per face (p_share, false alarms per face, tracks per face, initiation time). No key
figure, waveform set or confirmation mode changes (103 values differ). Test missions: common S slots 0.1 to 4.3 % of
the time (eclipse 0.004 to 0.19 dB), every confirmation latency held (1.4 ms to 0.11 s at 95 %), no shedding, no
late track update. The always-on power and thermal-inertia entries are now read at import (the "every parameter is
read" test had found them unread when their functions were not called).
N3 19.6 min: 200 passed; the snapshot tests and test_every_parameter_is_read_by_the_code failed before the fix and
the refreeze.

## 2026-10-25 -- step 4.3 bis: scan order within a volume (radar_model V6.19.0)

Causes: new outputs only -- per sector and face, key "scan" in the scheduler report (real pointings of the hexagonal
lattice in (u, v), frame transmit time, uniformity of the coverage at 10 / 25 / 50 % of the frame, mean step, pointing
rate) and "revisit_spread_s" per face timeline; the search releases of the synthetic timeline now follow the real
pointings, so the start delays move by a few ms (189 values differ). No key figure, waveform set or confirmation mode
changes. Test missions: real pointings 0 to +20 % over the plan's area estimate (edge effects of small volumes);
uniformity at 25 % of the frame 0.92 to 1.0 on the large volumes, down to 0.36 on small or ragged ones.
Bench fix: the 'model' case stored the internal look-space object of ku_look (its repr holds a memory address, so the
snapshot only matched through the case cache); it is now dropped from the case output.
N3 21.6 min: 203 passed; only the snapshot tests failed before the refreeze.

## 2026-10-25 -- step 4.4: compute and data (radar_model V6.20.0)

Causes: new outputs -- report key "compute" (per face and band: operations per function, transported bits,
simultaneous RX beams, buffers, capacity, power of the processing and the corrected draw / heat, processing latency
per sector, cost of an in-flight reselection); Ku looks of the tracking budget carry their numbers (ku_confirm_wf,
wf_params); the confirmation latency now holds the computed processing latency instead of the provisional 1 ms
(scheduler.processing_s removed), so every latency_q95_s moves; two missions get new warnings: S partition waveforms
(4 sub-beams over 13.7 positions) need 155 to 171 simultaneous RX beams against the 64 of the beamformer
(mission_synthetic_mix F3-F4, mission_redundancy_demo F1, F3, F4). 92 values differ; no key figure, waveform set or
confirmation mode changes.
N3 20.7 min: 206 passed; only the snapshot tests and test_compute_budget_invariants (reads the snapshots) failed
before the refreeze.
