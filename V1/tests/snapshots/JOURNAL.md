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
