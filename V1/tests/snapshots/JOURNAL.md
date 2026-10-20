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
