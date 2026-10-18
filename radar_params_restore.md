# SPACE AESA RADAR — FIXED PARAMETERS (restore baseline)
Date: 2026-10-18 (model WF_RANGE_CALC V6.0.10). Ref.: Radar Performance Model V5.6.0 (Ku), near-complete overhaul. Rule: optimistic 2050 extrapolation for all components; V5.6.0 values (PAE, T_sys 400 K, 8 dB losses, power chain) = already 2050 values.
Legend: [V] locked by the user; [D] derived/proposed, not locked; [?] conflict or to be confirmed.

## 1. Geometry
- [V] 4 faces; modelling on 1 isolated face
- [V] Antenna dimensions = V5.6.0 (same location, same ship)
- [V] 196 TRSAs/face, 16×16 mechanical grid; TRSA rows: 6, 8, 10, 12, 14, 16, 16, 16, 16, 16, 16, 14, 12, 10, 8, 6 (numbered 1–196, symmetric)
- [V] Tile 260 mm + 2 mm gap (mean tile pitch 262 mm); radiating area 13.25 m²/face
- [V] Dual-band stack: Ku in front, S behind; Ku→S blockage 0.2 dB

## 2. Power / cooling
- [V] Total electrical power, 4 faces: 8 MW, freely distributed (e.g. 4 MW on the threat face, 1 MW × 3 others)
- [V] Reference per face: 2 MW electrical → 1.375 MW average RF
- [V] Max cooling: 2 MW/face (simple check parameter)
- [V] Chain PAE: 75 % (replaces 65 % everywhere, including V5.6.0)
- [V] Power chain: PSU 95 % × distribution 99.5 % × (1 − 3 % overhead) = 0.917; 2 MW × 0.917 × 0.75 = 1.375 MW
- [V] Working assumption: radar not thermally limited; thermal averaging window undefined
- [V] Dynamic allocation: search may use 100 % of the power if there is nothing to track
- [V] Energy buffer at TRM level

## 3. S band (search only)
- [V] 5×5 S TRMs per TRSA; 4,900 S TRMs/face; pitch 52.4 mm (5×5 Ku cells per S element)
- [V] Band 2.7–2.9 GHz (200 MHz)
- [V] Max scan 75° up to 2.85 GHz; 70° from 2.85 to 2.9 GHz (the only anti-grating-lobe measure retained). Grating lobes: limit 2.91 GHz at 75°, 2.95 GHz at 70°
- [V] 2 kW peak/TRM → 9.8 MW peak/face; max duty cycle at full peak ≈ 14.0 %
- [V] Search sector per face: disc of radius 75° (az, el), truncated |az| ≤ 45° (90° sector); no duplication between faces; refresh 120 s
- [D] S beam ≈ 1.3° over 2.7–2.9 GHz; positions counted in direction cosines (u, v), 1/cos θ broadening included: ≈ 4,308 positions/face at 2.8 GHz (faces tilted 20° toward the bow, ship axis vertical, sector recomputed in the face frame: area 2.069) (spacing 1.04 HPBW); sector = disc of radius sin 75° cut by the PLANES az = ±45° between faces, i.e. |u| ≤ sin45·√(1−v²) (2u²+v²≤1) in (u, v), area 2.198 (V6.0.10; the old straight cut |u|≤sin45, area 2.463, spilled over in elevation: +11 % positions); the old angular count (4,425) ignored the broadening
- [V] S roles: long-range search (full sector, quasi-continuous by default), very long range (VLR, zone and % of time = operator parameters, no default allocation), short range cued on a detected threat, calibration (small continuous share)
- [V] Confirmation, promotion, tracking: Ku only

## 4. Ku band (confirmation, promotion, high-precision tracking)
- [V] 25×25 Ku TRMs per TRSA; 625/TRSA; 122,500 Ku TRMs/face; pitch 10.48 mm (cell 10.4 mm, 1.08 cm²)
- [V] 75 W peak/TRM (GaN-on-diamond microchannels, ≈ 69 W/cm²) → 9.19 MW peak/face; max duty cycle ≈ 15.0 % with 75 % PAE
- [V] Band 13.5–14.5 GHz (1 GHz) = 5 × S band; scan 75° up to 14.25 GHz, 70° above; grating-lobe limit 14.55 GHz (75°), 14.75 GHz (70°); η_illum Ku 0.88/0.88
- [D] Ku beam ≈ 5× narrower than S (≈ 25× more positions for the same sector)
- [V] V5.6.0 losses: 8 dB, T_sys 400 K, η_illum 0.88, η_ohm 0.96, scan loss cos^1.1 (0.6 dB at 70°), band-edge loss 1.5 dB, multibeam 0.8 dB at 8 beams (exact V5.6.0 definition unknown; in a digital array: receive = 0 dB since beams are formed digitally without a divider; transmit = term to keep, a function of the number of beams superimposed per TRM, linked to the peak-to-average ratio, PA back-off and PAE; avoidable with 1 beam/frequency or a subset of TRMs at a time; superposition vs partition not decided; energy is shared P/N between beams, handled by the scheduler)

## 5. RF model — S chain (values used)
- TX illumination 1.0 (uniform); RX illumination 0.88 (digital weighting)
- Pre-LNA loss 0.3 dB
- [V] T_sys S = 346.5 K (deep-space value; Ku 400 K)
- [V] S losses (dB): DSP 2.0; CFAR 0.5; calibration 0.5; pre-LNA 0.3; Ku blockage 0.2; TX line 0.3 (PA output → element, integrated in the TRM); polarisation 0.5 (dual-polarisation RX); buffer droop 5 J/TRM (0.24 dB on WF-LRS, rejected if the pulse uses > 50 % of the buffer); multibeam 0.8 @ 8 beams; migration 1.0 (compensated); band edge 1.5 max. Ku: 8 dB lumped
- [V] Lattice: spacing 1.04 HPBW (hexagonal mesh, worst-case optimum; 1.1 in V6.0.9) → ≈ 4,308 positions at 2.8 GHz (u,v count, sector bounded by the az ±45° planes), mean TX loss 1.81 dB (1.672·s²), worst point −4.3 dB (range ×0.78); digital receive steered onto the target; enabled by default
- Swerling 1; Pd = 0.9; Pfa = 1e-6 per range-Doppler cell; single-step detection
- [V] Standard Target: RCS 1000 m², SW1, closing speed 5000 m/s (replaces 1500 m²)
- [V] S waveform: abandoned, to be redefined; RF model = generic engine evaluating different waveforms

## 6. Threats / requirements
- [V] Reference threat: space KKV, real RCS 0.12–0.15 m²; ST 5000 m/s kept for migration/Doppler for now; KKV 25 km/s
- [V] KV closing speed = 25 km/s (15 km/s max LEO crossing + 7 km/s KKV + margin); warning time ≥ 30 s; DR = V·(30 s + RR), optimal RR ≈ 10 s → DR ≈ 1,000 km; setting: cislunar (The Lunar War), KKV 6–7 km/s at impact, Δv 6 km/s (10+ with a stage)
- [V] Assumption: KVs launched from a platform; short-range search = circular sector of radius 10° centred on the carrier
- [V] Side-on RCS of the target ship: 3000 m² (front and side views provided)
- [V] Question asked: S-band search at 10,000 km, revisit 120 s

## 7. Fully digital architecture (all bands, all modes)
- [V] Central computer → command table per TRM (frequency, pattern, timings); power allocation table per TRSA (derived from the TRM tables); TRSA coolant flow a function of that table
- [V] Power distributed at TRSA level then split per TRM; cooling managed at TRSA level
- [V] The TRSA never rejects a table; overall consistency guaranteed by the central computer alone
- [V] ADC per TRM on receive; hierarchical digital beamforming (TRM → TRSA → computer); raw data output on request only
- [V] Ku instantaneous bandwidth = 200 MHz (hardware ceiling; requires digital true time delay per TRM when scanning); S: undefined
- [D] Estimated raw data rate/face (16-bit I+Q, before TRSA beamforming): Ku ≈ 3.92 Tb/s per 10 MHz → ≈ 78 Tb/s at 200 MHz; S 200 MHz ≈ 31 Tb/s
- Face inputs: power, coolant, reference clock/sync, bidirectional optical links (incoming commands, outgoing data/telemetry/calibration)
- Local protections (T, I, U) deterministic, with priority over the tables [D]
- Preloaded tables + common sync pulse [D]

## 8. Scheduler
- [V] Permanent operation over the whole frequency range; band centre preferred; low frequencies at large scan angles; 70–75° zone searched with suitable frequencies (75° domain always fully scanned)
- [V] Energy budget per face and per time (not per waveform)
- Assumption: AI/optimisation-based scheduling over a deterministic core [D]

## 9. Open
- S and Ku waveforms: remain open (dynamic input, central computer, no fixed catalogue); frequency reserve zones (EW) to be defined
- Not modelled: antenna temperature by direction (Earth limb, Sun)
- Thermal averaging window; degraded behaviour on link loss; snapshot buffer depth
- Grid loss in the range computation; Ku↔S blockage/isolation (0.2 dB retained)

## Ship frame (confirmed 2026-10-18)
- [V] "Lighthouse" model: Z = longitudinal axis, +Z = forward (nose: faces + telescope), −Z = aft (thrusters, behind the radiators); X = radiator axis; Y = third transverse axis; right-handed frame
- [V] φ = azimuth about Z (from +X toward +Y); ε = elevation from the XY plane toward +Z (+90° = +Z)
- [V] 4 faces: normals at (φ, ε) = (0°, 20°), (90°, 20°), (180°, 20°), (270°, 20°); tilt 20° toward +Z (SECTOR_TILT_DEG, measured ≈ 22° on the drawing). Labelled F1–F4 in the tool
- [D] Responsibility sectors = minimum scan angle: boundaries φ = 45°, 135°, 225°, 315° (planes containing Z); scan ≤ 75° (70° above 2.85 GHz); ~22.2 % of the sphere per face; uncovered ~11 % (ε < −55°, −Z side)
- [D] Above 2.85 GHz (70°): coverage down to ε −50°, +Z reached at exactly 90°

## Mission inputs (mission preparation → computer)
- [V] Rear limit ε_min and front limit ε_max: set by the user during mission preparation, sent to the computer; independent of the face tilt (20°) and of the scan limit θ. Default −90° / +90° (no limit); physical floor ≈ −55° (θ 75°, tilt 20°)
- [D] Implementation: SECTOR_EPS_MIN_DEG / SECTOR_EPS_MAX_DEG in WF_RANGE_CALC V6.0.10 (sector area and beam-position count); e.g. ε_min = −20° → ≈ 3,600 positions/face (4,308 without limit); ε_max = 18° → ≈ 2,470
- [D] Interactive visualisation: "Radar face sectors" page (volumes.html) (sliders ε_min, ε_max, tilt, θ, spacing); only scanned volumes are drawn

## Pre-mission optimiser (V1.0, 2026-10-18)
mission_optimizer.py (uses WF_RANGE_CALC_V6.0.10.py); JSON/CSV input (mission_example.json/.csv), unlimited number of sectors:
id, shape box(phi_min,phi_max,eps_min,eps_max) or cap(phi0,eps0,radius), rcs_m2, swerling, range_km, rr_target_s, v_close_min/max_mps,
options res_max_m (default 2 m), allow_ambiguous, freq_ghz_candidates. global: eps_min/eps_max, face_elec_mw.
Output per sector: WF (f, coherent/non-coherent, tau, N, PRI, B, Btx), ranges, target/alone/mission RR, budget per face (energy %, average kW, time on air %), flags.
