# PROJECT CONTEXT — Space AESA radar, "The Lunar War"
Date: 2026-10-18. User: radar expert (French-speaking, prefers concise answers, in French). This file summarises the whole discussion and the parameters; the locked parameter details are in `radar_params_restore.md` (read it in full, it is authoritative) and in the header of `WF_RANGE_CALC_V6.0.10.py`.

## 1. Project and roles
- Setting: near-future cislunar ("The Lunar War"), NISO-class ships, KKVs (kinetic kill vehicles) at up to ~25 km/s closing speed.
- Space AESA radar, **fully digital**, 4 faces (arrays), S band (search) and Ku band (confirmation / promotion / tracking). Overhaul of "Radar Performance Model V5.6.0". Rule: optimistic 2050 extrapolation.
- **User's role**: operator. **Role of the tool**: non-real-time scheduler / pre-mission optimiser. The operator provides requirements per watch sector; the tool designs the waveforms and the energy allocation and flags infeasibilities.
- Current simplified mode: **a single face is modelled** (4,900 S TRMs, 2 MW electrical → 1.375 MW average RF), then extrapolated to the 4 faces.
- Marking convention: [V] locked by the user, [D] derived/decided, not locked, [?] to be confirmed.

## 2. Key radar parameters (summary; see radar_params_restore.md for everything)
- Power: 8 MW electrical in total, 2 MW/face as reference; chain PSU 0.95 × distribution 0.995 × (1−3 %) = 0.917; PAE 0.75 → average RF per face 1.375 MW. Energy buffer 5 J/TRM.
- S: 2.7–2.9 GHz, 4,900 TRMs × 2 kW = 9.8 MW peak/face, max duty ≈ 14 %, max scan 75° (70° above 2.85 GHz), illumination TX 1.0 / RX 0.88, T_sys 346.5 K, losses: DSP 2.0, CFAR 0.5, calibration 0.5, pre-LNA 0.3, Ku blockage 0.2, TX line 0.3, polarisation 0.5, buffer droop (pulse-dependent), multibeam 0.8 dB @8, migration 1.0, band edge ≤1.5, grid (spacing). S roles: SEARCH/TWS only.
- Ku: 13.5–14.5 GHz, 122,500 TRMs × 75 W = 9.19 MW peak/face, max instantaneous bandwidth 200 MHz (digital true time delay required), T_sys 400 K, 8 dB lumped losses. Roles: CONFIRM/PROMOTE/TRACK.
- Radiating area 13.25 m²/face, square envelope 4.19 m. S beam ≈ 1.30° at 2.8 GHz.
- Detection: Pd 0.9, Pfa 1e-6, Swerling 1 by default, single-step detection, coherent or non-coherent integration (free waveforms, dynamic input).
- Standard target ST: 1,000 m², 5,000 m/s; KKV: 0.135 m² (0.12–0.15 real, **to be confirmed in S band**), 25 km/s; SHIP: 3,000 m² side-on.
- Beam-position lattice: hexagonal mesh in direction cosines (u,v), **spacing 1.04 HPBW [V]** (worst case; mean loss 1.81 dB, worst point 4.3 dB).
- Search refresh 120 s; warning time ≥ 30 s; DR = V·(T_warn+RR), optimal RR = T_warn/3 (≈10 s for KKV → ~1,000 km).
- Energy: R ∝ E^(1/4).

## 3. Geometry (confirmed by the user)
- "Lighthouse" model: Z = longitudinal axis, **+Z = forward (nose: faces + telescope)**, −Z = aft (thrusters, behind the radiators); X = radiator axis; Y = third axis; φ = azimuth about Z from +X toward +Y; ε = elevation from the XY plane toward +Z.
- 4 faces at φ = 0/90/180/270°, **tilted 20° toward +Z**: normals at (φ,ε) = (0,20), (90,20), (180,20), (270,20). Faces at nose level. Labelled F1–F4 in the tool.
- Responsibility sector of each face = zone of minimum scan angle: boundaries = the planes φ = 45°, 135°, 225°, 315° (containing Z). In the face frame: disc of radius sin θmax ∩ |u| ≤ w·cosT − v·sinT ∩ ε window.
- Face frame: d = u·e_u + v·e_v + w·n; n = (cosT cosφk, cosT sinφk, sinT); e_u = (−sinφk, cosφk, 0); e_v = (−sinT cosφk, −sinT sinφk, cosT); ε = asin(v cosT + w sinT).
- Results: 4,308 positions/face at 2.8 GHz (17,233 in total), sector area 2.0685; full cone without sharing 6,105/face → 29.4 % saving (7,186 positions). Each face ≈ 22.2 % of the sphere, total coverage 88.9 %, uncovered 11.1 % (ε < −55°, −Z side). Overlap between neighbouring faces grows with ε.
- Mission zone inputs: **ε_min (rear limit) and ε_max (front limit)**, adjustable during mission preparation, independent of the tilt and of θ (variables SECTOR_EPS_MIN_DEG / SECTOR_EPS_MAX_DEG).
- Not modelled: shadowing by hull / radiators / telescope; antenna temperature by direction.

## 4. Discussion history (condensed timeline)
1. Overlap ratio between beams, detection holes → spacing set to 1.04 HPBW to close the holes at the worst points.
2. The user spots a geometry error ("diabolo" shape in angle space: the straight cut |u| ≤ sin45 spills past the az=±45° plane in elevation, +11 % positions) → sector corrected with the φ=±45° planes (2u²+v²≤1).
3. 360° coverage maps in az/el, then faces tilted 20° forward; repeated az/el frame confusions → the user annotates a sketch, "lighthouse" X/Y/Z frame, φ/ε defined.
4. Computation of the radar-time saving without overlap (4,308 vs 6,105 positions/face).
5. φ,ε diagrams of the responsibility zones and beam positions; 3D view with the ship at the centre; interactive version (`volumes.html` "Radar face sectors", sliders for tilt, θ, spacing, ε_min, ε_max; faces mounted at the nose; unscanned zones not shown).
6. Mission inputs: ε_min/ε_max; then **multiple watch sectors** (target RCS, angular coordinates, target RR, expected speeds, required range, unlimited number) and optimiser output (suggested WF per sector, estimated RR, radar budget).
7. `mission_optimizer.py` written and tested (JSON/CSV). Latest example results: S01 (box φ315→45, ε0→60, 1000 m², 5,000 km, RR 20 s) 31 % face energy; S02 (KKV, 15° cap around (90°,20°), 500 km, RR 5 s) 11 %; S03 (360°, ε≥−40°, 3,000 m², 8,000 km, RR 120 s) 38 % per face.
8. Request: a **tool with a real-time 3D window** for mission input, mission-preparation validation, then cost calculation and a **second 3D results window** (time on air, RR, range on target RCS…). The user asked for a **local Python tool** rather than an HTML page.
9. V1 tool built in `V1/` (launcher, mission-preparation window, background optimiser worker, results window). See `README.txt`. Later: the launcher was merged into `main.py` (single entry point, `launcher.py` removed); per-face / per-sector show-hide in both 3D views (`view_controls.py`).
10. Performance: 99 % of the optimiser's time was spent in `sector_area_uv()` (20,000-step Python loop called by every `solve()`), now cached → exact calculation for the 3-sector example ≈ 1 s (was ≈ 3.5 min), cost table generation ≈ 1 s (was 5–6 min, table bit-identical). Optimiser results identical to `mission_report.json` within 1e-15 relative: that reference was produced on another machine (Linux), the last-digit float differences predate the cache. GPU not relevant at the current search-space size.
11. All tool modules and documentation translated into English (GUI, messages, identifiers; `pm_window.py` → `mission_prep_window.py`; faces D1–D4 → F1–F4; result CSV headers renamed). The user still prefers to converse in French.
12. Review fixes: results window gets an energy colour scale (green ≤ 50 %, amber → 100 %, red beyond) with two energy metrics (sector alone / face total load) and a PARTIAL banner after a cancel (the prep window asks before opening a partial result); validation now rejects v_close_max ≤ 0 (was a ZeroDivisionError in the optimiser), zero-width azimuth spans and eps_min = eps_max; `geometry.py` now shares mission_optimizer's single copy of the radar model. For future scaling: per-process memoised solve() ranges, sectors optimised in a persistent pool of worker processes (processes, not threads: GIL) from 6 sectors up, overlap check restricted to bounding-box intersections. 24 sectors: 9.1 s sequential → 1.4 s with a warm pool (16 workers); results identical to the sequential run.
13. Optimiser v2 (from `V1/AUDIT/SPEC_optimiseur_v2.md`, with audit corrections): one waveform per 5° scan-angle bin (≤ 4 per sector, merged by dynamic programming), objective min max(energy, time on air), ranking = extra unscanned fraction caused by the frequency, then positions short of range (1 % tolerance), then the objective; RR dilation uses max(ΣE, ΣT); tracking margin per face (power left for Ku, Ku duty). Audit corrections to the spec: power scale kept per 1° bin (the spec's per-10°-bin worst-angle scale inflated energy: VEILLE_KV 75 % → 57 % once fixed), acceptance criterion "v2 not worse than v1 on (short %, max(E,T))" instead of "energy ≤ v1" (self-contradictory), and R(θ) = R(0)·h(f, θ) (h independent of the waveform, checked at run time) so each candidate costs one solve() per frequency. Results on `mission_veille500.json` (reconstructed, original not supplied): VEILLE_KV 62 %/102 % (v1) → 57 %/13 % (energy/time), cone 7 % → 0.4 % of positions short of range, tracking margin 532 kW/face; example S03 66 % → 31 %. Fast estimate fixed (8,000 km requests looked unreachable; unreachable cells were costed at Ppk × 1 s): denser range grid + per-cell reach; now within −1/+4 % of the exact v2.

## 5. Files (in V1/)
- `WF_RANGE_CALC_V6.0.10.py`: radar model (solve(), Waveform, Target, BandConfig, n_search_positions, sector_area_uv, losses…). Selftest: `python WF_RANGE_CALC_V6.0.10.py --selftest` (all tests pass).
- `mission_optimizer.py`: pre-mission optimiser (see §6).
- `mission_example.json` / `.csv`: example input format (made-up values for testing).
- `mission_report.json`: example output.
- `gen_cost_table.py`: generator of the precomputed cost table used for fast estimation while editing.
- Tool modules: see `README.txt`.
- At the root: `radar_params_restore.md` (locked parameters, reference); `volumes.html` (viz3d): reference for the 3D rendering (unit sphere, Z vertical, sectors coloured by face, boundaries, ship, axes, normals, mouse orbit) in three.js r128, reused as the visual reference.

## 6. Pre-mission optimiser (v1 description; superseded by v2, see §4.13 and the mission_optimizer.py header)
- Input: `global` {eps_min_deg, eps_max_deg, face_elec_mw} + `sectors[]`: id, shape = box (phi_min_deg, phi_max_deg with wrap-around through 0 allowed, eps_min_deg, eps_max_deg) or cap (phi0_deg, eps0_deg, radius_deg), rcs_m2, swerling, range_km, rr_target_s, v_close_min_mps, v_close_max_mps; options res_max_m (default 2 m → 75 MHz), allow_ambiguous, freq_ghz_candidates.
- Method: 0.2° sphere raster; owner of each direction = responsible face; positions = Σ w·dΩ/cell(f) per face and per 1° scan-angle bin; waveform search (f ∈ {2.7; 2.8; 2.9}, coherent/non-coherent, Btx, bandwidth, N, τ, unambiguous PRI ≥ 2R/c) minimising energy, with power reduced position by position ("no excess energy") to just reach the required range; migration evaluated at v_max.
- Output per sector: WF (f, integration, τ, N, PRI, B, Btx, dwell), ranges (worst/best scan angle, % of positions short of range), target RR / RR alone / RR in mission (stretched proportionally if the face is over-subscribed: RR×max(1, Σ energy fractions)), budget per face (positions, energy %, average kW, time on air %), detection→impact delay at v_max, flags (range not reached, missing kW, part of the sector not scanned); totals per face; warning on overlapping sectors (budgeted twice).
- Defaults: cost = energy (Ppk/Pavg ≈ 2.1, average power is the binding resource); time on air reported separately. Computation time: ≈ 1 s for 3 sectors (after the sector-area cache, see §4.10); parallel from 6 sectors (see §4.12).

## 7. SPECIFICATION of the tool (user's request)
"Provide a Python tool in which the user can define the mission inputs, with a real-time (RT) display of the watch volumes in a 3D window (with the desired radar time, RR, range on target RCS, etc.), and validate the mission preparation before running the cost calculation. After the optimisation, show its result in a second 3D window, in terms of radar time, RR, range on target RCS, etc."
Implementation proposals (implemented in V1):
1. **Window 1 (mission preparation)**: form (list of sectors: add/delete/duplicate, the fields above + global ε_min/ε_max, JSON/CSV import/export compatible with `mission_optimizer.py`) + real-time 3D view (PyVista+PyQt) with the same rendering as volumes.html (unit sphere, ship, X/Y/Z axes, faces, φ=45°… boundaries, semi-transparent coloured watch sectors, per-sector labels: range, RCS, target RR, speeds, estimated radar time). Fast cost estimate while editing (precomputed table interpolated over R, RCS, v, θ); per-face gauges (energy %, time on air %); alerts (overlap, outside scanned zone, range > capability). **Validate mission prep** button (consistency checks: unique ids, bounds, φ/ε, min speed ≤ max speed, RR and range > 0…).
2. **Calculation**: call `mission_optimizer.run()` (separate thread, progress bar, cancellable), result saved as JSON.
3. **Window 2 (results)**: same 3D view but coloured by result (e.g. achieved RR vs target, energy %), labels: suggested WF, range reached (worst/best scan angle), RR alone / in mission, radar time (time on air %), energy and kW per face, infeasibility flags (missing kW); summary table and per-face totals; JSON/CSV export of the result.
4. Recommendations: reuse the modules without rewriting them; do not duplicate the geometry (replicate the §3 formulas exactly); non-regression tests (4,308 positions/face, 17,233 total; example S01/S02/S03); interface in English.

## 8. Open points
- Map of approach speeds by direction and T_warn (to be provided by the operator); frequency reserve zones (EW) not implemented; KKV RCS in S band; T_sys(direction); hull/radiator/telescope shadowing; S/Ku waveforms still open (dynamic input); `make_geometry.py` not updated for the tilt; Swerling > 1 not handled in the precomputed table; dwell interleaving (time on air 66–81 % for a full sector) to be validated; instantaneous-bandwidth requirements (TTD required even in S band at 75 MHz at large scan angles).
