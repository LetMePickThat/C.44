---
name: estimate-vs-exact
description: "Fast estimate OK vs exact run NOK on MOONTEST_VELOCITY t=0 s (memory bandwidth 119-149 %): comparison started 12 Nov 2026, exact run cancelled by the user, to resume"
metadata:
  node_type: memory
  type: project
  modified: 2026-11-12T16:30:00.000Z
---

**Asked by the user (12 Nov 2026, afternoon)**: compare the fast estimate and the exact calculation (two screenshots,
MOONTEST_VELOCITY.json, t = 0 s, Calculate: exact). The headless exact run was CANCELLED by the user before it ended:
no exact figures extracted yet. Resume from "To do" below.

**Screens (code before V6.44.0 just need, see [[just-need-allocation]])**

| face | estimate E | estimate S | estimate Ku | exact verdict |
|---|---|---|---|---|
| F1 | 1.68 MW (+1611 kW) | 12 % | 0 % | memory bandwidth 119.3 % NOK |
| F2 | 1.60 MW (+1533) | 12 % | 0 % | memory bandwidth 149.0 % NOK (limiting resource) |
| F3 | 1.60 MW (+1534) | 12 % | 0 % | memory bandwidth 148.9 % NOK |
| F4 | 1.51 MW (+1439) | 11 % | 0 % | NOK (value hidden on the screen) |

Estimate: OK on every face. Exact: 4 NOK, 0 degraded, 108 OK (load ceiling 80 %), PRIO_SEARCH over-subscribed.
Saturation advice: S01 rr_target 30 -> 102 s, S02 30 -> 164 s (F2), 163 s (F3, F4).
The Results-tab live bars show the badge "ESTIMATE" (not "recalibrated"): they are the estimate, not the exact result.

**Findings so far (code read, not yet measured)**
1. The fast estimate prices only energy, S time and Ku time (the live bars). Memory bandwidth (and memory, compute,
   heat, beams) exist only in the exact run: core/compute.py (look_cost: mem_bits = writes of the received samples +
   one read per dwell; flag at line ~289), margin.utilisation "memory bandwidth" = traffic / memory_bw_tbps
   (50 TB/s per face, radar_system.toml). So an estimate "OK" says nothing on the memory: structural blind spot.
2. The NOK is NOT caused by the PRIO_SEARCH reuse: margin.py R2 already counts memory bandwidth among the reuse
   constraints and the verdict is on the revisit asked (R5); the advice (RR 30 -> 102/164 s) shows ~150 % at the
   asked 30 s already.
3. The calibration of the estimate by the exact run (core/.calibration_cache.json, per volume, e and t only) shows
   large per-volume gaps on energy / S time too: factors x4.2-4.6, x0.85, x0.36, x1.5-2.7 (4 volumes, order 1-4,
   presumably SP, S01, S02, S03: to confirm). Even without the memory, the raw estimate is far from the exact run.
4. Labels of the estimate: worst RR 2.71 s for S01/S02/S03 (PRIO_SEARCH served revisit), target 30 s.

**To do (ask the user before running: exact run = several minutes)**
- Re-run the exact calculation, now with V6.44.0 (just need: confirmation / tracks no longer reserved, may change the
  traffic), and extract per face: energy, S, Ku, memory, memory traffic TB/s, and per volume its share of the traffic
  (look_cost x rate). Then compare with the estimate face by face and volume by volume.
- Explain the 4x calibration factor on the first volume (which one, why the table under-prices it).
- Proposal to make to the user (not decided): a memory-bandwidth bar in the fast estimate, traffic per pointing
  taken from the cost table like the energy (proportional to the pointing rate), or at least an alert when the
  estimate cannot judge the memory.

**Headless exact run: `python tools/exact_compare.py [scenario.json] [result.pkl]`** (run from app/; defaults
scenarios/MOONTEST_VELOCITY.json and tools/.exact_compare.pkl). Mission built as the GUI does
(sim.document.mission_for_calc); step counter printed live and written to <result>.progress.txt; at the end the
verdict, the peak test and, nominal then at the peak, per face every resource's utilisation and the memory traffic
against the bandwidth with each item's share (search / confirmation / tracks per volume); result pickled.
Per face in the result: res["compute"]["faces"][k]["memory"] (traffic_TB_per_s, bandwidth_TB_per_s),
res["power_allocation"]["faces"][k], res["scheduler"]["faces"][k]; or margin.utilisation(res); the peak's budgets
under res["peak"]["budgets"] (optimizer.peak_out).

**Working tree at the time**: V6.44.0 just need coded by another session, NOT committed then (committed since, with this note) (allocation, cost_estimate,
margin, montecarlo, optimizer, radar_model, scheduler, tracking, validation, results_tab, scene_view, sim_view, docs).
Repo rule: origin/main is the reference ([[feedback-repo-version-is-reference]]); commit or settle it first.

**EXACT RUN DONE, main PC, 12 Nov 2026 ~20:25 (V6.44.0, 39.2 min, `--workers 2 --cap-gb 7`)**: result in
tools/.exact_compare.pkl (not tracked, local exclude), log tools/.exact_compare.log. Verdict NOK 4 / OK 112, limiting F2 memory
bandwidth 149.0 %; the same figures as before V6.44.0 (just need changed nothing here: no target in the nominal view,
confirmation traffic 0; peak ABSORBED, n_targets 0, peak = nominal).

| face | need MW (always-on) | S time | Ku time | mem. traffic TB/s (of 50) | traffic: search SP / S03 / S01 / S02 |
|---|---|---|---|---|---|
| F1 | 0.326 (0.074) | 1.6 % | 0.3 % | 59.7 (119.3 %) | 65 / 30 / 4 / 1 % |
| F2 | 0.340 (0.076) | 1.7 % | 0.3 % | 74.5 (149.0 %) | 57 / 40 / 3 / 1 % |
| F3 | 0.339 (0.076) | 1.7 % | 0.3 % | 74.5 (148.9 %) | 57 / 40 / 3 / 1 % |
| F4 | 0.339 (0.076) | 1.7 % | 0.3 % | 74.5 (148.9 %) | 57 / 40 / 3 / 1 % |

Other resources all low (S beams 22-27 %, compute 6-10 %, memory 4 %). PRIO_SEARCH reuse blocked (g = 1.0 for every
volume, "F1 memory bandwidth already at 119 %"): revisits stay at 30 s. No volume fully relieved (shares 46 / 45 / 9 %).
Estimate (screens, older code) vs exact: energy 1.5-1.7 MW vs 0.33-0.34 MW (x ~5), S time 11-12 % vs 1.6-1.7 % (x ~7).
HYPOTHESIS, not verified: the estimate prices the PRIO_SEARCH served revisit (labels: worst RR 2.71 s, i.e. ~11x the
30 s rate) that the exact run refuses because of the memory bandwidth. To check next: the estimate now (V6.44.0) on the
same scenario, and which revisit it uses. RAM of this run: one process up to 7 GB (cap reached by the first SP solve and
by the worker phases), PC free down to 19.6 GB (the cap holds per process, not the PC total: trimmed pages stay in RAM).

**Measure A done (tools/estimate_compare.py, 12 Nov 2026 evening, 10 min; log tools/.estimate_compare.log)**:
hypothesis CONFIRMED. The window's estimate (PRIO_SEARCH, no calibration in the cache for these volumes) reproduces the
screens exactly (F1 1.682 MW / 12.4 % S) with every revisit served at 2.71 s; the exact run keeps 30 s (reuse g = 1,
memory bandwidth). At the revisit asked (realloc NONE):

| per face | estimate raw | estimate recalibrated | exact |
|---|---|---|---|
| energy MW | 0.20-0.22 | 0.28-0.29 | 0.33-0.34 |
| S time | 1.0-1.1 % | 1.6 % | 1.65-1.74 % |
| Ku time | 0 | 0 | 0.25 % (confirmation of false alarms, not in the estimate) |

Per volume, exact / estimate at the same request (calibration.measure, not recorded): SP x4.2-4.6, S03 x1.6-2.7,
S01 x0.85, S02 x0.36 (energy; S time alike): totals partly compensate, the volumes are far off. The cost table is NOT
current (is_current False: radar_system.toml changed since tools/gen_cost_table.py). Even recalibrated, PRIO_SEARCH
gives 4.13 s and 1.6 MW: the estimate cannot see what stops the reuse in the exact run.
Three causes, to treat in order (user to decide): 1) the reuse limited only by energy / S / Ku time in the estimate;
2) per-volume pricing errors (SP x4.5 the worst; stale table?); 3) no confirmation load in the estimate.

**a done (12 Nov 2026 night)**: cost table regenerated (tools/gen_cost_table.py, 3.8 min, step counter added); the
measure gives the SAME figures: the live estimate prices the volumes with their cost profiles (core/cost_profile), the
table is only the fallback while a profile is computed. The stale table was not a cause.

**b diagnosis (per volume, exact / estimate at the same request, scratchpad scripts on the pickle)**:

| volume | positions | energy per position | total | exact choice |
|---|---|---|---|---|
| SP (800 km) | 1.05 | 4.25 | 4.48 | spacing 1.3, relief 9 %, sky +4.15 dB, Moon clutter 59 dB |
| S01 (15000) | 0.62 | 1.36 | 0.85 | spacing 1.8 |
| S02 (8000) | 0.30 | 1.21 | 0.36 | spacing 2.0, relief 46 % |
| S03 (4000) | 0.26 | 6.86 | 1.77 | spacing 2.0, relief 45 %, sky +4.15 dB, Moon clutter: 32 x 50 us coherent |

Positions: fully explained by the lattice spacing the optimiser picks among beam_spacing_options_hpbw (the profile
assumes beam_spacing_hpbw 1.4: factor (1.4 / sp)^2) and by the relief. CONFIRMED BUG (code read, optimizer.py
2769-2779): the exact result keeps the RELIEVED plans, calibration.measure compares them with the estimate of the
WHOLE volume: the ratio carries the relief, which the live estimate then applies again. Energy per position: the
profile leaves the sky aside (T_sys rise 4.15 dB = x2.6 for SP and S03, the Moon in the main lobe) and the surface
clutter (the optimiser then takes multi-pulse coherent waveforms); the wider spacings cost more per position (S01,
S02 x1.2-1.4).

**V6.45.0 (b1-b4, c, d) measured, 12 Nov 2026 ~22:00 (T1, tools/estimate_compare.py on the V6.44.0 exact pickle)**:
bug found by the first T1 and fixed (frame_m is stored flat: the geometry comparison of the instant profile raised,
S01-S03 were left out of the estimate). Window estimate (PRIO_SEARCH, no calibration) against the exact run:

| per face | before (V6.44) | V6.45 | exact |
|---|---|---|---|
| energy MW | 1.51-1.68 | 0.30-0.35 | 0.33-0.34 |
| S time | 11-12 % | 1.46-1.82 % | 1.65-1.74 % |
| Ku time | 0 | 0.26 % | 0.25 % |
| revisits | 2.71 s | 30 s | 30 s |
| memory bandwidth | not seen | 121 % (F1-F4) | 119 / 149 / 149 / 149 % |
| state | OK | NOT_TENABLE | NOK |

Faces within -12 % / +6 % (F4 the lowest: the face split differs a little). Per volume, exact / estimate: SP 0.91-1.00,
S01 0.97-1.05; S02 0.46-0.48 and S03 0.44-0.76 still carry the relief (the pickle predates b1, no relief code):
T3 (exact run again, then the measure) needed to check b1 and the relieved volumes.

**T3 done (12 Nov 2026 22:50, exact run V6.45.0 51.8 min `--workers 2 --cap-gb 7`, PC free min 15.6 GB)**: exact
result unchanged (NOK 4 / OK 112, F2 memory bandwidth 149.0 %). b1 CHECKED: exact / estimate at the same request SP
1.00-1.10, S01 0.97-1.05, S02 0.97-1.00 (was 0.47), S03 0.79-1.36 (the volume most cut by the Moon clutter, 11.6 % of
its directions: its face split remains off). Per face, raw estimate -12 % / +6 % of the exact energy, recalibrated
(calibration measured on this run) -5 % / +6 % on the energy, -8 % / +8 % on the S time, Ku time +4 %; revisits 30 s
and NOT_TENABLE (memory bandwidth 121 %, exact 119-149 %) in every variant. Left: S03's face split, the memory
bandwidth face split (F2-F4 121 % against 149 %).

**Exact run V6.44.0 relaunched 12 Nov 2026 16:49 on the laptop**: crashed after ~30 min with BrokenProcessPool
(a worker killed while the PC went on battery, power saving; in the second pass started by the relief, S frequency
evaluation of a re-planned volume: code V6.44.0 does not touch it; cause not verified). No figure. TO DO: run
tools/exact_compare.py on the MAIN PC (more memory), on mains power, after asking the user (expect 30 min or more).
