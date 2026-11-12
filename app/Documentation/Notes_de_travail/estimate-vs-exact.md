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

**Exact run V6.44.0 relaunched 12 Nov 2026 16:49 on the laptop**: crashed after ~30 min with BrokenProcessPool
(a worker killed while the PC went on battery, power saving; in the second pass started by the relief, S frequency
evaluation of a re-planned volume: code V6.44.0 does not touch it; cause not verified). No figure. TO DO: run
tools/exact_compare.py on the MAIN PC (more memory), on mains power, after asking the user (expect 30 min or more).
