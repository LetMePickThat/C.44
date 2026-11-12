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

**Headless exact run (script was in the session scratchpad, lost: rewrite it)**
```python
import sys, pickle
sys.path.insert(0, ".")                                 # run from app/
from sim import document as doc_m
from core import optimizer as mo

def main():
    d = doc_m.load(r"scenarios\MOONTEST_VELOCITY.json")
    mission, alerts = doc_m.mission_for_calc(d)        # navigation of the scenario at t_nav_s, as the GUI
    res = mo.run(mission, stage_cb=print)
    pickle.dump(res, open(sys.argv[1], "wb"))

if __name__ == "__main__":                              # mandatory on Windows: the optimiser uses a process pool
    main()                                               # (without it: BrokenProcessPool)
```
Per face in the result: res["compute"]["faces"][k]["memory"] (traffic_TB_per_s, bandwidth_TB_per_s),
res["power_allocation"]["faces"][k], res["scheduler"]["faces"][k]; or margin.utilisation(res).

**Working tree at the time**: V6.44.0 just need coded by another session, NOT committed then (committed since, with this note) (allocation, cost_estimate,
margin, montecarlo, optimizer, radar_model, scheduler, tracking, validation, results_tab, scene_view, sim_view, docs).
Repo rule: origin/main is the reference ([[feedback-repo-version-is-reference]]); commit or settle it first.

**Exact run V6.44.0 relaunched 12 Nov 2026 16:49** (headless, per face / per volume traffic, nominal and at the peak): still running at 17:15 (> 25 min); the PC was then moved on battery (power saving): its duration is not representative.
