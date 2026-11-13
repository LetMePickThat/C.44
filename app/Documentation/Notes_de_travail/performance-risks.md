---
name: performance-risks
description: "risk that the final model is too heavy for a current PC: measured figures, ranked risks (6.4 radar in the loop, Pmiss time profile, Monte-Carlo, memory, R3 re-plans) and their planned remedies"
metadata:
  type: project
---

Assessment asked by the user on 10 Nov 2026 (machine: 16 logical CPUs). Keep it in mind when framing 5.5-5.7 and phase 6.

**Measured, under control (low risk):** scenario propagation (moon 1.1 s, Jupiter 310 bodies 2.6 s, 1011 bodies
1.4 s); 3D playback 0.10 s per instant (light playback), 1.2 s per full rebuild; fast estimate ~0.2 s per instant, cost
profile 3-5 s per volume once; exact run of one mission at one instant 1.5-9 min (N3 cases 56-540 s, 8 workers).

**Risks, most serious first:**
1. **6.4 radar in the loop: HIGH if naive.** ~4 faces x ~1000 positions per revisit + tracks = hundreds to thousands of
   looks per scenario second, millions per hour of scenario; the full RF model per look in Python (~1 ms cached) =
   tens of minutes to hours per run, plus cache memory. Remedy (planned): 6.5 fast models = per-waveform / per-angle
   tables interpolated and numba-compiled, done BEFORE or WITH 6.4, not after.
2. **Pmiss time profile (option A, parked): MEDIUM-HIGH.** One exact run (1-8 min) per flagged instant = hours for a few
   tens of instants. Remedies: the fast estimate (now ~20 % with the cost profiles) everywhere, the exact run only at
   flagged instants, a cap on exact runs (open question 3), a run cache on disk (C5).
3. **Monte-Carlo (5.4): MEDIUM.** Already heavy, not profiled yet; acceptable because on demand only.
4. **Memory: MEDIUM.** An N3 once filled 60 GB (absolute-precision rule, removed); optimiser range caches ~50 MB per
   process x 8 workers; 6.4 would multiply the caches if unbounded.
5. **R3 confirmation re-plans: LOW-MEDIUM.** Up to 4 re-plannings per degraded volume (1.5 -> ~3 min on the test case);
   a mission with many degraded volumes could double or triple its run time.

**Exact result at every instant, stored (quantified 13 Nov 2026, user: future implementation where each instant is computed exactly and possibly kept in the file):** measured on MOONTEST_VELOCITY (4 volumes, Moon, t = 0): one exact result 261 kB JSON (sectors 84, peak 55, verdict 38, tracking 29 kB) / 35 kB gzip; + the 7 252 search dwell durations for the Timeline 79 kB / 17 kB gzip; total ~340 kB raw, ~52 kB gzip per instant. Instants over 6000 s with the profile rule (2 deg turn, at least every 60 s): ~120 in lunar orbit (period ~9 200 s, ~235 deg in 6000 s), 150-300 with attitude manoeuvres; 1 per second = 6000. File: 120 instants ~41 MB raw / ~6 MB gzip, 300 ~100 / ~16 MB, 6000 ~2 GB / ~310 MB. Duration of a mission does not matter today (one exact result kept, one instant). Remedies: a compressed side file (as the simulator .npz journal) indexed by instant, only the summary verdict per instant in the work file; dedupe what does not change between instants (to measure). TIME dominates the size: 39-52 min per exact instant on MOONTEST (V6.44) = 80-100 h for 120 instants: plan reuse between neighbouring instants is decisive (see risk 2).

**Proposal made to the user (not decided yet):** set a target duration per use (e.g. Pmiss < 1 min interactive, a
1-hour scenario with the radar in the loop < 10 min) to steer these choices. See [[radar-roadmap]],
[[next-session-reminders]].
