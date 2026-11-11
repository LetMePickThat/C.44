---
name: ask-before-tests
description: "Always ask before running any test; full battery (N3) at most once a day or every two days, on the user's call; model work comes first"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 4cab080b-3d2d-433f-9bf7-8468a15bf664
  modified: 2026-11-11T19:52:19.114Z
---

Before running ANY test or test-like computation (pytest at any level, tests/run_tests.py, the model self-test,
impact report, update_snapshots, gen_cost_table, timing/profiling runs of missions), say what would run, why and its
expected duration, and wait for the user's OK. The full battery (N3: suite + snapshots + impact + cost table) runs AT
MOST once a day, or every two days, when the user asks. Between two N3: nothing by default; propose at most an N1 or a
few targeted tests with their duration. More than 30 min of tests is unacceptable.

**Why:** user, 2026-10-23 evening: hours spent running tests and timings with no progress on the model ("Plus de 30mn
pour exécuter les tests est inacceptable... Sinon, on n'avance pas").
Clarified the same evening: Pmiss runtime is NOT the variable the model is designed around (the
operator computes one or two missions a day at most); the trouble with long tests is running them all the time.
UPDATE 2026-10-25 (user): the absolute-precision rule is REMOVED (tests too long and too heavy: a N3 filled 60 GB of
RAM). Approximations are now allowed if they do not denature the model: propose each with its effect on results
(measured or estimated) and its time gain; the user validates.
UPDATE 2026-11-12 (user: "On ne peut pas perdre des heures entières de temps de développement à cause des tests"):
ALWAYS check the expected duration first (tests/.test_durations.json, the last N3's per-item times in
tests/.n3_progress.txt, or the previous run of the same command) and STATE IT in every request to run anything. Also
say what else it forces: tests/testbench.code_fingerprint hashes core/*.py, so ANY edit in core/ invalidates every
cached case and the next impact report / update_snapshots recomputes all 11 cases (~70 min) -- do not touch core/
between computing the cases and the refreeze; gui/ edits are free.
Look up EACH selected test in tests/.test_durations.json before running a "-k" selection: e.g.
test_inertial_frame_with_identity_attitude_is_the_ship_frame alone takes ~23 min (1380 s), next to 0 s frame tests.
**How to apply:** keep moving on the model (roadmap steps); when a step changes values, note that snapshots are stale
and leave the N3 to the user's next slot. Never chain long runs. Protocol in Choix_architecturaux 1.3; see
[[session-workflow]], [[radar-roadmap]].
