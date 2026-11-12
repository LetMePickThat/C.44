---
name: step-counter-in-runs
description: "Every test / headless run script prints a step counter (step n, elapsed, stage) live, flushed, and to a progress file"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 0dd2c2c5-8667-43a3-84f8-673aae7f5678
  modified: 2026-11-12T17:15:16.828Z
---

Every test or headless computation script written from now on shows a step counter while it runs: step number, elapsed time, the stage name (optimizer stage_cb, cases, items), printed with flush and also written to a progress file next to its output; never hide the stages behind grep / tail (the output then only appears at the end).

**Why:** user, 12 Nov 2026: an exact MOONTEST_VELOCITY run of 30 min showed nothing until it crashed (BrokenProcessPool on battery); "Systématiquement ajouter un compteur d'étapes dans les tests à partir de maintenant".
**How to apply:** use or extend app/tools/exact_compare.py's counter; for long runs prefer the main PC (more memory) — commit the script and the doc so it runs there ([[feedback-repo-version-is-reference]]); ask before running ([[ask-before-tests]] in the repo notes).
