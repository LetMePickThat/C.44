---
name: test-values-not-reference
description: "Test missions and frozen test values are NOT reference/acceptance values of the radar — valid for regression tests only, never for real use"
metadata:
  type: project
---

The radar's capabilities are not established yet, so there is no "reference mission" and no "reference value".
Mission files used by the test bench (mission_example, mission_veille500, mission_redundancy_demo, synthetic ones)
are TEST missions; the numbers frozen from them (snapshots, ex-SPEC §6 acceptance values) only detect unintended
changes of the code. They must never be presented or used as achieved/required radar performance.

**Why:** user correction 2026-10-20 (step 0.2 of [[radar-roadmap]]).
**How to apply:** say "missions de test" / "valeurs figées de test", not "référence"; name files and functions
accordingly (tests/missions/, snapshots, not golden/reference); flag it in docs and test headers.
