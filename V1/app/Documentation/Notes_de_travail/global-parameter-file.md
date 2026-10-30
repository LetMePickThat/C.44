---
name: global-parameter-file
description: "Tool architecture must let system-architecture variables be edited quickly, ideally from one global parameter file"
metadata:
  type: project
---

Architecture requirement (user, 2026-10-20): the tool(s) must make it fast to change any radar-system variable when
the system architecture changes — likely one global, human-editable parameter file (values, units, [V]/[D]/[?] tag,
source) that every module reads; no constants buried in code (radar_model, optimizer, tracking, cost_estimate,
geometry, GUI). Derived artefacts (core/cost_table.json via tools/gen_cost_table.py, core/antenna_cache.json rebuilt automatically) must be regenerable from it.

**Why:** the radar architecture still evolves; values keep changing (e.g. tilt 20 -> 25 deg, Pfa 1e-6 -> 1e-8).
**How to apply:** design step 0.1 of [[radar-roadmap]] around this; every later step adds its variables to that file.
