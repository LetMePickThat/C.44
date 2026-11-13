---
name: no-embedded-target
description: "The radar tool simulates a fictional ship that will never exist: no embedded / flight / certification target, ever"
metadata:
  node_type: memory
  type: project
  originSessionId: 119ec3bb-5b32-440f-b9e4-cfabe13590b0
  modified: 2026-11-12T20:41:02.765Z
---

The whole project is a simulation for a ship that does not exist and is not meant to exist (user, 12 Nov 2026). Nothing
has to be planned for embedded / on-board execution: no certification, no flight computer port, no "same code on the
ground and on board", no MISRA / DO-178C / ECSS arguments.

**Why:** the user corrected a C++ vs Rust analysis that weighed embedded certification.
**How to apply:** judge performance work (see [[performance-risks]]) only by the tool's own needs: speed of the exact
calculation for the operator (live exact instead of the fast estimate, time profile) and the radar in the loop of the
scenario simulator (6.4), RAM on the main PC, development cost while the model still changes. Language choices: numba
first; a native kernel only if measured hot spots need it, picked for build ease and safety, not certifiability.

DECISION (user, 12 Nov 2026): finish the model itself in Python; consider a switch to another language (C++ / Rust)
only once the maths are locked for good. Do not propose a port before that; speed work until then stays in Python
(algorithmic gains, numba) and never changes results.
