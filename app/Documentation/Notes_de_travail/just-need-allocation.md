---
name: just-need-allocation
description: "Confirmation/tracking at just need: new_targets_per_rr / n_targets = peak to absorb, never reserved (user rule 12 Nov 2026, coded V6.44.0, not tested)"
metadata:
  node_type: memory
  type: project
  originSessionId: 0dd2c2c5-8667-43a3-84f8-673aae7f5678
  modified: 2026-11-12T15:15:09.587Z
---

User rule 2026-11-12: the radar must not reserve confirmation/track resources in bulk for potential targets (new_targets_per_rr, n_targets) that may never come (e.g. meteorite cloud n = 1000).

Decisions (J1-J6, Choix_architecturaux § "Juste besoin"):
- by default only the real load consumes; the rest goes to the operator priority via realloc_mode, under the load ceiling, also in the Monte-Carlo;
- an influx takes back P5 (margin reuse) then P4 search below spec (flagged, down to stopping it); never P1 nor established tracks;
- new_targets_per_rr and n_targets = PEAK to absorb: verdict on absorbability, no reservation, no new parameter;
- the 26 Oct track-saturation rule merges into it (beyond the peak).

Coded 2026-11-12 (V6.44.0, NOT tested yet): tracking views nominal/peak (mission global "_targets"), optimizer.peak_absorption, allocation.peak_summary, verdict family "peak", saturation + PRIO_TRACK + MC at the peak, fast estimate nominal + peak.

**Why:** reserving for absent targets wastes resources and can make missions infeasible for large n.
**How to apply:** judge over the flight plan ([[project-rmp-availability]]); allocation rule [[project-final-allocation-rule]].
