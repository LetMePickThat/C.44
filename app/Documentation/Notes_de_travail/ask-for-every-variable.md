---
name: ask-for-every-variable
description: "For each roadmap step, ask the user the value/decision for EVERY variable involved; never pick values silently"
metadata:
  type: feedback
---

At each step of [[radar-roadmap]], ask the user explicitly for every variable the step introduces or touches
(value, source, status [V]/[D]/[?]), not only the ones flagged (D). Propose a default with its justification, but the
user decides; record locked values as [V] in the parameter file ([[global-parameter-file]]).

**Why:** user instruction 2026-10-20 ("reposer la question pour toutes les variables"); the model's values are system
design decisions owned by the user.
**How to apply:** before coding a step, list its variables (name, proposed value, unit, rationale) and wait for answers.
