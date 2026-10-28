---
name: group-by-topic
description: "Everything linked to one subject must be handled in the same place (same step, same question); dependent values are settled after the values they depend on"
metadata:
  type: feedback
---

Everything related to a given subject must be treated in the same place: one step / one question block, one model,
one parameter section -- not scattered across steps or questions. When a value depends on another (e.g. tau_max
derived from the TRM energy buffer), do not ask the user to validate it before the value it depends on.

**Why:** user, 2026-10-20 (step 1.3, Q6/Q7): "D'une manière générale, faire en sorte que tout ce qui est lié à un
sujet donné soit traité au même endroit" and "ne pas les valider tout de suite ... les valeurs dépendent de la
réserve d'énergie de Q7".

**How to apply:** when preparing a step's questions, group them by subject and order them by dependency; when a
topic straddles two roadmap steps, propose to move the whole topic to one step (say which) instead of splitting it.
Links: [[ask-for-every-variable]], [[explain-before-coding]].
