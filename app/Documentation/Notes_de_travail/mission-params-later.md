---
name: mission-params-later
description: "Operational mission parameters (target RR, ranges...) belong to the operator's mission prep (Pmiss); don't tune or recommend them while building the simulation tool"
metadata:
  type: feedback
---

The revisit targets and other operational parameters of the test missions (e.g. 20 s RR on the anti-KKV sectors)
come from an old model. The operator's mission preparation ("Pmiss") is where target RRs are set; standard
mission parameters will be discussed at that phase, not while building the simulation tool.

**Why:** user, 2026-10-21 (step 2.3), after I proposed advising a 3-5 s RR for anti-KKV sectors.
**How to apply:** when a test-mission result looks bad because of its mission parameters, explain the physics
briefly but do not propose renvois or changes to mission values; keep the focus on the tool and the radar model.
Links: [[test-values-not-reference]], [[radar-roadmap]].
