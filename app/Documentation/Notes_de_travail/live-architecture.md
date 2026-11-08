---
name: live-architecture
description: "Live (real-time, KSP-like) simulator, started in the RADARv6_RS fork and now the app (merged 8 Nov 2026, aa86f67): move the whole simulator to a real-time (KSP-like) architecture, no precomputed propagation between edits; decisions of 8 Nov 2026"
metadata:
  type: project
---

MERGED 8 Nov 2026: the fork RADARv6_RS replaced the app (commit aa86f67 on main, pulled into RADARv6; rollback tag
avant-RS-2026-11-08 on 8837d94, on GitHub). Work resumes in RADARv6; do not push from RADARv6_RS.
Started as the fork RADARv6_RS (copy of RADARv6 at commit 8837d94, 8 Nov 2026): the simulator moves to a live architecture
(user, 8 Nov 2026). Goal: no propagation to wait for between edits; initial states and manoeuvre series projected
continuously, up to the mission's end date. Decisions (user, 8 Nov 2026):

- A1 going back in time: yes, by replaying the recorded history.
- A2 an edit in the past: re-simulate (from the last checkpoint before the earliest instant the edit touches).
- A3 end date: optional in principle; the computation front STOPS at the end date (no projection beyond it).
- B1 every body is projected. B2 horizon: up to the mission's end date. B3 the projection is the full model (whole
  fleet, PN, Singer, interactions); to revisit later. B4 projections shown while computing refreshed 2x per second.
- C1 NO "on rails" bodies (the third-body loss is sensible in lunar orbit): every body always integrated with the
  full model (RK4 0.1 s, J2, other bodies, PN, Singer); the clock is held at a speed the real-time computation can
  follow. C2 the time-warp list kept (x1 .. x100000); above what the machine computes, the clock follows the front.
  Measured on the desktop (full propagation, loading included): moon 37 bodies ~x1000, Jupiter 310 ~x370,
  TEST_MAX_LOAD 1011 ~x100; after C2-C3 (8 Nov 2026): ~x1300, ~x820, ~x320.
- Design agreed: the projection IS the same deterministic simulation computed ahead of the clock (a computation
  front in the background toward the end date); the clock never passes the front; an edit restarts the front from
  the last checkpoint before the instant it touches; checkpoints = full fleet state (r, v, attitudes, PN, Singer
  RNG) every 10 s of simulated time (user).
- D1 the radar is left out for now; in the end the radar model must run inside the live simulator, not from a
  recorded run. D2 the .npz journal kept for now (fed by the history).

Progress: L1 DONE 8 Nov 2026 (sim/propagate.LiveSim; bit-identical to RADARv6 on the 3 scenarios; test_sim 61
passed, test_window_closes_during_the_playback crashed its worker once, passes alone). L2 DONE 8 Nov 2026 (checkpoints every
sim.checkpoint_s = 10 s, restore(ck, source); bit-identical restores on the 3 scenarios, numba and not). L3 DONE 8 Nov 2026
(sim/live.first_change + resume; Singer / group draws seeded by row name, 1b; PN by range edit -> 0, 2b; duration:
longer goes on, shorter cut (my proposal, user silent); 14 edits bit-identical to full runs). Open: geometry RCS /
tumble draws still seeded by row rank. L4 DONE 8 Nov 2026 (sim/live.Front;
bursts of edits bit-identical). test_sim 66 passed after L1-L3. Geometry RCS / tumble draws by name: done (user yes).
L5 DONE 8 Nov 2026 (L5a geometry by the front, L5b a, L5c, L5d; tool.display.sim_front_refresh_hz = 2; test_sim 66
passed after adapting 4 tests). Known regression: renaming an object restarts the front from its entry. Not seen by
the user on screen yet. Next: L6 journal from the history, L7 clean-up.
Also 8 Nov 2026 (after L5): objects drawn at their size (BODY_SHAPES), scenario editor fixes (targets by name,
rename_refs, vector boxes in the handles' colours, red Remove); see Choix_architecturaux.
Node during a drag (changed 8 Nov 2026, user): instant one-body preview only, the front told at the release
(gui/sim_view.DRAG_SENDS_FRONT). Live edits (8 Nov 2026): sim/live.affected_rows; the view draws the untouched rows'
planned paths beyond the front from the last complete run (_base, _ahead_of). B (the front computing only the touched
rows) pending the user's decision.

**Why:** the current app must re-propagate the whole scenario after each edit (waits, camera resets, stale boxes).
**How to apply:** work in RADARv6; one step at a time with its guard (see Choix_architecturaux, 8 Nov 2026 entries);
explain each step and ask its values first ([[explain-before-coding]], [[ask-for-every-variable]]), ask before tests
([[ask-before-tests]]).
