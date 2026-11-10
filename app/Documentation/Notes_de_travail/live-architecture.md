---
name: live-architecture
description: "Live scenario editing (fork RADARv6_RS, merged as the app 8 Nov 2026, aa86f67): what was planned (a KSP-like computation front), what was finally merged (an approximate live preview + the real computation on demand), and which live pieces exist unused"
metadata:
  type: project
---

MERGED 8 Nov 2026: the fork RADARv6_RS replaced the app (commit aa86f67 on main; rollback tag avant-RS-2026-11-08 on
8837d94, on GitHub). Do not push from RADARv6_RS.

## What the app does (code at c11b0a2, checked 10 Nov 2026)
The design changed late on 8 Nov 2026 (user, confirmed 10 Nov 2026): NO computation front in the UI. Instead:
- While editing: the scenario's PREVIEW (sim/preview.py, compiled in sim/fast_preview.py) recomputed at once at each
  change, node drag included (gui/sim_view: preview_timer, _PreviewWorker, DRAG_FLEET_HZ 10). Approximate: same forces
  as sim/propagate but RK4 at two rates (free rows PREVIEW_STEP_S 15 s, finer steps for burns / PN / closing rows),
  no Singer, attitudes at the free steps, other bodies only above TIDE_MIN_KM. Status bar: "preview (approximate): ...".
- Once validated: "Record run..." (SimView.run_journal) = the REAL computation (python -m sim.run, full model, separate
  process, progress bar "real computation: x %"), journal .npz written next to the scenario (the radar's entry point),
  then the run shown. The camera is kept after an edit (show_run, keep).

## Pieces of the planned live architecture that exist
- L1 sim/propagate.LiveSim: the full propagation (propagate() = LiveSim(sc).run().truth()); bit-identical to the
  former code on the 3 scenarios. Used by sim.run.
- L2 checkpoints every sim.checkpoint_s = 10 s (LiveSim.checkpoint / checkpoint_before / restore). Exist, unused by the UI.
- L3 sim/live.first_change + resume (Singer / group / RCS / tumble draws seeded by row name; PN by range edit -> 0;
  duration: longer goes on, shorter cut) and live.affected_rows. Exist, used only by tests/test_sim.py.
- NOT in the code (never committed): L4 sim/live.Front, L5 front-driven view (tool.display.sim_front_refresh_hz,
  DRAG_SENDS_FRONT, _base / _ahead_of), the clock held at the front. L6 (journal from the history) and L7 (clean-up)
  of the former plan are therefore moot as stated; decision B (front computing only the touched rows) moot too.

## Former plan (decisions of 8 Nov 2026, kept for reference; superseded by the preview design above)
A1 going back in time by replaying the history; A2 an edit in the past re-simulated from the last checkpoint before it;
A3 the front stops at the end date; B1-B4 every body projected up to the end date with the full model, refreshed 2x/s;
C1 no "on rails" bodies; C2 time-warp list kept, the clock following the front; D1 the radar left out for now (in the
end the radar model must run inside the live simulator, not from a recorded run); D2 the .npz journal kept.
Measured speeds (full propagation, desktop, after C2-C3): moon 37 bodies ~x1300, Jupiter 310 ~x820, TEST_MAX_LOAD 1011 ~x320.

## Open
- Unused L2 / L3 code: KEPT for now (user, 10 Nov 2026; may serve a later live radar, D1). Do not remove it unasked.
- Preview vs real run gap: not quantified on screen by the user (preview measured < 1 m off at 30 s steps after 1 h
  in low orbit; 15 s now).

**Why:** the user wants scenario edits without waiting for a full propagation (camera resets, stale boxes).
**How to apply:** describe the app as preview + Record run; do not tell the user to look for a front. Explain each step
and ask its values first ([[explain-before-coding]], [[ask-for-every-variable]]), ask before tests ([[ask-before-tests]]).
