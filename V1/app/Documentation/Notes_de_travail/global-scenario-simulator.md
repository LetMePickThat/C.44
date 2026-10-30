---
name: global-scenario-simulator
description: "The radar sim will be one part of a future global scenario simulator that feeds it object positions and the ship attitude; sky and volumes in the MISSION frame since step 3.0"
metadata:
  type: project
---

User, 2026-10-21 (step 1.5, Q2): a global scenario simulator will exist later; the radar sim we build is only one
part of it. The scenario sim gives the radar sim the position of every object (targets, planets, stars, etc.). The
radar sim works in the SHIP frame; the global sim uses another frame (possibly centred on the local gravity source)
and does the frame transforms. The radar model must account for effects of a body in front of the faces (e.g. thermal
noise rise) but must NOT compute where objects are (no ephemerides, orbits, attitude in the radar model).
An old prototype exists in RADARv5/SIMULATOR/10-BLEEDING EDGE: proof of concept only, ~1 year behind on RF; take
NOTHING from it (no values, models, nor interface format) -- user, 2026-10-21. See [[radarv5-obsolete]].

UPDATE 2026-10-22 (step 3.0, user Q3/Q4/Q5): mission volumes AND the sky (bodies, galactic frame) are given in a
MISSION frame (inertial / local orbital / two directions); the sim supplies the ship ATTITUDE relative to that frame and
the RADAR applies the rotation to the ship frame (rotation only, still no ephemerides / orbits in the radar). Regions
not centred on the ship (parallax) stay the sim's job. This supersedes "works in the SHIP frame" above for the inputs (the computation stays in the ship frame).

ARCHITECTURE 2026-10-22 (user D1-D6, detail in Feuille_de_route.md step 3.0): TRUTH vs KNOWLEDGE. 4 layers:
1 global sim (truth, Newtonian propagation, object attitude / RCS-aspect / manoeuvres); 2 converter (SIM side): truth ->
ship frame for the RF sim + a NAVIGATION MESSAGE (attitude, own orbit, bodies) for the computer; 3 radar COMPUTER (ship
side, knowledge only: Pmiss, nav message, its own detections; converts volumes to the ship frame = steering, scheduler,
waveforms, tracks); 4 RF sim (SIM side: plays the computer's orders against the truth, returns detections). The computer
NEVER reads target truth. Today's tool = layer 3 in mission prep on a nav snapshot. Volume frames: ship, inertial,
RTN, LIMB (horizon discovery band relative to the limb, follows altitude), two directions; track-relative -> 3.3.
User does NOT want a separate roadmap entry for the per-object detection mode (it is 4.3 / 5.4).

RUNTIME NOTE (user, 2026-10-25): outside Pmiss, the global scenario sim will probably need SIMPLIFIED (less accurate)
approaches to keep real-time / near-real-time computation. The "never trade accuracy for speed" rule was later
removed altogether (2026-10-25, [[ask-before-tests]]); simplified models to define later (4.3, 4.4, 5.4) and calibrate on the
full model. Recorded in Choix_architecturaux 1.3 and Feuille_de_route 5.4.

**Why:** separation of concerns decided by the user.
**How to apply:** radar-model inputs about the environment = object lists in the ship frame (direction, apparent size /
distance, type); in tests / missions they are given explicitly. Links: [[radar-roadmap]], [[global-parameter-file]].
