# Jaw-Entry Grasp + Third-Person Video Capture — Design

**Date:** 2026-07-22
**Status:** Approved design, pre-implementation
**Predecessor:** `2026-07-21-g2-contact-hold-design.md` (honest stop #2: colliders + contact
detection work; press-from-above SHOVES light objects — cone fled 0.79 m, bag 0.63 m —
because the object never enters the finger–palm jaw; runbook §12.6/§12.6a).

## 1. Goals (user-approved)

1. **Third-person video capture** — a reusable sim capability, delivering to the user
   (in order): (a) a baseline locomotion clip (robot walking several goals — gait,
   turns), (b) baseline grasp-failure clips (`cone_0`, `bag_0` shove), all BEFORE any
   grasp changes; then (c) after-clips of the jaw pinch, same camera, comparable.
   Clips are sent to the user as mp4 as soon as each is captured.
2. **Jaw-entry grasp** for the `contact_hold` tier: approach places the object inside
   the finger–palm jaw window, then close to pinch. **Acceptance:** pick + 10 m carry
   + place, exit 0, TWICE, on the measured-fit target (cone or bag — chosen by
   measuring the jaw window vs both objects' cross-sections; the 1.2 m pipe stays
   out: unreachable as authored). Honest-stop contract (~2 GPU sessions on pinch
   physics). `contact_hold` stays opt-in; G1 remains the default tier.

## 2. Video capture

- `sim_app` flags: `--video-out DIR` (off by default), `--video-fps` (default 24).
- One extra static camera prim; pose computed from the scenario: look-at point =
  midpoint of robot spawn and objects centroid; camera offset ~3.5 m back, 2 m up.
- Replicator RGB annotator on that camera's render product — the same API family
  `gt_capture.py` already uses (P4 Task 15e proved annotators + physics survive on
  the field_smoke scenes; the SIGSEGV was warehouse+GT-live specific; this is
  RGB-only on field scenes).
- JPEG frames written at the requested sim-time rate; `ffmpeg` encodes `capture.mp4`
  at shutdown (implementation verifies ffmpeg availability; frames are kept on
  encode failure).
- Kinematic tier unaffected (flag-gated). Video must never kill the sim (same
  never-fatal contract as GT capture: exception → log, disable video, sim continues).

## 3. Jaw-entry grasp phases (contact_hold only; G1 byte-identical)

**Measure first:** one-shot GPU dump of finger + palm mesh AABBs at the open-gripper
pose → the **jaw window** (slot between palm plate and finger arc): height, depth,
mouth axis in the gripper frame. Decides the acceptance target (cone vs bag,
whichever cross-section fits with margin) and pins all constants from data.

Phases replacing press-from-above after VALIDATE:

1. **JAW_STAGE** — IK the OPEN gripper to a staging pose: jaw mouth at object
   mid-height, standoff behind the object along the approach axis. Reuses the P4
   align+standoff machinery for base placement (head-on, 0.70–0.90 m) — not rebuilt.
2. **JAW_ADVANCE** — servo the gripper forward along the jaw mouth axis until the
   object center lies inside the jaw window (pure geometry predicate: registry object
   pose vs gripper pose + measured window; unit-testable). **Shove detection:**
   object displaced beyond the window during advance → back off once, retry from
   JAW_STAGE; second shove → `failed` ("shoved").
3. **CLOSE_PINCH** — close `f1x` until finger↔object contact is reported AND the
   object remains inside the window (palm side of the pinch). Contact decides the
   close angle — no fixed target.
4. **LIFT_VERIFY** — raise the gripper; held ⟺ contact persists AND the object's
   displacement tracks the gripper's (delta tracking). Then the existing carry /
   drop-monitor / CARRY-re-verify machinery takes over unchanged.

Every phase timeboxed → `failed` with a phase-named message. All terminal hygiene
(arm ownership return, collider disable, command zeroing, `reset()`) rides the
existing reviewed funnels untouched. Collider enable point stays at the transition
into the first jaw phase (the G2 cycle's measured lesson: enable too early = ground
drag; too late = PhysX misses registration).

## 4. Branch

Continue `feature/isaac_sim_g2_contact` (@ 79729bc). dcist_sim-only; push harelb.

## 5. Testing

- **Unit (fake seam, existing patterns):** every phase transition; the jaw-window
  containment predicate (pure math, synthetic poses); shove-retry-then-fail;
  per-phase timeouts; terminal hygiene through the new phases (arm + colliders on
  every exit); G1/magic/kinematic byte-identical (gating).
- **GPU:** baseline videos (locomotion, cone shove, bag shove) delivered BEFORE
  grasp changes; jaw measurement dump; acceptance = §1.2 bar with after-videos.
- Suites stay green (isaac 159 + new, ros 23).

## 6. Risks

| Risk | Mitigation |
| --- | --- |
| Neither cone nor bag fits the jaw window | measured up front (first plan task) — if neither fits, STOP at that task with the numbers (asset decision goes to the user before any phase work) |
| Advance shoves the object anyway (friction with ground during entry) | shove-retry + slower advance speed as tuning levers; honest-stop contract |
| Replicator video + physics instability outside field scenes | video is field-scene-scoped for this effort; never-fatal contract isolates it |
| IK lateral authority limits at jaw-stage poses | staging reuses the head-on envelope (P4 §12.5); jaw mouth axis is aligned with the approach axis by construction |
