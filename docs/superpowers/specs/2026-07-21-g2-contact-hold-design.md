# G2 Contact Hold — Design

**Date:** 2026-07-21
**Status:** Approved design, pre-implementation
**Predecessor:** `2026-07-20-isaac-sim-phase4-physics-design.md` §6.2; P4 Task 14
(honest stop: machinery complete behind `contact_hold`, blocked because the
arm/finger links carry no PhysX colliders — nothing to detect contact with).

## 1. Goal (user-approved)

Unblock the G2 tier: friction-based holding where the object stays fully
dynamic and can genuinely slip and drop. **Acceptance bar:**
`grasp_smoke.py --contact-hold` on `field_smoke_physics` — pick + **10 m
carry + place** of a pinchable object, exit 0, **twice**; measured slip
behavior reported. `contact_hold` **stays a per-robot opt-in**; G1
validated-attach remains the default physics grasp tier. Honest-stop contract
applies (≈2 GPU tuning sessions), but unlike Task 14 the physical
prerequisite now exists.

## 2. Decision

Gripper-only colliders, active only during the grasp window (chosen over
always-on full-arm colliders, which risk re-destabilizing the walking policy —
the arm folds against the body in stow, recreating the kinematic-collider
wrestling-match bug class P4 eliminated).

## 3. Collider provisioning

At spawn (physics mode, robots with `contact_hold: true` only), apply
`UsdPhysics.CollisionAPI` + convex-hull approximation to exactly two link
subtrees of the Spot articulation:
- `arm0_link_fngr` (moving finger), and
- the fixed jaw/palm geometry it closes against (palm side of
  `arm0_link_wr1`; the implementation plan pins the exact prim from a one-shot
  prim-tree dump).

Both colliders start **disabled** (`collisionEnabled = false`). Both prims get
a **high-friction physics material** (static/dynamic ≈ 1.2 — rubber-pad-like;
convex-on-convex pinches shed objects without it) and a
`UsdPhysics.FilteredPairsAPI` entry excluding the robot's own body/arm links —
belt-and-braces so an accidentally-still-enabled collider can only ever touch
world objects, never wrestle the robot.

## 4. The toggle (rides the arm-ownership handoff)

- `PhysicsGraspBackend` takes the arm (existing `set_arm_hold(False)` window)
  → enable gripper colliders.
- Every terminal path that re-stows the arm (`_finish`) → disable them,
  **except** a successful contact hold: colliders stay enabled through the
  carry (they ARE the hold) and disable after place/drop/reset.
- Enable happens only inside the grasp window post-deploy (arm away from
  body); the filtered-pairs guard covers the failure case anyway.
- Same terminal-hygiene pattern that survived P4's review cycles: every
  terminal path accounted for, including exceptions, drop, and `reset()`.

## 5. Hold mechanics and tuning

Task 14's machinery is reused as-is where possible: close `arm0_f1x` toward
0 rad with position targets; poll PhysX contact between finger and target
(API quirks solved: 2-tuple `get_contact_report` on 6.0.1, contact-report
threshold attr). Retuned against measurement: `CONTACT_PRESS_M`, poll window,
close speed. Target object: **pipe first** (cylinder pinches naturally
between finger and palm), bag second; the cone remains documented as
un-pinchable with a single finger.

Carry: object fully dynamic; existing drop monitor (gripper↔object > 0.3 m →
`failed`, "dropped") stays. Fold in Task 14's deferred CARRY fix: re-verify
gripper↔object distance **before** declaring the grasp succeeded (kills the
one-tick optimistic-success window).

## 6. Branch

`feature/isaac_sim_g2_contact` off `feature/isaac_sim_phase4`. dcist_sim-only;
no submodule work. Push harelb only.

## 7. Testing

- **Unit (fake seam, existing patterns in `test_grasp_backends.py`):**
  collider-toggle state machine — enabled exactly during grasp window + carry;
  disabled on every terminal path including drop, place, exception, `reset()`;
  filtered-pairs/material applied once at provisioning; G1 and magic tiers
  byte-identical (all new behavior gated on `contact_hold`); kinematic tier
  untouched by construction (colliders only exist in physics mode).
- **GPU:** `grasp_smoke.py --contact-hold` = the acceptance bar (§1); a short
  walking sanity check with colliders provisioned-but-disabled (stow window)
  confirming zero effect on gait.

## 8. Risks

| Risk | Mitigation |
| --- | --- |
| Pinch sheds the object (friction/geometry) | high-friction material + pipe-first target + press/speed retune; honest stop with measurements after ~2 GPU sessions |
| Collider toggle leaks (enabled while stowed) | filtered-pairs guard makes the worst case "touches world objects only"; unit tests over every terminal path |
| Walking regression from provisioning | colliders disabled outside grasp window + gait sanity check in acceptance |
