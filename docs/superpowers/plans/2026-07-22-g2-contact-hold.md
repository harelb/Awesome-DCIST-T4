# G2 Contact Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unblock the G2 friction-hold grasp tier (P4 Task 14's honest stop) by giving the gripper PhysX colliders active only during the grasp window, per `docs/superpowers/specs/2026-07-21-g2-contact-hold-design.md`.

**Architecture:** Collider provisioning (disabled, high-friction material, self-collision-filtered) on exactly two gripper prims at spawn for `contact_hold` robots; a toggle riding the existing arm-ownership handoff in `PhysicsGraspBackend`; retune of Task 14's existing close/press/poll machinery; acceptance via `grasp_smoke.py --contact-hold` (pipe, 10 m carry, twice).

**Tech Stack:** Python (dcist_sim_isaac, USD/PhysX APIs deferred-imported), Isaac Sim 6.0.1, pytest (spark_env, no ROS).

## Global Constraints

- Branch `feature/isaac_sim_g2_contact` off `feature/isaac_sim_phase4`. dcist_sim-only; NO submodule changes. Push harelb only.
- G1 (`contact_hold: false`), magic tier, and kinematic tier byte-identical: every new behavior gated on physics mode + `spec.contact_hold`.
- `contact_hold` stays opt-in (spec §1); acceptance = `grasp_smoke.py --contact-hold` pick + 10 m carry + place of the PIPE, exit 0, TWICE, slip behavior reported. Honest-stop contract: ~2 GPU tuning sessions, then documented stop with measurements.
- Unit suites stay green (isaac 145 + new, ros 23). GPU rules identical to P4 (sourcing, EULA env, PYTHONPATH append, venvs, ≤600000 ms timeouts, teardown, nvidia-smi intruder check).
- Terminal-hygiene contract (P4-established): every terminal path — success, every failure phase, drop, exception, `reset()` — must leave arm ownership returned and (new) colliders in the correct state; reviewers will trace all of them.

---

### Task 1: Gripper collider provisioning (disabled, filtered, high-friction)

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/spot_robot.py` (provisioning helper, called from `__init__` for `contact_hold` robots)
- Test: `dcist_sim/dcist_sim_isaac/test/test_gripper_colliders.py` (pure prim-path/config logic only — USD calls are GPU-verified in Task 3)

**Interfaces:**
- Produces: `provision_gripper_colliders(stage, robot_prim_path) -> dict` returning `{"finger": <path>, "palm": <path>}` — the two collider prim paths; `set_gripper_colliders_enabled(stage, paths, enabled: bool)`. Task 2's backend consumes both via a narrow accessor on the robot (`robot.gripper_collider_paths`).
- Constants: `GRIPPER_FRICTION_STATIC = 1.2`, `GRIPPER_FRICTION_DYNAMIC = 1.1`, `PALM_RELATIVE_PATH` (pinned in Step 1).

- [ ] **Step 1: Pin the palm prim.** One-shot [GPU] prim dump (headless smoke boot is enough): print the subtree of `/World/<robot>/arm0_link_wr1` (names + whether each prim is a UsdGeom.Mesh). Pick the fixed-jaw mesh the finger closes against (Task 14's report describes the gripper chain; the jaw is the wr1-side mesh adjacent to `arm0_link_fngr`'s arc). Record the exact relative path as `PALM_RELATIVE_PATH` with a comment citing the dump. Include the dump output in your report.
- [ ] **Step 2: Failing unit tests** (pure parts: path construction + enable-state bookkeeping with a fake stage object exposing `GetPrimAtPath`; follow `test_grasp_backends.py`'s fake style):

```python
from dcist_sim_isaac.spot_robot import gripper_collider_paths_for

def test_collider_paths_are_finger_and_palm():
    paths = gripper_collider_paths_for("/World/hilbert")
    assert paths["finger"] == "/World/hilbert/arm0_link_fngr"
    assert paths["palm"].startswith("/World/hilbert/arm0_link_wr1")
```

(`gripper_collider_paths_for(prim_path) -> dict` is the pure path builder the provisioning uses.)

- [ ] **Step 3: Implement.** In `spot_robot.py` (deferred-import pattern):

```python
GRIPPER_FRICTION_STATIC = 1.2   # rubber-pad-like; convex-on-convex pinches
GRIPPER_FRICTION_DYNAMIC = 1.1  # shed objects without high friction
PALM_RELATIVE_PATH = "arm0_link_wr1/<pinned in Task 1 Step 1>"

def gripper_collider_paths_for(robot_prim_path):
    return {
        "finger": f"{robot_prim_path}/{GRIPPER_RELATIVE_PATH}",
        "palm": f"{robot_prim_path}/{PALM_RELATIVE_PATH}",
    }

def provision_gripper_colliders(robot_prim_path):
    """Colliders on the two gripper prims: convex hull, DISABLED, high
    friction, self-collision-filtered against the robot (spec §3). Physics
    mode + contact_hold robots only (caller gates)."""
    import omni.usd
    from pxr import UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    paths = gripper_collider_paths_for(robot_prim_path)
    # High-friction material shared by both prims.
    mat_path = f"{robot_prim_path}/gripper_phys_material"
    material = UsdShade.Material.Define(stage, mat_path)
    phys_mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    phys_mat.CreateStaticFrictionAttr(GRIPPER_FRICTION_STATIC)
    phys_mat.CreateDynamicFrictionAttr(GRIPPER_FRICTION_DYNAMIC)

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    for key, path in paths.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"gripper collider prim missing: {path}")
        col = UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        col.CreateCollisionEnabledAttr(False)          # starts disabled
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, materialPurpose="physics")
        # Belt-and-braces: never collide with the robot's own links.
        filtered = UsdPhysics.FilteredPairsAPI.Apply(prim)
        filtered.CreateFilteredPairsRel().AddTarget(robot_prim.GetPath())
    return paths

def set_gripper_colliders_enabled(paths, enabled):
    import omni.usd
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    for path in paths.values():
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath(path)).GetCollisionEnabledAttr().Set(bool(enabled))
```

(Verify exact API spellings against the installed pxr — `MaterialBindingAPI.Bind`'s physics-purpose signature and `FilteredPairsAPI` rel name; cite what you find. The filtered-pairs target = whole robot subtree; if per-prim targets are required by the schema, target the body + arm link prims explicitly.)

In `SpotSimRobot.__init__`: for physics-mode robots with `spec.contact_hold`, call `provision_gripper_colliders` after spawn and store `self.gripper_collider_paths`; else `self.gripper_collider_paths = None`.

- [ ] **Step 4: Unit tests pass; full suite green; commit** — `feat(dcist_sim): gripper collider provisioning (disabled, filtered, high-friction)`

---

### Task 2: Toggle wiring + CARRY re-verify in PhysicsGraspBackend

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_grasp_backends.py` (extend at the existing fake seam)

**Interfaces:**
- Consumes: `robot.gripper_collider_paths` + `set_gripper_colliders_enabled` (Task 1). The fakes replace the toggle with a recording stub (extend FakeArm or add a FakeColliders recorder — follow the file's pattern).
- Produces: collider-state contract for Task 3's GPU work — enabled exactly [arm-taken .. terminal], EXCEPT successful contact hold: stays enabled through carry, disabled after place/drop/reset.

- [ ] **Step 1: Failing tests** (fake seam; complete cases):

```python
# 1. contact_hold grasp: colliders enabled at accept (arm taken), still
#    enabled after _succeed_grasp (carry), disabled after place detach.
# 2. contact_hold grasp fails at CONTACT_CLOSE ("no contact"): colliders
#    disabled on the failure terminal.
# 3. drop during carry: colliders disabled when the drop monitor fires.
# 4. exception mid-phase (fake arm raises in deploy): colliders disabled
#    via the _fail/_finish funnel.
# 5. reset() with an in-flight contact op AND with a contact-held object:
#    colliders disabled in both.
# 6. G1 robot (contact_hold False): the collider toggle is NEVER called.
# 7. CARRY re-verify: fake reports gripper<->object distance 0.5m at the
#    would-be success tick -> grasp terminates "failed"/"dropped", never
#    reports succeeded (kills the one-tick optimistic-success window).
```

- [ ] **Step 2: Run — FAIL. Step 3: Implement.** The toggle lives beside the arm-ownership calls: where the op takes the arm (`set_arm_hold(False)`) → `set_gripper_colliders_enabled(paths, True)` if `op.contact_hold`; in `_finish` → disable UNLESS the op is a successful contact grasp (carry keeps them); in the drop monitor + place DETACH + `reset()` → disable. CARRY re-verify: before `_succeed_grasp`, check gripper↔object distance ≤ `CONTACT_DROP_DIST_M`; if not, route to the drop path. Keep G1/magic paths byte-identical (gate every new line on `op.contact_hold` / `held["mode"] == "contact"`).
- [ ] **Step 4: Tests pass (isaac suite full run); commit** — `feat(dcist_sim): gripper collider toggle on grasp window + CARRY re-verify`

---

### Task 3: [GPU] Gait sanity, retune, acceptance

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/grasp_smoke.py` (only if the pipe target needs a flag — check; Task 14 added `--contact-hold`, 15i added `--carry`)
- Modify: `docs/sim_runbook.md` (G2 section update: unblocked, tuning values, evidence)
- Possibly tune: `CONTACT_PRESS_M`, `CONTACT_POLL_S`, close speed in `grasp_backends.py`

- [ ] **Step 1: [GPU] Gait sanity.** field_smoke_physics variant with `contact_hold: true` (scratchpad copy): boot, colliders provisioned-but-disabled, command a 10 m walk — assert no falls and gait indistinguishable from baseline (nav_status + odom). This proves provisioning alone is inert.
- [ ] **Step 2: [GPU] Contact + hold tuning.** `grasp_smoke.py --contact-hold` targeting the PIPE (add `--target pipe_0` support if the script lacks target selection — small arg + selection plumb). Iterate press depth/close speed/poll window until pick reliably attaches by friction (contact reported, object rides without pin). Record every tuning iteration with numbers.
- [ ] **Step 3: [GPU] Acceptance.** Pick + 10 m carry + place of the pipe, exit 0, TWICE. Report slip events verbatim. Bag as a bonus data point (not gated). Honest stop if the pinch fundamentally sheds after ~2 sessions — wip commit + measurements + runbook note, per spec §1.
- [ ] **Step 4: Runbook update + commit(s)** — `feat(dcist_sim): G2 contact hold unblocked - gripper colliders + retune` + `docs(sim): G2 acceptance evidence`; push `git push harelb feature/isaac_sim_g2_contact`.

---

## Self-Review Notes

- Spec §3 provisioning (two prims, disabled start, friction material, filtered pairs) → Task 1; §4 toggle semantics incl. carry-keeps-colliders → Task 2; §5 retune + CARRY re-verify → Tasks 2-3; §7 gait sanity + unit seam → Tasks 1-3; §1 bar (pipe, 10 m, twice, opt-in) → Task 3.
- Names consistent: `gripper_collider_paths_for` / `provision_gripper_colliders` / `set_gripper_colliders_enabled` / `robot.gripper_collider_paths` across Tasks 1-2.
- Deliberate open item: `PALM_RELATIVE_PATH` pinned by Task 1 Step 1's prim dump (spec assigns this to the plan's dump step; it is a discovery, not a placeholder).
