"""Pure-python unit coverage for Task 1's gripper-collider path builder and
`__init__` gate (G2 contact-hold, spec Sec3). Isaac/pxr-touching provisioning
(`provision_gripper_colliders`, `set_gripper_colliders_enabled`) is GPU-
verified only (task-1-report.md) -- same convention as
`test_spot_robot_recovery.py` covering `_terminal_recovery_reason` alone.

Fix round (review): the finger collider path moved from the bare
`GRIPPER_RELATIVE_PATH` link to its `visuals` Mesh child
(`GRIPPER_MESH_RELATIVE_PATH`) -- `PhysicsMeshCollisionAPI` can only be
applied to a `UsdGeomMesh` (usdPhysics schema.usda), matching `stage.py`'s
`_make_dynamic` `IsA(UsdGeom.Mesh)` convention and `PALM_RELATIVE_PATH`'s
own mesh-child choice. See spot_robot.py's module comment above
`GRIPPER_MESH_RELATIVE_PATH` for the GPU re-verification (AABB sweep) and
the contact-report reconciliation (actor vs. collider fields).
"""
from dcist_sim_isaac.spot_robot import (
    GRIPPER_MESH_RELATIVE_PATH, PALM_RELATIVE_PATH, _wants_gripper_colliders,
    gripper_collider_paths_for)


def test_collider_paths_are_finger_and_palm():
    paths = gripper_collider_paths_for("/World/hilbert")
    assert paths["finger"] == "/World/hilbert/arm0_link_fngr/visuals"
    assert paths["palm"].startswith("/World/hilbert/arm0_link_wr1")


def test_collider_paths_use_the_pinned_relative_path_constants():
    """Ties the path builder to the module constants directly, so a future
    edit to either constant (e.g. a re-pin of PALM_RELATIVE_PATH) can't
    silently desync from `gripper_collider_paths_for` without this test
    catching it."""
    paths = gripper_collider_paths_for("/World/hilbert")
    assert paths["finger"] == f"/World/hilbert/{GRIPPER_MESH_RELATIVE_PATH}"
    assert paths["palm"] == f"/World/hilbert/{PALM_RELATIVE_PATH}"


def test_collider_paths_are_per_robot():
    """Bookkeeping sanity: a fresh dict keyed off whatever robot prim path is
    passed in -- no shared/cached state leaking between robots."""
    paths_a = gripper_collider_paths_for("/World/hilbert")
    paths_b = gripper_collider_paths_for("/World/turing")
    assert paths_a["finger"] != paths_b["finger"]
    assert paths_a["palm"] != paths_b["palm"]
    assert paths_b["finger"] == "/World/turing/arm0_link_fngr/visuals"


def test_wants_gripper_colliders_requires_physics_mode_and_contact_hold():
    # physics mode (kinematic=False) + contact_hold=True -> provision
    assert _wants_gripper_colliders(kinematic=False, contact_hold=True) is True


def test_wants_gripper_colliders_false_for_kinematic_robot():
    # kinematic-tier robots never run PhysX contacts, even if a scenario
    # somehow set contact_hold (scenario.py itself already forbids this
    # combination, but the gate is defense-in-depth).
    assert _wants_gripper_colliders(kinematic=True, contact_hold=True) is False


def test_wants_gripper_colliders_false_without_contact_hold():
    # physics-mode G1 robots (grasping: physics, no contact_hold) are
    # unaffected -- every pre-Task-1 scenario must stay bit-for-bit.
    assert _wants_gripper_colliders(kinematic=False, contact_hold=False) is False


def test_wants_gripper_colliders_false_for_kinematic_no_contact_hold():
    assert _wants_gripper_colliders(kinematic=True, contact_hold=False) is False
