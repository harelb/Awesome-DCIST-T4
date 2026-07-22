"""Stage construction for dcist_sim_isaac.

Builds a World, ground plane, distant light, the scenario's environment
USD (if present on disk), all `RobotSpec`s (as kinematic `SpotSimRobot`s
-- see spot_robot.py) and all `ObjectSpec`s (referenced + posed + given
a USD semantic label for Task 9's registry / GT output).

Task 9: every spawned object is also marked a *kinematic* rigid body
(`_mark_kinematic`, mirroring `spot_robot.py`'s robot-kinematic marking)
and registered in a `grasp.ObjectRegistry` returned on `SimStage`. Objects
are magic-attach targets -- `grasp.GraspBackend` always sets their world
pose directly (grasp/place/reset), never relies on PhysX to move them --
so kinematic-marking them prevents gravity/contacts from fighting those
writes or otherwise moving an unheld object out from under a future
grasp attempt. `RigidBodyAPI` is applied fresh to the top-level object
prim if the source USD didn't already author one (P1 placeholder assets
may be plain meshes with no physics authored at all), so "kinematic" is
guaranteed regardless of what Task 10's real assets end up authoring.

Task 6 (P4 physics mode, `scenario.physics_mode`): when any robot in the
scenario has `locomotion: policy` or `grasping: physics`, the stage is
built physics-capable instead of kinematic-only --
  - robots with `locomotion: policy` are spawned NOT kinematic (their
    articulation stands under gravity/PhysX until Task 8's
    PolicyDriveBackend drives it; robots with `locomotion: kinematic`
    are unaffected -- see `_spawn_robots`);
  - objects become dynamic rigid bodies with convex-hull colliders
    instead of kinematic (`_make_dynamic`, spec §5) so they fall/settle
    and can be pushed;
  - every environment mesh gets a static triangle-mesh collider
    (`_collide_environment`, spec §5);
  - a `Costmap2D` is baked from the live PhysX scene right after
    `world.reset()` (`costmap_bake.bake_costmap`) and stored on
    `SimStage.costmap`/`.costmap_raw`.
In kinematic-only scenarios (physics_mode is False) every one of these
branches is skipped and behavior is bit-for-bit identical to pre-Task-6.
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class SimStage:
    """Everything sim_app.py's main loop / RosBridge need."""

    world: object
    robots: list = field(default_factory=list)
    # Task 9: `grasp.ObjectRegistry` of every spawned object, and the
    # scenario's magic-grasp selection radius (`grasp.DEFAULT_GRASP_RADIUS`
    # if unset in the YAML -- see scenario.py).
    registry: object = None
    grasp_radius: float = 1.5
    # Task 6: `costmap.Costmap2D` baked from the live PhysX scene, physics
    # mode only (None in kinematic-only scenarios). `costmap` is the
    # inflated map local_planner.py should navigate against; `costmap_raw`
    # is the pre-inflation map, kept for Task 10's diagnostics/visualization.
    costmap: object = None
    costmap_raw: object = None


def _yaw_to_quat_wxyz(yaw: float):
    import math

    import numpy as np

    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def _spawn_robots(world, scenario) -> list:
    from dcist_sim_isaac.spot_robot import SpotSimRobot

    robots = []
    for spec in scenario.robots:
        robots.append(
            SpotSimRobot(world, spec, kinematic=(spec.locomotion == "kinematic"))
        )
    return robots


def _mark_kinematic(prim) -> None:
    """Make `prim` (and any `RigidBodyAPI` descendants) a kinematic rigid
    body -- see this module's docstring for why. Mirrors
    `spot_robot.py`'s robot-kinematic marking, plus applies
    `RigidBodyAPI` fresh to the top-level prim if absent (placeholder
    object assets may have no physics APIs at all).
    """
    from pxr import Usd, UsdPhysics

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    for child in Usd.PrimRange(prim):
        if child != prim and child.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(child).CreateKinematicEnabledAttr(True)


def _make_dynamic(prim) -> None:
    """Dynamic rigid body + convex-hull collider (spec §5): objects fall,
    settle, and can be pushed. Applied to the top-level object prim
    (mirrors `_mark_kinematic`'s "apply RigidBodyAPI fresh if absent"
    guarantee -- P1 placeholder assets may be plain meshes with no
    physics authored at all)."""
    from pxr import Usd, UsdGeom, UsdPhysics

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    for child in Usd.PrimRange(prim):
        if child.IsA(UsdGeom.Mesh) and not child.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(child)
            UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr(
                "convexHull")


def _spawn_objects(scenario, physics_mode: bool = False):
    import numpy as np
    import omni.usd
    from isaacsim.core.prims import XFormPrim
    from isaacsim.core.utils.semantics import add_labels
    from isaacsim.core.utils.stage import add_reference_to_stage

    from dcist_sim_isaac.grasp import ObjectRegistry

    registry = ObjectRegistry()
    stage = omni.usd.get_context().get_stage()
    for obj in scenario.objects:
        usd_path = scenario.resolve_path(obj.usd)
        if not os.path.exists(usd_path):
            logger.warning(
                "object usd '%s' not found on disk; skipping object '%s' "
                "(expected until Task 10 assets exist)",
                usd_path, obj.object_id,
            )
            continue

        prim_path = f"/World/objects/{obj.object_id}"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        xform = XFormPrim(prim_path)
        quat = _yaw_to_quat_wxyz(obj.yaw)
        xform.set_world_poses(
            positions=np.array([[obj.x, obj.y, obj.z]]),
            orientations=np.array([quat]),
        )

        prim = stage.GetPrimAtPath(prim_path)
        add_labels(prim, labels=[obj.label], instance_name="class")
        if physics_mode:
            _make_dynamic(prim)
        else:
            _mark_kinematic(prim)

        registry.add(
            object_id=obj.object_id,
            prim_path=prim_path,
            label=obj.label,
            graspable=obj.graspable,
            spawn_pos=(obj.x, obj.y, obj.z),
            spawn_quat=quat,
        )

    return registry


def _collide_environment(stage) -> int:
    """Static triangle-mesh colliders on every environment mesh (spec §5).
    Returns the number of meshes that got a collider (0 = the prerequisite
    check FAILED -- caller raises)."""
    from pxr import Usd, UsdGeom, UsdPhysics

    env = stage.GetPrimAtPath("/World/Environment")
    if not env.IsValid():
        return 0
    n = 0
    for prim in Usd.PrimRange(env):
        if prim.IsA(UsdGeom.Mesh):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
            n += 1
    return n


def build_stage(scenario) -> SimStage:
    # isaacsim.core.api.World is the 6.0 location (unchanged from 5.x);
    # verified against the installed 6.0.1.0 package - see
    # dcist_sim_isaac/README.md "API mapping".
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    import omni.usd
    from pxr import UsdLux

    # Task 8 (P4): physics-mode scenarios must run the World at the pretrained
    # policy's native rate (500 Hz physics / 60 Hz render). At any other
    # physics_dt the policy's decimation (counted in physics steps, not
    # elapsed time) silently drifts off its 50 Hz training rate and the walk
    # destabilises (policy_spike_report.md §6). Kinematic-only scenarios keep
    # the bare `World()` (defaults) -- bit-for-bit pre-Task-8 behavior.
    if scenario.physics_mode:
        from dcist_sim_isaac.drive_backends import (
            POLICY_PHYSICS_DT, POLICY_RENDERING_DT)
        world = World(physics_dt=POLICY_PHYSICS_DT,
                      rendering_dt=POLICY_RENDERING_DT)
    else:
        world = World()
    world.scene.add_default_ground_plane()

    # Distant light so the scene isn't pitch black for any future
    # camera/rendering work (Task 8). Use the USD API directly: the
    # omni.kit.commands CreatePrim command fails on 6.0 with
    # "'Property' object has no attribute 'Set'" for light intensity
    # (the attribute is namespaced 'inputs:intensity' in current UsdLux).
    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    light.CreateIntensityAttr(3000.0)

    env_path = scenario.resolve_path(scenario.environment_usd)
    if os.path.exists(env_path):
        add_reference_to_stage(usd_path=env_path, prim_path="/World/Environment")
    else:
        logger.warning(
            "environment usd '%s' not found on disk; continuing without it "
            "(expected until Task 10 assets exist)",
            env_path,
        )

    if scenario.physics_mode:
        n_colliders = _collide_environment(stage)
        if n_colliders == 0:
            raise RuntimeError(
                "physics mode requires environment collision meshes; "
                f"'{env_path}' produced none (spec §5 prerequisite)")
        logger.info("physics mode: %d environment meshes collidable", n_colliders)

    robots = _spawn_robots(world, scenario)
    registry = _spawn_objects(scenario, scenario.physics_mode)

    world.reset()

    # Boot order is load-bearing (GPU-verified for Tasks 8-17):
    #   world.reset() -> initialize loop -> settle 120 frames -> costmap bake.
    # The initialize loop MUST run before the settle: it sets the arm to its
    # stowed pose, applies the articulation gains, and activates each policy
    # robot's walking-policy physics callback. If we settled first, the
    # articulation would step ~2 sim-seconds with an uninitialized policy and
    # the default deployed-forward arm -- the Task-9 topple regime. Settling
    # AFTER init lets the active policy own the articulation during settle,
    # exactly as it did before this loop was moved out of sim_app.py.

    # Camera.initialize() (Task 8) needs a valid physics sim view, which
    # only exists after world.reset() -- see spot_robot.py's comment at
    # the SimZedCamera construction site. Task 8 (P4): policy robots' drive
    # backend is initialized here too (articulation view + physics handles +
    # the walking policy's physics callback only exist post-reset); kinematic
    # robots have drive_backend None and are unaffected.
    for robot in robots:
        robot.camera.initialize()
        if robot.drive_backend is not None:
            robot.drive_backend.initialize(world)

    # Settle (Task 6, moved here from sim_app.py in the P4 final fix): with the
    # policy/arm now initialized (above), let dynamic objects fall/settle onto
    # the environment colliders BEFORE the costmap bake below, so object
    # footprints are stamped at their settled (resting) poses rather than their
    # spawn poses. render=False keeps this cheap and deterministic. Physics mode
    # only -- kinematic scenarios have no dynamics to settle. Doing this inside
    # build_stage (after init, before bake) reproduces the old sim_app boot
    # regime exactly and makes `--bake-only` bake against settled poses too.
    if scenario.physics_mode:
        SETTLE_FRAMES = 120
        for _ in range(SETTLE_FRAMES):
            world.step(render=False)      # objects fall + settle (spec §5)

    # Costmap bake (Task 6): must run AFTER world.reset() -- the PhysX
    # scene query interface needs an initialized physics scene (see
    # costmap_bake.py's module docstring). Colliders were already applied
    # above, before robots/objects were spawned, so nothing here depends
    # on spawn order.
    costmap = None
    costmap_raw = None
    if scenario.physics_mode:
        from dcist_sim_isaac.costmap_bake import (
            bake_costmap, object_footprint_radius)
        # Task 15i: stamp each object's footprint into the costmap so the
        # planner keeps clearance from objects (they're excluded from the env
        # overlap bake by design). Positions are the live object poses from the
        # registry AFTER the settle loop above, so footprints use settled
        # (resting) poses, not spawn poses.
        # Task 15k: derive each footprint radius from the object's live USD
        # world bounds (so a wide/flat asset like the duffel bag gets a disc
        # that covers its true XY extent, not an undersized global 0.25 m --
        # the 15j onto-bag topple root cause). Iterate in a stable order so
        # object_xy and object_radii stay parallel.
        object_ids = list(registry.selection_snapshot().keys())
        snap = registry.selection_snapshot()
        object_xy = [(snap[oid]["pos"][0], snap[oid]["pos"][1])
                     for oid in object_ids]
        object_radii = [object_footprint_radius(registry.prim_path(oid))
                        for oid in object_ids]
        costmap, costmap_raw = bake_costmap(
            scenario.nav, object_xy, object_radii=object_radii)
        # Task 9: give every policy robot a go-to-target planner navigating
        # the SAME baked (inflated) costmap -- one bake per scenario, shared
        # by all policy robots, mirroring how the real BD local nav would
        # share one map. Kinematic robots have `drive_backend is None` and
        # are unaffected (their `set_target_pose` still slews directly,
        # pre-Task-6 behavior).
        for robot in robots:
            if robot.drive_backend is not None:
                robot.attach_planner(costmap, scenario.nav)

    return SimStage(
        world=world, robots=robots, registry=registry,
        grasp_radius=scenario.grasp_radius, costmap=costmap, costmap_raw=costmap_raw,
    )
