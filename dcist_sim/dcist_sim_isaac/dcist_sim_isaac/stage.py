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


def _yaw_to_quat_wxyz(yaw: float):
    import math

    import numpy as np

    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def _spawn_robots(world, scenario) -> list:
    from dcist_sim_isaac.spot_robot import SpotSimRobot

    robots = []
    for spec in scenario.robots:
        robots.append(SpotSimRobot(world, spec))
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


def _spawn_objects(scenario):
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


def build_stage(scenario) -> SimStage:
    # isaacsim.core.api.World is the 6.0 location (unchanged from 5.x);
    # verified against the installed 6.0.1.0 package - see
    # dcist_sim_isaac/README.md "API mapping".
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    import omni.usd
    from pxr import UsdLux

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

    robots = _spawn_robots(world, scenario)
    registry = _spawn_objects(scenario)

    world.reset()

    # Camera.initialize() (Task 8) needs a valid physics sim view, which
    # only exists after world.reset() -- see spot_robot.py's comment at
    # the SimZedCamera construction site.
    for robot in robots:
        robot.camera.initialize()

    return SimStage(
        world=world, robots=robots, registry=registry, grasp_radius=scenario.grasp_radius
    )
