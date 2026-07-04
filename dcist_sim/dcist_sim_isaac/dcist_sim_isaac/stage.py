"""Stage construction for dcist_sim_isaac.

Builds a World, ground plane, distant light, the scenario's environment
USD (if present on disk), all `RobotSpec`s (as kinematic `SpotSimRobot`s
-- see spot_robot.py) and all `ObjectSpec`s (referenced + posed + given
a USD semantic label for Task 9's registry / GT output).
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class SimStage:
    """Everything sim_app.py's main loop needs each frame."""

    world: object
    robots: list = field(default_factory=list)


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


def _spawn_objects(scenario) -> None:
    import numpy as np
    import omni.usd
    from isaacsim.core.prims import XFormPrim
    from isaacsim.core.utils.semantics import add_labels
    from isaacsim.core.utils.stage import add_reference_to_stage

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
    _spawn_objects(scenario)

    world.reset()
    return SimStage(world=world, robots=robots)
