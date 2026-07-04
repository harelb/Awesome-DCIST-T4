"""Minimal stage construction for dcist_sim_isaac.

This is the Task 5 stub: create a World, a ground plane, a distant light,
and (if it exists on disk) load the scenario's environment USD as a
reference. Tasks 7-9 will extend this to spawn robots and objects from
the Scenario.
"""
import logging
import os

logger = logging.getLogger(__name__)


def build_stage(scenario):
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

    world.reset()
    return world
