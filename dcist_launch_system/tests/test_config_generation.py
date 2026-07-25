"""Source-contract tests for generated static-map fleet sessions."""

from pathlib import Path

import yaml


LAUNCH_SYSTEM = Path(__file__).resolve().parents[1]
MANIFEST = LAUNCH_SYSTEM / "config_generation" / "experiment_manifest.yaml"
COMPONENTS = LAUNCH_SYSTEM / "config_generation" / "launch_components"
PLUGINS = LAUNCH_SYSTEM / "config_generation" / "base_params" / "omniplanner_plugins.yaml"


def _panes(component):
    return "\n".join(
        pane for window in component["windows"] for pane in window["panes"]
    )


def test_fleet_manifest_has_three_independent_session_sources():
    experiments = yaml.safe_load(MANIFEST.read_text())["experiments"]
    assert "isaac_fleet_static" not in experiments
    assert experiments["isaac_fleet_static_hamilton"]["launch_config"] == [
        "spot_isaac_static_executor"
    ]
    assert experiments["isaac_fleet_static_euclid"]["launch_config"] == [
        "spot_isaac_static_executor"
    ]
    assert experiments["isaac_fleet_static_willow"]["launch_config"] == [
        "willow_static_planning"
    ]


def test_static_executor_excludes_mapping_and_camera_frontends():
    executor = yaml.safe_load(
        (COMPONENTS / "spot_isaac_static_executor.yaml").read_text()
    )
    panes = _panes(executor)

    assert "launch_spot_executor:=true" in panes
    assert "--frame-id map --child-frame-id ${ADT4_ROBOT_NAME}/odom" in panes
    for forbidden in (
        "launch_hydra",
        "launch_roman",
        "launch_zed",
        "launch_spot_camera",
        "launch_spot_camera_driver",
        "launch_spot_camera_decompression",
        "launch_instance_segmentation",
        "launch_semantic_inference",
    ):
        assert forbidden not in panes


def test_willow_uses_one_absolute_shared_dsg_and_two_holding_inputs():
    willow = yaml.safe_load((COMPONENTS / "willow_static_planning.yaml").read_text())
    panes = _panes(willow)

    assert panes.count("launch_omniplanner:=true") == 1
    assert panes.count("ros2 run rmw_zenoh_cpp rmw_zenohd") == 1
    assert "heracles_dsg_out_topic:=/heracles/dsg_out" in panes
    assert "omniplanner_dsg_topic:=/heracles/dsg_out" in panes
    assert "robot_name:=hamilton launch_heracles_state_updater:=true" in panes
    assert "robot_name:=euclid launch_heracles_state_updater:=true" in panes
    assert "launch_spot_executor:=true" not in panes


def test_omniplanner_roster_includes_both_fleet_executors_not_willow():
    plugins = yaml.safe_load(PLUGINS.read_text())
    robots = {robot["robot_name"] for robot in plugins["robots"]}
    assert {"hamilton", "euclid"} <= robots
    assert "willow" not in robots
