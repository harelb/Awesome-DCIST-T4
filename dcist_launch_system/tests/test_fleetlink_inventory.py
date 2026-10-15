"""Static fleetlink inventory + seed relay.yaml sanity checks (no ROS needed)."""

import importlib.util
from collections import defaultdict
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "dcist_launch_system" / "scripts" / "fleetlink_inventory.py"
RELAY = REPO / "fleetlink" / "config" / "relay.yaml"


@pytest.fixture(scope="module")
def rows():
    spec = importlib.util.spec_from_file_location("fleetlink_inventory", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return {(r["kind"], r["name"]): r for r in mod.inventory()}


def test_known_rows(rows):
    plan = rows[("topic", "/{robot}/omniplanner_node/compiled_plan_out")]
    assert plan["dir"] == "down"
    assert rows[("topic", "/{robot}/spot_executor_node/action_result")]["dir"] == "up"
    assert rows[("service", "/{robot}/update_holding_state")]["dir"] == "up"
    assert ("tf", "map->{robot}/base_link") in rows
    assert rows[("topic", "/global/ros_system_monitor/table_in")]["dir"] == "up"


def test_relay_yaml_loads_without_both_direction_topics():
    cfg = yaml.safe_load(RELAY.read_text())
    dirs = defaultdict(set)
    for t in cfg["topics"]:
        assert t["dir"] in ("up", "down")
        assert t["mode"] in ("reliable", "latest_reliable", "latest")
        dirs[t["topic"]].add(t["dir"])
    assert dirs
    assert not [k for k, v in dirs.items() if len(v) > 1]
    for s in cfg["services"]:
        assert s["dir"] in ("up", "down")
    assert cfg["tf"]
