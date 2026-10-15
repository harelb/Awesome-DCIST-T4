"""Tests for scripts/generate_fleet_rviz.py."""

import importlib.util
import pathlib

import yaml

_SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1] / "scripts" / "generate_fleet_rviz.py"
)
_spec = importlib.util.spec_from_file_location("generate_fleet_rviz", _SCRIPT)
gfr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gfr)


def _all_topics(displays):
    for d in displays:
        yield from (t["Value"] for t in gfr._topics(d))
        yield from _all_topics(d.get("Displays", []))


def test_generated_file_has_group_per_robot(tmp_path):
    topology = tmp_path / "topology.yaml"
    topology.write_text(
        "machines:\n"
        "  alpha: {role: robot, color: '#ff0000'}\n"
        "  beta: {role: robot}\n"
        "  base: {role: base_station}\n"
    )
    output = tmp_path / "fleet.rviz"
    assert gfr.main(["--topology", str(topology), "--output", str(output)]) == 0

    config = yaml.safe_load(output.read_text())
    displays = config["Visualization Manager"]["Displays"]
    groups = {d["Name"]: d for d in displays if d["Class"] == "rviz_common/Group"}
    assert sorted(groups) == ["alpha (#ff0000)", f"beta ({gfr.rc.PALETTE[0]})"]

    # Global displays survive; nothing at top level still points at hamilton.
    names = {d["Name"] for d in displays}
    assert "Omniplanner Viz" in names
    top_level = [d for d in displays if d["Class"] != "rviz_common/Group"]
    assert not any(t.startswith("/hamilton/") for t in _all_topics(top_level))

    alpha = groups["alpha (#ff0000)"]["Displays"]
    assert alpha and all(t.startswith("/alpha/") for t in _all_topics(alpha))
    paths = [d for d in alpha if d["Class"] == "rviz_default_plugins/Path"]
    assert paths and all(d["Color"] == "255; 0; 0" for d in paths)


def test_default_template_matches_robot_displays():
    config = yaml.safe_load(gfr.DEFAULT_TEMPLATE.read_text())
    displays = config["Visualization Manager"]["Displays"]
    templated = {d["Name"] for d in displays if gfr.is_robot_display(d)}
    assert {"MLP Path", "MLP target", "Traversability Estimates"} <= templated
