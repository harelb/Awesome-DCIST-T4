"""Tests for dcist_launch_system.robot_colors."""

import json

import pytest

from dcist_launch_system import robot_colors as rc


def _topo(**machines):
    return {"machines": machines}


def test_palette_fallback_by_sorted_name():
    topo = _topo(
        zed={"role": "robot"},
        alpha={"role": "robot"},
        base={"role": "base_station"},
    )
    assert rc.robot_colors(topo) == {"alpha": rc.PALETTE[0], "zed": rc.PALETTE[1]}


def test_explicit_colors_win_and_are_skipped_by_fallback():
    topo = _topo(
        alpha={"role": "robot"},
        beta={"role": "robot", "color": rc.PALETTE[0].upper()},
        gamma={"role": "robot", "color": "not-a-color"},
    )
    colors = rc.robot_colors(topo)
    assert colors["beta"] == rc.PALETTE[0]
    # alpha and gamma fall back, skipping the color beta claimed.
    assert colors["alpha"] == rc.PALETTE[1]
    assert colors["gamma"] == rc.PALETTE[2]


def test_wraps_past_palette():
    topo = _topo(**{f"r{i:02d}": {"role": "robot"} for i in range(25)})
    colors = rc.robot_colors(topo)
    assert len(colors) == 25
    assert len(set(list(colors.values())[:20])) == 20


def test_real_topology_robots_distinct(topology_path):
    topo = rc.load_topology(topology_path)
    colors = rc.robot_colors(topo)
    robots = [n for n, m in topo["machines"].items() if m.get("role") == "robot"]
    assert sorted(colors) == sorted(robots)
    assert len(set(colors.values())) == len(colors)
    # Every explicit color comes from the shared palette so console/RViz match.
    assert set(colors.values()) <= set(rc.PALETTE)


def test_hex_helpers():
    assert rc.hex_to_rgb("#ff0000") == (1.0, 0.0, 0.0)
    assert rc.hex_to_rgba("00ff00", 0.5) == (0.0, 1.0, 0.0, 0.5)
    with pytest.raises(ValueError):
        rc.hex_to_rgb("#fff")


def test_ros_string_roundtrip():
    colors = {"a": "#010203", "b": "#abcdef"}
    s = rc.to_ros_string(colors)
    assert s == "a:#010203,b:#abcdef"
    assert rc.parse_ros_string(s) == colors
    assert rc.parse_ros_string(["a:#010203", "junk", "c:#xyz"]) == {"a": "#010203"}
    assert rc.parse_ros_string("") == {}


def test_cli_formats(tmp_path, capsys):
    path = tmp_path / "topology.yaml"
    path.write_text(
        "machines:\n"
        "  a: {role: robot, color: '#112233'}\n"
        "  b: {role: robot}\n"
        "  base: {role: base_station}\n"
    )
    rc.main(["--topology", str(path), "--format", "ros"])
    assert capsys.readouterr().out.strip() == f"a:#112233,b:{rc.PALETTE[0]}"
    rc.main(["--topology", str(path)])
    assert json.loads(capsys.readouterr().out) == {"a": "#112233", "b": rc.PALETTE[0]}
    rc.main(["--topology", str(path), "--name", "a", "--format", "hex"])
    assert capsys.readouterr().out.strip() == "#112233"
    rc.main(["--topology", str(path), "--name", "base", "--format", "hex"])
    assert capsys.readouterr().out.strip() == ""
