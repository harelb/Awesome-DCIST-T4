"""run-adt4: zenoh isolation in fleetlink mode, env precedence, process panes."""

import importlib.machinery
import importlib.util
import io
import json
import os
import pathlib
import shlex
import subprocess
import sys

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def run_adt4():
    path = PKG / "bin" / "run-adt4"
    loader = importlib.machinery.SourceFileLoader("run_adt4", str(path))
    spec = importlib.util.spec_from_loader("run_adt4", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def _script(shell):
    argv = shlex.split(shell)
    assert argv[:2] == ["zsh", "-ic"]
    return argv[2]


def test_zenoh_router_mode_keeps_connect_endpoints(run_adt4):
    window = run_adt4._make_zenoh_window("router")
    script = _script(window["panes"][0]["shell"])
    assert script.startswith(
        "source ${ADT4_WS}/install/setup.zsh && ros2 run rmw_zenoh_cpp rmw_zenohd; "
    )
    assert "ZENOH_CONFIG_OVERRIDE" not in script


def test_zenoh_fleetlink_mode_clears_connect_endpoints(run_adt4):
    window = run_adt4._make_zenoh_window("fleetlink")
    script = _script(window["panes"][0]["shell"])
    assert script.startswith(
        "source ${ADT4_WS}/install/setup.zsh && "
        "ZENOH_CONFIG_OVERRIDE='connect/endpoints=[]' ros2 run rmw_zenoh_cpp rmw_zenohd; "
    )


def _fake_fleet(mode_default="router"):
    def fleet_env(robot_name, mode=None):
        mode = mode or mode_default
        if mode == "fleetlink":
            return {
                "ADT4_FLEET_LINK": "fleetlink",
                "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7450",
                "ADT4_FLEETLINK_PORT": "7450",
            }
        return {
            "ADT4_FLEET_LINK": "router",
            "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7447",
            "ADT4_FLEETLINK_PORT": "7450",
        }

    return fleet_env


def _colors(name):
    return "hamilton:#42d4f4", {"hamilton": "#42d4f4"}.get(name, "")


def test_derive_fills_unset(run_adt4):
    env = {}
    derived = run_adt4._derive_env(env, "hamilton", _colors, _fake_fleet("fleetlink"))
    assert env["ADT4_ROBOT_COLOR"] == "#42d4f4"
    assert env["ADT4_FLEET_LINK"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"
    assert json.loads(env[run_adt4.DERIVED_ENV_VAR]) == derived


def test_explicit_env_wins(run_adt4):
    env = {"ADT4_FLEETLINK_CONNECT": "tcp/10.0.0.1:7450", "ADT4_ROBOT_COLOR": "#000000"}
    run_adt4._derive_env(env, "hamilton", _colors, _fake_fleet("fleetlink"))
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/10.0.0.1:7450"
    assert env["ADT4_ROBOT_COLOR"] == "#000000"


def test_explicit_mode_picks_endpoints(run_adt4):
    env = {"ADT4_FLEET_LINK": "fleetlink"}
    run_adt4._derive_env(env, "willow", _colors, _fake_fleet("router"))
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"


def test_stale_values_from_an_earlier_run_are_derived_again(run_adt4):
    # an earlier router-mode run's values, inherited via the tmux server env
    old = {}
    run_adt4._derive_env(old, "willow", _colors, _fake_fleet("router"))
    assert old["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"

    env = dict(old)
    env["ADT4_FLEET_LINK"] = "fleetlink"  # the user picks fleetlink explicitly
    run_adt4._derive_env(env, "willow", _colors, _fake_fleet("router"))
    assert env["ADT4_FLEET_LINK"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"

    # topology switched to fleetlink: inherited router mode is re-derived too
    env = dict(old)
    run_adt4._derive_env(env, "willow", _colors, _fake_fleet("fleetlink"))
    assert env["ADT4_FLEET_LINK"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"


def test_inherited_but_changed_value_counts_as_explicit(run_adt4):
    old = {}
    run_adt4._derive_env(old, "willow", _colors, _fake_fleet("router"))
    env = dict(old, ADT4_FLEETLINK_CONNECT="tcp/10.1.1.1:7447")
    run_adt4._derive_env(env, "willow", _colors, _fake_fleet("router"))
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/10.1.1.1:7447"


def test_robot_color_without_sourced_workspace(tmp_path):
    """run-adt4 finds dcist_launch_system (and topology.yaml) without a sourced ws.

    On the robots run-adt4 is on PATH, so it often runs where the package is not
    importable; it used to fall back to no color and router mode.
    """
    code = (
        "import importlib.machinery, importlib.util, sys\n"
        f"loader = importlib.machinery.SourceFileLoader('r', {str(PKG / 'bin' / 'run-adt4')!r})\n"
        "spec = importlib.util.spec_from_loader('r', loader)\n"
        "m = importlib.util.module_from_spec(spec); loader.exec_module(m)\n"
        "print(m._robot_colors('hamilton')[1])\n"
    )
    env = {
        k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "ADT4_TOPOLOGY")
    }
    ret = subprocess.run(
        [sys.executable, "-c", code],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
    )
    assert ret.returncode == 0, ret.stderr
    assert ret.stdout.strip().splitlines()[-1] == "#42d4f4"


def test_session_file_uses_process_panes_and_session_env(run_adt4, tmp_path):
    session = tmp_path / "s.yaml"
    session.write_text(
        yaml.safe_dump(
            {
                "session_name": "adt4_system",
                "environment": {"ADT4_FLEET_LINK": "$ADT4_FLEET_LINK"},
                "windows": [{"window_name": "w", "panes": [{"shell": "zsh -ic true"}]}],
            }
        )
    )
    info = run_adt4.SessionInfo(
        "willow",
        tmp_path / "out",
        platform_id="base_station",
        bag=tmp_path / "bag",
        trajectory=tmp_path / "traj.csv",
        record="base_station",
        fleet_link="fleetlink",
        session_env={"ADT4_FLEET_LINK": "fleetlink", "ADT4_ROBOT_NAME": "willow"},
    )
    buf = io.StringIO()
    run_adt4._write_session_to_file(session, info, buf)
    contents = yaml.safe_load(buf.getvalue())
    assert contents["environment"]["ADT4_FLEET_LINK"] == "fleetlink"
    assert contents["environment"]["ADT4_ROBOT_NAME"] == "willow"
    names = [w["window_name"] for w in contents["windows"]]
    assert names[0] == "zenoh"
    assert "play_rosbag" in names and "recording" in names
    for window in contents["windows"]:
        assert "window_shell" not in window
        assert "shell_command_before" not in window
        for pane in window["panes"]:
            assert pane["shell"].startswith("zsh -ic ")
    zenoh = contents["windows"][0]["panes"][0]["shell"]
    assert "connect/endpoints=[]" in zenoh
    bag = next(w for w in contents["windows"] if w["window_name"] == "play_rosbag")
    assert len(bag["panes"]) == 2
    assert "play_rosbag" in bag["panes"][0]["shell"]
    assert "csv_to_tf" in bag["panes"][1]["shell"]
    assert "play_rosbag" not in bag["panes"][1]["shell"]
