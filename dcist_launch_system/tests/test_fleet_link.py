"""fleet_link mode switch: router endpoints, fleetlink endpoints, fleet_env, launch env."""

import copy
import shlex
import subprocess
import sys

import pytest

from dcist_launch_system import fleet_env as fe
from dcist_launch_system.fleet.launch_workflow import (
    LaunchDeps,
    LaunchPlan,
    build_base_station_command,
    build_robot_commands,
    dry_run_lines,
    run_launch,
)
from dcist_launch_system.fleet_helpers import (
    fleet_link_mode,
    fleetlink_summary,
    generate_fleetlink_endpoints,
    generate_zenoh_endpoints,
    load_topology,
)

TOPO = {
    "networks": {
        "mit_wifi": {"subnet": "10.29.0.0/16"},
        "silvus": {"subnet": "192.168.100.0/24"},
        "penn_wifi": {"subnet": "192.168.129.0/24"},
    },
    "machines": {
        "alpha": {
            "role": "robot",
            "addresses": {"mit_wifi": "10.29.1.1", "silvus": "192.168.100.10"},
        },
        "beta": {"role": "robot", "addresses": {"mit_wifi": "10.29.1.2"}},
        "gamma": {
            "role": "base_station",
            "addresses": {"mit_wifi": "10.29.1.3", "silvus": "192.168.100.100"},
        },
        "delta": {"role": "base_station", "addresses": {"penn_wifi": "192.168.129.5"}},
    },
    "zenoh": {
        "port": 7447,
        "connect_network": "silvus",
        "fleetlink_port": 7450,
        "fleetlink_networks": ["silvus", "mit_wifi"],
    },
    "fleet_defaults": {"fleet_link": "router", "fleetlink_base": "gamma"},
}


def topo(mode="router", **zenoh):
    t = copy.deepcopy(TOPO)
    t["fleet_defaults"]["fleet_link"] = mode
    t["zenoh"].update(zenoh)
    return t


# --- endpoints -----------------------------------------------------------------


def test_router_mode_endpoints_unchanged():
    t = topo("router")
    assert fleet_link_mode(t) == "router"
    assert generate_zenoh_endpoints(t, "mit_wifi", "gamma") == [
        "tcp/10.29.1.1:7447",
        "tcp/10.29.1.2:7447",
    ]
    assert generate_zenoh_endpoints(t, "silvus", "gamma", robot_filter=["alpha"]) == [
        "tcp/192.168.100.10:7447"
    ]
    assert generate_zenoh_endpoints(t, "mit_wifi", "alpha") == []


@pytest.mark.parametrize("name", ["alpha", "beta", "gamma", "delta"])
def test_fleetlink_mode_no_router_endpoints(name):
    t = topo("fleetlink")
    for net in ("mit_wifi", "silvus", "penn_wifi"):
        assert generate_zenoh_endpoints(t, net, name) == []
        assert generate_zenoh_endpoints(t, net, name, robot_filter=["alpha"]) == []


def test_fleetlink_mode_unknown_machine_still_keyerror():
    # consumers report "no address" on KeyError; keep that contract
    with pytest.raises(KeyError):
        generate_zenoh_endpoints(topo("fleetlink"), "silvus", "nosuch")


def test_unknown_mode_falls_back_to_router():
    assert fleet_link_mode(topo("bogus")) == "router"
    assert fleet_link_mode({}) == "router"
    assert fleet_link_mode(None) == "router"


def test_fleetlink_endpoints_ordered_by_networks():
    t = topo("fleetlink")
    assert generate_fleetlink_endpoints(t) == [
        "tcp/192.168.100.100:7450",
        "tcp/10.29.1.3:7450",
    ]
    t["zenoh"]["fleetlink_networks"] = ["mit_wifi", "silvus"]
    assert generate_fleetlink_endpoints(t) == [
        "tcp/10.29.1.3:7450",
        "tcp/192.168.100.100:7450",
    ]
    # networks the base has no address on are skipped
    t["zenoh"]["fleetlink_networks"] = ["penn_wifi", "mit_wifi"]
    assert generate_fleetlink_endpoints(t) == ["tcp/10.29.1.3:7450"]


def test_fleetlink_endpoints_args_and_defaults():
    t = topo("fleetlink", fleetlink_port=17450)
    assert generate_fleetlink_endpoints(t, base="delta") == []
    assert generate_fleetlink_endpoints(t, networks=["mit_wifi"]) == [
        "tcp/10.29.1.3:17450"
    ]
    assert generate_fleetlink_endpoints(t, base="nosuch") == []
    # no fleetlink_networks: connect_network, then every base network
    del t["zenoh"]["fleetlink_networks"]
    assert generate_fleetlink_endpoints(t) == ["tcp/192.168.100.100:17450"]
    del t["zenoh"]["connect_network"]
    assert generate_fleetlink_endpoints(t, base="delta") == ["tcp/192.168.129.5:17450"]
    # no port: 7450
    del t["zenoh"]["fleetlink_port"]
    assert generate_fleetlink_endpoints(t, networks=["silvus"]) == [
        "tcp/192.168.100.100:7450"
    ]


def test_fleetlink_endpoint_ipv6():
    t = topo("fleetlink")
    t["machines"]["gamma"]["addresses"]["silvus"] = "fd00::1"
    assert generate_fleetlink_endpoints(t)[0] == "tcp/[fd00::1]:7450"


def test_fleetlink_summary():
    t = topo("fleetlink")
    assert fleetlink_summary(t) == "fleetlink client → gamma:7450"
    assert fleetlink_summary(t, "robot") == "fleetlink client → gamma:7450"
    assert "gamma" in fleetlink_summary(t, "base_station")


def test_real_topology_endpoints(topology_path):
    t = load_topology(topology_path)
    t["fleet_defaults"]["fleet_link"] = "fleetlink"
    eps = generate_fleetlink_endpoints(t)
    assert eps, "fleetlink_base must have an address on a fleetlink network"
    assert all(e.endswith(f":{t['zenoh']['fleetlink_port']}") for e in eps)
    for name in t["machines"]:
        for net in t["networks"]:
            assert generate_zenoh_endpoints(t, net, name) == []


# --- fleet_env -----------------------------------------------------------------


def test_fleet_env_router_mode():
    env = fe.fleet_env("alpha", topo("router"))
    assert env == {
        "ADT4_FLEET_LINK": "router",
        "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7447",
        "ADT4_FLEETLINK_PORT": "7450",
    }


def test_fleet_env_fleetlink_robot_and_base():
    t = topo("fleetlink")
    env = fe.fleet_env("alpha", t)
    assert env["ADT4_FLEET_LINK"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == (
        "tcp/192.168.100.100:7450,tcp/10.29.1.3:7450"
    )
    base = fe.fleet_env("gamma", t)
    assert base["ADT4_FLEET_LINK"] == "fleetlink"
    assert base["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"
    assert fe.fleet_env("alpha", t, network="mit_wifi")["ADT4_FLEETLINK_CONNECT"] == (
        "tcp/10.29.1.3:7450"
    )


def test_fleet_env_mode_override():
    env = fe.fleet_env("alpha", topo("fleetlink"), mode="router")
    assert env["ADT4_FLEET_LINK"] == "router"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"
    env = fe.fleet_env("alpha", topo("router"), mode="fleetlink")
    assert env["ADT4_FLEETLINK_CONNECT"].startswith("tcp/192.168.100.100:7450")
    warnings = []
    env = fe.fleet_env("alpha", topo("router"), mode="bogus", warn=warnings.append)
    assert env["ADT4_FLEET_LINK"] == "router" and warnings


def test_fleet_env_falls_back_to_router():
    warnings = []
    t = topo("fleetlink")
    # unknown machine
    assert (
        fe.fleet_env("nosuch", t, warn=warnings.append)["ADT4_FLEET_LINK"] == "router"
    )
    # base unreachable on every fleetlink network
    t2 = topo("fleetlink", fleetlink_networks=["penn_wifi"])
    env = fe.fleet_env("alpha", t2, warn=warnings.append)
    assert env["ADT4_FLEET_LINK"] == "router"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7447"
    # --network the base is not on: keep the full list
    env = fe.fleet_env("alpha", t, network="penn_wifi", warn=warnings.append)
    assert env["ADT4_FLEETLINK_CONNECT"].count(",") == 1
    # garbage topology never raises
    assert fe.fleet_env("alpha", {"machines": 5, "fleet_defaults": {"fleet_link": "fleetlink"}},
                        warn=warnings.append)["ADT4_FLEET_LINK"] == "router"  # fmt: skip
    assert len(warnings) == 4


def _cli(*args):
    return subprocess.run(
        [sys.executable, "-m", "dcist_launch_system.fleet_env", *args],
        capture_output=True,
        text=True,
        timeout=30,
    )


def _exports(stdout):
    out = {}
    for line in stdout.splitlines():
        assert line.startswith("export ")
        k, v = line[len("export ") :].split("=", 1)
        out[k] = shlex.split(v)[0]
    return out


def test_fleet_env_cli(tmp_path):
    import yaml

    path = tmp_path / "topology.yaml"
    path.write_text(yaml.safe_dump(topo("fleetlink")))
    r = _cli("alpha", "--topology", str(path))
    assert r.returncode == 0, r.stderr
    assert _exports(r.stdout) == {
        "ADT4_FLEET_LINK": "fleetlink",
        "ADT4_FLEETLINK_CONNECT": "tcp/192.168.100.100:7450,tcp/10.29.1.3:7450",
        "ADT4_FLEETLINK_PORT": "7450",
    }
    r = _cli("alpha", "--topology", str(path), "--network", "mit_wifi")
    assert _exports(r.stdout)["ADT4_FLEETLINK_CONNECT"] == "tcp/10.29.1.3:7450"
    r = _cli("alpha", "--topology", str(path), "--mode", "router")
    assert _exports(r.stdout)["ADT4_FLEET_LINK"] == "router"
    # a missing topology still exits 0 with router mode
    r = _cli("alpha", "--topology", str(tmp_path / "missing.yaml"))
    assert r.returncode == 0
    assert _exports(r.stdout)["ADT4_FLEET_LINK"] == "router"
    assert "router mode" in r.stderr


def test_fleet_env_cli_real_topology():
    r = _cli("willow")
    assert r.returncode == 0, r.stderr
    assert set(_exports(r.stdout)) == {
        "ADT4_FLEET_LINK",
        "ADT4_FLEETLINK_CONNECT",
        "ADT4_FLEETLINK_PORT",
    }


# --- launch workflow env -----------------------------------------------------------

OUT = "~/adt4_output"
RUNTIME = {
    "alpha": {"name": "alpha", "role": "robot", "ip": "10.29.1.1", "user": "swarm",
              "platform_id": "smaug", "online": True},
    "gamma": {"name": "gamma", "role": "base_station", "ip": "10.29.1.3",
              "user": "rrg", "platform_id": "base_station", "online": True},
}  # fmt: skip


def _env_of(cmd):
    """``VAR=value`` assignments right before run-adt4 / nohup in a launch command."""
    tail = cmd.split("source install/setup.zsh && ", 1)[1]
    env = {}
    for tok in shlex.split(tail):
        if "=" not in tok or tok.startswith("-"):
            break
        k, v = tok.split("=", 1)
        env[k] = v
    return env


def test_robot_commands_carry_fleet_env():
    plan = LaunchPlan(robots=["alpha"], session="spot-default", experiment="e1")
    [(_, cmd)] = build_robot_commands(plan, RUNTIME, OUT, topo("fleetlink"))
    env = _env_of(cmd)
    assert env["ADT4_FLEET_LINK"] == "fleetlink"
    assert (
        env["ADT4_FLEETLINK_CONNECT"] == "tcp/192.168.100.100:7450,tcp/10.29.1.3:7450"
    )
    assert "run-adt4 spot-default -n alpha" in cmd
    [(_, cmd)] = build_robot_commands(plan, RUNTIME, OUT, topo("router"))
    assert _env_of(cmd) == {
        "ADT4_FLEET_LINK": "router",
        "ADT4_FLEETLINK_CONNECT": "tcp/127.0.0.1:7447",
    }
    # no topology: unchanged command (run-adt4 derives the env itself)
    [(_, cmd)] = build_robot_commands(plan, RUNTIME, OUT)
    assert _env_of(cmd) == {}


def test_base_command_carries_fleet_link():
    cmd = build_base_station_command(
        RUNTIME["gamma"], "base_station-default", OUT, "e1", topo=topo("fleetlink")
    )
    env = _env_of(cmd)
    assert env["ADT4_FLEET_LINK"] == "fleetlink"
    assert env["ADT4_FLEETLINK_CONNECT"] == "tcp/127.0.0.1:7450"
    assert env["ADT4_FLEETLINK_PORT"] == "7450"
    assert "nohup " in cmd


def test_dry_run_lines_show_env():
    plan = LaunchPlan(robots=["alpha"], session="spot-default", experiment="e1")
    [line] = dry_run_lines(plan, RUNTIME, OUT, topo("fleetlink"))
    assert "ADT4_FLEET_LINK=fleetlink" in line


class _Rec:
    def __init__(self):
        self.lines = []

    def log(self, level, text):
        self.lines.append((level, text))

    def confirm(self, request):
        return None

    def progress(self, machine, pct):
        pass

    def local_zenohd_running(self):
        return False

    def stop_local_zenohd(self):
        pass


@pytest.mark.parametrize("mode", ["router", "fleetlink"])
def test_run_launch_phase0_and_env(mode):
    deployed, sshed = [], []

    def ssh(user, ip, cmd, timeout=5):
        sshed.append((ip, cmd))
        if "tmux has-session" in cmd:
            return (0, "OK", "") if any("nohup" in c for _, c in sshed) else (1, "", "")
        return (0, "", "") if "run-adt4" in cmd else (1, "", "")

    deps = LaunchDeps(
        ssh=ssh,
        rsync=lambda *a, **k: (True, "direct", "ok"),
        deploy_zenoh=lambda user, ip, eps: deployed.append((ip, eps)) or (True, "ok"),
        rviz=lambda name: f"/tmp/dcist_{name}.rviz",
        parallel=lambda fn, items: [(i, fn(i)) for i in items],
        sleep=lambda s: None,
    )
    plan = LaunchPlan(
        robots=["alpha"], session="spot-default", experiment="e1",
        base_station="gamma", bs_session="base_station-default",
    )  # fmt: skip
    rec = _Rec()
    t = topo(mode)
    assert run_launch(
        plan, topo=t, active_network="silvus", runtime_config=RUNTIME,
        output_root=OUT, callbacks=rec, deps=deps,
    )  # fmt: skip
    if mode == "fleetlink":
        # the base router is told to link to no robots (clears router-mode links)
        assert deployed == [("10.29.1.3", [])]
        assert any("fleetlink client → gamma:7450" in text for _, text in rec.lines)
    else:
        assert deployed == [("10.29.1.3", ["tcp/192.168.100.10:7447"])]
    robot_cmd = next(c for ip, c in sshed if ip == "10.29.1.1" and "run-adt4" in c)
    base_cmd = next(c for ip, c in sshed if ip == "10.29.1.3" and "nohup" in c)
    assert _env_of(robot_cmd)["ADT4_FLEET_LINK"] == mode
    assert _env_of(base_cmd)["ADT4_FLEET_LINK"] == mode


# --- generated tmux sessions ---------------------------------------------------------


def _session(name):
    import pathlib

    import yaml

    pkg = pathlib.Path(__file__).resolve().parents[1]
    session = yaml.safe_load(
        (pkg / "tmux" / "autogenerated" / f"{name}.yaml").read_text()
    )
    # panes are process panes (`shell: zsh -ic ...`); compare their commands
    sys.path.insert(0, str(pkg / "config_generation"))
    import generate_configs

    for window in session["windows"]:
        window["panes"] = [generate_configs.pane_command(p) for p in window["panes"]]
    return session


@pytest.mark.parametrize(
    "name",
    [
        "base_station-default",
        "base_station_heracles-default",
        "base_station_web-default",
    ],
)
def test_base_sessions_have_fleetlink_base_window(name):
    s = _session(name)
    assert s["environment"]["ADT4_FLEET_LINK"] == "$ADT4_FLEET_LINK"
    [win] = [w for w in s["windows"] if w["window_name"] == "fleetlink_base"]
    router, bridge = win["panes"]
    assert (
        '[ "$ADT4_FLEET_LINK" = fleetlink ]' in router and "fleetlink-router" in router
    )
    assert "fleetlink-bridge --ros-args" in bridge
    assert "-p zenoh_connect:=${ADT4_FLEETLINK_CONNECT:-tcp/127.0.0.1:7450}" in bridge
    assert "-p relay_config:=" in bridge and "-p topology:=" in bridge


@pytest.mark.parametrize("name", ["spot-default", "phoenix-default"])
def test_robot_sessions_pass_fleetlink_connect(name):
    s = _session(name)
    assert s["environment"]["ADT4_FLEETLINK_CONNECT"] == "$ADT4_FLEETLINK_CONNECT"
    [win] = [w for w in s["windows"] if w["window_name"] == "fleetlink"]
    assert (
        "-p zenoh_connect:=${ADT4_FLEETLINK_CONNECT:-tcp/127.0.0.1:7447}"
        in (win["panes"][0])
    )
    # the ROS relay only in fleetlink mode (router mode already merges the graphs)
    assert (
        '$([ "$ADT4_FLEET_LINK" = fleetlink ] && echo -p relay_config:=default'
        in (win["panes"][0])
    )
    assert not any(w["window_name"] == "fleetlink_base" for w in s["windows"])


@pytest.mark.parametrize(
    "name,window,pane",
    [
        ("spot-default", "fleetlink", 0),
        ("phoenix-default", "fleetlink", 0),
        ("base_station-default", "fleetlink_base", 1),
        ("base_station_web-default", "fleetlink_base", 1),
    ],
)
def test_fleetlink_tf_is_namespaced(name, window, pane):
    # the agent reads the robot's TF and the bridge writes the base's TF, both /<robot>/tf
    [win] = [w for w in _session(name)["windows"] if w["window_name"] == window]
    cmd = win["panes"][pane]
    assert "-r /tf:=/$ADT4_ROBOT_NAME/tf -r /tf_static:=/$ADT4_ROBOT_NAME/tf_static" in cmd


@pytest.mark.parametrize(
    "name,window",
    [
        ("spot-default", "tf_network"),
        ("spot_bag_lc-default", "tf_network"),
        ("base_station-default", "tf_relay"),
        ("base_station_web-default", "tf_relay"),
    ],
)
def test_sessions_relay_tf_to_base(name, window):
    assert any(w["window_name"] == window for w in _session(name)["windows"])
