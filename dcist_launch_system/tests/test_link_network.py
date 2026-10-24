"""Robot link network: which network robot <-> base links use, apart from the
network the console / SSH reach the machines on."""

from dcist_launch_system.fleet.launch_workflow import (
    LaunchDeps,
    LaunchPlan,
    build_robot_commands,
    dry_run_lines,
    run_launch,
)
from dcist_launch_system.fleet_helpers import generate_zenoh_endpoints, link_network

from test_launch_workflow import OUT, RUNTIME, FakeSsh, Recorder

TOPO = {
    "networks": {"mit_wifi": {"subnet": "10.28.0.0/14"}, "silvus": {"subnet": "192.168.100.0/24"}},
    "machines": {
        "smaug": {"role": "robot", "addresses": {"mit_wifi": "10.29.0.1", "silvus": "192.168.100.3"}},
        "topaz": {"role": "robot", "addresses": {"mit_wifi": "10.29.0.2"}},  # no radio
        "willow": {"role": "base_station", "addresses": {"mit_wifi": "10.31.0.9", "silvus": "192.168.100.100"}},
    },
    "zenoh": {"port": 7447, "connect_network": "silvus", "fleetlink_networks": ["silvus", "mit_wifi"]},
    "fleet_defaults": {"fleet_link": "router", "fleetlink_base": "willow"},
}


def test_link_network_precedence():
    assert link_network(TOPO, "mit_wifi", "mit_wifi") == "mit_wifi"  # the launch's choice
    assert link_network(TOPO, None, "mit_wifi") == "silvus"  # zenoh.connect_network
    assert link_network(TOPO, "nope", "mit_wifi") == "silvus"  # unknown names skipped
    no_pref = {**TOPO, "zenoh": {"port": 7447}}
    assert link_network(no_pref, None, "mit_wifi") == "mit_wifi"  # old behaviour


def _launch(plan, topo=TOPO):
    deployed = []
    rec = Recorder()
    deps = LaunchDeps(
        ssh=FakeSsh(),
        deploy_zenoh=lambda user, ip, eps: (deployed.append(eps), (True, "deployed"))[1],
        zenoh_endpoints=generate_zenoh_endpoints,
        rviz=lambda name: f"/tmp/dcist_{name}.rviz",
        parallel=lambda fn, items: [(i, fn(i)) for i in items],
        sleep=lambda s: None,
    )
    run_launch(plan, topo=topo, active_network="mit_wifi", runtime_config=RUNTIME,
               output_root=OUT, callbacks=rec, deps=deps)  # fmt: skip
    return deployed, rec


def _plan(**kw):
    return LaunchPlan(robots=["smaug"], session="spot-default", experiment="e1",
                      base_station="willow", bs_session="base_station-default",
                      preflight=False, **kw)  # fmt: skip


def test_router_mode_base_dials_robots_on_the_link_network():
    # console on MIT wifi, robot links over the radio (topology default)
    deployed, rec = _launch(_plan())
    assert deployed == [["tcp/192.168.100.3:7447"]]
    assert "robot links over silvus" in rec.text()
    # the launch can pick wifi instead
    deployed, _ = _launch(_plan(link_network="mit_wifi"))
    assert deployed == [["tcp/10.29.0.1:7447"]]


def test_robot_without_a_radio_address_stays_reachable_with_a_warning():
    plan = _plan()
    plan.robots = ["smaug", "topaz"]
    deployed, rec = _launch(plan)
    assert deployed == [["tcp/192.168.100.3:7447", "tcp/10.29.0.2:7447"]]
    assert ("warn", "topaz: no silvus address in topology; linking over mit_wifi") in rec.lines


def test_fleetlink_robot_dials_the_base_on_the_chosen_network_only():
    topo = {**TOPO, "fleet_defaults": {"fleet_link": "fleetlink", "fleetlink_base": "willow"}}
    [(_, cmd)] = build_robot_commands(_plan(link_network="silvus"), RUNTIME, OUT, topo)
    assert "ADT4_FLEETLINK_CONNECT=tcp/192.168.100.100:7450 " in cmd
    # topology default: failover order from zenoh.fleetlink_networks
    [(_, cmd)] = build_robot_commands(_plan(), RUNTIME, OUT, topo)
    assert "ADT4_FLEETLINK_CONNECT=tcp/192.168.100.100:7450,tcp/10.31.0.9:7450 " in cmd


def test_dry_run_names_the_link_network():
    lines = dry_run_lines(_plan(), RUNTIME, OUT, TOPO, "mit_wifi")
    assert lines[0] == "Robot links: silvus, router mode (base dials the robots); console/ssh: mit_wifi"


def test_launch_link_mode_overrides_the_topology():
    import pytest

    from dcist_launch_system.fleet.launch_workflow import LaunchError, with_link_mode

    # fleetlink for this launch: the base router links to no robot, robots dial the base
    deployed, rec = _launch(_plan(link_mode="fleetlink"))
    assert deployed == [[]]
    [(_, cmd)] = build_robot_commands(_plan(link_mode="fleetlink"), RUNTIME, OUT, with_link_mode(TOPO, "fleetlink"))
    assert "ADT4_FLEET_LINK=fleetlink " in cmd and "ADT4_FLEETLINK_CONNECT=tcp/192.168.100.100:7450" in cmd
    lines = dry_run_lines(_plan(link_mode="fleetlink"), RUNTIME, OUT, TOPO, "mit_wifi")
    assert lines[0].startswith("Robot links: silvus, fleetlink mode (robots dial the fleetlink base)")
    # the topology's own mode otherwise
    assert dry_run_lines(_plan(), RUNTIME, OUT, TOPO, "mit_wifi")[0].startswith("Robot links: silvus, router mode")
    with pytest.raises(LaunchError):
        with_link_mode(TOPO, "carrier-pigeon")
    no_base = {**TOPO, "fleet_defaults": {"fleet_link": "router"}}
    with pytest.raises(LaunchError, match="fleetlink_base"):
        with_link_mode(no_base, "fleetlink")
