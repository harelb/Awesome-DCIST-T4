"""Tests for fleet_helpers.validate_topology."""

import copy

from dcist_launch_system.fleet_helpers import load_topology, validate_topology


def test_sample_topology_is_valid(sample_topology):
    assert validate_topology(sample_topology) == []


def test_duplicate_address_on_same_network(sample_topology):
    topo = copy.deepcopy(sample_topology)
    topo["machines"]["beta"]["addresses"]["silvus"] = "192.168.100.10"
    assert validate_topology(topo) == [
        "duplicate silvus address 192.168.100.10: alpha and beta"
    ]
    # the same address on different networks is fine
    topo["machines"]["beta"]["addresses"] = {"mit_wifi": "192.168.100.10"}
    assert validate_topology(topo) == []


def test_duplicate_robot_color(sample_topology):
    topo = copy.deepcopy(sample_topology)
    topo["machines"]["alpha"]["color"] = "#E6194B"
    assert validate_topology(topo) == []  # colors are optional
    topo["machines"]["beta"]["color"] = "#e6194b"
    (problem,) = validate_topology(topo)
    assert "duplicate robot color" in problem and "alpha and beta" in problem


def test_fleet_link_and_base(sample_topology):
    topo = copy.deepcopy(sample_topology)
    topo["fleet_defaults"] = {"fleet_link": "fleetlink", "fleetlink_base": "gamma"}
    topo["zenoh"].update(fleetlink_port=7450, fleetlink_networks=["silvus", "mit_wifi"])
    assert validate_topology(topo) == []

    topo["fleet_defaults"]["fleet_link"] = "radio"
    assert any("fleet_link must be one of" in p for p in validate_topology(topo))
    topo["fleet_defaults"]["fleet_link"] = "fleetlink"

    topo["fleet_defaults"]["fleetlink_base"] = "alpha"
    assert validate_topology(topo) == [
        "fleet_defaults.fleetlink_base 'alpha' is not a base_station"
    ]
    topo["fleet_defaults"]["fleetlink_base"] = "nobody"
    assert validate_topology(topo) == [
        "fleet_defaults.fleetlink_base 'nobody' is not a machine"
    ]
    del topo["fleet_defaults"]["fleetlink_base"]
    assert validate_topology(topo) == [
        "fleet_link is 'fleetlink' but fleetlink_base is not set"
    ]


def test_fleetlink_port_and_networks(sample_topology):
    topo = copy.deepcopy(sample_topology)
    topo["zenoh"]["fleetlink_port"] = "7450"
    topo["zenoh"]["fleetlink_networks"] = ["silvus", "doodle"]
    problems = validate_topology(topo)
    assert "zenoh.fleetlink_port must be an int, got '7450'" in problems
    assert "zenoh.fleetlink_networks: unknown network 'doodle'" in problems


def test_real_topology(topology_path):
    """The shipped topology.yaml.  The lewis/sequia silvus clash is a known data
    problem (pending the correct address); once it is fixed this test fails and
    the expected list below should become []."""
    problems = validate_topology(load_topology(topology_path))
    assert problems == ["duplicate silvus address 192.168.100.105: lewis and sequia"]
