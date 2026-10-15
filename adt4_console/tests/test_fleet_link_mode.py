"""fleet_link mode in the console: receiver router default and /api/network/zenoh info."""

from adt4_console.cli import default_zenoh_connect, parse_args, settings_from_args
from adt4_console.routes.network import _fleet_link

TOPO = {
    "machines": {
        "r1": {"role": "robot", "addresses": {"silvus": "192.168.100.2"}},
        "base": {
            "role": "base_station",
            "addresses": {"silvus": "192.168.100.100", "mit_wifi": "10.29.1.3"},
        },
    },
    "zenoh": {"port": 7447, "fleetlink_port": 7450,
              "fleetlink_networks": ["silvus", "mit_wifi"]},
    "fleet_defaults": {"fleet_link": "fleetlink", "fleetlink_base": "base"},
}  # fmt: skip


def test_default_zenoh_connect():
    assert default_zenoh_connect(environ={}) == "tcp/127.0.0.1:7447"
    assert default_zenoh_connect(environ={"ADT4_FLEET_LINK": "router"}) == (
        "tcp/127.0.0.1:7447"
    )
    assert (
        default_zenoh_connect(
            environ={"ADT4_FLEET_LINK": "fleetlink", "ADT4_FLEETLINK_PORT": "17450"}
        )
        == "tcp/127.0.0.1:17450"
    )
    # port from topology.yaml when not exported (repo topology: 7450)
    assert default_zenoh_connect(environ={"ADT4_FLEET_LINK": "fleetlink"}) == (
        "tcp/127.0.0.1:7450"
    )


def test_cli_explicit_zenoh_connect_wins(monkeypatch):
    monkeypatch.setenv("ADT4_FLEET_LINK", "fleetlink")
    monkeypatch.setenv("ADT4_FLEETLINK_PORT", "7450")
    s = settings_from_args(parse_args(["--zenoh-connect", "tcp/1.2.3.4:9"]))
    assert s.zenoh_connect == "tcp/1.2.3.4:9"
    assert settings_from_args(parse_args([])).zenoh_connect == "tcp/127.0.0.1:7450"
    monkeypatch.delenv("ADT4_FLEET_LINK")
    assert settings_from_args(parse_args([])).zenoh_connect == "tcp/127.0.0.1:7447"


def test_network_fleet_link_info():
    info = _fleet_link(TOPO)
    assert info["fleet_link"] == "fleetlink"
    assert info["fleetlink"] == {
        "base": "base",
        "port": 7450,
        "endpoints": ["tcp/192.168.100.100:7450", "tcp/10.29.1.3:7450"],
        "summary": "fleetlink client → base:7450",
    }
    router = {**TOPO, "fleet_defaults": {"fleet_link": "router"}}
    assert _fleet_link(router) == {"fleet_link": "router"}
