import pathlib
import shutil
import socket
import sys

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
REPO = PKG.parent
for p in (PKG, REPO / "fleetlink", REPO / "dcist_launch_system" / "src"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from adt4_console.app import build_services, create_app  # noqa: E402
from adt4_console.fleet_service import FleetDeps  # noqa: E402
from adt4_console.settings import Settings  # noqa: E402

TOPOLOGY = REPO / "network_diagnostics" / "config" / "topology.yaml"
PENN = pathlib.Path("/home/harel/data/pennovation_2025/courtyard-alleyways/output")
NEO4J_URI = "neo4j://localhost:7697"
NEO4J_AUTH = ("neo4j", "neo4j_pw")


def port_open(host: str, port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def fake_fleet_deps(**over) -> FleetDeps:
    """Real topology parsing, no ping/ssh: every machine is online."""

    def filter_reachable(machines, **_):
        for m in machines:
            m.update(ping=True, ssh=True, online=True)
        return machines, []

    kw = dict(
        detect_network=lambda topo: "mit_wifi",
        filter_reachable=filter_reachable,
        get_remote_status=lambda user, ip, **_: {
            "tmux_sessions": "none",
            "load": "0/0/0",
        },
        get_local_ips=lambda: ["127.0.0.1"],
    )
    kw.update(over)
    return FleetDeps(**kw)


@pytest.fixture
def topology_copy(tmp_path):
    dst = tmp_path / "topology.yaml"
    shutil.copy(TOPOLOGY, dst)
    return dst


@pytest.fixture
def settings(tmp_path, topology_copy):
    return Settings(
        topology=str(topology_copy),
        network="mit_wifi",
        output_root=str(tmp_path / "out"),
        ros=False,
        fleetlink=False,
        fleet_poll=False,
        llm=False,
        static_dir=tmp_path / "static",
        confirm_timeout_s=5.0,
    )


def make_client(settings, **service_kw):
    from fastapi.testclient import TestClient

    service_kw.setdefault("fleet_deps", fake_fleet_deps())
    services = build_services(settings, **service_kw)
    services.fleet.refresh_reachability()
    return TestClient(create_app(settings, services)), services


@pytest.fixture
def client(settings):
    c, services = make_client(settings)
    with c:
        yield c
