"""fleetlink status: the receiver exists at once, but the console reports
``waiting`` until the zenoh session is open (no router yet), then ``connected``."""

import dataclasses
import socket
import threading

import pytest

from .conftest import make_client
from .test_live_plan import FakeReceiver


class OpeningTransport:
    """The bits of ``ZenohTransport`` the console looks at."""

    def __init__(self):
        self.endpoints = ["tcp/10.0.0.1:7450", "tcp/10.1.0.1:7450"]
        self.opened = threading.Event()
        self.open_error = "ZError: Unable to connect"


def test_disabled(client):
    h = client.get("/api/health").json()
    assert h["fleetlink"] is False and h["fleetlink_state"] == "disabled"
    assert h["fleetlink_error"] is None
    assert client.get("/api/live").json()["transport"]["state"] == "disabled"


def test_waiting_then_connected(settings):
    rx = FakeReceiver()
    rx.transport = OpeningTransport()
    client, svc = make_client(settings, receiver=rx)
    with client:
        h = client.get("/api/health").json()
        assert h["fleetlink"] is False and h["fleetlink_state"] == "waiting"
        assert "waiting for a zenoh router at tcp/10.0.0.1:7450" in h["fleetlink_error"]
        tr = client.get("/api/live").json()["transport"]
        assert tr["state"] == "waiting" and tr["endpoints"] == rx.transport.endpoints
        rx.transport.opened.set()
        h = client.get("/api/health").json()
        assert h["fleetlink"] is True and h["fleetlink_state"] == "connected"
        assert h["fleetlink_error"] is None
        assert client.get("/api/live").json()["transport"]["state"] == "connected"


def test_real_zenoh_without_router_waits(settings):
    """``--fleetlink`` with nothing listening: no exception, no retry thread in
    the console, just ``waiting`` (ZenohTransport retries in the background)."""
    pytest.importorskip("zenoh")
    with socket.socket() as s:  # a port nothing listens on
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    st = dataclasses.replace(
        settings, fleetlink=True, zenoh_connect=f"tcp/127.0.0.1:{port}"
    )
    client, svc = make_client(st)
    with client:
        assert svc.live.receiver is not None and svc.live.error is None
        h = client.get("/api/health").json()
        assert h["fleetlink_state"] == "waiting" and h["fleetlink"] is False
        assert not any(t.name == "fleetlink-connect" for t in threading.enumerate())
