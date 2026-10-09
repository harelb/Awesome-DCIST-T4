import os
import signal
import socket
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))
sys.path.insert(0, HERE)

ROUTER_PORT = 17448  # throwaway router; NEVER 7447 (the live system's router)
ROUTER_ENDPOINT = f"tcp/127.0.0.1:{ROUTER_PORT}"


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "integration: needs a zenoh router / ROS / real data"
    )


def _port_open(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) == 0


@pytest.fixture(scope="session")
def zenoh_router():
    """A throwaway rmw_zenohd on 127.0.0.1:17448 (killed at teardown)."""
    if not os.path.exists("/opt/ros/jazzy/setup.bash"):
        pytest.skip("ROS jazzy not installed (rmw_zenohd)")
    if _port_open(ROUTER_PORT):
        pytest.skip(f"port {ROUTER_PORT} already in use")
    override = (
        f'listen/endpoints=["{ROUTER_ENDPOINT}"];connect/endpoints=[];'
        "scouting/multicast/enabled=false;adminspace/enabled=false;plugins={}"
    )
    cmd = (
        "source /opt/ros/jazzy/setup.bash; "
        f"ZENOH_CONFIG_OVERRIDE='{override}' exec ros2 run rmw_zenoh_cpp rmw_zenohd"
    )
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    deadline = time.time() + 15.0
    while time.time() < deadline and not _port_open(ROUTER_PORT):
        if proc.poll() is not None:
            pytest.skip("rmw_zenohd exited during startup")
        time.sleep(0.1)
    if not _port_open(ROUTER_PORT):
        os.killpg(proc.pid, signal.SIGKILL)
        pytest.skip("rmw_zenohd did not start listening")
    yield ROUTER_ENDPOINT
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        os.killpg(proc.pid, signal.SIGKILL)
