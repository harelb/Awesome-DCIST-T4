"""Stop & save (hydra/shutdown then roman/shutdown, tmux fallback) and the
Take-lease route + lease state from the executor's runtime guards."""

import time
import types

import pytest

from adt4_console.routes import launch as launch_routes
from adt4_console.ros_bridge import RosBridge
from tests.conftest import make_client


class FakeHelpers:
    """stop_save_via_tmux plus a robot whose hydra exits after ``hydra_polls``
    checks and whose panes are shells once Ctrl-C was sent."""

    def __init__(self, result=(True, "shutdown requested from pane core.1"), hydra_polls=1):
        self.result = result
        self.calls = []
        self.ssh = []
        self.hydra_polls = hydra_polls
        self.stopped = False

    def stop_save_via_tmux(self, user, ip, robot=None):
        self.calls.append((user, ip, robot))
        return self.result

    def ssh_cmd(self, user, ip, cmd, timeout=5):
        self.ssh.append(cmd)
        if cmd.startswith("pgrep -f hydra_ros_node"):
            self.hydra_polls -= 1
            return 0, "up" if self.hydra_polls > 0 else "gone", ""
        if "send-keys" in cmd:
            self.stopped = True
            return 0, "", ""
        if "list-panes" in cmd:
            return 0, "zsh\nzsh" if self.stopped else "hydra_ros_node\nzsh", ""
        return 0, "", ""


def _fake_ros(svc, monkeypatch, fail=(), link=None, services_up=()):
    calls = []
    up = set(services_up)

    def call_service(name, srv_type, fields, timeout=10.0):
        calls.append((name, srv_type, timeout))
        if name in fail:
            raise TimeoutError(f"service {name} not available")
        if name.endswith("/hydra/shutdown"):
            up.discard(name)  # hydra exits after saving
        return {}

    monkeypatch.setattr(svc.ros, "available", True)
    monkeypatch.setattr(svc.ros, "call_service", call_service)
    monkeypatch.setattr(svc.ros, "link_status", lambda robot: link)
    monkeypatch.setattr(svc.ros, "has_service", lambda name: name in up)
    return calls


def _job(client, svc, job_id):
    job = svc.jobs.wait(job_id, timeout=10).to_dict()
    return job, " | ".join(line["text"] for line in job["log"])


@pytest.fixture
def app(settings, monkeypatch):
    monkeypatch.setattr(launch_routes, "HYDRA_EXIT_POLL_S", 0.01)
    helpers = FakeHelpers()
    client, svc = make_client(settings)
    svc.helpers = helpers
    with client:
        yield client, svc, helpers


def test_stop_save_calls_hydra_then_roman(app, monkeypatch):
    client, svc, helpers = app
    calls = _fake_ros(svc, monkeypatch, services_up={"/hamilton/hydra/shutdown"})
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["state"] == "succeeded", log
    assert [c[0] for c in calls] == [
        "/hamilton/hydra/shutdown",
        "/hamilton/roman/shutdown",
    ]
    assert all(c[1] == "std_srvs/srv/Empty" for c in calls)
    assert calls[1][2] >= 120  # roman saves before it answers
    assert job["result"] == {
        "method": "service",
        "hydra": True,
        "roman": True,
        "hydra_exited": True,
        "teardown": True,
    }
    assert "ROMAN map saved" in log and "hydra saved and exited" in log
    assert not helpers.calls
    # then the rest of the stack: Ctrl-C every pane, wait for shells, kill
    assert "C-c" in helpers.ssh[0] and helpers.ssh[-1] == "tmux kill-session -t adt4_system"
    assert "adt4_system stopped and closed" in log


def test_stop_save_roman_failure_is_a_warning(app, monkeypatch):
    client, svc, helpers = app
    _fake_ros(svc, monkeypatch, fail={"/hamilton/roman/shutdown"})
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["state"] == "succeeded", log
    assert job["result"]["roman"] is False
    assert "roman/shutdown failed" in log
    assert not helpers.calls


def test_stop_save_fleetlink_skips_exit_wait(app, monkeypatch):
    client, svc, _ = app
    _fake_ros(
        svc,
        monkeypatch,
        link={"connected": True},
        services_up={"/hamilton/hydra/shutdown"},
    )
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["result"]["hydra_exited"] is None
    assert "fleetlink mode" in log


def test_stop_save_falls_back_to_tmux_when_hydra_service_fails(app, monkeypatch):
    client, svc, helpers = app
    calls = _fake_ros(svc, monkeypatch, fail={"/hamilton/hydra/shutdown"})
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["state"] == "succeeded", log
    assert [c[0] for c in calls] == [
        "/hamilton/hydra/shutdown"
    ]  # no roman before hydra
    assert helpers.calls and helpers.calls[0][2] == "hamilton"
    assert job["result"]["method"] == "tmux"


def test_stop_save_without_ros_uses_tmux(app):
    client, svc, helpers = app
    helpers.result = (False, "pane core.1: hydra/shutdown is not available")
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["state"] == "failed"
    assert "not available" in log


# ------------------------------------------------------------------ lease


def test_take_lease_route(app, monkeypatch):
    client, svc, _ = app
    calls = []

    def call_service(name, srv_type, fields, timeout=10.0):
        calls.append((name, srv_type))
        return {
            "success": True,
            "message": "lease owner before: bosdyn.android.spotapp x; after: understanding-h",
        }

    monkeypatch.setattr(svc.ros, "available", True)
    monkeypatch.setattr(svc.ros, "call_service", call_service)
    r = client.post("/api/robots/hamilton/take_lease")
    assert r.status_code == 200
    assert r.json()["ok"] is True and "before: bosdyn" in r.json()["message"]
    assert calls == [
        ("/hamilton/spot_executor_node/take_lease", "std_srvs/srv/Trigger")
    ]


def test_take_lease_relay_failure_and_timeout(app, monkeypatch):
    client, svc, _ = app
    monkeypatch.setattr(svc.ros, "available", True)
    # fleetlink relay failure: default (empty) Trigger response
    monkeypatch.setattr(
        svc.ros, "call_service", lambda *a, **k: {"success": False, "message": ""}
    )
    r = client.post("/api/robots/hamilton/take_lease").json()
    assert r["ok"] is False and "no answer" in r["message"]

    def timeout(*a, **k):
        raise TimeoutError(
            "service /hamilton/spot_executor_node/take_lease not available"
        )

    monkeypatch.setattr(svc.ros, "call_service", timeout)
    assert client.post("/api/robots/hamilton/take_lease").status_code == 504


def test_take_lease_without_ros(client):
    assert client.post("/api/robots/hamilton/take_lease").status_code == 503


def _guards(owned, owner=None, known=True):
    m = types.SimpleNamespace(lease_owned=owned, lease_known=known)
    if owner is not None:
        m.lease_owner = owner
    return m


def test_lease_state_from_guards_in_plan():
    published = []
    hub = types.SimpleNamespace(
        publish=lambda topic, payload: published.append((topic, payload))
    )
    rb = RosBridge(types.SimpleNamespace(ros=False, planner_ns="willow"), hub=hub)
    assert rb.plan("hamilton")["lease"] is None
    rb._on_guards("hamilton", _guards(False, "bosdyn.android.spotapp tablet"))
    lease = rb.plan("hamilton")["lease"]
    assert (
        lease["known"]
        and not lease["owned"]
        and lease["owner"] == "bosdyn.android.spotapp tablet"
    )
    assert "_pub" not in lease and time.time() - lease["t"] < 5
    assert (
        published[-1][0] == "robot/hamilton/plan"
        and "_pub" not in published[-1][1]["lease"]
    )
    n = len(published)
    rb._on_guards("hamilton", _guards(False, "bosdyn.android.spotapp tablet"))
    assert len(published) == n  # unchanged within LEASE_REPUBLISH_S: not re-sent
    rb._on_guards("hamilton", _guards(True, "understanding-hamilton"))
    assert len(published) == n + 1 and published[-1][1]["lease"]["owned"]
    # a robot built before lease_owner
    rb._on_guards("euclid", _guards(True))
    assert rb.plan("euclid")["lease"]["owner"] == ""


def test_stop_save_without_teardown_leaves_the_session(app, monkeypatch):
    client, svc, helpers = app
    _fake_ros(svc, monkeypatch, services_up={"/hamilton/hydra/shutdown"})
    r = client.post("/api/robots/hamilton/stop_save", json={"teardown": False})
    job, log = _job(client, svc, r.json()["job_id"])
    assert job["state"] == "succeeded", log
    assert "teardown" not in job["result"] and not helpers.ssh


def test_stop_save_tmux_waits_for_hydra_before_teardown(app):
    client, svc, helpers = app
    helpers.hydra_polls = 3  # hydra still saving for two checks
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["state"] == "succeeded", log
    pgreps = [c for c in helpers.ssh if c.startswith("pgrep")]
    assert len(pgreps) == 3
    # no Ctrl-C before hydra exited
    first_keys = next(i for i, c in enumerate(helpers.ssh) if "send-keys" in c)
    assert first_keys > helpers.ssh.index(pgreps[-1])
    assert job["result"]["teardown"] is True


def test_stop_save_never_kills_a_hydra_that_is_still_saving(app, monkeypatch):
    client, svc, helpers = app
    monkeypatch.setattr(launch_routes, "HYDRA_EXIT_WAIT_S", 0.05)
    helpers.hydra_polls = 10**6
    job, log = _job(
        client, svc, client.post("/api/robots/hamilton/stop_save").json()["job_id"]
    )
    assert job["result"]["teardown"] is False
    assert "hydra is still saving" in log
    assert not any("send-keys" in c or "kill-session" in c for c in helpers.ssh)
