import time

from ruamel.yaml import YAML

from .conftest import make_client


class FakeSSH:
    """Scripted ``ssh_cmd(user, ip, cmd, timeout)``: first matching rule wins."""

    def __init__(self, rules):
        self.rules = rules  # [(substring, (rc, out, err))]
        self.calls = []

    def __call__(self, user, ip, cmd, timeout=5):
        self.calls.append(cmd)
        for needle, result in self.rules:
            if needle in cmd:
                return result
        return 0, "", ""

    def ran(self, needle):
        return [c for c in self.calls if needle in c]


def _launch_deps(ssh):
    from dcist_launch_system.fleet.launch_workflow import LaunchDeps

    return LaunchDeps(ssh=ssh, sleep=lambda s: None)


def _wait_confirm(client, job_id, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        job = client.get(f"/api/jobs/{job_id}").json()
        if job["confirm"] or job["state"] not in ("running", "waiting"):
            return job
        time.sleep(0.02)
    raise AssertionError("no confirmation requested")


PLAN = {"robots": ["euclid"], "session": "spot-default", "experiment": "exp1"}


def test_launch_kill_session_confirm_then_launch(settings):
    ssh = FakeSSH([("tmux has-session", (0, "", ""))])  # session already running
    client, svc = make_client(settings, launch_deps=_launch_deps(ssh))
    with client:
        job_id = client.post("/api/launch", json=PLAN).json()["job_id"]
        job = _wait_confirm(client, job_id)
        assert job["confirm"]["kind"] == "kill_sessions"
        assert job["confirm"]["detail"] == {"machines": ["euclid"]}
        r = client.post(f"/api/jobs/{job_id}/confirm", json={"answer": True})
        assert r.status_code == 200
        job = svc.jobs.wait(job_id).to_dict()
    assert job["state"] == "succeeded", job["log"]
    assert ssh.ran("tmux kill-session -t adt4_system")
    (launch_cmd,) = ssh.ran("run-adt4")
    assert (
        "spot-default" in launch_cmd
        and "-n euclid" in launch_cmd
        and "exp1" in launch_cmd
    )
    assert any(
        line["level"] == "ok" and "launched" in line["text"] for line in job["log"]
    )


def test_launch_large_overwrite_needs_typed_confirm(settings):
    ssh = FakeSSH(
        [
            ("tmux has-session", (1, "", "")),
            ("echo EXISTS", (0, "EXISTS", "")),
            ("du -sb", (0, str(3 * 1024**3), "")),
        ]
    )
    client, svc = make_client(settings, launch_deps=_launch_deps(ssh))
    with client:
        job_id = client.post("/api/launch", json=PLAN).json()["job_id"]
        job = _wait_confirm(client, job_id)
        assert job["confirm"]["required_text"] == "confirm"
        client.post(f"/api/jobs/{job_id}/confirm", json={"answer": True, "text": "yes"})
        job = svc.jobs.wait(job_id).to_dict()
    assert job["state"] == "failed"
    assert not ssh.ran("rm -rf") and not ssh.ran("run-adt4")


def test_launch_validation_and_dry_run(client):
    r = client.post("/api/launch", json={**PLAN, "experiment": ""})
    assert r.status_code == 400 and "experiment" in r.json()["detail"]
    r = client.post("/api/launch/dry_run", json={**PLAN, "robots": ["ghost"]})
    assert r.status_code == 400
    lines = client.post("/api/launch/dry_run", json=PLAN).json()["lines"]
    assert (
        len(lines) == 1 and lines[0].startswith("ssh swarm@") and "run-adt4" in lines[0]
    )
    d = client.get("/api/launch/defaults", params={"mode": "mapping"}).json()
    assert "spot-default" in d["sessions"] and "euclid" in d["robots"]


def test_topology_edit_delete_save_preserves_file(client, topology_copy):
    topology_copy.write_text(
        topology_copy.read_text().replace(
            "machines:\n", "machines:  # fleet machines\n", 1
        )
    )
    client.app.state.services.fleet.reload()
    edit = {
        "name": "euclid",
        "role": "robot",
        "platform": "smaug",
        "desc": "Edited desc",
        "mit_wifi": "10.29.1.2",
        "silvus": "192.168.100.3",
    }
    r = client.put("/api/topology/machines/euclid", json=edit)
    assert r.status_code == 200
    m = r.json()["machine"]
    assert m["addresses"]["mit_wifi"] == "10.29.1.2"
    assert m["addresses"]["penn_wifi"] == "192.168.129.234"  # untouched network kept
    assert m["platform_type"] == "spot"  # extra key kept
    fleet = {x["name"]: x for x in client.get("/api/fleet").json()["machines"]}
    assert fleet["euclid"]["ip"] == "10.29.1.2"

    # new machine, then delete another
    r = client.put("/api/topology/machines/newbot", json={**edit, "name": "newbot"})
    assert r.json()["name"] == "newbot"
    assert client.delete("/api/topology/machines/gauss").status_code == 200
    assert client.delete("/api/topology/machines/gauss").status_code == 404
    assert client.post("/api/topology/save").json()["ok"]

    saved = YAML().load(topology_copy.read_text())
    assert saved["machines"]["euclid"]["desc"] == "Edited desc"
    assert saved["machines"]["euclid"]["addresses"]["mit_wifi"] == "10.29.1.2"
    assert "newbot" in saved["machines"] and "gauss" not in saved["machines"]
    assert list(saved)[:2] == ["networks", "machines"]  # key order preserved
    assert "machines:  # fleet machines" in topology_copy.read_text()  # comment kept
