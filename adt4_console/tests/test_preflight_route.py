"""POST /api/launch/preflight and the preflight NO-GO confirmation of a launch."""

import time

from .conftest import make_client


def _out(*checks, head="a" * 40):
    lines = [f"@@C|{n}|{s}|{d}" for n, s, d in checks]
    return "\n".join(lines + [f"@@V|{head}|0|s1|main", f"@@T|{time.time()}"])


class PreflightSSH:
    """Preflight scripts answer from ``by_needle`` (first needle found in the script);
    a tmux session exists once run-adt4 ran; everything else succeeds."""

    def __init__(self, by_needle=(), default=None):
        self.by_needle = list(by_needle)
        self.default = default if default is not None else _out()
        self.calls = []
        self.launched = False

    def __call__(self, user, ip, cmd, timeout=5):
        self.calls.append(cmd)
        if "@@T" in cmd:
            for needle, out in self.by_needle:
                if needle in cmd:
                    return 0, out, ""
            return 0, self.default, ""
        if "run-adt4" in cmd:
            self.launched = True
        if "tmux has-session" in cmd:
            return (0, "OK", "") if self.launched else (1, "", "")
        return 0, "", ""


def _deps(ssh):
    from dcist_launch_system.fleet.launch_workflow import LaunchDeps

    return LaunchDeps(ssh=ssh, sleep=lambda s: None)


def test_preflight_route_reports_checks_and_verdicts(settings):
    ssh = PreflightSSH([("hardware_launch/config/gauss/ouster.yaml", _out(("ouster", "nogo", "Ouster does not answer ping at 192.168.92.99")))])
    client, _ = make_client(settings, launch_deps=_deps(ssh))
    with client:
        r = client.post("/api/launch/preflight", json={"robots": ["euclid", "gauss"], "session": "spot-default", "experiment": "e"})
    assert r.status_code == 200, r.text
    data = r.json()
    assert data["verdicts"] == {"euclid": "go", "gauss": "nogo"}
    assert data["nogo"] == 1
    [ouster] = [c for c in data["checks"] if c["name"] == "ouster"]
    assert ouster["machine"] == "gauss" and ouster["hint"]
    # the robots got their own platform's checks
    assert any("spot_reach" in c for c in ssh.calls) and any("clearpath-platform" in c for c in ssh.calls)


def test_preflight_route_unknown_robot(settings):
    client, _ = make_client(settings, launch_deps=_deps(PreflightSSH()))
    with client:
        r = client.post("/api/launch/preflight", json={"robots": ["nope"], "session": "s", "experiment": "e"})
    assert r.status_code == 400


def _wait_confirm(client, job_id, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        job = client.get(f"/api/jobs/{job_id}").json()
        if job["confirm"] or job["state"] not in ("running", "waiting"):
            return job
        time.sleep(0.02)
    raise AssertionError("no confirmation requested")


def test_launch_nogo_needs_confirmation_naming_the_issue(settings):
    ssh = PreflightSSH(default=_out(("spot_reach", "nogo", "Spot does not answer ping at 10.0.0.3")))
    client, svc = make_client(settings, launch_deps=_deps(ssh))
    with client:
        job_id = client.post("/api/launch", json={"robots": ["euclid"], "session": "spot-default", "experiment": "e"}).json()["job_id"]
        job = _wait_confirm(client, job_id)
        conf = job["confirm"]
        assert conf["kind"] == "preflight_nogo" and conf["yes_label"] == "Launch anyway"
        assert [(i["machine"], i["name"]) for i in conf["detail"]["issues"]] == [("euclid", "spot_reach")]
        assert "Spot does not answer ping at 10.0.0.3" in conf["message"]
        client.post(f"/api/jobs/{job_id}/confirm", json={"answer": False})
        job = svc.jobs.wait(job_id).to_dict()
    assert job["state"] == "failed"
    assert not any("run-adt4" in c for c in ssh.calls)
