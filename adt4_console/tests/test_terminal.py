"""Fleet terminals and tmux session stop/kill."""

import subprocess
import time

import pytest
from starlette.websockets import WebSocketDisconnect

from adt4_console import terminal as term

from .conftest import make_client


class FakeHelpers:
    def __init__(self, rc=0):
        self.rc = rc
        self.calls = []

    def ssh_cmd(self, user, ip, cmd, timeout=5):
        self.calls.append((user, ip, cmd))
        return self.rc, "", "boom" if self.rc else ""

    def get_local_ips(self):
        return ["127.0.0.1"]


@pytest.fixture
def app(settings):
    client, svc = make_client(settings)
    svc.helpers = FakeHelpers()
    with client:
        yield client, svc


def _machine(svc):
    return next(iter(svc.fleet.runtime_config.values()))


def test_session_names_are_validated():
    assert term.valid_session("adt4_system")
    for bad in ("", "a b", "x;rm -rf ~", "$(id)", "a" * 65):
        assert not term.valid_session(bad)
        with pytest.raises(ValueError):
            term.session_command("kill", bad)


def test_session_commands_run_in_a_shell():
    assert term.session_command("kill", "adt4_system") == "tmux kill-session -t adt4_system"
    stop = term.session_command("stop", "adt4_system")
    # Ctrl-C goes to every pane of every window; a real shell must accept it
    assert subprocess.run(["bash", "-n", "-c", stop]).returncode == 0
    assert "list-panes -s -t adt4_system" in stop and "C-c" in stop


def test_terminal_argv():
    remote = {"user": "swarm", "ip": "10.0.0.2"}
    argv = term.terminal_argv(remote)
    assert argv[:2] == ["ssh", "-tt"] and argv[-1] == "swarm@10.0.0.2"
    assert term.terminal_argv(remote, "adt4_system")[-1] == "tmux attach-session -t adt4_system"
    assert term.terminal_argv({"local": True}, "s1") == ["tmux", "attach-session", "-t", "s1"]
    assert term.terminal_argv({"local": True})[-1] == "-l"
    with pytest.raises(ValueError):
        term.terminal_argv(remote, "a;b")


def test_tmux_routes(app):
    client, svc = app
    m = _machine(svc)
    r = client.post(f"/api/fleet/{m['name']}/tmux/adt4_system/kill")
    assert r.status_code == 200, r.text
    assert svc.helpers.calls == [(m["user"], m["ip"], "tmux kill-session -t adt4_system")]
    # stop: a job that sends Ctrl-C, waits for the panes to exit, then kills
    svc.helpers.calls.clear()
    r = client.post(f"/api/fleet/{m['name']}/tmux/adt4_system/stop")
    assert r.status_code == 200, r.text
    job = svc.jobs.wait(r.json()["job_id"], timeout=10).to_dict()
    assert job["state"] == "succeeded", job["log"]
    cmds = [c[2] for c in svc.helpers.calls]
    assert "C-c" in cmds[0] and "list-panes" in cmds[1] and cmds[-1] == "tmux kill-session -t adt4_system"
    assert "adt4_system killed" in " | ".join(line["text"] for line in job["log"])

    assert client.post(f"/api/fleet/{m['name']}/tmux/adt4_system/reboot").status_code == 404
    assert client.post(f"/api/fleet/{m['name']}/tmux/a%3Bb/kill").status_code == 400
    assert client.post("/api/fleet/nope/tmux/adt4_system/kill").status_code == 404
    svc.helpers.rc = 1
    r = client.post(f"/api/fleet/{m['name']}/tmux/adt4_system/kill")
    assert r.status_code == 502 and "boom" in r.json()["detail"]


def test_terminal_is_off_by_default(app):
    client, svc = app
    m = _machine(svc)
    assert client.get("/api/config").json()["terminal"] is False
    with pytest.raises(WebSocketDisconnect) as e:
        with client.websocket_connect(f"/api/fleet/{m['name']}/terminal") as ws:
            ws.receive_bytes()
    assert e.value.code == 4403


def _read_until(ws, needle: bytes, limit=50) -> bytes:
    buf = b""
    for _ in range(limit):
        buf += ws.receive_bytes()
        if needle in buf:
            return buf
    raise AssertionError(f"{needle!r} not in {buf!r}")


def test_terminal_round_trip(settings, monkeypatch):
    settings.terminal = True
    client, svc = make_client(settings)
    m = _machine(svc)
    # a local shell instead of ssh; the PTY, resize and EOF handling are real
    monkeypatch.setattr(term, "terminal_argv", lambda machine, session=None: ["sh"])
    with client:
        with client.websocket_connect(f"/api/fleet/{m['name']}/terminal?cols=100&rows=30") as ws:
            ws.send_bytes(b"stty size; echo hi-$((40+2))\n")
            out = _read_until(ws, b"hi-42")
            assert b"30 100" in out
            ws.send_text('{"type": "resize", "cols": 120, "rows": 40}')
            time.sleep(0.1)
            ws.send_bytes(b"stty size\n")
            assert b"40 120" in _read_until(ws, b"40 120")
            ws.send_bytes(b"exit\n")
            with pytest.raises(WebSocketDisconnect):
                for _ in range(50):
                    ws.receive_bytes()

    with client:
        with pytest.raises(WebSocketDisconnect) as e:
            with client.websocket_connect(f"/api/fleet/{m['name']}/terminal?session=a;b") as ws:
                ws.receive_bytes()
        assert e.value.code == 4404


class PaneHelpers:
    """ssh_cmd fake: the panes run ``busy`` commands for ``busy_polls`` polls."""

    def __init__(self, busy="hydra_ros_node ros2", busy_polls=1, stop_rc=0):
        self.busy, self.busy_polls, self.stop_rc = busy, busy_polls, stop_rc
        self.cmds = []

    def ssh_cmd(self, user, ip, cmd, timeout=5):
        self.cmds.append(cmd)
        if "C-c" in cmd:
            return self.stop_rc, "", "no server running" if self.stop_rc else ""
        if "list-panes" in cmd:
            polls = sum("list-panes" in c and "C-c" not in c for c in self.cmds)
            return 0, (self.busy if polls <= self.busy_polls else "zsh zsh"), ""
        return 0, "", ""


def _stop(h, wait_s=1.0):
    log = []
    ok = term.stop_session(h.ssh_cmd, "u", "1.2.3.4", "adt4_system", lambda lvl, t: log.append((lvl, t)), wait_s=wait_s, poll_s=0.01)
    return ok, log


def test_stop_session_ctrl_c_waits_for_shells_then_kills():
    h = PaneHelpers(busy_polls=2)
    ok, log = _stop(h)
    assert ok
    assert "C-c" in h.cmds[0] and h.cmds[-1] == "tmux kill-session -t adt4_system"
    assert sum("list-panes" in c and "C-c" not in c for c in h.cmds) == 3  # 2 busy polls, then shells
    assert any("every pane exited" in t for _, t in log)


def test_stop_session_kills_anyway_after_the_timeout():
    h = PaneHelpers(busy_polls=10**6)
    t0 = time.monotonic()
    ok, log = _stop(h, wait_s=0.1)
    assert ok and time.monotonic() - t0 < 2
    assert h.cmds[-1] == "tmux kill-session -t adt4_system"
    assert any(lvl == "warn" and "killing anyway" in t and "hydra_ros_node" in t for lvl, t in log)


def test_stop_session_without_a_session_is_a_noop():
    h = PaneHelpers(stop_rc=1)
    ok, _ = _stop(h)
    assert ok and len(h.cmds) == 1
