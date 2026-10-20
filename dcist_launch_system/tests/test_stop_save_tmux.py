"""Stop & save tmux fallback: finds the core window's shutdown pane, retypes the
hydra/roman shutdown command and verifies that the service request was made."""

from unittest.mock import patch

from dcist_launch_system import fleet_helpers as fh

START_SHUTDOWN = (
    "zsh -ic 'source ${ADT4_WS}/install/setup.zsh; ADT4_PANE_CMD='\"'\"'ros2 service call "
    "/${ADT4_ROBOT_NAME}/hydra/shutdown std_srvs/Empty && ros2 service call "
    "/${ADT4_ROBOT_NAME}/roman/shutdown std_srvs/Empty'\"'\"' ADT4_PANE_PREFILL=1 exec zsh'"
)
START_ENV = "zsh -ic 'python3 show_environment.py; exec zsh'"


class FakeTmux:
    """ssh_cmd stand-in for one machine's tmux."""

    def __init__(
        self, panes, on_enter="requester: making request: std_srvs.srv.Empty_Request()"
    ):
        self.panes = panes  # idx -> dict(cur, start, text)
        self.on_enter = on_enter
        self.cmds = []

    def __call__(self, user, ip, cmd, timeout=5):
        self.cmds.append(cmd)
        if cmd.startswith("tmux list-panes"):
            if not self.panes:
                return 1, "", "can't find window: core"
            return (
                0,
                "\n".join(
                    f"{i}\t{p['cur']}\t{p['start']}" for i, p in self.panes.items()
                ),
                "",
            )
        if cmd.startswith("tmux capture-pane"):
            idx = cmd.rsplit(".", 1)[-1].strip("'")
            return 0, self.panes[idx]["text"], ""
        if "Enter" in cmd and "send-keys" in cmd:
            idx = cmd.split("-t ", 1)[1].split()[0].rsplit(".", 1)[-1].strip("'")
            if self.on_enter:
                self.panes[idx]["text"] += "\n" + self.on_enter
            return 0, "", ""
        return 0, "", ""


def _panes(start1=START_SHUTDOWN, cur1="zsh", text1="> ros2 service call ..."):
    return {
        "0": {"cur": "zsh", "start": START_ENV, "text": "environment"},
        "1": {"cur": cur1, "start": start1, "text": text1},
    }


def _run(fake, **kw):
    with patch.object(fh, "ssh_cmd", fake):
        return fh.stop_save_via_tmux(
            "swarm", "10.0.0.1", robot="hamilton", sleep=lambda s: None, **kw
        )


def test_shutdown_command_runs_hydra_then_roman():
    cmd = fh.shutdown_command("hamilton")
    assert cmd == (
        "ros2 service call /hamilton/hydra/shutdown std_srvs/srv/Empty && "
        "ros2 service call /hamilton/roman/shutdown std_srvs/srv/Empty"
    )


def test_finds_pane_by_start_command_and_verifies():
    fake = FakeTmux(_panes())
    ok, msg = _run(fake)
    assert ok, msg
    assert "core.1" in msg
    sent = [c for c in fake.cmds if "send-keys" in c]
    assert len(sent) == 1
    assert "adt4_system:core.1" in sent[0] and "C-u" in sent[0]
    assert (
        "/hamilton/hydra/shutdown" in sent[0] and "/hamilton/roman/shutdown" in sent[0]
    )


def test_finds_pane_by_content_when_start_command_is_opaque():
    panes = _panes(start1="zsh")
    panes["1"]["text"] = "> ros2 service call /hamilton/hydra/shutdown std_srvs/Empty"
    panes["0"]["start"] = "zsh"
    ok, _ = _run(FakeTmux(panes))
    assert ok


def test_falls_back_to_pane_1():
    panes = _panes(start1="zsh", text1="")
    panes["0"]["start"] = "zsh"
    fake = FakeTmux(panes)
    ok, msg = _run(fake)
    assert ok and "core.1" in msg


def test_busy_pane_is_interrupted_first():
    fake = FakeTmux(_panes(cur1="ros2"))
    ok, _ = _run(fake)
    assert ok
    assert any(c.endswith("C-c") for c in fake.cmds)


def test_service_unavailable_is_reported():
    fake = FakeTmux(_panes(), on_enter="waiting for service to become available...")
    ok, msg = _run(fake)
    assert not ok and "not available" in msg


def test_no_request_seen_fails():
    fake = FakeTmux(_panes(), on_enter="")
    ok, msg = _run(fake, verify_timeout=0.0)
    assert not ok and "no service request" in msg


def test_missing_window():
    ok, msg = _run(FakeTmux({}))
    assert not ok and "no tmux window" in msg
