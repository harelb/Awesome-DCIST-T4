"""Fleet machine terminals and tmux session control.

A terminal is ``ssh -tt user@ip`` (or a local login shell for the console's own
machine) on a PTY, optionally running ``tmux attach -t <session>``.  Closing the
browser tab hangs up the PTY, which only detaches an attached tmux client.
"""

from __future__ import annotations

import fcntl
import os
import pty
import re
import shlex
import shutil
import signal
import struct
import subprocess
import termios
import time

SESSION_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,64}$")

SSH_OPTS = [
    "-o",
    "ConnectTimeout=5",
    "-o",
    "BatchMode=yes",
    "-o",
    "StrictHostKeyChecking=no",
    "-o",
    "ServerAliveInterval=10",
]


def valid_session(name: str | None) -> bool:
    return bool(name) and SESSION_RE.match(name) is not None


def terminal_argv(machine: dict, session: str | None = None) -> list[str]:
    """Command that opens a shell (or attaches ``session``) on ``machine``."""
    if session is not None and not valid_session(session):
        raise ValueError(f"bad tmux session name {session!r}")
    attach = ["tmux", "attach-session", "-t", session] if session else None
    if machine.get("local"):
        if attach:
            return attach
        return [shutil.which("zsh") or shutil.which("bash") or "sh", "-l"]
    argv = ["ssh", "-tt", *SSH_OPTS, f"{machine['user']}@{machine['ip']}"]
    if attach:
        argv.append(shlex.join(attach))
    return argv


def session_command(action: str, session: str) -> str:
    """Shell command for a tmux session action: ``stop`` (Ctrl-C every pane) or ``kill``."""
    if not valid_session(session):
        raise ValueError(f"bad tmux session name {session!r}")
    q = shlex.quote(session)
    if action == "stop":
        return (
            f"tmux has-session -t {q} && "
            f"for p in $(tmux list-panes -s -t {q} -F '#{{pane_id}}'); "
            "do tmux send-keys -t $p C-c; done"
        )
    if action == "kill":
        return f"tmux kill-session -t {q}"
    raise ValueError(f"unknown action {action!r}")


SHELLS = {"zsh", "bash", "sh", "fish", "dash"}


def stop_session(ssh, user: str, ip: str, session: str, log, *, wait_s: float, poll_s: float = 1.0) -> bool:
    """Ctrl-C every pane of ``session``, wait up to ``wait_s`` for the panes to
    drop back to their shells, then kill the session.  ``ssh`` is
    ``helpers.ssh_cmd``, ``log(level, text)`` reports progress.  True once the
    session is gone (or was never there)."""
    q = shlex.quote(session)
    rc, out, err = ssh(user, ip, session_command("stop", session), timeout=10)
    if rc != 0:
        log("dim", f"{ip}: no {session} session to stop ({err or out or rc})")
        return True
    log("info", f"Sent Ctrl-C to every pane of {session}; waiting up to {wait_s:.0f} s for them to exit ...")
    busy: list[str] = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        time.sleep(poll_s)
        rc, out, _ = ssh(user, ip, f"tmux list-panes -s -t {q} -F '#{{pane_current_command}}'")
        busy = [c for c in out.split() if c not in SHELLS] if rc == 0 else []
        if not busy:
            log("ok", f"{session}: every pane exited after {time.monotonic() - t0:.0f} s")
            break
    else:
        log("warn", f"{session}: still running after {wait_s:.0f} s ({', '.join(sorted(set(busy)))}); killing anyway")
    rc, out, err = ssh(user, ip, session_command("kill", session), timeout=10)
    if rc != 0 and "can't find session" not in (err or out):
        log("error", f"kill {session} failed: {err or out or rc}")
        return False
    log("ok", f"{session} killed")
    return True


class Pty:
    """A child process on a pseudo-terminal."""

    def __init__(self, argv: list[str], cols: int = 80, rows: int = 24):
        self.fd, slave = pty.openpty()
        self.resize(cols, rows)
        env = dict(os.environ, TERM="xterm-256color")
        try:
            self.proc = subprocess.Popen(
                argv,
                stdin=slave,
                stdout=slave,
                stderr=slave,
                env=env,
                cwd=os.path.expanduser("~"),
                start_new_session=True,
                close_fds=True,
            )
        except OSError:
            os.close(self.fd)
            raise
        finally:
            os.close(slave)
        os.set_blocking(self.fd, False)

    def resize(self, cols: int, rows: int) -> None:
        cols = max(1, min(int(cols), 1000))
        rows = max(1, min(int(rows), 1000))
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))

    def read(self) -> bytes | None:
        """Available output: None if nothing yet, b"" at EOF (child exited)."""
        try:
            return os.read(self.fd, 65536)
        except BlockingIOError:
            return None
        except OSError:  # EIO once the child side is closed
            return b""

    def write(self, data: bytes) -> None:
        while data:
            try:
                n = os.write(self.fd, data)
            except BlockingIOError:
                continue
            data = data[n:]

    def close(self) -> None:
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGHUP)
                self.proc.wait(timeout=2)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        try:
            os.close(self.fd)
        except OSError:
            pass
