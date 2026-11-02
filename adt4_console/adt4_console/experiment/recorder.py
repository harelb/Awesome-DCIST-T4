"""Per-machine rosbag recorders for an experiment run, over ssh (local on the
base station: ``fleet_helpers.ssh_cmd`` runs locally for a local address).

The topic list comes from the console's own ``dcist_launch_system/recording/
<profile>_topics.txt`` (the files ``run-adt4 -r`` uses), is expanded here for
the machine (``ADT4_ROBOT_NAME``, ``ADT4_EACH_ROBOT``) and written to the
machine next to the bag, together with a small record script, so a robot with
an older checkout records exactly what this console expects.

On the machine (``~`` = the ssh user's home)::

    ~/adt4_output/experiments/<exp>/<run>/bags/<seg>/            the bag (ros2 bag -o)
    ~/adt4_output/experiments/<exp>/<run>/bags/<seg>.topics.txt
    ~/adt4_output/experiments/<exp>/<run>/bags/<seg>.record.sh
    ~/adt4_output/experiments/<exp>/<run>/bags/<seg>.log

``<seg>`` is the machine name, then ``<machine>_2``, ... after a restart.  The
recorder runs in tmux session ``adt4_rec`` and is stopped with a SIGINT
(``tmux send-keys C-c``) so rosbag2 finalizes the bag (metadata.yaml); nothing
here ever stops it on its own.
"""

from __future__ import annotations

import logging
import pathlib
import shlex
import time

log = logging.getLogger(__name__)

SESSION = "adt4_rec"
REMOTE_ROOT = "adt4_output/experiments"  # relative to the machine's $HOME
START_TIMEOUT_S = 15.0
STOP_TIMEOUT_S = 40.0
# rosbag2 writes in bursts (cache flush, mcap chunks): the bag only counts as
# stalled when it has not grown for STALL_S *and* 3x the longest gap between
# writes seen so far; NO_DATA_S without any byte written at all -> stalled
STALL_S = 60.0
NO_DATA_S = 120.0
CACHE_BYTES = {"full": None, "light": 2 * 1024 * 1024, "base": 256 * 1024}
# the running stack's ROS environment, copied into the recorder
ROS_ENV_PATTERN = "^(ROS_|RMW_|ZENOH_|ADT4_|CYCLONEDDS_|FASTRTPS_|FASTDDS_)[A-Z0-9_]*="
STACK_SESSION = "adt4_system"
LOW_DISK_S = 15 * 60  # less recording time left than this -> warn
STORAGE = "mcap"

# robot / base-station recorder states
STATES = ("starting", "recording", "stalled", "unreachable", "stopping", "stopped", "failed", "disabled")


def recording_dir() -> pathlib.Path | None:
    """The console's ``dcist_launch_system/recording`` directory."""
    here = pathlib.Path(__file__).resolve()
    candidates = [here.parents[3] / "dcist_launch_system" / "recording"]
    try:
        import dcist_launch_system

        root = pathlib.Path(dcist_launch_system.__file__).resolve()
        candidates += [root.parents[2] / "recording", root.parent / "recording"]
    except Exception:  # noqa: BLE001
        pass
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(pathlib.Path(get_package_share_directory("dcist_launch_system")) / "recording")
    except Exception:  # noqa: BLE001
        pass
    for c in candidates:
        if c.is_dir():
            return c
    return None


def topic_file_name(role: str, platform: str, profile: str) -> str:
    if role == "base_station":
        return "base_station_experiment_topics.txt"
    platform = platform or "spot"
    return f"{platform}_topics.txt" if profile == "full" else f"{platform}_{profile}_topics.txt"


def machine_topics(machine: dict, profile: str, robots: list[str], planner_ns: str | None = None,
                   rec_dir: pathlib.Path | None = None) -> list[tuple[str, bool]]:  # fmt: skip
    """``[(topic, optional)]`` this machine records for ``profile``."""
    from dcist_launch_system.record_topics import read_topic_file

    rec_dir = rec_dir or recording_dir()
    if rec_dir is None:
        raise FileNotFoundError("dcist_launch_system/recording not found")
    path = rec_dir / topic_file_name(machine["role"], machine.get("platform", ""), profile)
    if not path.is_file():
        raise FileNotFoundError(f"no topic file {path.name} for {machine['name']} ({profile})")
    name = planner_ns if machine["role"] == "base_station" and planner_ns else machine["name"]
    return read_topic_file(path, env={"ADT4_ROBOT_NAME": name}, robots=robots)


def seg_dir(exp_id: str, run_id: str, seg: str) -> str:
    """The bag dir relative to $HOME."""
    return f"{REMOTE_ROOT}/{exp_id}/{run_id}/bags/{seg}"


def record_script(bag: str, topics: list[str], robot_name: str, storage: str | None = STORAGE,
                  env: dict | None = None, cache_bytes: int | None = None) -> str:  # fmt: skip
    """The zsh script the recorder runs.  It joins the running stack's ROS
    environment (ROS_DOMAIN_ID, RMW, zenoh config, ... of the tmux session
    ``adt4_system``: global, then session values) and then ``env`` (explicit
    overrides, e.g. the console's own for the base station)."""
    from dcist_launch_system.record_topics import record_command

    argv = record_command(f"$HOME/{bag}", topics, storage=storage, max_cache_size=cache_bytes)
    # $HOME must expand: quote everything but the output path
    cmd = " ".join(a if a.startswith("$HOME/") else shlex.quote(a) for a in argv)
    exports = "".join(f"export {k}={shlex.quote(str(v))}\n" for k, v in (env or {}).items())
    return (
        "#!/usr/bin/env zsh\n"
        "# written by the ADT4 console (experiment run recorder)\n"
        f"if tmux has-session -t {STACK_SESSION} 2>/dev/null; then\n"
        f"  for scope in -g '-t {STACK_SESSION}'; do\n"
        f"    tmux show-environment ${{=scope}} 2>/dev/null | grep -E '{ROS_ENV_PATTERN}' | "
        'while IFS= read -r kv; do export "$kv"; done\n'
        "  done\n"
        "fi\n"
        f"{exports}"
        f"export ADT4_ROBOT_NAME={shlex.quote(robot_name)}\n"
        'source "${ADT4_WS:-$HOME/dcist_ws}/install/setup.zsh"\n'
        'echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" >&2\n'
        f'exec {cmd} > "$HOME/{bag}.log" 2>&1\n'
    )


def start_command(bag: str, topics: list[tuple[str, bool]], robot_name: str, storage: str | None = STORAGE,
                  env: dict | None = None, cache_bytes: int | None = None) -> str:  # fmt: skip
    """Shell command that writes the topic list + script next to the bag and
    starts the recorder in tmux; fails if a recorder session is already up."""
    script = record_script(bag, [t for t, _ in topics], robot_name, storage, env, cache_bytes)
    listing = "\n".join(t + ("  # optional" if opt else "") for t, opt in topics)
    parent = bag.rsplit("/", 1)[0]
    return (
        f'if tmux has-session -t {SESSION} 2>/dev/null; then echo "ALREADY_RECORDING"; exit 3; fi\n'
        f'if [ -e "$HOME/{bag}" ]; then echo "BAG_EXISTS"; exit 4; fi\n'
        f'mkdir -p "$HOME/{parent}" || exit 5\n'
        f"cat > \"$HOME/{bag}.topics.txt\" <<'ADT4_TOPICS_EOF'\n{listing}\nADT4_TOPICS_EOF\n"
        f"cat > \"$HOME/{bag}.record.sh\" <<'ADT4_SCRIPT_EOF'\n{script}ADT4_SCRIPT_EOF\n"
        f'tmux new-session -d -s {SESSION} "zsh -l $HOME/{bag}.record.sh" || exit 6\n'
        'echo "STARTED"\n'
    )


def probe_command(bag: str) -> str:
    """One line ``alive=0|1 sess=0|1 size=<bytes> files=<n> meta=0|1 free_kb=<kb>``.
    The pgrep pattern's ``[b]`` keeps it from matching this very shell."""
    parent = bag.rsplit("/", 1)[0]
    return (
        f'D="$HOME/{bag}"; '
        f'alive=0; pgrep -f "[b]ag record -o $D" >/dev/null && alive=1; '
        f"sess=0; tmux has-session -t {SESSION} 2>/dev/null && sess=1; "
        'size=$(du -sb "$D" 2>/dev/null | cut -f1); '
        'files=$(ls "$D" 2>/dev/null | grep -cE "\\.(mcap|db3)$"); '
        'meta=0; [ -f "$D/metadata.yaml" ] && meta=1; '
        f'free=$(df -Pk "$HOME/{parent}" 2>/dev/null | tail -1 | awk \'{{print $4}}\'); '
        'echo "alive=$alive sess=$sess size=${size:-0} files=${files:-0} meta=$meta free_kb=${free:-0}"'
    )


def parse_probe(text: str) -> dict | None:
    for line in reversed((text or "").splitlines()):
        if "alive=" in line:
            kv = dict(p.split("=", 1) for p in line.split() if "=" in p)
            try:
                return {
                    "alive": kv.get("alive") == "1",
                    "session": kv.get("sess") == "1",
                    "size": int(kv.get("size") or 0),
                    "files": int(kv.get("files") or 0),
                    "metadata": kv.get("meta") == "1",
                    "free_bytes": int(kv.get("free_kb") or 0) * 1024,
                }
            except ValueError:
                return None
    return None


def stop_command(bag: str, timeout_s: float = STOP_TIMEOUT_S) -> str:
    """SIGINT the recorder (tmux C-c), wait for rosbag2 to finalize, then close
    the session.  Prints STOPPED / STOP_TIMEOUT / NOT_RUNNING."""
    n = int(timeout_s * 2)
    return (
        f'D="$HOME/{bag}"; '
        f'if ! pgrep -f "[b]ag record -o $D" >/dev/null; then '
        f"tmux kill-session -t {SESSION} 2>/dev/null; echo NOT_RUNNING; exit 0; fi; "
        f"tmux send-keys -t {SESSION} C-c; "
        f"for i in $(seq {n}); do "
        f'pgrep -f "[b]ag record -o $D" >/dev/null || break; sleep 0.5; '
        f'[ $i -eq 10 ] && pkill -INT -f "[b]ag record -o $D"; done; '
        f'if pgrep -f "[b]ag record -o $D" >/dev/null; then echo STOP_TIMEOUT; exit 7; fi; '
        f"tmux kill-session -t {SESSION} 2>/dev/null; echo STOPPED"
    )


class MachineRecorder:
    """Bookkeeping + state machine of one machine's recorder."""

    def __init__(self, machine: dict, exp_id: str, run_id: str, clock=time.time):
        self.machine = machine  # {name, user, ip, role, platform}
        self.exp_id = exp_id
        self.run_id = run_id
        self.clock = clock
        self.segments: list[str] = []
        self.state = "stopped"
        self.error: str | None = None
        self.started_t: float | None = None
        self.stopped_t: float | None = None
        self.size = 0  # current segment
        self.size_done = 0  # finished segments
        self.files = 0
        self.free_bytes: int | None = None
        self.rate_bps: float | None = None
        self.last_growth_t: float | None = None
        self.last_probe_t: float | None = None
        self.topics: list[tuple[str, bool]] = []
        self._last_size_t: tuple[int, float] | None = None
        self.max_write_gap = 0.0  # longest time between two observed writes

    @property
    def name(self) -> str:
        return self.machine["name"]

    @property
    def bag(self) -> str | None:
        return seg_dir(self.exp_id, self.run_id, self.segments[-1]) if self.segments else None

    def next_segment(self) -> str:
        n = len(self.segments) + 1
        return self.name if n == 1 else f"{self.name}_{n}"

    def observe(self, probe: dict | None, now: float | None = None) -> str | None:
        """Fold a probe result in; returns the event kind for a state change
        (recorder_started / _stalled / _recovered / _died / _unreachable) or None."""
        now = self.clock() if now is None else now
        self.last_probe_t = now
        prev = self.state
        if self.state in ("stopped", "disabled", "stopping"):
            return None
        if probe is None:
            if self.state in ("recording", "stalled"):
                self.state = "unreachable"
            elif self.state == "starting" and now - (self.started_t or now) > START_TIMEOUT_S:
                self.state, self.error = "failed", "machine unreachable"
            return _change(prev, self.state)
        self.free_bytes = probe["free_bytes"]
        self.files = probe["files"]
        size = probe["size"]
        if self._last_size_t is not None and size > self._last_size_t[0]:
            if self.last_growth_t is not None:
                self.max_write_gap = max(self.max_write_gap, now - self.last_growth_t)
            # rate over the whole recording: writes come in bursts
            if self.started_t is not None and now > self.started_t:
                self.rate_bps = size / (now - self.started_t)
            self.last_growth_t = now
        elif self._last_size_t is None and size > 0:
            self.last_growth_t = now
        self._last_size_t = (size, now)
        self.size = size
        if not probe["alive"]:
            if self.state == "starting" and now - (self.started_t or now) <= START_TIMEOUT_S:
                return None
            self.state = "failed"
            self.error = self.error or ("recorder exited" if prev != "starting" else "recorder did not start")
            return _change(prev, self.state)
        if probe["files"] < 1 and self.state == "starting":
            if now - (self.started_t or now) > START_TIMEOUT_S:
                self.state, self.error = "failed", "recorder runs but opened no bag file"
            return _change(prev, self.state)
        if self.last_growth_t is not None:
            quiet = now - self.last_growth_t
            stalled = quiet > max(STALL_S, 3 * self.max_write_gap)
            self.error = f"bag has not grown for {quiet:.0f} s" if stalled else None
        else:
            stalled = now - (self.started_t or now) > NO_DATA_S
            self.error = "nothing written yet: are the topics publishing?" if stalled else None
        self.state = "stalled" if stalled else "recording"
        return _change(prev, self.state)

    def status(self, now: float | None = None) -> dict:
        now = self.clock() if now is None else now
        left = None
        if self.free_bytes is not None and self.rate_bps and self.state in ("recording", "stalled"):
            left = self.free_bytes / self.rate_bps
        return {
            "machine": self.name,
            "role": self.machine.get("role"),
            "platform": self.machine.get("platform"),
            "host": self.machine.get("ip"),
            "state": self.state,
            "error": self.error,
            "bag": self.bag,
            "segments": list(self.segments),
            "elapsed_s": None if self.started_t is None else round((self.stopped_t or now) - self.started_t, 1),
            "size_bytes": self.size_done + self.size,
            "files": self.files,
            "rate_bps": None if self.rate_bps is None else round(self.rate_bps),
            "free_bytes": self.free_bytes,
            "time_left_s": None if left is None else round(left),
            "low_disk": left is not None and left < LOW_DISK_S,
            "probe_age_s": None if self.last_probe_t is None else round(now - self.last_probe_t, 1),
            "last_write_age_s": None if self.last_growth_t is None else round(now - self.last_growth_t, 1),
            "topics": len(self.topics),
        }


def _change(prev: str, new: str) -> str | None:
    if prev == new:
        return None
    if new == "recording":
        return "recorder_started" if prev == "starting" else "recorder_recovered"
    return {"stalled": "recorder_stalled", "failed": "recorder_died", "unreachable": "recorder_unreachable"}.get(new)


def summarize(machines: list[dict], enabled: bool) -> dict:
    """Header-chip summary: tone ok / warn / err / off and the counts."""
    if not enabled:
        return {"tone": "off", "recording": 0, "total": len(machines), "size_bytes": 0}
    states = [m["state"] for m in machines]
    rec = sum(s == "recording" for s in states)
    if any(s == "failed" for s in states):
        tone = "err"
    elif any(s in ("stalled", "unreachable", "starting", "stopping") for s in states) or any(m.get("low_disk") for m in machines):
        tone = "warn"
    elif machines and all(s == "stopped" for s in states):
        tone = "off"
    else:
        tone = "ok"
    return {"tone": tone, "recording": rec, "total": len(machines),
            "size_bytes": sum(m.get("size_bytes") or 0 for m in machines)}  # fmt: skip
