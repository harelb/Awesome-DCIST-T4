"""Experiment runs (API.md §18): metrics, sanity, recorder state, the run flow
with a fake ssh layer, offload to a local "disk", CSV export, routes."""

from __future__ import annotations

import json
import pathlib

import pytest

from adt4_console.experiment import metrics as M
from adt4_console.experiment import offload as O
from adt4_console.experiment import recorder as R
from adt4_console.experiment import sanity as S
from adt4_console.experiment.service import RecordingNotReady
from dcist_launch_system.record_topics import parse_topic_lines

from .conftest import make_client


# ------------------------------------------------------------------ pure parts
def test_topic_file_parsing_optional_comments_and_each_robot():
    lines = ["# header", "/tf", "", "/${ADT4_ROBOT_NAME}/odom", "/${ADT4_ROBOT_NAME}/fix  # optional",
             "/${ADT4_EACH_ROBOT}/omniplanner_node/compiled_plan_out  # optional", "/$UNSET_VAR/x", "/tf"]
    out = parse_topic_lines(lines, env={"ADT4_ROBOT_NAME": "euclid"}, robots=["euclid", "hamilton"])
    assert out == [("/tf", False), ("/euclid/odom", False), ("/euclid/fix", True),
                   ("/euclid/omniplanner_node/compiled_plan_out", True),
                   ("/hamilton/omniplanner_node/compiled_plan_out", True)]


def test_every_profile_topic_file_exists_and_parses():
    rec = R.recording_dir()
    assert rec is not None
    for role, platform in (("robot", "spot"), ("robot", "phoenix"), ("base_station", "")):
        for profile in ("full", "light"):
            topics = R.machine_topics({"name": "euclid", "role": role, "platform": platform}, profile,
                                      ["euclid", "hamilton"], planner_ns="willow", rec_dir=rec)
            names = [t for t, _ in topics]
            assert names and all("$" not in t for t in names)
            if role == "base_station":
                assert "/adt4_console/experiment_events" in names
                assert "/willow/omniplanner_node/plan_status" in names
            else:
                assert "/euclid/omniplanner_node/compiled_plan_out" in names


def test_path_stats_skips_jumps_and_frame_changes():
    poses = [{"t": 0, "robot": "a", "x": 0, "y": 0, "frame": "map"},
             {"t": 1, "robot": "a", "x": 1, "y": 0, "frame": "map"},
             {"t": 2, "robot": "a", "x": 1, "y": 0, "frame": "map"},  # idle
             {"t": 3, "robot": "a", "x": 50, "y": 0, "frame": "map"},  # relocalization jump
             {"t": 4, "robot": "a", "x": 51, "y": 0, "frame": "map"},
             {"t": 5, "robot": "a", "x": 0, "y": 0, "frame": "odom"}]
    st = M.path_stats(poses)["a"]
    assert st["path_m"] == 2.0 and st["jumps"] == 2 and st["moving_s"] == 2.0 and st["idle_s"] == 1.0
    assert st["first_move_t"] == 0


def test_interventions_intervals_points_and_retraction():
    ev = [
        {"seq": 0, "t": 10, "kind": "teleop", "robot": "a", "active": True},
        {"seq": 1, "t": 15, "kind": "teleop", "robot": "a", "active": False},
        {"seq": 2, "t": 20, "kind": "operator_cmd", "robot": "b", "cmd": "pause"},
        {"seq": 3, "t": 26, "kind": "operator_cmd", "robot": "b", "cmd": "resume"},
        {"seq": 4, "t": 30, "kind": "estop", "robot": "a", "pressed": True},  # still pressed at the end
        {"seq": 5, "t": 31, "kind": "intervention", "robot": "b", "category": "reset", "note": "moved chair"},
        {"seq": 6, "t": 32, "kind": "map_edit"},
    ]
    items = M.interventions(ev, t_end=40)
    kinds = [(i["robot"], i["type"], i["duration_s"]) for i in items]
    assert kinds == [("a", "teleop", 5.0), ("b", "pause", 6.0), ("a", "estop", 10.0), ("b", "reset", 0.0),
                     ("", "map_edit", 0.0)]
    run = {"robots": ["a", "b"], "t_start": 0, "t_plan_sent": 5, "t_all_reached": 35, "t_end": 40}
    m = M.compute(run, ev, [], now=40)
    assert m["interventions"] == 5 and m["intervention_s"] == 21.0 and m["mission_time_s"] == 30.0
    ev.append({"seq": 7, "t": 33, "kind": "event_retracted", "target_seq": 6})
    assert M.compute(run, ev, [], now=40)["interventions"] == 4


def test_link_outage_and_action_counts():
    ev = [{"t": 1, "kind": "link", "robot": "a", "connected": False},
          {"t": 4, "kind": "link", "robot": "a", "connected": True},
          {"t": 5, "kind": "action_result", "robot": "a", "action_type": "FOLLOW", "status": "SUCCESS"},
          {"t": 6, "kind": "action_result", "robot": "a", "action_type": "PICK", "status": "FAILED"},
          {"t": 7, "kind": "llm_usage", "cost_usd": 0.01, "input_tokens": 100, "output_tokens": 20},
          {"t": 8, "kind": "planned", "planning_s": 1.5}]
    m = M.compute({"robots": ["a"], "t_start": 0}, ev, [], now=10)
    a = m["robots"]["a"]
    assert a["link_outage_s"] == 3.0 and a["actions_attempted"] == 2 and a["actions_succeeded"] == 1
    assert m["llm_cost_usd"] == 0.01 and m["llm_tokens"] == 120 and m["planning_s"] == 1.5


def test_sanity_evaluate():
    rep = {"free_bytes": 50 * 1024**3, "segments": [
        {"seg": "euclid", "metadata": True, "duration_s": 100.0, "size": 10,
         "topics": {"/euclid/odom": {"count": 1000}, "/tf": {"count": 2000}, "/euclid/fix": {"count": 0}},
         "gaps": {"/euclid/odom": 3.5}},
        {"seg": "euclid_2", "metadata": False, "size": 5, "topics": {}},
        {"seg": "hamilton", "metadata": True, "duration_s": 1, "topics": {}},  # another machine's
    ]}
    exp = [("/euclid/odom", False), ("/tf", False), ("/euclid/fix", True), ("/euclid/spot_executor_node/action_result", False)]
    r = S.evaluate("euclid", rep, ["euclid", "euclid_2"], window_s=100.0, expected=exp)
    msgs = " | ".join(f["msg"] for f in r["findings"])
    assert r["level"] == "err" and "not finalized" in msgs and "3.5 s gap" in msgs
    assert "action_result: no messages" in msgs and "fix" not in msgs
    ok = S.evaluate("euclid", rep, ["euclid"], window_s=100.0, expected=exp[:3])
    assert ok["level"] == "warn"  # only the gap
    assert S.evaluate("x", None, ["x"], 1.0)["level"] == "err"
    assert S.overall([ok, r]) == "err"


def test_recorder_probe_parse_and_state_machine():
    assert R.parse_probe("noise\nalive=1 sess=1 size=2048 files=1 meta=0 free_kb=10") == {
        "alive": True, "session": True, "size": 2048, "files": 1, "metadata": False, "free_bytes": 10240}
    t = [100.0]
    rec = R.MachineRecorder({"name": "euclid", "role": "robot"}, "e", "r1", clock=lambda: t[0])
    rec.segments, rec.state, rec.started_t = ["euclid"], "starting", 100.0
    probe = {"alive": True, "session": True, "size": 0, "files": 0, "metadata": False, "free_bytes": 10**9}
    assert rec.observe(probe) is None and rec.state == "starting"  # no bag file yet
    t[0] = 101
    # alive with an open bag file = recording, even before rosbag2 flushed a byte
    assert rec.observe({**probe, "files": 1}) == "recorder_started" and rec.state == "recording"
    probe["files"] = 1
    t[0] = 100 + R.NO_DATA_S + 1
    assert rec.observe(probe) == "recorder_stalled" and "nothing written" in rec.error
    t[0] += 1
    assert rec.observe({**probe, "size": 1000}) == "recorder_recovered"
    t[0] += 30
    rec.observe({**probe, "size": 9000})  # a 30 s gap between writes
    assert rec.rate_bps and rec.status()["time_left_s"] is not None
    t[0] += R.STALL_S + 1  # 61 s quiet < 3 x 30 s: still fine
    assert rec.observe({**probe, "size": 9000}) is None and rec.state == "recording"
    t[0] += 30  # 91 s quiet > 90 s
    assert rec.observe({**probe, "size": 9000}) == "recorder_stalled"
    assert rec.observe(None) == "recorder_unreachable"
    assert rec.observe({**probe, "alive": False, "size": 9000}) == "recorder_died" and rec.state == "failed"
    s = R.summarize([rec.status()], True)
    assert s["tone"] == "err"
    assert R.summarize([], False)["tone"] == "off"


def test_shell_commands_quote_and_guard():
    bag = R.seg_dir("exp", "r001", "euclid")
    cmd = R.start_command(bag, [("/tf", False), ("/euclid/fix", True)], "euclid")
    assert "ALREADY_RECORDING" in cmd and "tmux new-session -d -s adt4_rec" in cmd
    assert "/euclid/fix  # optional" in cmd and "exec ros2 bag record -o $HOME/" + bag in cmd
    assert '[b]ag record -o $D' in R.probe_command(bag) and "C-c" in R.stop_command(bag)


def test_lsblk_parse_external_only():
    data = {"blockdevices": [
        {"name": "nvme0n1", "rm": False, "hotplug": False, "tran": "nvme", "children": [
            {"name": "nvme0n1p2", "mountpoints": ["/"], "size": 1, "fsavail": 1},
            {"name": "nvme0n1p3", "mountpoints": ["/data"], "size": 1, "fsavail": 1}]},
        {"name": "sda", "rm": False, "hotplug": True, "tran": "usb", "children": [
            {"name": "sda1", "label": "T7", "mountpoints": ["/media/harel/T7"], "size": "2000", "fsavail": "1500",
             "fstype": "exfat"}]},
        {"name": "sdb", "rm": True, "mountpoint": "/mnt/usb", "size": 5, "fsavail": 4},
    ]}
    out = O.parse_lsblk(json.dumps(data))
    assert [d["path"] for d in out] == ["/media/harel/T7", "/mnt/usb"]
    assert out[0]["label"] == "T7" and out[0]["free_bytes"] == 1500 and out[0]["transport"] == "usb"
    assert O.parse_network_target("rrg@nas:/vol/bags")["host"] == "nas"
    with pytest.raises(ValueError):
        O.parse_network_target("nonsense")


def test_md5_listing_parse_and_compare(tmp_path):
    (tmp_path / "euclid").mkdir()
    (tmp_path / "euclid" / "a.mcap").write_bytes(b"abc")
    (tmp_path / "other").mkdir()
    (tmp_path / "other" / "b").write_bytes(b"x")
    loc = O.local_md5_listing(tmp_path, ["euclid"])
    assert list(loc) == ["euclid/a.mcap"]
    remote = O.parse_md5_listing(f"3 {loc['euclid/a.mcap'][1]} ./euclid/a.mcap")
    assert O.compare(remote, loc)["ok"]
    assert not O.compare({**remote, "euclid/b": (1, "x")}, loc)["ok"]


# ------------------------------------------------------------------ the run flow
class FakeHelpers:
    """ssh / rsync stand-in: one recorder per address, bags grow on every probe."""

    def __init__(self, fail_start=()):
        self.state: dict[str, dict] = {}
        self.fail_start = set(fail_start)
        self.cmds: list[tuple[str, str]] = []
        self.rsyncs: list[tuple] = []

    def get_local_ips(self):
        return ["127.0.0.1"]

    def run_parallel(self, fn, items, max_workers=8):
        return [(i, fn(i)) for i in items]

    def rsync_transfer(self, *a, **kw):
        self.rsyncs.append(a)
        return True, "direct", ""

    def ssh_cmd(self, user, ip, cmd, timeout=5):
        self.cmds.append((ip, cmd))
        st = self.state.setdefault(ip, {"alive": False, "size": 0, "meta": False, "segs": []})
        if "tmux new-session" in cmd:
            if ip in self.fail_start:
                return 1, "", "zsh: command not found: ros2"
            if st["alive"]:
                return 3, "ALREADY_RECORDING", ""
            st.update(alive=True, size=0, meta=False)
            seg = cmd.split("/bags/", 1)[1].split('"', 1)[0]
            listing = cmd.split("<<'ADT4_TOPICS_EOF'\n", 1)[1].split("\nADT4_TOPICS_EOF", 1)[0]
            st["segs"].append((seg, [t.partition("#")[0].strip() for t in listing.splitlines()]))
            return 0, "STARTED", ""
        if "alive=" in cmd and "echo" in cmd:
            if st["alive"]:
                st["size"] += 4096
            return 0, f"alive={int(st['alive'])} sess={int(st['alive'])} size={st['size']} files=1 meta={int(st['meta'])} free_kb=100000000", ""
        if "send-keys" in cmd:
            st.update(alive=False, meta=True)
            return 0, "STOPPED", ""
        if "ADT4_PY_EOF" in cmd:
            segs = [{"seg": seg, "metadata": st["meta"], "duration_s": 0.0, "size": st["size"], "files": 1,
                     "topics": {t: {"count": 100} for t in topics}, "gaps": {}} for seg, topics in st["segs"]]
            return 0, "ADT4_SANITY " + json.dumps({"free_bytes": 10**12, "segments": segs}), ""
        if "md5sum" in cmd:
            return 0, "", ""
        if "free_kb=" in cmd:
            return 0, "free_kb=1000 exp_kb=10", ""
        return 0, "", ""


@pytest.fixture
def exp_client(settings, tmp_path):
    settings.experiments_dir = str(tmp_path / "experiments")
    helpers = FakeHelpers()
    c, svc = make_client(settings, helpers=helpers)
    with c:
        yield c, svc, helpers


def _robots(svc):
    return [r for r in svc.fleet.robots() if svc.fleet.machine(r)][:2]


def test_run_flow_gate_records_before_send_prompt_end_sanity(exp_client, monkeypatch):
    c, svc, helpers = exp_client
    monkeypatch.setattr(R, "START_TIMEOUT_S", 2.0)
    ex = svc.experiments
    r1, r2 = _robots(svc)
    resp = c.post("/api/experiments", json={"name": "Lang2Mission Penn", "robots": [r1, r2], "bag_profile": "light"})
    assert resp.status_code == 200, resp.text
    assert resp.json()["id"] == "lang2mission_penn"
    # LLM + planning before the run: buffered into it
    ex._on_llm("gpt-4.1", 1000, 100, 0.002)
    ex.emit("planned", planner="omniplanner", ok=True, planning_s=0.7)

    ex.gate(r1, "pddl", {"robot_id": r1})  # what RosBridge._send does first
    st = ex.status()
    run = st["run"]
    assert run["status"] == "recording" and run["record"] and st["recording"]["enabled"]
    machines = {m["machine"]: m for m in st["recording"]["machines"]}
    assert set(machines) >= {r1, r2} and len(machines) == 3  # + the base station
    assert all(m["state"] == "recording" for m in machines.values())
    # the gate started the bags before anything was "sent"
    assert sum("tmux new-session" in cmd for _, cmd in helpers.cmds) == 3

    # goals, plans and results arrive through the ROS observers
    for r in (r1, r2):
        ex._on_ros("goal_sent", r, {"goal_kind": "pddl", "msg": {"robot_id": r}, "topic": "/x"})
        ex._on_ros("plan", r, {"plan_id": f"p{r}", "actions": [{"type": "FOLLOW"}, {"type": "FOLLOW"}]})
    import time

    ex.run["t_plan_sent"] -= 10  # the goals went out 10 s ago
    ex.run["t_start"] -= 20
    tick = [ex.run["t_plan_sent"] + 1]
    ex.log.clock = lambda: tick[0]  # pose samples 2 s apart
    ex._on_hub(f"robot/{r1}/state", {"pose": {"x": 0.0, "y": 0.0, "frame": "map"}})
    ex._last_pose.clear()
    tick[0] += 2
    ex._on_hub(f"robot/{r1}/state", {"pose": {"x": 3.0, "y": 4.0, "frame": "map"}})
    ex._on_ros("teleop_state", r2, {"active": False})  # initial off: not an event
    ex._on_ros("teleop_state", r2, {"active": True})
    ex._on_ros("teleop_state", r2, {"active": False})
    ex._on_ros("action_result", r1, {"plan_id": f"p{r1}", "action_index": 1, "action_type": "FOLLOW", "status": "SUCCESS"})
    assert ex.status()["prompt"] is None  # r2 still going
    ex._on_ros("action_result", r2, {"plan_id": f"p{r2}", "action_index": 1, "action_type": "FOLLOW", "status": "SUCCESS"})
    prompt = ex.status()["prompt"]
    assert prompt and prompt["robots"] == {r1: "reached", r2: "reached"} and prompt["paths"][r1] == 5.0
    # bags never stop on their own
    assert not any("send-keys" in cmd for _, cmd in helpers.cmds)
    assert c.post("/api/experiments/run/prompt", json={"action": "continue"}).json()["prompt"] is None
    assert c.post("/api/experiments/run/intervention", json={"robot": r1, "category": "reset", "note": "x"}).status_code == 200

    job = c.post("/api/experiments/run/end", json={"stop_bags": True}).json()["job_id"]
    j = svc.jobs.wait(job, timeout=20)
    assert j.state == "succeeded", j.log_lines
    assert sum("send-keys" in cmd for _, cmd in helpers.cmds) == 3
    st = ex.status()
    assert st["run"] is None and st["recording"]["machines"] == []
    exp_id, run_id = "lang2mission_penn", run["id"]
    detail = c.get(f"/api/experiments/{exp_id}/runs/{run_id}").json()
    assert detail["run"]["status"] == "checked" and detail["sanity"]["level"] in ("ok", "warn")
    kinds = [e["kind"] for e in detail["events"]]
    assert kinds[:2] == ["llm_usage", "planned"] and detail["events"][0]["pre_run"]
    for k in ("run_start", "recorder_starting", "recorder_started", "plan_sent", "plan_received", "robot_reached",
              "all_reached", "continue", "teleop", "intervention", "recorder_stopped", "run_end", "sanity"):
        assert k in kinds, k
    m = detail["metrics"]
    assert m["interventions"] == 2 and m["robots_reached"] == 2 and m["llm_cost_usd"] == 0.002
    assert m["planning_s"] == 0.7 and m["robots"][r1]["path_m"] == 5.0

    # outcome later, then CSV
    out = c.post(f"/api/experiments/{exp_id}/runs/{run_id}/outcome", json={"label": "success"})
    assert out.status_code == 200 and out.json()["outcome"]["label"] == "success"
    csv_text = c.get(f"/api/experiments/{exp_id}/export.csv").text
    assert csv_text.splitlines()[0].startswith("experiment,run,status") and run_id in csv_text
    assert r1 in c.get(f"/api/experiments/{exp_id}/export.csv?table=robots").text


def test_testing_mode_logs_but_never_records(exp_client):
    c, svc, helpers = exp_client
    ex = svc.experiments
    r1, _ = _robots(svc)
    c.post("/api/experiments", json={"name": "dry", "robots": [r1]})
    c.patch("/api/experiments/dry", json={"test_mode": True})
    ex.gate(r1, "goto")
    st = ex.status()
    assert st["test_mode"] and st["run"]["record"] is False and st["run"]["test_mode"]
    assert st["recording"]["summary"]["tone"] == "off" and not any("tmux" in cmd for _, cmd in helpers.cmds)
    ex._on_ros("goal_sent", r1, {"goal_kind": "goto", "msg": {}})
    assert st["run"]["id"] in [r["id"] for r in c.get("/api/experiments/dry/runs").json()["runs"]]
    # the export leaves test runs out unless asked
    ex.end_run(stop_bags=True)
    assert svc.jobs.list()  # the end job ran
    import time

    time.sleep(0.5)
    assert len(c.get("/api/experiments/dry/export.csv").text.splitlines()) == 1
    assert len(c.get("/api/experiments/dry/export.csv?include_test=true").text.splitlines()) == 2


def test_gate_refuses_when_a_recorder_fails_until_accepted(exp_client, monkeypatch):
    c, svc, helpers = exp_client
    monkeypatch.setattr(R, "START_TIMEOUT_S", 1.0)
    ex = svc.experiments
    r1, r2 = _robots(svc)
    helpers.fail_start.add(svc.fleet.machine(r2)["ip"])
    c.post("/api/experiments", json={"name": "gate", "robots": [r1, r2]})
    with pytest.raises(RecordingNotReady) as e:
        ex.gate(r1, "goto")
    assert [m["machine"] for m in e.value.machines] == [r2] and "ros2" in str(e.value)
    assert c.post("/api/experiments/run/accept_missing", json={"machines": [r2]}).status_code == 200
    ex.gate(r1, "goto")  # passes now
    assert {m["machine"]: m["state"] for m in ex.status()["recording"]["machines"]}[r2] == "disabled"


def test_no_experiment_gate_is_a_noop_and_routes_report(exp_client):
    c, svc, helpers = exp_client
    svc.experiments.gate("euclid", "goto")
    assert helpers.cmds == []
    body = c.get("/api/experiments").json()
    assert body["active"] is None and "full" in body["profiles"] and "reset" in body["intervention_categories"]
    assert c.get("/api/experiments/status").json()["run"] is None
    assert c.post("/api/experiments/run/end", json={}).status_code == 409
    assert c.post("/api/data/targets", json={"spec": "me@nas:/bags"}).status_code == 200
    assert any(t["kind"] == "network" for t in c.get("/api/data/targets").json()["targets"])


def test_offload_to_local_disk_verifies(exp_client, tmp_path):
    c, svc, helpers = exp_client
    ex = svc.experiments
    c.post("/api/experiments", json={"name": "off", "robots": []})
    run = ex.open_run(record=False)
    ex.end_run(stop_bags=False)
    import time

    time.sleep(0.5)
    run = ex.store.run("off", run["id"])
    # pretend the base recorded a segment that is already on the "disk"
    run["machines"] = {"base": {"machine": {"name": "base", "role": "base_station", "user": "u", "ip": "127.0.0.1"},
                                "segments": ["base"], "size_bytes": 3}}
    run["status"] = "checked"
    ex.store.save_run(run)
    disk = tmp_path / "ssd"
    seg = disk / "off" / run["id"] / "bags" / "base"
    seg.mkdir(parents=True)
    (seg / "b.mcap").write_bytes(b"abc")
    import hashlib

    md5 = hashlib.md5(b"abc").hexdigest()
    orig = helpers.ssh_cmd

    def ssh(user, ip, cmd, timeout=5):
        if "md5sum" in cmd:
            return 0, f"3 {md5} ./base/b.mcap", ""
        return orig(user, ip, cmd, timeout)

    helpers.ssh_cmd = ssh
    job = c.post(f"/api/experiments/off/runs/{run['id']}/offload", json={"target": f"disk:{disk}"}).json()["job_id"]
    j = svc.jobs.wait(job, timeout=10)
    assert j.state == "succeeded", j.log_lines
    saved = ex.store.run("off", run["id"])
    assert saved["status"] == "offloaded" and saved["offload"]["ok"]
    assert (pathlib.Path(ex.store.run_dir("off", run["id"])) / "offload_manifest.json").is_file()
