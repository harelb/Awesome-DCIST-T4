"""GET /api/maps: run contents, sizes (budgeted + cached), image store summary."""

import datetime
import json
import pathlib
import sqlite3
import subprocess
import time
import types

import pytest

from dcist_launch_system import fleet_helpers
from dcist_launch_system.fleet import maps as fleet_maps
from dcist_launch_system.fleet import run_scan

from adt4_console.routes import maps as maps_route

from .conftest import make_client

SCHEMA = """
CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
CREATE TABLE sensors (sensor_id INTEGER PRIMARY KEY, robot_id INTEGER, name TEXT);
CREATE TABLE frames (frame_id INTEGER PRIMARY KEY, robot_id INTEGER, sensor_id INTEGER,
  timestamp_ns INTEGER, kind INTEGER, anchor_node_id INTEGER, rgb_path TEXT, depth_path TEXT);
CREATE TABLE observations (obs_id INTEGER PRIMARY KEY, frame_id INTEGER);
INSERT INTO meta VALUES ('schema_version', '1');
INSERT INTO sensors VALUES (1, 0, 'cam');
"""


def make_store(run: pathlib.Path, keyframes=3, subs=2, missing_file=False, unanchored=0):
    store = run / "image_store"
    (store / "rgb").mkdir(parents=True)
    conn = sqlite3.connect(store / "store.sqlite")
    conn.executescript(SCHEMA)
    fid = 0
    for kind, n in ((0, keyframes), (1, subs)):
        for _ in range(n):
            fid += 1
            rel = f"rgb/{fid}.jpg"
            if not (missing_file and fid == 1):
                (store / rel).write_bytes(b"x" * 1000)
            anchor = None if (kind == 0 and fid <= unanchored) else fid
            conn.execute(
                "INSERT INTO frames VALUES (?, 0, 1, ?, ?, ?, ?, NULL)",
                (fid, fid, kind, anchor, rel),
            )
    conn.execute("INSERT INTO observations VALUES (1, 1)")
    conn.commit()
    conn.close()


def make_run(root: pathlib.Path, name: str, dsg=True, mesh=False) -> pathlib.Path:
    run = root / name
    (run / "roman").mkdir(parents=True)
    backend = run / "hydra" / "backend"
    backend.mkdir(parents=True)
    if dsg:
        (backend / "dsg.json").write_text("{}")
    if mesh:
        (backend / "mesh.ply").write_bytes(b"p" * 5000)
        (backend / "dsg_with_mesh.json").write_text("{}")
    return run


@pytest.fixture
def out_root(settings):
    root = pathlib.Path(settings.output_root)
    root.mkdir(parents=True, exist_ok=True)
    good = make_run(root, "imgstore_full_20261018", mesh=True)
    make_store(good)
    bad = make_run(root, "broken_store")
    make_store(bad, missing_file=True, unanchored=1)
    make_run(root, "03192026_afternoon", dsg=False)
    return root


@pytest.fixture(autouse=True)
def fresh_cache(tmp_path, monkeypatch):
    monkeypatch.setattr(maps_route, "_CACHE_FILE", tmp_path / "cache" / "sizes.json")
    monkeypatch.setattr(maps_route, "_size_cache", None)
    monkeypatch.setattr(maps_route, "_filling", set())


def test_scan_run_contents(out_root):
    e = run_scan.scan_run(str(out_root / "imgstore_full_20261018"))
    assert e["has_dsg"] and e["has_dsg_mesh"] and e["has_mesh"]
    assert e["has_image_store"]
    s = e["image_store"]
    assert s["frames"] == {"keyframe": 3, "subkeyframe": 2}
    assert s["frames_total"] == 5 and s["observations"] == 1
    assert s["check"] == {"ok": True, "problems": []}
    assert s["size_bytes"] >= 5000
    assert e["size_bytes"] >= s["size_bytes"] + 5000
    assert e["size_pending"] is False
    assert e["mtime"] > 0 and e["signature"] >= e["mtime"]

    b = run_scan.scan_run(str(out_root / "broken_store"))
    assert b["image_store"]["check"]["ok"] is False
    probs = " ".join(b["image_store"]["check"]["problems"])
    assert "unanchored" in probs and "missing" in probs

    n = run_scan.scan_run(str(out_root / "03192026_afternoon"))
    assert n["has_image_store"] is False and n["image_store"] is None
    assert n["has_dsg"] is False


def test_scan_cached_and_budget(out_root):
    run = str(out_root / "imgstore_full_20261018")
    e = run_scan.scan_run(run, measure=False)
    assert e["size_bytes"] is None and e["size_pending"]
    known = {"imgstore_full_20261018": [e["signature"], 123, 45]}
    c = run_scan.scan_run(run, known, measure=False)
    assert c["size_bytes"] == 123 and c["image_store"]["size_bytes"] == 45
    # a stale signature is re-measured (or pending without budget)
    stale = {"imgstore_full_20261018": [e["signature"] - 10, 123, 45]}
    assert run_scan.scan_run(run, stale, measure=False)["size_pending"]
    runs = run_scan.scan_root(str(out_root), budget=0)
    assert len(runs) == 3 and all(r["size_pending"] for r in runs)


def test_remote_command_roundtrip(out_root):
    """The shipped shell command works under plain sh with only python3 + base64."""
    cmd = fleet_maps.remote_scan_command(str(out_root), {}, None)
    out = subprocess.run(["sh", "-c", cmd], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    runs = {r["name"]: r for r in fleet_maps.parse_scan_output("banner\n" + out.stdout)}
    assert set(runs) == {"imgstore_full_20261018", "broken_store", "03192026_afternoon"}
    assert runs["imgstore_full_20261018"]["image_store"]["frames"]["keyframe"] == 3
    only = fleet_maps.remote_scan_command(str(out_root), {}, None, ["broken_store"])
    out = subprocess.run(["sh", "-c", only], capture_output=True, text=True, timeout=60)
    assert [r["name"] for r in fleet_maps.parse_scan_output(out.stdout)] == ["broken_store"]


def test_remote_scan_missing_root():
    cmd = fleet_maps.remote_scan_command("/nonexistent/adt4_output")
    out = subprocess.run(["sh", "-c", cmd], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0 and fleet_maps.parse_scan_output(out.stdout) == []


def test_name_date_and_human_size():
    d = lambda y, m, dd: datetime.datetime(y, m, dd).timestamp()  # noqa: E731
    assert fleet_maps.name_date("imgstore_full_20261018") == d(2026, 10, 18)
    assert fleet_maps.name_date("03192026_afternoon_scene_graph") == d(2026, 3, 19)
    assert fleet_maps.name_date("openset_benchmark_2026-07-16_floor3") == d(2026, 7, 16)
    assert fleet_maps.name_date("env_probe_2026-07") == d(2026, 7, 1)
    assert fleet_maps.name_date("b45_clutter_iter1") is None
    assert fleet_maps.name_date("x_20261399") is None  # invalid month
    assert fleet_maps.human_size(None) == "?"
    assert fleet_maps.human_size(512) == "512B"
    assert fleet_maps.human_size(5 * 1024**2) == "5.0M"
    assert fleet_maps.human_size(117 * 1024**2) == "117M"


def _fake_helpers(calls):
    def ssh_cmd(user, ip, cmd, timeout=5):
        calls.append(ip)
        r = subprocess.run(["sh", "-c", cmd], capture_output=True, text=True, timeout=timeout + 10)
        return r.returncode, r.stdout.strip(), r.stderr.strip()

    return types.SimpleNamespace(
        ssh_cmd=ssh_cmd,
        run_parallel=fleet_helpers.run_parallel,
        get_local_ips=lambda: ["127.0.0.1"],
    )


def test_list_maps_endpoint(settings, out_root):
    calls = []
    c, svc = make_client(settings, helpers=_fake_helpers(calls))
    with c:
        r = c.get("/api/maps")
        assert r.status_code == 200
        body = r.json()
    machines = body["machines"]
    local = maps_route._local_name(svc)
    rows = {e["name"]: e for e in machines[local]}
    e = rows["imgstore_full_20261018"]
    assert e["has_image_store"] and e["image_store"]["check"]["ok"]
    assert e["has_mesh"] and e["size_bytes"] > 0 and e["size"].endswith(("K", "B"))
    assert e["name_date"] is not None and e["is_prior"] is True
    assert rows["03192026_afternoon"]["has_dsg"] is False
    # every other machine was listed through one (fake) ssh call each
    remotes = [m for m in machines if m != local]
    assert remotes and len(calls) == len(remotes)
    for m in remotes:
        rr = {x["name"]: x for x in machines[m]}
        assert rr["imgstore_full_20261018"]["path"] == f"{settings.output_root}/imgstore_full_20261018"
        assert rr["imgstore_full_20261018"]["image_store"]["frames"]["subkeyframe"] == 2
    assert body["pending"] == []
    # sizes were cached on this host, keyed per machine
    cache = json.loads(maps_route._CACHE_FILE.read_text())
    assert str(out_root / "imgstore_full_20261018") in cache[local]


def test_list_maps_pending_then_filled(settings, out_root, monkeypatch):
    monkeypatch.setattr(maps_route, "LOCAL_BUDGET_S", 0.0)
    monkeypatch.setattr(maps_route, "REMOTE_BUDGET_S", 0.0)
    c, svc = make_client(settings, helpers=_fake_helpers([]))
    with c:
        body = c.get("/api/maps").json()
        local = maps_route._local_name(svc)
        assert local in body["pending"]
        assert all(e["size_pending"] for e in body["machines"][local])
        deadline = time.time() + 30
        while time.time() < deadline:
            body = c.get("/api/maps").json()
            if not body["pending"]:
                break
            time.sleep(0.2)
    assert body["pending"] == []
    assert all(e["size_bytes"] is not None for e in body["machines"][local])


def test_list_maps_unreachable_machine(settings, out_root):
    def ssh_cmd(user, ip, cmd, timeout=5):
        return 255, "", "ssh: connect to host: No route to host"

    helpers = types.SimpleNamespace(ssh_cmd=ssh_cmd, run_parallel=fleet_helpers.run_parallel)
    c, svc = make_client(settings, helpers=helpers)
    with c:
        body = c.get("/api/maps").json()
    local = maps_route._local_name(svc)
    others = [v for k, v in body["machines"].items() if k != local]
    assert others and all("No route" in v["error"] for v in others)
    assert isinstance(body["machines"][local], list)
