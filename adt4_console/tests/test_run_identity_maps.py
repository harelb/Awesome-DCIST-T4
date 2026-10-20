"""Run identity in the console: copy pairing on /api/maps, idempotent transfers,
collision-safe names, Verify pairing, and launch → prior registered in the viewer."""

import json
import os
import pathlib
import subprocess
import types

import pytest

from dcist_launch_system import fleet_helpers
from dcist_launch_system.fleet import run_identity as ri

from adt4_console.routes import maps as maps_route

from .conftest import make_client
from .test_launch_topology import FakeSSH
from .test_maps_listing import make_store

HAMILTON_IP = "10.31.144.106"


@pytest.fixture(autouse=True)
def fresh_cache(tmp_path, monkeypatch):
    monkeypatch.setattr(maps_route, "_CACHE_FILE", tmp_path / "cache" / "sizes.json")
    monkeypatch.setattr(maps_route, "_size_cache", None)
    monkeypatch.setattr(maps_route, "_filling", set())


class Fleet:
    """Every machine is a HOME directory under tmp; ssh/rsync run locally against it."""

    def __init__(self, tmp_path, monkeypatch):
        self.tmp = tmp_path
        self.base_home = tmp_path / "base_home"
        (self.base_home / "adt4_output").mkdir(parents=True)
        monkeypatch.setenv("HOME", str(self.base_home))
        self.rsyncs = []
        self.hashed = []

    def home(self, ip):
        if ip in ("127.0.0.1", None):
            return self.base_home
        h = self.tmp / "machines" / ip
        (h / "adt4_output").mkdir(parents=True, exist_ok=True)
        return h

    def root(self, ip=None):
        return self.home(ip) / "adt4_output"

    def ssh_cmd(self, user, ip, cmd, timeout=5):
        env = dict(os.environ, HOME=str(self.home(ip)))
        r = subprocess.run(["sh", "-c", cmd], capture_output=True, text=True, timeout=timeout + 30, env=env)
        return r.returncode, r.stdout.strip(), r.stderr.strip()

    def _expand(self, ip, path):
        return str(self.home(ip) / path[2:]) if path.startswith("~/") else path

    def rsync_transfer(self, su, sip, spath, du, dip, dpath, exclude=None, progress_callback=None, **_):
        self.rsyncs.append((sip, spath, dip, dpath))
        src, dst = self._expand(sip, spath), self._expand(dip, dpath)
        pathlib.Path(dst).mkdir(parents=True, exist_ok=True)
        cmd = ["rsync", "-a"] + [x for pat in (exclude or []) for x in ("--exclude", pat)]
        r = subprocess.run(cmd + [src + "/", dst + "/"], capture_output=True, text=True)
        return r.returncode == 0, "direct", r.stderr

    def hash_remote_experiment(self, user, ip, root, name):
        self.hashed.append((ip, name))
        return {"./hydra/backend/dsg.json": "h"}

    def helpers(self):
        return types.SimpleNamespace(
            ssh_cmd=self.ssh_cmd,
            run_parallel=fleet_helpers.run_parallel,
            get_local_ips=lambda: ["127.0.0.1"],
            rsync_transfer=self.rsync_transfer,
            hash_remote_experiment=self.hash_remote_experiment,
        )


def make_run(root: pathlib.Path, name: str, robot=None) -> pathlib.Path:
    run = root / name
    (run / "hydra" / "backend").mkdir(parents=True)
    (run / "roman").mkdir()
    (run / "hydra" / "backend" / "dsg.json").write_text('{"nodes": []}')
    (run / "roman" / "1.roman_map.pkl").write_bytes(b"r" * 100)
    if robot:
        ri.write_run_info(run, robot=robot, platform="topaz", session="spot-default")
    return run


@pytest.fixture
def fleet(tmp_path, monkeypatch, settings):
    settings.output_root = "~/adt4_output"
    return Fleet(tmp_path, monkeypatch)


def _wait(svc, job_id):
    return svc.jobs.wait(job_id).to_dict()


def _transfer(c, svc, **kw):
    body = {"src": "hamilton", "experiment": "map_x", **kw}
    job = _wait(svc, c.post("/api/maps/transfer", json=body).json()["job_id"])
    assert job["state"] == "succeeded", job["log"]
    return job


def test_transfer_names_copy_and_second_transfer_is_noop(settings, fleet):
    run = make_run(fleet.root(HAMILTON_IP), "map_x", robot="hamilton")
    rid = ri.read_run_info(run)["run_id"]
    make_run(fleet.root(), "map_x", robot="local")  # the base's own run, same experiment
    c, svc = make_client(settings, helpers=fleet.helpers())
    with c:
        plan = c.get("/api/maps/transfer/plan", params={"src": "hamilton", "experiment": "map_x"}).json()
        assert plan["status"] == "new" and plan["dest_name"] == "map_x__hamilton"
        job = _transfer(c, svc)
        assert job["result"]["dest_name"] == "map_x__hamilton"
        copy = fleet.root() / "map_x__hamilton"
        origin = json.loads((copy / ri.ORIGIN).read_text())
        assert origin["run_id"] == rid and origin["source_machine"] == "hamilton"
        assert origin["source_path"] == str(run) and origin["manifest_digest"]
        assert origin["images"] == "all" and origin["transferred_at"]
        assert ri.read_run_info(fleet.root() / "map_x")["robot"] == "local"  # untouched

        # second transfer: nothing moves
        n = len(fleet.rsyncs)
        job = _transfer(c, svc)
        assert job["result"]["up_to_date"] is True and len(fleet.rsyncs) == n
        assert any("Already up to date" in line["text"] for line in job["log"])
        # forced: copies again into the same dir
        job = _transfer(c, svc, force=True)
        assert job["result"]["dest_name"] == "map_x__hamilton" and len(fleet.rsyncs) == n + 1

        # robot changes the run → listing says outdated, transfer updates in place
        body = c.get("/api/maps").json()
        ham = {e["name"]: e for e in body["machines"]["hamilton"]}
        assert ham["map_x"]["copy"]["state"] == "in_sync"
        assert ham["map_x"]["copy"]["name"] == "map_x__hamilton"
        (run / "roman" / "2.roman_map.pkl").write_bytes(b"more")
        body = c.get("/api/maps").json()
        ham = {e["name"]: e for e in body["machines"]["hamilton"]}
        assert ham["map_x"]["copy"]["state"] == "outdated"
        plan = c.get("/api/maps/transfer/plan", params={"src": "hamilton", "experiment": "map_x"}).json()
        assert plan["status"] == "update" and plan["state"] == "outdated"
        job = _transfer(c, svc)
        assert job["result"]["status"] == "update"
        assert (copy / "roman" / "2.roman_map.pkl").is_file()
        assert not (fleet.root() / "map_x__hamilton__2").exists()
        body = c.get("/api/maps").json()
    local = {e["name"]: e for e in body["machines"]["local"]}
    assert local["map_x__hamilton"]["copy_of"]["machine"] == "hamilton"
    assert local["map_x__hamilton"]["copy_of"]["path"] == str(run)
    assert local["map_x__hamilton"]["copy_of"]["state"] == "in_sync"
    assert "copy_of" not in local["map_x"]


def test_collision_picks_suffix_and_never_overwrites(settings, fleet):
    make_run(fleet.root(HAMILTON_IP), "map_x", robot="hamilton")
    other = make_run(fleet.root(), "map_x__hamilton", robot="hamilton")  # another run
    other_id = ri.read_run_info(other)["run_id"]
    c, svc = make_client(settings, helpers=fleet.helpers())
    with c:
        plan = c.get("/api/maps/transfer/plan", params={"src": "hamilton", "experiment": "map_x"}).json()
        assert plan["status"] == "collision" and plan["dest_name"] == "map_x__hamilton__2"
        assert "holds a different run" in plan["note"]
        job = _transfer(c, svc)
    assert job["result"]["dest_name"] == "map_x__hamilton__2"
    assert any(line["level"] == "warn" and "__2" in line["text"] for line in job["log"])
    assert ri.read_run_info(other)["run_id"] == other_id
    assert not (other / ri.ORIGIN).exists()


def test_store_repacked_only_when_changed(settings, fleet, monkeypatch):
    run = make_run(fleet.root(HAMILTON_IP), "map_x", robot="hamilton")
    make_store(run)
    packs = []

    def fake_pipe(job, svc, src, dst, src_run, dst_run, images, compression):
        packs.append(dst_run)
        return True, "1.0 MB in 0.1s"

    monkeypatch.setattr(maps_route, "pipe_store", fake_pipe)
    c, svc = make_client(settings, helpers=fleet.helpers())
    with c:
        kw = {"images": "keyframes", "compression": "fast"}
        _transfer(c, svc, **kw)
        assert len(packs) == 1
        (run / "roman" / "2.pkl").write_bytes(b"x")  # map files change, store does not
        job = _transfer(c, svc, **kw)
        assert len(packs) == 1 and job["result"]["image_store"] == "unchanged"
        job = _transfer(c, svc, images="all", compression="lossless")  # settings changed
        assert len(packs) == 2


def test_legacy_copy_matched_by_contents(settings, fleet):
    run = make_run(fleet.root(HAMILTON_IP), "old_run")  # no run_info (pre run ids)
    dst = fleet.root() / "old_run"
    subprocess.run(["rsync", "-a", f"{run}/", f"{dst}/"], check=True)
    c, svc = make_client(settings, helpers=fleet.helpers())
    with c:
        body = c.get("/api/maps").json()
        ham = {e["name"]: e for e in body["machines"]["hamilton"]}
        assert ham["old_run"]["copy"]["state"] == "in_sync"
        assert ham["old_run"]["copy"]["matched_by"] == "contents"
        plan = c.get("/api/maps/transfer/plan", params={"src": "hamilton", "experiment": "old_run"}).json()
        assert plan["dest_name"] == "old_run" and plan["status"] == "update"
        job = _transfer(c, svc, experiment="old_run")  # adopts the copy, writes origin
        assert job["result"]["dest_name"] == "old_run"
        assert (dst / ri.ORIGIN).is_file()
        job = _transfer(c, svc, experiment="old_run")
        assert job["result"]["up_to_date"] is True
    # nothing was written on the robot
    assert sorted(os.listdir(run)) == ["hydra", "roman"]


def test_listing_not_copied_and_verify_uses_pairing(settings, fleet):
    make_run(fleet.root(HAMILTON_IP), "solo", robot="hamilton")
    c, svc = make_client(settings, helpers=fleet.helpers())
    with c:
        body = c.get("/api/maps").json()
        assert body["base"] == "local"
        ham = {e["name"]: e for e in body["machines"]["hamilton"]}
        assert ham["solo"]["copy"] == {"state": "not_copied", "machine": "local"}
        job = _wait(
            svc,
            c.post(
                "/api/maps/verify",
                json={"experiment": "solo", "names": {"hamilton": "solo", "local": "solo__hamilton"}},
            ).json()["job_id"],
        )
    assert job["state"] == "succeeded"
    assert sorted(fleet.hashed) == sorted([("127.0.0.1", "solo__hamilton"), (HAMILTON_IP, "solo")])


# --- launch → prior in the viewer ---------------------------------------------


def _launch_deps():
    from dcist_launch_system.fleet.launch_workflow import LaunchDeps

    ssh = FakeSSH([("tmux has-session", (1, "", ""))])
    return LaunchDeps(ssh=ssh, rsync=lambda *a, **k: (True, "direct", "ok"), sleep=lambda s: None)


def _launch(c, svc, **plan):
    body = {"robots": ["euclid"], "session": "spot-default", "experiment": "reloc_1", "mode": "relocalize", **plan}
    job = _wait(svc, c.post("/api/launch", json=body).json()["job_id"])
    assert job["state"] == "succeeded", job["log"]
    return job


def test_launch_registers_base_copy_of_prior_and_survives_restart(settings, fleet, tmp_path):
    settings.launch_state_path = str(tmp_path / "state" / "active_launch.json")
    run = make_run(fleet.root(HAMILTON_IP), "map_x", robot="hamilton")
    c, svc = make_client(settings, helpers=fleet.helpers(), launch_deps=_launch_deps())
    with c:
        _transfer(c, svc)  # base copy map_x__hamilton
        # launch with the robot's original as the prior: the base copy is what the viewer shows
        job = _launch(c, svc, prior_path="~/adt4_output/map_x", prior_source="hamilton")
        assert any("viewer: prior: map_x (hamilton)" in line["text"] for line in job["log"])
        active = c.get("/api/launch/active").json()
        sid = "file:prior:map_x__hamilton"
        assert active["default_sources"] == [sid, "live:euclid"]
        assert active["active"]["prior"]["local_path"] == str(fleet.root() / "map_x__hamilton")
        srcs = {s["id"]: s for s in c.get("/api/graph/sources").json()}
        assert srcs[sid]["label"] == "prior: map_x (hamilton)"
    assert run.is_dir()
    # console restart mid-run: the association and the source come back
    c2, svc2 = make_client(settings, helpers=fleet.helpers())
    with c2:
        assert c2.get("/api/launch/active").json()["default_sources"] == [sid, "live:euclid"]
        assert sid in {s["id"] for s in c2.get("/api/graph/sources").json()}
    # a launch without a prior replaces the association
    c3, svc3 = make_client(settings, helpers=fleet.helpers(), launch_deps=_launch_deps())
    with c3:
        _launch(c3, svc3, mode="mapping")
        assert c3.get("/api/launch/active").json()["default_sources"] == ["live:euclid"]


def test_launch_prior_only_on_robot_is_pending_until_copied(settings, fleet):
    make_run(fleet.root(HAMILTON_IP), "map_x", robot="hamilton")
    c, svc = make_client(settings, helpers=fleet.helpers(), launch_deps=_launch_deps())
    with c:
        job = _launch(c, svc, prior_path="~/adt4_output/map_x", prior_source="hamilton")
        assert any("prior not on this machine yet" in line["text"] for line in job["log"])
        active = c.get("/api/launch/active").json()
        assert active["active"]["prior"]["pending"] is True
        assert active["default_sources"] == ["live:euclid"]
        job = _transfer(c, svc)
        assert any("now in the viewer" in line["text"] for line in job["log"])
        active = c.get("/api/launch/active").json()
        assert active["default_sources"] == ["file:prior:map_x__hamilton", "live:euclid"]


def test_launch_with_local_prior(settings, fleet):
    make_run(fleet.root(), "base_map", robot="local")
    c, svc = make_client(settings, helpers=fleet.helpers(), launch_deps=_launch_deps())
    with c:
        _launch(c, svc, prior_path="~/adt4_output/base_map", prior_source=None)
        active = c.get("/api/launch/active").json()
        assert active["default_sources"][0] == "file:prior:base_map"
        assert active["active"]["prior"]["label"] == "prior: base_map (local)"
        assert c.delete("/api/launch/active").json()["default_sources"] == []
