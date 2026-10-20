"""Maps: listing, transfer, verification, Neo4j ingest, file-source registration (API.md §7)."""

from __future__ import annotations

import getpass
import os
import pathlib
import shlex
import subprocess
import sys
import threading
import time
from typing import Literal

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..graph_service import FileSource, NotFound
from . import services

router = APIRouter(prefix="/api/maps")

BAG_EXCLUDES = ["recorded_data/"]


def _notify_fn():
    """heracles' combined helper (both counters) when this heracles has it."""
    try:
        from heracles.graph_interface import notify_map_change
    except ImportError:
        return None
    return notify_map_change


def bump_map_counters(db, dirty_layers=None, affected_ids=None) -> int:
    """Bump BOTH Neo4j version counters after a write; returns ``_State.version``.

    ``_State{key:'global'}.version`` is what the console (and viz agents) poll;
    ``_MapState.version`` is what the heracles DSG publisher polls to republish
    within 0.5 s (``heracles_ros.map_state.bump_map_version``).  A writer that
    bumps only one leaves the other consumer stale.
    """
    notify = _notify_fn()
    if notify is not None:
        out = notify(db, list(dirty_layers or []), list(affected_ids or []))
        if isinstance(out, (tuple, list)) and out:  # (state_version, map_version)
            return int(out[0])
        if isinstance(out, int):
            return out
        from heracles.graph_interface import read_version

        return int(read_version(db))
    # same cypher as heracles graph_interface.record_change + map_state.bump_map_version
    records, _, _ = db.execute(
        "MERGE (s:_State {key: 'global'}) "
        "ON CREATE SET s.version = 1 "
        "ON MATCH SET s.version = s.version + 1 "
        "SET s.dirty_layers = $dl, s.affected_ids = $ai "
        "RETURN s.version AS v",
        parameters={"dl": list(dirty_layers or []), "ai": list(affected_ids or [])},
    )
    version = records[0]["v"] if records else 0
    db.execute(
        "MERGE (m:_MapState) SET m.version = coalesce(m.version, 0) + 1 "
        "RETURN m.version AS v"
    )
    return int(version)


def _local_name(svc) -> str:
    m = svc.fleet.local_machine
    return m["name"] if m else "local"


def _endpoint(svc, name: str | None) -> dict:
    """Machine dict for ``name``; ``None``/``"local"``/the local machine → this host."""
    if name in (None, "", "local", _local_name(svc)):
        m = svc.fleet.local_machine
        return (
            dict(m)
            if m
            else {"name": "local", "user": getpass.getuser(), "ip": "127.0.0.1"}
        )
    m = svc.fleet.machine(name)
    if m is None:
        raise HTTPException(404, f"unknown machine {name}")
    return m


def _du(path: pathlib.Path) -> str:
    try:
        out = subprocess.run(
            ["du", "-sh", str(path)], capture_output=True, text=True, timeout=5
        ).stdout
        return out.split()[0] if out else "?"
    except Exception:  # noqa: BLE001
        return "?"


def _local_entry(path: pathlib.Path, size: str) -> dict:
    from dcist_launch_system.fleet.maps import is_prior_map

    subdirs = (
        sorted(p.name for p in path.iterdir() if p.is_dir()) if path.is_dir() else []
    )
    entry = {"name": path.name, "subdirs": subdirs, "size": size, "path": str(path)}
    entry["is_prior"] = is_prior_map(entry)
    return entry


def list_local(svc) -> list[dict]:
    root = svc.settings.output_root_path
    dirs = sorted(p for p in root.iterdir() if p.is_dir()) if root.is_dir() else []
    extra = [s.run_dir for s in svc.graph.sources.values() if isinstance(s, FileSource)]
    dirs += [d for d in extra if d not in dirs and d.is_dir()]
    sizes = dict(svc.helpers.run_parallel(_du, dirs, max_workers=8)) if dirs else {}
    return [
        _local_entry(d, sizes.get(d) if isinstance(sizes.get(d), str) else "?")
        for d in dirs
    ]


@router.get("")
def list_maps(request: Request):
    from dcist_launch_system.fleet.maps import is_prior_map

    svc = services(request)
    h = svc.helpers
    out = {}
    try:
        out[_local_name(svc)] = list_local(svc)
    except Exception as e:  # noqa: BLE001
        out[_local_name(svc)] = {"error": str(e)}
    with svc.fleet.lock:
        remotes = [
            dict(m)
            for m in svc.fleet.runtime_config.values()
            if m.get("online") and not m.get("local")
        ]
    root = svc.settings.output_root

    def fetch(m):
        exps = h.list_remote_experiments(m["user"], m["ip"], root)
        for e in exps:
            e["is_prior"] = is_prior_map(e)
            e["path"] = f"{root}/{e['name']}"
        return exps

    for m, r in h.run_parallel(fetch, remotes) if remotes else []:
        out[m["name"]] = r if isinstance(r, list) else {"error": str(r)}
    return {"machines": out}


class TransferBody(BaseModel):
    src: str
    experiment: str
    dst: str | None = None
    as_name: str | None = None
    include_bags: bool = False
    bwlimit_kbps: int | None = None
    # image store: which frames to send and how (hydra_image_store pack/unpack).
    # images="all" + compression="none" keeps the plain rsync of the run dir.
    images: Literal["keyframes", "all", "none"] = "all"
    compression: Literal["none", "lossless", "fast"] = "none"


SSH = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10"]
STORE_DIR = "image_store"
# On a remote machine: spark_env python, with the workspace's hydra_image_store
# source on PYTHONPATH in case it is not pip-installed there.
REMOTE_PY = (
    'PY="${ADT4_ENV:-$HOME/environments/dcist}/spark_env/bin/python3"; '
    '[ -x "$PY" ] || PY=python3; '
    'export PYTHONPATH="${ADT4_WS:-$HOME/dcist_ws}/src/awesome_dcist_t4/hydra/'
    'hydra_image_store/src${PYTHONPATH:+:$PYTHONPATH}"; '
)


def _is_local(svc, m: dict) -> bool:
    if m.get("local"):
        return True
    try:
        return m["ip"] in svc.helpers.get_local_ips()
    except Exception:  # noqa: BLE001
        return m.get("ip") in ("127.0.0.1", "localhost")


def _local_store_env() -> dict:
    """Env for running ``python -m hydra_image_store`` locally."""
    env = dict(os.environ)
    try:
        import hydra_image_store  # noqa: F401

        return env
    except ImportError:
        pass
    repo = pathlib.Path(__file__).resolve().parents[3]
    for cand in (
        repo / "hydra" / "hydra_image_store" / "src",
        pathlib.Path(os.path.expanduser("~/dcist_ws/src/awesome_dcist_t4/hydra/hydra_image_store/src")),
    ):
        if cand.is_dir():
            env["PYTHONPATH"] = os.pathsep.join(
                [str(cand)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else [])
            )
            break
    return env


def pack_args(run_dir: str, images: str, compression: str) -> list[str]:
    """``hydra-image-store pack`` → archive on stdout (zstd-wrapped: the sqlite
    index and any raw bytes compress; JPEGs pass through cheaply)."""
    return [
        "pack", run_dir, "-", "--frames", images, "--compression", compression,
        "--zstd-level", "3", "--quiet",
    ]


def unpack_args(dest_run_dir: str) -> list[str]:
    """``hydra-image-store unpack`` ← archive on stdin; rebuilds
    ``<dest>/image_store`` (via ``image_store.partial``) and runs ``check_store``."""
    return ["unpack", "-", dest_run_dir, "--overwrite", "--quiet"]


def _unpack_summary(text: str) -> str:
    """Short summary of unpack's JSON stats (else the output tail)."""
    import json

    text = text.strip()
    try:
        stats = json.loads(text[text.index("{"):])
    except ValueError:
        return text[-400:]
    if not isinstance(stats, dict):
        return text[-400:]
    parts = [f"{k}={v}" for k, v in stats.items() if isinstance(v, (int, float, str, bool))]
    check = stats.get("check")
    if isinstance(check, dict) and "ok" in check:
        parts.append(f"check ok={check['ok']}")
    return ", ".join(parts[:8])


def store_command(svc, m: dict, args: list[str]) -> tuple[list[str], dict | None]:
    """argv (+ env) running ``python -m hydra_image_store <args>`` on machine ``m``."""
    if _is_local(svc, m):
        expanded = [os.path.expanduser(a) if a.startswith("~") else a for a in args]
        return [sys.executable, "-m", "hydra_image_store", *expanded], _local_store_env()
    remote_args = " ".join(
        a.replace("~", "$HOME", 1) if a.startswith("~") else shlex.quote(a) for a in args
    )
    script = REMOTE_PY + f'exec "$PY" -m hydra_image_store {remote_args}'
    return [*SSH, f"{m['user']}@{m['ip']}", f"sh -c {shlex.quote(script)}"], None


def pipe_store(job, svc, src: dict, dst: dict, src_run: str, dst_run: str, images, compression):
    """Stream ``pack`` on ``src`` into ``unpack`` on ``dst`` through this host.

    Returns (ok, message).  Bytes on the wire are counted and logged.
    """
    pack_cmd, pack_env = store_command(svc, src, pack_args(src_run, images, compression))
    unpack_cmd, unpack_env = store_command(svc, dst, unpack_args(dst_run))
    job.log("dim", "pack: " + " ".join(pack_cmd[-1:] if pack_env is None else pack_cmd))
    p_src = subprocess.Popen(pack_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=pack_env)
    p_dst = subprocess.Popen(
        unpack_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=unpack_env
    )
    errs: dict[str, bytes] = {}

    def drain(name, stream):
        errs[name] = stream.read()

    t_err = threading.Thread(target=drain, args=("pack", p_src.stderr), daemon=True)
    t_out = threading.Thread(target=drain, args=("unpack", p_dst.stdout), daemon=True)
    t_err.start()
    t_out.start()
    sent, next_log, t0 = 0, 8 << 20, time.time()
    try:
        while True:
            job.check_cancelled()
            chunk = p_src.stdout.read(1 << 16)
            if not chunk:
                break
            p_dst.stdin.write(chunk)
            sent += len(chunk)
            if sent >= next_log:
                rate = sent / max(time.time() - t0, 1e-3) / 1e6
                job.log("dim", f"image store: {sent / 1e6:.1f} MB sent ({rate:.1f} MB/s)")
                next_log += 8 << 20
    except BrokenPipeError:
        pass
    except BaseException:
        p_src.kill()
        p_dst.kill()
        raise
    finally:
        try:
            p_dst.stdin.close()
        except BrokenPipeError:
            pass
    rc_src, rc_dst = p_src.wait(), p_dst.wait()
    t_err.join(5)
    t_out.join(5)
    secs = time.time() - t0
    tail = lambda b: (b or b"").decode("utf-8", "replace").strip()[-400:]  # noqa: E731
    if rc_src != 0:
        return False, f"pack failed (rc {rc_src}): {tail(errs.get('pack'))}"
    if rc_dst != 0:
        return False, f"unpack failed (rc {rc_dst}): {tail(errs.get('unpack'))}"
    summary = _unpack_summary((errs.get("unpack") or b"").decode("utf-8", "replace"))
    return True, f"{sent / 1e6:.1f} MB in {secs:.1f}s" + (f"; {summary}" if summary else "")


@router.post("/transfer")
def transfer(body: TransferBody, request: Request):
    svc = services(request)
    src, dst = _endpoint(svc, body.src), _endpoint(svc, body.dst)
    if src["ip"] == dst["ip"]:
        raise HTTPException(400, "source and destination are the same machine")
    root = svc.settings.output_root
    dest_name = body.as_name or body.experiment

    plain = body.images == "all" and body.compression == "none"
    if body.images == "none" and body.compression != "none":
        raise HTTPException(400, "compression needs images=keyframes|all")

    def run(job):
        if body.bwlimit_kbps:
            job.log("warn", "bwlimit_kbps is not supported by rsync_transfer; ignoring")
        exclude = [] if body.include_bags else list(BAG_EXCLUDES)
        if not plain:
            exclude.append(f"/{STORE_DIR}/")
        job.log(
            "info",
            f"{src['name']}:{root}/{body.experiment} -> {dst['name']}:{root}/{dest_name}"
            + ("" if body.include_bags else " (excluding rosbags)")
            + (
                ""
                if plain
                else f"; image store: {body.images}"
                + (f", {body.compression}" if body.images != "none" else "")
            ),
        )
        ok, method, msg = svc.helpers.rsync_transfer(
            src["user"],
            src["ip"],
            f"{root}/{body.experiment}",
            dst["user"],
            dst["ip"],
            f"{root}/{dest_name}",
            exclude=exclude or None,
            progress_callback=lambda pct: job.set_progress(
                dst["name"], pct if plain or body.images == "none" else pct * 0.5
            ),
        )
        job.log("ok" if ok else "error", f"{method}: {msg[-500:] if msg else ''}")
        if not ok:
            return False
        result = {"method": method, "dest": f"{root}/{dest_name}"}
        if not plain and body.images != "none":
            job.log("info", f"Packing image store ({body.images}, {body.compression}) ...")
            ok, msg = pipe_store(
                job,
                svc,
                src,
                dst,
                f"{root}/{body.experiment}",
                f"{root}/{dest_name}",
                body.images,
                body.compression,
            )
            job.log("ok" if ok else "error", f"image store: {msg}")
            if not ok:
                return False
            result["image_store"] = msg
        job.set_progress(dst["name"], 100)
        return result

    job = svc.jobs.start(
        "transfer", f"Transfer {body.experiment} {src['name']}→{dst['name']}", run
    )
    return {"job_id": job.id}


class VerifyBody(BaseModel):
    experiment: str
    machines: list[str] | None = None


@router.post("/verify")
def verify(body: VerifyBody, request: Request):
    from dcist_launch_system.fleet.maps import compare_hashes

    svc = services(request)
    with svc.fleet.lock:
        names = body.machines or [
            n for n, m in svc.fleet.runtime_config.items() if m.get("online")
        ]
    targets = [_endpoint(svc, n) for n in names]
    if not targets:
        raise HTTPException(400, "no machines to verify")

    def run(job):
        def fetch(m):
            return svc.helpers.hash_remote_experiment(
                m["user"], m["ip"], svc.settings.output_root, body.experiment
            )

        hashes = {}
        for m, r in svc.helpers.run_parallel(fetch, targets):
            hashes[m["name"]] = r if isinstance(r, dict) else {}
            job.log("dim", f"{m['name']}: {len(hashes[m['name']])} files")
        rows = compare_hashes(hashes)
        bad = sum(not r.consistent for r in rows)
        job.log("ok" if not bad else "warn", f"{len(rows)} files, {bad} inconsistent")
        machines = sorted(hashes)
        return {
            "machines": machines,
            "rows": [
                {
                    "path": r.path,
                    "consistent": r.consistent,
                    "cells": {m: r.cell(m) for m in machines},
                }
                for r in rows
            ],
        }

    job = svc.jobs.start("verify", f"Verify {body.experiment}", run)
    return {"job_id": job.id}


class IngestBody(BaseModel):
    path: str
    target: str
    image_root: str | None = None


@router.post("/ingest")
def ingest(body: IngestBody, request: Request):
    svc = services(request)
    try:
        target = svc.graph.neo4j(body.target)
    except NotFound as e:
        raise HTTPException(404, str(e)) from None
    run_path = pathlib.Path(os.path.expanduser(body.path))
    try:
        dsg_path, run_dir = FileSource._resolve(run_path)
    except FileNotFoundError as e:
        raise HTTPException(400, str(e)) from None

    def run(job):
        import spark_dsg as dsg
        from heracles.graph_interface import store_mesh_path
        from heracles.utils import load_dsg_to_db

        t0 = time.time()
        job.log("info", f"Loading {dsg_path} ...")
        G = dsg.DynamicSceneGraph.load(str(dsg_path))
        job.log("ok", f"Loaded {G.num_nodes()} nodes / {G.num_edges()} edges")
        job.check_cancelled()
        current = target.count_nodes()
        answer = job.confirm(
            f"Ingesting will WIPE neo4j:{target.name} ({target.uri}), which currently holds "
            f"{current} nodes, and replace it with {dsg_path.name} ({G.num_nodes()} nodes).\n\n"
            "Type wipe to proceed.",
            kind="wipe_neo4j",
            required_text="wipe",
            yes_label="Wipe & ingest",
            no_label="Cancel",
            detail={"target": target.name, "uri": target.uri, "nodes": current},
        )
        if not answer:
            job.log("error", "Ingest aborted — database left untouched.")
            return False
        image_store = body.image_root or (
            str(run_dir / "image_store")
            if (run_dir / "image_store").is_dir()
            else None
        )
        job.log("info", f"Writing to {target.uri} (image store: {image_store or 'none'}) ...")
        load_dsg_to_db(target.uri, target.creds, G, image_store=image_store)
        db = target.db()
        target.query(
            "MERGE (m:_GraphMetadata {key: 'source'}) SET m.file_path = $p",
            p=str(dsg_path),
        )
        mesh = dsg_path.parent / "mesh.ply"
        if mesh.is_file():
            store_mesh_path(db, str(mesh))
            job.log("dim", f"mesh: {mesh}")
        version = bump_map_counters(db)
        target.invalidate()
        svc.graph.notify_version(target.id, "ingest")
        nodes = target.count_nodes()
        edges = int(target.query("MATCH ()-[r]->() RETURN count(r) AS c")[0]["c"])
        secs = round(time.time() - t0, 1)
        job.log(
            "ok",
            f"Ingested: {nodes} nodes, {edges} relationships in {secs}s (v{version})",
        )
        return {"nodes": nodes, "edges": edges, "seconds": secs, "version": version}

    job = svc.jobs.start("ingest", f"Ingest {run_dir.name} → neo4j:{target.name}", run)
    return {"job_id": job.id}


class RegisterBody(BaseModel):
    name: str
    path: str


@router.post("/register")
def register(body: RegisterBody, request: Request):
    svc = services(request)
    try:
        src = svc.graph.add_file(body.name, body.path)
    except FileNotFoundError as e:
        raise HTTPException(400, str(e)) from None
    svc.graph.notify_version(src.id, "registered")
    return src.info()
