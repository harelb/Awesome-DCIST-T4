"""Maps: listing, transfer, verification, Neo4j ingest, file-source registration (API.md §7)."""

from __future__ import annotations

import getpass
import os
import pathlib
import subprocess
import time

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..graph_service import FileSource, NotFound
from . import services

router = APIRouter(prefix="/api/maps")

BAG_EXCLUDES = ["recorded_data/"]


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


@router.post("/transfer")
def transfer(body: TransferBody, request: Request):
    svc = services(request)
    src, dst = _endpoint(svc, body.src), _endpoint(svc, body.dst)
    if src["ip"] == dst["ip"]:
        raise HTTPException(400, "source and destination are the same machine")
    root = svc.settings.output_root
    dest_name = body.as_name or body.experiment

    def run(job):
        if body.bwlimit_kbps:
            job.log("warn", "bwlimit_kbps is not supported by rsync_transfer; ignoring")
        job.log(
            "info",
            f"{src['name']}:{root}/{body.experiment} -> {dst['name']}:{root}/{dest_name}"
            + ("" if body.include_bags else " (excluding rosbags)"),
        )
        ok, method, msg = svc.helpers.rsync_transfer(
            src["user"],
            src["ip"],
            f"{root}/{body.experiment}",
            dst["user"],
            dst["ip"],
            f"{root}/{dest_name}",
            exclude=None if body.include_bags else BAG_EXCLUDES,
            progress_callback=lambda pct: job.set_progress(dst["name"], pct),
        )
        job.log("ok" if ok else "error", f"{method}: {msg[-500:] if msg else ''}")
        if ok:
            job.set_progress(dst["name"], 100)
        return {"method": method, "dest": f"{root}/{dest_name}"} if ok else False

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
        from heracles.graph_interface import bump_version, store_mesh_path
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
        image_root = body.image_root or (
            str(run_dir / "images") if (run_dir / "images").is_dir() else None
        )
        job.log("info", f"Writing to {target.uri} (images: {image_root or 'none'}) ...")
        load_dsg_to_db(target.uri, target.creds, G, image_folder_root=image_root)
        db = target.db()
        # heracles keeps no per-object image folder (only Observation nodes, and
        # only when *_meta.json carry timestamps); record it so the console can
        # serve object crops straight from Neo4j sources.
        folders = []
        for node in G.get_layer(dsg.DsgLayers.OBJECTS).nodes:
            f = getattr(node.attributes, "image_folder", "") or ""
            if f:
                base = os.path.basename(f.rstrip("/"))
                folders.append(
                    {"syms": [str(node.id), node.id.str(True), node.id.str(False)],
                     "f": os.path.join(image_root, base) if image_root else f}
                )  # fmt: skip
        if folders:
            target.query(
                "UNWIND $rows AS row MATCH (o:Object) WHERE o.nodeSymbol IN row.syms "
                "SET o.image_folder = row.f",
                rows=folders,
            )
        target.query(
            "MERGE (m:_GraphMetadata {key: 'source'}) SET m.file_path = $p",
            p=str(dsg_path),
        )
        mesh = dsg_path.parent / "mesh.ply"
        if mesh.is_file():
            store_mesh_path(db, str(mesh))
            job.log("dim", f"mesh: {mesh}")
        version = bump_version(db)
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
