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
from concurrent.futures import ThreadPoolExecutor
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


# ---------------------------------------------------------------------------
# Listing.  Each machine is scanned by dcist_launch_system.fleet.run_scan (in-process
# here, one ssh round trip elsewhere).  Directory sizes (du) are the slow part: they are
# cached per run keyed by its mtime signature (memory + a JSON file on this host, never
# on the robots), measured within a short budget during GET /api/maps, and whatever is
# left is measured by a background thread; the response lists those machines under
# ``pending`` so the page polls again.
# ---------------------------------------------------------------------------

LOCAL_BUDGET_S = 3.0
REMOTE_BUDGET_S = 4.0
REMOTE_TIMEOUT_S = 20
FILL_TIMEOUT_S = 600
_CACHE_FILE = pathlib.Path(os.path.expanduser("~/.cache/adt4_console/map_sizes.json"))
_cache_lock = threading.Lock()
_size_cache: dict[str, dict[str, list]] | None = None  # machine -> key -> [sig, size, store]
_filling: set[str] = set()


def _cache() -> dict[str, dict[str, list]]:
    global _size_cache
    if _size_cache is None:
        try:
            import json

            data = json.loads(_CACHE_FILE.read_text())
            _size_cache = data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            _size_cache = {}
    return _size_cache


def _cache_get(machine: str) -> dict[str, list]:
    with _cache_lock:
        return dict(_cache().get(machine, {}))


def _cache_put(machine: str, items: dict[str, list]) -> None:
    import json

    if not items:
        return
    with _cache_lock:
        _cache().setdefault(machine, {}).update(items)
        try:
            _CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
            tmp = _CACHE_FILE.with_suffix(".tmp")
            tmp.write_text(json.dumps(_cache()))
            tmp.replace(_CACHE_FILE)
        except OSError:
            pass


def _sized(entries: list[dict], key=lambda e: e["name"]) -> dict[str, list]:
    """Cache rows for the entries that got a size."""
    return {
        key(e): [e["signature"], e["size_bytes"], (e.get("image_store") or {}).get("size_bytes")]
        for e in entries
        if e.get("size_bytes") is not None and "signature" in e
    }


def _local_dirs(svc) -> list[pathlib.Path]:
    root = svc.settings.output_root_path
    dirs = (
        sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("."))
        if root.is_dir()
        else []
    )
    extra = [s.run_dir for s in svc.graph.sources.values() if isinstance(s, FileSource)]
    return dirs + [d for d in extra if d not in dirs and d.is_dir()]


def _scan_local(dirs, machine: str, budget: float | None) -> list[dict]:
    from dcist_launch_system.fleet.run_scan import scan_run

    cache = _cache_get(machine)
    deadline = None if budget is None else time.monotonic() + budget
    out = []
    for d in dirs:
        left = None if deadline is None else deadline - time.monotonic()
        known = {d.name: cache[str(d)]} if str(d) in cache else None
        try:
            out.append(scan_run(str(d), known, measure=left is None or left > 0.05, timeout=left))
        except Exception as e:  # noqa: BLE001
            out.append({"name": d.name, "path": str(d), "subdirs": [], "error": str(e)})
    _cache_put(machine, _sized(out, key=lambda e: e["path"]))
    return out


def _budget(machine: str, budget: float) -> float:
    """No foreground du while a background fill measures this machine."""
    with _cache_lock:
        return 0.0 if machine in _filling else budget


def _start_fill(machine: str, fn) -> None:
    """Measure the pending sizes of ``machine`` in the background (one at a time)."""
    with _cache_lock:
        if machine in _filling:
            return
        _filling.add(machine)

    def run():
        try:
            fn()
        except Exception:  # noqa: BLE001 - best effort; the next listing retries
            pass
        finally:
            with _cache_lock:
                _filling.discard(machine)

    threading.Thread(target=run, name=f"map-sizes-{machine}", daemon=True).start()


def _finish(entries: list[dict]) -> list[dict]:
    from dcist_launch_system.fleet.maps import finish_entry

    return [finish_entry(e) for e in entries]


def list_local(svc, pending: set | None = None) -> list[dict]:
    machine = _local_name(svc)
    dirs = _local_dirs(svc)
    out = _scan_local(dirs, machine, _budget(machine, LOCAL_BUDGET_S))
    todo = [pathlib.Path(e["path"]) for e in out if e.get("size_pending")]
    if todo:
        if pending is not None:
            pending.add(machine)
        _start_fill(machine, lambda: _scan_local(todo, machine, None))
    return _finish(out)


def list_remote(svc, m: dict, pending: set | None = None) -> list[dict]:
    from dcist_launch_system.fleet.maps import parse_scan_output, remote_scan_command

    h, root, name = svc.helpers, svc.settings.output_root, m["name"]
    cache = _cache_get(name)
    rc, stdout, stderr = h.ssh_cmd(
        m["user"], m["ip"], remote_scan_command(root, cache, _budget(name, REMOTE_BUDGET_S)),
        timeout=REMOTE_TIMEOUT_S,
    )
    if rc != 0:
        raise RuntimeError(f"listing failed (rc {rc}): {(stderr or stdout)[-300:]}")
    runs = parse_scan_output(stdout)
    _cache_put(name, _sized(runs))
    todo = [e["name"] for e in runs if e.get("size_pending")]
    if todo:
        if pending is not None:
            pending.add(name)

        def fill():
            rc, out, _ = h.ssh_cmd(
                m["user"], m["ip"],
                remote_scan_command(root, _cache_get(name), None, todo),
                timeout=FILL_TIMEOUT_S,
            )
            if rc == 0:
                _cache_put(name, _sized(parse_scan_output(out)))

        _start_fill(name, fill)
    for e in runs:
        e["abs_path"] = e.get("path")
        e["path"] = f"{root}/{e['name']}"
    return _finish(runs)


@router.get("")
def list_maps(request: Request):
    svc = services(request)
    h = svc.helpers
    out: dict = {}
    pending: set[str] = set()
    with svc.fleet.lock:
        remotes = [
            dict(m)
            for m in svc.fleet.runtime_config.values()
            if m.get("online") and not m.get("local")
        ]

    def fetch(m):
        return list_remote(svc, m, pending)

    # remote ssh scans run while the local scan does its own budgeted pass
    with ThreadPoolExecutor(max_workers=1) as pool:
        remote_future = pool.submit(lambda: h.run_parallel(fetch, remotes) if remotes else [])
        try:
            out[_local_name(svc)] = list_local(svc, pending)
        except Exception as e:  # noqa: BLE001
            out[_local_name(svc)] = {"error": str(e)}
        for m, r in remote_future.result():
            out[m["name"]] = r if isinstance(r, list) else {"error": str(r)}
    from dcist_launch_system.fleet.run_identity import pair_copies

    pair_copies(out, _local_name(svc))
    return {"machines": out, "pending": sorted(pending), "base": _local_name(svc)}


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
    # transfer even when the copy on the destination is already up to date
    force: bool = False


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


# ---------------------------------------------------------------------------
# Transfer planning: run identity decides where a copy goes and whether it is needed
# (dcist_launch_system.fleet.run_identity).  A robot's run lands on the base as
# <experiment>__<robot>; a second transfer of an unchanged run is a no-op; a name
# taken by another run gets a __N suffix.
# ---------------------------------------------------------------------------


def _scan_one(svc, m: dict, name: str, root: str | None = None) -> dict | None:
    """Full run_scan entry (no du) of ``<root>/<name>`` on ``m``; None if absent."""
    from dcist_launch_system.fleet.maps import parse_scan_output, remote_scan_command
    from dcist_launch_system.fleet.run_scan import scan_run

    root = root or svc.settings.output_root
    if _is_local(svc, m):
        path = pathlib.Path(os.path.expanduser(root)) / name
        if not path.is_dir():
            return None
        cache = _cache_get(m["name"])
        known = {name: cache[str(path)]} if str(path) in cache else None
        e = scan_run(str(path), known, measure=False)
        e["abs_path"] = str(path)
        return e
    rc, out, err = svc.helpers.ssh_cmd(
        m["user"], m["ip"],
        remote_scan_command(root, _cache_get(m["name"]), 0, [name]),
        timeout=REMOTE_TIMEOUT_S,
    )
    if rc != 0:
        raise HTTPException(502, f"{m['name']}: scan failed: {(err or out)[-300:]}")
    runs = parse_scan_output(out)
    if not runs:
        return None
    e = runs[0]
    e["abs_path"] = e.get("path")
    return e


def _dest_entries(svc, m: dict) -> list[dict]:
    """Identity listing (light scan) of the output root on ``m``."""
    from dcist_launch_system.fleet.maps import parse_scan_output, remote_scan_command
    from dcist_launch_system.fleet.run_scan import scan_root

    root = svc.settings.output_root
    if _is_local(svc, m):
        return scan_root(root, light=True)
    rc, out, err = svc.helpers.ssh_cmd(
        m["user"], m["ip"], remote_scan_command(root, None, 0, None, light=True),
        timeout=REMOTE_TIMEOUT_S,
    )
    if rc != 0:
        raise HTTPException(502, f"{m['name']}: listing failed: {(err or out)[-300:]}")
    return parse_scan_output(out)


def plan_transfer(svc, src: dict, dst: dict, experiment: str, as_name: str | None,
                  images: str, compression: str) -> dict:
    """Where the copy goes and what has to move.

    ``status``: ``new`` (no copy yet), ``update`` (copy exists, source changed or the
    image settings differ), ``up_to_date`` (nothing to do) or ``collision`` (the
    natural name holds a different run; a ``__N`` name was picked).
    """
    from dcist_launch_system.fleet import run_identity as ri
    from dcist_launch_system.fleet.run_scan import manifest

    source = _scan_one(svc, src, experiment)
    if source is None:
        raise HTTPException(404, f"{src['name']}: no run {experiment}")
    run_id = ri.canonical_id(source)
    dests = _dest_entries(svc, dst)
    if _is_local(svc, dst):  # legacy copies (no ids) are matched by their files
        same = next((d for d in dests if d.get("name") == experiment and not d.get("origin")), None)
        if same is not None:
            same["manifest"] = manifest(same["path"])
            same["dsg_mtime"] = _mtime_of(pathlib.Path(same["path"]) / "hydra/backend/dsg.json")
    copy, how = ri.find_copy(source, src["name"], dests)
    robot = (source.get("identity") or {}).get("robot") or src["name"]
    plain = images == "all" and compression == "none"
    out = {
        "run_id": run_id,
        "identity_source": (source.get("identity") or {}).get("source"),
        "robot": robot,
        "source_path": source.get("abs_path"),
        "source": source,
        "copy": None,
        "matched_by": how,
    }
    if copy is not None:
        origin = copy.get("origin") if how != "contents" else None
        state = ri.copy_state(source, origin, copy)
        same_mode = bool(origin) and origin.get("images") == images and (
            origin.get("compression") == compression
        )
        send_store = (not plain) and ri.store_needs_send(source, origin, images, compression)
        status = "up_to_date" if state == ri.IN_SYNC and same_mode else "update"
        note = (
            f"{dst['name']}:{copy['name']} is already an up-to-date copy"
            if status == "up_to_date"
            else f"updating {dst['name']}:{copy['name']}"
            + (" (robot changed since the copy)" if state == ri.OUTDATED else "")
            + ("" if same_mode or not origin else " (image settings changed)")
            + (" (adopting a copy made before run ids)" if how == "contents" else "")
        )
        out.update(
            copy={"name": copy["name"], "state": state, "origin": copy.get("origin")},
            dest_name=copy["name"],
            status=status,
            state=state,
            send_store=send_store,
            note=note,
        )
    else:
        wanted = as_name or ri.copy_name(experiment, robot)
        existing = {d["name"]: ri.entry_ids(d) for d in dests if d.get("name")}
        name, st = ri.choose_dest_name(wanted, existing, run_id)
        out.update(
            dest_name=name,
            status="collision" if st.startswith("collision") else "new",
            state=ri.NOT_COPIED,
            send_store=(not plain) and images != "none" and bool(source.get("has_image_store")),
            note=(
                f"{dst['name']}:{wanted} holds a different run; copying to {name} instead"
                if st.startswith("collision")
                else f"new copy {dst['name']}:{name}"
            ),
        )
    out["dest_path"] = f"{svc.settings.output_root}/{out['dest_name']}"
    return out


def _mtime_of(path: pathlib.Path):
    try:
        return path.stat().st_mtime
    except OSError:
        return None


def write_origin(svc, m: dict, dest_path: str, origin: dict) -> None:
    from dcist_launch_system.fleet import run_identity as ri

    if _is_local(svc, m):
        ri.write_json_atomic(os.path.join(os.path.expanduser(dest_path), ri.ORIGIN), origin)
        return
    rc, out, err = svc.helpers.ssh_cmd(
        m["user"], m["ip"], ri.remote_write_command(f"{dest_path}/{ri.ORIGIN}", origin), timeout=15
    )
    if rc != 0:
        raise RuntimeError(f"could not write {ri.ORIGIN} on {m['name']}: {(err or out)[-200:]}")


def find_local_copy(svc, run_id: str) -> dict | None:
    """This host's run (original or copy) holding ``run_id`` with a saved DSG."""
    from dcist_launch_system.fleet import run_identity as ri
    from dcist_launch_system.fleet.run_scan import scan_root

    for e in scan_root(svc.settings.output_root, light=True):
        if run_id in ri.entry_ids(e) and "hydra" in e.get("subdirs", []):
            return e
    return None


@router.get("/transfer/plan")
def transfer_plan(
    request: Request,
    src: str,
    experiment: str,
    dst: str | None = None,
    as_name: str | None = None,
    images: Literal["keyframes", "all", "none"] = "all",
    compression: Literal["none", "lossless", "fast"] = "none",
):
    """What POST /transfer would do (destination name, new/update/up_to_date)."""
    svc = services(request)
    s, d = _endpoint(svc, src), _endpoint(svc, dst)
    if s["ip"] == d["ip"]:
        raise HTTPException(400, "source and destination are the same machine")
    plan = plan_transfer(svc, s, d, experiment, as_name, images, compression)
    plan.pop("source", None)
    return plan


@router.post("/transfer")
def transfer(body: TransferBody, request: Request):
    svc = services(request)
    src, dst = _endpoint(svc, body.src), _endpoint(svc, body.dst)
    if src["ip"] == dst["ip"]:
        raise HTTPException(400, "source and destination are the same machine")
    root = svc.settings.output_root

    plain = body.images == "all" and body.compression == "none"
    if body.images == "none" and body.compression != "none":
        raise HTTPException(400, "compression needs images=keyframes|all")

    def run(job):
        from dcist_launch_system.fleet import run_identity as ri

        plan = plan_transfer(
            svc, src, dst, body.experiment, body.as_name, body.images, body.compression
        )
        dest_name = plan["dest_name"]
        job.log(
            "warn" if plan["status"] == "collision" else "dim",
            f"run {plan['run_id']} ({plan['identity_source']}): {plan['note']}",
        )
        if plan["status"] == "up_to_date" and not body.force:
            job.log("ok", f"Already up to date: {dst['name']}:{root}/{dest_name}")
            job.set_progress(dst["name"], 100)
            _attach_pending_prior(svc, job, dst, plan)
            return {
                "up_to_date": True,
                "status": "up_to_date",
                "dest": f"{root}/{dest_name}",
                "dest_name": dest_name,
                "run_id": plan["run_id"],
            }
        if body.bwlimit_kbps:
            job.log("warn", "bwlimit_kbps is not supported by rsync_transfer; ignoring")
        exclude = [] if body.include_bags else list(BAG_EXCLUDES)
        if not plain:
            exclude.append(f"/{STORE_DIR}/")
        send_store = not plain and body.images != "none" and (plan["send_store"] or body.force)
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
        result = {
            "method": method,
            "dest": f"{root}/{dest_name}",
            "dest_name": dest_name,
            "status": plan["status"],
            "run_id": plan["run_id"],
        }
        if not plain and body.images != "none" and not send_store:
            job.log("dim", "image store unchanged since the last copy; not re-packed")
            result["image_store"] = "unchanged"
        if send_store:
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
        previous = (plan.get("copy") or {}).get("origin")
        origin = ri.make_origin(
            plan["source"], src["name"], body.images, body.compression, previous
        )
        if body.include_bags:
            origin["include_bags"] = True
        try:
            write_origin(svc, dst, f"{root}/{dest_name}", origin)
            job.log("dim", f"wrote {ri.ORIGIN} (copy of {src['name']}:{origin['source_path']})")
        except Exception as e:  # noqa: BLE001
            job.log("warn", str(e))
        job.set_progress(dst["name"], 100)
        _attach_pending_prior(svc, job, dst, plan)
        return result

    job = svc.jobs.start(
        "transfer", f"Transfer {body.experiment} {src['name']}→{dst['name']}", run
    )
    return {"job_id": job.id}


def _attach_pending_prior(svc, job, dst: dict, plan: dict) -> None:
    """The active launch's prior was missing here: register it now that it is copied."""
    from ..launch_state import register_prior

    if svc.launch_state is None or not _is_local(svc, dst):
        return
    state = svc.launch_state.get()
    prior = (state or {}).get("prior")
    if not prior or prior.get("source_id") or prior.get("run_id") != plan["run_id"]:
        return
    prior["local_path"] = os.path.expanduser(plan["dest_path"])
    sid = register_prior(svc.graph, prior)
    if sid:
        prior.update(source_id=sid, pending=False)
        state["prior"] = prior
        svc.launch_state.set(state)
        svc.graph.notify_version(sid, "registered")
        job.log("ok", f"prior map of the active launch is now in the viewer ({prior['label']})")


class VerifyBody(BaseModel):
    experiment: str
    machines: list[str] | None = None
    # machine -> run name there, when copies are named differently
    # (hamilton: map_x, willow: map_x__hamilton); others use ``experiment``
    names: dict[str, str] | None = None


@router.post("/verify")
def verify(body: VerifyBody, request: Request):
    from dcist_launch_system.fleet.maps import compare_hashes

    svc = services(request)
    with svc.fleet.lock:
        names = body.machines or (
            list(body.names)
            if body.names
            else [n for n, m in svc.fleet.runtime_config.items() if m.get("online")]
        )
    targets = [_endpoint(svc, n) for n in names]
    run_names = dict(body.names or {})
    if not targets:
        raise HTTPException(400, "no machines to verify")

    def run(job):
        def fetch(m):
            return svc.helpers.hash_remote_experiment(
                m["user"], m["ip"], svc.settings.output_root,
                run_names.get(m["name"], body.experiment),
            )

        if run_names:
            job.log(
                "dim",
                "comparing " + " ↔ ".join(
                    f"{m['name']}:{run_names.get(m['name'], body.experiment)}" for m in targets
                ),
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
