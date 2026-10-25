"""Map editing on a Neo4j source (API.md §17), on heracles ``graph_edit``.

Every edit is one heracles ``GraphUnitOfWork`` (revision check, ``GraphMutation`` audit
with provenance, both map-version counters bumped in the same transaction), so the
heracles DSG publisher -- and through it plan repair on the robots -- sees whole edits
only.  After a commit the source is invalidated and ``graph/<src>/version`` +
``edit/<src>`` are published on the hub.

**Staged vs immediate.**  While a launch is active (``launch_state``), edits are
*staged*: validated with a dry run (new nodes get their symbol pinned then, so later
staged ops can refer to them) and kept in a per-source list until the operator publishes
them in one unit of work with a typed confirm.  Offline, edits apply immediately.
``settings.map_edit_mode`` forces either behaviour.

**Snapshots** live in ``<snapshots_dir>/<src name>/<id>/`` (heracles ``export_snapshot``:
``dsg.json`` + ``extras.json`` + ``meta.json`` + our ``info.json``).  A restore is a job
with a typed confirm ("restore") because it replaces the whole map.
"""

from __future__ import annotations

import io
import json
import logging
import os
import pathlib
import re
import shutil
import tempfile
import threading
import time
import zipfile

log = logging.getLogger(__name__)

PUBLISH_CONFIRM = "publish"
RESTORE_CONFIRM = "restore"
DELETE_CONFIRM = "delete"


class EditUnavailable(Exception):
    """The source cannot be edited (not Neo4j, heracles without graph_edit)."""


def _ge():
    try:
        from heracles import graph_edit
    except ImportError as e:
        raise EditUnavailable(f"heracles.graph_edit unavailable: {e}") from None
    return graph_edit


def _slug(text: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_.-]+", "-", text.strip()).strip("-.")
    return s[:48] or "snapshot"


class EditService:
    def __init__(self, settings, graph, hub=None, jobs=None, launch_state=None):
        self.settings = settings
        self.graph = graph
        self.hub = hub
        self.jobs = jobs
        self.launch_state = launch_state  # callable -> launch record or None
        d = getattr(settings, "snapshots_dir", None)
        self.snap_root = pathlib.Path(os.path.expanduser(d)) if d else pathlib.Path(
            tempfile.gettempdir()) / "adt4_console_snapshots"  # fmt: skip
        self._lock = threading.RLock()
        self._staged: dict[str, list[dict]] = {}  # src id -> [{op, summary, t, author}]

    # ------------------------------------------------------------- sources
    def source(self, src_id: str):
        src = self.graph.get(src_id)
        if getattr(src, "kind", None) != "neo4j":
            raise EditUnavailable(
                f"{src_id} is not a Neo4j map; only Neo4j maps can be edited (ingest it first)"
            )
        return src

    def _db(self, src_id: str):
        return self.source(src_id).db()

    # ---------------------------------------------------------------- mode
    def live(self) -> dict | None:
        """The active launch (edits stage) or None."""
        try:
            return self.launch_state() if self.launch_state is not None else None
        except Exception:  # noqa: BLE001
            return None

    def staged_mode(self) -> bool:
        mode = getattr(self.settings, "map_edit_mode", "auto")
        if mode == "staged":
            return True
        if mode == "immediate":
            return False
        return self.live() is not None

    def state(self, src_id: str) -> dict:
        ge = _ge()
        db = self._db(src_id)
        from heracles.graph_interface import get_map_version, read_version

        live = self.live()
        hist = ge.edit_history(db, limit=50)
        return {
            "src": src_id,
            "mode": "staged" if self.staged_mode() else "immediate",
            "live": bool(live),
            "launch": {k: live.get(k) for k in ("experiment", "robots", "started_at")} if live else None,
            "revision": ge.graph_revision(db),
            "state_version": read_version(db),
            "map_version": get_map_version(db),
            "staged": self.staged(src_id),
            "can_undo": any(h["undoable"] and not h["undo_of"] and h["kind"] == "map_edit" for h in hist),
        }

    # ---------------------------------------------------------------- edits
    def _provenance(self, src_id: str, author: dict | None) -> dict:
        p = {"source": "adt4_console", "src": src_id, "operator": "operator", "via": "ui"}
        p.update({k: v for k, v in (author or {}).items() if v is not None})
        return p

    def _after_commit(self, src_id: str, out: dict, reason: str) -> None:
        if out.get("dry_run") or not out.get("mutation_id"):
            return
        try:
            self.graph.get(src_id).invalidate()
        except Exception:  # noqa: BLE001
            pass
        self.graph.notify_version(src_id, reason)
        if self.hub is not None:
            self.hub.publish(f"edit/{src_id}", {"kind": "commit", "mutation_id": out["mutation_id"],
                                                "revision": out["revision"], "summary": out["summary"],
                                                "map_version": out.get("map_version")})  # fmt: skip

    def apply(self, src_id: str, ops: list[dict], *, author: dict | None = None,
              expected_revision: int | None = None, summary: str | None = None,
              dry_run: bool = False, stage: bool | None = None) -> dict:  # fmt: skip
        """Apply (or, in staged mode, stage) ``ops``; ``stage=False`` forces a commit
        (used by publish), ``dry_run`` never writes."""
        ge = _ge()
        db = self._db(src_id)
        if not dry_run and (self.staged_mode() if stage is None else stage):
            return self._stage(src_id, ops, author)
        out = ge.apply_edit(db, ops, provenance=self._provenance(src_id, author),
                            expected_revision=expected_revision, summary=summary,
                            dry_run=dry_run)  # fmt: skip
        self._after_commit(src_id, out, "map edit")
        return {**out, "staged": False}

    def undo(self, src_id: str, mutation_id: str | None = None, *, author: dict | None = None,
             force: bool = False) -> dict:  # fmt: skip
        ge = _ge()
        out = ge.undo_edit(self._db(src_id), mutation_id, provenance=self._provenance(src_id, author),
                           force=force)  # fmt: skip
        self._after_commit(src_id, out, "map edit undo")
        return out

    def history(self, src_id: str, limit: int = 50) -> list[dict]:
        return _ge().edit_history(self._db(src_id), limit=limit)

    def regions(self, src_id: str) -> list[dict]:
        return _ge().regions(self._db(src_id))

    def edits_since(self, src_id: str, revision: int) -> int:
        return _ge().edits_since(self._db(src_id), revision)

    def map_stamp(self, src_id: str) -> dict:
        """What a mission records: the edit revision and the map version it was planned on."""
        from heracles.graph_interface import get_map_version

        db = self._db(src_id)
        return {"map_revision": _ge().graph_revision(db), "map_version": get_map_version(db)}

    # --------------------------------------------------------------- staging
    def staged(self, src_id: str) -> list[dict]:
        with self._lock:
            return [dict(s) for s in self._staged.get(src_id, [])]

    def _stage(self, src_id: str, ops: list[dict], author: dict | None) -> dict:
        """Validate ``ops`` after the already-staged ones (dry run) and keep them."""
        ge = _ge()
        db = self._db(src_id)
        ops = [dict(o) for o in ops]
        with self._lock:
            staged = self._staged.setdefault(src_id, [])
            before = [s["op"] for s in staged]
            prov = self._provenance(src_id, author)
            dry = ge.apply_edit(db, before + ops, provenance=prov, dry_run=True)
            n_before = len(ge.apply_edit(db, before, provenance=prov, dry_run=True)["changes"]) if before else 0
            mine = dry["changes"][n_before:]
            lines = ge.summarize(mine)
            # pin the symbols the dry run allocated, so later staged ops can name them
            created = iter(c["symbol"] for c in mine if c["kind"] in ("create", "group"))
            for o in ops:
                if o.get("op") in ("create_node", "group"):
                    sym = next(created, None)
                    if not o.get("symbol"):
                        o["symbol"] = sym
            now = time.time()
            for i, o in enumerate(ops):
                staged.append({"op": o, "t": now, "author": (author or {}).get("operator"),
                               "summary": lines if i == 0 else []})  # fmt: skip
            out = {"staged": True, "count": len(staged), "summary": lines,
                   "staged_summary": dry["summary"], "affected": dry["affected"]}  # fmt: skip
        if self.hub is not None:
            self.hub.publish(f"edit/{src_id}", {"kind": "staged", "count": len(staged)})
        return out

    def discard(self, src_id: str, index: int | None = None) -> dict:
        with self._lock:
            staged = self._staged.get(src_id, [])
            if index is None:
                staged.clear()
            elif 0 <= index < len(staged):
                staged.pop(index)
            else:
                raise IndexError(index)
            n = len(staged)
        if self.hub is not None:
            self.hub.publish(f"edit/{src_id}", {"kind": "staged", "count": n})
        return {"count": n}

    def preview_staged(self, src_id: str) -> dict:
        ops = [s["op"] for s in self.staged(src_id)]
        if not ops:
            return {"count": 0, "summary": [], "affected": []}
        dry = _ge().apply_edit(self._db(src_id), ops, provenance={}, dry_run=True)
        return {"count": len(ops), "summary": dry["summary"], "affected": dry["affected"]}

    def publish(self, src_id: str, confirm: str, *, author: dict | None = None,
                expected_revision: int | None = None) -> dict:  # fmt: skip
        """Commit every staged op in one unit of work (typed confirm "publish")."""
        if confirm != PUBLISH_CONFIRM:
            raise ValueError(f'type "{PUBLISH_CONFIRM}" to publish the staged edits')
        with self._lock:
            ops = [s["op"] for s in self._staged.get(src_id, [])]
            if not ops:
                raise ValueError("nothing staged")
            out = self.apply(src_id, ops, author={**(author or {}), "via": "publish"},
                             expected_revision=expected_revision, stage=False,
                             summary=None)  # fmt: skip
            self._staged[src_id] = []
        if self.hub is not None:
            self.hub.publish(f"edit/{src_id}", {"kind": "staged", "count": 0})
        return {**out, "published": len(ops)}

    # ------------------------------------------------------------- snapshots
    def _snap_dir(self, src_id: str) -> pathlib.Path:
        return self.snap_root / _slug(self.source(src_id).name)

    def snapshots(self, src_id: str) -> list[dict]:
        d = self._snap_dir(src_id)
        out = []
        for p in sorted(d.glob("*/info.json")) if d.is_dir() else []:
            try:
                out.append(json.loads(p.read_text()))
            except ValueError:
                continue
        return sorted(out, key=lambda s: -s.get("created_at", 0))

    def snapshot(self, src_id: str, name: str, *, include_mesh: bool = False,
                 author: dict | None = None, mission: str | None = None) -> dict:  # fmt: skip
        ge = _ge()
        db = self._db(src_id)
        name = (name or "").strip() or time.strftime("snapshot %Y-%m-%d %H:%M")
        sid = f"{time.strftime('%Y%m%d-%H%M%S')}-{_slug(name)}"
        d = self._snap_dir(src_id) / sid
        meta = ge.export_snapshot(db, d, include_mesh=include_mesh)
        size = sum(p.stat().st_size for p in d.iterdir() if p.is_file())
        info = {"id": sid, "name": name, "src": src_id, "created_at": time.time(), "bytes": size,
                "author": (author or {}).get("operator"), "mission": mission, **meta}  # fmt: skip
        (d / "info.json").write_text(json.dumps(info))
        if self.hub is not None:
            self.hub.publish(f"edit/{src_id}", {"kind": "snapshot", "id": sid})
        return info

    def _snapshot_path(self, src_id: str, snap_id: str) -> pathlib.Path:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", snap_id or "") or snap_id.startswith("."):
            raise KeyError(snap_id)
        d = self._snap_dir(src_id) / snap_id
        if not (d / "info.json").is_file():
            raise KeyError(snap_id)
        return d

    def snapshot_info(self, src_id: str, snap_id: str) -> dict:
        return json.loads((self._snapshot_path(src_id, snap_id) / "info.json").read_text())

    def snapshot_zip(self, src_id: str, snap_id: str) -> bytes:
        d = self._snapshot_path(src_id, snap_id)
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for p in sorted(d.iterdir()):
                if p.is_file():
                    z.write(p, f"{snap_id}/{p.name}")
        return buf.getvalue()

    def delete_snapshot(self, src_id: str, snap_id: str) -> None:
        shutil.rmtree(self._snapshot_path(src_id, snap_id))

    def restore(self, src_id: str, snap_id: str, *, author: dict | None = None):
        """Start the restore job; it asks for the typed confirm "restore" first."""
        if self.jobs is None:
            raise RuntimeError("no job manager")
        d = self._snapshot_path(src_id, snap_id)
        info = self.snapshot_info(src_id, snap_id)
        src = self.source(src_id)

        def run(job):
            ge = _ge()
            db = src.db()
            now = ge.map_counts(db)
            answer = job.confirm(
                f"Restoring snapshot '{info['name']}' REPLACES the map in neo4j:{src.name} "
                f"({src.uri}): {now.get('Object', 0)} objects, {now.get('MeshPlace', 0) + now.get('Place', 0)} "
                f"places, {now.get('Room', 0)} regions now -> {info['counts'].get('Object', 0)} objects, "
                f"{info['counts'].get('MeshPlace', 0) + info['counts'].get('Place', 0)} places, "
                f"{info['counts'].get('Room', 0)} regions.\n\nType restore to proceed.",
                kind="restore_snapshot",
                required_text=RESTORE_CONFIRM,
                yes_label="Restore",
                no_label="Cancel",
                detail={"target": src.name, "uri": src.uri, "snapshot": snap_id, "now": now,
                        "snapshot_counts": info["counts"]},  # fmt: skip
            )
            if not answer:
                job.log("error", "Restore aborted - database left untouched.")
                return False
            job.log("info", f"Restoring {snap_id} into {src.uri} ...")
            counts = ge.restore_snapshot(db, d, log=lambda m: job.log("dim", m))
            src.invalidate()
            self.graph.notify_version(src.id, "snapshot restore")
            if self.hub is not None:
                self.hub.publish(f"edit/{src.id}", {"kind": "restore", "id": snap_id})
            job.log("ok", f"Restored: {counts}")
            return {"counts": counts, "snapshot": snap_id}

        return self.jobs.start("restore", f"Restore {info['name']} -> neo4j:{src.name}", run)
