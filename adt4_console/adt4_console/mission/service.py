"""Mission service: storage, previews, simulation runs and live runs (API.md §15)."""

from __future__ import annotations

import json
import logging
import os
import pathlib
import re
import threading
import time

from .model import Mission
from .paths import PlaceGraph
from .planning import World, preview
from .sim import SimRunner

log = logging.getLogger(__name__)


class MissionService:
    def __init__(self, settings, hub, graph, geo, live=None, ros=None, quad=None):
        self.settings = settings
        self.hub = hub
        self.graph = graph
        self.geo = geo
        self.live = live
        self.ros = ros
        self.quad = quad
        self.edits = None  # edit_service.EditService: the map revision a mission plans on
        d = getattr(settings, "missions_dir", None)
        self.dir = pathlib.Path(os.path.expanduser(d)) if d else None
        img = getattr(settings, "quad_images_dir", None)
        self.images_dir = pathlib.Path(os.path.expanduser(img)) if img else None
        self._lock = threading.RLock()
        self.missions: dict[str, Mission] = {}
        self.runs: dict[str, SimRunner] = {}  # mission id -> sim or live runner
        self._graphs: dict[str, tuple] = {}  # src -> (version, PlaceGraph)
        self._mosaics: dict[str, object] = {}
        self._quick: dict[str, tuple] = {}  # mission id -> ((id, revision, map version), errors)
        self._load_saved()

    # ------------------------------------------------------------ storage
    def _load_saved(self):
        if self.dir is None or not self.dir.is_dir():
            return
        for p in sorted(self.dir.glob("*.json")):
            try:
                m = Mission.model_validate_json(p.read_text())
                self.missions[m.id] = m
            except Exception as e:  # noqa: BLE001
                log.warning("skipping mission %s: %s", p, e)
        # missions stored before names were unique: the later ones get "(2)", ...
        loaded = sorted(self.missions.values(), key=lambda m: m.created_at)
        self.missions = {}
        for m in loaded:
            if m.name:
                m.name = self._unique_name(m.name, m.id)
            self.missions[m.id] = m

    def _save(self, m: Mission):
        if self.dir is None:
            return
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = self.dir / f"{m.id}.json.part"
        tmp.write_text(m.model_dump_json(indent=1))
        tmp.replace(self.dir / f"{m.id}.json")

    def list(self) -> list[dict]:
        with self._lock:
            ms = sorted(self.missions.values(), key=lambda m: -m.created_at)
            return [
                {"id": m.id, "name": m.name or m.text[:60], "text": m.text, "agents": len(m.agents),
                 "tasks": len(m.tasks), "created_at": m.created_at, "revision": m.revision,
                 "planner": m.planner, "run": self.runs[m.id].kind if m.id in self.runs else None,
                 "saved": m.saved, "decisions": len(m.decisions), "has_chat": bool(m.chat),
                 "errors": self._quick_errors(m)}  # fmt: skip
                for m in ms
            ]

    def _quick_errors(self, m: Mission) -> int:
        """Errors the mission list can flag without planning paths: symbol targets
        the map no longer has (a re-ingest renumbers nodes), agents without a start.
        Cached per (revision, map version)."""
        try:
            pg = self.place_graph(m.src)
            v = self.graph.get(m.src).version() if pg is not None else None
        except Exception:  # noqa: BLE001 — an unloadable map: the preview says why
            return 0
        key = (m.id, m.revision, v)
        hit = self._quick.get(m.id)
        if hit and hit[0] == key:
            return hit[1]
        n = sum(1 for a in m.agents if not a.start and not a.binding)
        if pg is not None and not pg.empty:
            n += sum(1 for t in m.tasks if isinstance(t.target, str) and not t.found_query()
                     and pg.symbol_pos(t.target) is None)  # fmt: skip
        self._quick[m.id] = (key, n)
        return n

    def get(self, mid: str) -> Mission:
        with self._lock:
            if mid not in self.missions:
                raise KeyError(mid)
            return self.missions[mid]

    def put(self, m: Mission) -> dict:
        """Store a new mission (or a revision: same id, revision bumped);
        returns ``{mission, preview}``."""
        with self._lock:
            if not m.src:
                m.src = self._default_src()
            old = self.missions.get(m.id)
            if m.name and (old is None or m.name != old.name):
                m.name = self._unique_name(m.name, m.id)
            self._stamp_map(m)
            if old is not None:
                m.revision = old.revision + 1
                run = self.runs.pop(m.id, None)
                if run is not None:
                    run.stop()
            self.missions[m.id] = m
            self._save(m)
        return {"mission": m.model_dump(), "preview": self.preview(m)}

    def _unique_name(self, name: str, mid: str) -> str:
        """``name``, or ``name (2)``, ``(3)``, ... when another mission has it
        (case-insensitive; a trailing ``(n)`` of ``name`` is its own base)."""
        taken = {(o.name or "").strip().lower() for o in self.missions.values() if o.id != mid}
        if name.lower() not in taken:
            return name
        base = re.sub(r"\s*\(\d+\)$", "", name)
        n = 2
        while f"{base} ({n})".lower() in taken:
            n += 1
        return f"{base} ({n})"

    # ------------------------------------------------------- map version
    def _edit_src(self, src_id):
        if self.edits is None or not src_id or not str(src_id).startswith("neo4j:"):
            return None
        return src_id

    def _stamp_map(self, m: Mission) -> None:
        """Record the map revision/version the mission is (re)planned on."""
        sid = self._edit_src(m.src)
        if sid is None:
            return
        try:
            stamp = self.edits.map_stamp(sid)
        except Exception as e:  # noqa: BLE001
            log.warning("map stamp for %s: %s", sid, e)
            return
        m.map_revision, m.map_version = stamp["map_revision"], stamp["map_version"]

    def map_status(self, m: Mission) -> dict | None:
        """``{planned_revision, revision, planned_map_version, map_version, edits_since,
        stale, message}``; None when the map is not an editable Neo4j map."""
        sid = self._edit_src(m.src)
        if sid is None:
            return None
        try:
            now = self.edits.map_stamp(sid)
            n = self.edits.edits_since(sid, m.map_revision or 0) if m.map_version is not None else 0
        except Exception as e:  # noqa: BLE001
            log.warning("map status for %s: %s", sid, e)
            return None
        # planned before map versions were recorded: unknown, so possibly stale
        unknown = m.map_version is None
        stale = unknown or now["map_version"] != m.map_version or n > 0
        msg = None
        if unknown:
            msg = "planned before map versions were recorded, so the map may have changed"
        elif stale:
            msg = "map changed since planning" + (f" ({n} edit{'s' if n != 1 else ''})" if n else "")
        return {"planned_revision": m.map_revision, "revision": now["map_revision"],
                "planned_map_version": m.map_version, "map_version": now["map_version"],
                "edits_since": n, "stale": stale, "unknown": unknown, "message": msg,
                "snapshot": m.map_snapshot}  # fmt: skip

    def rebase(self, mid: str) -> dict:
        """Accept the current map for this mission (after re-previewing it)."""
        with self._lock:
            m = self.get(mid)
            self._stamp_map(m)
            self._save(m)
        return {"mission": m.model_dump(), "preview": self.preview(m)}

    def snapshot_map(self, mid: str, name: str | None = None) -> dict:
        """Snapshot the mission's map (API.md §17.4) and record it on the mission."""
        m = self.get(mid)
        sid = self._edit_src(m.src)
        if sid is None:
            raise ValueError("the mission's map is not an editable Neo4j map")
        info = self.edits.snapshot(sid, name or f"mission {m.name or m.id}", mission=m.id)
        with self._lock:
            m.map_snapshot = info["id"]
            self._save(m)
        return info

    def delete(self, mid: str) -> None:
        with self._lock:
            run = self.runs.pop(mid, None)
            if run is not None:
                run.stop()
            self.missions.pop(mid, None)
            if self.dir is not None:
                (self.dir / f"{mid}.json").unlink(missing_ok=True)

    def _default_src(self):
        for s in self.graph.list() if self.graph is not None else []:
            if s["kind"] == "neo4j":
                return s["id"]
        for s in self.graph.list() if self.graph is not None else []:
            if s["kind"] == "file":
                return s["id"]
        return None

    # -------------------------------------------------------------- world
    def place_graph(self, src_id: str | None) -> PlaceGraph | None:
        if not src_id or self.graph is None:
            return None
        src = self.graph.get(src_id)
        v = src.version()
        hit = self._graphs.get(src_id)
        if hit and hit[0] == v:
            return hit[1]
        pg = PlaceGraph(src.loaded())
        if not pg.empty and pg.ids:
            try:  # rasterise the traversable corridor now, not on the first plan
                pg.place_cells(pg.ids[:1])
            except Exception as e:  # noqa: BLE001 — smoothing falls back to raw paths
                log.warning("path smoothing corridor unavailable: %s", e)
        self._graphs[src_id] = (v, pg)
        return pg

    def world_info(self, src_id: str | None) -> dict:
        """Map extent and a sensible default start (a place of the largest
        connected component, nearest the extent's centre) for new agents."""
        src_id = src_id or self._default_src()
        pg = self.place_graph(src_id) if src_id else None
        if pg is None or pg.empty:
            return {"src": src_id, "bounds": None, "start_hint": None}
        import networkx as nx

        lo, hi = pg.P.min(0), pg.P.max(0)
        giant = max(nx.connected_components(pg.nx), key=len)
        hint = pg.node_xy(pg.nearest((lo + hi) / 2, giant))
        return {"src": src_id, "bounds": [float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])],
                "start_hint": hint, "objects": len(pg.objects)}  # fmt: skip

    def robot_pose(self, robot: str):
        if self.live is None:
            return None
        st = self.live.state(robot) or {}
        p = st.get("pose") or {}
        if "x" in p and "y" in p:
            return [p["x"], p["y"], p.get("z", 0.0), p.get("yaw", 0.0)]
        return None

    def world(self, m: Mission) -> World:
        try:
            g = self.place_graph(m.src)
        except Exception as e:  # noqa: BLE001
            log.warning("no place graph for %s: %s", m.src, e)
            g = None
        return World(graph=g, robot_pose=self.robot_pose, quad_available=True)

    def preview(self, m: Mission | str) -> dict:
        if isinstance(m, str):
            m = self.get(m)
        out = preview(m, self.world(m))
        a = self.geo.anchor(m.src) if self.geo is not None else None
        out["geo"] = a.to_dict() if a is not None else None
        st = self.map_status(m)
        out["map"] = st
        if st and st["stale"]:
            out["issues"].append({"level": "warning", "code": "map_stale",
                                  "msg": f"{st['message']}: re-preview, then accept the current map or re-plan"})  # fmt: skip
        return out

    # --------------------------------------------------------- simulation
    def _publish(self, topic, data):
        if self.hub is not None:
            self.hub.publish(topic, data)

    def run(self, mid: str) -> SimRunner | None:
        return self.runs.get(mid)

    def sim(self, mid: str, cmd: str, value=None, auto_approve: bool | None = None) -> dict:
        """``cmd="replay"``: a fresh run that re-applies the saved operator
        decisions at their sim times (the mission plays out as it was approved)."""
        with self._lock:
            m = self.get(mid)
            run = self.runs.get(mid)
            fresh = cmd in ("new", "replay")
            if run is None or (run.kind == "live" and fresh) or cmd == "replay":
                if run is not None:
                    run.stop()
                world = self.world(m)
                make = None
                if cmd == "replay":
                    if not m.decisions:
                        raise ValueError("this mission has no saved run to replay (save it after simulating)")

                    def make():
                        from .sim import MissionSim

                        sim = MissionSim(m, world, False)
                        sim._replay = [dict(d) for d in m.decisions]
                        return sim

                run = SimRunner(m, world, self._publish, auto_approve=bool(auto_approve), make=make)
                self.runs[mid] = run
        if auto_approve is not None and cmd != "auto_approve":
            run.control("auto_approve", auto_approve)
        if cmd == "new":
            return run.control("reset")
        if cmd == "replay":
            out = run.control("play")
            st = self.map_status(m)
            if st and st["stale"] and isinstance(out, dict):  # the run was approved on another map
                out = {**out, "map": st, "warning": f"replaying on a different map: {st['message']}"}
            return out
        return run.control(cmd, value)

    def save(self, mid: str, name: str | None = None, chat: dict | None = None, keep_run: bool = True) -> dict:
        """Keep a mission: name it, attach its chat and (``keep_run``) the operator
        decisions of its current simulated run. Same id and revision (the plan
        does not change), so its run keeps going."""
        with self._lock:
            m = self.get(mid)
            if name is not None and name.strip():
                m.name = self._unique_name(name.strip(), m.id)
            m.saved = True
            if chat is not None:
                m.chat = chat
            run = self.runs.get(mid)
            if keep_run and run is not None and run.kind == "sim" and run.sim.decisions:
                m.decisions = [{k: v for k, v in d.items() if not k.startswith("_")} for d in run.sim.decisions]
            self._save(m)
        return {"mission": m.model_dump(), "decisions": len(m.decisions)}

    def state(self, mid: str) -> dict | None:
        run = self.runs.get(mid)
        return run.state() if run is not None else None

    # ------------------------------------------------------------- images
    def mosaic(self, src):
        from ..quad.camera import TileMosaic

        if src in self._mosaics:
            return self._mosaics[src]
        a = self.geo.anchor(src) if self.geo is not None else None
        layers = self.geo.tiles.layers() if a is not None else []
        mos = None
        if a is not None and layers:
            ids = [L["id"] for L in layers]
            layer = "esri_world_imagery" if "esri_world_imagery" in ids else ids[0]
            zmax = next((L["zmax"] for L in layers if L["id"] == layer), None) or 19
            mos = TileMosaic(self.geo.tiles, layer, a, int(zmax))
        self._mosaics[src] = mos
        return mos

    def image(self, mid: str, img_id: str) -> bytes:
        run = self.runs.get(mid)
        if run is None or img_id not in run.sim.captures:
            raise KeyError(img_id)
        with run.lock:
            return run.sim.render(img_id, self.mosaic(run.mission.src))

    # ----------------------------------------------------------- candidates
    def decide(self, mid: str, cid: str, approve: bool, commit: bool = False) -> dict:
        """Approve/reject a detection.  ``commit`` writes it into the mission's
        Neo4j map (DB-first) with its image; otherwise it lives in the run only."""
        run = self.runs.get(mid)
        if run is None:
            raise KeyError(mid)
        c = run.sim.candidates.get(cid)
        if c is None:
            raise KeyError(cid)
        sym = None
        out = {}
        if c["status"] == "rejected" and approve:
            raise ValueError(f"detection {cid} was rejected")
        if commit and c["status"] == "approved" and c.get("sym") and not str(c["sym"]).startswith("sim:"):
            return c  # already in the map
        if approve and commit:
            from .objects import add_object

            src = self._neo4j_for(run.mission.src)
            if src is None:
                raise ValueError("no Neo4j map to add the object to (simulation-only approve)")
            path = self._store_image(mid, c["image"], self.image(mid, c["image"]))
            out = add_object(src, c["label"], c["pos"], image_path=str(path) if path else None,
                             bbox_2d=c.get("bbox"), source="quad")  # fmt: skip
            sym = out["sym"]
            self.graph.notify_version(src.id, "quad detection")
        cand = run.decide(cid, approve, sym)
        return {**cand, **({"map_version": out.get("map_version")} if out else {})}

    def annotate(self, mid: str, img_id: str, bbox, label: str, query: str | None = None) -> dict:
        run = self.runs.get(mid)
        if run is None:
            raise KeyError(mid)
        return run.annotate(img_id, bbox, label, query)

    def _neo4j_for(self, src_id):
        if self.graph is None:
            return None
        ids = [src_id] + [s["id"] for s in self.graph.list() if s["kind"] == "neo4j"]
        for sid in ids:
            if sid and sid.startswith("neo4j:"):
                try:
                    return self.graph.get(sid)
                except Exception:  # noqa: BLE001
                    continue
        return None

    def _store_image(self, mid, img_id, jpeg):
        if self.images_dir is None:
            return None
        self.images_dir.mkdir(parents=True, exist_ok=True)
        p = self.images_dir / f"{mid}_{img_id}_{int(time.time())}.jpg"
        p.write_bytes(jpeg)
        return p.resolve()

    # ------------------------------------------------------------ live runs
    def execute(self, mid: str) -> dict:
        """Start the mission for real: bound agents drive their robots / the quad."""
        from .live import LiveIO, LiveRun

        m = self.get(mid)
        pv = self.preview(m)
        errors = [i for i in pv["issues"] if i["level"] == "error"]
        if errors:
            raise ValueError("fix the mission first: " + "; ".join(i["msg"] for i in errors))
        bound = [a for a in m.agents if a.binding]
        if not bound:
            raise ValueError("no agent is bound to a real robot; simulate it instead")
        if any(a.kind == "quad" for a in bound) and (self.quad is None or not self.quad.configured):
            raise ValueError("a quad is bound but the quad link is not configured (--quad-cmd-uri)")
        io = LiveIO(
            send_goto=self._send_goto,
            robot_pose=self.robot_pose,
            robot_results=lambda r: (self.ros.plan(r) or {}).get("results") if self.ros is not None else [],
            quad=self.quad,
        )
        world = self.world(m)
        with self._lock:
            old = self.runs.pop(mid, None)
            if old is not None:
                old.stop()
            run = SimRunner(m, world, self._publish, make=lambda: LiveRun(m, world, io), kind="live")
            self.runs[mid] = run
        return run.control("play")

    def _send_goto(self, robot, symbols):
        if self.ros is None:
            raise RuntimeError("ROS is off (--no-ros)")
        from ..routes.plan import omniplanner_points

        plugin = self.ros.find_plugin("goto", None)
        if plugin is None:
            raise RuntimeError("omniplanner has no goto plugin")
        return self.ros.send_goal(robot, plugin, {"robot_id": robot, "point_names_to_visit": omniplanner_points(symbols)})

    def abort(self, mid: str) -> dict:
        run = self.runs.get(mid)
        if run is None:
            raise KeyError(mid)
        run.control("pause")
        if run.kind == "live" and self.ros is not None:
            for a in run.mission.agents:
                if a.binding and a.kind != "quad":
                    try:
                        self.ros.executor_cmd(a.binding, "pause")
                    except Exception as e:  # noqa: BLE001
                        log.warning("pausing %s: %s", a.binding, e)
        return run.state()

    def stop(self):
        for run in list(self.runs.values()):
            run.stop()

    def export(self, mid: str) -> str:
        return json.dumps(self.get(mid).model_dump(), indent=1)
