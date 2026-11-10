"""Plan repair's scene graph edits (heracles_agents "SGET") from the console chat.

The heracles_agents repair chat edits the map with ten tools (add / move / remove
an object, add / remove a place, connect / disconnect two places, undo the last
edit, undo every chat edit, reset to the prior map). Their edits go into plan
repair v5's change log (heracles_ros.change_log, source ``sget/chat``), bump the
map version so the heracles publisher republishes at once, and are announced on
``/scene_graph/changes_applied`` so the planner judges them against the running
plan; undo / clear / reset are services of the scene change writer, which
rebuilds the map from the prior scene graph.

This module runs those same functions (``heracles_agents.scene_graph_edit``) for
the console. The chat model only *proposes* an SGET call; the operator approves
the card and only then does it run, against the console's Neo4j source. The
heracles_agents edit gate (edits only in an "SGET:" turn) is opened for exactly
that approved call: the operator's approval is the SGET request.

Used only while the scene change writer runs (plan repair on the base); without
it the chat keeps the console's own ``propose_map_edit`` path (graph_edit).
"""

from __future__ import annotations

import logging
import threading
import time

log = logging.getLogger(__name__)

WRITER_NAME = "scene_change_writer"
CHANGES_APPLIED_TOPIC = "/scene_graph/changes_applied"
SCENE_CHANGE_TYPE = "heracles_ros_interfaces/msg/SceneChangeMsg"
WRITER_TIMEOUT_S = 15.0
PROPOSAL_TTL_S = 600.0

# The SGET tools, as heracles_agents' chatdsg_repair registers them: the model sees
# these names and parameters; ``fn`` is the scene_graph_edit function that runs.
_STR, _NUM = {"type": "string"}, {"type": "number"}
TOOLS = {
    "add_scene_object": {
        "fn": "add_scene_object",
        "description": "Add an object to the scene graph at a map position. The object gets the "
        "next free id and is contained by its nearest MeshPlace.",
        "properties": {"object_class": {**_STR, "description": "Class of the object, e.g. 'chair'"},
                       "x": _NUM, "y": _NUM, "z": {**_NUM, "description": "z (m), default 0"}},
        "required": ["object_class", "x", "y"],
    },
    "move_scene_object": {
        "fn": "move_scene_object",
        "description": "Move an object in the scene graph to a map position.",
        "properties": {"symbol": {**_STR, "description": "The object's id, e.g. 'O15'"},
                       "x": _NUM, "y": _NUM, "z": {**_NUM, "description": "new z; default keeps its height"}},
        "required": ["symbol", "x", "y"],
    },
    "remove_scene_object": {
        "fn": "remove_scene_object",
        "description": "Remove an object from the scene graph.",
        "properties": {"symbol": {**_STR, "description": "The object's id, e.g. 'O15'"}},
        "required": ["symbol"],
    },
    "add_scene_place": {
        "fn": "add_scene_place",
        "description": "Add a traversable place (MeshPlace) at a map position, connected both ways "
        "to existing places.",
        "properties": {"x": _NUM, "y": _NUM,
                       "connect_to": {**_STR, "description": "Comma-separated places to connect it to, e.g. 't1104, t1405'"},
                       "symbol": {**_STR, "description": "Id for the new place, e.g. 't999'; empty for the next free one"}},
        "required": ["x", "y", "connect_to"],
    },
    "remove_scene_place": {
        "fn": "remove_scene_place",
        "description": "Delete a place (MeshPlace) with all its connections, e.g. where the robot can "
        "no longer go.",
        "properties": {"symbol": {**_STR, "description": "The place's id, e.g. 't444'"}},
        "required": ["symbol"],
    },
    "connect_scene_places": {
        "fn": "connect_scene_places",
        "description": "Connect two existing places, both ways.",
        "properties": {"place_a": {**_STR, "description": "A place id, e.g. 't444'"}, "place_b": _STR},
        "required": ["place_a", "place_b"],
    },
    "disconnect_scene_places": {
        "fn": "disconnect_scene_places",
        "description": "Remove the connection between two places, both ways, e.g. a blocked passage.",
        "properties": {"place_a": {**_STR, "description": "A place id, e.g. 't444'"}, "place_b": _STR},
        "required": ["place_a", "place_b"],
    },
    "undo_scene_edit": {
        "fn": "undo_scene_edit",
        "description": "Undo the latest scene graph edit made from the chat.",
        "properties": {}, "required": [],
    },
    "clear_scene_edits": {
        "fn": "clear_scene_edits",
        "description": "Undo every scene graph edit made from the chat; changes found by change "
        "detection stay.",
        "properties": {}, "required": [],
    },
    "reset_scene_graph": {
        "fn": "reset_scene_graph",
        "description": "Return the scene graph to the original prior map: every edit and every "
        "change detection result is dropped.",
        "properties": {}, "required": [],
    },
}  # fmt: skip
WRITER_TOOLS = ("undo_scene_edit", "clear_scene_edits", "reset_scene_graph")
# the viewer's map controls: action -> scene change writer service (std_srvs/Trigger)
WRITER_ACTIONS = {"undo": "undo_last_edit", "clear": "clear_edits", "reset": "reset"}
RESET_CONFIRM = "reset"  # typed by the operator: a reset drops every change to the map
_NO_EDIT = ("Refused", "Not done", "Not available", "Failed")

SYSTEM_NOTE = (
    "Map edits: plan repair runs on the base, so edit the scene graph ONLY with the scene edit "
    "tools (add_scene_object, move_scene_object, remove_scene_object, add_scene_place, "
    "remove_scene_place, connect_scene_places, disconnect_scene_places, undo_scene_edit, "
    "clear_scene_edits, reset_scene_graph). Each call is a proposal: it does not run until the "
    "operator approves its card, and you are told the outcome later. Every edit goes into plan "
    "repair's change log (undoable) and the planner replans on the edited map. run_cypher only "
    "reads; never write the map with Cypher."
)


def tool_schemas() -> list[dict]:
    """The SGET tools in the chat's (Anthropic-style) tool format."""
    out = []
    for name, t in TOOLS.items():
        out.append({
            "name": name,
            "description": t["description"] + " This only PROPOSES the edit: the operator approves "
            "it before it runs, and you are told the outcome.",
            "input_schema": {"type": "object", "properties": {
                **t["properties"],
                "summary": {"type": "string", "description": "one line for the operator: what and why"},
            }, "required": t["required"], "additionalProperties": False},
        })  # fmt: skip
    return out


class _Secret:
    """The ``get_secret_value`` heracles_agents' _connect reads."""

    def __init__(self, v):
        self._v = v or ""

    def get_secret_value(self):
        return self._v


class _Conf:
    """A HeraclesDsgInterface-shaped config (uri, username, password) for a source."""

    dsg_interface_type = "heracles"

    def __init__(self, uri, user, password):
        self.uri = uri
        self.username = _Secret(user)
        self.password = _Secret(password)


def _sge():
    """heracles_agents.scene_graph_edit, or None (not installed)."""
    try:
        from heracles_agents import scene_graph_edit

        return scene_graph_edit
    except Exception as e:  # noqa: BLE001
        log.debug("heracles_agents.scene_graph_edit unavailable: %s", e)
        return None


class SgetService:
    """Proposals and approved runs of SGET tools against a console Neo4j source."""

    def __init__(self, graph, ros=None, settings=None, module=None):
        self.graph = graph
        self.ros = ros
        self.settings = settings
        self._module = module  # tests inject a fake scene_graph_edit
        self._run_lock = threading.Lock()  # scene_graph_edit.gate is module-global
        self._lock = threading.Lock()
        self._items: dict[str, dict] = {}
        self._n = 0
        self._writer_cache = (0.0, False)

    # ------------------------------------------------------------ availability
    def module(self):
        return self._module if self._module is not None else _sge()

    def writer(self) -> str:
        ns = getattr(self.ros, "planner_ns", "") or ""
        return f"/{ns}/{WRITER_NAME}" if ns else f"/{WRITER_NAME}"

    def writer_running(self) -> bool:
        """The scene change writer's services are on the ROS graph (cached 5 s)."""
        t, ok = self._writer_cache
        if time.time() - t < 5.0:
            return ok
        ok = False
        ros = self.ros
        if ros is not None and getattr(ros, "available", False):
            try:
                ok = bool(ros.has_service(f"{self.writer()}/undo_last_edit"))
            except Exception:  # noqa: BLE001
                ok = False
        self._writer_cache = (time.time(), ok)
        return ok

    def available(self) -> bool:
        """SGET replaces the console's own edit path: plan repair's writer runs and
        heracles_agents is importable."""
        return self.module() is not None and self.writer_running()

    # ------------------------------------------------------- viewer map controls
    def status(self) -> dict:
        """What the viewer's Undo / Clear / Reset controls need: is the writer up."""
        running = self.writer_running()
        reason = None
        if not running:
            ros = self.ros
            if ros is None or not getattr(ros, "available", False):
                reason = "ROS is not running on this console"
            else:
                reason = (f"plan repair's scene change writer ({self.writer()}) is not running "
                          "on the base station")  # fmt: skip
        return {"writer": self.writer(), "running": running, "reason": reason,
                "actions": sorted(WRITER_ACTIONS)}  # fmt: skip

    def writer_action(
        self, action: str, src_id: str | None = None, confirm: str | None = None
    ) -> dict:
        """Undo the last edit / clear all edits / reset the map to its prior, through
        the scene change writer (it rebuilds the map); then reload the console's map.
        {ok, action, service, message}."""
        service = WRITER_ACTIONS.get(action)
        if service is None:
            raise ValueError(
                f"unknown map action {action!r} (use {', '.join(sorted(WRITER_ACTIONS))})"
            )
        if action == "reset" and (confirm or "").strip().lower() != RESET_CONFIRM:
            raise ValueError(f'type "{RESET_CONFIRM}" to reset the map to its prior')
        name = f"{self.writer()}/{service}"
        if not self.writer_running():
            return {"ok": False, "action": action, "service": name,
                    "message": self.status()["reason"]}  # fmt: skip
        ok, message = self._call_writer(service)
        if ok:
            src = self._source(src_id)
            if src is not None:
                self._refresh(src)
        log.info("map %s via %s: ok=%s %s", action, name, ok, message)
        return {"ok": bool(ok), "action": action, "service": name, "message": message}

    # --------------------------------------------------------------- proposals
    def propose(self, tool: str, args: dict, src_id: str) -> tuple[dict, dict]:
        """(card for the chat, result for the model). Nothing runs."""
        if tool not in TOOLS:
            return {}, {"error": f"unknown scene edit tool {tool}"}
        spec = TOOLS[tool]
        call = {
            k: args[k]
            for k in spec["properties"]
            if k in args and args[k] not in (None, "")
        }
        missing = [k for k in spec["required"] if k not in call]
        if missing:
            return {}, {"error": f"{tool} needs {', '.join(missing)}"}
        summary = str(args.get("summary") or "")
        with self._lock:
            now = time.time()
            self._items = {
                k: v for k, v in self._items.items() if now - v["t"] < PROPOSAL_TTL_S
            }
            self._n += 1
            pid = f"s{self._n}-{int(now)}"
            self._items[pid] = {
                "tool": tool,
                "args": call,
                "src": src_id,
                "summary": summary,
                "t": now,
            }
        card = {"type": "scene_edit", "id": pid, "tool": tool, "args": call, "summary": summary,
                "src": src_id, "text": describe(tool, call)}  # fmt: skip
        return card, {"pending_approval": True, "id": pid,
                      "note": "Not run yet. The operator approves or rejects it; you will be told."}  # fmt: skip

    def decide(self, pid: str, approve: bool) -> dict:
        """Run (or drop) a proposal by id: exactly the call on its card."""
        with self._lock:
            p = self._items.pop(pid, None)
        if p is None or time.time() - p["t"] >= PROPOSAL_TTL_S:
            return {"ok": False, "error": "unknown or expired scene edit proposal"}
        if not approve:
            return {
                "ok": True,
                "rejected": True,
                "tool": p["tool"],
                "text": describe(p["tool"], p["args"]),
            }
        return self.run(p["tool"], p["args"], p["src"])

    def undo(self, src_id: str | None) -> dict:
        """The card's Undo button: undo the latest chat edit (the operator's click)."""
        return self.run("undo_scene_edit", {}, src_id)

    # --------------------------------------------------------------------- run
    def run(self, tool: str, args: dict, src_id: str | None) -> dict:
        """Run one SGET tool now with the edit gate open; {ok, result, ...}."""
        mod = self.module()
        if mod is None:
            return {
                "ok": False,
                "tool": tool,
                "error": "heracles_agents is not installed",
            }
        src = self._source(src_id)
        if src is None:
            return {"ok": False, "tool": tool, "error": "no Neo4j map is connected"}
        fn = getattr(mod, TOOLS[tool]["fn"])
        kwargs = dict(args)
        if tool not in WRITER_TOOLS:
            kwargs["dsgdb_conf"] = _Conf(src.uri, *src.creds)
        gate = mod.gate
        with self._run_lock:
            saved = (gate.allowed, gate.report, gate.call_writer)
            gate.allowed = True
            gate.report = self._report
            gate.call_writer = self._call_writer
            try:
                result = str(fn(**kwargs))
            except Exception as e:  # noqa: BLE001
                log.exception("scene edit %s failed", tool)
                return {"ok": False, "tool": tool, "error": str(e)}
            finally:
                gate.allowed, gate.report, gate.call_writer = saved
        ok = not result.startswith(_NO_EDIT)
        if ok:
            self._refresh(src)
        version = _map_version(result)
        return {"ok": ok, "tool": tool, "result": result, "map_version": version,
                "undoable": ok and tool not in WRITER_TOOLS}  # fmt: skip

    # ------------------------------------------------------------------ hooks
    def _source(self, src_id):
        sid = src_id if src_id and str(src_id).startswith("neo4j:") else None
        if sid is None and self.graph is not None:
            for s in self.graph.list():
                if s.get("kind") == "neo4j":
                    sid = s["id"]
                    break
        if sid is None:
            return None
        try:
            src = self.graph.get(sid)
        except Exception:  # noqa: BLE001
            return None
        return src if hasattr(src, "creds") else None

    def _refresh(self, src) -> None:
        """Reload the edited map at once (the map-version poll is the fallback)."""
        try:
            src.invalidate()
            self.graph.notify_version(src.id, "scene edit")
        except Exception as e:  # noqa: BLE001
            log.warning("scene edit applied; refresh failed: %s", e)

    def _call_writer(self, service: str):
        """gate.call_writer: a std_srvs/Trigger of the scene change writer."""
        ros = self.ros
        if ros is None or not getattr(ros, "available", False):
            return (
                False,
                "ROS is not running, so the scene change writer cannot be reached",
            )
        name = f"{self.writer()}/{service}"
        try:
            res = ros.call_service(
                name, "std_srvs/srv/Trigger", {}, timeout=WRITER_TIMEOUT_S
            )
        except TimeoutError as e:
            return False, str(e)
        except Exception as e:  # noqa: BLE001
            return False, f"{name}: {e}"
        return bool(res.get("success")), str(res.get("message") or "")

    def _report(self, changes, map_version) -> None:
        """gate.report: announce the edit's object changes as the writer would, so
        the planner judges them against the running plan."""
        ros = self.ros
        if ros is None or not getattr(ros, "available", False):
            return
        now = time.time()
        fields = {
            "header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": "map"},
            "source": "sget/chat",
            "map_version": int(map_version),
            "changes": [{
                "kind": int(c["kind"]), "symbol": str(c["symbol"]),
                "semantic_label": str(c.get("label") or ""),
                "old_position": _point(c.get("old")), "new_position": _point(c.get("new")),
                "confidence": 1.0,
            } for c in changes],
        }  # fmt: skip
        try:
            ros.publish(CHANGES_APPLIED_TOPIC, SCENE_CHANGE_TYPE, fields)
        except Exception as e:  # noqa: BLE001
            log.warning("scene edit applied; announcing it failed: %s", e)


def _point(xyz) -> dict:
    if xyz is None:
        return {"x": 0.0, "y": 0.0, "z": 0.0}
    return {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])}


def _map_version(result: str):
    import re

    m = re.search(r"map_version (\d+)", result or "")
    return int(m.group(1)) if m else None


def describe(tool: str, args: dict) -> str:
    """One line for the card: what the call will do."""
    a = args
    return {
        "add_scene_object": lambda: f"Add a {a.get('object_class')} at ({a.get('x')}, {a.get('y')})",
        "move_scene_object": lambda: f"Move {a.get('symbol')} to ({a.get('x')}, {a.get('y')})",
        "remove_scene_object": lambda: f"Remove object {a.get('symbol')}",
        "add_scene_place": lambda: f"Add a place at ({a.get('x')}, {a.get('y')}) connected to {a.get('connect_to')}",
        "remove_scene_place": lambda: f"Delete place {a.get('symbol')} and its connections",
        "connect_scene_places": lambda: f"Connect places {a.get('place_a')} and {a.get('place_b')}",
        "disconnect_scene_places": lambda: f"Disconnect places {a.get('place_a')} and {a.get('place_b')}",
        "undo_scene_edit": lambda: "Undo the latest chat map edit",
        "clear_scene_edits": lambda: "Undo every chat map edit",
        "reset_scene_graph": lambda: "Reset the map to the prior scene graph (drops every change)",
    }.get(tool, lambda: tool)()  # fmt: skip
