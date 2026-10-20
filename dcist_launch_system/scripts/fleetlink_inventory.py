#!/usr/bin/env python3
"""Inventory of every ROS interface that crosses the robot <-> base-station boundary.

Read-only static analysis (no ROS needed) used to seed ``fleetlink/config/relay.yaml``.

How it works
------------
1. ``config_generation/experiment_manifest.yaml`` + ``launch_components/*.yaml`` are expanded
   into sessions. A session whose experiment name starts with ``base_station`` is a BASE
   session, everything else is a ROBOT session. The ``launch_<x>:=true`` flags (and
   ``ros2 run`` panes) of each side say which nodes of ``launch/master.launch.yaml`` run there.
2. Every base-side node's Python source (found through the package's ``setup.py`` entry
   points) is parsed with ``ast`` for create_publisher / create_subscription / create_client /
   create_service / ActionClient / lookup_transform (plus the adt4_console ros_bridge
   ``publish`` / ``call_service`` helpers). Launch remaps are applied, names are normalised
   (first path segment that is a robot -> ``{robot}``) and anything that stays inside the base
   namespace is dropped. What remains crosses the boundary.
3. The same is done for robot-side nodes to find the counterpart by resolved name.
4. ``rviz/dcist.rviz`` display topics are added (RViz runs on the base).

Heuristics, so rows that needed a guess carry ``uncertain: true`` and a note.

Direction: ``down`` = base -> robot (topic published by the base, service/action called by
the base); ``up`` = robot -> base. For services the direction is that of the request.

Usage:  fleetlink_inventory.py [--format markdown|yaml] [--live]
"""

from __future__ import annotations

import argparse
import ast
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
DLS = REPO / "dcist_launch_system"
GEN = DLS / "config_generation"
MASTER = DLS / "launch" / "master.launch.yaml"
RVIZ = DLS / "rviz" / "dcist.rviz"
PLUGINS = GEN / "base_params" / "omniplanner_plugins.yaml"

ROBOT_PLACEHOLDERS = {
    "self.name", "robot", "robot_name", "self.robot_name", "name", "robot_id", "r",
}
LOCAL_PLACEHOLDERS = {"planner_ns", "self.planner_ns", "self.planner_ns "}
SKIP_DIRS = {"build", "install", "log", "node_modules", "tests", "test", "__pycache__"}

# `{cmd}` in the console's executor_cmd(); routes/plan.py:124 restricts it to these two
PLACEHOLDER_EXPANSIONS = {"{cmd}": ["pause", "resume"], "{ex}": ["spot_executor_node"]}

# Panes that are not `ros2 run`/launch nodes but still talk ROS (name -> source files)
EXTRA_PANE_SOURCES = {
    "adt4-console": [
        "adt4_console/adt4_console/ros_bridge.py",
        "adt4_console/adt4_console/routes/launch.py",
        "adt4_console/adt4_console/routes/plan.py",
    ],
}
# Sources that are documented-but-not-launched by any component (remaps from docstring)
DOC_ONLY_NODES = {
    "goal_manager_node": "omniplanner/omniplanner_ros/src/omniplanner_ros/goal_manager_node.py",
}
SIBLING_PKGS = {"omniplanner_ros"}  # planner plugins are loaded dynamically: scan all modules

KINDS = {
    # call -> (kind, role, type-arg index, name-arg index)
    "create_publisher": ("topic", "pub", 0, 1),
    "create_subscription": ("topic", "sub", 0, 1),
    "create_client": ("service", "client", 0, 1),
    "create_service": ("service", "server", 0, 1),
    "ActionClient": ("action", "client", 1, 2),
    "ActionServer": ("action", "server", 1, 2),
}
TYPE_FOR_RVIZ = {
    "MarkerArray": "visualization_msgs/msg/MarkerArray",
    "Marker": "visualization_msgs/msg/Marker",
    "Path": "nav_msgs/msg/Path",
    "Map": "nav_msgs/msg/OccupancyGrid",
    "OccupancyGrid": "nav_msgs/msg/OccupancyGrid",
    "Image": "sensor_msgs/msg/Image",
    "PointCloud2": "sensor_msgs/msg/PointCloud2",
    "Pose": "geometry_msgs/msg/PoseStamped",
}


def rel(p: Path) -> str:
    try:
        return str(p.relative_to(REPO))
    except ValueError:
        return str(p)


# --------------------------------------------------------------------------- sessions


def load_yaml(p: Path):
    return yaml.safe_load(p.read_text()) or {}


def expand_launch_config(name, launch_configs, seen=()):
    if name in seen:
        return []
    if name not in launch_configs:
        return [name]  # a component
    out = []
    members = launch_configs[name]
    members = [members] if isinstance(members, str) else members
    for m in members:
        out += expand_launch_config(m, launch_configs, seen + (name,))
    return out


def load_sessions():
    manifest = load_yaml(GEN / "experiment_manifest.yaml")
    lcs = manifest.get("launch_configs", {})
    sessions = {}
    for exp, spec in manifest.get("experiments", {}).items():
        lc = spec.get("launch_config", [])
        lc = [lc] if isinstance(lc, str) else lc
        comps = []
        for x in lc:
            for c in expand_launch_config(x, lcs):
                if c not in comps:
                    comps.append(c)
        side = "base" if exp.startswith("base_station") else "robot"
        flags, panes = set(), []
        for c in comps:
            f = GEN / "launch_components" / f"{c}.yaml"
            if not f.exists():
                continue
            for w in load_yaml(f).get("windows", []):
                for p in w.get("panes", []):
                    if isinstance(p, str):
                        panes.append(p)
                        flags |= set(re.findall(r"(launch_\w+):=true", p))
        sessions[exp] = {"side": side, "components": comps, "flags": flags, "panes": panes}
    return sessions


# --------------------------------------------------------------------------- launch nodes


class LNode:
    def __init__(self, pkg, exe, name, flag, remaps, ns="{robot}", origin="master.launch.yaml"):
        self.pkg, self.exe, self.name, self.flag = pkg, exe, name, flag
        self.remaps = remaps  # list[(from, to)]
        self.ns = ns
        self.origin = origin


def _sub(s):
    return str(s).replace("$(var robot_name)", "{robot}").replace("$(var camera)", "{robot}_zed")


def master_nodes():
    doc = load_yaml(MASTER)
    nodes = []

    def walk(items, flag=None):
        for it in items or []:
            if not isinstance(it, dict):
                continue
            for key in ("node", "pyenv_node"):
                if key in it:
                    d = it[key]
                    f = flag
                    m = re.search(r"\$\(var (launch_\w+)\)", str(d.get("if", "")))
                    if m:
                        f = m.group(1)
                    remaps = [(r["from"], _sub(r["to"])) for r in d.get("remap", []) if "from" in r]
                    nodes.append(LNode(d.get("pkg"), d.get("exec"), d.get("name") or d.get("exec"), f, remaps))
            if "group" in it or "include" in it:
                g = it.get("group")
                if isinstance(g, list):
                    walk(g, flag)
                elif isinstance(g, dict):
                    m = re.search(r"\$\(var (launch_\w+)\)", str(g.get("if", "")))
                    walk(g.get("children", []), m.group(1) if m else flag)
                if "include" in it:
                    pass  # roman includes are robot-only (not scanned)

    walk(doc.get("launch", []))
    return nodes


def pane_nodes(panes):
    """`ros2 run <pkg> <exe> ... --ros-args -r a:=b` panes (ros_system_monitor)."""
    out = []
    for p in panes:
        m = re.search(r"ros2 run (\S+) (\S+)(.*)", p)
        if not m:
            m2 = re.match(r"\S*?/?(adt4-console)\b", p.strip().split()[0].split("/")[-1])
            if m2:
                out.append(LNode("adt4_console", "adt4-console", "adt4_console", None, [], ns="", origin="console pane"))
            continue
        pkg, exe, rest = m.groups()
        remaps, ns = [], "{robot}"
        for a, b in re.findall(r"-r\s+'?([^\s':]+):=([^\s']+)'?", rest):
            b = b.replace("$ADT4_ROBOT_NAME", "{robot}")
            if a == "__ns":
                ns = b
            else:
                remaps.append((a, b))
        out.append(LNode(pkg, exe, exe, None, remaps, ns=ns, origin="ros2 run pane"))
    return out


# --------------------------------------------------------------------------- source index


def iter_files(pattern):
    for p in REPO.glob(pattern):
        if not set(p.relative_to(REPO).parts) & SKIP_DIRS:
            yield p


_ENTRY = None


def entry_points():
    global _ENTRY
    if _ENTRY is None:
        _ENTRY = {}
        for setup in iter_files("**/setup.py"):
            txt = setup.read_text()
            for exe, mod in re.findall(r'"([\w\-]+)\s*=\s*([\w\.]+):\w+"', txt):
                parts = mod.split(".")
                for cand in [setup.parent / "src" / Path(*parts), setup.parent / Path(*parts)]:
                    if cand.with_suffix(".py").exists():
                        _ENTRY[exe] = cand.with_suffix(".py")
                        break
    return _ENTRY


def local_imports(path: Path, seen):
    """Transitively follow same-package imports (spot_tools_ros.x, .x)."""
    if path in seen or not path.exists():
        return
    seen.append(path)
    try:
        tree = ast.parse(path.read_text())
    except SyntaxError:
        return
    pkg_dir = path.parent
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom):
            mods = []
            if n.level:
                mods = [n.module.split(".")[-1]] if n.module else [a.name for a in n.names]
            elif n.module and n.module.split(".")[0] == pkg_dir.name:
                mods = [n.module.split(".")[-1]]
                if len(n.module.split(".")) == 1:
                    mods = [a.name for a in n.names]
            for m in mods:
                local_imports(pkg_dir / f"{m}.py", seen)


def node_sources(node: LNode):
    if node.exe in EXTRA_PANE_SOURCES:
        return [REPO / s for s in EXTRA_PANE_SOURCES[node.exe]]
    main = entry_points().get(node.exe)
    if main is None:
        return []
    if main.parent.name in SIBLING_PKGS:
        return sorted(p for p in main.parent.glob("*.py") if p.name != "__init__.py" and p.stem not in DOC_ONLY_NODES)
    seen = []
    local_imports(main, seen)
    return seen


# --------------------------------------------------------------------------- ast scan


class Iface:
    def __init__(self, kind, role, raw, typ, file, line, qos="", uncertain=False, note=""):
        self.kind, self.role, self.raw, self.typ = kind, role, raw, typ
        self.file, self.line, self.qos = file, line, qos
        self.uncertain, self.note = uncertain, note

    @property
    def loc(self):
        return f"{rel(self.file)}:{self.line}"


def _imports(tree):
    m = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom) and n.module:
            for a in n.names:
                mod = n.module.split(".")
                if len(mod) >= 2 and mod[-1] in ("msg", "srv", "action"):
                    m[a.asname or a.name] = "/".join(mod[-2:] + [a.name]) if False else f"{mod[-2]}/{mod[-1]}/{a.name}"
    return m


def _render(expr, scope, depth=0):
    """Candidate strings for an expression (f-strings keep {placeholders})."""
    if expr is None or depth > 3:
        return []
    if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
        return [expr.value]
    if isinstance(expr, ast.JoinedStr):
        out = ""
        for v in expr.values:
            if isinstance(v, ast.Constant):
                out += str(v.value)
            else:
                out += "{" + ast.unparse(v.value) + "}"
        return [out]
    if isinstance(expr, ast.Name) and expr.id in scope:
        return _render(scope[expr.id], scope, depth + 1)
    if isinstance(expr, ast.BoolOp):
        c = []
        for v in expr.values:
            c += _render(v, scope, depth + 1)
        return c
    if isinstance(expr, ast.IfExp):
        return _render(expr.body, scope, depth + 1)
    if isinstance(expr, ast.BinOp) and isinstance(expr.op, ast.Add):
        a, b = _render(expr.left, scope, depth + 1), _render(expr.right, scope, depth + 1)
        return [a[0] + b[0]] if a and b else []
    return []


def _config_key(expr, scope, depth=0):
    """name of the config attribute for `getattr(config, "KEY", ...) or default`."""
    if expr is None or depth > 3:
        return None
    if isinstance(expr, ast.Name) and expr.id in scope:
        return _config_key(scope[expr.id], scope, depth + 1)
    for n in ast.walk(expr):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "getattr"
                and len(n.args) >= 2 and isinstance(n.args[1], ast.Constant)):
            return n.args[1].value
    return None


def _qos(expr, scope):
    if expr is None:
        return ""
    if isinstance(expr, ast.Name) and expr.id in scope:
        expr = scope[expr.id]
    txt = ast.unparse(expr)
    notes = []
    if "TRANSIENT_LOCAL" in txt or "latching" in txt or "latched" in txt:
        notes.append("latched")
    if "KEEP_ALL" in txt:
        notes.append("keep_all")
    if "BEST_EFFORT" in txt:
        notes.append("best_effort")
    elif "RELIABLE" in txt:
        notes.append("reliable")
    if isinstance(expr, ast.Constant) and isinstance(expr.value, int):
        notes.append(f"depth {expr.value}")
    m = re.search(r"depth\s*=\s*(\d+)", txt)
    if m:
        notes.append(f"depth {m.group(1)}")
    return ", ".join(notes)


def scan_file(path: Path):
    try:
        tree = ast.parse(path.read_text())
    except (SyntaxError, OSError):
        return []
    imps = _imports(tree)
    out = []
    funcs = [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]

    def scope_for(call):
        chain = [f for f in funcs if f.lineno <= call.lineno <= (f.end_lineno or f.lineno)]
        chain.sort(key=lambda f: -f.lineno)  # innermost first
        scope = {}
        for f in chain or [tree]:
            for n in ast.walk(f):
                if isinstance(n, ast.Assign) and len(n.targets) == 1:
                    t = n.targets[0]
                    key = t.id if isinstance(t, ast.Name) else None
                    if key and key not in scope:
                        scope[key] = n.value
        return scope

    for call in [n for n in ast.walk(tree) if isinstance(n, ast.Call)]:
        fn = call.func
        fname = fn.attr if isinstance(fn, ast.Attribute) else fn.id if isinstance(fn, ast.Name) else ""
        scope = scope_for(call)
        if fname in KINDS:
            kind, role, ti, ni = KINDS[fname]
            if len(call.args) <= ni:
                continue
            names = _render(call.args[ni], scope)
            if not names:
                continue
            typ = ""
            ta = call.args[ti]
            if isinstance(ta, ast.Name):
                typ = imps.get(ta.id, "")
            qa = call.args[ni + 1] if len(call.args) > ni + 1 else None
            for kw in call.keywords:
                if kw.arg in ("qos_profile", "qos"):
                    qa = kw.value
            qos = _qos(qa, scope)
            if fname == "create_publisher" and not qos and qa is None:
                qos = ""
            ckey = _config_key(call.args[ni], scope)
            iface = Iface(kind, role, names[-1], typ, path, call.lineno, qos)
            iface.config_key = ckey
            iface.alt = names
            out.append(iface)
        elif fname in ("lookup_transform", "can_transform") and fname == "lookup_transform":
            frames = [ast.unparse(a) for a in call.args[:2]]
            out.append(Iface("tf", "lookup", "->".join(frames), "tf2_msgs/msg/TFMessage", path, call.lineno))
        elif fname in ("TransformBroadcaster", "StaticTransformBroadcaster"):
            out.append(Iface("tf", "bcast_static" if "Static" in fname else "bcast", "tf", "tf2_msgs/msg/TFMessage", path, call.lineno))
        elif fname == "call_service" and len(call.args) >= 2:
            names = _render(call.args[0], scope)
            typ = _render(call.args[1], scope)
            if names:
                out.append(Iface("service", "client", names[-1], typ[0] if typ else "", path, call.lineno))
        elif fname == "publish" and len(call.args) >= 3 and isinstance(call.args[1], (ast.Constant, ast.Name)):
            names = _render(call.args[0], scope)
            typ = _render(call.args[1], scope)
            if names and typ and "/msg/" in typ[0]:
                out.append(Iface("topic", "pub", names[-1], typ[0], path, call.lineno))
    return out


# --------------------------------------------------------------------------- normalisation


def robot_names():
    return [r["robot_name"] for r in load_yaml(PLUGINS).get("robots", [])]


def plugin_overrides():
    """config key -> normalised pattern when every executor robot agrees."""
    ov = defaultdict(set)
    names = robot_names()
    for r in load_yaml(PLUGINS).get("robots", []):
        if r.get("type") != "robot_executor":
            continue
        for k in ("action_result_topic", "runtime_guards_topic"):
            if r.get(k):
                ov[k].add(r[k].replace(r["robot_name"], "{robot}", 1))
    return {k: next(iter(v)) for k, v in ov.items() if len(v) == 1}


def normalise(name: str, names):
    """Return (pattern, local?). local = stays inside a base/planner namespace."""
    n = name.replace("$(var robot_name)", "{robot}")
    m = re.match(r"^/\{([^}]*)\}/", n)
    if m:
        if m.group(1) in LOCAL_PLACEHOLDERS:
            return n, True
        if m.group(1) in ROBOT_PLACEHOLDERS:
            return "/{robot}/" + n[m.end():], False
        return n, True
    m = re.match(r"^/([^/]+)/", n)
    if m and m.group(1) in names:
        return "/{robot}/" + n[m.end():], False
    if n.startswith("/global/"):
        return n, False
    if n.startswith("/"):
        return n, True  # other global names (/omniplanner_node/..., /tf handled elsewhere)
    return n, True  # relative / private -> inside the node's namespace


def apply_remaps(raw, remaps):
    for a, b in remaps:
        if raw == a:
            return b
    return raw


def resolve_robot_side(raw, node: LNode):
    n = apply_remaps(raw, node.remaps)
    if n.startswith("/"):
        return n
    if n.startswith("~/"):
        return f"/{node.ns.strip('/')}/{node.name}/{n[2:]}" if node.ns else f"/{node.name}/{n[2:]}"
    base = node.ns.strip("/")
    return f"/{base}/{n}" if base else f"/{n}"


def expand(pattern):
    for k, vals in PLACEHOLDER_EXPANSIONS.items():
        if k in pattern:
            return [pattern.replace(k, v) for v in vals]
    return [pattern]


# --------------------------------------------------------------------------- build inventory


def find_line(text_path: Path, regex):
    try:
        for i, line in enumerate(text_path.read_text().splitlines(), 1):
            if re.search(regex, line):
                return i
    except OSError:
        pass
    return 0


def goal_manager_remaps():
    p = REPO / DOC_ONLY_NODES["goal_manager_node"]
    out = []
    for a, b in re.findall(r"\{from:\s*(~/\w+),\s*to:\s*(/<robot>/[^}\s]+)\}", p.read_text()):
        out.append((a, b.replace("<robot>", "{robot}")))
    return out


_DROPPED = []


def build():
    sessions = load_sessions()
    nodes = master_nodes()
    names = robot_names()
    overrides = plugin_overrides()

    base_flags = set().union(*[s["flags"] for s in sessions.values() if s["side"] == "base"])
    robot_flags = set().union(*[s["flags"] for s in sessions.values() if s["side"] == "robot"])
    base_panes = [p for s in sessions.values() if s["side"] == "base" for p in s["panes"]]
    robot_panes = [p for s in sessions.values() if s["side"] == "robot" for p in s["panes"]]

    base_nodes = [n for n in nodes if n.flag in base_flags] + pane_nodes(base_panes)
    robot_nodes = [n for n in nodes if n.flag in robot_flags] + pane_nodes(robot_panes)
    # the base-side nodes also include goal_manager (docs-only) handled below

    # ---- robot-side index: resolved name -> list[Iface+node]
    robot_idx = defaultdict(list)
    robot_tf = []
    seen_nodes = set()
    for n in robot_nodes:
        key = (n.exe, n.name, n.ns)
        if key in seen_nodes:
            continue
        seen_nodes.add(key)
        for f in node_sources(n):
            for i in scan_file(f):
                if i.kind == "tf":
                    if i.role in ("bcast", "bcast_static"):
                        robot_tf.append((n, i))
                    continue
                i.node = n
                robot_idx[(i.kind, resolve_robot_side(i.raw, n))].append(i)

    rows = {}  # (kind, pattern) -> row

    def add(kind, pattern, typ, direction, user, role="", role_note="", uncertain=False, note="", base_qos=""):
        key = (kind, pattern)
        r = rows.setdefault(key, {
            "name": pattern, "kind": kind, "type": typ, "dir": direction,
            "base_users": [], "robot_side": [], "notes": [], "uncertain": False,
            "_roles": set(), "_rviz": False,
        })
        r["_roles"].add(role)
        r["_rviz"] |= user.startswith("rviz2_node")
        if direction != r["dir"]:
            r["uncertain"] = True
            r["notes"].append(f"CONFLICT: also seen as {direction} at {user}")
        if typ and not r["type"]:
            r["type"] = typ
        if user not in r["base_users"]:
            r["base_users"].append(user)
        if base_qos:
            r["notes"].append(f"base qos: {base_qos}")
        if note and note not in r["notes"]:
            r["notes"].append(note)
        r["uncertain"] |= uncertain

    def dir_of(role):
        return {"pub": "down", "sub": "up", "client": "down", "server": "up"}[role]

    def opposite(role):
        return {"pub": "sub", "sub": "pub", "client": "server", "server": "client"}[role]

    # ---- base-side nodes
    scanned_base = set()
    for n in base_nodes:
        key = (n.exe, n.name, n.ns)
        if key in scanned_base:
            continue
        scanned_base.add(key)
        for f in node_sources(n):
            for i in scan_file(f):
                if i.kind == "tf":
                    if i.role == "lookup":
                        add("tf", "map->{robot}/base_link", i.typ, "up",
                            f"{n.name} ({i.loc})", role="sub", note="TF consumer; frames in source: " + i.raw,
                            uncertain="base_link" not in i.raw and "child_frame" not in i.raw)
                    continue
                ckey = getattr(i, "config_key", None)
                raw = i.raw
                unc, note = False, ""
                if ckey and ckey in overrides:
                    raw, note = overrides[ckey], f"name from omniplanner_plugins.yaml:{ckey}"
                resolved = apply_remaps(raw, n.remaps)
                if n.ns.strip("/") not in ("{robot}", "") and not resolved.startswith("/"):
                    resolved = resolve_robot_side(raw, n)  # pane nodes with an explicit namespace
                pat, local = normalise(resolved, names)
                if local and not (ckey in overrides):
                    continue
                for p in expand(pat):
                    add(i.kind, p, i.typ, dir_of(i.role), f"{n.name} ({i.loc})", role=i.role, note=note,
                        uncertain=unc, base_qos=i.qos)
                    for ri in robot_idx.get((i.kind, p.replace("{robot}", "{robot}")), []):
                        if ri.role == opposite(i.role):
                            rr = f"{ri.node.name} ({ri.loc})" + (f" [{ri.qos}]" if ri.qos else "")
                            if rr not in rows[(i.kind, p)]["robot_side"]:
                                rows[(i.kind, p)]["robot_side"].append(rr)

    # ---- goal_manager (documented remaps; not launched by any component)
    gm_path = REPO / DOC_ONLY_NODES["goal_manager_node"]
    gm_remaps = goal_manager_remaps()
    gm = LNode("omniplanner_ros", "goal_manager_node", "goal_manager_node", None, gm_remaps)
    for i in scan_file(gm_path):
        if i.kind in ("tf",):
            continue
        r = apply_remaps(i.raw, gm_remaps)
        pat, local = normalise(r, names)
        if local:
            continue
        add(i.kind, pat, i.typ, dir_of(i.role), f"goal_manager_node ({i.loc})", role=i.role, uncertain=True,
            note="goal_manager remap is only documented in its docstring (not launched by any component)")

    # ---- RViz displays
    cls, rviz_rows = None, []
    lines = RVIZ.read_text().splitlines()
    in_topic = None
    for ln, line in enumerate(lines, 1):
        m = re.match(r"\s*-?\s*Class:\s*\S*/(\w+)", line)
        if m:
            cls = m.group(1)
        m = re.match(r"\s+(Update )?Topic:\s*$", line)
        if m:
            in_topic = bool(m.group(1))
            continue
        m = re.match(r"\s+Value:\s*(\S+)\s*$", line)
        if m and in_topic is not None and m.group(1) not in ("true", "false"):
            topic = m.group(1)
            typ = TYPE_FOR_RVIZ.get(cls or "", "")
            if in_topic:
                typ = "map_msgs/msg/OccupancyGridUpdate"
            pat, local = normalise(topic, names)
            if not local:
                rviz_rows.append((pat, typ, ln))
            in_topic = None
    for pat, typ, ln in rviz_rows:
        add("topic", pat, typ, "up", f"rviz2_node ({rel(RVIZ)}:{ln})", role="sub", uncertain=False,
            note="RViz-only display (heavy; relay only if needed)")
        r = rows[("topic", pat)]
        for (k, rn), ris in robot_idx.items():
            if k == "topic" and rn == pat:
                for ri in ris:
                    if ri.role == "pub":
                        rr = f"{ri.node.name} ({ri.loc})" + (f" [{ri.qos}]" if ri.qos else "")
                        if rr not in r["robot_side"]:
                            r["robot_side"].append(rr)
    add("tf", "map->{robot}/base_link", "tf2_msgs/msg/TFMessage", "up",
        role="sub", user=f"rviz2_node ({rel(RVIZ)}:{find_line(RVIZ, 'rviz_default_plugins/TF')})",
        note="RViz TF display + Fixed Frame map; needs /tf and /tf_static")

    # ---- TF robot-side counterparts
    tf_cp = []
    for n, i in robot_tf:
        tf_cp.append(f"{n.name} ({i.loc}, {'static' if 'static' in i.role else 'dynamic'})")
    for n in nodes:
        if n.exe == "static_transform_publisher" and n.flag in robot_flags:
            tf_cp.append(f"{n.name or n.exe} (launch/master.launch.yaml:{find_line(MASTER, 'exec: static_transform_publisher')}, static)")
    tf_cp.append("hydra (C++, publishes map->{robot}/odom backend TF; not statically scanned)")
    if ("tf", "map->{robot}/base_link") in rows:
        rows[("tf", "map->{robot}/base_link")]["robot_side"] = sorted(set(tf_cp))
        rows[("tf", "map->{robot}/base_link")]["notes"].append(
            "robot composes map->{robot}/base_link (hydra map->odom + sensor odom->body->base_link)")

    # ---- direction from the robot-side role; drop base-internal flows
    dropped = []
    for key in list(rows):
        kind, pat = key
        r = rows[key]
        if kind == "tf":
            continue
        if r["_rviz"]:
            r["dir"] = "up"  # RViz only consumes
            r["uncertain"] = False
            pubs = [x for (k, rn), ris in robot_idx.items() if k == kind and rn == pat for x in ris if x.role == "pub"]
            if not pubs and pat.startswith("/{robot}/hydra/"):
                r["robot_side"].append("hydra (C++; not statically scanned)")
            elif not pubs:
                r["uncertain"] = True
            continue
        robot_roles = {x.role for (k, rn), ris in robot_idx.items() if k == kind and rn == pat for x in ris}
        if robot_roles:
            r["dir"] = {"pub": "up", "sub": "down", "server": "down", "client": "up"}[sorted(robot_roles)[0]]
            r["notes"] = [x for x in r["notes"] if not x.startswith("CONFLICT")]
            r["uncertain"] = False if len(robot_roles) == 1 else True
            continue
        roles = r["_roles"]
        if r["_rviz"]:
            if pat.startswith("/{robot}/hydra/"):
                r["robot_side"].append("hydra (C++; not statically scanned)")
            continue
        if roles <= {"sub", "server"}:
            dropped.append((pat, "no robot-side producer found (base-local subscription?)", r))
            del rows[key]
        elif {"pub", "sub"} <= roles:
            dropped.append((pat, "published and consumed on the base only", r))
            del rows[key]
        else:
            r["dir"] = "down"
            r["notes"] = [x for x in r["notes"] if not x.startswith("CONFLICT")]
    _DROPPED[:] = dropped

    # ---- fix-ups / uncertainty flags
    for (kind, pat), r in rows.items():
        if kind in ("topic", "service") and not r["robot_side"]:
            r["uncertain"] = True
            r["notes"].append("no robot-side counterpart found by static scan")
        if not r["type"]:
            r["uncertain"] = True
        if kind == "topic" and "base qos" not in " ".join(r["notes"]) and r["dir"] == "down":
            pass

    # known manual annotations
    ann = {
        ("topic", "/{robot}/omniplanner_node/compiled_plan_out"):
            "ActionSequenceMsg for spot; nav_msgs/Path for phoenix (PhoenixPlanningAdaptor, omniplanner_node.py:242)",
        ("topic", "/{robot}/omniplanner_node/llm_response"):
            "console subscribes per robot but omniplanner remaps ~/llm_response to rviz2_node/llm_response "
            "inside the planner namespace (master.launch.yaml:~228): base-local, likely dead per-robot subscription",
        ("service", "/{robot}/update_holding_state"):
            "base serves as /<ADT4_ROBOT_NAME>/update_holding_state via remap (master.launch.yaml:~458); "
            "relay must serve it per robot",
        ("topic", "/{robot}/spot_executor_node/pause"):
            "no subscriber found in spot_tools_ros at this checkout (grep pause/resume): robot-side consumer unverified",
        ("topic", "/{robot}/spot_executor_node/resume"):
            "no subscriber found in spot_tools_ros at this checkout: robot-side consumer unverified",
        ("service", "/{robot}/hydra/shutdown"):
            "hydra_ros_node ~/shutdown (ianvs spinAndWait, Empty): answers at once, hydra "
            "then saves and exits; console stop_save + core pane call it",
        ("service", "/{robot}/roman/shutdown"):
            "roman_map_node (Empty): pickles the ROMAN map before answering; console "
            "stop_save + core pane call it",
        ("service", "/{robot}/spot_executor_node/take_lease"):
            "spot_executor_ros ~/take_lease (Trigger): forcibly takes the Spot lease; console",
    }
    for k, text in ann.items():
        if k in rows:
            rows[k]["notes"].append(text)
            if "llm_response" in k[1]:
                rows[k]["uncertain"] = True
    for r in rows.values():
        r.pop("_roles", None)
        r.pop("_rviz", None)
    return sorted(rows.values(), key=lambda r: (r["kind"], r["name"])), sessions


# --------------------------------------------------------------------------- output


def to_markdown(rows, sessions):
    out = ["# fleetlink inventory: ROS interfaces crossing robot <-> base", ""]
    out.append("Direction: down = base->robot, up = robot->base (services: direction of the request). "
               "`?` = uncertain (static heuristic).\n")
    out.append("| interface | kind | type | dir | base-side user | robot-side counterpart | notes |")
    out.append("|---|---|---|---|---|---|---|")
    for r in rows:
        q = " ?" if r["uncertain"] else ""
        out.append("| {} | {} | {} | {}{} | {} | {} | {} |".format(
            r["name"], r["kind"], r["type"] or "?", r["dir"], q,
            "<br>".join(r["base_users"]), "<br>".join(r["robot_side"]) or "-",
            "; ".join(dict.fromkeys(r["notes"])) or "-"))
    out.append("")
    if _DROPPED:
        out.append("Dropped (not crossing the boundary, or no robot-side peer found):\n")
        for pat, why, r in _DROPPED:
            out.append(f"- `{pat}` ({r['kind']}): {why}; {'; '.join(r['base_users'])}")
        out.append("")
    base = [k for k, v in sessions.items() if v["side"] == "base"]
    out.append("Base sessions: " + ", ".join(base))
    return "\n".join(out)


def live():
    for cmd in (["ros2", "node", "list", "-a"],):
        print("$", " ".join(cmd))
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        print(res.stdout or res.stderr)
        for node in res.stdout.split():
            info = subprocess.run(["ros2", "node", "info", node], capture_output=True, text=True, timeout=30)
            print(f"--- {node}\n{info.stdout}")


def inventory():
    return build()[0]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--format", choices=["markdown", "yaml"], default="markdown")
    ap.add_argument("--live", action="store_true", help="also run `ros2 node list/info` (needs a running graph)")
    a = ap.parse_args(argv)
    rows, sessions = build()
    if a.format == "yaml":
        print(yaml.safe_dump(rows, sort_keys=False, width=200))
    else:
        print(to_markdown(rows, sessions))
    if a.live:
        live()
    return 0


if __name__ == "__main__":
    sys.exit(main())
