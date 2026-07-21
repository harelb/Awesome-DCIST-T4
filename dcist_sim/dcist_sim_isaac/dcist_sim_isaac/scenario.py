"""Pure-python scenario loader for dcist_sim_isaac.

No Isaac / ROS imports here on purpose: this module must be importable
and testable with plain `python3` + `pyyaml`, independent of the Isaac
Sim python environment. See dcist_sim_isaac/README.md for rationale.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field

import yaml

LOCOMOTIONS = {"kinematic", "policy"}
GRASPING_MODES = {"magic", "physics"}

# Task 9: optional top-level scenario key controlling magic-grasp
# selection radius (grasp.py's `select_grasp_target`). Kept as a literal
# here (not imported from grasp.py) so this module's "stdlib+pyyaml
# only, no Isaac/ROS" contract never depends on another module's import
# graph -- keep both defaults in sync if this value ever changes.
DEFAULT_GRASP_RADIUS = 1.5


@dataclass
class RobotSpec:
    name: str
    x: float
    y: float
    z: float
    yaw: float  # radians (not degrees) -- rotation about world +Z
    locomotion: str
    grasping: str
    contact_hold: bool = False


@dataclass
class ObjectSpec:
    object_id: str
    usd: str
    label: str
    x: float
    y: float
    z: float
    yaw: float  # radians (not degrees) -- rotation about world +Z
    graspable: bool = True


@dataclass
class TourWaypoint:
    x: float
    y: float
    yaw: float  # radians (not degrees) -- rotation about world +Z
    dwell_s: float = 0.0


GT_MODES = {"live", "replay"}
GT_MODALITIES = {"rgb", "semantic", "instance", "bbox2d", "bbox3d", "depth"}
DEFAULT_GT_MODALITIES = ["rgb", "semantic", "instance", "bbox2d"]


@dataclass
class GtSemanticRule:
    match: str            # regex, re.search()ed against stage prim paths
    semantic_class: str   # YAML key is `class` (python keyword)


@dataclass
class GtSpec:
    enabled: bool = False
    mode: str = "live"    # live | replay (see gt_capture.py)
    rate_hz: float = 2.0
    modalities: list = field(default_factory=lambda: list(DEFAULT_GT_MODALITIES))
    semantics: list = field(default_factory=list)


@dataclass
class NavSpec:
    """Local-planner parameters (spec §4); used only in physics mode."""
    cell_size_m: float = 0.1
    inflation_radius_m: float = 0.45   # Spot half-width ~0.25 + margin
    snap_bound_m: float = 2.0          # tour waypoint snap search bound
    # Task 15k: minimum free standoff (m) BEYOND the inflated obstacle boundary
    # that a snapped goto-poi/rearrange goal must clear, so the base does not
    # park on the inflation edge immediately adjacent to an object footprint
    # (the 15j onto-bag topple). 0.0 = pre-15k behavior (snap to nearest free
    # cell). Only the live LocalPlanner goal-snap uses it; tour-waypoint
    # snapping (build_map) is unaffected.
    snap_standoff_m: float = 0.0
    stuck_timeout_s: float = 15.0
    max_lin_speed: float = 1.0
    max_ang_speed: float = 1.0


@dataclass
class Scenario:
    environment_usd: str
    robots: list = field(default_factory=list)
    objects: list = field(default_factory=list)
    # Task 9: magic-grasp selection radius (meters), optional in the
    # YAML (top-level `grasp_radius:` key), defaulting to
    # DEFAULT_GRASP_RADIUS.
    grasp_radius: float = DEFAULT_GRASP_RADIUS
    # Mapping-harness v2 (2026-07-18 spec): coverage waypoints build_map.py
    # drives, and the ~/adt4_output/<map_name>/ output directory name.
    tour: list = field(default_factory=list)
    map_name: str = ""
    gt: GtSpec = field(default_factory=GtSpec)
    # Task 15e: opt-in publishing of a ground-truth semantic LABEL image on
    # the ZED `semantic/gt_image_raw` topic hydra can be pointed at (isaac_sim
    # overlay), so hydra places DSG object nodes from pixel-perfect GT masks
    # instead of FastSAM's range-biased ones (see docs/sim_runbook.md §12.12).
    # This is INDEPENDENT of `gt` (the mapping-harness file capture): it is a
    # lightweight ROS publisher, not the manifest/frame writer. Default OFF so
    # kinematic/P1 behaviour is byte-identical (no annotator, no publisher).
    gt_semantics_pub: bool = False
    nav: NavSpec = field(default_factory=NavSpec)
    # Directory the scenario YAML lives in. Asset paths (environment_usd,
    # ObjectSpec.usd) are stored exactly as authored (relative to this
    # directory, matching the spec's `interfaces` contract) rather than
    # eagerly rewritten to absolute paths. Callers that need a real
    # filesystem path (e.g. stage.py loading a USD reference) should join
    # relative paths against `base_dir` via `resolve_path`.
    base_dir: str = ""

    @property
    def physics_mode(self) -> bool:
        return any(r.locomotion == "policy" or r.grasping == "physics"
                   for r in self.robots)

    def resolve_path(self, usd: str) -> str:
        if os.path.isabs(usd):
            return usd
        return os.path.join(self.base_dir, usd)


def _require(d: dict, key: str, context: str):
    if key not in d:
        raise ValueError(f"missing required field '{key}' in {context}")
    return d[key]


def load_scenario(path) -> Scenario:
    path = str(path)
    base_dir = os.path.dirname(os.path.abspath(path))

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    env = _require(data, "environment", "scenario")
    # Asset paths are kept exactly as authored (relative to the YAML's
    # directory); Scenario.resolve_path turns them into filesystem paths.
    environment_usd = _require(env, "usd", "environment")

    robots = []
    for i, r in enumerate(data.get("robots", [])):
        context = f"robots[{i}]"
        name = _require(r, "name", context)
        spawn = _require(r, "spawn", context)
        locomotion = _require(r, "locomotion", context)
        grasping = _require(r, "grasping", context)
        contact_hold = bool(r.get("contact_hold", False))

        if locomotion not in LOCOMOTIONS:
            raise ValueError(
                f"invalid locomotion '{locomotion}' for robot '{name}': "
                f"must be one of {sorted(LOCOMOTIONS)}"
            )
        if grasping not in GRASPING_MODES:
            raise ValueError(
                f"invalid grasping '{grasping}' for robot '{name}': "
                f"must be one of {sorted(GRASPING_MODES)}"
            )

        if contact_hold and grasping != "physics":
            raise ValueError(
                f"robot '{name}': contact_hold requires grasping: physics")

        robots.append(
            RobotSpec(
                name=name,
                x=float(_require(spawn, "x", f"{context}.spawn")),
                y=float(_require(spawn, "y", f"{context}.spawn")),
                z=float(_require(spawn, "z", f"{context}.spawn")),
                yaw=float(_require(spawn, "yaw", f"{context}.spawn")),
                locomotion=locomotion,
                grasping=grasping,
                contact_hold=contact_hold,
            )
        )

    objects = []
    seen_ids = set()
    for i, o in enumerate(data.get("objects", [])):
        context = f"objects[{i}]"
        object_id = _require(o, "id", context)
        usd = _require(o, "usd", context)
        label = _require(o, "label", context)
        pose = _require(o, "pose", context)

        if object_id in seen_ids:
            raise ValueError(f"duplicate object id '{object_id}'")
        seen_ids.add(object_id)

        objects.append(
            ObjectSpec(
                object_id=object_id,
                usd=usd,
                label=label,
                x=float(_require(pose, "x", f"{context}.pose")),
                y=float(_require(pose, "y", f"{context}.pose")),
                z=float(_require(pose, "z", f"{context}.pose")),
                yaw=float(_require(pose, "yaw", f"{context}.pose")),
                graspable=bool(o.get("graspable", True)),
            )
        )

    grasp_radius = float(data.get("grasp_radius", DEFAULT_GRASP_RADIUS))

    tour = []
    for i, w in enumerate(data.get("tour", [])):
        context = f"tour[{i}]"
        dwell_s = float(w.get("dwell_s", 0.0))
        if dwell_s < 0:
            raise ValueError(f"negative dwell_s in {context}")
        tour.append(
            TourWaypoint(
                x=float(_require(w, "x", context)),
                y=float(_require(w, "y", context)),
                yaw=float(_require(w, "yaw", context)),
                dwell_s=dwell_s,
            )
        )

    gt = GtSpec()
    gt_data = data.get("gt")
    if gt_data is not None:
        mode = gt_data.get("mode", "live")
        if mode not in GT_MODES:
            raise ValueError(
                f"invalid gt mode '{mode}': must be one of {sorted(GT_MODES)}"
            )
        rate_hz = float(gt_data.get("rate_hz", 2.0))
        if rate_hz <= 0:
            raise ValueError("gt rate_hz must be > 0")
        modalities = list(gt_data.get("modalities", DEFAULT_GT_MODALITIES))
        for m in modalities:
            if m not in GT_MODALITIES:
                raise ValueError(
                    f"unknown gt modality '{m}': must be from {sorted(GT_MODALITIES)}"
                )
        semantics = []
        for i, s in enumerate(gt_data.get("semantics", [])):
            context = f"gt.semantics[{i}]"
            pattern = _require(s, "match", context)
            try:
                re.compile(pattern)
            except re.error as e:
                raise ValueError(f"invalid regex in {context}: {e}")
            semantics.append(
                GtSemanticRule(
                    match=pattern, semantic_class=_require(s, "class", context)
                )
            )
        gt = GtSpec(
            enabled=bool(gt_data.get("enabled", True)),
            mode=mode,
            rate_hz=rate_hz,
            modalities=modalities,
            semantics=semantics,
        )

    nav = NavSpec()
    nav_data = data.get("nav")
    if nav_data is not None:
        nav_dict = {}
        for key in ["cell_size_m", "inflation_radius_m", "snap_bound_m", "stuck_timeout_s", "max_lin_speed", "max_ang_speed"]:
            if key in nav_data:
                value = float(nav_data[key])
                if value <= 0:
                    raise ValueError(f"nav.{key} must be > 0")
                nav_dict[key] = value
        # snap_standoff_m may be 0 (disabled); only reject negatives.
        if "snap_standoff_m" in nav_data:
            value = float(nav_data["snap_standoff_m"])
            if value < 0:
                raise ValueError("nav.snap_standoff_m must be >= 0")
            nav_dict["snap_standoff_m"] = value
        nav = NavSpec(**nav_dict)

    scenario = Scenario(
        environment_usd=environment_usd,
        robots=robots,
        objects=objects,
        grasp_radius=grasp_radius,
        tour=tour,
        map_name=str(data.get("map_name", "")),
        gt=gt,
        gt_semantics_pub=bool(data.get("gt_semantics_pub", False)),
        nav=nav,
        base_dir=base_dir,
    )

    if scenario.physics_mode and scenario.gt.enabled and scenario.gt.mode == "replay":
        raise ValueError(
            "gt.mode 'replay' is kinematic-only (teleporting a dynamic "
            "articulation is undefined); use mode 'live' in physics scenarios")

    return scenario
