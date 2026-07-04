"""Pure-python scenario loader for dcist_sim_isaac.

No Isaac / ROS imports here on purpose: this module must be importable
and testable with plain `python3` + `pyyaml`, independent of the Isaac
Sim python environment. See dcist_sim_isaac/README.md for rationale.
"""
from __future__ import annotations

import os
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
    yaw: float
    locomotion: str
    grasping: str


@dataclass
class ObjectSpec:
    object_id: str
    usd: str
    label: str
    x: float
    y: float
    z: float
    yaw: float
    graspable: bool = True


@dataclass
class Scenario:
    environment_usd: str
    robots: list = field(default_factory=list)
    objects: list = field(default_factory=list)
    # Task 9: magic-grasp selection radius (meters), optional in the
    # YAML (top-level `grasp_radius:` key), defaulting to
    # DEFAULT_GRASP_RADIUS.
    grasp_radius: float = DEFAULT_GRASP_RADIUS
    # Directory the scenario YAML lives in. Asset paths (environment_usd,
    # ObjectSpec.usd) are stored exactly as authored (relative to this
    # directory, matching the spec's `interfaces` contract) rather than
    # eagerly rewritten to absolute paths. Callers that need a real
    # filesystem path (e.g. stage.py loading a USD reference) should join
    # relative paths against `base_dir` via `resolve_path`.
    base_dir: str = ""

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

        robots.append(
            RobotSpec(
                name=name,
                x=float(_require(spawn, "x", f"{context}.spawn")),
                y=float(_require(spawn, "y", f"{context}.spawn")),
                z=float(_require(spawn, "z", f"{context}.spawn")),
                yaw=float(_require(spawn, "yaw", f"{context}.spawn")),
                locomotion=locomotion,
                grasping=grasping,
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

    return Scenario(
        environment_usd=environment_usd,
        robots=robots,
        objects=objects,
        grasp_radius=grasp_radius,
        base_dir=base_dir,
    )
