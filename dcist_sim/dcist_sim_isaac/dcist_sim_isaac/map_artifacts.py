"""Map output verification + provenance for the mapping harness.

Import contract: stdlib + pyyaml only at module import time. spark_dsg is
imported lazily inside the default graph-stats loader so pytest (plain
python3) never needs it -- tests inject a fake via `graph_stats`.
"""
import os
import subprocess
from dataclasses import dataclass
from datetime import datetime

import yaml

REQUIRED_FILES = ("dsg_with_mesh.json", "mesh.ply")
# Submodules stamped into provenance (paths relative to repo root).
PROVENANCE_REPOS = {"parent": ".", "spot_tools": "spot_tools",
                    "hydra": "hydra", "hydra_ros": "hydra_ros"}


@dataclass
class MapSanity:
    min_objects: int = 1
    min_places: int = 10
    min_mesh_vertices: int = 1000


def _spark_dsg_stats(dsg_path):
    import spark_dsg  # lazy: only available in spark_env

    g = spark_dsg.DynamicSceneGraph.load(dsg_path)
    mesh = g.mesh
    return {
        "objects": len(list(g.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes)),
        "places": len(list(g.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes)),
        # spark_dsg get_vertices() is 6xN (xyz+rgb) -- count columns.
        "mesh_vertices": 0 if mesh is None else mesh.get_vertices().shape[1],
    }


def verify_map(map_dir, sanity=None, graph_stats=None):
    sanity = sanity or MapSanity()
    failures = []
    for name in REQUIRED_FILES:
        p = os.path.join(map_dir, name)
        if not os.path.isfile(p):
            failures.append(f"missing {name}")
        elif os.path.getsize(p) == 0:
            failures.append(f"empty {name}")
    if failures:
        return failures

    stats = (graph_stats or _spark_dsg_stats)(
        os.path.join(map_dir, "dsg_with_mesh.json")
    )
    checks = (
        ("objects", sanity.min_objects),
        ("places", sanity.min_places),
        ("mesh_vertices", sanity.min_mesh_vertices),
    )
    for key, floor in checks:
        if stats[key] < floor:
            failures.append(f"{key}={stats[key]} below minimum {floor}")
    return failures


def _git_sha(repo_dir):
    return subprocess.run(
        ["git", "-C", repo_dir, "rev-parse", "HEAD"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()


def write_provenance(map_dir, scenario_path, tour_stats, repo_root, sha_fn=None,
                     scenario=None):
    sha_fn = sha_fn or _git_sha
    shas = {}
    for name, rel in PROVENANCE_REPOS.items():
        repo = os.path.join(repo_root, rel)
        try:
            shas[name] = sha_fn(repo)
        except Exception as e:  # missing submodule dir etc. -- record, don't die
            shas[name] = f"unavailable: {e}"
    with open(scenario_path, "r") as f:
        scenario_yaml = f.read()
    out = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "scenario_path": os.path.abspath(scenario_path),
        "scenario_yaml": scenario_yaml,
        "tour_stats": dict(tour_stats),
        "git": shas,
    }
    # Fidelity tier per robot (spec §7): which locomotion/grasp path this map
    # was actually built with, so a map's provenance records whether it came
    # from the kinematic or physics pipeline. Only emitted when the caller
    # threads the loaded scenario in (build_map does; tests may not).
    if scenario is not None:
        out["fidelity"] = {
            r.name: {
                "locomotion": r.locomotion,
                "grasping": r.grasping,
                "contact_hold": r.contact_hold,
            }
            for r in scenario.robots
        }
    path = os.path.join(map_dir, "provenance.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    return path
