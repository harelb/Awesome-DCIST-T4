"""Bake the viewer's robot models (public/models/*.glb) from the robots' URDF meshes.

Each model is the URDF's visual meshes placed by its joint chain in a fixed pose
(Spot standing, Husky as is), decimated to a few tens of thousands of triangles in
all and written as one GLB with one named mesh per URDF part and per-vertex
colours, in the frame the robot's pose is reported in (Spot: ``body``, Husky:
``base_link``; x forward, z up).

    spot  : spot_tools_ros/meshes/visual/spot_yellow (spot_macro.xacro joint origins)
    husky : clearpath_platform_description a200 (``apt-get download
            ros-jazzy-clearpath-platform-description``; dpkg-deb -x it anywhere)

Usage (needs trimesh, fast_simplification, pycollada):

    python scripts/bake_robot_models.py --spot <spot_tools_ros dir> \
        --clearpath <.../share/clearpath_platform_description> [--out public/models]
"""

from __future__ import annotations

import argparse
import math
import pathlib

import numpy as np
import trimesh
from trimesh import transformations as tf

# Spot: spot_macro.xacro joint origins; standing pose (hip_y, knee) in radians
SPOT_HIP_X = {"front_left": (0.29785, 0.055), "front_right": (0.29785, -0.055), "rear_left": (-0.29785, 0.055), "rear_right": (-0.29785, -0.055)}
SPOT_HIP_Y_OFFSET = 0.110945
SPOT_KNEE_ORIGIN = (0.025, 0.0, -0.3205)
SPOT_STAND = {"hip_x": 0.0, "hip_y": 0.8, "knee": -1.6}

# Husky A200: a200.urdf.xacro + drivetrain/wheels.urdf.xacro
A200_WHEEL_BASE = 0.512
A200_WHEEL_TRACK = 0.555
A200_WHEEL_Z = 0.03282

TARGET_TRIS = {"spot": 40000, "husky": 30000}
BOUNDS_TOL_M = 0.01  # a decimated part must still span its bounding box to 1 cm


def T(xyz=(0, 0, 0), axis=None, angle=0.0):
    m = tf.translation_matrix(xyz)
    if axis is not None and angle:
        m = m @ tf.rotation_matrix(angle, axis)
    return m


def load_colored(path: pathlib.Path) -> trimesh.Trimesh:
    """One mesh with per-vertex colours from its materials (OBJ+MTL or DAE)."""
    loaded = trimesh.load(path, force="scene")
    parts = []
    # every placement of every geometry, with its scene-graph transform (DAE nodes)
    for node in loaded.graph.nodes_geometry:
        mat4, name = loaded.graph[node]
        g = loaded.geometry[name].copy()
        color = np.array([180, 180, 180, 255], dtype=np.uint8)
        vis = g.visual
        mat = getattr(vis, "material", None)
        if mat is not None:
            c = getattr(mat, "main_color", None)
            if c is None and hasattr(mat, "to_simple"):
                c = mat.to_simple().main_color
            if c is not None:
                color = np.asarray(c, dtype=np.uint8)
        elif hasattr(vis, "face_colors") and vis.kind == "face":
            color = np.median(vis.face_colors, axis=0).astype(np.uint8)
        g.visual = trimesh.visual.ColorVisuals(g, vertex_colors=np.tile(color, (len(g.vertices), 1)))
        g.apply_transform(mat4)
        parts.append(g)
    return trimesh.util.concatenate(parts)


def decimate(mesh: trimesh.Trimesh, target: int) -> trimesh.Trimesh:
    if len(mesh.faces) <= target:
        return mesh
    out = mesh.simplify_quadric_decimation(face_count=target)
    # big flat faces made of slivers (the A200 top chassis) can collapse: keep the
    # original part when the result no longer spans its bounding box
    if np.abs(out.bounds - mesh.bounds).max() > BOUNDS_TOL_M:
        print(f"  decimation collapsed a part ({len(mesh.faces)} tris); keeping it whole")
        return mesh
    # carry colours over by nearest original vertex
    _, idx = trimesh.proximity.ProximityQuery(mesh).vertex(out.vertices)
    out.visual = trimesh.visual.ColorVisuals(out, vertex_colors=mesh.visual.vertex_colors[idx])
    return out


def bake_spot(spot_dir: pathlib.Path, color: str = "yellow") -> trimesh.Scene:
    vis = spot_dir / "meshes" / "visual" / f"spot_{color}"
    parts = [("body", vis / "body.obj", np.eye(4))]
    for leg, (hx, hy) in SPOT_HIP_X.items():
        side = 1 if leg.endswith("left") else -1
        hip = T((hx, hy, 0), (1, 0, 0), SPOT_STAND["hip_x"])
        upper = hip @ T((0, side * SPOT_HIP_Y_OFFSET, 0), (0, 1, 0), SPOT_STAND["hip_y"])
        lower = upper @ T(SPOT_KNEE_ORIGIN, (0, 1, 0), SPOT_STAND["knee"])
        parts += [
            (f"{leg}_hip", vis / f"{leg}_hip.obj", hip),
            (f"{leg}_upper_leg", vis / f"{leg}_upper_leg.obj", upper),
            (f"{leg}_lower_leg", vis / f"{leg}_lower_leg.obj", lower),
        ]
    return assemble(parts, TARGET_TRIS["spot"])


def bake_husky(cp_dir: pathlib.Path) -> trimesh.Scene:
    m = cp_dir / "meshes" / "a200"
    # the chassis top is coplanar with base_link's; lift it 3 mm so it wins the depth test
    parts = [("base_link", m / "base_link.dae", np.eye(4)), ("top_chassis", m / "top_chassis.dae", T((0, 0, 0.003)))]
    for sx, fb in ((1, "front"), (-1, "rear")):
        for sy, lr in ((1, "left"), (-1, "right")):
            parts.append((f"{fb}_{lr}_wheel", m / "wheels" / "outdoor.dae", T((sx * A200_WHEEL_BASE / 2, sy * A200_WHEEL_TRACK / 2, A200_WHEEL_Z))))
    return assemble(parts, TARGET_TRIS["husky"])


MIN_PART_TRIS = 3000  # boxy parts lose their shape (and gain holes) below this


def assemble(parts, target) -> trimesh.Scene:
    """One named mesh per URDF part (the viewer outlines each part's hull)."""
    loaded = []
    for name, path, mat in parts:
        g = load_colored(path)
        g.apply_transform(mat)
        loaded.append((name, g))
    total = sum(len(g.faces) for _, g in loaded)
    scene = trimesh.Scene()
    for name, g in loaded:  # each part keeps its share of the triangle budget
        share = int(target * len(g.faces) / total)
        scene.add_geometry(decimate(g, max(share, min(len(g.faces), MIN_PART_TRIS))), node_name=name, geom_name=name)
    return scene


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spot", type=pathlib.Path, required=True)
    ap.add_argument("--clearpath", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1] / "public" / "models")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    for name, scene in (("spot", bake_spot(a.spot)), ("husky", bake_husky(a.clearpath))):
        path = a.out / f"{name}.glb"
        path.write_bytes(scene.export(file_type="glb"))
        lo, hi = scene.bounds
        tris = sum(len(g.faces) for g in scene.geometry.values())
        print(f"{path}: {len(scene.geometry)} parts, {tris} tris, {path.stat().st_size / 1e6:.2f} MB, "
              f"bounds x[{lo[0]:.2f},{hi[0]:.2f}] y[{lo[1]:.2f},{hi[1]:.2f}] z[{lo[2]:.2f},{hi[2]:.2f}]")  # fmt: skip


if __name__ == "__main__":
    main()
