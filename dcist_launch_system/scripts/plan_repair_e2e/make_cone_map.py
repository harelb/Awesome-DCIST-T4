#!/usr/bin/env python3
"""A small prior map for the plan-repair v5 cone scenario (run_scenario.py).

Mesh places: a 10 x 4 grid 2 m apart (x 0..18, y 0..6), 4-connected. Rooms R0
(x <= 8) and R1 (x >= 10). Objects:

  O20  cone   (17, 3)  -- the goal: "go to the nearest cone" starts as O20
  O5   chair  ( 3, 6)  -- unrelated
  O6   tree   ( 9, 6)  -- unrelated

The robot (hilbert) starts at (1, 1). A cone added later at (5, 1) is closer.

    make_cone_map.py OUT_RUN_DIR   ->  OUT_RUN_DIR/hydra/backend/dsg.json
"""

import pathlib
import sys

import numpy as np
import spark_dsg

NX, NY, STEP = 10, 4, 2.0
LABELS = {0: "unknown", 1: "cone", 2: "chair", 3: "tree"}
OBJECTS = {20: ("cone", (17.0, 3.0)), 5: ("chair", (3.0, 6.0)), 6: ("tree", (9.0, 6.0))}


def pid(i, j):
    return spark_dsg.NodeSymbol("P", i * NY + j)


def build() -> spark_dsg.DynamicSceneGraph:
    G = spark_dsg.DynamicSceneGraph()
    G.add_layer(3, 1, spark_dsg.DsgLayers.MESH_PLACES)
    for i in range(NX):
        for j in range(NY):
            a = spark_dsg.PlaceNodeAttributes()
            a.position = np.array([i * STEP, j * STEP, 0.0])
            G.add_node(spark_dsg.DsgLayers.MESH_PLACES, pid(i, j).value, a)
    for i in range(NX):
        for j in range(NY):
            if i + 1 < NX:
                G.insert_edge(pid(i, j).value, pid(i + 1, j).value)
            if j + 1 < NY:
                G.insert_edge(pid(i, j).value, pid(i, j + 1).value)
    for r, (lo, hi) in enumerate([(0, 4), (5, 9)]):
        a = spark_dsg.RoomNodeAttributes()
        a.position = np.array([(lo + hi) / 2 * STEP, 3.0, 0.0])
        a.name = f"room {r}"
        rid = spark_dsg.NodeSymbol("R", r)
        G.add_node(spark_dsg.DsgLayers.ROOMS, rid.value, a)
        for i in range(lo, hi + 1):
            for j in range(NY):
                G.insert_edge(rid.value, pid(i, j).value)
    ids = {v: k for k, v in LABELS.items()}
    for idx, (cls, (x, y)) in OBJECTS.items():
        a = spark_dsg.ObjectNodeAttributes()
        a.position = np.array([x, y, 0.3])
        a.semantic_label = ids[cls]
        a.name = cls
        a.bounding_box = spark_dsg.BoundingBox(np.array([0.4, 0.4, 0.6]), np.array([x, y, 0.3]))
        sym = spark_dsg.NodeSymbol("O", idx)
        G.add_node(spark_dsg.DsgLayers.OBJECTS, sym.value, a)
        # the object's place: the nearest mesh place
        i, j = min(((i, j) for i in range(NX) for j in range(NY)),
                   key=lambda ij: (ij[0] * STEP - x) ** 2 + (ij[1] * STEP - y) ** 2)
        G.insert_edge(pid(i, j).value, sym.value)
    G.set_labelspace(spark_dsg.Labelspace(LABELS), 2, 0)
    return G


def main(out: str) -> pathlib.Path:
    path = pathlib.Path(out) / "hydra" / "backend" / "dsg.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    build().save(str(path), include_mesh=False)
    return path


if __name__ == "__main__":
    print(main(sys.argv[1] if len(sys.argv) > 1 else "cone_map"))
