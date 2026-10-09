"""A tiny Hydra-like run directory built with spark_dsg (objects, places, agents, mesh, images)."""

import datetime
import pathlib

import numpy as np

# sym index -> (label, position); label 1 = chair, 2 = table
OBJECTS = {0: (1, (1.0, 0.0, 0.0)), 1: (1, (9.0, 0.0, 0.0)), 2: (2, (2.0, 2.0, 0.0))}
LABELS = {0: "unknown", 1: "chair", 2: "table"}
JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 64 + b"\xff\xd9"

PLY = """ply
format ascii 1.0
element vertex 4
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face 2
property list uchar uint vertex_indices
end_header
0 0 0 255 0 0
1 0 0 0 255 0
0 1 0 0 0 255
30 30 0 10 10 10
3 0 1 2
3 0 1 3
"""


def build_run(root: pathlib.Path) -> pathlib.Path:
    import spark_dsg as dsg

    run = root / "synthetic_run"
    backend = run / "hydra" / "backend"
    backend.mkdir(parents=True)
    G = dsg.DynamicSceneGraph()
    G.set_labelspace(dsg.Labelspace(LABELS), 2, 0)
    for i, (label, pos) in OBJECTS.items():
        a = dsg.KhronosObjectAttributes()
        a.position = np.array(pos)
        a.semantic_label = label
        a.bounding_box = dsg.BoundingBox(
            np.array([0.5, 0.5, 0.5], np.float32), np.array(pos, np.float32)
        )
        # stale absolute folder from the robot; only the basename is meaningful
        a.image_folder = f"/media/robot/elsewhere/images/temp/O_{i + 10}"
        G.add_node(dsg.DsgLayers.OBJECTS, dsg.NodeSymbol("O", i), a)
    for i in range(2):
        p = dsg.PlaceNodeAttributes()
        p.position = np.array([float(i), 1.0, 0.0])
        G.add_node(3, dsg.NodeSymbol("P", i), p)
    for k, t in enumerate((30.0, 10.0, 20.0)):  # inserted out of time order
        ag = dsg.AgentNodeAttributes()
        ag.position = np.array([t, 0.0, 0.0])
        ag.timestamp = datetime.timedelta(seconds=t)
        G.add_node(2, dsg.NodeSymbol("a", k), ag, 97)
    G.insert_edge(dsg.NodeSymbol("P", 0), dsg.NodeSymbol("P", 1))
    G.insert_edge(dsg.NodeSymbol("P", 0), dsg.NodeSymbol("O", 0))
    G.save(str(backend / "dsg.json"), False)
    (backend / "mesh.ply").write_text(PLY)
    folder = run / "images" / "O_10"  # images of O(0)
    folder.mkdir(parents=True)
    for ts in (2, 1):
        (folder / f"frame_{ts}_rgb.jpg").write_bytes(JPEG)
        (folder / f"frame_{ts}_mask.png").write_bytes(b"png")
    return run
