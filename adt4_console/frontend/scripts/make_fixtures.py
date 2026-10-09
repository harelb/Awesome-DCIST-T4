#!/usr/bin/env python3
"""Generate wire-format fixtures for the frontend from the real fleetlink codec.

Run (from adt4_console/frontend):

    PYTHONPATH=../../fleetlink /home/harel/environments/dcist/spark_env/bin/python3 \
        scripts/make_fixtures.py

Writes:
  tests/fixtures/graph_small.msgpack   + graph_small.json   (expected decoded values)
  tests/fixtures/delta_small.msgpack   + delta_small.json   (delta + expected state after apply)
  tests/fixtures/tile_small.msgpack    + tile_small.json    (expected decoded vertices/faces)
  tests/fixtures/mock/                 (a synthetic building for scripts/mock-server.mjs)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import random

import msgpack
import numpy as np

from fleetlink import codec
from fleetlink.codec import GraphDelta, GraphState, NodeRecord

HERE = pathlib.Path(__file__).resolve().parent
FIX = HERE.parent / "tests" / "fixtures"
# HEAVY=1 writes a performance fixture (~1.6M triangles, ~400 tiles, ~7k nodes)
# to tests/fixtures/mock-heavy (git-ignored); use it with MOCK_FIXTURES=... .
HEAVY = os.environ.get("HEAVY") == "1"
MOCK = FIX / ("mock-heavy" if HEAVY else "mock")
MESH_STEP = 0.06 if HEAVY else 0.5
TILE_SIZE = 3.5 if HEAVY else 8.0
OBJ_DENSITY = 0.6 if HEAVY else 18.0
PLACE_STEP = 0.8 if HEAVY else 2.5


def packb(obj) -> bytes:
    return msgpack.packb(obj, use_bin_type=True)


def rec_json(r: NodeRecord) -> dict:
    return {
        "sym": r.sym,
        "layer": r.layer,
        "part": r.part,
        "pos": [float(np.float32(v)) for v in r.pos],
        "label": r.label,
        "bbox": [float(np.float32(v)) for v in r.bbox] if r.bbox else None,
        "color": list(r.color),
        "name": r.name,
        "active": r.active,
    }


def state_json(st: GraphState) -> dict:
    return {
        "nodes": {s: rec_json(r) for s, r in sorted(st.records.items())},
        "edges": sorted([list(e) for e in st.edges]),
        "labelspaces": st.labelspaces,
    }


# ----------------------------------------------------------------------------
# small unit fixtures
# ----------------------------------------------------------------------------


def small_fixtures():
    recs = {
        "O(1)": NodeRecord(
            "O(1)",
            2,
            0,
            (1.0, 2.0, 0.5),
            18,
            (0.5, 1.5, 0.0, 1.5, 2.5, 1.0),
            (255, 0, 0),
            "sign",
            True,
        ),
        "O(2)": NodeRecord(
            "O(2)", 2, 0, (-3.25, 0.125, 0.75), 4, None, (0, 0, 0), "", False
        ),
        "P(10)": NodeRecord("P(10)", 3, 0, (0.0, 0.0, 1.0), -1),
        "P(11)": NodeRecord("P(11)", 3, 0, (4.0, 0.0, 1.0), -1),
        "a(0)": NodeRecord("a(0)", 2, 97, (0.0, -1.0, 0.0), -1),
    }
    edges = {("P(10)", "P(11)"), ("O(1)", "P(10)"), ("a(0)", "P(10)")}
    ls = {"2/0": {"18": "sign", "4": "chair"}}
    snap = codec.encode_graph(recs, edges, ls)
    (FIX / "graph_small.msgpack").write_bytes(packb(snap))
    st = GraphState()
    st.load_snapshot(snap)
    (FIX / "graph_small.json").write_text(json.dumps(state_json(st), indent=1))

    # delta: move O(1), add O(3), delete O(2), remove one edge, add one
    new = dict(st.records)
    new["O(1)"] = NodeRecord(
        "O(1)",
        2,
        0,
        (1.5, 2.0, 0.5),
        18,
        (0.5, 1.5, 0.0, 2.5, 2.5, 1.0),
        (255, 0, 0),
        "sign",
        True,
    )
    new["O(3)"] = NodeRecord(
        "O(3)", 2, 0, (5.0, 5.0, 0.0), 4, None, (10, 20, 30), "chair", True
    )
    new.pop("O(2)")
    new_edges = (set(st.edges) - {("a(0)", "P(10)")}) | {("O(3)", "P(11)")}
    new_ls = {"2/0": {"18": "sign", "4": "chair", "7": "table"}}
    delta = st.diff(new, new_edges, new_ls)
    enc = delta.encode()
    enc["seq"] = 7
    (FIX / "delta_small.msgpack").write_bytes(packb(enc))
    st.apply(GraphDelta.decode(enc))
    (FIX / "delta_small.json").write_text(json.dumps(state_json(st), indent=1))

    # tile: a 2-triangle quad
    pos = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.25]],
        np.float32,
    )
    col = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255], [10, 20, 30]], np.uint8)
    faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    tiles = codec.tile_mesh(pos, col, faces, tile_size=8.0)
    ((key, tile),) = tiles.items()
    (FIX / "tile_small.msgpack").write_bytes(packb(tile))
    p, c, i = codec.decode_tile(tile)
    (FIX / "tile_small.json").write_text(
        json.dumps(
            {
                "key": key,
                "nv": tile["nv"],
                "nf": tile["nf"],
                "pos": [float(v) for v in p.reshape(-1)],
                "col": [int(v) for v in c.reshape(-1)],
                "idx": [int(v) for v in i.reshape(-1)],
            },
            indent=1,
        )
    )


# ----------------------------------------------------------------------------
# synthetic building for the mock server
# ----------------------------------------------------------------------------

LABELS = {
    0: "wall",
    1: "floor",
    2: "door",
    3: "table",
    4: "chair",
    5: "sign",
    6: "shelf",
    7: "trash can",
    8: "fire extinguisher",
    9: "box",
    10: "couch",
    11: "plant",
    12: "monitor",
    13: "cabinet",
    14: "vehicle",
    15: "barrel",
}
ROOMS = [  # (xmin, ymin, xmax, ymax, floor colour)
    (0, 0, 24, 16, (150, 140, 120)),
    (24, 0, 48, 16, (120, 135, 150)),
    (0, 16, 48, 24, (135, 135, 135)),  # hallway
    (0, 24, 20, 40, (130, 150, 120)),
    (20, 24, 48, 40, (155, 130, 125)),
]


def build_mesh(rng):
    verts, cols, faces = [], [], []

    def quad(p0, p1, p2, p3, c):
        b = len(verts)
        for p in (p0, p1, p2, p3):
            verts.append(p)
            jitter = rng.integers(-12, 12, 3)
            cols.append(np.clip(np.array(c) + jitter, 0, 255))
        faces.extend([[b, b + 1, b + 2], [b, b + 2, b + 3]])

    step = MESH_STEP
    for x0, y0, x1, y1, c in ROOMS:
        for x in np.arange(x0, x1, step):
            for y in np.arange(y0, y1, step):
                z = 0.02 * math.sin(x * 0.7) * math.cos(y * 0.5)
                quad(
                    (x, y, z),
                    (x + step, y, z),
                    (x + step, y + step, z),
                    (x, y + step, z),
                    c,
                )
    wall_c = (205, 200, 190)
    h = 2.6

    def wall(xa, ya, xb, yb, gaps=()):
        L = math.hypot(xb - xa, yb - ya)
        n = int(L / step)
        for k in range(n):
            t0, t1 = k / n, (k + 1) / n
            mid = (t0 + t1) / 2 * L
            if any(g0 <= mid <= g1 for g0, g1 in gaps):
                continue
            pa = (xa + (xb - xa) * t0, ya + (yb - ya) * t0)
            pb = (xa + (xb - xa) * t1, ya + (yb - ya) * t1)
            for z in np.arange(0, h, step):
                shade = int(30 * z / h)
                quad(
                    (pa[0], pa[1], z),
                    (pb[0], pb[1], z),
                    (pb[0], pb[1], z + step),
                    (pa[0], pa[1], z + step),
                    tuple(v - shade for v in wall_c),
                )

    wall(0, 0, 48, 0)
    wall(48, 0, 48, 40)
    wall(48, 40, 0, 40)
    wall(0, 40, 0, 0)
    wall(0, 16, 48, 16, gaps=[(9, 11), (33, 35)])
    wall(24, 0, 24, 16, gaps=[(7, 9)])
    wall(0, 24, 48, 24, gaps=[(8, 10), (30, 32)])
    wall(20, 24, 20, 40, gaps=[(9, 11)])
    return (
        np.array(verts, np.float32),
        np.array(cols, np.uint8),
        np.array(faces, np.uint32),
    )


def build_graph(rng):
    recs: dict[str, NodeRecord] = {}
    edges: set[tuple[str, str]] = set()
    # places: grid inside rooms
    places = []
    pid = 0
    for x in np.arange(2, 48, PLACE_STEP):
        for y in np.arange(2, 40, PLACE_STEP):
            if abs(y - 16) < 0.6 or abs(y - 24) < 0.6:
                continue
            sym = f"P({pid})"
            recs[sym] = NodeRecord(sym, 3, 0, (float(x), float(y), 1.0), -1)
            places.append((sym, x, y))
            pid += 1
    for i, (s, x, y) in enumerate(places):
        for s2, x2, y2 in places[i + 1 :]:
            if abs(x - x2) > PLACE_STEP * 1.1:
                break
            if math.hypot(x - x2, y - y2) < PLACE_STEP * 1.05 and not _crosses_wall(
                x, y, x2, y2
            ):
                edges.add((s, s2))
    place_xy = np.array([[p[1], p[2]] for p in places])
    # objects
    oid = 0
    for room_i, (x0, y0, x1, y1, _) in enumerate(ROOMS):
        n = int((x1 - x0) * (y1 - y0) / OBJ_DENSITY)
        for _ in range(n):
            lab = int(
                rng.choice([2, 3, 4, 4, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15])
            )
            x = float(rng.uniform(x0 + 1, x1 - 1))
            y = float(rng.uniform(y0 + 1, y1 - 1))
            sx, sy, sz = (rng.uniform(0.3, 1.4, 3) * [1, 1, 1.2]).tolist()
            z = sz / 2
            sym = f"O({oid})"
            recs[sym] = NodeRecord(
                sym,
                2,
                0,
                (x, y, z),
                lab,
                (x - sx / 2, y - sy / 2, 0.0, x + sx / 2, y + sy / 2, sz),
                (0, 0, 0),
                LABELS[lab],
                False,
            )
            near = places[
                int(np.argmin((place_xy[:, 0] - x) ** 2 + (place_xy[:, 1] - y) ** 2))
            ]
            edges.add((near[0], sym))
            oid += 1
    # rooms
    for i, (x0, y0, x1, y1, _) in enumerate(ROOMS):
        sym = f"R({i})"
        recs[sym] = NodeRecord(
            sym, 4, 0, ((x0 + x1) / 2, (y0 + y1) / 2, 1.0), -1, name=f"room {i}"
        )
        for s, x, y in places:
            if x0 <= x < x1 and y0 <= y < y1:
                edges.add((sym, s))
    recs["B(0)"] = NodeRecord("B(0)", 5, 0, (24.0, 20.0, 1.0), -1, name="building")
    for i in range(len(ROOMS)):
        edges.add(("B(0)", f"R({i})"))
    # agent trajectory: loop through rooms
    waypoints = [
        (4, 4),
        (20, 4),
        (20, 12),
        (30, 12),
        (44, 4),
        (44, 12),
        (34, 14),
        (34, 20),
        (10, 20),
        (9, 30),
        (4, 36),
        (16, 36),
        (16, 28),
        (10, 22),
        (31, 22),
        (31, 30),
        (44, 36),
        (26, 36),
        (26, 28),
        (40, 20),
    ]
    pts = []
    for (xa, ya), (xb, yb) in zip(waypoints, waypoints[1:]):
        n = max(2, int(math.hypot(xb - xa, yb - ya) / 0.6))
        for k in range(n):
            t = k / n
            pts.append((xa + (xb - xa) * t, ya + (yb - ya) * t))
    traj = []
    for k, (x, y) in enumerate(pts):
        sym = f"a({k})"
        recs[sym] = NodeRecord(sym, 2, 97, (float(x), float(y), 0.5), -1)
        if k:
            edges.add((f"a({k - 1})", sym))
        traj.append([1700000000.0 + k * 0.5, float(x), float(y), 0.5])
    labelspaces = {"2/0": {str(k): v for k, v in LABELS.items()}}
    return recs, edges, labelspaces, traj


def _crosses_wall(x, y, x2, y2):
    for wy in (16, 24):
        if (y - wy) * (y2 - wy) < 0:
            mx = (x + x2) / 2
            if not ((8 <= mx <= 11) or (30 <= mx <= 35)):
                return True
    return False


def mock_fixtures():
    rng = np.random.default_rng(3)
    random.seed(3)
    MOCK.mkdir(parents=True, exist_ok=True)
    (MOCK / "tiles").mkdir(exist_ok=True)
    pos, col, faces = build_mesh(rng)
    tiles = codec.tile_mesh(pos, col, faces, tile_size=TILE_SIZE)
    manifest = {"tile_size": TILE_SIZE, "tiles": {}}
    for k, t in tiles.items():
        (MOCK / "tiles" / f"{k}.msgpack").write_bytes(packb(t))
        manifest["tiles"][k] = {
            "hash": t["hash"],
            "nv": t["nv"],
            "nf": t["nf"],
            "origin": t["origin"],
        }
    (MOCK / "manifest.json").write_text(json.dumps(manifest))

    recs, edges, ls, traj = build_graph(rng)
    snap = codec.encode_graph(recs, edges, ls)
    (MOCK / "graph_prior.msgpack").write_bytes(packb(snap))
    (MOCK / "trajectory.json").write_text(json.dumps({"points": traj}))

    # live robot: first 40% of the trajectory and what it has seen so far
    n_traj = sum(1 for s in recs if s.startswith("a("))
    seen = int(n_traj * 0.4)
    live = {}
    for s, r in recs.items():
        if s.startswith("a("):
            if int(s[2:-1]) < seen:
                live[s] = r
        elif r.layer in (2, 3) and r.pos[1] < 16:
            live[s] = r
    live_edges = {e for e in edges if e[0] in live and e[1] in live}
    (MOCK / "graph_live.msgpack").write_bytes(
        packb(codec.encode_graph(live, live_edges, ls))
    )
    # a delta sequence the mock server streams for the live robot
    deltas = []
    st = GraphState()
    st.load_snapshot(codec.encode_graph(live, live_edges, ls))
    cur = dict(live)
    for k in range(seen, min(n_traj, seen + 60)):
        sym = f"a({k})"
        cur[sym] = recs[sym]
        cur_edges = {e for e in edges if e[0] in cur and e[1] in cur}
        d = st.diff(cur, cur_edges, None)
        st.apply(d)
        deltas.append(packb(d.encode()))
    (MOCK / "live_deltas.msgpack").write_bytes(packb(deltas))
    print(
        f"mesh {len(faces)} faces in {len(tiles)} tiles; graph {len(recs)} nodes {len(edges)} edges"
    )


if __name__ == "__main__":
    FIX.mkdir(parents=True, exist_ok=True)
    if not HEAVY:
        small_fixtures()
    mock_fixtures()
    print("fixtures written to", FIX)
