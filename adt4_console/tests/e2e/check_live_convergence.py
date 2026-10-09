#!/usr/bin/env python3
"""End-to-end check: does the console's live fleetlink mirror match what the robot saved?

Run after a bag/robot session that had the fleetlink window, once Hydra has saved
(``/<robot>/shutdown``):

    check_live_convergence.py --console http://localhost:8095 --robot euclid \
        --run-dir ~/adt4_output/<run> [--token TOKEN]

Compares the ``live:<robot>`` snapshot against ``<run>/hydra/backend/dsg.json``:
node set, positions (1 cm), labels and edges; and the live mesh tiles against
the saved mesh.  Exits non-zero if node/edge agreement is below ``--min-match``.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import urllib.request

import msgpack
import numpy as np


def fetch(url, token=None, raw=False):
    req = urllib.request.Request(url)
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, timeout=60) as r:
        data = r.read()
    return data if raw else json.loads(data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--console", default="http://localhost:8095")
    ap.add_argument("--robot", required=True)
    ap.add_argument("--run-dir", required=True, type=pathlib.Path)
    ap.add_argument("--token")
    ap.add_argument("--min-match", type=float, default=0.98)
    args = ap.parse_args()

    import spark_dsg as dsg
    from fleetlink import codec

    src = f"live:{args.robot}"
    snap = msgpack.unpackb(
        fetch(f"{args.console}/api/graph/{src}/snapshot", args.token, raw=True),
        raw=False,
        strict_map_key=False,
    )
    live = codec.GraphState()
    live.load_snapshot(snap)
    link = fetch(f"{args.console}/api/live", args.token)["robots"][args.robot]

    G = dsg.DynamicSceneGraph.load(str(args.run_dir / "hydra/backend/dsg.json"))
    saved = codec.graph_records(G)
    saved_edges = codec.graph_edges(G)

    common = saved.keys() & live.records.keys()
    only_saved = saved.keys() - live.records.keys()
    only_live = live.records.keys() - saved.keys()
    d = np.array(
        [np.linalg.norm(np.subtract(saved[s].pos, live.records[s].pos)) for s in common]
    )
    within = int((d <= 0.01 + 1e-6).sum()) if len(d) else 0
    label_ok = sum(saved[s].label == live.records[s].label for s in common)
    e_common = len(saved_edges & live.edges)

    by_layer = {}
    for s in saved:
        k = f"{saved[s].layer}/{saved[s].part}"
        hit = s in live.records
        a, b = by_layer.get(k, (0, 0))
        by_layer[k] = (a + 1, b + hit)

    node_match = within / max(len(saved), 1)
    edge_match = e_common / max(len(saved_edges), 1)
    report = {
        "saved_nodes": len(saved),
        "live_nodes": len(live.records),
        "common": len(common),
        "only_saved": len(only_saved),
        "only_live": len(only_live),
        "pos_within_1cm": within,
        "pos_err_max_m": float(d.max()) if len(d) else None,
        "pos_err_p95_m": float(np.percentile(d, 95)) if len(d) else None,
        "label_agree": label_ok,
        "saved_edges": len(saved_edges),
        "live_edges": len(live.edges),
        "edges_common": e_common,
        "node_match": round(node_match, 4),
        "edge_match": round(edge_match, 4),
        "by_layer(saved,live)": by_layer,
        "link": {
            k: link.get(k)
            for k in ("bytes", "gaps", "resyncs", "tiles", "staleness_s", "rtt_ms")
        },
        "examples_only_saved": sorted(only_saved)[:8],
        "examples_only_live": sorted(only_live)[:8],
    }
    mesh_ply = args.run_dir / "hydra/backend/mesh.ply"
    if mesh_ply.exists():
        report["saved_mesh_mb"] = round(mesh_ply.stat().st_size / 1e6, 1)
    print(json.dumps(report, indent=2))
    ok = node_match >= args.min_match and edge_match >= args.min_match
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
