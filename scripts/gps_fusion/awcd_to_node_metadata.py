#!/usr/bin/env python3
"""Turn recorded AWCD change sets into per-session node metadata for Hydra-Multi.

A later session relocalizes against a prior map with the active window change detector
(``relocalize_awcd``) and its change set is recorded to JSON (``awcd_change_recorder``):

* removed objects are keyed by node ids of the PRIOR map -> annotated on the prior run,
* added objects are keyed by track ids of the LIVE robot -> resolved to the live robot's
  scene graph nodes through its image store (``track_bindings``, following its merges).

Each annotation is ``{"awcd_change": {...}}`` and is written to ``node_metadata.json`` of the
prior / live Hydra run directories, which ``hydra-multi-offline-fusion`` merges into node
metadata. Set ``merge_exclusion_metadata_key: awcd_change`` on the objects update functor so
changed objects are never merged across sessions.
"""

import argparse
import json
import pathlib
import sqlite3

KEY = "awcd_change"


def track_to_node(store_dir):
    """Map track id -> canonical node id from an image store."""
    db = pathlib.Path(store_dir) / "store.sqlite"
    conn = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    parents = dict(conn.execute("SELECT child_id, parent_id FROM node_aliases"))
    bindings = dict(conn.execute("SELECT track_id, node_id FROM track_bindings"))
    conn.close()

    def canonical(node):
        seen = set()
        while node in parents and node not in seen:
            seen.add(node)
            node = parents[node]
        return node

    return {track: canonical(node) for track, node in bindings.items()}


def load_existing(path):
    return json.loads(path.read_text()) if path.exists() else {}


def annotate(annotations, node_id, change):
    entry = annotations.setdefault(str(node_id), {}).setdefault(KEY, {})
    detected_by = entry.get("detected_by", [])
    if change["detected_by"] not in detected_by:
        detected_by.append(change["detected_by"])
    entry.update({k: v for k, v in change.items() if k != "detected_by"})
    entry["detected_by"] = detected_by


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("changes", type=pathlib.Path, help="awcd_changes.json")
    parser.add_argument("--robot", required=True, help="robot name inside the changes file")
    parser.add_argument("--session", required=True, help="label of the live session")
    parser.add_argument("--prior-hydra", type=pathlib.Path, required=True,
                        help="prior run's hydra directory (gets removed annotations)")
    parser.add_argument("--live-hydra", type=pathlib.Path, required=True,
                        help="live run's hydra directory (gets added annotations)")
    parser.add_argument("--live-store", type=pathlib.Path, required=True,
                        help="live run's image store (binds added tracks to nodes)")
    parser.add_argument("--min-confidence", type=float, default=0.0)
    args = parser.parse_args()

    changes = json.loads(args.changes.read_text())[args.robot]
    prior_path = args.prior_hydra / "node_metadata.json"
    live_path = args.live_hydra / "node_metadata.json"
    prior, live = load_existing(prior_path), load_existing(live_path)

    num_removed = 0
    for obj in changes["removed"]:
        if obj["change_confidence"] < args.min_confidence:
            continue
        annotate(prior, obj["id"], {
            "type": "removed",
            "detected_by": args.session,
            "removed_at_ns": obj["stamp_ns"],
            "change_confidence": obj["change_confidence"],
            "num_frames_observed": obj["num_frames_observed"],
        })
        num_removed += 1

    bindings = track_to_node(args.live_store)
    num_added = num_unbound = 0
    for obj in changes["added"]:
        if obj["change_confidence"] < args.min_confidence:
            continue
        node = bindings.get(obj["id"])
        if node is None:
            num_unbound += 1
            continue
        annotate(live, node, {
            "type": "added",
            "detected_by": args.session,
            "added_at_ns": obj["stamp_ns"],
            "change_confidence": obj["change_confidence"],
            "num_frames_observed": obj["num_frames_observed"],
            "track_id": obj["id"],
        })
        num_added += 1

    prior_path.write_text(json.dumps(prior, indent=1))
    live_path.write_text(json.dumps(live, indent=1))
    print(json.dumps({
        "removed": num_removed,
        "added": num_added,
        "added_without_node": num_unbound,
        "prior_metadata": str(prior_path),
        "live_metadata": str(live_path),
    }, indent=1))


if __name__ == "__main__":
    main()
