#!/usr/bin/env python3
"""Give the traversability places of per-robot Hydra runs open-vocabulary features.

For every session, each MESH_PLACES node of ``<run>/hydra/frontend/dsg_with_mesh.json``
(the graph hydra-multi-offline-fusion reads) gets ``semantic_feature`` = the normalized,
visibility-weighted mean of the SigLIP2 image embeddings of the agent keyframes that
actually observed it: the place centre and 8 points inside its region-growing boundary are
projected into the keyframe (pose = frontend agent node it is anchored to, then
body_T_sensor) and a sample counts as seen when it is in the image, within ``--max-range``
and not occluded in the keyframe's depth image (invalid depth is accepted only up to
``--max-range-no-depth``). The weight of a keyframe is its fraction of visible samples.
Places no keyframe observed get the mean of their observed graph neighbours (a few rounds),
so the place graph the clustering walks stays connected; both counts are reported.

The patched copy is written to ``<out>/<session>/hydra/frontend/dsg_with_mesh.json``; the
runs are never modified. Keyframe embeddings come from a ``paths``/``emb`` npz keyed
``f<frame_id>`` with frame ids of the fused image store (``robot_index << 48 | frame_id``),
e.g. console/fused_final/image_store/cache/siglip2_emb.npz.
"""

import argparse
import json
import pathlib
import sys
import time

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

from hydra_image_store import FrameKind, ImageStore
from hydra_image_store.codec import pose_matrix, quat_to_rot

MESH_PLACES = (3, 1)


def load_sessions(path):
    out = []
    for line in pathlib.Path(path).read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            out.append(line.split()[0])
    return out


def place_samples(attrs, n_ring=8, frac=0.6):
    """Place centre plus ``n_ring`` points at ``frac`` of the boundary radius."""
    p = np.asarray(attrs["position"], float)
    radii = attrs.get("radii") or []
    pts = [p]
    if radii:
        nr = len(radii)
        for k in range(n_ring):
            j = int(round(k * nr / n_ring)) % nr
            a = 2.0 * np.pi * j / nr
            r = frac * float(radii[j])
            pts.append(p + np.array([r * np.cos(a), r * np.sin(a), 0.0]))
    return np.array(pts)


def visible_fraction(pts_w, T_ws, K, w, h, depth, args):
    R, t = T_ws[:3, :3], T_ws[:3, 3]
    pc = (pts_w - t) @ R  # world -> camera (R^T (p - t))
    z = pc[:, 2]
    ok = z > 0.3
    rng = np.linalg.norm(pc, axis=1)
    ok &= rng < args.max_range
    if not ok.any():
        return 0.0
    u = K[0, 0] * pc[:, 0] / np.maximum(z, 1e-6) + K[0, 2]
    v = K[1, 1] * pc[:, 1] / np.maximum(z, 1e-6) + K[1, 2]
    ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
    seen = 0
    for i in np.nonzero(ok)[0]:
        if depth is None:
            seen += rng[i] < args.max_range_no_depth
            continue
        dh, dw = depth.shape
        ui = int(u[i] * dw / w)
        vi = int(v[i] * dh / h)
        win = depth[max(vi - 2, 0):vi + 3, max(ui - 2, 0):ui + 3]
        win = win[win > 0]
        if win.size == 0:
            seen += rng[i] < args.max_range_no_depth
        elif np.median(win) >= z[i] - max(args.depth_tol, args.depth_tol_rel * z[i]):
            seen += 1
    return seen / len(pts_w)


def process(session, ridx, args, emb):
    t0 = time.time()
    src = pathlib.Path(args.runs) / session / "hydra/frontend/dsg_with_mesh.json"
    data = json.loads(src.read_text())
    nodes = data["nodes"]
    agents = {}
    places = []
    for n in nodes:
        a = n["attributes"]
        if "world_R_body" in a and n["layer"] == 2:
            q = a["world_R_body"]
            agents[int(n["id"])] = pose_matrix(a["position"], quat_to_rot(q["w"], q["x"], q["y"], q["z"]))
        elif (n["layer"], n.get("partition", 0)) == MESH_PLACES:
            places.append(n)
    centres = np.array([p["attributes"]["position"] for p in places])
    samples = [place_samples(p["attributes"]) for p in places]
    tree = cKDTree(centres)

    store = ImageStore.open(pathlib.Path(args.runs) / session / "image_store")
    calibs = {c.sensor_id: c for c in store.sensors()}
    frames = store.frames(kind=FrameKind.KEYFRAME)
    acc = np.zeros((len(places), emb["dim"]))
    wsum = np.zeros(len(places))
    nobs = np.zeros(len(places), int)
    n_used = n_noemb = 0
    src_count = {"anchor": 0, "odom": 0}
    for f in frames:
        key = f"f{(ridx << 48) | f.frame_id}"
        e = emb["map"].get(key)
        if e is None:
            n_noemb += 1
            continue
        c = calibs[f.sensor_id]
        world_T_body, srcname = store.body_pose(f, agents.get)
        src_count[srcname] += 1
        T_ws = world_T_body @ c.body_T_sensor
        depth = None
        if f.depth_path is not None and pathlib.Path(f.depth_path).exists():
            depth = np.asarray(Image.open(f.depth_path), dtype=np.float32) * c.depth_scale
        cand = tree.query_ball_point(T_ws[:3, 3], args.max_range + 5.0)
        K = c.K
        used = False
        for i in cand:
            frac = visible_fraction(samples[i], T_ws, K, c.width, c.height, depth, args)
            if frac <= 0:
                continue
            if args.range_scale > 0:  # favour keyframes close to the place (more local features)
                frac *= np.exp(-np.linalg.norm(centres[i] - T_ws[:3, 3]) / args.range_scale)
            acc[i] += frac * e
            wsum[i] += frac
            nobs[i] += 1
            used = True
        n_used += used

    feats = np.zeros_like(acc)
    observed = wsum > 0
    feats[observed] = acc[observed] / np.linalg.norm(acc[observed], axis=1, keepdims=True)

    # fill unobserved places from observed graph neighbours
    index = {int(p["id"]): i for i, p in enumerate(places)}
    nbrs = [[] for _ in places]
    for ed in data["edges"]:
        a, b = index.get(int(ed["source"])), index.get(int(ed["target"]))
        if a is not None and b is not None:
            nbrs[a].append(b)
            nbrs[b].append(a)
    has = observed.copy()
    filled = 0
    for _ in range(args.fill_rounds):
        new = []
        for i in np.nonzero(~has)[0]:
            ns = [j for j in nbrs[i] if has[j]]
            if ns:
                m = feats[ns].mean(axis=0)
                new.append((i, m / np.linalg.norm(m)))
        if not new:
            break
        for i, m in new:
            feats[i] = m
            has[i] = True
        filled += len(new)

    for i, p in enumerate(places):
        if has[i]:
            v = feats[i].astype(np.float32)
            # spark_dsg >= 1.1.7 stores semantic_feature (Eigen::VectorXf) as a plain list
            p["attributes"]["semantic_feature"] = [float(x) for x in v]

    out_dir = pathlib.Path(args.out) / session / "hydra/frontend"
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp = out_dir / "dsg_with_mesh.json.tmp"
    with open(tmp, "w") as fout:
        json.dump(data, fout, separators=(",", ":"))
    tmp.replace(out_dir / "dsg_with_mesh.json")
    stats = {
        "session": session,
        "robot_index": ridx,
        "places": len(places),
        "observed": int(observed.sum()),
        "filled_from_neighbours": int(filled),
        "no_feature": int((~has).sum()),
        "keyframes": len(frames),
        "keyframes_used": n_used,
        "keyframes_without_embedding": n_noemb,
        "pose_source": src_count,
        "obs_per_observed_place_median": float(np.median(nobs[observed])) if observed.any() else 0.0,
        "seconds": round(time.time() - t0, 1),
    }
    print(json.dumps(stats), flush=True)
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default="/data/dcist/penn_2025_meridian/runs")
    ap.add_argument("--sessions", default="/data/dcist/penn_2025_meridian/sessions.txt",
                    help="session order = robot index in the fused image store")
    ap.add_argument("--emb", default="/data/dcist/penn_2025_meridian/console/fused_final/image_store/cache/siglip2_emb.npz")
    ap.add_argument("--out", required=True, help="inputs dir, e.g. fusion/inputs_regions")
    ap.add_argument("--only", nargs="*", help="subset of sessions")
    ap.add_argument("--max-range", type=float, default=12.0)
    ap.add_argument("--max-range-no-depth", type=float, default=5.0)
    ap.add_argument("--depth-tol", type=float, default=0.5)
    ap.add_argument("--depth-tol-rel", type=float, default=0.1)
    ap.add_argument("--fill-rounds", type=int, default=3)
    ap.add_argument("--range-scale", type=float, default=0.0,
                    help="if > 0, weight keyframes by exp(-camera-to-place distance / scale)")
    args = ap.parse_args()

    d = np.load(args.emb)
    paths, e = d["paths"], d["emb"].astype(np.float64)
    e /= np.linalg.norm(e, axis=1, keepdims=True)
    emb = {"map": {str(p): e[i] for i, p in enumerate(paths)}, "dim": e.shape[1]}
    sessions = load_sessions(args.sessions)
    allstats = []
    for ridx, s in enumerate(sessions):
        if args.only and s not in args.only:
            continue
        allstats.append(process(s, ridx, args, emb))
    pathlib.Path(args.out).mkdir(parents=True, exist_ok=True)
    (pathlib.Path(args.out) / "place_features_stats.json").write_text(json.dumps(
        {"args": vars(args), "sessions": allstats}, indent=1))


if __name__ == "__main__":
    sys.exit(main())
