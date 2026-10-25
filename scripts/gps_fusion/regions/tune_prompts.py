#!/usr/bin/env python3
"""Compose prompt sets from a phrase bank, debias, cluster with aib_replica and plot (tuning only).

spec json: {"name": ["phrase", ...], ...}; writes <out>.png and <out>_queries.npz.
"""
import argparse, collections, json, pathlib, subprocess, sys
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument("--spec", required=True); ap.add_argument("--bank", required=True)
ap.add_argument("--image-mean", required=True); ap.add_argument("--feats", required=True)
ap.add_argument("--origin", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--score-threshold", type=float, default=0.01); ap.add_argument("--max-delta", type=float, default=0.005)
ap.add_argument("--top-k", type=int, default=2)
a = ap.parse_args()
spec = json.loads(pathlib.Path(a.spec).read_text())
b = np.load(a.bank); bank = {str(k): v for k, v in zip(b["phrases"], b["emb"])}
E = []
for name, phrases in spec.items():
    v = np.mean([bank[p] for p in phrases], axis=0); E.append(v / np.linalg.norm(v))
E = np.stack(E); mu = np.load(a.image_mean)["mean"].astype(np.float64); mu /= np.linalg.norm(mu)
E = E - (E @ mu)[:, None] * mu[None]; E /= np.linalg.norm(E, axis=1, keepdims=True)
q = a.out + "_queries.npz"; np.savez(q, names=np.array(list(spec)), emb=E.astype(np.float32))
here = pathlib.Path(__file__).parent
subprocess.run([sys.executable, str(here / "plot_regions.py"), "--tune", "--feats", a.feats, "--queries", q,
                "--score-threshold", str(a.score_threshold), "--max-delta", str(a.max_delta), "--top-k", str(a.top_k), "--origin", a.origin,
                "--title", pathlib.Path(a.out).name, "--out", a.out + ".png"], check=True)
