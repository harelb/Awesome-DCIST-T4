#!/usr/bin/env python3
"""Renumber a fused image store's console caches to a new session order.

Fused frame ids are ``session_index << 48 | frame_id``; the SAM3 feature cache
(``sam3_feats_*/f<id>-<sha1("f<id>")[:8]>.safetensors``) and the SigLIP2 embeddings
(``siglip2_emb.npz``, keys ``f<id>``) are keyed by them. When sessions are dropped or
reordered, entries are renamed to the new index; entries of dropped sessions are deleted
(they would otherwise be returned for another session's frames).

    rekey_caches.py OLD_SESSIONS NEW_SESSIONS CACHE_DIR [--dry-run]
"""
import argparse
import glob
import hashlib
import os

import numpy as np


def names(path):
    return [l.split()[0] for l in open(path) if l.strip() and not l.startswith("#")]


def safe_name(key):
    return f"{key}-{hashlib.sha1(key.encode()).hexdigest()[:8]}.safetensors"


ap = argparse.ArgumentParser()
ap.add_argument("old")
ap.add_argument("new")
ap.add_argument("cache")
ap.add_argument("--dry-run", action="store_true")
a = ap.parse_args()
old, new = names(a.old), names(a.new)
idx = {n: i for i, n in enumerate(new)}
remap = {i: idx.get(n) for i, n in enumerate(old)}  # None = dropped
print("index map:", {old[i]: (i, j) for i, j in remap.items()})


def rekey(fid):
    j = remap.get(fid >> 48)
    return None if j is None else (j << 48) | (fid & ((1 << 48) - 1))


for d in glob.glob(os.path.join(a.cache, "sam3_feats_*")):
    files = [f for f in os.listdir(d) if f.startswith("f") and f.endswith(".safetensors")]
    moves, drops = [], []
    for f in files:
        new_id = rekey(int(f[1:].split("-")[0]))
        if new_id is None:
            drops.append(f)
        else:
            moves.append((f, safe_name(f"f{new_id}")))
    # two passes through temporary names: a new name can equal another entry's old name
    if not a.dry_run:
        for f in drops:
            os.remove(os.path.join(d, f))
        for i, (f, _) in enumerate(moves):
            os.rename(os.path.join(d, f), os.path.join(d, f"tmp{i}.rekey"))
        for i, (_, g) in enumerate(moves):
            os.rename(os.path.join(d, f"tmp{i}.rekey"), os.path.join(d, g))
    print(f"{d}: {len(moves)} renamed, {len(drops)} dropped")

emb = os.path.join(a.cache, "siglip2_emb.npz")
if os.path.exists(emb):
    z = np.load(emb)
    keys, e = [str(k) for k in z["paths"]], z["emb"]
    out = [(f"f{rekey(int(k[1:]))}", v) for k, v in zip(keys, e) if rekey(int(k[1:])) is not None]
    if not a.dry_run:
        tmp = emb + ".tmp.npz"
        np.savez(tmp, paths=np.array([k for k, _ in out]), emb=np.stack([v for _, v in out]).astype(np.float32))
        os.replace(tmp, emb)
    print(f"{emb}: {len(out)} of {len(keys)} embeddings kept")
