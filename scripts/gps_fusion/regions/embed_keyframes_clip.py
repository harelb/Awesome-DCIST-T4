#!/usr/bin/env python3
"""CLIP embeddings of every keyframe of the per-session image stores (place_features.py --emb).

Writes ``<out>.npz`` (``paths`` = ``f<robot_index << 48 | frame_id>``, ``emb`` = mean of the
left/centre/right square-crop embeddings, see vlm_models.clip_crops) and ``<out>_center.npz``
(centre crop only), the same key scheme as the console's siglip2_emb.npz.

Run: PYTHONPATH=<super>/hydra/hydra_image_store/src HF_HUB_OFFLINE=1 \
     /home/harel/environments/dcist/sam3/bin/python embed_keyframes_clip.py --out ...
"""

import argparse
import pathlib
import sys
import time

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import vlm_models  # noqa: E402
from hydra_image_store import FrameKind, ImageStore  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default="/data/dcist/penn_2025_meridian/runs")
    ap.add_argument("--sessions", default="/data/dcist/penn_2025_meridian/sessions.txt")
    ap.add_argument("--out", required=True)
    ap.add_argument("--batch", type=int, default=24, help="crops per forward pass")
    ap.add_argument("--device", default="cuda")
    a = ap.parse_args()
    sessions = [l.split()[0] for l in pathlib.Path(a.sessions).read_text().splitlines()
                if l.strip() and not l.startswith("#")]
    items = []
    for r, s in enumerate(sessions):
        st = ImageStore.open(pathlib.Path(a.runs) / s / "image_store")
        items += [(f"f{(r << 48) | f.frame_id}", f.rgb_path) for f in st.frames(kind=FrameKind.KEYFRAME)]
    m = vlm_models.load("clip", a.device)
    keys, E3, EC = [], [], []
    t0 = time.time()
    step = 32
    for k in range(0, len(items), step):
        chunk = items[k:k + step]
        e3, ec = vlm_models.embed_images_clip(m, [Image.open(p) for _, p in chunk], a.batch)
        keys += [key for key, _ in chunk]
        E3.append(e3)
        EC.append(ec)
        if k % 640 == 0:
            print(f"{k + len(chunk)}/{len(items)} {time.time() - t0:.0f}s", flush=True)
    np.savez(a.out + ".npz", paths=np.array(keys), emb=np.concatenate(E3),
             model=vlm_models.MODELS["clip"], crops="left|centre|right mean")
    np.savez(a.out + "_center.npz", paths=np.array(keys), emb=np.concatenate(EC),
             model=vlm_models.MODELS["clip"], crops="centre")
    print("wrote", a.out, len(keys), f"{time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
