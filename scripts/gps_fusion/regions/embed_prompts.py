#!/usr/bin/env python3
"""Embed region prompts with the SigLIP2 text tower and write them as a hydra EmbeddingGroup.

The result is a yaml mapping usable as the ``queries`` of a hydra LayerClustering
(``type: from_config``, ``names`` + ``embeddings``), plus an npz with the same vectors.
Each prompt embedding is the renormalized mean over a few templates, matching the
L2-normalized image embeddings of sam3_frontend.EmbeddingFilter (same model, processor
and ``padding="max_length"``). Optionally prints how the prompts score against an image
embedding npz, to pick a score threshold.

With ``--debias`` the direction of the mean image embedding of ``--image-emb`` is projected
out of every prompt embedding (then renormalized). SigLIP2 image-text cosines sit in a narrow
band (~0.05-0.17) with large per-prompt offsets (t . mean_image), so raw scores let one or two
prompts ("parking lot", "sidewalk") win almost everywhere; removing the shared component makes
cos(t', x) ~ t . (x - mean) / |x|, i.e. it compares prompts on how an image differs from the
site average. The image/place features themselves stay raw SigLIP2 embeddings.

Run with the sam3 env: HF_HUB_OFFLINE=1 /home/harel/environments/dcist/sam3/bin/python
"""

import argparse
import os

import numpy as np
import yaml

MODEL = "google/siglip2-so400m-patch16-naflex"
TEMPLATES = ["a photo of a {}.", "a photo of the {}.", "an outdoor photo of a {}.", "a street-level photo of a {}."]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prompts", nargs="+", required=True,
                    help='"name" or "name=phrase a|phrase b" (phrases averaged into one name)')
    ap.add_argument("--out", required=True, help="output stem: <out>.yaml and <out>.npz")
    ap.add_argument("--image-emb", help="npz with emb (N, D) to print score statistics")
    ap.add_argument("--debias", action="store_true", help="project the mean image embedding out")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="siglip2", choices=["siglip2", "clip"],
                    help="clip = OpenCLIP ViT-H-14 laion2b (vlm_models.py); same templates")
    args = ap.parse_args()

    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import vlm_models

    m = vlm_models.load(args.model, args.device)
    vecs = []
    names = [p.split("=", 1)[0] for p in args.prompts]
    for p in args.prompts:
        # "name=phrase a|phrase b": the name's embedding averages all phrases x templates
        phrases = p.split("=", 1)[1].split("|") if "=" in p else [p]
        te = vlm_models.embed_texts(m, [t.format(ph) for ph in phrases for t in TEMPLATES])
        v = te.mean(0)
        vecs.append(v / np.linalg.norm(v))
    del m
    E = np.stack(vecs).astype(np.float32)
    if args.debias:
        I = np.load(args.image_emb)["emb"].astype(np.float64)
        I /= np.linalg.norm(I, axis=1, keepdims=True)
        mu = I.mean(0)
        mu /= np.linalg.norm(mu)
        E = E - (E @ mu)[:, None] * mu[None, :]
        E = (E / np.linalg.norm(E, axis=1, keepdims=True)).astype(np.float32)
        np.savez(args.out + "_image_mean.npz", mean=mu.astype(np.float32))
    np.savez(args.out + ".npz", names=np.array(names), emb=E)
    with open(args.out + ".yaml", "w") as f:
        yaml.safe_dump({"type": "from_config", "names": names,
                        "embeddings": [[round(float(x), 7) for x in v] for v in E]},
                       f, default_flow_style=None, width=1 << 20)
    print("wrote", args.out + ".yaml", E.shape)
    if args.image_emb:
        I = np.load(args.image_emb)["emb"].astype(np.float32)
        I /= np.linalg.norm(I, axis=1, keepdims=True)
        S = I @ E.T
        print("cosine percentiles (5/25/50/75/95) per prompt; top-1 share")
        top = S.argmax(1)
        for j, p in enumerate(names):
            q = np.percentile(S[:, j], [5, 25, 50, 75, 95])
            print(f"  {p:>14s}: " + " ".join(f"{x:.3f}" for x in q) + f"  top1 {np.mean(top == j):.2f}")
        mx = S.max(1)
        print("max-over-prompts percentiles (5/25/50/75/95):",
              " ".join(f"{x:.3f}" for x in np.percentile(mx, [5, 25, 50, 75, 95])))


if __name__ == "__main__":
    main()
