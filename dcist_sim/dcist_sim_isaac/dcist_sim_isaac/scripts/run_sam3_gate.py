"""Run SAM3 text-prompted segmentation over the render_gate.py output frames
and save overlay visualizations. Run in the sam3 venv:

  ~/environments/dcist/sam3/bin/python run_sam3_gate.py \
      --frames <render_gate_out_dir> --out <overlays_out_dir>
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
from PIL import Image

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "agentic_navigation"))

from agentic_navigation.sam3_frontend.config import Sam3FrontendConfig
from agentic_navigation.sam3_frontend.sam3_segmenter import Sam3Segmenter

PROMPTS = ["bag", "cone", "pipe", "tree", "trail"]

COLORS = {
    "bag": (255, 0, 0),
    "cone": (255, 165, 0),
    "pipe": (0, 200, 255),
    "tree": (0, 200, 0),
    "trail": (200, 0, 200),
}


def overlay_detections(image, detections):
    import cv2

    vis = image.copy()
    for det in detections:
        color = COLORS.get(det.label, (255, 255, 255))
        mask = det.mask
        colored = np.zeros_like(vis)
        colored[mask] = color
        vis = cv2.addWeighted(vis, 1.0, colored, 0.5, 0)
        x1, y1, x2, y2 = det.box.astype(int)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        cv2.putText(vis, f"{det.label}:{det.score:.2f}", (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    cfg = Sam3FrontendConfig(device="cuda", conf_threshold=0.4)
    seg = Sam3Segmenter(cfg)

    import cv2

    frames = sorted(glob.glob(os.path.join(args.frames, "*.png")))
    summary = []
    for fpath in frames:
        img = np.array(Image.open(fpath).convert("RGB"))
        dets = seg.segment(img, PROMPTS)
        vis = overlay_detections(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), dets)
        cv2.imwrite(os.path.join(args.out, os.path.basename(fpath)), vis)
        summary.append({
            "file": os.path.basename(fpath),
            "detections": [{"label": d.label, "score": float(d.score)} for d in dets],
        })
        print(os.path.basename(fpath), "->", [d.label for d in dets])

    with open(os.path.join(args.out, "sam3_results.json"), "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
