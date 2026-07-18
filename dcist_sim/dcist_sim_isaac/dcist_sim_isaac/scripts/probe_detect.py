"""YOLOE detection pass over probe_environments.py renders (spec §3.1/§3.5).

Run in the spark_env venv (has ultralytics), one probe out-dir at a time:
  ~/environments/dcist/spark_env/bin/python \
      dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_detect.py \
      --probe-out /tmp/probe --weights ~/dcist_ws/weights/yoloe-26m-seg.pt
Writes <probe-out>/<candidate>/hits.json + overlay PNGs, and appends a
per-class hit-rate table to <probe-out>/report.md. The class list below is
the candidate vocabulary for the isaac_sim instance_seg overlay (Task 12);
keep "" at index 0 (matches run_yoloe_gate.py / detection_utils.py).
"""
import argparse
import glob
import json
import os

import cv2
from ultralytics import YOLOE

CLASSES = ["", "pallet", "forklift", "shelf", "box", "cone", "bag", "pipe",
           "barrel", "ladder", "fire extinguisher", "chair", "table"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-out", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--conf", type=float, default=0.02)  # isaac_sim overlay value
    args = ap.parse_args()

    model = YOLOE(args.weights)
    model.set_classes(CLASSES)

    table = ["", "## YOLOE hits (conf>=%.2f)" % args.conf, "",
             "| candidate | " + " | ".join(c for c in CLASSES if c) + " |",
             "|" + "---|" * len([c for c in CLASSES if c]) + "---|"]
    for cand_dir in sorted(glob.glob(os.path.join(args.probe_out, "*", ""))):
        name = os.path.basename(cand_dir.rstrip("/"))
        frames = sorted(glob.glob(os.path.join(cand_dir, "frame_*.png")))
        if not frames:
            continue
        hits = {c: 0 for c in CLASSES if c}
        per_frame = []
        for fpath in frames:
            img = cv2.imread(fpath)
            dets = []
            for r in model.predict(img, conf=args.conf, verbose=False):
                for b in r.boxes:
                    cls = r.names[int(b.cls)]
                    conf = float(b.conf)
                    dets.append({"class": cls, "conf": round(conf, 3)})
                    if cls in hits:
                        hits[cls] += 1
                ov = r.plot()
                cv2.imwrite(fpath.replace("frame_", "overlay_"), ov)
            per_frame.append({"frame": os.path.basename(fpath), "dets": dets})
        with open(os.path.join(cand_dir, "hits.json"), "w") as f:
            json.dump({"hits": hits, "frames": per_frame}, f, indent=2)
        table.append("| " + name + " | "
                     + " | ".join(str(hits[c]) for c in CLASSES if c) + " |")

    with open(os.path.join(args.probe_out, "report.md"), "a") as f:
        f.write("\n".join(table) + "\n")


if __name__ == "__main__":
    main()
