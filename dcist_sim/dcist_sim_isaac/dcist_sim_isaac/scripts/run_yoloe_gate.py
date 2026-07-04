"""Run YOLOE (same weights/classes as the real spot_executor detector) over
the render_gate.py output frames and save overlay visualizations + a hits
JSON. Run in the spark_env venv (has ultralytics):

  ~/environments/dcist/spark_env/bin/python run_yoloe_gate.py \
      --frames <render_gate_out_dir> --out <overlays_out_dir> \
      --weights ~/dcist_ws/weights/yoloe-26m-seg.pt
"""
import argparse
import glob
import json
import os

import cv2
from ultralytics import YOLOE

CLASSES = ["", "bag", "cone", "pipe"]  # matches spot_tools/.../detection_utils.py:34


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--conf", type=float, default=0.15)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    model = YOLOE(args.weights)
    model.set_classes(CLASSES)

    frames = sorted(glob.glob(os.path.join(args.frames, "*.png")))
    results_summary = []
    for fpath in frames:
        img = cv2.imread(fpath)
        results = model.predict(img, conf=args.conf, verbose=False)
        dets = []
        for r in results:
            names = r.names
            for box in r.boxes:
                cls_id = int(box.cls[0])
                dets.append(
                    {
                        "class": names[cls_id],
                        "conf": float(box.conf[0]),
                        "xyxy": [float(v) for v in box.xyxy[0].tolist()],
                    }
                )
            overlay = r.plot()
            cv2.imwrite(os.path.join(args.out, os.path.basename(fpath)), overlay)
        results_summary.append({"file": os.path.basename(fpath), "detections": dets})
        print(os.path.basename(fpath), "->", [d["class"] for d in dets])

    with open(os.path.join(args.out, "yoloe_results.json"), "w") as f:
        json.dump(results_summary, f, indent=2)


if __name__ == "__main__":
    main()
