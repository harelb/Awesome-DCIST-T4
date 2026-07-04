"""Reduce yoloe_results.json / sam3_results.json (produced by
run_yoloe_gate.py / run_sam3_gate.py) to the per-image hit table and
headline hit-rate aggregates reported in
`dcist_sim/docs/render_gate_report.md`.

Hit definition (this is exactly what the committed report's numbers
reflect -- stated explicitly so the scoring is auditable/reproducible):

  For a given frame and target class, a "hit" is: at least one detection
  in that frame's detections list whose reported label equals the target
  class name, at the confidence threshold already baked into the results
  file by the gate script that produced it (YOLOE conf=0.15, SAM3
  conf=0.4 -- see run_yoloe_gate.py / run_sam3_gate.py). This is a
  presence/absence check per (frame, class); it does NOT check
  bounding-box location/IoU against a ground truth, and it does NOT
  penalize a detector for an extra or mislabeled box elsewhere in the
  frame. E.g. in frame_08_midday_az288_d2.5.png, YOLOE emits a real
  "cone" box (conf 0.83) on the actual cone AND a second box labeled
  "cone" (conf 0.20) that is actually sitting on the pipe -- that frame
  still counts as exactly one "cone" hit (there IS a box labeled cone)
  and zero "pipe" hits (no box is ever labeled "pipe" in this frame).
  This is the vocabulary-confusion confound noted in the report.

Usage:
  python3 score_gate.py \\
      --yoloe dcist_sim/docs/render_gate_data/yoloe_results.json \\
      --sam3 dcist_sim/docs/render_gate_data/sam3_results.json \\
      [--markdown]

Run with either python3 (stdlib only, no torch/ultralytics needed) since
it only reads the already-produced JSON result files -- it does not
re-run any model.
"""
import argparse
import json

CLASSES = ["cone", "bag", "pipe"]


def load_hits(path, label_key):
    """Return {filename: {class: bool}} for whether that class appears
    anywhere in that frame's detections list."""
    with open(path) as f:
        data = json.load(f)
    per_frame = {}
    for entry in data:
        labels = {d[label_key] for d in entry["detections"]}
        per_frame[entry["file"]] = {c: (c in labels) for c in CLASSES}
    return per_frame


def build_rows(yoloe_hits, sam3_hits):
    files = sorted(set(yoloe_hits) | set(sam3_hits))
    rows = []
    for f in files:
        row = {"file": f}
        for c in CLASSES:
            row[f"yoloe_{c}"] = yoloe_hits.get(f, {}).get(c, False)
            row[f"sam3_{c}"] = sam3_hits.get(f, {}).get(c, False)
        rows.append(row)
    return rows


def aggregate(rows):
    n = len(rows)
    totals = {}
    for detector in ("yoloe", "sam3"):
        for c in CLASSES:
            totals[(detector, c)] = sum(1 for r in rows if r[f"{detector}_{c}"])
    return totals, n


def print_table(rows):
    header = f"{'file':<32}" + "".join(f"{'Y-' + c:>8}" for c in CLASSES) + "".join(
        f"{'S-' + c:>8}" for c in CLASSES
    )
    print(header)
    for row in rows:
        cells = "".join(f"{'X' if row[f'yoloe_{c}'] else '.':>8}" for c in CLASSES)
        cells += "".join(f"{'X' if row[f'sam3_{c}'] else '.':>8}" for c in CLASSES)
        print(f"{row['file']:<32}{cells}")


def print_aggregate(totals, n):
    print()
    for detector in ("yoloe", "sam3"):
        for c in CLASSES:
            hits = totals[(detector, c)]
            print(f"{detector:<8} {c:<6} {hits}/{n} = {100 * hits / n:.0f}%")


def print_markdown(rows):
    print()
    print(
        "| # | file | YOLOE cone | YOLOE bag | YOLOE pipe | "
        "SAM3 cone | SAM3 bag | SAM3 pipe |"
    )
    print("|---|---|---|---|---|---|---|---|")
    for i, row in enumerate(rows):
        def mark(v):
            return "hit" if v else "-"

        print(
            f"| {i} | {row['file']} | {mark(row['yoloe_cone'])} | "
            f"{mark(row['yoloe_bag'])} | {mark(row['yoloe_pipe'])} | "
            f"{mark(row['sam3_cone'])} | {mark(row['sam3_bag'])} | "
            f"{mark(row['sam3_pipe'])} |"
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yoloe", required=True, help="path to yoloe_results.json")
    ap.add_argument("--sam3", required=True, help="path to sam3_results.json")
    ap.add_argument(
        "--markdown", action="store_true", help="also print a markdown table"
    )
    args = ap.parse_args()

    yoloe_hits = load_hits(args.yoloe, "class")
    sam3_hits = load_hits(args.sam3, "label")

    rows = build_rows(yoloe_hits, sam3_hits)
    totals, n = aggregate(rows)

    print_table(rows)
    print_aggregate(totals, n)
    if args.markdown:
        print_markdown(rows)


if __name__ == "__main__":
    main()
