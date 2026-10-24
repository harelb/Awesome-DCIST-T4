#!/usr/bin/env python3
"""Cross-session / cross-robot ROMAN loop closures for hydra-multi-offline-fusion.

Same procedure as the ``lcd`` step of ``dcist_launch_system/bin/run-offline-fusion`` (ROMAN
submap alignment between every pair of runs, including each run with itself), written
against the current ROMAN API. Writes ``<output>/loop_closures.json`` with robot ids
starting at 1, i.e. fuse with ``--starting-robot-id 1`` and the runs in the same order.

    cross_session_loop_closures.py OUTPUT RUN_DIR [RUN_DIR ...]

Each RUN_DIR is an ADT4 output directory with ``roman/roman_map.pkl`` (or ``1.roman_map.pkl``).
"""

import argparse
import copy
import json
import pathlib

from roman.align.submap_align import submap_align
from roman.params.submap_align_params import SubmapAlignInputOutput, SubmapAlignParams

DLS = pathlib.Path(__file__).absolute().parents[2] / "dcist_launch_system"


def roman_map(run: pathlib.Path) -> pathlib.Path:
    path = run / "roman" / "roman_map.pkl"
    if path.exists():
        return path
    candidates = sorted((run / "roman").glob("*roman_map.pkl"))
    if not candidates:
        raise FileNotFoundError(f"no ROMAN map in {run / 'roman'}")
    return candidates[0]


def stamp_ns(contents, idx):
    return int(contents["seconds"][idx]) * 1_000_000_000 + int(contents["nanoseconds"][idx])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("runs", type=pathlib.Path, nargs="+")
    parser.add_argument("--config", default="default", help="dcist_launch_system config")
    parser.add_argument("--no-single-robot", action="store_true",
                        help="skip loop closures of a run with itself")
    parser.add_argument("--shard", default=None, metavar="K/N",
                        help="only align every N-th pair starting at K (run N of these in "
                        "parallel, then once without --shard to collect)")
    args = parser.parse_args()
    shard = tuple(map(int, args.shard.split("/"))) if args.shard else None

    runs = [run.absolute() for run in args.runs]
    names = [run.name for run in runs]
    output = args.output.absolute()
    (output / "lcd").mkdir(parents=True, exist_ok=True)
    params = SubmapAlignParams.from_yaml(str(DLS / "config" / args.config / "roman_lc.yaml"))

    loop_closures = []
    pair_idx = -1
    for i in range(len(runs)):
        for j in range(i, len(runs)):
            if i == j and args.no_single_robot:
                continue
            pair_idx += 1
            if shard and pair_idx % shard[1] != shard[0]:
                continue
            sm_io = SubmapAlignInputOutput(
                inputs=[str(roman_map(runs[i])), str(roman_map(runs[j]))],
                output_dir=str(output / "lcd"),
                run_name=f"{names[i]}_{names[j]}",
                robot_names=[names[i], names[j]],
            )
            if not pathlib.Path(sm_io.output_lc_json).exists():  # reuse finished pairs
                pair_params = copy.deepcopy(params)
                pair_params.single_robot_lc = i == j
                submap_align(sm_params=pair_params, sm_io=sm_io)
            if shard:
                print(f"{names[i]} -> {names[j]}: aligned", flush=True)
                continue
            with open(sm_io.output_lc_json) as fin:
                contents = json.load(fin)
            print(f"{names[i]} -> {names[j]}: {len(contents)} loop closures")
            for lc in contents:
                # translation / rotation are names[0]_T_names[1]
                loop_closures.append({
                    "robot_from": names.index(lc["names"][1]) + 1,
                    "time_from": stamp_ns(lc, 1),
                    "robot_to": names.index(lc["names"][0]) + 1,
                    "time_to": stamp_ns(lc, 0),
                    "to_p_from": lc["translation"],
                    "to_R_from": dict(zip(lc["rotation_convention"], lc["rotation"])),
                })

    if shard:
        return
    out_path = output / "loop_closures.json"
    out_path.write_text(json.dumps(loop_closures, indent=2))
    print(f"wrote {len(loop_closures)} loop closures to {out_path}")


if __name__ == "__main__":
    main()
