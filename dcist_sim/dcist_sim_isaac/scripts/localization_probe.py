#!/usr/bin/env python3
"""Localization-probe acceptance harness (Perception Depth-Mode Filter, Task 3).

Compares the live DSG's object-node positions against a scenario's ground
-truth object spawn poses and asserts localization error stays under a bar.
THE ROBOT IS PARKED for this probe -- it never publishes nav/pick goals;
bringing up the sim + stack (and driving whatever tour populated the object
layer) is the runner's job (see docs/sim_runbook.md, build_map.py). Run
recipe (spark_env + ROS + workspace sourced; PYTHONPATH contract copied
verbatim from build_map.py's docstring):

    source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
    PYTHONPATH=dcist_sim/dcist_sim_isaac \
    ~/environments/dcist/spark_env/bin/python \
        dcist_sim/dcist_sim_isaac/scripts/localization_probe.py \
        --scenario dcist_sim/scenarios/warehouse_tour.yaml \
        --robot hilbert --bar 0.3 --settle-s 90 --json out.json

Exit: 0 iff every scenario GT object was matched to a same-label DSG object
node AND the worst per-object error is < --bar; 1 otherwise (unmatched
object, error over the bar, or the object layer never stabilized).

## Label-matching rule

Scenario objects carry a string `label` (e.g. "bag", "cone") and an
`id: <label>_<n>` naming convention (dcist_sim_isaac/scenario.py's
ObjectSpec; every scenario YAML under dcist_sim/scenarios/ follows this).
Live DSG object nodes only carry an integer `attributes.semantic_label` --
the instance_seg labelspace id khronos's active-window object pipeline
stamps from the GT/FastSAM label image (gt_semantics.py). This deployment
runs objects through khronos active_window, NOT hydra's (dead-for-us)
MeshSegmenter path: isaac_sim's hydra.yaml sets
`frontend.enable_mesh_objects: false`, so hydra/src/frontend/graph_builder.cpp
never constructs a MeshSegmenter at all. The live unpacking is
`khronos/khronos/src/active_window/object_detection/instance_forwarding.cpp`
(~line 202: `category_id = (id >> 16) & 0xFFFF`, gated on
`config.instance_id` -- isaac_sim's hydra.yaml active_window
`object_detector.instance_id: true`), which then flows through
`mesh_object_extractor.cpp:106` / `active_window.cpp`'s
`createObjectAttributes` (`object->semantic_label =
track.semantics->category_id`). So, same as before: `semantic_label` is
already the bare category id (e.g. 3 for "bag"), never the packed
`(category << 16) | instance` value -- only the file that computes it was
misidentified in an earlier revision of this docstring (it cited hydra's
mesh_segmenter.cpp/mesh_delta_clustering.cpp, which is compiled out here).

This script reverses `gt_semantics.LABELSPACE_NAME_TO_ID` (id -> name) to
turn a node's `semantic_label` back into a class-name string, builds
`(name, x, y)` tuples from the object layer, and hands them to
`localization_probe_lib.match_objects` alongside `{object_id: (x, y)}` GT
built straight from the scenario -- `match_objects` derives each GT
object's required label from its `object_id` (the `<label>_<n>` prefix;
see that module's docstring for the exact rule). Classes absent from the
labelspace (e.g. "pipe" -- see gt_semantics.py) can never be matched by
this probe: their GT objects always come back unmatched (error_m None),
which correctly fails the bar. This is a KNOWN GAP (bag/pipe detection is
an open follow-up per the mapping-harness runbook), not a probe bug.

## Object-layer stability wait

Flow: subscribe to the robot's backend DSG, wait for the object layer's
node COUNT to be non-empty and unchanged for 15 s straight (--settle-s is
the overall deadline on this wait), snapshot (label, x, y) per node,
match vs the scenario GT, print a PASS/FAIL table, optionally write
--json, and exit accordingly.

## Clock basis

--settle-s (and the 15 s stability window) are measured in the SAME clock
e2e_smoke.py uses for its stage deadlines: sim time when the stack runs
`use_sim_time` with a live /clock, else wall time (spec §2, "sim time
absorbs slowdown"). `_now_s`, `wait_until`, and `configure_clock_basis`
below are COPIED FROM e2e_smoke.py (dcist_sim/dcist_sim_isaac/scripts/
e2e_smoke.py), attributed at each function, rather than imported -- this
`scripts/` directory has no `__init__.py` (it holds standalone driver
scripts run directly with the interpreter, not an importable package), so
a plain `from e2e_smoke import ...` isn't available without a path hack.
"""
import argparse
import json
import os
import sys
import threading
import time

import rclpy
import spark_dsg
from hydra_ros import DsgSubscriber
from rclpy.node import Node
from rclpy.parameter import Parameter

from dcist_sim_isaac.gt_semantics import LABELSPACE_NAME_TO_ID
from dcist_sim_isaac.localization_probe_lib import (
    _label_from_object_id,
    match_objects,
    summarize,
)
from dcist_sim_isaac.scenario import load_scenario

STABILITY_S = 15.0  # object-node count must be unchanged this long to "settle"
DEFAULT_MAX_MATCH_M = 5.0  # search radius for candidate matches (not the bar)

# id -> name (e.g. 3 -> "bag"); reverses gt_semantics.LABELSPACE_NAME_TO_ID,
# the single source of truth for the instance_seg labelspace (kept in
# lockstep with dcist_launch_system/labelspaces/instance_seg.yaml by
# test_gt_semantics.py). id 0 ("ignore") covers background/unlabeled nodes,
# which then never match a real scenario label -- see module docstring.
ID_TO_LABEL = {v: k for k, v in LABELSPACE_NAME_TO_ID.items()}

# Detected-label synonyms (Task 4, live-verified). The closed-set YOLOE
# instance_seg frontend classifies the field duffel bag as "box" (id 17), NOT
# "bag" (id 3) -- documented in docs/sim_runbook.md §5 and confirmed live: the
# duffel's DSG object node carries semantic_label 17. Scenarios name that object
# `bag_<n>`, so match_objects derives the required label "bag" from the id and
# would never match the "box"-labeled node (the object IS detected and IS what
# the depth filter targets -- it is only mis-labeled). Canonicalize the detected
# label through this synonym table so a scenario "bag" object matches its "box"
# detection. This is SAFE across every scenario under dcist_sim/scenarios/: none
# author a `box_<n>` object (all object ids are bag_/cone_/pipe_), so it can
# never mis-canonicalize a genuine "box" GT object. A future scenario that adds
# real boxes AND bags would instead need a symmetric alias-group match in
# localization_probe_lib.match_objects.
LABEL_ALIASES = {"box": "bag"}


class LocalizationProbe(Node):
    def __init__(self, robot):
        super().__init__("localization_probe")
        self.robot = robot
        self.dsg = None
        self._lock = threading.Lock()
        DsgSubscriber(self, f"/{robot}/hydra/backend/dsg", self._dsg_cb)

    def _dsg_cb(self, header, dsg):
        with self._lock:
            self.dsg = dsg

    def object_nodes(self):
        """`(display_label, x, y, raw_label)` per live DSG object node.

        ``raw_label`` is the bare labelspace name for the node's
        ``semantic_label`` (e.g. ``"box"`` for the duffel); ``display_label``
        is that name after LABEL_ALIASES canonicalization (e.g. ``"bag"``).
        Matching uses ``display_label`` (element 0); ``raw_label`` (element 3)
        is carried through so match rows can report WHICH raw detection an
        object matched. Both may be None for an unknown ``semantic_label``.
        """
        with self._lock:
            g = self.dsg
        if g is None:
            return []
        out = []
        for n in g.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes:
            p = n.attributes.position
            raw_label = ID_TO_LABEL.get(n.attributes.semantic_label)
            display_label = LABEL_ALIASES.get(raw_label, raw_label)
            out.append((display_label, p[0], p[1], raw_label))
        return out


# --- copied from e2e_smoke.py (dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py) ---
# Same sim-time-aware deadline basis as that harness's stage windows; see this
# script's module docstring ("Clock basis") for why these are copied, not
# imported.


def _now_s(node):
    """Current time in the harness's active deadline basis, in seconds.

    Returns ROS time when the node's clock is sim-driven (``use_sim_time``
    set True and a live ``/clock`` -- ``ros_time_is_active``), else wall
    time. Passing ``node=None`` forces wall time.
    """
    if node is not None and node.get_clock().ros_time_is_active:
        return node.get_clock().now().nanoseconds / 1e9
    return time.time()


def wait_until(node, pred, timeout, poll=0.5):
    """Poll ``pred`` until truthy or ``timeout`` elapses in the node's clock basis."""
    end = _now_s(node) + timeout
    while _now_s(node) < end:
        v = pred()
        if v:
            return v
        time.sleep(poll)
    return pred()


def configure_clock_basis(node, sim_time_override):
    """Pick + install the deadline clock basis; return (is_sim, rtf_or_none).

    ``sim_time_override``: True forces sim, False forces wall, None
    auto-detects from a live ``/clock`` publisher.
    """
    if sim_time_override is None:
        detect_end = time.time() + 3.0
        is_sim = False
        while time.time() < detect_end:
            if node.count_publishers("/clock") > 0:
                is_sim = True
                break
            time.sleep(0.1)
    else:
        is_sim = bool(sim_time_override)

    if not is_sim:
        return False, None

    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    live = wait_until(
        None, lambda: node.get_clock().now().nanoseconds > 0, 10.0, poll=0.1
    )
    if not live:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, False)])
        print(
            "[loc_probe] WARN: sim time requested but /clock never started; "
            "falling back to WALL-clock deadlines"
        )
        return False, None

    w0, s0 = time.time(), node.get_clock().now().nanoseconds / 1e9
    time.sleep(2.0)
    w1, s1 = time.time(), node.get_clock().now().nanoseconds / 1e9
    wall_dt = w1 - w0
    rtf = (s1 - s0) / wall_dt if wall_dt > 0 else float("nan")
    return True, rtf


# --- end copied-from-e2e_smoke.py block ---


def wait_for_stable_objects(node, settle_s, stability_s=STABILITY_S, poll=0.5):
    """Wait for the object layer's node count to hold steady for ``stability_s``.

    Polls ``node.object_nodes()`` until its length has been unchanged for
    ``stability_s`` seconds straight (and is > 0), or ``settle_s`` elapses
    first -- both measured in the node's active clock basis (see
    ``_now_s``). Returns ``(nodes, settled)``: ``nodes`` is the last
    snapshot taken; ``settled`` is False if the deadline hit first.
    """
    end = _now_s(node) + settle_s
    last_count = None
    last_change = _now_s(node)
    nodes = []
    while _now_s(node) < end:
        nodes = node.object_nodes()
        count = len(nodes)
        if count != last_count:
            last_count = count
            last_change = _now_s(node)
        elif count > 0 and (_now_s(node) - last_change) >= stability_s:
            return nodes, True
        time.sleep(poll)
    return nodes, False


def print_table(rows, bar_m):
    print(f"[loc_probe] {'object_id':<20} {'error_m':>10}  status  matched_as")
    for r in sorted(rows, key=lambda r: r["object_id"]):
        if r["error_m"] is None:
            print(f"[loc_probe] {r['object_id']:<20} {'--':>10}  UNMATCHED")
        else:
            status = "PASS" if r["error_m"] < bar_m else "FAIL"
            # Show the raw detected label so an aliased match (e.g. a bag
            # matched via a `box` detection) is visible, not hidden.
            raw = r.get("raw_label")
            disp = r.get("label")
            matched_as = raw if raw == disp else f"{raw}->{disp}"
            print(
                f"[loc_probe] {r['object_id']:<20} {r['error_m']:>10.3f}  "
                f"{status}    {matched_as or ''}"
            )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", required=True, help="scenario YAML (GT source)")
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--bar", type=float, default=0.3, help="max allowed error (m)")
    ap.add_argument(
        "--settle-s", type=float, default=90.0,
        help="deadline (sim-time-aware) to wait for a stable object layer",
    )
    ap.add_argument(
        "--max-match-m", type=float, default=DEFAULT_MAX_MATCH_M,
        help="search radius (m) for candidate GT<->node matches; NOT the "
             "pass/fail bar (that's --bar)",
    )
    ap.add_argument("--json", dest="json_out", default=None, help="write results JSON here")
    ap.add_argument(
        "--sim-time", dest="sim_time", action="store_true", default=None,
        help="force sim-time (ROS-time) deadlines (default: auto-detect /clock)",
    )
    ap.add_argument(
        "--no-sim-time", dest="sim_time", action="store_false",
        help="force wall-clock deadlines",
    )
    args = ap.parse_args()

    scenario = load_scenario(args.scenario)
    gt = {o.object_id: (o.x, o.y) for o in scenario.objects}

    # match_objects derives each GT object's required label from its
    # object_id's `<label>_<n>` prefix (localization_probe_lib's naming
    # rule), NOT from ObjectSpec.label directly. Every scenario YAML in
    # dcist_sim/scenarios/ happens to follow that convention, but nothing
    # enforces it -- a scenario author who breaks it (e.g. `id: box_7` with
    # `label: pallet`) would silently corrupt matching (the GT object gets
    # compared against DSG nodes of the WRONG class, either matching
    # nothing or matching the wrong object) with no error, just a
    # confusing FAIL or a wrong-looking PASS. Cross-check and warn loudly
    # rather than fail the run -- Task 4 needs to see this if it happens.
    # LABEL_ALIASES canonicalizes detected node labels (e.g. box -> bag). If a
    # scenario object's id-derived label is on the SHADOWED side of an alias
    # (a key of LABEL_ALIASES, e.g. `box_<n>`), that object could never match a
    # node of its own raw class -- every such node was canonicalized to the
    # alias target (bag) before matching -- so it would silently go UNMATCHED
    # or steal a differently-classed detection. No current scenario authors such
    # ids (all are bag_/cone_/pipe_), so this is a guard for a future author.
    for o in scenario.objects:
        derived = _label_from_object_id(o.object_id)
        if derived != o.label:
            print(
                f"[loc_probe] WARN: object '{o.object_id}' has label "
                f"'{o.label}' but its id implies label '{derived}' -- "
                f"matching uses the id-derived label, so this object's "
                f"GT match may be silently wrong. Rename the object id to "
                f"'{o.label}_<n>' to fix."
            )
        if derived in LABEL_ALIASES:
            print(
                f"[loc_probe] WARN: object '{o.object_id}' has an id-derived "
                f"label '{derived}' that LABEL_ALIASES canonicalizes away "
                f"(-> '{LABEL_ALIASES[derived]}'). Detected '{derived}' nodes "
                f"are relabeled '{LABEL_ALIASES[derived]}' before matching, so "
                f"this object can never match its own class and will look "
                f"UNMATCHED. Rename the object or revisit LABEL_ALIASES."
            )

    rclpy.init()
    node = LocalizationProbe(args.robot)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    is_sim, rtf = configure_clock_basis(node, args.sim_time)
    if is_sim:
        print(f"[loc_probe] clock basis: sim (RTF observed {rtf:.3f})")
    else:
        print("[loc_probe] clock basis: wall")

    result = {
        "scenario": os.path.abspath(args.scenario),
        "robot": args.robot,
        "bar_m": args.bar,
        "settle_s": args.settle_s,
        "max_match_m": args.max_match_m,
    }
    try:
        print(
            f"[loc_probe] waiting up to {args.settle_s:.0f} s for the object "
            f"layer to appear and hold steady for {STABILITY_S:.0f} s ..."
        )
        nodes, settled = wait_for_stable_objects(node, args.settle_s)
        result["settled"] = settled
        result["node_count"] = len(nodes)
        if not settled:
            print(
                f"[loc_probe] FAIL: object layer never stabilized within "
                f"{args.settle_s:.0f} s (last count={len(nodes)})"
            )
            result["rows"] = []
            result["summary"] = {"worst_m": None, "ok": False}
            return 1

        rows = match_objects(gt, nodes, max_match_m=args.max_match_m)
        summary = summarize(rows, bar_m=args.bar)
        result["rows"] = rows
        result["summary"] = summary

        print_table(rows, args.bar)
        verdict = "PASS" if summary["ok"] else "FAIL"
        worst = "n/a" if summary["worst_m"] is None else f"{summary['worst_m']:.3f} m"
        print(
            f"[loc_probe] OVERALL: {verdict} "
            f"(worst error {worst}, bar {args.bar:.3f} m)"
        )
        return 0 if summary["ok"] else 1
    finally:
        if args.json_out:
            with open(args.json_out, "w") as f:
                json.dump(result, f, indent=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # os._exit teardown pattern copied from e2e_smoke.py: a plain sys.exit()
    # runs normal interpreter teardown, which under rmw_zenoh with a live
    # spin thread can SIGABRT (exit 134) AFTER the verdict is already
    # printed, clobbering an otherwise-clean PASS/FAIL exit code. This
    # changes ONLY the teardown exit path, not any assertion/threshold logic.
    _code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(_code)
