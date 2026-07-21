#!/usr/bin/env python3
"""Render a baked costmap (+ optional tour overlay) to a PNG for authoring the
physics mapping tour, and check tour waypoints against it (spec Task 16).

Runs in the spark_env venv (matplotlib + numpy; NO ROS/Isaac needed):

    PYTHONPATH=dcist_sim/dcist_sim_isaac \
    ~/environments/dcist/spark_env/bin/python \
        dcist_sim/dcist_sim_isaac/scripts/render_costmap.py \
        --costmap ~/adt4_output/warehouse_sim_physics/costmap.npz \
        --costmap-raw ~/adt4_output/warehouse_sim_physics/costmap_raw.npz \
        --scenario dcist_sim/scenarios/warehouse_tour_physics.yaml \
        --out /tmp/costmap.png

--check prints, per tour waypoint, whether it is free in the INFLATED map and
whether it has a 1-cell inflation margin (its cell + all 8 neighbors free) --
the Task 10 boundary-dwell trap. Exit 0 iff every waypoint is free WITH margin.
"""
import argparse
import os
import sys

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(THIS_DIR))  # dcist_sim_isaac package root

from dcist_sim_isaac.costmap import Costmap2D  # noqa: E402
from dcist_sim_isaac.scenario import load_scenario  # noqa: E402


def _extent(cm):
    ny, nx = cm.grid.shape
    x0, y0 = cm.origin_xy
    return [x0, x0 + nx * cm.resolution, y0, y0 + ny * cm.resolution]


def has_margin(cm, x, y):
    """True iff (x, y)'s cell and all 8 neighbors are FREE in `cm`."""
    cell = cm.world_to_grid(x, y)
    if cell is None:
        return False
    ix, iy = cell
    ny, nx = cm.grid.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            jx, jy = ix + dx, iy + dy
            if not (0 <= jx < nx and 0 <= jy < ny):
                return False
            if cm.grid[jy, jx] != cm.FREE:
                return False
    return True


def check_tour(cm, scenario):
    """Print a per-waypoint free/margin report; return list of bad indices."""
    bound = scenario.nav.snap_bound_m
    bad = []
    print(f"{'idx':>3} {'x':>8} {'y':>8}  {'free':>5} {'margin':>6}  "
          f"{'snap->':>6} note")
    for i, wp in enumerate(scenario.tour):
        free = cm.is_free_world(wp.x, wp.y)
        margin = has_margin(cm, wp.x, wp.y)
        note = ""
        if not (free and margin):
            snapped = cm.inflate(cm.resolution).nearest_free(wp.x, wp.y, bound)
            if snapped is None:
                note = f"NO free+margin cell within {bound} m -> FAIL"
                bad.append(i)
            else:
                d = ((snapped[0] - wp.x) ** 2 + (snapped[1] - wp.y) ** 2) ** 0.5
                note = f"would snap to ({snapped[0]:.2f},{snapped[1]:.2f}) d={d:.2f}m"
        print(f"{i:>3} {wp.x:>8.2f} {wp.y:>8.2f}  {str(free):>5} "
              f"{str(margin):>6}  {'':>6} {note}")
    return bad


def render(cm, cm_raw, scenario, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ny, nx = cm.grid.shape
    # RGB image: free=white, inflation-only=light orange, raw-occupied=black.
    img = np.ones((ny, nx, 3), dtype=float)
    infl = cm.grid == cm.OCCUPIED
    img[infl] = (1.0, 0.75, 0.4)   # inflation halo
    if cm_raw is not None:
        img[cm_raw.grid == cm_raw.OCCUPIED] = (0.1, 0.1, 0.1)  # real obstacle
    else:
        img[infl] = (0.1, 0.1, 0.1)

    fig, ax = plt.subplots(figsize=(9, 16))
    ax.imshow(img, origin="lower", extent=_extent(cm), interpolation="nearest")

    if scenario is not None:
        xs = [wp.x for wp in scenario.tour]
        ys = [wp.y for wp in scenario.tour]
        if xs:
            ax.plot(xs, ys, "-", color="tab:blue", lw=0.8, alpha=0.6)
            ax.scatter(xs, ys, c="tab:blue", s=18, zorder=3)
            for i, (x, y) in enumerate(zip(xs, ys)):
                ax.annotate(str(i), (x, y), fontsize=6, color="tab:blue")
        for r in scenario.robots:
            ax.scatter([r.x], [r.y], c="lime", marker="*", s=180, zorder=4,
                       edgecolors="k", label=f"spawn:{r.name}")
        if scenario.robots:
            ax.legend(loc="upper right", fontsize=7)

    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title(os.path.basename(out_path))
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print(f"[render_costmap] wrote {out_path}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--costmap", required=True, help="inflated costmap .npz")
    ap.add_argument("--costmap-raw", help="optional pre-inflation costmap .npz")
    ap.add_argument("--scenario", help="scenario YAML to overlay/check its tour")
    ap.add_argument("--out", default="/tmp/costmap.png")
    ap.add_argument("--check", action="store_true",
                    help="print per-waypoint free/margin report; exit 1 if any "
                         "waypoint has no free+margin cell within snap bound")
    args = ap.parse_args()

    cm = Costmap2D.load(args.costmap)
    cm_raw = Costmap2D.load(args.costmap_raw) if args.costmap_raw else None
    scenario = load_scenario(args.scenario) if args.scenario else None
    print(f"[render_costmap] inflated grid {cm.shape} res {cm.resolution} m "
          f"origin {cm.origin_xy} extent {_extent(cm)}", flush=True)

    rc = 0
    if args.check:
        if scenario is None:
            sys.exit("--check requires --scenario")
        bad = check_tour(cm, scenario)
        if bad:
            print(f"[render_costmap] CHECK FAIL: waypoints {bad} have no "
                  f"free+margin cell within snap bound", flush=True)
            rc = 1
        else:
            print("[render_costmap] CHECK OK: all waypoints free with margin",
                  flush=True)

    render(cm, cm_raw, scenario, args.out)
    return rc


if __name__ == "__main__":
    sys.exit(main())
