#!/usr/bin/env python3
"""Jaw-Entry Grasp plan, Task 2 (`.superpowers/sdd/task-2-brief.md`): one-shot
jaw-window measurement + acceptance-target fit-gate data collection.

Boots Isaac in-process (no ROS, no zenoh router -- mirrors
`fall_characterize.py`/`_bounds_probe.py`'s "SimulationApp -> build_stage ->
step -> read -> exit" pattern), on a `field_smoke_contact_hold.yaml`-shaped
scratch scenario (`locomotion: policy` + `grasping: physics` +
`contact_hold: true` -- the only spec combination that provisions the G2
gripper meshes, `spot_robot._wants_gripper_colliders`; this script only reads
their AABBs, it never enables the colliders). Then:

  1. DEPLOYS the arm via `grasp_backends._ArmInterface.deploy()` -- the SAME
     deploy machinery `PhysicsGraspBackend` uses (Task 13), so the measured
     window is the one every real grasp actually sees, not some other pose.
  2. Reads the live `arm0_f1x` joint's hard limits straight off the
     articulation (`SingleArticulation.dof_properties['lower'/'upper']`) --
     deliberately NOT `grasp_backends.GRIPPER_OPEN_RAD`/`GRIPPER_CLOSE_RAD`,
     since the point of this step is to independently VERIFY which limit is
     open rather than trust that comment.
  3. Commands f1x to each limit in turn, settles, and reads
     `arm0_link_fngr/visuals` (finger) + `arm0_link_wr1/visuals` (palm) world
     AABBs via `UsdGeom.BBoxCache` at each, PLUS `arm0_link_wr1`'s live world
     pose. Everything is then re-expressed in the **palm's (wr1) own local
     frame** at that instant (`grasp._rotate_vector`/`_quat_conjugate`,
     `grasp_backends`' own quaternion convention) before any comparison.

     WHY local-frame, not raw world AABBs (found the hard way, GPU, this
     script): a `locomotion: policy` Spot's floating base is never perfectly
     still -- over the several seconds needed to settle a new f1x target, an
     early world-frame-only version of this script measured the finger AND
     THE PALM (upstream of f1x, physically unmoved by it) drifting by the
     SAME ~0.45 m between the two settle windows (diagnosed with a throwaway
     `_drift_diag.py` probe logging `base_pose_xyzyaw()` + both AABBs at
     sub-second cadence). `base_pose_xyzyaw()` only reports yaw, not
     roll/pitch, so the standing policy's small body-tilt oscillation,
     amplified by the arm's ~0.85 m reach (lever arm), was invisible to that
     diagnostic yet plainly swinging the whole gripper assembly in world
     space. Since `arm0_link_fngr` hangs off `arm0_link_wr1` via ONLY the
     f1x revolute joint, the finger's pose EXPRESSED IN wr1's OWN local frame
     is a pure function of the f1x joint angle and is immune to any upstream
     (base/arm) sway -- exactly the quantity a "jaw window" measurement
     needs. (The palm's own AABB, re-expressed in its own frame, is printed
     too as a sanity check: it is expected to be near test-run-to-test-run
     constant regardless of f1x, confirming the transform is doing its job.)
  4. At the open limit (in the wr1-local frame), derives:
       - MOUTH AXIS: the unit vector from the palm-local box center to the
         finger-local box center (no further rotation needed -- already in
         the gripper's own frame, which is what `JAW_MOUTH_AXIS_GRIPPER`
         wants).
       - WINDOW HEIGHT / DEPTH: the finger-local + palm-local boxes'
         projected extents along the mouth axis (DEPTH) and along the
         in-frame axis closest to "up" once the mouth-axis component is
         removed (HEIGHT) -- both are UNION extents of the two boxes (palm
         plate -> finger tip arc), i.e. an outer-envelope size, not a strict
         empty-space gap; see the printed raw boxes to sanity-check by hand.
  5. Dumps `cone_0`/`bag_0`/`pipe_0` world AABBs (`/World/objects/<id>`) for
     cross-section comparison against the window.
  6. USER DECISION (2026-07-22, task-2-report.md): `cone_0`'s full BASE
     footprint doesn't clear the window, but a traffic cone TAPERS -- this
     script models that taper as LINEAR from the measured base
     cross-section (h=0) to a point apex (h=`cone_0`'s own measured total
     height), and solves for the minimum grasp height `h_fit` at which the
     shrunk cross-section clears both window dims (margin 0.02 m each),
     printing `JAW_GRASP_HEIGHT_M` + the cross-section there + how much cone
     remains above `h_fit` (sanity floor: >=0.05 m, needed to actually pinch
     something).

Every derived number is printed on its own `MEASURE ...` line so a human/
report can `grep MEASURE` the run log for the verbatim table.

Usage (Isaac Sim venv directly, no spark_env, no zenoh router needed):

    source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
    OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \\
    PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \\
    ~/environments/dcist/isaac_sim/bin/python \\
        dcist_sim/dcist_sim_isaac/scripts/measure_jaw.py \\
        --scenario /path/to/scratch_field_smoke_contact_hold.yaml
"""
import argparse
import sys

import numpy as np

# `grasp.py` is Isaac-import-free at module load (same "defer Isaac into
# methods" convention `grasp_backends.py` uses) -- safe to import here, ahead
# of `SimulationApp` construction inside `main()`.
from dcist_sim_isaac.grasp import _quat_conjugate, _rotate_vector

# Frames to hold the 6-servo deploy pose before reading anything (mirrors
# grasp_backends.DEPLOY_SETTLE_S=1.5 s at the policy's 60 Hz rendering_dt,
# plus generous margin -- this is a one-shot measurement, not perf-critical).
ARM_SETTLE_FRAMES_DEFAULT = 180
# Frames to hold a new f1x target before trusting the AABB it produced.
F1X_SETTLE_FRAMES_DEFAULT = 120


def _world_aabb(usd_stage, prim_path):
    """Fresh `UsdGeom.BBoxCache` PER CALL -- deliberately not a shared/reused
    cache. `UsdGeom.BBoxCache` memoizes per-prim bounds and, empirically (GPU,
    this script), does NOT invalidate that memo when a physics-driven joint
    moves the prim's transform between calls at the same `Usd.TimeCode` -- a
    reused cache silently returned the identical (first-computed) finger AABB
    for both the f1x lower AND upper limit even though
    `get_joint_positions()` confirmed the joint physically moved ~1.5 rad in
    between (diagnosed via a throwaway `_f1x_diag.py` probe). A new cache
    instance has no memo to go stale, at the cost of a cheap recompute."""
    from pxr import Usd, UsdGeom

    prim = usd_stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    return np.array([mn[0], mn[1], mn[2]], dtype=float), \
        np.array([mx[0], mx[1], mx[2]], dtype=float)


def _box_corners(box):
    mn, mx = box
    return np.array([
        [mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]],
        [mn[0], mx[1], mn[2]], [mn[0], mn[1], mx[2]],
        [mx[0], mx[1], mn[2]], [mx[0], mn[1], mx[2]],
        [mn[0], mx[1], mx[2]], [mx[0], mx[1], mx[2]],
    ], dtype=float)


def _to_local_box(world_box, ref_pos, ref_quat_wxyz):
    """Re-express `world_box` (world-frame AABB, (min,max)) in the local
    frame of `ref_pos`/`ref_quat_wxyz` (a live world pose) -- transforms all
    8 corners then re-derives axis-aligned min/max in that frame (a rotated
    box's corners don't just carry through on the min/max points). Uses the
    exact quaternion convention `grasp_backends`/`grasp` already use for the
    gripper<->object local offset (`_rotate_vector`/`_quat_conjugate`)."""
    q_conj = _quat_conjugate(tuple(float(v) for v in ref_quat_wxyz))
    corners = _box_corners(world_box)
    local_corners = np.array(
        [_rotate_vector(tuple(float(v) for v in (c - ref_pos)), q_conj)
         for c in corners], dtype=float)
    return local_corners.min(axis=0), local_corners.max(axis=0)


def _fmt_box(name, box):
    if box is None:
        return f"{name}: INVALID PRIM"
    mn, mx = box
    sz = mx - mn
    return (f"{name}: min=({mn[0]:.4f},{mn[1]:.4f},{mn[2]:.4f}) "
            f"max=({mx[0]:.4f},{mx[1]:.4f},{mx[2]:.4f}) "
            f"size=({sz[0]:.4f},{sz[1]:.4f},{sz[2]:.4f})")


def _center(box):
    mn, mx = box
    return (mn + mx) / 2.0


def _project_extent(box, axis_unit):
    """Min/max of `box`'s 8 corners projected onto unit vector `axis_unit`
    (same frame as the box) -- the box's extent along that axis regardless
    of whether the box is axis-aligned to it."""
    proj = _box_corners(box) @ axis_unit
    return float(proj.min()), float(proj.max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True,
                     help="scratch scenario yaml -- must have a "
                          "locomotion:policy + grasping:physics + "
                          "contact_hold:true robot (the only spec that "
                          "provisions arm0_link_fngr/visuals + "
                          "arm0_link_wr1/visuals as separate collidable "
                          "meshes; this script only reads their USD AABBs, "
                          "collider enable state is irrelevant here)")
    ap.add_argument("--arm-settle-frames", type=int,
                     default=ARM_SETTLE_FRAMES_DEFAULT)
    ap.add_argument("--f1x-settle-frames", type=int,
                     default=F1X_SETTLE_FRAMES_DEFAULT)
    args = ap.parse_args()

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": True})

    import omni.usd
    from isaacsim.core.prims import XFormPrim
    from isaacsim.core.utils.types import ArticulationAction

    from dcist_sim_isaac.scenario import load_scenario
    from dcist_sim_isaac.stage import build_stage
    from dcist_sim_isaac.grasp_backends import _ArmInterface
    from dcist_sim_isaac.spot_robot import (
        GRIPPER_MESH_RELATIVE_PATH, PALM_RELATIVE_PATH)

    scenario = load_scenario(args.scenario)
    stage = build_stage(scenario)
    world = stage.world
    robot = stage.robots[0]
    if robot.drive_backend is None:
        print("FATAL: robot has no drive_backend (need locomotion: policy "
              "-- see grasp_backends module SCOPE CUT)")
        sim_app.close()
        return 1

    # -- deploy via the SAME machinery PhysicsGraspBackend uses --------------
    arm = _ArmInterface(robot)
    arm.take_ownership()
    for _ in range(args.arm_settle_frames):
        arm.deploy()
        world.step(render=False)

    usd_stage = omni.usd.get_context().get_stage()

    finger_path = f"{robot.prim_path}/{GRIPPER_MESH_RELATIVE_PATH}"
    palm_path = f"{robot.prim_path}/{PALM_RELATIVE_PATH}"
    wr1_path = f"{robot.prim_path}/{PALM_RELATIVE_PATH.rsplit('/', 1)[0]}"
    wr1_xform = XFormPrim(wr1_path)

    # -- f1x joint limits, straight off the live articulation ---------------
    art = robot.drive_backend.articulation()
    arm_idx = robot.drive_backend.arm_dof_indices()
    arm_names = robot.drive_backend.arm_dof_names()
    f1x_candidates = [i for i, n in zip(arm_idx, arm_names) if "f1x" in n]
    if not f1x_candidates:
        print(f"FATAL: no f1x DOF found in arm_dof_names={arm_names}")
        sim_app.close()
        return 1
    f1x_i = f1x_candidates[0]
    lower = float(art.dof_properties["lower"][f1x_i])
    upper = float(art.dof_properties["upper"][f1x_i])
    print(f"MEASURE f1x_joint_limits lower={lower:.4f} upper={upper:.4f} rad")

    def set_f1x(rad):
        art.apply_action(ArticulationAction(
            joint_positions=np.array([float(rad)], dtype=float),
            joint_indices=[f1x_i]))

    def settle_at(rad, n_frames):
        for _ in range(n_frames):
            arm.deploy()   # keep holding the 6-servo deploy pose too
            set_f1x(rad)
            world.step(render=False)
        # Read the palm's live world pose AND both AABBs at this SAME instant
        # (no further world.step calls in between) so the wr1-local transform
        # below is self-consistent even though the base/arm may be swaying.
        wr1_pos, wr1_quat = wr1_xform.get_world_poses()
        wr1_pos, wr1_quat = wr1_pos[0], wr1_quat[0]
        finger_w = _world_aabb(usd_stage, finger_path)
        palm_w = _world_aabb(usd_stage, palm_path)
        finger_l = _to_local_box(finger_w, wr1_pos, wr1_quat)
        palm_l = _to_local_box(palm_w, wr1_pos, wr1_quat)
        return finger_w, palm_w, finger_l, palm_l

    finger_w_lo, palm_w_lo, finger_lo, palm_lo = settle_at(
        lower, args.f1x_settle_frames)
    finger_w_hi, palm_w_hi, finger_hi, palm_hi = settle_at(
        upper, args.f1x_settle_frames)

    print(f"MEASURE {_fmt_box('finger_WORLD@f1x=lower(' + f'{lower:.4f}' + ')', finger_w_lo)}")
    print(f"MEASURE {_fmt_box('palm_WORLD@f1x=lower', palm_w_lo)}")
    print(f"MEASURE {_fmt_box('finger_WORLD@f1x=upper(' + f'{upper:.4f}' + ')', finger_w_hi)}")
    print(f"MEASURE {_fmt_box('palm_WORLD@f1x=upper', palm_w_hi)}")
    print("MEASURE NOTE: world-frame boxes above are for reference only "
          "(the standing policy's base/arm sway moves BOTH finger and palm "
          "in world space by comparable amounts across the multi-second "
          "settle windows -- see module docstring); all derived numbers "
          "below use the wr1-LOCAL-frame boxes, which cancel that sway.")

    print(f"MEASURE {_fmt_box('finger_LOCAL@f1x=lower(' + f'{lower:.4f}' + ')', finger_lo)}")
    print(f"MEASURE {_fmt_box('palm_LOCAL@f1x=lower (sanity, ~= palm own AABB)', palm_lo)}")
    print(f"MEASURE {_fmt_box('finger_LOCAL@f1x=upper(' + f'{upper:.4f}' + ')', finger_hi)}")
    print(f"MEASURE {_fmt_box('palm_LOCAL@f1x=upper (sanity, ~= palm own AABB)', palm_hi)}")

    gap_lo = float(np.linalg.norm(_center(finger_lo) - _center(palm_lo)))
    gap_hi = float(np.linalg.norm(_center(finger_hi) - _center(palm_hi)))
    print(f"MEASURE finger_palm_center_dist_LOCAL@lower={gap_lo:.4f} m")
    print(f"MEASURE finger_palm_center_dist_LOCAL@upper={gap_hi:.4f} m")

    if gap_lo >= gap_hi:
        open_rad, closed_rad = lower, upper
        finger_open, palm_open = finger_lo, palm_lo
    else:
        open_rad, closed_rad = upper, lower
        finger_open, palm_open = finger_hi, palm_hi
    print(f"MEASURE open_direction_verified f1x_open={open_rad:.4f} rad "
          f"(gap={max(gap_lo, gap_hi):.4f} m) f1x_closed={closed_rad:.4f} rad "
          f"(gap={min(gap_lo, gap_hi):.4f} m)")

    # -- mouth axis: LOCAL dir palm-center -> finger-center @ open (already
    # in the gripper/wr1 frame -- no further rotation needed) ---------------
    axis_raw = _center(finger_open) - _center(palm_open)
    norm = float(np.linalg.norm(axis_raw))
    if norm < 1e-6:
        print("FATAL: finger/palm local-box centers coincide at the open "
              "limit; cannot derive a mouth axis")
        sim_app.close()
        return 1
    mouth_axis = axis_raw / norm
    print(f"MEASURE mouth_axis_gripper_frame=({mouth_axis[0]:.4f},"
          f"{mouth_axis[1]:.4f},{mouth_axis[2]:.4f})")

    # -- window height: extent along the in-frame axis closest to "up" (Z)
    # once the mouth-axis component is projected out (Gram-Schmidt) --------
    up_local = np.array([0.0, 0.0, 1.0])
    height_axis = up_local - float(np.dot(up_local, mouth_axis)) * mouth_axis
    h_norm = float(np.linalg.norm(height_axis))
    if h_norm < 1e-6:   # mouth axis IS local Z; fall back to local X
        up_local = np.array([1.0, 0.0, 0.0])
        height_axis = up_local - float(np.dot(up_local, mouth_axis)) * mouth_axis
        h_norm = float(np.linalg.norm(height_axis))
    height_axis = height_axis / h_norm
    width_axis = np.cross(mouth_axis, height_axis)
    print(f"MEASURE height_axis_gripper_frame=({height_axis[0]:.4f},"
          f"{height_axis[1]:.4f},{height_axis[2]:.4f})")
    print(f"MEASURE width_axis_gripper_frame=({width_axis[0]:.4f},"
          f"{width_axis[1]:.4f},{width_axis[2]:.4f})")

    # -- window depth/height/width: UNION extents of the finger + palm
    # boxes along each of the 3 orthonormal axes above (palm plate -> finger
    # tip arc for depth; the analogous envelope size for height/width) -----
    def union_extent(axis):
        p_lo, p_hi = _project_extent(palm_open, axis)
        f_lo, f_hi = _project_extent(finger_open, axis)
        lo, hi = min(p_lo, f_lo), max(p_hi, f_hi)
        return lo, hi, hi - lo

    depth_lo, depth_hi, window_depth = union_extent(mouth_axis)
    height_lo, height_hi, window_height = union_extent(height_axis)
    width_lo, width_hi, window_width = union_extent(width_axis)
    print(f"MEASURE mouth_axis_projection union=[{depth_lo:.4f},{depth_hi:.4f}]")
    print(f"MEASURE height_axis_projection union=[{height_lo:.4f},{height_hi:.4f}]")
    print(f"MEASURE width_axis_projection union=[{width_lo:.4f},{width_hi:.4f}]")
    print(f"MEASURE JAW_WINDOW_DEPTH_M={window_depth:.4f}")
    print(f"MEASURE JAW_WINDOW_HEIGHT_M={window_height:.4f}")
    print(f"MEASURE JAW_WINDOW_WIDTH_M={window_width:.4f}  "
          f"(not a shipped constant -- reference only)")

    # -- object cross-sections (world AABB of each spawned object) ----------
    object_bb = {}
    for obj in scenario.objects:
        obj_path = f"/World/objects/{obj.object_id}"
        bb = _world_aabb(usd_stage, obj_path)
        object_bb[obj.object_id] = bb
        print(f"MEASURE {_fmt_box('object_' + obj.object_id, bb)}")

    # -- cone-at-height fit (user decision, 2026-07-22, task-2-report.md):
    # cone_0's full BASE footprint doesn't fit the window (see the report),
    # but a real traffic cone TAPERS -- narrows linearly from its measured
    # base cross-section (h=0, ground) to a point apex (h=H_total, the
    # cone's own measured height). Find the minimum grasp height h_fit at
    # which the shrunk cross-section clears BOTH window dims (margin
    # 0.02 m each), pairing the LARGER cone-base axis with the LARGER
    # window dim and the smaller with the smaller (same pairing convention
    # the whole-object h=0 fit check in the report used) -- cone_0's base is
    # very nearly circular (x/y within 0.6% of each other in every measured
    # run) so this pairing choice barely matters here.
    cone_bb = object_bb.get("cone_0")
    if cone_bb is not None:
        mn, mx = cone_bb
        base_x, base_y = float(mx[0] - mn[0]), float(mx[1] - mn[1])
        h_total = float(mx[2] - mn[2])
        margin = 0.02
        height_eff = window_height - margin
        depth_eff = window_depth - margin
        win_big, win_small = max(height_eff, depth_eff), min(height_eff, depth_eff)
        if base_x >= base_y:
            obj_big, obj_small = base_x, base_y
        else:
            obj_big, obj_small = base_y, base_x
        frac_big = max(0.0, 1.0 - win_big / obj_big) if obj_big > 0 else 0.0
        frac_small = max(0.0, 1.0 - win_small / obj_small) if obj_small > 0 else 0.0
        frac_fit = max(frac_big, frac_small)   # binding (tighter) constraint
        h_fit = frac_fit * h_total
        cross_x_at_fit = base_x * (1.0 - frac_fit)
        cross_y_at_fit = base_y * (1.0 - frac_fit)
        remaining_above = h_total - h_fit
        print(f"MEASURE cone_taper_base=({base_x:.4f},{base_y:.4f}) "
              f"h_total={h_total:.4f}")
        print(f"MEASURE cone_fit_window height_eff={height_eff:.4f} "
              f"depth_eff={depth_eff:.4f}")
        print(f"MEASURE JAW_GRASP_HEIGHT_M={h_fit:.4f}")
        print(f"MEASURE cone_cross_section_at_h_fit=({cross_x_at_fit:.4f},"
              f"{cross_y_at_fit:.4f})")
        print(f"MEASURE cone_remaining_above_h_fit={remaining_above:.4f} m "
              f"(>=0.05 m required for a pinch: "
              f"{'PASS' if remaining_above >= 0.05 else 'FAIL'})")
    else:
        print("MEASURE cone_taper: cone_0 not found in scenario objects "
              "-- cannot compute JAW_GRASP_HEIGHT_M")

    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
