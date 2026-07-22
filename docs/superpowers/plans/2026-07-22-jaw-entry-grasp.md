# Jaw-Entry Grasp + Video Capture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the `contact_hold` tier actually hold: jaw-entry grasp geometry (object enters the finger–palm slot, then pinch), with third-person baseline/after videos delivered to the user, per `docs/superpowers/specs/2026-07-22-jaw-entry-grasp-design.md`.

**Architecture:** A flag-gated video-capture module in the sim (Replicator RGB on a static third-person camera + ffmpeg); a one-shot jaw-window measurement that gates target choice (cone vs bag); four new contact-hold-only phases (JAW_STAGE → JAW_ADVANCE → CLOSE_PINCH → LIFT_VERIFY) built on pure, unit-tested geometry predicates, riding the existing terminal-hygiene funnels.

**Tech Stack:** Python (dcist_sim_isaac; deferred Isaac/pxr imports), Replicator RGB annotator (gt_capture.py's API family), ffmpeg, pytest (spark_env, no ROS), Isaac Sim 6.0.1.

## Global Constraints

- Branch: continue `feature/isaac_sim_g2_contact` (@ 3ff2ede). dcist_sim-only; NO submodule pointer changes. Push harelb only.
- G1 (`contact_hold: false`), magic, kinematic tiers byte-identical — every new behavior gated on `op.contact_hold` / the video flag.
- Terminal-hygiene contract: every exit (each new phase's timeout/failure, shove-fail, exception, drop, `reset()`) returns arm ownership, disables colliders, zeroes commands — through the EXISTING `_fail`/`_finish`/`_drop_contact_hold` funnels (do not build parallel ones). Collider enable point = transition INTO JAW_STAGE (the measured G2 lesson: earlier = ground drag; later = PhysX misses registration — see runbook §12.6a).
- Video is never-fatal (exception → log, disable video, sim continues) and off by default.
- Acceptance (spec §1.2): pick + 10 m carry + place, exit 0, TWICE, on the measured-fit target (cone or bag); honest-stop after ~2 GPU tuning sessions on pinch physics; NO pin fake.
- Baseline videos (locomotion, cone shove, bag shove) are captured and reported BEFORE any grasp-behavior change lands (Task 1 completes and its mp4 paths are reported before Task 3 starts).
- Suites green: isaac 159 + new, ros 23. GPU environment rules identical to all prior tasks in this repo's ledger (zsh `.zsh` sourcing, EULA env, PYTHONPATH APPEND, isaac venv sim / spark_env smoke, zenoh router for smokes, timeouts ≤ 600000 ms, full teardown, nvidia-smi intruder check, never wait on notifications — poll with bounded foreground Bash; scratch files in the session scratchpad).

---

### Task 1: Video capture + the three baseline videos

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/video_capture.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py` (flags + wiring, mirroring the gt_capture integration points)
- Test: `dcist_sim/dcist_sim_isaac/test/test_video_capture.py` (pure pose math + rate gate)

**Interfaces:**
- Produces CLI: `sim_app --video-out DIR [--video-fps 24]`.
- Produces module API:

```python
def third_person_pose(robot_xy, objects_centroid_xy, back_m=3.5, up_m=2.0):
    """Camera position + look-at for a static third-person framing.
    look_at = midpoint(robot_xy, objects_centroid_xy) at z=0.5;
    position = look_at - back_m * unit(look_at - robot_xy... ) -- see Step 1 tests
    for the exact contract (behind the robot, looking over it at the midpoint;
    if robot_xy == centroid (no objects), fall back to offset along +X).
    Returns ((px, py, pz), (lx, ly, lz)). Pure math."""

class VideoCapture:
    def __init__(self, out_dir, fps, camera_pose)   # Isaac-deferred
    def attach(self) -> None        # create camera prim + rep render product + rgb annotator
    def maybe_capture(self, sim_t) -> bool          # rate-gated frame write (JPEG)
    def close(self) -> str | None   # ffmpeg encode -> <out_dir>/capture.mp4; None if encode failed (frames kept)
```

- Task 4 reuses the same flags for after-videos. The CONTROLLER sends the mp4s to the user — your report must list absolute mp4 paths.

- [ ] **Step 1: Failing tests** for `third_person_pose` (midpoint look-at; camera behind robot at back_m/up_m; degenerate robot==centroid falls back to +X offset; all asserted numerically with pytest.approx) and a pure rate-gate helper (reuse-or-mirror the remainder-carrying accumulator pattern from ros_bridge — a `RateGate(fps)` class with `.ready(sim_t)`; test 24 fps over synthetic timestamps, no drift).
- [ ] **Step 2: RED. Step 3: Implement** `video_capture.py`: pure helpers at module scope (stdlib/math only); `VideoCapture` with deferred imports, modeled on `gt_capture.py`'s annotator usage (`rep.create.render_product`, `rep.AnnotatorRegistry.get_annotator("rgb")` — copy its exact API calls; JPEG via cv2/PIL, whichever gt_capture's stack already ships — check; frame files `frame_%06d.jpg`). `close()` runs `ffmpeg -y -framerate <fps> -i frame_%06d.jpg -pix_fmt yuv420p capture.mp4` via subprocess (check `shutil.which("ffmpeg")`; absent → log + return None). Never-fatal wrapper on every public method (same pattern as sim_app's gt handling).
- [ ] **Step 4: Wire into sim_app**: flags; construct after stage build when `--video-out` (any tier — flag-gated, so kinematic unaffected by default); `attach()` next to gt attach; `maybe_capture(sim_time)` in the main loop beside gt's; `close()` at shutdown, log the mp4 path. GREEN unit tests + full isaac dir (159 + new).
- [ ] **Step 5: [GPU] Capture the three baselines** (NO grasp-code changes exist yet — that ordering is the point):
  1. **Locomotion clip:** field_smoke_physics scratch scenario + zenoh router + sim with `--video-out`; publish 3-4 goto target_poses (the Task-10-era pattern: `ros2 topic pub` PoseStamped to `/hilbert/sim/target_pose`, wait on `/sim/nav_status` reached between goals) covering ~15 m of walking with turns. ~60-90 s of video.
  2. **Cone shove:** `grasp_smoke.py --contact-hold --target cone_0` with the sim running `--video-out` (separate out dir).
  3. **Bag shove:** same with `--target bag_0`.
  Verify each mp4 plays (ffprobe duration > 10 s, non-zero streams). Report the three absolute mp4 paths PROMINENTLY. Full teardown.
- [ ] **Step 6: Commit** — `feat(dcist_sim): third-person video capture (--video-out) + baseline clips` (code only; mp4s stay out of git — put them under ~/adt4_output/g2_videos/baseline/).

---

### Task 2: Jaw-window measurement + target gate

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/measure_jaw.py` (one-shot, committed as a tool)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py` (constants block only)

**Interfaces:**
- Produces constants in `grasp_backends.py` (values from the measurement, cited):

```python
# Jaw window (measured by scripts/measure_jaw.py on <date>; gripper frame):
JAW_WINDOW_DEPTH_M = <measured>     # along mouth axis, palm plate -> finger tip arc
JAW_WINDOW_HEIGHT_M = <measured>    # palm plate -> finger underside at open
JAW_MOUTH_AXIS_GRIPPER = (<x>, <y>, <z>)  # unit vector, gripper frame
JAW_TARGET_OBJECT = "cone_0" | "bag_0"    # the measured-fit acceptance target
```

- [ ] **Step 1: The tool.** `measure_jaw.py`: boot headless on a field_smoke_physics scratch copy (contact_hold robot), deploy the arm (drive the existing deploy via a direct `_ArmInterface` use or replaying the backend's deploy targets — simplest: instantiate the backend's arm interface and call its deploy, since Task 13 built that machinery), open gripper (f1x to its open limit — read the joint's limit from the articulation), then `UsdGeom.BBoxCache` world AABBs of `arm0_link_fngr/visuals` and `arm0_link_wr1/visuals`; derive the window (gap between palm top face and finger underside; depth along the mouth axis; express the axis in the gripper frame via the link's world orientation). Print a table. Also print cone/bag/pipe world AABBs from their spawned prims for cross-sections.
- [ ] **Step 2: [GPU] Run it; decide.** Fit rule: object cross-section (the two axes perpendicular to the mouth axis at grasp height) ≤ window dims − 0.02 m margin each. If BOTH fit, prefer the cone (rigid, cleaner contact geometry). **If NEITHER fits: STOP the plan** — report the table; the asset decision goes to the user (spec §6).
- [ ] **Step 3: Commit** constants + tool — `feat(dcist_sim): jaw window measurement + acceptance target constants`. Report the measured table verbatim.

---

### Task 3: Jaw phases (TDD at the fake seam)

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_grasp_backends.py` (extend)

**Interfaces:**
- Consumes Task 2's constants. Produces pure predicates + phases:

```python
def object_in_jaw_window(gripper_pos, gripper_quat_wxyz, obj_pos,
                         mouth_axis_gripper=JAW_MOUTH_AXIS_GRIPPER,
                         depth_m=JAW_WINDOW_DEPTH_M, height_m=JAW_WINDOW_HEIGHT_M,
                         half_width_m=JAW_WINDOW_HEIGHT_M / 2):
    """True iff obj_pos lies inside the jaw slot volume anchored at the gripper.
    Pure: rotate obj-gripper delta into the gripper frame (reuse grasp.py's
    _rotate_vector/_quat_conjugate), check 0 <= along-mouth <= depth and
    |perp components| <= bounds. Unit-tested with synthetic poses."""

# _Op phases (contact_hold grasp only), replacing REACH_PREGRASP/DESCEND/
# CONTACT_CLOSE for jaw mode:
JAW_STAGE = "jaw_stage"; JAW_ADVANCE = "jaw_advance"
CLOSE_PINCH = "close_pinch"; LIFT_VERIFY = "lift_verify"
```

Semantics (each phase timeboxed; constants named `JAW_STAGE_TIMEOUT_S = 8.0`, `JAW_ADVANCE_TIMEOUT_S = 8.0`, `CLOSE_PINCH_TIMEOUT_S = 4.0`, `LIFT_VERIFY_TIMEOUT_S = 4.0`, tunable in Task 4):
- Colliders + contact reporting enable at the VALIDATE→JAW_STAGE transition.
- JAW_STAGE: IK servo (existing IkServo machinery) to staging pose = object position − standoff (`JAW_STAGE_STANDOFF_M = 0.18`) along the world-frame mouth axis (gripper frame axis rotated by current gripper orientation), at object mid-height. Converged → JAW_ADVANCE.
- JAW_ADVANCE: servo forward along the mouth axis in small increments until `object_in_jaw_window(...)`. Shove detection: object XY displacement from its phase-entry position > `JAW_SHOVE_TOL_M = 0.08` while NOT in-window → back off to staging pose and retry ONCE (`op.jaw_retries`); second shove → `_fail(op, "shoved")`.
- CLOSE_PINCH: close f1x incrementally (reuse the existing close machinery/step size) until `finger_object_contact(...)` AND still `object_in_jaw_window(...)`; contact w/o window (slipped out during close) → treat as shove-retry (same counter); timeout → `_fail(op, "no pinch contact")`.
- LIFT_VERIFY: record gripper + object z; raise gripper `JAW_LIFT_M = 0.12` via servo; held ⟺ contact still reported AND object z rose ≥ `JAW_LIFT_M * 0.5`. Held → `_succeed_grasp` (existing: keeps arm + colliders through carry); not held → `_drop_contact_hold`-style fail ("lift verify failed").
- G1 pin path and the old press-from-above code path: for contact_hold ops the jaw phases REPLACE the old sequence entirely (delete or dead-gate the old CONTACT_CLOSE press path — prefer delete with the honest-stop history preserved in the module docstring; the runbook keeps the archaeology).

- [ ] **Step 1: Failing tests** (fake seam; follow the file's patterns): (a) `object_in_jaw_window` pure cases — inside, beyond depth, below palm, lateral out, quaternion-rotated gripper (compute one rotated case by hand); (b) happy path phase walk stage→advance→pinch→lift→succeeded with colliders enabled exactly at VALIDATE→JAW_STAGE and kept through carry; (c) advance shove → back-off retry → second shove fails "shoved", colliders disabled, arm released; (d) close-pinch timeout fails "no pinch contact"; (e) lift-verify failure routes to drop path; (f) every phase timeout fails with its named message + full terminal hygiene; (g) G1 grasp never enters jaw phases (byte-identical trace).
- [ ] **Step 2: RED. Step 3: Implement** per the semantics above. **Step 4: GREEN** + full isaac dir. **Step 5: Commit** — `feat(dcist_sim): jaw-entry grasp phases for contact hold`.

---

### Task 4: [GPU] Tune, accept, after-videos, ship

**Files:**
- Modify: `docs/sim_runbook.md` (G2 sections: jaw design, tuning table, acceptance evidence or honest stop)
- Possibly tune: the `JAW_*` constants (committed with rationale)

- [ ] **Step 1: [GPU] Single-grasp tuning** on the Task-2 target: `grasp_smoke.py --contact-hold --target <JAW_TARGET_OBJECT>` with `--video-out` on the sim. Iterate JAW_* constants (advance step size/speed and shove tolerance first) until pick reliably pinches (contact + in-window + lift-verify). Record every iteration.
- [ ] **Step 2: [GPU] Acceptance:** pick + 10 m carry + place, exit 0, TWICE, videos on. Slip events verbatim. Honest-stop contract after ~2 sessions (wip commit + measurements + runbook; NO pin fake).
- [ ] **Step 3: After-videos:** the successful (or honest-fail) grasp clips + one carry clip → `~/adt4_output/g2_videos/after/`; report absolute paths PROMINENTLY (controller sends to user).
- [ ] **Step 4: Runbook + commits** (`feat(dcist_sim): G2 jaw-entry contact hold - acceptance` or `wip(...)` + `docs(sim): G2 jaw evidence + videos`), push `git push harelb feature/isaac_sim_g2_contact`. Suites green (isaac 159+new, ros 23).

---

## Self-Review Notes

- Spec §2 video (flags, pose, annotator family, never-fatal, ffmpeg, baseline-before-changes ordering) → Task 1; §3 measure-first + phases + shove/timeout semantics + enable point → Tasks 2-3; §1.2 bar + honest stop + after-videos → Task 4; §6 neither-fits STOP → Task 2 Step 2.
- Names consistent: `object_in_jaw_window`, JAW_* constants, `third_person_pose`, `VideoCapture` used identically across tasks.
- Videos deliberately out of git; controller delivers via file-send after Tasks 1 and 4.
