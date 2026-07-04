"""ZED-shaped camera publishing for dcist_sim_isaac (Task 8).

Feeds the ADT4 perception stack's exact subscription contract
(`master.launch.yaml:96-101` Hydra remaps, `:164-173` ROMAN remaps):

  /{name}/{name}_zed/rgb/image_rect_color    (sensor_msgs/Image)
  /{name}/{name}_zed/rgb/camera_info         (sensor_msgs/CameraInfo)
  /{name}/{name}_zed/depth/depth_registered  (sensor_msgs/Image, 32FC1, meters)
  /{name}/{name}_zed/depth/camera_info       (sensor_msgs/CameraInfo)

all at 15 Hz, RGB/depth stamps identical per frame pair, header.frame_id
= the real ZED optical frame name.

## Step 1 (contract pinning): found a real bag, used it instead of deriving

The task brief's Step 1 asked to pin the contract from a real Spot bag
via `ros2 bag info` + `ros2 topic echo --once ... -b <bag>`. That exact
CLI recipe doesn't work on this machine (`ros2 topic echo` has no `-b`/
`--storage` flag in this Jazzy build; `ros2 bag info <mcap>` also needs
`--storage mcap` explicitly or it can't detect the plugin). Used
`rosbag2_py.SequentialReader` directly instead (see
`/tmp/.../scratchpad/extract_bag.py` used during this task -- not
checked in, one-off).

Bag used: `/home/harel/data/west_point_2026/bravo_map_1_wed_afternoon_experiment_2_euclid/recorded_data/recorded_data_0.mcap`
(95 GB, real West Point 2026 field deployment, robot "euclid" -- a real
Spot per `reference_spot_tf_frame_remaps.md`). It has all four target
topics plus `/tf_static`. Extracted 2026-07-04 by deserializing the
first 3 messages of each of the 4 image/camera_info topics, and every
`/tf_static` message mentioning "euclid".

Every constant below is labeled BAG-MEASURED (read directly from this
bag) or DERIVED (inferred from `zed_overrides.yaml` /
`calibration.yaml` / zed-ros2-wrapper conventions because the bag
doesn't cover it -- Task 12 must validate these live against Hydra/
ROMAN, per the task-8 brief).

### BAG-MEASURED: image/camera_info contract

- Encodings: rgb='bgra8', depth='32FC1'; both is_bigendian=0.
- Resolution: 640x360 for BOTH rgb and depth (step=2560 = 640*4 bytes,
  consistent with bgra8 and float32 packing). This CORRECTS the task
  brief's derived guess of "~960x540" (which assumed 1920x1080 HD1080
  native / pub_downscale_factor 2.0, per `zed_overrides.yaml`) -- the
  actual euclid rig ran a different native mode (1280x720 HD720 / 2.0
  = 640x360). The bag wins over the derived guess; DERIVED_RESOLUTION
  below is kept only as a documented fallback and is NOT what this
  file uses.
- frame_id, byte-identical across all 4 topics and every sampled
  message: 'euclid_zed_left_camera_optical_frame' -- i.e. the pattern
  is "{name}_zed_left_camera_optical_frame" (underscore-joined, unlike
  the "{name}/..." slash convention `ros_bridge.py` uses for
  odom/body/joint frames).
- K (identical rgb/depth camera_info, every sampled message):
  fx=fy=261.9303894042969, cx=336.3063049316406, cy=167.84288024902344,
  distortion_model='rational_polynomial', D=[0,0,0,0,0] (already
  rectified -- image_rect_color/depth_registered are post-rectification
  topics).
- header.stamp is byte-identical between the rgb and depth message at
  the same sequence index (checked on 3 index-aligned pairs) --
  confirms "identical stamps per frame pair".
- Depth invalid-pixel convention (sampled 5 depth frames): BOTH `nan`
  (~8-9% of pixels) and `+inf` (~18-19% of pixels) appear in the same
  frame; finite values ranged over [0.60, 9.9999] m across all 5
  samples (an effective ~10 m cutoff in this deployment). DERIVED for
  the sim: Isaac's `distance_to_image_plane` annotator already returns
  `+inf` for "no geometry hit" pixels (its native "invalid" value), so
  we pass that through unchanged rather than trying to reproduce the
  nan/inf *split* -- any `isfinite()`-gated depth consumer (Hydra) only
  needs "invalid pixels are non-finite", which this satisfies.

### BAG-MEASURED + calibration.yaml: static TF chain to the optical frame

`platforms/topaz/calibration.yaml:3-11` already defines (as a
`transform_file_broadcaster` running in the real launch, unmodified by
this task):

    {name}/frontleft -> {name}_zed_camera_link

The remaining hops, all BAG-MEASURED from `/tf_static` (this bag), none
of which anything in our sim's launch will publish:

    {name}_zed_camera_link -> {name}_zed_camera_center   (t=(0,0,0.015), identity rotation)
    {name}_zed_camera_center -> {name}_zed_left_camera_frame  (t=(-0.01,0.06,0), identity rotation)
    {name}_zed_left_camera_frame -> {name}_zed_left_camera_optical_frame  (t=0, q=(0.5,-0.5,0.5,-0.5), the standard ROS optical-frame axis flip)

These three are, on real hardware, published internally by the
zed-ros2-wrapper's camera model (independent of its `publish_tf`
launch arg, which only gates the wrapper's *dynamic* map/odom
tracking TF) -- see Step-3 note below for why nothing in *our* launch
publishes them and `ros_bridge.py` must.

### CORRECTION to the task brief's Step 3 assumption: `body -> frontleft` is NOT from the URDF

The brief assumed `{name}/body -> {name}/frontleft` comes from
`robot_state_publisher` + `spot_tools_ros/urdf/spot.urdf.xacro` (i.e.
"already publishing since Task 7's joint_states"). Checked directly:
`grep -n -E "frontleft|\"head\"" spot_tools_ros/urdf/spot_macro.xacro`
returns nothing -- the xacro has no `head`/`frontleft`/`frontright`
links or joints at all. In the real bag, `{name}/body -> {name}/head`
(identity transform) and `{name}/head -> {name}/frontleft` are both in
`/tf_static`, i.e. **static**, not from the URDF -- they come from
Spot's own onboard camera-frame calibration (published once at startup
by the real `spot_tools_ros` driver from the bosdyn frame-tree
snapshot, not modeled anywhere in this sim). Since `body -> head` is
the identity, `body -> frontleft` == the bag's `head -> frontleft`
transform directly, which is what `BODY_TO_FRONTLEFT_XYZ`/`_QUAT_XYZW`
below hardcodes. **`ros_bridge.py` publishes this static transform too**
(nothing else in our stack will) -- flagged for Task 11's launch
wiring in the task-8 report.

### DERIVED items (Task 12 must validate live)

- `DERIVED_RESOLUTION = (960, 540)`: the brief's original guess from
  `zed_overrides.yaml`'s `pub_downscale_factor: 2.0` assuming HD1080
  native -- unused (bag measurement above wins) but kept as a
  documented paper trail.
- Depth clipping range fed to the Isaac camera sensor (`CLIP_NEAR_M`/
  `CLIP_FAR_M`): not in the bag (no way to probe the underlying
  hardware's clip planes from recorded topic data) -- chosen close to
  the bag's observed finite range [0.6, 10.0] m padded slightly.
- RGB rendering does not reproduce ZED-specific ISP behavior (auto
  exposure/white balance, NEURAL_PLUS depth-mode artifacts, lens
  vignetting) -- Isaac's path-traced/RTX render is a *different*
  image-formation model entirely; only geometry/intrinsics/contract
  shape are matched, not radiometric appearance.
"""
from __future__ import annotations

import logging
import math

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# BAG-MEASURED: image / camera_info contract (see module docstring).
# ---------------------------------------------------------------------------
RGB_ENCODING = "bgra8"
DEPTH_ENCODING = "32FC1"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
DISTORTION_MODEL = "rational_polynomial"
DISTORTION_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]
FX = 261.9303894042969
FY = 261.9303894042969
CX = 336.3063049316406
CY = 167.84288024902344

# Unused fallback -- see "DERIVED items" in the module docstring.
DERIVED_RESOLUTION = (960, 540)

CAMERA_HZ = 15.0

# DERIVED (not in the bag): Isaac camera clip planes, padded around the
# bag's observed finite-depth range [0.60, 10.0] m.
CLIP_NEAR_M = 0.15
CLIP_FAR_M = 15.0


def rgb_optical_frame(name: str) -> str:
    """BAG-MEASURED frame_id pattern: '{name}_zed_left_camera_optical_frame'."""
    return f"{name}_zed_left_camera_optical_frame"


# Both rgb and depth share the same optical frame_id in the bag (both
# are already registered/rectified into the left-camera optical frame).
depth_optical_frame = rgb_optical_frame


def zed_camera_link_frame(name: str) -> str:
    return f"{name}_zed_camera_link"


def zed_camera_center_frame(name: str) -> str:
    return f"{name}_zed_camera_center"


def zed_left_camera_frame(name: str) -> str:
    return f"{name}_zed_left_camera_frame"


def frontleft_frame(name: str) -> str:
    return f"{name}/frontleft"


# ---------------------------------------------------------------------------
# Static TF chain, body -> optical (see module docstring for provenance
# of each hop). All quaternions are ROS-convention scalar-last (x,y,z,w).
# ---------------------------------------------------------------------------

# {name}/body -> {name}/frontleft. BAG-MEASURED (this hop is NOT from the
# URDF -- see module docstring's "CORRECTION" section). Numerically
# equal to the bag's {name}/head -> {name}/frontleft transform because
# body -> head is the identity in the bag.
BODY_TO_FRONTLEFT_XYZ = (0.41614198818571146, 0.03876606197968771, 0.024440234358514057)
BODY_TO_FRONTLEFT_QUAT_XYZW = (
    0.1443657903462012, 0.8085566913365266, -0.22535380744755173, 0.5240326869018166,
)

# {name}/frontleft -> {name}_zed_camera_link. From
# platforms/topaz/calibration.yaml:3-11 (unmodified source of truth --
# reproduced here only so camera.py can compute the mount extrinsic;
# the actual static TF for this hop is published by the real launch's
# calibration_publisher, NOT by ros_bridge.py).
FRONTLEFT_TO_ZED_LINK_XYZ = (-0.1377625424350767, 0.021380403878896944, 0.057657655316766976)
FRONTLEFT_TO_ZED_LINK_QUAT_XYZW = (
    -0.1518423673416495, -0.7689097936324478, 0.20560492808050715, 0.5860445702207298,
)

# {name}_zed_camera_link -> {name}_zed_camera_center. BAG-MEASURED
# (/tf_static). Published by ros_bridge.py (see module docstring).
ZED_LINK_TO_CENTER_XYZ = (0.0, 0.0, 0.015)
ZED_LINK_TO_CENTER_QUAT_XYZW = (0.0, 0.0, 0.0, 1.0)

# {name}_zed_camera_center -> {name}_zed_left_camera_frame.
# BAG-MEASURED (/tf_static). Published by ros_bridge.py.
CENTER_TO_LEFT_FRAME_XYZ = (-0.01, 0.06, 0.0)
CENTER_TO_LEFT_FRAME_QUAT_XYZW = (0.0, 0.0, 0.0, 1.0)

# {name}_zed_left_camera_frame -> {name}_zed_left_camera_optical_frame.
# BAG-MEASURED (/tf_static); the standard ROS camera axis flip
# (optical +Z = look direction, +X = image-right, +Y = image-down).
# Published by ros_bridge.py.
LEFT_FRAME_TO_OPTICAL_XYZ = (0.0, 0.0, 0.0)
LEFT_FRAME_TO_OPTICAL_QUAT_XYZW = (0.5, -0.4999999999999999, 0.5, -0.5000000000000001)


def _quat_mul_xyzw(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def _quat_rotate_xyzw(q, v):
    x, y, z, w = q
    qv = np.array([x, y, z])
    uv = np.cross(qv, v)
    uuv = np.cross(qv, uv)
    return v + 2.0 * (w * uv + uuv)


def _compose(t1, q1, t2, q2):
    """Compose two (translation, quat_xyzw) transforms: result = T1 * T2."""
    t1 = np.asarray(t1, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    t2 = np.asarray(t2, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    t = t1 + _quat_rotate_xyzw(q1, t2)
    q = _quat_mul_xyzw(q1, q2)
    q = q / np.linalg.norm(q)
    return t, q


def body_to_optical_extrinsic():
    """Compose the full body -> optical static chain (see module docstring).

    Single source of truth: composes the five hardcoded hop constants
    above at call time rather than hardcoding the composed result, so
    the two can never drift apart. Returns
    `(translation_xyz: np.ndarray[3], quat_xyzw: np.ndarray[4])`.
    """
    t, q = BODY_TO_FRONTLEFT_XYZ, BODY_TO_FRONTLEFT_QUAT_XYZW
    for tn, qn in (
        (FRONTLEFT_TO_ZED_LINK_XYZ, FRONTLEFT_TO_ZED_LINK_QUAT_XYZW),
        (ZED_LINK_TO_CENTER_XYZ, ZED_LINK_TO_CENTER_QUAT_XYZW),
        (CENTER_TO_LEFT_FRAME_XYZ, CENTER_TO_LEFT_FRAME_QUAT_XYZW),
        (LEFT_FRAME_TO_OPTICAL_XYZ, LEFT_FRAME_TO_OPTICAL_QUAT_XYZW),
    ):
        t, q = _compose(t, q, tn, qn)
    return t, q


class SimZedCamera:
    """One Isaac RGB-D camera sensor, mounted at the ZED extrinsic.

    Mounted as a child prim of the robot's root prim (`robot.prim_path`)
    so that `SpotSimRobot._write_pose_to_stage()`'s per-step root-pose
    writeback carries this camera along for free via USD parent-child
    xform composition -- the same trick `spot_robot.py` already uses
    for leg/arm links and the gripper (see its module docstring).
    """

    PRIM_RELATIVE_PATH = "zed_camera"

    def __init__(self, robot):
        # Deferred imports: isaacsim.* only exists after SimulationApp
        # has booted (see dcist_sim_isaac/README.md).
        from isaacsim.sensors.camera import Camera

        self.robot = robot
        self.name = robot.spec.name
        self.prim_path = f"{robot.prim_path}/{self.PRIM_RELATIVE_PATH}"

        self._camera = Camera(
            prim_path=self.prim_path,
            name=f"{self.name}_zed_camera",
            resolution=(IMAGE_WIDTH, IMAGE_HEIGHT),
        )

        translation, quat_xyzw = body_to_optical_extrinsic()
        # Camera.set_local_pose's orientation is scalar-first (w,x,y,z)
        # -- do not confuse with the scalar-last (x,y,z,w) convention
        # used everywhere else in this file / ros_bridge.py. camera_axes
        # ="ros" tells Isaac to interpret the quaternion as a standard
        # ROS optical-frame rotation (+Z look / +X image-right / +Y
        # image-down), i.e. exactly the convention our composed static
        # TF chain already uses -- no extra axis juggling needed.
        x, y, z, w = quat_xyzw
        self._camera.set_local_pose(
            translation=translation,
            orientation=np.array([w, x, y, z], dtype=float),
            camera_axes="ros",
        )
        self._camera.set_clipping_range(CLIP_NEAR_M, CLIP_FAR_M)

        self._initialized = False

    def initialize(self) -> None:
        """Call once after `world.reset()` (needs a valid physics sim view)."""
        self._camera.initialize()
        self._camera.set_opencv_pinhole_properties(cx=CX, cy=CY, fx=FX, fy=FY)
        self._camera.add_distance_to_image_plane_to_frame()
        self._initialized = True
        logger.info(
            "mounted ZED sim camera for '%s' at %s (resolution=%dx%d)",
            self.name, self.prim_path, IMAGE_WIDTH, IMAGE_HEIGHT,
        )

    def get_frame(self):
        """Return `(rgba_uint8[H,W,4], distance_to_image_plane_f32[H,W])` or `None`.

        `None` means the renderer hasn't produced a frame yet (e.g. the
        first few ticks after `initialize()` while render products warm
        up) -- callers should skip publishing that step rather than
        publish garbage/zero-filled frames.
        """
        if not self._initialized:
            return None
        frame = self._camera.get_current_frame()
        # NOTE: the RGBA image lives under the dict key "rgb" (not
        # "rgba") -- verified empirically 2026-07-04 against the
        # installed 6.0.1.0 `isaacsim.sensors.camera.Camera`
        # (`initialize(attach_rgb_annotator=True)` attaches an
        # annotator *named* "rgb" whose `get_data()` actually returns
        # (H, W, 4) uint8 RGBA, matching `Camera.get_rgba()`'s own
        # implementation, which reads `self._custom_annotators["rgb"]`).
        # Passing "rgba" here (the seemingly-obvious key) silently
        # returns None forever -- cost real debugging time to find, so
        # documenting it prominently.
        rgba = frame.get("rgb")
        depth = frame.get("distance_to_image_plane")
        if rgba is None or depth is None or rgba.size == 0 or depth.size == 0:
            return None
        return rgba, depth


def rgba_to_bgra(rgba: np.ndarray) -> np.ndarray:
    """Isaac's rgb annotator returns RGBA uint8; ZED's bgra8 needs BGRA."""
    return rgba[..., [2, 1, 0, 3]]


def make_camera_info_msg(frame_id: str, stamp) -> "sensor_msgs.msg.CameraInfo":
    from sensor_msgs.msg import CameraInfo

    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = IMAGE_HEIGHT
    msg.width = IMAGE_WIDTH
    msg.distortion_model = DISTORTION_MODEL
    msg.d = list(DISTORTION_COEFFS)
    msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg
