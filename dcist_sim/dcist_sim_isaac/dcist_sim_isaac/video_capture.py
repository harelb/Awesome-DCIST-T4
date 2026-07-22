"""Third-person video capture for dcist_sim_isaac (JEG Task 1).

A watchable, user-facing third-person clip of whatever the sim is doing:
a static camera framed behind the robot, looking over it at the midpoint
between the robot and the objects it is working near. Frames are grabbed
rate-gated (default 24 fps), written as JPEGs, and encoded to a single
`capture.mp4` with ffmpeg on `close()`.

Two module-scope helpers are pure python (stdlib/math only, pytest-covered):
`third_person_pose` (camera geometry) and `RateGate` (drift-free frame
timing). The `VideoCapture` class is Isaac-only -- every `isaacsim`/`omni`
import is deferred inside a method (same contract as gt_capture.py), so this
module imports cleanly in a plain pytest env.

Never-fatal contract (spec, mirrors sim_app's GT handling): no public method
of `VideoCapture` may raise into the sim loop. A failed capture/attach is
logged and swallowed; a failed ffmpeg encode leaves the JPEG frames on disk
(nothing is lost) and `close()` returns None.
"""
import logging
import math
import os
import shutil
import subprocess

logger = logging.getLogger(__name__)

# Look-at height (m): eye level of the action, matches the sim camera height
# used elsewhere (render_gate CAMERA_HEIGHT_M) so the robot body is centered.
LOOK_AT_Z = 0.5


def third_person_pose(robot_xy, objects_centroid_xy, back_m=3.5, up_m=2.0):
    """Camera position + look-at for a static third-person framing.

    The camera looks at the midpoint between the robot and the objects'
    centroid (at z=`LOOK_AT_Z`), sitting `back_m` behind the robot along the
    robot->midpoint ray (so the robot lies between the camera and the
    look-at point -- "looking over the robot" at the scene) and `up_m` above
    the ground.

    Degenerate case (robot_xy == objects_centroid_xy, i.e. no objects): the
    robot->midpoint direction is undefined, so fall back to a +X offset --
    the camera sits back_m toward -X and looks toward +X at the robot.

    Pure math. Returns ``((px, py, pz), (lx, ly, lz))``.
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    cx, cy = float(objects_centroid_xy[0]), float(objects_centroid_xy[1])

    lx, ly = (rx + cx) / 2.0, (ry + cy) / 2.0
    lz = LOOK_AT_Z

    # Direction robot -> midpoint (== robot -> centroid direction).
    dx, dy = lx - rx, ly - ry
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        ux, uy = 1.0, 0.0  # degenerate: +X fallback
    else:
        ux, uy = dx / norm, dy / norm

    px = lx - back_m * ux
    py = ly - back_m * uy
    pz = up_m
    return (px, py, pz), (lx, ly, lz)


class RateGate:
    """Absolute-timestamp, remainder-carrying frame-rate gate.

    ``.ready(sim_t)`` returns True at most once per ``1/fps`` window. This is
    an ABSOLUTE-TIMESTAMP schedule gate, the same family as gt_capture's
    ``_next_t`` next-emit-time comparison (NOT ros_bridge's dt-accumulator,
    which sums per-frame dt and carries a remainder). The one refinement over
    gt_capture: it advances ``_next`` by exactly one period per emit
    (phase-locked to the first call) instead of re-anchoring to the current
    ``sim_t``, so over a long fine-grained timestamp stream there is zero
    accumulated drift. After a stall (sim_t jumps more than a period ahead) the
    gate emits once and resyncs rather than machine-gunning catch-up frames.
    """

    def __init__(self, fps):
        self.period = 1.0 / float(fps)
        self._next = None

    def ready(self, sim_t):
        if self._next is None:
            self._next = sim_t + self.period
            return True
        # small epsilon so a timestamp landing exactly on the boundary emits
        if sim_t + 1e-9 < self._next:
            return False
        self._next += self.period
        if self._next <= sim_t:
            # fell more than one period behind (long stall): resync to now so
            # we resume the cadence instead of firing every subsequent tick.
            self._next = sim_t + self.period
        return True


class VideoCapture:
    """Static third-person RGB video recorder (Isaac-only).

    Creates its own camera prim + Replicator render product + "rgb" annotator
    (gt_capture's annotator family), grabs rate-gated JPEG frames, and encodes
    them to ``<out_dir>/capture.mp4`` via ffmpeg on ``close()``. All public
    methods are never-fatal (see module docstring).
    """

    PRIM_PATH = "/World/third_person_cam"
    # 720p: watchable, and cheap enough not to drag the sim.
    WIDTH = 1280
    HEIGHT = 720

    def __init__(self, out_dir, fps, camera_pose):
        """`camera_pose` is ``((px,py,pz), (lx,ly,lz))`` from third_person_pose.

        NOT never-fatal: `os.makedirs` below can raise (e.g. `out_dir` collides
        with an existing FILE, or an unwritable path). That is deliberate -- a
        bad `--video-out` is an operator error worth surfacing -- so the CALL
        SITE (sim_app) wraps construction in try/except and degrades to
        video=None, matching how the rest of the video path fails soft. (For
        the record: GtCapture.__init__ has this same pre-existing gap in its
        `os.makedirs` + `open(manifest)`; NOT fixed here -- out of scope.)
        """
        self._out = out_dir
        self._fps = fps
        self._pose = camera_pose
        self._gate = RateGate(fps)
        self._annotator = None
        self._index = 0
        os.makedirs(out_dir, exist_ok=True)

    def attach(self):
        """Create the camera prim + render product + rgb annotator.

        Never raises: on any failure the capture is disabled (self._annotator
        stays None) and the sim continues.
        """
        try:
            import omni.replicator.core as rep

            (px, py, pz), (lx, ly, lz) = self._pose
            camera = rep.create.camera(position=(px, py, pz), look_at=(lx, ly, lz))
            rp = rep.create.render_product(camera, (self.WIDTH, self.HEIGHT))
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach([rp])
            self._annotator = ann
            logger.info(
                "video_capture: third-person camera at (%.2f, %.2f, %.2f) "
                "look-at (%.2f, %.2f, %.2f), %dx%d @ %g fps -> %s",
                px, py, pz, lx, ly, lz, self.WIDTH, self.HEIGHT, self._fps,
                self._out,
            )
        except Exception:  # noqa: BLE001 -- capture must never kill the sim
            logger.exception(
                "video_capture: attach failed; disabling video for this run")
            self._annotator = None

    def maybe_capture(self, sim_t):
        """Rate-gated frame write. Returns True iff a frame was written.

        Never raises. A None/empty annotator payload (render warm-up) does not
        consume the rate slot -- retried next frame, same as gt_capture.
        """
        if self._annotator is None:
            return False
        try:
            if not self._gate.ready(sim_t):
                return False
            data = self._annotator.get_data()
            if getattr(data, "ndim", 0) != 3 or getattr(data, "shape", (0,))[0] == 0:
                # annotator buffer not populated yet (post-attach warm-up):
                # don't burn the rate slot, retry next frame. This rewind
                # assumes `_gate.ready()` returned True on THIS call and did
                # exactly one `_next += period` (its only advance) -- so
                # subtracting one period restores the pre-call schedule
                # precisely. Valid because ready() is called once per
                # maybe_capture and only reached here after it returned True;
                # the rewind is inert if the same warm-up frame keeps failing
                # (it just keeps re-offering the slot).
                self._gate._next -= self._gate.period
                return False
            import imageio.v2 as imageio

            fn = os.path.join(self._out, f"frame_{self._index:06d}.jpg")
            imageio.imwrite(fn, data[:, :, :3])
            self._index += 1
            return True
        except Exception:  # noqa: BLE001 -- capture must never kill the sim
            logger.exception(
                "video_capture: frame capture failed; disabling video")
            self._annotator = None
            return False

    def close(self):
        """Encode captured JPEGs to <out_dir>/capture.mp4 via ffmpeg.

        Returns the mp4 path on success, or None if ffmpeg is absent or the
        encode failed (the JPEG frames are always left on disk either way).
        Never raises.
        """
        if self._index == 0:
            logger.warning("video_capture: no frames captured; nothing to encode")
            return None
        if shutil.which("ffmpeg") is None:
            logger.warning(
                "video_capture: ffmpeg not found on PATH; %d JPEG frames kept "
                "in %s (encode skipped)", self._index, self._out)
            return None
        out_mp4 = os.path.join(self._out, "capture.mp4")
        cmd = [
            "ffmpeg", "-y",
            "-framerate", str(self._fps),
            "-i", os.path.join(self._out, "frame_%06d.jpg"),
            "-pix_fmt", "yuv420p",
            out_mp4,
        ]
        try:
            proc = subprocess.run(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
            if proc.returncode != 0:
                logger.error(
                    "video_capture: ffmpeg encode failed (rc=%d); %d JPEG "
                    "frames kept in %s\n%s", proc.returncode, self._index,
                    self._out, proc.stderr.decode(errors="replace")[-2000:])
                return None
            logger.info("video_capture: encoded %d frames -> %s",
                        self._index, out_mp4)
            return out_mp4
        except Exception:  # noqa: BLE001 -- close must never raise
            logger.exception(
                "video_capture: ffmpeg invocation raised; %d JPEG frames kept "
                "in %s", self._index, self._out)
            return None
