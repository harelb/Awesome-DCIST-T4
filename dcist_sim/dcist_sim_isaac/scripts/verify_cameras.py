#!/usr/bin/env python3
"""Verify ZED-shaped camera publishing end-to-end over ROS (task-8-brief.md Step 4).

Plain rclpy -- run OUTSIDE the Isaac venv/process, against a sim_app.py
already running in another terminal (see camera.py's module docstring
for the full contract this checks against):

  source /opt/ros/jazzy/setup.zsh
  source ~/dcist_ws/install/setup.zsh
  python3 dcist_sim/dcist_sim_isaac/scripts/verify_cameras.py [robot_name]

(robot_name defaults to "hilbert", matching field_smoke.yaml.)

The frame_id/encoding/resolution constants below are duplicated (not
imported) from `camera.py` on purpose, matching `verify_robot_motion.py`'s
existing convention of being a fully self-contained plain-rclpy script
runnable without `dcist_sim_isaac` on `PYTHONPATH` -- if they ever drift
from `camera.py`'s actual constants, that drift itself is a bug this
script should catch (a hardcoded, independently-derived expectation is
the point of an external verification script).

Checks:
  1. All 4 topics (rgb/image_rect_color, rgb/camera_info,
     depth/depth_registered, depth/camera_info) publish at >= 14 Hz
     over an 8 s sampling window.
  2. RGB and depth image messages pair up with byte-identical stamps
     (`message_filters.TimeSynchronizer`, `exact` match).
  3. header.frame_id on every one of the 4 topics equals the expected
     ZED optical frame name.
  4. The depth image's center pixel is finite and in [0.2, 10.0] m
     (the task brief's smoke-scene sanity bound).
  5. Both camera_info messages have a nonzero K matrix (fx, fy != 0).

Prints PASS/FAIL per check and exits 0 iff all pass.
"""
import sys
import time

import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image

MIN_HZ = 14.0
SAMPLE_WINDOW_S = 8.0
DEPTH_MIN_M = 0.2
DEPTH_MAX_M = 10.0
EXPECTED_ENCODING_RGB = "bgra8"
EXPECTED_ENCODING_DEPTH = "32FC1"


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else "hilbert"
    expected_frame_id = f"{robot}_zed_left_camera_optical_frame"
    ns = f"/{robot}/{robot}_zed"

    rclpy.init()
    node = rclpy.create_node("verify_cameras")

    stamps = {"rgb": [], "depth": [], "rgb_info": [], "depth_info": []}
    latest = {}

    def mk_cb(key):
        def cb(msg):
            stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            latest[key] = msg
        return cb

    node.create_subscription(Image, f"{ns}/rgb/image_rect_color", mk_cb("rgb"), 10)
    node.create_subscription(Image, f"{ns}/depth/depth_registered", mk_cb("depth"), 10)
    node.create_subscription(CameraInfo, f"{ns}/rgb/camera_info", mk_cb("rgb_info"), 10)
    node.create_subscription(CameraInfo, f"{ns}/depth/camera_info", mk_cb("depth_info"), 10)

    # Separate exact-time-sync subscription pair to check stamp equality
    # (check 2) without disturbing the plain rate-counting subscriptions
    # above.
    synced_pairs = []
    rgb_sync_sub = Subscriber(node, Image, f"{ns}/rgb/image_rect_color")
    depth_sync_sub = Subscriber(node, Image, f"{ns}/depth/depth_registered")
    sync = TimeSynchronizer([rgb_sync_sub, depth_sync_sub], queue_size=30)
    sync.registerCallback(lambda rgb_msg, depth_msg: synced_pairs.append((rgb_msg, depth_msg)))

    print(f"Sampling {ns} camera topics for {SAMPLE_WINDOW_S}s ...")
    deadline = time.monotonic() + SAMPLE_WINDOW_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    all_ok = True

    def report(ok, message):
        nonlocal all_ok
        all_ok = all_ok and ok
        print(f"[{'PASS' if ok else 'FAIL'}] {message}")

    # --- Check 1: rate ------------------------------------------------------
    for key, topic in [
        ("rgb", f"{ns}/rgb/image_rect_color"),
        ("depth", f"{ns}/depth/depth_registered"),
        ("rgb_info", f"{ns}/rgb/camera_info"),
        ("depth_info", f"{ns}/depth/camera_info"),
    ]:
        n = len(stamps[key])
        hz = (n - 1) / (stamps[key][-1] - stamps[key][0]) if n >= 2 else 0.0
        report(hz >= MIN_HZ, f"{topic}: {hz:.2f} Hz over {n} msgs (>= {MIN_HZ} Hz)")

    # --- Check 2: RGB/depth stamp equality -----------------------------------
    report(
        len(synced_pairs) > 0,
        f"RGB/depth stamps identical per frame pair: {len(synced_pairs)} exact-stamp "
        f"pairs matched by message_filters.TimeSynchronizer",
    )

    # --- Check 3: frame_id --------------------------------------------------
    for key, topic in [
        ("rgb", "rgb/image_rect_color"), ("depth", "depth/depth_registered"),
        ("rgb_info", "rgb/camera_info"), ("depth_info", "depth/camera_info"),
    ]:
        msg = latest.get(key)
        ok = msg is not None and msg.header.frame_id == expected_frame_id
        got = None if msg is None else msg.header.frame_id
        report(ok, f"{topic} frame_id == '{expected_frame_id}' (got '{got}')")

    # --- Check 3b: encodings (bonus, cheap to check while we're here) -------
    rgb_msg = latest.get("rgb")
    if rgb_msg is not None:
        report(
            rgb_msg.encoding == EXPECTED_ENCODING_RGB,
            f"rgb encoding == '{EXPECTED_ENCODING_RGB}' (got '{rgb_msg.encoding}')",
        )
    depth_msg = latest.get("depth")
    if depth_msg is not None:
        report(
            depth_msg.encoding == EXPECTED_ENCODING_DEPTH,
            f"depth encoding == '{EXPECTED_ENCODING_DEPTH}' (got '{depth_msg.encoding}')",
        )

    # --- Check 4: depth center pixel ----------------------------------------
    if depth_msg is not None:
        arr = np.frombuffer(bytes(depth_msg.data), dtype=np.float32).reshape(
            depth_msg.height, depth_msg.width
        )
        cy, cx = depth_msg.height // 2, depth_msg.width // 2
        center = float(arr[cy, cx])
        ok = np.isfinite(center) and DEPTH_MIN_M <= center <= DEPTH_MAX_M
        report(
            ok,
            f"depth center pixel = {center:.3f} m, finite and in "
            f"[{DEPTH_MIN_M}, {DEPTH_MAX_M}] m",
        )
    else:
        report(False, "depth center pixel: no depth message received")

    # --- Check 5: camera_info K nonzero -------------------------------------
    for key, topic in [("rgb_info", "rgb/camera_info"), ("depth_info", "depth/camera_info")]:
        msg = latest.get(key)
        ok = msg is not None and msg.k[0] != 0.0 and msg.k[4] != 0.0
        report(ok, f"{topic}: K fx={None if msg is None else msg.k[0]}, "
                   f"fy={None if msg is None else msg.k[4]} both nonzero")

    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
