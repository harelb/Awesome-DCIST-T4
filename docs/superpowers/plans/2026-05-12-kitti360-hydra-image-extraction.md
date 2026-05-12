# KITTI-360 Standalone Hydra + Image Extraction — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert raw KITTI-360 data to a ROS2 MCAP bag and run it through Hydra to produce scene graphs with object crop images and agent-pose images, using a standalone tmuxp session (not integrated into the ADT4 experiment manifest).

**Architecture:** A Python converter (`kitti360_to_mcap.py`) reads KITTI-360 raw files, synthesizes depth by projecting LiDAR onto the camera plane, and writes an MCAP bag. A standalone tmuxp session launches Hydra with two config files — one configuring the ROS input receiver, one setting outdoor-scale hydra parameters with image extraction enabled. A shell wrapper sets env vars and loads the session.

**Tech Stack:** Python 3, `rosbags`, `numpy`, `opencv-python`, `scipy`; YAML (hydra config); tmuxp (session launch); ROS2 Humble.

---

## File Structure

```
scripts/kitti360/
  kitti360_to_mcap.py          # converter: CLI + all conversion logic
  kitti360_ros_input.yaml      # hydra_ros input config (RGBDImageReceiver)
  kitti360_hydra.yaml          # core hydra params (outdoor scale + image extraction)
  kitti360_session.yaml        # tmuxp session (3 windows)
  run_kitti360.sh              # wrapper: sets env vars, runs tmuxp
  tests/
    test_kitti360_to_mcap.py   # all unit + integration tests
    fixtures/
      perspective.txt          # sample P_rect_00 calibration
      calib_cam_to_velo.txt    # sample R, T extrinsics
      poses.txt                # 3 sample 4×4 world→cam0 poses
      timestamps.txt           # 3 sample timestamps
```

---

## Task 1: Create test fixtures

**Files:**
- Create: `scripts/kitti360/tests/__init__.py`
- Create: `scripts/kitti360/tests/fixtures/perspective.txt`
- Create: `scripts/kitti360/tests/fixtures/calib_cam_to_velo.txt`
- Create: `scripts/kitti360/tests/fixtures/poses.txt`
- Create: `scripts/kitti360/tests/fixtures/timestamps.txt`

- [ ] **Step 1: Create the directory structure**

```bash
mkdir -p /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360/tests/fixtures
touch /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360/tests/__init__.py
```

All pytest commands in this plan must be run from `scripts/kitti360/` so that `from kitti360_to_mcap import ...` resolves correctly:

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360
```

- [ ] **Step 2: Write `tests/fixtures/perspective.txt`**

These are real KITTI-360 sequence 00 calibration values:

```
P_rect_00: 552.554261 0.000000 682.049453 0.000000 0.000000 552.554261 238.769549 0.000000 0.000000 0.000000 1.000000 0.000000
P_rect_01: 552.554261 0.000000 682.049453 -328.318735 0.000000 552.554261 238.769549 0.000000 0.000000 0.000000 1.000000 0.000000
```

- [ ] **Step 3: Write `tests/fixtures/calib_cam_to_velo.txt`**

```
R: 0.04307104361 -0.08829286498 0.99517480507 -0.99004970380 0.12436960487 0.05378698768 -0.13118052683 -0.98794407235 -0.08210784020
T: -0.01198459927 -0.05403984174 -0.27218827498
```

- [ ] **Step 4: Write `tests/fixtures/poses.txt`**

Three poses: identity, 1 m forward, 2 m forward (4×4 row-major, T_world_cam0):

```
1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0
1.0 0.0 0.0 1.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0
1.0 0.0 0.0 2.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0
```

- [ ] **Step 5: Write `tests/fixtures/timestamps.txt`**

```
2013-05-28 09:31:16.479931000+00:00
2013-05-28 09:31:16.579931000+00:00
2013-05-28 09:31:16.679931000+00:00
```

- [ ] **Step 6: Install Python dependencies**

```bash
pip install rosbags numpy opencv-python scipy
```

Expected: all install without errors (or already present).

- [ ] **Step 7: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/
git commit -m "feat: add kitti360 conversion scaffolding and test fixtures"
```

---

## Task 2: Calibration parsing

**Files:**
- Create: `scripts/kitti360/kitti360_to_mcap.py` (calibration functions only)
- Test: `scripts/kitti360/tests/test_kitti360_to_mcap.py`

- [ ] **Step 1: Write the failing tests**

Create `scripts/kitti360/tests/test_kitti360_to_mcap.py`:

```python
from pathlib import Path
import numpy as np
import pytest

FIXTURES = Path(__file__).parent / 'fixtures'


def test_parse_perspective_returns_3x4():
    from kitti360_to_mcap import parse_perspective
    P = parse_perspective(FIXTURES / 'perspective.txt')
    assert P.shape == (3, 4)


def test_parse_perspective_values():
    from kitti360_to_mcap import parse_perspective
    P = parse_perspective(FIXTURES / 'perspective.txt')
    assert abs(P[0, 0] - 552.554261) < 1e-4   # fx
    assert abs(P[0, 2] - 682.049453) < 1e-4   # cx
    assert P[0, 3] == 0.0                      # no baseline for cam0


def test_parse_cam_to_velo_shapes():
    from kitti360_to_mcap import parse_cam_to_velo
    R, T = parse_cam_to_velo(FIXTURES / 'calib_cam_to_velo.txt')
    assert R.shape == (3, 3)
    assert T.shape == (3,)


def test_parse_cam_to_velo_R_is_rotation():
    from kitti360_to_mcap import parse_cam_to_velo
    R, _ = parse_cam_to_velo(FIXTURES / 'calib_cam_to_velo.txt')
    assert abs(np.linalg.det(R) - 1.0) < 1e-6
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360
python -m pytest tests/test_kitti360_to_mcap.py::test_parse_perspective_returns_3x4 -v
```

Expected: `ModuleNotFoundError: No module named 'kitti360_to_mcap'`

- [ ] **Step 3: Create `kitti360_to_mcap.py` with the two parsing functions**

```python
#!/usr/bin/env python3
"""Convert raw KITTI-360 data to a ROS2 MCAP bag."""

from pathlib import Path
import numpy as np


def parse_perspective(calib_path: Path) -> np.ndarray:
    """Return P_rect_00 as (3, 4) float64 array."""
    with open(calib_path) as f:
        for line in f:
            if line.startswith('P_rect_00:'):
                vals = [float(v) for v in line.split(':', 1)[1].split()]
                return np.array(vals, dtype=np.float64).reshape(3, 4)
    raise ValueError(f'P_rect_00 not found in {calib_path}')


def parse_cam_to_velo(calib_path: Path) -> tuple:
    """
    Parse calib_cam_to_velo.txt.

    Returns R (3, 3), T (3,) such that P_velo = R @ P_cam + T
    (i.e. transforms a point from cam0 frame into velodyne frame).
    """
    data: dict = {}
    with open(calib_path) as f:
        for line in f:
            if ':' in line:
                key, vals = line.split(':', 1)
                data[key.strip()] = [float(v) for v in vals.split()]
    R = np.array(data['R'], dtype=np.float64).reshape(3, 3)
    T = np.array(data['T'], dtype=np.float64)
    return R, T
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "calibration or perspective or cam_to_velo" -v
```

Expected: 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_to_mcap.py scripts/kitti360/tests/test_kitti360_to_mcap.py
git commit -m "feat: add calibration parsing for KITTI-360 converter"
```

---

## Task 3: Pose and timestamp parsing

**Files:**
- Modify: `scripts/kitti360/kitti360_to_mcap.py` (add two functions)
- Modify: `scripts/kitti360/tests/test_kitti360_to_mcap.py` (add tests)

- [ ] **Step 1: Add failing tests**

Append to `tests/test_kitti360_to_mcap.py`:

```python
def test_parse_poses_count():
    from kitti360_to_mcap import parse_poses
    poses = parse_poses(FIXTURES / 'poses.txt')
    assert len(poses) == 3


def test_parse_poses_shape():
    from kitti360_to_mcap import parse_poses
    poses = parse_poses(FIXTURES / 'poses.txt')
    assert all(p.shape == (4, 4) for p in poses)


def test_parse_poses_first_is_identity():
    from kitti360_to_mcap import parse_poses
    poses = parse_poses(FIXTURES / 'poses.txt')
    assert np.allclose(poses[0], np.eye(4))


def test_parse_poses_second_has_translation():
    from kitti360_to_mcap import parse_poses
    poses = parse_poses(FIXTURES / 'poses.txt')
    assert abs(poses[1][0, 3] - 1.0) < 1e-9   # 1 m in x


def test_parse_timestamps_count():
    from kitti360_to_mcap import parse_timestamps
    stamps = parse_timestamps(FIXTURES / 'timestamps.txt')
    assert len(stamps) == 3


def test_parse_timestamps_are_nanoseconds():
    from kitti360_to_mcap import parse_timestamps
    stamps = parse_timestamps(FIXTURES / 'timestamps.txt')
    # 2013 timestamps should be > 1e18 ns (year 2000 = ~9.46e17)
    assert all(s > 1_000_000_000_000_000_000 for s in stamps)


def test_parse_timestamps_monotonic():
    from kitti360_to_mcap import parse_timestamps
    stamps = parse_timestamps(FIXTURES / 'timestamps.txt')
    assert stamps[1] > stamps[0]
    assert stamps[2] > stamps[1]
```

- [ ] **Step 2: Run to confirm failure**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "poses or timestamps" -v
```

Expected: `ImportError` (functions not defined yet).

- [ ] **Step 3: Add `parse_poses` and `parse_timestamps` to `kitti360_to_mcap.py`**

Add after `parse_cam_to_velo`:

```python
from datetime import datetime, timezone


def parse_poses(poses_path: Path) -> list:
    """
    Parse poses.txt. Each line is 16 whitespace-separated floats (4×4 row-major).
    Also accepts 12-value lines (3×4), padded to 4×4.
    Returns list of (4, 4) float64 arrays — T_world_cam0.
    """
    poses = []
    with open(poses_path) as f:
        for line in f:
            vals = [float(v) for v in line.strip().split()]
            if len(vals) == 16:
                poses.append(np.array(vals, dtype=np.float64).reshape(4, 4))
            elif len(vals) == 12:
                mat = np.array(vals, dtype=np.float64).reshape(3, 4)
                T = np.eye(4, dtype=np.float64)
                T[:3, :] = mat
                poses.append(T)
    return poses


def parse_timestamps(ts_path: Path) -> list:
    """
    Parse timestamps.txt (one ISO 8601 timestamp per line).
    Returns list of integer nanosecond timestamps (int).
    """
    stamps = []
    with open(ts_path) as f:
        for line in f:
            line = line.strip()
            if line:
                dt = datetime.fromisoformat(line)
                stamps.append(int(dt.timestamp() * 1e9))
    return stamps
```

- [ ] **Step 4: Run tests**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "poses or timestamps" -v
```

Expected: 7 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_to_mcap.py scripts/kitti360/tests/test_kitti360_to_mcap.py
git commit -m "feat: add pose and timestamp parsing for KITTI-360 converter"
```

---

## Task 4: LiDAR-to-depth projection

**Files:**
- Modify: `scripts/kitti360/kitti360_to_mcap.py`
- Modify: `scripts/kitti360/tests/test_kitti360_to_mcap.py`

- [ ] **Step 1: Add failing tests**

Append to `tests/test_kitti360_to_mcap.py`:

```python
def _make_test_points() -> np.ndarray:
    """Return (N, 4) velodyne points: a few points directly in front of cam0."""
    # After applying calib_cam_to_velo (cam0→velo), these should project
    # to valid image pixels. Use simple points along +Z in cam0 frame.
    # In cam0 coords: (0, 0, 10) = 10 m straight ahead.
    # We'll just use points already in velo frame that map to cam0 +Z.
    return np.array([
        [0.27, 0.05, 10.0, 0.5],  # roughly: near-axis, 10 m out
        [0.27, 0.05,  5.0, 0.3],  # 5 m out
    ], dtype=np.float32)


def test_project_lidar_to_depth_shape():
    from kitti360_to_mcap import parse_perspective, parse_cam_to_velo, project_lidar_to_depth
    P = parse_perspective(FIXTURES / 'perspective.txt')
    R, T = parse_cam_to_velo(FIXTURES / 'calib_cam_to_velo.txt')
    pts = _make_test_points()
    depth = project_lidar_to_depth(pts, R, T, P, img_h=376, img_w=1408)
    assert depth.shape == (376, 1408)
    assert depth.dtype == np.float32


def test_project_lidar_to_depth_nonnegative():
    from kitti360_to_mcap import parse_perspective, parse_cam_to_velo, project_lidar_to_depth
    P = parse_perspective(FIXTURES / 'perspective.txt')
    R, T = parse_cam_to_velo(FIXTURES / 'calib_cam_to_velo.txt')
    pts = _make_test_points()
    depth = project_lidar_to_depth(pts, R, T, P, img_h=376, img_w=1408)
    assert (depth >= 0).all()


def test_project_lidar_to_depth_points_behind_camera_zeroed():
    from kitti360_to_mcap import parse_perspective, parse_cam_to_velo, project_lidar_to_depth
    P = parse_perspective(FIXTURES / 'perspective.txt')
    R, T = parse_cam_to_velo(FIXTURES / 'calib_cam_to_velo.txt')
    # Point behind camera in cam0 coords: large -Z in cam0 = behind the lens
    pts_behind = np.array([[-10.0, 0.0, -10.0, 0.5]], dtype=np.float32)
    depth = project_lidar_to_depth(pts_behind, R, T, P, img_h=376, img_w=1408)
    assert depth.max() == 0.0
```

- [ ] **Step 2: Run to confirm failure**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "project" -v
```

Expected: `ImportError` (function not defined).

- [ ] **Step 3: Add `project_lidar_to_depth` to `kitti360_to_mcap.py`**

```python
def project_lidar_to_depth(
    pts_velo: np.ndarray,
    R_cam_to_velo: np.ndarray,
    T_cam_to_velo: np.ndarray,
    P_rect: np.ndarray,
    img_h: int,
    img_w: int,
) -> np.ndarray:
    """
    Project velodyne points into cam0 image plane to produce a depth image.

    Args:
        pts_velo:        (N, 4) float32 — velodyne points (x, y, z, intensity)
        R_cam_to_velo:   (3, 3) — rotation from calib_cam_to_velo.txt
        T_cam_to_velo:   (3,)   — translation from calib_cam_to_velo.txt
        P_rect:          (3, 4) — camera projection matrix from perspective.txt
        img_h, img_w:    output image dimensions

    Returns:
        depth: (img_h, img_w) float32, depth in metres; 0.0 where no point.

    Convention: calib_cam_to_velo gives P_velo = R @ P_cam + T,
    so the inverse (velo→cam0) is: P_cam = R.T @ (P_velo - T).
    """
    # velo → cam0
    R_velo_to_cam = R_cam_to_velo.T
    T_velo_to_cam = -R_velo_to_cam @ T_cam_to_velo

    pts_xyz = pts_velo[:, :3].T.astype(np.float64)          # (3, N)
    pts_cam = R_velo_to_cam @ pts_xyz + T_velo_to_cam[:, None]  # (3, N)

    # Keep only points in front of camera
    front = pts_cam[2, :] > 0
    pts_cam = pts_cam[:, front]
    if pts_cam.shape[1] == 0:
        return np.zeros((img_h, img_w), dtype=np.float32)

    # Project: [u, v, d] = P_rect @ [X, Y, Z, 1]^T
    pts_h = np.vstack([pts_cam, np.ones((1, pts_cam.shape[1]))])  # (4, N)
    proj = P_rect @ pts_h                                          # (3, N)
    u = proj[0, :] / proj[2, :]
    v = proj[1, :] / proj[2, :]
    d = proj[2, :]

    # Keep within image bounds
    valid = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    u = u[valid].astype(np.int32)
    v = v[valid].astype(np.int32)
    d = d[valid]

    depth = np.zeros((img_h, img_w), dtype=np.float32)
    depth[v, u] = d.astype(np.float32)
    return depth
```

- [ ] **Step 4: Run tests**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "project" -v
```

Expected: 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_to_mcap.py scripts/kitti360/tests/test_kitti360_to_mcap.py
git commit -m "feat: add LiDAR-to-depth projection for KITTI-360 converter"
```

---

## Task 5: MCAP message builders

**Files:**
- Modify: `scripts/kitti360/kitti360_to_mcap.py`
- Modify: `scripts/kitti360/tests/test_kitti360_to_mcap.py`

- [ ] **Step 1: Add failing tests**

Append to `tests/test_kitti360_to_mcap.py`:

```python
def test_make_image_msg_rgb_shape():
    from kitti360_to_mcap import make_image_msg
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    img = np.zeros((10, 20, 3), dtype=np.uint8)
    msg = make_image_msg(ts, img, 'rgb8', 'cam0', 1_000_000_000)
    assert msg.height == 10
    assert msg.width == 20
    assert msg.encoding == 'rgb8'
    assert msg.step == 60   # 20 * 3


def test_make_image_msg_depth_encoding():
    from kitti360_to_mcap import make_image_msg
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    depth = np.zeros((10, 20), dtype=np.float32)
    msg = make_image_msg(ts, depth, '32FC1', 'cam0', 1_000_000_000)
    assert msg.encoding == '32FC1'
    assert msg.step == 80   # 20 * 4


def test_make_camera_info_msg_K_matches_P():
    from kitti360_to_mcap import make_camera_info_msg, parse_perspective
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    P = parse_perspective(FIXTURES / 'perspective.txt')
    msg = make_camera_info_msg(ts, P, 376, 1408, 'cam0', 1_000_000_000)
    assert msg.height == 376
    assert msg.width == 1408
    # K is upper-left 3x3 of P_rect
    K_expected = P[:3, :3].flatten()
    assert np.allclose(msg.k, K_expected)
    assert len(msg.p) == 12


def test_make_tf_msg_frame_ids():
    from kitti360_to_mcap import make_tf_msg
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    R = np.eye(3)
    t = np.array([1.0, 2.0, 3.0])
    msg = make_tf_msg(ts, R, t, 'world', 'cam0', 1_000_000_000)
    assert len(msg.transforms) == 1
    tf = msg.transforms[0]
    assert tf.header.frame_id == 'world'
    assert tf.child_frame_id == 'cam0'
    assert abs(tf.transform.translation.x - 1.0) < 1e-9
    assert abs(tf.transform.translation.z - 3.0) < 1e-9
```

- [ ] **Step 2: Run to confirm failure**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "make_image or make_camera or make_tf" -v
```

Expected: `ImportError`.

- [ ] **Step 3: Add message builder functions to `kitti360_to_mcap.py`**

Add the following imports at the top of the file (after existing imports):

```python
from scipy.spatial.transform import Rotation
```

Then add these functions after `project_lidar_to_depth`:

```python
def _make_header(typestore, frame_id: str, t_ns: int):
    Header = typestore.types['std_msgs/msg/Header']
    Time = typestore.types['builtin_interfaces/msg/Time']
    return Header(
        stamp=Time(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000)),
        frame_id=frame_id,
    )


def make_image_msg(typestore, image: np.ndarray, encoding: str, frame_id: str, t_ns: int):
    """Build a sensor_msgs/msg/Image. encoding: 'rgb8' or '32FC1'."""
    Image = typestore.types['sensor_msgs/msg/Image']
    if encoding == 'rgb8':
        assert image.ndim == 3 and image.shape[2] == 3
        h, w = image.shape[:2]
        step = w * 3
        data = np.ascontiguousarray(image, dtype=np.uint8).flatten()
    elif encoding == '32FC1':
        assert image.ndim == 2
        h, w = image.shape
        step = w * 4
        data = np.ascontiguousarray(image, dtype=np.float32).view(np.uint8).flatten()
    else:
        raise ValueError(f'Unsupported encoding: {encoding}')
    return Image(
        header=_make_header(typestore, frame_id, t_ns),
        height=h, width=w,
        encoding=encoding,
        is_bigendian=False,
        step=step,
        data=data,
    )


def make_camera_info_msg(typestore, P_rect: np.ndarray, h: int, w: int,
                          frame_id: str, t_ns: int):
    """Build a sensor_msgs/msg/CameraInfo from a 3×4 projection matrix."""
    CameraInfo = typestore.types['sensor_msgs/msg/CameraInfo']
    K = P_rect[:3, :3].flatten()
    return CameraInfo(
        header=_make_header(typestore, frame_id, t_ns),
        height=h, width=w,
        distortion_model='plumb_bob',
        d=np.zeros(5, dtype=np.float64),
        k=K.astype(np.float64),
        r=np.eye(3, dtype=np.float64).flatten(),
        p=P_rect.astype(np.float64).flatten(),
        binning_x=0,
        binning_y=0,
    )


def make_tf_msg(typestore, R: np.ndarray, t: np.ndarray,
                parent: str, child: str, t_ns: int):
    """Build a tf2_msgs/msg/TFMessage with one TransformStamped."""
    TFMessage = typestore.types['tf2_msgs/msg/TFMessage']
    TransformStamped = typestore.types['geometry_msgs/msg/TransformStamped']
    Transform = typestore.types['geometry_msgs/msg/Transform']
    Vector3 = typestore.types['geometry_msgs/msg/Vector3']
    Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
    q = Rotation.from_matrix(R).as_quat()   # [x, y, z, w]
    return TFMessage(
        transforms=[
            TransformStamped(
                header=_make_header(typestore, parent, t_ns),
                child_frame_id=child,
                transform=Transform(
                    translation=Vector3(x=float(t[0]), y=float(t[1]), z=float(t[2])),
                    rotation=Quaternion(x=float(q[0]), y=float(q[1]),
                                        z=float(q[2]), w=float(q[3])),
                ),
            )
        ]
    )
```

- [ ] **Step 4: Run tests**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "make_image or make_camera or make_tf" -v
```

Expected: 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_to_mcap.py scripts/kitti360/tests/test_kitti360_to_mcap.py
git commit -m "feat: add MCAP message builder helpers for KITTI-360 converter"
```

---

## Task 6: Full MCAP writer

**Files:**
- Modify: `scripts/kitti360/kitti360_to_mcap.py`
- Modify: `scripts/kitti360/tests/test_kitti360_to_mcap.py`

- [ ] **Step 1: Add failing integration test**

Append to `tests/test_kitti360_to_mcap.py`:

```python
import tempfile
import cv2


def _make_fake_drive(tmp_path: Path, n_frames: int = 3) -> Path:
    """Create a minimal KITTI-360-like directory structure for testing."""
    drive = '2013_05_28_drive_0000_sync'
    root = tmp_path / 'kitti360'

    # Calibration
    cal_dir = root / 'calibration'
    cal_dir.mkdir(parents=True)
    import shutil
    shutil.copy(FIXTURES / 'perspective.txt', cal_dir / 'perspective.txt')
    shutil.copy(FIXTURES / 'calib_cam_to_velo.txt', cal_dir / 'calib_cam_to_velo.txt')

    # Poses
    pose_dir = root / 'data_poses' / drive
    pose_dir.mkdir(parents=True)
    shutil.copy(FIXTURES / 'poses.txt', pose_dir / 'poses.txt')

    # Timestamps and images
    img_dir = root / 'data_2d_raw' / drive / 'image_00' / 'data_rect'
    img_dir.mkdir(parents=True)
    ts_dir = root / 'data_2d_raw' / drive / 'image_00'
    shutil.copy(FIXTURES / 'timestamps.txt', ts_dir / 'timestamps.txt')
    for i in range(n_frames):
        img = np.zeros((376, 1408, 3), dtype=np.uint8)
        cv2.imwrite(str(img_dir / f'{i:010d}.png'), img)

    # LiDAR
    lidar_dir = root / 'data_3d_raw' / drive / 'velodyne_points' / 'data'
    lidar_dir.mkdir(parents=True)
    for i in range(n_frames):
        pts = np.array([[0.27, 0.05, 10.0, 0.5],
                        [0.27, 0.05,  5.0, 0.3]], dtype=np.float32)
        pts.tofile(lidar_dir / f'{i:010d}.bin')

    return root


def test_write_mcap_creates_file(tmp_path):
    from kitti360_to_mcap import write_mcap
    root = _make_fake_drive(tmp_path)
    out = tmp_path / 'out.mcap'
    write_mcap(root=root, sequence=0, output=out)
    assert out.exists()
    assert out.stat().st_size > 0


def test_write_mcap_contains_expected_topics(tmp_path):
    from kitti360_to_mcap import write_mcap
    from rosbags.rosbag2 import Reader
    root = _make_fake_drive(tmp_path)
    out = tmp_path / 'out.mcap'
    write_mcap(root=root, sequence=0, output=out)

    with Reader(out) as reader:
        topics = {c.topic for c in reader.connections}

    assert '/kitti360/rgb/image_raw' in topics
    assert '/kitti360/depth/depth_registered' in topics
    assert '/kitti360/rgb/camera_info' in topics
    assert '/tf' in topics
    assert '/tf_static' in topics


def test_write_mcap_message_count(tmp_path):
    from kitti360_to_mcap import write_mcap
    from rosbags.rosbag2 import Reader
    root = _make_fake_drive(tmp_path, n_frames=3)
    out = tmp_path / 'out.mcap'
    write_mcap(root=root, sequence=0, output=out)

    with Reader(out) as reader:
        counts = {}
        for conn, _, _ in reader.messages():
            counts[conn.topic] = counts.get(conn.topic, 0) + 1

    assert counts['/kitti360/rgb/image_raw'] == 3
    assert counts['/kitti360/depth/depth_registered'] == 3
    assert counts['/kitti360/rgb/camera_info'] == 3
    assert counts['/tf'] == 3
    assert counts['/tf_static'] == 1   # static TF written once
```

- [ ] **Step 2: Run to confirm failure**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "write_mcap" -v
```

Expected: `ImportError: cannot import name 'write_mcap'`.

- [ ] **Step 3: Add `write_mcap` to `kitti360_to_mcap.py`**

Add at the top of the file:

```python
import cv2
```

Then add after the message builders:

```python
def write_mcap(root: Path, sequence: int, output: Path) -> None:
    """
    Convert one KITTI-360 drive to a ROS2 MCAP bag.

    Args:
        root:     path to KITTI-360 root (contains calibration/, data_2d_raw/, etc.)
        sequence: drive index, e.g. 0 for drive_0000
        output:   path to write the .mcap file
    """
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    drive = f'2013_05_28_drive_{sequence:04d}_sync'
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # --- calibration ---
    P_rect = parse_perspective(root / 'calibration' / 'perspective.txt')
    R_ctv, T_ctv = parse_cam_to_velo(root / 'calibration' / 'calib_cam_to_velo.txt')

    # --- poses + timestamps ---
    poses = parse_poses(root / 'data_poses' / drive / 'poses.txt')
    stamps = parse_timestamps(
        root / 'data_2d_raw' / drive / 'image_00' / 'timestamps.txt'
    )

    img_dir = root / 'data_2d_raw' / drive / 'image_00' / 'data_rect'
    lidar_dir = root / 'data_3d_raw' / drive / 'velodyne_points' / 'data'

    img_files = sorted(img_dir.glob('*.png'))
    bin_files = sorted(lidar_dir.glob('*.bin'))
    n = min(len(img_files), len(bin_files), len(poses), len(stamps))
    if n == 0:
        raise ValueError(f'No matching frames found in {root}')

    # Infer image size from first frame
    sample = cv2.imread(str(img_files[0]))
    img_h, img_w = sample.shape[:2]

    with Writer(output) as writer:
        conn_rgb = writer.add_connection(
            '/kitti360/rgb/image_raw', 'sensor_msgs/msg/Image', typestore=typestore
        )
        conn_depth = writer.add_connection(
            '/kitti360/depth/depth_registered', 'sensor_msgs/msg/Image', typestore=typestore
        )
        conn_info = writer.add_connection(
            '/kitti360/rgb/camera_info', 'sensor_msgs/msg/CameraInfo', typestore=typestore
        )
        conn_tf = writer.add_connection(
            '/tf', 'tf2_msgs/msg/TFMessage', typestore=typestore
        )
        conn_tf_static = writer.add_connection(
            '/tf_static', 'tf2_msgs/msg/TFMessage', typestore=typestore
        )

        # Static TF: velodyne → cam0
        # R_ctv, T_ctv give P_velo = R_ctv @ P_cam + T_ctv
        # → cam0's origin in velodyne frame is T_ctv; its orientation is R_ctv
        static_tf = make_tf_msg(typestore, R_ctv, T_ctv, 'velodyne', 'cam0', stamps[0])
        writer.write(
            conn_tf_static, stamps[0],
            typestore.serialize_cdr(static_tf, 'tf2_msgs/msg/TFMessage')
        )

        for i in range(n):
            t_ns = stamps[i]
            pose = poses[i]   # T_world_cam0

            # Dynamic TF: world → cam0
            R_wc = pose[:3, :3]
            t_wc = pose[:3, 3]
            tf_msg = make_tf_msg(typestore, R_wc, t_wc, 'world', 'cam0', t_ns)
            writer.write(
                conn_tf, t_ns,
                typestore.serialize_cdr(tf_msg, 'tf2_msgs/msg/TFMessage')
            )

            # RGB image
            bgr = cv2.imread(str(img_files[i]))
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rgb_msg = make_image_msg(typestore, rgb, 'rgb8', 'cam0', t_ns)
            writer.write(
                conn_rgb, t_ns,
                typestore.serialize_cdr(rgb_msg, 'sensor_msgs/msg/Image')
            )

            # Depth from LiDAR projection
            pts = np.fromfile(bin_files[i], dtype=np.float32).reshape(-1, 4)
            depth = project_lidar_to_depth(pts, R_ctv, T_ctv, P_rect, img_h, img_w)
            depth_msg = make_image_msg(typestore, depth, '32FC1', 'cam0', t_ns)
            writer.write(
                conn_depth, t_ns,
                typestore.serialize_cdr(depth_msg, 'sensor_msgs/msg/Image')
            )

            # Camera info
            info_msg = make_camera_info_msg(typestore, P_rect, img_h, img_w, 'cam0', t_ns)
            writer.write(
                conn_info, t_ns,
                typestore.serialize_cdr(info_msg, 'sensor_msgs/msg/CameraInfo')
            )
```

- [ ] **Step 4: Run tests**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -k "write_mcap" -v
```

Expected: 3 tests PASS.

- [ ] **Step 5: Run full test suite**

```bash
python -m pytest tests/test_kitti360_to_mcap.py -v
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_to_mcap.py scripts/kitti360/tests/test_kitti360_to_mcap.py
git commit -m "feat: add full MCAP writer for KITTI-360 converter"
```

---

## Task 7: CLI entry point

**Files:**
- Modify: `scripts/kitti360/kitti360_to_mcap.py` (add `main()` + `if __name__` block)

- [ ] **Step 1: Add the CLI block at the bottom of `kitti360_to_mcap.py`**

```python
import argparse


def main():
    parser = argparse.ArgumentParser(
        description='Convert raw KITTI-360 data to a ROS2 MCAP bag.'
    )
    parser.add_argument('--kitti360-root', required=True, type=Path,
                        help='Path to KITTI-360 root directory')
    parser.add_argument('--sequence', required=True, type=int,
                        help='Drive sequence index (e.g. 0 for drive_0000)')
    parser.add_argument('--output', required=True, type=Path,
                        help='Output .mcap file path')
    args = parser.parse_args()

    print(f'Converting sequence {args.sequence:04d} from {args.kitti360_root}')
    write_mcap(root=args.kitti360_root, sequence=args.sequence, output=args.output)
    print(f'Written to {args.output}')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify the CLI help works**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360
python kitti360_to_mcap.py --help
```

Expected output:
```
usage: kitti360_to_mcap.py [-h] --kitti360-root KITTI360_ROOT --sequence SEQUENCE --output OUTPUT
...
Convert raw KITTI-360 data to a ROS2 MCAP bag.
```

- [ ] **Step 3: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_to_mcap.py
git commit -m "feat: add CLI entry point to KITTI-360 MCAP converter"
```

---

## Task 8: Hydra YAML configs

**Files:**
- Create: `scripts/kitti360/kitti360_ros_input.yaml`
- Create: `scripts/kitti360/kitti360_hydra.yaml`

- [ ] **Step 1: Write `kitti360_ros_input.yaml`**

```yaml
---
input:
  type: RosInput
  inputs:
    camera:
      receiver:
        type: RGBDImageReceiver
        queue_size: 30
      sensor:
        type: camera_info
        min_range: 0.5
        max_range: 50.0
        extrinsics:
          type: ros
```

- [ ] **Step 2: Write `kitti360_hydra.yaml`**

This merges outdoor-scale params from `hydra/config/datasets/kitti_360.yaml` with image-storage settings from the `feature/image_storage` branch. `GenericUpdateFunctor` is required because `with_semantics: false` makes `UpdateObjectsFunctor` unusable (it uses `SemanticNearestNode` matching which requires label data).

```yaml
---
# Outdoor scale from hydra/config/datasets/kitti_360.yaml
map_window:
  type: spatial
  max_radius_m: 60.0

active_window:
  volumetric_map:
    voxels_per_side: 16
    voxel_size: 0.35
    truncation_distance: 1.0
    with_semantics: false
  projective_integrator:
    semantic_integrator: {type: SingleLabelIntegrator}
  object_detector:
    type: InstanceForwarding
    min_cluster_size: 30
    min_object_volume: 0.01
    max_range: 30.0
    min_range: 0.5
    instance_id: true
    verbosity: 1
  tracker:
    type: MaxIouTracker
    track_by: pixels
    min_semantic_iou: 0.25
    min_cross_iou: 0.1
    temporal_window: 3.0
    min_num_observations: 10
    verbosity: 0
  object_extractor:
    type: MeshObjectExtractor
    verbosity: 0
    min_object_allocation_confidence: 0.5
    min_object_volume: 0.005
    max_object_volume: 100.0
    min_dynamic_displacement: 1
    only_extract_reconstructed_objects: true
    min_object_reconstruction_confidence: 0.5
    min_object_reconstruction_observations: 10
    object_reconstruction_resolution: -0.05
    visualizer_classification: false
    save_object_images: true
    object_image_output_path: $<env ADT4_OUTPUT_DIR>/images
    projective_integrator:
      num_threads: 8

frontend:
  type: GraphBuilder
  agent_extractor:
    enabled: true
    image_output_path: $<env ADT4_OUTPUT_DIR>/agents
  enable_mesh_objects: false
  serialize_dsg_mesh: false
  clear_object_meshes: true
  pgmo:
    time_horizon: 20.0
    d_graph_resolution: 2.5
    mesh_resolution: 0.01
  graph_connector:
    layers:
      - parent_layer: MESH_PLACES
        child_layers:
          - {layer: OBJECTS}
  graph_updater:
    layer_updates:
      OBJECTS:
        prefix: O
        matcher: {type: IoUNodeMatcher, min_same_iou: 0.2, min_cross_iou: 0.5}
  surface_places:
    type: place_2d
    prefix: P
    pure_final_place_size: 1
    cluster_tolerance: 0.3
    min_cluster_size: 10
    max_cluster_size: 100000
    min_final_place_points: 10
    place_max_neighbor_z_diff: 0.5
    place_overlap_threshold: 0.0
  freespace_places:
    type: gvd
    filter_places: true
    min_places_component_size: 3
    filter_ground: false
    gvd:
      max_distance_m: 5.0
      min_distance_m: 0.5
      min_diff_m: 0.1
      voronoi_config:
        mode: L1_THEN_ANGLE
        min_distance_m: 0.30
        parent_l1_separation: 20
        parent_cos_angle_separation: 0.2
    graph:
      type: CompressionGraphExtractor
      compression_distance_m: 3.0
      min_node_distance_m: 0.6
      min_edge_distance_m: 0.3
      node_merge_distance_m: 1.5
      merge_policy: distance
    tsdf_interpolator:
      type: downsample
      ratio: 2

backend:
  type: BackendModule
  enable_zmq_interface: false
  publish_backend_tf: true
  serialize_dsg_mesh: false
  min_dsg_separation_s: 1
  publish_mesh: true
  min_mesh_separation_s: 10
  add_places_to_deformation_graph: true
  optimize_on_lc: true
  enable_node_merging: true
  update_functors:
    agents: {type: UpdateAgentsFunctor}
    objects:
      type: GenericUpdateFunctor
      verbosity: 0
      layer: OBJECTS
      node_matcher: {type: IoUNodeMatcher, min_same_iou: 0.1, min_cross_iou: 0.3}
      merge_proposer:
        strategy: {type: Pairwise}
    surface_places: {type: Update2dPlacesFunctor}
    places: {type: UpdatePlacesFunctor}
  pgmo:
    run_mode: FULL
    embed_trajectory_delta_t: 5.0
    num_interp_pts: 3
    interp_horizon: 10.0
    add_initial_prior: true
    optimizer:
      type: KimeraRpgoOptimizer
      solver: LM
      gnc: {inlier_probability: 0.9, mu_step: 1.6, max_iterations: 100}
    covariance:
      odom: 1.0e-02
      loop_close: 5.0e-02
      sg_loop_close: 1.0e-01
      prior: 1.0e-02
      mesh_mesh: 1.0e-02
      pose_mesh: 1.0e-02
      place_mesh: 1.0e-02
      place_edge: 10.0
      place_merge: 10.0
      object_merge: 10.0
```

- [ ] **Step 3: Validate both YAML files parse without errors**

```bash
python -c "
import yaml
for f in ['kitti360_ros_input.yaml', 'kitti360_hydra.yaml']:
    with open(f) as fh:
        yaml.safe_load(fh)
    print(f'{f}: OK')
"
```

Expected:
```
kitti360_ros_input.yaml: OK
kitti360_hydra.yaml: OK
```

- [ ] **Step 4: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_ros_input.yaml scripts/kitti360/kitti360_hydra.yaml
git commit -m "feat: add standalone hydra configs for KITTI-360 with image extraction"
```

---

## Task 9: tmuxp session and run script

**Files:**
- Create: `scripts/kitti360/kitti360_session.yaml`
- Create: `scripts/kitti360/run_kitti360.sh`

- [ ] **Step 1: Write `kitti360_session.yaml`**

```yaml
---
session_name: kitti360_hydra
suppress_history: false
options:
  default-command: /bin/zsh
windows:
  - window_name: core
    shell_command_before: source ${ADT4_WS}/install/setup.zsh
    layout: tiled
    panes:
      - ros2 run rmw_zenoh_cpp rmw_zenohd
      - python3 $ADT4_WS/src/awesome_dcist_t4/dcist_launch_system/scripts/show_environment.py

  - window_name: hydra
    shell_command_before: source ${ADT4_WS}/install/setup.zsh
    layout: tiled
    panes:
      - >-
        ros2 run hydra_ros hydra_ros_node
        --ros-args
        -p use_sim_time:=true
        --remap ~/input/camera/rgb/image_raw:=/kitti360/rgb/image_raw
        --remap ~/input/camera/depth_registered/image_rect:=/kitti360/depth/depth_registered
        --remap ~/input/camera/rgb/camera_info:=/kitti360/rgb/camera_info
        --
        --config-utilities-file $ADT4_WS/src/awesome_dcist_t4/scripts/kitti360/kitti360_ros_input.yaml
        --config-utilities-file $ADT4_WS/src/awesome_dcist_t4/scripts/kitti360/kitti360_hydra.yaml
        --config-utilities-yaml "{robot_id: 0, odom_frame: world, robot_frame: cam0, map_frame: map}"
        --config-utilities-yaml "{log_path: $ADT4_OUTPUT_DIR/hydra, output: {use_timestamp: false, overwrite: true}}"
        --config-utilities-yaml "{glog_level: 0, glog_verbosity: 0}"
      - >-
        ros2 run hydra_visualizer hydra_visualizer_node
        --ros-args
        -p use_sim_time:=true
        --remap ~/dsg:=/hydra/backend/dsg
        --
        --config-utilities-yaml "{glog_level: 0, glog_verbosity: 1}"

  - window_name: playback
    shell_command_before: source ${ADT4_WS}/install/setup.zsh
    layout: tiled
    panes:
      - ros2 bag play $KITTI360_BAG --clock --delay 5
```

- [ ] **Step 2: Write `run_kitti360.sh`**

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_FILE="$SCRIPT_DIR/kitti360_session.yaml"

usage() {
  echo "Usage: run_kitti360.sh -b <bag.mcap> -o <output_dir> [-f]"
  echo "  -b  path to converted KITTI-360 MCAP bag"
  echo "  -o  output directory (cleared if it exists)"
  echo "  -f  force — skip confirmation when output dir exists"
  exit 1
}

BAG=""
OUTPUT_DIR=""
FORCE=0

while getopts "b:o:f" opt; do
  case $opt in
    b) BAG="$OPTARG" ;;
    o) OUTPUT_DIR="$OPTARG" ;;
    f) FORCE=1 ;;
    *) usage ;;
  esac
done

[[ -z "$BAG" || -z "$OUTPUT_DIR" ]] && usage
[[ ! -f "$BAG" ]] && { echo "Bag not found: $BAG"; exit 1; }

if [[ -d "$OUTPUT_DIR" ]]; then
  if [[ $FORCE -eq 0 ]]; then
    read -r -p "Output '$OUTPUT_DIR' exists. Remove? [y/N] " confirm
    [[ "$confirm" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 1; }
  fi
  rm -rf "$OUTPUT_DIR"
fi
mkdir -p "$OUTPUT_DIR"

export ADT4_OUTPUT_DIR="$OUTPUT_DIR"
export KITTI360_BAG="$BAG"

echo "Output:  $ADT4_OUTPUT_DIR"
echo "Bag:     $KITTI360_BAG"
tmuxp load "$SESSION_FILE"
```

- [ ] **Step 3: Make the script executable**

```bash
chmod +x /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360/run_kitti360.sh
```

- [ ] **Step 4: Validate the YAML session file parses**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360
python -c "import yaml; yaml.safe_load(open('kitti360_session.yaml')); print('OK')"
```

Expected: `OK`

- [ ] **Step 5: Dry-run the shell script to verify flag parsing**

```bash
bash run_kitti360.sh 2>&1 | head -5
```

Expected: prints the usage message (exits with error because -b/-o not provided).

- [ ] **Step 6: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add scripts/kitti360/kitti360_session.yaml scripts/kitti360/run_kitti360.sh
git commit -m "feat: add tmuxp session and run script for standalone KITTI-360 hydra"
```

---

## End-to-end usage (after all tasks complete)

```bash
# 1. Download KITTI-360 sequence 0 from https://www.cvlibs.net/datasets/kitti-360/download.php
#    Requires: data_3d_raw, data_2d_raw, data_poses, calibration directories.

# 2. Convert to MCAP
cd /home/harel/dcist_ws/src/awesome_dcist_t4/scripts/kitti360
python kitti360_to_mcap.py \
  --kitti360-root /path/to/KITTI-360 \
  --sequence 0 \
  --output /path/to/kitti360_seq00.mcap

# 3. Run hydra
./run_kitti360.sh \
  -b /path/to/kitti360_seq00.mcap \
  -o /home/harel/adt4_output/kitti360_seq00

# Outputs land in:
#   /home/harel/adt4_output/kitti360_seq00/hydra/     (scene graph)
#   /home/harel/adt4_output/kitti360_seq00/images/    (object crops)
#   /home/harel/adt4_output/kitti360_seq00/agents/    (agent pose images)
```
