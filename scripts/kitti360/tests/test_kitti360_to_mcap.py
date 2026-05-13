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
    assert abs(np.linalg.det(R) - 1.0) < 1e-3
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-1)


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


def _make_test_points() -> np.ndarray:
    """Return (N, 4) velodyne points: a few points directly in front of cam0.

    With the fixture calibration, velo +X maps to cam0 +Z (in front of camera).
    [10, 0, 0] -> cam Z ~9.94, projects to u~701, v~175 (within 376x1408 image).
    [5,  0, 0] -> cam Z ~4.97, projects to u~696, v~160 (within 376x1408 image).
    """
    return np.array([
        [10.0, 0.0, 0.0, 0.5],
        [ 5.0, 0.0, 0.0, 0.3],
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
    # With fixture calibration, velo -X maps to cam0 -Z (behind camera).
    # [-10, 0, 0] -> cam Z ~ -9.96 (behind camera).
    pts_behind = np.array([[-10.0, 0.0, 0.0, 0.5]], dtype=np.float32)
    depth = project_lidar_to_depth(pts_behind, R, T, P, img_h=376, img_w=1408)
    assert depth.max() == 0.0


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
        pts = np.array([[10.0, 0.0, 0.0, 0.5],
                        [5.0, 0.0, 0.0, 0.3]], dtype=np.float32)
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
