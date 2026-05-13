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
