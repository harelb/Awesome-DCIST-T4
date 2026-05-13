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
