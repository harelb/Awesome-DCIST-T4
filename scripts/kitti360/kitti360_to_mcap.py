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
