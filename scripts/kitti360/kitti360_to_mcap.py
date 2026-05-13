#!/usr/bin/env python3
"""Convert raw KITTI-360 data to a ROS2 MCAP bag."""

from pathlib import Path
from datetime import datetime, timezone
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
