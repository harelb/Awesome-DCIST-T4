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

    # Project: [u*d, v*d, d] = P_rect @ [X, Y, Z, 1]^T
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
