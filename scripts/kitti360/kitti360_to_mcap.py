#!/usr/bin/env python3
"""Convert raw KITTI-360 data to a ROS2 MCAP bag."""

from pathlib import Path
from datetime import datetime, timezone
import argparse
import numpy as np
from scipy.spatial.transform import Rotation
import cv2


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
    RegionOfInterest = typestore.types['sensor_msgs/msg/RegionOfInterest']
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
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
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

    with Writer(output, version=9) as writer:
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
