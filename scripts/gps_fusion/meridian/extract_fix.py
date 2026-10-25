#!/usr/bin/env python3
"""Write /<robot>/fix of a bag as CSV (t, lat, lon, alt, status, cov_xx, cov_yy). Reads only that topic."""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
bag, robot, out = sys.argv[1:4]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
r.set_filter(rosbag2_py.StorageFilter(topics=[f"/{robot}/fix"]))
with open(out, "w") as f:
    f.write("# t,lat,lon,alt,status,cov_xx,cov_yy\n")
    while r.has_next():
        _, d, _ = r.read_next()
        m = deserialize_message(d, NavSatFix)
        f.write(f"{m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec:.3f},{m.latitude:.8f},{m.longitude:.8f},"
                f"{m.altitude:.3f},{m.status.status},{m.position_covariance[0]:.4f},{m.position_covariance[4]:.4f}\n")
