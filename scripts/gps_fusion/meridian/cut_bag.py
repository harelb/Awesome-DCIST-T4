#!/usr/bin/env python3
"""Cut a time window of a West Point bag down to what meridian reads.

Keeps rgb (as JPEG CompressedImage on <rgb>/compressed) + depth at --rate Hz (by header stamp,
so rgb/depth frames stay paired), both camera_infos, /tf and /tf_static.
"""
import argparse
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage, Image

p = argparse.ArgumentParser()
p.add_argument("bag"); p.add_argument("out"); p.add_argument("robot", help="robot name used in the ZED topic names")
p.add_argument("t0", type=float); p.add_argument("tf", type=float)
p.add_argument("--rate", type=float, default=5.0)
a = p.parse_args()

zed = f"/{a.robot}/{a.robot}_zed"
rgb, depth = f"{zed}/rgb/image_rect_color", f"{zed}/depth/depth_registered"
infos = [f"{zed}/rgb/camera_info", f"{zed}/depth/camera_info"]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {t.name: t.type for t in r.get_all_topics_and_types()}

w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=a.out, storage_id="mcap", storage_preset_profile="zstd_fast"),
       rosbag2_py.ConverterOptions("cdr", "cdr"))
out_types = {rgb + "/compressed": "sensor_msgs/msg/CompressedImage", depth: types[depth],
             "/tf": types["/tf"], "/tf_static": types["/tf_static"], **{i: types[i] for i in infos}}
for i, (name, typ) in enumerate(out_types.items()):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=typ, serialization_format="cdr"))

# tf_static is latched at the start of the recording
# (camera_info is in the filter so the scan can stop after 30 s instead of reading the whole bag)
r.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static", infos[0]]))
first = None
while r.has_next():
    topic, d, ts = r.read_next()
    first = ts if first is None else first
    if ts - first > 30e9:
        break
    if topic == "/tf_static":
        w.write("/tf_static", d, max(ts, int(a.t0 * 1e9)))

r.set_filter(rosbag2_py.StorageFilter(topics=[rgb, depth, "/tf"] + infos))
r.seek(int(a.t0 * 1e9))
last_bin = {rgb: None, depth: None}
counts = dict.fromkeys(out_types, 0)
while r.has_next():
    topic, d, ts = r.read_next()
    if ts > a.tf * 1e9:
        break
    if topic in last_bin:
        msg = deserialize_message(d, Image)
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        b = int(stamp * a.rate)
        if b == last_bin[topic]:
            continue
        last_bin[topic] = b
        if topic == rgb:
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            ch = msg.step // msg.width
            img = img[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
            if ch == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            out = CompressedImage(header=msg.header, format="bgr8; jpeg compressed bgr8",
                                  data=cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes())
            topic, d = rgb + "/compressed", serialize_message(out)
    w.write(topic, d, ts)
    counts[topic] += 1
del w
print(a.out, counts)
