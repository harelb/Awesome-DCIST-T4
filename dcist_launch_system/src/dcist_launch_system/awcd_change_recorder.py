"""Record the latest AWCD change set of each robot to JSON.

Each `khronos_msgs/AwcdChanges` message is the robot's complete, already-refined change set,
so the recorder keeps the latest message per robot and rewrites the output file atomically.
Removed objects are keyed by the prior scene graph node id, added objects by the live track
id (the robot's image store binds tracks to its scene graph nodes).

Output (one entry per robot)::

    {"<robot>": {"frame_id": ..., "stamp_ns": ..., "removed": [...], "added": [...]}}
"""

import json
import os
import pathlib

import rclpy
from khronos_msgs.msg import AwcdChanges
from rclpy.node import Node
from rclpy.time import Time


def _info_to_dict(info):
    return {
        "id": int(info.id),
        "stamp_ns": Time.from_msg(info.stamp).nanoseconds,
        "change_confidence": float(info.change_confidence),
        "num_frames_observed": int(info.num_frames_observed),
        "centroid": [info.centroid.x, info.centroid.y, info.centroid.z],
        "bbox_center": [info.bbox_center.x, info.bbox_center.y, info.bbox_center.z],
        "bbox_dimensions": [
            info.bbox_dimensions.x,
            info.bbox_dimensions.y,
            info.bbox_dimensions.z,
        ],
        "bbox_orientation_xyzw": [
            info.bbox_orientation.x,
            info.bbox_orientation.y,
            info.bbox_orientation.z,
            info.bbox_orientation.w,
        ],
        "semantic_label": int(info.semantic_label),
        "name": info.name,
        "confidence": float(info.confidence),
    }


def changes_to_dict(msg):
    """Convert an AwcdChanges message to a JSON-serializable dict."""
    return {
        "frame_id": msg.header.frame_id,
        "stamp_ns": Time.from_msg(msg.header.stamp).nanoseconds,
        "removed": [_info_to_dict(x) for x in msg.removed_objects],
        "added": [_info_to_dict(x) for x in msg.added_objects],
    }


class AwcdChangeRecorder(Node):
    def __init__(self):
        super().__init__("awcd_change_recorder")
        default = os.path.join(os.environ.get("ADT4_OUTPUT_DIR", "."), "awcd_changes.json")
        self.output = pathlib.Path(self.declare_parameter("output", default).value)
        self.latest = {}
        self.create_subscription(AwcdChanges, "/awcd_changes", self._on_changes, 10)
        self.get_logger().info(f"recording /awcd_changes to {self.output}")

    def _on_changes(self, msg):
        self.latest[msg.robot_name] = changes_to_dict(msg)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.output.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(self.latest, indent=1))
        tmp.replace(self.output)


def main():
    rclpy.init()
    node = AwcdChangeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
