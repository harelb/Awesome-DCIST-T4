"""Republish a free-flying camera pose as a ground-level robot body frame.

BEHAVIOR-1K / OmniGibson `og_ros` viewer recordings (and any other bag captured
from a disembodied camera) carry only the camera pose:

    world -> odom -> {camera_link, viewer_optical}

There is no robot body. Pinning hydra's `robot_frame` straight onto the camera
makes the whole stack believe the robot IS at eye height, and the traversability
place extractor then measures its free-space band
(`HeightTraversabilityEstimator`, band = [body_z - height_below,
body_z + height_above]) up around the camera instead of just above the floor.
Measured on behavior1k_apt_tour: with base_link == camera_link (z 0.66-1.14 m)
the band landed at eye level, nothing classified traversable, and the map came
out with a good mesh but ZERO places.

This node projects the camera pose down to a fixed height and publishes it as
the body frame, so the band covers [body_z, body_z + height_above]:

  * translation: the camera's (x, y), with z pinned to `body_z`
  * rotation:    yaw only, taken from where the camera is LOOKING -- the +Z axis
                 of an optical-convention `source_child` (x right, y down,
                 z forward) projected onto the ground plane

`body_z` is where a notional robot's BODY ORIGIN sits, NOT the floor. Because
`height_below` defaults to 0 the band starts exactly at body_z, so pinning
body_z to the floor puts the floor surface itself inside the band: those voxels
are observed with |sdf| < voxel_size, so they are not free, and with
`min_traversability: 1.0` + `pessimistic: true` every floor column comes out
INTRAVERSABLE. Measured on behavior1k_apt_tour: body_z 0.0 gave 1 place, where
the same config on a real Spot bag (body ~0.5 m up) gave 265. Keep body_z at a
real robot's standing body height above the floor.

Transforms are emitted with the SAME header stamp as the camera transform that
triggered them (this node echoes /tf rather than polling on a timer), so hydra's
odometry lookups interpolate exactly and never wait.

Note `source_child` should be the OPTICAL frame. In these bags `camera_link` is
NOT the ROS body convention -- it is `viewer_optical` flipped 180 degrees about
X (y up, z backward, i.e. an OpenGL/USD camera), so its +Z points BEHIND the
camera and would give a heading 180 degrees wrong.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _forward_axis_yaw(q):
    """Yaw of the optical +Z axis (the view direction) projected onto the ground."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    # third column of the rotation matrix == the frame's +Z axis in the parent frame
    fx = 2.0 * (x * z + y * w)
    fy = 2.0 * (y * z - x * w)
    if abs(fx) < 1e-9 and abs(fy) < 1e-9:
        # camera is looking straight up or down: ground heading is undefined
        return None
    return math.atan2(fy, fx)


class GroundProjectedBodyTf(Node):
    def __init__(self):
        super().__init__("ground_projected_body_tf")
        self.declare_parameter("source_parent", "odom")
        self.declare_parameter("source_child", "viewer_optical")
        self.declare_parameter("out_child", "base_link")
        self.declare_parameter("body_z", 0.5)

        self.source_parent = self.get_parameter("source_parent").value
        self.source_child = self.get_parameter("source_child").value
        self.out_child = self.get_parameter("out_child").value
        self.body_z = float(self.get_parameter("body_z").value)
        self.last_yaw = 0.0
        self.published = 0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(TFMessage, "/tf", qos)
        self.sub = self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        self.get_logger().info(
            f"projecting {self.source_parent} -> {self.source_child} onto z={self.body_z} "
            f"as {self.source_parent} -> {self.out_child}"
        )

    def on_tf(self, msg):
        out = []
        for tr in msg.transforms:
            if tr.header.frame_id != self.source_parent:
                continue
            if tr.child_frame_id != self.source_child:
                continue

            yaw = _forward_axis_yaw(tr.transform.rotation)
            if yaw is None:
                yaw = self.last_yaw
            self.last_yaw = yaw

            projected = TransformStamped()
            projected.header.stamp = tr.header.stamp
            projected.header.frame_id = self.source_parent
            projected.child_frame_id = self.out_child
            projected.transform.translation.x = tr.transform.translation.x
            projected.transform.translation.y = tr.transform.translation.y
            projected.transform.translation.z = self.body_z
            projected.transform.rotation.x = 0.0
            projected.transform.rotation.y = 0.0
            projected.transform.rotation.z = math.sin(yaw / 2.0)
            projected.transform.rotation.w = math.cos(yaw / 2.0)
            out.append(projected)

        if not out:
            return

        self.pub.publish(TFMessage(transforms=out))
        self.published += len(out)
        if self.published % 500 < len(out):
            self.get_logger().info(f"published {self.published} projected transforms")


def main(args=None):
    rclpy.init(args=args)
    node = GroundProjectedBodyTf()
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
