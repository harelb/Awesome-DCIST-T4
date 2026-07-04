"""ROS2 wiring for SimSpot: TF-based pose readback + RGB image subscription.

Mirrors the external_pose branch of spot_tools_ros.fake_spot_ros.FakeSpotRos,
but SimSpotRos never broadcasts TF or integrates pose itself -- Isaac Sim
owns the ground-truth pose and publishes the odom->body transform directly.
"""
import time

import numpy as np
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion

_TF_WARN_PERIOD_S = 5.0


class SimSpotRos:
    def __init__(
        self, host_node, sim_spot=None, odom_frame=None, body_frame=None, rgb_topic=None
    ):
        self.host_node = host_node
        self.attach(sim_spot)

        self.odom_frame_name = odom_frame
        self.body_frame_name = body_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.host_node)

        self._last_pose = None  # last successfully resolved np.array([x, y, yaw])
        self._last_tf_warn_time = 0.0

        self.bridge = CvBridge()
        self.rgb_sub = host_node.create_subscription(
            Image, rgb_topic, self.rgb_callback, 10
        )

    def attach(self, sim_spot):
        """Set/replace the SimSpot back-reference used by the RGB callback."""
        self.sim_spot = sim_spot

    def get_pose_fn(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame_name, self.body_frame_name, rclpy.time.Time()
            )
        except tf2_ros.TransformException as e:
            if self._last_pose is None:
                # Never resolved: SimSpot.get_pose() must return [x, y, yaw],
                # so there is nothing sane to return. Fail loudly at startup.
                raise RuntimeError(
                    f"SimSpotRos: TF {self.odom_frame_name} -> "
                    f"{self.body_frame_name} has never resolved; is the sim "
                    f"publishing this transform? ({e})"
                )
            # Transient dropout: keep the executor alive on the cached pose.
            now = time.time()
            if now - self._last_tf_warn_time > _TF_WARN_PERIOD_S:
                self._last_tf_warn_time = now
                self.host_node.get_logger().warn(
                    f"SimSpotRos: TF {self.odom_frame_name} -> "
                    f"{self.body_frame_name} lookup failed, using cached pose "
                    f"({e})"
                )
            return self._last_pose

        t = tf.transform.translation
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self._last_pose = np.array([t.x, t.y, yaw])
        return self._last_pose

    def rgb_callback(self, msg):
        if self.sim_spot is None:
            return
        self.sim_spot.latest_rgb = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )
