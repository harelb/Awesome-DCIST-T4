"""ROS2 wiring for SimSpot: TF-based pose readback + RGB image subscription.

Mirrors the external_pose branch of spot_tools_ros.fake_spot_ros.FakeSpotRos,
but SimSpotRos never broadcasts TF or integrates pose itself -- Isaac Sim
owns the ground-truth pose and publishes the odom->body transform directly.
"""
import numpy as np
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion


class SimSpotRos:
    def __init__(self, host_node, sim_spot, odom_frame, body_frame, rgb_topic):
        self.host_node = host_node
        self.attach(sim_spot)

        self.odom_frame_name = odom_frame
        self.body_frame_name = body_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.host_node)

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
            self.host_node.get_logger().warn(f"Could not get transform: {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return np.array([t.x, t.y, yaw])

    def rgb_callback(self, msg):
        if self.sim_spot is None:
            return
        self.sim_spot.latest_rgb = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )
