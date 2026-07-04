"""Auto-answers spot_executor's manipulation approval handshake so unattended
sim runs don't block on a human. Approves any request that carries a detection;
rejects no-detection requests (executor then retries/recovers)."""
import rclpy
from nlu_interface_rviz.msg import (
    ManipulationApprovalRequest,
    ManipulationApprovalResponse,
)
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


def build_response(request: ManipulationApprovalRequest) -> ManipulationApprovalResponse:
    resp = ManipulationApprovalResponse()
    resp.approve = bool(request.has_detection)
    resp.image_index = request.detection_image_index
    resp.image_x = request.image_x
    resp.image_y = request.image_y
    return resp


class AutoApprover(Node):
    def __init__(self):
        super().__init__("auto_approver")
        latching = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            ManipulationApprovalResponse, "pick_confirmation", 10
        )
        self.sub = self.create_subscription(
            ManipulationApprovalRequest, "manipulation_request",
            self.on_request, qos_profile=latching,
        )

    def on_request(self, msg):
        resp = build_response(msg)
        self.get_logger().info(f"Auto-approval: approve={resp.approve}")
        self.pub.publish(resp)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoApprover()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
