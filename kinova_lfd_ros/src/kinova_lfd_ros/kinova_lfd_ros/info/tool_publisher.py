"""Docstring for gen3.gen3_py.gen3_py.tool_publisher.

Publishes the end effector position relative to the base link

Written by Will Solow, 2025
"""

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TcpPosePublisher(Node):
    def __init__(self):
        super().__init__("tcp_pose_b_pub")
        self._tool_pub = self.create_publisher(PoseStamped, "/tcp_pose_b", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self._timer_callback)

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool_frame_link")

        self._base_frame = self.get_parameter("base_frame").value
        self._tool_frame = self.get_parameter("tool_frame").value
        self.ready = False

    def _timer_callback(self):
        if not self.ready:
            if self.tf_buffer.can_transform(
                self._base_frame, self._tool_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.ready = True
                self.get_logger().info("TF tree ready. Starting TCP publisher.")
            else:
                self.get_logger().warn("Waiting for TF tree to connect...")
                return
        try:
            t = self.tf_buffer.lookup_transform(self._base_frame, self._tool_frame, rclpy.time.Time())

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._base_frame

            msg.pose.position = Point(
                x=t.transform.translation.x,
                y=t.transform.translation.y,
                z=t.transform.translation.z,
            )

            msg.pose.orientation = Quaternion(
                x=t.transform.rotation.x,
                y=t.transform.rotation.y,
                z=t.transform.rotation.z,
                w=t.transform.rotation.w,
            )

            self._tool_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TcpPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
