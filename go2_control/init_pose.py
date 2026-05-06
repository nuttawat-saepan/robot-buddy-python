import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

import time


class InitPose(Node):

    def __init__(self):
        super().__init__('init_pose')

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(1.0, self.run_once)
        self.done = False

    def run_once(self):
        if self.done:
            return

        self.get_logger().info("Waiting Nav2 ready...")

        time.sleep(5.0)  # รอ AMCL / map / tf

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # ===== START POSE =====
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.55
        msg.pose.pose.position.z = 0.0

        # yaw = 0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0

        # covariance (สำคัญ!)
        msg.pose.covariance = [0.0] * 36

        self.pub.publish(msg)

        self.get_logger().info("INITIAL POSE SENT 😏")

        self.done = True


def main():
    rclpy.init()
    node = InitPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()