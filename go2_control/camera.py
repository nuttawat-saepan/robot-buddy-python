import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class CameraStreamNode(Node):
    def __init__(self, channel_arg=None):
        super().__init__('camera_stream')
        self.bridge = CvBridge()

        if channel_arg:
            ChannelFactoryInitialize(0, channel_arg)
            self.get_logger().info(f'Channel initialized with arg: {channel_arg}')
        else:
            ChannelFactoryInitialize(0)
            self.get_logger().info('Channel initialized with default settings')

        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

        self.publisher = self.create_publisher(Image, '/camera/stream', 10)
        self.timer = self.create_timer(0.2, self.publish_frame)

        self.get_logger().info('Camera stream node ready, publishing to /camera/stream')

    def publish_frame(self):
        code, data = self.client.GetImageSample()
        if code != 0 or data is None:
            self.get_logger().warn(f'Get image sample error. code: {code}')
            return

        try:
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warn('Decoded image is None, skipping publish')
                return

            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera stream: {e}')


def main():
    rclpy.init()
    node = CameraStreamNode(sys.argv[1] if len(sys.argv) > 1 else None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()