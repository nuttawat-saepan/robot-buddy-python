import asyncio
import json
import cv2
import av
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
import websockets

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
except ImportError:
    ChannelFactoryInitialize = None
    VideoClient = None


pcs = set()
global_track = None


# =========================
# Video Track
# =========================
class ROSVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame = None

    def update_frame(self, frame):
        self.frame = frame

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        if self.frame is None:
            # กัน timeout
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            frame = av.VideoFrame.from_ndarray(dummy, format="bgr24")
        else:
            frame = av.VideoFrame.from_ndarray(self.frame, format="bgr24")

        frame.pts = pts
        frame.time_base = time_base
        return frame


# =========================
# ROS Node
# =========================
class CameraNode(Node):
    def __init__(self, track):
        super().__init__('webrtc_camera')
        self.bridge = CvBridge()
        self.track = track
        self.sim_mode = self.declare_parameter('sim_mode', True).value
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/image_raw'
        ).value
        self.output_topic = self.declare_parameter(
            'output_topic', '/frontvideostream'
        ).value
        self.camera_interface = self.declare_parameter(
            'camera_interface', ''
        ).value
        self.show_debug_window = self.declare_parameter(
            'show_debug_window', True
        ).value

        self.publisher = None
        self.client = None

        if self.sim_mode:
            self.create_subscription(
                Image,
                self.image_topic,
                self.image_cb,
                1
            )
            self.get_logger().info(f'Sim camera source: {self.image_topic}')
        else:
            self.init_unitree_camera()
            self.publisher = self.create_publisher(Image, self.output_topic, 10)
            self.create_timer(0.03, self.publish_unitree_frame)
            self.get_logger().info(
                f'Real camera source ready, publishing to {self.output_topic}'
            )

    def init_unitree_camera(self):
        if ChannelFactoryInitialize is None or VideoClient is None:
            raise RuntimeError('unitree_sdk2py is required when sim_mode is false')

        if self.camera_interface:
            ChannelFactoryInitialize(0, self.camera_interface)
            self.get_logger().info(
                f'Channel initialized with arg: {self.camera_interface}'
            )
        else:
            ChannelFactoryInitialize(0)
            self.get_logger().info('Channel initialized with default settings')

        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.track.update_frame(frame)

            if self.show_debug_window:
                cv2.imshow("ROS Camera", frame)
                cv2.waitKey(1)

        except Exception as e:
            print("IMAGE ERROR:", e)

    def publish_unitree_frame(self):
        code, data = self.client.GetImageSample()
        if code != 0 or data is None:
            self.get_logger().warn(f'Get image sample error. code: {code}')
            return

        try:
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('Decoded image is None, skipping frame')
                return

            self.track.update_frame(frame)

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.publisher.publish(msg)

            if self.show_debug_window:
                cv2.imshow("Unitree Camera", frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Unitree image error: {e}')


# =========================
# WebRTC Handler
# =========================
async def handler(websocket):
    print("CLIENT CONNECTED")

    pc = RTCPeerConnection()
    pcs.add(pc)

    pc.addTrack(global_track)

    try:
        async for message in websocket:
            data = json.loads(message)

            if data["type"] == "offer":
                print("RECEIVED OFFER")

                offer = RTCSessionDescription(
                    sdp=data["sdp"],
                    type=data["type"]
                )

                await pc.setRemoteDescription(offer)

                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)

                await websocket.send(json.dumps({
                    "type": pc.localDescription.type,
                    "sdp": pc.localDescription.sdp
                }))

                print("SENT ANSWER")

    except Exception as e:
        print("WS ERROR:", e)

    finally:
        print("CLIENT DISCONNECTED")
        await pc.close()
        pcs.discard(pc)


# =========================
# MAIN
# =========================
def main():
    global global_track

    print("STARTING...")

    rclpy.init()

    global_track = ROSVideoTrack()
    node = CameraNode(global_track)

    async def async_main():
        server = await websockets.serve(
            handler,
            "0.0.0.0",
            9999,
            ping_interval=20,
            ping_timeout=20
        )

        print("WebRTC server started at ws://0.0.0.0:9999")

        loop = asyncio.get_running_loop()
        loop.run_in_executor(None, rclpy.spin, node)

        try:
            await asyncio.Future()
        finally:
            print("SHUTDOWN")
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()

    asyncio.run(async_main())


if __name__ == "__main__":
    main()
