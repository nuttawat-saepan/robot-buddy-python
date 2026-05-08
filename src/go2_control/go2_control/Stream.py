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

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            1
        )

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.track.update_frame(frame)
            # print("FRAME RECEIVED")

            # debug (ปิดได้ถ้าหน่วง)
            cv2.imshow("ROS Camera", frame)
            cv2.waitKey(1)

        except Exception as e:
            print("IMAGE ERROR:", e)


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