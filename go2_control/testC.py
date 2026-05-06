import json
import websocket
import rclpy
from rclpy.node import Node

from unitree_go.msg import SportModeState, LowState
from sensor_msgs.msg import PointCloud2


class RosBridgeRelay(Node):

    def __init__(self):
        super().__init__('rosbridge_relay')

        # ROS publishers (ในเครื่อง)
        self.sport_pub = self.create_publisher(SportModeState, '/lf/sportmodestate', 10)
        self.low_pub = self.create_publisher(LowState, '/lf/lowstate', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/utlidar/cloud', 10)

        # rosbridge websocket
        self.ws = websocket.WebSocketApp(
            "ws://192.168.68.53:9090",
            on_open=self.on_open,
            on_message=self.on_message
        )

    # ---------------------------
    # subscribe จาก robot
    # ---------------------------
    def on_open(self, ws):

        subs = [
            "/lf/sportmodestate",
            "/lf/lowstate",
            "/utlidar/cloud"
        ]

        for topic in subs:
            ws.send(json.dumps({
                "op": "subscribe",
                "topic": topic
            }))

        self.get_logger().info("Subscribed to rosbridge topics")

    # ---------------------------
    # receive data from rosbridge
    # ---------------------------
    def on_message(self, ws, message):

        msg = json.loads(message)

        if msg.get("op") != "publish":
            return

        topic = msg["topic"]
        data = msg["msg"]

        # map → ROS publish ในเครื่อง
        self.forward(topic, data)

    # ---------------------------
    # forward → local ROS
    # ---------------------------
    def forward(self, topic, data):

        if topic == "/lf/sportmodestate":
            msg = SportModeState()
            self.sport_pub.publish(msg)

        elif topic == "/lf/lowstate":
            msg = LowState()
            self.low_pub.publish(msg)

        elif topic == "/utlidar/cloud":
            msg = PointCloud2()
            self.lidar_pub.publish(msg)

    # ---------------------------
    # run websocket
    # ---------------------------
    def run_ws(self):
        self.ws.run_forever()


def main():
    rclpy.init()

    node = RosBridgeRelay()

    import threading
    threading.Thread(target=node.run_ws, daemon=True).start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()