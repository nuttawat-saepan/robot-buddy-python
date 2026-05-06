# import rclpy
# from rclpy.node import Node
# from unitree_go.msg import LowState, SportModeState
# from sensor_msgs.msg import PointCloud2, PointField
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped, Quaternion
# import struct
# import math
# import random
# from nav_msgs.msg import Odometry 


# class FakeRobot(Node):
#     def __init__(self):
#         super().__init__('fake_robot')

#         # ===== Publishers =====
#         self.low_pub = self.create_publisher(LowState, '/lf/lowstate', 10)
#         self.sport_pub = self.create_publisher(SportModeState, '/lf/sportmodestate', 10)
#         self.pc_pub = self.create_publisher(PointCloud2, '/utlidar/cloud', 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         # ===== SAFE rate (5 Hz) =====
#         self.timer = self.create_timer(0.2, self.publish_data)

#         self.frame_id = "radar"  # ต้องมี TF รองรับ
#         self.tf_broadcaster = TransformBroadcaster(self)
#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0

#     # =========================
#     # PointCloud Generator
#     # =========================
#     def publish_odom(self, stamp):
#         self.x += 0.05  # fake motion
#         self.th += 0.01

#         odom = Odometry()

#         odom.header.stamp = stamp.to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y

#         odom.pose.pose.orientation.w = 1.0

#         odom.twist.twist.linear.x = 0.1

#         self.odom_pub.publish(odom)

#     def publish_odom_tf(self, stamp):
#         t = TransformStamped()

#         t.header.stamp = stamp.to_msg()
#         t.header.frame_id = "odom"
#         t.child_frame_id = "base_link"

#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0

#         t.transform.rotation.w = 1.0

#         self.tf_broadcaster.sendTransform(t)

#     def publish_map_odom_tf(self, stamp):
#         t = TransformStamped()

#         t.header.stamp = stamp.to_msg()
#         t.header.frame_id = "map"
#         t.child_frame_id = "odom"

#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.0

#         t.transform.rotation.w = 1.0

#         self.tf_broadcaster.sendTransform(t)

#     def publish_tf(self, stamp):
#         t = TransformStamped()

#         t.header.stamp = stamp.to_msg()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "radar"

#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.0

#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = 0.0
#         t.transform.rotation.w = 1.0

#         self.tf_broadcaster.sendTransform(t)

#     def create_pointcloud(self, stamp=None):
#         msg = PointCloud2()

#         if stamp is None:
#             stamp = self.get_clock().now()
#         msg.header.stamp = stamp.to_msg()
#         msg.header.frame_id = "radar"

#         points = []

#         angle_min = -1.57
#         angle_increment = 0.0058
#         num_points = int((1.57 * 2) / angle_increment)

#         for i in range(num_points):

#             angle = angle_min + i * angle_increment

#             r = random.uniform(0.2, 9.5)
#             r += 0.03 * math.sin(2.0 * angle)

#             r = max(0.1, min(r, 10.0))

#             x = r * math.cos(angle)
#             y = r * math.sin(angle)

#             z = random.uniform(0.01, 0.25)  # no zero!

#             points.append((x, y, z))

#         msg.height = 1
#         msg.width = len(points)

#         msg.fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]

#         msg.point_step = 12
#         msg.row_step = msg.point_step * len(points)

#         msg.is_bigendian = False
#         msg.is_dense = True

#         msg.data = struct.pack('<' + 'fff' * len(points), *sum(points, ()))

#         return msg

#     def create_low_state(self):
#         low = LowState()
#         for i in range(len(low.motor_state)):
#             low.motor_state[i].q = random.uniform(-1.0, 1.0)
#             low.motor_state[i].dq = random.uniform(-1.0, 1.0)
#             low.motor_state[i].temperature = int(random.uniform(30, 60))

#         low.power_v = 24.0
#         low.bms_state.soc = int(80)
#         low.bms_state.current = int(1)
#         return low

#     def create_sport_state(self):
#         sport = SportModeState()
#         sport.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
#         sport.imu_state.gyroscope = [0.0, 0.0, 0.1]
#         sport.imu_state.accelerometer = [0.0, 0.0, 9.8]
#         sport.imu_state.temperature = int(40)

#         sport.error_code = int(0)
#         sport.mode = int(0)
#         sport.progress = 0.0
#         sport.gait_type = int(0)
#         sport.foot_raise_height = 0.0
#         sport.position = [0.0, 0.0, 0.0]
#         sport.body_height = 0.0
#         sport.velocity = [0.1, 0.0, 0.0]
#         sport.yaw_speed = 0.1
#         sport.range_obstacle = [0.0, 0.0, 0.0, 0.0]
#         sport.foot_force = [0, 0, 0, 0]
#         sport.foot_position_body = [0.0] * 12
#         sport.foot_speed_body = [0.0] * 12
#         return sport

#     # =========================
#     # Main loop
#     # =========================
#     def publish_data(self):
#         now = self.get_clock().now()

#         # TF first, then sensor data with the same timestamp
#         self.publish_map_odom_tf(now)
#         self.publish_tf(now)
#         self.publish_odom(now)
#         self.publish_odom_tf(now)

#         # PointCloud
#         pc = self.create_pointcloud(now)
#         self.pc_pub.publish(pc)

#         # Robot states
#         self.low_pub.publish(self.create_low_state())
#         self.sport_pub.publish(self.create_sport_state())


# # =========================
# # main
# # =========================
# def main():
#     rclpy.init()
#     node = FakeRobot()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import json
# import websocket
# import rclpy
# from rclpy.node import Node
# import threading

# from unitree_go.msg import SportModeState, LowState
# from sensor_msgs.msg import PointCloud2
# import std_msgs.msg


# class RosBridgeRelay(Node):

#     def __init__(self):
#         super().__init__('rosbridge_relay')

#         self.sport_pub = self.create_publisher(SportModeState, '/lf/sportmodestate', 10)
#         self.low_pub = self.create_publisher(LowState, '/lf/lowstate', 10)
#         self.lidar_pub = self.create_publisher(PointCloud2, '/utlidar/cloud', 10)

#         self.ws = websocket.WebSocketApp(
#             "ws://192.168.68.53:9090",
#             on_open=self.on_open,
#             on_message=self.on_message
#         )

#     # ---------------------------
#     def on_open(self, ws):
#         subs = [
#             "/lf/sportmodestate",
#             "/lf/lowstate",
#             "/utlidar/cloud"
#         ]

#         for topic in subs:
#             ws.send(json.dumps({
#                 "op": "subscribe",
#                 "topic": topic
#             }))

#     # ---------------------------
#     def on_message(self, ws, message):
#         msg = json.loads(message)

#         if msg.get("op") != "publish":
#             return

#         topic = msg["topic"]
#         data = msg["msg"]

#         # ---------------- SPORT ----------------
#         if topic == "/lf/sportmodestate":
#             msg_out = SportModeState()

#             # ถ้า field มีจริงใน json จะใส่ได้
#             for k, v in data.items():
#                 if hasattr(msg_out, k):
#                     setattr(msg_out, k, v)

#             self.sport_pub.publish(msg_out)

#         # ---------------- LOW ----------------
#         elif topic == "/lf/lowstate":
#             msg_out = LowState()

#             for k, v in data.items():
#                 if hasattr(msg_out, k):
#                     setattr(msg_out, k, v)

#             self.low_pub.publish(msg_out)

#         # ---------------- LIDAR ----------------
#         elif topic == "/utlidar/cloud":
#             msg_out = PointCloud2()

#             # header
#             msg_out.header = std_msgs.msg.Header()
#             msg_out.header.stamp = self.get_clock().now().to_msg()

#             header = data.get("header", {})
#             msg_out.header.frame_id = header.get("frame_id", "")

#             # metadata
#             msg_out.height = data.get("height", 0)
#             msg_out.width = data.get("width", 0)
#             msg_out.is_bigendian = data.get("is_bigendian", False)
#             msg_out.point_step = data.get("point_step", 0)
#             msg_out.row_step = data.get("row_step", 0)
#             msg_out.is_dense = data.get("is_dense", False)

#             # IMPORTANT: raw data
#             raw = data.get("data", [])

#             if isinstance(raw, str):
#                 # rosbridge มักส่ง base64
#                 import base64
#                 msg_out.data = base64.b64decode(raw)
#             else:
#                 msg_out.data = bytes(raw)

#             self.lidar_pub.publish(msg_out)

#     # ---------------------------
#     def run_ws(self):
#         self.ws.run_forever()


# def main():
#     rclpy.init()

#     node = RosBridgeRelay()

#     thread = threading.Thread(target=node.run_ws, daemon=True)
#     thread.start()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import json
import websocket
import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ROS subscriber
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # WebSocket
        self.ws = websocket.WebSocketApp(
            "ws://192.168.68.53:9090",
            on_open=self.on_open
        )

        self.lock = threading.Lock()

    # ---------------------------
    def on_open(self, ws):
        pass  # ไม่ต้อง subscribe อะไรแล้ว

    # ---------------------------
    # ROS -> rosbridge
    # ---------------------------
    def cmd_callback(self, msg: Twist):

        payload = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear": {
                    "x": msg.linear.x,
                    "y": msg.linear.y,
                    "z": msg.linear.z
                },
                "angular": {
                    "x": msg.angular.x,
                    "y": msg.angular.y,
                    "z": msg.angular.z
                }
            }
        }

        with self.lock:
            try:
                self.ws.send(json.dumps(payload))
            except:
                pass

    # ---------------------------
    def run_ws(self):
        self.ws.run_forever()


def main():
    rclpy.init()

    node = CmdVelBridge()

    thread = threading.Thread(target=node.run_ws, daemon=True)
    thread.start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()