import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import json
import os
import sys
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import threading
from tf2_ros import Buffer, TransformListener
import base64
import paho.mqtt.client as mqtt

class MultiGoal(Node):
    def __init__(self, wp_id):
        super().__init__('multi_goal')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.wp_id = wp_id

        self.state = "IDLE"
        self.index = 0
        self.sending = False
        self.stop_all = False

        # spin config
        self.spin_steps = 8
        self.spin_index = 0

        self.timer = self.create_timer(2.0, self.loop)

        self.bridge = CvBridge()
        self.latest_frame = None

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        
        self.json_pub = self.create_publisher(
            String,
            '/capture/imagest',
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/unitree_cmd',
            10
        )

        self.pub = self.create_publisher(
            CompressedImage, '/capture/image', 10)
        self.img_dir = "/home/usys/turtlebot3_ws/images"
        os.makedirs(self.img_dir, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_yaw = 0.0
        # MQTT config
        self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.mqtt_client.connect("192.168.31.58", 1883, 60)
        self.mqtt_client.loop_start()

        self.goals = []   # 🔥 รอ MQTT อย่างเดียว
        self.paused = False
        self.current_goal_handle = None
        self.state = "IDLE"  # IDLE / MOVING / SPINNING

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("📡 MQTT connected")
        client.subscribe("/navigate_to_pose")
        client.subscribe("robot/cmd")   # 🔥 เพิ่ม
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            raw = msg.payload.decode()
            self.get_logger().info(
                f"📥 Received {raw} waypoints"
            )

            topic = msg.topic

            # 🔥 parse แบบกันพังทุกกรณี
            try:
                data = json.loads(raw)

                # กัน double-encoded JSON
                if isinstance(data, str):
                    data = json.loads(data)

            except Exception as e:
                self.get_logger().error(f"JSON decode failed: {e}")
                return

            # =========================
            # 🔥 COMMAND CHANNEL
            # =========================
            if topic == "robot/cmd":
                cmd = data.get("cmd", "")

                if cmd == "pause":
                    self.state = "PAUSED"
                    self.paused = True
                    self.get_logger().warn("⏸ PAUSE")

                    if self.current_goal_handle:
                        self.current_goal_handle.cancel_goal_async()

                elif cmd == "resume":
                    self.state = "MOVING" if self.current_goal_handle else "IDLE"
                    self.paused = False
                    self.get_logger().warn("▶️ RESUME")

                elif cmd == "stop":
                    self.get_logger().warn("🛑 STOP")

                    if self.current_goal_handle:
                        self.current_goal_handle.cancel_goal_async()

                    self.goals = []
                    self.index = 0
                    self.sending = False

                elif cmd in ["stand", "sit"]:
                    self.send_unitree_cmd(cmd)

                return

            # =========================
            # 🔥 WAYPOINT CHANNEL
            # =========================
            if topic == "/navigate_to_pose":

                # 🔥 normalize: dict → list
                if isinstance(data, dict):
                    data = [data]

                if not isinstance(data, list):
                    self.get_logger().error("Waypoints must be dict or list")
                    return

                self.goals = [
                    {
                        "x": wp.get("x", 0),
                        "y": wp.get("y", 0),
                        "yaw": wp.get("yaw", 0),
                        "isCapture": wp.get("isCapture", False)
                    }
                    for wp in data
                ]

                self.index = 0
                self.sending = False

                self.get_logger().info(
                    f"📥 Received {len(self.goals)} waypoints"
                )

        except Exception as e:
            self.get_logger().error(f"MQTT handler error: {e}")

    def send_unitree_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps({"cmd": cmd})
        self.cmd_pub.publish(msg)

        self.get_logger().info(f"📤 Send unitree cmd: {cmd}")

    def get_current_yaw(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            q = tf.transform.rotation

            # quaternion → yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return yaw

        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")
            return 0.0

    def image_cb(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def capture(self):
        if self.latest_frame is None:
            self.get_logger().warn("No image yet")
            return

        frame = self.latest_frame

        # save file (optional)
        filename = os.path.join(
            self.img_dir,
            f"p{self.index}_s{self.spin_index}_{int(time.time())}.jpg"
        )
        cv2.imwrite(filename, frame)

        # encode image → base64
        _, jpg = cv2.imencode('.jpg', frame)
        img_b64 = base64.b64encode(jpg.tobytes()).decode('utf-8')

        # build JSON
        payload = {
            "capture_id": self.index,
            "image_index": self.spin_index,
            "type": "auto",
            "timestamp": int(time.time()),
            "image": img_b64
        }

        msg = String()
        msg.data = json.dumps(payload)

        self.json_pub.publish(msg)

        self.get_logger().info(
            f"📸 Sent JSON capture id={self.index} spin={self.spin_index}"
        )

    # def spin_result_callback(self, future):
    #     self.get_logger().info("🔄 Spin step done")

    #     # 📸 ถ่ายตรงนี้เลย
    #     self.capture()

    #     self.sending = False
    #     self.spin_next()

    # 🔁 main loop
    def loop(self):
        # if self.state in ["MOVING", "SPINNING"]:
        #     return

        if self.stop_all:
            return

        if self.paused:
            return   # ⛔ หยุดชั่วคราว

        if self.sending:
            return

        if len(self.goals) == 0:
            return

        if self.index >= len(self.goals):
            self.get_logger().info("All goals completed.")
            self.stop_system()
            return

        self.state = "MOVING"
        self.send_goal(self.goals[self.index])

    def safe_float(self, v):
        try:
            return float(v)
        except:
            return 0.0
    
    # 🚀 send goal
    def send_goal(self, goal):
        self.client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.safe_float(goal["x"])
        goal_msg.pose.pose.position.y = self.safe_float(goal["y"])

        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(goal["yaw"])

        self.get_logger().info(f"📍 Sending goal {self.index}: {goal}")

        self.state = "MOVING"
        self.sending = True

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # 📩 goal response
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected")
            self.stop_system()
            return

        self.get_logger().info("✅ Goal accepted")
        self.state = "MOVING"
        self.current_goal_handle = goal_handle   # 🔥 เก็บไว้ใช้ cancel

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # 📊 goal result
    def result_callback(self, future):
        result_wrapper = future.result()
        status = result_wrapper.status
        self.state = "IDLE"
        self.current_wp = self.goals[self.index]
        wp = self.current_wp

        if status == 4:
            self.get_logger().info(f"🎯 Goal {self.index} reached")

            self.current_x = wp["x"]
            self.current_y = wp["y"]
            
            # 📸 capture mode
            if wp.get("isCapture", False):
                self.get_logger().info("📸 Capture enabled → start spin")
                threading.Timer(2.0, self.start_spin).start()
            else:
                self.get_logger().info("➡️ No capture → next waypoint")

                self.index += 1
                self.sending = False
        
            return

        elif status == 5:
            msg = f"💥 ABORT at goal {self.index}"
            self.get_logger().warn(msg)
            self.mqtt_client.publish("robot/status", msg)
        elif status == 6:
            msg = f"⚠️ CANCELED at goal {self.index}"
            self.get_logger().warn(msg)
            self.mqtt_client.publish("robot/status", msg)
        else:
            msg = f"❓ UNKNOWN status {status}"
            self.get_logger().error(msg)
            self.mqtt_client.publish("robot/status", msg)

        self.stop_system()

    # 🔄 start spin
    def start_spin(self):
        self.state = "SPINNING"
        self.get_logger().info("🔄 Start spin based on current yaw")

        yaw = self.get_current_yaw()

        if yaw is None:
            self.get_logger().warn("TF not ready, retry spin in 1s")
            threading.Timer(1.0, self.start_spin).start()
            return

        self.base_yaw = yaw
        self.spin_index = 0

        self.spin_next()

    # 🔄 spin step
    def spin_next(self):
        if self.paused:
            return
        
        if not self.current_wp.get("isCapture", False):
            self.get_logger().info("⛔ Skip spin (no capture)")
            self.index += 1
            self.sending = False
            return

        if self.spin_index >= self.spin_steps:
            self.get_logger().info("✅ Spin completed")
            self.state = "IDLE"
            self.index += 1
            self.sending = False
            return

        offset = self.spin_index * (2 * math.pi / self.spin_steps)
        angle = self.base_yaw + offset

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.current_x
        goal_msg.pose.pose.position.y = self.current_y

        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(angle)

        self.get_logger().info(
            f"🔄 Spin {self.spin_index+1}/8 → {math.degrees(angle):.0f}° (base={math.degrees(self.base_yaw):.0f}°)"
        )

        self.sending = True
        self.spin_index += 1

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.spin_response_callback)

    def delayed_start_spin(self):
        self.get_logger().info("🔄 Start spin based on current yaw")

        self.base_yaw = self.get_current_yaw()
        self.spin_index = 0

        self.spin_next()

    # 📩 spin response
    def spin_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("❌ Spin rejected")
            self.stop_system()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    # 📊 spin result
    def spin_result_callback(self, future):
        self.get_logger().info("🔄 Spin done → wait 1s")

        threading.Timer(1.0, self.capture_once).start()

    def capture_once(self):
        self.capture()
        self.sending = False
        self.spin_next()

    # 🧹 stop system
    def stop_system(self):
        self.state = "IDLE"
        self.stop_all = False

        if hasattr(self, 'timer'):
            self.timer.cancel()

        self.get_logger().warn("🛑 Mission ended.")

    # 🔄 quaternion
    def yaw_to_quaternion(self, yaw):
        q = PoseStamped().pose.orientation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main():
    rclpy.init()

    node = MultiGoal(wp_id=0)  # หรือจะลบ wp_id ออกจาก class ไปเลยก็ได้

    try:
        while rclpy.ok() and not node.stop_all:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warn("User interrupted")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()