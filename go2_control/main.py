import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math
import json
import threading
import time
import paho.mqtt.client as mqtt
from cv_bridge import CvBridge
import cv2
import os
import base64
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, BatteryState


class MultiGoal(Node):
    def __init__(self):
        super().__init__('multi_goal')

        # ===== NAV2 =====
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ===== STATE =====
        self.current_goal = None
        self.pending_goal = []
        self.busy = False
        self.in_spin_mode = False
        self.waiting_cancel = False

        # ===== CAPTURE =====
        self.capture_mode = False
        self.capture_steps = 8
        self.capture_index = 0
        self.capture_base_yaw = 0.0

        # ===== SPIN (fallback nav spin) =====
        self.spin_steps = 8
        self.spin_index = 0
        self.base_yaw = 0.0

        self.battery_publish_topic = "/missions/battery"
        self.battery_source_topic = "/battery"
        self.battery_publish_interval = 30.0  # seconds

        self.sim_mode = self.declare_parameter('sim_mode', False).value

        self.last_battery_percent = 100.0 if self.sim_mode else None
        self.sim_battery_drain_rate = 0.5  # percent per publish interval

        # ===== LOOP =====
        self.create_timer(0.2, self.loop)
        self.create_timer(self.battery_publish_interval, self.publish_battery_periodic)

        # ===== MQTT =====
        self.mqtt = mqtt.Client()
        # 🔥 set callback ก่อน connect
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_disconnect = self.on_disconnect
        self.mqtt.on_message = self.on_mqtt

        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=5)

        while True:
            try:
                self.mqtt.connect("192.168.31.58", 1883, 60)
                break
            except Exception as e:
                self.get_logger().error(f"MQTT connect fail: {e}")
                time.sleep(3)

        self.mqtt.loop_start()
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_disconnect = self.on_disconnect

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("💀 MultiGoal ready")
        self.current_x = 0.0
        self.current_y = 0.0

        self.bridge = CvBridge()
        self.latest_frame = None

        self.img_dir = "/home/usys/unitree_ros2/images"
        os.makedirs(self.img_dir, exist_ok=True)

        self.spin_running = False
        self.capture_mode = False

        if self.sim_mode:
            self.get_logger().info('🧪 Simulation mode active: battery values will be simulated')
            self.battery_sub = None
            self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        else:
            self.battery_sub = self.create_subscription(
                BatteryState, self.battery_source_topic, self.battery_state_cb, 10)
            self.sub = self.create_subscription(
            Image, '/frontvideostream', self.image_cb, 10)
        
        # self.json_pub = self.create_publisher(
        #     String,
        #     '/missions/image',
        #     10
        # )
        self.cmd_pub = self.create_publisher(
            String,
            '/unitree_cmd',
            10
        )
        self.status_pub = None
        self.progress_pub = None

        self.index = 0
        self.mission = None
        self.mission_id = None
        self.mission_run_id = None
        self.status_topic = None
        self.progress_topic = None
        self.image_topic = None
        self.total_waypoints = 0
        self.current_index = -1
        self.current_name = ""
        self.paused  = False
        self.paused_goal = None  # 💀 เก็บ goal ที่ถูก pause
        self.stop_requested = False

        self.image_pub = None


    # ================= MQTT =================
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("🟢 MQTT connected")

            # 🔥 สำคัญ: reconnect แล้วต้อง subscribe ใหม่
            client.subscribe("/missions/control")
            client.subscribe("/missions/start")

        else:
            self.get_logger().error(f"🔴 MQTT connect failed: {rc}")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"⚠️ MQTT disconnected (rc={rc})")

        if rc != 0:
            self.get_logger().warn("🔁 Auto reconnecting...")

            try:
                client.reconnect()
            except Exception as e:
                self.get_logger().error(f"Reconnect failed: {e}")

    def on_mqtt(self, client, userdata, msg):
        try:
            raw = msg.payload.decode()
            topic = msg.topic

            self.get_logger().info(f"📥 MQTT [{topic}]: {raw}")

            try:
                data = json.loads(raw)
                if isinstance(data, str):
                    data = json.loads(data)
            except Exception as e:
                self.get_logger().error(f"JSON decode failed: {e}")
                return

            # ================= CMD =================
            if topic == "/missions/control":
                cmd = data.get("action", "")

                if cmd == "pause":
                    self.get_logger().warn("⏸ PAUSE")
                    self.paused = True
                    
                    # 💀 เก็บ goal ปัจจุบันไว้ก่อน
                    self.paused_goal = self.current_goal
                    
                    # 💀 cancel goal ถ้ามี
                    if hasattr(self, "current_goal_handle") and self.current_goal_handle:
                        self.waiting_cancel = True
                        future = self.current_goal_handle.cancel_goal_async()
                        
                        def done(_):
                            self.get_logger().info("🚫 cancel done")
                            self.waiting_cancel = False
                            self.busy = False
                        
                        future.add_done_callback(done)
                    
                    # 💀 ถ้าไม่มี goal_handle ให้ set busy = False ทันที
                    if not hasattr(self, "current_goal_handle") or not self.current_goal_handle:
                        self.busy = False
                        
                    self.publish_status("PAUSED", "Mission paused")
                        
                elif cmd == "resume":
                    self.get_logger().warn("▶️ RESUME")

                    # 💀 รอ cancel เสร็จก่อน
                    if self.waiting_cancel:
                        self.get_logger().warn("⏳ waiting cancel finish...")
                        threading.Timer(0.5, self._do_resume).start()
                        return
                    
                    self._do_resume()

                elif cmd == "stop":
                    self.paused = False
                    self.stop_requested = True
                    self.get_logger().warn("🛑 STOP")
                    if hasattr(self, "current_goal_handle") and self.current_goal_handle:
                        future = self.current_goal_handle.cancel_goal_async()
                        future.add_done_callback(lambda f: self.get_logger().info("🚫 cancel confirmed"))
                        self.cancel_waypoints()
                        self.publish_status("CANCELLED", "Mission canceled by user")
                        self.stop_requested = False
                    else:
                        self.cancel_waypoints()
                        self.publish_status("CANCELLED", "Mission canceled by user")
                        self.stop_requested = False

                    # self.pending_goal.clear()
                    # self.current_goal = None
                    # self.busy = False
                elif cmd in ["stand", "sit"]:
                    self.send_unitree_cmd(cmd)

                return

            # ================= WAYPOINT =================
            if topic == "/missions/start":
                self.get_logger().info("📦 Mission received")

                if isinstance(data, str):
                    data = json.loads(data)

                # 🔥 RESET STATE (สำคัญมาก)
                self.current_index = -1
                self.total_waypoints = 0
                self.current_name = ""

                # (optional แต่ควรมี)
                self.cancel_waypoints()

                self.mission = data
                self.mission_id = data.get("missionId")
                self.mission_run_id = data.get("runId")

                self.status_topic = data.get("statusTopic")
                self.progress_topic = data.get("progressTopic")
                self.image_topic = data.get("imageTopic")

                self.publish_status("PENDING", "Mission accepted")
                self.publish_progress("Mission queued")

                # if self.image_topic:
                #     self.image_pub = self.create_publisher(String, self.image_topic, 10)

                waypoints = data.get("waypoints", [])
                waypoints = sorted(waypoints, key=lambda w: w.get("sequence", 0))

                self.pending_goal.clear()
                self.total_waypoints = len(waypoints)

                for i, wp in enumerate(waypoints):
                    self.pending_goal.append({
                        "x": float(wp.get("x", 0.0)),
                        "y": float(wp.get("y", 0.0)),
                        "yaw": float(wp.get("yaw", 0.0)),
                        "is_capture": wp.get("isCapture", False),
                        "name": wp.get("name", f"WP-{i}")
                    })

                self.get_logger().info(
                    f"📥 Mission {self.mission_id} loaded → {len(self.pending_goal)} waypoints"
                )

                if not self.busy:
                    self.start_next()

        except Exception as e:
            self.get_logger().error(f"MQTT handler error: {e}")

    def on_resume_check(self):
        if self.paused_goal is None:
            return

        # 💀 reset state ให้ clean ก่อนส่ง goal ใหม่
        self.busy = True
        self.current_goal = self.paused_goal
        self.paused_goal = None

        self.get_logger().info("🚀 RESUME sending goal")

        self.send_goal(self.current_goal)

    def _do_resume(self):
        """💀 ฟังก์ชันสำหรับ resume หลัง cancel เสร็จ"""
        self.paused = False
        
        if self.paused_goal is None:
            self.get_logger().warn("⚠️ No paused goal to resume")
            return

        self.get_logger().info("🚀 RESUME sending goal")
        
        # 💀 reset state และส่ง goal ใหม่
        self.busy = True
        self.current_goal = self.paused_goal
        self.paused_goal = None
        
        self.publish_status("RUNNING", "Mission resumed")
        self.send_goal(self.current_goal)

    def try_resume(self):
        if self.paused_goal is None:
            return

        self.busy = True
        self.current_goal = self.paused_goal
        self.paused_goal = None

        self.send_goal(self.current_goal)

    def send_unitree_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps({"cmd": cmd})
        self.cmd_pub.publish(msg)

        self.get_logger().info(f"📤 Send unitree cmd: {cmd}")


    def publish_status(self, status, message=""):
        try:
            if not self.status_topic:
                return

            payload = {
                "runId": self.mission_run_id,
                "missionId": self.mission_id,
                "status": status,
                "message": message
            }

            msg = json.dumps(payload)

            if self.mqtt.is_connected():
                result =self.mqtt.publish(self.status_topic, msg)
                self.get_logger().info(f"📤 MQTT publish result = {result}")
            else:
                self.get_logger().warn("MQTT not connected, skip publish")
        except Exception as e:
            self.get_logger().error(f"Publish status failed: {e}")

    def publish_progress(self, message=""):
        try:
            self.get_logger().info(f"📡 progress_topic = {self.progress_topic}")
            self.get_logger().info(
                    f" current = {self.current_index + 1}"
                )
            self.get_logger().info(
                    f" total = {self.total_waypoints}"
                )
            self.get_logger().info(
                f" progress = {round(((self.current_index) / self.total_waypoints) * 100)}"
            )
            if not self.progress_topic:
                return

            progress = 0
            if self.total_waypoints > 0 and message != "Finish: progress = 100":
                progress = round(((self.current_index ) / self.total_waypoints) * 100)
            elif message == "Finish: progress = 100":
                progress = 100

            payload = {
                "runId": self.mission_run_id,
                "missionId": self.mission_id,
                "progress": progress,
                "currentWaypointIndex": self.current_index,
                "currentWaypointName": self.current_name,
                "message": message
            }

            msg = json.dumps(payload)

            # 🧠 ดู payload แบบ object (debug ลึก)
            self.get_logger().info(f"🧾 payload = {payload}")

            if self.mqtt.is_connected():
                result = self.mqtt.publish(self.progress_topic, msg)
                self.get_logger().info(f"📤 MQTT publish result = {result}")
            else:
                self.get_logger().warn("MQTT not connected, skip publish")
        except Exception as e:
            self.get_logger().error(f"Publish progress failed: {e}")

    def publish_image(self, img_b64):
        if img_b64 is None:
            self.get_logger().warn("No image to publish")
            return

        if not self.image_topic:
            self.get_logger().warn("No image topic set")
            return

        try:

            # 🔥 กันชื่อพัง
            safe_name = (self.current_name or "waypoint") \
                .replace(" ", "_") \
                .replace("/", "_")

            payload = {
                "runId": self.mission_run_id,
                "missionId": self.mission_id,
                "imageBase64": img_b64,
                "imageName": f"{safe_name}_wp{self.current_index}_s{self.spin_index}.jpg",
                "contentType": "image/jpeg",
                "waypointIndex": self.current_index,
                "waypointName": self.current_name
            }
            msg = json.dumps(payload)

            # 🚀 MQTT publish
            if self.mqtt.is_connected():
                self.mqtt.publish(self.image_topic, msg)
            else:
                self.get_logger().warn("MQTT not connected, skip publish")

            self.get_logger().info(
                f"📸 MQTT image sent → {safe_name} ({self.current_index})"
            )

        except Exception as e:
            self.get_logger().error(f"Publish image failed: {e}")

    def battery_state_cb(self, msg):
        try:
            percent = float(msg.percentage)
            if percent <= 1.0:
                percent *= 100.0
            self.last_battery_percent = max(0.0, min(percent, 100.0))
            self.get_logger().debug(f"Battery percent updated: {self.last_battery_percent:.1f}%")
        except Exception as e:
            self.get_logger().warn(f"Battery state parse failed: {e}")

    def publish_battery_periodic(self):
        if self.sim_mode:
            self.simulate_battery()

        if self.last_battery_percent is None:
            self.get_logger().warn("Battery percent unknown, skipping periodic publish")
            return

        self.publish_battery(self.last_battery_percent)

    def simulate_battery(self):
        if self.last_battery_percent is None:
            self.last_battery_percent = 100.0

        self.last_battery_percent = max(0.0, self.last_battery_percent - self.sim_battery_drain_rate)
        self.get_logger().debug(f"Simulated battery percent: {self.last_battery_percent:.1f}%")

    def publish_battery(self, percent):
        try:
            payload = {
                "runId": self.mission_run_id,
                "missionId": self.mission_id,
                "percent": percent,
                "timestamp": int(time.time() * 1000)  # milliseconds
            }

            msg = json.dumps(payload)

            # 🚀 MQTT publish to fixed topic
            if self.mqtt.is_connected():
                result = self.mqtt.publish(self.battery_publish_topic, msg)
                self.get_logger().info(f"🔋 Battery published: {percent}% → {result}")
            else:
                self.get_logger().warn("MQTT not connected, skip battery publish")
        except Exception as e:
            self.get_logger().error(f"Publish battery failed: {e}")

    def get_current_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

            q = trans.transform.rotation

            # quaternion → yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

            yaw = math.atan2(siny_cosp, cosy_cosp)
            return yaw

        except Exception as e:
            self.get_logger().warn(f"TF yaw failed: {e}")
            return 0.0

    # ================= LOOP =================
    def loop(self):
        if self.busy or self.paused or self.waiting_cancel or self.spin_running:
            return

        if len(self.pending_goal) > 0:
            self.start_next()

    # ================= START =================
    def start_next(self):
        self.get_logger().info(f"➡️ start_next() called, pending={len(self.pending_goal)}")
        
        if len(self.pending_goal) == 0:
            self.get_logger().warn("⚠️ No pending goals!")
            return

        self.current_goal = self.pending_goal.pop(0)
        goal = self.current_goal
        self.current_name = goal["name"]
        if self.current_goal is None:
            self.get_logger().error("current_goal is None WTF")
            return
        
        self.busy = True

        # 🔥 update progress index
        self.current_index += 1
        self.current_name = self.current_goal["name"]

        if self.current_index == 0:
            self.publish_status("RUNNING", "Mission started")
            self.publish_progress(f"Mission start: progress = 0")
        else:
            self.publish_progress(f"Navigating to {self.current_name}")

        self.send_goal(self.current_goal)

    # ================= SEND GOAL =================
    def send_goal(self, goal):
        self.client.wait_for_server()

        x = float(goal["x"])
        y = float(goal["y"])
        yaw = float(goal["yaw"])

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = self.yaw_to_q(yaw)

        self.get_logger().info(f"🚀 Go {goal}")

        future = self.client.send_goal_async(msg)
        future.add_done_callback(lambda f: self.on_goal(f, goal))

    # ================= GOAL =================
    def on_goal(self, future, goal):
        self.current_goal_handle = future.result()

        if not self.current_goal_handle.accepted:
            self.get_logger().warn("❌ rejected")
            self.finish()
            return

        self.current_goal_handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, goal)
        )

    # ================= RESULT =================
    def on_result(self, future, goal):
        status = future.result().status

        goal = self.current_goal

        if goal is None:
            self.get_logger().warn("⚠️ Goal already cleared, skip result")
            return

        x = goal["x"]
        y = goal["y"]
        yaw = goal["yaw"]
        is_capture = goal["is_capture"]

        if status == 4:
            self.get_logger().info("🎯 arrived")
            completed_waypoints = self.current_index + 1
            progress = round((completed_waypoints / self.total_waypoints) * 100) if self.total_waypoints > 0 else 0
            self.publish_progress(f"Arrived at waypoint {completed_waypoints} of {self.total_waypoints}: progress = {progress}")

            # 🔥 lock position
            self.current_x = x
            self.current_y = y

            if is_capture:
                self.get_logger().info("⏳ wait 2 sec before spin")

                self.capture_mode = True

                # ❌ ห้าม create_timer(self.start_spin())
                threading.Timer(2.0, self.start_spin).start()
            else:
                self.finish()
                
        elif status == 5:
            self.get_logger().warn(f"💥 ABORTED at goal: {self.current_goal}")
            # 💀 ถ้าเป็น pause ไม่ต้อง cancel เพราะจะ resume ต่อได้
            if self.paused:
                self.get_logger().info("⏸ Paused, waiting for resume")
                self.busy = False
                return
            self.publish_status("FAILED", "Map change or path blocked")
            # fallback logic (สำคัญ)
            self.cancel_waypoints()

        # ❌ CANCELED (manual cancel / preempt)
        elif status == 6:
            self.get_logger().warn(f"⚠️ CANCELED at goal: {self.current_goal}")
            # 💀 ถ้าเป็น pause ไม่ต้อง cancel เพราะจะ resume ต่อได้
            if self.paused:
                self.get_logger().info("⏸ Paused, waiting for resume")
                self.busy = False
                return
            if self.stop_requested:
                self.publish_status("CANCELLED", "Mission cancelled")
                self.stop_requested = False
            else:
                self.get_logger().warn(f"💥 ABORTED at goal: {self.current_goal}")
                self.publish_status("FAILED", "Map change or path blocked")
            self.cancel_waypoints()

        # 🤡 UNKNOWN
        else:
            self.get_logger().error(f"❓ UNKNOWN status: {status}")
            self.publish_status("CANCELLED", "Unknown")
            self.cancel_waypoints()

    # 🔄 start spin
    def start_spin(self):
        self.in_spin_mode = True
        self.get_logger().info("🔄 Start spin based on current yaw")
        self.publish_progress(f"Capturing at {self.current_name}")
        self.base_yaw = self.get_current_yaw()
        self.spin_index = 0
        self.spin_running = True

        self.spin_next()

    # 🔄 spin step
    def spin_next(self):
        if not self.spin_running:
            return

        if self.spin_index >= self.spin_steps:
            self.get_logger().info("✅ Spin completed")

            self.spin_running = False
            self.capture_mode = False
            self.finish()
            return

        offset = self.spin_index * (2 * math.pi / self.spin_steps)
        angle = self.base_yaw + offset

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = self.current_x
        goal_msg.pose.pose.position.y = self.current_y
        goal_msg.pose.pose.orientation = self.yaw_to_q(angle)

        self.get_logger().info(
            f"🔄 Spin {self.spin_index+1}/8 → {math.degrees(angle):.0f}°"
        )
        self.publish_progress(
            f"Scanning {self.current_name} ({self.spin_index+1}/{self.spin_steps})"
        )

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
            self.cancel_waypoints()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result_callback)

    def image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")
            return

        # try:
        #     if self.camera_stream_pub is not None:
        #         self.camera_stream_pub.publish(msg)
        # except Exception as e:
        #     self.get_logger().warn(f"camera stream publish failed: {e}")

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
        self.publish_image(img_b64)
        # payload = {
        #     "capture_id": self.index,
        #     "image_index": self.spin_index,
        #     "type": "auto",
        #     "timestamp": int(time.time()),
        #     "image": img_b64
        # }

        # msg = String()
        # msg.data = json.dumps(payload)

        # self.json_pub.publish(msg)

        self.get_logger().info(
            f"📸 Sent JSON capture id={self.index} spin={self.spin_index}"
        )

    # 📊 spin result
    def spin_result_callback(self, future):
        if not self.spin_running:
            return
        self.get_logger().info("🔄 Spin done → wait 1s")

        threading.Timer(1.0, self.capture_once).start()

    def capture_once(self):
        if not self.spin_running:
            return
        self.capture()
        threading.Timer(1.0, self.spin_next).start()

    # ================= FINISH =================
    def finish(self):
        self.get_logger().info(f"📍 finish() called, pending={len(self.pending_goal)}, spin={self.spin_running}")
        
        if len(self.pending_goal) == 0 and not self.spin_running:
            self.publish_progress("Finish: progress = 100")
            self.publish_status("COMPLETED", "Mission completed")
            self.current_goal = None
        elif len(self.pending_goal) > 0:
            # 💀 มี waypoint ต่อ → ไปต่อทันที
            self.get_logger().info("➡️ Continue to next waypoint")
            self.busy = False
            self.current_goal = None
            self.start_next()
            return
        
        self.in_spin_mode = False
        self.busy = False
        self.capture_mode = False

    def cancel_waypoints(self):
        self.get_logger().warn("🛑 CANCEL ALL WAYPOINTS")

        # 1. cancel current goal
        if hasattr(self, "current_goal_handle") and self.current_goal_handle:
            try:
                future = self.current_goal_handle.cancel_goal_async()
                future.add_done_callback(lambda f: self.get_logger().info("🚫 cancel confirmed"))
                self.get_logger().info("🚫 Current goal canceled")
            except Exception as e:
                self.get_logger().error(f"Cancel goal error: {e}")

        # 2. clear queue
        self.pending_goal.clear()
        self.get_logger().info("🧹 Pending goals cleared")
        self.current_index = -1
        self.current_name = ""
        self.mission = None
        self.mission_id = None
        self.mission_run_id = None
        self.in_spin_mode = False
        self.paused = False
        self.status_pub = None
        self.progress_pub = None

        # 3. reset state
        self.busy = False
        self.current_goal = None

        # 4. stop spin/capture pipelines
        self.capture_mode = False
        self.spin_running = False
        self.spin_index = 0

        self.get_logger().info("✅ Waypoint system fully canceled")

    # ================= UTIL =================
    def yaw_to_q(self, yaw):
        q = PoseStamped().pose.orientation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q


def main():
    rclpy.init()
    node = MultiGoal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()