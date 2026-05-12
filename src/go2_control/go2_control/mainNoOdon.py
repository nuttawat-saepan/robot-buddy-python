import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math
import json
import threading
import time
import uuid
import paho.mqtt.client as mqtt
from cv_bridge import CvBridge
import cv2
import os
import array
import base64
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, BatteryState
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu


def normalize_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


class MultiGoal(Node):
    def __init__(self):
        super().__init__('multi_goal')
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # ===== NAV2 =====
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ===== STATE =====
        self.current_goal = None
        self.current_goal_handle = None
        self.pending_goal = []
        self.busy = False
        self.in_spin_mode = False
        self.waiting_cancel = False
        self.odom = None
        self.imu = None
        self.map = None
        self.localization_active = False
        self.localization_request_id = None
        self.localization_context = None
        self.mission_tag_poses = {}
        self.pose_adjust_active = False
        self.pose_adjust_timer = None
        self.pose_correction_active = False
        self.pose_correction_completed = False
        self.pose_adjust_state = None

        # ===== CAPTURE =====
        self.capture_mode = False
        self.capture_steps = 8
        self.capture_index = 0
        self.capture_base_yaw = 0.0

        # ===== SPIN (fallback nav spin) =====
        self.spin_steps = 8
        self.spin_index = 0
        self.base_yaw = 0.0


        # ===== BATTERY =====
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("💀 MultiGoal ready")
        self.current_x = 0.0
        self.current_y = 0.0
        self.odom = None
        self.imu = None
        self.map = None
        self.last_cmd_vel = None

        self.bridge = CvBridge()
        self.latest_frame = None

        self.img_dir = "/home/usys/unitree_ros2/images"
        os.makedirs(self.img_dir, exist_ok=True)

        self.spin_running = False
        self.capture_mode = False
        self.last_cmd_vel = None

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
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.localization_request_pub = self.create_publisher(
            String,
            '/localization/request',
            10
        )
        self.localization_abort_pub = self.create_publisher(
            String,
            '/localization/abort',
            10
        )
        self.create_subscription(
            String,
            '/localization/status',
            self.localization_status_cb,
            10
        )
        self.status_pub = None
        self.progress_pub = None

        self.odom_topic = "/odom"
        self.imu_topic = "/imu/data"
        self.map_topic = "/map"

         # ===== SUB =====
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            map_qos
        )

        self.update_timer = self.create_timer(0.2, self.publish_data)

        # map sync
        self.map_publish_timer = self.create_timer(
            10.0,
            self.map_sync_loop
        )

        self.frontend_connected = False

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
            #  control joy topic
            client.subscribe("cmd_vel")

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

                if "localize" in data or cmd == "localize":
                    localize = data.get("localize", {})
                    if not isinstance(localize, dict):
                        localize = {}
                    if "tagPoses" in data and data["tagPoses"]:
                        self.mission_tag_poses = data["tagPoses"]
                    self.start_localization(localize, source="manual")

                elif cmd == "pause":
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
                    self.publish_status("CANCELLED", "Mission canceled by user")
                    if hasattr(self, "current_goal_handle") and self.current_goal_handle:
                        future = self.current_goal_handle.cancel_goal_async()
                        future.add_done_callback(lambda f: self.get_logger().info("🚫 cancel confirmed"))
                        self.cancel_waypoints()
                        self.stop_requested = False
                    else:
                        self.cancel_waypoints()
                        self.stop_requested = False

                    # self.pending_goal.clear()
                    # self.current_goal = None
                    # self.busy = False
                elif cmd in ["stand", "sit"]:
                    self.send_unitree_cmd(cmd)

                return

            # ================= WAYPOINT =================
            elif topic == "/missions/start":
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
                self.mission_tag_poses = data.get("tagPoses", {})
                if self.mission_tag_poses:
                    self.get_logger().info(
                        f"[tagPoses] received ids={list(self.mission_tag_poses.keys())}"
                    )
                else:
                    self.get_logger().warn("[tagPoses] not in mission payload → april_localizer will use YAML fallback")

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
                        "localize": self.parse_localize_config(wp.get("localize")),
                        "name": wp.get("name", f"WP-{i}")
                    })

                self.get_logger().info(
                    f"📥 Mission {self.mission_id} loaded → {len(self.pending_goal)} waypoints"
                )

                if not self.busy:
                    self.start_next()
            elif topic == "cmd_vel":
                self.get_logger().info(f"🎮 Joy command received: {data}")
                try:
                     # ===== parse Twist JSON =====
                    linear = data.get("linear", {})
                    angular = data.get("angular", {})

                    # ===== parse joy =====
                    vx = float(linear.get("x", 0.0))
                    vy = float(linear.get("y", 0.0))
                    wz = float(angular.get("z", 0.0))

                    # ===== clamp =====
                    vx = max(-1.0, min(1.0, vx))
                    wz = max(-1.5, min(1.5, wz))
                    vy = max(-1.0, min(1.0, vy))
                    self.get_logger().info(f"🎮 Parsed cmd_vel: vx={vx}, vy={vy}, wz={wz}")

                    # ===== publish =====
                    self.publish_cmd_vel(vx, wz, vy)

                except Exception as e:
                    self.get_logger().error(f"joy parse error: {e}")

        except Exception as e:
            self.get_logger().error(f"MQTT handler error: {e}")

    def parse_localize_config(self, raw):
        if not isinstance(raw, dict):
            return {"enabled": False}

        return {
            "enabled": bool(raw.get("enabled", False)),
            "scan": bool(raw.get("scan", False)),
            "expectedTagId": raw.get("expectedTagId"),
            "timeout": raw.get("timeout"),
            "rotationSpeed": raw.get("rotationSpeed"),
            "stopDelay": float(raw.get("stopDelay") or 1.0),
            "settleTime": float(raw.get("settleTime") or 2.0),
            "correctPosition": bool(raw.get("correctPosition", True)),
            "positionTolerance": float(raw.get("positionTolerance") or 0.15),
            "yawTolerance": float(raw.get("yawTolerance") or 0.15),
        }

    def normalize_tag_poses_for_robot(self, tag_poses):
        normalized = {}
        for tag_id, pose in tag_poses.items():
            if not isinstance(pose, dict):
                normalized[tag_id] = pose
                continue

            robot_pose = dict(pose)
            if "yawDegrees" in robot_pose:
                frontend_yaw = float(robot_pose["yawDegrees"])
                robot_pose["yawDegrees"] = normalize_degrees(frontend_yaw - 255.0)
            normalized[tag_id] = robot_pose
        return normalized

    def start_localization(self, localize, source="manual"):
        if self.localization_active:
            self.get_logger().warn("Localization already running")
            return

        request_id = str(uuid.uuid4())
        payload = {
            "requestId": request_id,
            "source": source,
            "runId": self.mission_run_id,
            "missionId": self.mission_id,
            "waypointIndex": self.current_index if source == "waypoint" else None,
            "waypointName": self.current_name if source == "waypoint" else None,
            "localize": {
                "scan": bool(localize.get("scan", False)),
            },
        }

        if localize.get("expectedTagId") is not None:
            payload["localize"]["expectedTagId"] = int(localize["expectedTagId"])
        if localize.get("timeout") is not None:
            payload["localize"]["timeout"] = float(localize["timeout"])
        if localize.get("rotationSpeed") is not None:
            payload["localize"]["rotationSpeed"] = float(localize["rotationSpeed"])
        if self.mission_tag_poses:
            payload["tagPoses"] = self.normalize_tag_poses_for_robot(self.mission_tag_poses)
            self.get_logger().info(
                f"[tagPoses] forwarding to april_localizer ids={list(self.mission_tag_poses.keys())}"
            )
        else:
            self.get_logger().warn("[tagPoses] empty → april_localizer will use YAML fallback")

        self.stop_robot()
        self.localization_active = True
        self.localization_request_id = request_id
        self.localization_context = source

        msg = String()
        msg.data = json.dumps(payload)
        self.localization_request_pub.publish(msg)
        self.publish_localization_event({
            "requestId": request_id,
            "source": source,
            "status": "REQUESTED",
            "message": "Localization requested",
            "scan": payload["localize"]["scan"],
        })

    def localization_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Localization status JSON failed: {e}")
            return

        self.publish_localization_event(data)

        if not self.localization_active:
            return

        if data.get("requestId") != self.localization_request_id:
            return

        status = data.get("status")
        if status == "SUCCESS":
            context = self.localization_context
            if context == "waypoint" and self.pose_adjust_state is not None:
                self.pose_adjust_state["localizationResult"] = data
            self.localization_active = False
            self.localization_request_id = None
            self.localization_context = None

            if context == "waypoint":
                self.publish_progress(f"Localized at {self.current_name}")
                self.settle_after_pose_adjust()

        elif status == "FAILED":
            context = self.localization_context
            message = data.get("message", "Localization failed")
            if context == "waypoint":
                self.publish_pose_adjust_event({
                    "eventType": "POSE_ADJUST_FAILED",
                    "requestId": data.get("requestId"),
                    "result": "LOCALIZATION_FAILED",
                    "message": message,
                    "localization": data,
                    "state": self.pose_adjust_state,
                })
            self.localization_active = False
            self.localization_request_id = None
            self.localization_context = None
            self.pose_adjust_state = None

            if context == "waypoint":
                self.publish_status("FAILED", message)
                self.cancel_waypoints()

    def publish_localization_event(self, data):
        payload = {
            "runId": self.mission_run_id,
            "missionId": self.mission_id,
            "waypointIndex": self.current_index,
            "waypointName": self.current_name,
            **data,
        }
        try:
            if self.mqtt.is_connected():
                self.mqtt.publish("/missions/localization", json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Publish localization event failed: {e}")

    def publish_pose_adjust_event(self, data):
        payload = {
            "runId": self.mission_run_id,
            "missionId": self.mission_id,
            "waypointIndex": self.current_index,
            "waypointName": self.current_name,
            **data,
        }
        try:
            if self.mqtt.is_connected():
                self.mqtt.publish("/missions/pose-adjust", json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Publish pose adjust event failed: {e}")

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def get_current_pose(self, warn=True):
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
                yaw,
            )

        except Exception as e:
            if warn:
                self.get_logger().warn(f"TF pose failed: {e}")
            return None

    def pose_to_dict(self, pose, source):
        if pose is None:
            return None

        return {
            "x": pose[0],
            "y": pose[1],
            "yaw": pose[2],
            "source": source,
        }

    def build_pose_delta(self, start_pose, end_pose):
        if start_pose is None or end_pose is None:
            return None

        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]
        dyaw = self.angle_error(end_pose[2], start_pose[2])
        return {
            "dx": dx,
            "dy": dy,
            "distance": math.hypot(dx, dy),
            "dyaw": dyaw,
            "dyawDegrees": math.degrees(dyaw),
        }

    def pose_dict_to_tuple(self, pose):
        if not isinstance(pose, dict):
            return None

        try:
            return (
                float(pose.get("x", 0.0)),
                float(pose.get("y", 0.0)),
                float(pose.get("yaw", 0.0)),
            )
        except Exception:
            return None

    def _send_localization_abort(self):
        msg = String()
        msg.data = json.dumps({"abort": True})
        self.localization_abort_pub.publish(msg)

    def odom_cb(self, msg):
        self.odom = msg

    def imu_cb(self, msg):
        self.imu = msg

    def map_cb(self, msg):
        self.map = msg
        self.get_logger().info(
            f"🗺 Map received: {msg.info.width}x{msg.info.height}"
        )

    def publish_full_map(self):

        if self.map is None:
            self.get_logger().info(
                "map not recieved yet, cannot publish"
            )
            return

        try:

            # int8[] -> bytes
            raw = array.array('b', self.map.data).tobytes()

            # bytes -> base64 string
            encoded = base64.b64encode(raw).decode('utf-8')

            payload = {
                "op": "publish",
                "topic": "/map",

                "msg": {
                    "header": {
                        "stamp": {
                            "secs": int(self.map.header.stamp.sec),
                            "nsecs": int(self.map.header.stamp.nanosec)
                        },
                        "frame_id": self.map.header.frame_id
                    },

                    "info": {
                        "resolution": self.map.info.resolution,
                        "width": self.map.info.width,
                        "height": self.map.info.height,

                        "origin": {
                            "position": {
                                "x": self.map.info.origin.position.x,
                                "y": self.map.info.origin.position.y,
                                "z": self.map.info.origin.position.z
                            },

                            "orientation": {
                                "x": self.map.info.origin.orientation.x,
                                "y": self.map.info.origin.orientation.y,
                                "z": self.map.info.origin.orientation.z,
                                "w": self.map.info.origin.orientation.w
                            }
                        }
                    },

                    "encoding": "base64-int8",

                    "data": encoded
                }
            }

            self.mqtt.publish(
                "map",
                json.dumps(payload)
            )
            self.last_map_publish = time.time()

            self.get_logger().info(
                f"🗺 compressed map sent "
                f"({len(raw)} bytes raw)"
            )

        except Exception as e:
            self.get_logger().error(
                f"publish map failed: {e}"
            )

    def map_sync_loop(self):

        if self.map is None:
            return

        self.publish_full_map()

    def publish_data(self):
        try:

            # ================= ODOM =================
            if self.odom is not None:
                map_pose = self.get_current_pose(warn=False)

                odom_data = {
                    "px": self.odom.pose.pose.position.x,
                    "py": self.odom.pose.pose.position.y,
                    "pz": self.odom.pose.pose.position.z,

                    "qx": self.odom.pose.pose.orientation.x,
                    "qy": self.odom.pose.pose.orientation.y,
                    "qz": self.odom.pose.pose.orientation.z,
                    "qw": self.odom.pose.pose.orientation.w,

                    "vx": self.odom.twist.twist.linear.x,
                    "vy": self.odom.twist.twist.linear.y,
                    "vz": self.odom.twist.twist.linear.z,

                    "wx": self.odom.twist.twist.angular.x,
                    "wy": self.odom.twist.twist.angular.y,
                    "wz": self.odom.twist.twist.angular.z
                }

                if map_pose is not None:
                    q = self.yaw_to_q(map_pose[2])
                    odom_data.update({
                        "frame_id": "map",
                        "pose_source": "map_tf",
                        "px": map_pose[0],
                        "py": map_pose[1],
                        "pz": 0.0,
                        "qx": q.x,
                        "qy": q.y,
                        "qz": q.z,
                        "qw": q.w,
                        "yaw": map_pose[2],
                        "raw_odom": {
                            "px": self.odom.pose.pose.position.x,
                            "py": self.odom.pose.pose.position.y,
                            "pz": self.odom.pose.pose.position.z,
                            "qx": self.odom.pose.pose.orientation.x,
                            "qy": self.odom.pose.pose.orientation.y,
                            "qz": self.odom.pose.pose.orientation.z,
                            "qw": self.odom.pose.pose.orientation.w,
                        }
                    })
                else:
                    odom_data.update({
                        "frame_id": "odom",
                        "pose_source": "odom",
                    })

                self.mqtt.publish(
                    "odom",
                    json.dumps(odom_data),
                    qos=0
                )

            # ================= IMU =================
            if self.imu is not None:

                imu_data = {
                    "ax": self.imu.linear_acceleration.x,
                    "ay": self.imu.linear_acceleration.y,
                    "az": self.imu.linear_acceleration.z,

                    "gx": self.imu.angular_velocity.x,
                    "gy": self.imu.angular_velocity.y,
                    "gz": self.imu.angular_velocity.z,

                    "qx": self.imu.orientation.x,
                    "qy": self.imu.orientation.y,
                    "qz": self.imu.orientation.z,
                    "qw": self.imu.orientation.w
                }

                self.mqtt.publish(
                    "imu",
                    json.dumps(imu_data),
                    qos=0
                )

        except Exception as e:
            self.get_logger().error(
                f"MQTT publish error: {e}"
            )

    def publish_cmd_vel(self, vx, wz, vy=0.0):
        try:
            self.get_logger().info(f"🎮 Parsed cmd_vel: vx={vx}, vy={vy}, wz={wz}")
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz

            self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"cmd_vel publish error: {e}")

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
            progress = self._mission_progress(message)
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

    def _mission_progress(self, message=""):
        if message == "Finish: progress = 100":
            return 100
        if self.total_waypoints <= 0:
            return 0

        completed = max(0, min(self.current_index + 1, self.total_waypoints))
        return round((completed / self.total_waypoints) * 100)

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
        if self.busy or self.paused or self.waiting_cancel or self.spin_running or self.localization_active:
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
        self.pose_correction_completed = False
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
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            self.publish_status("FAILED", "Nav2 action server not available")
            self.cancel_waypoints()
            return

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
        try:
            self.current_goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Send goal failed: {e}")
            self.publish_status("FAILED", "Failed to send goal")
            self.cancel_waypoints()
            return

        if self.current_goal_handle is None or not self.current_goal_handle.accepted:
            self.get_logger().warn("❌ rejected")
            self.publish_status("FAILED", "Goal rejected")
            self.cancel_waypoints()
            return

        self.current_goal_handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, goal)
        )

    # ================= RESULT =================
    def on_result(self, future, goal):
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f"Goal result failed: {e}")
            self.publish_status("FAILED", "Goal result failed")
            self.cancel_waypoints()
            return

        goal = self.current_goal

        if goal is None:
            self.get_logger().warn("⚠️ Goal already cleared, skip result")
            return

        x = goal["x"]
        y = goal["y"]
        yaw = goal["yaw"]
        is_capture = goal["is_capture"]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎯 arrived")
            completed_waypoints = self.current_index + 1
            progress = round((completed_waypoints / self.total_waypoints) * 100) if self.total_waypoints > 0 else 0
            self.publish_progress(f"Arrived at waypoint {completed_waypoints} of {self.total_waypoints}: progress = {progress}")

            # 🔥 lock position
            self.current_x = x
            self.current_y = y

            if self.pose_correction_active:
                self.pose_correction_active = False
                self.publish_progress(f"Position corrected at {self.current_name}")
                corrected_pose = self.get_current_pose(warn=False)
                self.publish_pose_adjust_event({
                    "eventType": "POSE_CORRECTION_COMPLETED",
                    "requestId": self.pose_adjust_state.get("requestId") if self.pose_adjust_state else None,
                    "result": "CORRECTION_REACHED",
                    "correctedPose": self.pose_to_dict(corrected_pose, "map_tf"),
                    "state": self.pose_adjust_state,
                })
                self.pose_adjust_state = None
                if is_capture:
                    self.get_logger().info("Wait 2 sec before capture spin")
                    self.capture_mode = True
                    threading.Timer(2.0, self.start_spin).start()
                else:
                    self.finish()
                return

            localize = goal.get("localize", {"enabled": False})
            if localize.get("enabled"):
                self.begin_waypoint_pose_adjust(localize)
                return

            if is_capture:
                self.get_logger().info("⏳ wait 2 sec before spin")

                self.capture_mode = True

                # ❌ ห้าม create_timer(self.start_spin())
                threading.Timer(2.0, self.start_spin).start()
            else:
                self.finish()
                
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"💥 ABORTED at goal: {self.current_goal}")
            # 💀 ถ้าเป็น pause ไม่ต้อง cancel เพราะจะ resume ต่อได้
            if self.paused:
                self.get_logger().info("⏸ Paused, waiting for resume")
                self.busy = False
                return
            if self.pose_correction_active:
                self.publish_pose_adjust_event({
                    "eventType": "POSE_CORRECTION_FAILED",
                    "requestId": self.pose_adjust_state.get("requestId") if self.pose_adjust_state else None,
                    "result": "CORRECTION_ABORTED",
                    "goalStatus": "ABORTED",
                    "state": self.pose_adjust_state,
                })
                self.pose_adjust_state = None
                self.pose_correction_active = False
            self.publish_status("FAILED", "Map change or path blocked")
            # fallback logic (สำคัญ)
            self.cancel_waypoints()

        # ❌ CANCELED (manual cancel / preempt)
        elif status == GoalStatus.STATUS_CANCELED:
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
                if self.pose_correction_active:
                    self.publish_pose_adjust_event({
                        "eventType": "POSE_CORRECTION_FAILED",
                        "requestId": self.pose_adjust_state.get("requestId") if self.pose_adjust_state else None,
                        "result": "CORRECTION_CANCELED",
                        "goalStatus": "CANCELED",
                        "state": self.pose_adjust_state,
                    })
                    self.pose_adjust_state = None
                    self.pose_correction_active = False
                self.publish_status("FAILED", "Map change or path blocked")
            self.cancel_waypoints()

        # 🤡 UNKNOWN
        else:
            self.get_logger().error(f"❓ UNKNOWN status: {status}")
            self.publish_status("CANCELLED", "Unknown")
            self.cancel_waypoints()

    # 🔄 start spin
    def begin_waypoint_pose_adjust(self, localize):
        self.pose_adjust_active = True
        self.stop_robot()
        self.publish_progress(f"Stopping before pose adjust at {self.current_name}")

        before_pose = self.get_current_pose(warn=False)
        self.pose_adjust_state = {
            "targetWaypoint": {
                "x": float(self.current_goal["x"]),
                "y": float(self.current_goal["y"]),
                "yaw": float(self.current_goal["yaw"]),
            } if self.current_goal else None,
            "beforePose": self.pose_to_dict(before_pose, "map_tf"),
            "localizeConfig": dict(localize),
            "startedAt": time.time(),
        }
        self.publish_pose_adjust_event({
            "eventType": "POSE_ADJUST_STARTED",
            "result": "PENDING",
            "beforePose": self.pose_adjust_state["beforePose"],
            "targetWaypoint": self.pose_adjust_state["targetWaypoint"],
            "localizeConfig": self.pose_adjust_state["localizeConfig"],
        })

        delay = max(0.0, float(localize.get("stopDelay", 1.0)))

        def start_after_stop():
            if not self.pose_adjust_active or self.current_goal is None:
                return
            self.publish_progress(f"Adjusting pose at {self.current_name}")
            self.start_localization(localize, source="waypoint")
            if self.pose_adjust_state is not None:
                self.pose_adjust_state["requestId"] = self.localization_request_id

        self.pose_adjust_timer = threading.Timer(delay, start_after_stop)
        self.pose_adjust_timer.start()

    def settle_after_pose_adjust(self):
        if not self.current_goal:
            return

        self.stop_robot()
        self.publish_progress(f"Pose adjusted, waiting to settle at {self.current_name}")

        localize = self.current_goal.get("localize", {"enabled": False})
        settle_time = max(0.0, float(localize.get("settleTime", 2.0)))

        self.pose_adjust_timer = threading.Timer(settle_time, self.after_waypoint_localization)
        self.pose_adjust_timer.start()

    def angle_error(self, target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def needs_position_correction(self, pose):
        if not self.current_goal:
            return False

        localize = self.current_goal.get("localize", {"enabled": False})
        if not localize.get("correctPosition", True):
            return False
        if self.pose_correction_completed:
            return False

        target_x = float(self.current_goal["x"])
        target_y = float(self.current_goal["y"])
        target_yaw = float(self.current_goal["yaw"])

        position_error = math.hypot(target_x - pose[0], target_y - pose[1])
        yaw_error = abs(self.angle_error(target_yaw, pose[2]))
        position_tolerance = max(0.0, float(localize.get("positionTolerance", 0.15)))
        yaw_tolerance = max(0.0, float(localize.get("yawTolerance", 0.15)))

        self.get_logger().info(
            f"Pose correction check: position_error={position_error:.3f}m, "
            f"yaw_error={math.degrees(yaw_error):.1f}deg"
        )

        return position_error > position_tolerance or yaw_error > yaw_tolerance

    def start_position_correction(self):
        if not self.current_goal:
            return

        self.pose_correction_active = True
        self.pose_correction_completed = True
        self.stop_robot()
        self.publish_progress(f"Correcting position at {self.current_name}")
        self.publish_pose_adjust_event({
            "eventType": "POSE_CORRECTION_STARTED",
            "requestId": self.pose_adjust_state.get("requestId") if self.pose_adjust_state else None,
            "result": "CORRECTION_NEEDED",
            "state": self.pose_adjust_state,
        })
        self.send_goal(self.current_goal)

    def after_waypoint_localization(self):
        self.pose_adjust_active = False
        self.pose_adjust_timer = None

        pose = self.get_current_pose()
        state = self.pose_adjust_state or {}
        before_pose = self.pose_dict_to_tuple(state.get("beforePose"))
        localized_pose = None
        localization_result = state.get("localizationResult")
        if isinstance(localization_result, dict):
            raw_pose = localization_result.get("pose")
            if isinstance(raw_pose, dict):
                try:
                    localized_pose = (
                        float(raw_pose.get("x", 0.0)),
                        float(raw_pose.get("y", 0.0)),
                        float(raw_pose.get("yaw", 0.0)),
                    )
                except Exception:
                    localized_pose = None

        if pose is not None:
            self.current_x = pose[0]
            self.current_y = pose[1]
            self.get_logger().info(
                f"Pose adjusted to x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={math.degrees(pose[2]):.1f} deg"
            )
            correction_needed = self.needs_position_correction(pose)
            self.publish_pose_adjust_event({
                "eventType": "POSE_ADJUST_SETTLED",
                "requestId": state.get("requestId"),
                "result": "CORRECTION_NEEDED" if correction_needed else "ADJUSTED_NO_CORRECTION",
                "beforePose": state.get("beforePose"),
                "localizedPose": self.pose_to_dict(localized_pose, "apriltag"),
                "afterSettlePose": self.pose_to_dict(pose, "map_tf"),
                "deltaFromBefore": self.build_pose_delta(before_pose, pose),
                "deltaFromLocalized": self.build_pose_delta(localized_pose, pose),
                "targetWaypoint": state.get("targetWaypoint"),
                "localizeConfig": state.get("localizeConfig"),
                "correctionNeeded": correction_needed,
            })
            if correction_needed:
                self.start_position_correction()
                return
            self.pose_adjust_state = None

        if self.current_goal and self.current_goal.get("is_capture"):
            self.get_logger().info("Wait 2 sec before capture spin")
            self.capture_mode = True
            threading.Timer(2.0, self.start_spin).start()
        else:
            self.finish()

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
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Spin goal failed: {e}")
            self.cancel_waypoints()
            return

        if goal_handle is None or not goal_handle.accepted:
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
        ok, jpg = cv2.imencode('.jpg', frame)
        if not ok:
            self.get_logger().warn("JPEG encode failed")
            return
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
        self.localization_active = False
        self.localization_request_id = None
        self.localization_context = None
        self.pose_adjust_active = False
        if self.pose_adjust_timer is not None:
            self.pose_adjust_timer.cancel()
        self.pose_adjust_timer = None
        self.pose_correction_active = False
        self.pose_correction_completed = False
        self.pose_adjust_state = None
        self.mission_tag_poses = {}
        self.status_pub = None
        self.progress_pub = None

        self._send_localization_abort()
        self.stop_robot()

        # 3. reset state
        self.busy = False
        self.current_goal = None
        self.current_goal_handle = None

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
