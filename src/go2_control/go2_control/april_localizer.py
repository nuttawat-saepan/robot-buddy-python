import json
import math
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:
    AprilTagDetectionArray = None


def yaw_to_quaternion(yaw):
    half = yaw / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def invert_pose_2d(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return -(c * x + s * y), (s * x - c * y), -yaw


def compose_pose_2d(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, normalize_angle(ayaw + byaw)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AprilLocalizer(Node):
    def __init__(self):
        super().__init__("april_localizer")

        self.detections_topic = self.declare_parameter(
            "detections_topic", "/detections"
        ).value
        self.request_topic = self.declare_parameter(
            "request_topic", "/localization/request"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/localization/status"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.default_camera_frame = self.declare_parameter(
            "camera_frame", "camera_link"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.default_timeout = float(
            self.declare_parameter("default_timeout", 8.0).value
        )
        self.default_scan_timeout = float(
            self.declare_parameter("default_scan_timeout", 20.0).value
        )
        self.default_rotation_speed = float(
            self.declare_parameter("rotation_speed", 0.25).value
        )
        self.min_detections = int(self.declare_parameter("min_detections", 3).value)
        self.initial_pose_covariance = float(
            self.declare_parameter("initial_pose_covariance", 0.05).value
        )
        self.tag_map_poses = self._load_tag_map_poses(
            self.declare_parameter("tag_map_poses", "{}").value
        )

        self.active_request = None
        self.accepted_poses = []
        self.last_scan_switch = 0.0
        self.scan_direction = 1.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.request_sub = self.create_subscription(
            String, self.request_topic, self.request_cb, 10
        )
        if AprilTagDetectionArray is None:
            self.detection_sub = None
            self.get_logger().error(
                "apriltag_msgs is not installed; AprilTag localization cannot run"
            )
        else:
            self.detection_sub = self.create_subscription(
                AprilTagDetectionArray, self.detections_topic, self.detection_cb, 10
            )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("AprilTag localizer ready")

    def _load_tag_map_poses(self, raw):
        try:
            data = json.loads(raw) if isinstance(raw, str) else raw
        except Exception as e:
            self.get_logger().error(f"Invalid tag_map_poses JSON: {e}")
            return {}

        poses = {}
        for tag_id, pose in data.items():
            try:
                poses[int(tag_id)] = (
                    float(pose.get("x", 0.0)),
                    float(pose.get("y", 0.0)),
                    math.radians(float(pose.get("yawDegrees")))
                    if "yawDegrees" in pose
                    else float(pose.get("yaw", 0.0)),
                )
            except Exception as e:
                self.get_logger().warn(f"Skipping invalid tag pose {tag_id}: {e}")
        return poses

    def request_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self._publish_status("FAILED", message=f"Invalid request JSON: {e}")
            return

        localize = data.get("localize", data)
        if not isinstance(localize, dict):
            self._publish_status("FAILED", message="localize payload must be an object")
            return

        scan = bool(localize.get("scan", False))
        timeout = float(
            localize.get(
                "timeout",
                self.default_scan_timeout if scan else self.default_timeout,
            )
        )
        request_id = data.get("requestId") or localize.get("requestId") or str(uuid.uuid4())

        self.active_request = {
            "requestId": request_id,
            "scan": scan,
            "expectedTagId": localize.get("expectedTagId"),
            "timeout": timeout,
            "rotationSpeed": float(
                localize.get("rotationSpeed", self.default_rotation_speed)
            ),
            "startedAt": time.monotonic(),
            "source": data.get("source", localize.get("source", "manual")),
        }
        self.accepted_poses = []
        self.last_scan_switch = time.monotonic()
        self.scan_direction = 1.0

        self._stop_robot()
        self._publish_status(
            "STARTED",
            message="AprilTag localization started",
            extra={"scan": scan, "expectedTagId": self.active_request["expectedTagId"]},
        )

    def update(self):
        if self.active_request is None:
            return

        elapsed = time.monotonic() - self.active_request["startedAt"]
        if elapsed > self.active_request["timeout"]:
            self._fail("AprilTag localization timed out")
            return

        if self.active_request["scan"]:
            now = time.monotonic()
            if now - self.last_scan_switch > 3.0:
                self.scan_direction *= -1.0
                self.last_scan_switch = now

            cmd = Twist()
            cmd.angular.z = (
                self.scan_direction * self.active_request["rotationSpeed"]
            )
            self.cmd_vel_pub.publish(cmd)

    def detection_cb(self, msg):
        if self.active_request is None:
            return

        for detection in msg.detections:
            tag_id = self._get_detection_id(detection)
            expected = self.active_request.get("expectedTagId")
            if expected is not None and tag_id != int(expected):
                continue

            robot_pose = self._calculate_robot_pose(tag_id, detection, msg.header)
            if robot_pose is None:
                continue

            self.accepted_poses.append(robot_pose)
            if len(self.accepted_poses) >= self.min_detections:
                self._succeed(tag_id)
            return

    def _get_detection_id(self, detection):
        raw_id = getattr(detection, "id", None)
        if isinstance(raw_id, (list, tuple)):
            return int(raw_id[0])
        return int(raw_id)

    def _calculate_robot_pose(self, tag_id, detection, header):
        if tag_id not in self.tag_map_poses:
            self.get_logger().warn(f"No map pose configured for AprilTag {tag_id}")
            return None

        try:
            tag_pose = detection.pose.pose.pose
        except AttributeError:
            try:
                tag_pose = detection.pose.pose
            except AttributeError:
                self.get_logger().warn("Unsupported AprilTag detection pose shape")
                return None

        camera_frame = header.frame_id or self.default_camera_frame
        tag_in_camera = (
            float(tag_pose.position.x),
            float(tag_pose.position.y),
            quaternion_to_yaw(tag_pose.orientation),
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                camera_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            base_in_camera = (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                quaternion_to_yaw(tf.transform.rotation),
            )
        except Exception as e:
            self.get_logger().warn(f"TF {camera_frame}->{self.base_frame} failed: {e}")
            return None

        tag_in_base = compose_pose_2d(invert_pose_2d(*base_in_camera), tag_in_camera)
        base_in_tag = invert_pose_2d(*tag_in_base)
        return compose_pose_2d(self.tag_map_poses[tag_id], base_in_tag)

    def _succeed(self, tag_id):
        self._stop_robot()
        pose = self._average_pose(self.accepted_poses[-self.min_detections :])
        self._publish_initial_pose(pose)
        self._publish_status(
            "SUCCESS",
            message="AprilTag localization succeeded",
            extra={
                "tagId": tag_id,
                "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
            },
        )
        self.active_request = None
        self.accepted_poses = []

    def _fail(self, message):
        self._stop_robot()
        self._publish_status("FAILED", message=message)
        self.active_request = None
        self.accepted_poses = []

    def _average_pose(self, poses):
        x = sum(p[0] for p in poses) / len(poses)
        y = sum(p[1] for p in poses) / len(poses)
        sin_sum = sum(math.sin(p[2]) for p in poses)
        cos_sum = sum(math.cos(p[2]) for p in poses)
        return x, y, math.atan2(sin_sum, cos_sum)

    def _publish_initial_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        q = yaw_to_quaternion(pose[2])
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = self.initial_pose_covariance
        msg.pose.covariance[7] = self.initial_pose_covariance
        msg.pose.covariance[35] = self.initial_pose_covariance
        self.initial_pose_pub.publish(msg)

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status, message="", extra=None):
        payload = {
            "requestId": self.active_request.get("requestId")
            if self.active_request
            else None,
            "source": self.active_request.get("source") if self.active_request else None,
            "status": status,
            "message": message,
            "timestamp": int(time.time() * 1000),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
