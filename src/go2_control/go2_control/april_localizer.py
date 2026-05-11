import json
import math
import time
import uuid

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo
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


def quaternion_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.identity(3)
    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def yaw_from_matrix(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def make_transform(rotation, translation):
    transform = np.identity(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.array(translation, dtype=np.float64)
    return transform


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
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera_info"
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
        self.tag_size = float(self.declare_parameter("tag_size", 0.16).value)
        self.tag_map_poses = self._load_tag_map_poses(
            self.declare_parameter("tag_map_poses", "{}").value
        )

        self.active_request = None
        self.accepted_poses = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_scan_switch = 0.0
        self.scan_direction = 1.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.request_sub = self.create_subscription(
            String, self.request_topic, self.request_cb, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_cb, 10
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
        self.create_subscription(String, "/localization/abort", self.abort_cb, 10)
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("AprilTag localizer ready")

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _load_tag_map_poses(self, raw):
        try:
            data = json.loads(raw) if isinstance(raw, str) else raw
        except Exception as e:
            self.get_logger().error(f"Invalid tag_map_poses JSON: {e}")
            return {}

        poses = {}
        for tag_id, pose in data.items():
            try:
                yaw = (
                    math.radians(float(pose.get("yawDegrees")))
                    if "yawDegrees" in pose
                    else float(pose.get("yaw", 0.0))
                )
                roll = (
                    math.radians(float(pose.get("rollDegrees")))
                    if "rollDegrees" in pose
                    else float(pose.get("roll", 0.0))
                )
                pitch = (
                    math.radians(float(pose.get("pitchDegrees")))
                    if "pitchDegrees" in pose
                    else float(pose.get("pitch", 0.0))
                )
                poses[int(tag_id)] = {
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "z": float(pose.get("z", 0.0)),
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "is_3d": any(key in pose for key in ("z", "roll", "pitch", "rollDegrees", "pitchDegrees")),
                }
            except Exception as e:
                self.get_logger().warn(f"Skipping invalid tag pose {tag_id}: {e}")
        return poses

    def request_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self._publish_status("FAILED", message=f"Invalid request JSON: {e}")
            return

        if "tagPoses" in data and data["tagPoses"]:
            self.tag_map_poses = self._load_tag_map_poses(data["tagPoses"])
            self.get_logger().info(
                f"Using tagPoses from request: {sorted(self.tag_map_poses.keys())}"
            )
        else:
            self.reload_tag_map_poses()

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

    def reload_tag_map_poses(self):
        raw = self.get_parameter("tag_map_poses").value
        self.tag_map_poses = self._load_tag_map_poses(raw)
        self.get_logger().info(
            f"Loaded AprilTag map poses for IDs: {sorted(self.tag_map_poses.keys())}"
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

            # stop rotating as soon as first valid detection arrives
            if self.active_request.get("scan") and len(self.accepted_poses) == 0:
                self._stop_robot()
                self.active_request["scan"] = False

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

        camera_frame = header.frame_id or self.default_camera_frame
        tag_in_camera = self._get_tag_pose_from_detection(detection)
        if tag_in_camera is None:
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            tag_in_base = self._transform_pose_to_base(tag_in_camera, tf)
        except Exception as e:
            self.get_logger().warn(f"TF {self.base_frame}<-{camera_frame} failed: {e}")
            return None

        map_pose = self.tag_map_poses[tag_id]
        if isinstance(tag_in_base, dict) and map_pose.get("is_3d"):
            return self._calculate_robot_pose_3d(map_pose, tag_in_base)

        base_in_tag = invert_pose_2d(*tag_in_base)
        tag_map_pose_2d = (map_pose["x"], map_pose["y"], map_pose["yaw"])
        return compose_pose_2d(tag_map_pose_2d, base_in_tag)

    def _get_tag_pose_from_detection(self, detection):
        try:
            tag_pose = detection.pose.pose.pose
            return (
                float(tag_pose.position.x),
                float(tag_pose.position.y),
                quaternion_to_yaw(tag_pose.orientation),
            )
        except AttributeError:
            pass

        try:
            tag_pose = detection.pose.pose
            return (
                float(tag_pose.position.x),
                float(tag_pose.position.y),
                quaternion_to_yaw(tag_pose.orientation),
            )
        except AttributeError:
            pass

        return self._estimate_tag_pose_from_corners(detection)

    def _estimate_tag_pose_from_corners(self, detection):
        if self.camera_matrix is None:
            self.get_logger().warn("No CameraInfo yet; cannot estimate AprilTag pose")
            return None
        if self.tag_size <= 0.0:
            self.get_logger().warn("tag_size must be greater than 0")
            return None

        try:
            image_points = np.array(
                [[corner.x, corner.y] for corner in detection.corners],
                dtype=np.float64,
            )
        except Exception as e:
            self.get_logger().warn(f"AprilTag corners unavailable: {e}")
            return None

        if image_points.shape != (4, 2):
            self.get_logger().warn("AprilTag detection must contain 4 corners")
            return None

        half = self.tag_size / 2.0
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            self.get_logger().warn("solvePnP failed for AprilTag detection")
            return None

        rotation, _ = cv2.Rodrigues(rvec)
        return {
            "translation": tvec.reshape(3),
            "rotation": rotation,
        }

    def _transform_pose_to_base(self, tag_in_camera, camera_to_base_tf):
        tf_rotation = quaternion_to_matrix(camera_to_base_tf.transform.rotation)
        tf_translation = np.array(
            [
                camera_to_base_tf.transform.translation.x,
                camera_to_base_tf.transform.translation.y,
                camera_to_base_tf.transform.translation.z,
            ],
            dtype=np.float64,
        )

        if isinstance(tag_in_camera, dict):
            tag_translation_base = (
                tf_rotation @ tag_in_camera["translation"] + tf_translation
            )
            tag_rotation_base = tf_rotation @ tag_in_camera["rotation"]
            return {
                "translation": tag_translation_base,
                "rotation": tag_rotation_base,
            }

        tag_x, tag_y, tag_yaw = tag_in_camera
        tag_translation_base = tf_rotation @ np.array([tag_x, tag_y, 0.0]) + tf_translation
        base_yaw_camera = quaternion_to_yaw(camera_to_base_tf.transform.rotation)
        return (
            float(tag_translation_base[0]),
            float(tag_translation_base[1]),
            normalize_angle(base_yaw_camera + tag_yaw),
        )

    def _calculate_robot_pose_3d(self, tag_map_pose, tag_in_base):
        tag_in_map = make_transform(
            rpy_to_matrix(
                tag_map_pose["roll"],
                tag_map_pose["pitch"],
                tag_map_pose["yaw"],
            ),
            [tag_map_pose["x"], tag_map_pose["y"], tag_map_pose["z"]],
        )
        tag_in_base_transform = make_transform(
            tag_in_base["rotation"],
            tag_in_base["translation"],
        )
        base_in_map = tag_in_map @ np.linalg.inv(tag_in_base_transform)
        return (
            float(base_in_map[0, 3]),
            float(base_in_map[1, 3]),
            yaw_from_matrix(base_in_map[:3, :3]),
        )

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

    def abort_cb(self, msg):
        if self.active_request is None:
            return
        self._stop_robot()
        self.active_request = None
        self.accepted_poses = []
        self.get_logger().info("Localization aborted")

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
