import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

import json
import os
import math
import threading
import subprocess
import re

SAVE_DIR = "waypoints"
os.makedirs(SAVE_DIR, exist_ok=True)


class WaypointGen(Node):

    def __init__(self):
        super().__init__('waypoint_gen')

        self.current_pose = None
        self.waypoints = []
        self.running = True
        self.bootstrap_done = False
        self.bootstrap_pose = None

        # subscribe AMCL
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info("Waiting for /amcl_pose stream...")

        # ไม่ block แล้ว (เหมือน ros2 topic echo)
        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.start()

    def bootstrap_amcl(self):
        if self.bootstrap_done:
            return

        print("[BOOTSTRAP] getting /amcl_pose once via ros2 echo...")

        result = subprocess.run(
            ["ros2", "topic", "echo", "/amcl_pose", "--once"],
            capture_output=True,
            text=True
        ).stdout

        # ดึงค่า x,y แบบง่าย ๆ (กันพัง)
        try:
            x = re.search(r"x:\s*([-\d.]+)", result)
            y = re.search(r"y:\s*([-\d.]+)", result)
            yaw = 0.0

            if x and y:
                self.bootstrap_pose = {
                    "x": float(x.group(1)),
                    "y": float(y.group(1)),
                    "yaw": yaw
                }

                print("[BOOTSTRAP OK]", self.bootstrap_pose)

        except:
            print("[BOOTSTRAP FAIL]")

        self.bootstrap_done = True

    # -------------------------
    # CALLBACK (echo style buffer)
    # -------------------------
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    # -------------------------
    # QUAT -> YAW
    # -------------------------
    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # -------------------------
    # GET LATEST (echo behavior)
    # -------------------------
    def get_latest_pose(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose

    # -------------------------
    # ADD WAYPOINT
    # -------------------------
    def add_waypoint(self):
        pose = self.current_pose

        # ถ้ายังไม่มี AMCL → ใช้ bootstrap
        if pose is None:

            if not self.bootstrap_done:
                self.bootstrap_amcl()

            if self.bootstrap_pose is None:
                print("[WARN] no pose available")
                return

            self.waypoints.append(self.bootstrap_pose)
            print(f"[ADD BOOTSTRAP] WP{len(self.waypoints)} -> {self.bootstrap_pose}")
            return

        yaw = self.quaternion_to_yaw(pose.orientation)

        wp = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw
        }

        self.waypoints.append(wp)
        print(f"[ADD] WP{len(self.waypoints)} -> {wp}")

    # -------------------------
    # SAVE FILE
    # -------------------------
    def get_next_filename(self):
        i = 1
        while True:
            path = os.path.join(SAVE_DIR, f"wp{i}.json")
            if not os.path.exists(path):
                return path
            i += 1

    def save_and_exit(self):
        if len(self.waypoints) == 0:
            print("[WARN] No waypoints")
            self.running = False
            return

        path = self.get_next_filename()

        with open(path, "w") as f:
            json.dump({"waypoints": self.waypoints}, f, indent=4)

        print(f"[SAVED] {path}")

        self.running = False
        self.destroy_node()

    # -------------------------
    # KEYBOARD LOOP
    # -------------------------
    def keyboard_loop(self):
        while self.running:
            cmd = input().strip()

            if cmd == "a":
                self.add_waypoint()

            elif cmd == "q":
                self.save_and_exit()
                break


# -------------------------
# MAIN
# -------------------------
def main():
    rclpy.init()
    node = WaypointGen()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()