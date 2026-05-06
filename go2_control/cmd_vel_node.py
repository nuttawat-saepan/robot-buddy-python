import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time
import sys
import roslibpy
import json
from std_msgs.msg import String


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ===== INIT ROSBRIDGE =====
        self.ros_client = roslibpy.Ros(host='192.168.68.53', port=9090)
        self.ros_client.run()
        
        # Create topic for cmd_vel publishing
        self.cmd_vel_pub = roslibpy.Topic(self.ros_client, '/cmd_vel', 'geometry_msgs/Twist')
        
        self.override_stopped = False
        self.last_cmd = None

        # ===== PARAMETERS =====
        self.max_vx = self.declare_parameter("max_vx", 0.22).value
        self.max_vy = self.declare_parameter("max_vy", 0.0).value
        self.max_vyaw = self.declare_parameter("max_vyaw", 1.0).value

        self.alpha = self.declare_parameter("alpha", 0.2).value   # smoothing
        self.max_acc = self.declare_parameter("max_acc", 0.05).value

        # ===== STATE =====
        self.stopped = False   # 🔥 กันยิง stop ซ้ำ
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vyaw = 0.0

        self.last_time = self.get_clock().now()

        # ===== SUB =====
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )
        self.sub_cmd = self.create_subscription(
            String,
            '/unitree_cmd',
            self.cmd_callback,
            10
        )

        # ===== TIMER =====
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("CmdVel Bridge (Smoothed) started")

    def callback(self, msg):
        self.last_time = self.get_clock().now()

        self.target_vx = self.clamp(msg.linear.x / self.max_vx)
        self.target_vy = self.clamp(msg.linear.y / self.max_vy) if self.max_vy > 0 else 0.0
        self.target_vyaw = self.clamp(msg.angular.z / self.max_vyaw)

        self.get_logger().info(
            f"📥 cmd_vel | raw: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, yaw={msg.angular.z:.2f}"
        )

    def cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cmd = data.get("cmd", "")

            # 🔥 กันยิงซ้ำ
            if cmd == self.last_cmd:
                return

            self.last_cmd = cmd

            # ===== COMMAND (publish via rosbridge) =====
            if cmd == "stand":
                self.get_logger().info("🟢 STAND")
                # Publish stand command
                cmd_msg = {"cmd": "stand"}
                self.publish_cmd(cmd_msg)

            elif cmd == "sit":
                self.get_logger().info("🪑 SIT")
                # Publish sit command
                cmd_msg = {"cmd": "sit"}
                self.publish_cmd(cmd_msg)

            elif cmd == "stop":
                self.get_logger().warn("🛑 STOP")
                # Publish stop command
                cmd_msg = {"cmd": "stop"}
                self.publish_cmd(cmd_msg)

            else:
                self.get_logger().warn(f"Unknown cmd: {cmd}")

        except Exception as e:
            self.get_logger().error(f"cmd parse error: {e}")

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # ===== ไม่มี cmd_vel เข้ามา =====
        if dt > 0.3:
            if not self.stopped:
                self.get_logger().warn("🛑 STOP (no cmd_vel)")
                self.publish_twist(0.0, 0.0, 0.0)
                self.stopped = True
            return

        # ===== smoothing =====
        self.vx = (1 - self.alpha) * self.vx + self.alpha * self.target_vx
        self.vy = (1 - self.alpha) * self.vy + self.alpha * self.target_vy
        self.vyaw = (1 - self.alpha) * self.vyaw + self.alpha * self.target_vyaw

        # ===== acceleration limit =====
        self.vx = self.limit_acc(self.vx, self.prev_vx)
        self.vy = self.limit_acc(self.vy, self.prev_vy)
        self.vyaw = self.limit_acc(self.vyaw, self.prev_vyaw)

        self.prev_vx = self.vx
        self.prev_vy = self.vy
        self.prev_vyaw = self.vyaw

        # ===== ตรวจว่าเป็น 0 =====
        if abs(self.vx) < 0.01 and abs(self.vy) < 0.01 and abs(self.vyaw) < 0.01:
            if not self.stopped:
                self.get_logger().info("🛑 STOP (zero velocity)")
                self.publish_twist(0.0, 0.0, 0.0)
                self.stopped = True
            return
        else:
            self.stopped = False  # 🔥 มีการเคลื่อนที่ → reset

        # ===== publish via rosbridge =====
        self.get_logger().info(
            f"🚀 Move | vx={self.vx:.2f}, vy={self.vy:.2f}, yaw={self.vyaw:.2f}"
        )
        self.publish_twist(self.vx, self.vy, self.vyaw)

    def publish_twist(self, vx, vy, vyaw):
        """Publish Twist message via rosbridge."""
        twist_msg = {
            'linear': {
                'x': float(vx),
                'y': float(vy),
                'z': 0.0
            },
            'angular': {
                'x': 0.0,
                'y': 0.0,
                'z': float(vyaw)
            }
        }
        self.cmd_vel_pub.publish(twist_msg)

    def publish_cmd(self, cmd_msg):
        """Publish command via rosbridge."""
        cmd_topic = roslibpy.Topic(self.ros_client, '/unitree_cmd', 'std_msgs/String')
        cmd_topic.publish({'data': json.dumps(cmd_msg)})

    def close_rosbridge(self):
        """Close rosbridge connection."""
        if hasattr(self, 'ros_client') and self.ros_client.is_connected:
            self.ros_client.close()
            self.get_logger().info('Rosbridge connection closed.')

    def clamp(self, v):
        return max(-1.0, min(1.0, v))

    def limit_acc(self, current, prev):
        delta = current - prev
        if delta > self.max_acc:
            return prev + self.max_acc
        if delta < -self.max_acc:
            return prev - self.max_acc
        return current

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_rosbridge()
        node.destroy_node()
        rclpy.shutdown()