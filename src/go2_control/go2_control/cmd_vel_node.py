import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import json

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ===== INIT UNITREE =====
        ChannelFactoryInitialize(0, "enp5s0")

        self.client = SportClient()
        self.client.SetTimeout(1.0)
        self.client.Init()

        # ===== LIMIT =====
        self.max_vx = 1.0
        self.max_vy = 1.0  
        self.max_vyaw = 1.0

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # ===== STATE =====
        self.last_cmd = None
        self.last_time = self.get_clock().now()
        self.stopped = False
        self.zero_sent = False

        # ===== SUB =====
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            qos
        )

        self.sub_cmd = self.create_subscription(
            String,
            '/unitree_cmd',
            self.cmd_callback,
            qos
        )

        # ===== TIMER (auto stop only) =====
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("CmdVel Direct + AutoStop + ZeroOnce started")

    # =========================
    # CMD_VEL → MOVE
    # =========================
    def callback(self, msg: Twist):
        try:
            self.last_time = self.get_clock().now()

            self.target_vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
            self.target_vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
            self.target_wz = max(-self.max_vyaw, min(self.max_vyaw, msg.angular.z))

        except Exception as e:
            self.get_logger().error(f"cmd_vel error: {e}")

    def update(self):
        try:
            if abs(self.target_vx) < 1e-3 and abs(self.target_wz) < 1e-3 and abs(self.target_vy) < 1e-3:
                if not self.zero_sent:
                    self.client.Move(0.0, 0.0, 0.0)
                    self.get_logger().info("🛑 STOP (zero velocity)")
                    self.zero_sent = True
                return

            self.zero_sent = False
            self.stopped = False

            self.client.Move(self.target_vx, self.target_vy, self.target_wz)
            self.get_logger().info("MOVE cmd_vel: vx=%.2f, wz=%.2f" % (self.target_vx, self.target_wz))
        except Exception as e:
            self.get_logger().error(f"update error: {e}")   

    # =========================
    # AUTO STOP (กัน cmd หาย)
    # =========================
    # def update(self):
    #     try:
    #         dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

    #         if dt > 0.2:
    #             if not self.stopped:
    #                 self.client.Move(0.0, 0.0, 0.0)
    #                 self.get_logger().info("🛑 STOP (zero velocity)")
    #                 self.stopped = True
    #                 self.zero_sent = True

    #     except Exception as e:
    #         self.get_logger().error(f"update error: {e}")

    # =========================
    # COMMAND
    # =========================
    def cmd_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = data.get("cmd", "")

            if cmd == self.last_cmd:
                return
            self.last_cmd = cmd

            if cmd == "stand":
                self.get_logger().info("STAND")
                self.client.StandUp()

            elif cmd == "sit":
                self.get_logger().info("SIT")
                self.client.Sit()

            elif cmd == "stop":
                self.get_logger().warn("STOP")
                self.client.StopMove()

            else:
                self.get_logger().warn(f"Unknown cmd: {cmd}")

        except Exception as e:
            self.get_logger().error(f"cmd parse error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()