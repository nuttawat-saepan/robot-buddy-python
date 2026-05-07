#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, BatteryState, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from go2_interfaces.msg import MotorTemperature
from tf2_ros import TransformBroadcaster

# Motor index order
MOTOR_INDICES = (3, 4, 5, 13, 0, 1, 2, 12, 9, 10, 11, 15, 6, 7, 8, 14)

JOINT_SUFFIXES = (
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FL_foot_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FR_foot_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RL_foot_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RR_foot_joint',
)


class RobotRead(Node):

    def __init__(self):
        super().__init__('robot_read_node')

        # ===== Names =====
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self._joint_names = list(JOINT_SUFFIXES)
        self._motor_names = list(JOINT_SUFFIXES)

        self.sim_mode = self.declare_parameter('sim_mode', False).value

        # ===== Messages =====
        self._imu_msg = Imu()

        self._imu_temp_msg = Float32()

        self._odom_msg = Odometry()
        

        self._tf_msg = TransformStamped()
        self._battery_msg = BatteryState()
        self._battery_msg.header.frame_id = 'base_link'

        self._joint_state_msg = JointState()
        self._joint_state_msg.name = self._joint_names

        self._motor_temp_msg = MotorTemperature()
        self._motor_temp_msg.motor_name = self._motor_names

        if self.sim_mode:
            self._imu_msg.header.frame_id = 'imu_link'
            self._odom_msg.header.frame_id = 'odom'
            self._odom_msg.child_frame_id = 'base_footprint'
            self._tf_msg.header.frame_id = 'odom'
            self._tf_msg.child_frame_id = 'base_footprint'
        else:
            self._imu_msg.header.frame_id = 'imu'
            self._odom_msg.header.frame_id = 'odom'
            self._odom_msg.child_frame_id = 'base_link'
            self._tf_msg.header.frame_id = 'odom'
            self._tf_msg.child_frame_id = 'base_link'

        # ===== TF =====
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== Subscribers (native ROS2) =====
        self.create_subscription(
            SportModeState,
            '/lf/sportmodestate',
            self._sport_callback,
            qos_best_effort
        )

        self.create_subscription(
            LowState,
            '/lf/lowstate',
            self.low_state_callback,
            qos_best_effort
        )

        self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.lidar_callback,
            qos_best_effort
        )

        # ===== Publishers =====
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.imu_temp_pub = self.create_publisher(Float32, 'imu_temp', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_temp_pub = self.create_publisher(MotorTemperature, 'motor_temp', 10)

        self.get_logger().info("💀 RobotRead (ROS2 native) ready")

    # ================= SPORT =================
    def _sport_callback(self, msg):
        stamp = self.get_clock().now().to_msg()

        # IMU
        imu = self._imu_msg
        imu.header.stamp = stamp
        imu.orientation.x = float(msg.imu_state.quaternion[1])
        imu.orientation.y = float(msg.imu_state.quaternion[2])
        imu.orientation.z = float(msg.imu_state.quaternion[3])
        imu.orientation.w = float(msg.imu_state.quaternion[0])
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu)

        self._imu_temp_msg.data = float(msg.imu_state.temperature)
        self.imu_temp_pub.publish(self._imu_temp_msg)

        # ODOM
        odom = self._odom_msg
        odom.header.stamp = stamp
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
        odom.pose.pose.orientation.x = float(msg.imu_state.quaternion[1])
        odom.pose.pose.orientation.y = float(msg.imu_state.quaternion[2])
        odom.pose.pose.orientation.z = float(msg.imu_state.quaternion[3])
        odom.pose.pose.orientation.w = float(msg.imu_state.quaternion[0])

        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)

        self.odom_pub.publish(odom)

        # TF
        tf = self._tf_msg
        tf.header.stamp = stamp
        tf.transform.translation.x = float(odom.pose.pose.position.x)
        tf.transform.translation.y = float(odom.pose.pose.position.y)
        tf.transform.translation.z = float(odom.pose.pose.position.z)
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

    # ================= LIDAR =================
    def lidar_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.sim_mode:
            msg.header.frame_id = 'base_scan'
        else:
            msg.header.frame_id = 'radar'
        self.lidar_pub.publish(msg)

    # ================= LOW STATE =================
    def low_state_callback(self, msg):
        stamp = self.get_clock().now().to_msg()
        motors = msg.motor_state

        # Battery
        bat = self._battery_msg
        bat.header.stamp = stamp
        bat.voltage = float(msg.power_v)
        bat.current = float(msg.bms_state.current)
        bat.percentage = float(msg.bms_state.soc) / 100.0
        self.battery_pub.publish(bat)

        # Joint
        js = self._joint_state_msg
        js.header.stamp = stamp
        if self.sim_mode:
            # ในโหมด sim เราอาจจะไม่มีข้อมูล joint จริงๆ ก็ได้ (ขึ้นกับว่า Gazebo ส่งมาหรือเปล่า)
            # ถ้าไม่มี เราก็แค่ปล่อยเป็น 0 ทั้งหมด หรือจะเอาค่าจาก TB3 มาหลอกๆ ก็ได้ครับ
            # เปลี่ยนชื่อ Joint ให้ตรงกับ URDF ของ TurtleBot3 
            # ปกติ TB3 จะมีแค่ 2 joints สำหรับล้อ
            js.name = ['wheel_left_joint', 'wheel_right_joint']
            
            # สมมติว่าเอาค่าจากมอเตอร์ตัวที่ 0 และ 1 ใน LowState มาเป็นค่าล้อ
            # หรือถ้าไม่ได้ใช้ ก็เซตเป็น 0.0 ไว้ครับ
            js.position = [float(motors[0].q), float(motors[1].q)]
            js.velocity = [float(motors[0].dq), float(motors[1].dq)]
            
            # หมายเหตุ: ถ้าคุณรัน 'robot_state_publisher' ของ TB3 อยู่แล้ว 
            # การพ่น joint_states ซ้ำอาจทำให้ล้อใน RViz สั่น (Jitter)
            # ถ้าเกิดอาการนั้น ให้คอมเมนต์บรรทัด publish ข้างล่างนี้ทิ้งครับ
        else:
            js.position = [float(motors[i].q) for i in MOTOR_INDICES]
            js.velocity = [float(motors[i].dq) for i in MOTOR_INDICES]
        self.joint_state_pub.publish(js)

        # Motor temp
        mt = self._motor_temp_msg
        mt.header.stamp = stamp
        mt.temperature = [float(motors[i].temperature) for i in MOTOR_INDICES]
        self.motor_temp_pub.publish(mt)


def main():
    rclpy.init()
    node = RobotRead()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
