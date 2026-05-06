#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import roslibpy
import threading

from unitree_go.msg import LowState, SportModeState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, BatteryState, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from go2_interfaces.msg import MotorTemperature
from tf2_ros import TransformBroadcaster

# Motor index order: FL(3,4,5,13) FR(0,1,2,12) RL(9,10,11,15) RR(6,7,8,14)
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

        self.declare_parameter('prefix', '')
        self.prefix = self.get_parameter('prefix').value
        self.get_logger().info(f"Using prefix: '{self.prefix}'")

        # Cached name lists
        self._joint_names = [f'{self.prefix}{s}' for s in JOINT_SUFFIXES]
        self._motor_names = list(self._joint_names)

        # Pre-allocate reusable message objects
        self._imu_msg = Imu()
        self._imu_msg.header.frame_id = f'{self.prefix}imu'
        self._imu_temp_msg = Float32()
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = f'{self.prefix}odom'
        self._odom_msg.child_frame_id = f'{self.prefix}base_link'
        self._tf_msg = TransformStamped()
        self._tf_msg.header.frame_id = f'{self.prefix}odom'
        self._tf_msg.child_frame_id = f'{self.prefix}base_link'
        self._battery_msg = BatteryState()
        self._battery_msg.header.frame_id = f'{self.prefix}base'
        self._joint_state_msg = JointState()
        self._joint_state_msg.name = self._joint_names
        self._motor_temp_msg = MotorTemperature()
        self._motor_temp_msg.motor_name = self._motor_names

        # Rosbridge WebSocket client
        self.ros_client = roslibpy.Ros(host='192.168.68.53', port=9090)
        self.ros_client.run()
        
        # Subscribe to topics via rosbridge
        self.sport_mode_subscriber = roslibpy.Topic(self.ros_client, '/lf/sportmodestate', 'unitree_go/SportModeState')
        self.sport_mode_subscriber.subscribe(self._sport_callback)
        
        self.low_state_state_subscriber = roslibpy.Topic(self.ros_client, '/lf/lowstate', 'unitree_go/LowState')
        self.low_state_state_subscriber.subscribe(self.low_state_subscriber_callback)
        
        self.lidar_subcriber = roslibpy.Topic(self.ros_client, '/utlidar/cloud', 'sensor_msgs/PointCloud2')
        self.lidar_subcriber.subscribe(self.publish_lidar)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.imu_temp_pub = self.create_publisher(Float32, 'imu_temp', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motor_temp_pub = self.create_publisher(MotorTemperature, 'motor_temp', 10)

        self.get_logger().info('Node initialized successfully.')

    def close_rosbridge(self):
        """Close rosbridge connection."""
        if hasattr(self, 'ros_client') and self.ros_client.is_connected:
            if hasattr(self, 'sport_mode_subscriber'):
                self.sport_mode_subscriber.unsubscribe()
            if hasattr(self, 'low_state_state_subscriber'):
                self.low_state_state_subscriber.unsubscribe()
            if hasattr(self, 'lidar_subcriber'):
                self.lidar_subcriber.unsubscribe()
            self.ros_client.close()
            self.get_logger().info('Rosbridge connection closed.')

    def _sport_callback(self, msg):
        """Publish IMU, Odometry and TF directly from sport mode callback."""
        stamp = self.get_clock().now().to_msg()

        # Publish IMU data
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

        # Publish Odometry
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

        # Publish TF transform
        tf = self._tf_msg
        tf.header.stamp = stamp
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def publish_lidar(self, msg):

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.prefix}radar'
        self.lidar_pub.publish(msg)

    def low_state_subscriber_callback(self, msg):

        stamp = self.get_clock().now().to_msg()
        motors = msg.motor_state

        # Battery
        bat = self._battery_msg
        bat.header.stamp = stamp
        bat.voltage = msg.power_v
        bat.current = float(msg.bms_state.current)
        bat.percentage = float(msg.bms_state.soc) / 100.0
        self.battery_pub.publish(bat)

        # Joint states
        js = self._joint_state_msg 
        js.header.stamp = stamp
        js.position = [float(motors[i].q) for i in MOTOR_INDICES]
        js.velocity = [float(motors[i].dq) for i in MOTOR_INDICES]
        self.joint_state_pub.publish(js)

        # Motor temperatures
        mt = self._motor_temp_msg
        mt.header.stamp = stamp
        mt.temperature = [float(motors[i].temperature) for i in MOTOR_INDICES]
        self.motor_temp_pub.publish(mt)


def main(args=None):
    rclpy.init(args=args)
    node = RobotRead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_rosbridge()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()