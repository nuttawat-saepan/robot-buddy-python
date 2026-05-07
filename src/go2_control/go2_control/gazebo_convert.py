import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from unitree_go.msg import (
    LowState,
    SportModeState,
    MotorState
)

class GazeboToUnitreeBridge(Node):
    def __init__(self):
        super().__init__('gazebo_to_unitree_bridge')

        self._current_odom = None
        self._current_imu = None
        self._current_joint = None

        self.declare_parameter('low_state_publish_hz', 20.0)
        self.declare_parameter('sport_state_publish_hz', 20.0)
        low_state_publish_hz = self.get_parameter('low_state_publish_hz').value
        sport_state_publish_hz = self.get_parameter('sport_state_publish_hz').value

        # Subscribe ข้อมูลจาก TB3/Gazebo
        self.create_subscription(JointState, '/sim_joint_states', self.joint_cb, 10)
        self.create_subscription(Odometry, '/sim_odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/sim_imu', self.imu_cb, 10)

        # Publish ให้ RobotWrite
        self.low_state_pub = self.create_publisher(LowState, '/lf/lowstate', 10)
        self.sport_state_pub = self.create_publisher(SportModeState, '/lf/sportmodestate', 10)
        self.create_timer(self._rate_to_period(low_state_publish_hz), self.publish_low_state)
        self.create_timer(self._rate_to_period(sport_state_publish_hz), self.publish_sport_state)
        self.get_logger().info(
            'Gazebo -> Unitree bridge started '
            f'(lowstate={low_state_publish_hz} Hz, sportmodestate={sport_state_publish_hz} Hz)'
        )

    def _rate_to_period(self, rate_hz):
        if rate_hz <= 0.0:
            self.get_logger().warn(f'Invalid publish rate {rate_hz} Hz, using 20.0 Hz')
            rate_hz = 20.0
        return 1.0 / rate_hz

    def joint_cb(self, msg):
        self._current_joint = msg

    def publish_low_state(self):
        if self._current_joint is None:
            return

        low_state = LowState()
        # จำลองค่า Battery
        low_state.power_v = 12.5
        low_state.power_a = 1.2
        low_state.bms_state.soc = 95

         # Fake IMU
        # -------------------------
        if self._current_imu is not None:

            low_state.imu_state.quaternion = [
                self._current_imu.orientation.w,
                self._current_imu.orientation.x,
                self._current_imu.orientation.y,
                self._current_imu.orientation.z
            ]

            low_state.imu_state.gyroscope = [
                self._current_imu.angular_velocity.x,
                self._current_imu.angular_velocity.y,
                self._current_imu.angular_velocity.z
            ]

            low_state.imu_state.accelerometer = [
                self._current_imu.linear_acceleration.x,
                self._current_imu.linear_acceleration.y,
                self._current_imu.linear_acceleration.z
            ]
        
        # วนลูปสร้าง MotorState 16 ตัวตามที่ RobotWrite ต้องการ
        # เราจะเอาความเร็วล้อจาก TB3 มาหลอกใส่ในบาง Joint หรือปล่อยเป็น 0 ไว้
        for i in range(20):
            ms = MotorState()
            # ถ้าต้องการให้มีเลขวิ่งใน joint_states ของ RobotWrite
            # อาจจะเอาค่าจากล้อ TB3 (index 0, 1) มาใส่ขำๆ ก็ได้ครับ
            if i < len(self._current_joint.position):
                ms.q = self._current_joint.position[i]
                if i < len(self._current_joint.velocity):
                    ms.dq = self._current_joint.velocity[i]
                else:
                    ms.dq = 0.0
            else:
                ms.q = 0.0
                ms.dq = 0.0
            ms.temperature = 35
            low_state.motor_state[i] = ms
        
         # Fake foot force
        # -------------------------
        low_state.foot_force = [20, 20, 20, 20]
        low_state.foot_force_est = [20, 20, 20, 20]
        
        self.low_state_pub.publish(low_state)

    def odom_cb(self, msg):
        # เก็บค่าไว้ส่งใน SportModeState (High Level)
        self._current_odom = msg

    def imu_cb(self, msg):
        self._current_imu = msg

    def publish_sport_state(self):
        if self._current_odom is None:
            return

        if self._current_imu is None:
            return
            
        sport = SportModeState()

        # Fake robot mode
        # -------------------------
        sport.mode = 1
        sport.gait_type = 1
        sport.progress = 1.0

        # แปลง Odom -> SportModeState
        sport.position = [self._current_odom.pose.pose.position.x, 
                          self._current_odom.pose.pose.position.y, 
                          self._current_odom.pose.pose.position.z]
        
        # Velocity
        # -------------------------
        sport.velocity = [
            self._current_odom.twist.twist.linear.x,
            self._current_odom.twist.twist.linear.y,
            self._current_odom.twist.twist.linear.z
        ]

        # -------------------------
        # Angular velocity
        # -------------------------
        sport.yaw_speed = (
            self._current_odom.twist.twist.angular.z
        )

        # แปลง IMU (อย่าลืมสลับ Quaternion เป็น [w, x, y, z])
        sport.imu_state.quaternion = [self._current_imu.orientation.w,
                                      self._current_imu.orientation.x,
                                      self._current_imu.orientation.y,
                                      self._current_imu.orientation.z]
        
         # -------------------------
        # IMU gyro
        # -------------------------
        sport.imu_state.gyroscope = [
            self._current_imu.angular_velocity.x,
            self._current_imu.angular_velocity.y,
            self._current_imu.angular_velocity.z
        ]

        # -------------------------
        # IMU accel
        # -------------------------
        sport.imu_state.accelerometer = [
            self._current_imu.linear_acceleration.x,
            self._current_imu.linear_acceleration.y,
            self._current_imu.linear_acceleration.z
        ]

        # -------------------------
        # Fake foot force
        # -------------------------
        sport.foot_force = [20, 20, 20, 20]

        self.sport_state_pub.publish(sport)

def main(args=None):

    rclpy.init(args=args)

    node = GazeboToUnitreeBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
