import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = False  # 🔥 ของจริง = False
    urdf_file = os.path.join(
        get_package_share_directory('go2_control'),
        'urdf',
        'robot.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    pc2scan_param = os.path.expanduser(
        '~/unitree_ros2/src/go2_control/pc2scan.yaml'
    )

    return LaunchDescription([

        # ===== SLAM TOOLBOX =====
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # ===== POINTCLOUD → LASERSCAN =====
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan',
            output='screen',

            remappings=[
                ('cloud_in', '/pointcloud'),
                ('scan', '/scan')
            ],

            parameters=[pc2scan_param, {
                'use_sim_time': use_sim_time
            }]
        ),

        # ===== ROBOT STATE / SENSOR =====
        Node(
            package='go2_control',
            executable='go2w_read',
            name='go2w_read',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        # ===== CMD_VEL BRIDGE =====
        Node(
            package='go2_control',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # ================= FAKE JOINTS =================
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher'
        # ),

        # ===== CAMERA (OPTIONAL) =====
        # Node(
        #     package='go2_control',
        #     executable='camera',
        #     name='camera',
        #     arguments=['wlp4s0'],  # 👈 เปลี่ยนตาม interface จริง
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time
        #     }]
        # ),

        # ===== RVIZ =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
    ])