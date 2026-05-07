import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():

    # ================= CONFIG =================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.expanduser('~/unitree_ros2/map/house_map.yaml')
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.expanduser('~/unitree_ros2/src/go2_control/nav2_params_go2w.yaml')
        # default=os.path.expanduser('~/unitree_ros2/src/go2_control/nav2go2_params.yaml')
    )

    pc2scan_param = os.path.expanduser(
        '~/unitree_ros2/src/go2_control/pc2scan.yaml'
    )

    urdf_file = os.path.join(
        get_package_share_directory('go2_control'),
        'urdf',
        'robot.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ================= NAV2 =================
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # ================= RVIZ =================
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    return LaunchDescription([

        # ================= ARGUMENTS =================
        DeclareLaunchArgument('map', default_value=map_dir),
        DeclareLaunchArgument('params_file', default_value=param_dir),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='go2'),
        DeclareLaunchArgument('prefix', default_value=''),

        # ================= NAV2 =================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true'
            }.items()
        ),

        # ================= RVIZ =================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
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

        # ================= YOUR CONTROL NODES =================
        Node(
            package = 'go2_control',
            executable = 'go2w_read',
            name='go2w_read',

            output='screen'
        ),
        
        Node(
            package='go2_control',
            executable='cmd_vel_node',
            name='cmd_vel_node',
            
            output='screen'
        ),

        Node(
            package = 'go2_control',
            executable = 'camera',
            name='camera',
            arguments=['wlp4s0'],  # 👈 เปลี่ยนตาม interface จริง
            output='screen'
        ),

        Node(
            package='go2_control',
            executable='main',
            name='main_node',
            output='screen',
            parameters=[{
                         'sim_mode': True
                         }]
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan',
            output='screen',

            remappings=[
                ('cloud_in', 'pointcloud'),
                ('scan', '/scan')
            ],

            parameters=[pc2scan_param]
        ),

        # # ================= INIT POSE (delay) =================
        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='go2_control',
        #             executable='init_pose',
        #             name='init_pose',
        #             output='screen'
        #         )
        #     ]
        # ),
    ])


        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     # arguments=['-d', rviz_config_dir], # ถ้ามีไฟล์ config ให้ใช้บรรทัดนี้
        #     output='screen'
        # ),
 