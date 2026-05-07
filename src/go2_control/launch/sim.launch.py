import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_house_sim.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.55'
        }.items()
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # map_dir = LaunchConfiguration(
    #     'map',
    #     default=os.path.join(
    #         get_package_share_directory('turtlebot3_navigation2'),
    #         'map',
    #         'map.yaml'))
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.expanduser('~/unitree_ros2/map/house_map.yaml')
    )

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    pc2scan_param = os.path.expanduser(
        '~/unitree_ros2/src/go2_control/pc2scan_sim.yaml'
    )

    # param_file_name = TURTLEBOT3_MODEL + '.yaml'
    # if ROS_DISTRO == 'humble':
    #     param_dir = LaunchConfiguration(
    #         'params_file',
    #         default=os.path.join(
    #             get_package_share_directory('turtlebot3_navigation2'),
    #             'param',
    #             ROS_DISTRO,
    #             param_file_name))
    # else:
    #     param_dir = LaunchConfiguration(
    #         'params_file',
    #         default=os.path.join(
    #             get_package_share_directory('turtlebot3_navigation2'),
    #             'param',
    #             param_file_name))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.expanduser('~/unitree_ros2/src/go2_control/nav2_params.yaml')
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz')
    
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='TF frame prefix'
        ),

        gazebo_launch, 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': frame_prefix
            }]
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan_sim',
            output='screen',

            remappings=[
                ('cloud_in', '/utlidar/cloud'),
                ('scan', '/scan')
            ],

            parameters=[pc2scan_param, {
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # ================= YOUR CONTROL NODES =================
        Node(
            package = 'go2_control',
            executable = 'gazebo_convert',
            name='gazebo_convert',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package = 'go2_control',
            executable = 'go2w_read',
            name='go2w_read',
            parameters=[{'use_sim_time': use_sim_time,
                         'sim_mode': True}],
            output='screen'
        ),
        

        Node(
            package='go2_control',
            executable='main',
            name='main_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'sim_mode': True
                         }]
        ),

        Node(
            package='go2_control',
            executable='Stream',
            name='Stream',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'sim_mode': True
                         }]
        ),


        # ================= AUTO INIT POSE =================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='go2_control',
                    executable='init_pose',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),
        
    ])
