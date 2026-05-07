from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mode = LaunchConfiguration('mode')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_control'),
                'launch',
                'sim.launch.py'
            )
        ),
        condition=LaunchConfigurationEquals('mode', 'sim'),
        launch_arguments={
            'sim_mode': 'true'
        }.items()
    )

    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_control'),
                'launch',
                'real.launch.py'
            )
        ),
        condition=LaunchConfigurationEquals('mode', 'real'),
        launch_arguments={
            'sim_mode': 'false'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='real',
            description='sim or real'
        ),

        sim_launch,
        real_launch
    ])