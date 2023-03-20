import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('mocha_buono')
    world = LaunchConfiguration('world')

    visi = Node(
        package='mocha_buono',
        executable='visualize',
        name='visualize',
        output='screen',
    )

    return LaunchDescription([
        visi,
        DeclareLaunchArgument(
            'param',
            default_value='value.conf',
            description='Choose one file path to load settings'
        ),
    ])
