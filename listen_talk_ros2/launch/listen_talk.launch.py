"""Launch file for launching rviz and gazebo."""
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path = get_package_share_path('listen_talk_ros2')
    default_rviz_config_path = path / 'jarvis_linux.ppn'
    # default_model_path_yaml = path / 'nav2_params.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='keyword_path', default_value=str(default_rviz_config_path),
                              description='Absolute path to the wake_up keyword file'
                              ),
        Node(
            package='listen_talk_ros2',
            executable='listen',
            name='listen',
            output='screen',
            # parameters=['keyword_path'],
            # arguments='keyword_path',
        ),

        Node(
            package='listen_talk_ros2',
            executable='talk',
            name='talk',
            output='screen',
            # parameters=['keyword_path'],
            # arguments='keyword_path',
        )
    ])
