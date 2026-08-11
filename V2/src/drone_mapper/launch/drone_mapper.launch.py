#!/usr/bin/env python3
"""
Launch file for drone_mapper package.

Launches the nodes for geospatial mapping:
- map_publisher: Loads and publishes the map point cloud (latched)
- drone_localization_node.py: Subscribes to telemetry, converts to UTM, publishes Path/Pose/TF/status

Usage:
    ros2 launch drone_mapper drone_mapper.launch.py

Note: drone_vision must be launched separately to provide /telemetry/data
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('drone_mapper')

    # Global config file (shared with drone_vision)
    config_file = os.path.join(pkg_share, 'config.yaml')
    if not os.path.exists(config_file):
        config_file = os.path.normpath(os.path.join(pkg_share, '..', '..', 'config.yaml'))

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to global config.yaml'
    )

    # Map Publisher Node
    map_publisher = Node(
        package='drone_mapper',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[],
    )

    # Drone Localization Node (single Python node)
    drone_localization_node = Node(
        package='drone_mapper',
        executable='drone_localization_node.py',
        name='drone_localization_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[],
    )

    return LaunchDescription([
        config_arg,
        map_publisher,
        drone_localization_node,
    ])