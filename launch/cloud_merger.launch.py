# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Author: Manuel Beschi <manuel.beschi@unibs.it>

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('surface_reconstruction'), 'config', 'cloud_merger.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('input_cloud', default_value='filtered_cloud',
                              description='Filtered cloud published by node 1'),
        Node(
            package='surface_reconstruction',
            executable='cloud_merger_node',
            name='cloud_merger_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('filtered_cloud', LaunchConfiguration('input_cloud'))],
        ),
    ])
