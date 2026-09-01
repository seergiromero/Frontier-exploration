# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('qped_bringup')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz',
                                   'default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # change it to screen if you wanna see RVIZ output in terminal
        output={'both': 'log'},
        arguments=['-d', rviz_config_file,
                   '--ros-args', '--log-level', 'ERROR']
    )

    return LaunchDescription([
        declare_rviz_config_file_cmd,
        rviz_node
    ])
