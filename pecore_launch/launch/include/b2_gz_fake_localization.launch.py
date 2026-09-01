# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Publish the pose as the odom->base_link TF
    # Bridge ROS <-> Gazebo
    # Table of msg equivalences: https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge
    ros_gz_fake_localization_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_fake_localization_bridge',
        namespace='b2',
        arguments=[
            '/model/b2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/b2/pose','/tf'),
        ],
        output='screen'
    )


    # Fake global localization
    fake_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map b2/odom'.split(' '),
        output='screen'
    )

    return LaunchDescription([
        ros_gz_fake_localization_bridge,
        fake_map_odom
    ])