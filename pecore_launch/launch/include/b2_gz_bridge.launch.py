# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Bridge ROS <-> Gazebo
    # Table of msg equivalences: https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace='b2',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/b2/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/model/b2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/b2/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
            '/model/b2/camera/front/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/b2/camera/front/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/b2/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/model/b2/pose','/b2/pose'),
            ('/model/b2/imu','/b2/imu/raw'),
            ('/model/b2/odometry_with_covariance','/b2/odom/raw'),
            ('/model/b2/camera/front/rgb/camera_info', '/b2/camera/front/rgb/camera_info'),
            ('/model/b2/camera/front/rgb/image_raw', '/b2/camera/front/rgb/image_raw'),
            ('/model/b2/lidar/points', '/b2/lidar/points_old'),
        ],
        output='screen'
    )

    return LaunchDescription([
        ros_gz_bridge,
    ])