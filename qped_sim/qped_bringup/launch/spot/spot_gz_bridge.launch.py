# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Bridge ROS <-> Gazebo
    # Table of msg equivalences: https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace='spot',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/spot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/spot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/spot/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
            '/model/spot/battery/linear_battery/state@sensor_msgs/msg/BatteryState[gz.msgs.BatteryState',
            '/model/spot/camera/frontleft/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/spot/camera/frontleft/depth/disparity@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/frontleft/depth/disparity/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/spot/camera/frontleft/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/frontright/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/spot/camera/frontright/depth/disparity@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/frontright/depth/disparity/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/spot/camera/frontright/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/left/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/spot/camera/left/depth/disparity@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/left/depth/disparity/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/spot/camera/left/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/right/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/spot/camera/right/depth/disparity@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/right/depth/disparity/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/spot/camera/right/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/back/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/spot/camera/back/depth/disparity@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/spot/camera/back/depth/disparity/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/spot/camera/back/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/model/spot/pose','/tf'),
            ('/model/spot/imu','/spot/imu/raw'),
            ('/model/spot/odometry_with_covariance','/spot/odom/raw'),
            ('/model/spot/battery/linear_battery/state','/spot/battery/state'),
            ('/model/spot/camera/frontleft/depth/camera_info', '/spot/camera/frontleft/depth/camera_info'),
            ('/model/spot/camera/frontleft/depth/disparity', '/spot/camera/frontleft/depth/disparity'),
            ('/model/spot/camera/frontleft/depth/disparity/points', '/spot/camera/frontleft/depth/disparity/points'),
            ('/model/spot/camera/frontleft/depth/image_raw', '/spot/camera/frontleft/depth/image_raw'),
            ('/model/spot/camera/left/depth/camera_info', '/spot/camera/left/depth/camera_info'),
            ('/model/spot/camera/left/depth/disparity', '/spot/camera/left/depth/disparity'),
            ('/model/spot/camera/left/depth/disparity/points', '/spot/camera/left/depth/disparity/points'),
            ('/model/spot/camera/left/depth/image_raw', '/spot/camera/left/depth/image_raw'),
            ('/model/spot/camera/frontright/depth/camera_info', '/spot/camera/frontright/depth/camera_info'),
            ('/model/spot/camera/frontright/depth/disparity', '/spot/camera/frontright/depth/disparity'),
            ('/model/spot/camera/frontright/depth/disparity/points', '/spot/camera/frontright/depth/disparity/points'),
            ('/model/spot/camera/frontright/depth/image_raw', '/spot/camera/frontright/depth/image_raw'),
            ('/model/spot/camera/right/depth/camera_info', '/spot/camera/right/depth/camera_info'),
            ('/model/spot/camera/right/depth/disparity', '/spot/camera/right/depth/disparity'),
            ('/model/spot/camera/right/depth/disparity/points', '/spot/camera/right/depth/disparity/points'),
            ('/model/spot/camera/right/depth/image_raw', '/spot/camera/right/depth/image_raw'),
            ('/model/spot/camera/back/depth/camera_info', '/spot/camera/back/depth/camera_info'),
            ('/model/spot/camera/back/depth/disparity', '/spot/camera/back/depth/disparity'),
            ('/model/spot/camera/back/depth/disparity/points', '/spot/camera/back/depth/disparity/points'),
            ('/model/spot/camera/back/depth/image_raw', '/spot/camera/back/depth/image_raw'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=os.path.join('/opt/ros/humble/lib/:', get_package_prefix('champ_gazebo'), 'lib/')),
        ros_gz_bridge,
    ])