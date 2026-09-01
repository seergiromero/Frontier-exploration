import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    parameter_file = os.path.join(
      get_package_share_directory('qped_nav'),
      'config',
      'LIOSAM_params.yaml'
      )

    lio_sam_imuPreintegration = Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        )
    
    lio_sam_imageProjection = Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        )
    
    lio_sam_featureExtraction = Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        )
    
    lio_sam_mapOptimization = Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        )
    
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )

    ld = LaunchDescription()
    ld.add_action(lio_sam_imuPreintegration)
    ld.add_action(lio_sam_imageProjection)
    ld.add_action(lio_sam_featureExtraction)
    ld.add_action(lio_sam_mapOptimization)
    ld.add_action(rviz2)
    return ld