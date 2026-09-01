from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Rviz config file
    rviz_config_file = PathJoinSubstitution([FindPackageShare("pecore_launch"), "rviz", "simulation.rviz"]) 
    world_file = PathJoinSubstitution([FindPackageShare('pecore_launch'), 'worlds', 'maze.sdf'])  
    use_gui = LaunchConfiguration('use_gui')

    pkg_share = FindPackageShare("pecore_launch").find("pecore_launch")
    model_path = os.path.join(pkg_share, 'models')  

    declared_arguments = []

    # Sim time
    declared_arguments.append(SetParameter(name='use_sim_time', value=True))

    # Gazebo gui
    declared_arguments.append(DeclareLaunchArgument(
        'use_gui',
        default_value='False',
        description='Launch Gazebo with GUI (true) or headless (false)')
    )

    # Extend Gazebo resource path
    declared_arguments.append(SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[model_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    )

    # B2W Robot
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pecore_launch'), 
                             'launch/include/b2_gz_bringup.launch.py')
            ),
            launch_arguments=[('rviz_config', rviz_config_file),
                              ('world_file', world_file),
                              ('use_gui', use_gui),]
        )
    )

    # B2W Perception
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pecore_launch'), 
                             'launch/include/b2_perception.launch.py')
            )
        )
    )

    # Fake Localization
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pecore_launch'), 
                             'launch/include/b2_gz_fake_localization.launch.py')
            )
        )
    )

    # B2W Navigation
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pecore_launch'), 
                             'launch/include/b2_navigation.launch.py')
            )
        )
    )

    return LaunchDescription(declared_arguments)