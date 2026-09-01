# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from pathlib import Path, PurePath

def generate_launch_description():

# =================================================================
# Initial definitions and parameters
# =================================================================

    # Environment variables
    gazebo_set_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[
                                    EnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                                        default_value='$GZ_SIM_RESOURCE_PATH'),
                                    '~/.gz/models/:',
                                    str(PurePath(Path(get_package_share_directory('qped_gazebo'),'models')))
                                ])
    # Get here directories of packages
    qped_bringup_share_dir = get_package_share_directory(
        'qped_bringup')
    qped_description_share_dir = get_package_share_directory(
        'qped_description')
    default_world_file = os.path.join(get_package_share_directory(
        'pecore_launch'), 'worlds', 'boxes.sdf')

    # Launch arguments
    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time', default = True)
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    champ_params = LaunchConfiguration('champ_params')
    world_file = LaunchConfiguration('world_file', default = default_world_file)
    use_gui = LaunchConfiguration('use_gui')

    declare_use_simulator = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='whether to use Gazebo Simulation.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='If true, use simulated clock')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='whether or not to launch rviz')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            qped_bringup_share_dir, 'rviz', 'b2_view.rviz'),
        description='...')
    declare_champ_params = DeclareLaunchArgument(
        'champ_params',
        default_value=os.path.join(
            qped_bringup_share_dir, 'config', 'b2_champ_params.yaml'),
        description='path to locks params.')
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_file,
        description='Gazebo world.')
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='True',
        description='Launch Gazebo with GUI (true) or headless (false)'
    )

# =================================================================
# MODELS
# =================================================================

    # Robot
    xacro_file_name = 'b2/b2.urdf.xacro'
    xacro_full_dir = os.path.join(
        qped_description_share_dir, 'urdf', xacro_file_name)
    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=[('tf', '/tf'),
                    ('tf_static', '/tf_static')])

# =================================================================
# CONTROL
# =================================================================

    # Champ controller
    declare_quadruped_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller',
        name='qped_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[champ_params],
        # remappings=[('cmd_vel', 'vox_nav/cmd_vel')]
        )

    # Joint state broadcaster
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/controller_manager',
                   '--ros-args', '--disable-external-lib-logs'],
        output = 'log',
    )

    # Joint trajectory controller
    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller',
        arguments=['joint_trajectory_controller', 
                   '--controller-manager', '/controller_manager',
                   '--ros-args', '--disable-external-lib-logs'],
        remappings=[('cmd_vel', 'vox_nav/cmd_vel')],
        output = 'log',
    )

# =================================================================
# Gazebo
# =================================================================

    #Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('pecore_launch'),
                'launch',
                'include/gz_sim.launch.py'
            )
        ]),
        launch_arguments=[
            ('gz_args',
                [TextSubstitution(text='-r -v 4 '), world_file]
            ),
            ('prefix', 'xterm -iconic -geometry 1x1 -e'),
            ('use_gui', use_gui),
        ],
    )
    # Spawn the robot in Gazebo
    declare_spawn_entity_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='ros_gz_sim',
        condition=IfCondition(use_simulator),
        arguments=['-name', 'b2',
                   '-allow_renaming', 'true',
                   '-topic', '/robot_description', '-z', '0.84',
                   '--ros-args', '--log-level', 'warn'],
        output='log',
        parameters=[{'use_sim_time': use_sim_time}])

    # ROS <-> GZ bridge
    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pecore_launch'), 'launch'), '/include/b2_gz_bridge.launch.py'
            ]),
        )
    
    lidarRepublisher = Node(
        package='qped_gazebo',
        executable='lidar_republisher',
        name='lidar_republisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'input_topic': '/b2/lidar/points_old',
                     'output_topic': '/b2/lidar/points',}],
        remappings=[('/scan', '/scan')]
    )

    imuRepublisher = Node(
        package='pecore_launch',
        executable='imu_cov_republisher',
        name='imu_cov_republisher',
        output='screen')

    poseHeightRepublisher = Node(
        package='pecore_launch',
        executable='pose_array_to_height_pose',
        name='pose_array_to_height_pose',
        output='screen')

# =================================================================
# Visualization
# =================================================================

    #  INCLUDE RVIZ LAUNCH FILE IF use_rviz IS SET TO TRUE
    declare_rviz_launch_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(qped_bringup_share_dir,
                     'launch',
                     'common/rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
        'rviz_config': rviz_config
    }.items())

# =================================================================

    return LaunchDescription([
        gazebo_set_resource_path,
        declare_use_gui,
        gazebo_server,
        lidarRepublisher,
        imuRepublisher,
        poseHeightRepublisher,
        # timed_actions
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=declare_spawn_entity_to_gazebo_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        declare_use_sim_time,
        declare_use_simulator,
        declare_world_file,
        declare_robot_state_publisher_node,
        declare_spawn_entity_to_gazebo_node,
        declare_champ_params,
        declare_quadruped_controller_node,
        ros_gz_bridge,
        declare_use_rviz,
        declare_rviz_config,
        declare_rviz_launch_include,
    ])
