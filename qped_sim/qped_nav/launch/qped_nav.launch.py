from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above
        ])
    )

    declared_arguments.append(DeclareLaunchArgument(
        'fast_LIMO',
        default_value='False',
        description = 'Whether to run fast_LIMO instead of LIO-SAM')
    )
    use_fast_LIMO = LaunchConfiguration('fast_LIMO')


    # # # Localization
    # # declared_arguments.append(
    # #     IncludeLaunchDescription(
    # #         PythonLaunchDescriptionSource(
    # #             os.path.join(get_package_share_directory('qped_nav'), 
    # #                          'launch/localization.launch.py')
    # #         )   
    # #     )
    # # )



    # Fake global localization
    declared_arguments.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map b2/odom'.split(' '),
            output='screen'
            )
    )

    # # GNSS conversion
    # declared_arguments.append(
    #     Node(
    #         package='qped_nav', 
    #         executable='navsat_odom', 
    #         name='navsat_odom',
    #         output='screen'
    #     )
    # )



    # # Octomap republisher
    # declared_arguments.append(
    #     Node(
    #         package='qped_nav',
    #         executable='octomap_to_map',
    #         output='screen'
    #         )
    # )
    declared_arguments.append(
        Node(
            package='mapconversion',
            executable='map_conversion_oct_node',
            name='map_conversion',
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'minimum_z': 0.1},
                {'max_slope_ugv': 0.5},
                {'slope_estimation_size': 3},
                {'minimum_occupancy': 10},
                {'partial_map_updates': True},
                #QoS parameters
                {'subscriber_qos_reliable': True},
                {'subscriber_qos_transient_local': False},
                {'publisher_qos_reliable': True},
                {'publisher_qos_transient_local': True},
            ],
            remappings=[
                ('octomap', 'octomap_binary'),
                ('/mapUGV', '/map')
            ]
        )
    )

    # # LIO-SAM
    # declared_arguments.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory('qped_nav'), 
    #                          'launch/include/run_LIOSAM.launch.py')
    #         ),
    #         condition=UnlessCondition(use_fast_LIMO)
    #     )
    # )

    # # fast-LIMO
    # declared_arguments.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory('qped_nav'), 
    #                          'launch/include/fast_limo.launch.py')
    #         ),
    #         condition=IfCondition(use_fast_LIMO)   
    #     )
    # )

    # Launch octomap
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('qped_nav'), 
                             'launch/include/octomap.launch.py')
            )
        )
    )


    # Launch main simulation
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('qped_nav'), 
                             'launch/include/nav2.launch.py')
            )
        )
    )

    return LaunchDescription(declared_arguments)