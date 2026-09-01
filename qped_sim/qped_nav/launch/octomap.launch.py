from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the Octomap server node
        Node(
            package='octomap_server',
            executable='tracking_octomap_server_node',
            name='tracking_octomap_server',
            output='screen',
            parameters=[{'listen_changes': True, 'topic_changes':'/b2/lidar/points'}],
            # remappings=[('/scan', '/b2/lidar/points')]
        ),
        
        # Launch the Octomap Publisher Python script as a Node
        Node(
            package='qped_nav', 
            executable='octomap_publisher',
            name='octomap_republisher_node',
            output='screen',  
            parameters=[
                {'use_sim_time': True}  
            ],
            # remappings=[
            #     ('/octomap_binary', '/octomap_binary'),  # If needed, remap the service topic
            #     ('/octomap_full', '/octomap_full')  # Remap the topic if required
            # ]
        )
    ])
