from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  config_octomap_server = PathJoinSubstitution(
      [FindPackageShare('qped_nav'),
        'config',
        'octomap_server.yaml'],
  )

  # obstacle_detector = Node(
  #   package='qped_nav', 
  #   executable='obstacle_detection', 
  #   name='obstacle_detector',
  #   output='screen',
  #   remappings=[('cloud_in', '/b2/lidar/points'),
  #               ('cloud_out', '/velodyne/obstacles')]        
  # )

  octomap_server = Node(
    package='octomap_server', 
    executable='octomap_server_node', 
    name='octomap_server',
    output='screen',
    parameters=[config_octomap_server],
    remappings=[('cloud_in', '/b2/lidar/points')]
    # ,
    #             ('/projected_map', '/map')]        
  )


  ld = LaunchDescription()
  # ld.add_action(obstacle_detector)
  ld.add_action(octomap_server)

  return LaunchDescription([ld])