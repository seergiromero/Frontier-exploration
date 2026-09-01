
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  config_octomap_server = PathJoinSubstitution(
      [FindPackageShare('pecore_launch'),
        'config',
        'octomap_server.yaml'],
  )

  obstacle_detector = Node(
    package='pecore_launch', 
    executable='obstacle_detection', 
    name='obstacle_detector',
    output='screen',
    remappings=[('cloud_in', '/b2/lidar/points'),
                ('cloud_out', '/b2/lidar/obstacles')]        
  )

  octomap_server = Node(
      package='octomap_server',
      executable='octomap_server_node',
      name='octomap_server',
      output='log',
      parameters=[config_octomap_server],
      arguments=['--ros-args', '--log-level', 'error'],
      remappings=[
          ('cloud_in', '/b2/lidar/obstacles'),
          # ('/projected_map', '/map')
      ]
  )

  map_converter =  Node(
      package='mapconversion',
      executable='map_conversion_oct_node',
      name='map_conversion',
      output='screen',
      parameters=[{
          'map_frame': 'map',
          'minimum_z': 0.1,
          'max_slope_ugv': 0.5,
          'slope_estimation_size': 3,
          'minimum_occupancy': 10,
          'partial_map_updates': True,
          #QoS parameters
          'subscriber_qos_reliable': True,
          'subscriber_qos_transient_local': False,
          'publisher_qos_reliable': True,
          'publisher_qos_transient_local': True,
      }],
      remappings=[
          ('octomap', 'octomap_binary'),
          ('/mapUGV', '/map')
      ]
  )

  ld = LaunchDescription()
  ld.add_action(obstacle_detector)
  ld.add_action(octomap_server)
  ld.add_action(map_converter)

  return LaunchDescription([ld])