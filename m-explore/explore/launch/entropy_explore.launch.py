import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    # Load entropy-based exploration parameters from YAML config file
    config = os.path.join(
        get_package_share_directory("explore_lite"), 
        "config", 
        "entropy_params.yaml"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true", 
        description="Use simulation/Gazebo clock"
    )
    
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the entropy explorer node",
    )
    
    # Remap global TF topics to relative namespace to avoid conflicts in multi-robot scenarios
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    
    # Configure entropy-based autonomous exploration node
    node = Node(
        package="explore_lite",
        name="entropy_explorer_node",
        namespace=namespace,
        executable="entropy_explorer",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
    )
    
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(node)
    
    return ld