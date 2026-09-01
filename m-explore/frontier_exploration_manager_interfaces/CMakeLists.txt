cmake_minimum_required(VERSION 3.8)
project(frontier_exploration_manager_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExploreArea.action"
  DEPENDENCIES builtin_interfaces
)

# Must export runtime support
ament_export_dependencies(rosidl_default_runtime)

ament_package()