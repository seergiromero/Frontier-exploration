cmake_minimum_required(VERSION 3.5)
project(frontier_exploration_manager)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(explore_lite REQUIRED)
find_package(frontier_exploration_manager_interfaces REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  nav2_costmap_2d
  visualization_msgs
  frontier_exploration_manager_interfaces
)

link_directories(${explore_lite_LIBRARY_DIRS})  

add_executable(frontier_exploration_manager
  src/frontier_exploration_manager.cpp
)

target_include_directories(frontier_exploration_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${explore_lite_INCLUDE_DIRS}
)

target_link_libraries(frontier_exploration_manager
  ${explore_lite_LIBRARIES}
  "${cpp_typesupport_target}"
)

ament_target_dependencies(frontier_exploration_manager
  ${DEPENDENCIES}
)

install(
  TARGETS frontier_exploration_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()