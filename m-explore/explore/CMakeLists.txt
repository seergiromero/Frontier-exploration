cmake_minimum_required(VERSION 3.5)
project(explore_lite)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(visualization_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  map_msgs
  nav2_costmap_2d
  visualization_msgs
)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------
include_directories(include)

# ------------------------------------------------------------------------------
# BUILD THE LIBRARY (core exploration functionality)
# ------------------------------------------------------------------------------
add_library(explore_lite_core
  src/costmap_client.cpp
  src/frontier_search.cpp
  # src/costmap_tools.cpp   # uncomment only if file exists
)

target_include_directories(explore_lite_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(explore_lite_core ${DEPENDENCIES})


# ------------------------------------------------------------------------------
# BUILD THE ORIGINAL explore EXECUTABLE
# ------------------------------------------------------------------------------
add_executable(explore
  src/explore.cpp
)

target_link_libraries(explore
  explore_lite_core
)

ament_target_dependencies(explore ${DEPENDENCIES})

add_executable(entropy_explorer
  src/entropy_explorer.cpp
)

target_link_libraries(entropy_explorer
  explore_lite_core
)

ament_target_dependencies(entropy_explorer ${DEPENDENCIES})


# ------------------------------------------------------------------------------
# INSTALL
# ------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS explore_lite_core
  EXPORT export_explore_lite
  DESTINATION lib
)

# Install executable
install(
  TARGETS explore entropy_explorer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# EXPORT
# ------------------------------------------------------------------------------
ament_export_targets(export_explore_lite HAS_LIBRARY_TARGET)
ament_export_libraries(explore_lite_core)
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()