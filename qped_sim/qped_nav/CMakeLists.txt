cmake_minimum_required(VERSION 3.5)
project(qped_nav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
# find_package(slam_toolbox REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# Create the executable for the node
# add_executable(octomap_publisher src/octomap_publisher.cpp)
# add_executable(obstacle_detection src/obstacle_detection.cpp)
# add_executable(navsat_odom src/navsat_odom.cpp)

# Link dependencies to the executable
# ament_target_dependencies(
#   octomap_publisher
#   rclcpp
#   octomap_msgs
#   sensor_msgs
# )

# ament_target_dependencies(
#   obstacle_detection
#   rclcpp
#   octomap_msgs
#   sensor_msgs
# )

# ament_target_dependencies(
#   navsat_odom
#   rclcpp
#   sensor_msgs
#   tf2
#   tf2_ros
#   nav_msgs
# )

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  # obstacle_detection
  # octomap_publisher
  # navsat_odom
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
