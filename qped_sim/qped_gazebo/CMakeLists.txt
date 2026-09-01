# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)
project(qped_gazebo)

# # set(CMAKE_CXX_STANDARD 14)
# # # if(NOT CMAKE_CXX_STANDARD)
# # # endif()
# # add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)
# # set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
# # set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(lidar_republisher src/lidar_republisher.cpp)
ament_target_dependencies(
  lidar_republisher
  rclcpp
  sensor_msgs
)

install(TARGETS
  lidar_republisher
  DESTINATION lib/${PROJECT_NAME})

# Plugins
add_subdirectory(plugins)

install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME})

ament_package()
