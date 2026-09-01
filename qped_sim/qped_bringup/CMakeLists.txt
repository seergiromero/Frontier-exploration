# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

cmake_minimum_required(VERSION 3.5)
project(qped_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qped_description REQUIRED)
 
install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME})

ament_package()