# Copyright (C) Angel Santamaria Navarro 
# All Rights Reserved 2024
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)
project(plugins)

# Plugins
add_subdirectory(sensor_odometer)
add_subdirectory(sensor_lidar)
add_subdirectory(default_lidar)

# GZ system plugins
add_subdirectory(custom_sensor_system)
add_subdirectory(custom_lidar_system)
