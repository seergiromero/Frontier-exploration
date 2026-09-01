cmake_minimum_required(VERSION 3.8)
project(pecore_launch)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# box follower
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# IMU covariance republisher
add_executable(imu_cov_republisher src/sensor_rectifications/imu_cov_republisher.cpp)
target_include_directories(imu_cov_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_cov_republisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(imu_cov_republisher rclcpp sensor_msgs tf2_ros )
target_link_libraries(imu_cov_republisher "${cpp_typesupport_target}")
install(TARGETS imu_cov_republisher DESTINATION lib/${PROJECT_NAME})

# Pose height republisher
add_executable(pose_array_to_height_pose src/sensor_rectifications/pose_array_to_height_pose.cpp)
target_include_directories(pose_array_to_height_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_array_to_height_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pose_array_to_height_pose rclcpp geometry_msgs)
target_link_libraries(pose_array_to_height_pose "${cpp_typesupport_target}")
install(TARGETS pose_array_to_height_pose DESTINATION lib/${PROJECT_NAME})

# Obstacle detection
add_executable(obstacle_detection src/obstacle_detection/obstacle_detection.cpp)
target_include_directories(obstacle_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(obstacle_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(obstacle_detection rclcpp sensor_msgs tf2_ros tf2_sensor_msgs)
target_link_libraries(obstacle_detection "${cpp_typesupport_target}")
install(TARGETS obstacle_detection DESTINATION lib/${PROJECT_NAME})

# Box follower
add_executable(box_follower src/box_follower/box_follower.cpp)
target_include_directories(box_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(box_follower rclcpp sensor_msgs ros_gz tf2_ros geometry_msgs std_srvs nav_msgs)
target_link_libraries(box_follower
  gz-transport13::core
  gz-msgs10::core
)
install(TARGETS box_follower DESTINATION lib/${PROJECT_NAME})

# GPS fix and drifting Odometry producer
add_executable(pose_to_gps src/pose_to_gps/pose_to_gps.cpp)
target_include_directories(pose_to_gps PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_to_gps PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pose_to_gps rclcpp sensor_msgs geometry_msgs)
target_link_libraries(pose_to_gps "${cpp_typesupport_target}")
install(TARGETS pose_to_gps DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
