#ifndef OBSTACLE_DETECTION_HPP_
#define OBSTACLE_DETECTION_HPP_

#include <chrono>
#include <string>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace std::chrono_literals;

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();

private:
  // Clock (for throttled logging)
  rclcpp::Clock clock_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pc_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Frames
  std::string map_frame_;
  std::string base_frame_;

  // Z passthrough limits in MAP frame
  double robot_height_;   // keep 0.0 <= z <= robot_height_

  // Robot body footprint (for notch, in BASE_LINK frame)
  double fp_x_fw_;
  double fp_x_bw_;
  double fp_y_lw_;
  double fp_y_rw_;
  double fp_z_uw_;
  double fp_z_dw_;

  // Main callback
  void callbackPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Filters
  void passThroughZ(sensor_msgs::msg::PointCloud2 & cloud_map);
  void notchFootprint(sensor_msgs::msg::PointCloud2 & cloud_base);
};

#endif  // OBSTACLE_DETECTION_HPP_
