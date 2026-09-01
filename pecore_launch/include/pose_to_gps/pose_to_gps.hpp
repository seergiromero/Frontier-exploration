#ifndef POSE_TO_GPS_HPP_
#define POSE_TO_GPS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <random>
#include <cmath>
#include <chrono>

class PoseToGpsNode : public rclcpp::Node
{
public:
  PoseToGpsNode();

private:
  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;

  // RNG for drift simulation
  std::default_random_engine rng_;
  std::normal_distribution<double> noise_;

  // Accumulated drift
  double drift_x_;
  double drift_y_;
  double drift_z_;

  // Previous data for velocity estimation
  bool has_prev_;
  rclcpp::Time prev_time_;
  double prev_x_;
  double prev_y_;
  double prev_z_;
  double prev_yaw_;

  // Reference origin for GPS simulation
  double origin_lat_;
  double origin_lon_;
  double origin_alt_;

  // Conversion ratio (meters to degrees)
  double meters_to_lat_;
  double meters_to_lon_;

  // --- Altitude controls (for NavSatFix.altitude) ---
  bool   use_fixed_altitude_;   // if true, force altitude to fixed_altitude_m_
  double fixed_altitude_m_;     // absolute altitude to publish (ellipsoid meters)
  double altitude_bias_m_;      // constant bias added to computed altitude
  double altitude_scale_;       // scale factor applied to computed altitude
  double geoid_separation_m_;   // add this if your local altitude is MSL and you want ellipsoid

};

#endif
