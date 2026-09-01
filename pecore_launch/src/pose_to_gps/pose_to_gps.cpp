#include "pose_to_gps/pose_to_gps.hpp"


using namespace std::chrono_literals;

PoseToGpsNode::PoseToGpsNode()
: Node("pose_to_gps")
{
  RCLCPP_INFO(get_logger(), "Starting PoseToGpsNode with drifting odometry...");

  // Reference GPS coordinates (e.g., Barcelona)
  origin_lat_ = declare_parameter("origin_lat", 41.3888);
  origin_lon_ = declare_parameter("origin_lon", 2.1590);
  origin_alt_ = declare_parameter("origin_alt", 0.0);

  // Approximate conversion
  meters_to_lat_ = 1.0 / 111320.0;
  meters_to_lon_ = 1.0 / (111320.0 * std::cos(origin_lat_ * M_PI / 180.0));

  // Publishers
  gps_pub_  = create_publisher<sensor_msgs::msg::NavSatFix>("/b2/gps/fix", 10);

  // Subscriber
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "/b2/pose", 10,
    std::bind(&PoseToGpsNode::poseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to /b2/pose and publishing /b2/gps/fix and /b2/odom/drift");
}

void PoseToGpsNode::poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.empty()) return;

  const auto &pose = msg->poses.front();
  auto now = this->now();

  // --- GPS message ---
  sensor_msgs::msg::NavSatFix gps_msg;
  gps_msg.header.stamp = now;
  gps_msg.header.frame_id = "b2/base_link";
  gps_msg.latitude  = origin_lat_ + pose.position.y * meters_to_lat_;
  gps_msg.longitude = origin_lon_ + pose.position.x * meters_to_lon_;
  gps_msg.altitude  = origin_alt_ + pose.position.z;
  gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  gps_msg.position_covariance = {1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3};
  gps_pub_->publish(gps_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToGpsNode>());
  rclcpp::shutdown();
  return 0;
}
