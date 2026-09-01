#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <algorithm>
#include <cstdint>

static inline builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & t)
{
  builtin_interfaces::msg::Time out;
  const int64_t nsec = t.nanoseconds();
  out.sec = static_cast<int32_t>(nsec / 1000000000LL);
  out.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
  return out;
}

class PoseArrayToHeightPose : public rclcpp::Node
{
public:
  PoseArrayToHeightPose()
  : rclcpp::Node("posearray_to_height_pose")
  {
    in_topic_   = declare_parameter<std::string>("in_topic", "/b2/pose");
    out_topic_  = declare_parameter<std::string>("out_topic", "/b2/height_pose");
    frame_id_   = declare_parameter<std::string>("frame_id", "map");
    pose_index_ = declare_parameter<int>("pose_index", 0);
    z_offset_   = declare_parameter<double>("z_offset", 0.0);
    var_z_      = declare_parameter<double>("var_z", 0.01);

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(out_topic_, 10);

    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      in_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&PoseArrayToHeightPose::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Subscribing %s → publishing %s (frame=%s, idx=%d, z_offset=%.3f, var_z=%.4f)",
      in_topic_.c_str(), out_topic_.c_str(), frame_id_.c_str(), pose_index_, z_offset_, var_z_);
  }

private:
  void cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "PoseArray is empty on %s", in_topic_.c_str());
      return;
    }

    const int i = std::clamp(pose_index_, 0, static_cast<int>(msg->poses.size()) - 1);
    const auto &p = msg->poses.at(i);

    geometry_msgs::msg::PoseWithCovarianceStamped out;

    // Use incoming stamp if valid; otherwise use current time (manual conversion)
    builtin_interfaces::msg::Time stamp = msg->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = to_builtin_time(this->get_clock()->now());
    }
    out.header.stamp = stamp;
    out.header.frame_id = frame_id_;

    // Only Z is used by the EKF for this input (X/Y left 0 and disabled in EKF config)
    out.pose.pose.position.x = 0.0;
    out.pose.pose.position.y = 0.0;
    out.pose.pose.position.z = p.position.z + z_offset_;

    // Identity orientation (EKF uses IMU for orientation)
    out.pose.pose.orientation.w = 1.0;
    out.pose.pose.orientation.x = 0.0;
    out.pose.pose.orientation.y = 0.0;
    out.pose.pose.orientation.z = 0.0;

    // 6x6 covariance (row-major). Put variance on Z only.
    for (double &c : out.pose.covariance) c = 0.0;
    out.pose.covariance[14] = var_z_;

    pub_->publish(out);
  }

  // Params
  std::string in_topic_, out_topic_, frame_id_;
  int pose_index_;
  double z_offset_, var_z_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseArrayToHeightPose>());
  rclcpp::shutdown();
  return 0;
}
