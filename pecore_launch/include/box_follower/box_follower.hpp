#ifndef BOX_FOLLOWER__BOX_FOLLOWER_HPP_
#define BOX_FOLLOWER__BOX_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/scene.pb.h>

#include <std_srvs/srv/empty.hpp>

#include <mutex>
#include <string>

class BoxFollower : public rclcpp::Node
{
public:
  explicit BoxFollower(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // ROS callback
  void robotPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Gazebo helpers
  bool waitForGazeboServices();
  bool waitForBoxPresence();
  bool setBoxPose(const gz::msgs::Pose &pose);
  void resetOctomap();

  // Pose_V callback
  void poseInfoCallback(const gz::msgs::Pose_V &msg);
  bool getCachedBoxPose(gz::msgs::Pose &pose);

  // Debug timer
  void debugTimerCallback();

  // Members
  gz::transport::Node gz_node_;
  std::string world_name_;
  std::string box_name_;
  std::string pose_topic_;
  double desired_distance_;
  double max_speed_;
  double max_angular_speed_;

  gz::msgs::Pose last_box_pose_;
  std::mutex pose_mutex_;
  bool box_pose_received_ = false;

  rclcpp::CallbackGroup::SharedPtr cb_group_subscriber_;
  rclcpp::CallbackGroup::SharedPtr cb_group_services_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_;

  // Octomap reset client (/octomap_server/reset)
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
};

#endif  // BOX_FOLLOWER__BOX_FOLLOWER_HPP_
