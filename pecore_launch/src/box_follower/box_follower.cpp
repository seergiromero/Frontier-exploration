#include "box_follower/box_follower.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

BoxFollower::BoxFollower(const rclcpp::NodeOptions &options)
: Node("box_follower", options)
{
  // Parameters
  world_name_        = declare_parameter("world_name", "void");
  box_name_          = declare_parameter("box_name", "aruco_box");
  pose_topic_        = declare_parameter("pose_topic", "/b2/pose");
  desired_distance_  = declare_parameter("desired_distance", 2.0);
  max_speed_         = declare_parameter("max_speed", 1.0);
  max_angular_speed_ = declare_parameter("max_angular_speed", 1.0);

  // Callback groups
  cb_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_services_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Subscribe to /world/<world>/pose/info
  std::string topic = "/world/" + world_name_ + "/pose/info";
  bool ok = gz_node_.Subscribe<gz::msgs::Pose_V>(
      topic,
      [this](const gz::msgs::Pose_V &msg)
      {
        this->poseInfoCallback(msg);
      });

  if (!ok)
    RCLCPP_ERROR(get_logger(), "Failed to subscribe to [%s]", topic.c_str());
  else
    RCLCPP_INFO(get_logger(), "Subscribed to [%s]", topic.c_str());

  // Wait for Gazebo and box
  waitForGazeboServices();
  waitForBoxPresence();

  // Octomap reset client
  octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>(
      "/octomap_server/reset", rclcpp::ServicesQoS(), cb_group_services_);
  while (rclcpp::ok() && !octomap_reset_client_->service_is_ready()) {
    // Attempt a quick wait without blocking the executor for long
    auto ok = octomap_reset_client_->wait_for_service(1000ms);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Service /octomap_server/reset not available yet.");
    }
  }

  // ROS subscriber
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_subscriber_;

  sub_pose_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pose_topic_, 10,
      std::bind(&BoxFollower::robotPoseCallback, this, std::placeholders::_1),
      sub_opts);

  RCLCPP_INFO(get_logger(),
              "BoxFollower ready for [%s] in world [%s], keeping %.2fm distance (max speed %.2f m/s)",
              box_name_.c_str(), world_name_.c_str(), desired_distance_, max_speed_);
}

// ---------------------------------------------------------------------------
// Wait until /world/<world>/set_pose appears in the service list

bool BoxFollower::waitForGazeboServices()
{
  const auto timeout = std::chrono::seconds(10);
  const auto sleep_dur = std::chrono::milliseconds(500);
  auto start = this->now();

  while ((this->now() - start) < timeout)
  {
    std::vector<std::string> services;
    gz_node_.ServiceList(services);

    for (const auto &srv : services)
    {
      if (srv == "/world/" + world_name_ + "/set_pose")
      {
        RCLCPP_INFO(get_logger(),
                    "Gazebo service [/world/%s/set_pose] is available.",
                    world_name_.c_str());
        return true;
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for /world/%s/set_pose service...",
                         world_name_.c_str());
    rclcpp::sleep_for(sleep_dur);
  }

  RCLCPP_WARN(get_logger(),
              "Timed out waiting for Gazebo service [/world/%s/set_pose].",
              world_name_.c_str());
  return false;
}

// ---------------------------------------------------------------------------

bool BoxFollower::waitForBoxPresence()
{
  RCLCPP_INFO(get_logger(), "Waiting for box [%s] pose updates...", box_name_.c_str());
  const auto timeout = std::chrono::seconds(15);
  auto start = this->now();

  while ((this->now() - start) < timeout)
  {
    if (box_pose_received_)
    {
      RCLCPP_INFO(get_logger(), "Box [%s] detected via /pose/info.", box_name_.c_str());
      return true;
    }
    rclcpp::sleep_for(500ms);
  }

  RCLCPP_WARN(get_logger(), "Timeout waiting for box [%s] pose.", box_name_.c_str());
  return false;
}

// ---------------------------------------------------------------------------
// Called continuously with Pose_V messages

void BoxFollower::poseInfoCallback(const gz::msgs::Pose_V &msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  for (int i = 0; i < msg.pose_size(); ++i)
  {
    if (msg.pose(i).name() == box_name_)
    {
      last_box_pose_ = msg.pose(i);
      box_pose_received_ = true;
      break;
    }
  }
}

// ---------------------------------------------------------------------------

bool BoxFollower::getCachedBoxPose(gz::msgs::Pose &pose)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!box_pose_received_)
    return false;
  pose = last_box_pose_;
  return true;
}

// ---------------------------------------------------------------------------

bool BoxFollower::setBoxPose(const gz::msgs::Pose &pose)
{
  gz::msgs::Boolean rep;
  bool result = false;

  bool executed = gz_node_.Request(
      "/world/" + world_name_ + "/set_pose",
      pose, 1000, rep, result);

  return executed && result && rep.data();
}

// ---------------------------------------------------------------------------

void BoxFollower::resetOctomap()
{
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  (void)octomap_reset_client_->async_send_request(
      req,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        RCLCPP_INFO(this->get_logger(), "Octomap reset requested after box pose update.");
      });
}

// ---------------------------------------------------------------------------

void BoxFollower::robotPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
  if (msg->poses.empty())
    return;

  const auto &pose = msg->poses[0];
  double rx = pose.position.x;
  double ry = pose.position.y;

  double qx = pose.orientation.x;
  double qy = pose.orientation.y;
  double qz = pose.orientation.z;
  double qw = pose.orientation.w;
  double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));

  gz::msgs::Pose boxPose;
  if (!getCachedBoxPose(boxPose))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No cached box pose yet");
    return;
  }

  double bx = boxPose.position().x();
  double by = boxPose.position().y();
  double bz = boxPose.position().z();

  double dist_euc = std::sqrt(std::pow(rx-bx,2) + std::pow(ry-by,2));
  if (dist_euc > desired_distance_)
    return;

  double desired_x = rx + desired_distance_ * std::cos(yaw);
  double desired_y = ry + desired_distance_ * std::sin(yaw);

  double dx = desired_x - bx;
  double dy = desired_y - by;
  double dist = std::hypot(dx, dy);
  if (dist < 1e-4)
    return;

  double dt = 0.05;
  double max_step = max_speed_ * dt;
  if (dist > max_step)
  {
    dx *= (max_step / dist);
    dy *= (max_step / dist);
  }

  double offset_x = 1.0;

  gz::msgs::Pose newPose;
  newPose.set_name(box_name_);
  newPose.mutable_position()->set_x(bx + dx + offset_x);
  newPose.mutable_position()->set_y(by + dy);
  newPose.mutable_position()->set_z(bz);

  double box_yaw = yaw;
  double half = box_yaw * 0.5;
  newPose.mutable_orientation()->set_w(std::cos(half));
  newPose.mutable_orientation()->set_z(std::sin(half));

  if (!setBoxPose(newPose))
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to set box pose");
  else
  {
    // Reset Octomap in the mutually-exclusive service group
    this->resetOctomap();
  }
}

// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoxFollower>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
