#include "frontier_exploration_manager/frontier_exploration_manager.hpp"


#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace frontier_exploration_manager
{
Explore::Explore()
  : Node("frontier_exploration_manager_node")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  const std::string nav2_action_name =
    this->declare_parameter<std::string>("nav2_action_name",
                                         "navigate_to_pose");
  const bool auto_start = 
    this->declare_parameter<bool>("auto_start", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("auto_start", exploration_active_);

  exploration_active_ = auto_start;
  returning_home_ = false;

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, nav2_action_name);

  // -----------------------------------------------------------
  // Action Server for "ExploreArea"
  // -----------------------------------------------------------
  exploration_action_server_ = rclcpp_action::create_server<ExploreArea>(
      this,
      "explore_area",   // Action name to call externally
      std::bind(&Explore::handle_goal, this, _1, _2),
      std::bind(&Explore::handle_cancel, this, _1),
      std::bind(&Explore::handle_accepted, this, _1));
  

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_exploration_manager/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "frontier_exploration_manager/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "[frontier_exploration_manager]: Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  exploring_timer_->execute_callback(std::shared_ptr<void>());
}

Explore::~Explore()
{
  stop();

  if (current_goal_handle_)
  {
      auto result = std::make_shared<ExploreArea::Result>();
      result->success = false;
      result->summary = "[frontier_exploration_manager]: Shutting down exploration.";
  
      current_goal_handle_->succeed(result);
      current_goal_handle_.reset();
  }
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();

    if (current_goal_handle_)
    {
      auto result = std::make_shared<ExploreArea::Result>();
      result->success = false;
      result->summary = "[frontier_exploration_manager]: STOP Requested using the resume topic.";
  
      current_goal_handle_->succeed(result);
      current_goal_handle_.reset();
    }

  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::msg::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
#ifdef ELOQUENT
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#elif DASHING
  m.lifetime = rclcpp::Duration(0);  // deprecated in galactic warning
#else
  m.lifetime = rclcpp::Duration::from_seconds(0);  // foxy onwards
#endif
  // m.lifetime = rclcpp::Duration::from_nanoseconds(0); // suggested in
  // galactic
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    // m.type = visualization_msgs::msg::Marker::POINTS;
    // m.id = int(id);
    // // m.pose.position = {}; // compile warning
    // m.scale.x = 0.1;
    // m.scale.y = 0.1;
    // m.scale.z = 0.1;
    // m.points = frontier.points;
    // if (goalOnBlacklist(frontier.centroid)) {
    //   m.color = red;
    // } else {
    //   m.color = blue;
    // }
    // markers.push_back(m);
    // ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // If exploration is disabled, skip the loop
  if (!exploration_active_){
    RCLCPP_INFO_ONCE(logger_, "[frontier_exploration_manager]: Waiting for activation action.");
    return;
  }

  returning_home_ = false;
  
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "[frontier_exploration_manager]: No frontiers found, stopping.");
    stop(true);

    if (current_goal_handle_)
    {
      auto result = std::make_shared<ExploreArea::Result>();
      result->success = true;
      result->summary = "[frontier_exploration_manager]: No frontiers found, stopping.";
  
      current_goal_handle_->succeed(result);
      current_goal_handle_.reset();
    }

    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "[frontier_exploration_manager]: All frontiers traversed/tried out, stopping.");
    stop(true);

    if (current_goal_handle_)
    {
      auto result = std::make_shared<ExploreArea::Result>();
      result->success = true;
      result->summary = "[frontier_exploration_manager]: All frontiers traversed/tried out, stopping.";
  
      current_goal_handle_->succeed(result);
      current_goal_handle_.reset();
    }

    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // send_goal_options.goal_response_callback =
  // std::bind(&Explore::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);


  if (current_goal_handle_)
  {
    auto feedback = std::make_shared<ExploreArea::Feedback>();
  
    // High-level status
    if (returning_home_) {
      feedback->status = "returning_home";
      feedback->returning_home = true;
    } else if (!exploration_active_) {
      feedback->status = "stopped";
    } else {
      feedback->status = "exploring";
    }
   
    current_goal_handle_->publish_feedback(feedback);
  }

}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Returning to initial pose.");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, send_goal_options);

  returning_home_ = true;
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: Goal was aborted");
      frontier_blacklist_.push_back(frontier_goal);
      RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: Adding current goal to black list");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "[frontier_exploration_manager]: Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "[frontier_exploration_manager]: Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Exploration started.");
  returning_home_ = false;
  exploration_active_ = true;
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Exploration stopped.");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
    exploration_active_ = false;
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "[frontier_exploration_manager]: Exploration resuming.");
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  exploring_timer_->execute_callback(std::shared_ptr<void>());
}

// ===================================
// Action to activate the exploration
// ===================================
rclcpp_action::GoalResponse
Explore::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExploreArea::Goal> goal)
{
  RCLCPP_INFO(get_logger(),
              "[frontier_exploration_manager]: Requested exploration of area '%s'",
              goal->area.c_str());

  if (exploration_active_) {
    RCLCPP_WARN(get_logger(), "[frontier_exploration_manager]: Exploration already active: rejecting goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Explore::handle_cancel(
    [[maybe_unused]] const std::shared_ptr<GoalHandleExploreArea> goal_handle)
{
  RCLCPP_WARN(get_logger(), "[frontier_exploration_manager]: Cancelled exploration!");

  exploration_active_ = false;

  stop(true);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void Explore::handle_accepted(
    const std::shared_ptr<GoalHandleExploreArea> goal_handle)
{
  current_goal_handle_ = goal_handle;

  RCLCPP_INFO(get_logger(), "[frontier_exploration_manager]: Explore_area goal accepted");

  // START EXPLORATION
  start();
  
  // Reset progress variables if needed
  last_progress_ = this->now();
  resuming_ = true;

  frontier_blacklist_.clear();
}



}  // namespace frontier_exploration_manager

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<frontier_exploration_manager::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
