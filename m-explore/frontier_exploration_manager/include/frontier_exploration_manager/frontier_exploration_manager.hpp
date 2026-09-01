#ifndef FRONTIER_EXPLORATION_MANAGER__FRONTIER_EXPLORATION_MANAGER_HPP_
#define FRONTIER_EXPLORATION_MANAGER__FRONTIER_EXPLORATION_MANAGER_HPP_

#include "explore/costmap_client.h"
#include "explore/frontier_search.h"

#include "frontier_exploration_manager_interfaces/action/explore_area.hpp"

#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace frontier_exploration_manager
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  // Action client to request navigation to Nav2
  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  // Action server to call for the exploration
  using ExploreArea = frontier_exploration_manager_interfaces::action::ExploreArea;
  using GoalHandleExploreArea = rclcpp_action::ServerGoalHandle<ExploreArea>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;
  rclcpp::Logger logger_ = rclcpp::get_logger("ExploreNode");
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  explore::Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);
  bool returning_home_;

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  bool resuming_ = false;
  bool exploration_active_ = false;

  // Action Server to activate the exploration
  rclcpp_action::Server<ExploreArea>::SharedPtr exploration_action_server_;
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ExploreArea::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleExploreArea> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleExploreArea> goal_handle);

  // Store current running goal
  std::shared_ptr<GoalHandleExploreArea> current_goal_handle_;
};

}  // namespace frontier_exploration_manager

#endif
