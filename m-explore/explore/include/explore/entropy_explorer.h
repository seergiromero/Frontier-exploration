/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Sergi Romero
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef ENTROPY_EXPLORER_H_
#define ENTROPY_EXPLORER_H_

// Standard library
#include <vector>
#include <random>
#include <memory>
#include <cmath>
#include <queue>

// ROS2 core
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Actions
#include <nav2_msgs/action/navigate_to_pose.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// Nav2 Costmap - NOTA: usa .hpp no .h
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// Headers del proyecto m-explore
#include <explore/costmap_client.h>
#include <explore/frontier_search.h>

namespace explore
{

// Estructura simple para nodos RRT
struct RRTNode {
geometry_msgs::msg::Point position;
double cost;
int parent_idx;
double information_gain;

RRTNode() : cost(0.0), parent_idx(-1), information_gain(0.0) {}
};

class EntropyExplorer : public rclcpp::Node
{
public:
EntropyExplorer();
~EntropyExplorer();

private:
// ========== Core Functions ==========
void makePlan();
void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

// ========== Entropy Calculations ==========
double calculateCellEntropy(uint8_t cost_value);
double calculateInformationGain(
    const geometry_msgs::msg::Point& position);

// ========== RRT* Sampling ==========
std::vector<RRTNode> generateRRTCandidates(
    const geometry_msgs::msg::Point& start_pos);

geometry_msgs::msg::Point sampleHighEntropyRegion();

int findNearestNode(
    const std::vector<RRTNode>& tree,
    const geometry_msgs::msg::Point& point);

geometry_msgs::msg::Point steer(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to,
    double step_size);

bool isPointValid(const geometry_msgs::msg::Point& point);

bool isPathValid(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to);

std::vector<int> findNearbyNodes(
    const std::vector<RRTNode>& tree,
    const geometry_msgs::msg::Point& point,
    double radius);

// ========== Utilities ==========
double distance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b);

geometry_msgs::msg::Point selectBestGoal(
    const std::vector<RRTNode>& candidates,
    const geometry_msgs::msg::Point& current_pos);

bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

bool isAdjacentToFreeSpace(unsigned int mx, unsigned int my);

// Verificar si la exploración está completa
bool isExplorationComplete();

// ========== Navigation ==========
void reachedGoal(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result,
    const geometry_msgs::msg::Point& goal);

void stop(bool finished_exploring);
void resume();
void returnToInitialPose();

// ========== Visualization ==========
void visualizeCandidates(
    const std::vector<RRTNode>& candidates,
    const geometry_msgs::msg::Point& selected_goal);

// ========== Type Aliases ==========
using NavigationGoalHandle = 
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

// ========== ROS2 Components ==========
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;
Costmap2DClient costmap_client_;

rclcpp::TimerBase::SharedPtr exploring_timer_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr move_base_client_;

// ========== Parameters ==========
double planner_frequency_;
double progress_timeout_;
bool visualize_;
bool return_to_init_;
std::string robot_base_frame_;

int rrt_samples_;
double rrt_step_size_;
double rrt_goal_bias_;
double info_gain_weight_;
double distance_weight_;
double sensor_range_;
double min_information_gain_;

// ========== State Variables ==========
geometry_msgs::msg::Point prev_goal_;
geometry_msgs::msg::Pose initial_pose_;
std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
rclcpp::Time last_progress_;
double prev_distance_;
size_t last_markers_count_;
bool resuming_;

std::mt19937 rng_;
rclcpp::Logger logger_;
};

}  // namespace explore

#endif  // ENTROPY_EXPLORER_H_