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

#include <explore/entropy_explorer.h>
#include <thread>

namespace explore
{

// ==================== Constructor ====================
EntropyExplorer::EntropyExplorer()
  : Node("entropy_explorer_node")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)  // Client to access Nav2 global costmap
  , prev_distance_(0)
  , last_markers_count_(0)
  , resuming_(false)
  , rng_(std::random_device{}())  
  , logger_(rclcpp::get_logger("entropy_explorer"))
{
  // Declare parameters
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", true);
  this->declare_parameter<bool>("return_to_init", false);
//  this->declare_parameter<std::string>("robot_base_frame", "b2/base_link");
  
  // Entropy-based parameters
  this->declare_parameter<int>("rrt_samples", 100);  // Number of nodes to generate in RRT* tree
  this->declare_parameter<float>("rrt_step_size", 2.0);  // Max distance between consecutive nodes (m)
  this->declare_parameter<float>("rrt_goal_bias", 0.15);  // Probability to sample high-entropy regions
  this->declare_parameter<float>("info_gain_weight", 2.0);  // Weight for information gain in objective
  this->declare_parameter<float>("distance_weight", 1.0);  // Weight for distance (exploration vs efficiency trade-off)
  this->declare_parameter<float>("sensor_range", 5.0);  // Sensor radius for info gain calculation (m)
  this->declare_parameter<float>("min_information_gain", 0.5);  // Minimum threshold for valid goals
  
  // Get parameters
  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", progress_timeout_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  
  this->get_parameter("rrt_samples", rrt_samples_);
  this->get_parameter("rrt_step_size", rrt_step_size_);
  this->get_parameter("rrt_goal_bias", rrt_goal_bias_);
  this->get_parameter("info_gain_weight", info_gain_weight_);
  this->get_parameter("distance_weight", distance_weight_);
  this->get_parameter("sensor_range", sensor_range_);
  this->get_parameter("min_information_gain", min_information_gain_);
  

  // Initialize action client
  // IMPORTANT: Nav2 uses NavigateToPose for goal-based navigation
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, "navigate_to_pose");

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "entropy_explorer/candidates", 10);
  }

  // Resume/stop subscription
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&EntropyExplorer::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting for navigation action server...");
  move_base_client_->wait_for_action_server();  // BLOCKING call until Nav2 is ready
  RCLCPP_INFO(logger_, "Connected to navigation server");

  // Get initial pose if return_to_init is enabled
  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Recording initial pose");
    geometry_msgs::msg::TransformStamped transform;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      // Lookup transform from map to base_link to save initial position
      transform = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transform.transform.translation.x;
      initial_pose_.position.y = transform.transform.translation.y;
      initial_pose_.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Failed to get initial pose: %s", ex.what());
      return_to_init_ = false;
    }
  }

  // Start exploration timer
  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  
  // Execute first planning iteration immediately (kickstart exploration)
  exploring_timer_->execute_callback(std::shared_ptr<void>());
  
  RCLCPP_INFO(logger_, "Entropy Explorer initialized successfully");
}

EntropyExplorer::~EntropyExplorer()
{
  stop(false);
}

// ==================== Shannon Entropy Calculation ====================
double EntropyExplorer::calculateCellEntropy(uint8_t cost_value)
{
  // Convert costmap value to probability
  double p_occupied;
  
  // IMPORTANT: Costmap convention: 0-100 = occupancy probability, 255 = unknown
  if (cost_value == 255) {  // Unknown (NO_INFORMATION)
    p_occupied = 0.5;  // Maximum uncertainty → maximum entropy
  } else {
    p_occupied = cost_value / 100.0;
  }
  
  // Shannon entropy: H(X) = -p*log(p) - (1-p)*log(1-p)
  // Measures uncertainty: 0 = complete certainty, 1 = maximum uncertainty
  if (p_occupied <= 0.0 || p_occupied >= 1.0) {
    return 0.0;  // No uncertainty (cell is fully known)
  }
  
  double entropy = -p_occupied * std::log2(p_occupied) 
                  - (1.0 - p_occupied) * std::log2(1.0 - p_occupied);
  
  return entropy;
}

// ==================== RRT* Sampling ====================
std::vector<RRTNode> EntropyExplorer::generateRRTCandidates(
    const geometry_msgs::msg::Point& start_pos)
{
  std::vector<RRTNode> tree;
  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  
  // Initialize tree with start position (RRT* root node)
  RRTNode start_node;
  start_node.position = start_pos;
  start_node.cost = 0.0;
  start_node.parent_idx = -1;  // No parent (this is the root)
  tree.push_back(start_node);
  
  // Get map bounds for uniform sampling
  double origin_x = costmap->getOriginX();
  double origin_y = costmap->getOriginY();
  double size_x = costmap->getSizeInMetersX();
  double size_y = costmap->getSizeInMetersY();
  
  std::uniform_real_distribution<double> dist_x(origin_x, origin_x + size_x);
  std::uniform_real_distribution<double> dist_y(origin_y, origin_y + size_y);
  std::uniform_real_distribution<double> dist_bias(0.0, 1.0);
  
  // RRT* expansion - Incremental tree construction
  for (int i = 0; i < rrt_samples_; ++i) {
    geometry_msgs::msg::Point random_point;
    
    // IMPORTANT: Heuristic goal biasing towards unknown regions
    if (dist_bias(rng_) < rrt_goal_bias_) {
      random_point = sampleHighEntropyRegion();  // Sample unknown cells
    } else {
      random_point.x = dist_x(rng_);  // Uniform random sampling
      random_point.y = dist_y(rng_);
      random_point.z = 0.0;
    }
    
    // Find nearest node in tree (basic RRT algorithm)
    int nearest_idx = findNearestNode(tree, random_point);
    if (nearest_idx < 0) continue;
    
    // Steer towards random point (limits step size)
    geometry_msgs::msg::Point new_point = steer(
        tree[nearest_idx].position, random_point, rrt_step_size_);
    
    // Check if new point is valid (collision-free)
    if (!isPointValid(new_point)) continue;
    
    double rewire_radius = rrt_step_size_ * 2.0;
    std::vector<int> nearby_indices = findNearbyNodes(
        tree, new_point, rewire_radius);
    
    // Find best parent (minimum cost connection)
    int best_parent = nearest_idx;
    double best_cost = tree[nearest_idx].cost + 
                      distance(tree[nearest_idx].position, new_point);
    
    for (int idx : nearby_indices) {
      double candidate_cost = tree[idx].cost + 
                            distance(tree[idx].position, new_point);
      // Rewiring: change parent if it reduces cost and path is valid
      if (candidate_cost < best_cost && 
          isPathValid(tree[idx].position, new_point)) {
        best_parent = idx;
        best_cost = candidate_cost;
      }
    }
    
    // Add new node
    RRTNode new_node;
    new_node.position = new_point;
    new_node.cost = best_cost;  // Accumulated cost from root
    new_node.parent_idx = best_parent;
    new_node.information_gain = calculateInformationGain(new_point);  // Exploration metric
    tree.push_back(new_node);
  }
  
  return tree;
}

// ==================== Helper Functions ====================
geometry_msgs::msg::Point EntropyExplorer::sampleHighEntropyRegion()
{
  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  std::vector<geometry_msgs::msg::Point> high_entropy_cells;
  
  // OPTIMIZATION: Sparse sampling (every 5 cells) for efficiency
  for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x += 5) {
    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y += 5) {
      if (costmap->getCost(x, y) == 255) {  // NO_INFORMATION
        geometry_msgs::msg::Point p;
        costmap->mapToWorld(x, y, p.x, p.y);
        p.z = 0.0;
        high_entropy_cells.push_back(p);
      }
    }
  }
  
  if (!high_entropy_cells.empty()) {
    // Uniform random selection from unknown cells
    std::uniform_int_distribution<size_t> dist(0, high_entropy_cells.size() - 1);
    return high_entropy_cells[dist(rng_)];
  }
  
  // Fallback: random point if no unknown cells available
  geometry_msgs::msg::Point p;
  p.x = costmap->getOriginX() + 
        std::uniform_real_distribution<double>(0, costmap->getSizeInMetersX())(rng_);
  p.y = costmap->getOriginY() + 
        std::uniform_real_distribution<double>(0, costmap->getSizeInMetersY())(rng_);
  p.z = 0.0;
  return p;
}

int EntropyExplorer::findNearestNode(
    const std::vector<RRTNode>& tree,
    const geometry_msgs::msg::Point& point)
{
  if (tree.empty()) return -1;
  
  // Linear search for nearest node (O(n) complexity)
  // NOTE: For large trees, consider using KD-tree for O(log n) queries
  int nearest_idx = 0;
  double min_dist = distance(tree[0].position, point);
  
  for (size_t i = 1; i < tree.size(); ++i) {
    double dist = distance(tree[i].position, point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  
  return nearest_idx;
}

geometry_msgs::msg::Point EntropyExplorer::steer(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to,
    double step_size)
{
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  
  geometry_msgs::msg::Point result;
  // Limit maximum expansion distance of tree
  if (dist <= step_size) {
    result = to;  // If within step, use target directly
  } else {
    // Interpolate in direction of target
    double ratio = step_size / dist;
    result.x = from.x + dx * ratio;
    result.y = from.y + dy * ratio;
    result.z = 0.0;
  }
  
  return result;
}

bool EntropyExplorer::isPointValid(const geometry_msgs::msg::Point& point)
{
  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  unsigned int mx, my;
  
  if (!costmap->worldToMap(point.x, point.y, mx, my)) {
    return false;  // Outside map bounds
  }
  
  uint8_t cost = costmap->getCost(mx, my);
  
  // Reject if in obstacle (allow unknown for exploration)
  return cost < 253;  // Less than LETHAL_OBSTACLE (253-254)
}

bool EntropyExplorer::isPathValid(
    const geometry_msgs::msg::Point& from,
    const geometry_msgs::msg::Point& to)
{
  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  // Number of checks based on costmap resolution (one check per cell)
  int steps = static_cast<int>(dist / costmap->getResolution());
  
  // Linear interpolation for collision checking
  for (int i = 0; i <= steps; ++i) {
    double ratio = static_cast<double>(i) / steps;
    geometry_msgs::msg::Point check_point;
    check_point.x = from.x + dx * ratio;
    check_point.y = from.y + dy * ratio;
    check_point.z = 0.0;
    
    if (!isPointValid(check_point)) {
      return false;  // Path blocked
    }
  }
  
  return true;
}

std::vector<int> EntropyExplorer::findNearbyNodes(
    const std::vector<RRTNode>& tree,
    const geometry_msgs::msg::Point& point,
    double radius)
{
  std::vector<int> nearby;
  
  // Linear search for nodes within radius
  // Used for rewiring in RRT*
  for (size_t i = 0; i < tree.size(); ++i) {
    if (distance(tree[i].position, point) <= radius) {
      nearby.push_back(i);
    }
  }
  
  return nearby;
}

double EntropyExplorer::distance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);  // 2D Euclidean distance
}

// ==================== Goal Selection ====================
geometry_msgs::msg::Point EntropyExplorer::selectBestGoal(
const std::vector<RRTNode>& candidates,
const geometry_msgs::msg::Point& current_pos)
{
double best_score = -std::numeric_limits<double>::infinity();
geometry_msgs::msg::Point best_goal = current_pos;
bool found_valid_goal = false;
int best_idx = -1;

RCLCPP_INFO(logger_, "Evaluating %zu candidates", candidates.size());

for (size_t i = 0; i < candidates.size(); ++i) {
const auto& node = candidates[i];

// Skip root node (index 0) - this is robot's current position
if (i == 0) {
  RCLCPP_INFO(logger_, "Skipping root node");
  continue;
}

// Skip nodes too close to robot (avoid trivial goals)
if (distance(node.position, current_pos) < 0.3) {
  RCLCPP_INFO(logger_, "Skipping too close node");
  continue;
}

// Skip blacklisted nodes (previous failed goals)
if (goalOnBlacklist(node.position)) {
  RCLCPP_INFO(logger_, "Skipping blacklisted node");
  continue;
}

// Filter candidates with low information gain
if (node.information_gain < min_information_gain_) {
  RCLCPP_INFO(logger_, "Skipping low info gain: %.3f", node.information_gain);
  continue;
}

double dist = distance(node.position, current_pos);
double info_gain = node.information_gain;

double score = info_gain_weight_ * info_gain 
                - distance_weight_ * (dist / 10.0);  // Distance normalization

if (score > best_score) {
  best_score = score;
  best_goal = node.position;
  best_idx = i;
  found_valid_goal = true;
  RCLCPP_INFO(logger_, "New best candidate %zu", i);
}
}

if (found_valid_goal) {
RCLCPP_INFO(logger_, "Selected goal idx %d at (%.2f, %.2f) with score %.3f", 
            best_idx, best_goal.x, best_goal.y, best_score);
} else {
RCLCPP_WARN(logger_, "No valid exploration goals found in %zu candidates", 
            candidates.size());
// Show why no goals were found (debugging)
if (!candidates.empty()) {
  for (size_t i = 0; i < std::min(candidates.size(), (size_t)5); ++i) {
    RCLCPP_WARN(logger_, "Candidate %zu: pos (%.2f, %.2f), info=%.3f", 
                i, candidates[i].position.x, candidates[i].position.y, 
                candidates[i].information_gain);
  }
}
}

return best_goal;
}

// ==================== Main Planning Function ====================
void EntropyExplorer::makePlan()
{
  // FIRST: Check if exploration is complete (termination criterion)
  if (isExplorationComplete()) {
      RCLCPP_INFO(logger_, "========================================");
      RCLCPP_INFO(logger_, "  EXPLORATION COMPLETED SUCCESSFULLY!  ");
      RCLCPP_INFO(logger_, "========================================");
      stop(true);
      return;
  }
  
  auto pose = costmap_client_.getRobotPose();
  
  // Generate RRT* candidates (exploration tree)
  auto candidates = generateRRTCandidates(pose.position);
  
  RCLCPP_INFO(logger_, "Generated %zu RRT candidates", candidates.size());
  
  // Check if RRT generated valid candidates
  if (candidates.size() <= 1) {
      RCLCPP_WARN(logger_, "No valid RRT candidates. Checking exploration status...");
      
      // Double-check if we actually finished
      if (isExplorationComplete()) {
          RCLCPP_INFO(logger_, "Confirmed: Exploration complete!");
          stop(true);
      } else {
          RCLCPP_WARN(logger_, "No candidates but exploration not complete. Continuing...");
          // Wait briefly and retry (avoid busy-waiting)
          std::this_thread::sleep_for(std::chrono::seconds(2));
      }
      return;
  }
  
  // Select best goal (multi-objective optimization)
  geometry_msgs::msg::Point target = selectBestGoal(candidates, pose.position);

// Verify selected goal is valid (sanity check)
bool target_in_candidates = false;
for (const auto& node : candidates) {
  if (distance(target, node.position) < 0.1) {
    target_in_candidates = true;
    break;
  }
}

// Fallback if selectBestGoal returned invalid position
if (!target_in_candidates) {
  RCLCPP_ERROR(logger_, "Selected goal NOT in candidates list!");
  RCLCPP_ERROR(logger_, "Target: (%.2f, %.2f)", target.x, target.y);
  
  // Fallback: select candidate with highest information
  double max_info = 0;
  for (size_t i = 1; i < candidates.size(); ++i) {
    if (candidates[i].information_gain > max_info && 
        !goalOnBlacklist(candidates[i].position) &&
        distance(candidates[i].position, pose.position) > 0.5) {
      max_info = candidates[i].information_gain;
      target = candidates[i].position;
      target_in_candidates = true;
    }
  }
}

// Visualize for debugging in RViz
if (visualize_) {
  visualizeCandidates(candidates, target);
}

// Check progress (detect if robot is stuck)
bool same_goal = distance(prev_goal_, target) < 0.5;
prev_goal_ = target;

if (!same_goal) {
  last_progress_ = this->now();  // Reset timer if goal changed
}

// Timeout check (goal unreachable after X seconds)
if ((this->now() - last_progress_ > 
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
  frontier_blacklist_.push_back(target);  // Add to blacklist
  RCLCPP_WARN(logger_, "Goal timeout, blacklisting and replanning");
  makePlan();  // Recursive replanning
  return;
}

if (resuming_) {
  resuming_ = false;
}

// If goal is same as previous, don't resend (avoid spam)
if (same_goal) {
  return;
}

// Send goal to Nav2
auto goal = nav2_msgs::action::NavigateToPose::Goal();
goal.pose.pose.position = target;
goal.pose.pose.orientation.w = 1.0;  // No preferred orientation
goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
goal.pose.header.stamp = this->now();

auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
// Callback when robot reaches or fails goal
options.result_callback = [this, target](const NavigationGoalHandle::WrappedResult& result) {
  reachedGoal(result, target);
};

move_base_client_->async_send_goal(goal, options);
RCLCPP_INFO(logger_, "Sent navigation goal to (%.2f, %.2f)", target.x, target.y);
}

// ==================== Visualization ====================
void EntropyExplorer::visualizeCandidates(
const std::vector<RRTNode>& candidates,
const geometry_msgs::msg::Point& selected_goal)
{
visualization_msgs::msg::MarkerArray markers;

auto pose = costmap_client_.getRobotPose();

// Calculate scores for all candidates (same metric as selectBestGoal)
std::vector<double> scores;
double max_score = -std::numeric_limits<double>::infinity();
double min_score = std::numeric_limits<double>::infinity();

for (size_t i = 0; i < candidates.size(); ++i) {
if (i == 0) {
  scores.push_back(0.0);  // Root node (robot position)
  continue;
}

double dist = distance(candidates[i].position, pose.position);
double info_gain = candidates[i].information_gain;

// IMPORTANT: Same calculation as selectBestGoal (consistency is crucial)
double score = info_gain_weight_ * info_gain 
                - distance_weight_ * (dist / 10.0);

scores.push_back(score);
max_score = std::max(max_score, score);
min_score = std::min(min_score, score);
}

// Avoid division by zero in normalization
if (max_score - min_score < 0.01) {
max_score = min_score + 1.0;
}

RCLCPP_INFO(logger_, "Score range: [%.3f, %.3f]", min_score, max_score);

// Visualize with score-based colors
for (size_t i = 0; i < candidates.size(); ++i) {
if (i == 0) continue;  // Skip root

visualization_msgs::msg::Marker marker;
marker.header.frame_id = costmap_client_.getGlobalFrameID();
marker.header.stamp = this->now();
marker.ns = "candidates";
marker.id = i;
marker.type = visualization_msgs::msg::Marker::SPHERE;
marker.action = visualization_msgs::msg::Marker::ADD;
marker.pose.position = candidates[i].position;
marker.pose.orientation.w = 1.0;

bool is_selected = (distance(candidates[i].position, selected_goal) < 0.1);

if (is_selected) {
  // Selected goal: BLUE and larger
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
} else {
  // Normalize score between 0 and 1
  double normalized_score = (scores[i] - min_score) / (max_score - min_score);
  
  // Size proportional to score
  double scale = 0.15 + normalized_score * 0.2;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  
  // Color gradient based on score (red = low, yellow = medium, green = high)
  if (normalized_score < 0.5) {
    marker.color.r = 1.0;
    marker.color.g = normalized_score * 2.0;
    marker.color.b = 0.0;
  } else {
    marker.color.r = 1.0 - (normalized_score - 0.5) * 2.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  marker.color.a = 0.6;
}

marker.lifetime = rclcpp::Duration::from_seconds(3.0);  // Auto-delete after 3s
markers.markers.push_back(marker);

// Text label with score above marker
visualization_msgs::msg::Marker text;
text.header = marker.header;
text.ns = "candidate_text";
text.id = i + 10000;  // Offset to avoid ID collision
text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
text.action = visualization_msgs::msg::Marker::ADD;
text.pose.position = candidates[i].position;
text.pose.position.z += 0.3;  // Raise above sphere
text.pose.orientation.w = 1.0;
text.scale.z = 0.12;  // Text height

// Display SCORE instead of raw info_gain
text.text = std::to_string(i) + ":" + 
            std::to_string(static_cast<int>(scores[i] * 10)) + "s";  // "s" = score

text.color.r = 1.0;
text.color.g = 1.0;
text.color.b = 1.0;
text.color.a = 1.0;
text.lifetime = rclcpp::Duration::from_seconds(3.0);
markers.markers.push_back(text);
}

// Line from robot to selected goal
visualization_msgs::msg::Marker line;
line.header.frame_id = costmap_client_.getGlobalFrameID();
line.header.stamp = this->now();
line.ns = "goal_connection";
line.id = 0;
line.type = visualization_msgs::msg::Marker::LINE_STRIP;
line.action = visualization_msgs::msg::Marker::ADD;

line.points.push_back(pose.position);
line.points.push_back(selected_goal);

line.scale.x = 0.05;  // Line width
line.color.r = 1.0;
line.color.g = 1.0;
line.color.b = 0.0;  // Yellow line
line.color.a = 0.9;
line.lifetime = rclcpp::Duration::from_seconds(3.0);
markers.markers.push_back(line);

marker_array_publisher_->publish(markers);
}

// ==================== Other Functions ====================
void EntropyExplorer::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop(false);
  }
}

bool EntropyExplorer::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  // Check if goal is too close to any blacklisted position
  for (const auto& blacklisted : frontier_blacklist_) {
    if (distance(goal, blacklisted) < 0.5) {
      return true;
    }
  }
  return false;
}

void EntropyExplorer::reachedGoal(
    const NavigationGoalHandle::WrappedResult& result,
    const geometry_msgs::msg::Point& goal)
{
  // Handle navigation result
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Reached exploration goal");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(logger_, "Goal aborted, blacklisting");
      frontier_blacklist_.push_back(goal);  // Prevent retry of failed goal
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Goal canceled");
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code");
      break;
  }
  
  // Trigger next planning cycle
  makePlan();
}

void EntropyExplorer::stop(bool finished)
{
  RCLCPP_INFO(logger_, "Stopping exploration");
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
  
  if (return_to_init_ && finished) {
    returnToInitialPose();
  }
}

void EntropyExplorer::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Resuming exploration");
  exploring_timer_->reset();
  exploring_timer_->execute_callback(std::shared_ptr<void>());
}

void EntropyExplorer::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose");
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();
  
  auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  move_base_client_->async_send_goal(goal, options);
}



// ==================== NEW FUNCTIONS ====================

// 1. Detect if cell is frontier (adjacent to free space)
bool EntropyExplorer::isAdjacentToFreeSpace(unsigned int mx, unsigned int my)
{
  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  
  // Check 8-connected neighborhood
  for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;  // Skip center cell
          
          int nx = mx + dx;
          int ny = my + dy;
          
          if (nx >= 0 && nx < static_cast<int>(costmap->getSizeInCellsX()) &&
              ny >= 0 && ny < static_cast<int>(costmap->getSizeInCellsY())) {
              
              uint8_t cost = costmap->getCost(nx, ny);
              // Free space (no obstacle, no unknown)
              if (cost < 253 && cost != 255) {
                  return true;  // Found adjacent free cell
              }
          }
      }
  }
  return false;
}

// 2. Check if exploration is complete
bool EntropyExplorer::isExplorationComplete()
{
  nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
  
  int total_cells = 0;
  int unknown_cells = 0;
  int frontier_cells = 0;
  
  // OPTIMIZATION: Sampling for efficiency (check every N cells)
  int step = 3;
  
  for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x += step) {
      for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y += step) {
          total_cells++;
          uint8_t cost = costmap->getCost(x, y);
          
          if (cost == 255) {  // Unknown
              unknown_cells++;
              
              // Check if it's a valid frontier (adjacent to explored area)
              if (isAdjacentToFreeSpace(x, y)) {
                  frontier_cells++;
              }
          }
      }
  }
  
  double unknown_ratio = static_cast<double>(unknown_cells) / total_cells;
  double frontier_ratio = static_cast<double>(frontier_cells) / total_cells;
  
  RCLCPP_INFO(logger_, "Exploration stats - Unknown: %.1f%%, Frontiers: %.1f%%", 
              unknown_ratio * 100, frontier_ratio * 100);
  
  // IMPORTANT: Completion criteria (tunable thresholds)
  bool few_unknowns = unknown_ratio < 0.08;  // <8% unknown
  bool few_frontiers = frontier_ratio < 0.015;  // <1.5% frontiers
  
  if (few_unknowns || few_frontiers) {
      RCLCPP_INFO(logger_, "Exploration completion criteria met!");
      return true;
  }
  
  return false;
}

// 3. MODIFIED calculateInformationGain to prioritize structured areas
double EntropyExplorer::calculateInformationGain(
const geometry_msgs::msg::Point& position)
{
double total_info_gain = 0.0;
int unknown_near_obstacles = 0;  // Unknown cells near structure
int unknown_in_open = 0;  // Unknown cells in open space
int cells_in_range = 0;

nav2_costmap_2d::Costmap2D* costmap = costmap_client_.getCostmap();
double resolution = costmap->getResolution();

unsigned int mx, my;
if (!costmap->worldToMap(position.x, position.y, mx, my)) {
    return 0.0;  // Point outside map
}

int range_cells = static_cast<int>(sensor_range_ / resolution);

// Iterate through cells within sensor range
for (int dx = -range_cells; dx <= range_cells; ++dx) {
    for (int dy = -range_cells; dy <= range_cells; ++dy) {
        double dist = std::sqrt(dx * dx + dy * dy) * resolution;
        if (dist > sensor_range_) continue;  // Circular sensor range
        
        int cx = mx + dx;
        int cy = my + dy;
        
        if (cx < 0 || cx >= static_cast<int>(costmap->getSizeInCellsX()) ||
            cy < 0 || cy >= static_cast<int>(costmap->getSizeInCellsY())) {
            continue;
        }
        
        uint8_t cost = costmap->getCost(cx, cy);
        
        if (cost == 255) {  // Unknown cell
            // IMPROVEMENT: Search radius in METERS, not cells
            double obstacle_search_radius = 0.2;  // 20 cm in meters
            int obstacle_radius_cells = static_cast<int>(obstacle_search_radius / resolution);
            
            bool near_obstacle = false;
            
            // Search for obstacles in expanded radius
            for (int ox = -obstacle_radius_cells; ox <= obstacle_radius_cells; ++ox) {
                for (int oy = -obstacle_radius_cells; oy <= obstacle_radius_cells; ++oy) {
                    double obs_dist = std::sqrt(ox * ox + oy * oy) * resolution;
                    if (obs_dist > obstacle_search_radius) continue;
                    
                    int check_x = cx + ox;
                    int check_y = cy + oy;
                    
                    if (check_x >= 0 && check_x < costmap->getSizeInCellsX() &&
                        check_y >= 0 && check_y < costmap->getSizeInCellsY()) {
                        
                        uint8_t check_cost = costmap->getCost(check_x, check_y);
                        
                        // IMPROVEMENT: Lower threshold to detect obstacles
                        // Includes inflation layer and actual obstacles
                        if (check_cost >= 200) {  // Changed from 253 to 200
                            near_obstacle = true;
                            break;
                        }
                    }
                }
                if (near_obstacle) break;
            }
            
            // Distance-based weight decay
            double weight = 1.0 / (1.0 + dist);
            
            if (near_obstacle) {
                // VERY HIGH weight for unknown near obstacles (structured exploration)
                total_info_gain += weight * 5.0;  // Increased from 3.0 to 5.0
                unknown_near_obstacles++;
            } else {
                // VERY LOW weight for unknown in open areas (avoid open space)
                total_info_gain += weight * 0.05;  // Reduced from 0.2 to 0.05
                unknown_in_open++;
            }
            
            cells_in_range++;
        }
    }
}

// IMPROVEMENT: More aggressive structure bonus
double structure_bonus = 1.0;

if (unknown_near_obstacles > 2) {  // Reduced from 5 to 2
    // Saturated logarithmic bonus
    structure_bonus = 1.0 + 1.5 * std::log1p(unknown_near_obstacles / 2.0);
    // Example: 5 obstacles → bonus ~2.5x
    //         10 obstacles → bonus ~3.5x
}

// IMPROVEMENT: SEVERE penalty for open space
if (unknown_in_open > unknown_near_obstacles * 2) {  // Stricter threshold
    structure_bonus *= 0.02;  // 98% penalty
    RCLCPP_DEBUG(logger_, "Heavy penalty at (%.2f, %.2f): open=%d vs struct=%d", 
                  position.x, position.y, unknown_in_open, unknown_near_obstacles);
}

double avg_gain = cells_in_range > 0 ? total_info_gain / cells_in_range : 0.0;
double final_gain = avg_gain * structure_bonus;

// IMPROVEMENT: Stricter dead zone detector
if (cells_in_range > 10 && unknown_near_obstacles < 2) {
    RCLCPP_DEBUG(logger_, "Dead zone at (%.2f, %.2f): only %d obstacles nearby", 
                  position.x, position.y, unknown_near_obstacles);
    return 0.0;  // Zero out completely
}

// Logging for debugging high-value points
if (final_gain > 0.5) {
    RCLCPP_INFO(logger_, 
        "Point (%.2f, %.2f): struct=%d, open=%d, bonus=%.2f, final_gain=%.2f", 
        position.x, position.y, unknown_near_obstacles, unknown_in_open, 
        structure_bonus, final_gain);
}

return final_gain;
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore::EntropyExplorer>());
  rclcpp::shutdown();
  return 0;
}