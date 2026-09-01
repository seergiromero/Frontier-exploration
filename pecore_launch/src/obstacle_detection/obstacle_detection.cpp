#include "obstacle_detection/obstacle_detection.hpp"

using std::placeholders::_1;

ObstacleDetector::ObstacleDetector()
: Node("obstacle_detection"),
  clock_(RCL_ROS_TIME),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Parameters (you can later expose these as declare_parameter)
  map_frame_ = "map";
  base_frame_ = "b2/base_link";

  // Robot body footprint (for notch, in base_link frame)
  fp_x_fw_ = 0.25;
  fp_x_bw_ = -1.0;
  fp_y_lw_ = 0.25;
  fp_y_rw_ = -fp_y_lw_;
  fp_z_uw_ = 0.8;
  fp_z_dw_ = -0.2;

  // Z passthrough in MAP frame: keep [0.0, robot_height_]
  robot_height_ = 1.0;  // tune this to your robot

  // Subscribers / Publishers
  subscriber_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in", 10, std::bind(&ObstacleDetector::callbackPC, this, _1));

  publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "cloud_out", 10);
}

void ObstacleDetector::callbackPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(
    this->get_logger(),
    "[obstacle_detector]: Started to receive PointClouds!");

  const std::string cloud_frame = msg->header.frame_id;

  // ------------------------------------------------------------------
  // 1) Transform incoming cloud to MAP frame
  // ------------------------------------------------------------------
  sensor_msgs::msg::PointCloud2 cloud_map;
  try {
    auto tf_map_from_sensor = tf_buffer_.lookupTransform(
      map_frame_,         // target
      cloud_frame,        // source
      msg->header.stamp,
      100ms);

    tf2::doTransform(*msg, cloud_map, tf_map_from_sensor);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), clock_, 2000,
      "[obstacle_detector]: Could not transform cloud %s -> %s: %s",
      cloud_frame.c_str(), map_frame_.c_str(), ex.what());
    return;
  }

  // ------------------------------------------------------------------
  // 2) Z passthrough in MAP frame (remove ground below 0 and above robot)
  // ------------------------------------------------------------------
  passThroughZ(cloud_map);

  // ------------------------------------------------------------------
  // 3) Transform filtered cloud to BASE frame and apply notch
  // ------------------------------------------------------------------
  sensor_msgs::msg::PointCloud2 cloud_base;
  bool have_base_cloud = false;

  try {
    // NOTE: target = base, source = map (we want map → base)
    auto tf_base_from_map = tf_buffer_.lookupTransform(
      base_frame_,   // target
      map_frame_,    // source
      msg->header.stamp,
      100ms);

    tf2::doTransform(cloud_map, cloud_base, tf_base_from_map);

    // Apply notch in base_link frame
    notchFootprint(cloud_base);
    have_base_cloud = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), clock_, 2000,
      "[obstacle_detector]: Could not transform map cloud to base frame %s: %s",
      base_frame_.c_str(), ex.what());
    // If we don't have base transform, we skip notch and just use map cloud
  }

  // ------------------------------------------------------------------
  // 4) Choose which cloud to transform back to output frame
  // ------------------------------------------------------------------
  sensor_msgs::msg::PointCloud2 cloud_for_output;
  if (have_base_cloud) {
    cloud_for_output = cloud_base;  // frame = base_link
  } else {
    cloud_for_output = cloud_map;   // frame = map
  }

  // ------------------------------------------------------------------
  // 5) Transform filtered cloud back to original sensor frame (if possible)
  // ------------------------------------------------------------------
  sensor_msgs::msg::PointCloud2 cloud_out;
  try {
    auto tf_sensor_from_current = tf_buffer_.lookupTransform(
      cloud_frame,                      // target
      cloud_for_output.header.frame_id, // source (base_link or map)
      msg->header.stamp,
      100ms);

    tf2::doTransform(cloud_for_output, cloud_out, tf_sensor_from_current);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), clock_, 2000,
      "[obstacle_detector]: Could not transform filtered cloud back to %s: %s. "
      "Publishing in %s frame instead.",
      cloud_frame.c_str(), ex.what(),
      cloud_for_output.header.frame_id.c_str());
    cloud_out = cloud_for_output;  // publish in map/base frame
  }

  publisher_pc_->publish(cloud_out);
}

// ------------------------------------------------------------------
// Z passthrough in MAP frame
// Keeps points with 0.0 <= z <= robot_height_
// ------------------------------------------------------------------
void ObstacleDetector::passThroughZ(sensor_msgs::msg::PointCloud2 & cloud_map)
{
  for (sensor_msgs::PointCloud2Iterator<float> it(cloud_map, "x");
       it != it.end(); ++it)
  {
    const float x = it[0];
    const float y = it[1];
    const float z = it[2];

    if ((!std::isfinite(x) ||
          x < -50.0 ||
          x > 50.0) ||
        (!std::isfinite(y) ||
          y < -50.0 ||
          y > 50.0) ||
        (!std::isfinite(z) ||
          z < 0.1f ||
          z > static_cast<float>(robot_height_)))
    {
      it[0] = std::numeric_limits<float>::quiet_NaN();
      it[1] = std::numeric_limits<float>::quiet_NaN();
      it[2] = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

// ------------------------------------------------------------------
// Notch filtering in BASE_LINK frame
// Removes points inside the box [fp_x_bw_, fp_x_fw_] x [fp_y_rw_, fp_y_lw_] x [fp_z_dw_, fp_z_uw_]
// ------------------------------------------------------------------
void ObstacleDetector::notchFootprint(sensor_msgs::msg::PointCloud2 & cloud_base)
{
  for (sensor_msgs::PointCloud2Iterator<float> it(cloud_base, "x");
       it != it.end(); ++it)
  {
    const float xb = it[0];
    const float yb = it[1];
    const float zb = it[2];

    if (!std::isfinite(xb) || !std::isfinite(yb) || !std::isfinite(zb)) {
      continue;
    }

    const bool inside_xy =
      (fp_x_bw_ < xb && xb < fp_x_fw_) &&
      (fp_y_rw_ < yb && yb < fp_y_lw_);

    const bool inside_z =
      (fp_z_dw_ < zb && zb < fp_z_uw_);

    if (inside_xy && inside_z) {
      it[0] = std::numeric_limits<float>::quiet_NaN();
      it[1] = std::numeric_limits<float>::quiet_NaN();
      it[2] = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
