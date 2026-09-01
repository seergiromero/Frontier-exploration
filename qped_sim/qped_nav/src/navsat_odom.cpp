#include <chrono>
#include <string.h>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class navsatOdom : public rclcpp::Node
{
public:
    navsatOdom();

private:
    void timerCallback();
    tf2::Transform msgPoseToTf2Transform(const geometry_msgs::msg::Pose &msg);
    tf2::Transform msgTransformToTf2Transform(const geometry_msgs::msg::Transform &msg);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

navsatOdom::navsatOdom() : Node("navsat_odom"), tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer))
{
    // Odometry publisher
    this->publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps", 10);

    // Timer where we will get the transformation and publish the Odometry
    this->timer = this->create_wall_timer(200ms, std::bind(&navsatOdom::timerCallback, this));
}

void navsatOdom::timerCallback()
{
    // INFO
    RCLCPP_INFO_ONCE(this->get_logger(), "[navsat_odom]: Running");

    // Get the transformation from the TF tree
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        transform_stamped = this->tf_buffer->lookupTransform("map", "b2/body", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform 'map' to 'b2/body': %s", ex.what());
        return;
    }

    // Convert the transformation to tf2::Transform
    tf2::Transform world_H_bl = msgTransformToTf2Transform(transform_stamped.transform);

    // Random noise generator
    double sigma = 0.1; // meters
    double min = -3*sigma;
    double max = 3*sigma;
    std::random_device seeder;
    std::mt19937 rng(seeder());
    std::uniform_real_distribution<> dist(min, max); // uniform, unbiased
    
    // Publish the global localization as an odometry message
    nav_msgs::msg::Odometry msg_out;
    msg_out.header.stamp = this->get_clock()->now();
    msg_out.header.frame_id = "map";
    msg_out.child_frame_id = "b2/body";
    msg_out.pose.pose.position.x = world_H_bl.getOrigin().getX() + dist(rng);
    msg_out.pose.pose.position.y = world_H_bl.getOrigin().getY() + dist(rng);
    msg_out.pose.pose.position.z = world_H_bl.getOrigin().getZ();
    // To simplify, let's assume the GPS provides good heading
    msg_out.pose.pose.orientation.x = world_H_bl.getRotation().getX();
    msg_out.pose.pose.orientation.y = world_H_bl.getRotation().getY();
    msg_out.pose.pose.orientation.z = world_H_bl.getRotation().getZ();
    msg_out.pose.pose.orientation.w = world_H_bl.getRotation().getW();
    msg_out.pose.covariance[0] = 100;
    msg_out.pose.covariance[7] = 100;
    msg_out.pose.covariance[14] = 1e-3;
    msg_out.pose.covariance[21] = 1e3;
    msg_out.pose.covariance[28] = 1e3;
    msg_out.pose.covariance[35] = 1e3;

    this->publisher_odom->publish(msg_out);
}

tf2::Transform navsatOdom::msgPoseToTf2Transform(const geometry_msgs::msg::Pose &msg)
{
    tf2::Vector3 tf_trans(msg.position.x, 
                          msg.position.y, 
                          msg.position.z);  
    tf2::Quaternion tf_quat(msg.orientation.x,
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w);
    return tf2::Transform(tf_quat, tf_trans);
}

tf2::Transform navsatOdom::msgTransformToTf2Transform(const geometry_msgs::msg::Transform &msg)
{
    tf2::Vector3 tf_trans(msg.translation.x, 
                          msg.translation.y, 
                          msg.translation.z);  
    tf2::Quaternion tf_quat(msg.rotation.x,
                            msg.rotation.y,
                            msg.rotation.z,
                            msg.rotation.w);
    return tf2::Transform(tf_quat, tf_trans);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<navsatOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}