// Straight up copy from PECORE

#ifndef OBSTACLE_DETECTION
#define OBSTACLE_DETECTION

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstacleDetector : public rclcpp::Node
{
private:
    rclcpp::Clock clock;

    // PointCloud Subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pc;
    void callbackPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

    // PointCloud publication
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc;

public:
    ObstacleDetector();
};

#endif // v


ObstacleDetector::ObstacleDetector() : Node("obstacle_detection"), clock(RCL_ROS_TIME)
{
    // Subscribers and Publishers
    this->subscriber_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", 10, std::bind(&ObstacleDetector::callbackPC, this, _1));
    
    this->publisher_pc = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 10);

}

void ObstacleDetector::callbackPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
    RCLCPP_INFO_ONCE(this->get_logger(), "[obstacle_detector]: Started to receive PointClouds!");

    auto cloud_out = sensor_msgs::msg::PointCloud2(*msg);

    for (sensor_msgs::PointCloud2Iterator<float> it(cloud_out, "x"); it != it.end(); ++it) {
        if (it[2]<-0.0)
        {
            it[0] = std::numeric_limits<float>::quiet_NaN();
            it[1] = std::numeric_limits<float>::quiet_NaN();
            it[2] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    this->publisher_pc->publish(cloud_out);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();

    return 0;
}
