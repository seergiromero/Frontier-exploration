#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

class OctomapPublisher : public rclcpp::Node
{
public:
    OctomapPublisher()
    : Node("octomap_publisher")  // Create node with unique name
    {
        client_ = this->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_full_published", 10);
    }

    // Method to request the octomap from the service
    void request_octomap()
    {
        // Make sure the client is ready before making a request
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service /octomap_binary not available.");
            return;
        }

        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();

        // Call the service
        auto future = client_->async_send_request(request);

        // Wait for the service response
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        if (future.valid()) {
            RCLCPP_INFO(this->get_logger(), "Published OctoMap");
            // Get the octomap message from the service response
            auto octomap = future.get()->map;

            // Publish Octomap for RViz visualization
            octomap_pub_->publish(octomap);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service /octomap_binary");
        }
    }

private:
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Initialize the node once
    auto node = std::make_shared<OctomapPublisher>();

    // Create a Rate object to set the loop frequency (5Hz in this case)
    rclcpp::Rate rate(0.2);  // 5 seconds per loop

    // Keep spinning until the program is terminated
    while (rclcpp::ok()) {
        // Request the octomap and publish it
        node->request_octomap();

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
