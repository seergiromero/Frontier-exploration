#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>

class PointCloudRepublisher : public rclcpp::Node {
public:
    PointCloudRepublisher() : Node("pointcloud_republisher") {
        this->declare_parameter<std::string>("input_topic", "/b2/lidar/points_old");
        this->declare_parameter<std::string>("output_topic", "/b2/lidar/points");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudRepublisher::pointcloud_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        last_callback_time_ = this->now();
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Calculate number of points.
        size_t num_points = msg->width * msg->height;
        // New field: FLOAT32 takes 4 bytes.
        constexpr size_t new_field_size = sizeof(float);
        // New point step: old point_step plus space for "time"
        size_t new_point_step = msg->point_step + new_field_size;

        // Create a new PointCloud2 message.
        sensor_msgs::msg::PointCloud2 new_msg;
        new_msg.header = msg->header;
        new_msg.height = msg->height;
        new_msg.width = msg->width;
        new_msg.is_bigendian = msg->is_bigendian;
        new_msg.is_dense = msg->is_dense;

        // Copy the original fields exactly.
        new_msg.fields = msg->fields;
        // Append the new "time" field.
        sensor_msgs::msg::PointField time_field;
        time_field.name = "time";
        // We want to append it after the original data, so set its offset to the original point_step.
        time_field.offset = msg->point_step;
        time_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        time_field.count = 1;
        new_msg.fields.push_back(time_field);

        // Set new point and row steps.
        new_msg.point_step = new_point_step;
        new_msg.row_step = new_point_step * new_msg.width;

        // Allocate new data array.
        new_msg.data.resize(new_msg.row_step * new_msg.height);

        // Compute time increment based on period.
        auto current_time = this->now();
        auto period = (current_time - last_callback_time_).seconds();
        last_callback_time_ = current_time;
        // float time_value = 0.0f;
        // float time_increment = period / static_cast<float>(msg->width);

        // For each point, copy the original data and write the new "time" field.
        for (size_t i = 0; i < num_points; ++i) {
            // Pointer to the i-th point in the original data.
            const uint8_t* src_ptr = msg->data.data() + i * msg->point_step;
            // Pointer to the i-th point in the new data.
            uint8_t* dst_ptr = new_msg.data.data() + i * new_point_step;

            // Copy the original point data (first msg->point_step bytes).
            memcpy(dst_ptr, src_ptr, msg->point_step);

            // Write the time value into the new field.
            // Since we set the "time" field's offset to msg->point_step,
            // we can write it at dst_ptr + msg->point_step.
            float* time_ptr = reinterpret_cast<float*>(dst_ptr + msg->point_step);
            int h_index = i%static_cast<int>(msg->width);
            *time_ptr = h_index/static_cast<float>(msg->width) * period;

            // time_value += time_increment;
        }

        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Time last_callback_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRepublisher>());
    rclcpp::shutdown();
    return 0;
}
