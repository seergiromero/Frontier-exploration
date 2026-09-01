#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovInjector : public rclcpp::Node {
public:
  ImuCovInjector() : Node("imu_cov_injector") {
    in_topic_  = declare_parameter<std::string>("in_topic", "/b2/imu/raw");
    out_topic_ = declare_parameter<std::string>("out_topic", "/b2/imu/data");
    double ori_std = declare_parameter<double>("ori_std", 2.0*M_PI/180.0);
    double gyr_std = declare_parameter<double>("gyr_std", 0.02*M_PI/180.0);
    double acc_std = declare_parameter<double>("acc_std", 0.03);

    set_diag(ori_cov_, ori_std*ori_std);
    set_diag(gyr_cov_, gyr_std*gyr_std);
    set_diag(acc_cov_, acc_std*acc_std);

    sub_ = create_subscription<sensor_msgs::msg::Imu>(in_topic_, 50,
      [this](sensor_msgs::msg::Imu::SharedPtr m){
        m->orientation_covariance = ori_cov_;
        m->angular_velocity_covariance = gyr_cov_;
        m->linear_acceleration_covariance = acc_cov_;
        pub_->publish(*m);
      });
    pub_ = create_publisher<sensor_msgs::msg::Imu>(out_topic_, 50);
  }
private:
  void set_diag(std::array<double,9> &cov, double var){
    cov = {var,0,0, 0,var,0, 0,0,var};
  }
  std::string in_topic_, out_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::array<double,9> ori_cov_, gyr_cov_, acc_cov_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovInjector>());
  rclcpp::shutdown();
  return 0;
}
