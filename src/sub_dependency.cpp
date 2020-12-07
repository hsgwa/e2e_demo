#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class SubDependencyNode : public rclcpp::Node {
 public:
  SubDependencyNode():Node("sub_dependency_node") {

    int callback_duration_ns;
    declare_parameter<int>("callback_duration_ns", 100000000);
    get_parameter<int>("callback_duration_ns", callback_duration_ns);
    callback_duration_ = std::chrono::nanoseconds(callback_duration_ns);

    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {

      auto msg_tmp = std::make_unique<sensor_msgs::msg::Image>();
      msg_tmp->header.stamp = msg->header.stamp;
      rclcpp::sleep_for(callback_duration_);
      pub1_->publish(std::move(msg_tmp));

      msg_ = std::move(msg);
    };
    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
        (void) msg;
        rclcpp::sleep_for(callback_duration_);
        if (msg_) {
          pub2_->publish(std::move(msg_));
        }
    };
    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);
    pub1_ = create_publisher<sensor_msgs::msg::Image>("output1", 1);
    pub2_ = create_publisher<sensor_msgs::msg::Image>("output2", 1);
  }

 private:
  std::chrono::nanoseconds callback_duration_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
  sensor_msgs::msg::Image::UniquePtr msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
