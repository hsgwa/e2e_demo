#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class SubDependencyNode : public rclcpp::Node {
 public:
  SubDependencyNode():Node("sub_dependency_node") {
    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {

      auto msg_tmp = std::make_unique<sensor_msgs::msg::Image>();
      msg_tmp->header.stamp = msg->header.stamp;
      pub1_->publish(std::move(msg_tmp));

      msg_ = std::move(msg);
    };
    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
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
