#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class NoDependencyNode : public rclcpp::Node {
 public:
  NoDependencyNode():Node("no_dependency_node") {
    auto callback =
        [&](sensor_msgs::msg::Image::UniquePtr msg) {
          pub_->publish(std::move(msg));
        };
    sub_ = create_subscription<sensor_msgs::msg::Image>("input", 1, callback);
    pub_ = create_publisher<sensor_msgs::msg::Image>("output", 1);
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
