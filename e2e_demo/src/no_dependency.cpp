#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class NoDependencyNode : public rclcpp::Node {
 public:
  NoDependencyNode():Node("no_dependency_node") {

    int callback_duration_ns;
    declare_parameter<int>("callback_duration_ns", 100000000);
    get_parameter<int>("callback_duration_ns", callback_duration_ns);
    callback_duration_ = std::chrono::nanoseconds(callback_duration_ns);

    auto callback =
        [&](sensor_msgs::msg::Image::UniquePtr msg) {
          rclcpp::sleep_for(callback_duration_);
          pub_->publish(std::move(msg));
        };
    sub_ = create_subscription<sensor_msgs::msg::Image>("input", 1, callback);
    pub_ = create_publisher<sensor_msgs::msg::Image>("output", 1);
  }

 private:
  std::chrono::nanoseconds callback_duration_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
