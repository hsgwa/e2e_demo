#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ActuatorDummy : public rclcpp::Node {
public:
  ActuatorDummy() : Node("actuator_dummy_node") {
    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {(void)msg;};

    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {(void)msg;};

    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorDummy>());
  rclcpp::shutdown();
  return 0;
}
