#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ActuatorDummy : public rclcpp::Node {
public:
  ActuatorDummy() : Node("actuator_dummy_node") {
    int callback_duration_ns;
    declare_parameter<int>("callback_duration_ns", 100000000);
    get_parameter<int>("callback_duration_ns", callback_duration_ns);
    callback_duration_ = std::chrono::nanoseconds(callback_duration_ns);

    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
                       (void)msg;
                       rclcpp::sleep_for(callback_duration_);
                     };

    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
                       (void)msg;
                       rclcpp::sleep_for(callback_duration_);
                     };

    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);

  }

private:
  std::chrono::nanoseconds callback_duration_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorDummy>());
  rclcpp::shutdown();
  return 0;
}
