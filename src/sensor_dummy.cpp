#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class SensorDummy : public rclcpp::Node {
public:
  SensorDummy() : Node("sensor_dummy_node") {
    auto callback = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      pub_->publish(std::move(msg));
    };
    pub_ = create_publisher<sensor_msgs::msg::Image>("input", 1);
    timer_ = create_wall_timer(1s, callback);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDummy>());
  rclcpp::shutdown();
  return 0;
}
