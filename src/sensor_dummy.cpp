#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class SensorDummy : public rclcpp::Node {
public:
  SensorDummy() : Node("sensor_dummy_node") {
    int period_ms;
    declare_parameter<int>("period_ms", 1000);
    get_parameter<int>("period_ms", period_ms);
    auto period = std::chrono::milliseconds(period_ms);

    auto callback1 = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      rclcpp::sleep_for(100ms);
      pub1_->publish(std::move(msg));
    };
    auto callback2 = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      rclcpp::sleep_for(100ms);
      pub2_->publish(std::move(msg));
    };
    pub1_ = create_publisher<sensor_msgs::msg::Image>("input1", 1);
    timer1_ = create_wall_timer(period, callback1);
    pub2_ = create_publisher<sensor_msgs::msg::Image>("input2", 1);
    timer2_ = create_wall_timer(period, callback2);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr timer2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDummy>());
  rclcpp::shutdown();
  return 0;
}
