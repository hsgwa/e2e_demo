#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class SensorDummy : public rclcpp::Node {
public:
  SensorDummy() : Node("sensor_dummy_node") {
    int period1_ns, period2_ns;
    declare_parameter<int>("period1_ns", 1000000000);
    declare_parameter<int>("period2_ns", 1000000000);
    get_parameter<int>("period1_ns", period1_ns);
    get_parameter<int>("period2_ns", period2_ns);
    auto period1 = std::chrono::nanoseconds(period1_ns);
    auto period2 = std::chrono::nanoseconds(period2_ns);

    int callback_duration_ns;
    declare_parameter<int>("callback_duration_ns", 100000000);
    get_parameter<int>("callback_duration_ns", callback_duration_ns);
    callback_duration_ = std::chrono::nanoseconds(callback_duration_ns);

    auto callback1 = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      rclcpp::sleep_for(callback_duration_);
      pub1_->publish(std::move(msg));
    };
    auto callback2 = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      rclcpp::sleep_for(callback_duration_);
      pub2_->publish(std::move(msg));
    };
    pub1_ = create_publisher<sensor_msgs::msg::Image>("input1", 1);
    timer1_ = create_wall_timer(period1, callback1);
    pub2_ = create_publisher<sensor_msgs::msg::Image>("input2", 1);
    timer2_ = create_wall_timer(period2, callback2);
  }

private:
  std::chrono::nanoseconds callback_duration_;
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
