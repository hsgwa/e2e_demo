#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class TimerDependencyNode : public rclcpp::Node {
 public:
  TimerDependencyNode():Node("timer_dependency_node") {
    int period_ns;
    declare_parameter<int>("period_ns", 1000000000);
    get_parameter<int>("period_ns", period_ns);

    int callback_duration_ns;
    declare_parameter<int>("callback_duration_ns", 100000000);
    get_parameter<int>("callback_duration_ns", callback_duration_ns);
    callback_duration_ = std::chrono::nanoseconds(callback_duration_ns);

    auto period = std::chrono::nanoseconds(period_ns);
    auto sub_callback = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      rclcpp::sleep_for(callback_duration_);
      msg_ = std::move(msg);
    };
    auto timer_callback = [&]() {
                            rclcpp::sleep_for(callback_duration_);
                            if (msg_) {
                              pub_->publish(std::move(msg_));
                            }
                          };

    pub_ = create_publisher<sensor_msgs::msg::Image>("output", 1);
    sub_ = create_subscription<sensor_msgs::msg::Image>("input", 1, sub_callback);
    timer_ = create_wall_timer(period, timer_callback);
  }

 private:
  std::chrono::nanoseconds callback_duration_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  sensor_msgs::msg::Image::UniquePtr msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
