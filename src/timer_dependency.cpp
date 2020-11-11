#include <chrono>
#include <memory>

#include "communication_trace/comm_tracer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class TimerDependencyNode : public rclcpp::Node {
 public:
  TimerDependencyNode():Node("timer_dependency_node") {
    int period_ms;
    declare_parameter<int>("period_ms", 1000);
    get_parameter<int>("period_ms", period_ms);

    auto period = std::chrono::milliseconds(period_ms);
    auto sub_callback = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      sub_tracer_->publish(msg->header);
      msg_ = std::move(msg);
    };
    auto timer_callback = [&]() {
      if (msg_) {
        pub_tracer_->publish(msg_->header);
        pub_->publish(std::move(msg_));
      }
    };

    pub_ = create_publisher<sensor_msgs::msg::Image>("output", 1);
    pub_tracer_ = std::make_shared<CommTracer>(get_name(), pub_->get_topic_name());
    sub_ = create_subscription<sensor_msgs::msg::Image>("input", 1, sub_callback);
    sub_tracer_ = std::make_shared<CommTracer>(get_name(), sub_->get_topic_name());
    timer_ = create_wall_timer(period, timer_callback);
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::shared_ptr<CommTracer> pub_tracer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::shared_ptr<CommTracer> sub_tracer_;
  sensor_msgs::msg::Image::UniquePtr msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
