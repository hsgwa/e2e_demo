#include <chrono>
#include <memory>

#include "communication_trace/comm_trace.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_trace/message_trace.hpp"

using namespace std::chrono_literals;
using namespace message_trace;
using namespace communication_trace;

class TimerDependencyNode : public rclcpp::Node {
 public:
  TimerDependencyNode():Node("timer_dependency_node") {
    int period_ms;
    declare_parameter<int>("period_ms", 1000);
    get_parameter<int>("period_ms", period_ms);

    auto period = std::chrono::milliseconds(period_ms);
    auto sub_callback = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      com_sub_tracer_->publish(msg->header);
      msg_sub_tracer_->update(&msg->header);

      rclcpp::sleep_for(100ms);

      msg_ = std::move(msg);
    };
    auto timer_callback = [&]() {
      rclcpp::sleep_for(100ms);

      if (msg_) {
        com_pub_tracer_->publish(msg_->header);
        msg_pub_tracer_->update(&msg_->header);
        pub_->publish(std::move(msg_));
      }
    };

    timer_ = create_wall_timer(period, timer_callback);

    pub_ = create_publisher<sensor_msgs::msg::Image>("output", 1);
    sub_ = create_subscription<sensor_msgs::msg::Image>("input", 1, sub_callback);

    com_pub_tracer_ = std::make_shared<CommTrace>(get_name(), pub_->get_topic_name());
    com_sub_tracer_ = std::make_shared<CommTrace>(get_name(), sub_->get_topic_name());

    msg_pub_tracer_ = std::make_shared<MessageTrace>(get_name(), pub_->get_topic_name());
    msg_sub_tracer_ = std::make_shared<MessageTrace>(get_name(), sub_->get_topic_name());
  }

  std::shared_ptr<MessageTrace> msg_sub_tracer_;
  std::shared_ptr<MessageTrace> msg_pub_tracer_;

  std::shared_ptr<CommTrace> com_pub_tracer_;
  std::shared_ptr<CommTrace> com_sub_tracer_;

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  sensor_msgs::msg::Image::UniquePtr msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<TimerDependencyNode>();
  exec.add_node(node);

  exec.add_node(node->com_sub_tracer_);
  exec.add_node(node->com_pub_tracer_);
  exec.add_node(node->msg_pub_tracer_);
  exec.add_node(node->msg_sub_tracer_);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
