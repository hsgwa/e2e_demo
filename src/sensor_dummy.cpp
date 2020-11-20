#include <chrono>
#include <memory>

#include "communication_trace/comm_trace.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_trace/message_trace.hpp"

using namespace std::chrono_literals;
using namespace message_trace;
using namespace communication_trace;

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

      msg_pub1_tracer_->update(&msg->header);
      com_pub1_tracer_->publish(msg->header);
      pub1_->publish(std::move(msg));
    };

    auto callback2 = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();

      rclcpp::sleep_for(100ms);

      msg_pub2_tracer_->update(&msg->header);
      com_pub2_tracer_->publish(msg->header);
      pub2_->publish(std::move(msg));
    };

    pub1_ = create_publisher<sensor_msgs::msg::Image>("input1", 1);
    pub2_ = create_publisher<sensor_msgs::msg::Image>("input2", 1);

    timer1_ = create_wall_timer(period, callback1);
    timer2_ = create_wall_timer(period, callback2);

    com_pub1_tracer_ = std::make_shared<CommTrace>(get_name(), pub1_->get_topic_name());
    com_pub2_tracer_ = std::make_shared<CommTrace>(get_name(), pub2_->get_topic_name());

    msg_pub1_tracer_ = std::make_shared<MessageTrace>(get_name(), pub1_->get_topic_name());
    msg_pub2_tracer_ = std::make_shared<MessageTrace>(get_name(), pub2_->get_topic_name());
  }

  std::shared_ptr<CommTrace> com_pub1_tracer_;
  std::shared_ptr<CommTrace> com_pub2_tracer_;

  std::shared_ptr<MessageTrace> msg_pub1_tracer_;
  std::shared_ptr<MessageTrace> msg_pub2_tracer_;

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  rclcpp::TimerBase::SharedPtr timer1_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr timer2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<SensorDummy>();
  exec.add_node(node);

  // TODO: clean these procedure
  exec.add_node(node->com_pub1_tracer_);
  exec.add_node(node->com_pub2_tracer_);
  exec.add_node(node->msg_pub1_tracer_);
  exec.add_node(node->msg_pub2_tracer_);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

