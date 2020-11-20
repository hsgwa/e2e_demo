#include <chrono>
#include <memory>

#include "communication_trace/comm_trace.hpp"
#include "message_trace/message_trace.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

using namespace message_trace;
using namespace communication_trace;

class ActuatorDummyNode : public rclcpp::Node {
public:
  ActuatorDummyNode() : Node("actuator_dummy_node") {
    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      com_sub1_tracer_->publish(msg->header);
      msg_sub1_tracer_->update(&msg->header);

      rclcpp::sleep_for(100ms);
      pub1_->publish(msg->header.stamp);
    };

    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      com_sub2_tracer_->publish(msg->header);
      msg_sub2_tracer_->update(&msg->header);

      rclcpp::sleep_for(100ms);

    };

    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);

    com_sub1_tracer_ = std::make_shared<CommTrace>(get_name(), sub1_->get_topic_name());
    com_sub2_tracer_ = std::make_shared<CommTrace>(get_name(), sub2_->get_topic_name());

    msg_sub1_tracer_ = std::make_shared<MessageTrace>(get_name(), sub1_->get_topic_name());
    msg_sub2_tracer_ = std::make_shared<MessageTrace>(get_name(), sub2_->get_topic_name());
  }

  std::shared_ptr<CommTrace> com_sub1_tracer_;
  std::shared_ptr<CommTrace> com_sub2_tracer_;

  std::shared_ptr<MessageTrace> msg_sub1_tracer_;
  std::shared_ptr<MessageTrace> msg_sub2_tracer_;

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<ActuatorDummyNode>();
  exec.add_node(node);
  exec.add_node(node->com_sub1_tracer_);
  exec.add_node(node->com_sub2_tracer_);
  exec.add_node(node->msg_sub1_tracer_);
  exec.add_node(node->msg_sub2_tracer_);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
