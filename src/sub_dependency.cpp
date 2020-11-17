#include <chrono>
#include <memory>

#include "communication_trace/comm_trace.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_trace/message_trace.hpp"

using namespace message_trace;
using namespace communication_trace;

class SubDependencyNode : public rclcpp::Node {
 public:
  SubDependencyNode():Node("sub_dependency_node") {
    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      com_sub1_tracer_->publish(msg->header);
      msg_sub1_tracer_->update(&msg->header);

      auto msg_tmp = std::make_unique<sensor_msgs::msg::Image>();
      msg_tmp->header.stamp = msg->header.stamp;

      msg_pub1_tracer_->update(&msg->header);
      com_pub1_tracer_->publish(msg->header);
      pub1_->publish(std::move(msg_tmp));

      msg_ = std::move(msg);
    };
    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      com_sub2_tracer_->publish(msg->header);
      msg_sub2_tracer_->update(&msg->header);


      if (msg_) {
        msg_pub2_tracer_->update(&msg_->header, &msg->header);
        com_pub2_tracer_->publish(msg_->header);
        pub2_->publish(std::move(msg_));
      }
    };

    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);
    pub1_ = create_publisher<sensor_msgs::msg::Image>("output1", 1);
    pub2_ = create_publisher<sensor_msgs::msg::Image>("output2", 1);

    com_sub1_tracer_ = std::make_shared<CommTrace>(get_name(), sub1_->get_topic_name());
    com_sub2_tracer_ = std::make_shared<CommTrace>(get_name(), sub2_->get_topic_name());
    com_pub1_tracer_ = std::make_shared<CommTrace>(get_name(), pub1_->get_topic_name());
    com_pub2_tracer_ = std::make_shared<CommTrace>(get_name(), pub2_->get_topic_name());

    msg_sub1_tracer_ = std::make_shared<MessageTrace>(get_name(), sub1_->get_topic_name());
    msg_sub2_tracer_ = std::make_shared<MessageTrace>(get_name(), sub2_->get_topic_name());
    msg_pub1_tracer_ = std::make_shared<MessageTrace>(get_name(), pub1_->get_topic_name());
    msg_pub2_tracer_ = std::make_shared<MessageTrace>(get_name(), pub2_->get_topic_name());
  }

  std::shared_ptr<CommTrace> com_pub1_tracer_;
  std::shared_ptr<CommTrace> com_pub2_tracer_;
  std::shared_ptr<CommTrace> com_sub1_tracer_;
  std::shared_ptr<CommTrace> com_sub2_tracer_;

  std::shared_ptr<MessageTrace> msg_pub1_tracer_;
  std::shared_ptr<MessageTrace> msg_pub2_tracer_;
  std::shared_ptr<MessageTrace> msg_sub1_tracer_;
  std::shared_ptr<MessageTrace> msg_sub2_tracer_;

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;

  sensor_msgs::msg::Image::UniquePtr msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<SubDependencyNode>();
  exec.add_node(node);
  exec.add_node(node->com_pub1_tracer_);
  exec.add_node(node->com_sub1_tracer_);
  exec.add_node(node->com_pub2_tracer_);
  exec.add_node(node->com_sub2_tracer_);
  exec.add_node(node->msg_pub1_tracer_);
  exec.add_node(node->msg_sub1_tracer_);
  exec.add_node(node->msg_pub2_tracer_);
  exec.add_node(node->msg_sub2_tracer_);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
