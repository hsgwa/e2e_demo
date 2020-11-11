#include <chrono>
#include <memory>

#include "communication_trace/comm_tracer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class SubDependencyNode : public rclcpp::Node {
 public:
  SubDependencyNode():Node("sub_dependency_node") {
    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      sub1_tracer_->publish(msg->header);

      auto msg_tmp = std::make_unique<sensor_msgs::msg::Image>();
      msg_tmp->header.stamp = msg->header.stamp;
      pub1_tracer_->publish(msg->header);
      pub1_->publish(std::move(msg_tmp));

      msg_ = std::move(msg);
    };
    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      sub2_tracer_->publish(msg->header);
      if (msg_) {
        pub2_tracer_->publish(msg_->header);
        pub2_->publish(std::move(msg_));
      }
    };
    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub1_tracer_ = std::make_shared<CommTracer>(get_name(), sub1_->get_topic_name());
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);
    sub2_tracer_ = std::make_shared<CommTracer>(get_name(), sub2_->get_topic_name());
    pub1_ = create_publisher<sensor_msgs::msg::Image>("output1", 1);
    pub1_tracer_ = std::make_shared<CommTracer>(get_name(), pub1_->get_topic_name());
    pub2_ = create_publisher<sensor_msgs::msg::Image>("output2", 1);
    pub2_tracer_ = std::make_shared<CommTracer>(get_name(), pub2_->get_topic_name());
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  std::shared_ptr<CommTracer> pub1_tracer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  std::shared_ptr<CommTracer> pub2_tracer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  std::shared_ptr<CommTracer> sub1_tracer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
  std::shared_ptr<CommTracer> sub2_tracer_;
  sensor_msgs::msg::Image::UniquePtr msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
