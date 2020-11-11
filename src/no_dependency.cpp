#include <chrono>
#include <memory>

#include "communication_trace/comm_tracer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class NoDependencyNode : public rclcpp::Node {
 public:
  NoDependencyNode():Node("no_dependency_node") {
    auto callback = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      sub_tracer_->publish(msg->header);
      pub_tracer_->publish(msg->header);
      pub_->publish(std::move(msg));
    };
    sub_ = create_subscription<sensor_msgs::msg::Image>("input", 1, callback);
    sub_tracer_ = std::make_shared<CommTracer>(get_name(), sub_->get_topic_name());
    pub_ = create_publisher<sensor_msgs::msg::Image>("output", 1);
    pub_tracer_ = std::make_shared<CommTracer>(get_name(), pub_->get_topic_name());
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::shared_ptr<CommTracer> pub_tracer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::shared_ptr<CommTracer> sub_tracer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NoDependencyNode>());
  rclcpp::shutdown();
  return 0;
}
