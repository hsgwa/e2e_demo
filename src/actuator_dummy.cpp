#include <chrono>
#include <memory>

#include "communication_trace/comm_tracer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ActuatorDummy : public rclcpp::Node {
public:
  ActuatorDummy() : Node("actuator_dummy_node") {
    auto callback1 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      (void)msg;
      pub1_tracer_->publish(msg->header);
    };

    auto callback2 = [&](sensor_msgs::msg::Image::UniquePtr msg) {
      pub2_tracer_->publish(msg->header);
    };

    sub1_ = create_subscription<sensor_msgs::msg::Image>("input1", 1, callback1);
    sub2_ = create_subscription<sensor_msgs::msg::Image>("input2", 1, callback2);

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  std::shared_ptr<CommTracer> pub1_tracer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
  std::shared_ptr<CommTracer> pub2_tracer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorDummy>());
  rclcpp::shutdown();
  return 0;
}
