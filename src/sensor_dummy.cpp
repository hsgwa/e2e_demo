#include <chrono>
#include <memory>

#include "communication_trace/comm_tracer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

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

      pub1_tracer_->publish(msg->header);
      pub1_->publish(std::move(msg));
    };

    auto callback2 = [&]() {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();

      auto msg_trace = std::make_unique<communication_trace_msgs::msg::CommunicationTrace>();
      msg_trace->header.stamp = msg->header.stamp;
      msg_trace->stamp = now();

      pub2_tracer_->publish(msg->header);
      pub2_->publish(std::move(msg));
    };

    pub1_ = create_publisher<sensor_msgs::msg::Image>("input1", 1);
    pub1_tracer_ = std::make_shared<CommTracer>(get_name(), pub1_->get_topic_name(), "_pub");
    timer1_ = create_wall_timer(period, callback1);

    pub2_ = create_publisher<sensor_msgs::msg::Image>("input2", 1);
    pub2_tracer_ = std::make_shared<CommTracer>(get_name(), pub2_->get_topic_name(), "_pub");
    timer2_ = create_wall_timer(period, callback2);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub1_;
  std::shared_ptr<CommTracer> pub1_tracer_;
  rclcpp::TimerBase::SharedPtr timer1_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub2_;
  std::shared_ptr<CommTracer> pub2_tracer_;
  rclcpp::TimerBase::SharedPtr timer2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDummy>());
  rclcpp::shutdown();
  return 0;
}
