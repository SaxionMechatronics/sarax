#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace sarax_torque_poc
{

class TorquePublisher : public rclcpp::Node
{
public:
  TorquePublisher()
  : rclcpp::Node("sarax_torque_publisher")
  {
    const std::string model = declare_parameter<std::string>("model_name", "sarax_plus_0");
    joints_ = declare_parameter<std::vector<std::string>>(
      "joints", std::vector<std::string>{"mani_joint_1", "mani_joint_2"});
    efforts_ = declare_parameter<std::vector<double>>(
      "efforts", std::vector<double>{-10.5, -10.5});
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    effort_limit_ = declare_parameter<double>("effort_limit", 30.0);
    sinusoid_amp_ = declare_parameter<double>("sinusoid_amplitude", 0.0);
    sinusoid_freq_ = declare_parameter<double>("sinusoid_frequency_hz", 0.25);

    if (joints_.size() != efforts_.size()) {
      RCLCPP_FATAL(
        get_logger(),
        "Parameters 'joints' (size %zu) and 'efforts' (size %zu) must have the same length.",
        joints_.size(), efforts_.size());
      throw std::runtime_error("joints/efforts length mismatch");
    }

    publishers_.reserve(joints_.size());
    for (const auto & joint : joints_) {
      const std::string topic = "/model/" + model + "/joint/" + joint + "/cmd_force";
      publishers_.push_back(create_publisher<std_msgs::msg::Float64>(topic, rclcpp::QoS(10)));
      RCLCPP_INFO(get_logger(), "Publishing torque to %s", topic.c_str());
    }

    start_ = now();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->tick(); });
  }

private:
  void tick()
  {
    const double t = (now() - start_).seconds();
    const double modulation = (sinusoid_amp_ > 0.0)
      ? sinusoid_amp_ * std::sin(2.0 * M_PI * sinusoid_freq_ * t)
      : 0.0;

    for (size_t i = 0; i < publishers_.size(); ++i) {
      const double raw = efforts_[i] + modulation;
      const double clamped = std::clamp(raw, -effort_limit_, effort_limit_);
      std_msgs::msg::Float64 msg;
      msg.data = clamped;
      publishers_[i]->publish(msg);
    }
  }

  std::vector<std::string> joints_;
  std::vector<double> efforts_;
  double effort_limit_{30.0};
  double sinusoid_amp_{0.0};
  double sinusoid_freq_{0.25};
  rclcpp::Time start_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
};

}  // namespace sarax_torque_poc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sarax_torque_poc::TorquePublisher>());
  rclcpp::shutdown();
  return 0;
}
