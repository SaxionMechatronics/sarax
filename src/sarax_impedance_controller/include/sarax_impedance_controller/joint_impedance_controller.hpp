// Joint-space impedance controller for ros2_control.
//
// The actual control law is pluggable: the concrete ImpedanceLaw is loaded at
// configure time via pluginlib, selected by the `impedance_law` parameter.
// This keeps new laws additive -- downstream packages register their own law
// against the sarax_impedance_controller/ImpedanceLaw plugin base and drop it
// into the `impedance_law` parameter.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>

#include <sarax_msgs/msg/impedance_gains.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "sarax_impedance_controller/impedance_law.hpp"

namespace sarax_impedance_controller
{

class JointImpedanceController : public controller_interface::ControllerInterface
{
public:
  JointImpedanceController();
  ~JointImpedanceController() override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::vector<std::string> joint_names_;
  Eigen::VectorXd stiffness_;
  Eigen::VectorXd damping_;
  Eigen::VectorXd effort_limits_;
  double          max_position_error_{std::numeric_limits<double>::infinity()};
  std::string     impedance_law_name_;
  std::string     robot_description_;

  size_t n_joints_{0};

  // Pluggable law
  std::unique_ptr<pluginlib::ClassLoader<ImpedanceLaw>> law_loader_;
  ImpedanceLawPtr law_;

  // Reference
  using JointPoint = trajectory_msgs::msg::JointTrajectoryPoint;
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointPoint>> reference_buffer_;
  rclcpp::Subscription<JointPoint>::SharedPtr reference_sub_;

  // Live gain override
  using GainsMsg = sarax_msgs::msg::ImpedanceGains;
  realtime_tools::RealtimeBuffer<std::shared_ptr<GainsMsg>> gains_buffer_;
  rclcpp::Subscription<GainsMsg>::SharedPtr gains_sub_;

  // Working buffers to avoid allocation in update().
  Eigen::VectorXd q_, q_dot_;
  Eigen::VectorXd q_d_, q_dot_d_;
  Eigen::VectorXd tau_;
};

}  // namespace sarax_impedance_controller
