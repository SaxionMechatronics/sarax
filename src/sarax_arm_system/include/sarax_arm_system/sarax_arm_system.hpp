// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "sarax_arm_system/arm_transmission.hpp"

namespace sarax_arm_system
{
/// Thin arm system: owns a motor-space SystemInterface (hardware parameter `inner_plugin`, e.g.
/// sarax_tmotor_ak60_driver/TmotorAk60System or mock_components/GenericSystem), forwards the
/// lifecycle to it and maps its states and commands through the URDF transmission. The
/// controllers see joint-space interfaces under the joint names. The inner driver is unmodified
/// and knows nothing about the belt. With one joint, or no transmission, the mapping is identity.
class SaraxArmSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  using Callback = hardware_interface::CallbackReturn;
  Callback forward(
    Callback (hardware_interface::SystemInterface::*callback)(const rclcpp_lifecycle::State &),
    uint8_t new_state_id, const char * new_state_label, const rclcpp_lifecycle::State & previous);

  // Declared first so it is destroyed last: the inner object's code lives in the loaded library.
  std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> loader_;
  pluginlib::UniquePtr<hardware_interface::SystemInterface> inner_;
  std::optional<ArmTransmission> transmission_;
  size_t n_{0};

  // Joint-space values exported to controllers (pointers stay valid: sized once).
  AxisValues joint_state_, joint_command_;
  // Handles of the inner driver, in joint order.
  struct InnerHandles
  {
    std::optional<hardware_interface::StateInterface> position, velocity, effort;
    std::optional<hardware_interface::CommandInterface> command;
  };
  std::array<InnerHandles, 2> inner_handles_;
  bool handles_ready_{false};
  void collect_inner_handles();
};
}  // namespace sarax_arm_system
