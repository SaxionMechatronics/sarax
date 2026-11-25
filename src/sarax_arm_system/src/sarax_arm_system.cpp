// SPDX-License-Identifier: BSD-3-Clause
#include "sarax_arm_system/sarax_arm_system.hpp"

#include <cmath>
#include <stdexcept>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace sarax_arm_system
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using lifecycle_msgs::msg::State;
static const auto logger = rclcpp::get_logger("sarax_arm_system");

CallbackReturn SaraxArmSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  try {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {return CallbackReturn::ERROR;}
    n_ = info.joints.size();
    if (n_ < 1 || n_ > 2) {throw std::invalid_argument("Configure one or two joints");}
    for (const auto & joint : info.joints) {
      if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "effort") {
        throw std::invalid_argument("Joint '" + joint.name + "': only an effort command is supported");
      }
    }
    // Humble parses <transmission> into HardwareInfo but does not apply it: load it here.
    if (n_ == 2) {
      if (info.transmissions.size() != 1) {
        throw std::invalid_argument("Two joints need exactly one <transmission> in the URDF");
      }
      const auto & t = info.transmissions[0];
      if (t.joints.size() != 2 || t.joints[0].name != info.joints[0].name ||
        t.joints[1].name != info.joints[1].name)
      {
        throw std::invalid_argument("<transmission> joints must be the hardware joints, in order");
      }
      transmission_.emplace(t);
    }
    const auto plugin = info.hardware_parameters.at("inner_plugin");
    loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
      "hardware_interface", "hardware_interface::SystemInterface");
    inner_ = loader_->createUniqueInstance(plugin);
    inner_->set_state(rclcpp_lifecycle::State(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"));
    return inner_->on_init(info);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

CallbackReturn SaraxArmSystem::forward(
  Callback (hardware_interface::SystemInterface::*callback)(const rclcpp_lifecycle::State &),
  uint8_t new_state_id, const char * new_state_label, const rclcpp_lifecycle::State & previous)
{
  if (!inner_) {return CallbackReturn::ERROR;}
  const auto result = ((*inner_).*callback)(previous);
  if (result == CallbackReturn::SUCCESS) {
    // The driver's own services read its lifecycle state, so keep it in step with ours.
    inner_->set_state(rclcpp_lifecycle::State(new_state_id, new_state_label));
  }
  return result;
}

CallbackReturn SaraxArmSystem::on_configure(const rclcpp_lifecycle::State & s)
{
  return forward(&hardware_interface::SystemInterface::on_configure,
           State::PRIMARY_STATE_INACTIVE, "inactive", s);
}
CallbackReturn SaraxArmSystem::on_activate(const rclcpp_lifecycle::State & s)
{
  return forward(&hardware_interface::SystemInterface::on_activate,
           State::PRIMARY_STATE_ACTIVE, "active", s);
}
CallbackReturn SaraxArmSystem::on_deactivate(const rclcpp_lifecycle::State & s)
{
  return forward(&hardware_interface::SystemInterface::on_deactivate,
           State::PRIMARY_STATE_INACTIVE, "inactive", s);
}
CallbackReturn SaraxArmSystem::on_cleanup(const rclcpp_lifecycle::State & s)
{
  return forward(&hardware_interface::SystemInterface::on_cleanup,
           State::PRIMARY_STATE_UNCONFIGURED, "unconfigured", s);
}
CallbackReturn SaraxArmSystem::on_shutdown(const rclcpp_lifecycle::State & s)
{
  return forward(&hardware_interface::SystemInterface::on_shutdown,
           State::PRIMARY_STATE_FINALIZED, "finalized", s);
}
CallbackReturn SaraxArmSystem::on_error(const rclcpp_lifecycle::State & s)
{
  return forward(&hardware_interface::SystemInterface::on_error,
           State::PRIMARY_STATE_UNCONFIGURED, "unconfigured", s);
}

void SaraxArmSystem::collect_inner_handles()
{
  if (handles_ready_) {return;}
  for (auto & state : inner_->export_state_interfaces()) {
    for (size_t i = 0; i < n_; ++i) {
      if (state.get_prefix_name() != info_.joints[i].name) {continue;}
      const auto name = state.get_interface_name();
      if (name == "position") {inner_handles_[i].position.emplace(std::move(state));}
      else if (name == "velocity") {inner_handles_[i].velocity.emplace(std::move(state));}
      else if (name == "effort") {inner_handles_[i].effort.emplace(std::move(state));}
      break;
    }
  }
  for (auto & command : inner_->export_command_interfaces()) {
    for (size_t i = 0; i < n_; ++i) {
      if (command.get_prefix_name() == info_.joints[i].name &&
        command.get_interface_name() == "effort")
      {
        inner_handles_[i].command.emplace(std::move(command));
        break;
      }
    }
  }
  handles_ready_ = true;
}

std::vector<hardware_interface::StateInterface> SaraxArmSystem::export_state_interfaces()
{
  collect_inner_handles();
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < n_; ++i) {
    for (const auto & state : info_.joints[i].state_interfaces) {
      double * value = state.name == "position" ? &joint_state_.position[i] :
        state.name == "velocity" ? &joint_state_.velocity[i] :
        state.name == "effort" ? &joint_state_.effort[i] : nullptr;
      if (value) {result.emplace_back(info_.joints[i].name, state.name, value);}
    }
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> SaraxArmSystem::export_command_interfaces()
{
  collect_inner_handles();
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < n_; ++i) {
    result.emplace_back(info_.joints[i].name, "effort", &joint_command_.effort[i]);
  }
  return result;
}

return_type SaraxArmSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto result = inner_->read(time, period);
  if (result != return_type::OK) {return result;}
  AxisValues actuator;
  const auto get = [](const auto & handle) {return handle ? handle->get_value() : std::nan("");};
  for (size_t i = 0; i < n_; ++i) {
    actuator.position[i] = get(inner_handles_[i].position);
    actuator.velocity[i] = get(inner_handles_[i].velocity);
    actuator.effort[i] = get(inner_handles_[i].effort);
  }
  joint_state_ = transmission_ ? transmission_->to_joint(actuator) : actuator;
  return return_type::OK;
}

return_type SaraxArmSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const AxisValues actuator = transmission_ ? transmission_->to_actuator(joint_command_) : joint_command_;
  for (size_t i = 0; i < n_; ++i) {
    if (inner_handles_[i].command) {inner_handles_[i].command->set_value(actuator.effort[i]);}
  }
  return inner_->write(time, period);
}
}  // namespace sarax_arm_system

PLUGINLIB_EXPORT_CLASS(sarax_arm_system::SaraxArmSystem, hardware_interface::SystemInterface)
