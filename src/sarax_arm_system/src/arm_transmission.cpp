// SPDX-License-Identifier: BSD-3-Clause
#include "sarax_arm_system/arm_transmission.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
// Humble's transmission headers do not include these themselves.
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <transmission_interface/four_bar_linkage_transmission.hpp>
#include <transmission_interface/four_bar_linkage_transmission_loader.hpp>

namespace sarax_arm_system
{
namespace ti = transmission_interface;

struct ArmTransmission::Impl
{
  AxisValues joint, actuator;  // Storage the handles point into.
  std::shared_ptr<ti::Transmission> transmission;
};

ArmTransmission::ArmTransmission(const hardware_interface::TransmissionInfo & info)
: impl_(std::make_unique<Impl>())
{
  if (info.type != "transmission_interface/FourBarLinkageTransmission") {
    throw std::invalid_argument("Unsupported transmission type '" + info.type + "'");
  }
  if (info.joints.size() != 2 || info.actuators.size() != 2) {
    throw std::invalid_argument("Transmission '" + info.name + "' needs two joints and two actuators");
  }
  impl_->transmission = ti::FourBarLinkageTransmissionLoader().load(info);
  if (!impl_->transmission) {  // The loader logs the reason and returns null.
    throw std::invalid_argument("Cannot load transmission '" + info.name + "'");
  }

  std::vector<ti::JointHandle> joints;
  std::vector<ti::ActuatorHandle> actuators;
  for (size_t i = 0; i < 2; ++i) {
    joints.emplace_back(info.joints[i].name, "position", &impl_->joint.position[i]);
    joints.emplace_back(info.joints[i].name, "velocity", &impl_->joint.velocity[i]);
    joints.emplace_back(info.joints[i].name, "effort", &impl_->joint.effort[i]);
    actuators.emplace_back(info.actuators[i].name, "position", &impl_->actuator.position[i]);
    actuators.emplace_back(info.actuators[i].name, "velocity", &impl_->actuator.velocity[i]);
    actuators.emplace_back(info.actuators[i].name, "effort", &impl_->actuator.effort[i]);
  }
  impl_->transmission->configure(joints, actuators);
}

ArmTransmission::~ArmTransmission() = default;

AxisValues ArmTransmission::to_joint(const AxisValues & actuator)
{
  impl_->actuator = actuator;
  impl_->transmission->actuator_to_joint();
  return impl_->joint;
}

AxisValues ArmTransmission::to_actuator(const AxisValues & joint)
{
  impl_->joint = joint;
  impl_->transmission->joint_to_actuator();
  return impl_->actuator;
}

std::array<double, 2> effort_limits_from_info(const hardware_interface::HardwareInfo & info, size_t n)
{
  std::array<double, 2> limits{0.0, 0.0};
  for (size_t i = 0; i < n && i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];
    const hardware_interface::InterfaceInfo * effort = nullptr;
    for (const auto & command : joint.command_interfaces) {
      if (command.name == "effort") {effort = &command;}
    }
    const auto fail = [&joint](const std::string & what) {
        return std::invalid_argument("Joint '" + joint.name + "': " + what);
      };
    if (!effort || effort->min.empty() || effort->max.empty()) {
      throw fail("the effort command interface needs min and max (the URDF effort_limit)");
    }
    double lo = 0.0, hi = 0.0;
    try {
      lo = std::stod(effort->min);
      hi = std::stod(effort->max);
    } catch (const std::exception &) {
      throw fail("effort min/max are not numbers");
    }
    if (!std::isfinite(lo) || !std::isfinite(hi) || hi <= 0.0) {
      throw fail("effort limit must be finite and greater than zero");
    }
    if (std::abs(lo + hi) > 1e-9 * hi) {throw fail("effort limit must be symmetric (min = -max)");}
    limits[i] = hi;
  }
  return limits;
}

void clamp_actuator_effort(AxisValues & actuator, const std::array<double, 2> & limits, size_t n)
{
  for (size_t i = 0; i < n && i < 2; ++i) {
    double & tau = actuator.effort[i];
    tau = std::isnan(tau) ? 0.0 : std::clamp(tau, -limits[i], limits[i]);
  }
}
}  // namespace sarax_arm_system
