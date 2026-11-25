// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <memory>
#include <hardware_interface/hardware_info.hpp>

namespace sarax_arm_system
{
/// Per-axis position [rad], velocity [rad/s] and effort [N*m] of the two arm axes.
struct AxisValues
{
  std::array<double, 2> position{0.0, 0.0};
  std::array<double, 2> velocity{0.0, 0.0};
  std::array<double, 2> effort{0.0, 0.0};
};

/// Motor <-> joint mapping of the two-axis arm, built from the URDF <transmission>.
/// ros2_control (Humble) only parses <transmission> into HardwareInfo::transmissions and never
/// applies it, so this class loads a transmission_interface::FourBarLinkageTransmission from that
/// info and owns its handles. Actuator 1 -> joint 1; actuator 2 -> joint 2 relative to the base
/// (joint2 = a2 / n_a2 - a1 / n_a1 at unit joint reductions). Negative reductions flip direction.
class ArmTransmission
{
public:
  /// Throws std::invalid_argument / transmission_interface::Exception on an unsupported layout.
  explicit ArmTransmission(const hardware_interface::TransmissionInfo & info);
  ~ArmTransmission();
  ArmTransmission(const ArmTransmission &) = delete;
  ArmTransmission & operator=(const ArmTransmission &) = delete;

  /// Motor-space measurements -> joint-space measurements.
  AxisValues to_joint(const AxisValues & actuator);
  /// Joint-space commands -> motor-space commands.
  AxisValues to_actuator(const AxisValues & joint);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Per-actuator effort limit [N*m] of the first `n` joints, from the effort command interface
/// `min`/`max` of each joint in HardwareInfo (the URDF `effort_limit`, the one source of the
/// torque limit; Humble does not carry the joint `<limit effort>` itself into HardwareInfo).
/// Joint i is driven by actuator i. Throws std::invalid_argument when a limit is missing,
/// not finite, not greater than zero, or not symmetric (min != -max).
std::array<double, 2> effort_limits_from_info(const hardware_interface::HardwareInfo & info, size_t n);

/// Clamps the actuator-space effort symmetrically to [-limits, +limits] (first `n` axes).
void clamp_actuator_effort(AxisValues & actuator, const std::array<double, 2> & limits, size_t n);
}  // namespace sarax_arm_system
