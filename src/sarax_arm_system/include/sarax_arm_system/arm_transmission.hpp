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
}  // namespace sarax_arm_system
