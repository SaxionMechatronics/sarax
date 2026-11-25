// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <hardware_interface/hardware_info.hpp>
#include "sarax_arm_system/arm_transmission.hpp"

using sarax_arm_system::ArmTransmission;
using sarax_arm_system::AxisValues;

namespace
{
hardware_interface::TransmissionInfo make_info(double belt_reduction, double j1 = 1.0, double j2 = 1.0)
{
  hardware_interface::TransmissionInfo info;
  info.name = "belt";
  info.type = "transmission_interface/FourBarLinkageTransmission";
  const auto joint = [](const char * name, const char * role, double r) {
      hardware_interface::JointInfo j;
      j.name = name; j.role = role; j.mechanical_reduction = r;
      return j;
    };
  const auto actuator = [](const char * name, const char * role, double r) {
      hardware_interface::ActuatorInfo a;
      a.name = name; a.role = role; a.mechanical_reduction = r;
      return a;
    };
  info.joints = {joint("mani_joint_1", "joint1", j1), joint("mani_joint_2", "joint2", j2)};
  info.actuators = {actuator("mani_motor_1", "actuator1", 1.0),
    actuator("mani_motor_2", "actuator2", belt_reduction)};
  return info;
}

void expect_near(const AxisValues & a, const AxisValues & b)
{
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_NEAR(a.position[i], b.position[i], 1e-12);
    EXPECT_NEAR(a.velocity[i], b.velocity[i], 1e-12);
    EXPECT_NEAR(a.effort[i], b.effort[i], 1e-12);
  }
}

AxisValues sample()
{
  AxisValues v;
  v.position = {0.4, -1.1};
  v.velocity = {0.25, 0.7};
  v.effort = {1.5, -0.8};
  return v;
}
}  // namespace

TEST(ArmTransmission, UnitRatioIsRelativeToBase)
{
  ArmTransmission t(make_info(1.0));
  // Motor 2 is referenced to the base: joint 2 = motor 2 - motor 1 (a serial chain).
  const auto joint = t.to_joint(sample());
  EXPECT_NEAR(joint.position[0], 0.4, 1e-12);
  EXPECT_NEAR(joint.position[1], -1.1 - 0.4, 1e-12);
  EXPECT_NEAR(joint.velocity[1], 0.7 - 0.25, 1e-12);
  // Torque: tau_a2 = tau_j2, tau_a1 = tau_j1 - tau_j2.
  const auto actuator = t.to_actuator(sample());
  EXPECT_NEAR(actuator.effort[1], -0.8, 1e-12);
  EXPECT_NEAR(actuator.effort[0], 1.5 + 0.8, 1e-12);
}

TEST(ArmTransmission, JointToActuatorToJointRoundTrips)
{
  for (const double belt : {1.0, -1.0, 2.5, -2.5, 0.4}) {
    ArmTransmission t(make_info(belt));
    const auto joint = sample();
    const auto actuator = t.to_actuator(joint);
    expect_near(t.to_joint(actuator), joint);
  }
}

TEST(ArmTransmission, NegativeSignFlipsMotor2)
{
  ArmTransmission t(make_info(-1.0));
  AxisValues joint;
  joint.position = {0.0, 0.5};
  joint.velocity = {0.0, 0.2};
  joint.effort = {0.0, 0.3};
  const auto a = t.to_actuator(joint);
  EXPECT_NEAR(a.position[1], -0.5, 1e-12);
  EXPECT_NEAR(a.velocity[1], -0.2, 1e-12);
  EXPECT_NEAR(a.effort[1], -0.3, 1e-12);
}

TEST(ArmTransmission, RatioScalesAnglesAndInverselyTorque)
{
  ArmTransmission t(make_info(2.5));
  AxisValues joint;
  joint.position = {0.0, 0.8};
  joint.velocity = {0.0, 0.4};
  joint.effort = {0.0, 5.0};
  const auto a = t.to_actuator(joint);
  EXPECT_NEAR(a.position[1], 2.5 * 0.8, 1e-12);   // Motor turns 2.5x more than the joint.
  EXPECT_NEAR(a.velocity[1], 2.5 * 0.4, 1e-12);
  EXPECT_NEAR(a.effort[1], 5.0 / 2.5, 1e-12);     // Motor torque is 2.5x smaller.
}

TEST(ArmTransmission, PowerIsConserved)
{
  ArmTransmission t(make_info(-2.5));
  const auto joint = sample();
  const auto a = t.to_actuator(joint);
  const double joint_power = joint.effort[0] * joint.velocity[0] + joint.effort[1] * joint.velocity[1];
  const double motor_power = a.effort[0] * a.velocity[0] + a.effort[1] * a.velocity[1];
  EXPECT_NEAR(joint_power, motor_power, 1e-12);
}

TEST(ArmTransmission, NanPassesThrough)
{
  ArmTransmission t(make_info(1.0));
  AxisValues nan;
  nan.position = nan.velocity = nan.effort = {std::nan(""), std::nan("")};
  EXPECT_TRUE(std::isnan(t.to_joint(nan).position[1]));
}

TEST(ArmTransmission, RejectsWrongLayout)
{
  auto info = make_info(1.0);
  info.type = "transmission_interface/SimpleTransmission";
  EXPECT_THROW(ArmTransmission{info}, std::invalid_argument);
  info = make_info(1.0);
  info.actuators.pop_back();
  EXPECT_THROW(ArmTransmission{info}, std::invalid_argument);
  EXPECT_ANY_THROW(ArmTransmission{make_info(0.0)});
}

namespace
{
hardware_interface::HardwareInfo make_hw(const std::string & min, const std::string & max)
{
  hardware_interface::HardwareInfo info;
  for (const char * name : {"mani_joint_1", "mani_joint_2"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    hardware_interface::InterfaceInfo effort;
    effort.name = "effort";
    effort.min = min;
    effort.max = max;
    joint.command_interfaces.push_back(effort);
    info.joints.push_back(joint);
  }
  return info;
}
}  // namespace

TEST(ActuatorLimit, ReadFromEffortInterfaceOfTheUrdf)
{
  const auto limits = sarax_arm_system::effort_limits_from_info(make_hw("-5.0", " 5.0"), 2);
  EXPECT_DOUBLE_EQ(limits[0], 5.0);
  EXPECT_DOUBLE_EQ(limits[1], 5.0);
}

TEST(ActuatorLimit, MissingOrBadLimitIsRejected)
{
  using sarax_arm_system::effort_limits_from_info;
  EXPECT_THROW(effort_limits_from_info(make_hw("", ""), 2), std::invalid_argument);
  EXPECT_THROW(effort_limits_from_info(make_hw("-5", "x"), 2), std::invalid_argument);
  EXPECT_THROW(effort_limits_from_info(make_hw("0", "0"), 2), std::invalid_argument);
  EXPECT_THROW(effort_limits_from_info(make_hw("-3", "5"), 2), std::invalid_argument);
  hardware_interface::HardwareInfo none;
  none.joints.resize(2);
  EXPECT_THROW(effort_limits_from_info(none, 2), std::invalid_argument);
}

TEST(ActuatorLimit, ClampActsPerMotorAfterTheTransmission)
{
  ArmTransmission t(make_info(1.0));
  const auto limits = sarax_arm_system::effort_limits_from_info(make_hw("-5", "5"), 2);
  // Both joint torques are within 5 N*m, but motor 1 carries tau_j1 - tau_j2 = 8 N*m.
  AxisValues joint;
  joint.effort = {4.0, -4.0};
  auto actuator = t.to_actuator(joint);
  EXPECT_NEAR(actuator.effort[0], 8.0, 1e-12);
  sarax_arm_system::clamp_actuator_effort(actuator, limits, 2);
  EXPECT_NEAR(actuator.effort[0], 5.0, 1e-12);
  EXPECT_NEAR(actuator.effort[1], -4.0, 1e-12);
  // Symmetric, and within-limit values pass unchanged.
  joint.effort = {-4.0, 4.0};
  actuator = t.to_actuator(joint);
  sarax_arm_system::clamp_actuator_effort(actuator, limits, 2);
  EXPECT_NEAR(actuator.effort[0], -5.0, 1e-12);
  EXPECT_NEAR(actuator.effort[1], 4.0, 1e-12);
  joint.effort = {1.0, 2.0};
  actuator = t.to_actuator(joint);
  sarax_arm_system::clamp_actuator_effort(actuator, limits, 2);
  EXPECT_NEAR(actuator.effort[0], -1.0, 1e-12);
  EXPECT_NEAR(actuator.effort[1], 2.0, 1e-12);
}

TEST(ActuatorLimit, NanCommandBecomesZero)
{
  AxisValues a;
  a.effort = {std::nan(""), 3.0};
  sarax_arm_system::clamp_actuator_effort(a, {5.0, 5.0}, 2);
  EXPECT_EQ(a.effort[0], 0.0);
  EXPECT_EQ(a.effort[1], 3.0);
}
