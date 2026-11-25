// The torque limit comes from the URDF <limit effort> and clamps the command symmetrically.
#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

#include "sarax_impedance_controller/effort_limits.hpp"

using sarax_impedance_controller::clamp_effort;
using sarax_impedance_controller::effort_limits_from_urdf;
using sarax_impedance_controller::motion_limits_from_urdf;

namespace
{
const char * kUrdf = R"(<robot name="r"><link name="a"/><link name="b"/><link name="c"/>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="5.0" velocity="3"/></joint>
  <joint name="j2" type="revolute"><parent link="b"/><child link="c"/><axis xyz="1 0 0"/>
    <limit lower="-1.9" upper="2.6" effort="3.5" velocity="1"/></joint></robot>)";
}  // namespace

TEST(EffortLimits, ReadFromUrdfInJointOrder)
{
  Eigen::VectorXd limits;
  std::string error;
  ASSERT_TRUE(effort_limits_from_urdf(kUrdf, {"j2", "j1"}, limits, error)) << error;
  EXPECT_DOUBLE_EQ(limits[0], 3.5);
  EXPECT_DOUBLE_EQ(limits[1], 5.0);
}

TEST(EffortLimits, UnknownJointOrBadUrdfFails)
{
  Eigen::VectorXd limits;
  std::string error;
  EXPECT_FALSE(effort_limits_from_urdf(kUrdf, {"nope"}, limits, error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(effort_limits_from_urdf("", {"j1"}, limits, error));
  EXPECT_FALSE(effort_limits_from_urdf("<robot", {"j1"}, limits, error));
}

TEST(MotionLimits, ReadFromUrdfInJointOrder)
{
  Eigen::VectorXd lo, hi, vel;
  std::string error;
  ASSERT_TRUE(motion_limits_from_urdf(kUrdf, {"j2", "j1"}, lo, hi, vel, error)) << error;
  EXPECT_DOUBLE_EQ(lo[0], -1.9);
  EXPECT_DOUBLE_EQ(hi[0], 2.6);
  EXPECT_DOUBLE_EQ(vel[0], 1.0);
  EXPECT_DOUBLE_EQ(vel[1], 3.0);
}

TEST(MotionLimits, MissingJointOrBadVelocityFails)
{
  Eigen::VectorXd lo, hi, vel;
  std::string error;
  EXPECT_FALSE(motion_limits_from_urdf(kUrdf, {"nope"}, lo, hi, vel, error));
  EXPECT_FALSE(motion_limits_from_urdf("", {"j1"}, lo, hi, vel, error));
  const std::string zero_velocity =
    R"(<robot name="r"><link name="a"/><link name="b"/><joint name="j" type="revolute">
    <parent link="a"/><child link="b"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="5" velocity="0"/></joint></robot>)";
  EXPECT_FALSE(motion_limits_from_urdf(zero_velocity, {"j"}, lo, hi, vel, error));
  EXPECT_NE(error.find("velocity"), std::string::npos);
}

TEST(EffortLimits, ClampIsSymmetric)
{
  Eigen::VectorXd limits(2);
  limits << 5.0, 5.0;
  Eigen::VectorXd tau(2);
  tau << 9.0, -9.0;
  clamp_effort(tau, limits);
  EXPECT_DOUBLE_EQ(tau[0], 5.0);
  EXPECT_DOUBLE_EQ(tau[1], -5.0);
}

TEST(EffortLimits, ClampLeavesTorquesInsideLimitUntouched)
{
  Eigen::VectorXd limits(2);
  limits << 5.0, 5.0;
  Eigen::VectorXd tau(2);
  tau << 4.99, -0.3;
  clamp_effort(tau, limits);
  EXPECT_DOUBLE_EQ(tau[0], 4.99);
  EXPECT_DOUBLE_EQ(tau[1], -0.3);
}

#ifdef TEST_URDF_PATH
TEST(EffortLimits, ShippedUrdfHasFiveNewtonMetresAndRos1SecondJointLimits)
{
  std::ifstream file(TEST_URDF_PATH);
  std::stringstream ss;
  ss << file.rdbuf();
  Eigen::VectorXd limits;
  std::string error;
  ASSERT_TRUE(effort_limits_from_urdf(ss.str(), {"mani_joint_1", "mani_joint_2"}, limits, error))
    << error;
  EXPECT_DOUBLE_EQ(limits[0], 5.0);
  EXPECT_DOUBLE_EQ(limits[1], 5.0);
}
#endif
