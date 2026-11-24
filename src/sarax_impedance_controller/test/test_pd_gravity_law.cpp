// Pins the analytic SARAX gravity model (kept until system identification checks the
// URDF): g1 = -g (m2 d2 + m3 L2) sin q1, g2 = -g m3 d3 sin q2.
#include <cmath>

#include <gtest/gtest.h>

#include "sarax_impedance_controller/laws/pd_gravity_law.hpp"

using sarax_impedance_controller::GravityParameters;
using sarax_impedance_controller::ImpedanceInput;
using sarax_impedance_controller::PDGravityLaw;

namespace
{
const std::vector<std::string> kJoints{"mani_joint_1", "mani_joint_2"};

ImpedanceInput at(double q1, double q2)
{
  ImpedanceInput in;
  in.q = Eigen::Vector2d(q1, q2);
  in.q_d = in.q;                 // no PD error: tau is gravity only
  in.q_dot = Eigen::Vector2d::Zero();
  in.q_dot_d = Eigen::Vector2d::Zero();
  in.stiffness = Eigen::Vector2d(10.0, 10.0);
  in.damping = Eigen::Vector2d(1.0, 1.0);
  return in;
}

GravityParameters params()
{
  GravityParameters p;
  p.g = 9.81; p.L2 = 0.46; p.m2 = 0.72; p.d2 = 0.23; p.m3 = 0.3; p.d3 = 0.25;
  return p;
}
}  // namespace

TEST(PDGravityLaw, MatchesAnalyticModel)
{
  PDGravityLaw law;
  const auto p = params();
  ASSERT_TRUE(law.configure_gravity(p));
  ASSERT_TRUE(law.initialize("", kJoints));

  const double q1 = 0.7, q2 = -0.4;
  const auto tau = law.compute(at(q1, q2));
  ASSERT_EQ(tau.size(), 2);
  EXPECT_NEAR(tau[0], -p.g * (p.m2 * p.d2 + p.m3 * p.L2) * std::sin(q1), 1e-12);
  EXPECT_NEAR(tau[1], -p.g * p.m3 * p.d3 * std::sin(q2), 1e-12);
}

TEST(PDGravityLaw, ZeroAtHangingPose)
{
  PDGravityLaw law;
  ASSERT_TRUE(law.configure_gravity(params()));
  ASSERT_TRUE(law.initialize("", kJoints));
  const auto tau = law.compute(at(0.0, 0.0));
  EXPECT_NEAR(tau.norm(), 0.0, 1e-12);
}

TEST(PDGravityLaw, AddsPdTerm)
{
  PDGravityLaw law;
  ASSERT_TRUE(law.configure_gravity(params()));
  ASSERT_TRUE(law.initialize("", kJoints));
  auto in = at(0.0, 0.0);
  in.q_d = Eigen::Vector2d(0.1, -0.2);
  const auto tau = law.compute(in);
  EXPECT_NEAR(tau[0], 10.0 * 0.1, 1e-12);
  EXPECT_NEAR(tau[1], 10.0 * -0.2, 1e-12);
}

TEST(PDGravityLaw, SingleMotorSetup)
{
  PDGravityLaw law;
  ASSERT_TRUE(law.configure_gravity(params()));
  ASSERT_TRUE(law.initialize("", {"mani_joint_1"}));
  ImpedanceInput in;
  in.q = Eigen::VectorXd::Constant(1, 0.5);
  in.q_d = in.q;
  in.q_dot = in.q_dot_d = Eigen::VectorXd::Zero(1);
  in.stiffness = in.damping = Eigen::VectorXd::Ones(1);
  EXPECT_EQ(law.compute(in).size(), 1);
}

TEST(PDGravityLaw, RejectsInvalidSetup)
{
  PDGravityLaw law;
  auto bad = params();
  bad.d2 = bad.L2 + 0.1;                     // COM beyond the link
  EXPECT_FALSE(law.configure_gravity(bad));
  bad = params();
  bad.m3 = std::nan("");
  EXPECT_FALSE(law.configure_gravity(bad));

  ASSERT_TRUE(law.configure_gravity(params()));
  EXPECT_FALSE(law.initialize("", {"unknown_joint"}));
  EXPECT_FALSE(law.initialize("", {"mani_joint_1", "mani_joint_1"}));
  EXPECT_FALSE(law.initialize("", {}));
}
