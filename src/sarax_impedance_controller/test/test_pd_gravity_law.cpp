// Pins the analytic SARAX gravity model in serial URDF joint coordinates (kept until system
// identification checks the URDF), with theta2 = q1 - q2 because the URDF joint axes are
// antiparallel (mani_joint_1 about -y, mani_joint_2 about +y of base_link):
//   tau1 = -g [(m2 d2 + m3 L2) sin q1 + m3 d3 sin(q1 - q2)],  tau2 = g m3 d3 sin(q1 - q2).
// It is also compared with PDPinocchioGravityLaw on a URDF built from the same constants.
#include <cmath>
#include <sstream>

#include <gtest/gtest.h>

#include "sarax_impedance_controller/laws/pd_gravity_law.hpp"
#include "sarax_impedance_controller/laws/pd_pinocchio_gravity_law.hpp"

using sarax_impedance_controller::GravityParameters;
using sarax_impedance_controller::ImpedanceInput;
using sarax_impedance_controller::PDGravityLaw;
using sarax_impedance_controller::PDPinocchioGravityLaw;

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
  const double s2 = std::sin(q1 - q2);
  EXPECT_NEAR(tau[0], -p.g * ((p.m2 * p.d2 + p.m3 * p.L2) * std::sin(q1) + p.m3 * p.d3 * s2), 1e-12);
  EXPECT_NEAR(tau[1], p.g * p.m3 * p.d3 * s2, 1e-12);
}

// Two point-mass links on the antiparallel axes of sarax_plus.urdf.xacro, with the masses and
// COM distances of GravityParameters (link 2 = URDF link3, link 3 = URDF link4).
TEST(PDGravityLaw, MatchesPinocchioLawOverGrid)
{
  const auto p = params();
  std::ostringstream urdf;
  urdf << R"(<robot name="arm"><link name="base_link"/>
  <link name="l2"><inertial><origin xyz="0 0 )" << p.d2 << R"("/><mass value=")" << p.m2
       << R"("/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="l3"><inertial><origin xyz="0 0 )" << p.d3 << R"("/><mass value=")" << p.m3
       << R"("/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="mani_joint_1" type="revolute"><parent link="base_link"/><child link="l2"/>
    <axis xyz="0 -1 0"/><limit lower="-3.2" upper="3.2" effort="10" velocity="10"/></joint>
  <joint name="mani_joint_2" type="revolute"><parent link="l2"/><child link="l3"/>
    <origin xyz="0 0 )" << p.L2 << R"("/><axis xyz="0 1 0"/>
    <limit lower="-3.2" upper="3.2" effort="10" velocity="10"/></joint></robot>)";

  PDGravityLaw analytic;
  ASSERT_TRUE(analytic.configure_gravity(p));
  ASSERT_TRUE(analytic.initialize("", kJoints));
  PDPinocchioGravityLaw pinocchio;
  ASSERT_TRUE(pinocchio.initialize(urdf.str(), kJoints));

  for (double q1 = -3.0; q1 <= 3.0; q1 += 0.5) {
    for (double q2 = -3.0; q2 <= 3.0; q2 += 0.5) {
      const auto a = analytic.compute(at(q1, q2));
      const auto b = pinocchio.compute(at(q1, q2));
      EXPECT_NEAR(a[0], b[0], 1e-9) << "q=(" << q1 << ", " << q2 << ")";
      EXPECT_NEAR(a[1], b[1], 1e-9) << "q=(" << q1 << ", " << q2 << ")";
    }
  }
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
  const auto tau = law.compute(in);
  ASSERT_EQ(tau.size(), 1);
  // Joint 2 is held at 0, as in PDPinocchioGravityLaw.
  PDGravityLaw both;
  ASSERT_TRUE(both.configure_gravity(params()));
  ASSERT_TRUE(both.initialize("", kJoints));
  EXPECT_NEAR(tau[0], both.compute(at(0.5, 0.0))[0], 1e-12);
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
