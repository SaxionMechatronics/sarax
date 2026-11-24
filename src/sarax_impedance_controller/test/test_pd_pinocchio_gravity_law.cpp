// Pins the Pinocchio gravity law against a hand-derived reference for sarax_plus.urdf.xacro.
//
// Kinematics read off the URDF: base_link -> link1 is Rx(90 deg), link1 -> link2 is Ry(90 deg),
// so the arm axis (link y) points along +z of base_link at q = 0 and the joint axes lie along
// -y (mani_joint_1) and +y (mani_joint_2) of base_link. With theta1 = q1 and theta2 = q1 - q2
// (the axes are antiparallel) the height of each centre of mass above the joint-1 axis is
//   link3: d3 cos(theta1)                       m3 = 0.363477, d3 = 0.2587
//   link4: L cos(theta1) + d4 cos(theta2)       m4 = 0.113791, L = 0.4, d4 = 0.2
// and g(q) = dU/dq with U = g * sum(m * height):
//   tau1 = -g [ (m3 d3 + m4 L) sin(theta1) + m4 d4 sin(theta2) ]
//   tau2 =  g  m4 d4 sin(theta2)
// ee_link (0.01 kg, 0.4 m along link4) is lumped into link4. Link1 and link2 are fixed to the base and do not load the joints.
#include <cmath>
#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

#include "sarax_impedance_controller/laws/pd_pinocchio_gravity_law.hpp"

using sarax_impedance_controller::ImpedanceInput;
using sarax_impedance_controller::PDPinocchioGravityLaw;

namespace
{
const std::vector<std::string> kJoints{"mani_joint_1", "mani_joint_2"};

std::string urdf()
{
  std::ifstream file(TEST_URDF_PATH);
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

Eigen::Vector2d reference(double q1, double q2)
{
  const double g = 9.81;
  // link4 plus the fixed ee_link (0.01 kg at 0.4 m along link4) act as one body on joint 2
  const double m3 = 0.363477, d3 = 0.2587, L = 0.4;
  const double m4 = 0.113791 + 0.01, d4 = (0.113791 * 0.2 + 0.01 * 0.4) / m4;
  const double th2 = q1 - q2;
  return {-g * ((m3 * d3 + m4 * L) * std::sin(q1) + m4 * d4 * std::sin(th2)),
    g * m4 * d4 * std::sin(th2)};
}

ImpedanceInput at(const Eigen::VectorXd & q)
{
  ImpedanceInput in;
  in.q = q;
  in.q_d = q;
  in.q_dot = in.q_dot_d = Eigen::VectorXd::Zero(q.size());
  in.stiffness = Eigen::VectorXd::Constant(q.size(), 10.0);
  in.damping = Eigen::VectorXd::Constant(q.size(), 1.0);
  return in;
}
}  // namespace

TEST(PDPinocchioGravityLaw, MatchesHandComputedReference)
{
  PDPinocchioGravityLaw law;
  ASSERT_TRUE(law.initialize(urdf(), kJoints));
  const double configs[][2] = {
    {0.0, 0.0}, {0.7, -0.4}, {-1.2, 0.9}, {1.5, 2.0}, {-0.3, -2.5}, {0.5, 0.5}, {1.0, 0.0}};
  for (const auto & c : configs) {
    const auto tau = law.compute(at(Eigen::Vector2d(c[0], c[1])));
    const Eigen::Vector2d ref = reference(c[0], c[1]);
    EXPECT_NEAR(tau[0], ref[0], 1e-6) << "q=(" << c[0] << ", " << c[1] << ")";
    EXPECT_NEAR(tau[1], ref[1], 1e-6) << "q=(" << c[0] << ", " << c[1] << ")";
  }
}

TEST(PDPinocchioGravityLaw, AddsPdTerm)
{
  PDPinocchioGravityLaw law;
  ASSERT_TRUE(law.initialize(urdf(), kJoints));
  auto in = at(Eigen::Vector2d(0.0, 0.0));
  in.q_d = Eigen::Vector2d(0.1, -0.2);
  in.q_dot = Eigen::Vector2d(0.5, 0.0);
  const auto tau = law.compute(in);
  EXPECT_NEAR(tau[0], 10.0 * 0.1 - 1.0 * 0.5, 1e-9);
  EXPECT_NEAR(tau[1], 10.0 * -0.2, 1e-9);
}

TEST(PDPinocchioGravityLaw, SingleJointAndOrder)
{
  PDPinocchioGravityLaw law;
  ASSERT_TRUE(law.initialize(urdf(), {"mani_joint_2", "mani_joint_1"}));
  const auto tau = law.compute(at(Eigen::Vector2d(-0.4, 0.7)));  // (q2, q1)
  const Eigen::Vector2d ref = reference(0.7, -0.4);
  EXPECT_NEAR(tau[0], ref[1], 1e-6);
  EXPECT_NEAR(tau[1], ref[0], 1e-6);

  PDPinocchioGravityLaw one;
  ASSERT_TRUE(one.initialize(urdf(), {"mani_joint_1"}));
  const auto t1 = one.compute(at(Eigen::VectorXd::Constant(1, 0.7)));
  ASSERT_EQ(t1.size(), 1);
  // Joint 2 is held at its neutral (q2 = 0) in the model.
  EXPECT_NEAR(t1[0], reference(0.7, 0.0)[0], 1e-6);
}

TEST(PDPinocchioGravityLaw, RejectsInvalidSetup)
{
  PDPinocchioGravityLaw law;
  EXPECT_FALSE(law.initialize("", kJoints));
  EXPECT_FALSE(law.initialize("not xml", kJoints));
  EXPECT_FALSE(law.initialize(urdf(), {}));
  EXPECT_FALSE(law.initialize(urdf(), {"unknown_joint"}));
  EXPECT_FALSE(law.initialize(urdf(), {"mani_joint_1", "mani_joint_1"}));
  EXPECT_FALSE(law.initialize(urdf(), {"rotor_1_joint"}));  // continuous: 2 config dims
}
