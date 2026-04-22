#include <gtest/gtest.h>

#include "sarax_impedance_controller/laws/pd_law.hpp"

using sarax_impedance_controller::ImpedanceInput;
using sarax_impedance_controller::PDLaw;

namespace
{
ImpedanceInput make_input(size_t n)
{
  ImpedanceInput in;
  in.q         = Eigen::VectorXd::Zero(n);
  in.q_dot     = Eigen::VectorXd::Zero(n);
  in.q_d       = Eigen::VectorXd::Zero(n);
  in.q_dot_d   = Eigen::VectorXd::Zero(n);
  in.stiffness = Eigen::VectorXd::Ones(n) * 100.0;
  in.damping   = Eigen::VectorXd::Ones(n) * 10.0;
  return in;
}
}  // namespace

TEST(PDLaw, ZeroErrorGivesZeroTorque)
{
  PDLaw law;
  ASSERT_TRUE(law.initialize("", {"j1", "j2"}));

  auto in = make_input(2);
  auto tau = law.compute(in);

  EXPECT_EQ(tau.size(), 2);
  EXPECT_DOUBLE_EQ(tau[0], 0.0);
  EXPECT_DOUBLE_EQ(tau[1], 0.0);
}

TEST(PDLaw, PositionErrorProducesProportionalTorque)
{
  PDLaw law;
  ASSERT_TRUE(law.initialize("", {"j1", "j2"}));

  auto in = make_input(2);
  in.q[0] = 0.1;           // q - q_d = 0.1
  auto tau = law.compute(in);
  EXPECT_DOUBLE_EQ(tau[0], -100.0 * 0.1);   // -Kp * (q - q_d) = -10
  EXPECT_DOUBLE_EQ(tau[1], 0.0);
}

TEST(PDLaw, VelocityErrorProducesProportionalTorque)
{
  PDLaw law;
  ASSERT_TRUE(law.initialize("", {"j1"}));

  auto in = make_input(1);
  in.q_dot[0] = 2.0;      // q_dot - q_dot_d = 2.0
  auto tau = law.compute(in);
  EXPECT_DOUBLE_EQ(tau[0], -10.0 * 2.0);    // -Kd * (q_dot - q_dot_d) = -20
}
