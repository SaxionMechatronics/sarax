// The fold/extend ramp: ends exactly on the pose, keeps every joint within its velocity limit.
#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "sarax_impedance_controller/pose_ramp.hpp"

using sarax_impedance_controller::PoseRamp;

namespace
{
Eigen::VectorXd vec(double a, double b)
{
  Eigen::VectorXd v(2);
  v << a, b;
  return v;
}
}  // namespace

TEST(PoseRamp, DurationFollowsTheSlowestJoint)
{
  PoseRamp ramp;
  // j1: 1.0 rad at 2 rad/s -> 0.5 s of mean velocity; j2: 0.6 rad at 0.5 rad/s -> 1.2 s.
  const double duration = ramp.start(vec(0.0, 0.0), vec(1.0, 0.6), vec(2.0, 0.5));
  EXPECT_NEAR(duration, PoseRamp::kPeakFactor * 1.2, 1e-12);
  EXPECT_TRUE(ramp.active());
}

TEST(PoseRamp, EndsExactlyOnTheTargetWithZeroVelocity)
{
  PoseRamp ramp;
  const Eigen::VectorXd target = vec(0.7, -0.4);
  ramp.start(vec(-0.2, 0.3), target, vec(1.0, 1.0));
  Eigen::VectorXd q = vec(-0.2, 0.3), qdot = Eigen::VectorXd::Zero(2);
  int steps = 0;
  while (ramp.active() && steps++ < 100000) {
    ramp.step(0.004, q, qdot);
  }
  EXPECT_FALSE(ramp.active());
  EXPECT_EQ(q[0], target[0]);
  EXPECT_EQ(q[1], target[1]);
  EXPECT_EQ(qdot.norm(), 0.0);
}

TEST(PoseRamp, NeverExceedsTheVelocityLimitAndIsMonotonic)
{
  PoseRamp ramp;
  const Eigen::VectorXd vmax = vec(1.5, 0.4);
  ramp.start(vec(0.0, 0.0), vec(-1.2, 0.9), vmax);
  Eigen::VectorXd q = vec(0.0, 0.0), qdot = Eigen::VectorXd::Zero(2);
  double peak0 = 0.0, peak1 = 0.0, prev0 = 0.0;
  while (ramp.active()) {
    ramp.step(0.002, q, qdot);
    peak0 = std::max(peak0, std::abs(qdot[0]));
    peak1 = std::max(peak1, std::abs(qdot[1]));
    EXPECT_LE(q[0], prev0 + 1e-12);  // moves toward -1.2 only
    prev0 = q[0];
  }
  EXPECT_LE(peak0, vmax[0] + 1e-9);
  EXPECT_LE(peak1, vmax[1] + 1e-9);
  EXPECT_NEAR(peak1, vmax[1], 1e-3);  // the slowest joint runs at its limit
}

TEST(PoseRamp, SamePoseStartsNoRamp)
{
  PoseRamp ramp;
  EXPECT_EQ(ramp.start(vec(0.3, 0.3), vec(0.3, 0.3), vec(1.0, 1.0)), 0.0);
  EXPECT_FALSE(ramp.active());
}

TEST(PoseRamp, CancelStopsAndStepDoesNothing)
{
  PoseRamp ramp;
  ramp.start(vec(0.0, 0.0), vec(1.0, 1.0), vec(1.0, 1.0));
  Eigen::VectorXd q = vec(0.0, 0.0), qdot = Eigen::VectorXd::Zero(2);
  ramp.step(0.1, q, qdot);
  const Eigen::VectorXd mid = q;
  ramp.cancel();
  ramp.step(0.1, q, qdot);
  EXPECT_FALSE(ramp.active());
  EXPECT_EQ(q, mid);
}
