// Reference values come from sarax_px4_bridge.frames (Python), including a tilted attitude.
#include <gtest/gtest.h>

#include "sarax_px4_bridge/frames.hpp"

using namespace sarax::frames;  // NOLINT

namespace
{
Eigen::Quaterniond norm_px4(double w, double x, double y, double z)
{
  return Eigen::Quaterniond(w, x, y, z).normalized();
}
}  // namespace

TEST(Frames, VectorsAreInvolutions)
{
  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  EXPECT_TRUE(ned_to_enu(v).isApprox(Eigen::Vector3d(2.0, 1.0, -3.0)));
  EXPECT_TRUE(enu_to_ned(ned_to_enu(v)).isApprox(v));
  EXPECT_TRUE(frd_to_flu(v).isApprox(Eigen::Vector3d(1.0, -2.0, -3.0)));
  EXPECT_TRUE(flu_to_frd(frd_to_flu(v)).isApprox(v));
}

TEST(Frames, HeadingEastIsIdentityInEnuFlu)
{
  // Level, nose pointing East (NED heading 90 deg): ENU/FLU attitude is the identity.
  const auto q = px4_quat_to_enu_flu(std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5));
  EXPECT_NEAR(q.x(), 0.0, 1e-9);
  EXPECT_NEAR(q.y(), 0.0, 1e-9);
  EXPECT_NEAR(q.z(), 0.0, 1e-9);
  EXPECT_NEAR(q.w(), 1.0, 1e-9);
}

TEST(Frames, TiltedAttitudeMatchesPython)
{
  const auto n1 = norm_px4(0.9, 0.1, -0.2, 0.3);
  const auto q1 = px4_quat_to_enu_flu(n1.w(), n1.x(), n1.y(), n1.z());
  EXPECT_NEAR(q1.x(), -0.072547625, 1e-8);
  EXPECT_NEAR(q1.y(), 0.217642875, 1e-8);
  EXPECT_NEAR(q1.z(), 0.43528575, 1e-8);
  EXPECT_NEAR(q1.w(), 0.8705715, 1e-8);

  const auto n2 = norm_px4(0.5, -0.5, 0.5, 0.5);
  const auto q2 = px4_quat_to_enu_flu(n2.w(), n2.x(), n2.y(), n2.z());
  EXPECT_NEAR(q2.x(), 0.0, 1e-8);
  EXPECT_NEAR(q2.y(), -0.707106781, 1e-8);
  EXPECT_NEAR(q2.z(), 0.0, 1e-8);
  EXPECT_NEAR(q2.w(), 0.707106781, 1e-8);
}

TEST(Frames, QuaternionRoundTrip)
{
  const Eigen::Quaterniond q(0.9, 0.1, -0.2, 0.3);
  const auto p = enu_flu_to_px4_quat(q.normalized());
  const auto back = px4_quat_to_enu_flu(p.w(), p.x(), p.y(), p.z());
  EXPECT_TRUE(back.isApprox(canonical(q), 1e-9));
}

TEST(Frames, Yaw)
{
  EXPECT_NEAR(yaw_ned_to_enu(0.3), 1.2707963267948967, 1e-12);
  EXPECT_NEAR(yaw_enu_to_ned(-1.2), 2.7707963267948967, 1e-12);
  EXPECT_NEAR(yaw_from_quat(quat_from_yaw(0.7)), 0.7, 1e-12);
}
