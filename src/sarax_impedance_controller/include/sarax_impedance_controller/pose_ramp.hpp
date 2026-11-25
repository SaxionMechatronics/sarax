// Minimum-jerk ramp between two joint configurations, used by the fold/extend service.
//
// All joints start and stop together. The duration is the shortest one at which no joint exceeds
// its velocity limit: the minimum-jerk profile peaks at 1.875 x the mean velocity, so
// T = 1.875 * max_i |q1_i - q0_i| / vmax_i. Header-only and ROS-free so it can be unit tested.
#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace sarax_impedance_controller
{

class PoseRamp
{
public:
  /// Peak velocity of the minimum-jerk profile over its mean velocity.
  static constexpr double kPeakFactor = 1.875;

  /// Starts a ramp from `q0` to `q1`. `vmax` are the per-joint velocity limits (all > 0).
  /// Returns the duration [s]; 0 when the poses coincide (no ramp is started).
  double start(const Eigen::VectorXd & q0, const Eigen::VectorXd & q1, const Eigen::VectorXd & vmax)
  {
    q0_ = q0;
    q1_ = q1;
    double duration = 0.0;
    for (Eigen::Index i = 0; i < q0.size(); ++i) {
      duration = std::max(duration, kPeakFactor * std::abs(q1[i] - q0[i]) / vmax[i]);
    }
    duration_ = duration;
    time_ = 0.0;
    active_ = duration > 0.0;
    return duration;
  }

  void cancel() { active_ = false; }
  bool active() const { return active_; }
  double duration() const { return duration_; }

  /// Advances the ramp by `dt` and writes the reference. On the last step `q` is exactly the
  /// target, `qdot` is zero and the ramp ends. Does nothing when no ramp is active.
  void step(double dt, Eigen::VectorXd & q, Eigen::VectorXd & qdot)
  {
    if (!active_) {
      return;
    }
    time_ += std::max(dt, 0.0);
    const double tau = std::min(time_ / duration_, 1.0);
    const double s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
    const double ds = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / duration_;
    q = q0_ + s * (q1_ - q0_);
    qdot = ds * (q1_ - q0_);
    if (tau >= 1.0) {
      q = q1_;
      qdot.setZero();
      active_ = false;
    }
  }

private:
  Eigen::VectorXd q0_, q1_;
  double duration_{0.0};
  double time_{0.0};
  bool active_{false};
};

}  // namespace sarax_impedance_controller
