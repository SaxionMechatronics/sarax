// Reference watchdog for the joint impedance controller.
//
// A streamed reference (~/reference) must keep arriving. When none has arrived for
// `timeout` seconds, the reference expires and the controller holds the current pose.
// A persistent reference (the q_ref_N parameters) never expires. timeout = 0 disables
// the watchdog. Header-only and ROS-free so it can be unit tested directly.
#pragma once

namespace sarax_impedance_controller
{

class ReferenceWatchdog
{
public:
  explicit ReferenceWatchdog(double timeout_s = 0.5) : timeout_s_(timeout_s) {}

  void set_timeout(double timeout_s) { timeout_s_ = timeout_s; }
  double timeout() const { return timeout_s_; }

  /// Forget the current reference (on activation).
  void reset()
  {
    expired_ = false;
    last_s_ = 0.0;
  }

  /// Call once per control cycle.
  ///   have_reference: a reference has been received since activation
  ///   fresh:          a new reference message arrived since the last call
  ///   persistent:     the current reference never expires
  ///   now_s:          controller time (s)
  /// Returns true only in the cycle in which the reference expires, so the caller
  /// can latch the hold pose once.
  bool update(bool have_reference, bool fresh, bool persistent, double now_s)
  {
    if (fresh) {
      last_s_ = now_s;
      expired_ = false;
    }
    if (have_reference && !expired_ && !persistent && timeout_s_ > 0.0 &&
      now_s - last_s_ > timeout_s_)
    {
      expired_ = true;
      return true;
    }
    return false;
  }

  bool expired() const { return expired_; }

  /// True while a reference is being tracked (received and not expired).
  bool active(bool have_reference) const { return have_reference && !expired_; }

private:
  double timeout_s_;
  double last_s_{0.0};
  bool expired_{false};
};

}  // namespace sarax_impedance_controller
