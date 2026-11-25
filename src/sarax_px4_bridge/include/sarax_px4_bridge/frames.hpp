// Frame conventions for SARAX+, C++ mirror of sarax_px4_bridge.frames (Python).
//
// Inside ROS everything is REP-103: world = ENU, body = FLU.  PX4 uses NED / FRD.
// Convert only at the PX4 boundary (px4_msgs), with these functions.  Quaternions
// are Eigen::Quaterniond; PX4 attitude messages carry (w, x, y, z) as float[4].
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace sarax::frames
{

inline const Eigen::Matrix3d & R_ENU_NED()
{
  static const Eigen::Matrix3d R = (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
  return R;
}

inline const Eigen::Matrix3d & R_FLU_FRD()
{
  static const Eigen::Matrix3d R = Eigen::Vector3d(1, -1, -1).asDiagonal();
  return R;
}

/// Position / velocity / acceleration / direction, NED -> ENU (own inverse).
inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d & v) {return R_ENU_NED() * v;}
inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d & v) {return R_ENU_NED() * v;}

/// Body-frame vector, FRD -> FLU (own inverse).
inline Eigen::Vector3d frd_to_flu(const Eigen::Vector3d & v) {return R_FLU_FRD() * v;}
inline Eigen::Vector3d flu_to_frd(const Eigen::Vector3d & v) {return R_FLU_FRD() * v;}

/// Attitude R(FRD->NED) -> R(FLU->ENU), and back.
inline Eigen::Matrix3d rot_ned_frd_to_enu_flu(const Eigen::Matrix3d & R)
{
  return R_ENU_NED() * R * R_FLU_FRD();
}
inline Eigen::Matrix3d rot_enu_flu_to_ned_frd(const Eigen::Matrix3d & R)
{
  return R_ENU_NED() * R * R_FLU_FRD();
}

inline Eigen::Quaterniond canonical(Eigen::Quaterniond q)
{
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return q;
}

/// PX4 attitude quaternion (w, x, y, z; FRD->NED) -> ROS quaternion (FLU->ENU), w >= 0.
inline Eigen::Quaterniond px4_quat_to_enu_flu(double w, double x, double y, double z)
{
  const Eigen::Matrix3d R =
    rot_ned_frd_to_enu_flu(Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix());
  return canonical(Eigen::Quaterniond(R));
}

/// ROS quaternion (FLU->ENU) -> PX4 attitude quaternion (w, x, y, z; FRD->NED), w >= 0.
inline Eigen::Quaterniond enu_flu_to_px4_quat(const Eigen::Quaterniond & q)
{
  return canonical(Eigen::Quaterniond(rot_enu_flu_to_ned_frd(q.normalized().toRotationMatrix())));
}

inline double wrap_angle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

/// PX4 heading (from North, clockwise) <-> ENU yaw (from East, counter-clockwise).
inline double yaw_ned_to_enu(double yaw_ned) {return wrap_angle(M_PI / 2.0 - yaw_ned);}
inline double yaw_enu_to_ned(double yaw_enu) {return wrap_angle(M_PI / 2.0 - yaw_enu);}

inline double yaw_from_quat(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quat_from_yaw(double yaw)
{
  return Eigen::Quaterniond(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
}

}  // namespace sarax::frames
