// The arm torque limit has one source: the <limit effort> of each joint in the URDF.
// The controller reads it from robot_description and clamps its command to +-limit.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace sarax_impedance_controller
{

/// Effort limits (N.m) of `joints` read from the URDF `<limit effort>`, in the order of `joints`.
/// Returns false and sets `error` when the URDF does not parse, a joint is missing, or a joint
/// has no finite effort limit greater than zero.
bool effort_limits_from_urdf(
  const std::string & urdf_xml, const std::vector<std::string> & joints,
  Eigen::VectorXd & limits, std::string & error);

/// Position and velocity limits of `joints` from the URDF `<limit lower upper velocity>`, in the order
/// of `joints`. Returns false and sets `error` when a joint is missing, has no limit, a velocity
/// limit that is not finite and greater than zero, or lower > upper.
bool motion_limits_from_urdf(
  const std::string & urdf_xml, const std::vector<std::string> & joints,
  Eigen::VectorXd & lower, Eigen::VectorXd & upper, Eigen::VectorXd & velocity,
  std::string & error);

/// Clamps `tau` symmetrically to [-limits, +limits], element-wise.
void clamp_effort(Eigen::VectorXd & tau, const Eigen::VectorXd & limits);

}  // namespace sarax_impedance_controller
