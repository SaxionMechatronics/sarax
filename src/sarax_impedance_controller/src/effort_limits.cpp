#include "sarax_impedance_controller/effort_limits.hpp"

#include <algorithm>
#include <cmath>

#include <urdf_parser/urdf_parser.h>

namespace sarax_impedance_controller
{

bool effort_limits_from_urdf(
  const std::string & urdf_xml, const std::vector<std::string> & joints,
  Eigen::VectorXd & limits, std::string & error)
{
  const auto model = urdf::parseURDF(urdf_xml);
  if (!model) {
    error = "robot_description is empty or not a valid URDF";
    return false;
  }
  Eigen::VectorXd result(static_cast<Eigen::Index>(joints.size()));
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto it = model->joints_.find(joints[i]);
    if (it == model->joints_.end()) {
      error = "joint '" + joints[i] + "' is not in the URDF";
      return false;
    }
    const auto & limit = it->second->limits;
    if (!limit || !std::isfinite(limit->effort) || limit->effort <= 0.0) {
      error = "joint '" + joints[i] + "' has no <limit effort> greater than zero in the URDF";
      return false;
    }
    result[static_cast<Eigen::Index>(i)] = limit->effort;
  }
  limits = result;
  return true;
}

void clamp_effort(Eigen::VectorXd & tau, const Eigen::VectorXd & limits)
{
  for (Eigen::Index i = 0; i < tau.size(); ++i) {
    tau[i] = std::clamp(tau[i], -limits[i], limits[i]);
  }
}

}  // namespace sarax_impedance_controller
