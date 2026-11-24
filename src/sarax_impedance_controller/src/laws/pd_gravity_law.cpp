#include "sarax_impedance_controller/laws/pd_gravity_law.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>

namespace sarax_impedance_controller
{

PDGravityLaw::PDGravityLaw() = default;
PDGravityLaw::~PDGravityLaw() = default;

bool PDGravityLaw::configure_gravity(const GravityParameters & parameters)
{
  for (double value : {parameters.g, parameters.L2, parameters.m2,
      parameters.d2, parameters.m3, parameters.d3})
  {
    if (!std::isfinite(value)) {
      RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"), "Gravity parameters must be finite");
      return false;
    }
  }
  if (parameters.g <= 0.0 || parameters.L2 <= 0.0 ||
    parameters.m2 <= 0.0 || parameters.m3 <= 0.0 ||
    parameters.d2 < 0.0 || parameters.d2 > parameters.L2 || parameters.d3 < 0.0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
      "Require g, L2, m2, m3 > 0; 0 <= d2 <= L2; d3 >= 0");
    return false;
  }
  gravity_ = parameters;
  return true;
}

bool PDGravityLaw::initialize(const std::string & /*urdf_xml*/,
                              const std::vector<std::string> & joints)
{
  index_1_ = index_2_ = -1;
  size_ = 0;
  if (joints.empty() || joints.size() > 2 ||
    (joints.size() == 2 && joints[0] == joints[1]))
  {
    RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
      "Configure one or two distinct SARAX motor joints");
    return false;
  }

  const double c1 = gravity_.g * (gravity_.m2 * gravity_.d2 + gravity_.m3 * gravity_.L2);
  const double c2 = gravity_.g * gravity_.m3 * gravity_.d3;
  if (!std::isfinite(c1) || !std::isfinite(c2)) {
    RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"), "Gravity coefficients overflow");
    return false;
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i] == "mani_joint_1") {
      index_1_ = static_cast<int>(i);
    } else if (joints[i] == "mani_joint_2") {
      index_2_ = static_cast<int>(i);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
        "Analytic gravity model does not define joint '%s'", joints[i].c_str());
      index_1_ = index_2_ = -1;
      return false;
    }
  }
  size_ = joints.size();
  return true;
}

Eigen::VectorXd PDGravityLaw::compute(const ImpedanceInput & in)
{
  const double q1 = index_1_ >= 0 ? in.q[index_1_] : 0.0;
  const double q2 = index_2_ >= 0 ? in.q[index_2_] : 0.0;
  const double c1 = gravity_.g * (gravity_.m2 * gravity_.d2 + gravity_.m3 * gravity_.L2);
  const double c2 = gravity_.g * gravity_.m3 * gravity_.d3;
  const double s2 = std::sin(q1 - q2);  // the joint axes are antiparallel in the URDF

  Eigen::VectorXd tau_g = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(size_));
  if (index_1_ >= 0) {
    tau_g[index_1_] = -c1 * std::sin(q1) - c2 * s2;
  }
  if (index_2_ >= 0) {
    tau_g[index_2_] = c2 * s2;
  }

  // tau = -Kp (q - q_d) - Kd (q_dot - q_dot_d) + g(q)
  return -in.stiffness.cwiseProduct(in.q - in.q_d)
         -in.damping.cwiseProduct(in.q_dot - in.q_dot_d)
         + tau_g;
}

}  // namespace sarax_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  sarax_impedance_controller::PDGravityLaw,
  sarax_impedance_controller::ImpedanceLaw)
