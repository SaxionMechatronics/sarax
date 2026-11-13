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
  gravity_coefficients_.clear();
  if (joints.empty() || joints.size() > 2 ||
    (joints.size() == 2 && joints[0] == joints[1]))
  {
    RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
      "Configure one or two distinct SARAX motor joints");
    return false;
  }

  const double gravity_1 = gravity_.g * (gravity_.m2 * gravity_.d2 + gravity_.m3 * gravity_.L2);
  const double gravity_2 = gravity_.g * gravity_.m3 * gravity_.d3;
  if (!std::isfinite(gravity_1) || !std::isfinite(gravity_2)) {
    RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"), "Gravity coefficients overflow");
    return false;
  }

  for (const auto & jname : joints) {
    if (jname == "mani_joint_1") {
      gravity_coefficients_.push_back(gravity_1);
    } else if (jname == "mani_joint_2") {
      gravity_coefficients_.push_back(gravity_2);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
        "Hardware gravity model does not define joint '%s'", jname.c_str());
      gravity_coefficients_.clear();
      return false;
    }
  }
  return true;
}

Eigen::VectorXd PDGravityLaw::compute(const ImpedanceInput & in)
{
  Eigen::VectorXd tau_g(gravity_coefficients_.size());
  // ROS 1 hardware angles are used directly, including in the one-motor setup.
  for (size_t i = 0; i < gravity_coefficients_.size(); ++i) {
    const auto index = static_cast<Eigen::Index>(i);
    tau_g[index] = -gravity_coefficients_[i] * std::sin(in.q[index]);
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
