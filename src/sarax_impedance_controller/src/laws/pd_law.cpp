#include "sarax_impedance_controller/laws/pd_law.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace sarax_impedance_controller
{

bool PDLaw::initialize(const std::string & /*urdf_xml*/,
                       const std::vector<std::string> & joints)
{
  n_joints_ = joints.size();
  return n_joints_ > 0;
}

Eigen::VectorXd PDLaw::compute(const ImpedanceInput & in)
{
  // tau = -Kp (q - q_d) - Kd (q_dot - q_dot_d)
  return -in.stiffness.cwiseProduct(in.q - in.q_d)
         - in.damping.cwiseProduct(in.q_dot - in.q_dot_d);
}

}  // namespace sarax_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  sarax_impedance_controller::PDLaw,
  sarax_impedance_controller::ImpedanceLaw)
