#include "sarax_impedance_controller/laws/pd_gravity_law.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <rclcpp/rclcpp.hpp>

namespace sarax_impedance_controller
{

PDGravityLaw::PDGravityLaw() = default;
PDGravityLaw::~PDGravityLaw() = default;

namespace
{
// For a single-DoF (revolute / prismatic) joint these agree; for multi-DoF
// joints they differ. All joints the impedance controller claims are single-DoF.
struct JointIdx { int q{0}; int v{0}; };
}

bool PDGravityLaw::initialize(const std::string & urdf_xml,
                              const std::vector<std::string> & joints)
{
  model_ = std::make_unique<pinocchio::Model>();
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, *model_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
                 "Failed to build Pinocchio model from URDF: %s", e.what());
    return false;
  }

  data_ = std::make_unique<pinocchio::Data>(*model_);

  joint_to_qidx_.clear();
  joint_to_vidx_.clear();
  joint_to_qidx_.reserve(joints.size());
  joint_to_vidx_.reserve(joints.size());

  for (const auto & jname : joints) {
    if (!model_->existJointName(jname)) {
      RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
                   "Joint '%s' not found in Pinocchio model", jname.c_str());
      return false;
    }
    const auto jid = model_->getJointId(jname);
    const auto & jdata = model_->joints[jid];
    if (jdata.nq() != 1 || jdata.nv() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("PDGravityLaw"),
                   "Joint '%s' is not single-DoF (nq=%d, nv=%d); unsupported.",
                   jname.c_str(), jdata.nq(), jdata.nv());
      return false;
    }
    joint_to_qidx_.push_back(jdata.idx_q());
    joint_to_vidx_.push_back(jdata.idx_v());
  }

  q_full_ = pinocchio::neutral(*model_);
  return true;
}

Eigen::VectorXd PDGravityLaw::compute(const ImpedanceInput & in)
{
  // Fill the controlled joints into the full q vector; other DoF stay at neutral.
  // For sarax_plus there are no unactuated joints so this is a 2-element write.
  for (size_t i = 0; i < joint_to_qidx_.size(); ++i) {
    q_full_[joint_to_qidx_[i]] = in.q[static_cast<Eigen::Index>(i)];
  }

  // g(q) via RNEA with zero velocity and zero acceleration.
  const Eigen::VectorXd zeros_v = Eigen::VectorXd::Zero(model_->nv);
  pinocchio::rnea(*model_, *data_, q_full_, zeros_v, zeros_v);
  const Eigen::VectorXd & tau_g_full = data_->tau;  // size nv

  Eigen::VectorXd tau_g(joint_to_vidx_.size());
  for (size_t i = 0; i < joint_to_vidx_.size(); ++i) {
    tau_g[static_cast<Eigen::Index>(i)] = tau_g_full[joint_to_vidx_[i]];
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
