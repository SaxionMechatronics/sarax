#include "sarax_impedance_controller/laws/pd_pinocchio_gravity_law.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sarax_impedance_controller
{

struct PDPinocchioGravityLaw::Impl
{
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd q_full;
  Eigen::VectorXd zero_v;
  std::vector<Eigen::Index> idx_q;
  std::vector<Eigen::Index> idx_v;
};

PDPinocchioGravityLaw::PDPinocchioGravityLaw() = default;
PDPinocchioGravityLaw::~PDPinocchioGravityLaw() = default;

bool PDPinocchioGravityLaw::initialize(
  const std::string & urdf_xml, const std::vector<std::string> & joints)
{
  const auto logger = rclcpp::get_logger("PDPinocchioGravityLaw");
  impl_.reset();
  if (urdf_xml.empty()) {
    RCLCPP_ERROR(logger, "robot_description is empty; the Pinocchio law needs the URDF");
    return false;
  }
  if (joints.empty()) {
    RCLCPP_ERROR(logger, "No joints configured");
    return false;
  }

  auto impl = std::make_unique<Impl>();
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, impl->model);  // fixed base
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Cannot build a Pinocchio model from robot_description: %s", e.what());
    return false;
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      if (joints[i] == joints[j]) {
        RCLCPP_ERROR(logger, "Joint '%s' is listed twice", joints[i].c_str());
        return false;
      }
    }
    if (!impl->model.existJointName(joints[i])) {
      RCLCPP_ERROR(logger, "Joint '%s' is not in robot_description", joints[i].c_str());
      return false;
    }
    const auto id = impl->model.getJointId(joints[i]);
    if (impl->model.nqs[id] != 1 || impl->model.nvs[id] != 1) {
      RCLCPP_ERROR(logger, "Joint '%s' is not a 1-DoF joint", joints[i].c_str());
      return false;
    }
    impl->idx_q.push_back(impl->model.idx_qs[id]);
    impl->idx_v.push_back(impl->model.idx_vs[id]);
  }

  impl->data = pinocchio::Data(impl->model);
  impl->q_full = pinocchio::neutral(impl->model);
  impl->zero_v = Eigen::VectorXd::Zero(impl->model.nv);
  impl_ = std::move(impl);
  return true;
}

Eigen::VectorXd PDPinocchioGravityLaw::gravity_torques(const Eigen::VectorXd & q)
{
  const auto n = static_cast<Eigen::Index>(impl_->idx_q.size());
  for (Eigen::Index i = 0; i < n; ++i) {
    impl_->q_full[impl_->idx_q[static_cast<size_t>(i)]] = q[i];
  }
  // With v = a = 0, RNEA returns g(q) (gravity from model.gravity, -z by default).
  const Eigen::VectorXd & tau_full =
    pinocchio::rnea(impl_->model, impl_->data, impl_->q_full, impl_->zero_v, impl_->zero_v);
  Eigen::VectorXd tau(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    tau[i] = tau_full[impl_->idx_v[static_cast<size_t>(i)]];
  }
  return tau;
}

Eigen::VectorXd PDPinocchioGravityLaw::compute(const ImpedanceInput & in)
{
  // tau = -Kp (q - q_d) - Kd (q_dot - q_dot_d) + g(q)
  return -in.stiffness.cwiseProduct(in.q - in.q_d)
         -in.damping.cwiseProduct(in.q_dot - in.q_dot_d)
         + gravity_torques(in.q);
}

}  // namespace sarax_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  sarax_impedance_controller::PDPinocchioGravityLaw,
  sarax_impedance_controller::ImpedanceLaw)
