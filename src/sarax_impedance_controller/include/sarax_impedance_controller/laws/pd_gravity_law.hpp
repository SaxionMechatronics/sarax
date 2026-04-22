// PD + gravity compensation, with g(q) from Pinocchio.
//
// The controller's claimed joints form a subset of the URDF's actuated joints.
// We build a pinocchio::Model from the URDF and, each tick, drive it with the
// joint_q vector padded for any unactuated joints (none exist in sarax_plus,
// but the logic is kept general).
#pragma once

#include <memory>

// pinocchio::Model is a typedef, not a class — it cannot be forward-declared.
// Include the real headers to get the full type.
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "sarax_impedance_controller/impedance_law.hpp"

namespace sarax_impedance_controller
{

class PDGravityLaw : public ImpedanceLaw
{
public:
  PDGravityLaw();
  ~PDGravityLaw() override;

  bool initialize(
    const std::string & urdf_xml,
    const std::vector<std::string> & joints) override;

  Eigen::VectorXd compute(const ImpedanceInput & in) override;

  std::string name() const override { return "PDGravity"; }

private:
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data>  data_;
  std::vector<int> joint_to_qidx_;   ///< controller joint index -> Pinocchio q index
  std::vector<int> joint_to_vidx_;   ///< controller joint index -> Pinocchio v index
  Eigen::VectorXd q_full_;           ///< scratch for Pinocchio's full q vector
};

}  // namespace sarax_impedance_controller
