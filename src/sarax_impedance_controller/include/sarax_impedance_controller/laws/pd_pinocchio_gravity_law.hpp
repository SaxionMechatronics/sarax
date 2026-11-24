// PD + gravity torques from Pinocchio's RNEA on the robot_description.
//
// The model is the URDF itself (base_link fixed, gravity along -z of base_link), so the
// link masses, centres of mass and joint axes are never duplicated in parameters. The
// gravity term is computed in joint coordinates, i.e. q is the URDF joint angle.
#pragma once

#include <memory>

#include "sarax_impedance_controller/impedance_law.hpp"

namespace sarax_impedance_controller
{

class PDPinocchioGravityLaw : public ImpedanceLaw
{
public:
  PDPinocchioGravityLaw();
  ~PDPinocchioGravityLaw() override;

  bool initialize(
    const std::string & urdf_xml,
    const std::vector<std::string> & joints) override;

  Eigen::VectorXd compute(const ImpedanceInput & in) override;

  /// Gravity torques g(q) for the configured joints (same order as `joints`).
  Eigen::VectorXd gravity_torques(const Eigen::VectorXd & q);

  std::string name() const override { return "PDPinocchioGravity"; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;  ///< keeps Pinocchio out of this public header
};

}  // namespace sarax_impedance_controller
