// PD + analytic gravity of the 2-link arm in serial URDF joint coordinates (q1, q2 as in
// robot_description). With theta1 = q1 and theta2 = q1 - q2 (the URDF joint axes are antiparallel):
//   tau1 = -g [ (m2 d2 + m3 L2) sin(theta1) + m3 d3 sin(theta2) ]
//   tau2 =  g m3 d3 sin(theta2)
// Link 2/3 of GravityParameters are the URDF's link3/link4. A joint that is not configured is
// held at q = 0, like PDPinocchioGravityLaw.
#pragma once

#include "sarax_impedance_controller/impedance_law.hpp"

namespace sarax_impedance_controller
{

class PDGravityLaw : public ImpedanceLaw
{
public:
  PDGravityLaw();
  ~PDGravityLaw() override;

  bool configure_gravity(const GravityParameters & parameters) override;

  bool initialize(
    const std::string & urdf_xml,
    const std::vector<std::string> & joints) override;

  Eigen::VectorXd compute(const ImpedanceInput & in) override;

  std::string name() const override { return "PDGravity"; }

private:
  GravityParameters gravity_;
  /// Position of joint 1 / joint 2 in ImpedanceInput::q, -1 if that joint is not configured.
  int index_1_{-1};
  int index_2_{-1};
  std::size_t size_{0};
};

}  // namespace sarax_impedance_controller
