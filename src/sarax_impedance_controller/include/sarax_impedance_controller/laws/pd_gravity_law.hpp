// PD + the original SARAX hardware gravity equations, using measured motor angles.
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
  std::vector<double> gravity_coefficients_;  ///< per-joint coefficient of -sin(q)
};

}  // namespace sarax_impedance_controller
