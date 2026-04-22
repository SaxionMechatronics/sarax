// Plain PD law: tau = -Kp (q - q_d) - Kd (q_dot - q_dot_d).
// No gravity, no inertia feed-forward. Ships as the minimal baseline.
#pragma once

#include "sarax_impedance_controller/impedance_law.hpp"

namespace sarax_impedance_controller
{

class PDLaw : public ImpedanceLaw
{
public:
  bool initialize(
    const std::string & urdf_xml,
    const std::vector<std::string> & joints) override;

  Eigen::VectorXd compute(const ImpedanceInput & in) override;

  std::string name() const override { return "PD"; }

private:
  size_t n_joints_{0};
};

}  // namespace sarax_impedance_controller
