// Abstract impedance-law interface.
//
// An ImpedanceLaw maps (measured state, desired state, gains) to a joint-torque
// command. Concrete laws are registered via pluginlib so third-party packages
// can add new laws without modifying sarax_impedance_controller.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace sarax_impedance_controller
{

/// Snapshot of measured + desired state and per-joint gains, rebuilt every tick.
struct ImpedanceInput
{
  Eigen::VectorXd q;          ///< measured joint positions (n)
  Eigen::VectorXd q_dot;      ///< measured joint velocities (n)
  Eigen::VectorXd q_d;        ///< desired joint positions (n)
  Eigen::VectorXd q_dot_d;    ///< desired joint velocities (n)
  Eigen::VectorXd stiffness;  ///< per-joint Kp (n)
  Eigen::VectorXd damping;    ///< per-joint Kd (n)
};

/// Pure-virtual base class for joint-space impedance laws.
class ImpedanceLaw
{
public:
  virtual ~ImpedanceLaw() = default;

  /// Called once, after construction, from the controller's on_configure().
  /// @param urdf_xml   the full robot_description string
  /// @param joints     ordered list of actuated joints the controller claims
  /// @return           true on success; false causes the controller to refuse to configure
  virtual bool initialize(
    const std::string & urdf_xml,
    const std::vector<std::string> & joints) = 0;

  /// Compute the commanded torque vector in the same joint order as `joints`.
  virtual Eigen::VectorXd compute(const ImpedanceInput & in) = 0;

  /// Short identifier for logging / debugging (e.g. "PDGravity").
  virtual std::string name() const = 0;
};

using ImpedanceLawPtr = std::shared_ptr<ImpedanceLaw>;

}  // namespace sarax_impedance_controller
