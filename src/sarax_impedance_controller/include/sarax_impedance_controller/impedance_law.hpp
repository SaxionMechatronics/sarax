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

/// Configuration-time constants for the original SARAX hardware gravity model.
struct GravityParameters
{
  double g{9.81};       ///< gravity magnitude (m/s^2)
  double L2{0.4};       ///< first moving link length (m)
  double m2{0.363477};  ///< first moving link mass (kg)
  double d2{0.2587};    ///< its center-of-mass distance from the pivot (m)
  double m3{0.113791};  ///< second moving link mass (kg)
  double d3{0.2};       ///< its center-of-mass distance from the pivot (m)
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

  /// Called before initialize(); laws without gravity keep their existing behavior.
  virtual bool configure_gravity(const GravityParameters & /*parameters*/) { return true; }

  /// Compute the commanded torque vector in the same joint order as `joints`.
  virtual Eigen::VectorXd compute(const ImpedanceInput & in) = 0;

  /// Short identifier for logging / debugging (e.g. "PDGravity").
  virtual std::string name() const = 0;
};

using ImpedanceLawPtr = std::shared_ptr<ImpedanceLaw>;

}  // namespace sarax_impedance_controller
