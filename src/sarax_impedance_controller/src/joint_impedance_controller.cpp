#include "sarax_impedance_controller/joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace sarax_impedance_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

JointImpedanceController::JointImpedanceController() = default;
JointImpedanceController::~JointImpedanceController() = default;

// --------------------------------------------------------------------------
// Lifecycle
// --------------------------------------------------------------------------

CallbackReturn JointImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("stiffness", {});
    auto_declare<std::vector<double>>("damping", {});
    auto_declare<std::vector<double>>("effort_limits", {});
    auto_declare<double>("max_position_error",
                         std::numeric_limits<double>::infinity());
    auto_declare<std::string>(
      "impedance_law",
      "sarax_impedance_controller::PDGravityLaw");
    auto_declare<std::string>("robot_description_param", "robot_description");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  joint_names_ = node->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter is empty");
    return CallbackReturn::ERROR;
  }
  n_joints_ = joint_names_.size();

  const auto k = node->get_parameter("stiffness").as_double_array();
  const auto d = node->get_parameter("damping").as_double_array();
  if (k.size() != n_joints_ || d.size() != n_joints_) {
    RCLCPP_ERROR(node->get_logger(),
      "stiffness/damping must have %zu entries (one per joint)", n_joints_);
    return CallbackReturn::ERROR;
  }
  stiffness_ = Eigen::Map<const Eigen::VectorXd>(k.data(), k.size());
  damping_   = Eigen::Map<const Eigen::VectorXd>(d.data(), d.size());

  auto el = node->get_parameter("effort_limits").as_double_array();
  if (el.empty()) {
    el.assign(n_joints_, std::numeric_limits<double>::infinity());
  } else if (el.size() != n_joints_) {
    RCLCPP_ERROR(node->get_logger(),
      "effort_limits must be empty or have %zu entries", n_joints_);
    return CallbackReturn::ERROR;
  }
  effort_limits_ = Eigen::Map<const Eigen::VectorXd>(el.data(), el.size());

  max_position_error_ = node->get_parameter("max_position_error").as_double();
  impedance_law_name_ = node->get_parameter("impedance_law").as_string();

  // Fetch robot_description from the controller manager's parameters.
  // ros2_control copies robot_description onto the CM node; read it with
  // a best-effort parameter client against the CM.
  auto rd_param_name = node->get_parameter("robot_description_param").as_string();
  robot_description_ = node->get_parameter_or<std::string>(rd_param_name, "");
  if (robot_description_.empty()) {
    // Fallback: ask the controller_manager directly.
    auto cm_client = node->create_client<rcl_interfaces::srv::GetParameters>(
      "/controller_manager/get_parameters");
    if (cm_client->wait_for_service(std::chrono::seconds(2))) {
      auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      req->names = {rd_param_name};
      auto fut = cm_client->async_send_request(req);
      if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto resp = fut.get();
        if (!resp->values.empty() && resp->values[0].type ==
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
          robot_description_ = resp->values[0].string_value;
        }
      }
    }
  }
  if (robot_description_.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "robot_description is empty; dynamics-based laws will likely fail.");
  }

  // Load the impedance law via pluginlib.
  try {
    law_loader_ = std::make_unique<pluginlib::ClassLoader<ImpedanceLaw>>(
      "sarax_impedance_controller", "sarax_impedance_controller::ImpedanceLaw");
    law_ = law_loader_->createSharedInstance(impedance_law_name_);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(node->get_logger(),
      "Failed to load impedance_law '%s': %s",
      impedance_law_name_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  if (!law_->initialize(robot_description_, joint_names_)) {
    RCLCPP_ERROR(node->get_logger(),
      "ImpedanceLaw '%s' failed to initialize", law_->name().c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node->get_logger(),
    "Configured with %zu joints and impedance_law='%s'",
    n_joints_, law_->name().c_str());

  // Working buffers.
  q_     = Eigen::VectorXd::Zero(n_joints_);
  q_dot_ = Eigen::VectorXd::Zero(n_joints_);
  q_d_     = Eigen::VectorXd::Zero(n_joints_);
  q_dot_d_ = Eigen::VectorXd::Zero(n_joints_);
  tau_   = Eigen::VectorXd::Zero(n_joints_);

  // Reference (q_d, q_dot_d) comes via a JointTrajectoryPoint topic.
  reference_sub_ = node->create_subscription<JointPoint>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](std::shared_ptr<JointPoint> msg) {
      reference_buffer_.writeFromNonRT(msg);
    });

  gains_sub_ = node->create_subscription<GainsMsg>(
    "~/gains", rclcpp::SystemDefaultsQoS(),
    [this](std::shared_ptr<GainsMsg> msg) {
      gains_buffer_.writeFromNonRT(msg);
    });

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // On activation, set q_d <- current q and q_dot_d <- 0 so the controller
  // holds the current configuration until someone sends a reference.
  for (size_t i = 0; i < n_joints_; ++i) {
    q_d_[static_cast<Eigen::Index>(i)] = state_interfaces_[i * 3 + 0].get_value();
    q_dot_d_[static_cast<Eigen::Index>(i)] = 0.0;
  }
  reference_buffer_.initRT(std::shared_ptr<JointPoint>());
  gains_buffer_.initRT(std::shared_ptr<GainsMsg>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Release all command interfaces to zero to avoid lingering torque.
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

// --------------------------------------------------------------------------
// Interface claims
// --------------------------------------------------------------------------

InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(n_joints_);
  for (const auto & j : joint_names_) {
    conf.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(n_joints_ * 3);
  for (const auto & j : joint_names_) {
    conf.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
    conf.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

// --------------------------------------------------------------------------
// update()
// --------------------------------------------------------------------------

return_type JointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read current state. State interfaces are stored in the same order the
  // controller claimed them: [q0, qd0, tau0, q1, qd1, tau1, ...].
  for (size_t i = 0; i < n_joints_; ++i) {
    q_    [static_cast<Eigen::Index>(i)] = state_interfaces_[i * 3 + 0].get_value();
    q_dot_[static_cast<Eigen::Index>(i)] = state_interfaces_[i * 3 + 1].get_value();
  }

  // Update reference from the last received JointTrajectoryPoint, if any.
  auto ref_msg = *reference_buffer_.readFromRT();
  if (ref_msg) {
    const auto & p = *ref_msg;
    if (p.positions.size() == n_joints_) {
      for (size_t i = 0; i < n_joints_; ++i) {
        q_d_[static_cast<Eigen::Index>(i)] = p.positions[i];
      }
    }
    if (p.velocities.size() == n_joints_) {
      for (size_t i = 0; i < n_joints_; ++i) {
        q_dot_d_[static_cast<Eigen::Index>(i)] = p.velocities[i];
      }
    } else {
      q_dot_d_.setZero();
    }
  }

  // Update gains from the last ImpedanceGains message, if any.
  auto gains_msg = *gains_buffer_.readFromRT();
  if (gains_msg &&
      gains_msg->stiffness.size() == n_joints_ &&
      gains_msg->damping.size() == n_joints_)
  {
    for (size_t i = 0; i < n_joints_; ++i) {
      stiffness_[static_cast<Eigen::Index>(i)] = gains_msg->stiffness[i];
      damping_  [static_cast<Eigen::Index>(i)] = gains_msg->damping[i];
    }
  }

  // Clamp position error before the Kp term is applied.
  Eigen::VectorXd q_err = q_ - q_d_;
  if (std::isfinite(max_position_error_)) {
    for (Eigen::Index i = 0; i < q_err.size(); ++i) {
      q_err[i] = std::clamp(q_err[i], -max_position_error_, max_position_error_);
    }
  }

  ImpedanceInput in;
  in.q         = q_;
  in.q_dot     = q_dot_;
  in.q_d       = q_ - q_err;   // effective q_d after clamping
  in.q_dot_d   = q_dot_d_;
  in.stiffness = stiffness_;
  in.damping   = damping_;

  tau_ = law_->compute(in);

  // Saturate to per-joint effort limits.
  for (Eigen::Index i = 0; i < tau_.size(); ++i) {
    if (std::isfinite(effort_limits_[i])) {
      tau_[i] = std::clamp(tau_[i], -effort_limits_[i], effort_limits_[i]);
    }
  }

  for (size_t i = 0; i < n_joints_; ++i) {
    command_interfaces_[i].set_value(tau_[static_cast<Eigen::Index>(i)]);
  }
  return return_type::OK;
}

}  // namespace sarax_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  sarax_impedance_controller::JointImpedanceController,
  controller_interface::ControllerInterface)
