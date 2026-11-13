// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// SPDX-License-Identifier: BSD-3-Clause
#include "sarax_tmotor_ak60_driver/tmotor_ak60_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <linux/can/raw.h>
#include <net/if.h>
#include <set>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace sarax_tmotor_ak60_driver
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
static const auto logger = rclcpp::get_logger("sarax_ak60");

static double number(const std::string & text)
{
  size_t used = 0;
  double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) {throw std::invalid_argument("Expected a finite number");}
  return value;
}

TmotorAk60System::~TmotorAk60System() {stop_services(); close_socket();}

CallbackReturn TmotorAk60System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (initialized_) {return CallbackReturn::ERROR;}
  try {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS ||
      info.joints.empty() || info.joints.size() > 2)
    {
      throw std::invalid_argument("Configure one or two joints");
    }
    can_interface_ = info.hardware_parameters.at("can_interface");
    const auto verified = info.hardware_parameters.at("hardware_verified");
    if (verified != "true" && verified != "false") {throw std::invalid_argument("hardware_verified must be true/false");}
    hardware_verified_ = verified == "true";
    if (can_interface_.empty() || can_interface_.size() >= IFNAMSIZ) {
      throw std::invalid_argument("Invalid CAN interface name");
    }
    const auto & parameters = info.hardware_parameters;
    feedback_timeout_ = number(parameters.count("feedback_timeout") ?
      parameters.at("feedback_timeout") : "0.1");
    startup_timeout_ = number(parameters.count("startup_timeout") ?
      parameters.at("startup_timeout") : "1.0");
    if (feedback_timeout_ <= 0 || startup_timeout_ <= 0) {
      throw std::invalid_argument("Feedback and startup timeouts must be positive finite seconds");
    }
    joints_.resize(info.joints.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto & joint = info.joints[i];
      std::set<std::string> states;
      for (const auto & state : joint.state_interfaces) {states.insert(state.name);}
      if (joint.name.empty() || joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != "effort" || joint.state_interfaces.size() != 3 ||
        states != std::set<std::string>{"position", "velocity", "effort"})
      {
        throw std::invalid_argument("Each joint needs effort command and position/velocity/effort states");
      }
      double id = number(joint.parameters.at("motor_id"));
      double limit = number(joint.parameters.at("effort_limit"));
      if (id < 1 || id > 255 || id != std::floor(id) || limit <= 0 || limit > 15) {
        throw std::invalid_argument("Invalid motor_id or effort_limit");
      }
      joints_[i].name = joint.name;
      joints_[i].motor = Ak60Protocol(static_cast<uint8_t>(id), static_cast<float>(limit));
    }
    if (joints_.size() == 2 && (joints_[0].name == joints_[1].name ||
      joints_[0].motor.motor_id() == joints_[1].motor.motor_id()))
    {
      throw std::invalid_argument("Joint names and motor IDs must be unique");
    }
    if (rclcpp::ok()) {start_services();}
    initialized_ = true;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> TmotorAk60System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  if (initialized_) {
    for (auto & joint : joints_) {
      result.emplace_back(joint.name, "position", &joint.position);
      result.emplace_back(joint.name, "velocity", &joint.velocity);
      result.emplace_back(joint.name, "effort", &joint.effort);
    }
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> TmotorAk60System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  if (initialized_) {
    for (auto & joint : joints_) {result.emplace_back(joint.name, "effort", &joint.command);}
  }
  return result;
}

void TmotorAk60System::clear_measurements()
{
  activation_start_ = {};
  for (auto & joint : joints_) {
    joint.position = joint.velocity = joint.effort = std::numeric_limits<double>::quiet_NaN();
    joint.has_feedback = false;
    joint.last_feedback = {};
    joint.command = 0.0;
  }
}

// Configure opens an existing interface; it neither enables nor zeros motors.
CallbackReturn TmotorAk60System::on_configure(const rclcpp_lifecycle::State &)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || socket_ >= 0 || !hardware_verified_) {
    RCLCPP_ERROR(logger, "Configure blocked: initialize and verify hardware first (see README)");
    return CallbackReturn::ERROR;
  }
  socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(::if_nametoindex(can_interface_.c_str()));
  const can_err_mask_t errors = CAN_ERR_MASK;
  const canid_t filter_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  std::vector<can_filter> filters{{0, filter_mask}};
  for (const auto & joint : joints_) {
    filters.push_back({joint.motor.motor_id(), filter_mask});
  }
  if (socket_ < 0 || address.can_ifindex == 0 ||
    ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) ||
    ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) ||
    ::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)))
  {
    close_socket();
    return CallbackReturn::ERROR;
  }
  // After a failed stop, first retry it on the reopened connection.
  if (!stop_motors()) {close_socket(); return CallbackReturn::ERROR;}
  clear_measurements();
  faulted_ = false;
  return CallbackReturn::SUCCESS;
}

bool TmotorAk60System::send_frame(const can_frame & frame) const
{
  return ::send(socket_, &frame, sizeof(frame), MSG_DONTWAIT | MSG_NOSIGNAL) == sizeof(frame);
}

bool TmotorAk60System::receive_feedback()
{
  for (unsigned count = 0; count < 64; ++count) {
    can_frame frame{};
    // Nonblocking receive. MSG_TRUNC also exposes oversized datagrams as invalid.
    auto bytes = ::recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT | MSG_TRUNC);
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {return true;}
    if (bytes != sizeof(frame) || (frame.can_id & CAN_ERR_FLAG)) {return false;}
    for (auto & joint : joints_) {
      float position, velocity, effort;
      if (joint.motor.decode_feedback(frame, position, velocity, effort)) {
        joint.position = position; joint.velocity = velocity; joint.effort = effort;
        joint.last_feedback = Clock::now();  // Processing time, not original frame arrival.
        joint.has_feedback = true;
      }
    }
  }
  return false;  // Bounded receive work; a saturated queue is a communication fault.
}

bool TmotorAk60System::stop_motors()
{
  if (!enabled_ || socket_ < 0) {return true;}
  bool success = true;
  for (auto & joint : joints_) {
    joint.command = 0.0;
    const bool zero_sent = send_frame(joint.motor.encode_command(0, 0, 0, 0, 0));
    const bool disable_sent = send_frame(joint.motor.special_command(Ak60Protocol::Operation::Disable));
    success = zero_sent && disable_sent && success;  // Still attempt every motor after a failure.
  }
  if (success) {enabled_ = false;}
  else {RCLCPP_ERROR(logger, "Stop transmission failed; motor state unknown");}
  return success;
}

return_type TmotorAk60System::fail(const char * reason)
{
  RCLCPP_ERROR(logger, "%s", reason);
  stop_motors();
  clear_measurements();
  faulted_ = true;
  return return_type::ERROR;
}

// Explicit activation enables the configured motors and sends zero effort once.
CallbackReturn TmotorAk60System::on_activate(const rclcpp_lifecycle::State &)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0 || enabled_ || faulted_) {return CallbackReturn::ERROR;}
  clear_measurements();
  activation_start_ = Clock::now();
  enabled_ = true;  // Even a partial enable must be followed by a stop attempt.
  for (auto & joint : joints_) {
    if (!send_frame(joint.motor.special_command(Ak60Protocol::Operation::Enable))) {
      fail("Enable transmission failed"); return CallbackReturn::ERROR;
    }
  }
  for (auto & joint : joints_) {
    if (!send_frame(joint.motor.encode_command(0, 0, 0, 0, 0))) {
      fail("Startup transmission failed"); return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn TmotorAk60System::on_deactivate(const rclcpp_lifecycle::State &)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const bool stopped = stop_motors();
  clear_measurements();
  if (!stopped) {faulted_ = true;}
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

// Cleanup/error/shutdown release CAN after a best-effort zero-effort + disable.
CallbackReturn TmotorAk60System::close_socket()
{
  const bool stopped = stop_motors();
  if (socket_ >= 0) {::close(socket_); socket_ = -1;}
  clear_measurements();
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn TmotorAk60System::on_cleanup(const rclcpp_lifecycle::State &)
{const std::lock_guard<std::mutex> lock(mutex_); return close_socket();}
CallbackReturn TmotorAk60System::on_error(const rclcpp_lifecycle::State & state)
{return on_cleanup(state);}
CallbackReturn TmotorAk60System::on_shutdown(const rclcpp_lifecycle::State & state)
{return on_cleanup(state);}

return_type TmotorAk60System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (faulted_) {return return_type::ERROR;}
  if (socket_ < 0) {return return_type::OK;}
  if (!receive_feedback()) {return fail("CAN receive failed");}
  return return_type::OK;
}

return_type TmotorAk60System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (faulted_) {return return_type::ERROR;}
  if (!enabled_) {return return_type::OK;}
  // Check the entire torque vector before clipping or transmitting its first frame.
  for (const auto & joint : joints_) {
    if (!std::isfinite(joint.command)) {return fail("Nonfinite effort command");}
  }
  // Until each configured motor has replied, keep soliciting feedback at zero effort.
  const auto now = Clock::now();
  bool all_have_feedback = true;
  for (const auto & joint : joints_) {
    if (!joint.has_feedback) {all_have_feedback = false;}
    // A silent motor must stop the group before more torque is sent.
    else if (std::chrono::duration<double>(now - joint.last_feedback).count() > feedback_timeout_) {
      return fail("Motor feedback timeout");
    }
  }
  if (!all_have_feedback &&
    std::chrono::duration<double>(now - activation_start_).count() > startup_timeout_)
  {
    return fail("Initial motor feedback timeout");
  }
  for (const auto & joint : joints_) {
    double limit = joint.motor.effort_limit();
    float torque = all_have_feedback ? static_cast<float>(std::clamp(joint.command, -limit, limit)) : 0.0F;
    // p_des=v_des=0; motor Kp=Kd=0 makes both correction terms inactive.
    if (!send_frame(joint.motor.encode_command(0, 0, 0, 0, torque))) {return fail("Partial effort transmission");}
  }
  return return_type::OK;
}
}  // namespace sarax_tmotor_ak60_driver

PLUGINLIB_EXPORT_CLASS(sarax_tmotor_ak60_driver::TmotorAk60System, hardware_interface::SystemInterface)
