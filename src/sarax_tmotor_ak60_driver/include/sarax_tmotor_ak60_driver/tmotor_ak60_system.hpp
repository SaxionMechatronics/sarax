// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <limits>
#include <mutex>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include "sarax_tmotor_ak60_driver/ak60_protocol.hpp"

namespace sarax_tmotor_ak60_driver
{
// Owns the configured joints (one or two) and one SocketCAN descriptor.
// controller_manager calls lifecycle callbacks and read()/write().
class TmotorAk60System : public hardware_interface::SystemInterface
{
public:
  ~TmotorAk60System() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  friend class TmotorAk60SystemTest;  // Tests attach local Unix socket pairs, never CAN.
  using Clock = std::chrono::steady_clock;
  struct Joint
  {
    std::string name;
    Ak60Protocol motor;
    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double effort{std::numeric_limits<double>::quiet_NaN()};
    double command{0.0};
    bool has_feedback{false};
    Clock::time_point last_feedback{};
  };
  bool send_frame(const can_frame &) const;
  bool receive_feedback();
  bool stop_motors();
  void clear_measurements();
  hardware_interface::return_type fail(const char * reason);
  hardware_interface::CallbackReturn close_socket();

  std::vector<Joint> joints_;  // Sized once in on_init; exported pointers stay valid.
  std::mutex mutex_;          // Serializes lifecycle operations with CAN I/O.
  int socket_{-1};
  bool initialized_{false}, enabled_{false}, faulted_{false}, hardware_verified_{false};
  std::string can_interface_;
  double feedback_timeout_{0.1}, startup_timeout_{1.0};  // Seconds; verify against the feedback rate.
  Clock::time_point activation_start_{};
};
}  // namespace sarax_tmotor_ak60_driver
