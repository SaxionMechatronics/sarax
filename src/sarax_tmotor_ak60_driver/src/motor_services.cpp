// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// SPDX-License-Identifier: BSD-3-Clause
#include "sarax_tmotor_ak60_driver/tmotor_ak60_system.hpp"

#include <utility>

namespace sarax_tmotor_ak60_driver
{
void TmotorAk60System::start_services()
{
  service_node_ = std::make_shared<rclcpp::Node>(
    info_.name, rclcpp::NodeOptions().use_global_arguments(false)
    .start_parameter_services(false).start_parameter_event_publisher(false));
  // Manager replies must run separately from the waiting Trigger callback.
  manager_callback_group_ = service_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  const auto manager = info_.hardware_parameters.find("controller_manager");
  const std::string manager_name = manager == info_.hardware_parameters.end() ?
    "/controller_manager" : manager->second;
  manager_client_ = service_node_->create_client<SetHardwareState>(
    manager_name + "/set_hardware_component_state", rmw_qos_profile_services_default,
    manager_callback_group_);
  using Operation = Ak60Protocol::Operation;
  for (const auto & entry : {
      std::make_pair("enableAllMotors", Operation::Enable),
      std::make_pair("disableAllMotors", Operation::Disable),
      std::make_pair("zeroAllMotors", Operation::Zero)})
  {
    motor_services_.push_back(service_node_->create_service<Trigger>(
      std::string("~/") + entry.first,
      [this, operation = entry.second](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
      {motor_service(operation, *response);}));
  }
  service_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(service_node_);
  services_stopping_ = false;
  service_thread_ = std::thread([this]() {
    try {
      while (!services_stopping_ && rclcpp::ok()) {
        service_executor_->spin_once(std::chrono::milliseconds(100));
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(service_node_->get_logger(), "Motor service executor: %s", error.what());
    }
  });
}

void TmotorAk60System::stop_services()
{
  services_stopping_ = true;
  if (service_thread_.joinable()) {service_thread_.join();}
  motor_services_.clear();
  service_executor_.reset();
  manager_client_.reset();
  manager_callback_group_.reset();
  service_node_.reset();
}

void TmotorAk60System::motor_service(
  Ak60Protocol::Operation operation, Trigger::Response & response)
{
  using Operation = Ak60Protocol::Operation;
  using State = lifecycle_msgs::msg::State;
  response.success = false;
  try {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (socket_ < 0) {
        response.message = "Configure the hardware component first; CAN is not open";
        return;
      }
      if (operation != Operation::Disable && faulted_) {
        response.message = "Hardware is faulted; recover through its lifecycle first";
        return;
      }
      if (operation == Operation::Zero) {
        // Changing the encoder origin during torque control changes its position error.
        if (enabled_ || get_state().id() != State::PRIMARY_STATE_INACTIVE) {
          response.message = "Disable the motors and leave the hardware inactive before zeroing";
          return;
        }
        std::string failed_ids;
        for (const auto & joint : joints_) {
          if (!send_frame(joint.motor.special_command(Operation::Zero))) {
            if (!failed_ids.empty()) {failed_ids += ", ";}
            failed_ids += std::to_string(joint.motor.motor_id());
          }
        }
        clear_measurements();
        response.success = failed_ids.empty();
        if (response.success) {
          response.message = "Zero commands sent for all configured motors; verify feedback";
        } else {
          faulted_ = true;
          response.message = "Zero transmission failed for motor IDs: " + failed_ids +
            "; partial zeroing is possible; recover through the lifecycle before enabling";
        }
        return;
      }
    }

    // Use the existing lifecycle so enabled_ and manager state stay consistent.
    if (!manager_client_->wait_for_service(std::chrono::seconds(1))) {
      response.message = "Controller manager's set_hardware_component_state service is unavailable";
      return;
    }
    auto request = std::make_shared<SetHardwareState::Request>();
    request->name = info_.name;
    request->target_state.id = operation == Operation::Enable ?
      State::PRIMARY_STATE_ACTIVE : State::PRIMARY_STATE_INACTIVE;
    rclcpp::executors::SingleThreadedExecutor replies;
    replies.add_callback_group(manager_callback_group_, service_node_->get_node_base_interface());
    auto future = manager_client_->async_send_request(request);
    if (replies.spin_until_future_complete(future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      manager_client_->remove_pending_request(future);
      response.message = "Lifecycle request not confirmed; it may still complete; check hardware state";
      return;
    }
    const auto result = future.get();
    response.success = result->ok && result->state.id == request->target_state.id;
    response.message = response.success ?
      (operation == Operation::Enable ? "Hardware active; configured motors enabled" :
      "Hardware inactive; configured motors disabled") :
      "Lifecycle transition failed; hardware state: " + result->state.label;
  } catch (const std::exception & error) {
    response.message = std::string("Motor service failed: ") + error.what();
  }
}
}  // namespace sarax_tmotor_ak60_driver
