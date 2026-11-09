// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <array>
#include <cmath>
#include <sys/socket.h>
#include <unistd.h>
#include <gtest/gtest.h>
#include "sarax_tmotor_ak60_driver/tmotor_ak60_system.hpp"

namespace sarax_tmotor_ak60_driver
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

TEST(Ak60Protocol, IndependentPacketBytesAndSpecialCommands)
{
  Ak60Protocol motor(7, 15);
  auto frame = motor.encode_command(6.25F, -25, 125, 2.5F, 7.5F);
  const std::array<uint8_t, 8> expected{0xBF, 0xFF, 0x3F, 0xF3, 0xFF, 0x7F, 0xFB, 0xFF};
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), frame.data));
  EXPECT_EQ(frame.can_id, 7U); EXPECT_EQ(frame.can_dlc, 8);
  for (auto operation : {Ak60Protocol::Operation::Enable, Ak60Protocol::Operation::Disable,
      Ak60Protocol::Operation::Zero})
  {
    frame = motor.special_command(operation);
    for (size_t i = 0; i < 7; ++i) {EXPECT_EQ(frame.data[i], 0xFF);}
    EXPECT_EQ(frame.data[7], static_cast<uint8_t>(operation));
  }
}

TEST(Ak60Protocol, LimitsKeepTheOriginalScaleAndNonfiniteInputsAreRejected)
{
  Ak60Protocol motor(1, 2);
  auto positive = motor.encode_command(0, 0, 0, 0, 99);
  auto negative = motor.encode_command(0, 0, 0, 0, -99);
  EXPECT_EQ(positive.data[6], 0x09); EXPECT_EQ(positive.data[7], 0x10); // +2: code 2320.
  EXPECT_EQ(negative.data[6], 0x06); EXPECT_EQ(negative.data[7], 0xEE); // -2: code 1774.
  for (float bad : {NAN, INFINITY, -INFINITY}) {
    for (size_t field = 0; field < 5; ++field) {
      std::array<float, 5> values{}; values[field] = bad;
      EXPECT_THROW(motor.encode_command(values[0], values[1], values[2], values[3], values[4]),
        std::invalid_argument);
    }
  }
  EXPECT_THROW(Ak60Protocol(0, 2), std::invalid_argument);
  EXPECT_THROW(Ak60Protocol(1, 16), std::invalid_argument);
}

TEST(Ak60Protocol, DecodesLegacyFeedbackAndRejectsMalformedFrames)
{
  Ak60Protocol motor(7, 2);
  can_frame frame{}; frame.can_id = 7; frame.can_dlc = 6;
  const uint8_t data[6]{7, 0x12, 0x34, 0xAB, 0xC7, 0x89};
  std::copy_n(data, 6, frame.data);
  float position, velocity, effort;
  ASSERT_TRUE(motor.decode_feedback(frame, position, velocity, effort));
  EXPECT_FLOAT_EQ(position, static_cast<float>(4660.0 * 25 / 65535 - 12.5));
  EXPECT_FLOAT_EQ(velocity, static_cast<float>(2748.0 * 100 / 4095 - 50));
  EXPECT_FLOAT_EQ(effort, static_cast<float>(1929.0 * 30 / 4095 - 15));
  for (auto flag : {CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG, 1U}) {
    auto bad = frame; bad.can_id ^= flag;
    EXPECT_FALSE(motor.decode_feedback(bad, position, velocity, effort));
  }
  for (int length : {0, 5, 7, 8}) {
    auto bad = frame; bad.can_dlc = length;
    EXPECT_FALSE(motor.decode_feedback(bad, position, velocity, effort));
  }
  frame.data[0] = 8;
  EXPECT_FALSE(motor.decode_feedback(frame, position, velocity, effort));
}

TEST(Ak60Protocol, DecodesCapturedMotorTwoReplyOnSharedCanIdZero)
{
  Ak60Protocol motor(2, 0.2F), other_motor(1, 0.2F);
  can_frame frame{}; frame.can_id = 0; frame.can_dlc = 6;
  const uint8_t data[6]{2, 0x84, 0x22, 0x7F, 0xF7, 0xFF};
  std::copy_n(data, 6, frame.data);
  float position, velocity, effort;
  ASSERT_TRUE(motor.decode_feedback(frame, position, velocity, effort));
  EXPECT_FLOAT_EQ(position, static_cast<float>(33826.0 * 25 / 65535 - 12.5));
  EXPECT_FLOAT_EQ(velocity, static_cast<float>(2047.0 * 100 / 4095 - 50));
  EXPECT_FLOAT_EQ(effort, static_cast<float>(2047.0 * 30 / 4095 - 15));
  EXPECT_FALSE(other_motor.decode_feedback(frame, position, velocity, effort));
  for (auto address : {CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG, 1U, 3U}) {
    auto bad = frame; bad.can_id = address;
    EXPECT_FALSE(motor.decode_feedback(bad, position, velocity, effort));
  }
  for (int length : {0, 5, 7, 8}) {
    auto bad = frame; bad.can_dlc = length;
    EXPECT_FALSE(motor.decode_feedback(bad, position, velocity, effort));
  }
}

// No transport class or alternate runtime backend: use the same send/recv
// calls on connected AF_UNIX datagrams. Only this friend fixture attaches FDs.
class TmotorAk60SystemTest : public ::testing::Test
{
protected:
  using Clock = std::chrono::steady_clock;
  TmotorAk60System system;
  int peer{-1};
  rclcpp_lifecycle::State previous;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
  hardware_interface::HardwareInfo info(size_t motor_count = 2)
  {
    hardware_interface::HardwareInfo result;
    result.hardware_parameters = {{"can_interface", "NEVER_OPENED"}, {"hardware_verified", "false"},
      {"feedback_timeout", "10"}, {"startup_timeout", "20"}};
    for (size_t id = 1; id <= motor_count; ++id) {
      hardware_interface::ComponentInfo joint;
      joint.name = "mani_joint_" + std::to_string(id);
      joint.parameters = {{"motor_id", std::to_string(id)}, {"effort_limit", id == 1 ? "2" : "15"}};
      hardware_interface::InterfaceInfo interface;
      interface.name = "effort"; joint.command_interfaces.push_back(interface);
      for (const char * name : {"position", "velocity", "effort"}) {
        interface.name = name; joint.state_interfaces.push_back(interface);
      }
      result.joints.push_back(joint);
    }
    return result;
  }
  void attach_socket()
  {
    if (peer >= 0) {::close(peer);}
    int sockets[2];
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, sockets), 0);
    system.socket_ = sockets[0]; peer = sockets[1];
    system.faulted_ = false;
  }
  void initialize(size_t motor_count = 2)
  {
    ASSERT_EQ(system.on_init(info(motor_count)), CallbackReturn::SUCCESS);
    expect_timeouts(10, 20);
    states = system.export_state_interfaces(); commands = system.export_command_interfaces();
    attach_socket();
  }
  void TearDown() override {system.on_cleanup(previous); if (peer >= 0) {::close(peer);}}
  std::vector<can_frame> packets()
  {
    std::vector<can_frame> result;
    can_frame frame{};
    while (::recv(peer, &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame)) {result.push_back(frame);}
    return result;
  }
  void reply(uint8_t id)
  {
    can_frame frame{}; frame.can_id = 0; frame.can_dlc = 6;
    const uint8_t data[6]{id, 0x90, 0, 0xA0, 0x06, 0};
    std::copy_n(data, 6, frame.data);
    ::send(peer, &frame, sizeof(frame), MSG_DONTWAIT | MSG_NOSIGNAL);
  }
  void activate()
  {
    ASSERT_EQ(system.on_activate(previous), CallbackReturn::SUCCESS);
    for (const auto & frame : packets()) {
      if (frame.data[0] != 0xFF) {reply(frame.can_id);}
    }
    ASSERT_EQ(read(), return_type::OK);
  }
  return_type read() {return system.read(rclcpp::Time(0), rclcpp::Duration(0, 4000000));}
  return_type write() {return system.write(rclcpp::Time(0), rclcpp::Duration(0, 4000000));}
  void expect_timeouts(double feedback, double startup)
  {
    EXPECT_DOUBLE_EQ(system.feedback_timeout_, feedback);
    EXPECT_DOUBLE_EQ(system.startup_timeout_, startup);
  }
  void expire_startup()
  {
    system.activation_start_ -= std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(system.startup_timeout_ + 1.0));
  }
  void expire_feedback(size_t index)
  {
    system.joints_.at(index).last_feedback -= std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(system.feedback_timeout_ + 1.0));
  }
  void expect_feedback_reset()
  {
    EXPECT_EQ(system.activation_start_, Clock::time_point{});
    for (const auto & joint : system.joints_) {
      EXPECT_FALSE(joint.has_feedback);
      EXPECT_EQ(joint.last_feedback, Clock::time_point{});
    }
  }
  void expect_stop(size_t motor_count)
  {
    const auto stopped = packets(); ASSERT_EQ(stopped.size(), motor_count * 2);
    for (size_t i = 0; i < motor_count; ++i) {
      EXPECT_EQ(stopped[2 * i].can_id, i + 1);
      EXPECT_EQ(stopped[2 * i].data[6], 0x07); EXPECT_EQ(stopped[2 * i].data[7], 0xFF);
      EXPECT_EQ(stopped[2 * i + 1].can_id, i + 1);
      EXPECT_EQ(stopped[2 * i + 1].data[7], 0xFD);
    }
    expect_feedback_reset();
    EXPECT_EQ(write(), return_type::ERROR); EXPECT_TRUE(packets().empty());
  }
  void fill_send_queue()
  {
    int bytes = 4096;
    ASSERT_EQ(::setsockopt(system.socket_, SOL_SOCKET, SO_SNDBUF, &bytes, sizeof(bytes)), 0);
    can_frame filler{}; filler.can_id = 99;
    while (system.send_frame(filler)) {}
    can_frame dropped{};
    ASSERT_EQ(::recv(peer, &dropped, sizeof(dropped), MSG_DONTWAIT), sizeof(dropped));
  }
};

TEST_F(TmotorAk60SystemTest, ValidatesConfigurationAndBlocksUnverifiedHardware)
{
  initialize();
  TmotorAk60System blocked;
  ASSERT_EQ(blocked.on_init(info()), CallbackReturn::SUCCESS);
  EXPECT_EQ(blocked.on_configure(previous), CallbackReturn::ERROR); // No CAN open.
  auto invalid = info(); invalid.joints[0].parameters.erase("effort_limit");
  TmotorAk60System missing; EXPECT_EQ(missing.on_init(invalid), CallbackReturn::ERROR);
  invalid = info(); invalid.joints[1].parameters["motor_id"] = "256";
  TmotorAk60System bad_id; EXPECT_EQ(bad_id.on_init(invalid), CallbackReturn::ERROR);
  TmotorAk60System no_motors; EXPECT_EQ(no_motors.on_init(info(0)), CallbackReturn::ERROR);
  TmotorAk60System too_many; EXPECT_EQ(too_many.on_init(info(3)), CallbackReturn::ERROR);
  invalid = info(); invalid.joints[1].parameters["motor_id"] = "1";
  TmotorAk60System duplicate; EXPECT_EQ(duplicate.on_init(invalid), CallbackReturn::ERROR);
  for (const auto & state : states) {EXPECT_TRUE(std::isnan(state.get_value()));}
}

TEST_F(TmotorAk60SystemTest, StartupMappingEffortAndPersistentInterfaces)
{
  initialize();
  for (int cycle = 0; cycle < 2; ++cycle) {
    activate();
    EXPECT_FLOAT_EQ(states[0].get_value(), static_cast<float>(36864.0 * 25 / 65535 - 12.5));
    commands[0].set_value(99); commands[1].set_value(-99);
    ASSERT_EQ(write(), return_type::OK);
    auto sent = packets(); ASSERT_EQ(sent.size(), 2U);
    EXPECT_EQ(sent[0].can_id, 1U); EXPECT_EQ(sent[1].can_id, 2U);
    EXPECT_EQ(sent[0].data[6], 0x09); EXPECT_EQ(sent[0].data[7], 0x10);
    EXPECT_EQ(sent[1].data[6], 0); EXPECT_EQ(sent[1].data[7], 0);
    EXPECT_EQ(sent[0].data[3], 0xF0); EXPECT_EQ(sent[0].data[4], 0); EXPECT_EQ(sent[0].data[5], 0);
    ASSERT_EQ(system.on_deactivate(previous), CallbackReturn::SUCCESS);
    sent = packets(); ASSERT_EQ(sent.size(), 4U);
    EXPECT_EQ(sent[1].data[7], 0xFD); EXPECT_EQ(sent[3].data[7], 0xFD);
    EXPECT_TRUE(std::isnan(states[0].get_value()));
    expect_feedback_reset();
    ASSERT_EQ(system.on_cleanup(previous), CallbackReturn::SUCCESS);
    expect_feedback_reset();
    attach_socket(); // Old exported handles remain valid across reconnection.
  }
}

TEST_F(TmotorAk60SystemTest, RejectsEntireNonfiniteBatchBeforeNormalTransmission)
{
  initialize();
  for (double bad : {NAN, INFINITY, -INFINITY}) {
    activate(); commands[0].set_value(1); commands[1].set_value(bad);
    EXPECT_EQ(write(), return_type::ERROR);
    auto stopped = packets(); ASSERT_EQ(stopped.size(), 4U);
    EXPECT_EQ(stopped[0].data[6], 0x07); EXPECT_EQ(stopped[0].data[7], 0xFF); // Only zero effort.
    EXPECT_EQ(stopped[1].data[7], 0xFD); EXPECT_EQ(stopped[3].data[7], 0xFD);
    EXPECT_EQ(write(), return_type::ERROR); EXPECT_TRUE(packets().empty());
    system.on_cleanup(previous); attach_socket();
  }
}

TEST_F(TmotorAk60SystemTest, KeepsTheLatestStateWhenNoNewReplyArrives)
{
  initialize(); activate();
  expire_startup(); // The startup deadline no longer applies once both motors replied.
  const double last_position = states[0].get_value();
  EXPECT_EQ(read(), return_type::OK);
  EXPECT_EQ(states[0].get_value(), last_position);
  commands[0].set_value(1);
  EXPECT_EQ(write(), return_type::OK);
  auto sent = packets(); ASSERT_EQ(sent.size(), 2U);
  EXPECT_EQ(sent[0].data[6], 0x08); EXPECT_EQ(sent[0].data[7], 0x88);
}

TEST_F(TmotorAk60SystemTest, TimeoutParametersAcceptDefaultsAndRejectInvalidValues)
{
  auto configuration = info();
  configuration.hardware_parameters.erase("feedback_timeout");
  configuration.hardware_parameters.erase("startup_timeout");
  ASSERT_EQ(system.on_init(configuration), CallbackReturn::SUCCESS);
  expect_timeouts(0.1, 1.0);
  for (const char * parameter : {"feedback_timeout", "startup_timeout"}) {
    for (const char * bad : {"0", "-1", "nan", "inf", "-inf", "", "0.1junk", "1e309"}) {
      configuration = info(); configuration.hardware_parameters[parameter] = bad;
      TmotorAk60System invalid;
      EXPECT_EQ(invalid.on_init(configuration), CallbackReturn::ERROR) << parameter << "=" << bad;
    }
  }
}

TEST_F(TmotorAk60SystemTest, MissingInitialFeedbackStopsTheSingleMotor)
{
  initialize(1);
  ASSERT_EQ(system.on_activate(previous), CallbackReturn::SUCCESS);
  packets();
  commands[0].set_value(1);
  ASSERT_EQ(write(), return_type::OK);
  auto sent = packets(); ASSERT_EQ(sent.size(), 1U);
  EXPECT_EQ(sent[0].data[6], 0x07); EXPECT_EQ(sent[0].data[7], 0xFF);
  expire_startup();
  EXPECT_EQ(write(), return_type::ERROR);
  expect_stop(1);
}

TEST_F(TmotorAk60SystemTest, OneMotorNeverReplyingStillExpiresStartup)
{
  initialize();
  ASSERT_EQ(system.on_activate(previous), CallbackReturn::SUCCESS);
  packets(); expire_startup();
  reply(1); ASSERT_EQ(read(), return_type::OK);
  commands[0].set_value(1); commands[1].set_value(1);
  EXPECT_EQ(write(), return_type::ERROR);
  expect_stop(2);
}

TEST_F(TmotorAk60SystemTest, FeedbackLossAfterStartupStopsDespiteInvalidReplies)
{
  initialize(1); activate();
  commands[0].set_value(1);
  ASSERT_EQ(write(), return_type::OK); packets();
  expire_feedback(0);
  can_frame invalid{}; invalid.can_id = 1; invalid.can_dlc = 8; invalid.data[0] = 1;
  ASSERT_EQ(::send(peer, &invalid, sizeof(invalid), MSG_DONTWAIT), sizeof(invalid));
  ASSERT_EQ(read(), return_type::OK);
  EXPECT_EQ(write(), return_type::ERROR);
  expect_stop(1);
}

TEST_F(TmotorAk60SystemTest, RepliesCannotRefreshTheOtherMotorsFeedbackTime)
{
  initialize();
  for (size_t silent : {0U, 1U}) {
    activate();
    commands[0].set_value(1); commands[1].set_value(1);
    expire_feedback(silent);
    reply(static_cast<uint8_t>(2 - silent)); ASSERT_EQ(read(), return_type::OK);
    EXPECT_EQ(write(), return_type::ERROR);
    expect_stop(2);
    ASSERT_EQ(system.on_cleanup(previous), CallbackReturn::SUCCESS); attach_socket();
  }
}

TEST_F(TmotorAk60SystemTest, SendsZeroEffortUntilEveryConfiguredMotorHasReplied)
{
  initialize();
  ASSERT_EQ(system.on_activate(previous), CallbackReturn::SUCCESS);
  auto sent = packets(); ASSERT_EQ(sent.size(), 4U);
  EXPECT_EQ(sent[0].data[7], 0xFC); EXPECT_EQ(sent[1].data[7], 0xFC);
  commands[0].set_value(1); commands[1].set_value(1);
  reply(1); ASSERT_EQ(read(), return_type::OK);
  EXPECT_EQ(write(), return_type::OK);
  sent = packets(); ASSERT_EQ(sent.size(), 2U);
  for (const auto & frame : sent) {EXPECT_EQ(frame.data[6], 0x07); EXPECT_EQ(frame.data[7], 0xFF);}
  reply(2); ASSERT_EQ(read(), return_type::OK);
  EXPECT_EQ(write(), return_type::OK);
  sent = packets(); ASSERT_EQ(sent.size(), 2U);
  for (const auto & frame : sent) {EXPECT_EQ(frame.data[6], 0x08); EXPECT_EQ(frame.data[7], 0x88);}
}

TEST_F(TmotorAk60SystemTest, OneMotorExportsAndCommandsOnlyItsConfiguredJoint)
{
  initialize(1);
  ASSERT_EQ(states.size(), 3U); ASSERT_EQ(commands.size(), 1U);
  ASSERT_EQ(system.on_activate(previous), CallbackReturn::SUCCESS);
  auto sent = packets(); ASSERT_EQ(sent.size(), 2U);
  for (const auto & frame : sent) {EXPECT_EQ(frame.can_id, 1U);}
  reply(2); ASSERT_EQ(read(), return_type::OK); // The other motor is not configured.
  EXPECT_TRUE(std::isnan(states[0].get_value()));
  reply(1); ASSERT_EQ(read(), return_type::OK);
  commands[0].set_value(1);
  ASSERT_EQ(write(), return_type::OK);
  sent = packets(); ASSERT_EQ(sent.size(), 1U);
  EXPECT_EQ(sent[0].can_id, 1U);
  EXPECT_EQ(sent[0].data[6], 0x08); EXPECT_EQ(sent[0].data[7], 0x88);
  ASSERT_EQ(system.on_deactivate(previous), CallbackReturn::SUCCESS);
  sent = packets(); ASSERT_EQ(sent.size(), 2U);
  EXPECT_EQ(sent[1].can_id, 1U); EXPECT_EQ(sent[1].data[7], 0xFD);
}

TEST_F(TmotorAk60SystemTest, MalformedReceiveAndSocketFailureStopTheGroup)
{
  initialize();
  activate();
  const char incomplete = 0;
  ASSERT_EQ(::send(peer, &incomplete, 1, MSG_DONTWAIT), 1);
  EXPECT_EQ(read(), return_type::ERROR); EXPECT_EQ(packets().size(), 4U);
  system.on_cleanup(previous); attach_socket(); activate();
  ::close(peer); peer = -1;
  EXPECT_EQ(write(), return_type::ERROR);
  EXPECT_EQ(system.on_error(previous), CallbackReturn::ERROR); // A stop cannot be delivered.
}

TEST_F(TmotorAk60SystemTest, PartialSendFaultsAndCleanupRetriesTheStop)
{
  initialize();
  activate(); fill_send_queue(); commands[0].set_value(1); commands[1].set_value(1);
  EXPECT_EQ(write(), return_type::ERROR);
  auto sent = packets();
  EXPECT_EQ(std::count_if(sent.begin(), sent.end(), [](const can_frame & f) {return f.can_id == 1;}), 1);
  EXPECT_EQ(std::count_if(sent.begin(), sent.end(), [](const can_frame & f) {return f.can_id == 2;}), 0);
  EXPECT_EQ(system.on_error(previous), CallbackReturn::SUCCESS);
  auto stopped = packets(); ASSERT_EQ(stopped.size(), 4U);
  EXPECT_EQ(stopped[1].data[7], 0xFD); EXPECT_EQ(stopped[3].data[7], 0xFD);
}
}  // namespace sarax_tmotor_ak60_driver
