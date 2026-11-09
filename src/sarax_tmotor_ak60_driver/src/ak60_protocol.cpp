// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// SPDX-License-Identifier: BSD-3-Clause
#include "sarax_tmotor_ak60_driver/ak60_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sarax_tmotor_ak60_driver
{
Ak60Protocol::Ak60Protocol(uint8_t motor_id, float effort_limit)
: motor_id_(motor_id), effort_limit_(effort_limit)
{
  if (motor_id == 0 || !std::isfinite(effort_limit) || effort_limit <= 0 || effort_limit > 15) {
    throw std::invalid_argument("AK60 needs motor ID 1..255 and effort_limit in (0,15] N.m");
  }
}

unsigned Ak60Protocol::float_to_uint(float value, float minimum, float maximum, int bits)
{
  const float span = maximum - minimum;
  // Retain ROS 1's float subtraction, double multiplier and truncating cast.
  return static_cast<unsigned>((value - minimum) * (bits == 16 ? 65535.0 : 4095.0) / span);
}

float Ak60Protocol::uint_to_float(unsigned value, float minimum, float maximum, int bits)
{
  const float span = maximum - minimum;
  return static_cast<float>(static_cast<float>(value) * span /
    (bits == 16 ? 65535.0 : 4095.0) + minimum);
}

can_frame Ak60Protocol::encode_command(float position, float velocity, float kp, float kd, float torque) const
{
  for (float value : {position, velocity, kp, kd, torque}) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Nonfinite AK60 command");}
  }
  const auto p = float_to_uint(std::clamp(position, -12.5F, 12.5F), -12.5F, 12.5F, 16);
  const auto v = float_to_uint(std::clamp(velocity, -50.0F, 50.0F), -50.0F, 50.0F, 12);
  // Small correction to ROS 1: encode kp as kp and kd as kd, not swapped.
  const auto k_p = float_to_uint(std::clamp(kp, 0.0F, 500.0F), 0.0F, 500.0F, 12);
  const auto k_d = float_to_uint(std::clamp(kd, 0.0F, 5.0F), 0.0F, 5.0F, 12);
  const auto t = float_to_uint(std::clamp(torque, -effort_limit_, effort_limit_), -15.0F, 15.0F, 12);
  can_frame frame{};
  frame.can_id = motor_id_;
  frame.can_dlc = 8;
  frame.data[0] = p >> 8;                         // Position high byte.
  frame.data[1] = p & 0xFF;                       // Position low byte.
  frame.data[2] = v >> 4;                         // Velocity high 8 bits.
  frame.data[3] = ((v & 0xF) << 4) | (k_p >> 8);  // Velocity low 4, Kp high 4.
  frame.data[4] = k_p & 0xFF;                     // Kp low 8 bits.
  frame.data[5] = k_d >> 4;                       // Kd high 8 bits.
  frame.data[6] = ((k_d & 0xF) << 4) | (t >> 8);  // Kd low 4, torque high 4.
  frame.data[7] = t & 0xFF;                       // Torque low 8 bits.
  return frame;
}

can_frame Ak60Protocol::special_command(Operation operation) const
{
  const auto code = static_cast<uint8_t>(operation);
  if (code < 0xFC || code > 0xFE) {throw std::invalid_argument("Unknown AK60 operation");}
  can_frame frame{};
  frame.can_id = motor_id_;
  frame.can_dlc = 8;
  std::fill_n(frame.data, 7, 0xFF);
  frame.data[7] = code;
  return frame;
}

bool Ak60Protocol::decode_feedback(
  const can_frame & frame, float & position, float & velocity, float & effort) const
{
  // Some firmware shares CAN ID 0; payload byte 0 still identifies each motor.
  if ((frame.can_id != 0 && frame.can_id != motor_id_) ||
    frame.can_dlc != 6 || frame.data[0] != motor_id_) {return false;}
  const unsigned p = (frame.data[1] << 8) | frame.data[2];
  const unsigned v = (frame.data[3] << 4) | (frame.data[4] >> 4);
  const unsigned t = ((frame.data[4] & 0xF) << 8) | frame.data[5];
  position = uint_to_float(p, -12.5F, 12.5F, 16);
  velocity = uint_to_float(v, -50.0F, 50.0F, 12);
  effort = uint_to_float(t, -15.0F, 15.0F, 12);
  return true;
}
}  // namespace sarax_tmotor_ak60_driver
