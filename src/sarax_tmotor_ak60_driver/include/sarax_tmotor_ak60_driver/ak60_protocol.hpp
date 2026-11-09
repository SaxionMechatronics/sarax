// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <linux/can.h>

namespace sarax_tmotor_ak60_driver
{
// Owns one motor's address and software torque limit. No ROS or socket ownership.
class Ak60Protocol
{
public:
  enum class Operation : uint8_t { Enable = 0xFC, Disable = 0xFD, Zero = 0xFE };
  explicit Ak60Protocol(uint8_t motor_id = 1, float effort_limit = 2.0F);
  can_frame encode_command(float position, float velocity, float kp, float kd, float torque) const;
  can_frame special_command(Operation operation) const;
  bool decode_feedback(const can_frame &, float & position, float & velocity, float & effort) const;
  float effort_limit() const {return effort_limit_;}
  uint8_t motor_id() const {return motor_id_;}

private:
  static unsigned float_to_uint(float value, float minimum, float maximum, int bits);
  static float uint_to_float(unsigned value, float minimum, float maximum, int bits);
  uint8_t motor_id_;
  float effort_limit_;
};
}  // namespace sarax_tmotor_ak60_driver
