// BSD 3-Clause License
// Copyright (c) 2024 SMART Research Group - Saxion University of Applied Sciences
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include<tmotor_ak60_drivers_cpp_node.h>

can_msgs::Frame tmotorAk60::convertCanMessage(tmotor_ak60_drivers_cpp::MotorCommand motor_values){
  can_msgs::Frame can_msg;
  // 0: position[15-8]]
  // 1: position[[7-0]]
  // 2: velocity[[11-4]
  // 3: velocity[[3-0], kp[11-8]
  // 4: [kp[7-0]]
  // 5: [kd[11-4]
  // 6: [kd[3-4], torque[11-8]
  // 7: [torque[7-0]]

  float p_des = constrain(motor_values.p_in, P_MIN, P_MAX);
  float v_des = constrain(motor_values.v_in, V_MIN, V_MAX);
  float kp = constrain(motor_values.kp_in, KP_MIN, KP_MAX);
  float kd = constrain(motor_values.kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(motor_values.t_in, t_min, t_max); 

  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);  
  unsigned int kp_int = float_to_uint(kd, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kp, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  
  can_msg.id = can_id;
  can_msg.dlc = 8;

  can_msg.data[0] = p_int >> 8;
  can_msg.data[1] = p_int & 0xFF;
  can_msg.data[2] = v_int >> 4;
  can_msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  can_msg.data[4] = kp_int & 0xFF;
  can_msg.data[5] = kd_int >> 4;
  can_msg.data[6] = ((kd_int&0xF)<<4)|(t_int>>8); 
  can_msg.data[7] = t_int & 0xFF;
  
  return can_msg;
}


tmotor_ak60_drivers_cpp::MotorOutput tmotorAk60::decipherMessage(const can_msgs::Frame::ConstPtr& msg){
  tmotor_ak60_drivers_cpp::MotorOutput motor_output;
  if(msg->is_error){
    ROS_ERROR("MESSAGE HAS AN ERROR");
  }

  int id = msg->id;
  unsigned int p_int =  (msg->data[1]<<8)|msg->data[2];
  unsigned int v_int = (msg->data[3] << 4 | (msg->data[4] >> 4));
  unsigned int i_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];

  float p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  motor_output.p_out = p_out;
  motor_output.v_out = v_out;
  motor_output.t_out = t_out;
  motor_output.can_id = can_id; //this can be removed?

  return motor_output;
}

unsigned int tmotorAk60::float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;

  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  } 
  if (bits == 16){
    pgg = (unsigned int) ((x - offset)*65535.0/span);
  }
  return pgg;
}

float tmotorAk60::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if(bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if(bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}

float tmotorAk60::constrain(float x, float min, float max) {
  if(x < min) {
    return min;
  }
  else if(max < x) {
    return max;
  }
  else
    return x;
}

//TODO REMOVE THIS FUNCTION IF ARCHITECTURE CHANGES
void tmotorAk60::setMotorCommandCallback(const tmotor_ak60_drivers_cpp::MotorCommand motor_value)
{
  p_in = motor_value.p_in;
  v_in = motor_value.v_in;
  kp_in = motor_value.kp_in;
  kd_in = motor_value.kd_in;
  t_in = motor_value.t_in;
}