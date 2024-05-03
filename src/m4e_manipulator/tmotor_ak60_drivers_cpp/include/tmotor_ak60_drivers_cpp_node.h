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

#ifndef _TMOTOR_AK60_DRIVERS_CPP_H_
#define _TMOTOR_AK60_DRIVERS_CPP_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "can_msgs/Frame.h"
#include "std_srvs/Empty.h"

#include "tmotor_ak60_drivers_cpp/MotorCommand.h"
#include "tmotor_ak60_drivers_cpp/MotorOutput.h"


//value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -15.0f
#define T_MAX 15.0f



class tmotorAk60{
    protected:
        int can_id = 0x01; 

        float p_in = 0.0;   //position
        float v_in = 0.0;   //velocity
        float kp_in = 1.0; //stiffness
        float kd_in = 0.0;  //dampening
        float t_in = 0.0f;  //torque

        float t_min = -15.0f;
        float t_max = 15.0f;
        
    public:
        tmotorAk60(int can_id_address, float t_minimum, float t_maximum) {
            can_id = can_id_address;
            t_min = t_minimum;
            t_max = t_maximum;
        };

        unsigned int float_to_uint(float x, float x_min, float x_max, int bits);
        float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
        float constrain(float x, float min, float max);

        void setMotorCommandCallback(const tmotor_ak60_drivers_cpp::MotorCommand motor_value);
        void unpackCanMsg(const can_msgs::Frame::ConstPtr& msg);

        can_msgs::Frame convertCanMessage(tmotor_ak60_drivers_cpp::MotorCommand motor_values);

        tmotor_ak60_drivers_cpp::MotorOutput decipherMessage(const can_msgs::Frame::ConstPtr& msg);

};

#endif