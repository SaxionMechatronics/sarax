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

#ifndef _M4E_ROS_CONTROLLER_NODE_H_
#define _M4E_ROS_CONTROLLER_NODE_H_

#define POSITION_CONTROL 0
#define EFFORT_CONTROL 1

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/effort_joint_interface.h>


#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <tmotor_ak60_drivers_cpp_node.h>
#include <tmotor_ak60_drivers_cpp/MotorOutput.h>
#include <tmotor_ak60_drivers_cpp/TriggerMotor.h>
#include <tmotor_ak60_drivers_cpp/SetMotorVal.h>
#include <std_srvs/Trigger.h>


class M4ERosController : public hardware_interface::RobotHW
{
private:
    ros::NodeHandle nh;

    // ros::Publisher can_mag_pub = nh.advertise<can_msgs::Frame>("/sent_messages", 1000);
    ros::Subscriber motor_reading_sub = nh.subscribe("/received_messages", 1000, &M4ERosController::storeMotorValues, this);
    ros::Subscriber matlab_reading_sub = nh.subscribe<sensor_msgs::JointState>("/matlab_controller/commands", 1, &M4ERosController::receive_matlab_command, this);

    ros::Publisher can_mag_pub = nh.advertise<can_msgs::Frame>("/sent_messages", 1000);

    ros::ServiceServer enable_motor_service = nh.advertiseService("m4e_ros_controller/enableMotor", &M4ERosController::enterMotorMode, this);
    ros::ServiceServer disable_motor_service = nh.advertiseService("m4e_ros_controller/disableMotor", &M4ERosController::exitMotorMode, this);
    ros::ServiceServer set_motor_zero = nh.advertiseService("m4e_ros_controller/zeroMotor", &M4ERosController::zeroMotor, this);
    ros::ServiceServer set_motor_values = nh.advertiseService("m4e_ros_controller/setMotorValues", &M4ERosController::setMotorValues, this);

    ros::ServiceServer enable_all_motor_service = nh.advertiseService("m4e_ros_controller/enableAllMotors", &M4ERosController::enableAllMotors, this);
    ros::ServiceServer disable_all_motor_service = nh.advertiseService("m4e_ros_controller/disableAllMotors", &M4ERosController::disablelAllMotors, this);
    ros::ServiceServer set_motor_all_zero = nh.advertiseService("m4e_ros_controller/zeroAllMotors", &M4ERosController::zeroAllMotors, this);

    hardware_interface::JointStateInterface jnt_state_interface;
    
    hardware_interface::PositionJointInterface jnt_position_interface;
    hardware_interface::EffortJointInterface jnt_effort_interface;

    int control_type = POSITION_CONTROL;

    double cmd[3] = { 0 };
    double pos[3] = { 0 };
    double vel[3] = { 0 };
    double eff[3] = { 0 };

    double pos_reading[3] = { 0 };
    double vel_reading[3] = { 0 };
    double eff_reading[3] = { 0 };

    float motor_kp = 1;
    float motor_kd = 0;

    sensor_msgs::JointState joint_state_command; 
    tmotorAk60 base_motor = tmotorAk60(0x01, -2, 2);
    tmotorAk60 elbow_motor = tmotorAk60(0x02, -15.0, 15.0);
    tmotorAk60 end_effector_motor = tmotorAk60(0x03, -15.0, 15.0);
    
public:
    M4ERosController(int control_type, float i_motor_kp, float i_motor_kd){ 
        control_type = control_type;
        ROS_INFO("Control type: %d", control_type);
        motor_kp = i_motor_kp;
        motor_kd = i_motor_kd;
        
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("joint1", &pos_reading[0], &vel_reading[0], &eff_reading[0]);
        jnt_state_interface.registerHandle(state_handle_a);
        
        hardware_interface::JointStateHandle state_handle_b("joint2", &pos_reading[1], &vel_reading[1], &eff_reading[1]);
        jnt_state_interface.registerHandle(state_handle_b);

        hardware_interface::JointStateHandle state_handle_c("joint3", &pos_reading[2], &vel_reading[2], &eff_reading[2]);
        jnt_state_interface.registerHandle(state_handle_c);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        if(control_type == POSITION_CONTROL){
            hardware_interface::JointHandle pos_handle_a(state_handle_a, &pos[0]);
            jnt_position_interface.registerHandle(pos_handle_a);

            hardware_interface::JointHandle pos_handle_b(state_handle_b, &pos[1]);
            jnt_position_interface.registerHandle(pos_handle_b);

            hardware_interface::JointHandle pos_handle_c(state_handle_c, &pos[2]);
            jnt_position_interface.registerHandle(pos_handle_c);

            registerInterface(&jnt_position_interface);
        }

        else if(control_type == EFFORT_CONTROL){
            hardware_interface::JointHandle eff_handle_a(state_handle_a, &eff[0]);
            jnt_effort_interface.registerHandle(eff_handle_a);

            hardware_interface::JointHandle eff_handle_b(state_handle_b, &eff[1]);
            jnt_effort_interface.registerHandle(eff_handle_b);

            hardware_interface::JointHandle eff_handle_c(state_handle_c, &eff[2]);
            jnt_effort_interface.registerHandle(eff_handle_c);

            registerInterface(&jnt_effort_interface);
        }
    }

    void storeMotorValues(const can_msgs::Frame::ConstPtr& msg);
    void receive_matlab_command(const sensor_msgs::JointState::ConstPtr& msg);

    bool enterMotorMode(tmotor_ak60_drivers_cpp::TriggerMotor::Request  &req,
                        tmotor_ak60_drivers_cpp::TriggerMotor::Response &res);
    bool enableAllMotors(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
    bool exitMotorMode(tmotor_ak60_drivers_cpp::TriggerMotor::Request  &req,
                       tmotor_ak60_drivers_cpp::TriggerMotor::Response &res);
    bool disablelAllMotors(std_srvs::Trigger::Request  &req,
                    std_srvs::Trigger::Response &res);
    bool zeroMotor(tmotor_ak60_drivers_cpp::TriggerMotor::Request  &req,
                   tmotor_ak60_drivers_cpp::TriggerMotor::Response &res);
    bool zeroAllMotors(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);

    void sendMotorValues();

    bool setMotorValues(tmotor_ak60_drivers_cpp::SetMotorVal::Request  &req,
                    tmotor_ak60_drivers_cpp::SetMotorVal::Response &res);
};

#endif