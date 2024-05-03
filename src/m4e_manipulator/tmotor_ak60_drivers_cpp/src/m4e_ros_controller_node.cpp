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

#include "m4e_ros_controller_node.h"
#include <thread>

void control(ros::Rate rate, M4ERosController* controller, controller_manager::ControllerManager* cm){
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    while (ros::ok())
    {
        std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_duration = this_time - last_time;
        ros::Duration elapsed(elapsed_duration.count());
        last_time = this_time;

        cm->update(ros::Time::now(), elapsed);
        controller->sendMotorValues();
        rate.sleep();
    }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "m4e_ros_controller_node");
    ros::NodeHandle controller_nh("");
    int control_type = POSITION_CONTROL;
    float tmp_kp = 0.0f;
    float tmp_kd = 1.0f;

    if(controller_nh.getParam("/m4e_mani/m4e_ros_controller_node/control_type",control_type)){
        if(control_type == POSITION_CONTROL){
            ROS_INFO("Controller set to position control");
            tmp_kp = 0.4f;
            tmp_kd = 1.0f;
        }
        else if(control_type == EFFORT_CONTROL){
            ROS_INFO("Controller set to effort control");
            tmp_kp = 0.0f;
            tmp_kd = 0.0f;
        }
    }
    else{
        ROS_WARN("Unable to find control type parameter, defaulting to position control");
    }

    M4ERosController controller(control_type, tmp_kp, tmp_kd);

    controller_manager::ControllerManager cm(&controller, controller_nh);
    std::thread control_thread(&control, ros::Rate(250), &controller, &cm);
    
    ros::spin();
    return 0;
}

void M4ERosController::sendMotorValues(){
    tmotor_ak60_drivers_cpp::MotorCommand motor_values;
    motor_values.kp_in = motor_kp;
    motor_values.kd_in = motor_kd;

    for(int i = 0; i < 3; i++){
        motor_values.p_in = pos[i];
        motor_values.v_in = vel[i];
        motor_values.t_in = eff[i];

        if(i == 0){
            can_msgs::Frame can_msg = base_motor.convertCanMessage(motor_values);
            can_mag_pub.publish(can_msg);
        }
        else if(i == 1){
            can_msgs::Frame can_msg = elbow_motor.convertCanMessage(motor_values);
            can_mag_pub.publish(can_msg);
        }
        else if(i == 2){
            can_msgs::Frame can_msg = end_effector_motor.convertCanMessage(motor_values);
            can_mag_pub.publish(can_msg);
        }
    }
}

void M4ERosController::storeMotorValues(const can_msgs::Frame::ConstPtr& msg){
    //todo make a list of the can devices and check if the input is coming from the correct motor.
    tmotor_ak60_drivers_cpp::MotorOutput motor_msg;

    switch (msg->data[0])
    {
        //base_motor
        case 0x01:
            motor_msg = base_motor.decipherMessage(msg);
            pos_reading[0] = motor_msg.p_out;
            vel_reading[0] = motor_msg.v_out;
            eff_reading[0] = motor_msg.t_out;
            break;
        
        // //elbow_motor
        case 0x02:
            motor_msg = elbow_motor.decipherMessage(msg);
            pos_reading[1] = motor_msg.p_out;
            vel_reading[1] = motor_msg.v_out;
            eff_reading[1] = motor_msg.t_out;
            break;

        // //end_effector
        case 0x03:
            motor_msg = end_effector_motor.decipherMessage(msg);

            pos_reading[2] = motor_msg.p_out;
            vel_reading[2] = motor_msg.v_out;
            eff_reading[2] = motor_msg.t_out;
            break;

        default:
            ROS_WARN("Incoming message from an unknown motor");
            break;
    }
}

bool M4ERosController::setMotorValues(tmotor_ak60_drivers_cpp::SetMotorVal::Request  &req,
                    tmotor_ak60_drivers_cpp::SetMotorVal::Response &res){
    motor_kp = req.kp_in;
    motor_kd = req.kd_in;

    res.result = true;
    return true;

}

void M4ERosController::receive_matlab_command(const sensor_msgs::JointState::ConstPtr& msg){
    joint_state_command.name = msg->name;
    
    joint_state_command.position = msg->position;
    joint_state_command.velocity = msg->velocity;   
    joint_state_command.effort = msg->effort;
}

bool M4ERosController::enableAllMotors(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res){
    can_msgs::Frame can_msg;

    can_msg.header.stamp = ros::Time::now();
    can_msg.id = 1;     //start with base
    can_msg.dlc = 8;

    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = 0xFC;


    ROS_INFO("Motors enabled");
    can_mag_pub.publish(can_msg);

    ros::spinOnce();

    can_msg.id = 2;
    can_mag_pub.publish(can_msg);

    ros::spinOnce();

    can_msg.id = 3;
    can_mag_pub.publish(can_msg);

    res.success = true;
    res.message = "Motors enabled";

    return true;
}

bool M4ERosController::disablelAllMotors(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res){
    can_msgs::Frame can_msg;

    can_msg.header.stamp = ros::Time::now();
    can_msg.id = 1;     //start with base
    can_msg.dlc = 8;

    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = 0xFD;


    ROS_INFO("Motors disabled");
    can_mag_pub.publish(can_msg);

    ros::spinOnce();

    can_msg.id = 2;
    can_mag_pub.publish(can_msg);

    ros::spinOnce();

    can_msg.id = 3;
    can_mag_pub.publish(can_msg);

    res.success = true;
    res.message = "Motors disabled";

    return true;
}

bool M4ERosController::zeroAllMotors(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res){
    can_msgs::Frame can_msg;

    can_msg.header.stamp = ros::Time::now();
    can_msg.id = 1;     //start with base
    can_msg.dlc = 8;

    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = 0xFE;


    ROS_INFO("Motors zeroed");
    can_mag_pub.publish(can_msg);

    ros::spinOnce();

    can_msg.id = 2;
    can_mag_pub.publish(can_msg);

    ros::spinOnce();

    can_msg.id = 3;
    can_mag_pub.publish(can_msg);

    res.success = true;
    res.message = "Motors zeroed";

    return true;
}

bool M4ERosController::enterMotorMode(tmotor_ak60_drivers_cpp::TriggerMotor::Request  &req,
                                      tmotor_ak60_drivers_cpp::TriggerMotor::Response &res){
    can_msgs::Frame can_msg;

    can_msg.header.stamp = ros::Time::now();
    can_msg.id = req.can_id;
    can_msg.dlc = 8;

    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = 0xFC;


    ROS_INFO("Motor enabled");
    can_mag_pub.publish(can_msg);

    res.result = true;
    return true;
}

bool M4ERosController::exitMotorMode(tmotor_ak60_drivers_cpp::TriggerMotor::Request  &req,
                                     tmotor_ak60_drivers_cpp::TriggerMotor::Response &res){
    can_msgs::Frame can_msg;

    can_msg.header.stamp = ros::Time::now();
    can_msg.id = req.can_id;
    can_msg.dlc = 8;

    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = 0xFD;


    ROS_INFO("Motor disabled");
    can_mag_pub.publish(can_msg);

    res.result = true;
    return true;
}


bool M4ERosController::zeroMotor(tmotor_ak60_drivers_cpp::TriggerMotor::Request  &req,
                                 tmotor_ak60_drivers_cpp::TriggerMotor::Response &res){
    can_msgs::Frame can_msg;

    can_msg.header.stamp = ros::Time::now();
    can_msg.id = req.can_id;
    can_msg.dlc = 8;

    can_msg.data[0] = 0xFF;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0xFF;
    can_msg.data[3] = 0xFF;
    can_msg.data[4] = 0xFF;
    can_msg.data[5] = 0xFF;
    can_msg.data[6] = 0xFF;
    can_msg.data[7] = 0xFE;


    ROS_INFO("Motor zeroed");
    can_mag_pub.publish(can_msg);

    res.result = true; 
    return true;
}