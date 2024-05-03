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

#include "m4e_mani_impedance_control/controller_node.h"

controller_node::controller_node() {
    loadParams();
    // Initialize Control Gains (TODO: move to dynamic config or params)
    joints_d_positions_.setZero();
    joints_d_velocities_.setZero();
    joints_d_torques_.setZero();
    actuators_control_stamped_msg.position.resize(3);
    actuators_control_stamped_msg.effort.resize(3);
    actuators_control_stamped_msg.velocity.resize(3);
    actuators_control_stamped_msg.name.resize(3);
    // Subscriptions:
    joints_trajectory_sub_ =nh_.subscribe(                                               // Read command
            joints_trajectory_topic_, 1,
            &controller_node::jointsTrajCallback, this);

    joints_states_sub_ =nh_.subscribe(                                               // Read odometry
            joints_states_topic_, 1,
            &controller_node::jointsStatesCallback, this);

    // Publications:
    actuator_1_control_pub_ = nh_.advertise<std_msgs::Float64>
            (joints_actuator_1_control_topic_, 1);
    actuator_2_control_pub_ = nh_.advertise<std_msgs::Float64>
            (joints_actuator_2_control_topic_, 1);
    actuator_3_control_pub_ = nh_.advertise<std_msgs::Float64>
            (joints_actuator_3_control_topic_, 1);
    actuators_control_pub_ = nh_.advertise<sensor_msgs::JointState>
            (joints_actuators_control_topic_, 1);

    // Initialize Dynamic Reconfigure
    // dynamic_reconfigure_server_.setCallback(boost::bind(&controller_node::dynamicReconfigureCallback, this, _1, _2));

    // Services:
    zero_motors_client_ = nh_.serviceClient<std_srvs::Trigger>
            (zeroing_client_name_);
    if (zero_motors_at_startup_) callZeroingClient();
}

controller_node::~controller_node() = default;                                  // Deconstruct

void controller_node::loadParams() {
    ros::NodeHandle priv_nh("~");
    if (!priv_nh.getParam("/manipulator_controller/topics_names/joints_trajectory_topic", joints_trajectory_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_trajectory_topic parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/topics_names/joints_states_topic", joints_states_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_states_topic_ parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/topics_names/joints_actuator_1_control_topic", joints_actuator_1_control_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_actuator_control_topic_ parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/topics_names/joints_actuator_2_control_topic", joints_actuator_2_control_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_actuator_control_topic_ parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/topics_names/joints_actuator_3_control_topic", joints_actuator_3_control_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_actuator_control_topic_ parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/topics_names/joints_actuators_control_topic", joints_actuators_control_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_actuators_control_topic parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/zero_motors_at_startup", zero_motors_at_startup_)){
        ROS_ERROR("Could not find /zero_motors_at_startup parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/services_names/zeroing_client_name", zeroing_client_name_)){
        ROS_ERROR("Could not find /services_names/zeroing_client_name parameter!");
    }
    if (!priv_nh.getParam("/manipulator_controller/simulation", simulation_mode_)){
        ROS_ERROR("Could not find /simulation parameter!");
    }
    ROS_INFO_ONCE("[mani-controller-node] Parameters loaded.");

}

void controller_node::publishActuatorControlMsg() {
    actuator_1_msg.data = constrain(joints_d_torques_[0], _min_torque, _max_torque);
    actuator_2_msg.data = constrain(joints_d_torques_[1], _min_torque, _max_torque);
    actuator_3_msg.data = constrain(joints_d_torques_[2], _min_torque, _max_torque);
    actuator_2_control_pub_.publish(actuator_2_msg);
    actuator_3_control_pub_.publish(actuator_3_msg);
    publishStampedActuatorControlMsg();
    ROS_INFO_ONCE("[mani-controller-node] Controller published first Actuator msg.");
}

void controller_node::publishStampedActuatorControlMsg(){
    actuators_control_stamped_msg.header.stamp = ros::Time::now();
    actuators_control_stamped_msg.effort[0] = joints_d_torques_[0];
    actuators_control_stamped_msg.effort[1] = joints_d_torques_[1];
    actuators_control_stamped_msg.effort[2] = joints_d_torques_[2];
    actuators_control_pub_.publish(actuators_control_stamped_msg);
    ROS_INFO_ONCE("[mani-controller-node] Controller published first Stamped Actuator msg.");
}

void controller_node::updateControllerOutput() {
    //  calculate controller output
    joints_d_torques_.setZero();
    gravity_comp_.setZero();
    impedance_control_.setZero();
    gravity_comp_ << 0, - _g*sin(joints_positions_[1])*(_L_2*_m_3 + _d_2*_m_2), - _d_3*_g*_m_3*sin(joints_positions_[2]);
    impedance_control_ = _D.asDiagonal() * (joints_d_velocities_ - joints_velocities_) + _K.asDiagonal() * (joints_d_positions_ - joints_positions_);
    joints_d_torques_ = gravity_comp_ + impedance_control_;
    publishActuatorControlMsg();
}

void controller_node::jointsTrajCallback(const sensor_msgs::JointStateConstPtr& traj_msg){
    joints_d_positions_[0] = traj_msg->position[0];
    joints_d_positions_[1] = traj_msg->position[1];
    joints_d_positions_[2] = traj_msg->position[2];
    joints_d_velocities_[0] = traj_msg->velocity[0];
    joints_d_velocities_[1] = traj_msg->velocity[1];
    joints_d_velocities_[2] = traj_msg->velocity[2];
    ROS_INFO_ONCE("[mani-controller-node] Controller got first trajectory message.");
}

void controller_node::jointsStatesCallback(const sensor_msgs::JointStateConstPtr& states_msg){
    // If in simulation, the second joint measurements is processed to match the real robot transmission
    if(simulation_mode_){
        joints_positions_[0] = 0.0;
        joints_positions_[1] = states_msg->position[0];
        joints_positions_[2] = states_msg->position[1] - states_msg->position[0];
        joints_velocities_[0] = 0.0;
        joints_velocities_[1] = states_msg->velocity[0];
        joints_velocities_[2] = states_msg->velocity[1] - states_msg->velocity[0];
        joints_torques_[0] = 0.0;
        joints_torques_[1] = states_msg->effort[0];
        joints_torques_[2] = states_msg->effort[1];
    }
    else{
        joints_positions_[0] = 0.0;
        joints_positions_[1] = states_msg->position[0];
        joints_positions_[2] = states_msg->position[1];
        joints_velocities_[0] = 0.0;
        joints_velocities_[1] = states_msg->velocity[0];
        joints_velocities_[2] = states_msg->velocity[1];
        joints_torques_[0] = 0.0;
        joints_torques_[1] = states_msg->effort[0];
        joints_torques_[2] = states_msg->effort[1];
    }
    
    ROS_INFO_ONCE("[mani-controller-node] Controller got first joints states message.");
}

float controller_node::constrain(float x, float min, float max) {
  if(x < min) {
    return min;
  }
  else if(max < x) {
    return max;
  }
  else
    return x;
}

void controller_node::callZeroingClient(){
    std_srvs::Trigger srv;
    zero_motors_client_.waitForExistence();
    ros::Duration(1.0).sleep();  // Sleep for one second until the hardware connects etc
    if(zero_motors_client_.exists() && zero_motors_client_.isValid() && zero_motors_client_.call(srv)){   
        ROS_WARN("[mani-controller-node] Motors are zeroed!");
    }
    else {
        ROS_ERROR("[mani-controller-node] Motors cannot be zeroed!");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    controller_node
            controller_node_;
    ros::Rate rate(125);
    while (ros::ok()){
        ros::spinOnce();
        controller_node_.updateControllerOutput();
        rate.sleep();
    }

    return 0;
}
