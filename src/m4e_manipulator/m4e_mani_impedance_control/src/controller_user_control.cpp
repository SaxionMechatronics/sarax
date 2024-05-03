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

#include "m4e_mani_impedance_control/controller_user_control.h"

controller_user_control::controller_user_control() : dynamic_reconfigure_server_(mutex_) {
    loadParams();
    // Initialize Control Gains (TODO: move to dynamic config or params)
    joints_d_positions_.setZero();
    joints_d_velocities_.setZero();
    trajectory_stamped_msg.position.resize(3);
    trajectory_stamped_msg.effort.resize(3);
    trajectory_stamped_msg.velocity.resize(3);
    trajectory_stamped_msg.name.resize(3);
    // Publications:
    trajectory_pub_ = nh_.advertise<sensor_msgs::JointState>
            (joints_trajectory_topic_, 1);

    // Services:
    fold_arm_srv_ = nh_.advertiseService(fold_arm_client_name_, &controller_user_control::foldingCallback, this);
    extend_arm_srv_ = nh_.advertiseService(extend_arm_client_name_, &controller_user_control::extendingCallback, this);

    // Initialize Dynamic Reconfigure
    dynamic_reconfigure_server_.setCallback(boost::bind(&controller_user_control::dynamicReconfigureCallback, this, _1, _2));
}

controller_user_control::~controller_user_control() = default;                                  // Deconstruct

void controller_user_control::loadParams() {
    ros::NodeHandle priv_nh("~");
    if (!priv_nh.getParam("/m4e_mani_impedance_user_control/topics_names/joints_trajectory_topic", joints_trajectory_topic_)){
        ROS_ERROR("Could not find /topics_names/joints_trajectory_topic parameter!");
    }
    if (!priv_nh.getParam("/m4e_mani_impedance_user_control/services_names/fold_arm_client_name", fold_arm_client_name_)){
        fold_arm_client_name_ = "m4e_mani_user_control/foldArm";
        ROS_ERROR("Could not find /m4e_mani_impedance_user_control/services_names/fold_arm_client_name parameter!");
    }
    if (!priv_nh.getParam("/m4e_mani_impedance_user_control/services_names/extend_arm_client_name", extend_arm_client_name_)){
        extend_arm_client_name_ = "m4e_mani_user_control/extendArm";
        ROS_ERROR("Could not find /m4e_mani_impedance_user_control/services_names/extend_arm_client_name parameter!");
    }
    ROS_INFO_ONCE("[manipulator_user_control] Parameters loaded.");
}


void controller_user_control::publishStampedTrajMsg(){
    trajectory_stamped_msg.header.stamp = ros::Time::now();
    trajectory_stamped_msg.position[0] = joints_d_positions_[0];
    trajectory_stamped_msg.position[1] = joints_d_positions_[1];
    trajectory_stamped_msg.position[2] = joints_d_positions_[2];
    trajectory_stamped_msg.velocity[0] = 0;
    trajectory_stamped_msg.velocity[2] = 0;
    trajectory_stamped_msg.velocity[3] = 0;
    trajectory_pub_.publish(trajectory_stamped_msg);
    ROS_INFO_ONCE("[manipulator_user_control] Controller published first Stamped Trajectory msg.");
}

void controller_user_control::dynamicReconfigureCallback(m4e_mani_impedance_control::parametersConfig &config,
                                                               uint32_t level) {
    boost::recursive_mutex::scoped_lock lock(mutex_);  // Lock the mutex
    setJointsPositions(Eigen::Vector3d(config.joint_1_cmd * 0.01745329252, 
        config.joint_2_cmd  * 0.01745329252, config.joint_3_cmd  * 0.01745329252));
    ROS_INFO("[manipulator_user_control] Dynamic-reconfig parameters updated!");
}

bool controller_user_control::foldingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
    // Retrieve the start values
    Eigen::Vector3d start_positions = joints_d_positions_;
    // Fixed end values
    Eigen::Vector3d end_positions(0, 75* 0.01745329252, 80* 0.01745329252);

    // Compute the increment based on the duration and the difference between start and end values
    float duration = 5.0;  // Duration in seconds
    Eigen::Vector3d increments = (end_positions - start_positions) / duration;

    // Publish gradually increasing values to the topic over 5 seconds
    ros::Rate rate(10);  // Publish rate (10 Hz)
    Eigen::Vector3d current_value = start_positions;
    float elapsedTime = 0.0;

    while (elapsedTime < duration)
    {
        publishStampedTrajMsg();
        current_value = start_positions + increments * elapsedTime;
        setJointsPositions(current_value);
        elapsedTime += rate.expectedCycleTime().toSec();
        rate.sleep();
    }

    // Publish the final value
    publishStampedTrajMsg();
    ROS_INFO("[manipulator_user_control] Folding arm service called!");
    m4e_mani_impedance_control::parametersConfig new_config;
    new_config.joint_1_cmd = end_positions[0] / 0.01745329252;
    new_config.joint_2_cmd = end_positions[1] / 0.01745329252;
    new_config.joint_3_cmd = end_positions[2] / 0.01745329252;
    dynamic_reconfigure_server_.updateConfig(new_config);
    res.success = true;  // Set the response value
    return true;
}

bool controller_user_control::extendingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // Retrieve the start values
    Eigen::Vector3d start_positions = joints_d_positions_;
    // Fixed end values
    Eigen::Vector3d end_positions(0, -60* 0.01745329252, 90* 0.01745329252);

    // Compute the increment based on the duration and the difference between start and end values
    float duration = 5.0;  // Duration in seconds
    Eigen::Vector3d increments = (end_positions - start_positions) / duration;

    // Publish gradually increasing values to the topic over 5 seconds
    ros::Rate rate(10);  // Publish rate (10 Hz)
    Eigen::Vector3d current_value = start_positions;
    float elapsedTime = 0.0;

    while (elapsedTime < duration)
    {
        publishStampedTrajMsg();
        current_value = start_positions + increments * elapsedTime;
        setJointsPositions(current_value);
        elapsedTime += rate.expectedCycleTime().toSec();
        rate.sleep();
    }

    // Publish the final value
    publishStampedTrajMsg();
    ROS_INFO("[manipulator_user_control] Folding arm service called!");
    m4e_mani_impedance_control::parametersConfig new_config;
    new_config.joint_1_cmd = end_positions[0] / 0.01745329252;
    new_config.joint_2_cmd = end_positions[1] / 0.01745329252;
    new_config.joint_3_cmd = end_positions[2] / 0.01745329252;
    dynamic_reconfigure_server_.updateConfig(new_config);
    res.success = true;  // Set the response value
    return true;
}

float controller_user_control::constrain(float x, float min, float max) {
  if(x < min) {
    return min;
  }
  else if(max < x) {
    return max;
  }
  else
    return x;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "manipulator_user_control");

    controller_user_control
            controller_user_control;
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        controller_user_control.publishStampedTrajMsg();
        rate.sleep();
    }

    return 0;
}
