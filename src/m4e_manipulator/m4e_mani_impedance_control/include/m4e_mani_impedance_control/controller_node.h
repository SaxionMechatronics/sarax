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

#ifndef M4E_MANI_IMPEDANCE_CONTROL_CONTROLLER_NODE_H
#define M4E_MANI_IMPEDANCE_CONTROL_CONTROLLER_NODE_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include "tf/tf.h"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Core>

#include <string>

class controller_node {
public:
    controller_node();
    virtual ~controller_node();
    void updateControllerOutput();

private:
    ros::NodeHandle nh_;
    // controller controller_;
    std::string namespace_;
    // subscribers
    ros::Subscriber joints_trajectory_sub_;
    ros::Subscriber joints_states_sub_;
    // Publishers
    ros::Publisher actuator_1_control_pub_;
    ros::Publisher actuator_2_control_pub_;
    ros::Publisher actuator_3_control_pub_;
    ros::Publisher actuators_control_pub_;
    // Services
    // dynamic_reconfigure::Server<interaction_controller::parametersConfig> dynamic_reconfigure_server_;
    ros::ServiceClient zero_motors_client_;

    // Messages
    std_msgs::Float64 actuator_1_msg;
    std_msgs::Float64 actuator_2_msg;
    std_msgs::Float64 actuator_3_msg;
    sensor_msgs::JointState actuators_control_stamped_msg;
    // Topics names
    std::string joints_trajectory_topic_;
    std::string joints_states_topic_;
    std::string joints_actuator_1_control_topic_;
    std::string joints_actuator_2_control_topic_;
    std::string joints_actuator_3_control_topic_;
    std::string joints_actuators_control_topic_;
    // Services Names
    std::string zeroing_client_name_;

    // Logic switches
    bool active_safety_switch_ = false;
    bool zero_motors_at_startup_ = true;
    bool simulation_mode_ = false;

    void loadParams();
    // CallBacks
    void jointsTrajCallback(const sensor_msgs::JointStateConstPtr& traj_msg);
    void jointsStatesCallback(const sensor_msgs::JointStateConstPtr& states_msg);
    void publishActuatorControlMsg();
    void publishStampedActuatorControlMsg();
    void callZeroingClient();
    float constrain(float x, float min, float max);

    // Variables
    Eigen::Vector3f joints_positions_;
    Eigen::Vector3f joints_velocities_;
    // Eigen::Vector3f joints_accelerations_;
    Eigen::Vector3f joints_torques_;
    Eigen::Vector3f joints_d_positions_;
    Eigen::Vector3f joints_d_velocities_;
    // Eigen::Vector3f joints_d_accelerations_;
    Eigen::Vector3f joints_d_torques_;
    Eigen::Vector3f gravity_comp_;
    Eigen::Vector3f impedance_control_;


    // Parameters TODO: move to Parameters Server and/or Dynamic Config
    float _g = 9.81;
    float _L_1 = 0.4;
    float _L_2 = 0.4;
    float _L_3 = 0.4;
    float _m_1 = 0.4;
    float _m_2 = 0.4;
    float _m_3 = 0.4;
    float _d_2 = 0.5 *_L_2;
    float _d_3 = 0.5 *_L_3;
    float _min_torque = -5;
    float _max_torque = 5;
    Eigen::Vector3f _K{0.5, 5, 5};
    Eigen::Vector3f _D{0.1, 0.5, 0.5};
};

#endif //M4E_MANI_IMPEDANCE_CONTROL_CONTROLLER_NODE_H
