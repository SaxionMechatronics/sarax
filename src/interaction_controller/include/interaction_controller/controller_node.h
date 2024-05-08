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

#ifndef INTERACTION_CONTROLLER_CONTROLLER_NODE_H
#define INTERACTION_CONTROLLER_CONTROLLER_NODE_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "mavros_msgs/State.h"
#include "mavros_msgs/ActuatorControl.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include "tf/tf.h"
#include "tf2_ros/transform_broadcaster.h"

#include <string>

#include <interaction_controller/parametersConfig.h>
#include "interaction_controller/controller.h"

class controller_node {
public:
    controller_node();
    virtual ~controller_node();
    void updateControllerOutput();

private:
    ros::NodeHandle nh_;
    controller controller_;
    std::string namespace_;
    // subscribers
    ros::Subscriber cmd_trajectory_sub_;
    ros::Subscriber cmd_pose_sub_;
    ros::Subscriber cmd_traj_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber battery_sub_;
    // Publishers
    ros::Publisher motor_velocity_reference_pub_;
    ros::Publisher actuator_control_pub_;
    ros::Publisher attitude_target_pub_;
    ros::Publisher voltage_comp_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    // Services
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    dynamic_reconfigure::Server<interaction_controller::parametersConfig> dynamic_reconfigure_server_;
    // Messages
    mavros_msgs::AttitudeTarget attitude_target_msg;
    mavros_msgs::ActuatorControl actuator_control_msg;
    std_msgs::Float32 voltage_comp_msg;
    // Topics names
    std::string command_pose_topic_;
    std::string command_traj_topic_;
    std::string odometry_topic_;
    std::string state_topic_;
    std::string battery_topic_;
    std::string actuator_control_topic_;
    std::string attitude_target_topic_;
    // Services Names
    std::string arming_client_name_;
    std::string set_mode_client_name_;
    // Logic switches
    bool use_attitude_target_;
    bool first_odometry_message_ = true;
    bool initialize_yaw_from_odometry_;
    bool control_initialized_frame_;     // Control the position in the modified world frame (rotated world frame by initial yaw)
    // Outdoor testing parameters
    double initial_yaw_deg_ = 0.0;
    double initial_yaw_rad_ = 0.0;
    // Voltage compensation
    float _model_slope = 0.0;
    float _model_max_voltage = 0.0;
    float _model_max_voltage_percentage_constant = 0.0;
    float _model_max_voltage_compensation = 0.0;
    bool _use_voltage_compensation = false;
    bool valid_battery_reading_ = false;
    float voltage_percentage_ = 0.0;
    float voltage_compensation_throttle_ = 0.0;
    void updateVoltageCompThrottle();

    mav_msgs::EigenTrajectoryPointDeque commands_;
    std::deque<ros::Duration> command_waiting_times_;
    mavros_msgs::State current_state_;
    bool connected_ = false;

    void loadParams();
    void secureConnection();
    // CallBacks
    void commandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
    void commandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& traj_msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);
    void stateCallBack(const mavros_msgs::State::ConstPtr& msg);
    void batteryCallBack(const sensor_msgs::BatteryStateConstPtr& msg);
    void dynamicReconfigureCallback(const interaction_controller::parametersConfig&, const uint32_t);
    void publishActuatorControlMsg(const Eigen::Vector4d& controller_output);
    void publishAttitudeTargetMsg(const Eigen::Vector4d& controller_output, const Eigen::Quaterniond& desired_quaternion);
};



#endif //interaction_controller_CONTROLLER_NODE_H
