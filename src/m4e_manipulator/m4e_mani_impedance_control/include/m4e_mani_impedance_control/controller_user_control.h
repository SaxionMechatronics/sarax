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

#ifndef M4E_MANI_IMPEDANCE_CONTROL_CONTROLLER_USER_CONTROL_H
#define M4E_MANI_IMPEDANCE_CONTROL_CONTROLLER_USER_CONTROL_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <m4e_mani_impedance_control/parametersConfig.h>
#include "tf/tf.h"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Core>

#include <string>

class controller_user_control {
public:
    controller_user_control();
    virtual ~controller_user_control();
    void publishStampedTrajMsg();
    void setJointsPositions(const Eigen::Vector3d &q){
        joints_d_positions_ = q;
    }

private:
    ros::NodeHandle nh_;
    // controller controller_;
    std::string namespace_;
    // subscribers
    ros::Subscriber joints_trajectory_sub_;
    ros::Subscriber joints_states_sub_;
    // Publishers
    ros::Publisher trajectory_pub_;
    // Services
    ros::ServiceServer fold_arm_srv_;
    ros::ServiceServer extend_arm_srv_;
    boost::recursive_mutex mutex_;
    dynamic_reconfigure::Server<m4e_mani_impedance_control::parametersConfig> dynamic_reconfigure_server_{};
    // Messages
    sensor_msgs::JointState trajectory_stamped_msg;
    // Topics names
    std::string joints_trajectory_topic_;
    // Services Names
    std::string fold_arm_client_name_;
    std::string extend_arm_client_name_;
    // Logic switches
    bool active_safety_switch_ = false;

    void loadParams();
    // CallBacks
    void dynamicReconfigureCallback(m4e_mani_impedance_control::parametersConfig&, uint32_t level);
    bool foldingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool extendingCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    float constrain(float x, float min, float max);

    // Variables
    Eigen::Vector3d joints_d_positions_;
    Eigen::Vector3f joints_d_velocities_;
};

#endif //M4E_MANI_IMPEDANCE_CONTROL_CONTROLLER_USER_CONTROL_H
