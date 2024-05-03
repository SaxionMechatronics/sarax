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

#ifndef INTERACTION_CONTROLLER_CONTROLLER_H
#define INTERACTION_CONTROLLER_CONTROLLER_H

#include <mav_msgs/eigen_mav_msgs.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/HilActuatorControls.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <Eigen/Eigen>
#include "ros/ros.h"
#include "interaction_controller/MathHelper.h"

class controller {
public:
    controller();
    void compute_ControlAllocation_and_ActuatorEffect_matrices();
    void calculateControllerOutput(Eigen::VectorXd *controller_output, Eigen::Quaterniond *desired_quaternion);

    // Setters
    void setOdometry(const mav_msgs::EigenOdometry& odometry){
        odometry_ = odometry;
        R_b_w = odometry_.orientation_W_B.toRotationMatrix();
        R_w_b = R_b_w.transpose();
        position_b_w = odometry_.position_W;
        H_b_w = MathHelper::HmatrixComposition(R_b_w,position_b_w);
        H_w_b = MathHelper::inverseH(H_b_w);
        // Publish the currect position command each time an odometry msg is read
        publishCommand();
    }

    void setTrajectoryPoint(
            const mav_msgs::EigenTrajectoryPoint& command_trajectory, const double_t initial_yaw_rad){
        command_trajectory_ = command_trajectory;
        command_trajectory_.setFromYaw(initial_yaw_rad);
        controller_active_ = true;
    }

    void setKPositionGain(const Eigen::Vector3d &PositionGain){
        position_gain_ = PositionGain;
    }

    void setKVelocityGain(const Eigen::Vector3d &VelocityGain){
        velocity_gain_ = VelocityGain;
    }

    void setKAttitudeGain(const Eigen::Vector3d &AttitudeGain){
        attitude_gain_ = AttitudeGain;
    }

    void setKAngularRateGain(const Eigen::Vector3d AngularRateGain){
        angular_rate_gain_ = AngularRateGain;
    }

    void setKO(const Eigen::Vector3d &kO){
        K_o = kO.asDiagonal();
        G_o = 0.5 * K_o.trace() * Eigen::Matrix3d::Identity() - K_o;
    }

    void setDO(const Eigen::Vector3d &dO){
        D_o = dO.asDiagonal();
    }

    void setPositionCmd(const Eigen::Vector3d &positionCmd) {
        if (use_dynamic_rec_cmds_) {
            command_trajectory_.position_W = positionCmd;
            command_trajectory_.velocity_W.setZero();
            command_trajectory_.acceleration_W.setZero();
            command_trajectory_.jerk_W.setZero();
            command_trajectory_.snap_W.setZero();
            command_trajectory_.angular_velocity_W.setZero();
            command_trajectory_.angular_acceleration_W.setZero();
        }
    }

    void setYawCmd(double_t yawCmd) {
        if (use_dynamic_rec_cmds_) {
            const double kDegToRad = M_PI / 180.0;
            const double yaw_cmd = kDegToRad * yawCmd;
            command_trajectory_.setFromYaw(yaw_cmd);
        }
    }

    void setUseDynamicRecCmds(bool useDynamicRecCmds) {
        use_dynamic_rec_cmds_ = useDynamicRecCmds;
        if (useDynamicRecCmds && !controller_active_) controller_active_ = true;
    }

    void setUsePx4Inverse(bool usePx4Inverse) {
        use_px4_inverse_ = usePx4Inverse;
    }

    void setEPOffset(const Eigen::Vector3d &ePOffset) {
        e_p_offset = ePOffset;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // ROS attributes
    ros::NodeHandle nodeHandle_controller_;
    ros::Publisher torque3D_pub_;
    ros::Publisher desired_R_pub_;
    ros::Publisher force3D_pub_;
    ros::Publisher damper_pub_;
    ros::Publisher spring_pub_;
    ros::Publisher pos_error_pub_;
    ros::Publisher att_error_pub_;
    ros::Publisher command_pub_;
    ros::Publisher inverse_pub_;
    ros::Publisher simple_normalization_pub_;
    ros::Publisher omega_pub_;
    ros::Publisher pwm_pub_;

    // Debug publishers
    void publishTorque3D(const Eigen::Vector3d&) const;
    void publishDesiredR(const Eigen::Matrix3d &R_d) const;
    void publishForce3D(const  Eigen::Vector3d &forces) const;
    void publishDamper(const Eigen::Vector3d&) const;
    void publishSpring(const Eigen::Vector3d&) const;
    void publishPosError(const Eigen::Vector3d &error) const;
    void publishAttError(const Eigen::Vector3d &error) const;
    void publishInverse(const Eigen::Vector4d &inverse) const;
    void publishSN(const Eigen::Vector4d &sp) const;
    void publishOmega(const Eigen::VectorXd &omega) const;
    void publishPWM(const Eigen::VectorXd &pwm) const;
    void publishCommand() const;
    bool controller_active_;
    bool use_Lee_attitude_controller_;
    bool use_dynamic_rec_cmds_;
    bool use_px4_inverse_;
    bool in_sitl_mode_;
    // UAV Parameter
    double _uav_mass;
    double _arm_length;
    int _num_of_arms;
    Eigen::Vector3d _inertia_matrix;
    double _moment_constant;
    double _thrust_constant;
    double _max_rotor_speed;
    double _gravity;
    double _max_rotor_thrust;
    double _max_roll_torque;
    double _max_pitch_torque;
    double _max_yaw_torque;
    double _max_thrust;
    Eigen::Vector3d _omega_to_pwm_coefficients;
    int _PWM_MIN;
    int _PWM_MAX;
    int _input_scaling;
    int _zero_position_armed;
    // Controller Gains Lee
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;
    // Controller gains Geometric Attitude controller
    // Rotational Stiffness matrix
    Eigen::Matrix3d K_o;
    // Rotational co-stiffnesses matrix
    Eigen::Matrix3d G_o;
    // Rotational damping matrix
    Eigen::Matrix3d D_o;
    Eigen::MatrixXd torques_and_thrust_to_rotor_velocities_;
    Eigen::MatrixXd throttles_to_normalized_torques_and_thrust_;
    mav_msgs::EigenTrajectoryPoint command_trajectory_;
    mav_msgs::EigenOdometry odometry_;
    // Controller variables
    Eigen::Matrix3d R_b_w;
    Eigen::Matrix4d H_b_w;
    Eigen::Matrix3d R_w_b;
    Eigen::Matrix4d H_w_b;
    Eigen::Vector3d position_b_w;
    Eigen::Vector3d e_p_offset;
    Eigen::Vector4d simpleNormalization (Eigen::Vector4d *wrench);
    Eigen::Vector4d px4Inverse (Eigen::Vector4d *wrench);
    Eigen::Vector4d px4InverseSITL (Eigen::Vector4d *wrench);
    void computeTrajectoryTracking(double *T, Eigen::Matrix3d *R_d_w) const;
    void computeAttitudeTracking_LeeController(const Eigen::Matrix3d& R_d_w, Eigen::Vector3d *tau) const;
    void computeAttitudeTracking_impedanceController(const Eigen::Matrix3d& R_d_w, Eigen::Vector3d *tau) const;
    void getUAVParameters();
};

#endif //INTERACTION_CONTROLLER_CONTROLLER_H
