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

#include "../include/interaction_controller/controller.h"


controller::controller() : controller_active_(false) {
    getUAVParameters();
    torque3D_pub_ = nodeHandle_controller_.advertise<geometry_msgs::Vector3Stamped>("debug/torque3D", 1);
    desired_R_pub_ = nodeHandle_controller_.advertise<nav_msgs::Odometry>("debug/dersired_R", 1);
    force3D_pub_ = nodeHandle_controller_.advertise<geometry_msgs::Vector3Stamped>("debug/force3D", 1);
    damper_pub_ = nodeHandle_controller_.advertise<geometry_msgs::Vector3Stamped>("debug/damper", 1);
    spring_pub_ = nodeHandle_controller_.advertise<geometry_msgs::Vector3Stamped>("debug/spring", 1);
    pos_error_pub_ = nodeHandle_controller_.advertise<geometry_msgs::Vector3Stamped>("debug/pos_error", 1);
    att_error_pub_ = nodeHandle_controller_.advertise<geometry_msgs::Vector3Stamped>("debug/att_error", 1);
    command_pub_ = nodeHandle_controller_.advertise<nav_msgs::Odometry>("debug/command_pose", 1);
    inverse_pub_ = nodeHandle_controller_.advertise<mavros_msgs::ActuatorControl>("debug/inverse", 1);
    simple_normalization_pub_ = nodeHandle_controller_.advertise<mavros_msgs::ActuatorControl>("debug/sn", 1);
    omega_pub_ = nodeHandle_controller_.advertise<mavros_msgs::HilActuatorControls>("debug/omega", 1);
    pwm_pub_ = nodeHandle_controller_.advertise<mavros_msgs::HilActuatorControls>("debug/pwm", 1);
}

void controller::compute_ControlAllocation_and_ActuatorEffect_matrices() {
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;
    Eigen::MatrixXd mixing_matrix;
    if (_num_of_arms == 6){
        const double kS = std::sin(30 * kDegToRad);
        const double kC = std::cos(30 * kDegToRad);
        rotor_velocities_to_torques_and_thrust.resize(4, 6);
        mixing_matrix.resize(6,4);
        rotor_velocities_to_torques_and_thrust <<    -1, 1, kS, -kS, -kS, kS,
                0, 0, kC, -kC, kC, -kC,
                -1, 1, -1, 1, 1, -1,
                1, 1, 1, 1, 1, 1;
        mixing_matrix << -1, 0, -1, 1,
                1, 0, 1, 1,
                0.5, 0.866085, -1, 1,
                -0.5, -0.866085, 1, 1,
                -0.5, 0.866085, 1, 1,
                0.5, -0.866085, -1, 1;
        torques_and_thrust_to_rotor_velocities_.resize(6, 4);
        throttles_to_normalized_torques_and_thrust_.resize(4,6);
        // Hardcoded because the online calculation of pesudo-inverse can be not accurate
        throttles_to_normalized_torques_and_thrust_ << -0.3333,    0.3333,    0.1667,   -0.1667,   -0.1667,    0.1667,
                    0,         0,    0.2874,   -0.2874,    0.2874,   -0.2874,
                    -0.1667,    0.1667,   -0.1667,    0.1667,    0.1667,   -0.1667,
                    0.1667,    0.1667,    0.1667,    0.1667,    0.1667,    0.1667;
    }
    else if (_num_of_arms == 4){
        const double kS = std::sin(45 * kDegToRad);
        rotor_velocities_to_torques_and_thrust.resize(4, 4);
        mixing_matrix.resize(4,4);
        rotor_velocities_to_torques_and_thrust <<    -kS, kS, kS, -kS,
                -kS, kS, -kS, kS,
                1, 1, -1, -1,
                1, 1, 1, 1;
        mixing_matrix <<   -0.71,  0.71,  1.0,   1.0,
                 0.71, -0.71,  1.0,   1.0,
                0.71,  0.71, -1.0,   1.0,
                -0.71, -0.71, -1.0,   1.0;
        torques_and_thrust_to_rotor_velocities_.resize(4, 4);
        throttles_to_normalized_torques_and_thrust_.resize(4,4);
        // Hardcoded because the online calculation of pesudo-inverse can be not accurate
        throttles_to_normalized_torques_and_thrust_ << -0.35211268,  0.35211268,  0.35211268, -0.35211268,
                0.35211268, -0.35211268,  0.35211268, -0.35211268,
                0.25,        0.25,       -0.25,       -0.25,
                0.25,        0.25,        0.25,        0.25;
    }
    else if (_num_of_arms == 44){  // this is for the coaxial quad configuration
        const double kS = std::sin(45 * kDegToRad);
        rotor_velocities_to_torques_and_thrust.resize(4, 8);
        mixing_matrix.resize(8,4);
        rotor_velocities_to_torques_and_thrust <<    -kS, kS, kS, -kS, kS, -kS, -kS, kS,
                -kS, -kS, kS, kS, -kS, -kS, kS, kS,
                1, -1, 1, -1, 1, -1, 1, -1,
                1, 1, 1, 1, 1, 1, 1, 1;
        mixing_matrix <<   -0.707107,  0.707107,  1.000000,  1.000000,
                            0.707107,  0.707107, -1.000000,  1.000000,
                            0.707107, -0.707107,  1.000000,  1.000000,
                           -0.707107, -0.707107, -1.000000,  1.000000,
                            0.707107,  0.707107,  1.000000,  1.000000,
                           -0.707107,  0.707107, -1.000000,  1.000000,
                           -0.707107, -0.707107,  1.000000,  1.000000,
                            0.707107, -0.707107, -1.000000,  1.000000;
        torques_and_thrust_to_rotor_velocities_.resize(8, 4);
        throttles_to_normalized_torques_and_thrust_.resize(4,8);
        // Hardcoded because the online calculation of pesudo-inverse can be not accurate
        throttles_to_normalized_torques_and_thrust_ << -0.17677664,  0.17677664,  0.17677664, -0.17677664,  0.17677664,-0.17677664, -0.17677664,  0.17677664,
                0.17677664,  0.17677664, -0.17677664, -0.17677664,  0.17677664, 0.17677664, -0.17677664, -0.17677664,
                0.125     , -0.125     ,  0.125     , -0.125     ,  0.125     , -0.125     ,  0.125     , -0.125,
                0.125     ,  0.125     ,  0.125     ,  0.125     ,  0.125     , 0.125     ,  0.125     ,  0.125;
    }
    else {
        ROS_ERROR("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices");
    }
    // Calculate Control allocation matrix: Wrench to Rotational velocities
    Eigen::Vector4d k;  // Helper diagonal matrix.
    k << _thrust_constant * _arm_length, _thrust_constant * _arm_length,
            _moment_constant * _thrust_constant, _thrust_constant;
    rotor_velocities_to_torques_and_thrust = k.asDiagonal() * rotor_velocities_to_torques_and_thrust;
    // In coaxial config the thrust coefficient of the lower motors is 40% reduced (experimentally obtained)
    // to simplify the control allocation: all the thrust coefficients are reduced by 20%.
    if (_num_of_arms == 44) {
        rotor_velocities_to_torques_and_thrust.row(3) *= 0.8;
    }
    torques_and_thrust_to_rotor_velocities_.setZero();
    torques_and_thrust_to_rotor_velocities_ =
            rotor_velocities_to_torques_and_thrust.completeOrthogonalDecomposition().pseudoInverse();
}

void controller::calculateControllerOutput(
        Eigen::VectorXd *controller_output, Eigen::Quaterniond *desired_quaternion) {
    assert(controller_output);

    controller_output->resize(4);
    // Return 0 velocities on all rotors, until the first command is received.
    if (!controller_active_) {
        controller_output->setZero();
        return;
    }

    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;
    computeTrajectoryTracking(&thrust, &R_d_w);
    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;
    
    // Attitude tracking.
    Eigen::Vector3d tau;
    if (use_Lee_attitude_controller_) computeAttitudeTracking_LeeController(R_d_w, &tau);
    else computeAttitudeTracking_impedanceController(R_d_w, &tau);

    // Normalize the wrench
    Eigen::Vector4d torque_thrust;
    torque_thrust << tau, thrust;
    Eigen::Vector4d normalized_torque_thrust_SN;
    Eigen::Vector4d normalized_torque_thrust_inverse;
    if (in_sitl_mode_) normalized_torque_thrust_inverse = px4InverseSITL(&torque_thrust);
    else normalized_torque_thrust_inverse = px4Inverse(&torque_thrust);
    normalized_torque_thrust_SN = simpleNormalization(&torque_thrust);
    // Publish the normalized wrench to debug
    publishInverse(normalized_torque_thrust_inverse);
    publishSN(normalized_torque_thrust_SN);
    // return the selected normalized wrench to control loop
    if (use_px4_inverse_){
        *controller_output = normalized_torque_thrust_inverse;
    }
    else {
        *controller_output = normalized_torque_thrust_SN;
    }
}

void controller::computeTrajectoryTracking(double *T, Eigen::Matrix3d *R_d_w) const {
    assert(T);
    assert(R_d_w);

    // Transform velocity to world frame.
    const Eigen::Vector3d I_v = R_b_w * odometry_.velocity_B;

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
            position_b_w - command_trajectory_.position_W - e_p_offset;
    const Eigen::Vector3d e_v = I_v - command_trajectory_.velocity_W;
    Eigen::Vector3d I_a_ref = command_trajectory_.acceleration_W;

    const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p) -
                                  velocity_gain_.cwiseProduct(e_v) +
            _uav_mass * _gravity * Eigen::Vector3d::UnitZ() + I_a_ref;
    publishPosError(e_p);
    publishForce3D(I_a_d);
    *T = I_a_d.dot(R_b_w.col(2));
    Eigen::Vector3d B_z_d;
    B_z_d = I_a_d;
    B_z_d.normalize();

    // Calculate Desired Rotational Matrix
    const double yaw_ref = command_trajectory_.getYaw();
    const Eigen::Vector3d B_x_d(std::cos(yaw_ref), std::sin(yaw_ref), 0.0);
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();
    R_d_w->col(0) = B_y_d.cross(B_z_d);
    R_d_w->col(1) = B_y_d;
    R_d_w->col(2) = B_z_d;
    publishDesiredR(*R_d_w);
}

void controller::computeAttitudeTracking_LeeController(const Eigen::Matrix3d& R_d_w, Eigen::Vector3d *tau) const {
    assert(tau);

    Eigen::Matrix3d R_IB_d;
    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_b_w - R_b_w.transpose() * R_d_w)   ;
    Eigen::Vector3d e_R;
    e_R = MathHelper::unskew(e_R_matrix);
    publishAttError(e_R);
    const Eigen::Vector3d omega_ref =
            command_trajectory_.getYawRate() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d omega = odometry_.angular_velocity_B;
    const Eigen::Vector3d e_omega = omega - R_b_w.transpose() * R_IB_d * omega_ref;
    *tau = -attitude_gain_.cwiseProduct(e_R)
           - angular_rate_gain_.cwiseProduct(e_omega)
           + omega.cross(_inertia_matrix.asDiagonal() * omega);
    publishTorque3D(*tau);
}

void controller::computeAttitudeTracking_impedanceController(const Eigen::Matrix3d& R_d_w, Eigen::Vector3d *tau) const {
    assert(tau);
    Eigen::Matrix3d R_IB_d;
    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_b_w - R_b_w.transpose() * R_d_w)   ;
    Eigen::Vector3d e_R;
    e_R = MathHelper::unskew(e_R_matrix);
    publishAttError(e_R);
    Eigen::Matrix3d R_b_d;
    R_b_d = R_d_w.transpose() * R_b_w;
    Eigen::Matrix<double, 3, 3> spring_torque_tilde;
    Eigen::Vector3d spring_torque_vector;
    Eigen::Vector3d damper_torque;
    spring_torque_tilde.setZero();
    damper_torque.setZero();
    // Spring Torque Calculation
    spring_torque_tilde = -(2.0 * MathHelper::antisym(G_o * R_b_d));
    spring_torque_vector = MathHelper::unskew(spring_torque_tilde);
    // Damper Torque Calculation
    damper_torque = D_o * odometry_.angular_velocity_B;
    *tau = spring_torque_vector - damper_torque;
    publishSpring(spring_torque_vector);
    publishDamper(damper_torque);
    publishTorque3D(*tau);
}

Eigen::Vector4d controller::px4Inverse(Eigen::Vector4d *wrench) {
    Eigen::VectorXd omega;
    Eigen::VectorXd pwm;
    Eigen::VectorXd throttles;
    Eigen::Vector4d normalized_torque_and_thrust;
    Eigen::VectorXd ones_temp;
    normalized_torque_and_thrust.setZero();
    if (_num_of_arms == 6){
        omega.resize(6);
        omega.setZero();
        pwm.resize(6);
        pwm.setZero();
        throttles.resize(6);
        throttles.setZero();
        ones_temp.resize(6);
        ones_temp = Eigen::VectorXd::Ones(6,1);
    }
    else if (_num_of_arms == 4){
        omega.resize(4);
        omega.setZero();
        pwm.resize(4);
        pwm.setZero();
        throttles.resize(4);
        throttles.setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4,1);
    }
    else if (_num_of_arms == 44){
        omega.resize(8);
        omega.setZero();
        pwm.resize(8);
        pwm.setZero();
        throttles.resize(8);
        throttles.setZero();
        ones_temp.resize(8);
        ones_temp = Eigen::VectorXd::Ones(8,1);
    }
    else {
        ROS_ERROR("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices");
    }
    // Control allocation: Wrench to Rotational velocities (omega)
    omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
    for (int i = 0; i < omega.size(); i++){
        if (omega[i] <= 0){
            omega[i] = 0.0;
        }
    }
    omega = omega.cwiseSqrt();
    // Publish Omega to help debug
    publishOmega(omega);
    // Rotational velocities to PWM signals
    pwm = (_omega_to_pwm_coefficients(0) * omega.cwiseProduct(omega)) + (_omega_to_pwm_coefficients(1) * omega) +
          (_omega_to_pwm_coefficients(2) * ones_temp);
    // Publish PWM to help debug
    publishPWM(pwm);
    // PWM signals to throttles
    throttles = (pwm - (_PWM_MIN * ones_temp));
    throttles /= (_PWM_MAX - _PWM_MIN);
    // Inverse Mixing: throttles to normalized torques and thrust
    normalized_torque_and_thrust = throttles_to_normalized_torques_and_thrust_ * throttles;
    return normalized_torque_and_thrust;
}

Eigen::Vector4d controller::px4InverseSITL(Eigen::Vector4d *wrench) {
    Eigen::VectorXd omega;
    Eigen::VectorXd throttles;
    Eigen::Vector4d normalized_torque_and_thrust;
    normalized_torque_and_thrust.setZero();
    Eigen::VectorXd ones_temp;
    if (_num_of_arms == 6){
        omega.resize(6);
        omega.setZero();
        throttles.resize(6);
        throttles.setZero();
        ones_temp.resize(6);
        ones_temp = Eigen::VectorXd::Ones(6,1);
    }
    else if (_num_of_arms == 4){
        omega.resize(4);
        omega.setZero();
        throttles.resize(4);
        throttles.setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4,1);
    }
    else if (_num_of_arms == 44){
        omega.resize(8);
        omega.setZero();
        throttles.resize(8);
        throttles.setZero();
        ones_temp.resize(8);
        ones_temp = Eigen::VectorXd::Ones(8,1);
    }
    else {
        ROS_ERROR("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices");
    }
    // Control allocation: Wrench to Rotational velocities (omega)
    omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
    omega = omega.cwiseSqrt();
    publishOmega(omega);
    // omega to throttles
    throttles = (omega - (_zero_position_armed * ones_temp));
    throttles /= (_input_scaling);
    publishPWM(throttles);
    // Inverse Mixing: throttles to normalized torques and thrust
    normalized_torque_and_thrust = throttles_to_normalized_torques_and_thrust_ * throttles;
    return normalized_torque_and_thrust;
}

Eigen::Vector4d controller::simpleNormalization(Eigen::Vector4d *wrench) {
    //  Assuming that:
    //  wrench = [roll_torque; pitch_torque; yaw_torque; thrust]
    Eigen::Vector4d normalizedWrench;

    // Normalize
    normalizedWrench[0] = (*wrench)[0] / _max_roll_torque;      // normalize roll torque
    normalizedWrench[1] = (*wrench)[1] / _max_pitch_torque;     // normalize pitch torque
    normalizedWrench[2] = (*wrench)[2] / _max_yaw_torque;       // normalize yaw torque
    normalizedWrench[3] = (*wrench)[3] / _max_thrust;           // Normalize F_z aka thrust
    // Limit the torques to [-1,1]
    // TODO: use std:max and std::min instead of ifs and for
    for (int i = 0; i < 4; ++i) {
        if (normalizedWrench[i] > 1){
            normalizedWrench[i] = 1;
        }
        else if (normalizedWrench(i) < -1) {
            normalizedWrench[i] = -1;
        }
    }
    // Limit the thrust to [0,1]
    if (normalizedWrench[3] > 1){
        normalizedWrench[3] = 1;
    }
    else if (normalizedWrench[3] < 0) {
        normalizedWrench[3] = 0;
    }
    return normalizedWrench;
}

void controller::getUAVParameters(){
    ros::NodeHandle _nh;
    if (!_nh.getParam("uav_parameters/mass", _uav_mass)){
        ROS_ERROR("Could not find uav_parameters/mass parameter!");
    }
    if (!_nh.getParam("use_Lee_attitude_controller", use_Lee_attitude_controller_)){
        ROS_ERROR("Could not find use_Lee_attitude_controller parameter!");
    }
    if (!_nh.getParam("uav_parameters/arm_length", _arm_length)){
        ROS_ERROR("Could not find uav_parameters/arm_length parameter!");
    }
    if (!_nh.getParam("uav_parameters/num_of_arms", _num_of_arms)){
        ROS_ERROR("Could not find uav_parameters/num_of_arms parameter!");
    }
    if (!_nh.getParam("uav_parameters/inertia/x", _inertia_matrix[0])){
        ROS_ERROR("Could not find uav_parameters/inertia/x parameter!");
    }
    if (!_nh.getParam("uav_parameters/inertia/y", _inertia_matrix[1])){
        ROS_ERROR("Could not find uav_parameters/inertia/y parameter!");
    }
    if (!_nh.getParam("uav_parameters/inertia/z", _inertia_matrix[2])){
        ROS_ERROR("Could not find uav_parameters/inertia/z parameter!");
    }
    if (!_nh.getParam("uav_parameters/moment_constant", _moment_constant)){
        ROS_ERROR("Could not find uav_parameters/moment_constant parameter!");
    }
    if (!_nh.getParam("uav_parameters/thrust_constant", _thrust_constant)){
        ROS_ERROR("Could not find uav_parameters/thrust_constant parameter!");
    }
    if (!_nh.getParam("uav_parameters/max_rotor_speed", _max_rotor_speed)){
        ROS_ERROR("Could not find uav_parameters/max_rotor_speed parameter!");
    }
    if (!_nh.getParam("uav_parameters/gravity", _gravity)){
        ROS_ERROR("Could not find uav_parameters/gravity parameter!");
    }
    if (!_nh.getParam("uav_parameters/omega_to_pwm_coefficient/x_2", _omega_to_pwm_coefficients[0])){
        ROS_ERROR("Could not find uav_parameters/omega_to_pwm_coefficient_a2 parameter!");
    }
    if (!_nh.getParam("uav_parameters/omega_to_pwm_coefficient/x_1", _omega_to_pwm_coefficients[1])){
        ROS_ERROR("Could not find uav_parameters/omega_to_pwm_coefficient_x_1 parameter!");
    }
    if (!_nh.getParam("uav_parameters/omega_to_pwm_coefficient/x_0", _omega_to_pwm_coefficients[2])){
        ROS_ERROR("Could not find uav_parameters/omega_to_pwm_coefficient_x_0 parameter!");
    }
    if (!_nh.getParam("uav_parameters/PWM_MIN", _PWM_MIN)){
        ROS_ERROR("Could not find uav_parameters/PWM_MIN parameter!");
    }
    if (!_nh.getParam("uav_parameters/PWM_MAX", _PWM_MAX)){
        ROS_ERROR("Could not find uav_parameters/PWM_MAX parameter!");
    }
    if (!_nh.getParam("uav_parameters/zero_position_armed", _zero_position_armed)){
        ROS_ERROR("Could not find uav_parameters/zero_position_armed parameter!");
    }
    if (!_nh.getParam("uav_parameters/input_scaling", _input_scaling)){
        ROS_ERROR("Could not find uav_parameters/input_scaling parameter!");
    }
    if (!_nh.getParam("sitl_mode", in_sitl_mode_)){
        ROS_ERROR("Could not find sitl_mode parameter!");
    }

    if (_num_of_arms == 6){
        const double DegToRad = M_PI / 180.0;
        const double cos60 = std::cos(DegToRad*60);
        const double sin60 = std::sin(DegToRad*60);
        _max_rotor_thrust = _thrust_constant * (std::pow(_max_rotor_speed,2));
        _max_thrust = 6 * _max_rotor_thrust;
        _max_roll_torque = (2 * cos60 * _arm_length * _max_rotor_thrust) + (_max_rotor_thrust * _arm_length);
        _max_pitch_torque = 2 * sin60 * _arm_length * _max_rotor_thrust;
        _max_yaw_torque = 6 * _moment_constant * _max_rotor_thrust;
    }
    else if (_num_of_arms == 4){
        const double DegToRad = M_PI / 180.0;
        const double cos60 = std::cos(DegToRad*60);
        const double sin60 = std::sin(DegToRad*60);
        _max_rotor_thrust = _thrust_constant * (std::pow(_max_rotor_speed,2));
        _max_thrust = 4 * _max_rotor_thrust;
        _max_roll_torque = 2 * cos60 * _arm_length * _max_rotor_thrust;
        _max_pitch_torque = 2 * sin60 * _arm_length * _max_rotor_thrust;
        _max_yaw_torque = 4 * _moment_constant * _max_rotor_thrust;
    }
    else {
        ROS_ERROR("[controller] Unknown UAV num_of_arms parameter");
    }
    compute_ControlAllocation_and_ActuatorEffect_matrices();
}

void controller::publishTorque3D(const Eigen::Vector3d &torque) const {
    geometry_msgs::Vector3Stamped torque_msg;
    torque_msg.header.stamp = ros::Time::now();
    torque_msg.header.frame_id = "base_link";
    torque_msg.vector.x = torque.x();
    torque_msg.vector.y = torque.y();
    torque_msg.vector.z = torque.z();
    torque3D_pub_.publish(torque_msg);
}

void controller::publishDesiredR(const Eigen::Matrix3d &R_d) const {
    nav_msgs::Odometry R_msg;
    R_msg.header.frame_id = "map";
    R_msg.header.stamp = ros::Time::now();
    R_msg.child_frame_id = "base_link";
    R_msg.pose.pose.position.x = odometry_.position_W.x();
    R_msg.pose.pose.position.y = odometry_.position_W.y();
    R_msg.pose.pose.position.z = odometry_.position_W.z();
    Eigen::Quaterniond quaternion_d;
    quaternion_d = R_d;
    R_msg.pose.pose.orientation.x = quaternion_d.x();
    R_msg.pose.pose.orientation.y = quaternion_d.y();
    R_msg.pose.pose.orientation.z = quaternion_d.z();
    R_msg.pose.pose.orientation.w = quaternion_d.w();
    desired_R_pub_.publish(R_msg);
}

void controller::publishForce3D(const Eigen::Vector3d &forces) const {
    geometry_msgs::Vector3Stamped forces_msg;
    forces_msg.header.stamp = ros::Time::now();
    forces_msg.header.frame_id = "map";
    forces_msg.vector.x = forces.x();
    forces_msg.vector.y = forces.y();
    forces_msg.vector.z = forces.z();
    force3D_pub_.publish(forces_msg);
}

void controller::publishDamper(const Eigen::Vector3d &damper) const {
    geometry_msgs::Vector3Stamped damper_msg;
    damper_msg.header.stamp = ros::Time::now();
    damper_msg.header.frame_id = "base_link";
    damper_msg.vector.x = damper.x();
    damper_msg.vector.y = damper.y();
    damper_msg.vector.z = damper.z();
    damper_pub_.publish(damper_msg);
}

void controller::publishSpring(const Eigen::Vector3d &spring) const {
    geometry_msgs::Vector3Stamped spring_msg;
    spring_msg.header.stamp = ros::Time::now();
    spring_msg.header.frame_id = "base_link";
    spring_msg.vector.x = spring.x();
    spring_msg.vector.y = spring.y();
    spring_msg.vector.z = spring.z();
    spring_pub_.publish(spring_msg);
}

void controller::publishPosError(const Eigen::Vector3d &error) const {
    geometry_msgs::Vector3Stamped error_msg;
    error_msg.header.stamp = ros::Time::now();
    error_msg.header.frame_id = "map";
    error_msg.vector.x = error.x();
    error_msg.vector.y = error.y();
    error_msg.vector.z = error.z();
    pos_error_pub_.publish(error_msg);
}

void controller::publishAttError(const Eigen::Vector3d &error) const {
    geometry_msgs::Vector3Stamped error_msg;
    error_msg.header.stamp = ros::Time::now();
    error_msg.header.frame_id = "base_link";
    error_msg.vector.x = error.x();
    error_msg.vector.y = error.y();
    error_msg.vector.z = error.z();
    att_error_pub_.publish(error_msg);
}

void controller::publishCommand() const {
    nav_msgs::Odometry cmd_msg;
    cmd_msg.header.frame_id = "map";
    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.child_frame_id = "base_link";
    cmd_msg.pose.pose.position.x = command_trajectory_.position_W.x();
    cmd_msg.pose.pose.position.y = command_trajectory_.position_W.y();
    cmd_msg.pose.pose.position.z = command_trajectory_.position_W.z();
    cmd_msg.pose.pose.orientation.x = command_trajectory_.orientation_W_B.x();
    cmd_msg.pose.pose.orientation.y = command_trajectory_.orientation_W_B.y();
    cmd_msg.pose.pose.orientation.z = command_trajectory_.orientation_W_B.z();
    cmd_msg.pose.pose.orientation.w = command_trajectory_.orientation_W_B.w();
    command_pub_.publish(cmd_msg);
}

void controller::publishInverse(const Eigen::Vector4d &inverse) const {
    mavros_msgs::ActuatorControl inverse_msg;
    // Prepare ActControl msg
    inverse_msg.header.stamp = ros::Time::now();
    inverse_msg.header.frame_id = "base_link";
    inverse_msg.group_mix = 0;
    inverse_msg.controls[0] = inverse[0];
    inverse_msg.controls[1] = - inverse[1];       // minus sign because PX4 uses NED frame
    inverse_msg.controls[2] = - inverse[2];       // minus sign because PX4 uses NED frame
    inverse_msg.controls[3] = inverse[3];
    if (use_px4_inverse_) inverse_msg.controls[4] = 1;
    else inverse_msg.controls[4] = 0;
    inverse_pub_.publish(inverse_msg);
}

void controller::publishSN(const Eigen::Vector4d &sn) const {
    mavros_msgs::ActuatorControl sn_msg;
    // Prepare ActControl msg
    sn_msg.header.stamp = ros::Time::now();
    sn_msg.header.frame_id = "base_link";
    sn_msg.group_mix = 0;
    sn_msg.controls[0] = sn[0];
    sn_msg.controls[1] = - sn[1];       // minus sign because PX4 uses NED frame
    sn_msg.controls[2] = - sn[2];       // minus sign because PX4 uses NED frame
    sn_msg.controls[3] = sn[3];
    if (!use_px4_inverse_) sn_msg.controls[4] = 1;
    else sn_msg.controls[4] = 0;
    simple_normalization_pub_.publish(sn_msg);
}

void controller::publishOmega(const Eigen::VectorXd &omega) const {
    mavros_msgs::HilActuatorControls omega_msg;
    // Prepare ActControl msg
    omega_msg.header.stamp = ros::Time::now();
    omega_msg.header.frame_id = "base_link";
    omega_msg.controls[0] = omega[0];
    omega_msg.controls[1] = omega[1];
    omega_msg.controls[2] = omega[2];
    omega_msg.controls[3] = omega[3];
    if (_num_of_arms == 6){
        omega_msg.controls[4] = omega[4];
        omega_msg.controls[5] = omega[5];
    }
    else if (_num_of_arms == 44){
        omega_msg.controls[4] = omega[4];
        omega_msg.controls[5] = omega[5];
        omega_msg.controls[6] = omega[6];
        omega_msg.controls[7] = omega[7];
    }
    omega_pub_.publish(omega_msg);
}

void controller::publishPWM(const Eigen::VectorXd &pwm) const {
    mavros_msgs::HilActuatorControls pwm_msg;
    // Prepare ActControl msg
    pwm_msg.header.stamp = ros::Time::now();
    pwm_msg.header.frame_id = "base_link";
    pwm_msg.controls[0] = pwm[0];
    pwm_msg.controls[1] = pwm[1];
    pwm_msg.controls[2] = pwm[2];
    pwm_msg.controls[3] = pwm[3];
    if (_num_of_arms == 6)
    {
        pwm_msg.controls[4] = pwm[4];
        pwm_msg.controls[5] = pwm[5];
    }
    else if (_num_of_arms == 44){
        pwm_msg.controls[4] = pwm[4];
        pwm_msg.controls[5] = pwm[5];
        pwm_msg.controls[6] = pwm[6];
        pwm_msg.controls[7] = pwm[7];
    }
    pwm_pub_.publish(pwm_msg);
}