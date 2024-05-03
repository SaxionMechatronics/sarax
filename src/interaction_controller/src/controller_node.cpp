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

#include "interaction_controller/controller_node.h"



controller_node::controller_node() {
    loadParams();
    // Services and connection
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            (arming_client_name_);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            (set_mode_client_name_);
    // Subscriptions:
    cmd_pose_sub_ =nh_.subscribe(                                               // Read command
            command_pose_topic_, 1,
            &controller_node::commandPoseCallback, this);

    cmd_traj_sub_ =nh_.subscribe(                                               // Read command
            command_traj_topic_, 1,
            &controller_node::commandTrajectoryCallback, this);

    odometry_sub_ =nh_.subscribe(                                               // Read odometry
            odometry_topic_, 1,
            &controller_node::odometryCallback, this);

    state_sub_ = nh_.subscribe<mavros_msgs::State>                                // Read Statues
            (state_topic_, 10,
             &controller_node::stateCallBack, this);
    if(_use_voltage_compensation){
        battery_sub_ = nh_.subscribe<sensor_msgs::BatteryState>                                // Read battery status
                (battery_topic_, 10,
                &controller_node::batteryCallBack, this);
    }

    // Publications:
    actuator_control_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>
            (actuator_control_topic_, 10);

    attitude_target_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>
            (attitude_target_topic_, 10);

    if(_use_voltage_compensation){
        voltage_comp_pub_ = nh_.advertise<std_msgs::Float32>
                ("/debug/voltage_comp", 10);
    }
    // Initialize Dynamic Reconfigure
    dynamic_reconfigure_server_.setCallback(boost::bind(&controller_node::dynamicReconfigureCallback, this, _1, _2));
    connected_ = false;
    secureConnection();
}

controller_node::~controller_node() = default;                                  // Deconstruct

void controller_node::loadParams() {
    ros::NodeHandle _nh;
    if (!_nh.getParam("use_attitude_target_msgs", use_attitude_target_)){
        ROS_ERROR("Could not find use_attitude_target_msgs parameter!");
    }
    if (!_nh.getParam("initialize_yaw_from_odometry", initialize_yaw_from_odometry_)){
        ROS_ERROR("Could not find initialize_yaw_from_odometry parameter!");
    }
    if (!_nh.getParam("topics_names/command_pose_topic", command_pose_topic_)){
        ROS_ERROR("Could not find topics_names/command_pose_topic parameter!");
    }
    if (!_nh.getParam("topics_names/command_traj_topic", command_traj_topic_)){
        ROS_ERROR("Could not find topics_names/command_traj_topic parameter!");
    }
    if (!_nh.getParam("/control_initialized_frame", control_initialized_frame_)){
        ROS_ERROR("Could not find control_initialized_frame parameter!");
    }
    if (!_nh.getParam("topics_names/odometry_topic", odometry_topic_)){
        ROS_ERROR("Could not find topics_names/odometry_topic parameter!");
    }
    if (!_nh.getParam("topics_names/state_topic", state_topic_)){
        ROS_ERROR("Could not find topics_names/state_topic parameter!");
    }
    if (!_nh.getParam("topics_names/actuator_control_topic", actuator_control_topic_)){
        ROS_ERROR("Could not find topics_names/actuator_control_topic parameter!");
    }
    if (!_nh.getParam("topics_names/attitude_target_topic", attitude_target_topic_)){
        ROS_ERROR("Could not find topics_names/attitude_target_topic parameter!");
    }
    if (!_nh.getParam("services_names/arming_client_name", arming_client_name_)){
        ROS_ERROR("Could not find services_names/arming_client_name parameter!");
    }
    if (!_nh.getParam("services_names/set_mode_client_name", set_mode_client_name_)){
        ROS_ERROR("Could not find services_names/set_mode_client_name parameter!");
    }
    if (!_nh.getParam("topics_names/battery_topic", battery_topic_)){
        ROS_ERROR("Could not find topics_names/battery_topic parameter!");
    }
    if (!_nh.getParam("uav_parameters/voltage_compensation/model_slope", _model_slope)){
        ROS_ERROR("Could not find uav_parameters/voltage_compensation/model_slope parameter!");
    }
    if (!_nh.getParam("uav_parameters/voltage_compensation/model_max_voltage", _model_max_voltage)){
        ROS_ERROR("Could not find uav_parameters/voltage_compensation/model_max_voltage parameter!");
    }
    if (!_nh.getParam("uav_parameters/voltage_compensation/model_max_voltage_percentage_constant", _model_max_voltage_percentage_constant)){
        ROS_ERROR("Could not find uav_parameters/voltage_compensation/model_max_voltage_percentage_constant parameter!");
    }
    if (!_nh.getParam("uav_parameters/voltage_compensation/model_max_voltage_compensation", _model_max_voltage_compensation)){
        ROS_ERROR("Could not find uav_parameters/voltage_compensation/model_max_voltage_compensation parameter!");
    }
    if (!_nh.getParam("uav_parameters/voltage_compensation/use_voltage_compensation", _use_voltage_compensation)){
        ROS_ERROR("Could not find uav_parameters/voltage_compensation/use_voltage_compensation parameter!");
    }
}

void controller_node::secureConnection() {
    ros::Rate rate(20);
    // Wait for FCU connection
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("[controller-node] FCU connected!");

    // Fill the buffer
    mavros_msgs::ActuatorControl tempMsg;
    tempMsg.group_mix = 0;
    tempMsg.controls[1] = 0;
    tempMsg.controls[2] = 0;
    tempMsg.controls[3] = 0;
    tempMsg.controls[4] = 0.1;
    for(int i = 100; ros::ok() && i > 0; --i){
        actuator_control_pub_.publish(tempMsg);
        ros::spinOnce();                        //resposible to handle communication events, e.g. arriving messages
        rate.sleep();
    }
    ROS_INFO_ONCE("[controller-node] Done filling buffer!");

    /*  *********************************
     *              ARM & OFFOARD
     *  *********************************
     */
    ros::Time last_request = ros::Time::now();
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO_ONCE("[controller-node] Start ARMING and OB.");

    // DO NOT ARM OR GO TO OFFBOARD AT THE START-UP
    // TO BE DELETED LATER

    // while (current_state_.mode != "OFFBOARD" && !current_state_.armed ){
    //     if (current_state_.mode != "OFFBOARD" &&
    //         set_mode_client_.exists() &&
    //         set_mode_client_.isValid()){
    //         set_mode_client_.call(offb_set_mode);
    //         ROS_INFO_ONCE("[controller-node] Offboard enabled");
    //     }
    //     if (!current_state_.armed &&
    //         arming_client_.exists() &&
    //         arming_client_.isValid()){
    //         arming_client_.call(arm_cmd);
    //         ROS_INFO_ONCE("[controller-node] Vehicle armed");
    //     }
    //     ros::spinOnce();
    // }
    // ROS_INFO("[controller-node] Out of arming and OFFb loop");
    connected_ = true;
}


void controller_node::commandPoseCallback(
        const geometry_msgs::PoseStampedConstPtr& pose_msg) {                   // When a command is received
    // Clear all pending commands.
    commands_.clear();
    command_waiting_times_.clear();
    mav_msgs::EigenTrajectoryPoint eigen_reference;
    if (control_initialized_frame_)
    {
        geometry_msgs::PoseStamped cmd_pose_rotated;
        cmd_pose_rotated.header = pose_msg->header;
        cmd_pose_rotated.pose = pose_msg->pose;
        cmd_pose_rotated.pose.position.x = (std::cos(initial_yaw_rad_) * pose_msg->pose.position.x) 
                                    - (std::sin(initial_yaw_rad_) * pose_msg->pose.position.y);
        cmd_pose_rotated.pose.position.y = (std::sin(initial_yaw_rad_) * pose_msg->pose.position.x)
                                    + (std::cos(initial_yaw_rad_) * pose_msg->pose.position.y);
        mav_msgs::eigenTrajectoryPointFromPoseMsg(cmd_pose_rotated, &eigen_reference);
    }
    else{
        mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    }
    commands_.push_front(eigen_reference);
    controller_.setTrajectoryPoint(commands_.front(), initial_yaw_rad_);          // Send the command to controller_ obj
    commands_.pop_front();
    ROS_INFO_ONCE("[controller-node] Controller got first command message.");
}

void controller_node::commandTrajectoryCallback(
        const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& traj_msg) {                   // When a command is received
    // Clear all pending commands.
    commands_.clear();
    command_waiting_times_.clear();

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    // This function (eigenTrajectoryPointFromTrajMsg) is not a standard function
    // I have implemented it into mav_msgs package myself. Such that:
    // https://github.com/ethz-asl/mav_comm/pull/89
    mav_msgs::eigenTrajectoryPointFromTrajMsg(*traj_msg, &eigen_reference);
    commands_.push_front(eigen_reference);

    ROS_INFO_ONCE("[controller-node] Controller got first command message.");
    controller_.setTrajectoryPoint(commands_.front(), initial_yaw_rad_);          // Send the command to controller_ obj
    commands_.pop_front();
}

void controller_node::odometryCallback(                                       // read odometry and take action
        const nav_msgs::OdometryConstPtr& odometry_msg) {                       // THE MAIN connection to controller class
    if (connected_) {
        //  Debug message
        ROS_INFO_ONCE("[controller-node] Controller got first odometry message.");
        // send odometry to controller_ obj
        mav_msgs::EigenOdometry odometry;
        mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
        controller_.setOdometry(odometry);
    }
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = odometry_msg->header.frame_id;
    transform_msg.child_frame_id = odometry_msg->child_frame_id;
    transform_msg.transform.translation.x = odometry_msg->pose.pose.position.x;
    transform_msg.transform.translation.y = odometry_msg->pose.pose.position.y;
    transform_msg.transform.translation.z = odometry_msg->pose.pose.position.z;
    transform_msg.transform.rotation.x = odometry_msg->pose.pose.orientation.x;
    transform_msg.transform.rotation.y = odometry_msg->pose.pose.orientation.y;
    transform_msg.transform.rotation.z = odometry_msg->pose.pose.orientation.z;
    transform_msg.transform.rotation.w = odometry_msg->pose.pose.orientation.w;
    tf_broadcaster_.sendTransform(transform_msg);
    // initialize yaw
    if (first_odometry_message_ && initialize_yaw_from_odometry_ && current_state_.armed){
        const double kRadToDeg = 180.0/M_PI;
        double _roll,_pitch,_yaw;
        // Convert from quaternion to RPY Euler angles
        tf::Quaternion q(odometry_msg->pose.pose.orientation.x,odometry_msg->pose.pose.orientation.y,odometry_msg->pose.pose.orientation.z,odometry_msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(_roll, _pitch, _yaw);
        // Save the initial yaw
        initial_yaw_rad_ = _yaw;
        initial_yaw_deg_ = kRadToDeg * _yaw;
        // Finalize
        first_odometry_message_ = false;
        ROS_INFO("[controller-node] Yaw initialized.");
        ROS_INFO("[controller-node] Initial roll: %f .", _roll);
        ROS_INFO("[controller-node] Initial pitch: %f .", _pitch);
        ROS_INFO("[controller-node] Initial yaw: %f .", _yaw);
        ROS_INFO("[controller-node] Initial yaw in deg: %f .", initial_yaw_deg_);
    }
}

void controller_node::stateCallBack(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
    if (msg->armed){
        ROS_INFO_ONCE("[controller-node] ARMED - State_msg.");
    }
    else {
        ROS_INFO("[controller-node] NOT ARMED - State_msg.");
    }

    if (msg->mode == "OFFBOARD"){
        ROS_INFO_ONCE("[controller-node] OFFBOARD - State_msg.");
    }
    else {
        ROS_INFO("[controller-node] NOT OFFBOARD - State_msg.");
    }
}

void controller_node::batteryCallBack(const sensor_msgs::BatteryStateConstPtr& msg){
    if (_use_voltage_compensation){
        ROS_INFO_ONCE("[controller-node] Controller got first battery msg.");
        if (msg->location == "id0"){    // Only read the status of battery #1 (in case of multiple batteries)
            voltage_percentage_ = (msg->voltage) / _model_max_voltage;
            if (voltage_percentage_ < 0.85 || voltage_percentage_ > 1.1){
                ROS_WARN("[controller-node] Invalid voltage value: %f", msg->voltage);
                ROS_WARN("[controller-node] Turning off Voltage compensation.");
                valid_battery_reading_ = false;
            }
            else {
                valid_battery_reading_ = true;
                updateVoltageCompThrottle();
            }   
            ROS_INFO_ONCE("[controller-node] Battery voltage = %f", msg->voltage);
        }
    }
}

void controller_node::dynamicReconfigureCallback(const interaction_controller::parametersConfig &config,
                                                               const uint32_t level) {
    controller_.setKPositionGain(Eigen::Vector3d(config.K_p_x, config.K_p_y, config.K_p_z));
    controller_.setKVelocityGain(Eigen::Vector3d(config.K_v_x, config.K_v_y, config.K_v_z));
    controller_.setKAttitudeGain(Eigen::Vector3d(config.K_R_x, config.K_R_y, config.K_R_z));
    controller_.setKAngularRateGain(Eigen::Vector3d(config.K_w_x, config.K_w_y, config.K_w_z));
    controller_.setKO(Eigen::Vector3d(config.K_o_x, config.K_o_y, config.K_o_z));
    controller_.setDO(Eigen::Vector3d(config.D_x, config.D_y, config.D_z));
    if (control_initialized_frame_){
        controller_.setPositionCmd(Eigen::Vector3d((std::cos(initial_yaw_rad_) * config.x_cmd) 
                                    - (std::sin(initial_yaw_rad_) * config.y_cmd),
                                    (std::sin(initial_yaw_rad_) * config.x_cmd) 
                                    + (std::cos(initial_yaw_rad_) * config.y_cmd),
                                    config.z_cmd));
        controller_.setYawCmd(config.yaw_cmd + initial_yaw_deg_);

    }
    else{
        controller_.setPositionCmd(Eigen::Vector3d(config.x_cmd, config.y_cmd, config.z_cmd));
        controller_.setYawCmd(config.yaw_cmd);
    }
    controller_.setUseDynamicRecCmds(config.use_dyn_cmds);
    controller_.setUsePx4Inverse(config.use_px4_inverse);
    controller_.setEPOffset(Eigen::Vector3d(config.off_x, config.off_y, config.off_z));
    ROS_INFO("[controller-node] Dynamic-reconfig parameters updated!");
}

void controller_node::updateVoltageCompThrottle(){
    voltage_compensation_throttle_ = _model_slope * (voltage_percentage_ - _model_max_voltage_percentage_constant);
    if (voltage_compensation_throttle_ < 0) voltage_compensation_throttle_ = 0;
    else if (voltage_compensation_throttle_ > _model_max_voltage_compensation) voltage_compensation_throttle_ = 0.2;
    voltage_comp_msg.data = voltage_compensation_throttle_;
    voltage_comp_pub_.publish(voltage_comp_msg);

}

void controller_node::publishActuatorControlMsg(const Eigen::Vector4d& controller_output) {
    // Prepare ActControl msg
    actuator_control_msg.header.stamp = ros::Time::now();
    actuator_control_msg.header.frame_id = "base_link";
    actuator_control_msg.group_mix = 0;
    actuator_control_msg.controls[0] = controller_output[0];
    actuator_control_msg.controls[1] = - controller_output[1];       // minus sign because PX4 uses NED frame
    actuator_control_msg.controls[2] = - controller_output[2];       // minus sign because PX4 uses NED frame
    actuator_control_msg.controls[3] = controller_output[3];
    if (controller_output[3] < 0.2){
        actuator_control_msg.controls[3] = 0.2;
    }
    else {
        actuator_control_msg.controls[3] = controller_output[3];
    }
    actuator_control_pub_.publish(actuator_control_msg);
}

void controller_node::publishAttitudeTargetMsg(const Eigen::Vector4d& controller_output, const Eigen::Quaterniond& desired_quaternion) {
    // According to: Alexis, Kostas, Aerial robotic contact-based inspection: planning and control
    // It is safe to use PID Attitude controllers for interaction
    
    // Prepare AttitudeTarget msg
    attitude_target_msg.header.stamp = ros::Time::now();
    attitude_target_msg.header.frame_id = "base_link";
    // attitude_target_msg.type_mask = 0x80; // ignore_attitude
    attitude_target_msg.orientation.x = desired_quaternion.x();
    attitude_target_msg.orientation.y = desired_quaternion.y();
    attitude_target_msg.orientation.z = desired_quaternion.z();
    attitude_target_msg.orientation.w = desired_quaternion.w();
    // attitude_target_msg.body_rate.x = desAngRates(0);
    // attitude_target_msg.body_rate.y = - desAngRates(1);
    // attitude_target_msg.body_rate.z = - desAngRates(2);
    if (controller_output[3] < 0.2){
        attitude_target_msg.thrust = 0.2;
    }
    else {
        if(_use_voltage_compensation) {
            attitude_target_msg.thrust = controller_output(3) + voltage_compensation_throttle_;
        }
        else attitude_target_msg.thrust = controller_output(3);
    }
    attitude_target_pub_.publish(attitude_target_msg);
}

void controller_node::updateControllerOutput() {
    //  calculate controller output
    Eigen::VectorXd controller_output;
    Eigen::Quaterniond desired_quaternion;
    controller_.calculateControllerOutput(&controller_output, &desired_quaternion);
    if (use_attitude_target_) publishAttitudeTargetMsg(controller_output, desired_quaternion);
    else publishActuatorControlMsg(controller_output);
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
