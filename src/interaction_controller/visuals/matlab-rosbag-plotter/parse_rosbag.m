%% Script to parse the rosbag QUICKLY
%
%   Ayham Alharbat
%
%% Parse Position data
topic_select.odometryIn = select(bag,'Topic','/mavros/odometry/in');
structs.odometry = readMessages(topic_select.odometryIn,'DataFormat','struct');
odometry_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.odometry);
plotdata.estimated_states.p.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.odometry);
plotdata.estimated_states.p.Time = plotdata.estimated_states.p.Time + (odometry_Nsec./1e9);
settings.t0 = plotdata.estimated_states.p.Time(1);
plotdata.estimated_states.p.Time = plotdata.estimated_states.p.Time - settings.t0 - settings.time_offset;
plotdata.estimated_states.p.Data(:,1) = cellfun(@(m) double(m.Pose.Pose.Position.X),structs.odometry);
plotdata.estimated_states.p.Data(:,2) = cellfun(@(m) double(m.Pose.Pose.Position.Y),structs.odometry);
plotdata.estimated_states.p.Data(:,3) = cellfun(@(m) double(m.Pose.Pose.Position.Z),structs.odometry);
clear odometry_Nsec
%% read orientation data
plotdata.estimated_states.q.Time = plotdata.estimated_states.p.Time;
plotdata.estimated_states.q.Data(:,1) = cellfun(@(m) double(m.Pose.Pose.Orientation.X),structs.odometry);
plotdata.estimated_states.q.Data(:,2) = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),structs.odometry);
plotdata.estimated_states.q.Data(:,3) = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),structs.odometry);
plotdata.estimated_states.q.Data(:,4) = cellfun(@(m) double(m.Pose.Pose.Orientation.W),structs.odometry);
% Convert to euler angles
% Stack quat in a matrix to convert them to RPY
RPY = quatToEuler([plotdata.estimated_states.q.Data(:,4)';plotdata.estimated_states.q.Data(:,1)';...
    plotdata.estimated_states.q.Data(:,2)';plotdata.estimated_states.q.Data(:,3)']);
plotdata.estimated_states.euler.Time = plotdata.estimated_states.q.Time;
plotdata.estimated_states.euler.Data(:,1) = RPY(1,:) .* (180/pi);
plotdata.estimated_states.euler.Data(:,2) = RPY(2,:) .* (180/pi);
plotdata.estimated_states.euler.Data(:,3) = RPY(3,:) .* (180/pi);
clear RPY
%% read velocity data
plotdata.estimated_states.v.Time = plotdata.estimated_states.p.Time;
plotdata.estimated_states.v.Data(:,1) = cellfun(@(m) double(m.Twist.Twist.Linear.X),structs.odometry);
plotdata.estimated_states.v.Data(:,2) = cellfun(@(m) double(m.Twist.Twist.Linear.Y),structs.odometry);
plotdata.estimated_states.v.Data(:,3) = cellfun(@(m) double(m.Twist.Twist.Linear.Z),structs.odometry);
%% read angular velocity data
plotdata.estimated_states.omega.Time = plotdata.estimated_states.p.Time;
plotdata.estimated_states.omega.Data(:,1) = cellfun(@(m) double(m.Twist.Twist.Angular.X),structs.odometry);
plotdata.estimated_states.omega.Data(:,2) = cellfun(@(m) double(m.Twist.Twist.Angular.Y),structs.odometry);
plotdata.estimated_states.omega.Data(:,3) = cellfun(@(m) double(m.Twist.Twist.Angular.Z),structs.odometry);
%% Read references
topic_select.pose_cmd = select(bag,'Topic','/debug/command_pose');
structs.pose_cmd = readMessages(topic_select.pose_cmd,'DataFormat','struct');
pose_cmd_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.pose_cmd);
plotdata.references.p.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.pose_cmd);
plotdata.references.p.Time = plotdata.references.p.Time + (pose_cmd_Nsec./1e9);
plotdata.references.p.Time = plotdata.references.p.Time - settings.t0 - settings.time_offset;
plotdata.references.p.Data(:,1) = cellfun(@(m) double(m.Pose.Pose.Position.X),structs.pose_cmd);
plotdata.references.p.Data(:,2) = cellfun(@(m) double(m.Pose.Pose.Position.Y),structs.pose_cmd);
plotdata.references.p.Data(:,3) = cellfun(@(m) double(m.Pose.Pose.Position.Z),structs.pose_cmd);
clear pose_cmd_Nsec
%% Read position error
topic_select.error_position = select(bag,'Topic','/debug/pos_error');
structs.error_position = readMessages(topic_select.error_position,'DataFormat','struct');
error_position_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.error_position);
plotdata.errors.p.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.error_position);
plotdata.errors.p.Time = plotdata.errors.p.Time + (error_position_Nsec./1e9);
plotdata.errors.p.Time = plotdata.errors.p.Time - settings.t0 - settings.time_offset;
plotdata.errors.p.Data(:,1) = cellfun(@(m) double(m.Vector.X),structs.error_position);
plotdata.errors.p.Data(:,2) = cellfun(@(m) double(m.Vector.Y),structs.error_position);
plotdata.errors.p.Data(:,3) = cellfun(@(m) double(m.Vector.Z),structs.error_position);
clear error_position_Nsec
%% Read attitude error
topic_select.att_error = select(bag,'Topic','/debug/att_error');
structs.att_error = readMessages(topic_select.att_error,'DataFormat','struct');
att_error_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.att_error);
plotdata.errors.attitude.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.att_error);
plotdata.errors.attitude.Time = plotdata.errors.attitude.Time + (att_error_Nsec./1e9);
plotdata.errors.attitude.Time = plotdata.errors.attitude.Time - settings.t0 - settings.time_offset;
plotdata.errors.attitude.Data(:,1) = cellfun(@(m) double(m.Vector.X),structs.att_error);
plotdata.errors.attitude.Data(:,2) = cellfun(@(m) double(m.Vector.Y),structs.att_error);
plotdata.errors.attitude.Data(:,3) = cellfun(@(m) double(m.Vector.Z),structs.att_error);
clear att_error_Nsec
%% Read ActuatorControl topic
topic_select.actuator_control = select(bag,'Topic','/mavros/actuator_control');
structs.actuator_control = readMessages(topic_select.actuator_control,'DataFormat','struct');
ActCmdsNsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.actuator_control);
plotdata.controller.actuator_control.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.actuator_control);
plotdata.controller.actuator_control.Time = plotdata.controller.actuator_control.Time + (ActCmdsNsec./1e9);
plotdata.controller.actuator_control.Time = plotdata.controller.actuator_control.Time - settings.t0 - settings.time_offset;
plotdata.controller.actuator_control.xtorque = cellfun(@(m) double(m.Controls(1)),structs.actuator_control);
plotdata.controller.actuator_control.ytorque = cellfun(@(m) double(m.Controls(2)),structs.actuator_control);
plotdata.controller.actuator_control.ztorque = cellfun(@(m) double(m.Controls(3)),structs.actuator_control);
plotdata.controller.actuator_control.thrust  = cellfun(@(m) double(m.Controls(4)),structs.actuator_control);
clear ActCmdsNsec
%% Read desired Force commands from the controller
topic_select.force_d = select(bag,'Topic','/debug/force3D');
structs.force_d = readMessages(topic_select.force_d,'DataFormat','struct');
force_d_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.force_d);
plotdata.controller.force_d.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.force_d);
plotdata.controller.force_d.Time = plotdata.controller.force_d.Time + (force_d_Nsec./1e9);
plotdata.controller.force_d.Time = plotdata.controller.force_d.Time - settings.t0 - settings.time_offset;
plotdata.controller.force_d.Data(:,1) = cellfun(@(m) double(m.Vector.X),structs.force_d);
plotdata.controller.force_d.Data(:,2) = cellfun(@(m) double(m.Vector.Y),structs.force_d);
plotdata.controller.force_d.Data(:,3) = cellfun(@(m) double(m.Vector.Z),structs.force_d);
clear force_d_Nsec
%% Read desired Torque commands from the controller
topic_select.torque_d = select(bag,'Topic','/debug/torque3D');
structs.torque_d = readMessages(topic_select.torque_d,'DataFormat','struct');
torque_d_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.torque_d);
plotdata.controller.torque_d.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.torque_d);
plotdata.controller.torque_d.Time = plotdata.controller.torque_d.Time + (torque_d_Nsec./1e9);
plotdata.controller.torque_d.Time = plotdata.controller.torque_d.Time - settings.t0 - settings.time_offset;
plotdata.controller.torque_d.Data(:,1) = cellfun(@(m) double(m.Vector.X),structs.torque_d);
plotdata.controller.torque_d.Data(:,2) = cellfun(@(m) double(m.Vector.Y),structs.torque_d);
plotdata.controller.torque_d.Data(:,3) = cellfun(@(m) double(m.Vector.Z),structs.torque_d);
clear torque_d_Nsec
%% Read desired Orientation commands from the controller
if (isempty(structs.actuator_control))
    topic_select.attitude_target = select(bag,'Topic','/mavros/setpoint_raw/attitude');
    structs.attitude_target = readMessages(topic_select.attitude_target,'DataFormat','struct');
    attitude_target_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.attitude_target);
    plotdata.references.q.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.attitude_target);
    plotdata.references.q.Time = plotdata.references.q.Time + (attitude_target_Nsec./1e9);
    plotdata.references.q.Time = plotdata.references.q.Time - settings.t0 - settings.time_offset;
    plotdata.references.q.Data(:,1) = cellfun(@(m) double(m.Orientation.X),structs.attitude_target);
    plotdata.references.q.Data(:,2) = cellfun(@(m) double(m.Orientation.Y),structs.attitude_target);
    plotdata.references.q.Data(:,3) = cellfun(@(m) double(m.Orientation.Z),structs.attitude_target);
    plotdata.references.q.Data(:,4) = cellfun(@(m) double(m.Orientation.W),structs.attitude_target);
    plotdata.controller.actuator_control.thrust  = cellfun(@(m) double(m.Thrust),structs.attitude_target);
    plotdata.controller.actuator_control.Time  = plotdata.references.q.Time;
else
    topic_select.attitude_target = select(bag,'Topic','debug/dersired_R');
    structs.attitude_target = readMessages(topic_select.attitude_target,'DataFormat','struct');
    attitude_target_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.attitude_target);
    plotdata.references.q.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.attitude_target);
    plotdata.references.q.Time = plotdata.references.q.Time + (attitude_target_Nsec./1e9);
    plotdata.references.q.Time = plotdata.references.q.Time - settings.t0 - settings.time_offset;
    plotdata.references.q.Data(:,1) = cellfun(@(m) double(m.Pose.Pose.Orientation.X),structs.attitude_target);
    plotdata.references.q.Data(:,2) = cellfun(@(m) double(m.Pose.Pose.Orientation.Y),structs.attitude_target);
    plotdata.references.q.Data(:,3) = cellfun(@(m) double(m.Pose.Pose.Orientation.Z),structs.attitude_target);
    plotdata.references.q.Data(:,4) = cellfun(@(m) double(m.Pose.Pose.Orientation.W),structs.attitude_target);
end
% Convert to euler angles
% Stack quat in a matrix to convert them to RPY
d_RPY = quatToEuler([plotdata.references.q.Data(:,4)';plotdata.references.q.Data(:,1)';...
    plotdata.references.q.Data(:,2)';plotdata.references.q.Data(:,3)']);
plotdata.references.euler.Time = plotdata.references.q.Time;
plotdata.references.euler.Data(:,1) = d_RPY(1,:) .* (180/pi);
plotdata.references.euler.Data(:,2) = d_RPY(2,:) .* (180/pi);
plotdata.references.euler.Data(:,3) = d_RPY(3,:) .* (180/pi);
clear attitude_target_Nsec d_RPY
%% Read optitrack raw data (not fused in PX4's EKF2)
topic_select.optitrack = select(bag,'Topic','/mavros/vision_pose/pose');
structs.optitrack = readMessages(topic_select.optitrack,'DataFormat','struct');
optitrack_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.optitrack);
plotdata.optitrack.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.optitrack);
plotdata.optitrack.Time = plotdata.optitrack.Time + (optitrack_Nsec./1e9);
plotdata.optitrack.Time = plotdata.optitrack.Time - settings.t0 - settings.time_offset;
plotdata.optitrack.position.x = cellfun(@(m) double(m.Pose.Position.X),structs.optitrack);
plotdata.optitrack.position.y = cellfun(@(m) double(m.Pose.Position.Y),structs.optitrack);
plotdata.optitrack.position.z = cellfun(@(m) double(m.Pose.Position.Z),structs.optitrack);
plotdata.optitrack.orientation.x = cellfun(@(m) double(m.Pose.Orientation.X),structs.optitrack);
plotdata.optitrack.orientation.y = cellfun(@(m) double(m.Pose.Orientation.Y),structs.optitrack);
plotdata.optitrack.orientation.z = cellfun(@(m) double(m.Pose.Orientation.Z),structs.optitrack);
plotdata.optitrack.orientation.w = cellfun(@(m) double(m.Pose.Orientation.W),structs.optitrack);
% Convert to euler angles
% Stack quat in a matrix to convert them to RPY
optitrack_RPY = quatToEuler([plotdata.optitrack.orientation.w';plotdata.optitrack.orientation.x';...
    plotdata.optitrack.orientation.y';plotdata.optitrack.orientation.z']);
plotdata.optitrack.euler.x = optitrack_RPY(1,:) .* (180/pi);
plotdata.optitrack.euler.y = optitrack_RPY(2,:) .* (180/pi);
plotdata.optitrack.euler.z = optitrack_RPY(3,:) .* (180/pi);
clear optitrack_Nsec optitrack_RPY
%% Parse battery data
topic_select.battery = select(bag,'Topic','/mavros/battery');
structs.battery = readMessages(topic_select.battery,'DataFormat','struct');
battery_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.battery);
plotdata.battery.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.battery);
plotdata.battery.Time = plotdata.battery.Time + (battery_Nsec./1e9);
plotdata.battery.Time = plotdata.battery.Time - settings.t0 - settings.time_offset;
plotdata.battery.voltage = cellfun(@(m) double(m.Voltage),structs.battery);
plotdata.battery.current = cellfun(@(m) double(m.Current),structs.battery);
clear battery_Nsec
%% Parse voltage comp data
try
    topic_select.voltage_comp = select(bag,'Topic','/debug/voltage_comp');
    structs.voltage_comp = readMessages(topic_select.voltage_comp,'DataFormat','struct');
    plotdata.voltage_comp.Data = cellfun(@(m) double(m.Data),structs.voltage_comp);
    plotdata.voltage_comp.Time = plotdata.controller.force_d.Time;
catch
    warning('/debug/voltage_comp topic is not available');
end
%% parse arm joints states
if (settings.with_arm)
    topic_select.estimated_states.joints_states = select(bag,'Topic','/m4e_mani/joint_states');
    structs.estimated_states.joints_states = readMessages(topic_select.estimated_states.joints_states,'DataFormat','struct');
    joints_states_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.Time = plotdata.estimated_states.joints_states.Time + (joints_states_Nsec./1e9);
    settings.t0 = plotdata.estimated_states.joints_states.Time(1);
    plotdata.estimated_states.joints_states.Time = plotdata.estimated_states.joints_states.Time - settings.t0 - settings.time_offset;
    
    %           NOTE THE SWITCH BETWEEN JOINT 2 AND 3, AND THE NEGATIVE SIGN
    
    plotdata.estimated_states.joints_states.position(:,1) = cellfun(@(m) double(m.Position(1)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.position(:,2) = -cellfun(@(m) double(m.Position(3)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.position(:,3) = -cellfun(@(m) double(m.Position(2)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.velocity(:,1) = cellfun(@(m) double(m.Velocity(1)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.velocity(:,2) = -cellfun(@(m) double(m.Velocity(3)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.velocity(:,3) = -cellfun(@(m) double(m.Velocity(2)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.effort(:,1) = cellfun(@(m) double(m.Effort(1)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.effort(:,2) = cellfun(@(m) double(m.Effort(3)),structs.estimated_states.joints_states);
    plotdata.estimated_states.joints_states.effort(:,3) = cellfun(@(m) double(m.Effort(2)),structs.estimated_states.joints_states);
    
    %% parse arm torque control commands
    topic_select.effort_commands = select(bag,'Topic','/m4e_mani/joint_efforts_stamped');
    structs.effort_commands = readMessages(topic_select.effort_commands,'DataFormat','struct');
    arm_torque_commands_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.effort_commands);
    plotdata.controller.arm_torque_commands.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.effort_commands);
    plotdata.controller.arm_torque_commands.Time = plotdata.controller.arm_torque_commands.Time + (arm_torque_commands_Nsec./1e9);
    plotdata.controller.arm_torque_commands.Time = plotdata.controller.arm_torque_commands.Time - settings.t0;
    plotdata.controller.arm_torque_commands.Data(:,1) = cellfun(@(m) double(m.Vector.X),structs.effort_commands);
    plotdata.controller.arm_torque_commands.Data(:,2) = cellfun(@(m) double(m.Vector.Y),structs.effort_commands);
    plotdata.controller.arm_torque_commands.Data(:,3) = cellfun(@(m) double(m.Vector.Z),structs.effort_commands);
    
    %% parse arm joints references
    topic_select.theta_references = select(bag,'Topic','/m4e_mani/joints_commands');
    structs.theta_references = readMessages(topic_select.theta_references,'DataFormat','struct');
    theta_references_Nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),structs.theta_references);
    plotdata.references.theta_references.Time = cellfun(@(m) double(m.Header.Stamp.Sec),structs.theta_references);
    plotdata.references.theta_references.Time = plotdata.references.theta_references.Time + (theta_references_Nsec./1e9);
    plotdata.references.theta_references.Time = plotdata.references.theta_references.Time - settings.t0 - settings.time_offset;
    plotdata.references.theta_references.Data(:,1) = cellfun(@(m) double(m.Vector.X),structs.theta_references);
    plotdata.references.theta_references.Data(:,2) = cellfun(@(m) double(m.Vector.Y),structs.theta_references);
    plotdata.references.theta_references.Data(:,3) = cellfun(@(m) double(m.Vector.Z),structs.theta_references);
end
%% clean up
clear topic_select structs 
%% Required function
function euler = quatToEuler(quat)
%	Converts quaternions vector to ZYX Euler angles (RPY)
%
%   The unit quaternions (quat) must be written as a 4-by-N matrix (with the scalar
%   element as the FIRST component)
%   The returned Euler angles vector will be written in radiants as a 3-by-N vector.

a = quat(1,:);
b = quat(2,:);
c = quat(3,:);
d = quat(4,:);

A11 = a.^2 + b.^2 - c.^2 - d.^2;
A12 = 2*b.*c - 2*a.*d;
A13 = 2*a.*c + 2*b.*d;
% A21 = 2*a.*d + 2*b.*c;
% A22 = a.^2 + b.^2 + c.^2 - d.^2;
A23 = 2*c.*d - 2*a.*b;
% A31 = 2*b.*d - 2*a.*c;
% A32 = 2*a.*b + 2*c.*d;
A33 = a.^2 - b.^2 - c.^2 + d.^2;

% R = [A11(1) A12(1) A13(1) ; A21(1) A22(1) A23(1) ; A31(1) A32(1) A33(1)];

phi = -atan2(A23,A33);
theta = asin(A13);
psi = -atan2(A12,A11);

euler = [phi; theta; psi];

end
