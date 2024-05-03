#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_NODE_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_trajectory_generation_example/example_planner.h>
#include "m4e_mav_trajectory_planner/PlanTakeoff.h"


class ExamplePlannerNode {
  public:
    ExamplePlannerNode();

  private:
    ros::NodeHandle nh_;
    ExamplePlanner planner;

    ros::Subscriber waypoint_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::ServiceServer takeoff_service;

    void waypointCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool takeOffServiceRequest(m4e_mav_trajectory_planner::PlanTakeoff::Request& req,
                          m4e_mav_trajectory_planner::PlanTakeoff::Response& res);
    void loadParameters();
    mav_trajectory_generation::Trajectory trajectory;
    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
    Eigen::Vector3d wp_position, wp_velocity;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    // Topics names
    std::string waypoint_topic_;
    std::string unsampled_traj_topic_;
    std::string traj_markers_topic_;
    std::string odometry_topic_;
    std::string plan_takeoff_service_name_;
};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_NODE_H
