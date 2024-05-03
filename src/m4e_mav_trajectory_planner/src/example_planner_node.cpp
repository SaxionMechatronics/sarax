/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include <mav_trajectory_generation_example/example_planner_node.h>

ExamplePlannerNode::ExamplePlannerNode()
{
  loadParameters();
  // Subscribe to "waypoint" topic
  waypoint_sub_ = nh_.subscribe(waypoint_topic_, 1, &ExamplePlannerNode::waypointCallback, this);

  // Subscribe to "odom" topic
  odom_sub_ = nh_.subscribe(odometry_topic_, 1, &ExamplePlannerNode::odomCallback, this);

  // Publish to "trajectory" topic
  pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>(unsampled_traj_topic_, 1);
  // create publisher for RVIZ markers
  pub_markers_ =
          nh_.advertise<visualization_msgs::MarkerArray>(traj_markers_topic_, 0);

  // Create the service server
  takeoff_service = nh_.advertiseService(plan_takeoff_service_name_, &ExamplePlannerNode::takeOffServiceRequest, this);

}

void ExamplePlannerNode::waypointCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  // Callback function for "waypoint" topic
  // define set point
  wp_position << msg->vector.x, msg->vector.y, msg->vector.z;
  wp_velocity << 0.0, 0.0, 0.0;
  planner.planTrajectory(wp_position, wp_velocity, &trajectory);
//  planner.publishTrajectory(trajectory);
  publishTrajectory(trajectory);
}

void ExamplePlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
  // Pass the odometry info to the planner
  planner.setCurrentPose(current_pose_);
  planner.setCurrentVelocity(current_velocity_);
}

void ExamplePlannerNode::loadParameters() {
  double max_v_;
  double max_a_;
  // Load params
  if (!nh_.getParam("uav_parameters/max_v", max_v_)){
    ROS_ERROR("[planner_node] param max_v not found.");
    planner.setMaxV(2);
  }
  else{
    planner.setMaxV(max_v_);
  }
  if (!nh_.getParam("uav_parameters/max_a", max_a_)){
    ROS_ERROR("[planner_node] param max_a not found.");
    planner.setMaxA(2);
  }
  else{
    planner.setMaxA(max_a_);
  }
  if (!nh_.getParam("topics_names/waypoint_topic", waypoint_topic_)){
    ROS_ERROR("[planner_node] Could not find topics_names/waypoint_topic parameter!");
    waypoint_topic_ = "waypoint";
  }
  if (!nh_.getParam("topics_names/odometry_topic", odometry_topic_)){
    ROS_ERROR("[planner_node] Could not find topics_names/odometry_topic parameter!");
    odometry_topic_ = "mavros/odometry/in";
  }
  if (!nh_.getParam("topics_names/unsampled_trajectory_topic", unsampled_traj_topic_)){
    ROS_ERROR("[planner_node] Could not find topics_names/unsampled_trajectory_topic parameter!");
    unsampled_traj_topic_ = "unsampled_trajectory";
  }
  if (!nh_.getParam("topics_names/traj_markers_topic", traj_markers_topic_)){
    ROS_ERROR("[planner_node] Could not find topics_names/traj_markers_topic parameter!");
    traj_markers_topic_ = "trajectory_markers";
  }
  if (!nh_.getParam("services_names/plan_takeoff_client_name", plan_takeoff_service_name_)){
    ROS_ERROR("[planner_node] Could not find services_names/plan_takeoff_client_name parameter!");
    plan_takeoff_service_name_ = "plan_takeoff";
  }
}

bool ExamplePlannerNode::publishTrajectory(const mav_trajectory_generation::Trajectory &_trajectory) {
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
          0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory( _trajectory,
                                                distance,
                                                frame_id,
                                                &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg( _trajectory,
                                                                  &msg);
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  pub_trajectory_.publish(msg);

  return true;
}

bool ExamplePlannerNode::takeOffServiceRequest(m4e_mav_trajectory_planner::PlanTakeoff::Request& req,
                                              m4e_mav_trajectory_planner::PlanTakeoff::Response& res){
  // Define set points
  Eigen::Vector3d start_position(current_pose_.translation()[0], current_pose_.translation()[1], -2);
  Eigen::Vector3d wp_position(current_pose_.translation()[0], current_pose_.translation()[1], req.goal_altitude);
  Eigen::Vector3d wp_velocity(0.0, 0.0, 0.0);

  try {
    // Perform trajectory planning
    bool planning_success = planner.planTrajectory(wp_position, wp_velocity, start_position, &trajectory);

    if (planning_success){
      // Publish the generated trajectory
      publishTrajectory(trajectory);

      // Set the response flag to indicate success
      res.success = true;
    }
    else{
      // Planning failed, set the response flag to indicate failure
      res.success = false;
    }
  }
  catch (const std::exception& e)
  {
    // An exception occurred during planning
    ROS_ERROR_STREAM("[planner_node] Exception occurred during takeoff trajectory planning: " << e.what());
    res.success = false;
  }

  return true;  // Return true to indicate a successful service call
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ExamplePlannerNode example_planner_node;

  ros::Rate rate(100); // Set the loop rate to 50Hz

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}