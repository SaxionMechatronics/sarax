#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <Eigen/Dense>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

class ExamplePlanner {
  public:
    ExamplePlanner();

    // Plans a trajectory to take off from the current position and
    // fly to the given position (while maintaining yaw).
    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory);

    // Plans a trajectory to take off from the start position and
    // fly to the given position (while maintaining yaw).
    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        const Eigen::VectorXd& start_pos,
                        mav_trajectory_generation::Trajectory* trajectory);
                        
    void setCurrentPose(const Eigen::Affine3d &currentPose) {
      current_pose_ = currentPose;
    }

    void setCurrentVelocity(const Eigen::Vector3d &currentVelocity) {
      current_velocity_ = currentVelocity;
    }

    void setMaxV(double maxV) {
      max_v_ = maxV;
    }

    void setMaxA(double maxA) {
      max_a_ = maxA;
    }

private:
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  double max_v_{2}; // m/s
  double max_a_{2}; // m/s^2

};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
