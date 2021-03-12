/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Recovery behavior of step back and steer turning
 *
 * \author Masaru Morita
 */

#ifndef STEPBACK_AND_STEERTURN_RECOVERY_H
#define STEPBACK_AND_STEERTURN_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

namespace gm=geometry_msgs;
namespace cmap=costmap_2d;
namespace blp=base_local_planner;
using std::vector;
using std::max;
namespace stepback_and_steerturn_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class StepBackAndSteerTurnRecovery : public nav_core::RecoveryBehavior
{
public:
  
  /// Doesn't do anything: initialize is where the actual work happens
  StepBackAndSteerTurnRecovery();

  ~StepBackAndSteerTurnRecovery();

  /// Initialize the parameters of the behavior
  void initialize (std::string n, tf2_ros::Buffer* tf,
                   costmap_2d::Costmap2DROS* global_costmap,
                   costmap_2d::Costmap2DROS* local_costmap);

  /// Run the behavior
  void runBehavior();

private:
  enum COSTMAP_SEARCH_MODE
  {
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD
  };

  enum TURN_DIRECTION
  {
    LEFT,
    RIGHT,
  };

  enum TURN_NO
  {
    FIRST_TURN = 0,
    SECOND_TURN = 1,
  };
  static const int CNT_TURN = 2;

  gm::Twist TWIST_STOP;

  gm::Pose2D getCurrentLocalPose () const;
  gm::Twist scaleGivenAccelerationLimits (const gm::Twist& twist, const double time_remaining) const;
  gm::Pose2D getPoseToObstacle (const gm::Pose2D& current, const gm::Twist& twist) const;
  double normalizedPoseCost (const gm::Pose2D& pose) const;
  gm::Twist transformTwist (const gm::Pose2D& pose) const;
  void moveSpacifiedLength (const gm::Twist twist, const double length, const COSTMAP_SEARCH_MODE mode = FORWARD) const;
  double getCurrentDiff(const gm::Pose2D initialPose, const COSTMAP_SEARCH_MODE mode = FORWARD) const;
  double getCurrentDistDiff(const gm::Pose2D initialPose, const double distination, const COSTMAP_SEARCH_MODE mode = FORWARD) const;
  double getMinimalDistanceToObstacle(const COSTMAP_SEARCH_MODE mode) const;
  int determineTurnDirection();
  double getDistBetweenTwoPoints(const gm::Pose2D pose1, const gm::Pose2D pose2) const;


  ros::NodeHandle nh_;
  costmap_2d::Costmap2DROS* global_costmap_;
  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2D costmap_; // Copy of local_costmap_, used by world model
  std::string name_;
  tf2_ros::Buffer* tf_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher recover_run_pub_;
  bool initialized_;

  // Memory owned by this object
  // Mutable because footprintCost is not declared const
  mutable base_local_planner::CostmapModel* world_model_;

  gm::Twist base_frame_twist_;
  
  double duration_;
  double linear_speed_limit_;
  double angular_speed_limit_;
  double linear_acceleration_limit_;
  double angular_acceleration_limit_;
  double controller_frequency_;
  double simulation_frequency_;
  double simulation_inc_;

  bool only_single_steering_;
  int trial_times_;
  double obstacle_patience_;
  double obstacle_check_frequency_;
  double sim_angle_resolution_;
  //-- back
  double linear_vel_back_;
  double step_back_length_;
  double step_back_timeout_;
  //-- steer
  double linear_vel_steer_;
  double angular_speed_steer_;
  double turn_angle_;
  double steering_timeout_;
  //-- forward
  double linear_vel_forward_;
  double step_forward_length_;
  double step_forward_timeout_;

};

} // namespace stepback_and_steerturn_recovery

#endif // include guard
