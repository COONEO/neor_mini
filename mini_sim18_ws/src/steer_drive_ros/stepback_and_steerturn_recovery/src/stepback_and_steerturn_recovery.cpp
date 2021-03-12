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
 * \author Masaru Morita
 * 
 */

#include <stepback_and_steerturn_recovery/stepback_and_steerturn_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS( stepback_and_steerturn_recovery::StepBackAndSteerTurnRecovery, nav_core::RecoveryBehavior)

namespace stepback_and_steerturn_recovery
{

StepBackAndSteerTurnRecovery::StepBackAndSteerTurnRecovery () :
  global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false)
{
    TWIST_STOP.linear.x = 0.0;
    TWIST_STOP.linear.y = 0.0;
    TWIST_STOP.linear.z = 0.0;
    TWIST_STOP.angular.x = 0.0;
    TWIST_STOP.angular.y = 0.0;
    TWIST_STOP.angular.z = 0.0;
}

StepBackAndSteerTurnRecovery::~StepBackAndSteerTurnRecovery ()
{
  delete world_model_;
}

void StepBackAndSteerTurnRecovery::initialize (std::string name, tf2_ros::Buffer* tf,
                                cmap::Costmap2DROS* global_cmap, cmap::Costmap2DROS* local_cmap)
{
  ROS_ASSERT(!initialized_);
  name_ = name;
  tf_ = tf;
  local_costmap_ = local_cmap;
  global_costmap_ = global_cmap;
  /*
  local_costmap_->getCostmapCopy(costmap_);
  world_model_ = new blp::CostmapModel(costmap_);
  */
  world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

  cmd_vel_pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
  recover_run_pub_ = nh_.advertise<std_msgs::Bool>("recover_run", 10);
  ros::NodeHandle private_nh("~/" + name);

  /*
  {
      bool found=true;
      found = found && private_nh.getParam("linear_x", base_frame_twist_.linear.x);
      found = found && private_nh.getParam("linear_y", base_frame_twist_.linear.y);
      found = found && private_nh.getParam("angular_z", base_frame_twist_.angular.z);
      if (!found) {
          ROS_FATAL_STREAM ("Didn't find twist parameters in " << private_nh.getNamespace());
          ros::shutdown();
      }
  }
  */

  private_nh.param("duration", duration_, 1.0);
  private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
  private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
  private_nh.param("linear_acceleration_limit", linear_acceleration_limit_, 4.0);
  private_nh.param("angular_acceleration_limit", angular_acceleration_limit_, 3.2);
  private_nh.param("controller_frequency", controller_frequency_, 20.0);
  private_nh.param("simulation_frequency", simulation_frequency_, 5.0);
  private_nh.param("simulation_inc", simulation_inc_, 1/simulation_frequency_);

  private_nh.param("only_single_steering", only_single_steering_, true);
  private_nh.param("trial_times", trial_times_, 5);
  private_nh.param("obstacle_patience", obstacle_patience_, 0.5);
  private_nh.param("obstacle_check_frequency", obstacle_check_frequency_, 5.0);
  private_nh.param("sim_angle_resolution", sim_angle_resolution_, 0.1);

  // back
  private_nh.param("linear_vel_back", linear_vel_back_, -0.3);
  private_nh.param("step_back_length", step_back_length_, 1.0);
  private_nh.param("step_back_timeout", step_back_timeout_, 15.0);
  //-- steer
  private_nh.param("linear_vel_steer", linear_vel_steer_, 0.3);
  private_nh.param("angular_speed_steer", angular_speed_steer_, 0.5);
  private_nh.param("turn_angle", turn_angle_, 2.0);
  private_nh.param("steering_timeout", steering_timeout_, 15.0);
  //-- forward
  private_nh.param("linear_vel_forward", linear_vel_forward_, 0.3);
  private_nh.param("step_forward_length", step_forward_length_, 0.5);
  private_nh.param("step_forward_timeout", step_forward_timeout_, 15.0);

  /*
  ROS_INFO_STREAM_NAMED ("top", "Initialized twist recovery with twist " <<
                          base_frame_twist_ << " and duration " << duration_);
  */

  ROS_INFO_NAMED ("top", "Initialized with only_single_steering = %s", (only_single_steering_ ? "true" : "false"));
  ROS_INFO_NAMED ("top", "Initialized with recovery_trial_times = %d", trial_times_);
  ROS_INFO_NAMED ("top", "Initialized with obstacle_patience = %.2f", obstacle_patience_);
  ROS_INFO_NAMED ("top", "Initialized with obstacle_check_frequency = %.2f", obstacle_check_frequency_);
  ROS_INFO_NAMED ("top", "Initialized with simulation_frequency = %.2f", simulation_frequency_);
  ROS_INFO_NAMED ("top", "Initialized with sim_angle_resolution = %.2f", sim_angle_resolution_);
  ROS_INFO_NAMED ("top", "Initialized with linear_vel_back = %.2f, step_back_length = %.2f, step_back_steering = %.2f",
                  linear_vel_back_, step_back_length_, step_back_timeout_);
  ROS_INFO_NAMED ("top", "Initialized with linear_vel_steer = %.2f, angular_speed_steer = %.2f,"
                         " turn_angle = %.2f, steering_timeout = %.2f",
                  linear_vel_steer_, angular_speed_steer_, turn_angle_, steering_timeout_);
  ROS_INFO_NAMED ("top", "Initialized with linear_vel_forward = %.2f, step_forward_length = %.2f, step_forward_timeout = %.2f",
                  linear_vel_forward_, step_forward_length_, step_forward_timeout_);

  initialized_ = true;
}

gm::Twist scaleTwist (const gm::Twist& twist, const double scale)
{
  gm::Twist t;
  t.linear.x = twist.linear.x * scale;
  t.linear.y = twist.linear.y * scale;
  t.angular.z = twist.angular.z * scale;
  return t;
}

gm::Pose2D forwardSimulate (const gm::Pose2D& p, const gm::Twist& twist, const double t=1.0)
{
  gm::Pose2D p2;
  const double linear_vel = twist.linear.x;
  p2.theta = p.theta + twist.angular.z;//*t;
  p2.x = p.x + linear_vel * cos(p2.theta)*t;
  p2.y = p.y + linear_vel * sin(p2.theta)*t;

  return p2;
}

/// Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
double StepBackAndSteerTurnRecovery::normalizedPoseCost (const gm::Pose2D& pose) const
{
  gm::Point p;
  p.x = pose.x;
  p.y = pose.y;

  unsigned int pose_map_idx_x, pose_map_idx_y;
  costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
  costmap->worldToMap(p.x, p.y, pose_map_idx_x, pose_map_idx_y);  // convert point unit from [m] to [idx]
  ROS_DEBUG_NAMED ("top", "Trying to get cost at (%d, %d) in getCost", pose_map_idx_x, pose_map_idx_y);
  const double c = costmap->getCost(pose_map_idx_x, pose_map_idx_y);

  return c < 0 ? 1e9 : c;
}


/// Return the maximum d <= duration_ such that starting at the current pose, the cost is nonincreasing for
/// d seconds if we follow twist
/// It might also be good to have a threshold such that we're allowed to have lethal cost for at most
/// the first k of those d seconds, but this is not done
gm::Pose2D StepBackAndSteerTurnRecovery::getPoseToObstacle (const gm::Pose2D& current, const gm::Twist& twist) const
{
  double cost = 0;
  cost = normalizedPoseCost(current);
  double t; // Will hold the first time that is invalid
  gm::Pose2D current_tmp = current;
  double next_cost;

  ROS_DEBUG_NAMED ("top", " ");
  for (t=simulation_inc_; t<=duration_ + 500; t+=simulation_inc_) {
      ROS_DEBUG_NAMED ("top", "start loop");
      current_tmp = forwardSimulate(current, twist, t);
      ROS_DEBUG_NAMED ("top", "finish fowardSimulate");
      next_cost = normalizedPoseCost(current_tmp);
      ROS_DEBUG_NAMED ("top", "finish Cost");
    //if (next_cost > cost) {
    if (/*next_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||*/ next_cost == costmap_2d::LETHAL_OBSTACLE) {
      ROS_DEBUG_STREAM_NAMED ("cost", "Cost at " << t << " and pose " << forwardSimulate(current, twist, t)
                              << " is " << next_cost << " which is greater than previous cost " << cost);
      break;
    }
    cost = next_cost;
  }
  ROS_DEBUG_NAMED ("top", "cost = %.2f, next_cost = %.2f", cost, next_cost);
  ROS_DEBUG_NAMED ("top", "twist.linear.x = %.2f, twist.angular.z = %.2f", twist.linear.x, twist.angular.z);
  ROS_DEBUG_NAMED ("top", "init = (%.2f, %.2f, %.2f), current = (%.2f, %.2f, %.2f)",
                  current.x, current.y, current.theta, current_tmp.x, current_tmp.y, current_tmp.theta);
  ROS_DEBUG_NAMED ("top", "time = %.2f", t);

  // return t-simulation_inc_;
  return current_tmp;
}

double linearSpeed (const gm::Twist& twist)
{
  return sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
}

double angularSpeed (const gm::Twist& twist)
{
  return fabs(twist.angular.z);
}

// Scale twist so we can stop in the given time, and so it's within the max velocity
gm::Twist StepBackAndSteerTurnRecovery::scaleGivenAccelerationLimits (const gm::Twist& twist, const double time_remaining) const
{
  const double linear_speed = linearSpeed(twist);
  const double angular_speed = angularSpeed(twist);
  const double linear_acc_scaling = linear_speed/(time_remaining*linear_acceleration_limit_);
  const double angular_acc_scaling = angular_speed/(time_remaining*angular_acceleration_limit_);
  const double acc_scaling = max(linear_acc_scaling, angular_acc_scaling);
  const double linear_vel_scaling = linear_speed/linear_speed_limit_;
  const double angular_vel_scaling = angular_speed/angular_speed_limit_;
  const double vel_scaling = max(linear_vel_scaling, angular_vel_scaling);
  return scaleTwist(twist, max(1.0, max(acc_scaling, vel_scaling)));
}

// Get pose in local costmap framoe
gm::Pose2D StepBackAndSteerTurnRecovery::getCurrentLocalPose () const
{
  gm::PoseStamped p;
  local_costmap_->getRobotPose(p);
  gm::Pose2D pose;
  pose.x = p.pose.position.x;
  pose.y = p.pose.position.y;
  tf::Quaternion q(
    p.pose.orientation.x,
    p.pose.orientation.y,
    p.pose.orientation.z,
    p.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose.theta = yaw;
  return pose;
}

void StepBackAndSteerTurnRecovery::moveSpacifiedLength (const gm::Twist twist, const double distination, const COSTMAP_SEARCH_MODE mode) const
{
    double distination_cmd = distination;
    double min_dist_to_obstacle = getMinimalDistanceToObstacle(mode);

    std::string mode_name;
    double time_out;

    switch (mode) {
    case FORWARD:
        mode_name = "FORWARD";
        time_out = step_forward_timeout_;
        if (min_dist_to_obstacle < distination)
        {
          distination_cmd = min_dist_to_obstacle - obstacle_patience_;

          ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
          ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
          ROS_WARN_NAMED ("top", "moving length is switched from %.2f [m] to %.2f in %s", distination, distination_cmd,mode_name.c_str());
        }
        break;
    case FORWARD_LEFT:
        mode_name = "FORWARD_LEFT";
        time_out = steering_timeout_;
        if (min_dist_to_obstacle < obstacle_patience_)
        {
          distination_cmd = 0.0;

          ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
          ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
          ROS_WARN_NAMED ("top", "stop turning because an obstacle is too close in %s", mode_name.c_str());
        }
        break;
    case FORWARD_RIGHT:
        mode_name = "FORWARD_RIGHT";
        time_out = steering_timeout_;
        if (min_dist_to_obstacle < obstacle_patience_)
        {
          distination_cmd = 0.0;

          ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
          ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
          ROS_WARN_NAMED ("top", "stop turning because an obstacle is too close in %s", mode_name.c_str());
        }
        break;
    case BACKWARD:
        mode_name = "BACKWARD";
        time_out = step_back_timeout_;
        if (min_dist_to_obstacle < distination)
        {
          distination_cmd = min_dist_to_obstacle - 2 * obstacle_patience_;

          ROS_WARN_NAMED ("top", "obstacle detected before moving %s", mode_name.c_str());
          ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist_to_obstacle, mode_name.c_str());
          ROS_WARN_NAMED ("top", "moving length is switched from %.2f [m] to %.2f in %s", distination, distination_cmd, mode_name.c_str());
        }
        break;
    default:
        break;
    }

    const double frequency = 5.0;
    ros::Rate r(frequency);

    const gm::Pose2D initialPose = getCurrentLocalPose();

    int log_cnt = 0;
    int log_frequency = (int)obstacle_check_frequency_;

    ros::Time time_begin = ros::Time::now();
    while (double dist_diff = getCurrentDistDiff(initialPose, distination_cmd, mode) > 0.01)
    {
        double remaining_time = dist_diff / base_frame_twist_.linear.x;
        double min_dist = getMinimalDistanceToObstacle(mode);

        // time out
        if(time_out > 0.0 &&
                time_begin + ros::Duration(time_out) < ros::Time::now())
        {
            //cmd_vel_pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, remaining_time));
            cmd_vel_pub_.publish(TWIST_STOP);
            ROS_WARN_NAMED ("top", "time out at %s", mode_name.c_str());
            ROS_WARN_NAMED ("top", "%.2f [sec] elapsed.", time_out);
            break;
        }

        // detect an obstacle
        if(min_dist < obstacle_patience_)
        {
            //cmd_vel_pub_.publish(scaleGivenAccelerationLimits(TWIST_STOP, remaining_time));
            cmd_vel_pub_.publish(TWIST_STOP);
            ROS_WARN_NAMED ("top", "obstacle detected at %s", mode_name.c_str());
            ROS_WARN_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist, mode_name.c_str());
            break;
        }

        //cmd_vel_pub_.publish(scaleGivenAccelerationLimits(twist, remaining_time));
        cmd_vel_pub_.publish(twist);
        if(log_cnt++ % log_frequency == 0)
        {
            ROS_DEBUG_NAMED ("top", "no obstacle around");
            ROS_INFO_NAMED ("top", "min dist to obstacle = %.2f [m] in %s", min_dist, mode_name.c_str());
        }

        ros::spinOnce();
        r.sleep();
    }

    return;
}

double StepBackAndSteerTurnRecovery::getCurrentDiff(const gm::Pose2D initialPose, const COSTMAP_SEARCH_MODE mode) const
{

    const gm::Pose2D& currentPose = getCurrentLocalPose();
    ROS_DEBUG_NAMED ("top", "current pose (%.2f, %.2f, %.2f)", currentPose.x,
                       currentPose.y, currentPose.theta);

    double current_diff;

    switch (mode) {
    case FORWARD:
    case BACKWARD:
        current_diff = getDistBetweenTwoPoints(currentPose, initialPose);
        ROS_DEBUG_NAMED ("top", "current_diff in translation = %.2f", current_diff);
        break;
    case FORWARD_LEFT:
    case FORWARD_RIGHT:
        current_diff = initialPose.theta - currentPose.theta;
        current_diff = fabs(current_diff);
        ROS_DEBUG_NAMED ("top", "initialPose.Theta = %.2f, currentPose.theta = %.2f", initialPose.theta, currentPose.theta);
        ROS_DEBUG_NAMED ("top", "current_diff in angle = %.2f", current_diff);
    default:
        break;
    }

    return current_diff;
}

double StepBackAndSteerTurnRecovery::getCurrentDistDiff(const gm::Pose2D initialPose, const double distination, COSTMAP_SEARCH_MODE mode) const
{
    const double dist_diff = distination - getCurrentDiff(initialPose, mode);
    ROS_DEBUG_NAMED ("top", "dist_diff = %.2f", dist_diff);

    return dist_diff;
}

double StepBackAndSteerTurnRecovery::getMinimalDistanceToObstacle(const COSTMAP_SEARCH_MODE mode) const
{
    double max_angle, min_angle;
    gm::Twist twist = TWIST_STOP;

    switch (mode) {
    case FORWARD:
        twist.linear.x = linear_vel_forward_;
        max_angle = M_PI/3.0;
        min_angle = -M_PI/3.0;
        break;
    case FORWARD_LEFT:
        twist.linear.x = linear_vel_forward_;
        max_angle = M_PI/2.0;
        min_angle = 0.0;
        break;
    case FORWARD_RIGHT:
        twist.linear.x = linear_vel_forward_;
        max_angle = 0.0;
        min_angle = -M_PI/2.0;
        break;
    case BACKWARD:
        twist.linear.x = linear_vel_back_;
        max_angle = M_PI/3.0;
        min_angle = -M_PI/3.0;
        break;
    default:
        break;
    }

    const gm::Pose2D& current = getCurrentLocalPose();
    double min_dist = INFINITY;

    for(double angle = min_angle; angle < max_angle; angle+=sim_angle_resolution_)
      {
          twist.angular.z = angle;
          gm::Pose2D pose_to_obstacle = getPoseToObstacle(current, twist);
          double dist_to_obstacle = getDistBetweenTwoPoints(current, pose_to_obstacle);

          if(dist_to_obstacle < min_dist)
              min_dist = dist_to_obstacle;
    }

    ROS_DEBUG_NAMED ("top", "min_dist = %.2f", min_dist);

    return min_dist;
}

int StepBackAndSteerTurnRecovery::determineTurnDirection()
{
    // simulate and evaluate cost
    const gm::Pose2D& current = getCurrentLocalPose();

    gm::Twist twist = TWIST_STOP;
    twist.linear.x = linear_vel_forward_;

    vector<double> dist_to_obstacle_r;
    vector<double> dist_to_obstacle_l;
    double max = M_PI/2.0;
    double min = - max;
    for(double angle = min; angle < max; angle+=sim_angle_resolution_)
    {
        twist.angular.z = angle;
        gm::Pose2D pose_to_obstacle = getPoseToObstacle(current, twist);
        double dist_to_obstacle = getDistBetweenTwoPoints(current, pose_to_obstacle);

        ROS_DEBUG_NAMED ("top", "(%.2f, %.2f, %.2f) for %.2f [m] to obstacle",
                         twist.linear.x, twist.linear.y, twist.angular.z, dist_to_obstacle);

        if(angle > 0.0)
            dist_to_obstacle_l.push_back(dist_to_obstacle);
        else if(angle < 0.0)
            dist_to_obstacle_r.push_back(dist_to_obstacle);
        else
            ;// do nothing
    }

    // determine the directoin to go from cost
    /*
    double sum_l = 0.0;
    double sum_r = 0.0;
    double ave_l = 0.0;
    double ave_r = 0.0;
    for(int i = 0; i < dist_to_obstacle_l.size(); i++)
        sum_l += dist_to_obstacle_l[i];
    for(int i = 0; i < dist_to_obstacle_r.size(); i++)
        sum_r += dist_to_obstacle_r[i];
    ave_l = sum_l / dist_to_obstacle_l.size();
    ave_r = sum_r / dist_to_obstacle_r.size();
    ROS_DEBUG_NAMED ("top", "sum_l = %.2f, sum_r = %.2f", sum_l, sum_r);
    ROS_DEBUG_NAMED ("top", "size_l = %d, size_r = %d", (int)dist_to_obstacle_l.size(), (int)dist_to_obstacle_r.size());
    ROS_DEBUG_NAMED ("top", "ave_l = %.2f, ave_r = %.2f", ave_l, ave_r);
    */

    double min_l = *min_element(dist_to_obstacle_l.begin(), dist_to_obstacle_l.end());
    double min_r = *min_element(dist_to_obstacle_r.begin(), dist_to_obstacle_r.end());
    ROS_INFO_NAMED ("top", "min_l = %.2f [m], min_r = %.2f [m]", min_l, min_r);

    int ret_val;

    if(min_l < min_r)
        ret_val = RIGHT; // if obstacle settles on left, turn right
    else
        ret_val = LEFT; // vice versa

    return ret_val;
}

double StepBackAndSteerTurnRecovery::getDistBetweenTwoPoints(const gm::Pose2D pose1, const gm::Pose2D pose2) const
{
    double dist_to_obstacle = (pose1.x - pose2.x) * (pose1.x - pose2.x) +
            (pose1.y - pose2.y) * (pose1.y - pose2.y);
    return sqrt(dist_to_obstacle);
}

void StepBackAndSteerTurnRecovery::runBehavior ()
{
  ROS_ASSERT (initialized_);

  ROS_INFO_NAMED ("top", "*****************************************************");
  ROS_INFO_NAMED ("top", "********Start StepBackAndSteerTurnRecovery!!!********");
  ROS_INFO_NAMED ("top", "*****************************************************");

  std_msgs::Bool run_state;

  // when starting recovery, topic /run_state_ shifts to true
  run_state.data = true;
  recover_run_pub_.publish(run_state);

  int cnt = 0;
  const double stop_duaration = 1.0;
  while(true)
  {
      cnt++;
      ROS_INFO_NAMED ("top", "==== %d th recovery trial ====", cnt);

      // Figure out how long we can safely run the behavior
      const gm::Pose2D& initialPose = getCurrentLocalPose();

      // initial pose
      ROS_DEBUG_NAMED ("top", "initial pose (%.2f, %.2f, %.2f)", initialPose.x,
                       initialPose.y, initialPose.theta);
      ros::Rate r(controller_frequency_);

      // step back
      base_frame_twist_.linear.x = linear_vel_back_;
      ROS_INFO_NAMED ("top", "attempting step back");
      moveSpacifiedLength(base_frame_twist_, step_back_length_, BACKWARD);
      ROS_INFO_NAMED ("top", "complete step back");

      double final_diff = getCurrentDiff(initialPose);
      ROS_DEBUG_NAMED ("top", "final_diff = %.2f",final_diff);

      // stop
      for (double t=0; t<stop_duaration; t+=1/controller_frequency_) {
          cmd_vel_pub_.publish(TWIST_STOP);
          r.sleep();
      }

      int turn_dir = determineTurnDirection();
      int costmap_search_mode[CNT_TURN];

      double z;
      if(turn_dir == LEFT)
      {
          z = angular_speed_steer_;
          costmap_search_mode[FIRST_TURN] = FORWARD_LEFT;
          costmap_search_mode[SECOND_TURN] = FORWARD_RIGHT;
          ROS_INFO_NAMED ("top", "attempting to turn left at the 1st turn");
      }
      else
      {
          z = -1 * angular_speed_steer_;
          costmap_search_mode[FIRST_TURN] = FORWARD_RIGHT;
          costmap_search_mode[SECOND_TURN] = FORWARD_LEFT;
          ROS_INFO_NAMED ("top", "attemping to turn right at the 1st turn");
      }

      // clear way
      //-- first steering
      gm::Twist twist;
      twist = TWIST_STOP;
      twist.linear.x = linear_vel_steer_;
      twist.angular.z = z;
      moveSpacifiedLength(twist, turn_angle_, (COSTMAP_SEARCH_MODE)costmap_search_mode[FIRST_TURN]);
      ROS_INFO_NAMED ("top", "complete the 1st turn");

      if(!only_single_steering_) {
          //-- go straight
          ROS_INFO_NAMED ("top", "attemping step forward");
          twist = TWIST_STOP;
          twist.linear.x = linear_vel_forward_;
          moveSpacifiedLength(twist, step_forward_length_, FORWARD);
          ROS_INFO_NAMED ("top", "complete step forward");

          //-- second steering
          ROS_INFO_NAMED ("top", "attempting second turn");
          twist = TWIST_STOP;
          twist.linear.x = linear_vel_steer_;
          twist.angular.z = -z;
          moveSpacifiedLength(twist, turn_angle_, (COSTMAP_SEARCH_MODE)costmap_search_mode[SECOND_TURN]);
          ROS_INFO_NAMED ("top", "complete second turn");
      }

      // stop
      for (double t=0; t<stop_duaration; t+=1/controller_frequency_) {
          cmd_vel_pub_.publish(TWIST_STOP);
          r.sleep();
      }

      // check trial times
      if(cnt == trial_times_)
      {
          ROS_INFO_NAMED ("top", "break after %d times recovery", cnt);
          break;
      }

      // check clearance forward
      const  gm::Pose2D& current = getCurrentLocalPose();
      double max_angle = 0.1;
      double min_angle = -max_angle;
      double max_clearance = 0;
      twist.linear.x = 3.0;
      for(double angle = min_angle; angle < max_angle; angle+=sim_angle_resolution_)
      {
          twist.angular.z = angle;
          gm::Pose2D pose_to_obstacle = getPoseToObstacle(current, twist);
          double dist_to_obstacle = getDistBetweenTwoPoints(current, pose_to_obstacle);

          if(dist_to_obstacle > max_clearance)
              max_clearance = dist_to_obstacle;
      }

      if(max_clearance < 3.0)
      {
          ROS_INFO_NAMED ("top", "continue recovery because the robot couldn't get clearance");
          ROS_DEBUG_NAMED ("top", "continue at (%.2f, %.2f, %.2f) for max_clearance %.2f m",
                          twist.linear.x, twist.linear.y, twist.angular.z, max_clearance);
          continue;
      }
      else
      {
          ROS_INFO_NAMED ("top", "break recovery because the robot got clearance");
          ROS_DEBUG_NAMED ("top", "break at (%.2f, %.2f, %.2f) for max_clearance %.2f m",
                          twist.linear.x, twist.linear.y, twist.angular.z, max_clearance);
          break;
      }
  }

  // when finishing recovery, topic /run_state_ shifts to false
  run_state.data = false;
  recover_run_pub_.publish(run_state);

  ROS_INFO_NAMED ("top", "*****************************************************");
  ROS_INFO_NAMED ("top", "********Finish StepBackAndSteerTurnRecovery!!********");
  ROS_INFO_NAMED ("top", "*****************************************************");

}


} // namespace stepback_and_steerturn_recovery
