#ifndef neor_mini_gazebo_plugin__neor_mini_gazebo_plugin_HPP_
#define neor_mini_gazebo_plugin__neor_mini_gazebo_plugin_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

double sign_of(double x)
{
  if (x > 0)
  {
    return 1;
  }
  if (x < 0)
  {
    return -1;
  }
  return 0;
}

class BicycleModel
{
  double wheelbase_length;
  double steer_angle;
  double max_steer_angle;

public:
  BicycleModel(double wheelbase_length, double steer_angle = 0.0, double max_steer_angle = M_PI / 4.0)
  {
    this->wheelbase_length = wheelbase_length;
    this->steer_angle = steer_angle;
    this->max_steer_angle = max_steer_angle;
  }

  void set_steer_angle(double steer_angle)
  {
    if (fabs(steer_angle) > this->max_steer_angle)
    {
      steer_angle = sign_of(steer_angle) * this->max_steer_angle;
    }
    this->steer_angle = steer_angle;
  }

  void set_rear_curvature(double k_rear)
  {
    this->steer_angle = atan(k_rear * this->wheelbase_length);
  }

  double get_rear_curvature()
  {
    return tan(this->steer_angle) / this->wheelbase_length;
  }

  void set_front_curvature(double k_front)
  {
    this->steer_angle = asin(this->wheelbase_length * k_front);
  }

  double get_front_curvature()
  {
    return sin(this->steer_angle) / this->wheelbase_length;
  }

  double get_steer_angle() { return this->steer_angle; }

  BicycleModel get_offset_bicycle(double delta_y)
  {
    if (this->steer_angle == 0.0)
    {
      return BicycleModel(wheelbase_length, 0.0);
    }
    BicycleModel new_bike(this->wheelbase_length);
    new_bike.set_rear_curvature(1. / (1. / this->get_rear_curvature() - delta_y));
    return new_bike;
  }
};

class AckermannModel : public BicycleModel
{
  double front_wheelbase_width;

public:
  AckermannModel(double wheelbase_length, double front_wheelbase_width, double steer_angle = 0.0) : BicycleModel(wheelbase_length, steer_angle)
  {
    this->front_wheelbase_width = front_wheelbase_width;
  }

  BicycleModel get_left_bicycle()
  {
    return this->get_offset_bicycle(this->front_wheelbase_width / 2.);
  }

  BicycleModel get_right_bicycle()
  {
    return this->get_offset_bicycle(-1. * this->front_wheelbase_width / 2.);
  }
};

namespace neor_mini_gazebo_plugin
{

  class CarGazeboPlugin : public gazebo::ModelPlugin
  {
  public:
    CarGazeboPlugin();

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    void Update();

    using JointState = sensor_msgs::msg::JointState;

    std::string robot_namespace_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;

    gazebo::common::Time last_sim_time_;
    gazebo::common::Time last_update_time_;
    double update_period_ms_;

    gazebo::event::ConnectionPtr update_connection_;

    std::map<std::string, std::pair<gazebo::physics::JointPtr, gazebo::common::PID>> joints_;
    std::map<std::string, double> joint_targets_;

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double steer = 0;
    double velocity = 0;
    const double wheelbase_length = 0.45;
    const double front_wheelbase_width = 0.42;
    const double rear_wheelbase_width = 0.42;

    const double wheel_diameter = 0.125;
    const double max_speed = 3;     // just for joy control.
    const double max_turn_angle = M_PI / 4.0;
    AckermannModel car_model = {0.25, 0.25};

    gazebo::physics::JointControllerPtr jc;
    gazebo::physics::JointPtr fl_str_joint;
    gazebo::physics::JointPtr fr_str_joint;
    gazebo::physics::JointPtr fl_axle_joint;
    gazebo::physics::JointPtr fr_axle_joint;
    gazebo::physics::JointPtr bl_axle_joint;
    gazebo::physics::JointPtr br_axle_joint;

    gazebo::common::PID fl_pid, fr_pid, bl_pid, br_pid;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
    {
      if (msg->buttons[0] > 0)
      {
        ackermann_msgs::msg::AckermannDriveStamped ad;
        ad.drive.steering_angle = msg->axes[0];
        ad.drive.speed =
            pow(fabs(msg->axes[1]), 2) * sign_of(msg->axes[1]) * max_speed;
        ackermann_pub->publish(ad);
      }
    }

    void ackermann_callback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
      // RCLCPP_INFO(ros_node_->get_logger(),"subscribe speed: %f steering_angle: %f",msg->drive.speed ,msg->drive.steering_angle);

      car_model.set_steer_angle(msg->drive.steering_angle);

      jc->SetPositionTarget(fl_str_joint->GetScopedName(), car_model.get_left_bicycle().get_steer_angle());

      jc->SetPositionTarget(fr_str_joint->GetScopedName(), car_model.get_right_bicycle().get_steer_angle());

      // RCLCPP_INFO(ros_node_->get_logger()," steer angle: %lf ",car_model.get_right_bicycle().get_steer_angle() );

      double curvature = car_model.get_rear_curvature();
      double speed = msg->drive.speed;
      double meters_per_rev = M_PI * wheel_diameter;
      double revs_per_sec = speed / meters_per_rev;
      double rads_per_sec = 2 * M_PI * revs_per_sec;
      if (curvature == 0)
      {
        jc->SetVelocityTarget(bl_axle_joint->GetScopedName(), rads_per_sec);
        jc->SetVelocityTarget(br_axle_joint->GetScopedName(), rads_per_sec);
      }
      else
      {
        double radius = 1. / curvature;
        double left_radius = radius - rear_wheelbase_width / 2.;
        double right_radius = radius + rear_wheelbase_width / 2.;

        jc->SetVelocityTarget(bl_axle_joint->GetScopedName(), rads_per_sec * left_radius / radius);
        jc->SetVelocityTarget(br_axle_joint->GetScopedName(), rads_per_sec * right_radius / radius);
      }

      // RCLCPP_INFO(ros_node_->get_logger(),"rads_per_sec is : %lf",rads_per_sec);
      // std::cout<<" joint name: "<<bl_axle_joint->GetScopedName();
      // RCLCPP_INFO(ros_node_->get_logger(), "left_rear_joint : joint force: %lf", bl_axle_joint->GetForce(0));
    }

    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
      ackermann_msgs::msg::AckermannDriveStamped ad;
      ad.drive.speed = msg->linear.x;

      if (msg->linear.x == 0)
      {
        ad.drive.steering_angle = 0;
      }
      else
      {
        car_model.set_rear_curvature(msg->angular.z / msg->linear.x);
        ad.drive.steering_angle = car_model.get_steer_angle();
      }
      // RCLCPP_INFO(ros_node_->get_logger(),"Subscribe linear : %f , angular: %f",msg->linear.x,msg->angular.z);

      ackermann_pub->publish(ad);
    }

  private:
    gazebo::physics::JointPtr get_joint(const char *joint_name)
    {
      auto joint = model_->GetJoint(joint_name);
      if (joint.get() == 0)
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to get_joint %s", joint_name);
      }

      return joint;
    }
  };

} // namespace neor_mini_gazebo_plugin

#endif // neor_mini_gazebo_plugin__neor_mini_gazebo_plugin_HPP_