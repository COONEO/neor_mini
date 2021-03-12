
#ifndef STEER_BOT_HARDWARE_GAZEBO_H
#define STEER_BOT_HARDWARE_GAZEBO_H

#include <vector>
#include <string>

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

enum FRONT_REAR
{
    FRONT = 0, REAR = 1,
};

namespace steer_bot_hardware_gazebo
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

class SteerBotHardwareGazebo : public gazebo_ros_control::RobotHWSim
{
public:

  SteerBotHardwareGazebo();

  bool initSim(const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

private:
  void CleanUp();

  void GetJointNames(ros::NodeHandle &_nh);
  void GetWheelJointNames(ros::NodeHandle &_nh);
  void GetSteerJointNames(ros::NodeHandle &_nh);

  void RegisterHardwareInterfaces();
  void RegisterWheelInterface();
  void RegisterSteerInterface();
  void RegisterInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          hardware_interface::JointCommandInterface& _jnt_cmd_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff, double& _jnt_cmd);
  void RegisterInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff);
  void RegisterJointStateInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff);
  void RegisterCommandJointInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      hardware_interface::JointCommandInterface& _jnt_cmd_interface,
      const std::string _jnt_name, double& _jnt_cmd);
  double ComputeEffCommandFromVelError(const int _index, const int _front_rear, ros::Duration _period);

  void GetCurrentState(std::vector<double>& _jnt_pos, std::vector<double>& _jnt_vel, std::vector<double>& _jnt_eff,
                       const int _if_index, const int _sim_jnt_index);

private:
  // constant
  enum {
      INDEX_RIGHT = 0,
      INDEX_LEFT = 1
  };
  // Raw data
  unsigned int n_dof_;
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  // Joint limits interface
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;

  // PID controllers
  std::vector<control_toolbox::Pid> pids_;

  // new added
  ros::NodeHandle nh_;
  std::string ns_;

  // common
  hardware_interface::JointStateInterface jnt_state_interface_;
  //
  // rear wheel
  //-- actual joint(single actuator)
  //---- joint name
  std::string rear_wheel_jnt_name_;
  //---- joint interface parameters
  double rear_wheel_jnt_pos_;
  double rear_wheel_jnt_vel_;
  double rear_wheel_jnt_eff_;
  //---- joint interface command
  double rear_wheel_jnt_vel_cmd_;
  //---- Hardware interface: joint
  hardware_interface::VelocityJointInterface rear_wheel_jnt_vel_cmd_interface_;
  //hardware_interface::JointStateInterface wheel_jnt_state_interface_;
  //
  //-- virtual joints(two rear wheels)
  //---- joint name
  std::vector<std::string> virtual_rear_wheel_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_rear_wheel_jnt_pos_;
  std::vector<double> virtual_rear_wheel_jnt_vel_;
  std::vector<double> virtual_rear_wheel_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_rear_wheel_jnt_vel_cmd_;
  //-- virtual joints(two front wheels)
  //---- joint name
  std::vector<std::string> virtual_front_wheel_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_front_wheel_jnt_pos_;
  std::vector<double> virtual_front_wheel_jnt_vel_;
  std::vector<double> virtual_front_wheel_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_front_wheel_jnt_vel_cmd_;

  // front steer
  //-- actual joint(single actuator)
  //---- joint name
  std::string front_steer_jnt_name_;
  //---- joint interface parameters
  double front_steer_jnt_pos_;
  double front_steer_jnt_vel_;
  double front_steer_jnt_eff_;
  //---- joint interface command
  double front_steer_jnt_pos_cmd_;
  //---- Hardware interface: joint
  hardware_interface::PositionJointInterface front_steer_jnt_pos_cmd_interface_;
  //hardware_interface::JointStateInterface steer_jnt_state_interface_;
  //
  //-- virtual joints(two steers)
  //---- joint name
  std::vector<std::string> virtual_front_steer_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_front_steer_jnt_pos_;
  std::vector<double> virtual_front_steer_jnt_vel_;
  std::vector<double> virtual_front_steer_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_front_steer_jnt_pos_cmd_;
  //---- Hardware interface: joint
  //hardware_interface::VelocityJointInterface front_wheel_jnt_vel_cmd_interface_;

  // ackermann link mechanism
  bool enable_ackermann_link_ ;
  double wheel_separation_w_;
  double wheel_separation_h_;

  int log_cnt_;

  template <class T>
  std::string containerToString(const T& cont, const std::string& prefix)
  {
    std::stringstream ss;
    ss << prefix;
    std::copy(cont.begin(), --cont.end(), std::ostream_iterator<typename T::value_type>(ss, prefix.c_str()));
    ss << *(--cont.end());
    return ss.str();
  }

};

} // namespace steer_bot_hardware_gazebo

#endif // STEER_BOT_HARDWARE_GAZEBO_H
