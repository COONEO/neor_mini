
#include <angles/angles.h>

#include <urdf_parser/urdf_parser.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <pluginlib/class_list_macros.h>

#include <steer_bot_hardware_gazebo/steer_bot_hardware_gazebo.h>

namespace steer_bot_hardware_gazebo
{
  using namespace hardware_interface;

  SteerBotHardwareGazebo::SteerBotHardwareGazebo()
    : gazebo_ros_control::RobotHWSim()
    , ns_("steer_bot_hardware_gazebo/")
    , log_cnt_(0)
  {}


  bool SteerBotHardwareGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle nh,
      gazebo::physics::ModelPtr model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    using gazebo::physics::JointPtr;

    nh_ = nh;

    // Simulation joints
    sim_joints_ = model->GetJoints();
    n_dof_ = sim_joints_.size();

    this->CleanUp();
    this->GetJointNames(nh_);
    this->RegisterHardwareInterfaces();

    nh_.getParam(ns_ + "wheel_separation_w", wheel_separation_w_);
    nh_.getParam(ns_ + "wheel_separation_h", wheel_separation_h_);
    ROS_INFO_STREAM("wheel_separation_w = " << wheel_separation_w_);
    ROS_INFO_STREAM("wheel_separation_h = " << wheel_separation_h_);

    nh_.getParam(ns_ + "enable_ackermann_link", enable_ackermann_link_);
    ROS_INFO_STREAM("enable_ackermann_link = " << (enable_ackermann_link_ ? "true" : "false"));

    // Position joint limits interface
    std::vector<std::string> cmd_handle_names = front_steer_jnt_pos_cmd_interface_.getNames();
    for (size_t i = 0; i < cmd_handle_names.size(); ++i)
    {
      const std::string name = cmd_handle_names[i];

      // unless current handle is not pos interface for steer, skip
      if(name != virtual_front_steer_jnt_names_[INDEX_RIGHT] && name != virtual_front_steer_jnt_names_[INDEX_LEFT])
          continue;

      JointHandle cmd_handle = front_steer_jnt_pos_cmd_interface_.getHandle(name);

      using namespace joint_limits_interface;
      boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(name);
      JointLimits limits;
      SoftJointLimits soft_limits;
      if (!getJointLimits(urdf_joint, limits) || !getSoftJointLimits(urdf_joint, soft_limits))
      {
        ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
      }
      else
      {
        jnt_limits_interface_.registerHandle(
            PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

        ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
      }
    }

    // PID controllers for wheel
    const int virtual_jnt_cnt_ = virtual_rear_wheel_jnt_names_.size();
    pids_.resize(virtual_jnt_cnt_);

    for (size_t i = 0; i < virtual_jnt_cnt_; ++i)
    {
      const std::string jnt_name = virtual_rear_wheel_jnt_names_[i];
      const ros::NodeHandle joint_nh(nh, "gains/" +  jnt_name);

      ROS_INFO_STREAM("Trying to set pid param of '" << jnt_name << " ' at PID proc in init()");
      if (!pids_[i].init(joint_nh))
      {
        ROS_INFO_STREAM("Faied to set pid param of '" << jnt_name << " ' at PID proc in init()");
        return false;
      }
      ROS_INFO_STREAM("Succeeded to set pid param of '" << jnt_name << " ' at PID proc in init()");
    }

    return true;
  }

  void SteerBotHardwareGazebo::readSim(ros::Time time, ros::Duration period)
  {
    for(int i = 0; i <  sim_joints_.size(); i++)
    {
      std::string gazebo_jnt_name;
      gazebo_jnt_name = sim_joints_[i]->GetName();

      if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_RIGHT])
      {
        GetCurrentState(virtual_rear_wheel_jnt_pos_, virtual_rear_wheel_jnt_vel_, virtual_rear_wheel_jnt_eff_, INDEX_RIGHT, i);
      }
      else if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_LEFT])
      {
        GetCurrentState(virtual_rear_wheel_jnt_pos_, virtual_rear_wheel_jnt_vel_, virtual_rear_wheel_jnt_eff_, INDEX_LEFT, i);
      }
      else if(gazebo_jnt_name == virtual_front_wheel_jnt_names_[INDEX_RIGHT])
      {
        GetCurrentState(virtual_front_wheel_jnt_pos_, virtual_front_wheel_jnt_vel_, virtual_front_wheel_jnt_eff_, INDEX_RIGHT, i);
      }
      else if(gazebo_jnt_name == virtual_front_wheel_jnt_names_[INDEX_LEFT])
      {
        GetCurrentState(virtual_front_wheel_jnt_pos_, virtual_front_wheel_jnt_vel_, virtual_front_wheel_jnt_eff_, INDEX_LEFT, i);
      }
      else if(gazebo_jnt_name == virtual_front_steer_jnt_names_[INDEX_RIGHT])
      {
        GetCurrentState(virtual_front_steer_jnt_pos_, virtual_front_steer_jnt_vel_, virtual_front_steer_jnt_eff_, INDEX_RIGHT, i);
      }
      else if(gazebo_jnt_name == virtual_front_steer_jnt_names_[INDEX_LEFT])
      {
        GetCurrentState(virtual_front_steer_jnt_pos_, virtual_front_steer_jnt_vel_, virtual_front_steer_jnt_eff_, INDEX_LEFT, i);
      }
      else
      {
        // do nothing
      }
    }

    front_steer_jnt_pos_ = (virtual_front_steer_jnt_pos_[INDEX_RIGHT] + virtual_front_steer_jnt_pos_[INDEX_LEFT]) / virtual_front_steer_jnt_pos_.size();
    front_steer_jnt_vel_ = (virtual_front_steer_jnt_vel_[INDEX_RIGHT] + virtual_front_steer_jnt_vel_[INDEX_LEFT]) / virtual_front_steer_jnt_vel_.size();
    front_steer_jnt_eff_ = (virtual_front_steer_jnt_eff_[INDEX_RIGHT] + virtual_front_steer_jnt_eff_[INDEX_LEFT]) / virtual_front_steer_jnt_eff_.size();
  }

  void SteerBotHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits
    jnt_limits_interface_.enforceLimits(period);

    log_cnt_++;
    for(int i = 0; i <  sim_joints_.size(); i++)
    {
      std::string gazebo_jnt_name;
      gazebo_jnt_name = sim_joints_[i]->GetName();

      // wheels
      if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_RIGHT])
      {
        //const double eff_cmd = 2*ComputeEffCommandFromVelError(INDEX_RIGHT, REAR, period);
        //sim_joints_[i]->SetForce(0u, eff_cmd);
        //if(log_cnt_ % 500 == 0)
        //{
        //  ROS_DEBUG_STREAM("rear_wheel_jnt_vel_cmd_ = " << rear_wheel_jnt_vel_cmd_);
        //  ROS_DEBUG_STREAM("virtual_rear_wheel_jnt_vel_[INDEX_RIGHT] = " << virtual_rear_wheel_jnt_vel_[INDEX_RIGHT]);
        //  ROS_DEBUG_STREAM("error[INDEX_RIGHT] " <<  rear_wheel_jnt_vel_cmd_ - virtual_rear_wheel_jnt_vel_[INDEX_RIGHT]);
        //  ROS_DEBUG_STREAM("command[INDEX_RIGHT] = " << eff_cmd);
        //}
        sim_joints_[i]->SetVelocity(0, rear_wheel_jnt_vel_cmd_);
      }
      else if(gazebo_jnt_name == virtual_rear_wheel_jnt_names_[INDEX_LEFT])
      {
        //const double eff_cmd = 2*ComputeEffCommandFromVelError(INDEX_LEFT, REAR, period);
        //sim_joints_[i]->SetForce(0u, eff_cmd);
        //if(log_cnt_ % 500 == 0)
        //{
        //  ROS_DEBUG_STREAM("rear_wheel_jnt_vel_cmd_ = " << rear_wheel_jnt_vel_cmd_);
        //  ROS_DEBUG_STREAM("virtual_rear_wheel_jnt_vel_[INDEX_LEFT] = " << virtual_rear_wheel_jnt_vel_[INDEX_LEFT]);
        //  ROS_DEBUG_STREAM("error[INDEX_LEFT] " <<  rear_wheel_jnt_vel_cmd_ - virtual_rear_wheel_jnt_vel_[INDEX_LEFT]);
        //  ROS_DEBUG_STREAM("command[INDEX_LEFT] = " << eff_cmd);
        //}
        sim_joints_[i]->SetVelocity(0, rear_wheel_jnt_vel_cmd_);
      }
      else if(gazebo_jnt_name == virtual_front_wheel_jnt_names_[INDEX_RIGHT])
      {
        //const double eff_cmd = 2*ComputeEffCommandFromVelError(INDEX_RIGHT, FRONT, period);
        //sim_joints_[i]->SetForce(0u, eff_cmd);
        //ROS_INFO_STREAM("command[INDEX_RIGHT] = " << eff_cmd);
        sim_joints_[i]->SetVelocity(0, rear_wheel_jnt_vel_cmd_); // added for gazebo in kinetic
      }
      else if(gazebo_jnt_name == virtual_front_wheel_jnt_names_[INDEX_LEFT])
      {
        //const double eff_cmd = 2*ComputeEffCommandFromVelError(INDEX_LEFT, FRONT, period);
        //sim_joints_[i]->SetForce(0u, eff_cmd);
        //ROS_INFO_STREAM("command[INDEX_LEFT] = " << eff_cmd);
        sim_joints_[i]->SetVelocity(0, rear_wheel_jnt_vel_cmd_); // added for gazebo in kinetic

      }
      // steers
      else if(gazebo_jnt_name == front_steer_jnt_name_)
      {
        front_steer_jnt_pos_ = front_steer_jnt_pos_cmd_;
        ROS_INFO_STREAM("front_steer_jnt_pos_ '" << front_steer_jnt_pos_ << " ' at writeSim()");
      }
      else if(gazebo_jnt_name == virtual_front_steer_jnt_names_[INDEX_RIGHT])
      {
        double pos_cmd = 0.0;
        if(enable_ackermann_link_)
        {
          const double h = wheel_separation_h_;
          const double w = wheel_separation_w_;
          pos_cmd = atan2(2*h*tan(front_steer_jnt_pos_cmd_), 2*h + w/2.0*tan(front_steer_jnt_pos_cmd_));
          ROS_DEBUG_STREAM("ackermann steer angle: " << pos_cmd << " at RIGHT");
        }
        else
        {
          pos_cmd = front_steer_jnt_pos_cmd_;
        }
#if GAZEBO_MAJOR_VERSION >= 9
        sim_joints_[i]->SetPosition(0, pos_cmd, true);
#else
        sim_joints_[i]->SetPosition(0, pos_cmd);
#endif
      }
      else if(gazebo_jnt_name == virtual_front_steer_jnt_names_[INDEX_LEFT])
      {
        double pos_cmd = 0.0;
        if(enable_ackermann_link_)
        {
          const double h = wheel_separation_h_;
          const double w = wheel_separation_w_;
          pos_cmd = atan2(2*h*tan(front_steer_jnt_pos_cmd_), 2*h - w/2.0*tan(front_steer_jnt_pos_cmd_));
          ROS_DEBUG_STREAM("ackermann steer angle: " << pos_cmd << " at LEFT");
        }
        else
        {
          pos_cmd = 2*front_steer_jnt_pos_cmd_;
        }
#if GAZEBO_MAJOR_VERSION >= 9
        sim_joints_[i]->SetPosition(0, pos_cmd, true);
#else
        sim_joints_[i]->SetPosition(0, pos_cmd);
#endif
      }
      else
      {
        // do nothing
      }
    }

    // steer joint for steer_drive_controller
    front_steer_jnt_pos_ = front_steer_jnt_pos_cmd_;
    if(log_cnt_ % 500 == 0)
    {
      ROS_DEBUG_STREAM("front_steer_jnt_pos_ '" << front_steer_jnt_pos_ << " ' at writeSim()");
    }
    // wheel joint for steer_drive_controller
    rear_wheel_jnt_pos_ = 0.5 * (virtual_rear_wheel_jnt_pos_[INDEX_RIGHT] + virtual_rear_wheel_jnt_pos_[INDEX_LEFT]);
    rear_wheel_jnt_vel_ = 0.5 * (virtual_rear_wheel_jnt_vel_[INDEX_RIGHT] + virtual_rear_wheel_jnt_vel_[INDEX_LEFT]);
    if(log_cnt_ % 500 == 0)
    {
      ROS_DEBUG_STREAM("rear_wheel_jnt_pos_ '" << rear_wheel_jnt_pos_ << " ' at writeSim()");
      ROS_DEBUG_STREAM("rear_wheel_jnt_vel_ '" << rear_wheel_jnt_vel_ << " ' at writeSim()");
    }
  }

  void SteerBotHardwareGazebo::CleanUp()
  {
    // wheel
    //-- wheel joint names
    rear_wheel_jnt_name_.empty();
    virtual_rear_wheel_jnt_names_.clear();
    //-- actual rear wheel joint
    rear_wheel_jnt_pos_ = 0;
    rear_wheel_jnt_vel_ = 0;
    rear_wheel_jnt_eff_ = 0;
    rear_wheel_jnt_vel_cmd_ = 0;
    //-- virtual rear wheel joint
    virtual_rear_wheel_jnt_pos_.clear();
    virtual_rear_wheel_jnt_vel_.clear();
    virtual_rear_wheel_jnt_eff_.clear();
    virtual_rear_wheel_jnt_vel_cmd_.clear();
    //-- virtual front wheel joint
    virtual_front_wheel_jnt_pos_.clear();
    virtual_front_wheel_jnt_vel_.clear();
    virtual_front_wheel_jnt_eff_.clear();
    virtual_front_wheel_jnt_vel_cmd_.clear();

    // steer
    //-- steer joint names
    front_steer_jnt_name_.empty();
    virtual_front_steer_jnt_names_.clear();
    //-- front steer joint
    front_steer_jnt_pos_ = 0;
    front_steer_jnt_vel_ = 0;
    front_steer_jnt_eff_ = 0;
    front_steer_jnt_pos_cmd_ = 0;
    //-- virtual wheel joint
    virtual_front_steer_jnt_pos_.clear();
    virtual_front_steer_jnt_vel_.clear();
    virtual_front_steer_jnt_eff_.clear();
    virtual_front_steer_jnt_pos_cmd_.clear();
  }

  void SteerBotHardwareGazebo::GetJointNames(ros::NodeHandle &_nh)
  {
    this->GetWheelJointNames(_nh);
    this->GetSteerJointNames(_nh);
  }

  void SteerBotHardwareGazebo::GetWheelJointNames(ros::NodeHandle &_nh)
  {
    // wheel joint to get linear command
    _nh.getParam(ns_ + "rear_wheel", rear_wheel_jnt_name_);

    // virtual wheel joint for gazebo control
    _nh.getParam(ns_ + "virtual_rear_wheels", virtual_rear_wheel_jnt_names_);
    int dof = virtual_rear_wheel_jnt_names_.size();
    virtual_rear_wheel_jnt_pos_.resize(dof);
    virtual_rear_wheel_jnt_vel_.resize(dof);
    virtual_rear_wheel_jnt_eff_.resize(dof);
    virtual_rear_wheel_jnt_vel_cmd_.resize(dof);

    _nh.getParam(ns_ + "virtual_front_wheels", virtual_front_wheel_jnt_names_);
    dof = virtual_front_wheel_jnt_names_.size();
    virtual_front_wheel_jnt_pos_.resize(dof);
    virtual_front_wheel_jnt_vel_.resize(dof);
    virtual_front_wheel_jnt_eff_.resize(dof);
    virtual_front_wheel_jnt_vel_cmd_.resize(dof);

  }

  void SteerBotHardwareGazebo::GetSteerJointNames(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", front_steer_jnt_name_);

    // virtual steer joint for gazebo control
    _nh.getParam(ns_ + "virtual_front_steers", virtual_front_steer_jnt_names_);

    const int dof = virtual_front_steer_jnt_names_.size();
    virtual_front_steer_jnt_pos_.resize(dof);
    virtual_front_steer_jnt_vel_.resize(dof);
    virtual_front_steer_jnt_eff_.resize(dof);
    virtual_front_steer_jnt_pos_cmd_.resize(dof);
  }

  void SteerBotHardwareGazebo::RegisterHardwareInterfaces()
  {
    this->RegisterSteerInterface();
    this->RegisterWheelInterface();

    // register mapped interface to ros_control
    registerInterface(&jnt_state_interface_);
    registerInterface(&rear_wheel_jnt_vel_cmd_interface_);
    registerInterface(&front_steer_jnt_pos_cmd_interface_);
  }

  void SteerBotHardwareGazebo::RegisterInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          hardware_interface::JointCommandInterface& _jnt_cmd_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff,  double& _jnt_cmd)
  {
    // register joint (both JointState and CommandJoint)
    this->RegisterJointStateInterfaceHandle(_jnt_state_interface, _jnt_name,
                                            _jnt_pos, _jnt_vel, _jnt_eff);
    this->RegisterCommandJointInterfaceHandle(_jnt_state_interface, _jnt_cmd_interface,
                                              _jnt_name, _jnt_cmd);
  }

  void SteerBotHardwareGazebo::RegisterInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff)
  {
    // register joint (both JointState and CommandJoint)
    this->RegisterJointStateInterfaceHandle(_jnt_state_interface, _jnt_name,
                                            _jnt_pos, _jnt_vel, _jnt_eff);
  }

  void SteerBotHardwareGazebo::RegisterWheelInterface()
  {
    // actual wheel joints
    this->RegisterInterfaceHandles(
          jnt_state_interface_, rear_wheel_jnt_vel_cmd_interface_,
          rear_wheel_jnt_name_, rear_wheel_jnt_pos_, rear_wheel_jnt_vel_, rear_wheel_jnt_eff_, rear_wheel_jnt_vel_cmd_);

    // virtual rear wheel joints
    for (int i = 0; i < virtual_rear_wheel_jnt_names_.size(); i++)
    {
      this->RegisterInterfaceHandles(
            jnt_state_interface_, rear_wheel_jnt_vel_cmd_interface_,
            virtual_rear_wheel_jnt_names_[i], virtual_rear_wheel_jnt_pos_[i], virtual_rear_wheel_jnt_vel_[i], virtual_rear_wheel_jnt_eff_[i], virtual_rear_wheel_jnt_vel_cmd_[i]);
    }
    // virtual front wheel joints
    for (int i = 0; i < virtual_front_wheel_jnt_names_.size(); i++)
    {
      this->RegisterInterfaceHandles(
            jnt_state_interface_, //front_wheel_jnt_vel_cmd_interface_,
            virtual_front_wheel_jnt_names_[i], virtual_front_wheel_jnt_pos_[i], virtual_front_wheel_jnt_vel_[i], virtual_front_wheel_jnt_eff_[i]/*, virtual_front_wheel_jnt_vel_cmd_[i]*/);
    }
  }

  void SteerBotHardwareGazebo::RegisterSteerInterface()
  {
    // actual steer joints
    this->RegisterInterfaceHandles(
          jnt_state_interface_, front_steer_jnt_pos_cmd_interface_,
          front_steer_jnt_name_, front_steer_jnt_pos_, front_steer_jnt_vel_, front_steer_jnt_eff_, front_steer_jnt_pos_cmd_);

    // virtual steer joints
    for (int i = 0; i < virtual_front_steer_jnt_names_.size(); i++)
    {
      this->RegisterInterfaceHandles(
            jnt_state_interface_, front_steer_jnt_pos_cmd_interface_,
            virtual_front_steer_jnt_names_[i], virtual_front_steer_jnt_pos_[i], virtual_front_steer_jnt_vel_[i], virtual_front_steer_jnt_eff_[i], virtual_front_steer_jnt_pos_cmd_[i]);
    }
  }

  void SteerBotHardwareGazebo::RegisterJointStateInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff)
  {
    hardware_interface::JointStateHandle state_handle(_jnt_name,
                                                      &_jnt_pos,
                                                      &_jnt_vel,
                                                      &_jnt_eff);
    _jnt_state_interface.registerHandle(state_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the JointStateInterface");
  }

  void SteerBotHardwareGazebo::RegisterCommandJointInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      hardware_interface::JointCommandInterface& _jnt_cmd_interface,
      const std::string _jnt_name, double& _jnt_cmd)
  {
    // joint command
    hardware_interface::JointHandle _handle(_jnt_state_interface.getHandle(_jnt_name),
                                            &_jnt_cmd);
    _jnt_cmd_interface.registerHandle(_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the CommandJointInterface");
  }


  double SteerBotHardwareGazebo::ComputeEffCommandFromVelError(
    const int _index, const int _front_rear, ros::Duration _period)
  {
    double vel_error = 0;
    if(_front_rear == FRONT)
        vel_error = rear_wheel_jnt_vel_cmd_ - virtual_front_wheel_jnt_vel_[_index];
    else if(_front_rear == REAR)
        vel_error = rear_wheel_jnt_vel_cmd_ - virtual_rear_wheel_jnt_vel_[_index];

    ROS_DEBUG_STREAM("vel_error = " << vel_error);
    if(fabs(vel_error) < 0.1)
    {
      vel_error = 0.0;
      ROS_DEBUG_STREAM("too small. vel_error <- 0");
    }
    else
      ROS_DEBUG_STREAM("not small. ");

    const double command = pids_[_index].computeCommand(vel_error, _period);
    ROS_DEBUG_STREAM("command =" << command);

    const double effort_limit = 10.0;
    const double effort = clamp(command,
                                -effort_limit, effort_limit);
    return effort;
  }

  void SteerBotHardwareGazebo::GetCurrentState(std::vector<double>& _jnt_pos, std::vector<double>& _jnt_vel, std::vector<double>& _jnt_eff,
                                               const int _if_index, const int _sim_jnt_index)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    _jnt_pos[_if_index] +=
        angles::shortest_angular_distance(_jnt_pos[_if_index], sim_joints_[_sim_jnt_index]->Position(0u));
#else
    _jnt_pos[_if_index] +=
        angles::shortest_angular_distance(_jnt_pos[_if_index], sim_joints_[_sim_jnt_index]->GetAngle(0u).Radian());
#endif
    _jnt_vel[_if_index] = sim_joints_[_sim_jnt_index]->GetVelocity(0u);
    _jnt_eff[_if_index] = sim_joints_[_sim_jnt_index]->GetForce(0u);
  }

} // namespace rosbook_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(steer_bot_hardware_gazebo::SteerBotHardwareGazebo, gazebo_ros_control::RobotHWSim)
