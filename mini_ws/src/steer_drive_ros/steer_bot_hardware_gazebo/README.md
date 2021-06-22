### Parameters

```yaml
steer_bot_hardware_gazebo:
    # rear wheel frame
    rear_wheel  : 'base_to_wheel'
    # steer frame
    front_steer : 'base_to_steer'
    # virtual rear frame for Gazebo
    virtual_rear_wheels  : ['base_to_right_rear_wheel', 'base_to_left_rear_wheel']
    virtual_front_wheels : ['base_to_right_front_wheel', 'base_to_left_front_wheel']
    # virtual steer frame for Gazebo
    virtual_front_steers : ['base_to_right_front_steer', 'base_to_left_front_steer']

    # ackermann link mechanism
    # true for ackermann, false for parallel link. default: true
    enable_ackermann_link: true
    # wheels separatation between the right & the left
    wheel_separation_w : 0.5
    # wheels separatation between the front & the rear
    wheel_separation_h : 0.79
```

```yaml
gains:
　# PID Gains for rear wheel velocity control
  base_to_right_rear_wheel  :  {p: 100000.0, d: 10.0, i: 0.50, i_clamp: 3.0}
  base_to_left_rear_wheel   :  {p: 100000.0, d: 10.0, i: 0.50, i_clamp: 3.0}
```

### In case of inavailable of SetVelocity method
- maybe specification
  - http://answers.ros.org/question/118546/setting-joint-velocity-in-gazebo/
- VELOCITY_PID is one way to solve it
