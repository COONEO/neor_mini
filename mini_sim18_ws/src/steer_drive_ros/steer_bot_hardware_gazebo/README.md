### パラメータ

```yaml
steer_bot_hardware_gazebo:
    # 後輪フレーム
    rear_wheel  : 'base_to_wheel'
    # ステアフレーム
    front_steer : 'base_to_steer'
    # Gazebo用仮想後輪フレーム
    virtual_rear_wheels  : ['base_to_right_rear_wheel', 'base_to_left_rear_wheel']
    virtual_front_wheels : ['base_to_right_front_wheel', 'base_to_left_front_wheel']
    # Gazebo用仮想ステアフレーム
    virtual_front_steers : ['base_to_right_front_steer', 'base_to_left_front_steer']

    # ackermann link mechanism
    # trueならアッカーマン, falseならパラレルリンク．デフォルトはtrue．
    enable_ackermann_link: true
    # タイヤ間距離（左右）
    wheel_separation_w : 0.5
    # タイヤ間距離（前後）
    wheel_separation_h : 0.79
```

```yaml
gains:
　# 後輪速度制御用PIDゲイン
  base_to_right_rear_wheel  :  {p: 100000.0, d: 10.0, i: 0.50, i_clamp: 3.0}
  base_to_left_rear_wheel   :  {p: 100000.0, d: 10.0, i: 0.50, i_clamp: 3.0}
```

### SetVelocityが使えない．
- 仕様っぽい
  - http://answers.ros.org/question/118546/setting-joint-velocity-in-gazebo/
- VELOCITY_PIDでやるしかない．
