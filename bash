rostopic pub --once /left_tcp std_msgs/Float64MultiArray "data: [-0.4, 0.3, 0.9, 0.0, 0.0, -1.57]"
sudo ip link set can1 up type can bitrate 1000000
rostopic pub --once /left_arm_joints std_msgs/Float64MultiArray "data: [720, 90, 100, 180, 90, 180]"
rosservice call /set_zero "name: 'L_extend'"
(base) firefly@firefly:~/tashan/mgl/dm2c-controller$ rostopic pub -1 /left_arm/move_arm/goal common_comms/MoveArmActionGoal "
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: '1'
goal:
  arm_group: 'left_arm'
  frame_id: 'l_wrist_r_Link'
  target_pose:
    position:
      x: -0.06 
      y: 0.0
      z: 0.187
    orientation:
      x: 0
      y: 0
      z: 0
      w: 1
  named_target: ''
"
publishing and latching message for 3.0 seconds
(base) firefly@firefly:~/tashan/mgl/dm2c-controller$ rostopic pub -1 /left_gripper/gripper_cmd/goal common_comms/GripperCommandActionGoal \
> "header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}
> goal_id: {stamp: {secs: 0, nsecs: 0}, id: '0'}
> goal: {command: 'close'}"
publishing and latching message for 3.0 seconds

