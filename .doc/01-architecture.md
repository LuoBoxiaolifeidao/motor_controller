# 整体架构概览

## 系统架构图

```
┌─────────────────────────────────────────────────────────┐
│  ROS 层 (RobotServer.cpp → ControllerNode)              │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐      │
│  │ TCP目标  │  │ 关节目标 │  │ MoveArm Action   │      │
│  │ (IK求解) │  │ (直驱)   │  │ (named_target/pose)│    │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘      │
│       └──────────────┴───────────────┘                 │
│                      ▼                                  │
│              driveJoints()                              │
│         (写入目标, 等待到位)                              │
│                      │                                  │
│       ┌──────────────┼──────────────┐                  │
│       ▼              ▼              ▼                   │
│  writePdo(DM2C) writePdo(MWD)  writePdo(FT)            │
│       │              │              │                   │
│  ┌────┴────┐   ┌─────┴─────┐  ┌───┴────┐              │
│  │DM2CController│MWD485Controller│SMS_STS_Shared│      │
│  │ (Modbus RTU) │ (0x3E私有协议) │ (飞特协议)   │      │
│  └────┬────┘   └─────┬─────┘  └───┬────┘              │
│       │              │              │                   │
├───────┴──────────────┴──────────────┴───────────────────┤
│  SerialPort 层 (Linux termios 串口)                      │
│  /dev/ttyACM0       /dev/ttysWK0                        │
│  38400 bps          115200 bps                          │
├─────────────────────────────────────────────────────────┤
│  电机驱动器:                                              │
│  [关节1] DM2C升降柱  [关节2] MWD485肩部  [关节3] MWD485伸展 │
│  [关节4-6] 飞特STS舵机(腕部)  [夹爪] 飞特舵机              │
└─────────────────────────────────────────────────────────┘
```

## 关节与执行器对应关系

| 索引 | 关节名 | 电机/驱动器 | 协议 | 通信接口 | 运动方向 |
|------|--------|------------|------|----------|---------|
| 0 | **升降柱 (Lift)** | DM2C RS556 | Modbus RTU | /dev/ttyACM0, 38400 | Z轴上下 |
| 1 | **肩部 (Shoulder)** | MWD485 | 0x3E私有协议 | /dev/ttysWK0, 115200 | 绕Z旋转(前后摆动) |
| 2 | **伸展 (Extend)** | MWD485 | 0x3E私有协议 | /dev/ttysWK0, 115200 | 直线伸缩 |
| 3 | **腕部俯仰 (Wrist1)** | 飞特 STS | 飞特私有协议 | /dev/ttysWK0, 115200 | 腕部旋转 |
| 4 | **腕部偏航 (Wrist2)** | 飞特 STS | 飞特私有协议 | /dev/ttysWK0, 115200 | 腕部旋转 |
| 5 | **腕部翻滚 (Wrist3)** | 飞特 STS | 飞特私有协议 | /dev/ttysWK0, 115200 | 腕部旋转 |
| 夹爪 | **Gripper** | 飞特舵机 | 飞特私有协议 | /dev/ttysWK0, 115200 | 开/闭 |

## 8自由度 IK 模型

机械臂在 URDF 中定义为 8 自由度：
- q[0] = 升降柱 (直线移动)
- q[1] = 肩部 (旋转)
- q[2/3/4] = 伸展 A/B/C (三段直线伸缩，映射到1个电机)
- q[5/6/7] = 腕部 pitch/yaw/roll (旋转)

IK 模型 8DOF → 实际控制 6 个物理电机，其中 q[2]+q[3]+q[4] 合并为一个伸展电机。

## 数据流概览

```
输入层:
  /left_arm_joints  (Float64MultiArray) → ArmJointsCB → driveJoints
  /left_tcp         (Float64MultiArray) → TcpCB → IK → driveJoints
  /left_arm/move_arm (MoveArmAction)    → executeMoveArm → IK → driveJoints
  /left_gripper/gripper_cmd (GripperAction) → executeGripper
  /set_zero         (Service)           → setZeroCB

输出层:
  /left_arm_joint_states (Float64MultiArray) → 各关节位置+速度

100Hz 主循环 (loop):
  readPdo(DM2C) → readPdo(MWD_Shoulder) → readPdo(MWD_Extend) → readPdo(FT)
  → publishState (每10个循环发布一次)
```

## 关键代码文件

| 文件 | 大小 | 职责 |
|------|------|------|
| ControllerNode.cpp | ~810行 | 核心控制节点：读写PDO、驱动关节、IK调用、夹爪控制 |
| DM2CController.cpp | ~247行 | DM2C电机封装：位置/速度/触发 |
| ModbusFrameBuilder.cpp | ~115行 | Modbus RTU帧构建 + CRC16 |
| SerialPort.cpp | ~94行 | Linux串口底层操作 |
| s_curve_trajectory.hpp | ~306行 | S型7段曲线核心算法 |
| synced_trajectory.cpp | ~69行 | 多轴时间同步 |
| left_arm_ik.cpp | ~227行 | 解析法逆运动学 |
| motors.yaml | ~84行 | 全部电机参数配置 |
