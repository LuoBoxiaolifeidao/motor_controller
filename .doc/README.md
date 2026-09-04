# 机械臂控制代码文档

本文件夹包含对 mgl 机械臂控制代码的完整分析文档。每次重新分析这套代码时，请先阅读本文件夹下的文档。

## 文件索引

| 文件 | 内容 |
|------|------|
| [01-architecture.md](01-architecture.md) | 整体架构概览、目录结构、关节与执行器对应关系 |
| [02-communication.md](02-communication.md) | 底层通信：串口层、Modbus RTU 帧构建、CRC16 校验 |
| [03-motor-control.md](03-motor-control.md) | 电机控制：DM2C、MWD485、飞特舵机的运动控制详解 |
| [04-gripper.md](04-gripper.md) | 夹爪控制：张开/闭合流程、参数配置 |
| [05-path-planning.md](05-path-planning.md) | 路径规划：S型7段曲线、多轴同步 |
| [06-kinematics.md](06-kinematics.md) | 正逆运动学：DH参数、解析法IK、FK链式计算 |
| [07-remote-control.md](07-remote-control.md) | 遥控操作：双方案对比、协议定义、操作指南 |

## 代码目录结构

```
mgl/
├── dm2c-rs556-controller/    # 🔧 DM2C独立测试工具 (不被主工程引用)
│   ├── DM2CController.h/cpp   # (测试版)
│   ├── ModbusFrameBuilder.h/cpp
│   ├── SerialPort.h/cpp        # (测试版: 无mutex, usleep盲等)
│   ├── main.cpp
│   └── stop.cpp
├── dm2c-controller/            # (旧版) DM2C控制器 catkin workspace
│   └── src/left_arm_ik.cpp     # 逆运动学求解 (旧版)
├── robot_controller/           # ✅ ROS主工程
│   ├── src/
│   │   ├── RobotServer.cpp     # ROS节点入口
│   │   ├── ControllerNode.cpp  # 核心控制节点
│   │   ├── SCurvPlan.cpp       # S曲线规划器封装
│   │   └── left_arm_ik.cpp     # IK (新版本)
│   ├── motors/
│   │   ├── DM2C/               # ★ 实际使用的串口/Modbus/DM2C驱动
│   │   │   ├── SerialPort.h/cpp
│   │   │   ├── ModbusFrameBuilder.h/cpp
│   │   │   └── DM2CController.h/cpp
│   │   ├── MWD/                # MWD485电机驱动
│   │   ├── s_curve_trajectory.hpp  # S曲线核心算法
│   │   └── synced_trajectory.cpp   # 多轴同步
│   ├── config/motors.yaml       # 电机参数配置
│   └── include/robot_controller/ # 头文件
└── .doc/                       # 本文档文件夹
```

## 快速核心概念

- **6关节左臂**: 升降柱(DM2C) + 肩部(MWD485) + 伸展(MWD485) + 腕部×3(飞特)
- **通信协议**: DM2C(Modbus RTU), MWD485(0x3E私有协议), 飞特私有协议 (腕部/夹爪)
- **控制模式**: 绝对位置、相对位置、速度模式
- **路径规划**: S型7段曲线 + 多轴时间同步
- **IK方法**: 解析几何法 + 迭代优化 (5次迭代)
- **夹爪**: 飞特舵机 ID=23, open=1748, close=900
