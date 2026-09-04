# 遥控操作方案设计文档

> 适用读者：接手本项目的工程师、AI 助手。
> 本文档记录了遥控操作需求的完整讨论过程、方案对比、工程设计问题修正、最终决策及实现计划。

---

## 一、背景

### 1.1 机械臂系统

- **硬件**：6 自由度左臂，RK3568 ARM 开发板运行 ROS Melodic
- **驱动**：DM2C（升降柱，Modbus RTU）+ MWD485（肩部/伸展，0x3E私有协议）+ 飞特 STS ×3（腕部，私有协议）+ 飞特舵机（夹爪）
- **ROS 接口**：`ControllerNode` 核心控制节点，100Hz 主循环，已有完整电机驱动和 IK 解算

### 1.2 遥控需求

- **场景**：操作者站在机械臂旁边，直接看着机器人操作。**没有摄像头回传画面。**
- **操作目标**：完成抓取（grasp）、搬运（safe_carry）、放置（place）、归位（home）、准备（ready）等动作
- **遥控方式**：手机 App（iOS/Android，由另一个团队开发）通过 WiFi 连接 RK3568
- **操作模式**：类似遥控底盘 —— 推摇杆 → 电机转 → 眼睛看结果 → 再调

### 1.3 关键约束

| 约束 | 说明 |
|------|------|
| 无视觉反馈 | 没有摄像头回传画面，操作者直接目视机械臂 |
| 无标定 | 没有相机-机械臂手眼标定 |
| 手机App | 遥控端不能运行 ROS，只能 WebSocket + JSON |
| 通信桥 | rosbridge（`rosbridge_server`），WebSocket JSON → ROS topic |

---

## 二、方案探索过程

### 2.1 初始方向：TCP 笛卡尔空间位姿遥控

最初的思路是手机 App 发送末端 TCP 位姿 `[x, y, z, roll, pitch, yaw]`，机载端 IK 解算 → 驱动 6 个关节。优势是：

- 操作直觉：想"手往哪移"就直接推摇杆，不需要关心关节怎么转
- IK 自动处理 6 轴耦合，末端直线运动不会跑偏
- IK 精度：位置 < 1mm，姿态 < 0.05 rad

### 2.2 质疑与风险分析

讨论中提出了三个层次的质疑：

**第一层：IK 可靠性**

遥控操作是 30Hz 持续流式输入，不是单次目标点。IK 在实时流式场景下有三类风险：

| 风险 | 说明 | 影响 |
|------|------|------|
| 目标移出工作空间 | 操作者推摇杆让末端出界 → IK 不收敛 → 返回失败 | 机械臂不动了，操作者不知道为什么 |
| 腕部解不连续 | 当前 `solve_wrist_from_R` 只返回单解，相邻帧可能跳变到另一组解 | 腕部突然翻转 180°，危险 |
| 奇异点附近不稳定 | 腕部 s4≈0 时 q3/q5 耦合，解算质量下降 | 末端抖动 |

**第二层：目标位姿从哪来**

TCP 坐标不是凭空产生的。在无摄像头、无视觉检测、无标定的场景下，操作者无法知道"杯子在坐标系里的 (x,y,z) 是多少"。TCP 方案依赖于一个明确的数字目标，但这个数字从哪来？

**第三层：操作者实际怎么操作**

操作者是站在机器人旁边目视操作的。他看到的是关节运动，不是 XYZ 坐标。推摇杆让肩部转 3° 是看得见的反馈；推摇杆让末端往 X+ 移动 5cm 是看不见的。在这个场景下，关节空间控制反而更符合操作者的感知。

### 2.3 方向修正：关节空间直驱

经过三轮讨论，明确了实际场景后，方案从 TCP 笛卡尔空间控制修正为**关节空间直驱**。

核心判断依据：

> 操作者眼睛直接看着机械臂，他需要控制的是"这个关节转多少"，而不是"末端在空间的 XYZ 是多少"。关节直驱和遥控底盘的逻辑完全相同 —— 推摇杆 → 电机动 → 眼睛看到动了多少 → 够了就停。

---

## 三、最终方案：关节空间直驱

### 3.1 整体架构

```
┌──────────────────────────────────────────────────────────────┐
│                        RK3568 (机载端)                        │
│                                                              │
│  roslaunch robot_controller teleop.launch                     │
│                                                              │
│  ┌─────────────────┐       ros topic     ┌─────────────────┐ │
│  │ rosbridge_server │◄══════════════════►│ ControllerNode  │ │
│  │ (WebSocket       │                    │                 │ │
│  │  ws://0.0.0.0:   │                    │ 新增3个方法:     │ │
│  │  9090)           │ /left_teleop_joints │ driveJointsTeleop│ │
│  │                  │  (遥控专用topic)    │ stopMotorsSoft  │ │
│  │                  │                    │ getJointPositions│ │
│  │                  │ /left_gripper_cmd  │                 │ │
│  │                  │ /left_estop        │ TeleopHandler   │ │
│  │                  │ /left_reset (新增) │ (新建独立文件)  │ │
│  │                  │                    │                 │ │
│  │                  │ /left_arm_joint_   │                 │ │
│  │                  │ states (反馈)      │ publishState()  │ │
│  └────────┬────────┘                    └────────┬────────┘ │
│           │ WebSocket JSON                       │ 串口      │
│           │                                      ▼           │
│  ┌────────┴──────────┐                     6 个电机          │
│  │   手机App          │                                     │
│  │  (iOS/Android)    │                                     │
│  │                   │                                     │
│  │  虚拟摇杆 → 关节  │                                     │
│  │  按钮 → 夹爪/急停 │                                     │
│  │  显示 → 实时状态  │                                     │
│  └───────────────────┘                                     │
│                                                              │
│  操作者站在机械臂旁边，直接目视操作                             │
└──────────────────────────────────────────────────────────────┘
```

### 3.2 为什么选这个方案

| 维度 | 理由 |
|------|------|
| **操作直觉** | 操作者看到的是关节运动，推摇杆 → 关节转 → 眼睛确认，直接对应 |
| **无依赖** | 不需要相机、不需要标定、不需要物体检测、不需要坐标系 |
| **安全性** | 关节限位直接裁剪，每个轴独立，不会出现 IK 跳变 |
| **易调试** | 某个关节不动 → 直接看到 → 排查该电机/驱动器 |
| **学习成本** | 类似遥控底盘，几分钟上手 |

---

## 四、协议定义

### 4.1 通信方式

```
App ──WebSocket──► rosbridge_server (RK3568:9090) ──ROS topic──► ControllerNode

协议: rosbridge v2.0 (JSON over WebSocket)
```

### 4.2 App → RK3568 控制消息

**关节控制（持续发送，20~30Hz）**：

```json
{
  "op": "publish",
  "topic": "/left_teleop_joints",
  "msg": {
    "layout": {
      "dim": [{"label": "joints", "size": 6, "stride": 6}],
      "data_offset": 0
    },
    "data": [450.0, 32.0, 120.0, -15.0, 8.0, 45.0]
  }
}
```

`data` 字段含义：

| 索引 | 关节 | 单位 | 范围 | 驱动 |
|------|------|------|------|------|
| 0 | 升降柱 (Lift) | mm | [0, 950] | DM2C RS556 |
| 1 | 肩部 (Shoulder) | ° | [0, 90] | MWD485 |
| 2 | 伸展 (Extend) | mm | [0, 360] | MWD485 |
| 3 | 腕俯仰 (Wrist Pitch) | ° | [-180, 180] | 飞特 STS |
| 4 | 腕偏航 (Wrist Yaw) | ° | [-90, 90] | 飞特 STS |
| 5 | 腕翻滚 (Wrist Roll) | ° | [-180, 180] | 飞特 STS |

> **关键设计决策**：遥控使用专用 topic `/left_teleop_joints`，**不复用** `/left_arm_joints`。
> 原因：`/left_arm_joints` 的回调 `ArmJointsCB` 最终调用 `driveJoints()`，后者内部有 `while(ros::ok())` 阻塞等待死区的死循环（[ControllerNode.cpp:379](robot_controller/src/ControllerNode.cpp#L379)）。30Hz 流式指令会瞬间堵死 ROS 回调队列。

**夹爪控制（按需发送）**：

```json
{"op": "publish", "topic": "/left_gripper_cmd", "msg": {"data": "open"}}
{"op": "publish", "topic": "/left_gripper_cmd", "msg": {"data": "close"}}
```

**硬急停（按需发送）**：

```json
{"op": "publish", "topic": "/left_estop", "msg": {}}
```

**软停止恢复（按需发送）**：

```json
{"op": "publish", "topic": "/left_reset", "msg": {}}
```

### 4.3 RK3568 → App 反馈消息

**订阅**（App 连接后发一次）：

```json
{"op": "subscribe", "topic": "/left_arm_joint_states"}
```

**接收**（RK3568 10Hz 自动推送）：

```json
{
  "topic": "/left_arm_joint_states",
  "msg": {
    "data": [450.2, 32.5, 120.3, -15.0, 8.2, 45.1,
             0.0, 2.1, 5.3, 0.5, 0.0, 1.2]
  }
}
```

| 索引 | 含义 | 单位 |
|------|------|------|
| 0-5 | 6 关节实际位置 | 同控制单位 |
| 6-11 | 6 关节实际速度 | mm/s 或 °/s |

### 4.4 协议总览

```
App 发送:
  /left_teleop_joints  Float64MultiArray[6]  20~30Hz  关节目标
  /left_gripper_cmd    String                 按需     夹爪 "open"/"close"
  /left_estop          Empty                  按需     硬急停（不可恢复）
  /left_reset          Empty                  按需     软停止恢复

App 接收:
  /left_arm_joint_states  Float64MultiArray[12]  10Hz  关节位置+速度
```

---

## 五、手机 App 实现要点

### 5.1 摇杆 → 关节增量映射

```swift
// 初始值：从反馈读取当前实际位置
var joints: [Double] = []

// 速度表
enum Gear {
    case slow, medium, fast
    var scale: Double {
        switch self {
        case .slow:   return 1.0
        case .medium: return 4.0
        case .fast:   return 10.0
        }
    }
}

// 摇杆灵敏度（档位×基速）
// 升降柱:  1.0 × 60mm/s @ 满摇杆
// 肩部:    1.0 × 8°/s   @ 满摇杆
// 伸展:    1.0 × 40mm/s @ 满摇杆
// 腕部:    1.0 × 20°/s  @ 满摇杆

// 30Hz 定时器每帧:
func update(dt: Double) {
    let g = currentGear.scale

    joints[0] += leftStickY  * g * 60.0 * dt   // 升降柱
    joints[1] += leftStickX  * g * 8.0  * dt   // 肩部
    joints[2] += rightStickY * g * 40.0 * dt   // 伸展
    joints[3] += rightStickX * g * 20.0 * dt   // 腕俯仰

    if lbPressed {
        joints[4] += rightStickY * g * 20.0 * dt  // 腕偏航
        joints[5] += rightStickX * g * 20.0 * dt  // 腕翻滚
    }

    // 裁剪到限位
    joints[0] = clamp(joints[0], 0, 950)
    joints[1] = clamp(joints[1], 0, 90)
    joints[2] = clamp(joints[2], 0, 360)
    joints[3] = clamp(joints[3], -180, 180)
    joints[4] = clamp(joints[4], -90, 90)
    joints[5] = clamp(joints[5], -180, 180)

    sendJointTargets(joints)  // → /left_teleop_joints
}
```

### 5.2 UI 布局建议

```
┌──────────────────────────────┐
│  机械臂遥控                  │
│                              │
│  升降: 450mm   肩部: 32°    │  ← 实时反馈读数
│  伸展: 120mm   腕俯仰: -15° │
│  腕偏航: 8°    腕翻滚: 45°  │
│  夹爪: 闭合                  │
│                              │
│   ┌─────────┐ ┌─────────┐   │
│   │ 左摇杆   │ │ 右摇杆   │   │
│   │ ↑升降   │ │ ↑伸展   │   │
│   │←肩→    │ │←腕俯→  │   │
│   │ ↓       │ │ ↓       │   │
│   └─────────┘ └─────────┘   │
│                              │
│  [LB] 按住切换腕偏航/翻滚     │
│                              │
│  速度: 慢  |  中  |  快      │
│                              │
│  [🤖 闭合]  [📂 张开]        │
│  [🏠 Home]  [🛑 急停]        │
│  [🔄 恢复]  (软停止后恢复)   │
└──────────────────────────────┘
```

### 5.3 操作流程示例：抓取杯子

```
操作者站在机器人旁边:

1. 慢档 → 左摇杆下拉 → 升降柱下降 → 看到夹爪降到杯子高度 → 松手
2. 右摇杆上推 → 伸展臂伸出 → 看到夹爪靠近杯子 → 松手
3. 左摇杆左右 → 肩部微调 → 看到夹爪对准杯子正上方 → 松手
4. 按住LB + 右摇杆 → 腕部姿态微调 → 看到夹爪角度合适 → 松手
5. 点[闭合] → 夹爪抓住杯子
6. 左摇杆上推 → 升降柱上升 → 杯子提起（完成 Grasp）
7. 左摇杆左右 → 肩部转 → 移到放置点上方（SafeCarry）
8. 慢档 → 左摇杆下拉 → 杯子下降 → 接触桌面 → 松手
9. 点[张开] → 释放杯子（完成 Place）
10. 点[Home] → 自动收拢
```

---

## 六、工程设计问题分析与修正

初始方案经代码审查发现三处致命漏洞。本节逐一给出根因和修正方案。

### 6.1 问题一：driveJoints() 阻塞死锁

**根因** — [ControllerNode.cpp:379](robot_controller/src/ControllerNode.cpp#L379)：

```cpp
void ControllerNode::driveJoints(const std::vector<double>& target) {
    // set targets, brakeRelease, writePdo x4 ...
    ros::Rate rate(100);
    while (ros::ok() && running_) {   // ← 阻塞等待所有关节到位
        if (all_in_zone) break;
        rate.sleep();
    }
}
```

如果复用 `/left_arm_joints` → `ArmJointsCB` → `driveJoints()`，30Hz 遥控流每 33ms 触发一次回调，每次回调阻塞 100ms~2s。ROS AsyncSpinner 仅 2 个线程，回调队列瞬间堵塞，节点假死。

**修正**：

1. 遥控关节指令使用**专用 topic** `/left_teleop_joints`，不复用 `/left_arm_joints`
2. ControllerNode 新增 `driveJointsTeleop()` — 只设 `targetPos_` + 调 `writePdo()`，**不等待到位**
3. TeleopHandler 回调中直接调用 `driveJointsTeleop()`，绕过 `driveJoints()`

### 6.2 问题二：无防跳变限幅

**根因** — [ControllerNode.cpp:574](robot_controller/src/ControllerNode.cpp#L574)：

```cpp
case Motor::DM2C: {
    double err = targetPos_[Lift] - curState_[Lift].position;  // 初始可达 300mm+
    jointDM2C_->moveToRelativePosition(-err / DM2CTRAC, max_speed, acc, acc);
}
```

App 初始化或重连时，目标值可能是预设值（如 `[300,0,0,0,0,0]`），而机械臂实际位置远在 `[600,45,150,...]`。第一帧指令如直接写入，升降柱会以最大速度冲刺 300mm，极易损坏机械结构。

**修正**：

TeleopHandler 收到每帧指令后，先从 ControllerNode 取当前真实位置，计算 `|target - current|`：

| 关节 | 单帧最大跳变 |
|------|:---:|
| 升降柱 (mm) | 10 |
| 肩部 (°) | 5 |
| 伸展 (mm) | 10 |
| 腕部 (°) | 5 |

任一关节超限 → 丢弃整帧 + `ROS_WARN`，等待下一帧正常指令。

### 6.3 问题三：急停死锁无法恢复

**根因** — [ControllerNode.cpp:347](robot_controller/src/ControllerNode.cpp#L347)：

```cpp
void ControllerNode::stopAll() {
    running_ = false;   // ← 终止主循环，不可逆
    // stop motors, lock brakes, disable torque ...
}
```

`stopAll()` 设 `running_ = false` 导致主循环永久退出。初始方案中，超时和急停按钮都调用 `stopAll()`，且 `estopped_` 标志置 true 后无复位路径。一次 0.6 秒网络波动 → 整系统永久死锁，必须重启节点。

**修正**：区分两级停止：

| 等级 | 触发条件 | 动作 | running_ | 恢复方式 |
|------|---------|------|:---:|---------|
| **软停止** | 通信超时 0.5s | `stopMotorsSoft()` | 保持 true | App 点恢复 → `/left_reset` |
| **硬急停** | App 急停按钮 | `stopAll()` | 设为 false | 必须重启节点 |

新增 `/left_reset` topic：收到后清除软停止标志，下一帧指令自动重新释放刹车 + 使能扭矩。

### 6.4 问题四：初始化时序 — "落地成盒"

**根因**：TeleopHandler 构造函数中：

```cpp
last_cmd_time_ = ros::Time::now();   // ← 从节点启动瞬间开始计时
```

`roslaunch` 启动节点后，主循环 `loop()` 立刻开始运行。此时手机 App 根本还没连上 WiFi，更别说发指令。节点启动 0.5 秒后，`checkTimeout()` 判定超时，触发 `softStop()` 并设 `soft_stopped_ = true`。机械臂一开机就进入急停锁死状态。

**修正**：

1. 构造函数中 `last_cmd_time_` 初始化为 `ros::Time(0)`
2. `checkTimeout()` 中增加判断：`last_cmd_time_.isZero()` 时直接返回 false，**不作超时检查**
3. 只有当 App 至少发过一次指令（`jointCmdCB` 更新了时间戳）后，超时检测才真正生效

### 6.5 问题五：串口总线带宽与锁竞争

**根因**：肩部（MWD485）、伸展（MWD485）和腕部（飞特 STS）共享物理串口 `/dev/ttysWK0`（[02-communication.md](02-communication.md) 第四节），通过 `ft_mutex_` 和 SerialPort 内部互斥锁保护。

遥操作触发后：

| 操作 | 来源 | 频率 | 设备 |
|------|------|------|------|
| `readPdo` ×3 | `loop()` 线程 | **100Hz** | MWD485 肩部, MWD485 伸展, 飞特 |
| `writePdo` ×3 | ROS 回调线程 | **30Hz** | MWD485 肩部, MWD485 伸展, 飞特 |

两线程以 **390 次/秒** 抢同一把锁访问 115200 bps 串口总线。在 RK3568 ARM 上极易锁竞争（Lock Contention）：

- **轻则**：串口数据碰撞 → CRC 校验失败 → 读空/写丢
- **重则**：写线程长期抢不到锁 → 控制指令延迟 → 机械臂动作发飘

**修正**：遥操作模式下，`readPdo` 频率从 100Hz 降至 **20Hz**。数据量：60 read + 90 write = **150 次/秒**，串口 115200 bps 理论极限约 480 次/秒，留有 3 倍余量。

实现方式：`loop()` 中增加 `teleop_.isActive()` 判断，活跃时用 20Hz 速率读取，非活跃时保持 100Hz。

### 6.6 问题六：TCP 网络拥塞 — 帧堆积

**根因**：rosbridge 底层使用 WebSocket over TCP。TCP 协议保证送达：丢包时底层死等重传。展厅 WiFi 环境（2.4G/5G 干扰、数百台设备）下：

```
现场 WiFi 短暂丢包 (100~500ms)
    │
    ▼
TCP 栈缓冲 App 发出的连续几十帧关节目标
    │
    ▼ (网络恢复)
几十帧历史数据在几毫秒内倾泻到 /left_teleop_joints
```

虽然 6.2 节的防跳变会拦截单帧过大跳变，但**连续多帧正常幅度的快速涌入**仍会导致机械臂短时间剧烈抖动。

**定性**：此问题的根因在 App 端和网络层，底层 C++ 代码无法根治。

**缓解措施**（需记录的技术边界）：

1. **App 端建议**：每条消息带单调递增的序列号，`jointCmdCB` 检查并丢弃乱序/过时帧
2. **rosbridge 调参**：适当降低 WebSocket 发送缓冲区大小
3. **排障指引**：如果操作员反馈"手感卡顿、时好时坏"，优先排查 WiFi 信道干扰和路由器负载，这不是底层控制代码的 Bug

---

## 七、修正后的实现代码

遥控逻辑独立为 `TeleopHandler` 类。ControllerNode 通过 `std::function` 回调暴露底层能力。

### 7.1 设计原则

```
┌──────────────────────────────────────────────────────┐
│  ControllerNode（新增 3 个方法 + 构造传回调）          │
│                                                      │
│  新增方法:                                            │
│    driveJointsTeleop(target)  — 非阻塞直写 writePdo   │
│    stopMotorsSoft()           — 停电机但保持 loop     │
│    getJointPositions()        — 返回当前关节位置      │
│                                                      │
│  构造函数中注册回调到 TeleopHandler:                   │
│    · DriveFn    → driveJointsTeleop                  │
│    · SoftStopFn → stopMotorsSoft                     │
│    · HardStopFn → stopAll                            │
│    · StateFn    → getJointPositions                  │
│    · GripperFn  → EnableTorque + WritePosEx          │
│    · ResetFn    → brakeRelease + EnableTorque        │
│                                                      │
│  loop() 中: teleop_.checkTimeout() → 超时调软停止      │
├──────────────────────────────────────────────────────┤
│  TeleopHandler（新建，~130行）                         │
│                                                      │
│  订阅:                                                │
│    /left_teleop_joints  ← 遥控专用 topic              │
│    /left_gripper_cmd    ← 夹爪                       │
│    /left_estop          ← 硬急停                      │
│    /left_reset          ← 软停止恢复                   │
│                                                      │
│  核心逻辑:                                            │
│    jointCmdCB: 跳变检查 → 首次释放刹车 → 非阻塞 drive  │
│    checkTimeout: 超时 → SoftStopFn (不杀 loop)       │
│    estopCB:     硬急停 → HardStopFn                  │
│    resetCB:     清除软停止标志                         │
│    gripperCmdCB: 夹爪开/闭                            │
└──────────────────────────────────────────────────────┘
```

### 7.2 改动清单

| # | 文件 | 操作 | 行数 |
|---|------|------|:---:|
| 1 | `TeleopHandler.h` | **新建** | 55 |
| 2 | `TeleopHandler.cpp` | **新建** | 75 |
| 3 | `ControllerNode.h` | 加 include + 成员 + 3 方法声明 | +6 |
| 4 | `ControllerNode.cpp` | 3 新方法 + 构造传回调 + loop 超时 | +35 |
| 5 | `CMakeLists.txt` | 加编译源文件 | +1 |

### 7.3 新建文件

#### TeleopHandler.h

文件位置：`robot_controller/include/robot_controller/TeleopHandler.h`

```cpp
#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <functional>
#include <vector>

struct GripperConfig {
    int id;
    int open;    // 1748
    int close;   // 900
};

class TeleopHandler {
public:
    using DriveFn     = std::function<void(const std::vector<double>&)>;
    using SoftStopFn  = std::function<void()>;
    using HardStopFn  = std::function<void()>;
    using StateFn     = std::function<std::vector<double>()>;
    using GripperFn   = std::function<void(int id, int target)>;
    using ResetFn     = std::function<void()>;

    TeleopHandler(ros::NodeHandle& nh,
                  DriveFn     drive_fn,
                  SoftStopFn  soft_stop_fn,
                  HardStopFn  hard_stop_fn,
                  StateFn     state_fn,
                  GripperFn   gripper_fn,
                  ResetFn     reset_fn,
                  const GripperConfig& cfg);

    /// 每 500ms 调用一次。超时 → 软停止（可恢复）
    /// @return true = 已触发停止
    bool checkTimeout(double timeout_sec = 0.5);

    /// 遥控是否处于活跃状态
    bool isActive() const { return active_ && !soft_stopped_ && !hard_stopped_; }

private:
    void jointCmdCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void gripperCmdCB(const std_msgs::String::ConstPtr& msg);
    void estopCB(const std_msgs::Empty::ConstPtr& msg);
    void resetCB(const std_msgs::Empty::ConstPtr& msg);

    ros::Subscriber joint_sub_;
    ros::Subscriber gripper_cmd_sub_;
    ros::Subscriber estop_sub_;
    ros::Subscriber reset_sub_;
    ros::Time       last_cmd_time_;

    DriveFn     drive_fn_;
    SoftStopFn  soft_stop_fn_;
    HardStopFn  hard_stop_fn_;
    StateFn     state_fn_;
    GripperFn   gripper_fn_;
    ResetFn     reset_fn_;
    GripperConfig gripper_cfg_;

    bool active_        = false;
    bool soft_stopped_  = false;
    bool hard_stopped_  = false;

    // 防跳变阈值: 单帧最大允许变化量
    static constexpr double MAX_JUMP_MM  = 10.0;
    static constexpr double MAX_JUMP_DEG = 5.0;
};
```

#### TeleopHandler.cpp

文件位置：`robot_controller/src/TeleopHandler.cpp`

```cpp
#include "TeleopHandler.h"
#include <cmath>

TeleopHandler::TeleopHandler(ros::NodeHandle& nh,
                             DriveFn     drive_fn,
                             SoftStopFn  soft_stop_fn,
                             HardStopFn  hard_stop_fn,
                             StateFn     state_fn,
                             GripperFn   gripper_fn,
                             ResetFn     reset_fn,
                             const GripperConfig& cfg)
    : drive_fn_(drive_fn)
    , soft_stop_fn_(soft_stop_fn)
    , hard_stop_fn_(hard_stop_fn)
    , state_fn_(state_fn)
    , gripper_fn_(gripper_fn)
    , reset_fn_(reset_fn)
    , gripper_cfg_(cfg)
{
    joint_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
        "left_teleop_joints", 1, &TeleopHandler::jointCmdCB, this);
    gripper_cmd_sub_ = nh.subscribe<std_msgs::String>(
        "left_gripper_cmd", 1, &TeleopHandler::gripperCmdCB, this);
    estop_sub_ = nh.subscribe<std_msgs::Empty>(
        "left_estop", 1, &TeleopHandler::estopCB, this);
    reset_sub_ = nh.subscribe<std_msgs::Empty>(
        "left_reset", 1, &TeleopHandler::resetCB, this);
    // 初始化为 0: 首次收到 App 指令前不做超时检查 (防止"落地成盒")
    last_cmd_time_ = ros::Time(0);
}

// ── 关节指令回调: 跳变检查 → 非阻塞驱动 ────────────
void TeleopHandler::jointCmdCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 6) return;
    if (hard_stopped_)         return;
    if (soft_stopped_)         return;

    last_cmd_time_ = ros::Time::now();
    std::vector<double> target(msg->data.begin(), msg->data.begin() + 6);

    // 防跳变: 与当前真实位置比较, 超限则丢弃该帧
    auto current = state_fn_();
    if (current.size() >= 6) {
        const double max_jump[6] = {
            MAX_JUMP_MM, MAX_JUMP_DEG, MAX_JUMP_MM,
            MAX_JUMP_DEG, MAX_JUMP_DEG, MAX_JUMP_DEG
        };
        for (int i = 0; i < 6; ++i) {
            if (std::abs(target[i] - current[i]) > max_jump[i]) {
                ROS_WARN("[Teleop] JUMP REJECT joint[%d]: cur=%.1f→tgt=%.1f",
                         i, current[i], target[i]);
                return;  // 丢弃, 等下一帧
            }
        }
    }

    // 首帧指令: 释放刹车 + 使能扭矩
    if (!active_) {
        reset_fn_();
        active_ = true;
    }

    // 非阻塞直写电机
    drive_fn_(target);
}

// ── 夹爪回调 ─────────────────────────────────────
void TeleopHandler::gripperCmdCB(const std_msgs::String::ConstPtr& msg)
{
    int target;
    if (msg->data == "open")       target = gripper_cfg_.open;
    else if (msg->data == "close") target = gripper_cfg_.close;
    else {
        ROS_WARN("[Teleop] unknown gripper cmd: %s", msg->data.c_str());
        return;
    }
    ROS_INFO("[Teleop] gripper → %d", target);
    gripper_fn_(gripper_cfg_.id, target);
}

// ── 硬急停: 不可恢复 ──────────────────────────────
void TeleopHandler::estopCB(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_WARN("[Teleop] HARD ESTOP — requires node restart");
    hard_stopped_ = true;
    hard_stop_fn_();  // → stopAll() → running_=false
}

// ── 软停止恢复 ────────────────────────────────────
void TeleopHandler::resetCB(const std_msgs::Empty::ConstPtr& msg)
{
    if (hard_stopped_) {
        ROS_WARN("[Teleop] cannot reset after hard estop");
        return;
    }
    if (!soft_stopped_) return;

    ROS_INFO("[Teleop] RESET — clearing soft stop");
    soft_stopped_ = false;
    active_       = false;       // 下一帧 jointCmdCB 会重新 release brake
    last_cmd_time_ = ros::Time(0);  // 重置: 等下一帧真实指令到达后再开始计时
}

// ── 超时检测 (ControllerNode::loop 每 500ms 调用) ─
bool TeleopHandler::checkTimeout(double timeout_sec)
{
    if (hard_stopped_ || soft_stopped_) return true;
    if (last_cmd_time_.isZero())        return false;  // 尚未收到首帧指令, 不报超时

    if ((ros::Time::now() - last_cmd_time_).toSec() > timeout_sec) {
        ROS_WARN("[Teleop] timeout (%.1fs) — soft stop", timeout_sec);
        soft_stopped_ = true;
        soft_stop_fn_();  // → stopMotorsSoft(), 不杀 loop
        return true;
    }
    return false;
}
```

### 7.4 ControllerNode 新增内容

#### ControllerNode.h（+6 行）

```cpp
// include 区:
#include "TeleopHandler.h"

// 类成员:
TeleopHandler teleop_;

// 方法声明:
void driveJointsTeleop(const std::vector<double>& target);
void stopMotorsSoft();
std::vector<double> getJointPositions() const;
```

#### ControllerNode.cpp — 3 个新方法（+30 行）

```cpp
// === 非阻塞遥控驱动: 设 targetPos_ + 直写 writePdo, 不等待到位 ===
void ControllerNode::driveJointsTeleop(const std::vector<double>& target)
{
    for (int i = 0; i < kJointNum; ++i)
        targetPos_[i] = target[i];

    writePdo(Motor::DM2C);
    writePdo(Motor::MWD_Shoulder);
    writePdo(Motor::MWD_Extend);
    writePdo(Motor::FT);
    // 不调 brakeRelease/brakeLock — 由 TeleopHandler 的 active_ 状态管理
}

// === 软停止: 停电机 + 锁刹车, running_ 保持 true ===
void ControllerNode::stopMotorsSoft()
{
    jointDM2C_->stopAllMotion();
    jointShoulder_->stop();
    jointShoulder_->brakeLock();
    jointExtend_->stop();
    jointExtend_->brakeLock();
    for (int id : ft_cfg_.ids)
        jointFT_->EnableTorque(id, 0);
    // 关键: 不设 running_ = false; loop 继续运行
}

// === 读当前关节位置供防跳变检查 ===
std::vector<double> ControllerNode::getJointPositions() const
{
    std::vector<double> pos(kJointNum, 0.0);
    for (int i = 0; i < kJointNum; ++i)
        pos[i] = curState_[i].position;
    return pos;
}
```

#### 构造函数 — 注册回调（初始化列表）

```cpp
ControllerNode::ControllerNode()
    : teleop_(nh_,
              // drive_fn: 非阻塞直写
              [this](const std::vector<double>& t){ driveJointsTeleop(t); },
              // soft_stop_fn: 超时软停止
              [this](){ stopMotorsSoft(); },
              // hard_stop_fn: 急停按钮
              [this](){ stopAll(); },
              // state_fn: 当前关节位置 (防跳变)
              [this](){ return getJointPositions(); },
              // gripper_fn
              [this](int id, int target) {
                  if (!running_) return;
                  std::lock_guard<std::mutex> lock(ft_mutex_);
                  jointFT_->EnableTorque(id, 1);
                  jointFT_->WritePosEx(id, static_cast<s16>(target), 100, 100);
              },
              // reset_fn: 首次指令/恢复时释放刹车+使能
              [this](){
                  jointShoulder_->brakeRelease();
                  jointExtend_->brakeRelease();
                  for (int id : ft_cfg_.ids)
                      jointFT_->EnableTorque(id, 1);
              },
              GripperConfig{gripper_cfg_.id, gripper_cfg_.open, gripper_cfg_.close})
{
    // ... 现有初始化代码不变 ...
```

#### loop() — 超时检测 + 遥操作降频（问题四/五修正）

```cpp
// loop() 内:
ros::Rate rate(100);  // 主循环 100Hz
int loop_cnt = 0;
while (ros::ok() && running_) {
    {
        std::lock_guard<std::mutex> lock(curState_mutex_);

        // 遥操作活跃时降频读取: 20Hz (让出串口带宽给 30Hz 写入)
        bool teleop_active = teleop_.isActive();
        int  read_divider  = teleop_active ? 5 : 1;   // 100/5=20Hz vs 100Hz

        if (loop_cnt % read_divider == 0) {
            readPdo(Motor::DM2C);           // /dev/ttyACM0 (独立串口, 不受影响)
            readPdo(Motor::MWD_Shoulder);   // /dev/ttysWK0
            readPdo(Motor::MWD_Extend);     // /dev/ttysWK0
            readPdo(Motor::FT);             // /dev/ttysWK0
        }
    }

    if (++loop_cnt % 10 == 0)
        publishState();  // 10Hz 发布 (不变)

    // 超时检测: 每 500ms
    static int timeout_cnt = 0;
    if (++timeout_cnt >= 50) {
        timeout_cnt = 0;
        teleop_.checkTimeout(0.5);  // 首次指令到达前不报超时 (last_cmd_time_=0)
    }
    // 注意: 不 break; 软停止后 loop 继续, 等 /left_reset 恢复

    rate.sleep();
}
```

### 7.5 CMakeLists.txt

```cmake
add_executable(robot_controller
  src/RobotServer.cpp
  src/ControllerNode.cpp
  src/TeleopHandler.cpp        # ← 新增
  src/SCurvPlan.cpp
  src/left_arm_ik.cpp
  motors/synced_trajectory.cpp
  ${DM2C_SRCS}
  ${MWD485_SRCS}
  ${MWD_SRCS}
)
```

### 7.6 启动文件

[teleop.launch](robot_controller/launch/teleop.launch)：

```xml
<launch>
  <node name="dm2c_controller" pkg="robot_controller"
        type="robot_controller" output="screen"/>

  <node name="rosbridge_websocket" pkg="rosbridge_server"
        type="rosbridge_websocket" output="screen">
    <param name="port" value="9090"/>
    <param name="address" value="0.0.0.0"/>
  </node>
</launch>
```

### 7.7 启动

```bash
# RK3568 上（一次性）
sudo apt install ros-melodic-rosbridge-server

# 编译
cd ~/catkin_ws && catkin_make

# 启动
roslaunch robot_controller teleop.launch
```

### 7.8 改动汇总

| 文件 | 操作 | 行数 |
|------|------|:---:|
| `TeleopHandler.h` | 新建 | 55 |
| `TeleopHandler.cpp` | 新建 | 75 |
| `ControllerNode.h` | 改 | +6 |
| `ControllerNode.cpp` | 改 | +35 |
| `CMakeLists.txt` | 改 | +1 |
| **合计** | | **~172** |

---

## 八、安全机制

### 8.1 五级防护

```
Level 1 — 初始化保护
  · last_cmd_time_ 初始化为 ros::Time(0)
  · 未收到首帧指令前，超时检测不生效
  · 避免节点一启动就"落地成盒"

Level 2 — 防跳变限幅
  · TeleopHandler 每帧检查 |target-current| ≤ 单帧阈值
  · 超限 → 丢弃该帧 + ROS_WARN
  · 阈值: 直线 10mm, 旋转 5°

Level 3 — 串口带宽保护
  · 遥操作活跃时，readPdo 自动从 100Hz 降至 20Hz
  · 释放总线带宽给 30Hz writePdo
  · 非遥操作时自动恢复 100Hz

Level 4 — 关节限位
  · App 端: 关节目标值裁剪到 [min, max]
  · 机载端: ControllerNode 已有 minPos_/maxPos_

Level 5 — 通信保护
  · /left_teleop_joints 超过 0.5s 无新消息 → 软停止
  · 软停止: 停止电机 + 锁刹车, loop 继续运行
  · 可通过 /left_reset 恢复

Level 6 — 硬件急停
  · App 急停按钮 → /left_estop → stopAll()
    - DM2C: PRO_TRIGGER = 0x0040
    - MWD485 肩部: stop() + brakeLock()
    - MWD485 伸展: stop() + brakeLock()
    - 飞特腕部: EnableTorque(id, 0)
    - running_ = false → loop 退出（不可恢复，需重启节点）
  · 电机自身过流/过热/过速硬件报警
```

### 8.2 断连与恢复流程

```
正常运行时:
  App 20~30Hz → /left_teleop_joints

网络波动 0.5s 未收到指令:
  checkTimeout() 触发 → soft_stop_fn_() → stopMotorsSoft()
  · 电机停转 + 刹车锁死
  · loop 继续运行
  · soft_stopped_ = true

App 重新连接, 点 [恢复]:
  /left_reset → resetCB()
  · soft_stopped_ = false
  · active_ = false
  · 下一帧 jointCmdCB: reset_fn_() → brakeRelease + EnableTorque
  · 恢复正常遥控

App 点 [急停]:
  /left_estop → estopCB()
  · hard_stopped_ = true
  · hard_stop_fn_() → stopAll() → running_=false
  · 必须重启节点
```

---

## 九、备选方案：TCP 笛卡尔空间遥控

以下方案经过讨论后被搁置，保留供未来参考。适用于有摄像头回传画面、或需要自动抓取的场景。

### 9.1 适用场景

- 有摄像头标定和物体位姿检测
- 操作者通过屏幕操作（非目视）
- 需要 IK 保证末端直线运动精度

### 9.2 风险与限制

| 风险 | 说明 |
|------|------|
| IK 实时收敛 | 遥操作 30Hz 流式输入，工作空间边界处 IK 可能不收敛 |
| 腕部跳跃 | 当前 `solve_wrist_from_R` 无多解比较，相邻帧可能跳变 180° |
| 目标位姿来源 | 需要相机+物体检测，或操作者手动示教 |
| 操作直觉 | 无屏幕反馈时，操作者无法感知 XYZ 坐标变化 |

---

## 十、附录：协议速查卡

```
┌──────────────────────────────────────────────────────────────────┐
│                   手机App ↔ RK3568 遥控协议                       │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  连接: ws://<RK3568_IP>:9090  (rosbridge v2.0 JSON over WebSocket)│
│                                                                  │
│  App 发送 ────────────────────────────────────────────────────── │
│  关节控制   → /left_teleop_joints  Float64MultiArray[6] 20~30Hz │
│              [升降mm, 肩部°, 伸展mm, 腕俯仰°, 腕偏航°, 腕翻滚°] │
│              范围: [0~950, 0~90, 0~360, ±180, ±90, ±180]       │
│  夹爪       → /left_gripper_cmd    String "open"/"close"        │
│  硬急停     → /left_estop          Empty                        │
│  软停恢复   → /left_reset          Empty                        │
│  订阅反馈   → {"op":"subscribe","topic":"/left_arm_joint_states"}│
│                                                                  │
│  App 接收 ←───────────────────────────────────────────────────── │
│  关节状态   ← /left_arm_joint_states Float64MultiArray[12] 10Hz │
│              [6关节位置 + 6关节速度]                              │
│                                                                  │
│  预设位姿 ────────────────────────────────────────────────────── │
│  Home:  [300, 0,   0, 0,   0, 0  ]  收拢待机                    │
│  Ready: [600, 0,   0, 0,   0, 0  ]  预抓取姿态                   │
│  Carry: [300, 63, 0, 129, 90,-90]  搬运姿态（高过障碍物）        │
│                                                                  │
│  安全 ───────────────────────────────────────────────────────────│
│  单帧跳变限幅:  直线≤10mm, 旋转≤5° (超限丢弃)                    │
│  通信超时 0.5s → 软停止 (可恢复, App 点 [恢复])                   │
│  急停按钮 → 硬急停 (不可恢复, 需重启节点)                         │
└──────────────────────────────────────────────────────────────────┘
```

---

## 十一、修订记录

| 日期 | 修订内容 |
|------|---------|
| 2026-08-06 | 初稿：两种方案对比，推荐 TCP 位姿控制 |
| 2026-08-06 | v2：确认部署环境为 RK3568 + 手机App + rosbridge |
| 2026-08-06 | v3：讨论 IK 风险、目标位姿来源、无摄像头 → 修正为关节直驱 |
| 2026-08-06 | v4：代码审查发现三个致命漏洞并修正 — 阻塞死锁、跳变保护、急停死锁 |
| 2026-08-06 | v5：追加三个进阶工程问题 — 初始化"落地成盒"、串口锁竞争、TCP帧堆积 |
