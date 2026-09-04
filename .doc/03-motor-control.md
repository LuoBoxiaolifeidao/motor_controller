# 电机控制详解

## 一、升降柱 (Joint::Lift) — DM2C RS556

### 向上/向下运动

文件: `robot_controller/src/ControllerNode.cpp`

#### 位置读取 (`readPdo(DM2C)`, 第516行)

```cpp
case Motor::DM2C: {
    uint32_t enc = jointDM2C_->readActualPosition();   // 读编码器值
    double speed = std::abs(jointDM2C_->readActualSpeed()); // 读速度

    // 计算位置增量: 编码器差值 × 丝杠导程系数
    double delta = encoderDelta(enc, dm2c_value_.second) * DM2CTRAC;  // DM2CTRAC = 0.0014 mm/tick
    dm2c_value_.first -= delta;   // 累积位置 (负号: 编码器增加=下降)
    dm2c_value_.second = enc;     // 保存本次编码器值供下次差分

    curState_[Joint::Lift].position = dm2c_value_.first;  // 更新当前关节位置(mm)
    curState_[Joint::Lift].velocity = speed;
}
```

编码器增量处理 (第74行):
```cpp
static int32_t encoderDelta(uint32_t current, uint32_t last) {
    uint32_t raw = current - last;
    if (raw < 0x80000000)
        return static_cast<int32_t>(raw);           // 正向
    return static_cast<int32_t>(raw) - 0x100000000; // 负向 (处理32位回绕)
}
```

#### 位置写入 (`writePdo(DM2C)`, 第573行)

```cpp
case Motor::DM2C: {
    // 计算位置误差 (mm)
    double err = targetPos_[Joint::Lift] - curState_[Joint::Lift].position;

    // 转换为编码器增量 → 相对位置模式
    jointDM2C_->moveToRelativePosition(
        static_cast<int32_t>(-err / DM2CTRAC),      // 编码器增量
        static_cast<int16_t>(dm2c_cfg_.max_speed),   // 最大速度 RPM
        static_cast<uint16_t>(accel),                 // 加速度
        static_cast<uint16_t>(accel));                // 减速度
}
```

**核心逻辑**: 升降柱是"差多少走多少"的 **位置闭环**。
- **向上**: target > cur → err > 0 → `-err/DM2CTRAC` 为负 → 电机负向旋转(上升)
- **向下**: target < cur → err < 0 → `-err/DM2CTRAC` 为正 → 电机正向旋转(下降)

#### DM2C 三种控制模式

| 模式 | 寄存器值 (0x6200) | 调用方法 |
|------|-------------------|---------|
| 绝对位置 | 0x0001 | `moveToAbsolutePosition(pos, ...)` |
| 相对位置 | 0x0041 | `moveToRelativePosition(dist, ...)` |
| 速度模式 | 0x0002 | `startVelocityMode(speed, ...)` |

每种模式都有单寄存器版本和批量多寄存器版本 (Batch)，批量版更快。

---

## 二、肩部 (Joint::Shoulder) — MWD485

### 向前/向后摆动

```cpp
case Motor::MWD_Shoulder: {
    // 目标角度(°) × 减速比(10:1) × 100(精度0.01°)
    int32_t target = static_cast<int32_t>(
        targetPos_[Joint::Shoulder] * shoulder_cfg_.gear_ratio * 100.0);
    uint16_t speed = static_cast<uint16_t>(
        shoulder_cfg_.max_speed * shoulder_cfg_.gear_ratio);
    jointShoulder_->multiTurnPosition(target, speed);  // 多圈绝对位置控制
}
```

### 刹车控制

```cpp
// 运动前
jointShoulder_->brakeRelease();  // 释放刹车
// 到位后
jointShoulder_->brakeLock();     // 锁住 (防止重力下坠)
```

---

## 三、伸展关节 (Joint::Extend) — MWD485

### 伸出/收回

```cpp
case Motor::MWD_Extend: {
    // 负号: 电机正方向与关节正方向相反
    // 1.48: 伸展机构的机械传动系数
    int32_t target = static_cast<int32_t>(
        -targetPos_[Joint::Extend] * extend_cfg_.gear_ratio * 100.0 * 1.48f);
    uint16_t speed = static_cast<uint16_t>(
        extend_cfg_.max_speed * extend_cfg_.gear_ratio);
    jointExtend_->multiTurnPosition(target, speed);
}
```

---

## 四、腕部 (Joint::Wrist1/2/3) — 飞特 STS 舵机

### 俯仰/偏航/翻滚

```cpp
case Motor::FT: {
    // 三合一写: 3个腕部舵机同步写入
    for (int k = 0; k < 3; ++k) {
        ids[k] = ft_cfg_.ids[k];                                    // {10, 11, 12}

        // URDF角度 → 电机角度 (零点偏移 + 方向反转)
        double motor_deg = ftUrdfToMotor(targetPos_[Joint::Wrist1 + k]);

        // 角度 → 舵机编码器值 (4096 = 360°)
        pos[k] = static_cast<s16>(motor_deg * FT_TICK_PER_DEG);     // 4096/360 ≈ 11.38

        // 速度换算: 1 °/s ≈ 11.8 编码值
        spd[k] = dpsToRawSpeed(ft_cfg_.max_speed[k]);

        // 加速度换算: 1 DPS² ≈ 8.7 编码值
        acc[k] = dps2ToRawAccel(ft_cfg_.max_accel[k]);
    }
    jointFT_->SyncWritePosEx(ids, 3, pos, spd, acc);  // 同步写入3个舵机
}
```

### 飞特舵机零点标定

```cpp
// 飞特舵机零点标定在 URDF 0° 对应的电机 180° 位置
// 且转动方向与 URDF 相反
static constexpr double FT_ZERO_OFFSET_DEG = 180.0;

// 电机编码器值 → URDF角度
static inline double ftMotorToUrdf(double motor_deg) {
    return FT_ZERO_OFFSET_DEG - motor_deg;  // 180° - 电机角度
}

// URDF角度 → 电机编码器值
static inline double ftUrdfToMotor(double urdf_deg) {
    return FT_ZERO_OFFSET_DEG - urdf_deg;  // 180° - URDF角度
}
```

### 飞特舵机状态读取

```cpp
// 同步批量读取 4 个舵机 (3个腕部 + 1个夹爪)
jointFT_->syncReadPacketTx(ids, 4, SMS_STS_PRESENT_POSITION_L, 4);
for (int k = 0; k < 3; ++k) {
    jointFT_->syncReadPacketRx(ids[k], rxBuf);
    curState_[Joint::Wrist1 + k].position =
        ftMotorToUrdf(jointFT_->syncReadRxPacketToWrod(15) * FT_DEG_PER_TICK);
    curState_[Joint::Wrist1 + k].velocity =
        rawSpeedToDps(jointFT_->syncReadRxPacketToWrod(15));
}
// 夹爪位置也在同一批里读
jointFT_->syncReadPacketRx(gripper_id, rxBuf);
gripper_position_ = jointFT_->syncReadRxPacketToWrod(15);
```

---

## 五、驱动关节的公共流程 (`driveJoints`)

```cpp
void ControllerNode::driveJoints(const std::vector<double>& target) {
    // 1. 存目标
    finalTarget_[i] = target[i];
    targetPos_[i]   = finalTarget_[i];

    // 2. 释放刹车
    jointShoulder_->brakeRelease();
    jointExtend_->brakeRelease();

    // 3. 一次性写入所有电机
    writePdo(Motor::DM2C);
    writePdo(Motor::MWD_Shoulder);
    writePdo(Motor::MWD_Extend);
    writePdo(Motor::FT);

    // 4. 轮询等待所有关节到位 (100Hz)
    while (ros::ok() && running_) {
        // 每个关节检查: |目标 - 实际| < dead_zone
        for (int i = 0; i < kJointNum; ++i) {
            if (std::abs(finalTarget_[i] - curState_[i].position) > dz[i])
                all_in_zone = false;  // 还有关节没到位
        }
        if (all_in_zone) break;  // 全部到位
        rate.sleep();
    }

    // 5. 到位后上锁
    jointShoulder_->brakeLock();
    jointExtend_->brakeLock();
}
```

### 死区参数 (dead_zone)

| 关节 | dead_zone | 含义 |
|------|-----------|------|
| 升降柱 | 3.0 mm | 误差 < 3mm 即认为到位 |
| 肩部 | 3.0° | 误差 < 3° 即认为到位 |
| 伸展 | 2.0° | 误差 < 2° 即认为到位 |
| 腕部1/2/3 | 各 3° | 误差 < 3° 即认为到位 |
| 夹爪 | 40 tick (~3.5°) | 误差 < 40 编码器值即认为到位 |

---

## 六、紧急停止

```cpp
void ControllerNode::stopAll() {
    running_ = false;

    jointDM2C_->stopAllMotion();       // 写 PRO_TRIGGER = 0x0040
    jointShoulder_->stop();            // MWD485 停止
    jointShoulder_->brakeLock();       // 肩部上锁
    jointExtend_->stop();             // MWD485 停止
    jointExtend_->brakeLock();        // 伸展上锁
    for (int id : ft_cfg_.ids)
        jointFT_->EnableTorque(id, 0); // 腕部失能 (卸力)
    cv_.notify_all();
}
```

---

## 七、电机配置汇总

文件: `robot_controller/config/motors.yaml`

| 参数 | DM2C (升降) | MWD485 (肩) | MWD485 (伸) | 飞特 (腕) |
|------|------------|-------------|-------------|-----------|
| ID | 2 | 7 | 8 | 10,11,12 |
| 串口 | /dev/ttyACM0 | /dev/ttysWK0 | /dev/ttysWK0 | /dev/ttysWK0 |
| 波特率 | 38400 | 115200 | 115200 | 115200 |
| 减速比 | — | 10:1 | 6:1 | — |
| 最大速度 | 60 RPM | 8 °/s | 40 °/s | 20 °/s |
| 最大加速度 | 200 | 150 | 70 | 20 |
| 死区 | 3.0 mm | 3.0° | 2.0° | 3° |
