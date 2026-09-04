# 夹爪控制

## 夹爪参数配置

文件: `robot_controller/config/motors.yaml`

```yaml
left_gripper:
  ids: 23            # 飞特舵机 ID
  port: "/dev/ttysWK0"
  baudrate: 115200
  open: 1748         # 张开位置 (编码器值)
  close: 900         # 闭合位置 (编码器值)
  dead_zone: 40      # 到位容忍区间 (编码器值)
```

- **张开**: 舵机转到编码器位置 1748 (约 153°)
- **闭合**: 舵机转到编码器位置 900 (约 79°)
- **行程**: 1748 - 900 = 848 tick ≈ 74.5°

## 控制流程

文件: `robot_controller/src/ControllerNode.cpp` 第751行

### 接口

ROS Action: `/left_gripper/gripper_cmd` (GripperCommandAction)
- `goal.command = "open"` → 张开
- `goal.command = "close"` → 闭合

### 执行流程

```cpp
void ControllerNode::executeGripper(const GripperGoalConstPtr& goal) {
    // 1. 根据命令确定目标位置
    int target;
    if (goal->command == "open") {
        target = gripper_cfg_.open;   // 1748
    } else if (goal->command == "close") {
        target = gripper_cfg_.close;  // 900
    } else {
        return;  // 未知命令
    }

    // 2. 使能扭矩 + 发送位置指令
    {
        std::lock_guard<std::mutex> lock(ft_mutex_);  // 飞特总线互斥锁
        jointFT_->EnableTorque(gripper_cfg_.id, 1);    // 使能 (否则舵机无力)
        jointFT_->WritePosEx(gripper_cfg_.id,
                             static_cast<s16>(target),  // 目标位置
                             100,                        // 速度 (较低)
                             100);                       // 加速度
    }

    // 3. 轮询等待到位 (20Hz)
    ros::Rate rate(20);
    while (ros::ok()) {
        int pos = gripper_position_;                      // 100Hz主循环更新的当前位置
        if (std::abs(pos - target) <= gripper_cfg_.dead_zone) {  // dead_zone = 40
            break;  // 到位
        }
        feedback.current_state = goal->command + "_ing";  // "open_ing" / "close_ing"
        gripper_as_->publishFeedback(feedback);
        rate.sleep();
    }
}
```

### 夹爪位置读取

在 `readPdo(FT)` 中与腕部舵机一起批量读取 (第548-563行):

```cpp
// 同步批量读取: 3个腕部 + 1个夹爪 = 4个舵机
u8 ids[4];
for (int k = 0; k < 3; ++k) ids[k] = ft_cfg_.ids[k];  // 10, 11, 12
ids[3] = static_cast<u8>(gripper_cfg_.id);              // 23

jointFT_->syncReadPacketTx(ids, 4, SMS_STS_PRESENT_POSITION_L, 4);
// ... 读腕部 ...
if (jointFT_->syncReadPacketRx(ids[3], rxBuf) == 4) {
    gripper_position_ = jointFT_->syncReadRxPacketToWrod(15);
}
```

---

## 关键注意事项

1. **扭矩必须使能**: `EnableTorque(id, 1)` 必须在 `WritePosEx` 之前调用，否则舵机不产生力矩
2. **飞特总线互斥**: 夹爪和腕部共享 `/dev/ttysWK0` 总线，所有操作必须持有 `ft_mutex_`
3. **低速度**: 夹爪使用速度 100 (远低于腕部的 ~236 编码值)，以避免夹坏物体
4. **无刹车**: 飞特舵机没有物理刹车，依靠内部电磁力矩保持位置
5. **轮询频率**: 20Hz (50ms间隔)，因为夹爪动作本身较快，不需要 100Hz
