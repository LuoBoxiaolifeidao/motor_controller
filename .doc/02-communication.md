# 底层通信：串口与 Modbus RTU

## 一、串口层 — SerialPort

**文件**: `robot_controller/motors/DM2C/SerialPort.cpp`（主工程实际使用）

> 注：`dm2c-rs556-controller/SerialPort.cpp` 是独立测试工具版本，**不被主工程使用**。

### 物理串口打开

```cpp
// 使用 Linux termios API (非 O_SYNC 模式，降低CPU占用)
fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
```

### 串口配置参数

- **数据格式**: 8N1 (8位数据、无校验位、1停止位)
- **无硬件流控**: `CRTSCTS` 关闭
- **无软件流控**: `lflag=0`, `oflag=0`
- **超时设置**: `VMIN=0, VTIME=5` (0.5秒超时)
- **波特率**: DM2C 用 38400, MWD485/飞特用 115200

### 线程安全

所有公开接口都有 `std::lock_guard<std::mutex>` 保护，因为 MWD485 和飞特舵机共享同一物理串口 (`/dev/ttysWK0`)：

```cpp
// SerialPort.h 新增成员
std::mutex mutex_;
```

### 数据发送

```cpp
bool SerialPort::send(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);   // 线程安全
    tcflush(fd_, TCOFLUSH);                      // 清空输出缓冲
    ssize_t bytes_written = write(fd_, data.data(), data.size());
    if (bytes_written != data.size()) {
        tcflush(fd_, TCOFLUSH);
        return false;
    }
    tcdrain(fd_);                                // 等待发送完成
    return true;
}
```

### 数据接收 — 使用 `select()` 而非盲等

```cpp
std::vector<uint8_t> SerialPort::receive(int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);   // 线程安全
    std::vector<uint8_t> data;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) return data;                   // 超时或错误，返回空

    uint8_t buffer[256];
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        data.assign(buffer, buffer + bytes_read);
    }
    return data;
}
```

> 与测试版 (`dm2c-rs556-controller`) 的关键区别：用 `select()` 监听 fd 可读事件代替 `usleep()` 盲等，不浪费固定等待时间。

### 事务操作 `transaction()` — send + receive 原子操作

供 MWD485Controller 使用，发送后立即等待响应，整个过程上锁保证原子性：

```cpp
std::vector<uint8_t> SerialPort::transaction(const std::vector<uint8_t>& data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    tcflush(fd_, TCIOFLUSH);          // 清空双向缓冲
    write(fd_, data.data(), data.size());
    tcdrain(fd_);

    // select 等待响应 ...
    select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    read(fd_, buffer, sizeof(buffer));
    return response;
}
```

### `transactionNoFlush()` — 仅清输入缓冲的事务

与 `transaction()` 的区别：只清输入不清输出，用于快速连续通信场景（避免清掉还没发完的数据）。

### `writeRaw()` / `readBytes()` — 飞特舵机专用

```cpp
bool writeRaw(const std::vector<uint8_t>& data)   // 纯写，不 tcdrain
std::vector<uint8_t> readBytes(int n, int timeout_ms)  // 读取指定字节数
```

飞特舵机 (`SMS_STS_Shared`) 通过 `SerialPort::create()` 持有串口实例，直接调用 `writeRaw`/`readBytes` 进行私有协议通信，不走 Modbus 帧。

### 缓冲区清空

```cpp
void SerialPort::flushInput()  { std::lock_guard<std::mutex> lock(mutex_); tcflush(fd_, TCIFLUSH); }
void SerialPort::flushOutput() { std::lock_guard<std::mutex> lock(mutex_); tcflush(fd_, TCOFLUSH); }
```

### 单例模式

同名物理串口全局只创建一个 shared_ptr，防止多个对象同时操作同一串口产生冲突：

```cpp
static std::map<std::string, std::shared_ptr<SerialPort>> ports_;

std::shared_ptr<SerialPort> SerialPort::create(const std::string& name, int baudrate) {
    auto it = ports_.find(name);
    if (it != ports_.end()) return it->second;  // 已存在就直接返回
    auto port = std::shared_ptr<SerialPort>(new SerialPort(name, baudrate));
    ports_[name] = port;
    return port;
}
```

### 新旧版本对比

| 特性 | 测试版 (dm2c-rs556-controller) | ✅ 主工程 (robot_controller/motors/DM2C) |
|------|-------------------------------|----------------------------------------|
| `open()` | `O_RDWR \| O_NOCTTY \| O_SYNC` | `O_RDWR \| O_NOCTTY` |
| 线程安全 | ❌ 无锁 | ✅ `std::mutex` |
| `receive()` | `usleep()` 盲等 | `select()` 超时监听 |
| `transaction()` | ❌ | ✅ send+receive 原子操作 |
| `transactionNoFlush()` | ❌ | ✅ |
| `writeRaw()` | ❌ | ✅ 飞特舵机用 |
| `readBytes()` | ❌ | ✅ 飞特舵机用 |
| 使用者 | `dm2c_monitor` 测试工具 | `robot_controller` 主程序 |

---

## 二、Modbus RTU 帧构建 — ModbusFrameBuilder

**文件**: `robot_controller/motors/DM2C/ModbusFrameBuilder.cpp`

### Modbus RTU 帧格式

```
| 从站地址 | 功能码 | 数据... | CRC16低字节 | CRC16高字节 |
|  1 byte  | 1 byte| N bytes |  1 byte    |  1 byte     |
```

三种功能码：
- **0x06** = 写单个寄存器
- **0x03** = 读保持寄存器
- **0x10** = 批量写多个寄存器

### CRC16 校验

```cpp
uint16_t ModbusFrameBuilder::calculateCRC16(const uint8_t* data, int length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];               // XOR 当前字节
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;        // 多项式 0xA001
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

注意：CRC 附加时 **低字节在前，高字节在后** (小端序)。

### 写单个寄存器帧 (0x06)

```cpp
// 构建 8 字节帧
std::vector<uint8_t> buildWriteFrame(slave_id, 0x06, reg_addr, value)
// 帧结构: [slave_id, 0x06, reg_H, reg_L, val_H, val_L, CRC_L, CRC_H]
// 例: [0x02, 0x06, 0x62, 0x00, 0x00, 0x10, 0x??, 0x??]
//       从站2  写    模式寄存器=0x6200  启动=0x0010
```

### 读寄存器帧 (0x03)

```cpp
// 读单个: 等同于 count=1
buildReadFrame(slave_id, 0x03, reg_addr, 1)
// 帧结构: [slave_id, 0x03, reg_H, reg_L, 0x00, count, CRC_L, CRC_H]
```

### 批量写多个寄存器帧 (0x10)

```cpp
// 一次写入6个连续寄存器 (从 start_reg 开始)
buildMultiWriteFrame(slave_id, 0x10, start_reg, {val0, val1, ..., val5})
// 帧结构:
// [slave, 0x10, reg_H, reg_L, count_H, count_L, byte_count,
//  val0_H, val0_L, val1_H, val1_L, ..., CRC_L, CRC_H]
//
// 优势: 6次单独写入 = 6 × 50ms = 300ms
//       1次批量写入 = 20ms (快15倍)
```

---

## 三、DM2C 寄存器映射

**文件**: `robot_controller/motors/DM2C/DM2CController.h`

### 控制寄存器

| 寄存器地址 | 名称 | 说明 |
|-----------|------|------|
| 0x6200 | PRO_MODE | 运行模式: 0x0001=绝对位置, 0x0041=相对位置, 0x0002=速度 |
| 0x6201 | PRO_POSITION_HIGH | 目标位置高16位 |
| 0x6202 | PRO_POSITION_LOW | 目标位置低16位 |
| 0x6203 | PRO_SPEED | 目标速度 (RPM) |
| 0x6204 | PRO_ACCEL | 加速度 |
| 0x6205 | PRO_DECEL | 减速度 |
| 0x6002 | PRO_TRIGGER | 触发: 0x0010=启动, 0x0040=停止, 0x0021=设零 |

### 状态寄存器

| 寄存器地址 | 名称 | 说明 |
|-----------|------|------|
| 0x1003 | STATUS | 运行状态 |
| 0x2203 | ALARM | 报警状态 |
| 0x602A-0x602B | CMDPOS | 命令位置 (32位) |
| 0x602C-0x602D | CURPOS | 实际位置 (32位) |
| 0x1045 | CURSPEED | 实际速度 |

### 读取响应帧解析

```cpp
// 读单个寄存器响应: [slave, 0x03, byte_count=2, val_H, val_L, CRC_L, CRC_H]
uint16_t DM2CController::readRegister(uint16_t reg_addr) {
    auto response = serial_->receive(200);  // 200ms超时
    if (response.size() >= 7 &&
        response[0] == slave_id &&           // 地址匹配
        response[1] == 0x03) {               // 功能码匹配
        // 验证 CRC
        uint16_t crc = calculateCRC16(response.data(), response.size() - 2);
        uint16_t received_crc = (response[response.size()-1] << 8) | response[response.size()-2];
        if (crc == received_crc)
            return (response[3] << 8) | response[4];  // 大端序拼接
    }
    return 0xFFFF;  // 读取失败标志
}
```

---

## 四、DM2CController 操作模式

每次操作都是"写寄存器 + 等待 + 清缓冲"的同步模式：

```
send(frame) → sleep(50ms) → flushInput()
```

### 读寄存器流程

```
flushInput() → send(read_frame) → receive(200ms) → 校验CRC → 返回数据
```

注意：**读操作前先 flushInput**（清掉缓冲区中的脏数据），**写操作后 flushInput**（清掉电机的回显/应答）。

---

## 五、MWD485 私有协议 (0x3E 帧头)

**文件**: `robot_controller/motors/MWD/MWD485Controller.cpp`

> 重要：MWD485 肩部/伸展电机**不使用 Modbus RTU**，而是帧头 `0x3E` 的私有协议。真正使用 Modbus RTU 的只有 DM2C 升降柱。

### 帧格式

```
| 帧头  | 命令码 | 电机ID | 长度 | 头校验和 | 数据... | 数据校验和 |
| 0x3E  | 1 byte | 1 byte |1 byte| 1 byte  | N bytes | 1 byte    |
```

- **帧头**: 固定 `0x3E`
- **命令码**: 0x80~0xC1
- **头校验和**: `(帧头 + 命令码 + 电机ID + 长度) & 0xFF`
- **数据校验和**: 数据所有字节求和 `& 0xFF`（无数据时省略）

### 常用命令码

| 命令码 | 名称 | 说明 |
|--------|------|------|
| 0x88 | CMD_ENABLE | 使能电机 |
| 0x80 | CMD_DISABLE | 失能 |
| 0x81 | CMD_STOP | 停止 |
| 0x8C | CMD_BRAKE | 刹车控制 (数据: 0x01释放 / 0x00锁死) |
| 0xA4 | CMD_POS_MULTI2 | 多圈绝对位置 (8字节角度 + 4字节速度) |
| 0xA8 | CMD_POS_INCR2 | 增量位置 (4字节增量 + 4字节速度) |
| 0x92 | CMD_READ_ANGLE | 读角度 |
| 0x9A | CMD_READ_STATE1 | 读状态 (温度/电压/电流/错误) |
| 0x19 | CMD_SET_ZERO | 设零 |

### 多圈位置控制示例 (肩部转到 32°)

```cpp
// multiTurnPosition(angle_centideg, maxSpeed_dps)
// angle = 32° × 10(减速比) × 100 = 32000 = 0x7D00
// 帧: [0x3E, 0xA4, 0x07, 0x0C, 头校验和,
//       角度8字节小端, 速度4字节小端, 数据校验和]
```

### 与 Modbus RTU 的关键区别

| | Modbus RTU (DM2C) | 0x3E 私有 (MWD485) |
|--|-------------------|--------------------|
| 帧头 | 无（从站地址开头） | `0x3E` |
| 校验 | CRC16（多项式 0xA001） | 简单字节求和 |
| 写位置 | 两步：填寄存器 + 触发 | 一步：直接发位置帧 |
| 寻址 | 从站地址 1 字节 | 电机 ID 1 字节 |

---

## 六、飞特舵机通信 (SMS_STS)

飞特舵机使用 **私有同步读写协议**，不走 Modbus：

- **同步批量读取**: `syncReadPacketTx(ids, count, start_addr, len)` → `syncReadPacketRx(id, rxBuf)`
- **同步批量写入**: `SyncWritePosEx(ids, count, positions, speeds, accels)`
- **单舵机写入**: `WritePosEx(id, position, speed, accel)`
- **编码器**: 4096 tick = 360°

飞特舵机和 MWD485 **共享同一个物理串口** (`/dev/ttysWK0`)，通过 `ft_mutex_` 互斥锁保护并发访问。

---

## 七、多线程模型与锁机制

### 线程架构总览

系统共有 **4 个线程** 参与串口通信：

```
main() [线程1: 主线程]
  ├─ ControllerNode() 构造
  │    └─ loop_thread_ = std::thread(...)  [线程2: 100Hz 状态读取]
  │
  ├─ ros::AsyncSpinner(2)  [线程3, 线程4: ROS 回调]
  │    ├─ ArmJointsCB()      → driveJoints() → writePdo()
  │    ├─ TcpCB()            → driveJoints() → writePdo()
  │    ├─ executeMoveArm()   → driveJoints() → writePdo()
  │    └─ executeGripper()   → FT 写操作
  │
  └─ ros::waitForShutdown()
```

| 线程 | 创建方式 | 频率 | 职责 |
|------|---------|------|------|
| 主线程 | `main()` | — | ROS spin、构造、析构 |
| `loop_thread_` | `std::thread` | 100 Hz | 持续读取所有电机状态 (`readPdo`) |
| AsyncSpinner #1 | ROS spinner | 事件驱动 | 处理 actionlib 回调 (`executeMoveArm`) |
| AsyncSpinner #2 | ROS spinner | 事件驱动 | 处理 topic 回调 (`ArmJointsCB`, `TcpCB`) |

### 串口共享关系

```
┌──────────────────────────────────────────────────────────────┐
│                     ControllerNode                            │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  loop_thread_ (100Hz)                ROS 回调线程             │
│  ┌──────────────────┐          ┌──────────────────────┐      │
│  │ readPdo() 循环    │          │ driveJoints()         │      │
│  │  ├─ DM2C (读)    │          │  └─ writePdo()        │      │
│  │  ├─ MWD_Shoulder │          │      ├─ DM2C (写)    │      │
│  │  ├─ MWD_Extend   │          │      ├─ MWD_Shoulder  │      │
│  │  └─ FT (读)      │          │      ├─ MWD_Extend    │      │
│  └──────────────────┘          │      └─ FT (写)      │      │
│                                └──────────────────────┘      │
│                                      ↓                       │
│                              SerialPort (单例 + mutex)        │
│                                      ↓                       │
│              ┌──────────────────────┴──────────────────┐     │
│          /dev/ttyACM0                          /dev/ttysWK0 │
│          (仅 DM2C 升降柱)               (MWD肩+MWD伸+飞特3+夹)│
└──────────────────────────────────────────────────────────────┘
```

| 串口 | 波特率 | 设备 | 使用方 |
|------|--------|------|--------|
| `/dev/ttyACM0` | 38400 | DM2C 升降柱 | `jointDM2C_` (独占，无竞争) |
| `/dev/ttysWK0` | 115200 | MWD485 肩部 | `jointShoulder_` |
| `/dev/ttysWK0` | 115200 | MWD485 伸展 | `jointExtend_` |
| `/dev/ttysWK0` | 115200 | 飞特腕部 ×3 | `jointFT_` |
| `/dev/ttysWK0` | 115200 | 飞特夹爪 | `jointFT_` |

### 三层锁保护

```
// 以 /dev/ttysWK0 上一次 MWD 状态读取为例的完整锁路径：

readPdo(Motor::MWD_Shoulder)
  │
  ├─ ft_mutex_.lock()              // 第1层: ControllerNode 层
  │    │                            //   串行化 MWD+FT 多帧操作
  │    │
  │    └─ jointShoulder_->refreshState()
  │         │
  │         └─ serial_->transaction(frame, timeout)
  │               │
  │               ├─ SerialPort::mutex_.lock()   // 第2层: I/O 层
  │               ├─ tcflush(TCIOFLUSH)          //   保护 fd 单次收发
  │               ├─ write() + tcdrain()
  │               ├─ select() + read()
  │               └─ SerialPort::mutex_.unlock()
  │
  └─ ft_mutex_.unlock()

// 同时另一个线程的 writePdo 被 ft_mutex_ 挡在门外
// SerialPort 单例 (第0层) 确保同一端口名只有一个 fd 实例
```

| 层级 | 机制 | 作用范围 | 防什么 |
|------|------|---------|--------|
| 第0层 | `SerialPort::create()` 单例 | 全进程，按端口名去重 | 同一串口被多次 open |
| 第1层 | `SerialPort::mutex_` | 单个 `transaction()`/`send()`/`receive()` 调用 | 单次收发帧数据不被交织 |
| 第2层 | `ft_mutex_` / `dm2c_mutex_` | ControllerNode 层 | MWD+FT 或 DM2C 的多帧操作不被打断 |

### 线程间数据流

```
loop_thread_ [线程2, 100 Hz]:
  while(running_) {
      lock(curState_mutex_)
        readPdo(DM2C)         // SerialPort 事务 (dm2c_cfg_.port)
        readPdo(MWD_Shoulder) // lock(ft_mutex_) + 3次 SerialPort 事务
        readPdo(MWD_Extend)   // lock(ft_mutex_) + 3次 SerialPort 事务
        readPdo(FT)           // lock(ft_mutex_) + syncReadPacketTx/Rx
      unlock(curState_mutex_)
      publishState()          // 每 100ms 一次 (每10次循环)
  }

driveJoints() [线程3 或 线程4]:
  writePdo(所有电机)           // lock(ft_mutex_) 用于 MWD485+FT
  while 未到达目标:
      lock(curState_mutex_)   // 拿 curState_ 快照
      snapshot = curState_
      unlock(curState_mutex_)
      检查死区
      sleep(10ms)
```

**关键设计**：loop 线程负责写入 `curState_`，运动等待循环只读快照。两个方向不会同时对 `curState_` 做写操作。

### 已知问题

#### 1. `curState_mutex_` 持有期间做阻塞 I/O

```cpp
// ControllerNode::loop() — 问题所在
while (ros::ok() && running_) {
    std::lock_guard<std::mutex> lock(curState_mutex_);  // 拿锁
    readPdo(Motor::DM2C);         // 可能阻塞 ~150ms (3次重试)
    readPdo(Motor::MWD_Shoulder); // 可能阻塞 ~200ms
    readPdo(Motor::MWD_Extend);   // 可能阻塞 ~200ms
    readPdo(Motor::FT);           // 可能阻塞 ~200ms
}   // 释放锁 — 最坏情况持有锁 ~750ms
```

**影响**：当通信失败触发重试时，运动等待循环 (`driveJoints`) 可能长达 750ms 读不到状态快照，影响到达判断的实时性。

#### 2. MWD485Controller 的 `tx_mutex_` 未使用

```cpp
// MWD485Controller.h:71 — 已声明但 cpp 中从未 lock
std::mutex tx_mutex_;  // 预留字段，当前未启用
```

当前靠 ControllerNode 层的 `ft_mutex_` + SerialPort 层的 `mutex_` 双层保护。如果 MWD485Controller 被独立使用（如在 `MWD485Server` 测试工具中），它自身不提供线程安全保护。

#### 3. `ft_mutex_` 的串行化代价

`/dev/ttysWK0` 上挂载了 6 个设备，`ft_mutex_` 保证它们不打架，但也意味着每次 `readPdo` 循环中 MWD 肩部、MWD 伸展、飞特腕部必须**串行读取**。如果其中一个设备响应慢（如飞特 `syncReadPacketTx` 超时），会拖慢整个 100Hz 循环的节奏。

#### 4. `refreshState()` 的多事务非原子性

```cpp
void MWD485Controller::refreshState() {
    auto data1 = sendAndRecv(CMD_READ_STATE1, ...);  // 事务1
    auto data2 = sendAndRecv(CMD_READ_STATE2, ...);  // 事务2
    auto dataAngle = sendAndRecv(CMD_READ_ANGLE, ...); // 事务3
    // 三次事务之间有间隙，状态可能不一致
}
```

三次独立的事务不是原子的。在高速运动中，`cacheAngle_` 和 `cacheSpeed_` 可能来自不同的时刻，存在微小的时间偏差。当前通过 `ft_mutex_` 保证不被写操作打断（读-改-写一致性），但不保证读-读之间的瞬时一致性。
