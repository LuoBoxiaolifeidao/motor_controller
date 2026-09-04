#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <map>
#include <memory>

#include "SerialPort.h"
#include "ModbusFrameBuilder.h"

namespace DM2C_Registers {
    // 控制模式值
    static constexpr uint16_t MODE_ABSOLUTE_POSITION = 0x0001;  // 绝对位置模式
    static constexpr uint16_t MODE_VELOCITY          = 0x0002;  // 速度模式
    static constexpr uint16_t MODE_RELATIVE_POSITION = 0x0041;  // 相对位置模式
    
    // 控制寄存器
    static constexpr uint16_t PRO_MODE              = 0x6200;  // 运行模式寄存器
    static constexpr uint16_t PRO_POSITION_HIGH     = 0x6201;  // 位置高位
    static constexpr uint16_t PRO_POSITION_LOW      = 0x6202;  // 位置低位
    static constexpr uint16_t PRO_SPEED             = 0x6203;  // 速度
    static constexpr uint16_t PRO_ACCEL             = 0x6204;  // 加速度
    static constexpr uint16_t PRO_DECEL             = 0x6205;  // 减速度
    static constexpr uint16_t PRO_TRIGGER           = 0x6002;  // 触发寄存器
    
    // 触发值
    static constexpr uint16_t TRIGGER_START         = 0x0010;  // 启动运行
    static constexpr uint16_t TRIGGER_STOP          = 0x0040;  // 停止运行
    
    // 状态寄存器
    static constexpr uint16_t STATUS                = 0x1003;  // 运行状态
    static constexpr uint16_t ALARM                 = 0x2203;  // 报警状态
    static constexpr uint16_t CMDPOS_H              = 0x602A;  // 命令位移高位
    static constexpr uint16_t CMDPOS_L              = 0x602B;  // 命令位移低位
    static constexpr uint16_t CURPOS_H              = 0x602C;  // 实际位移高位
    static constexpr uint16_t CURPOS_L              = 0x602D;  // 实际位移低位
    static constexpr uint16_t CURSPEED              = 0x1045;  // 实际速度

    // 手动设零
    static constexpr uint16_t SETZERO               = 0x0021;  // 手动设零
};

enum LiftColumn {
    RightArm = 1,
    LeftArm
};

class DM2CController {
private:
    std::shared_ptr<SerialPort> serial_;
    LiftColumn side_;
    
    // 基础控制方法
    void setControlMode(uint16_t mode);
    void setTargetPosition(int32_t position);
    void setSpeed(int16_t speed_rpm);
    void setAcceleration(uint16_t accel);
    void setDeceleration(uint16_t decel);
    void triggerMotion(uint16_t trigger_value);
    
public:
    DM2CController(LiftColumn side, const std::string& port_name, int baudrate);
    DM2CController(const DM2CController&) = default;
    DM2CController& operator=(const DM2CController&) = default;
    DM2CController(DM2CController&&) noexcept = default;
    DM2CController& operator=(DM2CController&&) noexcept = default;
    
    // 绝对位置控制 单寄存器版本
    void moveToAbsolutePosition(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration);
    
    // 相对位置控制
    void moveToRelativePosition(int32_t relative_distance, int16_t speed, uint16_t acceleration, uint16_t deceleration);
    
    // 速度控制
    void startVelocityMode(int16_t target_speed, uint16_t acceleration, uint16_t deceleration);

    // 绝对位置控制 多寄存器版本
    void moveToAbsolutePositionBatch(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration);

    // 相对位置控制 多寄存器版本
    void moveToRelativePositionBatch(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration);

    // 速度控制 多寄存器版本
    void startVelocityModeBatch(int16_t speed, uint16_t acceleration, uint16_t deceleration);
    
    // 停止控制
    void stopAllMotion();
    
    // 状态读取
    uint16_t readRegister(uint16_t reg_addr);
    std::vector<uint8_t> readRegisters(uint16_t reg_addr, int count);
    uint16_t readStatus();
    uint16_t readAlarm();
    uint32_t readCommandPosition();
    uint32_t readActualPosition();
    uint16_t readActualSpeed();

    // 设零操作
    void setCurrentPositionAsZero();

};
