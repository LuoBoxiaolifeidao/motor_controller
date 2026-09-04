#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>

#include "SerialPort.h"
#include "ModbusFrameBuilder.h"

namespace DM2C_Registers {
    static constexpr uint16_t MODE_ABSOLUTE_POSITION = 0x0001;
    static constexpr uint16_t MODE_VELOCITY          = 0x0002;
    static constexpr uint16_t MODE_RELATIVE_POSITION = 0x0041;
    
    static constexpr uint16_t PRO_MODE              = 0x6200;
    static constexpr uint16_t PRO_POSITION_HIGH     = 0x6201;
    static constexpr uint16_t PRO_POSITION_LOW      = 0x6202;
    static constexpr uint16_t PRO_SPEED             = 0x6203;
    static constexpr uint16_t PRO_ACCEL             = 0x6204;
    static constexpr uint16_t PRO_DECEL             = 0x6205;

    static constexpr uint16_t PR1_MODE              = 0x6208;
    static constexpr uint16_t PR1_POSITION_HIGH     = 0x6209;
    static constexpr uint16_t PR1_POSITION_LOW      = 0x620A;
    static constexpr uint16_t PR1_SPEED             = 0x620B;
    static constexpr uint16_t PR1_ACCEL             = 0x620C;
    static constexpr uint16_t PR1_DECEL             = 0x620D;

    static constexpr uint16_t PRO_TRIGGER           = 0x6002;
    
    static constexpr uint16_t TRIGGER_START0        = 0x0010;
    static constexpr uint16_t TRIGGER_START1        = 0x0011;
    static constexpr uint16_t TRIGGER_STOP          = 0x0040;
    
    static constexpr uint16_t STATUS                = 0x1003;
    static constexpr uint16_t ALARM                 = 0x2203;
    static constexpr uint16_t CMDPOS_H              = 0x602A;
    static constexpr uint16_t CMDPOS_L              = 0x602B;
    static constexpr uint16_t CURPOS_H              = 0x602C;
    static constexpr uint16_t CURPOS_L              = 0x602D;
    static constexpr uint16_t CURSPEED              = 0x1045;

    static constexpr uint16_t SETZERO               = 0x0021;

    static constexpr uint16_t EEPROMReg             = 0x1801;
    static constexpr uint16_t SAVEEPROM             = 0x2211;
};

enum LiftColumn {
    RightArm = 1,
    LeftArm
};

class DM2CController {
private:
    std::shared_ptr<SerialPort> serial_;
    LiftColumn side_;
    int curRPIndex_ = 0;

    void triggerMotion(uint16_t trigger_value);

    bool sendWithRetry(const std::vector<uint8_t>& frame);

    uint16_t readRegister(uint16_t reg_addr);
    std::vector<uint8_t> readRegisters(uint16_t reg_addr, int count);

    uint32_t lastPosition_ = 0;

    static std::vector<DM2CController*> instances_;
    static std::mutex instances_mutex_;
    static std::atomic<int> instanceCount_;

public:
    DM2CController(LiftColumn side, const std::string& port_name, int baudrate);
    ~DM2CController();

    DM2CController(const DM2CController&) = delete;
    DM2CController& operator=(const DM2CController&) = delete;
    DM2CController(DM2CController&&) noexcept = default;
    DM2CController& operator=(DM2CController&&) noexcept = default;

    void moveToAbsolutePosition(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration);
    void moveToRelativePosition(int32_t relative_distance, int16_t speed, uint16_t acceleration, uint16_t deceleration);
    void startVelocityMode(int16_t target_speed, uint16_t acceleration, uint16_t deceleration);
    void stopAllMotion();

    uint32_t readActualPosition();
    int16_t  readActualSpeed();
    uint16_t readStatus();
    uint16_t readAlarm();
    uint32_t readCMDPosition();

    void setCurrentPositionAsZero();
    void saveParametersToEEPROM();
};
