#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include "CANComMWD.h"

namespace MWD_CMD {
    static constexpr uint8_t DISABLE    = 0x80;
    static constexpr uint8_t STOP       = 0x81;
    static constexpr uint8_t ENABLE     = 0x88;
    static constexpr uint8_t READ_ANGLE = 0x92;
    static constexpr uint8_t READ_STATE1= 0x9A;
    static constexpr uint8_t READ_STATE2= 0x9C;
    static constexpr uint8_t READ_ACCEL = 0x33;
    static constexpr uint8_t WRITE_ACCEL= 0x34;
    static constexpr uint8_t POS_MULTI2 = 0xA4;
    static constexpr uint8_t BRAKE_CTRL = 0x8C;
};

class MWDController {
public:
    MWDController(int motor_id, const std::string& can_iface, uint32_t bitrate = 1000000);
    ~MWDController();

    MWDController(const MWDController&) = delete;
    MWDController& operator=(const MWDController&) = delete;

    void enable();
    void disable();
    void stop();

    void brakeRelease();   // 通电释放刹车
    void brakeLock();      // 断电启动刹车
    uint8_t getBrakeState(); // 读取抱闸器状态 (0x00=刹车启动, 0x01=刹车释放)

    void setAcceleration(int32_t accel_dps2);
    void multiTurnPosition(int32_t angle_mdeg, uint16_t maxSpeed_dps);

    void clearError();
    void setZeroToROM();

    int16_t getSpeed();
    uint16_t getEncoder();
    int64_t getMotorAngle();
    int8_t getTemperature();
    uint16_t getVoltage();
    uint16_t getBusCurrent();
    uint8_t getMotorState();
    uint8_t getErrorState();
    int32_t getAcceleration();

private:
    int motor_id_;
    uint32_t can_id_;
    std::shared_ptr<CANComMWD> can_;

    void pollAngle();
    void pollState1();
    void pollState2();
    void pollAccel();

    mutable std::mutex state_mutex_;
    int16_t  speed_        = 0;
    uint16_t encoder_      = 0;
    int64_t  motorAngle_   = 0;
    int8_t   temperature_  = 0;
    uint16_t voltage_      = 0;
    uint16_t busCurrent_   = 0;
    uint8_t  motorState_   = 0;
    uint8_t  errorState_   = 0;
    int32_t  acceleration_ = 0;
};
