#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include "../DM2C/SerialPort.h"

class MWD485Controller {
public:
    explicit MWD485Controller(int motor_id, const std::string& port, int baudrate = 115200);
    ~MWD485Controller();

    MWD485Controller(const MWD485Controller&) = delete;
    MWD485Controller& operator=(const MWD485Controller&) = delete;

    // motor control
    void enable();
    void disable();
    void stop();
    void brakeRelease();
    void brakeLock();
    int  getBrakeState();
    void clearError();
    void setAcceleration(int32_t accel_dps2);
    void setZeroToROM();

    // multi-turn position control
    void multiTurnPosition(int32_t angle_mdeg, uint16_t maxSpeed_dps);
    void multiTurnPosition(int64_t angle, uint32_t maxSpeed);

    // incremental position
    void incrementalPosition(int32_t delta, uint32_t maxSpeed);

    // speed control
    void speedControl(int32_t speed);

    // state reads — refresh all cached values from motor, then getters return last valid
    void refreshState();
    int64_t  getMotorAngle()  const { return cacheAngle_; }
    int16_t  getSpeed()       const { return cacheSpeed_; }
    uint16_t getEncoder()     const { return cacheEncoder_; }
    int8_t   getTemperature() const { return cacheTemp_; }
    uint16_t getVoltage()     const { return cacheVoltage_; }
    uint16_t getBusCurrent()  const { return cacheCurrent_; }
    uint8_t  getMotorState()  const { return cacheMotorState_; }
    uint8_t  getErrorState()  const { return cacheErrorState_; }
    int32_t  getAcceleration();

    // set any angle (RAM)
    void setAnyAngle(int32_t angle);

    // set speed limit (via control param 0x20)
    void setSpeedLimit(int32_t speedLimit);

    // set angle limit (via control param 0x22, 0=disable)
    void setAngleLimit(int32_t angleLimit);
    int32_t getAngleLimit();

    // fire-and-forget send (no reply wait, for sync multi-motor start)
    void sendMultiTurnPosition(int32_t angle_centideg, uint16_t maxSpeed_dps);

    int getMotorId() const { return motor_id_; }

private:
    std::shared_ptr<SerialPort> serial_;
    int motor_id_;

    std::mutex tx_mutex_;

    // cached state — only updated on successful read
    int64_t  cacheAngle_      = 0;
    int16_t  cacheSpeed_      = 0;
    uint16_t cacheEncoder_    = 0;
    int8_t   cacheTemp_       = 0;
    uint16_t cacheVoltage_    = 0;
    uint16_t cacheCurrent_    = 0;
    uint8_t  cacheMotorState_ = 0;
    uint8_t  cacheErrorState_ = 0;

    // frame helpers
    std::vector<uint8_t> buildFrame(uint8_t cmd, const std::vector<uint8_t>& data = {}) const;
    bool parseResponse(const std::vector<uint8_t>& raw, uint8_t expectedCmd,
                       std::vector<uint8_t>& data) const;

    // send+recv with retry
    std::vector<uint8_t> sendAndRecv(uint8_t cmd,
                                      const std::vector<uint8_t>& txData = {},
                                      int timeoutMs = 200, int retries = 3);

    // write control param
    void writeParam(uint8_t paramId, const std::vector<uint8_t>& value);

    // read control param
    std::vector<uint8_t> readParam(uint8_t paramId);
};
