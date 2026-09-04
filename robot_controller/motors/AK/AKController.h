#pragma once
#include "CANComm.h"
#include <mutex>

class AKController {

public:
    explicit AKController(const std::string& interface, uint32_t bitrate);
    ~AKController();
    
    bool moveToAbsolutePosition(int id, int32_t target_position, int16_t speed, int16_t acceleration);
    bool setCurrentPositionAsZero(int id);
    bool setCurrent(int id, float current);
    bool readCanState();

    float position[3];
    float speed[3];
    float current[3];
    int8_t temperature[3];
    uint8_t errorcode[3];

    mutable std::mutex state_mutex_;
private:
    CANComm can_comm_;
    uint32_t set_pos_spd_acc_canid[3];
    uint32_t set_current_canid[3];
    uint32_t set_zero_canid[3];
    uint32_t get_status_canid[3];                                    
};

