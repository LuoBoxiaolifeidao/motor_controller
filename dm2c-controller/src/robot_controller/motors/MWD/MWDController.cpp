#include "MWDController.h"
#include <cstring>
#include <cstdio>

using namespace MWD_CMD;

MWDController::MWDController(int motor_id, const std::string& can_iface, uint32_t bitrate)
    : motor_id_(motor_id), can_id_(0x140 + motor_id)
{
    can_ = CANComMWD::create(can_iface, bitrate);
}

MWDController::~MWDController() {}

void MWDController::enable() {
    uint8_t data[8] = {ENABLE, 0,0,0,0,0,0,0};
    can_->send(can_id_, data);
}

void MWDController::disable() {
    uint8_t data[8] = {DISABLE, 0,0,0,0,0,0,0};
    can_->send(can_id_, data);
}

void MWDController::stop() {
    uint8_t data[8] = {STOP, 0,0,0,0,0,0,0};
    can_->send(can_id_, data);
}

void MWDController::setAcceleration(int32_t accel_dps2) {
    uint8_t data[8] = {WRITE_ACCEL, 0, 0, 0, 0, 0, 0, 0};
    data[4] = static_cast<uint8_t>(accel_dps2 & 0xFF);
    data[5] = static_cast<uint8_t>((accel_dps2 >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>((accel_dps2 >> 16) & 0xFF);
    data[7] = static_cast<uint8_t>((accel_dps2 >> 24) & 0xFF);
    can_->send(can_id_, data);
}

void MWDController::multiTurnPosition(int32_t angle_mdeg, uint16_t maxSpeed_dps) {
    uint8_t data[8] = {POS_MULTI2, 0, 0, 0, 0, 0, 0, 0};
    data[2] = static_cast<uint8_t>(maxSpeed_dps & 0xFF);
    data[3] = static_cast<uint8_t>((maxSpeed_dps >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(angle_mdeg & 0xFF);
    data[5] = static_cast<uint8_t>((angle_mdeg >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>((angle_mdeg >> 16) & 0xFF);
    data[7] = static_cast<uint8_t>((angle_mdeg >> 24) & 0xFF);
    can_->send(can_id_, data);
}

void MWDController::brakeRelease() {
    uint8_t data[8] = {BRAKE_CTRL, 0x01, 0,0,0,0,0,0};  // 0x01: 通电, 刹车释放
    can_->send(can_id_, data);
}

void MWDController::brakeLock() {
    uint8_t data[8] = {BRAKE_CTRL, 0x00, 0,0,0,0,0,0};  // 0x00: 断电, 刹车启动
    can_->send(can_id_, data);
}

uint8_t MWDController::getBrakeState() {
    uint8_t send[8] = {BRAKE_CTRL, 0x10, 0,0,0,0,0,0};  // 0x10: 读取抱闸器状态
    uint8_t recv[8] = {};
    
    if (can_->transaction(can_id_, send, recv, 30) && recv[0] == BRAKE_CTRL) {
        return recv[1];  // 0x00=刹车启动, 0x01=刹车释放
    }
    return 0xFF;  // 读取失败
}

void MWDController::clearError() {
    uint8_t data[8] = {0x9B, 0,0,0,0,0,0,0};
    can_->send(can_id_, data);
}

void MWDController::setZeroToROM() {
    uint8_t data[8] = {0x19, 0,0,0,0,0,0,0};
    can_->send(can_id_, data);
}

static bool poll_can(uint32_t can_id, uint8_t cmd, uint8_t* out,
                     std::shared_ptr<CANComMWD>& can)
{
    uint8_t send[8] = {cmd, 0,0,0,0,0,0,0};
    for (int r = 0; r < 5; r++) {
        if (can->transaction(can_id, send, out, 30) && out[0] == cmd)
            return true;
    }
    return false;
}

void MWDController::pollAngle() {
    uint8_t recv[8] = {};
    if (!poll_can(can_id_, READ_ANGLE, recv, can_)) return;

    int64_t angle = 0;
    angle |= static_cast<int64_t>(recv[1]) & 0xFF;
    angle |= (static_cast<int64_t>(recv[2]) & 0xFF) << 8;
    angle |= (static_cast<int64_t>(recv[3]) & 0xFF) << 16;
    angle |= (static_cast<int64_t>(recv[4]) & 0xFF) << 24;
    angle |= (static_cast<int64_t>(recv[5]) & 0xFF) << 32;
    angle |= (static_cast<int64_t>(recv[6]) & 0xFF) << 40;
    angle |= (static_cast<int64_t>(recv[7]) & 0xFF) << 48;
    if (angle & (1LL << 55)) angle |= 0xFF00000000000000LL;

    std::lock_guard<std::mutex> lock(state_mutex_);
    motorAngle_ = angle;
}

void MWDController::pollState1() {
    uint8_t recv[8] = {};
    if (!poll_can(can_id_, READ_STATE1, recv, can_)) return;

    std::lock_guard<std::mutex> lock(state_mutex_);
    temperature_ = static_cast<int8_t>(recv[1]);
    voltage_     = static_cast<uint16_t>(recv[2] | (recv[3] << 8));
    busCurrent_  = static_cast<uint16_t>(recv[4] | (recv[5] << 8));
    motorState_  = recv[6];
    errorState_  = recv[7];
}

void MWDController::pollState2() {
    uint8_t recv[8] = {};
    if (!poll_can(can_id_, READ_STATE2, recv, can_)) return;

    std::lock_guard<std::mutex> lock(state_mutex_);
    temperature_ = static_cast<int8_t>(recv[1]);
    speed_       = static_cast<int16_t>(recv[4] | (recv[5] << 8));
    encoder_     = static_cast<uint16_t>(recv[6] | (recv[7] << 8));
}

void MWDController::pollAccel() {
    uint8_t recv[8] = {};
    if (!poll_can(can_id_, READ_ACCEL, recv, can_)) return;

    int32_t accel = static_cast<int32_t>(recv[4] | (recv[5] << 8) | (recv[6] << 16) | (recv[7] << 24));
    std::lock_guard<std::mutex> lock(state_mutex_);
    acceleration_ = accel;
}

int16_t MWDController::getSpeed() {
    pollState2();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return speed_;
}

uint16_t MWDController::getEncoder() {
    pollState2();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return encoder_;
}

int64_t MWDController::getMotorAngle() {
    pollAngle();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motorAngle_;
}

int8_t MWDController::getTemperature() {
    pollState1();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return temperature_;
}

uint16_t MWDController::getVoltage() {
    pollState1();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return voltage_;
}

uint16_t MWDController::getBusCurrent() {
    pollState1();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return busCurrent_;
}

uint8_t MWDController::getMotorState() {
    pollState1();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return motorState_;
}

uint8_t MWDController::getErrorState() {
    pollState1();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return errorState_;
}

int32_t MWDController::getAcceleration() {
    pollAccel();
    std::lock_guard<std::mutex> lock(state_mutex_);
    return acceleration_;
}
