#include "DM2CController.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace DM2C_Registers;

std::vector<DM2CController*> DM2CController::instances_;
std::mutex DM2CController::instances_mutex_;
std::atomic<int> DM2CController::instanceCount_{0};


DM2CController::DM2CController(LiftColumn side, const std::string& port_name, int baudrate)
    : side_(side)
{
    serial_ = SerialPort::create(port_name, baudrate);
    std::lock_guard<std::mutex> lock(instances_mutex_);
    instances_.push_back(this);
    instanceCount_++;
}

DM2CController::~DM2CController()
{
    std::lock_guard<std::mutex> lock(instances_mutex_);
    auto it = std::find(instances_.begin(), instances_.end(), this);
    if (it != instances_.end()) {
        instances_.erase(it);
    }
    instanceCount_--;
}

static constexpr int MAX_RETRIES = 3;
static constexpr int RESPONSE_TIMEOUT_MS = 150;

bool DM2CController::sendWithRetry(const std::vector<uint8_t>& frame) {
    for (int i = 0; i < MAX_RETRIES; ++i) {
        auto response = serial_->transaction(frame, RESPONSE_TIMEOUT_MS);
        if (response.size() >= 4 && response[0] == frame[0] && response[1] == frame[1]) {
            uint16_t crc = ModbusFrameBuilder::calculateCRC16(response.data(), response.size() - 2);
            uint16_t received_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
            if (crc == received_crc)
                return true;
        }
         if (i < MAX_RETRIES - 1)
             std::this_thread::sleep_for(std::chrono::milliseconds(20));
     }
     std::cerr << "DM2C sendWithRetry failed after " << MAX_RETRIES << " attempts" << std::endl;
     return false;
}

void DM2CController::triggerMotion(uint16_t trigger_value) {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_TRIGGER, trigger_value);
    sendWithRetry(frame);
}

void DM2CController::moveToAbsolutePosition(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration) {
    std::vector<uint16_t> reg_values;
    
    reg_values.push_back(MODE_ABSOLUTE_POSITION);
    reg_values.push_back(static_cast<uint16_t>((target_position >> 16) & 0xFFFF));
    reg_values.push_back(static_cast<uint16_t>(target_position & 0xFFFF));
    reg_values.push_back(static_cast<uint16_t>(speed));
    reg_values.push_back(acceleration);
    reg_values.push_back(deceleration);
    
    auto prIndex = curRPIndex_ == 0? PRO_MODE : PR1_MODE;
    auto triggerIndex = curRPIndex_ == 0? TRIGGER_START0 : TRIGGER_START1;
    auto frame = ModbusFrameBuilder::buildMultiWriteFrame(side_, prIndex, reg_values);
    
    sendWithRetry(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    triggerMotion(triggerIndex);
    curRPIndex_ = !curRPIndex_;
}

void DM2CController::moveToRelativePosition(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration){
    std::vector<uint16_t> reg_values;
    
    reg_values.push_back(MODE_RELATIVE_POSITION);
    reg_values.push_back(static_cast<uint16_t>((target_position >> 16) & 0xFFFF));
    reg_values.push_back(static_cast<uint16_t>(target_position & 0xFFFF));
    reg_values.push_back(static_cast<uint16_t>(speed));
    reg_values.push_back(acceleration);
    reg_values.push_back(deceleration);
    
    auto prIndex = curRPIndex_ == 0? PRO_MODE : PR1_MODE;
    auto triggerIndex = curRPIndex_ == 0? TRIGGER_START0 : TRIGGER_START1;
    auto frame = ModbusFrameBuilder::buildMultiWriteFrame(side_, prIndex, reg_values);
    sendWithRetry(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    triggerMotion(triggerIndex);
    curRPIndex_ = !curRPIndex_;
}

void DM2CController::startVelocityMode(int16_t speed, uint16_t acceleration, uint16_t deceleration){
    std::vector<uint16_t> reg_values;
    
    reg_values.push_back(MODE_VELOCITY);
    reg_values.push_back(0x00);
    reg_values.push_back(0x00);
    reg_values.push_back(speed);
    reg_values.push_back(acceleration);
    reg_values.push_back(deceleration);
                                
    auto prIndex = curRPIndex_ == 0? PRO_MODE : PR1_MODE;
    auto triggerIndex = curRPIndex_ == 0? TRIGGER_START0 : TRIGGER_START1;
    auto frame = ModbusFrameBuilder::buildMultiWriteFrame(side_, prIndex, reg_values);
    sendWithRetry(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    triggerMotion(triggerIndex);
    curRPIndex_ = !curRPIndex_;
}

void DM2CController::stopAllMotion() {
    triggerMotion(TRIGGER_STOP);
}

uint16_t DM2CController::readRegister(uint16_t reg_addr) {
    auto read_frame = ModbusFrameBuilder::buildReadFrame(side_, reg_addr);
    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        auto response = serial_->transaction(read_frame, RESPONSE_TIMEOUT_MS);
        if (response.size() >= 7 && response[0] == read_frame[0] && response[1] == read_frame[1]) {
            auto crc = ModbusFrameBuilder::calculateCRC16(response.data(), response.size() - 2);
            uint16_t received_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
            if (crc == received_crc)
                return (response[3] << 8) | response[4];
        }
        if (attempt < MAX_RETRIES - 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0xFFFF;
}

std::vector<uint8_t> DM2CController::readRegisters(uint16_t reg_addr, int count) {
    auto read_frame = ModbusFrameBuilder::buildReadFrame(side_, reg_addr, count);
    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        auto response = serial_->transaction(read_frame, RESPONSE_TIMEOUT_MS);
        if (response.size() >= 7 && response[0] == read_frame[0] && response[1] == read_frame[1]) {
            auto crc = ModbusFrameBuilder::calculateCRC16(response.data(), response.size() - 2);
            uint16_t received_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
            if (crc == received_crc) {
                std::vector<uint8_t> values(response.begin() + 3, response.end() - 2);
                return values;
            }
        }
        if (attempt < MAX_RETRIES - 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return {};
}

uint32_t DM2CController::readActualPosition() {
    auto values = readRegisters(CURPOS_H, 2);
    if (values.size() != 4) return lastPosition_;   // 读失败返回上次有效值, 避免污染增量
    uint16_t high = (values[0] << 8) | values[1];
    uint16_t low  = (values[2] << 8) | values[3];
    lastPosition_ = (static_cast<uint32_t>(high) << 16) | low;
    return lastPosition_;
}

int16_t DM2CController::readActualSpeed() {
    return static_cast<int16_t>(readRegister(CURSPEED));
}

uint16_t DM2CController::readStatus() {
    return readRegister(STATUS);
}

uint16_t DM2CController::readAlarm() {
    return readRegister(ALARM);
}

uint32_t DM2CController::readCMDPosition() {
    auto values = readRegisters(CMDPOS_H, 2);
    if (values.size() != 4) return 0xFFFF;
    uint16_t high = (values[0] << 8) | values[1];
    uint16_t low  = (values[2] << 8) | values[3];
    return (static_cast<uint32_t>(high) << 16) | low;
}

void DM2CController::setCurrentPositionAsZero() {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_TRIGGER, SETZERO);
    sendWithRetry(frame);
    saveParametersToEEPROM();
}

void DM2CController::saveParametersToEEPROM() {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, EEPROMReg, SAVEEPROM);
    sendWithRetry(frame);
}
