#include "DM2CController.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace DM2C_Registers;
// using namespace std::chrono_literals;

// 构造函数
DM2CController::DM2CController(LiftColumn side, const std::string& port_name, int baudrate) 
    : side_(side) {
        serial_ = SerialPort::create(port_name, baudrate);
    }

// 基础控制方法

void DM2CController::setControlMode(uint16_t mode) {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_MODE, mode);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}

void DM2CController::setTargetPosition(int32_t position) {
    uint16_t pos_high = (position >> 16) & 0xFFFF;
    uint16_t pos_low = position & 0xFFFF;
    
    auto frame_high = ModbusFrameBuilder::buildWriteFrame(side_, PRO_POSITION_HIGH, pos_high);
    auto frame_low = ModbusFrameBuilder::buildWriteFrame(side_, PRO_POSITION_LOW, pos_low);
    
    serial_->send(frame_high);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
    serial_->send(frame_low);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}

void DM2CController::setSpeed(int16_t speed_rpm) {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_SPEED, speed_rpm);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}

void DM2CController::setAcceleration(uint16_t accel) {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_ACCEL, accel);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}

void DM2CController::setDeceleration(uint16_t decel) {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_DECEL, decel);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}

void DM2CController::triggerMotion(uint16_t trigger_value) {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_TRIGGER, trigger_value);
    for(auto n : frame)
	printf("0x%X ", n);
    std::cout<<std::endl;
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}


// 绝对位置控制 写入单个寄存器
void DM2CController::moveToAbsolutePosition(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration) {
    // 设置绝对位置模式
    setControlMode(MODE_ABSOLUTE_POSITION);
    
    // 设置目标位置
    setTargetPosition(target_position);
    
    // 设置运动参数
    setSpeed(speed);
    setAcceleration(acceleration);
    setDeceleration(deceleration);
    
    // 触发运行
    triggerMotion(TRIGGER_START);
}

// 相对位置控制
void DM2CController::moveToRelativePosition(int32_t relative_distance, int16_t speed, uint16_t acceleration, uint16_t deceleration) {
    // 设置相对位置模式
    setControlMode(MODE_RELATIVE_POSITION);
    
    // 设置相对距离
    setTargetPosition(relative_distance);
    
    // 设置运动参数
    setSpeed(speed);
    setAcceleration(acceleration);
    setDeceleration(deceleration);
    
    // 触发运行
    triggerMotion(TRIGGER_START);
}

// 速度控制
void DM2CController::startVelocityMode(int16_t target_speed, uint16_t acceleration, uint16_t deceleration) {
    // 设置速度模式
    setControlMode(MODE_VELOCITY);
    
    // 设置运动参数
    setSpeed(target_speed);
    setAcceleration(acceleration);
    setDeceleration(deceleration);
    
    // 触发运行
    triggerMotion(TRIGGER_START);
}

// 绝对位置控制 同时写入多个寄存器
void DM2CController::moveToAbsolutePositionBatch(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration) {
    std::vector<uint16_t> reg_values;
    
    reg_values.push_back(MODE_ABSOLUTE_POSITION);                                   // 0x6200: 控制模式
    reg_values.push_back(static_cast<uint16_t>((target_position >> 16) & 0xFFFF));  // 0x6201: 位置高16位
    reg_values.push_back(static_cast<uint16_t>(target_position & 0xFFFF));          // 0x6202: 位置低16位
    reg_values.push_back(static_cast<uint16_t>(speed));                             // 0x6203: 速度
    reg_values.push_back(acceleration);                                             // 0x6204: 加速度
    reg_values.push_back(deceleration);                                             // 0x6205: 减速度
    
    auto frame = ModbusFrameBuilder::buildMultiWriteFrame(side_, PRO_MODE, reg_values);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    triggerMotion(TRIGGER_START);
}

// 相对位置控制 同时写入多个寄存器
void DM2CController::moveToRelativePositionBatch(int32_t target_position, int16_t speed, uint16_t acceleration, uint16_t deceleration){
    std::vector<uint16_t> reg_values;
    
    reg_values.push_back(MODE_RELATIVE_POSITION);                                   // 0x6200: 控制模式
    reg_values.push_back(static_cast<uint16_t>((target_position >> 16) & 0xFFFF));  // 0x6201: 位置高16位
    reg_values.push_back(static_cast<uint16_t>(target_position & 0xFFFF));          // 0x6202: 位置低16位
    reg_values.push_back(static_cast<uint16_t>(speed));                             // 0x6203: 速度
    reg_values.push_back(acceleration);                                             // 0x6204: 加速度
    reg_values.push_back(deceleration);                                             // 0x6205: 减速度
    
    auto frame = ModbusFrameBuilder::buildMultiWriteFrame(side_, PRO_MODE, reg_values);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    triggerMotion(TRIGGER_START);
}

// 速度控制 同时写入多个寄存器
void DM2CController::startVelocityModeBatch(int16_t speed, uint16_t acceleration, uint16_t deceleration){
    std::vector<uint16_t> reg_values;
    
    reg_values.push_back(MODE_VELOCITY);                                   // 0x6200: 控制模式
    reg_values.push_back(0x00);                                                     // 0x6201: 位置高16位
    reg_values.push_back(0x00);                                                     // 0x6202: 位置低16位
    reg_values.push_back(static_cast<uint16_t>(speed));                             // 0x6203: 速度
    reg_values.push_back(acceleration);                                             // 0x6204: 加速度
    reg_values.push_back(deceleration);                                             // 0x6205: 减速度
    
    auto frame = ModbusFrameBuilder::buildMultiWriteFrame(side_, PRO_MODE, reg_values);
    serial_->send(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    triggerMotion(TRIGGER_START);
}

// 停止控制 
void DM2CController::stopAllMotion() {
    triggerMotion(TRIGGER_STOP);
}

// 状态读取 
uint16_t DM2CController::readRegister(uint16_t reg_addr) {
    auto read_frame = ModbusFrameBuilder::buildReadFrame(side_, reg_addr);
    serial_->flushInput();
    serial_->send(read_frame);
    
    auto response = serial_->receive(200);
    
    if (response.size() >= 7 && response[0] == read_frame[0] && response[1] == read_frame[1]) {
        auto crc = ModbusFrameBuilder::calculateCRC16(response.data(), response.size() - 2);
        uint16_t received_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
        if (crc == received_crc) 
            return (response[3] << 8) | response[4];
    }
    
    return 0xFFFF;  // 错误标志
}

//一次读取多个寄存器
std::vector<uint8_t> DM2CController::readRegisters(uint16_t reg_addr, int count){
    auto read_frame = ModbusFrameBuilder::buildReadFrame(side_, reg_addr, count);
    serial_->flushInput();
    serial_->send(read_frame);
    
    auto response = serial_->receive(200);
    if (response.size() >= 7 && response[0] == read_frame[0] && response[1] == read_frame[1]) {
        auto crc = ModbusFrameBuilder::calculateCRC16(response.data(), response.size() - 2);
        uint16_t received_crc = (response[response.size() - 1] << 8) | response[response.size() - 2];
        if (crc == received_crc) {
            std::vector<uint8_t> valus(response.begin()+3, response.end()-2);
            return valus;
        }
    }
    return {};
}
uint16_t DM2CController::readStatus() {
    return readRegister(STATUS);
}

uint16_t DM2CController::readAlarm() {
    return readRegister(ALARM);
}

uint32_t DM2CController::readCommandPosition() {
    auto values = readRegisters(CMDPOS_H, 2);
    uint16_t high = (values[0] << 8) | values[1];
    uint16_t low  = (values[2] << 8) | values[3];
    return (static_cast<int32_t>(high) << 16) | low;
}

uint32_t DM2CController::readActualPosition() {
    auto values = readRegisters(CURPOS_H, 2);
    uint16_t high = (values[0] << 8) | values[1];
    uint16_t low  = (values[2] << 8) | values[3];
    return (static_cast<int32_t>(high) << 16) | low;
}

uint16_t DM2CController::readActualSpeed() {
    return readRegister(CURSPEED);
}

// 设零操作
void DM2CController::setCurrentPositionAsZero() {
    auto frame = ModbusFrameBuilder::buildWriteFrame(side_, PRO_TRIGGER, SETZERO);
    serial_->send(frame);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_->flushInput();
}
