#pragma once

#include <vector>
#include <cstdint>

class ModbusFrameBuilder {

public:
    ModbusFrameBuilder() = delete;

    // 计算 Modbus CRC16
    static uint16_t calculateCRC16(const uint8_t* data, int length) ;
    
    // 提取16位值的高字节
    static uint8_t getHighByte(uint16_t value);
    
    // 提取16位值的低字节
    static uint8_t getLowByte(uint16_t value);
    
    // 构建基本帧（无CRC）
    static std::vector<uint8_t> buildBasicFrame(const uint8_t slave_id, uint8_t function_code, uint16_t reg_addr, uint16_t value);
    
    // 构建读取多个寄存器的基本帧
    static std::vector<uint8_t> buildReadMultipleFrame(const uint8_t slave_id, uint16_t reg_addr, uint16_t count);
    
    // 为任意长度的帧添加CRC
    static void addCRC(std::vector<uint8_t>& basic_frame);
    
    // 构建完整的写寄存器帧
    static std::vector<uint8_t> buildWriteFrame(const uint8_t slave_id, const uint16_t reg_addr, const uint16_t value);
    
    // 构建完整的读单个寄存器帧
    static std::vector<uint8_t> buildReadFrame(const uint8_t slave_id, const uint16_t reg_addr);
    
    // 构建完整的读多个寄存器帧
    static std::vector<uint8_t> buildReadFrame(const uint8_t slave_id, const uint16_t reg_addr, const uint16_t count);

    // 构建完整的写多个寄存器帧
    static std::vector<uint8_t> buildMultiWriteFrame(const uint8_t slave_id, const uint16_t start_reg_addr, const std::vector<uint16_t>& values);
};
