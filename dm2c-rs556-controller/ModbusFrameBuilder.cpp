#include "ModbusFrameBuilder.h"


// 计算 Modbus CRC16
uint16_t ModbusFrameBuilder::calculateCRC16(const uint8_t* data, int length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 提取16位值的高字节
uint8_t ModbusFrameBuilder::getHighByte(uint16_t value) {
    return (value >> 8) & 0xFF;
}

// 提取16位值的低字节
uint8_t ModbusFrameBuilder::getLowByte(uint16_t value) {
    return value & 0xFF;
}


// 构建基本帧（无CRC）
std::vector<uint8_t> ModbusFrameBuilder::buildBasicFrame(const uint8_t slave_id, uint8_t function_code, uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> frame(6);
    
    frame[0] = slave_id;                    // 从站地址
    frame[1] = function_code;                // 功能码
    frame[2] = getHighByte(reg_addr);        // 寄存器地址高字节
    frame[3] = getLowByte(reg_addr);         // 寄存器地址低字节
    frame[4] = getHighByte(value);           // 数据高字节
    frame[5] = getLowByte(value);            // 数据低字节
    
    return frame;
}

// 构建读取多个寄存器的基本帧
std::vector<uint8_t> ModbusFrameBuilder::buildReadMultipleFrame(const uint8_t slave_id, uint16_t reg_addr, uint16_t count) {
    std::vector<uint8_t> frame(6);
    
    frame[0] = slave_id;                    // 从站地址
    frame[1] = 0x03;                         // 功能码: 读保持寄存器
    frame[2] = getHighByte(reg_addr);        // 寄存器地址高字节
    frame[3] = getLowByte(reg_addr);         // 寄存器地址低字节
    frame[4] = getHighByte(count);           // 寄存器数量高字节
    frame[5] = getLowByte(count);            // 寄存器数量低字节
    
    return frame;
}

// 为任意长度的帧添加CRC
void ModbusFrameBuilder::addCRC(std::vector<uint8_t>& basic_frame) {
    uint16_t crc = calculateCRC16(basic_frame.data(), basic_frame.size());
    
    basic_frame.push_back(getLowByte(crc));   // CRC低字节在前
    basic_frame.push_back(getHighByte(crc));  // CRC高字节
}

// 构建完整的写寄存器帧
std::vector<uint8_t> ModbusFrameBuilder::buildWriteFrame(const uint8_t slave_id, const uint16_t reg_addr, const uint16_t value) {
    auto basic_frame = buildBasicFrame(slave_id, 0x06, reg_addr, value);
    addCRC(basic_frame);
    return basic_frame;
}

// 构建完整的读单个寄存器帧
std::vector<uint8_t> ModbusFrameBuilder::buildReadFrame(const uint8_t slave_id, const uint16_t reg_addr) {
    return buildReadFrame(slave_id, reg_addr, 1);
}

// 构建完整的读多个寄存器帧
std::vector<uint8_t> ModbusFrameBuilder::buildReadFrame(const uint8_t slave_id, const uint16_t reg_addr, const uint16_t count) {
    auto basic_frame = buildReadMultipleFrame(slave_id, reg_addr, count);
    addCRC(basic_frame);
    return basic_frame;
}

// 构建完整的写多个寄存器帧
std::vector<uint8_t> ModbusFrameBuilder::buildMultiWriteFrame(const uint8_t slave_id, const uint16_t start_reg_addr, const std::vector<uint16_t>& values) {
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);
    frame.push_back(0x10);
    
    // 3. 起始地址 (16位，高字节在前)
    frame.push_back((start_reg_addr >> 8) & 0xFF);
    frame.push_back(start_reg_addr & 0xFF);
    
    // 4. 寄存器数量 (16位，高字节在前)
    uint16_t reg_count = static_cast<uint16_t>(values.size());
    frame.push_back(getHighByte(reg_count));
    frame.push_back(getLowByte(reg_count));
    
    // 5. 字节数
    uint8_t byte_count = reg_count * 2;
    frame.push_back(byte_count);
    
    // 6. 数据
    for (uint16_t value : values) {
        frame.push_back(getHighByte(value));
        frame.push_back(getLowByte(value));
    }
    
    addCRC(frame);
    return frame;                                                    
}
