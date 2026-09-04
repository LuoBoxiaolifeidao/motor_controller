#ifndef TACTILE_SENSOR_DRIVER__SENSOR_CMD_HPP_
#define TACTILE_SENSOR_DRIVER__SENSOR_CMD_HPP_

#include <vector>
#include <cstdint>
#include <memory>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include "tactile_sensor_driver/ch341_driver.hpp"

class SensorCmd {
public:
    explicit SensorCmd(std::shared_ptr<Ch341Driver> ch341);

    // 传感器指令集
    static constexpr uint8_t CMD_GET_CHANNEL_NUM          = 0x01;
    static constexpr uint8_t CMD_GET_SENSOR_CAP_DATA      = 0x60;
    static constexpr uint8_t CMD_GET_SENSOR_CHANNEL       = 0x62;
    static constexpr uint8_t CMD_SET_SENSOR_AUTO_DAC      = 0x63;
    static constexpr uint8_t CMD_GET_SENSOR_ERR_CODE      = 0x64;
    static constexpr uint8_t CMD_GET_SENSOR_TEST_HZ       = 0x6C;
    static constexpr uint8_t CMD_SET_SENSOR_IIC_ADDR      = 0x70;
    static constexpr uint8_t CMD_GET_SENSOR_IIC_ADDR      = 0x71;
    static constexpr uint8_t CMD_SET_SENSOR_CDC_SYNC      = 0x72;
    static constexpr uint8_t CMD_SET_SENSOR_CDC_START_OFFSET = 0x73;
    static constexpr uint8_t CMD_SET_SENSOR_RESTART       = 0x77;
    static constexpr uint8_t CMD_SET_SENSOR_SEND_TYPE     = 0x7F;
    static constexpr uint8_t CMD_GET_VERSION              = 0xA0;
    static constexpr uint8_t CMD_SOFT_RESTART             = 0xA1;
    static constexpr uint8_t CMD_GET_TYPE                 = 0xA2;
    static constexpr uint8_t CMD_SET_TYPE                 = 0xA3;
    static constexpr uint8_t CMD_SET_INF                  = 0xA5;
    static constexpr uint8_t CMD_GET_PRG                  = 0xA6;

    // 核心功能函数
    uint8_t get_addr(uint8_t addr);
    bool set_sensor_send_type(uint8_t addr, uint8_t send_type);
    bool set_sensor_cap_offset(uint8_t addr, uint8_t offset);
    int get_sensor_project_index(uint8_t addr);
    bool get_sensor_cap_data(uint8_t addr, std::vector<uint8_t>& buf);
    bool set_sensor_sync(uint8_t addr);
    bool set_addr(uint8_t addr, uint8_t new_addr);

private:
    // 校验和处理
    void calc_sum(std::vector<uint8_t>& pack);
    bool check_sum(const std::vector<uint8_t>& pack);

    std::shared_ptr<Ch341Driver> ch341_;
};

#endif  // TACTILE_SENSOR_DRIVER__SENSOR_CMD_HPP_