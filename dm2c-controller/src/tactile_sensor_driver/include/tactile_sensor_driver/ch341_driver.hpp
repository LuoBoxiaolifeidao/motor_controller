#ifndef TACTILE_SENSOR_DRIVER__CH341_DRIVER_HPP_
#define TACTILE_SENSOR_DRIVER__CH341_DRIVER_HPP_

#include <vector>
#include <string>
#include <cstdint>
#include <memory>
#include <ros/ros.h>

// 外部C函数
extern "C" {
    int CH34xOpenDevice(const char* name);
    void CH34xCloseDevice(int fd);
    bool CH34xGetInput(int fd, uint32_t* status);
    bool CH34xWriteData(int fd, void* buf, uint32_t* len);
    bool CH34xWriteRead(int fd, uint32_t write_len, void* write_buf, 
                        uint32_t read_step, uint32_t read_anno, 
                        uint32_t* read_len, void* read_buf);
    bool CH34xSetOutput(int fd, uint32_t out_en, uint32_t out_val, uint32_t out_status);
    bool CH34xSetStream(int fd, uint32_t mode);
}

// 移到类外面，GCC7 就不会报 undefined reference
constexpr uint8_t CMD_I2C_STREAM = 0xAA;
constexpr uint8_t CMD_I2C_STM_STA = 0x74;
constexpr uint8_t CMD_I2C_STM_STO = 0x75;
constexpr uint8_t CMD_I2C_STM_OUT = 0x80;
constexpr uint8_t CMD_I2C_STM_IN  = 0xC0;
constexpr uint8_t CMD_I2C_STM_MAX = 63;
constexpr uint8_t CMD_I2C_STM_END = 0x00;
constexpr uint8_t CMD_I2C_STM_MS  = 0x50;
constexpr uint32_t STATE_BIT_INT  = 0x00000400;

class Ch341Driver {
public:
    static constexpr uint32_t IIC_SPEED_20 = 0;
    static constexpr uint32_t IIC_SPEED_100 = 1;
    static constexpr uint32_t IIC_SPEED_400 = 2;
    static constexpr uint32_t IIC_SPEED_750 = 3;

    Ch341Driver();
    ~Ch341Driver();

    bool init();
    bool open();
    void disconnect();
    bool connect_check();
    
    int write(uint8_t addr, const std::vector<uint8_t>& data);
    int read(uint8_t addr, std::vector<uint8_t>& data);
    
    bool set_speed(uint32_t speed);
    void set_int(bool lvl);

private:
    // 这里已经删掉那一堆 static constexpr 了

    int fd_;
    uint32_t device_id_;
    ros::NodeHandle nh_;
};

#endif