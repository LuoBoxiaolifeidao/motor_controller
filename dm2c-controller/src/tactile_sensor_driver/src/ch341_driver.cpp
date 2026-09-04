#include "tactile_sensor_driver/ch341_driver.hpp"
#include <glob.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

Ch341Driver::Ch341Driver() 
    : fd_(-1), device_id_(0) {}

Ch341Driver::~Ch341Driver() {
    disconnect();
}

bool Ch341Driver::init() {
    ROS_INFO("CH341 driver init");
    return true;
}

bool Ch341Driver::open() {
    glob_t glob_result;
    int return_value = glob("/dev/ch34x_pis*", GLOB_TILDE, NULL, &glob_result);
    
    if (return_value != 0) {
        ROS_ERROR("Linux: 未找到 /dev/ch34x_pis* 设备");
        globfree(&glob_result);
        return false;
    }

    std::string device_path = glob_result.gl_pathv[0];
    ROS_INFO("Find device: %s", device_path.c_str());

    fd_ = CH34xOpenDevice(device_path.c_str());
    globfree(&glob_result);

    if (fd_ == -1) {
        ROS_ERROR("Linux: 打开设备失败 (fd=-1)");
        return false;
    }

    ROS_INFO("Linux: Device is open (fd=%d)", fd_);
    return true;
}

void Ch341Driver::disconnect() {
    if (fd_ != -1) {
        CH34xCloseDevice(fd_);
        fd_ = -1;
        ROS_INFO("Device is disconnet");
    }
}

bool Ch341Driver::connect_check() {
    if (fd_ == -1) return false;
    return CH34xGetInput(fd_, &device_id_);
}

int Ch341Driver::write(uint8_t addr, const std::vector<uint8_t>& data) {
    if (fd_ == -1) return 0;

    std::vector<uint8_t> pack;
    std::vector<uint8_t> tmp_data = data;
    size_t s_len = data.size();
    size_t cnt = 20;
    size_t pack_num = s_len / cnt;
    size_t remainder = s_len % cnt;

    auto send_packet = [&](std::vector<uint8_t>& p) {
        uint32_t len = p.size();
        return CH34xWriteData(fd_, p.data(), &len);
    };

    pack.push_back(CMD_I2C_STREAM);
    pack.push_back(CMD_I2C_STM_STA);
    pack.push_back(CMD_I2C_STM_OUT | 1);
    pack.push_back(addr << 1);

    for (size_t i = 0; i < pack_num; ++i) {
        pack.push_back(CMD_I2C_STM_OUT | static_cast<uint8_t>(cnt));
        pack.insert(pack.end(), tmp_data.begin(), tmp_data.begin() + cnt);
        tmp_data.erase(tmp_data.begin(), tmp_data.begin() + cnt);
        pack.push_back(CMD_I2C_STM_END);

        if (!send_packet(pack)) return 0;
        pack.clear();
        pack.push_back(CMD_I2C_STREAM);
    }

    if (remainder >= 1) {
        pack.push_back(CMD_I2C_STM_OUT | static_cast<uint8_t>(remainder));
        pack.insert(pack.end(), tmp_data.begin(), tmp_data.end());
    }

    pack.push_back(CMD_I2C_STM_STO);
    pack.push_back(CMD_I2C_STM_END);

    if (!send_packet(pack)) return 0;
    return s_len;
}

int Ch341Driver::read(uint8_t addr, std::vector<uint8_t>& data) {
    if (fd_ == -1) return 0;
    
    size_t r_len = data.size();
    std::vector<uint8_t> pack;
    std::vector<uint8_t> read_buf_total;
    
    size_t pack_num = r_len / 30;
    size_t remainder = r_len % 30;
    if (remainder == 0 && pack_num > 0) {
        remainder = 30;
        pack_num -= 1;
    }

    pack.push_back(CMD_I2C_STREAM);
    pack.push_back(CMD_I2C_STM_STA);
    pack.push_back(CMD_I2C_STM_OUT | 1);
    pack.push_back((addr << 1) | 0x01);
    pack.push_back(CMD_I2C_STM_MS | 1);

    auto do_read = [&]() {
        uint32_t rec_len = 0;
        uint8_t rec_buf[CMD_I2C_STM_MAX];
        if (!CH34xWriteRead(fd_, pack.size(), pack.data(), CMD_I2C_STM_MAX, 1, &rec_len, rec_buf)) {
            return false;
        }
        read_buf_total.insert(read_buf_total.end(), rec_buf, rec_buf + rec_len);
        return true;
    };

    for (size_t i = 0; i < pack_num; ++i) {
        pack.push_back(CMD_I2C_STM_IN | 30);
        pack.push_back(CMD_I2C_STM_END);
        
        if (!do_read()) return 0;
        
        pack.clear();
        pack.push_back(CMD_I2C_STREAM);
    }

    if (remainder > 1) {
        pack.push_back(CMD_I2C_STM_IN | static_cast<uint8_t>(remainder - 1));
    }
    pack.push_back(CMD_I2C_STM_IN | 0);
    pack.push_back(CMD_I2C_STM_STO);
    pack.push_back(CMD_I2C_STM_END);

    if (!do_read()) return 0;

    data = read_buf_total;
    return data.size();
}

bool Ch341Driver::set_speed(uint32_t speed) {
    if (fd_ == -1) return false;
    if (!CH34xSetStream(fd_, speed)) {
        ROS_WARN("设置速度失败");
        return false;
    }
    return true;
}

void Ch341Driver::set_int(bool lvl) {
    if (fd_ == -1) return;
    uint32_t status = 0;
    CH34xGetInput(fd_, &status);
    usleep(10000); // 10ms
    
    if (lvl) {
        CH34xSetOutput(fd_, 0x03, 0xFF00, status | STATE_BIT_INT);
    } else {
        CH34xSetOutput(fd_, 0x03, 0xFF00, status & (~STATE_BIT_INT));
    }
}