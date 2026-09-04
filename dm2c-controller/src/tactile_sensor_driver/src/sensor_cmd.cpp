#include "tactile_sensor_driver/sensor_cmd.hpp"

SensorCmd::SensorCmd(std::shared_ptr<Ch341Driver> ch341)
    : ch341_(ch341) {}

void SensorCmd::calc_sum(std::vector<uint8_t>& pack) {
    if (pack.size() < 5) return;
    
    uint16_t sum = 0;
    for (uint8_t val : pack) {
        sum += val;
    }
    // 添加两个字节的校验和 (小端序)
    pack.push_back(static_cast<uint8_t>(sum & 0xFF));
    pack.push_back(static_cast<uint8_t>((sum >> 8) & 0xFF));
}

bool SensorCmd::check_sum(const std::vector<uint8_t>& pack) {
    if (pack.size() < 5) return false;
    
    uint16_t sum = 0;
    // 计算除最后两个校验字节之外的所有字节之和
    for (size_t i = 0; i < pack.size() - 2; ++i) {
        sum += pack[i];
    }
    
    uint8_t chk_l = static_cast<uint8_t>(sum & 0xFF);
    uint8_t chk_h = static_cast<uint8_t>((sum >> 8) & 0xFF);
    
    return (chk_l == pack[pack.size() - 2] && chk_h == pack[pack.size() - 1]);
}

uint8_t SensorCmd::get_addr(uint8_t addr) {
    std::vector<uint8_t> pack = {0xAA, 0x55, 0x03, CMD_GET_SENSOR_IIC_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};
    calc_sum(pack);
    ch341_->write(addr, pack);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    std::vector<uint8_t> rx_buf(11, 0);
    ch341_->read(addr, rx_buf);
    
    if (check_sum(rx_buf) && rx_buf.size() >= 8) {
        return rx_buf[7];
    }
    return 0;
}

bool SensorCmd::set_sensor_send_type(uint8_t addr, uint8_t send_type) {
    std::vector<uint8_t> pack = {0xAA, 0x55, 0x03, CMD_SET_SENSOR_SEND_TYPE, 0x00, 0x00, 0x00, send_type, 0x00};
    calc_sum(pack);
    ch341_->write(addr, pack);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    std::vector<uint8_t> rx_buf(11, 0);
    ch341_->read(addr, rx_buf);
    
    // 检查回复：命令码最高位置1 (0x7F | 0x80 = 0xFF)
    return check_sum(rx_buf) && rx_buf.size() > 3 && (rx_buf[3] == (CMD_SET_SENSOR_SEND_TYPE | 0x80));
}

bool SensorCmd::set_sensor_cap_offset(uint8_t addr, uint8_t offset) {
    std::vector<uint8_t> pack = {0xAA, 0x55, 0x03, CMD_SET_SENSOR_CDC_START_OFFSET, 0x00, 0x00, 0x00, offset, 0x00};
    calc_sum(pack);
    ch341_->write(addr, pack);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    std::vector<uint8_t> rx_buf(6, 0); // Python 里是 6
    ch341_->read(addr, rx_buf);
    
    return check_sum(rx_buf) && rx_buf.size() > 3 && (rx_buf[3] == (CMD_SET_SENSOR_CDC_START_OFFSET | 0x80));
}

int SensorCmd::get_sensor_project_index(uint8_t addr) {
    std::vector<uint8_t> pack = {0xAA, 0x55, 0x03, CMD_GET_PRG, 0x00, 0x00, 0x00, 0x00, 0x00};
    calc_sum(pack);
    ch341_->write(addr, pack);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    std::vector<uint8_t> rx_buf(11, 0);
    ch341_->read(addr, rx_buf);
    
    if (check_sum(rx_buf) && rx_buf.size() >= 9) {
        return static_cast<int>(rx_buf[7]) + (static_cast<int>(rx_buf[8]) << 8);
    }
    return 0;
}

bool SensorCmd::get_sensor_cap_data(uint8_t addr, std::vector<uint8_t>& buf) {
    size_t target_len = buf.size();
    int read_len = ch341_->read(addr, buf);
    
    if (static_cast<size_t>(read_len) != target_len) {
        buf.assign(target_len, 0);
        return false;
    }

    // 检查包头 0x55 0xAA (注意 Python 里的逻辑是 buf[0]==0x55, buf[1]==0xAA)
    if (buf.size() >= 2 && buf[0] == 0x55 && buf[1] == 0xAA && check_sum(buf)) {
        return true;
    }
    return false;
}

bool SensorCmd::set_sensor_sync(uint8_t addr) {
    std::vector<uint8_t> pack = {0xAA, 0x55, 0x03, CMD_SET_SENSOR_CDC_SYNC, 0x00, 0x00, 0x00, 0x00, 0x00};
    calc_sum(pack);
    ch341_->write(addr, pack);
    return true;
}

bool SensorCmd::set_addr(uint8_t addr, uint8_t new_addr) {
    (void)addr; (void)new_addr; // 暂未实现
    return false;
}