#ifndef TACTILE_SENSOR_DRIVER__FINGER_HPP_
#define TACTILE_SENSOR_DRIVER__FINGER_HPP_

#include <vector>
#include <memory>
#include <chrono>
#include <ros/ros.h>
#include "tactile_sensor_driver/sensor_para.hpp"
#include "tactile_sensor_driver/ch341_driver.hpp"
#include "tactile_sensor_driver/sensor_cmd.hpp"

struct CapData {
    uint8_t sensor_index;
    std::vector<uint32_t> channel_cap_data;
    std::vector<float> tf;
    std::vector<uint16_t> tf_dir;
    std::vector<float> nf;
    std::vector<uint32_t> s_prox_cap_data;
    std::vector<uint32_t> m_prox_cap_data;

    void reset();
    void init(uint8_t addr, int ydds_num, int s_prox_num, int m_prox_num, int cap_channel_num);
    void deinit();
};

class Finger {
public:
    Finger(int pca_idx, std::shared_ptr<Ch341Driver> ch341);
    
    bool check_sensor();
    bool cap_read();
    void disconnected();
    void connected(uint8_t addr);
    void sync_sensor();

    // 获取解析后的数据
    const CapData& get_read_data() const { return read_data_; }
    bool is_connected() const { return connect_; }

private:
    uint32_t parse_uint24(const uint8_t* buf);
    uint32_t parse_uint32(const uint8_t* buf);

    std::unique_ptr<SensorCmd> sns_cmd_; // 驱动命令类
    int pca_idx_;
    
    CapData read_data_;
    FingerParamTS project_para_; // 当前手指的参数配置
    
    uint8_t addr_;
    bool connect_;
    uint8_t pack_idx_;
    std::chrono::steady_clock::time_point connect_timer_;
    std::vector<uint8_t> raw_buffer_;
};

#endif  // TACTILE_SENSOR_DRIVER__FINGER_HPP_