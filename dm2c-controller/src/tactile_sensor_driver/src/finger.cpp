#include "tactile_sensor_driver/finger.hpp"
#include <cstring> // for memcpy

// --- CapData 实现 ---
void CapData::reset() {
    sensor_index = 0;
    channel_cap_data.clear();
    tf.clear();
    tf_dir.clear();
    nf.clear();
    s_prox_cap_data.clear();
    m_prox_cap_data.clear();
}

void CapData::init(uint8_t addr, int ydds_num, int s_prox_num, int m_prox_num, int cap_channel_num) {
    sensor_index = addr;
    channel_cap_data.assign(cap_channel_num, 0);
    tf.assign(ydds_num, 0.0f);
    tf_dir.assign(ydds_num, 0);
    nf.assign(ydds_num, 0.0f);
    s_prox_cap_data.assign(s_prox_num, 0);
    m_prox_cap_data.assign(m_prox_num, 0);
}

void CapData::deinit() {
    reset();
}

// --- Finger 实现 ---
Finger::Finger(int pca_idx, std::shared_ptr<Ch341Driver> ch341)
    : sns_cmd_(std::make_unique<SensorCmd>(ch341)),
      pca_idx_(pca_idx),
      project_para_(SensorPara::finger_params[0]) // 默认使用第一个参数
{
    disconnected();
}

void Finger::disconnected() {
    addr_ = 0xFF;
    connect_ = false;
    pack_idx_ = 0;
    read_data_.deinit();
}

void Finger::connected(uint8_t addr) {
    addr_ = addr;
    connect_ = true;
    connect_timer_ = std::chrono::steady_clock::now();
    pack_idx_ = 0;
    
    // 初始化原始数据缓冲区
    raw_buffer_.assign(project_para_.pack_len, 0);
    
    read_data_.init(addr, 
                    project_para_.ydds_num,
                    project_para_.s_prox_num,
                    project_para_.m_prox_num,
                    project_para_.sensor_num);
}

bool Finger::check_sensor() {
    uint8_t addr_read = sns_cmd_->get_addr(0);
    if (addr_read == 0) return false;

    sns_cmd_->set_sensor_send_type(addr_read, 0);
    sns_cmd_->set_sensor_cap_offset(addr_read, addr_read);

    int project_read = sns_cmd_->get_sensor_project_index(addr_read);
    ROS_INFO("Detected Project ID: %d", project_read);
    
    bool found = false;
    for (const auto& pro : SensorPara::finger_params) {
        if (pro.prg == project_read) {
            project_para_ = pro; // C++ 结构体直接赋值是深拷贝
            ROS_INFO("Finger Connected: %s", project_para_.name.c_str());
            found = true;
            break;
        }
    }
    
    if (!found) {
        ROS_WARN("Project not found, using default parameters");
    }
            
    connected(addr_read);
    return true;
}

uint32_t Finger::parse_uint24(const uint8_t* buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
}

uint32_t Finger::parse_uint32(const uint8_t* buf) {
    uint32_t val;
    std::memcpy(&val, buf, 4);
    return val;
}

bool Finger::cap_read() {
    if (!connect_) return false;
    
    bool rcv_flag = false;
    for (int retry = 0; retry < 3; ++retry) {
        if (sns_cmd_->get_sensor_cap_data(addr_, raw_buffer_)) {
            // data[4] 是包序号
            if (raw_buffer_[4] != pack_idx_) {
                pack_idx_ = raw_buffer_[4];
                connect_timer_ = std::chrono::steady_clock::now();
                
                int cap_byte = project_para_.cap_byte;
                int sens_num = project_para_.sensor_num;
                
                // 1. 解析原始电容数据
                for (int j = 0; j < sens_num; ++j) {
                    int base = 6 + j * cap_byte;
                    if (cap_byte == 4) {
                        read_data_.channel_cap_data[j] = parse_uint32(&raw_buffer_[base]);
                    } else {
                        read_data_.channel_cap_data[j] = parse_uint24(&raw_buffer_[base]);
                    }
                }

                // 2. 解析力数据 (YDDS)
                int ydds_offset = 6 + sens_num * cap_byte;
                int ydds_num = project_para_.ydds_num;
                
                if (project_para_.ydds_type == 2) {
                    for (int i = 0; i < ydds_num; ++i) {
                        int start = ydds_offset + i * sizeof(DynamicYddsComTs);
                        DynamicYddsComTs inst;
                        std::memcpy(&inst, &raw_buffer_[start], sizeof(DynamicYddsComTs));
                        
                        read_data_.nf[i] = inst.nf;
                        read_data_.tf[i] = inst.tf;
                        read_data_.tf_dir[i] = inst.tfDir;
                        read_data_.s_prox_cap_data[i] = inst.prox;
                    }
                } 
                else if (project_para_.ydds_type == 4) {
                    int sz = sizeof(DynamicYddsU16Ts);
                    for (int i = 0; i < ydds_num; ++i) {
                        int start = ydds_offset + i * sz;
                        DynamicYddsU16Ts inst;
                        std::memcpy(&inst, &raw_buffer_[start], sz);
                        
                        read_data_.nf[i] = inst.nf / 100.0f;
                        read_data_.tf[i] = inst.tf / 100.0f;
                        read_data_.tf_dir[i] = inst.tfDir;
                    }
                    
                    int s_prox_off = ydds_offset + ydds_num * sz;
                    for (int i = 0; i < project_para_.s_prox_num; ++i) {
                        read_data_.s_prox_cap_data[i] = parse_uint24(&raw_buffer_[s_prox_off + i * cap_byte]);
                    }

                    int m_prox_off = s_prox_off + project_para_.s_prox_num * cap_byte;
                    for (int i = 0; i < project_para_.m_prox_num; ++i) {
                        read_data_.m_prox_cap_data[i] = parse_uint24(&raw_buffer_[m_prox_off + i * cap_byte]);
                    }
                }
                rcv_flag = true;
            }
            break; 
        }
    }

    // 检查超时 (2秒)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - connect_timer_).count();
    if (elapsed > 2) {
        ROS_WARN("Sensor Timeout: addr=%d", addr_);
        disconnected();
    }
            
    return rcv_flag;
}

void Finger::sync_sensor() {
    if (connect_) {
        sns_cmd_->set_sensor_sync(addr_);
    }
}