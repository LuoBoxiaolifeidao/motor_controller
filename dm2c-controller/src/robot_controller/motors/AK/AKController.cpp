#include "AKController.h"
#include <iostream>  // 使用标准输出替代ROS日志
#include <cstring>

AKController::AKController(const std::string& interface, uint32_t bitrate)
    : can_comm_(interface) {
    
    // 初始化CAN ID
    set_pos_spd_acc_canid[1] = 0x0668;
    set_pos_spd_acc_canid[2] = 0x0004;
    set_current_canid[1] = 0x0168;
    set_current_canid[2] = 0x0001;
    set_zero_canid[1] = 0x0568;
    set_zero_canid[2] = 0x0002;
    get_status_canid[1] = 0x2968;
    get_status_canid[2] = 0x0006;
    
    // 索引0不使用
    set_pos_spd_acc_canid[0] = 0;
    set_current_canid[0] = 0;
    set_zero_canid[0] = 0;
    get_status_canid[0] = 0;
    can_comm_.init(bitrate, { get_status_canid[1],  get_status_canid[2]});
}

AKController::~AKController() {
    can_comm_.close();
}

bool AKController::moveToAbsolutePosition(int id, int32_t target_position, 
                                          int16_t speed, int16_t acceleration) {
    // 参数检查
    if (id != 1 && id != 2) {
        std::cerr << "[ERROR] 无效的电机ID: " << id << "，只支持1或2" << std::endl;
        return false;
    }
    
    if (target_position < -36000 || target_position > 36000) {
        std::cerr << "[ERROR] 位置超出范围(-36000, 36000): " << target_position << std::endl;
        return false;
    }
    
    // 数据打包
    uint8_t can_buffer[8] = {0}; 
    int32_t pos_scaled = static_cast<int32_t>(target_position * 10000.0f);
    int16_t spd_scaled = static_cast<int16_t>(speed / 10.0f);
    int16_t acc_scaled = static_cast<int16_t>(acceleration / 10.0f);

    can_buffer[0] = (pos_scaled >> 24) & 0xFF; 
    can_buffer[1] = (pos_scaled >> 16) & 0xFF; 
    can_buffer[2] = (pos_scaled >> 8)  & 0xFF;  
    can_buffer[3] = (pos_scaled >> 0)  & 0xFF;  
    can_buffer[4] = (spd_scaled >> 8) & 0xFF;  
    can_buffer[5] = (spd_scaled >> 0) & 0xFF;  
    can_buffer[6] = (acc_scaled >> 8) & 0xFF;   
    can_buffer[7] = (acc_scaled >> 0) & 0xFF;   

    // 发送CAN帧
    bool send_ok = can_comm_.sendFrame(set_pos_spd_acc_canid[id], can_buffer, 8);

    return send_ok;
}   

bool AKController::setCurrentPositionAsZero(int id) {
    if (id != 1 && id != 2) {
        std::cerr << "[ERROR] 无效的电机ID: " << id << "，只支持1或2" << std::endl;
        return false;
    }
    
    uint8_t can_buffer[8] = {1}; 
    bool send_ok = can_comm_.sendFrame(set_zero_canid[id], can_buffer, 8);

    if (send_ok) {
        std::cout << "[INFO] 电机" << id << " 零点设置成功" << std::endl;
    } else {
        std::cerr << "[WARN] 电机" << id << " 零点设置失败" << std::endl;
    }
    return send_ok;  
}

bool AKController::setCurrent(int id, float current) {
    if (id != 1 && id != 2) {
        return false;
    }

    uint8_t can_buffer[8] = {0};
    int32_t current_scaled = static_cast<int32_t>(current * (0x0FA0 / 4.0f));
    can_buffer[0] = (current_scaled >> 24) & 0xFF;
    can_buffer[1] = (current_scaled >> 16) & 0xFF;
    can_buffer[2] = (current_scaled >> 8)  & 0xFF;
    can_buffer[3] = (current_scaled >> 0)  & 0xFF;

    bool send_ok = can_comm_.sendFrame(set_current_canid[id], can_buffer, 8);
    return send_ok;
}

bool AKController::readCanState(){
    CANFrame frame;
    ros::Rate rate(10);
    int idx = 0;
    while (ros::ok()) {
        if(can_comm_.receive(frame, 10)){
            if (frame.can_id == get_status_canid[1]) idx = 1;
            else if (frame.can_id == get_status_canid[2]) idx = 2;
            else continue;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                int16_t pos_int = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
				if(static_cast<float>(pos_int) * 0.1f == -2816.0)
                	continue;
                position[idx] = static_cast<float>(pos_int) * 0.1f;
                //std::cout<<" ak pos="<<position[idx]<<std::endl;

                int16_t spd_int = (static_cast<int16_t>(frame.data[2]) << 8) | frame.data[3];
                speed[idx] = static_cast<float>(spd_int) * 10.0f;
                int16_t cur_int = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];
                current[idx] = static_cast<float>(cur_int) * 0.01f;

                temperature[idx] = static_cast<int8_t>(frame.data[6]);
                errorcode[idx] = frame.data[7];
            }
        }
        rate.sleep();
    }
    return true;
}


