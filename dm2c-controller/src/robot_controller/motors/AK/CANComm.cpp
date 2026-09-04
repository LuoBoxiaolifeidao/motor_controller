#include "CANComm.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstdio>

CANComm::CANComm(const std::string& interface)
    : can_socket_(-1), interface_(interface), is_initialized_(false) {}

CANComm::~CANComm() {
    close();
}



bool CANComm::init(uint32_t bitrate, vector<uint32_t> get_status_canid) {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("Failed to get CAN interface: " + interface_);
        close();
        return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        throw std::runtime_error("Failed to bind socket");
        close();
        return false;
    }
    if (get_status_canid.size() > 0) {
        // 分配 filter 数组
    	int num_filters = get_status_canid.size();
    	struct can_filter *rfilter = new struct can_filter[num_filters];
    
    	for (int i = 0; i < num_filters; i++) {
        	rfilter[i].can_id   = get_status_canid[i];    // 你要接收的 ID
        	rfilter[i].can_mask = CAN_EFF_MASK;           // 精确匹配扩展帧 ID
    	}
    
    	// 应用 filter
    	if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, 
        	           rfilter, sizeof(struct can_filter) * num_filters) < 0) {
        	delete[] rfilter;
        	throw std::runtime_error("Failed to set CAN filter");
    	}
    
    	delete[] rfilter;
    }
    is_initialized_ = true;
    return true;
}

bool CANComm::receive(CANFrame& frame, int timeout_ms) {
    if (!is_initialized_) {
        return false;
    }
    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 接收CAN帧
    struct can_frame can_frame;
    ssize_t reL_cv_len = read(can_socket_, &can_frame, sizeof(can_frame));
    if (reL_cv_len != sizeof(can_frame)) {
        // 超时或接收失败
        return false;
    }

    // 填充自定义CANFrame
    frame.can_id = can_frame.can_id & CAN_EFF_MASK;  // 过滤扩展帧标志
    frame.dlc = can_frame.can_dlc;
    memcpy(frame.data, can_frame.data, 8);

    return true;
}

bool CANComm::sendFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (!is_initialized_) {
        return false;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    ssize_t bytes_sent = write(can_socket_, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        ROS_WARN("[CANComm] send faild:%s | size:%zd", strerror(errno), bytes_sent);
        return false;
    }

    return true;
}

void CANComm::close() {
    if (can_socket_ >= 0) {
        ::close(can_socket_);
        can_socket_ = -1;
        is_initialized_ = false;
        ROS_INFO("[CANComm] socket cloed");
    }
}

bool CANComm::isInitialized() const { return is_initialized_; }
void CANComm::setInterface(const std::string& interface) {
    if (is_initialized_) close();
    interface_ = interface;
}
std::string CANComm::getInterface() const { return interface_; }
