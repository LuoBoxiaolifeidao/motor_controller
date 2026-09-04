#include "CANComMWD.h"

#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

std::map<std::string, std::shared_ptr<CANComMWD>> CANComMWD::ports_;

std::shared_ptr<CANComMWD> CANComMWD::create(const std::string& interface, uint32_t bitrate) {
    auto it = ports_.find(interface);
    if (it != ports_.end()) return it->second;
    auto p = std::shared_ptr<CANComMWD>(new CANComMWD(interface, bitrate));
    ports_[interface] = p;
    return p;
}

CANComMWD::CANComMWD(const std::string& interface, uint32_t bitrate)
    : interface_(interface), bitrate_(bitrate) {
    if (!open()) throw std::runtime_error("CANComMWD: failed to open " + interface);
}

CANComMWD::~CANComMWD() {
    if (socket_ >= 0) { ::close(socket_); socket_ = -1; }
}

bool CANComMWD::open() {
    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(socket_); socket_ = -1; return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ::close(socket_); socket_ = -1; return false;
    }

    return true;
}

bool CANComMWD::send(uint32_t can_id, const uint8_t* data) {
    std::lock_guard<std::mutex> lock(mutex_);

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id & CAN_SFF_MASK;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    return ::write(socket_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANComMWD::receive(Frame& frame, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(socket_, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (::select(socket_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) return false;

    struct can_frame cframe;
    if (::read(socket_, &cframe, sizeof(cframe)) != sizeof(cframe)) return false;

    frame.can_id = cframe.can_id & CAN_SFF_MASK;
    std::memcpy(frame.data, cframe.data, 8);
    return true;
}

bool CANComMWD::transaction(uint32_t can_id, const uint8_t* send_data, uint8_t* recv_data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    struct timeval tv0 = {0, 0};
    fd_set rfds;

    while (true) {
        FD_ZERO(&rfds);
        FD_SET(socket_, &rfds);
        if (::select(socket_ + 1, &rfds, nullptr, nullptr, &tv0) <= 0) break;
        struct can_frame junk;
        ::read(socket_, &junk, sizeof(junk));
    }

    struct can_frame sframe;
    std::memset(&sframe, 0, sizeof(sframe));
    sframe.can_id = can_id & CAN_SFF_MASK;
    sframe.can_dlc = 8;
    std::memcpy(sframe.data, send_data, 8);

    if (::write(socket_, &sframe, sizeof(sframe)) != sizeof(sframe)) return false;

    tv0.tv_sec = timeout_ms / 1000;
    tv0.tv_usec = (timeout_ms % 1000) * 1000;

    FD_ZERO(&rfds);
    FD_SET(socket_, &rfds);
    if (::select(socket_ + 1, &rfds, nullptr, nullptr, &tv0) <= 0) return false;

    struct can_frame rframe;
    if (::read(socket_, &rframe, sizeof(rframe)) != sizeof(rframe)) return false;
    if ((rframe.can_id & CAN_SFF_MASK) != can_id) return false;

    std::memcpy(recv_data, rframe.data, 8);
    return true;
}
