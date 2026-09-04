#pragma once

#include <string>
#include <cstdint>
#include <mutex>
#include <memory>
#include <map>
#include <vector>

class CANComMWD {
public:
    struct Frame {
        uint32_t can_id;
        uint8_t data[8];
    };

    static std::shared_ptr<CANComMWD> create(const std::string& interface, uint32_t bitrate = 1000000);

    ~CANComMWD();
    CANComMWD(const CANComMWD&) = delete;
    CANComMWD& operator=(const CANComMWD&) = delete;

    bool send(uint32_t can_id, const uint8_t* data);
    bool receive(Frame& frame, int timeout_ms = 50);

    bool transaction(uint32_t can_id, const uint8_t* send_data, uint8_t* recv_data, int timeout_ms = 50);

    int fd() const { return socket_; }

private:
    CANComMWD(const std::string& interface, uint32_t bitrate);
    bool open();

    int socket_ = -1;
    std::string interface_;
    uint32_t bitrate_;
    std::mutex mutex_;

    static std::map<std::string, std::shared_ptr<CANComMWD>> ports_;
};
