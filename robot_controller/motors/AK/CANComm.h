#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <string>
#include <vector>
#include <cstdint>
#include <ros/ros.h>
using namespace std;
struct CANFrame {
    uint32_t can_id;    
    uint8_t data[8];    
    uint8_t dlc;       
};

class CANComm {
private:
    int can_socket_;
    std::string interface_;
    bool is_initialized_;
public:
    explicit CANComm(const std::string& interface = "can0");
    ~CANComm();

    bool init(uint32_t bitrate, vector<uint32_t> get_status_canid);
    bool sendFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
    bool receive(CANFrame& frame, int timeout_ms = 100);
    void close();
    bool isInitialized() const;
    void setInterface(const std::string& interface);
    std::string getInterface() const;
};

#endif // CAN_COMM_H
