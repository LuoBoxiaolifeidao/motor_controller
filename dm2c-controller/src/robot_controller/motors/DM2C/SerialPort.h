#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <mutex> 
#include <shared_mutex>
#include <cstdint>
#include <stdexcept>
#include <atomic>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <cstring>
#include <sys/select.h>

class SerialPort {
private:
    int fd_;
    std::string port_name_;
    int baudrate_;
    static std::map<std::string, std::shared_ptr<SerialPort>> ports_;
    std::mutex mutex_; 
    
    void openPhysicalPort() ;
    
    SerialPort(const std::string& port_name, int baudrate) 
        : port_name_(port_name), baudrate_(baudrate) {
        openPhysicalPort();
    }

    SerialPort(const SerialPort&) = delete;
    SerialPort(SerialPort&&) noexcept = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort& operator=(SerialPort&&) noexcept = delete;
public:

    ~SerialPort() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    static std::shared_ptr<SerialPort> create(const std::string& name, int baudrate);

    void flushInput();
    void flushOutput();

    bool send(const std::vector<uint8_t>& data) ;
    std::vector<uint8_t> receive(int timeout_ms = 200) ;

    std::vector<uint8_t> transaction(const std::vector<uint8_t>& data, int timeout_ms = 200);

    std::vector<uint8_t> transactionNoFlush(const std::vector<uint8_t>& data, int timeout_ms = 50);

    std::vector<uint8_t> readBytes(int n, int timeout_ms);
    bool writeRaw(const std::vector<uint8_t>& data);

    int getFd() const { return fd_; }
};
