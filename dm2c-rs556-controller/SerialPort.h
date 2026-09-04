#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <cstdint>
#include <stdexcept>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <cstring>

class SerialPort {
private:
    int fd_;
    std::string port_name_;
    int baudrate_;
    static std::map<std::string, std::shared_ptr<SerialPort>> ports_;
    
    // 串口配置
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
    
    // 工厂方法创建，避免同一个串口重复创建
    static std::shared_ptr<SerialPort> create(const std::string& name, int baudrate);

    // 清空输入缓冲区
    void flushInput();
    
    // 清空输出缓冲区
    void flushOutput();

    // 发送数据
    bool send(const std::vector<uint8_t>& data) ;
    
    // 接收数据
    std::vector<uint8_t> receive(int timeout_ms = 200) ;

};
