#include "SerialPort.h"

std::map<std::string, std::shared_ptr<SerialPort>> SerialPort::ports_;

std::shared_ptr<SerialPort> SerialPort::create(const std::string& name, int baudrate) {
    auto it = ports_.find(name);
    if (it != ports_.end()) {
        return it->second;
    }
    auto port = std::shared_ptr<SerialPort>(new SerialPort(name, baudrate));
    ports_[name] = port;
    return port;
}

void SerialPort::openPhysicalPort() {
        fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port: " + port_name_);
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        // 配置串口参数
        speed_t speed = B38400;
        switch(baudrate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B38400; break;
        }
        
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            throw std::runtime_error("Failed to set serial port attributes");
        }
        
        tcflush(fd_, TCIFLUSH);
    }

// 发送数据
bool SerialPort::send(const std::vector<uint8_t>& data) {
    tcflush(fd_, TCOFLUSH);
    ssize_t bytes_written = write(fd_, data.data(), data.size());
    if (bytes_written != data.size()) {
        tcflush(fd_, TCOFLUSH);
        return false;
    }
    
    tcdrain(fd_);
    return true;
}

// 接收数据
std::vector<uint8_t> SerialPort::receive(int timeout_ms) {
    std::vector<uint8_t> data;
    
    usleep(timeout_ms * 1000);
    
    uint8_t buffer[256];
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
    
    if (bytes_read > 0) {
        data.assign(buffer, buffer + bytes_read);
    }

    return data;
}


// 清空输入缓冲区
void SerialPort::flushInput() {  //read
    tcflush(fd_, TCIFLUSH);
}

// 清空输出缓冲区
void SerialPort::flushOutput() {//write
    tcflush(fd_, TCOFLUSH);
}