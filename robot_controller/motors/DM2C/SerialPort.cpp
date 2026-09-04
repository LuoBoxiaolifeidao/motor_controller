#include "SerialPort.h"
#include <sys/select.h>
#include <sys/time.h>

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
        fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port: " + port_name_);
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
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

bool SerialPort::send(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    tcflush(fd_, TCOFLUSH);
    ssize_t bytes_written = write(fd_, data.data(), data.size());
    if (bytes_written != data.size()) {
        tcflush(fd_, TCOFLUSH);
        return false;
    }
    
    tcdrain(fd_);
    return true;
}

std::vector<uint8_t> SerialPort::receive(int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> data;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) return data;
    
    uint8_t buffer[256];
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        data.assign(buffer, buffer + bytes_read);
    }

    return data;
}


std::vector<uint8_t> SerialPort::transaction(const std::vector<uint8_t>& data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    tcflush(fd_, TCIOFLUSH);
    ssize_t bytes_written = write(fd_, data.data(), data.size());
    if (bytes_written != data.size()) {
        tcflush(fd_, TCOFLUSH);
        return {};
    }
    tcdrain(fd_);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    std::vector<uint8_t> response;
    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) return response;

    uint8_t buffer[256];
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        response.assign(buffer, buffer + bytes_read);
    }
    return response;
}

std::vector<uint8_t> SerialPort::transactionNoFlush(const std::vector<uint8_t>& data, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    tcflush(fd_, TCIFLUSH);
    ssize_t bytes_written = write(fd_, data.data(), data.size());
    if (bytes_written != data.size()) {
        return {};
    }
    tcdrain(fd_);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    std::vector<uint8_t> response;
    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) return response;

    uint8_t buffer[256];
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        response.assign(buffer, buffer + bytes_read);
    }
    return response;
}

void SerialPort::flushInput() {
    std::lock_guard<std::mutex> lock(mutex_);
    tcflush(fd_, TCIFLUSH);
}

void SerialPort::flushOutput() {
    std::lock_guard<std::mutex> lock(mutex_);
    tcflush(fd_, TCOFLUSH);
}

bool SerialPort::writeRaw(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    ssize_t bytes_written = write(fd_, data.data(), data.size());
    return bytes_written == static_cast<ssize_t>(data.size());
}

std::vector<uint8_t> SerialPort::readBytes(int n, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint8_t> data;
    auto start = std::chrono::steady_clock::now();

    while (static_cast<int>(data.size()) < n) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) break;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        struct timeval tv;
        int remaining = timeout_ms - elapsed;
        tv.tv_sec = 0;
        tv.tv_usec = std::min(remaining * 1000, 50000);

        int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret <= 0) continue;

        uint8_t buf[256];
        ssize_t bytes_read = read(fd_, buf, sizeof(buf));
        if (bytes_read > 0)
            data.insert(data.end(), buf, buf + bytes_read);
    }
    return data;
}