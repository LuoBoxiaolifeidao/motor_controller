/**
 * S7舵机C++控制库实现
 */
// #define DEBUG
#include "s7_servo.hpp"

S7Servo::S7Servo() : fd_(-1), servo_id_(1) {}

S7Servo::~S7Servo() {
    close();
}

// 计算校验和: 除前两个0xFF外所有字节之和取反
uint8_t S7Servo::calcChecksum(const std::vector<uint8_t>& data) {
    uint8_t sum = 0;
    for (size_t i = 0; i < data.size(); i++) {
        sum += data[i];
    }
    return ~sum;  // 取反
}

bool S7Servo::init(const char* port, int baudrate, int servo_id) {
    servo_id_ = servo_id;

    // 打开串口 (阻塞模式)
    fd_ = open(port, O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cerr << "[错误] 无法打开串口: " << port << std::endl;
        return false;
    }

    // 获取当前串口配置
    struct termios options;
    if (tcgetattr(fd_, &options) != 0) {
        std::cerr << "[错误] 获取串口属性失败" << std::endl;
        close();
        return false;
    }

    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8N1 模式 (8位数据, 无校验, 1位停止位)
    options.c_cflag &= ~PARENB;   // 无校验
    options.c_cflag &= ~CSTOPB;   // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8位数据位

    // 使能接收和本地模式
    options.c_cflag |= (CLOCAL | CREAD);

    // 关闭硬件流控
    options.c_cflag &= ~CRTSCTS;

    // 关闭软件流控
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 原始输入模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 关闭输出处理
    options.c_oflag &= ~OPOST;

    // 设置超时 (读超时)
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;  // 100ms超时

    // 应用配置
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        std::cerr << "[错误] 串口配置失败" << std::endl;
        close();
        return false;
    }

    // 刷新缓冲区
    tcflush(fd_, TCIOFLUSH);

    connected_ = true;
    std::cout << "[信息] 串口初始化成功: " << port << " @ " << baudrate << " bps" << std::endl;
    std::cout << "[信息] 舵机ID: " << servo_id_ << std::endl;

    return true;
}

void S7Servo::close() {
    if (fd_ >= 0) {
        ::close(fd_);  // 使用全局close函数
        fd_ = -1;
    }
    connected_ = false;
}

bool S7Servo::sendPacket(const std::vector<uint8_t>& packet) {
    if (fd_ < 0) return false;

    std::lock_guard<std::mutex> lock(tx_mutex_);

    // 打印发送的数据 (调试用)
    #ifdef DEBUG
    std::cout << "[发送] ";
    for (auto byte : packet) {
        printf("%02X ", byte);
    }
    std::cout << std::endl;
    #endif

    ssize_t written = write(fd_, packet.data(), packet.size());
    if (written != static_cast<ssize_t>(packet.size())) {
        std::cerr << "[错误] 发送数据失败" << std::endl;
        return false;
    }

    // 等待数据发送完成
    tcdrain(fd_);
    return true;
}

bool S7Servo::receivePacket(std::vector<uint8_t>& packet, int timeout_ms) {
    if (fd_ < 0) return false;

    packet.clear();

    fd_set read_fds;
    struct timeval timeout;

    uint8_t byte;
    int retries = 0;
    const int max_retries = 50;

    // 等待帧头 0xFF 0xFF
    while (retries < max_retries) {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = timeout_ms * 1000;

        int ret = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);
        if (ret <= 0) {
            retries++;
            continue;
        }

        if (read(fd_, &byte, 1) == 1) {
            if (byte == 0xFF) {
                packet.push_back(byte);
                if (read(fd_, &byte, 1) == 1 && byte == 0xFF) {
                    packet.push_back(byte);
                    break;
                } else {
                    packet.clear();
                }
            }
        }
    }

    if (packet.size() < 2) {
        return false;
    }

    // 读取ID
    if (read(fd_, &byte, 1) != 1) return false;
    packet.push_back(byte);

    // 读取长度
    if (read(fd_, &byte, 1) != 1) return false;
    packet.push_back(byte);
    uint8_t length = byte;

    // 读取剩余数据 (length + 1 包含校验和)
    int remaining = length + 1;
    while (remaining > 0) {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = timeout_ms * 1000;

        if (select(fd_ + 1, &read_fds, NULL, NULL, &timeout) <= 0) {
            break;
        }

        if (read(fd_, &byte, 1) == 1) {
            packet.push_back(byte);
            remaining--;
        }
    }

    // 验证校验和
   // if (packet.size() >= 4) {
     //   std::vector<uint8_t> data_for_checksum(packet.begin() + 2, packet.end() - 1);
       // uint8_t expected_checksum = calcChecksum(data_for_checksum);
       // uint8_t received_checksum = packet.back();

       // if (expected_checksum != received_checksum) {
        //    std::cerr << "[错误] 校验和错误: 期望 0x" << std::hex << (int)expected_checksum
          //            << ", 收到 0x" << (int)received_checksum << std::dec << std::endl;
        //    return false;
 //       }
 //   }

    // 打印接收的数据 (调试用)
    #ifdef DEBUG
    std::cout << "[接收] ";
    for (auto byte : packet) {
        printf("%02X ", byte);
    }
    std::cout << std::endl;
    #endif

    return packet.size() >= 4;
}

uint16_t S7Servo::combine16(uint8_t low, uint8_t high) {
    return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
}

bool S7Servo::ping() {
    std::vector<uint8_t> packet = {
        0xFF, 0xFF,
        static_cast<uint8_t>(servo_id_),
        0x02,    // 长度
        FUNC_PING
    };
    packet.push_back(calcChecksum({packet.begin() + 2, packet.end()}));

    if (!sendPacket(packet)) return false;

    std::vector<uint8_t> response;
    if (!receivePacket(response)) return false;

    return response.size() >= 5 && response[4] == STATUS_OK;
}

bool S7Servo::readData(uint8_t addr, uint8_t len, std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet = {
        0xFF, 0xFF,
        static_cast<uint8_t>(servo_id_),
        0x04,              // 长度: 功能码+地址+长度+校验和
        FUNC_READ,
        addr,
        len
    };
    packet.push_back(calcChecksum({packet.begin() + 2, packet.end()}));

    if (!sendPacket(packet)) return false;

    std::vector<uint8_t> response;
    if (!receivePacket(response)) return false;

    // 解析应答: 0xFF 0xFF ID LEN ERROR DATA... CHECKSUM
    if (response.size() < 6) return false;

    data.clear();
    for (size_t i = 5; i < response.size() - 1; i++) {
        data.push_back(response[i]);
    }

    return response[4] == STATUS_OK;
}

bool S7Servo::writeData(uint8_t addr, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet = {
        0xFF, 0xFF,
        static_cast<uint8_t>(servo_id_),
        static_cast<uint8_t>(data.size() + 3),  // 长度: 地址+数据+2
        FUNC_WRITE,
        addr
    };
    packet.insert(packet.end(), data.begin(), data.end());
    packet.push_back(calcChecksum({packet.begin() + 2, packet.end()}));

    if (!sendPacket(packet)) return false;

    std::vector<uint8_t> response;
    if (!receivePacket(response)) return false;

    return response.size() >= 5 && response[4] == STATUS_OK;
}

bool S7Servo::setTorqueEnable(bool enable) {
    return writeData(ADDR_TORQUE_ENABLE, {static_cast<uint8_t>(enable ? 1 : 0)});
}

bool S7Servo::getTorqueEnable() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_TORQUE_ENABLE, 1, data)) return false;
    return data[0] == 1;
}

bool S7Servo::setTargetPosition(uint16_t position) {
    position = std::min(position, static_cast<uint16_t>(POSITION_MAX));
    return writeData(ADDR_TARGET_POS_L, {
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF)
    });
}

uint16_t S7Servo::getTargetPosition() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_TARGET_POS_L, 2, data)) return 0;
    return combine16(data[0], data[1]);
}

bool S7Servo::setMovingSpeed(uint16_t speed) {
    speed = std::min(speed, static_cast<uint16_t>(SPEED_MAX));
    return writeData(ADDR_MOVING_SPEED_L, {
        static_cast<uint8_t>(speed & 0xFF),
        static_cast<uint8_t>((speed >> 8) & 0xFF)
    });
}

uint16_t S7Servo::getMovingSpeed() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_MOVING_SPEED_L, 2, data)) return 0;
    return combine16(data[0], data[1]);
}

bool S7Servo::setAcceleration(uint8_t accel) {
    accel = std::min(accel, static_cast<uint8_t>(ACCEL_MAX));
    return writeData(ADDR_TARGET_ACCEL, {accel});
}

uint8_t S7Servo::getAcceleration() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_TARGET_ACCEL, 1, data)) return 0;
    return data[0];
}

uint16_t S7Servo::getPresentPosition() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_PRESENT_POS_L, 2, data)) return 0;
    return combine16(data[0], data[1]);
}

uint16_t S7Servo::getPresentSpeed() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_PRESENT_SPEED_L, 2, data)) return 0;
    return combine16(data[0], data[1]);
}

std::vector<uint16_t> S7Servo::getPresentPosition(std::vector<int> ids) {
    std::vector<uint8_t> data;
    std::vector<uint16_t> ans;
    auto tmpFd = servo_id_;
    for(auto id : ids){
        servo_id_ = id;
        if (!readData(ADDR_PRESENT_POS_L, 2, data))
            ans.push_back(0);
        else
            ans.push_back(combine16(data[0], data[1]));
    }
    servo_id_ = tmpFd;
    return ans;
}

std::vector<uint16_t> S7Servo::getPresentSpeed(std::vector<int> ids) {
    std::vector<uint8_t> data;
    std::vector<uint16_t> ans;
    auto tmpFd = servo_id_;
    for(auto id : ids){
	std::cout<<id;
        servo_id_ = id;
        if (!readData(ADDR_PRESENT_SPEED_L, 2, data))
            ans.push_back(0);
        else
            ans.push_back(combine16(data[0], data[1]));
    }
    servo_id_ = tmpFd;
    return ans;
}


uint8_t S7Servo::getPresentVoltage() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_PRESENT_VOLTAGE, 1, data)) return 0;
    return data[0];
}

uint8_t S7Servo::getPresentTemp() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_PRESENT_TEMP, 1, data)) return 0;
    return data[0];
}

uint16_t S7Servo::getPresentCurrent() {
    std::vector<uint8_t> data;
    if (!readData(ADDR_PRESENT_CURRENT_L, 2, data)) return 0;
    uint16_t current = combine16(data[0], data[1]);
    // 2048代表0电流
    return (current >= 2048) ? (current - 2048) : 0;
}

// 静态转换函数
uint16_t S7Servo::angleToPosition(double angle) {
    angle = std::max(0.0, std::min(360.0, angle));
    return static_cast<uint16_t>(angle / 360.0 * POSITION_MAX);
}

double S7Servo::positionToAngle(uint16_t position) {
    return position / static_cast<double>(POSITION_MAX) * 360.0;
}

double S7Servo::speedToRPM(uint16_t speed) {
    return speed / static_cast<double>(SPEED_MAX) * 60.0;
}

uint16_t S7Servo::rpmToSpeed(double rpm) {
    rpm = std::max(0.0, std::min(60.0, rpm));
    return static_cast<uint16_t>(rpm / 60.0 * SPEED_MAX);
}

bool S7Servo::enableServo(uint16_t speed, uint8_t accel) {
    // 步骤1: 先失能 (修改Flash区前必须先失能)
    if (!setTorqueEnable(false)) {
        std::cerr << "[警告] 失能失败" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 步骤2: 设置速度
    if (!setMovingSpeed(speed)) {
        std::cerr << "[警告] 设置速度失败" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 步骤3: 设置加速度
    if (!setAcceleration(accel)) {
        std::cerr << "[警告] 设置加速度失败" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 步骤4: 使能
    if (!setTorqueEnable(true)) {
        std::cerr << "[错误] 使能失败" << std::endl;
        return false;
    }

    std::cout << "[信息] 舵机使能成功 (速度:" << speed << " 加速度:" << (int)accel << ")" << std::endl;
    return true;
}

bool S7Servo::enableServo(std::vector<int>ids, uint16_t speed, uint8_t accel) {
    // 步骤1: 先失能 (修改Flash区前必须先失能)
    auto tmpid = servo_id_;
    for(auto id : ids){
        servo_id_ = id;
        
        std::cout<<"id = "<<servo_id_<<" enableServo"<<enableServo(speed, accel);
    }
    servo_id_ = tmpid;
    
}

bool S7Servo::disableServo() {
    return setTorqueEnable(false);
}

bool S7Servo::disableServo(std::vector<int>ids){
    auto tmpid = servo_id_;
    for(auto id : ids){
        servo_id_ = id;
        setTorqueEnable(false);
    }
    servo_id_ = tmpid;
}

bool S7Servo::moveToPosition(uint16_t position, uint16_t speed) {
    // 如果指定了新速度
    if (speed > 0) {
        if (!setMovingSpeed(speed)) {
            std::cerr << "[警告] 设置速度失败" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return setTargetPosition(position);
}

bool S7Servo::moveToAngle(double angle, double rpm) {
    uint16_t position = angleToPosition(angle);
    uint16_t speed = rpmToSpeed(rpm);

    #ifdef DEBUG
    std::cout << "[调试] 移动到 " << angle << "° (位置:" << position
              << " 速度:" << speed << " RPM:" << rpm << ")" << std::endl;
    #endif

    return moveToPosition(position, speed);
}

bool S7Servo::waitForTarget(int timeout_ms, uint16_t tolerance) {
    auto start_time = std::chrono::steady_clock::now();
    uint16_t target = getTargetPosition();

    while (true) {
        uint16_t current = getPresentPosition();

        #ifdef DEBUG
        std::cout << "[调试] 当前位置:" << current << " 目标:" << target << std::endl;
        #endif
        std::cout << "[调试] 当前位置:" << current << " 目标:" << target <<" 速度："<<getPresentSpeed()<< std::endl;
        // 检查是否到达目标 (带容差)
        if (abs(static_cast<int>(current) - static_cast<int>(target)) <= tolerance) {
            return true;
        }

        // 检查超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time
        ).count();

        if (elapsed >= timeout_ms) {
            std::cerr << "[警告] 等待目标位置超时" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}


bool S7Servo::syncWriteData(uint8_t start_addr, uint8_t data_length, 
                           const std::vector<int>& servo_ids,
                           const std::vector<std::vector<uint8_t>>& write_data) {
    if (fd_ < 0) {
        std::cerr << "[错误] 串口未连接" << std::endl;
        return false;
    }
    
    if (servo_ids.empty()) {
        std::cerr << "[错误] 舵机ID列表为空" << std::endl;
        return false;
    }
    
    if (servo_ids.size() != write_data.size()) {
        std::cerr << "[错误] ID数量与数据数量不匹配" << std::endl;
        return false;
    }
    
    // 检查所有舵机数据长度是否一致
    for (size_t i = 0; i < write_data.size(); i++) {
        if (write_data[i].size() != data_length) {
            std::cerr << "[错误] 舵机" << (int)servo_ids[i] 
                      << "的数据长度不匹配: 期望" << (int)data_length 
                      << "字节, 实际" << write_data[i].size() << "字节" << std::endl;
            return false;
        }
    }
    
    // 根据文档4.1.5节构建同步写数据包
    // 公式: 长度 = (N+1)*M+4, 其中N=写入每个舵机的参数个数, M=舵机个数
    uint8_t M = static_cast<uint8_t>(servo_ids.size());  // 舵机个数
    uint8_t N = data_length;                            // 每个舵机写入字节数
    uint8_t length = static_cast<uint8_t>((N + 1) * M + 4);
    
    // 开始构建数据包
    std::vector<uint8_t> packet;
    
    // 1. 包头
    packet.push_back(0xFF);
    packet.push_back(0xFF);
    
    // 2. 广播ID (必须为0xFE)
    packet.push_back(0xFE);
    
    // 3. 长度
    packet.push_back(length);
    
    // 4. 功能码 (同步写)
    packet.push_back(FUNC_SYNC_WRITE);
    
    // 5. 起始地址
    packet.push_back(start_addr);
    
    // 6. 写入字节个数
    packet.push_back(N);
    
    // 7. 添加每个舵机的数据
    for (size_t i = 0; i < M; i++) {
        // 舵机ID
        packet.push_back(servo_ids[i]);
        
        // 舵机数据
        for (uint8_t byte : write_data[i]) {
            packet.push_back(byte);
        }
    }
    
    // 8. 计算校验和
    uint8_t checksum = 0;
    for (size_t i = 2; i < packet.size(); i++) {
        checksum += packet[i];
    }
    checksum = ~checksum;  // 取反
    packet.push_back(checksum);
    
    #ifdef DEBUG
    std::cout << "[同步写] 发送" << (int)M << "个舵机, 起始地址: 0x" 
              << std::hex << (int)start_addr << std::dec << std::endl;
    std::cout << "  数据包: ";
    for (auto byte : packet) {
        printf("%02X ", byte);
    }
    std::cout << std::endl;
    #endif
    
    // 发送数据
    std::lock_guard<std::mutex> lock(tx_mutex_);
    ssize_t written = write(fd_, packet.data(), packet.size());
    
    if (written != static_cast<ssize_t>(packet.size())) {
        std::cerr << "[错误] 同步写发送失败" << std::endl;
        return false;
    }
    
    // 等待数据发送完成
    tcdrain(fd_);
    
    // 根据文档6.注意事项第3点, 同步写后需要至少100微秒间隔
    usleep(200);  // 200微秒, 略大于最小要求
    
    return true;
}

bool S7Servo::syncSetTargetPositions(const std::vector<int>& servo_ids,
                                    const std::vector<uint16_t>& positions) {
    if (servo_ids.size() != positions.size()) {
        std::cerr << "[错误] ID数量与位置数量不匹配" << std::endl;
        return false;
    }
    
    // 目标位置寄存器地址: 0x1E (低字节), 0x1F (高字节)
    const uint8_t TARGET_POS_ADDR = ADDR_TARGET_POS_L;
    const uint8_t DATA_LENGTH = 2;  // 2字节位置值
    
    // 准备每个舵机的数据
    std::vector<std::vector<uint8_t>> write_data;
    
    for (size_t i = 0; i < servo_ids.size(); i++) {
        uint16_t pos = positions[i];
        
        // 确保位置在有效范围内
        if (pos > POSITION_MAX) {
            std::cerr << "[警告] 舵机" << (int)servo_ids[i] 
                      << "位置超出范围: " << pos << ", 将限制为" << POSITION_MAX << std::endl;
            pos = POSITION_MAX;
        }
        
        // 位置值拆分为低字节和高字节
        std::vector<uint8_t> servo_data = {
            static_cast<uint8_t>(pos & 0xFF),        // 低字节
            static_cast<uint8_t>((pos >> 8) & 0xFF)  // 高字节
        };
        
        write_data.push_back(servo_data);
        
        #ifdef DEBUG
        std::cout << "  舵机ID " << std::setw(3) << (int)servo_ids[i] 
                  << ": 位置=" << std::setw(4) << pos << std::endl;
        #endif
    }
    
    return syncWriteData(TARGET_POS_ADDR, DATA_LENGTH, servo_ids, write_data);
}

bool S7Servo::syncSetMovingSpeeds(const std::vector<int>& servo_ids,
                                 const std::vector<uint16_t>& speeds) {
    if (servo_ids.size() != speeds.size()) {
        std::cerr << "[错误] ID数量与速度数量不匹配" << std::endl;
        return false;
    }
    
    // 运动速度寄存器地址: 0x20 (低字节), 0x21 (高字节)
    const uint8_t MOVING_SPEED_ADDR = ADDR_MOVING_SPEED_L;
    const uint8_t DATA_LENGTH = 2;  // 2字节速度值
    
    // 准备每个舵机的数据
    std::vector<std::vector<uint8_t>> write_data;
    
    for (size_t i = 0; i < servo_ids.size(); i++) {
        uint16_t speed = speeds[i];
        
        if (speed > SPEED_MAX) {
            speed = SPEED_MAX;
        }
        
        std::vector<uint8_t> servo_data = {
            static_cast<uint8_t>(speed & 0xFF),        // 低字节
            static_cast<uint8_t>((speed >> 8) & 0xFF)  // 高字节
        };
        
        write_data.push_back(servo_data);
    }
    
    return syncWriteData(MOVING_SPEED_ADDR, DATA_LENGTH, servo_ids, write_data);
}


bool S7Servo::syncSetAccelerations(const std::vector<int>& servo_ids,
                                   const std::array<double, 4>& accelerations) {
    if (servo_ids.size() != accelerations.size()) {
        std::cerr << "[错误] ID数量与加速度数量不匹配" << std::endl;
        return false;
    }
    
    // 加速度寄存器地址: 0x49 (低字节), 0x4A (高字节)
    const uint8_t ACCELERATION_ADDR = ADDR_TARGET_ACCEL;
    const uint8_t DATA_LENGTH = 2;  // 2字节加速度值
    
    // 准备每个舵机的数据
    std::vector<std::vector<uint8_t>> write_data;
    
    for (size_t i = 0; i < servo_ids.size(); i++) {
        uint16_t acc = accelerations[i];
  
        // 加速度值拆分为低字节和高字节
        std::vector<uint8_t> servo_data = {
            static_cast<uint8_t>(acc & 0xFF),        // 低字节
            static_cast<uint8_t>((acc >> 8) & 0xFF)  // 高字节
        };
        
        write_data.push_back(servo_data);
        
        #ifdef DEBUG
        std::cout << "  舵机ID " << std::setw(3) << (int)servo_ids[i] 
                  << ": 加速度=" << std::setw(3) << acc << std::endl;
        #endif
    }
    
    return syncWriteData(ACCELERATION_ADDR, DATA_LENGTH, servo_ids, write_data);
}

bool S7Servo::clearError() {
    // 假设错误寄存器地址为 0x24（请根据你的S7手册确认地址）
    const uint8_t ERROR_REGISTER_ADDR = 0x24;
    std::vector<uint8_t> clear_data = {0x00}; // 写入0以清除错误
    
    return writeData(ERROR_REGISTER_ADDR, clear_data);
}
