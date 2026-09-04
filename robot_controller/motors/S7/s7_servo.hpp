/**
 * S7舵机C++控制库
 * 适用于RK3568 + Ubuntu 18.04 + RS485接口
 *
 * 编译: g++ -o s7_servo_test main.cpp s7_servo.cpp -std=c++11 -lpthread
 * 运行: sudo ./s7_servo_test
 */

#ifndef S7_SERVO_HPP
#define S7_SERVO_HPP

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <algorithm>

// ============================================================
// 功能码定义
// ============================================================
#define FUNC_PING           0x01    // Ping检测
#define FUNC_READ           0x02    // 读数据
#define FUNC_WRITE          0x03    // 写数据
#define FUNC_REG_WRITE      0x04    // 寄存器写(暂不使能)
#define FUNC_REG_ACTION     0x05    // 寄存器使能
#define FUNC_FACTORY_RESET  0x06    // 恢复出厂设置
#define FUNC_SET_MIDDLE     0x07    // 设置中点
#define FUNC_CLEAR_MIDDLE   0x08    // 清除中点
#define FUNC_SYNC_WRITE     0x83    // 同步写(多舵机)

// ============================================================
// 控制表地址 (RAM区 - 掉电后数据复位)
// ============================================================
#define ADDR_TORQUE_ENABLE       24      // 力矩使能 (0=失能, 1=使能)
#define ADDR_LED                 25      // LED开关
#define ADDR_PID_D               26      // PID微分增益
#define ADDR_PID_I               27      // PID积分增益
#define ADDR_PID_P               28      // PID比例增益
#define ADDR_TARGET_POS_L        30      // 目标位置低8位
#define ADDR_TARGET_POS_H        31      // 目标位置高8位
#define ADDR_MOVING_SPEED_L      32      // 运动速度低8位
#define ADDR_MOVING_SPEED_H      33      // 运动速度高8位
#define ADDR_PRESENT_POS_L       36      // 当前位置低8位
#define ADDR_PRESENT_POS_H       37      // 当前位置高8位
#define ADDR_PRESENT_SPEED_L     38      // 当前速度低8位
#define ADDR_PRESENT_SPEED_H     39      // 当前速度高8位
#define ADDR_PRESENT_VOLTAGE     42      // 当前电压 (0.1V单位)
#define ADDR_PRESENT_TEMP        43      // 当前温度 (摄氏度)
#define ADDR_PRESENT_CURRENT_L   68      // 当前电流低8位
#define ADDR_PRESENT_CURRENT_H   69      // 当前电流高8位
#define ADDR_TARGET_ACCEL        73      // 目标加速度

// ============================================================
// 控制表地址 (Flash区 - 掉电后数据保存)
// ============================================================
#define ADDR_MODEL_L             0       // 舵机型号低8位
#define ADDR_MODEL_H             1       // 舵机型号高8位
#define ADDR_FIRMWARE_VERSION    2       // 固件版本
#define ADDR_ID                  3       // 舵机ID
#define ADDR_BAUDRATE            4       // 波特率
#define ADDR_MIN_ANGLE_L         6       // 可旋转角度下限低8位
#define ADDR_MIN_ANGLE_H         7       // 可旋转角度下限高8位
#define ADDR_MAX_ANGLE_L         8       // 可旋转角度上限低8位
#define ADDR_MAX_ANGLE_H         9       // 可旋转角度上限高8位
#define ADDR_TEMP_LIMIT          11      // 温度上限
#define ADDR_VOLT_MIN            12      // 电压下限 (0.1V单位)
#define ADDR_VOLT_MAX            13      // 电压上限 (0.1V单位)
#define ADDR_MAX_CURRENT_L       14      // 最大电流低8位
#define ADDR_MAX_CURRENT_H       15      // 最大电流高8位
#define ADDR_ANGLE_OFFSET_L      20      // 角度偏移低8位
#define ADDR_ANGLE_OFFSET_H      21      // 角度偏移高8位

// ============================================================
// 状态码定义
// ============================================================
#define STATUS_OK           0x00    // 正常
#define ERR_INSTRUCTION     0x40    // 指令功能码有误
#define ERR_OVERLOAD        0x20    // 过载
#define ERR_CHECKSUM        0x10    // 校验和有误
#define ERR_TEMP_OVER       0x04    // 温度过高
#define ERR_ANGLE_LIMIT     0x02    // 角度超出范围

// ============================================================
// 参数范围定义
// ============================================================
#define POSITION_MAX        4095    // 最大位置值 (0-4095对应0-360度)
#define SPEED_MAX           1023    // 最大速度值 (0-1023对应0-60RPM)
#define ACCEL_MAX           255     // 最大加速度值

class S7Servo {
private:
    int fd_;                                    // 串口文件描述符
    int servo_id_;                              // 舵机ID
    std::mutex tx_mutex_;                       // 发送锁
    std::atomic<bool> connected_{false};         // 连接状态

    // 计算校验和 (除0xFF开头外所有字节之和取反)
    uint8_t calcChecksum(const std::vector<uint8_t>& data);

    // 发送数据包
    bool sendPacket(const std::vector<uint8_t>& packet);

    // 接收数据包
    bool receivePacket(std::vector<uint8_t>& packet, int timeout_ms = 100);

    // 组合16位数据 (小端序)
    uint16_t combine16(uint8_t low, uint8_t high);
public:
    /**
     * 构造函数
     */
    S7Servo();

    /**
     * 析构函数
     */
    ~S7Servo();

    /**
     * 初始化串口连接
     * @param port 串口设备路径 (如 "/dev/ttyS0", "/dev/ttyUSB0")
     * @param baudrate 波特率 (默认115200, RK3568标准串口波特率)
     * @param servo_id 舵机ID (默认1)
     * @return 成功返回true
     */
    bool init(const char* port, int baudrate = 115200, int servo_id = 1);

    /**
     * 关闭连接
     */
    void close();

    /**
     * 检查连接状态
     */
    bool isConnected() const { return connected_; }

    /**
     * Ping舵机 (检测连接)
     */
    bool ping();

    /**
     * 读数据
     * @param addr 控制表地址
     * @param len 读取字节数
     * @param data 读取到的数据
     */
    bool readData(uint8_t addr, uint8_t len, std::vector<uint8_t>& data);

    /**
     * 写数据
     * @param addr 控制表地址
     * @param data 要写入的数据
     */
    bool writeData(uint8_t addr, const std::vector<uint8_t>& data);

    // ============================================================
    // 力矩控制
    // ============================================================

    /**
     * 设置力矩使能状态
     * @param enable true=使能, false=失能
     */
    bool setTorqueEnable(bool enable);

    /**
     * 获取力矩使能状态
     */
    bool getTorqueEnable();

    // ============================================================
    // 位置控制
    // ============================================================

    /**
     * 设置目标位置 (0-4095)
     * @param position 目标位置值
     */
    bool setTargetPosition(uint16_t position);

    /**
     * 获取目标位置
     */
    uint16_t getTargetPosition();

    /**
     * 获取当前位置 (0-4095)
     */
    uint16_t getPresentPosition();
    std::vector<uint16_t> getPresentPosition(std::vector<int> ids);

    // ============================================================
    // 速度控制
    // ============================================================

    /**
     * 设置运动速度 (0-1023)
     * @param speed 速度值 (0=最快, 1023=最慢)
     */
    bool setMovingSpeed(uint16_t speed);

    /**
     * 获取运动速度
     */
    uint16_t getMovingSpeed();

    /**
     * 获取当前速度
     */
    uint16_t getPresentSpeed();
    std::vector<uint16_t> getPresentSpeed(std::vector<int> ids);

    // ============================================================
    // 加速度控制
    // ============================================================

    /**
     * 设置加速度 (0-255)
     * @param accel 加速度值 (0=不受控, 推荐254)
     */
    bool setAcceleration(uint8_t accel);

    /**
     * 获取加速度
     */
    uint8_t getAcceleration();

    // ============================================================
    // 状态监测
    // ============================================================

    /**
     * 获取当前电压 (0.1V单位)
     * @return 电压值 (如返回85表示8.5V)
     */
    uint8_t getPresentVoltage();

    /**
     * 获取当前温度 (摄氏度)
     */
    uint8_t getPresentTemp();

    /**
     * 获取当前电流
     */
    uint16_t getPresentCurrent();

    /**
     * 清错
     */
    bool clearError();

    // ============================================================
    // 便利转换函数 (静态)
    // ============================================================

    /**
     * 角度转位置
     * @param angle 角度 (0-360度)
     * @return 位置值 (0-4095)
     */
    static uint16_t angleToPosition(double angle);

    /**
     * 位置转角度
     * @param position 位置值 (0-4095)
     * @return 角度 (0-360度)
     */
    static double positionToAngle(uint16_t position);

    /**
     * 速度值转RPM
     * @param speed 速度值 (0-1023)
     * @return RPM (0-60)
     */
    static double speedToRPM(uint16_t speed);

    /**
     * RPM转速度值
     * @param rpm 转速 (0-60RPM)
     * @return 速度值 (0-1023)
     */
    static uint16_t rpmToSpeed(double rpm);

    // ============================================================
    // 综合控制函数
    // ============================================================

    /**
     * 使能舵机 (推荐使用此函数初始化)
     * @param speed 初始速度 (0-1023)
     * @param accel 初始加速度 (0-255, 推荐254)
     */
    bool enableServo(uint16_t speed = 100, uint8_t accel = 254);
    bool enableServo(std::vector<int>ids, uint16_t speed, uint8_t accel);
    /**
     * 失能舵机
     */
    bool disableServo();
    bool disableServo(std::vector<int>ids);
    /**
     * 移动到指定位置
     * @param position 目标位置 (0-4095)
     * @param speed 运动速度 (0=使用当前速度)
     */
    bool moveToPosition(uint16_t position, uint16_t speed = 0);

    /**
     * 移动到指定角度 (最常用的函数)
     * @param angle 目标角度 (0-360度)
     * @param rpm 运动转速 (0-60RPM)
     */
    bool moveToAngle(double angle, double rpm = 10);

    /**
     * 等待到达目标位置
     * @param timeout_ms 超时时间(毫秒)
     * @param tolerance 容差范围(位置值)
     */
    bool waitForTarget(int timeout_ms = 10000, uint16_t tolerance = 10);

    bool syncWriteData(uint8_t start_addr, uint8_t data_length, 
                  const std::vector<int>& servo_ids,
                  const std::vector<std::vector<uint8_t>>& write_data);

    /**
     * 同步设置多个舵机目标位置
     * 适用于关节协同运动
     * 
     * @param servo_ids 舵机ID列表
     * @param positions 目标位置列表 (0-4095)
     * @return 成功返回true
     */
    bool syncSetTargetPositions(const std::vector<int>& servo_ids,
                            const std::vector<uint16_t>& positions);

    /**
     * 同步设置多个舵机运动速度
     * 
     * @param servo_ids 舵机ID列表
     * @param speeds 速度值列表 (0-1023)
     * @return 成功返回true
     */
    bool syncSetMovingSpeeds(const std::vector<int>& servo_ids,
                            const std::vector<uint16_t>& speeds);

    /**
     * 同步设置多个舵机加速度
     * 
     * @param servo_ids 舵机ID列表
     * @param speeds 速度值列表 (0-1023)
     * @return 成功返回true
     */
    bool syncSetAccelerations(const std::vector<int>& servo_ids,
                              const std::array<double, 4>& accelerations);
};

#endif // S7_SERVO_HPP
