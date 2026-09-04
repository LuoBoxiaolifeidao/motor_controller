#include "DM2CController.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

int main() {

    std::string port = "/dev/ttysWK1";
    int baudrate = 38400;
    uint8_t slave_id = 2;
    
    std::cout << "初始化DM2C控制器..." << std::endl;
    std::cout << "串口: " << port << std::endl;
    std::cout << "波特率: " << baudrate << std::endl;
    std::cout << "从站ID: " << (int)slave_id << std::endl;
    DM2CController controller(LiftColumn::RightArm, port, baudrate);
    controller.stopAllMotion();

    DM2CController controller1(LiftColumn::LeftArm, port, baudrate);
    controller1.stopAllMotion();

    return 0;
}
