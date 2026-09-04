#include "DM2CController.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <cstdlib>

std::atomic<bool> stop_requested{false};

void signalHandler(int) {
    stop_requested = true;
}
 
void moveBlocking(DM2CController& ctrl, int32_t dist, int16_t speed,
                  uint16_t accel, uint16_t decel, uint32_t tolerance = 200) {
    ctrl.moveToRelativePosition(dist, speed, accel, decel);
    // 等 readActualPosition 先更新出非零值，确保正在运动
    while (!stop_requested && ctrl.readActualPosition() == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uint32_t prev = ctrl.readActualPosition();
    while (!stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        uint32_t cur = ctrl.readActualPosition();
        if (cur == 0 || std::abs((int)cur - (int)prev) < tolerance)
            break; // 到位或已停止
        prev = cur;
    }
}



int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <LeftArm|RightArm> <speed>" << std::endl;
        return 1;
    }

    std::string side_str = argv[1];
    LiftColumn side;
    if (side_str == "LeftArm") {
        side = LiftColumn::LeftArm;
    } else if (side_str == "RightArm") {
        side = LiftColumn::RightArm;
    } else {
        std::cerr << "Invalid motor name. Use LeftArm or RightArm." << std::endl;
        return 1;
    }

    int16_t speed = static_cast<int16_t>(std::atoi(argv[2]));

    std::string port = "/dev/ttyACM0";
    int baudrate = 38400;

    std::cout << "初始化DM2C控制器..." << std::endl;
    std::cout << "串口: " << port << std::endl;
    std::cout << "波特率: " << baudrate << std::endl;
    std::cout << "电机: " << side_str << " (ID=" << (int)side << ")" << std::endl;
    std::cout << "速度: " << (int)speed << " RPM" << std::endl;
    std::cout << "按 Ctrl+C 停止" << std::endl;

    DM2CController controller(side, port, baudrate);

    std::signal(SIGINT, signalHandler);
 //   std::thread mover([&]() {
 //       while (!stop_requested) {
 //           moveBlocking(controller, 50000, 50, 100, 100);
 //           moveBlocking(controller, 10000, 50, 100, 100);
 //       }
 //   });

    controller.startVelocityModeBatch(speed, 100, 100);

//    controller.moveToRelativePosition(50000, 50, 100, 100);
    while (!stop_requested) {
        std::cout << controller.readStatus() << std::endl;
        std::cout << controller.readAlarm() << std::endl;
        std::cout << controller.readActualSpeed() << std::endl;
        std::cout << controller.readActualPosition() << std::endl;
    }


    std::cout << "\n接收到停止信号，正在停止电机..." << std::endl;
    controller.stopAllMotion();
    std::cout << "电机已停止" << std::endl;

    return 0;
}
