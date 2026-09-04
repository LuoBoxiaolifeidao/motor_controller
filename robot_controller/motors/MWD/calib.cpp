#include "MWDController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

static volatile bool show_angle = true;
void sig_handler(int) { show_angle = false; }

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sig_handler);

    int motor_id = 1;
    std::string can_iface = "can1";
    uint32_t bitrate = 1000000;

    if (argc >= 2) motor_id = std::stoi(argv[1]);
    if (argc >= 3) can_iface = argv[2];
    if (argc >= 4) bitrate = std::stoul(argv[3]);

    std::cout << "=== MWD Calibration ===" << std::endl;
    std::cout << "ID=" << motor_id << "  CAN=" << can_iface << "  bitrate=" << bitrate << std::endl;
    std::cout << "1. Move motor to ZERO position" << std::endl;
    std::cout << "2. Ctrl+C to calibrate" << std::endl;

    MWDController motor(motor_id, can_iface, bitrate);
    motor.enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (show_angle) {
        int64_t ang  = motor.getMotorAngle();
        int16_t spd  = motor.getSpeed();
        int8_t  tmp  = motor.getTemperature();
        uint16_t v   = motor.getVoltage();
        uint8_t  st  = motor.getMotorState();
        uint8_t  err = motor.getErrorState();
        uint16_t enc = motor.getEncoder();

        std::cout << "\rangle=" << (ang / 1000.0) << " deg"
                  << "  raw=" << ang
                  << "  spd=" << spd << " dps"
                  << "  enc=" << enc
                  << "  tmp=" << (int)tmp << " C"
                  << "  v=" << (v / 100.0) << " V"
                  << "  st=" << (int)st
                  << "  err=" << (int)err
                  << "   " << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    std::cout << "Angle before zero: " << motor.getMotorAngle() / 1000.0 << " deg" << std::endl;
    motor.setZeroToROM();
    std::cout << "SetZero sent (0x19). POWER CYCLE to take effect." << std::endl;

    motor.disable();
    return 0;
}
