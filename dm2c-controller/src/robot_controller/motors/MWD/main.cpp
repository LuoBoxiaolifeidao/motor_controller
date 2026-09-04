#include "MWDController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>

static volatile bool keep_running = true;
void signal_handler(int) { keep_running = false; }

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    int motor_id = 1;
    std::string can_iface = "can1";
    uint32_t bitrate = 1000000;
    double target_deg = -999;

    if (argc >= 2) motor_id = std::stoi(argv[1]);
    if (argc >= 3) can_iface = argv[2];
    if (argc >= 4) bitrate = std::stoul(argv[3]);
    if (argc >= 5) target_deg = std::stod(argv[4]);

    MWDController motor(motor_id, can_iface, bitrate);
    motor.enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    motor.clearError();
    motor.setAcceleration(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (target_deg > -900)
        motor.multiTurnPosition(static_cast<int32_t>(target_deg * 1000), 100);

    auto t0 = std::chrono::steady_clock::now();
    auto t_last = t0;
    double ang_last = 0, spd_last = 0;
    bool first = true;

    while (keep_running) {
        auto t = std::chrono::steady_clock::now();
        int64_t ang = motor.getMotorAngle();
        int16_t spd = motor.getSpeed();
        int32_t acc = motor.getAcceleration();

        double ms = std::chrono::duration<double, std::milli>(t - t0).count();
        double dt = std::chrono::duration<double, std::milli>(t - t_last).count();

        double calc_spd = 0, calc_acc = 0;
        if (!first && dt > 0) {
            calc_spd = (ang - ang_last) / (dt * 1000.0) * 1000.0;   // deg/s
            calc_acc = (spd - spd_last) / (dt / 1000.0);             // dps/s, from CAN speed diff
        }

        std::cout << ms << " " << ang << " " << spd << " " << acc
                  << " " << calc_spd << " " << calc_acc << std::endl;

        t_last = t;
        ang_last = ang;
        spd_last = spd;
        first = false;
    }

    motor.stop();
    return 0;
}
