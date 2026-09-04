#include "MWD485Controller.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <sstream>
#include <cstring>
#include <vector>
#include <algorithm>

static constexpr double GEAR_RATIO = 10.0;

static inline int32_t degToMotor(double deg) { return static_cast<int32_t>(deg * GEAR_RATIO * 100.0); }
static inline double motorToDeg(int64_t raw) { return raw / 100.0 / GEAR_RATIO; }
static inline uint16_t dpsToMotor(double dps) { return static_cast<uint16_t>(dps * GEAR_RATIO); }

static void printState(MWD485Controller& motor, const char* label = "")
{
    int64_t angle  = motor.getMotorAngle();
    int16_t speed  = motor.getSpeed();
    uint8_t state  = motor.getMotorState();
    uint8_t error  = motor.getErrorState();
    int8_t  temp   = motor.getTemperature();

    std::cout << "  [" << label << " id=" << motor.getMotorId() << "]"
              << " angle=" << std::fixed << std::setprecision(1) << motorToDeg(angle) << "deg"
              << " speed=" << std::setw(5) << std::abs(speed) / GEAR_RATIO << "dps"
              << " temp=" << (int)temp << "C"
              << " state=" << std::hex << (int)state << std::dec
              << " err=" << std::hex << (int)error << std::dec
              << std::endl;
}

static void printUsage()
{
    std::cout << "Commands:\n"
              << "  q/quit     - exit\n"
              << "  state      - print all motor states\n"
              << "  enable     - enable all motors\n"
              << "  disable    - disable all motors\n"
              << "  stop       - stop all motors\n"
              << "  brake      - release brake on all motors\n"
              << "  unbrake    - lock brake on all motors\n"
              << "  clearerr   - clear error flags on all motors\n"
              << "  spos <deg> [speed]  - SYNC all motors (fire-and-forget, simultaneous start)\n"
              << "  pos <deg> [speed]   - all motors absolute position\n"
              << "  inc <deg> [speed]   - all motors incremental\n"
              << "  speed <dps>         - all motors speed control\n"
              << "  pos1 <deg> [speed]  - motor 1 only\n"
              << "  pos2 <deg> [speed]  - motor 2 only\n"
              << "  p2 <deg1> <deg2> [speed] - motor1 deg1, motor2 deg2 (sync)\n"
              << "  setzero             - set zero on all\n"
              << "  limit [deg]         - read/set angle limit (0=disable)\n"
              << "  help      - this message\n";
}

static bool parsePosArgs(std::istringstream& iss, double& deg, double& speedDeg)
{
    if (!(iss >> deg)) return false;
    speedDeg = 180.0;
    iss >> speedDeg;
    return true;
}

int main(int argc, char** argv)
{
    std::string port = (argc >= 2) ? argv[1] : "/dev/ttysWK0";
    int id1 = (argc >= 3) ? std::stoi(argv[2]) : 7;
    int id2 = (argc >= 4) ? std::stoi(argv[3]) : 8;
    int baudrate = (argc >= 5) ? std::stoi(argv[4]) : 115200;

    std::vector<int> motorIds = {id1, id2};

    std::cout << "MWD RS485 Test - port=" << port << " baud=" << baudrate
              << " motors=" << motorIds.size() << "\n";

    std::vector<std::shared_ptr<MWD485Controller>> motors;
    for (int id : motorIds) {
        auto m = std::make_shared<MWD485Controller>(id, port, baudrate);
        std::cout << "  Enabling motor " << id << "...\n";
        m->enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        m->clearError();
        motors.push_back(m);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    int32_t angleLimit = motors[0]->getAngleLimit();
    std::cout << "GEAR_RATIO=" << GEAR_RATIO << " angle_limit=" << angleLimit/100.0/GEAR_RATIO << "deg\n";
    printUsage();

    bool running = true;

    while (running) {
        std::cout << "  angle=";
        for (size_t i = 0; i < motors.size(); ++i) {
            motors[i]->refreshState();
            if (i > 0) std::cout << " /";
            std::cout << std::fixed << std::setprecision(1)
                      << motorToDeg(motors[i]->getMotorAngle());
        }
        std::cout << " deg\r" << std::flush;

        // non-blocking read
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 200000};  // 200ms
        int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) continue;

        std::string line;
        std::getline(std::cin, line);
        if (line.empty()) continue;
        std::cout << "\n";

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd == "q" || cmd == "quit") {
            running = false;
        }
        else if (cmd == "state") {
            for (auto& m : motors) printState(*m);
        }
        else if (cmd == "enable") {
            for (auto& m : motors) m->enable();
            std::cout << "Enabled\n";
        }
        else if (cmd == "disable") {
            for (auto& m : motors) m->disable();
            std::cout << "Disabled\n";
        }
        else if (cmd == "stop") {
            for (auto& m : motors) m->stop();
            std::cout << "Stopped\n";
        }
        else if (cmd == "brake") {
            for (auto& m : motors) m->brakeRelease();
            std::cout << "Brakes released\n";
        }
        else if (cmd == "unbrake") {
            for (auto& m : motors) m->brakeLock();
            std::cout << "Brakes locked\n";
        }
        else if (cmd == "clearerr") {
            for (auto& m : motors) m->clearError();
            std::cout << "Errors cleared\n";
        }
        else if (cmd == "pos") {
            double deg, speedDeg;
            if (!parsePosArgs(iss, deg, speedDeg)) continue;
            std::cout << "All motors to " << deg << "deg at " << speedDeg << "dps\n";
            for (auto& m : motors) {
                m->brakeRelease();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                m->multiTurnPosition(degToMotor(deg), dpsToMotor(speedDeg));
            }
        }
        else if (cmd == "spos") {
            double deg, speedDeg;
            if (!parsePosArgs(iss, deg, speedDeg)) continue;
            std::cout << "SYNC all motors to " << deg << "deg at " << speedDeg << "dps (fire-and-forget)\n";
            for (auto& m : motors) m->brakeRelease();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            for (auto& m : motors)
                m->sendMultiTurnPosition(degToMotor(deg), dpsToMotor(speedDeg));
        }
        else if (cmd == "inc") {
            double deg, speedDeg;
            if (!parsePosArgs(iss, deg, speedDeg)) continue;
            std::cout << "All motors inc " << deg << "deg at " << speedDeg << "dps\n";
            for (auto& m : motors) {
                m->brakeRelease();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                m->incrementalPosition(degToMotor(deg),
                    static_cast<uint32_t>(speedDeg * GEAR_RATIO));
            }
        }
        else if (cmd == "speed") {
            double dps;
            if (!(iss >> dps)) { std::cout << "Usage: speed <dps>\n"; continue; }
            for (auto& m : motors) m->speedControl(static_cast<int32_t>(dps * GEAR_RATIO * 100.0));
        }
        else if (cmd == "pos1" || cmd == "pos2") {
            int idx = (cmd == "pos1") ? 0 : 1;
            if (idx >= (int)motors.size()) { std::cout << "No motor " << (idx+1) << "\n"; continue; }
            double deg, speedDeg;
            if (!parsePosArgs(iss, deg, speedDeg)) continue;
            auto& m = motors[idx];
            m->brakeRelease();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            m->multiTurnPosition(degToMotor(deg), dpsToMotor(speedDeg));
            std::cout << "Motor " << (idx+1) << " to " << deg << "deg\n";
        }
        else if (cmd == "spos1" || cmd == "spos2") {
            int idx = (cmd == "spos1") ? 0 : 1;
            if (idx >= (int)motors.size()) { std::cout << "No motor " << (idx+1) << "\n"; continue; }
            double deg, speedDeg;
            if (!parsePosArgs(iss, deg, speedDeg)) continue;
            auto& m = motors[idx];
            m->brakeRelease();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            m->sendMultiTurnPosition(degToMotor(deg), dpsToMotor(speedDeg));
            std::cout << "Motor " << (idx+1) << " (fire-and-forget) to " << deg << "deg\n";
        }
        else if (cmd == "setzero") {
            for (auto& m : motors) m->setZeroToROM();
            std::cout << "Zero set (ROM) - may need power cycle\n";
        }
        else if (cmd == "limit") {
            double deg;
            if (iss >> deg) {
                int32_t lim = (deg <= 0) ? 0 : degToMotor(deg);
                for (auto& m : motors) m->setAngleLimit(lim);
                std::cout << "Angle limit set to " << deg << "deg\n";
            } else {
                int32_t lim = motors[0]->getAngleLimit();
                std::cout << "Angle limit: " << motorToDeg(lim) << "deg\n";
            }
        }
        else if (cmd == "help") {
            printUsage();
        }
        else {
            std::cout << "Unknown: " << cmd << " (type 'help')\n";
        }
    }

    std::cout << "\nDisabling motors...\n";
    for (auto& m : motors) m->disable();
    return 0;
}
