#pragma once
#include <string>
#include <vector>

struct DM2CConfig {
    std::string port = "/dev/ttyACM0";
    int baudrate = 38400;
    int id = 1;
    double min_speed = 1.0;
    double max_speed = 150.0;
    int max_accel = 100;
    double dead_zone = 1.0;
};

struct S7Config {
    std::string port = "/dev/ttysWK1";
    int baudrate = 115200;
    std::vector<int> ids = {11, 12, 13};
    std::vector<int> max_speed = {50, 50, 50};
    std::vector<int> max_accel = {60, 60, 60};
};

struct FTConfig {
    std::string port = "/dev/ttysWK0";
    int baudrate = 115200;
    std::vector<int> ids = {10, 11, 12};
    std::vector<int> min_speed = {3, 3, 3};
    std::vector<int> max_speed = {300, 300, 300};
    std::vector<int> max_accel = {50, 50, 50};
    std::vector<int> dead_zone = {3, 3, 3};
    std::vector<int> j_max = {1000, 1000, 1000};
};

struct AKConfig {
    std::string interface = "can0";
    int bitrate = 1000000;
    int id = 1;
    double min_speed = 1.0;
    double max_speed = 700.0;
    int max_accel = 500;
    double dead_zone = 3.0;
    double j_max = 150.0;
};

struct MWDConfig {
    std::string interface = "can1";
    int bitrate = 1000000;
    int id = 1;
    double min_speed = 1.0;
    double max_speed = 180.0;
    int max_accel = 500;
    double dead_zone = 1.0;
    double j_max = 50.0;
};

struct MWD485Config {
    int id = 1;
    std::string port = "/dev/ttysWK0";
    int baudrate = 115200;
    double gear_ratio = 10.0;
    double min_speed = 1.0;
    double max_speed = 150.0;
    int max_accel = 100;
    double dead_zone = 1.0;
    double j_max = 50.0;
};

struct GripperConfig {
    int id = 23;
    std::string port = "/dev/ttysWK0";
    int baudrate = 115200;
    int open = 2048;
    int close = 1024;
    int dead_zone = 40;
};
