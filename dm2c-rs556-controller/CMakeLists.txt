cmake_minimum_required(VERSION 3.10)
project(dm2c_monitor)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)
add_executable(dm2c_monitor main.cpp SerialPort.cpp  DM2CController.cpp ModbusFrameBuilder.cpp)
target_link_libraries(dm2c_monitor -pthread)
add_executable(stop stop.cpp SerialPort.cpp  DM2CController.cpp ModbusFrameBuilder.cpp)
