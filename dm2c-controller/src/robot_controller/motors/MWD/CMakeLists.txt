cmake_minimum_required(VERSION 3.10)
project(mwd_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../DM2C)

add_executable(mwd_test main.cpp CANComMWD.cpp MWDController.cpp)
target_link_libraries(mwd_test pthread)

add_executable(mwd_calib calib.cpp CANComMWD.cpp MWDController.cpp)
target_link_libraries(mwd_calib pthread)

add_executable(mwd485_test
    MWD485Controller.cpp MWD485Server.cpp ../DM2C/SerialPort.cpp)
target_link_libraries(mwd485_test pthread)
