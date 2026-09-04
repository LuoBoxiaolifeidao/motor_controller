cmake_minimum_required(VERSION 3.10.2)
project(robot_controller)

# ===== Build Config =====
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB DM2C_SRCS motors/DM2C/*.cpp)
list(REMOVE_ITEM DM2C_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/motors/DM2C/DM2CServer.cpp")
file(GLOB AK_SRCS   motors/AK/*.cpp)
file(GLOB S7_SRCS   motors/S7/*.cpp)
file(GLOB MWD_SRCS  motors/MWD/CANComMWD.cpp motors/MWD/MWDController.cpp)
file(GLOB MWD485_SRCS motors/MWD/MWD485Controller.cpp motors/DM2C/SerialPort.cpp)

find_package(yaml-cpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  common_comms
  message_generation
  roscpp
  std_msgs
  hardware_interface
  controller_manager
  controller_manager_msgs
  pluginlib
  tactile_sensor_msgs
)

add_service_files(
  FILES
  GetState.srv
  SetZero.srv
)

add_action_files(
  FILES
  Move.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

include_directories(
  include/${PROJECT_NAME}
  hardware_interface
  motors
  motors/DM2C
  motors/S7
  motors/MWD
  motors/FTServo_Linux
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_subdirectory(motors/FTServo_Linux)

# --- dm2c_controller ---
add_executable(dm2c_controller
  ${DM2C_SRCS}
  motors/DM2C/DM2CServer.cpp
)
add_dependencies(dm2c_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dm2c_controller ${catkin_LIBRARIES})


# --- robot_controller ---
add_executable(robot_controller
  src/RobotServer.cpp
  src/ControllerNode.cpp
  src/SCurvPlan.cpp
  src/left_arm_ik.cpp
  motors/synced_trajectory.cpp
  ${DM2C_SRCS}
  ${MWD485_SRCS}
  ${MWD_SRCS}
)
add_dependencies(robot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_controller SCServo yaml-cpp ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})


# ===== Install =====
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

