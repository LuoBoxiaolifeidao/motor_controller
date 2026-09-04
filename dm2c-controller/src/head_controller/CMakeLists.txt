cmake_minimum_required(VERSION 3.10.2)
project(head_controller)

# ===== Build Config =====
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  roslib
  std_msgs
  common_comms
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  common_comms
)

include_directories(
  include/${PROJECT_NAME}
  motors/FTServo_Linux
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(motors/FTServo_Linux)

# --- head_controller ---
add_executable(head_controller
  src/head_controller_node.cpp
  src/HeadController.cpp
)
add_dependencies(head_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_controller SCServo_head yaml-cpp ${catkin_LIBRARIES})

# ===== Install =====
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(PROGRAMS scripts/test_head_sweep.py scripts/test_cancel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
