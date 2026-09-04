cmake_minimum_required(VERSION 3.8)
project(arm_ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(dual_arm_node
  src/dual_arm_node.cpp
  src/right_arm_ik.cpp
  src/left_arm_ik.cpp
)
target_include_directories(dual_arm_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(dual_arm_node
  rclcpp rclcpp_action std_msgs control_msgs kdl_parser orocos_kdl_vendor tf2 tf2_ros
)
install(TARGETS dual_arm_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pick_left.py scripts/pick_right.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
ament_package()
