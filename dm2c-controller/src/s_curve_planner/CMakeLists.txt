cmake_minimum_required(VERSION 3.8)
project(s_curve_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Core library: S-curve trajectory + synced trajectory
set(CORE_SRCS src/synced_trajectory.cpp)
add_library(s_curve_planner ${CORE_SRCS})
target_include_directories(s_curve_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Controller plugin
add_library(s_curve_controller SHARED
  src/s_curve_controller.cpp
  src/left_arm_ik.cpp
  src/right_arm_ik.cpp
)
target_include_directories(s_curve_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(s_curve_controller
  rclcpp rclcpp_lifecycle controller_interface hardware_interface
  pluginlib std_msgs kdl_parser orocos_kdl_vendor
)
target_link_libraries(s_curve_controller s_curve_planner)

pluginlib_export_plugin_description_file(controller_interface s_curve_plugin.xml)

# Demo executable
add_executable(s_curve_demo src/main.cpp)
target_include_directories(s_curve_demo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(s_curve_demo s_curve_planner)

# Install
install(TARGETS s_curve_planner s_curve_controller s_curve_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

ament_package()
