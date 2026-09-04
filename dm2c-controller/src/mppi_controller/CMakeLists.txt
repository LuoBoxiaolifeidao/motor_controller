cmake_minimum_required(VERSION 3.8)
project(mppi_controller)

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

add_library(mppi_controller SHARED
  src/mppi_controller.cpp
  src/left_arm_ik.cpp
  src/right_arm_ik.cpp
)
target_include_directories(mppi_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mppi_controller
  rclcpp rclcpp_lifecycle controller_interface hardware_interface
  pluginlib std_msgs kdl_parser orocos_kdl_vendor
)
target_link_libraries(mppi_controller)

pluginlib_export_plugin_description_file(controller_interface mppi_plugin.xml)

install(TARGETS mppi_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})

ament_package()
