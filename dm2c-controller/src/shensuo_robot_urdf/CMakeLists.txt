cmake_minimum_required(VERSION 3.8)
project(shensuo_robot_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf worlds models rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
