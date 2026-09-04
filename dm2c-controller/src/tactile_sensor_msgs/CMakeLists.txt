cmake_minimum_required(VERSION 3.0.2)
project(tactile_sensor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs  # 原有依赖
  std_msgs       # 原有依赖
  message_generation  # ROS1消息生成必备
)

# 声明要生成的.msg文件（对应原有msg文件）
add_message_files(
  FILES
  FingerData.msg   # 原有msg
  TactileArray.msg # 原有msg
)

# 生成消息（依赖std_msgs/geometry_msgs）
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# 声明catkin包导出（运行时依赖）
catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  std_msgs 
  message_runtime  # ROS1运行时消息依赖
)


