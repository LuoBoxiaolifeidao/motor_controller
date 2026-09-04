cmake_minimum_required(VERSION 3.10)
project(tactile_sensor_driver)

# 仅对 GCC / Clang 开启严格警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 统一C++标准，强制C++14，关闭编译器扩展
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找ROS1依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tactile_sensor_msgs
)

# 生成消息 (如果tactile_sensor_msgs集成到当前包，否则需单独创建msgs包)
# 注意：需把tactile_sensor_msgs的.msg文件复制到当前包的msg/目录下
# add_message_files(
#   FILES
#   FingerData.msg
#   TactileArray.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ch347_lib  # 声明导入的库，确保lib目录下有libch347.so
  CATKIN_DEPENDS roscpp std_msgs tactile_sensor_msgs
)

# 包含头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 导入第三方库 (libch347.so)
add_library(ch347_lib SHARED IMPORTED)
set_target_properties(ch347_lib PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libch347.so"
)

# 创建可执行程序
add_executable(tactile_node
  src/tactile_node.cpp
  src/ch341_driver.cpp
  src/finger.cpp
  src/sensor_cmd.cpp
)

# 链接依赖
target_link_libraries(tactile_node
  ${catkin_LIBRARIES}
  ch347_lib
)

# 依赖消息生成 (必须加，否则msg头文件找不到)
add_dependencies(tactile_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装
install(TARGETS tactile_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES lib/libch347.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 安装launch文件 (ROS1 launch是xml格式，需重写)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)