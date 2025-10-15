cmake_minimum_required(VERSION 3.8)
project(gripper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)

# 查找Modbus库
pkg_check_modules(MODBUS REQUIRED libmodbus)

# 动作和消息定义文件
set(action_files
  "action/GripperControl.action"
)
set(msg_files
  "msg/TactileData.msg"
  
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  DEPENDENCIES action_msgs std_msgs
)

# 包含头文件目录（关键配置）
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${MODBUS_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(gripper_action_server
  src/gripper.cpp
  src/tactile_sensor.cpp
  src/gripper_action_server.cpp
  



  

)

# 链接依赖库
ament_target_dependencies(gripper_action_server
  rclcpp
  rclcpp_action
  std_msgs
)

# 链接Modbus库
target_link_libraries(gripper_action_server
  ${MODBUS_LIBRARIES}
)

# 链接接口类型支持
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(gripper_action_server "${cpp_typesupport_target}")

# 指定头文件安装路径
target_include_directories(gripper_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS
  gripper_action_server
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_package()

