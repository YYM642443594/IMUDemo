cmake_minimum_required(VERSION 3.5)
project(zhixiang_imu_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData1.msg"
  "msg/ImuData2.msg"
  "msg/ImuData3.msg"
  "srv/SetPush.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

add_executable(talker src/serial_port.cpp src/serial_parse.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
rosidl_target_interfaces(talker ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(listener src/sub_spec.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
rosidl_target_interfaces(listener ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS talker listener DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
