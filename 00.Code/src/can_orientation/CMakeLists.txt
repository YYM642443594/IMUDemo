cmake_minimum_required(VERSION 3.8)
project(can_orientation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanImuData.msg"
  DEPENDENCIES std_msgs
)

add_executable(can_orientation_node src/ahrs.cpp)
ament_target_dependencies(can_orientation_node rclcpp sensor_msgs std_msgs)

add_executable(can_imu_listener src/sub_spec.cpp)
ament_target_dependencies(can_imu_listener rclcpp sensor_msgs std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(can_orientation_node "${cpp_typesupport_target}")
target_link_libraries(can_imu_listener "${cpp_typesupport_target}")

install(TARGETS can_orientation_node can_imu_listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
