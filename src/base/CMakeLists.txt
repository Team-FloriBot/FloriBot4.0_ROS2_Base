cmake_minimum_required(VERSION 3.8)
project(base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelTicks4.msg"
  "msg/WheelVelocities4.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(base_kinematics
  src/library/kinematics_calculator.cpp
)
ament_target_dependencies(base_kinematics rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(base_kinematics ${cpp_typesupport_target})

add_executable(kinematics_node
  src/nodes/kinematics_node.cpp
)
ament_target_dependencies(kinematics_node
  geometry_msgs nav_msgs rclcpp rcl_interfaces std_msgs tf2 tf2_ros
)
target_link_libraries(kinematics_node base_kinematics ${cpp_typesupport_target})

add_executable(hardware_node
  src/nodes/hardware_node.cpp
)
ament_target_dependencies(hardware_node
  rclcpp sensor_msgs std_msgs
)
target_link_libraries(hardware_node ${cpp_typesupport_target})

install(TARGETS
  base_kinematics
  kinematics_node
  hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
