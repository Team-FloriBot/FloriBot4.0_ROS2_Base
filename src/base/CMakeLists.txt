cmake_minimum_required(VERSION 3.5)
project(base)

# Empfehlung: C++17 für modernere ROS2 Distributionen
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# 1. Pakete finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rosidl_default_generators REQUIRED)

# 2. Eigene Interfaces (Messages) generieren
set(msg_files
  "msg/Wheels.msg"
  "msg/Angle.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 3. Libraries erstellen
# Bibliothek 1: Antriebslogik
add_library(articulated_drive src/drives/articulated_drive.cpp)
target_include_directories(articulated_drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(articulated_drive 
  rclcpp tf2_ros std_msgs geometry_msgs nav_msgs sensor_msgs Eigen3
)

# Bibliothek 2: Publisher Logik
add_library(basePublisher src/base_node/base_publisher.cpp)
target_include_directories(basePublisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Linkt gegen die eigene Lib und ROS2
target_link_libraries(basePublisher articulated_drive)
ament_target_dependencies(basePublisher 
  rclcpp tf2_ros std_msgs geometry_msgs nav_msgs sensor_msgs Eigen3
)

# 4. Executables (Nodes) erstellen
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node articulated_drive basePublisher)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp tf2_ros std_msgs geometry_msgs nav_msgs sensor_msgs Eigen3
)

# 5. Interface-Verknüpfung (Damit die Nodes die eigenen Messages finden)
# WICHTIG: Das muss für alle Ziele gemacht werden, die deine .msg Dateien nutzen
rosidl_target_interfaces(articulated_drive ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(basePublisher ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- 6. Export für andere Pakete (wie plc_connection) ---
# Wir exportieren die Bibliotheken und die Runtime für die Messages
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(articulated_drive basePublisher)

# --- 7. Installation ---
# Bibliotheken installieren
install(TARGETS articulated_drive basePublisher ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Verzeichnisse und Dateien
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
