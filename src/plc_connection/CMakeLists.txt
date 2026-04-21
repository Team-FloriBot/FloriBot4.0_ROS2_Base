cmake_minimum_required(VERSION 3.5)
project(plc_connection)

# Standard auf C++17 (ROS2 Humble+ Standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# 1. Pakete finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(base REQUIRED)

# 2. Interfaces (Services) generieren
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCount.srv"
  DEPENDENCIES std_msgs
)

# 3. Bibliotheken definieren
# Strategie: Jedes Target bekommt seine eigenen Abhängigkeiten direkt zugewiesen

# --- Library: socket ---
add_library(socket src/network/socket/socket.cpp)
target_include_directories(socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(socket rclcpp)

# --- Library: udp_socket ---
add_library(udp_socket src/network/udp/udp_socket.cpp)
target_include_directories(udp_socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(udp_socket socket)
ament_target_dependencies(udp_socket rclcpp)

# --- Library: plc_connection_lib ---
add_library(plc_connection_lib src/plc_connection_lib.cpp)
target_include_directories(plc_connection_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(plc_connection_lib socket udp_socket)
ament_target_dependencies(plc_connection_lib rclcpp base tf2 tf2_ros std_msgs geometry_msgs)

# 4. Executable (Node)
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node plc_connection_lib) # Zieht socket/udp_socket automatisch mit
ament_target_dependencies(${PROJECT_NAME}_node rclcpp base tf2 tf2_ros std_msgs geometry_msgs)

# 5. Interface Verknüpfung (WICHTIG für .srv Dateien)
rosidl_target_interfaces(plc_connection_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 6. Exporte für andere Pakete
ament_export_include_directories(include)
ament_export_libraries(socket udp_socket plc_connection_lib)
ament_export_dependencies(rclcpp base rosidl_default_runtime)

# 7. Installation
# Bibliotheken installieren
install(TARGETS socket udp_socket plc_connection_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Node installieren
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Verzeichnisse installieren
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()