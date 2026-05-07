cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY src/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF / Xacro files
install(DIRECTORY src/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright check until license headers are added
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint; only relevant when package is in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
