cmake_minimum_required(VERSION 3.8)
project(walle_description)

# Compiler options for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install directories (URDF, launch, and scripts)
install(
  DIRECTORY urdf launch scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables
install(
  PROGRAMS
    scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks for now (uncomment when license is added)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Package the project
ament_package()
