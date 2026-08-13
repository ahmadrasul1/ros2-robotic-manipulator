cmake_minimum_required(VERSION 3.8)
project(rascl_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Hardware interface plugin library
add_library(
  rascl_hardware_interface
  SHARED
  src/rascl_hardware_interface.cpp
)

target_include_directories(
  rascl_hardware_interface
  PUBLIC
    include
)

ament_target_dependencies(
  rascl_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export plugin description to pluginlib
pluginlib_export_plugin_description_file(
  hardware_interface
  rascl_hardware_interface.xml
)

# Install C++ library
install(
  TARGETS rascl_hardware_interface
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin XML
install(
  FILES rascl_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python backend scripts
install(
  PROGRAMS
    scripts/pysoem_bridge.py
    scripts/pdo_csp_bridge.py
    scripts/hardware_bridge.py
    scripts/wait_for_bridge_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install cyclic PDO configuration
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Keep these only while the project does not have full copyright/license headers.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()