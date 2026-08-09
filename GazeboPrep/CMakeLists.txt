cmake_minimum_required(VERSION 3.16)
project(gazebo_prep)

# ---------------------------------------------------------------------------
# Compiler settings
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# C++ node: balance_point (three-layer cascade controller)
# ---------------------------------------------------------------------------
add_executable(balance_point src/balance_point.cpp)

target_include_directories(balance_point PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(balance_point
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  rcl_interfaces)

install(TARGETS balance_point
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Python module + executable wrappers
#
# ament_python_install_package() installs the importable module; the thin
# wrappers in scripts/ are what `ros2 run gazebo_prep <name>` resolves to.
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/standing_mode
  scripts/rc_teleop
  scripts/wireless_gui_bridge
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Resources
# ---------------------------------------------------------------------------
install(DIRECTORY
  urdf
  meshes
  config
  worlds
  launch
  DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright/licence linters are noise for a private project.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
