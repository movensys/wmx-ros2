cmake_minimum_required(VERSION 3.10)
project(nova_diff_drive_logic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Header-only logic library (WMX-free, ROS-free).
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(_t
      test_diff_drive_kinematics
      test_odometry_integrator
      test_odom_delta_accumulator
      test_accel_estimator)
    ament_add_gtest(${_t} test/${_t}.cpp)
    target_include_directories(${_t} PRIVATE include)
  endforeach()
endif()

ament_export_include_directories(include)
ament_package()
