cmake_minimum_required(VERSION 3.5)
project(wmx_ros2_moveit2_package)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

include_directories(/opt/lmx/include /opt/lmx/sample/Header)
link_directories(/opt/lmx/lib)

install(DIRECTORY
  src
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


add_executable(cr3a_interface src/cr3a_interface.cpp)
target_link_libraries(cr3a_interface
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    coremotionapi
    ioapi
    wmx3api
    imdll
)
install(TARGETS cr3a_interface DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
