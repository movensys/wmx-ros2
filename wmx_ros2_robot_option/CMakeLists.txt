cmake_minimum_required(VERSION 3.10)
project(wmx_ros2_robot_option)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wmx_ros2_message REQUIRED)

set(WMX3_SDK_PATH "/opt/wmx3" CACHE PATH "Path to WMX3 SDK installation")

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# wmx_robot_option_node reflects the wmx_arm_server robot option. It links the
# WMX3 robot-motion module (robotmotionapi) in addition to the core libraries;
# the SDK at WMX3_SDK_PATH must provide the RobotMotion option.
add_executable(wmx_robot_option_node src/wmx_robot_option_node.cpp)
ament_target_dependencies(wmx_robot_option_node
  rclcpp std_msgs std_srvs wmx_ros2_message)
target_include_directories(wmx_robot_option_node PRIVATE
  include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_robot_option_node
  robotmotionapi coremotionapi ioapi wmx3api imdll)
target_link_directories(wmx_robot_option_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_robot_option_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

install(TARGETS
  wmx_robot_option_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_robot_option_node.py
    TIMEOUT 60
  )
endif()

ament_package()
