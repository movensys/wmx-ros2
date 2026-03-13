cmake_minimum_required(VERSION 3.10)
project(wmx_ros2_message)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetEngine.srv"
  "srv/SetAxis.srv"
  "srv/SetAxisGearRatio.srv"
  "srv/GetIoBit.srv"
  "srv/GetIoBytes.srv"
  "srv/SetIoBit.srv"
  "srv/SetIoBytes.srv"

  "srv/EcatGetNetworkState.srv"
  "srv/EcatRegisterRead.srv"
  "srv/EcatResetStatistics.srv"
  "srv/EcatStartHotconnect.srv"

  "srv/LoadWmxParams.srv"
  "srv/GetWmxParams.srv"

  "msg/AxisVelocity.msg"
  "msg/AxisState.msg"
  "msg/AxisPose.msg"
  DEPENDENCIES std_msgs
)

install(DIRECTORY
  srv
  msg
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
