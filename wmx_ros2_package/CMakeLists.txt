cmake_minimum_required(VERSION 3.10)
project(wmx_ros2_package)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wmx_ros2_message REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(WMX3_SDK_PATH "/opt/lmx" CACHE PATH "Path to WMX3 SDK installation")

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(manipulator_state src/manipulator_state.cpp)
ament_target_dependencies(manipulator_state rclcpp std_msgs sensor_msgs std_srvs wmx_ros2_message)
target_include_directories(manipulator_state PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(manipulator_state coremotionapi ioapi wmx3api imdll)
target_link_directories(manipulator_state PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(manipulator_state PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(follow_joint_trajectory_server src/follow_joint_trajectory_server.cpp)
ament_target_dependencies(follow_joint_trajectory_server
  rclcpp rclcpp_action std_msgs std_srvs control_msgs trajectory_msgs wmx_ros2_message)
target_include_directories(follow_joint_trajectory_server PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(follow_joint_trajectory_server coremotionapi advancedmotionapi ioapi wmx3api imdll)
target_link_directories(follow_joint_trajectory_server PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(follow_joint_trajectory_server PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_engine_node src/wmx_engine_node.cpp)
ament_target_dependencies(wmx_engine_node rclcpp std_msgs std_srvs wmx_ros2_message)
target_include_directories(wmx_engine_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_engine_node ecapi wmx3api imdll)
target_link_directories(wmx_engine_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_engine_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_core_motion_node src/wmx_core_motion_node.cpp)
ament_target_dependencies(wmx_core_motion_node rclcpp std_msgs std_srvs wmx_ros2_message)
target_include_directories(wmx_core_motion_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_core_motion_node coremotionapi wmx3api imdll)
target_link_directories(wmx_core_motion_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_core_motion_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_io_node src/wmx_io_node.cpp)
ament_target_dependencies(wmx_io_node rclcpp std_msgs wmx_ros2_message)
target_include_directories(wmx_io_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_io_node ioapi wmx3api imdll)
target_link_directories(wmx_io_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_io_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

add_executable(wmx_ethercat_node src/wmx_ethercat_node.cpp)
ament_target_dependencies(wmx_ethercat_node rclcpp std_msgs wmx_ros2_message)
target_include_directories(wmx_ethercat_node PRIVATE include ${WMX3_SDK_PATH}/include ${WMX3_SDK_PATH}/sample/Header)
target_link_libraries(wmx_ethercat_node ecapi wmx3api imdll)
target_link_directories(wmx_ethercat_node PRIVATE ${WMX3_SDK_PATH}/lib)
target_compile_definitions(wmx_ethercat_node PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")

install(TARGETS
  manipulator_state
  follow_joint_trajectory_server
  wmx_engine_node
  wmx_core_motion_node
  wmx_io_node
  wmx_ethercat_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
