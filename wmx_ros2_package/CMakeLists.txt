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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wmx_ros2_message REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
)

include_directories(include /opt/lmx/include /opt/lmx/sample/Header)
link_directories(/opt/lmx/lib)

install(DIRECTORY
  src
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(manipulator_state src/manipulator_state.cpp)
ament_target_dependencies(manipulator_state rclcpp std_msgs sensor_msgs std_srvs wmx_ros2_message)
target_link_libraries(manipulator_state coremotionapi ioapi wmx3api imdll)

add_executable(follow_joint_trajectory_server src/follow_joint_trajectory_server.cpp)
target_link_libraries(follow_joint_trajectory_server
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${rclcpp_action_LIBRARIES}
    ${control_msgs_LIBRARIES}
    ${trajectory_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    coremotionapi
    advancedmotionapi
    ioapi
    wmx3api
    imdll
)

#add_executable(diff_drive_controller src/diff_drive_controller.cpp)
#target_link_libraries(diff_drive_controller
#  PRIVATE
#    ${rclcpp_LIBRARIES}
#    ${std_msgs_LIBRARIES}
#    ${sensor_msgs_LIBRARIES}
#    ${std_srvs_LIBRARIES}
#    coremotionapi
#    wmx3api
#    imdll
#)


add_executable(wmx_engine_node src/wmx_engine_node.cpp)
ament_target_dependencies(wmx_engine_node rclcpp std_msgs std_srvs wmx_ros2_message)
target_link_libraries(wmx_engine_node ecapi wmx3api imdll)

add_executable(wmx_core_motion_node src/wmx_core_motion_node.cpp)
ament_target_dependencies(wmx_core_motion_node rclcpp std_msgs std_srvs wmx_ros2_message)
target_link_libraries(wmx_core_motion_node coremotionapi wmx3api imdll)

add_executable(wmx_io_node src/wmx_io_node.cpp)
ament_target_dependencies(wmx_io_node rclcpp std_msgs wmx_ros2_message)
target_link_libraries(wmx_io_node ioapi wmx3api imdll)

add_executable(wmx_ethercat_node src/wmx_ethercat_node.cpp)
ament_target_dependencies(wmx_ethercat_node rclcpp std_msgs wmx_ros2_message diagnostic_msgs)
target_link_libraries(wmx_ethercat_node ecapi wmx3api imdll)

install(TARGETS
  manipulator_state
  follow_joint_trajectory_server
  #diff_drive_controller
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
