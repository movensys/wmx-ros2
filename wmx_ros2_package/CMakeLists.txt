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
find_package(control_msgs REQUIRED)        # added
find_package(trajectory_msgs REQUIRED)     # added

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
)

include_directories(include /opt/lmx/include /opt/lmx/sample/Header)
link_directories(/opt/lmx/lib)

add_library(wmx_ros2_general_lib include/wmx_general_header/wmx_ros2_general.cpp
                                 include/wmx_general_header/wmx_ros2_engine.cpp
                                 include/wmx_general_header/wmx_ros2_core_motion.cpp)
set_target_properties(wmx_ros2_general_lib PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(wmx_ros2_general_lib rclcpp rclcpp_action wmx_ros2_message std_srvs)
target_link_libraries(wmx_ros2_general_lib coremotionapi wmx3api imdll)

install(DIRECTORY
  src
  config
  launch
  example
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(wmx_ros2_general_node src/wmx_ros2_general_node.cpp)
target_link_libraries(wmx_ros2_general_node wmx_ros2_general_lib)

add_executable(wmx_ros2_general_example example/wmx_ros2_general_example.cpp)
ament_target_dependencies(wmx_ros2_general_example rclcpp wmx_ros2_message std_srvs)   

add_executable(cr3a_state src/cr3a_state.cpp)
target_link_libraries(cr3a_state
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    coremotionapi
    ioapi
    wmx3api
    imdll
)

add_executable(cr3a_action src/cr3a_action.cpp)
target_link_libraries(cr3a_action
  PRIVATE
    ${rclcpp_LIBRARIES}
    ${rclcpp_action_LIBRARIES}
    ${control_msgs_LIBRARIES}
    ${trajectory_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    coremotionapi
    advancedmotionapi
    ioapi
    wmx3api
    imdll
)

install(TARGETS 
  cr3a_state
  cr3a_action
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
