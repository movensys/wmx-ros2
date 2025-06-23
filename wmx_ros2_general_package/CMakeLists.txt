cmake_minimum_required(VERSION 3.5)
project(wmx_ros2_general_package)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wmx_ros2_message REQUIRED) 

include_directories(include /opt/lmx/include /opt/lmx/sample/Header)
link_directories(/opt/lmx/lib)

add_library(wmx_ros2_engine_lib include/wmx_header/wmx_ros2_engine.cpp)
#add_library(wmx_ros2_core_motion_lib include/wmx_header/wmx_ros2_core_motion.cpp)

set_target_properties(wmx_ros2_engine_lib PROPERTIES LINKER_LANGUAGE CXX)
#set_target_properties(wmx_ros2_core_motion_lib PROPERTIES LINKER_LANGUAGE CXX)

ament_target_dependencies(wmx_ros2_engine_lib rclcpp wmx_ros2_message std_srvs)
#ament_target_dependencies(wmx_ros2_core_motion_lib rclcpp)

target_link_libraries(wmx_ros2_engine_lib wmx3api imdll)
#target_link_libraries(wmx_ros2_core_motion_lib wmx3api coremotionapi imdll)

install(DIRECTORY
  src
  launch
  example
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(wmx_ros2_engine_node src/wmx_ros2_engine_node.cpp)
target_link_libraries(wmx_ros2_engine_node wmx_ros2_engine_lib)

#add_executable(wmx_ros2_core_motion_node src/wmx_ros2_core_motion_node.cpp)
#target_link_libraries(wmx_ros2_core_motion_node wmx_ros2_core_motion_lib)

add_executable(wmx_ros2_core_motion_node src/wmx_ros2_core_motion_node.cpp)
ament_target_dependencies(wmx_ros2_core_motion_node rclcpp)
target_link_libraries(wmx_ros2_core_motion_node wmx3api coremotionapi imdll)


#add_executable(wmx_engine_example example/wmx_engine_example.cpp)
#ament_target_dependencies(wmx_engine_example rclcpp wmx_ros2_message std_srvs)   
#target_link_libraries(wmx_engine_example wmx3api imdll)

install(TARGETS 
  wmx_ros2_engine_node
  wmx_ros2_core_motion_node 

  #wmx_general_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
