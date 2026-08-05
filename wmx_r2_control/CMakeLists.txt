cmake_minimum_required(VERSION 3.10)
project(wmx_r2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-fsigned-char)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(WMX3_SDK_PATH "/opt/wmx3" CACHE PATH "Path to WMX3 SDK installation")

# The WMX3 SDK only exists on WMX Linux machines. Guard the hardware plugin so
# this package still configures on a plain ROS 2 box or a CI runner, where the
# SDK-free unit tests below can then still be built and run.
if(EXISTS "${WMX3_SDK_PATH}/include")
  set(WMX3_SDK_FOUND TRUE)
else()
  set(WMX3_SDK_FOUND FALSE)
  message(STATUS
    "WMX3 SDK not found at ${WMX3_SDK_PATH}: skipping wmx_system_hardware. "
    "SDK-free unit tests still build.")
endif()

if(WMX3_SDK_FOUND)
  add_library(wmx_system_hardware SHARED
    src/wmx_system_hardware.cpp
    src/wmx_cyclic_stream.cpp
  )
  target_include_directories(wmx_system_hardware PRIVATE
    include
    ${WMX3_SDK_PATH}/include
    ${WMX3_SDK_PATH}/sample/Header
  )
  target_compile_definitions(wmx_system_hardware PRIVATE WMX3_SDK_PATH="${WMX3_SDK_PATH}/")
  target_link_directories(wmx_system_hardware PRIVATE ${WMX3_SDK_PATH}/lib)
  # cyclicbufferapi before wmx3api: static archives must precede what they need.
  target_link_libraries(wmx_system_hardware coremotionapi cyclicbufferapi wmx3api imdll)
  ament_target_dependencies(wmx_system_hardware
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )

  # Export the hardware plugin to pluginlib
  pluginlib_export_plugin_description_file(hardware_interface wmx_r2_control_plugin.xml)

  install(TARGETS wmx_system_hardware
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES wmx_r2_control_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # WMX3 SDK headers are not available on lint runners; rely on CI ament_cpplint.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Cyclic-buffer stream timing: header-only, free of WMX and ROS types, so this
  # builds and runs without the WMX3 SDK (see include/wmx_r2_control/).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cyclic_stream_timing test/test_cyclic_stream_timing.cpp)
  target_include_directories(test_cyclic_stream_timing PRIVATE include)

  # WmxCyclicStream guard clauses. These reject before the class touches the
  # engine, so they need no hardware -- but they do need the SDK headers, so
  # they cannot run on a CI runner.
  if(WMX3_SDK_FOUND)
    ament_add_gtest(test_wmx_cyclic_stream test/test_wmx_cyclic_stream.cpp)
    target_include_directories(test_wmx_cyclic_stream PRIVATE
      include
      ${WMX3_SDK_PATH}/include
      ${WMX3_SDK_PATH}/sample/Header
    )
    target_link_directories(test_wmx_cyclic_stream PRIVATE ${WMX3_SDK_PATH}/lib)
    target_link_libraries(test_wmx_cyclic_stream
      wmx_system_hardware coremotionapi cyclicbufferapi wmx3api imdll)
  endif()
endif()

ament_export_include_directories(include)
if(WMX3_SDK_FOUND)
  ament_export_libraries(wmx_system_hardware)
endif()
ament_package()
