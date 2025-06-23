#include "wmx_ros2_core_motion.hpp"

#include <thread>
#include <chrono>

WmxRos2CoreMotion::WmxRos2CoreMotion() : Node("wmx_ros2_core_motion_node"), wmx3LibCm_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "wmx_ros2_core_motion_node is ready");
}

WmxRos2CoreMotion::~WmxRos2CoreMotion(){
    RCLCPP_INFO(this->get_logger(), "Stop wmx_ros2_core_motion_node");

    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "wmx_ros2_core_motion_node stopped");
}