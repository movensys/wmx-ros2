#include "wmx_ros2_general.hpp"

#include <thread>
#include <chrono>

WmxRos2General::WmxRos2General() : Node("wmx_ros2_general_node"), wmx3LibCm_(&wmx3Lib_) {  
    axisStatePeriod_ = std::chrono::milliseconds(1000 / rate_);
    axisStateTimer_ = this->create_wall_timer(axisStatePeriod_, std::bind(&WmxRos2General::axisStateStep, this));

    startEngine();
    RCLCPP_INFO(this->get_logger(), "wmx_ros2_general_node is ready");
}

WmxRos2General::~WmxRos2General(){
    stopCommunication();
    stopEngine();

    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "wmx_ros2_general_node is stopped");
}