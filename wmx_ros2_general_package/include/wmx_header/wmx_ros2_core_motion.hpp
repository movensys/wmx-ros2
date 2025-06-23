#ifndef WMX_ROS2_CORE_MOTION_HPP
#define WMX_ROS2_CORE_MOTION_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <bits/stdc++.h>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "wmx_ros2_message/srv/set_axis.hpp"
#include "wmx_ros2_message/srv/set_axis_mode.hpp"
#include "wmx_ros2_message/msg/axis_velocity.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace wmx3Api;
using namespace std;

class WmxRos2CoreMotion : public rclcpp::Node {
public:
    WmxRos2CoreMotion(); 
    ~WmxRos2CoreMotion(); 

private:
    int err_;
    char errString_[256];
    char buffer_[512];

    WMX3Api wmx3Lib_;    
    CoreMotion wmx3LibCm_;    
    CoreMotionStatus cmStatus_;

    wmx3Api::Velocity::VelCommand velocity_ = wmx3Api::Velocity::VelCommand();

    rclcpp::Subscription<wmx_ros2_message::msg::AxisVelocity>::SharedPtr axisVelSub_ = 
                    this->create_subscription<wmx_ros2_message::msg::AxisVelocity>("/wmx/axis/velocity",
                    1, std::bind(&WmxRos2CoreMotion::axisVelCallback, this, _1));

    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisOnService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_axis_on", 
                    std::bind(&WmxRos2CoreMotion::setAxisOn, this, _1, _2));    
    
    rclcpp::Service<wmx_ros2_message::srv::SetAxisMode>::SharedPtr setAxisModeService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxisMode>("/wmx/axis/set_axis_mode", 
                    std::bind(&WmxRos2CoreMotion::setAxisMode, this, _1, _2));    

    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr clearAlarmService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/clear_alarm", 
                    std::bind(&WmxRos2CoreMotion::clearAlarm, this, _1, _2));    
    
    void axisVelCallback(const wmx_ros2_message::msg::AxisVelocity::SharedPtr msg);

    void setAxisOn(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    
    void setAxisMode(const std::shared_ptr<wmx_ros2_message::srv::SetAxisMode::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxisMode::Response> response);

    void clearAlarm(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
};

#endif  // WMX_ROS2_CORE_MOTION_HPP
