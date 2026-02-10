#ifndef WMX_ROS2_GENERAL_HPP
#define WMX_ROS2_GENERAL_HPP

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
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_ros2_message/srv/set_engine.hpp"
#include "wmx_ros2_message/srv/set_axis.hpp"
#include "wmx_ros2_message/srv/set_axis_gear_ratio.hpp"
#include "wmx_ros2_message/msg/axis_velocity.hpp"
#include "wmx_ros2_message/msg/axis_state.hpp"
#include "wmx_ros2_message/msg/axis_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace wmx3Api;
using namespace std;

class WmxRos2General : public rclcpp::Node {
public:
    WmxRos2General(); 
    ~WmxRos2General(); 

private:
    int axisCount_ = 2;
    int err_;
    char errString_[256];
    char buffer_[512];

    const int rate_ = 100;
    std::chrono::milliseconds axisStatePeriod_;
    rclcpp::TimerBase::SharedPtr axisStateTimer_;
    wmx_ros2_message::msg::AxisState axisStateMsg_;

    WMX3Api wmx3Lib_;      
    CoreMotion wmx3LibCm_;    
    CoreMotionStatus cmStatus_;

    wmx3Api::Velocity::VelCommand velocity_;
    wmx3Api::Motion::PosCommand position_;
    Config::HomeParam homeParam_;
    
    rclcpp::Service<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineService_ = 
                    this->create_service<wmx_ros2_message::srv::SetEngine>("/wmx/engine/set_device", 
                    std::bind(&WmxRos2General::setEngine, this, _1, _2));

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_ =
                    this->create_service<std_srvs::srv::SetBool>("/wmx/engine/set_comm", 
                    std::bind(&WmxRos2General::setComm, this, _1, _2));
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_ = 
                    this->create_service<std_srvs::srv::Trigger>("/wmx/engine/get_status", 
                    std::bind(&WmxRos2General::getEngineStatus, this, _1, _2));
    
    rclcpp::Publisher<wmx_ros2_message::msg::AxisState>::SharedPtr axisStatePub_ = 
                    this->create_publisher<wmx_ros2_message::msg::AxisState>("/wmx/axis/state", 1); 

    rclcpp::Subscription<wmx_ros2_message::msg::AxisVelocity>::SharedPtr axisVelSub_ = 
                    this->create_subscription<wmx_ros2_message::msg::AxisVelocity>("/wmx/axis/velocity",
                    1, std::bind(&WmxRos2General::axisVelCallback, this, _1));

    rclcpp::Subscription<wmx_ros2_message::msg::AxisPose>::SharedPtr axisPoseSub_ = 
                    this->create_subscription<wmx_ros2_message::msg::AxisPose>("/wmx/axis/position",
                    1, std::bind(&WmxRos2General::axisPoseCallback, this, _1));
    
    rclcpp::Subscription<wmx_ros2_message::msg::AxisPose>::SharedPtr axisPoseRelativeSub_ = 
                    this->create_subscription<wmx_ros2_message::msg::AxisPose>("/wmx/axis/position/relative",
                    1, std::bind(&WmxRos2General::axisPoseRelativeCallback, this, _1));

    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisOnService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_on", 
                    std::bind(&WmxRos2General::setAxisOn, this, _1, _2));    
    
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr clearAlarmService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/clear_alarm", 
                    std::bind(&WmxRos2General::clearAlarm, this, _1, _2)); 

    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisModeService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_mode", 
                    std::bind(&WmxRos2General::setAxisMode, this, _1, _2));    
                    
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisPolarityService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/set_polarity", 
                    std::bind(&WmxRos2General::setAxisPolarity, this, _1, _2));    
    
    rclcpp::Service<wmx_ros2_message::srv::SetAxisGearRatio>::SharedPtr setAxisGearRatioService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxisGearRatio>("/wmx/axis/set_gear_ratio", 
                    std::bind(&WmxRos2General::setAxisGearRatio, this, _1, _2));
    
    rclcpp::Service<wmx_ros2_message::srv::SetAxis>::SharedPtr setHomingService_ = 
                    this->create_service<wmx_ros2_message::srv::SetAxis>("/wmx/axis/homing", 
                    std::bind(&WmxRos2General::setHoming, this, _1, _2));
    
    void startEngine();
    void stopEngine();
    void stopCommunication();
    void axisStateStep();

    void setEngine(const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response);
    void setComm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);             
    void getEngineStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response); 
    
    void axisPoseCallback(const wmx_ros2_message::msg::AxisPose::SharedPtr msg);
    void axisPoseRelativeCallback(const wmx_ros2_message::msg::AxisPose::SharedPtr msg);
    
    void axisVelCallback(const wmx_ros2_message::msg::AxisVelocity::SharedPtr msg);

    void setAxisOn(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
                    
    void setAxisMode(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
                
    void clearAlarm(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
                
    void setAxisPolarity(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
    
    void setHoming(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response);
                
    void setAxisGearRatio(const std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Response> response);    
};

#endif  // WMX_ROS2_GENERAL_HPP