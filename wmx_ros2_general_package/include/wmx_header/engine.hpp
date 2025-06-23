#ifndef ENGINE_HPP
#define ENGINE_HPP

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
#include "wmx_ros2_message/srv/set_engine.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "WMX3Api.h"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace wmx3Api;
using namespace std;

class WmxEngine : public rclcpp::Node {
public:
    WmxEngine(); 
    ~WmxEngine(); 

private:
    int err_;
    char errString_[256];
    char buffer_[512];

    WMX3Api wmx3Lib_;      
    
    rclcpp::Service<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineService_ = 
                    this->create_service<wmx_ros2_message::srv::SetEngine>("/wmx/engine/set_engine", 
                    std::bind(&WmxEngine::setEngine, this, _1, _2));

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_ =
                    this->create_service<std_srvs::srv::SetBool>("/wmx/engine/set_comm", 
                    std::bind(&WmxEngine::setComm, this, _1, _2));
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_ = 
                    this->create_service<std_srvs::srv::Trigger>("/wmx/engine/get_engine_status", 
                    std::bind(&WmxEngine::getEngineStatus, this, _1, _2));

    void stopEngine();
    void stopCommunication();

    void setEngine(const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response);
    void setComm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);             
    void getEngineStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);   
};

#endif  // ENGINE_HPP