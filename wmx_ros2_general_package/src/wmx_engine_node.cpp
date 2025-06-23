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
    
    rclcpp::Service<wmx_ros2_message::srv::SetEngine>::SharedPtr setEngineService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCommService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr getEngineStatusService_;

    void stopEngine();
    void stopCommunication();

    void setEngine(const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
                    std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response);
    void setComm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);             
    void getEngineStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);   
};

WmxEngine::WmxEngine() : Node("wmx_engine_node") {  
    RCLCPP_INFO(this->get_logger(), "Start wmx_engine_node");

    setEngineService_ = this->create_service<wmx_ros2_message::srv::SetEngine>("/wmx/engine/set_engine", std::bind(&WmxEngine::setEngine, this, _1, _2));
    setCommService_ = this->create_service<std_srvs::srv::SetBool>("/wmx/engine/set_comm", std::bind(&WmxEngine::setComm, this, _1, _2));
    getEngineStatusService_ = this->create_service<std_srvs::srv::Trigger>("/wmx/engine/get_engine_status", std::bind(&WmxEngine::getEngineStatus, this, _1, _2));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "wmx_engine_node is ready");
}

WmxEngine::~WmxEngine(){
    RCLCPP_INFO(this->get_logger(), "Stop wmx_engine_node");

    stopCommunication();
    stopEngine();

    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "wmx_engine_node stopped");
}

void WmxEngine::getEngineStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    wmx3Api::EngineStatus status; 
    wmx3Lib_.GetEngineStatus(&status);

    std::string status_str;

    switch (status.state) { 
        case wmx3Api::EngineState::Idle:           status_str = "Idle"; break;
        case wmx3Api::EngineState::Running:        status_str = "Running"; break;
        case wmx3Api::EngineState::Communicating:  status_str = "Communicating"; break;
        case wmx3Api::EngineState::Shutdown:       status_str = "Shutdown"; break;
        case wmx3Api::EngineState::Unknown:        status_str = "Unknown"; break;
        default:                                   status_str = "Invalid"; break;
    }

    response->success = true;
    response->message = status_str;
}

void WmxEngine::setComm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        err_ = wmx3Lib_.StartCommunication(INFINITE);

        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to start communication. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Communication is started");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    } 
    else {
        err_ = wmx3Lib_.StopCommunication(INFINITE);;
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to stop communication. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_);
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Communication is stopped");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
} 

void WmxEngine::setEngine(const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
                          std::shared_ptr<wmx_ros2_message::srv::SetEngine::Response> response) {
    if (request->data) {
        err_ = wmx3Lib_.CreateDevice(request->path.c_str(), DeviceType::DeviceTypeNormal, INFINITE);
        wmx3Lib_.SetDeviceName(request->name.c_str());

        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to create device. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Created a device with name: %s", request->name.c_str());
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    } 
    else {
        err_ = wmx3Lib_.CloseDevice();
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to close device. Error=%d (%s)", err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_);
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Device stopped");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
} 

void WmxEngine::stopEngine(){
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Device stopped");
    }
}

void WmxEngine::stopCommunication(){
    err_ = wmx3Lib_.StopCommunication(INFINITE);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to stop communication");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Communication stopped");
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WmxEngine>());
    rclcpp::shutdown();
    return 0;
}
