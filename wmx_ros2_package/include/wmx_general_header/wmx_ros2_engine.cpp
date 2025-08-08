#include "wmx_ros2_general.hpp"

#include <thread>
#include <chrono>

void WmxRos2General::getEngineStatus(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
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

void WmxRos2General::setComm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
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

void WmxRos2General::setEngine(const std::shared_ptr<wmx_ros2_message::srv::SetEngine::Request> request,
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

void WmxRos2General::stopEngine(){
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Device stopped");
    }
}

void WmxRos2General::stopCommunication(){
    err_ = wmx3Lib_.StopCommunication(INFINITE);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to stop communication");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Communication stopped");
    }
}