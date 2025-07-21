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

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class Cr3aRobot : public rclcpp::Node {
public:
    Cr3aRobot(); 
    ~Cr3aRobot(); 

    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;            
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;    
    
    void startEngine();
    void stopEngine();
    void startCommunication();
    void stopCommunication();
};


Cr3aRobot::Cr3aRobot() : Node("cr3a_robot_node"), wmx3LibCm_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "start cr3a_robot_node");

    startEngine();  
    startCommunication();
    
    RCLCPP_INFO(this->get_logger(), "cr3a_robot_node ready");
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

Cr3aRobot::~Cr3aRobot(){
    RCLCPP_INFO(this->get_logger(), "Stop cr3a_robot_node");

    stopCommunication();
    stopEngine();
    
    RCLCPP_INFO(this->get_logger(), "cr3a_robot_node stopped");
}

void Cr3aRobot::startEngine(){
    err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, INFINITE);
    wmx3Lib_.SetDeviceName("DiffDriveROS2");
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to create device. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Created a device");
    }
}

void Cr3aRobot::startCommunication(){
    err_ = wmx3Lib_.StartCommunication(INFINITE);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to start communication. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Start communication");
    }
}

void Cr3aRobot::stopEngine(){
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Device stopped");
    }
}

void Cr3aRobot::stopCommunication(){
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
    rclcpp::spin(std::make_shared<Cr3aRobot>());
    rclcpp::shutdown();
    return 0;
}
