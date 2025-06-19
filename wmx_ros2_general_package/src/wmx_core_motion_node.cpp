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

class WmxCoreMotion : public rclcpp::Node {
public:
    WmxCoreMotion(); 
    ~WmxCoreMotion(); 
    
    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;    
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;    

    /*
    void setAxisStatus();
    void getAxisStatus();
    void clearAlarm();
    void getArmStatus();

    void setAxisMode();
    void getAxisMode();
    void setAxisPolarity();
    void getAxisPolarity();
    void setAxisGearRatio();
    void getAxisGearRatio();
    void setAxisEncoderMode();
    void getAxisEncoderMode();

    void setHoming();
    void getHoming();
    
    void setAxisPosition();
    void setAxisVelocity();
    */
};

WmxCoreMotion::WmxCoreMotion() : Node("wmx_core_motion_node"), wmx3LibCm_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "Start wmx_core_motion_node");

    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node is ready");
}

WmxCoreMotion::~WmxCoreMotion(){
    RCLCPP_INFO(this->get_logger(), "Stop wmx_core_motion_node");

    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "wmx_core_motion_node stopped");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WmxCoreMotion>());
    rclcpp::shutdown();
    return 0;
}
