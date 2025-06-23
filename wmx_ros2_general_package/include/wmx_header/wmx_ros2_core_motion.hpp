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

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class WmxRos2CoreMotion : public rclcpp::Node {
public:
    WmxRos2CoreMotion(); 
    ~WmxRos2CoreMotion(); 
    
    int err_;
    char errString_[256];

private:
    //WMX3Api wmx3Lib_;    
    //CoreMotion wmx3LibCm_;
    //CoreMotionStatus cmStatus_;      

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

#endif  // WMX_ROS2_ENGINE_HPP


