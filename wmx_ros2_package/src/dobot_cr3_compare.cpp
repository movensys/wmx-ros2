#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

#define WMX_PARAM_FILE_PATH "/home/mic-713/wmx_ros2_ws/src/wmx_ros2_application/wmx_ros2_package/config/cr3a_wmx_parameters.xml"

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class Cr3aRobot : public rclcpp::Node {
public:
    Cr3aRobot(); 
    ~Cr3aRobot(); 

    int jointNumber_;
    std::vector<std::string> jointNames_;
    int jointFeedbackRate_;
    std::string encoderJointTopic_;
    std_msgs::msg::Float64MultiArray cmdJointMsg_;

    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;            
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;

    wmx3Api::Motion::LinearIntplCommand lin_ = wmx3Api::Motion::LinearIntplCommand();
    
    rclcpp::TimerBase::SharedPtr encoderJointTimer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoderJointPub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmdJointSub_;
    
    void encoderJointStep();
    void cmdJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void setRosParameter();
    void startEngine();
    void stopEngine();
    void startCommunication();
    void stopCommunication();
    void setServoOn(int axis);
    void setServoOff(int axis);
    void clearAlarm(int axis);
};

Cr3aRobot::Cr3aRobot() : Node("cr3a_robot_node"), wmx3LibCm_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "start cr3a_robot_node");

    setRosParameter();
    startEngine();  

    startCommunication();

    wmx3LibCm_.config->ImportAndSetAll((char*)WMX_PARAM_FILE_PATH);

    for(int i=0; i<jointNumber_;i++){
        clearAlarm(i);
        setServoOn(i);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    encoderJointTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / jointFeedbackRate_), std::bind(&Cr3aRobot::encoderJointStep, this)); 

    encoderJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(encoderJointTopic_, 1);  

    cmdJointSub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/mvsk/trajectory", 1, std::bind(&Cr3aRobot::cmdJointCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "cr3a_robot_node ready");
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

Cr3aRobot::~Cr3aRobot(){
    RCLCPP_INFO(this->get_logger(), "Stop cr3a_robot_node");

    for(int i=0; i<jointNumber_;i++){
        setServoOff(i);
    }

    stopCommunication();
    stopEngine();
    
    RCLCPP_INFO(this->get_logger(), "cr3a_robot_node stopped");
}

void Cr3aRobot::cmdJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    cmdJointMsg_ = *msg;

    lin_.axisCount = 6;
    for (int i = 0; i < 6; ++i) {
        lin_.axis[i] = i;  
        lin_.target[i] = cmdJointMsg_.data[i];      
    }

    lin_.profile.type = ProfileType::Trapezoidal;
    lin_.profile.velocity = 0.1;
    lin_.profile.acc = 0.1;
    lin_.profile.dec = 0.1;

    wmx3LibCm_.motion->StartLinearIntplPos(&lin_);
}

void Cr3aRobot::setRosParameter(){
    this->declare_parameter<int>("joint_number", 1);
    this->declare_parameter<int>("joint_feedback_rate", 10);
    this->declare_parameter<std::vector<std::string>>("joint_name", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    this->declare_parameter<std::string>("encoder_joint_topic", "/joint_states");
    
    this->get_parameter("joint_number", jointNumber_);
    this->get_parameter("joint_name", jointNames_);
    this->get_parameter("joint_feedback_rate", jointFeedbackRate_);
    this->get_parameter("encoder_joint_topic", encoderJointTopic_);
}

void Cr3aRobot::encoderJointStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    std::vector<CoreMotionAxisStatus*> cmAxisStatus_(jointNumber_);
    for (int i = 0; i < jointNumber_; ++i) {
        cmAxisStatus_[i] = &cmStatus_.axesStatus[i];
    }

    sensor_msgs::msg::JointState encoderJointMsg_;
    encoderJointMsg_.header.stamp = this->get_clock()->now();
 
    for (int i = 0; i < jointNumber_; ++i) {
        encoderJointMsg_.name.push_back(jointNames_[i]);
        encoderJointMsg_.position.push_back(cmAxisStatus_[i]->actualPos);
    }

    encoderJointPub_->publish(encoderJointMsg_);
    
    cout<<"Current Joint State"<<endl;
    for (int i = 0; i < jointNumber_; ++i) {
        cout<<"state: "<<cmAxisStatus_[i]->actualPos<<endl;
    }
    cout<<endl;
}

void Cr3aRobot::clearAlarm(int axis){
    err_ = wmx3LibCm_.axisControl->ClearAmpAlarm(axis);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to clear alarm axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Clear alarm axis %d", axis);
    }
}

void Cr3aRobot::setServoOn(int axis){
    err_ = wmx3LibCm_.axisControl->SetServoOn(axis, 1);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Servo %d error to on. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Servo %d on", axis);
    }
}

void Cr3aRobot::setServoOff(int axis){
    err_ = wmx3LibCm_.axisControl->SetServoOn(axis, 0);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Servo %d error to off. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Servo %d off", axis);
    }
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
