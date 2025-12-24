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

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "IOApi.h"

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class ManipulatorState : public rclcpp::Node {
public:
    ManipulatorState(); 
    ~ManipulatorState(); 

    int jointNumber_;
    int jointFeedbackRate_;
    std::vector<std::string> jointNames_;
    std::string encoderJointTopic_;
    std::string isaacsimJointTopic_;
    std::string wmxParamFilePath_;
    float gripperCloseValue_;
    float gripperOpenValue_;

    unsigned char outData_;
    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;            
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;
    Io Wmx3Lib_Io_;
    
    rclcpp::TimerBase::SharedPtr encoderJointTimer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoderJointPub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr isaacsimJointPub_;
    void encoderJointStep();
    void setRosParameter();

    void startEngine();
    void stopEngine();
    void setWmxParam(char* path);
    void startCommunication();
    void stopCommunication();
    void setServoOn(int axis);
    void setServoOff(int axis);
    void clearAlarm(int axis);
};

ManipulatorState::ManipulatorState() : Node("manipulator_state_node"), wmx3LibCm_(&wmx3Lib_), Wmx3Lib_Io_(&wmx3Lib_)  {  
    RCLCPP_INFO(this->get_logger(), "start manipulator_state_node");

    setRosParameter();

    startEngine();
    startCommunication();
    setWmxParam((char*)wmxParamFilePath_.c_str());

    for(int i=0; i<jointNumber_;i++){
        clearAlarm(i);
        setServoOn(i);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    encoderJointTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / jointFeedbackRate_), 
                                                 std::bind(&ManipulatorState::encoderJointStep, this)); 

    encoderJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(encoderJointTopic_, 1);
    isaacsimJointPub_ = this->create_publisher<sensor_msgs::msg::JointState>(isaacsimJointTopic_, 1);  

    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "manipulator_state_node is ready");
}

ManipulatorState::~ManipulatorState(){
    RCLCPP_INFO(this->get_logger(), "Stop manipulator_state_node");

    for(int i=0; i<jointNumber_;i++){
        setServoOff(i);
    }

    stopCommunication();
    stopEngine();
    
    RCLCPP_INFO(this->get_logger(), "manipulator_state_node is stopped");
}

void ManipulatorState::setRosParameter(){
    this->declare_parameter<int>("joint_number", 0);
    this->declare_parameter<int>("joint_feedback_rate", 0);
    this->declare_parameter<float>("gripper_open_value", 0);
    this->declare_parameter<float>("gripper_close_value", 0);
    this->declare_parameter<std::vector<std::string>>("joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
    this->declare_parameter<std::string>("encoder_joint_topic", "/manipulator_node/no_param");
    this->declare_parameter<std::string>("isaacsim_joint_topic", "/manipulator_node/no_param");
    this->declare_parameter<std::string>("wmx_param_file_path", "");

    this->get_parameter("joint_number", jointNumber_);
    this->get_parameter("joint_feedback_rate", jointFeedbackRate_);
    this->get_parameter("gripper_open_value", gripperOpenValue_);
    this->get_parameter("gripper_close_value", gripperCloseValue_);
    this->get_parameter("joint_name", jointNames_);
    this->get_parameter("encoder_joint_topic", encoderJointTopic_);
    this->get_parameter("isaacsim_joint_topic", isaacsimJointTopic_);
    this->get_parameter("wmx_param_file_path", wmxParamFilePath_);
}

void ManipulatorState::encoderJointStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    std::vector<CoreMotionAxisStatus*> cmAxisStatus_(jointNumber_);
    for (int i = 0; i < jointNumber_; ++i) {
        cmAxisStatus_[i] = &cmStatus_.axesStatus[i];
    }

    sensor_msgs::msg::JointState encoderJointMsg_;
 
    for (int i = 0; i < jointNumber_; ++i) {
        encoderJointMsg_.name.push_back(jointNames_[i]);
        encoderJointMsg_.position.push_back(cmAxisStatus_[i]->actualPos);
        encoderJointMsg_.velocity.push_back(cmAxisStatus_[i]->actualVelocity);
    }

    for (int i = 0; i < 2; ++i) {
        encoderJointMsg_.name.push_back(jointNames_[6+i]);
        Wmx3Lib_Io_.GetOutBit(0, 0, &outData_);
        if(outData_){
            encoderJointMsg_.position.push_back(gripperCloseValue_);
            encoderJointMsg_.velocity.push_back(0.000);
        }
        else{
            encoderJointMsg_.position.push_back(gripperOpenValue_);
            encoderJointMsg_.velocity.push_back(0.000);
        }        
    }

    isaacsimJointPub_->publish(encoderJointMsg_);
    encoderJointMsg_.header.stamp = this->get_clock()->now();
    encoderJointPub_->publish(encoderJointMsg_);
}

void ManipulatorState::setWmxParam(char* path){
    err_ = wmx3LibCm_.config->ImportAndSetAll(path);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set WMX params. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Success to set WMX params");
    }
}

void ManipulatorState::clearAlarm(int axis){
    err_ = wmx3LibCm_.axisControl->ClearAmpAlarm(axis);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to clear alarm axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Clear alarm axis %d", axis);
    }
}

void ManipulatorState::setServoOn(int axis){
    err_ = wmx3LibCm_.axisControl->SetServoOn(axis, 1);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Servo %d error to on. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Servo %d on", axis);
    }
}

void ManipulatorState::setServoOff(int axis){
    err_ = wmx3LibCm_.axisControl->SetServoOn(axis, 0);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Servo %d error to off. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Servo %d off", axis);
    }
}

void ManipulatorState::startEngine(){
    err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, INFINITE);
    wmx3Lib_.SetDeviceName("ManipulatorState");
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to create device. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Created a device");
    }
}

void ManipulatorState::startCommunication(){
    err_ = wmx3Lib_.StartCommunication(INFINITE);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to start communication. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Start communication");
    }
}

void ManipulatorState::stopEngine(){
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Device stopped");
    }
}

void ManipulatorState::stopCommunication(){
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
    rclcpp::spin(std::make_shared<ManipulatorState>());
    rclcpp::shutdown();
    return 0;
}