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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController(); 
    ~DiffDriveController(); 

    int leftAxis_;
    int rightAxis_;
    int leftAxisPolarity_;
    int rightAxisPolarity_;

    int rate_;
    double gearNumerator_;
    double gearDenumerator_;
    double accTime_;
    double decTime_;
    double wheelRadius_;
    double wheelToWheel_;

    std::string cmdVelTopic_;
    std::string encoderVelTopic_;
    std::string encoderOmegaTopic_;
    std::string encoderOdometeryTopic_;

    std::chrono::milliseconds cmdVelPeriod_;
    std::chrono::milliseconds encoderOmegaPeriod_;
    std::chrono::milliseconds encoderOdometryPeriod_;
    
    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;            
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;    
    
    std::vector<double> cmdOmega_;
    std::vector<double> encoderOmega_;
    std::vector<double> encoderOdometry_;
    
    geometry_msgs::msg::Twist cmdVelMsg_;
    std_msgs::msg::Float64MultiArray encoderOmegaMsg_;
    nav_msgs::msg::Odometry encoderOdometryMsg_;
    
    rclcpp::TimerBase::SharedPtr cmdVelTimer_; 
    rclcpp::TimerBase::SharedPtr encoderOmegaTimer_;
    rclcpp::TimerBase::SharedPtr encoderOdometryTimer_;  

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr encoderVelPub_;  
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr encoderOmegaPub_;  
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoderOdometeryPub_; 
    
    wmx3Api::Velocity::VelCommand m_velocity = wmx3Api::Velocity::VelCommand();

    void cmdVelStep(); 
    void encoderOmegaStep();
    void encoderOdometryStep();

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    std::vector<double> cmdCalculateOmega(double cmdLinearX, double cmdOmegaZ);
    std::vector<double> encoderCalculateOdometry(double omegaLeft, double omegaRight);

    void setRosParameter();
    void startEngine();
    void stopEngine();
    void startCommunication();
    void stopCommunication();
    void setServoOn(int axis);
    void setServoOff(int axis);
    void clearAlarm(int axis);
    void setEncoderMode(int axis);
    void setServoPolarity(int axis);
    void setGearRatio(int axis, double numerator, double denumerator);
    void setAxisMode(int axis); 
    void setPolarity(int axis, int polarity);
    void setVelocity(int axis, double omega);
};


DiffDriveController::DiffDriveController() : Node("diff_drive_controller"), wmx3LibCm_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "start diff_drive_interface");

    setRosParameter();
    startEngine();  
    
    startCommunication();
    
    clearAlarm(leftAxis_);
    clearAlarm(rightAxis_);

    setEncoderMode(leftAxis_);
    setEncoderMode(rightAxis_);

    setPolarity(leftAxis_, leftAxisPolarity_);
    setPolarity(rightAxis_, rightAxisPolarity_);

    setGearRatio(leftAxis_, gearNumerator_, gearDenumerator_);
    setGearRatio(rightAxis_, gearNumerator_, gearDenumerator_);

    setAxisMode(leftAxis_);
    setAxisMode(rightAxis_);   

    setServoOn(leftAxis_);
    setServoOn(rightAxis_);

    cmdVelPeriod_ = std::chrono::milliseconds(1000 / rate_);
    encoderOmegaPeriod_ = std::chrono::milliseconds(1000 / rate_);
    encoderOdometryPeriod_ = std::chrono::milliseconds(1000 / rate_);
    
    cmdVelTimer_ = this->create_wall_timer(cmdVelPeriod_, std::bind(&DiffDriveController::cmdVelStep, this));
    encoderOmegaTimer_ = this->create_wall_timer(encoderOmegaPeriod_, std::bind(&DiffDriveController::encoderOmegaStep, this));
    encoderOdometryTimer_ = this->create_wall_timer(encoderOdometryPeriod_, std::bind(&DiffDriveController::encoderOdometryStep, this));

    cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmdVelTopic_, 1, std::bind(&DiffDriveController::cmdCallback, this, _1));
    encoderVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>(encoderVelTopic_, 1); 
    encoderOmegaPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(encoderOmegaTopic_, 1);
    encoderOdometeryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(encoderOdometeryTopic_, 1); 

    RCLCPP_INFO(this->get_logger(), "diff_drive_interface ready");
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

DiffDriveController::~DiffDriveController(){
    RCLCPP_INFO(this->get_logger(), "Stop amr_ros2_interface");

    setVelocity(leftAxis_, 0.0);
    setVelocity(rightAxis_, 0.0);

    setServoOff(leftAxis_);
    setServoOff(rightAxis_);

    stopCommunication();
    stopEngine();
    
    RCLCPP_INFO(this->get_logger(), "diff_drive_interface stopped");
}

void DiffDriveController::setRosParameter(){
    this->declare_parameter<int>("left_axis", 0);
    this->declare_parameter<int>("right_axis", 1);
    this->declare_parameter<int>("left_axis_polarity", 1);
    this->declare_parameter<int>("right_axis_polarity", 1);

    this->declare_parameter<int>("rate", 10);
    this->declare_parameter<double>("gear_numerator", 100000000.0);
    this->declare_parameter<double>("gear_denumerator", 36.0);
    this->declare_parameter<double>("acc_time", 10000.0);
    this->declare_parameter<double>("dec_time", 10000.0);
    this->declare_parameter<double>("wheel_radius", 0.09);
    this->declare_parameter<double>("wheel_to_wheel", 0.55);

    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("encoder_vel_topic", "/cmd_vel_check");
    this->declare_parameter<std::string>("encoder_omega_topic", "/velocity_controller/commands");
    this->declare_parameter<std::string>("encoder_odometry_topic", "/odom_enc");

    this->get_parameter("left_axis", leftAxis_);
    this->get_parameter("right_axis", rightAxis_);
    this->get_parameter("left_axis_polarity", leftAxisPolarity_);
    this->get_parameter("right_axis_polarity", rightAxisPolarity_);

    this->get_parameter("rate", rate_);
    this->get_parameter("gear_numerator", gearNumerator_);
    this->get_parameter("gear_denumerator", gearDenumerator_);
    this->get_parameter("acc_time", accTime_);
    this->get_parameter("dec_time", decTime_);
    this->get_parameter("wheel_radius", wheelRadius_);
    this->get_parameter("wheel_to_wheel", wheelToWheel_);

    this->get_parameter("cmd_vel_topic", cmdVelTopic_);
    this->get_parameter("encoder_vel_topic", encoderVelTopic_);
    this->get_parameter("encoder_omega_topic", encoderOmegaTopic_);
    this->get_parameter("encoder_odometry_topic", encoderOdometeryTopic_);
}

void DiffDriveController::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmdVelMsg_ = *msg;
    encoderVelPub_->publish(cmdVelMsg_);
}

void DiffDriveController::cmdVelStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    CoreMotionAxisStatus *cmAxis_left_status = &cmStatus_.axesStatus[leftAxis_];
    CoreMotionAxisStatus *cmAxis_right_status = &cmStatus_.axesStatus[rightAxis_];

    if(cmStatus_.engineState == wmx3Api::EngineState::T::Communicating){
            
        if(!cmAxis_left_status->ampAlarm && !cmAxis_right_status->ampAlarm){   
                
            if(cmAxis_left_status->servoOn && cmAxis_right_status->servoOn){
                cmdOmega_ = cmdCalculateOmega(cmdVelMsg_.linear.x, cmdVelMsg_.angular.z);

                setVelocity(leftAxis_, cmdOmega_[0]);
                setVelocity(rightAxis_, cmdOmega_[1]);
            }
                
            else{
                RCLCPP_WARN(this->get_logger(), "Servo off. Please set servo on");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
            
        else{
            RCLCPP_WARN(this->get_logger(), "Servo alarm on. Please clear servo alarm");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    else{
        RCLCPP_WARN(this->get_logger(), "Communication or engine off. Please start the engine or communication");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void DiffDriveController::encoderOmegaStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    CoreMotionAxisStatus *cmAxis_left = &cmStatus_.axesStatus[leftAxis_];
    CoreMotionAxisStatus *cmAxis_right = &cmStatus_.axesStatus[rightAxis_];    

    encoderOmega_ = {cmAxis_left->actualVelocity, cmAxis_right->actualVelocity};
    
    encoderOmegaMsg_.data.clear();
    encoderOmegaMsg_.data.push_back(encoderOmega_[0]);
    encoderOmegaMsg_.data.push_back(encoderOmega_[1]);
    encoderOmegaPub_->publish(encoderOmegaMsg_);
}

void DiffDriveController::encoderOdometryStep() {
    encoderOdometry_ = encoderCalculateOdometry(encoderOmega_[0], encoderOmega_[1]);

    encoderOdometryMsg_.header.stamp = this->get_clock()->now();
    encoderOdometryMsg_.twist.twist.linear.x = encoderOdometry_[0];
    encoderOdometryMsg_.twist.twist.linear.y = 0.0;
    encoderOdometryMsg_.twist.twist.angular.z = encoderOdometry_[1];
    encoderOdometeryPub_->publish(encoderOdometryMsg_);
}

std::vector<double> DiffDriveController::cmdCalculateOmega(double cmdLinearX, double cmdOmegaZ) {
    return {(2 * cmdLinearX - cmdOmegaZ * wheelToWheel_) / (2 * wheelRadius_), 
            (2 * cmdLinearX + cmdOmegaZ * wheelToWheel_) / (2 * wheelRadius_)};
}

std::vector<double> DiffDriveController::encoderCalculateOdometry(double omegaLeft, double omegaRight) {
    return {((omegaRight * wheelRadius_) + (omegaLeft * wheelRadius_)) / 2.0, 
            ((omegaRight * wheelRadius_) - (omegaLeft * wheelRadius_)) / wheelToWheel_};
}

void DiffDriveController::setVelocity(int axis, double omega){
    m_velocity.axis = axis;
    m_velocity.profile.velocity = omega;
    
    m_velocity.profile.type = ProfileType::T::TimeAccTrapezoidal;
    m_velocity.profile.accTimeMilliseconds = accTime_;
    m_velocity.profile.decTimeMilliseconds = decTime_;

    err_ = wmx3LibCm_.velocity->StartVel(&m_velocity);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to move motor %d. Error=%d (%s)", axis, err_, errString_);
    }
}

void DiffDriveController::clearAlarm(int axis){
    err_ = wmx3LibCm_.axisControl->ClearAmpAlarm(axis);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to clear alarm axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Clear alarm axis %d", axis);
    }
}

void DiffDriveController::setEncoderMode(int axis){
    err_ = wmx3LibCm_.config->SetAbsoluteEncoderMode(axis, false);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set encoder mode axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set encoder mode axis %d", axis);
    }
}

void DiffDriveController::setGearRatio(int axis, double numerator, double denumerator){
    err_ = wmx3LibCm_.config->SetGearRatio(axis, numerator, denumerator);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set gear ratio axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set gear ratio axis %d \t %.6f \t %.6f", axis, numerator, denumerator);
    }
}

void DiffDriveController::setPolarity(int axis, int polarity){
    err_ = wmx3LibCm_.config->SetAxisPolarity(axis, polarity);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set polarity axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set polarity axis %d \t %d", axis, polarity);
    }
}

void DiffDriveController::setAxisMode(int axis){
    err_ = wmx3LibCm_.axisControl->SetAxisCommandMode(axis, AxisCommandMode::Velocity);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set axis %d command mode to velocity. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set velocity mode axis %d", axis);
    }
}

void DiffDriveController::setServoOff(int axis){
    err_ = wmx3LibCm_.axisControl->SetServoOn(axis, 0);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Servo %d error to off. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Servo %d off", axis);
    }
}

void DiffDriveController::setServoOn(int axis){
    err_ = wmx3LibCm_.axisControl->SetServoOn(axis, 1);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Servo %d error to on. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Servo %d on", axis);
    }
}

void DiffDriveController::startEngine(){
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

void DiffDriveController::startCommunication(){
    err_ = wmx3Lib_.StartCommunication(INFINITE);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to start communication. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Start communication");
    }
}

void DiffDriveController::stopEngine(){
    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Device stopped");
    }
}

void DiffDriveController::stopCommunication(){
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
    rclcpp::spin(std::make_shared<DiffDriveController>());
    rclcpp::shutdown();
    return 0;
}