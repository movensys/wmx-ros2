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

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class Cr3aRobot : public rclcpp::Node {
public:
    Cr3aRobot(); 
    ~Cr3aRobot(); 

    int axisNumber_;
    int rate_;

    long double jointMsg_[6];

    std::vector<int64_t> AxisPolarity_;
    std::vector<double> gearNumerator_;
    std::vector<double> gearDenumerator_;
    std::vector<double> omega_;
    std::vector<double> acc_;
    std::vector<double> dec_;

    std::string cmdJointTopic_;
    std::string encoderJointTopic_;

    bool isCommPrev_ = false;

    std::chrono::milliseconds cmdJointPeriod_;
    std::chrono::milliseconds encoderJointPeriod_;

    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;            
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;
    
    wmx3Api::Motion::PosCommand m_position = wmx3Api::Motion::PosCommand();

    sensor_msgs::msg::JointState cmdJointMsg_;
    std_msgs::msg::Float64MultiArray encoderJointMsg_;
    
    rclcpp::TimerBase::SharedPtr cmdJointTimer_; 
    rclcpp::TimerBase::SharedPtr encoderJointTimer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmdJointSub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr encoderJointPub_;  
    
    void cmdJointStep();
    void cmdJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void encoderJointStep();

    void setRosParameter();
    void startEngine();
    void stopEngine();
    void startCommunication();
    void stopCommunication();
    void setServoOn(int axis);
    void setServoOff(int axis);
    void clearAlarm(int axis);
    void setEncoderMode(int axis);
    void setAxisMode(int axis);
    void setPolarity(int axis, int polarity);
    void setGearRatio(int axis, double numerator, double denumerator);
    void setPosition(int axis, double position, double omega, double acc, double dec);
};

Cr3aRobot::Cr3aRobot() : Node("cr3a_robot_node"), wmx3LibCm_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "start cr3a_robot_node");

    setRosParameter();
    startEngine();  

    startCommunication();
    for(int i=0; i<axisNumber_;i++){
        setServoOn(i);
    }

    cmdJointPeriod_ = std::chrono::milliseconds(1000 / rate_);
    encoderJointPeriod_ = std::chrono::milliseconds(1000 / rate_);

    cmdJointTimer_ = this->create_wall_timer(cmdJointPeriod_, std::bind(&Cr3aRobot::cmdJointStep, this));
    encoderJointTimer_ = this->create_wall_timer(encoderJointPeriod_, std::bind(&Cr3aRobot::encoderJointStep, this));
    
    cmdJointSub_ = this->create_subscription<sensor_msgs::msg::JointState>(cmdJointTopic_, 1, std::bind(&Cr3aRobot::cmdJointCallback, this, _1));
    encoderJointPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(encoderJointTopic_, 1); 

    RCLCPP_INFO(this->get_logger(), "cr3a_robot_node ready");
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

Cr3aRobot::~Cr3aRobot(){
    RCLCPP_INFO(this->get_logger(), "Stop cr3a_robot_node");

    for(int i=0; i<axisNumber_;i++){
        setServoOff(i);
    }

    stopCommunication();
    stopEngine();
    
    RCLCPP_INFO(this->get_logger(), "cr3a_robot_node stopped");
}

void Cr3aRobot::setRosParameter(){
    this->declare_parameter<int>("axis_number", 6);
    this->declare_parameter<int>("rate", 10);
    this->declare_parameter<std::string>("cmd_joint_topic", "/joint_states");
    this->declare_parameter<std::string>("encoder_joint_topic", "/enc_joint");
    this->declare_parameter<std::vector<int64_t>>("axis_polarity", {1, 1, 1, 1, 1, 1});
    this->declare_parameter<std::vector<double>>("gear_numerator", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>("gear_denumerator", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>("omega", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("acc", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dec", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    
    this->get_parameter("axis_number", axisNumber_);
    this->get_parameter("rate", rate_);
    this->get_parameter("cmd_joint_topic", cmdJointTopic_);
    this->get_parameter("encoder_joint_topic", encoderJointTopic_);
    this->get_parameter("axis_polarity", AxisPolarity_);
    this->get_parameter("gear_numerator", gearNumerator_);
    this->get_parameter("gear_denumerator", gearDenumerator_);
    this->get_parameter("omega", omega_);
    this->get_parameter("acc", acc_);
    this->get_parameter("dec", dec_);

    size_t axis_num = static_cast<size_t>(axisNumber_);
    if (AxisPolarity_.size() != axis_num ||
        gearNumerator_.size() != axis_num ||
        gearDenumerator_.size() != axis_num ||
        omega_.size() != axis_num ||
        acc_.size() != axis_num ||
        dec_.size() != axis_num) {
        RCLCPP_FATAL(this->get_logger(), "Parameter size mismatch with axis_number");
        rclcpp::shutdown();
        return;
    }
}

void Cr3aRobot::encoderJointStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    std::vector<CoreMotionAxisStatus*> cmAxisStatus_(axisNumber_);
    for (int i = 0; i < axisNumber_; ++i) {
        cmAxisStatus_[i] = &cmStatus_.axesStatus[i];
    }

    encoderJointMsg_.data.clear();
 
    for (int i = 0; i < axisNumber_; ++i) {
        encoderJointMsg_.data.push_back(cmAxisStatus_[i]->actualPos);
    }

    encoderJointPub_->publish(encoderJointMsg_);
    
    cout<<"Current Joint State"<<endl;
    for (int i = 0; i < axisNumber_; ++i) {
        cout<<cmAxisStatus_[i]->actualPos<<endl;
    }
    cout<<endl;
    
    /*
    for (int i = 0; i < axisNumber_; ++i) {
        cout<<gearNumerator_[i]<<endl;
        cout<<gearDenumerator_[i]<<endl;
        cout<<omega_[i]<<endl;
        cout<<acc_[i]<<endl;
        cout<<dec_[i]<<endl;
    }
    cout<<endl;
    */
}

void Cr3aRobot::cmdJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    cmdJointMsg_ = *msg;

    for(int i=0; i<axisNumber_;i++){ 
        jointMsg_[i] = cmdJointMsg_.position[i];
    }
}

void Cr3aRobot::cmdJointStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    std::vector<CoreMotionAxisStatus*> cmAxisStatus_(axisNumber_);
    for (int i = 0; i < axisNumber_; ++i) {
        cmAxisStatus_[i] = &cmStatus_.axesStatus[i];
    }

    if(cmStatus_.engineState == wmx3Api::EngineState::T::Communicating && isCommPrev_ == true){

        if (!cmAxisStatus_[0]->ampAlarm && !cmAxisStatus_[1]->ampAlarm && !cmAxisStatus_[2]->ampAlarm && 
            !cmAxisStatus_[3]->ampAlarm && !cmAxisStatus_[4]->ampAlarm && !cmAxisStatus_[5]->ampAlarm){
            
            if( cmAxisStatus_[0]->servoOn && cmAxisStatus_[1]->servoOn && cmAxisStatus_[2]->servoOn &&
                cmAxisStatus_[3]->servoOn && cmAxisStatus_[4]->servoOn && cmAxisStatus_[5]->servoOn){
                
                for (int i = 0; i < axisNumber_; ++i) {
                    setPosition(i, jointMsg_[i], omega_[i], acc_[i], dec_[i]);
                }
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
        
    else if(cmStatus_.engineState == wmx3Api::EngineState::T::Communicating && isCommPrev_ == false){
        for(int i=0; i<axisNumber_;i++){
            clearAlarm(i);
            setEncoderMode(i);
            setPolarity(i, AxisPolarity_[i]);
            setGearRatio(i, gearNumerator_[i], gearDenumerator_[i]);
            setAxisMode(i);
        }

        isCommPrev_ = true;
        RCLCPP_INFO(this->get_logger(), "Axis is configured");
    }
    
    else{
        isCommPrev_ = false;
        RCLCPP_WARN(this->get_logger(), "Communication or engine off. Please start the engine or communication");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void Cr3aRobot::setPosition(int axis, double position, double omega, double acc, double dec){
    m_position.axis = axis;
    m_position.target = position;
    m_position.profile.velocity = omega;
    
    m_position.profile.type = ProfileType::T::Trapezoidal;
    m_position.profile.acc = acc;
    m_position.profile.dec = dec;

    err_ = wmx3LibCm_.motion->StartPos(&m_position);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to move motor %d. Error=%d (%s)", axis, err_, errString_);
    }
}

void Cr3aRobot::setGearRatio(int axis, double numerator, double denumerator){
    err_ = wmx3LibCm_.config->SetGearRatio(axis, numerator, denumerator);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set gear ratio axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set gear ratio axis %d", axis);
    }
}

void Cr3aRobot::setPolarity(int axis, int polarity){
    err_ = wmx3LibCm_.config->SetAxisPolarity(axis, polarity);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set polarity axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set polarity axis %d", axis);
    }
}

void Cr3aRobot::setAxisMode(int axis){
    err_ = wmx3LibCm_.axisControl->SetAxisCommandMode(axis, AxisCommandMode::Position);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set axis %d command mode to Position. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set Position mode axis %d", axis);
    }
}

void Cr3aRobot::setEncoderMode(int axis){
    err_ = wmx3LibCm_.config->SetAbsoluteEncoderMode(axis, false);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set encoder mode axis %d. Error=%d (%s)", axis, err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Set encoder mode axis %d", axis);
    }
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
