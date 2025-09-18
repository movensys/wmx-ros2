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
#include "IOApi.h"

#define WMX_PARAM_FILE_PATH "/home/jetstream/wmx_ros2_ws/src/wmx_ros2_application/wmx_ros2_package/config/cr3a_wmx_parameters.xml"

using std::placeholders::_1;
using namespace wmx3Api;
using namespace std;

class Cr3aRobot : public rclcpp::Node {
public:
    Cr3aRobot(); 
    ~Cr3aRobot(); 

    int axisNumber_;
    int rate_;

    std::vector<std::string> jointNames_;
    std::string gripperName_;

    long double jointMsg_[6] = {0.0L, 0.0L, 0.0L, 0.0L, 0.0L, 0.0L};
    float gripperMsg_ = 0.0;

    double omega_, acc_, dec_;

    std::string cmdJointTopic_;
    std::string encoderJointTopic_;

    std::chrono::milliseconds cmdJointPeriod_;
    std::chrono::milliseconds encoderJointPeriod_;

    int err_;
    char errString_[256];

private:
    WMX3Api wmx3Lib_;            
    CoreMotionStatus cmStatus_;  
    CoreMotion wmx3LibCm_;
    Io Wmx3Lib_Io_;
    
    wmx3Api::Motion::LinearIntplCommand lin_ = wmx3Api::Motion::LinearIntplCommand();

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
};

Cr3aRobot::Cr3aRobot() : Node("cr3a_robot_node"), wmx3LibCm_(&wmx3Lib_), Wmx3Lib_Io_(&wmx3Lib_) {  
    RCLCPP_INFO(this->get_logger(), "start cr3a_robot_node");

    setRosParameter();
    startEngine();  

    startCommunication();

    wmx3LibCm_.config->ImportAndSetAll((char*)WMX_PARAM_FILE_PATH);

    for(int i=0; i<axisNumber_;i++){
        clearAlarm(i);
        setServoOn(i);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
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
    this->declare_parameter<std::string>("gripper_name", "gripper");

    this->declare_parameter<std::vector<std::string>>("joint_name", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    this->declare_parameter<std::string>("cmd_joint_topic", "/joint_states");
    this->declare_parameter<std::string>("encoder_joint_topic", "/enc_joint");
    
    this->declare_parameter<double>("omega", 0.0);
    this->declare_parameter<double>("acc", 0.0);
    this->declare_parameter<double>("dec", 0.0);
    
    this->get_parameter("gripper_name", gripperName_);
    this->get_parameter("joint_name", jointNames_);
    this->get_parameter("axis_number", axisNumber_);
    this->get_parameter("rate", rate_);
    this->get_parameter("cmd_joint_topic", cmdJointTopic_);
    this->get_parameter("encoder_joint_topic", encoderJointTopic_);
    this->get_parameter("omega", omega_);
    this->get_parameter("acc", acc_);
    this->get_parameter("dec", dec_);
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
        cout<<"command: "<<jointMsg_[i]<<"\t state: "<<cmAxisStatus_[i]->actualPos<<"\t omega: "<<omega_<<"\t acc: "<<acc_<<"\t dec: "<<dec_<<endl;
    }
    cout<<"gripper: "<< gripperMsg_<<endl;
    cout<<endl;
}

void Cr3aRobot::cmdJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    cmdJointMsg_ = *msg;

    for (int i = 0; i < axisNumber_; ++i) {
        const std::string& expected_name = jointNames_[i];

        auto it = std::find(msg->name.begin(), msg->name.end(), expected_name);
        if (it != msg->name.end()) {
            size_t index = std::distance(msg->name.begin(), it);

            if (index < msg->position.size()) {
                jointMsg_[i] = msg->position[index];
            } else {
                RCLCPP_WARN(this->get_logger(), "Position index out of bounds for joint: %s", expected_name.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Joint name %s not found in incoming message", expected_name.c_str());
        }
    }

    const std::string& expected_name = gripperName_;
    auto it = std::find(msg->name.begin(), msg->name.end(), expected_name);
    if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it);

        if (index < msg->position.size()) {
            gripperMsg_ = msg->position[index];
        } else {
            RCLCPP_WARN(this->get_logger(), "Position index out of bounds for gripper: %s", expected_name.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Gripper name %s not found in incoming message", expected_name.c_str());
    }
}

void Cr3aRobot::cmdJointStep() {
    wmx3LibCm_.GetStatus(&cmStatus_);

    std::vector<CoreMotionAxisStatus*> cmAxisStatus_(axisNumber_);
    for (int i = 0; i < axisNumber_; ++i) {
        cmAxisStatus_[i] = &cmStatus_.axesStatus[i];
    }

    if(cmStatus_.engineState == wmx3Api::EngineState::T::Communicating){

        if (!cmAxisStatus_[0]->ampAlarm && !cmAxisStatus_[1]->ampAlarm && !cmAxisStatus_[2]->ampAlarm && 
            !cmAxisStatus_[3]->ampAlarm && !cmAxisStatus_[4]->ampAlarm && !cmAxisStatus_[5]->ampAlarm){
            
            if( cmAxisStatus_[0]->servoOn && cmAxisStatus_[1]->servoOn && cmAxisStatus_[2]->servoOn &&
                cmAxisStatus_[3]->servoOn && cmAxisStatus_[4]->servoOn && cmAxisStatus_[5]->servoOn){
                
                lin_.axisCount = 6;
                for (int i = 0; i < axisNumber_; ++i) {
                    lin_.axis[i] = i;  
                    lin_.target[i] = jointMsg_[i];      
                }

                lin_.profile.type = ProfileType::Trapezoidal;
                lin_.profile.velocity = omega_;
                lin_.profile.acc = acc_;
                lin_.profile.dec = dec_;

                wmx3LibCm_.motion->StartLinearIntplPos(&lin_);

                if(gripperMsg_ >= 0.005){
                    Wmx3Lib_Io_.SetOutBit(0x00, 0x00, 0xFF);
                }
                else{
                    Wmx3Lib_Io_.SetOutBit(0x00, 0x00, 0x00);
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
    
    else{
        RCLCPP_WARN(this->get_logger(), "Communication or engine off. Please start the engine or communication");
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
