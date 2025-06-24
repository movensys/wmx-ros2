#include "wmx_ros2_core_motion.hpp"

#include <thread>
#include <chrono>

WmxRos2CoreMotion::WmxRos2CoreMotion() : Node("wmx_ros2_core_motion_node"), wmx3LibCm_(&wmx3Lib_) {  
    axisStatePeriod_ = std::chrono::milliseconds(1000 / rate_);
    axisStateTimer_ = this->create_wall_timer(axisStatePeriod_, std::bind(&WmxRos2CoreMotion::axisStateStep, this));

    RCLCPP_INFO(this->get_logger(), "wmx_ros2_core_motion_node is ready");
}

WmxRos2CoreMotion::~WmxRos2CoreMotion(){
    RCLCPP_INFO(this->get_logger(), "wmx_ros2_core_motion_node is stopped");
}

void WmxRos2CoreMotion::axisStateStep(){
    wmx3LibCm_.GetStatus(&cmStatus_);

    axisStateMsg_.amp_alarm.clear();
    axisStateMsg_.servo_on.clear();
    axisStateMsg_.home_done.clear();
    axisStateMsg_.in_pos.clear();
    axisStateMsg_.negative_ls.clear();
    axisStateMsg_.positive_ls.clear();
    axisStateMsg_.home_switch.clear();
    axisStateMsg_.pos_cmd.clear();
    axisStateMsg_.velocity_cmd.clear();
    axisStateMsg_.actual_pos.clear();
    axisStateMsg_.actual_velocity.clear();
    axisStateMsg_.actual_torque.clear();

    for (int i = 0; i < 10; ++i) {
        axisStateMsg_.amp_alarm.push_back(cmStatus_.axesStatus[i].ampAlarm);
        axisStateMsg_.servo_on.push_back(cmStatus_.axesStatus[i].servoOn);
        axisStateMsg_.home_done.push_back(cmStatus_.axesStatus[i].homeDone);
        axisStateMsg_.in_pos.push_back(cmStatus_.axesStatus[i].inPos);
        axisStateMsg_.negative_ls.push_back(cmStatus_.axesStatus[i].negativeLS);
        axisStateMsg_.positive_ls.push_back(cmStatus_.axesStatus[i].positiveLS);
        axisStateMsg_.home_switch.push_back(cmStatus_.axesStatus[i].homeSwitch);

        axisStateMsg_.pos_cmd.push_back(cmStatus_.axesStatus[i].posCmd);
        axisStateMsg_.velocity_cmd.push_back(cmStatus_.axesStatus[i].velocityCmd);
        axisStateMsg_.actual_pos.push_back(cmStatus_.axesStatus[i].actualPos);
        axisStateMsg_.actual_velocity.push_back(cmStatus_.axesStatus[i].actualVelocity);
        axisStateMsg_.actual_torque.push_back(cmStatus_.axesStatus[i].actualTorque);
    }
    axisStatePub_->publish(axisStateMsg_);
}

void WmxRos2CoreMotion::axisVelCallback(const wmx_ros2_message::msg::AxisVelocity::SharedPtr msg) {
    velocity_.axis = msg->index;
    velocity_.profile.velocity = msg->velocity;
    velocity_.profile.type = ProfileType::T::Trapezoidal; //msg->profile
    velocity_.profile.acc = msg->acc;
    velocity_.profile.dec = msg->dec;

    err_ = wmx3LibCm_.velocity->StartVel(&velocity_);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to move velocity motor %d. Error=%d (%s)", msg->index, err_, errString_);
    }
}

void WmxRos2CoreMotion::setAxisMode(const std::shared_ptr<wmx_ros2_message::srv::SetAxisMode::Request> request,
                std::shared_ptr<wmx_ros2_message::srv::SetAxisMode::Response> response){
    
    if(request->mode == 0){
        err_ = wmx3LibCm_.axisControl->SetAxisCommandMode(request->index, AxisCommandMode::Position);
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis %d to Position mode. Error=%d (%s)", request->index, err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Set axis %d in Position mode", request->index);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
    else if(request->mode == 1){
        err_ = wmx3LibCm_.axisControl->SetAxisCommandMode(request->index, AxisCommandMode::Velocity);
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis %d to Velocity mode. Error=%d (%s)", request->index, err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Set axis %d in Velocity mode", request->index);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
    else{
        snprintf(buffer_, sizeof(buffer_), "Wrong axis %d mode", request->index);
        RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
    }
}

void WmxRos2CoreMotion::setAxisOn(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    err_ = wmx3LibCm_.axisControl->SetServoOn(request->index, request->data);
    if (request->data) {
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis %d on. Error=%d (%s)", request->index, err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Set axis %d on", request->index);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
    else{
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis %d off. Error=%d (%s)", request->index, err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Set axis %d off", request->index);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
}

void WmxRos2CoreMotion::clearAlarm(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    err_ = wmx3LibCm_.axisControl->ClearAmpAlarm(request->index);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        snprintf(buffer_, sizeof(buffer_), "Failed to clear alarm axis %d. Error=%d (%s)", request->index, err_, errString_);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_); 
    } 
    else {
        snprintf(buffer_, sizeof(buffer_), "Clear alarm axis %d", request->index);
        RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        response->success = true;
        response->message = std::string(buffer_);
    }
}

void WmxRos2CoreMotion::setAxisPolarity(const std::shared_ptr<wmx_ros2_message::srv::SetAxisMode::Request> request,
                std::shared_ptr<wmx_ros2_message::srv::SetAxisMode::Response> response){
    
    if (request->mode == 1 || request->mode==-1) {
        err_ = wmx3LibCm_.config->SetAxisPolarity(request->index, request->mode);
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis polarity %d: %d. Error=%d (%s)", request->index, request->mode, err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            response->success = false;
            response->message = std::string(buffer_); 
        } 
        else {
            snprintf(buffer_, sizeof(buffer_), "Set axis polrity %d: %d", request->index, request->mode);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            response->success = true;
            response->message = std::string(buffer_);
        }
    }
    else{
        snprintf(buffer_, sizeof(buffer_), "Wrong polarity value");
        RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_);
    }
}

void WmxRos2CoreMotion::setAxisGearRatio(const std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Request> request,
                std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Response> response){
    err_ = wmx3LibCm_.config->SetGearRatio(request->index, request->numerator, request->denumerator);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        snprintf(buffer_, sizeof(buffer_), "Failed to set gear ratio axis %d. Error=%d (%s)", request->index, err_, errString_);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        response->success = false;
        response->message = std::string(buffer_); 
    } 
    else {
        snprintf(buffer_, sizeof(buffer_), "Set gear ratio axis %d", request->index);
        RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        response->success = true;
        response->message = std::string(buffer_);
    }  
}