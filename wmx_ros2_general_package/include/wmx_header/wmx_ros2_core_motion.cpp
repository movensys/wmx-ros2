#include "wmx_ros2_general.hpp"

#include <thread>
#include <chrono>

void WmxRos2General::axisStateStep(){
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

    wmx3LibCm_.GetStatus(&cmStatus_);

    for (int i = 0; i < axisCount_; ++i) {
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

void WmxRos2General::axisPoseCallback(const wmx_ros2_message::msg::AxisPose::SharedPtr msg) {
    
    size_t axis_count = msg->index.size();
    for(size_t i=0; i<axis_count; i++){
        position_.axis = msg->index[i];
        position_.target = msg->target[i];
        position_.profile.velocity = msg->velocity[i];
        position_.profile.type = ProfileType::T::Trapezoidal; //msg->profile[i]
        position_.profile.acc = msg->acc[i];
        position_.profile.dec = msg->dec[i];

        err_ = wmx3LibCm_.motion->StartMov(&position_);

        cout<<position_.target<<endl;
    
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            RCLCPP_ERROR(this->get_logger(), "Failed to move position motor %d. Error=%d (%s)", msg->index[i], err_, errString_);
        }
    }
}

void WmxRos2General::axisVelCallback(const wmx_ros2_message::msg::AxisVelocity::SharedPtr msg) {
    
    size_t axis_count = msg->index.size();
    for(size_t i=0; i<axis_count; i++){
        velocity_.axis = msg->index[i];
        velocity_.profile.velocity = msg->velocity[i];
        velocity_.profile.type = ProfileType::T::Trapezoidal; //msg->profile[i]
        velocity_.profile.acc = msg->acc[i];
        velocity_.profile.dec = msg->dec[i];

        err_ = wmx3LibCm_.velocity->StartVel(&velocity_);
    
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            RCLCPP_ERROR(this->get_logger(), "Failed to move velocity motor %d. Error=%d (%s)", msg->index[i], err_, errString_);
        }
    }
}

void WmxRos2General::setAxisMode(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                                std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    bool all_success = true;
    std::stringstream msg_stream;

    size_t axis_count = request->index.size();
    for (size_t i = 0; i < axis_count; ++i)
    {
        int axis_index = request->index[i];
        int mode_value = request->data[i];

        if (mode_value == 0) {
            err_ = wmx3LibCm_.axisControl->SetAxisCommandMode(axis_index, AxisCommandMode::Position);
        }
        else if (mode_value == 1) {
            err_ = wmx3LibCm_.axisControl->SetAxisCommandMode(axis_index, AxisCommandMode::Velocity);
        }
        else {
            snprintf(buffer_, sizeof(buffer_), "Invalid mode %d for axis %d", mode_value, axis_index);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
            msg_stream << buffer_ << "; ";
            all_success = false;
            continue;
        }

        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis %d mode %d. Error=%d (%s)", axis_index, mode_value, err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            all_success = false;
        }
        else {
            const char* mode_str = (mode_value == 0) ? "Position" : "Velocity";
            snprintf(buffer_, sizeof(buffer_), "Set axis %d in %s mode", axis_index, mode_str);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        }
        msg_stream << buffer_ << "; ";
    }

    response->success = all_success;
    response->message = msg_stream.str();
}


void WmxRos2General::setAxisOn(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    bool all_success = true;
    std::stringstream msg_stream;

    size_t axis_count = request->index.size();
    for (size_t i = 0; i < axis_count; ++i)
    {
        int axis_index = request->index[i];
        int on_off = request->data[i];

        err_ = wmx3LibCm_.axisControl->SetServoOn(axis_index, on_off);

        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to set axis %d %s. Error=%d (%s)",
                     axis_index,
                     on_off ? "on" : "off",
                     err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            all_success = false;
        } else {
            snprintf(buffer_, sizeof(buffer_), "Set axis %d %s", axis_index, on_off ? "on" : "off");
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        }

        msg_stream << buffer_ << "; ";
    }

    response->success = all_success;
    response->message = msg_stream.str();
}


void WmxRos2General::setHoming(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    bool all_success = true;
    std::stringstream msg_stream;

    size_t axis_count = request->index.size();
    for (size_t i = 0; i < axis_count; ++i)
    {
        wmx3LibCm_.config->GetHomeParam(request->index[i], &homeParam_);
        homeParam_.homeType = Config::HomeType::CurrentPos;
        wmx3LibCm_.config->SetHomeParam(request->index[i], &homeParam_);
        wmx3LibCm_.home->StartHome(request->index[i]);
        wmx3LibCm_.motion->Wait(request->index[i]);

        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to homing axis %d. Error=%d (%s)", request->index[i], err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            all_success = false;
        }
        else {
            snprintf(buffer_, sizeof(buffer_), "Homing axis %d", request->index[i]);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        }
        msg_stream << buffer_ << "; ";
    }

    response->success = all_success;
    response->message = msg_stream.str();
}

void WmxRos2General::clearAlarm(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    bool all_success = true;
    std::stringstream msg_stream;

    size_t axis_count = request->index.size();
    for (size_t i = 0; i < axis_count; ++i)
    {
        err_ = wmx3LibCm_.axisControl->ClearAmpAlarm(request->index[i]);
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            snprintf(buffer_, sizeof(buffer_), "Failed to clear alarm axis %d. Error=%d (%s)", request->index[i], err_, errString_);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            all_success = false;
        }
        else {
            snprintf(buffer_, sizeof(buffer_), "Cleared alarm axis %d", request->index[i]);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        }
        msg_stream << buffer_ << "; ";
    }

    response->success = all_success;
    response->message = msg_stream.str();
}

void WmxRos2General::setAxisPolarity(const std::shared_ptr<wmx_ros2_message::srv::SetAxis::Request> request,
                                    std::shared_ptr<wmx_ros2_message::srv::SetAxis::Response> response){
    bool all_success = true;
    std::stringstream msg_stream;

    size_t axis_count = request->index.size();
    for (size_t i = 0; i < axis_count; ++i)
    {
        if (request->data[i] != 1 && request->data[i] != -1) {
            snprintf(buffer_, sizeof(buffer_), "Invalid polarity value for axis %d: %d", request->index[i], request->data[i]);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            msg_stream << buffer_ << "; ";
            all_success = false;
            continue;
        }

        err_ = wmx3LibCm_.config->SetAxisPolarity(request->index[i], request->data[i]);

        if (err_ != ErrorCode::None) {
            snprintf(buffer_, sizeof(buffer_), "Failed to set polarity on axis %d", request->index[i]);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
            all_success = false;
        } else {
            snprintf(buffer_, sizeof(buffer_), "Set polarity on axis %d", request->index[i]);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        }

        msg_stream << buffer_ << "; ";
    }

    response->success = all_success;
    response->message = msg_stream.str();
}

void WmxRos2General::setAxisGearRatio(const std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Request> request,
                                    std::shared_ptr<wmx_ros2_message::srv::SetAxisGearRatio::Response> response){
    bool all_success = true;
    std::stringstream msg_stream;

    size_t axis_count = request->index.size();
    for (size_t i = 0; i < axis_count; ++i)
    {
        err_ = wmx3LibCm_.config->SetGearRatio(
            request->index[i],
            request->numerator[i],
            request->denumerator[i]
        );

        if (err_ != ErrorCode::None) {
            all_success = false;
            snprintf(buffer_, sizeof(buffer_), "Failed to set gear ratio on axis %d", request->index[i]);
            RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
        } else {
            snprintf(buffer_, sizeof(buffer_), "Set gear ratio on axis %d", request->index[i]);
            RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        }

        msg_stream << buffer_ << "; ";
    }

    response->success = all_success;
    response->message = msg_stream.str();
}
