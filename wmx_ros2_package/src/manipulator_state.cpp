#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wmx_ros2_message/srv/set_axis.hpp"

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
    float gripperCloseValue_;
    float gripperOpenValue_;
    std::vector<std::string> jointNames_;
    std::string encoderJointTopic_;
    std::string isaacsimJointTopic_;
    std::string gazeboJointTopic_;
    std::string wmxParamFilePath_;

    unsigned char gripperData_;
    int err_;
    char errString_[256];

private:
    bool initialized_ = false;
    std::atomic<bool> initializing_{false};

    WMX3Api wmx3Lib_;
    CoreMotionStatus cmStatus_;
    std::unique_ptr<CoreMotion> wmx3LibCm_;
    std::unique_ptr<Io> wmx3Lib_Io_;
    Config::AxisParam axisParam_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coreMotionReadySub_;
    rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr clearAlarmClient_;
    rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr setAxisOnClient_;

    rclcpp::TimerBase::SharedPtr encoderJointTimer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoderJointPub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr isaacsimJointPub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gazeboJointPub_;

    std::thread init_thread_;

    void onCoreMotionReady(const std_msgs::msg::Bool::SharedPtr msg);
    void runInitSequence();
    bool callSetAxisService(
        rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
        const std::string & service_name,
        const std::vector<int32_t> & index,
        const std::vector<int32_t> & data);
    void publishJointState();
    void setRosParameter();
    void setWmxParam(char* path);
    void getWmxParam();
};

ManipulatorState::ManipulatorState() : Node("manipulator_state") {
    RCLCPP_INFO(this->get_logger(), "start manipulator_state");

    setRosParameter();

    auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
    coreMotionReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
        "wmx/core_motion/ready", ready_qos,
        std::bind(&ManipulatorState::onCoreMotionReady, this, _1));

    clearAlarmClient_ = this->create_client<wmx_ros2_message::srv::SetAxis>(
        "wmx/axis/clear_alarm");

    setAxisOnClient_ = this->create_client<wmx_ros2_message::srv::SetAxis>(
        "wmx/axis/set_on");

    RCLCPP_INFO(this->get_logger(), "manipulator_state waiting for core_motion...");
}

ManipulatorState::~ManipulatorState() {
    RCLCPP_INFO(this->get_logger(), "Stop manipulator_state");

    if (init_thread_.joinable()) {
        init_thread_.join();
    }

    if (initialized_) {
        if (encoderJointTimer_) {
            encoderJointTimer_->cancel();
        }

        for (int i = 0; i < jointNumber_; i++) {
            err_ = wmx3LibCm_->axisControl->SetServoOn(i, 0);
            if (err_ != ErrorCode::None) {
                wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
                RCLCPP_ERROR(this->get_logger(), "Servo %d error to off. Error=%d (%s)", i, err_, errString_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Servo %d off", i);
            }
        }

        err_ = wmx3Lib_.CloseDevice();
        if (err_ != ErrorCode::None) {
            wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
            RCLCPP_ERROR(this->get_logger(), "Failed to close device. Error=%d (%s)", err_, errString_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Device closed");
        }
    }

    RCLCPP_INFO(this->get_logger(), "manipulator_state is stopped");
}

void ManipulatorState::onCoreMotionReady(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || initialized_ || initializing_.exchange(true)) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "CoreMotion ready — starting init on dedicated thread...");

    // Join any previous thread (e.g. from a failed retry)
    if (init_thread_.joinable()) {
        init_thread_.join();
    }

    // Spawn dedicated thread so blocking wait_for() doesn't block executor
    init_thread_ = std::thread(&ManipulatorState::runInitSequence, this);
}

void ManipulatorState::runInitSequence() {
    unsigned int timeout = 10000;
    static constexpr int kMaxDeviceRetries = 30;

    for (int attempt = 1; attempt <= kMaxDeviceRetries; ++attempt) {
        err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
        if (err_ == ErrorCode::None) {
            break;
        }
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        if (err_ == ErrorCode::StartProcessLockError) {
            RCLCPP_WARN(this->get_logger(), "Device lock busy, retrying in 1s... (%d/%d)",
                        attempt, kMaxDeviceRetries);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to attach to device. Error=%d (%s)", err_, errString_);
            initializing_ = false;
            return;
        }
    }

    if (err_ != ErrorCode::None) {
        RCLCPP_FATAL(this->get_logger(), "Device lock busy after %d retries, giving up", kMaxDeviceRetries);
        initializing_ = false;
        return;
    }

    wmx3Lib_.SetDeviceName("ManipulatorState");

    RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

    wmx3LibCm_  = std::make_unique<CoreMotion>(&wmx3Lib_);
    wmx3Lib_Io_ = std::make_unique<Io>(&wmx3Lib_);

    setWmxParam((char*)wmxParamFilePath_.c_str());
    getWmxParam();

    // Build axis index/data vectors
    std::vector<int32_t> allAxes;
    std::vector<int32_t> zeroData(jointNumber_, 0);
    std::vector<int32_t> onData(jointNumber_, 1);
    for (int i = 0; i < jointNumber_; i++) {
        allAxes.push_back(i);
    }

    // Clear alarms on all axes
    if (!callSetAxisService(clearAlarmClient_, "wmx/axis/clear_alarm", allAxes, zeroData)) {
        RCLCPP_ERROR(this->get_logger(), "Init failed at clear_alarm — node will not retry");
        initializing_ = false;
        return;
    }

    // Set servo on for all axes
    if (!callSetAxisService(setAxisOnClient_, "wmx/axis/set_on", allAxes, onData)) {
        RCLCPP_ERROR(this->get_logger(), "Init failed at set_on — node will not retry");
        initializing_ = false;
        return;
    }

    // Create publishers and timer on the node (thread-safe in rclcpp)
    encoderJointPub_   = this->create_publisher<sensor_msgs::msg::JointState>(encoderJointTopic_, 1);
    isaacsimJointPub_  = this->create_publisher<sensor_msgs::msg::JointState>(isaacsimJointTopic_, 1);
    gazeboJointPub_    = this->create_publisher<std_msgs::msg::Float64MultiArray>(gazeboJointTopic_, 1);

    encoderJointTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / jointFeedbackRate_),
        std::bind(&ManipulatorState::publishJointState, this));

    initialized_ = true;
    coreMotionReadySub_.reset();
    RCLCPP_INFO(this->get_logger(), "manipulator_state is ready");
}

bool ManipulatorState::callSetAxisService(
    rclcpp::Client<wmx_ros2_message::srv::SetAxis>::SharedPtr client,
    const std::string & service_name,
    const std::vector<int32_t> & index,
    const std::vector<int32_t> & data)
{
    const int max_retries = 5;
    const auto service_timeout = std::chrono::seconds(10);
    const auto call_timeout = std::chrono::seconds(15);

    if (!client->wait_for_service(service_timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
        return false;
    }

    for (int attempt = 1; attempt <= max_retries; attempt++) {
        auto request = std::make_shared<wmx_ros2_message::srv::SetAxis::Request>();
        request->index = index;
        request->data = data;

        RCLCPP_INFO(this->get_logger(), "Calling %s (attempt %d/%d)", service_name.c_str(), attempt, max_retries);

        auto future = client->async_send_request(request);
        if (future.wait_for(call_timeout) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "Service call %s timed out (attempt %d/%d)",
                service_name.c_str(), attempt, max_retries);
            continue;
        }

        auto response = future.get();
        if (!response->success) {
            // Server may not be initialized yet -- retry instead of aborting
            if (response->message.find("not initialized") != std::string::npos) {
                RCLCPP_WARN(this->get_logger(),
                    "%s: server not ready yet (attempt %d/%d), retrying...",
                    service_name.c_str(), attempt, max_retries);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            RCLCPP_ERROR(this->get_logger(), "%s failed: %s",
                service_name.c_str(), response->message.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "%s succeeded: %s", service_name.c_str(), response->message.c_str());
        return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Service call %s failed after %d attempts", service_name.c_str(), max_retries);
    return false;
}

void ManipulatorState::setRosParameter() {
    this->declare_parameter<int>("joint_number", 0);
    this->declare_parameter<int>("joint_feedback_rate", 0);
    this->declare_parameter<float>("gripper_open_value", 0);
    this->declare_parameter<float>("gripper_close_value", 0);
    this->declare_parameter<std::vector<std::string>>("joint_name", {"j1", "j2", "j3", "j4", "j5", "j6"});
    this->declare_parameter<std::string>("encoder_joint_topic", "/encoder_joint_topic/no_param");
    this->declare_parameter<std::string>("isaacsim_joint_topic", "/isaacsim_joint_topic/no_param");
    this->declare_parameter<std::string>("gazebo_joint_topic", "/gazebo_joint_topic/no_param");
    this->declare_parameter<std::string>("wmx_param_file_path", "/wmx_param_file_path/no_param");

    this->get_parameter("joint_number", jointNumber_);
    this->get_parameter("joint_feedback_rate", jointFeedbackRate_);
    this->get_parameter("gripper_open_value", gripperOpenValue_);
    this->get_parameter("gripper_close_value", gripperCloseValue_);
    this->get_parameter("joint_name", jointNames_);
    this->get_parameter("encoder_joint_topic", encoderJointTopic_);
    this->get_parameter("isaacsim_joint_topic", isaacsimJointTopic_);
    this->get_parameter("gazebo_joint_topic", gazeboJointTopic_);
    this->get_parameter("wmx_param_file_path", wmxParamFilePath_);

    RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
    RCLCPP_INFO(this->get_logger(), "joint_number: %d", jointNumber_);
    RCLCPP_INFO(this->get_logger(), "joint_feedback_rate: %d", jointFeedbackRate_);
    RCLCPP_INFO(this->get_logger(), "gripper_open_value: %f", gripperOpenValue_);
    RCLCPP_INFO(this->get_logger(), "gripper_close_value: %f", gripperCloseValue_);

    std::string joint_names_str;
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        if (i > 0) joint_names_str += ", ";
        joint_names_str += jointNames_[i];
    }
    RCLCPP_INFO(this->get_logger(), "joint_name: [%s]", joint_names_str.c_str());

    RCLCPP_INFO(this->get_logger(), "encoder_joint_topic: %s", encoderJointTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "isaacsim_joint_topic: %s", isaacsimJointTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "gazebo_joint_topic: %s", gazeboJointTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "wmx_param_file_path: %s", wmxParamFilePath_.c_str());
    RCLCPP_INFO(this->get_logger(), "===========================");
}

void ManipulatorState::publishJointState() {
    wmx3LibCm_->GetStatus(&cmStatus_);

    sensor_msgs::msg::JointState encoderJointMsg_;
    std_msgs::msg::Float64MultiArray gazeboJointMsg_;
    gazeboJointMsg_.data.resize(8);

    for (int i = 0; i < jointNumber_; ++i) {
        encoderJointMsg_.name.push_back(jointNames_[i]);
        encoderJointMsg_.position.push_back(cmStatus_.axesStatus[i].actualPos);
        encoderJointMsg_.velocity.push_back(cmStatus_.axesStatus[i].actualVelocity);
        gazeboJointMsg_.data[i] = cmStatus_.axesStatus[i].actualPos;
    }

    for (int i = 0; i < 2; ++i) {
        encoderJointMsg_.name.push_back(jointNames_[jointNumber_+i]);
        wmx3Lib_Io_->GetOutBit(0, 0, &gripperData_);
        if (gripperData_) {
            encoderJointMsg_.position.push_back(gripperCloseValue_);
            encoderJointMsg_.velocity.push_back(0.000);
            gazeboJointMsg_.data[jointNumber_+i] = gripperCloseValue_;
        } else {
            encoderJointMsg_.position.push_back(gripperOpenValue_);
            encoderJointMsg_.velocity.push_back(0.000);
            gazeboJointMsg_.data[jointNumber_+i] = gripperOpenValue_;
        }
    }

    isaacsimJointPub_->publish(encoderJointMsg_);
    encoderJointMsg_.header.stamp = this->get_clock()->now();
    encoderJointPub_->publish(encoderJointMsg_);

    gazeboJointPub_->publish(gazeboJointMsg_);
}

void ManipulatorState::setWmxParam(char* path) {
    err_ = wmx3LibCm_->config->ImportAndSetAll(path);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to set WMX params. Error=%d (%s)", err_, errString_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Success to set WMX params");
    }
}

void ManipulatorState::getWmxParam() {
    err_ = wmx3LibCm_->config->GetAxisParam(&axisParam_);
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to get axis params. Error=%d (%s)", err_, errString_);
    } else {
        for (int axis = 0; axis < jointNumber_; axis++) {
            RCLCPP_INFO(this->get_logger(), "axis: %d, numerator: %f", axis, axisParam_.gearRatioNumerator[axis]);
            RCLCPP_INFO(this->get_logger(), "axis: %d, denominator: %f", axis, axisParam_.gearRatioDenominator[axis]);
            RCLCPP_INFO(this->get_logger(), "axis: %d, polarity: %d", axis, (int)axisParam_.axisPolarity[axis]);
            RCLCPP_INFO(this->get_logger(), "axis: %d, abs encoder: %d", axis, axisParam_.absoluteEncoderMode[axis]);
            RCLCPP_INFO(this->get_logger(), "axis: %d, mode: %d", axis, axisParam_.axisCommandMode[axis]);
        }
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
