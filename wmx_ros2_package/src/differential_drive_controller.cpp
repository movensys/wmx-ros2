// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::CoreMotionAxisStatus;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;
using wmx3Api::WMX3Api;

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController();
  ~DiffDriveController();

  int leftAxis_;
  int rightAxis_;

  int rate_;
  double accTime_;
  double decTime_;
  double wheelRadius_;
  double wheelToWheel_;

  std::string cmdVelTopic_;
  std::string encoderOmegaTopic_;
  std::string encoderOdometeryTopic_;

  int err_;
  char errString_[256];

private:
  bool initialized_ = false;
  std::atomic<bool> initializing_{false};

  WMX3Api wmx3Lib_;
  CoreMotionStatus cmStatus_;
  std::unique_ptr<CoreMotion> wmx3LibCm_;
  Velocity::VelCommand velCommand_;

  std::vector<double> cmdOmega_;
  std::vector<double> encoderOmega_{0.0, 0.0};
  std::vector<double> encoderOdometry_;

  geometry_msgs::msg::Twist cmdVelMsg_;
  std_msgs::msg::Float64MultiArray encoderOmegaMsg_;
  nav_msgs::msg::Odometry encoderOdometryMsg_;

  rclcpp::TimerBase::SharedPtr cmdVelTimer_;
  rclcpp::TimerBase::SharedPtr encoderOmegaTimer_;
  rclcpp::TimerBase::SharedPtr encoderOdometryTimer_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr encoderOmegaPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoderOdometeryPub_;

  std::thread init_thread_;

  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void runInitSequence();

  void cmdVelStep();
  void encoderOmegaStep();
  void encoderOdometryStep();

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  std::vector<double> cmdCalculateOmega(double cmdLinearX, double cmdOmegaZ);
  std::vector<double> encoderCalculateOdometry(double omegaLeft, double omegaRight);

  void setRosParameter();
  void clearAlarm(int axis);
  void setServoOn(int axis);
  void setServoOff(int axis);
  void setVelocity(int axis, double omega);
};

DiffDriveController::DiffDriveController() : Node("differential_drive_controller")
{
  RCLCPP_INFO(this->get_logger(), "start differential_drive_controller");

  setRosParameter();

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&DiffDriveController::onEngineReady, this, _1));

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller waiting for engine...");
}

DiffDriveController::~DiffDriveController()
{
  RCLCPP_INFO(this->get_logger(), "Stop differential_drive_controller");

  if (init_thread_.joinable()) {
    init_thread_.join();
  }

  if (initialized_) {
    if (cmdVelTimer_) {cmdVelTimer_->cancel();}
    if (encoderOmegaTimer_) {encoderOmegaTimer_->cancel();}
    if (encoderOdometryTimer_) {encoderOdometryTimer_->cancel();}

    setVelocity(leftAxis_, 0.0);
    setVelocity(rightAxis_, 0.0);

    setServoOff(leftAxis_);
    setServoOff(rightAxis_);

    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device. Error=%d (%s)", err_, errString_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is stopped");
}

void DiffDriveController::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || initialized_ || initializing_.exchange(true)) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — starting init on dedicated thread...");

  // Join any previous thread (e.g. from a failed retry)
  if (init_thread_.joinable()) {
    init_thread_.join();
  }

  // Spawn dedicated thread so blocking device-attach retries don't block the executor
  init_thread_ = std::thread(&DiffDriveController::runInitSequence, this);
}

void DiffDriveController::runInitSequence()
{
  unsigned int timeout = 10000;
  static constexpr int kMaxDeviceRetries = 30;

  for (int attempt = 1; attempt <= kMaxDeviceRetries; ++attempt) {
    err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
    if (err_ == ErrorCode::None) {
      break;
    }
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Device lock busy, retrying in 1s... (%d/%d)",
        attempt, kMaxDeviceRetries);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach to device. Error=%d (%s)", err_, errString_);
      initializing_ = false;
      return;
    }
  }

  if (err_ != ErrorCode::None) {
    RCLCPP_FATAL(
      this->get_logger(), "Device lock busy after %d retries, giving up", kMaxDeviceRetries);
    initializing_ = false;
    return;
  }

  wmx3Lib_.SetDeviceName("differential_drive_controller");
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  wmx3LibCm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  clearAlarm(leftAxis_);
  clearAlarm(rightAxis_);

  setServoOn(leftAxis_);
  setServoOn(rightAxis_);

  // Create publishers / subscription on the node (thread-safe in rclcpp)
  encoderOmegaPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    encoderOmegaTopic_, 1);
  encoderOdometeryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    encoderOdometeryTopic_, 1);

  cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmdVelTopic_, 1, std::bind(&DiffDriveController::cmdCallback, this, _1));

  auto period = std::chrono::milliseconds(1000 / rate_);
  cmdVelTimer_ = this->create_wall_timer(
    period, std::bind(&DiffDriveController::cmdVelStep, this));
  encoderOmegaTimer_ = this->create_wall_timer(
    period, std::bind(&DiffDriveController::encoderOmegaStep, this));
  encoderOdometryTimer_ = this->create_wall_timer(
    period, std::bind(&DiffDriveController::encoderOdometryStep, this));

  initialized_ = true;
  engineReadySub_.reset();
  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is ready");
}

void DiffDriveController::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmdVelMsg_ = *msg;
}

void DiffDriveController::cmdVelStep()
{
  wmx3LibCm_->GetStatus(&cmStatus_);

  CoreMotionAxisStatus * cmAxisLeftStatus = &cmStatus_.axesStatus[leftAxis_];
  CoreMotionAxisStatus * cmAxisRightStatus = &cmStatus_.axesStatus[rightAxis_];

  if (cmStatus_.engineState != EngineState::T::Communicating) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Communication or engine off. Please start the engine or communication");
    return;
  }

  if (cmAxisLeftStatus->ampAlarm || cmAxisRightStatus->ampAlarm) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo alarm on. Please clear servo alarm");
    return;
  }

  if (!cmAxisLeftStatus->servoOn || !cmAxisRightStatus->servoOn) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo off. Please set servo on");
    return;
  }

  cmdOmega_ = cmdCalculateOmega(cmdVelMsg_.linear.x, cmdVelMsg_.angular.z);
  setVelocity(leftAxis_, cmdOmega_[0]);
  setVelocity(rightAxis_, cmdOmega_[1]);
}

void DiffDriveController::encoderOmegaStep()
{
  wmx3LibCm_->GetStatus(&cmStatus_);

  encoderOmega_ = {
    cmStatus_.axesStatus[leftAxis_].actualVelocity,
    cmStatus_.axesStatus[rightAxis_].actualVelocity};

  encoderOmegaMsg_.data.clear();
  encoderOmegaMsg_.data.push_back(encoderOmega_[0]);
  encoderOmegaMsg_.data.push_back(encoderOmega_[1]);
  encoderOmegaPub_->publish(encoderOmegaMsg_);
}

void DiffDriveController::encoderOdometryStep()
{
  encoderOdometry_ = encoderCalculateOdometry(encoderOmega_[0], encoderOmega_[1]);

  encoderOdometryMsg_.header.stamp = this->get_clock()->now();
  encoderOdometryMsg_.twist.twist.linear.x = encoderOdometry_[0];
  encoderOdometryMsg_.twist.twist.linear.y = 0.0;
  encoderOdometryMsg_.twist.twist.angular.z = encoderOdometry_[1];
  encoderOdometeryPub_->publish(encoderOdometryMsg_);
}

std::vector<double> DiffDriveController::cmdCalculateOmega(double cmdLinearX, double cmdOmegaZ)
{
  return {
    (2 * cmdLinearX - cmdOmegaZ * wheelToWheel_) / (2 * wheelRadius_),
    (2 * cmdLinearX + cmdOmegaZ * wheelToWheel_) / (2 * wheelRadius_)};
}

std::vector<double> DiffDriveController::encoderCalculateOdometry(
  double omegaLeft, double omegaRight)
{
  return {
    ((omegaRight * wheelRadius_) + (omegaLeft * wheelRadius_)) / 2.0,
    ((omegaRight * wheelRadius_) - (omegaLeft * wheelRadius_)) / wheelToWheel_};
}

void DiffDriveController::setVelocity(int axis, double omega)
{
  velCommand_.axis = axis;
  velCommand_.profile.velocity = omega;
  velCommand_.profile.type = ProfileType::T::TimeAccTrapezoidal;
  velCommand_.profile.accTimeMilliseconds = accTime_;
  velCommand_.profile.decTimeMilliseconds = decTime_;

  err_ = wmx3LibCm_->velocity->StartVel(&velCommand_);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(), "Failed to move motor %d. Error=%d (%s)", axis, err_, errString_);
  }
}

void DiffDriveController::clearAlarm(int axis)
{
  err_ = wmx3LibCm_->axisControl->ClearAmpAlarm(axis);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(), "Failed to clear alarm axis %d. Error=%d (%s)", axis, err_, errString_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Clear alarm axis %d", axis);
  }
}

void DiffDriveController::setServoOn(int axis)
{
  err_ = wmx3LibCm_->axisControl->SetServoOn(axis, 1);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Servo %d error to on. Error=%d (%s)", axis, err_, errString_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Servo %d on", axis);
  }
}

void DiffDriveController::setServoOff(int axis)
{
  err_ = wmx3LibCm_->axisControl->SetServoOn(axis, 0);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Servo %d error to off. Error=%d (%s)", axis, err_, errString_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Servo %d off", axis);
  }
}

void DiffDriveController::setRosParameter()
{
  this->declare_parameter<int>("left_axis", 0);
  this->declare_parameter<int>("right_axis", 1);

  this->declare_parameter<int>("rate", 10);
  this->declare_parameter<double>("acc_time", 1.0);
  this->declare_parameter<double>("dec_time", 1.0);
  this->declare_parameter<double>("wheel_radius", 0.09);
  this->declare_parameter<double>("wheel_to_wheel", 0.55);

  this->declare_parameter<std::string>("cmd_vel_topic", "/diff_drive/no_param");
  this->declare_parameter<std::string>("encoder_omega_topic", "/diff_drive/no_param");
  this->declare_parameter<std::string>("encoder_odometry_topic", "/diff_drive/no_param");

  this->get_parameter("left_axis", leftAxis_);
  this->get_parameter("right_axis", rightAxis_);

  this->get_parameter("rate", rate_);
  this->get_parameter("acc_time", accTime_);
  this->get_parameter("dec_time", decTime_);
  this->get_parameter("wheel_radius", wheelRadius_);
  this->get_parameter("wheel_to_wheel", wheelToWheel_);

  this->get_parameter("cmd_vel_topic", cmdVelTopic_);
  this->get_parameter("encoder_omega_topic", encoderOmegaTopic_);
  this->get_parameter("encoder_odometry_topic", encoderOdometeryTopic_);

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "left_axis: %d, right_axis: %d", leftAxis_, rightAxis_);
  RCLCPP_INFO(this->get_logger(), "rate: %d", rate_);
  RCLCPP_INFO(this->get_logger(), "acc_time: %f, dec_time: %f", accTime_, decTime_);
  RCLCPP_INFO(this->get_logger(), "wheel_radius: %f", wheelRadius_);
  RCLCPP_INFO(this->get_logger(), "wheel_to_wheel: %f", wheelToWheel_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmdVelTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_omega_topic: %s", encoderOmegaTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_odometry_topic: %s", encoderOdometeryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
