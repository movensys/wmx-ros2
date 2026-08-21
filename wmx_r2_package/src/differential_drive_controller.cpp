// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "differential_drive_controller.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <vector>

using std::placeholders::_1;

using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace ddl = diff_drive;

DifferentialDriveControllerApi::DifferentialDriveControllerApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config)
{
}

DifferentialDriveControllerApi::~DifferentialDriveControllerApi()
{
  if (cm_) {
    releaseDevice();
  }
}

std::string DifferentialDriveControllerApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int DifferentialDriveControllerApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (cm_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  const int err = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout_);
  if (err != ErrorCode::None) {
    message = "Failed to attach to device. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  wmx3Lib_.SetDeviceName(deviceName_);
  cm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  message = "Attached to WMX3 device";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void DifferentialDriveControllerApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  cm_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(logger_, "Failed to close device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "Device closed");
  }
}

int DifferentialDriveControllerApi::importAndSetAll(
  const std::string & path, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot set WMX params. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  std::vector<char> pathBuffer(path.begin(), path.end());
  pathBuffer.push_back('\0');

  const int err = cm_->config->ImportAndSetAll(pathBuffer.data());
  if (err != ErrorCode::None) {
    message = "Failed to set WMX params. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    return err;
  }

  message = "Success to set WMX params";
  return ErrorCode::None;
}

int DifferentialDriveControllerApi::getStatus(
  int leftAxis, int rightAxis,
  AxisFeedback & left, AxisFeedback & right, bool & communicating,
  std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot read status. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);

  communicating = (status.engineState == EngineState::T::Communicating);

  const auto & rawLeft = status.axesStatus[leftAxis];
  left.actualPos = rawLeft.actualPos;
  left.actualVelocity = rawLeft.actualVelocity;
  left.servoOn = rawLeft.servoOn;
  left.ampAlarm = rawLeft.ampAlarm;

  const auto & rawRight = status.axesStatus[rightAxis];
  right.actualPos = rawRight.actualPos;
  right.actualVelocity = rawRight.actualVelocity;
  right.servoOn = rawRight.servoOn;
  right.ampAlarm = rawRight.ampAlarm;

  return err;
}

int DifferentialDriveControllerApi::startVel(int axis, double omega, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot move axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  Velocity::VelCommand velCommand;
  velCommand.axis = axis;
  velCommand.profile.velocity = omega;
  velCommand.profile.type = ProfileType::T::TimeAccTrapezoidal;
  velCommand.profile.accTimeMilliseconds = config_.accTimeMilliseconds;
  velCommand.profile.decTimeMilliseconds = config_.decTimeMilliseconds;

  const int err = cm_->velocity->StartVel(&velCommand);
  if (err != ErrorCode::None) {
    message = "Failed to move motor " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(omega);
  return ErrorCode::None;
}

DifferentialDriveController::DifferentialDriveController()
: LifecycleNode("differential_drive_controller"),
  prevLoopTime_(0, 0, RCL_ROS_TIME),
  prevAccelTime_(0, 0, RCL_ROS_TIME),
  lastCmdTime_(0, 0, RCL_ROS_TIME)
{
  setRosParameter();

  model_ = ddl::DiffDriveModel{wheelRadius_, wheelToWheel_};
  accel_ = std::make_unique<ddl::AccelEstimator>(accelAlpha_);

  DifferentialDriveControllerApi::Config config;
  config.accTimeMilliseconds = accTime_;
  config.decTimeMilliseconds = decTime_;
  api_ = std::make_unique<DifferentialDriveControllerApi>(this->get_logger(), config);

  RCLCPP_INFO(
    this->get_logger(),
    "differential_drive_controller is unconfigured, waiting for configure...");
}

DifferentialDriveController::~DifferentialDriveController()
{
  api_.reset();
  RCLCPP_INFO(this->get_logger(), "differential_drive_controller stopped");
}

bool DifferentialDriveController::isNodeActive() const
{
  return isNodeActive_.load();
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring differential_drive_controller...");

  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return CallbackReturn::FAILURE;
  }

  setWmxParam(wmxParamFilePath_);

  encoderOmegaPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    encoderOmegaTopic_, 1);
  encoderOdometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    encoderOdometryTopic_, 1);
  odomDeltasPub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    odomDeltasTopic_, 1);
  odomAccelPub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
    odomAccelTopic_, 1);
  if (publishTf_) {
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  cmdVelStampedSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    cmdVelTopic_, 1, std::bind(&DifferentialDriveController::cmdStampedCallback, this, _1));

  controlTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / rate_),
    std::bind(&DifferentialDriveController::controlStep, this));
  controlTimer_->cancel();

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is configured");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isNodeActive_ = true;

  havePrev_ = false;
  haveCmd_ = false;
  haveAccelClock_ = false;
  lastSentValid_ = false;

  controlTimer_->reset();

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is active");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  controlTimer_->cancel();

  setVelocity(leftAxis_, 0.0);
  setVelocity(rightAxis_, 0.0);

  isNodeActive_ = false;

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is inactive");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isNodeActive_ = false;

  controlTimer_.reset();
  cmdVelStampedSub_.reset();
  encoderOmegaPub_.reset();
  encoderOdometryPub_.reset();
  odomDeltasPub_.reset();
  odomAccelPub_.reset();
  tfBroadcaster_.reset();

  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

DifferentialDriveController::CallbackReturn DifferentialDriveController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void DifferentialDriveController::cmdStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!isNodeActive()) {
    return;
  }

  cmdVelMsg_ = msg->twist;
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  lastCmdTime_ = (stamp.nanoseconds() > 0) ? stamp : this->get_clock()->now();
  haveCmd_ = true;
}

void DifferentialDriveController::controlStep()
{
  const rclcpp::Time now = this->get_clock()->now();

  DifferentialDriveControllerApi::AxisFeedback left;
  DifferentialDriveControllerApi::AxisFeedback right;
  bool communicating = false;
  std::string message;

  api_->getStatus(leftAxis_, rightAxis_, left, right, communicating, message);

  if (!communicating) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Communication or engine off. Please start the engine or communication");
    havePrev_ = false;
    lastSentValid_ = false;
    return;
  }

  const ddl::WheelOmega enc{left.actualVelocity, right.actualVelocity};
  const ddl::BodyVel body = model_.forward(enc);  // {vx, vyaw}, vy is 0 for diff-drive

  if (havePrev_) {
    const double dt = (now - prevLoopTime_).seconds();
    const double dPhiLeft = (left.actualPos - prevPosLeft_) * posUnitScale_;
    const double dPhiRight = (right.actualPos - prevPosRight_) * posUnitScale_;
    const bool finiteDt = std::isfinite(dt) && dt > 0.0;
    const bool jumped = finiteDt &&
      (std::abs(dPhiLeft - left.actualVelocity * dt) > jumpGuardTol_ ||
      std::abs(dPhiRight - right.actualVelocity * dt) > jumpGuardTol_);
    if (jumped) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Encoder position jump (homing/rollover?) -- re-baselining odometry this cycle");
    } else {
      const ddl::BodyVel d = model_.forwardDelta(dPhiLeft, dPhiRight);  // {ds, dtheta}
      integrator_.odometryPoseCalculation(d.linear, d.angular);
      deltas_.odometryDeltaAccumulation(d.linear, d.angular);
    }
  }
  prevPosLeft_ = left.actualPos;
  prevPosRight_ = right.actualPos;
  prevLoopTime_ = now;
  havePrev_ = true;

  publishOmega(enc);
  publishOdometry(now, body);
  publishDeltas(now);
  publishAccel(now, body);
  if (publishTf_) {publishTf(now);}

  if (left.ampAlarm || right.ampAlarm) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo alarm on. Please clear servo alarm");
    lastSentValid_ = false;  // force a resend once the alarm clears
    return;
  }
  if (!left.servoOn || !right.servoOn) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo off. Please set servo on");
    lastSentValid_ = false;  // force a resend once servo is back on
    return;
  }

  const bool stale = !haveCmd_ || (now - lastCmdTime_).seconds() > cmdVelTimeout_;
  const ddl::BodyVel cmd =
    stale ? ddl::BodyVel{0.0, 0.0} : ddl::BodyVel{cmdVelMsg_.linear.x, cmdVelMsg_.angular.z};

  const ddl::WheelOmega target = model_.inverse(cmd);
  commandWheels(target.left, target.right);
}

void DifferentialDriveController::commandWheels(double omegaLeft, double omegaRight)
{
  if (lastSentValid_ && omegaLeft == lastSentLeft_ && omegaRight == lastSentRight_) {
    return;
  }
  const bool okLeft = setVelocity(leftAxis_, omegaLeft);
  const bool okRight = setVelocity(rightAxis_, omegaRight);
  if (okLeft && okRight) {
    lastSentLeft_ = omegaLeft;
    lastSentRight_ = omegaRight;
    lastSentValid_ = true;
  } else {
    lastSentValid_ = false;
  }
}

bool DifferentialDriveController::setVelocity(int axis, double omega)
{
  std::string message;
  return api_->startVel(axis, omega, message) == ErrorCode::None;
}

void DifferentialDriveController::publishOmega(const ddl::WheelOmega & enc)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {enc.left, enc.right};
  encoderOmegaPub_->publish(msg);
}

void DifferentialDriveController::publishOdometry(
  const rclcpp::Time & stamp, const ddl::BodyVel & body)
{
  const ddl::Pose2D & pose = integrator_.pose();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odomFrame_;
  msg.child_frame_id = baseFrame_;

  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  msg.pose.pose.orientation = yawToQuaternion(pose.theta);

  msg.twist.twist.linear.x = body.linear;
  msg.twist.twist.linear.y = 0.0;     // diff-drive: no lateral motion
  msg.twist.twist.angular.z = body.angular;

  constexpr double kSmall = 0.01;
  constexpr double kLarge = 99999.0;
  msg.pose.covariance[0] = kSmall;    // x
  msg.pose.covariance[7] = kSmall;    // y
  msg.pose.covariance[14] = kLarge;   // z
  msg.pose.covariance[21] = kLarge;   // roll
  msg.pose.covariance[28] = kLarge;   // pitch
  msg.pose.covariance[35] = kSmall;   // yaw
  msg.twist.covariance[0] = kSmall;   // vx
  msg.twist.covariance[7] = kSmall;   // vy
  msg.twist.covariance[14] = kLarge;  // vz
  msg.twist.covariance[21] = kLarge;  // v_roll
  msg.twist.covariance[28] = kLarge;  // v_pitch
  msg.twist.covariance[35] = kSmall;  // vyaw

  encoderOdometryPub_->publish(msg);
}

void DifferentialDriveController::publishDeltas(const rclcpp::Time & stamp)
{
  const ddl::OdomDelta delta = deltas_.take();
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odomFrame_;
  msg.twist.linear.x = delta.linear;
  msg.twist.angular.z = delta.angular;
  odomDeltasPub_->publish(msg);
}

void DifferentialDriveController::publishAccel(
  const rclcpp::Time & stamp, const ddl::BodyVel & body)
{
  if (!haveAccelClock_) {
    prevAccelTime_ = stamp;
    haveAccelClock_ = true;
    return;
  }
  const double dtAccel = (stamp - prevAccelTime_).seconds();
  const double targetDt = (accelPublishRate_ > 0.0) ? 1.0 / accelPublishRate_ : 0.0;
  if (dtAccel < targetDt) {
    return;
  }

  const ddl::BodyAccel accel = accel_->update(body, dtAccel);
  geometry_msgs::msg::AccelStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = baseFrame_;
  msg.accel.linear.x = accel.linear;
  msg.accel.angular.z = accel.angular;
  odomAccelPub_->publish(msg);

  prevAccelTime_ = stamp;
}

void DifferentialDriveController::publishTf(const rclcpp::Time & stamp)
{
  const ddl::Pose2D & pose = integrator_.pose();
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odomFrame_;
  tf.child_frame_id = baseFrame_;
  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.rotation = yawToQuaternion(pose.theta);
  tfBroadcaster_->sendTransform(tf);
}

geometry_msgs::msg::Quaternion DifferentialDriveController::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void DifferentialDriveController::setWmxParam(const std::string & path)
{
  std::string message;
  if (api_->importAndSetAll(path, message) != ErrorCode::None) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  }
}

void DifferentialDriveController::setRosParameter()
{
  this->declare_parameter<int>("left_axis", 0);
  this->declare_parameter<int>("right_axis", 1);

  this->declare_parameter<int>("rate", 100);
  this->declare_parameter<double>("acc_time", 1.0);
  this->declare_parameter<double>("dec_time", 1.0);
  this->declare_parameter<double>("wheel_radius", 0.095);
  this->declare_parameter<double>("wheel_to_wheel", 0.55);

  this->declare_parameter<double>("cmd_vel_timeout", 0.25);
  this->declare_parameter<double>("accel_publish_rate", 10.0);
  this->declare_parameter<double>("accel_alpha", 0.3);
  this->declare_parameter<bool>("publish_tf", false);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<double>("pos_unit_scale", 1.0);
  this->declare_parameter<double>("jump_guard_tol", 0.5);

  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
  this->declare_parameter<std::string>("encoder_omega_topic", "/omega_enc");
  this->declare_parameter<std::string>("encoder_odometry_topic", "/odom_enc");
  this->declare_parameter<std::string>("odom_deltas_topic", "/odom_deltas");
  this->declare_parameter<std::string>("odom_accel_topic", "/odom_accel");
  this->declare_parameter<std::string>("wmx_param_file_path", "/diff_drive/no_param");

  this->get_parameter("left_axis", leftAxis_);
  this->get_parameter("right_axis", rightAxis_);

  this->get_parameter("rate", rate_);
  this->get_parameter("acc_time", accTime_);
  this->get_parameter("dec_time", decTime_);
  this->get_parameter("wheel_radius", wheelRadius_);
  this->get_parameter("wheel_to_wheel", wheelToWheel_);

  this->get_parameter("cmd_vel_timeout", cmdVelTimeout_);
  this->get_parameter("accel_publish_rate", accelPublishRate_);
  this->get_parameter("accel_alpha", accelAlpha_);
  this->get_parameter("publish_tf", publishTf_);
  this->get_parameter("odom_frame", odomFrame_);
  this->get_parameter("base_frame", baseFrame_);
  this->get_parameter("pos_unit_scale", posUnitScale_);
  this->get_parameter("jump_guard_tol", jumpGuardTol_);

  this->get_parameter("cmd_vel_topic", cmdVelTopic_);
  this->get_parameter("encoder_omega_topic", encoderOmegaTopic_);
  this->get_parameter("encoder_odometry_topic", encoderOdometryTopic_);
  this->get_parameter("odom_deltas_topic", odomDeltasTopic_);
  this->get_parameter("odom_accel_topic", odomAccelTopic_);
  this->get_parameter("wmx_param_file_path", wmxParamFilePath_);

  if (rate_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "rate must be > 0; falling back to 100 Hz");
    rate_ = 100;
  }
  if (wheelRadius_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "wheel_radius must be > 0; falling back to 0.095");
    wheelRadius_ = 0.095;
  }
  if (wheelToWheel_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "wheel_to_wheel must be > 0; falling back to 0.55");
    wheelToWheel_ = 0.55;
  }
  if (accelPublishRate_ < 0.0) {
    RCLCPP_WARN(this->get_logger(), "accel_publish_rate must be >= 0; falling back to 10.0");
    accelPublishRate_ = 10.0;
  }
  if (!std::isfinite(posUnitScale_) || posUnitScale_ == 0.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "pos_unit_scale must be finite and non-zero; falling back to 1.0");
    posUnitScale_ = 1.0;
  }
  if (!(jumpGuardTol_ > 0.0)) {
    RCLCPP_WARN(this->get_logger(), "jump_guard_tol must be > 0; falling back to 0.5");
    jumpGuardTol_ = 0.5;
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "left_axis: %d, right_axis: %d", leftAxis_, rightAxis_);
  RCLCPP_INFO(this->get_logger(), "rate: %d", rate_);
  RCLCPP_INFO(this->get_logger(), "acc_time: %f, dec_time: %f", accTime_, decTime_);
  RCLCPP_INFO(this->get_logger(), "wheel_radius: %f", wheelRadius_);
  RCLCPP_INFO(this->get_logger(), "wheel_to_wheel: %f", wheelToWheel_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %f", cmdVelTimeout_);
  RCLCPP_INFO(
    this->get_logger(), "accel_publish_rate: %f, accel_alpha: %f",
    accelPublishRate_, accelAlpha_);
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publishTf_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "odom_frame: %s, base_frame: %s",
    odomFrame_.c_str(), baseFrame_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "pos_unit_scale: %f, jump_guard_tol: %f",
    posUnitScale_, jumpGuardTol_);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: %s", cmdVelTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_omega_topic: %s", encoderOmegaTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "encoder_odometry_topic: %s", encoderOdometryTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_deltas_topic: %s", odomDeltasTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_accel_topic: %s", odomAccelTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "wmx_param_file_path: %s", wmxParamFilePath_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DifferentialDriveController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
