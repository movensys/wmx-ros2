// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// Standalone differential-drive controller (rclcpp node).
//
// Drives the WMX3 wheel axes directly via CoreMotion StartVel and exposes the
// autonomy contract consumed by the upstream autonomy stack:
//   in : /cmd_vel_safe          geometry_msgs/TwistStamped  (header stamp drives the
//                                                            stale-command safety timeout)
//   out: /odom_enc              nav_msgs/Odometry      (EKF odom0 input; fuses
//                                                       vx, vy(=0), vyaw only)
//        /odom_deltas           geometry_msgs/TwistStamped  (DistanceTraveled monitor)
//        /odom_accel            geometry_msgs/AccelStamped  (Motion monitor, rate-limited)
//        /omega_enc             std_msgs/Float64MultiArray  (per-wheel actualVelocity)
//        /tf  odom->base_link   (optional; OFF by default — the EKF owns this TF when
//                                an IMU is present)
//
// The WMX-free math (kinematics, dead-reckoning, deltas, accel EMA) lives in the
// unit-tested differential_drive_controller.hpp; this node is the ROS/WMX wiring around it.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "differential_drive_controller.hpp"

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

namespace ddl = diff_drive;

class DifferentialDriveController : public rclcpp::Node
{
public:
  DifferentialDriveController();
  ~DifferentialDriveController();

  // Geometry / axes
  int leftAxis_;
  int rightAxis_;
  int rate_;
  double accTime_;
  double decTime_;
  double wheelRadius_;
  double wheelToWheel_;

  // Contract / behaviour
  double cmdVelTimeout_;
  double accelPublishRate_;
  double accelAlpha_;
  bool publishTf_;
  std::string odomFrame_;
  std::string baseFrame_;
  double posUnitScale_;   // wheel-rad per WMX user-unit of actualPos (1.0 if already rad)
  double jumpGuardTol_;   // [rad] max |dPhi - actualVelocity*dt| before re-baselining

  // Topics
  std::string cmdVelTopic_;
  std::string encoderOmegaTopic_;
  std::string encoderOdometryTopic_;
  std::string odomDeltasTopic_;
  std::string odomAccelTopic_;
  std::string wmxParamFilePath_;

  int err_;
  char errString_[256];

private:
  std::atomic<bool> initialized_{false};
  std::atomic<bool> initializing_{false};

  // --- WMX ---
  WMX3Api wmx3Lib_;
  CoreMotionStatus cmStatus_;
  std::unique_ptr<CoreMotion> wmx3LibCm_;
  Velocity::VelCommand velCommand_;

  // --- diff-drive logic ---
  ddl::DiffDriveModel model_;
  ddl::OdometryIntegrator integrator_;
  ddl::OdomDeltaAccumulator deltas_;
  std::unique_ptr<ddl::AccelEstimator> accel_;

  // --- Loop / command state ---
  // Pose & /odom_deltas are dead-reckoned from per-wheel encoder POSITION deltas
  // (actualPos) — more precise than velocity*dt. prevLoopTime_ now only feeds the
  // jump-guard's expected step (actualVelocity*dt); havePrev_ gates the first valid
  // cycle (no previous position/time sample yet) and re-baselining on recovery.
  rclcpp::Time prevLoopTime_;
  double prevPosLeft_ = 0.0;
  double prevPosRight_ = 0.0;
  bool havePrev_ = false;
  rclcpp::Time prevAccelTime_;
  bool haveAccelClock_ = false;

  geometry_msgs::msg::Twist cmdVelMsg_;
  rclcpp::Time lastCmdTime_;
  bool haveCmd_ = false;

  // resend-only-on-change cache (see commandWheels)
  double lastSentLeft_ = 0.0;
  double lastSentRight_ = 0.0;
  bool lastSentValid_ = false;

  // --- ROS interfaces ---
  rclcpp::TimerBase::SharedPtr controlTimer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelStampedSub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr encoderOmegaPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoderOdometryPub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odomDeltasPub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr odomAccelPub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::thread init_thread_;

  // Lifecycle
  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void runInitSequence();
  void setRosParameter();
  void setWmxParam(char * path);

  // Control loop
  void cmdStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void controlStep();
  void commandWheels(double omegaLeft, double omegaRight);
  bool setVelocity(int axis, double omega);

  // Publishers
  void publishOmega(const ddl::WheelOmega & enc);
  void publishOdometry(const rclcpp::Time & stamp, const ddl::BodyVel & body);
  void publishDeltas(const rclcpp::Time & stamp);
  void publishAccel(const rclcpp::Time & stamp, const ddl::BodyVel & body);
  void publishTf(const rclcpp::Time & stamp);

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
};

DifferentialDriveController::DifferentialDriveController()
: Node("differential_drive_controller"),
  prevLoopTime_(0, 0, RCL_ROS_TIME),
  prevAccelTime_(0, 0, RCL_ROS_TIME),
  lastCmdTime_(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "start differential_drive_controller");

  setRosParameter();

  model_ = ddl::DiffDriveModel{wheelRadius_, wheelToWheel_};
  accel_ = std::make_unique<ddl::AccelEstimator>(accelAlpha_);

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&DifferentialDriveController::onEngineReady, this, _1));

  RCLCPP_INFO(this->get_logger(), "differential_drive_controller waiting for engine...");
}

DifferentialDriveController::~DifferentialDriveController()
{
  RCLCPP_INFO(this->get_logger(), "Stop differential_drive_controller");

  if (init_thread_.joinable()) {
    init_thread_.join();
  }

  if (initialized_) {
    if (controlTimer_) {controlTimer_->cancel();}

    setVelocity(leftAxis_, 0.0);
    setVelocity(rightAxis_, 0.0);

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

void DifferentialDriveController::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
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
  init_thread_ = std::thread(&DifferentialDriveController::runInitSequence, this);
}

void DifferentialDriveController::runInitSequence()
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

  setWmxParam(const_cast<char *>(wmxParamFilePath_.c_str()));

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

  // Command input is TwistStamped (mandatory): the staleness timeout keys off the
  // publisher's stamp, which stops the robot on crash/disconnect. A zero/unset stamp
  // falls back to arrival time so a stampless publisher does not look infinitely stale.
  cmdVelStampedSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    cmdVelTopic_, 1, std::bind(&DifferentialDriveController::cmdStampedCallback, this, _1));

  // Single control loop: one GetStatus per cycle drives both odometry and command.
  auto period = std::chrono::milliseconds(1000 / rate_);
  controlTimer_ = this->create_wall_timer(
    period, std::bind(&DifferentialDriveController::controlStep, this));

  initialized_ = true;
  engineReadySub_.reset();
  RCLCPP_INFO(this->get_logger(), "differential_drive_controller is ready");
}

void DifferentialDriveController::cmdStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  cmdVelMsg_ = msg->twist;
  // Key the staleness timeout off the publisher's stamp when present: this rejects
  // stale or buffered commands (not just gaps in receipt), the safety property the
  // stamped contract is for. A zero/unset stamp falls back to arrival time so a
  // stampless publisher does not look infinitely stale and brick the robot. The stamp
  // shares the control loop's clock domain (RCL_ROS_TIME / sim time when use_sim_time).
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  lastCmdTime_ = (stamp.nanoseconds() > 0) ? stamp : this->get_clock()->now();
  haveCmd_ = true;
}

void DifferentialDriveController::controlStep()
{
  const rclcpp::Time now = this->get_clock()->now();

  // Single status read for this cycle.
  wmx3LibCm_->GetStatus(&cmStatus_);
  const CoreMotionAxisStatus * left = &cmStatus_.axesStatus[leftAxis_];
  const CoreMotionAxisStatus * right = &cmStatus_.axesStatus[rightAxis_];

  // If the engine isn't communicating the status is not trustworthy: don't
  // publish odometry or command. Drop our caches so the position baseline, the
  // jump-guard clock, and the StartVel resend re-baseline cleanly on recovery
  // (re-anchoring to the current absolute encoder position, which may have moved
  // or homed while we were not reading it).
  if (cmStatus_.engineState != EngineState::T::Communicating) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Communication or engine off. Please start the engine or communication");
    havePrev_ = false;
    lastSentValid_ = false;
    return;
  }

  // ---- Odometry path (encoder feedback is valid even with servo off) ----
  // Twist (-> /odom_enc, /omega_enc, /odom_accel) comes from the servo's own
  // actualVelocity: it is the cleaner velocity signal and the localization EKF
  // fuses only the twist (vx/vy/vyaw) from /odom_enc.
  const ddl::WheelOmega enc{left->actualVelocity, right->actualVelocity};
  const ddl::BodyVel body = model_.forward(enc);  // {vx, vyaw}, vy is 0 for diff-drive

  // Pose & /odom_deltas are dead-reckoned from per-wheel encoder POSITION deltas:
  // more precise than velocity*dt (dt-free, no constant-velocity assumption, no
  // dt-jitter sensitivity). actualPos is a WMX user-unit; posUnitScale_ converts it
  // to wheel radians (1.0 when the loaded WMX param file already scales to rad, as
  // our config does -- verified: actualVelocity reads rad/s under the same scaling).
  if (havePrev_) {
    const double dt = (now - prevLoopTime_).seconds();
    const double dPhiLeft = (left->actualPos - prevPosLeft_) * posUnitScale_;
    const double dPhiRight = (right->actualPos - prevPosRight_) * posUnitScale_;
    // Jump guard: a homing, encoder rollover, or position glitch shows up as a
    // per-step angle delta wildly inconsistent with actualVelocity*dt. On a trip,
    // re-baseline (contribute nothing this cycle) instead of integrating a bogus
    // jump; legitimate high-speed motion stays consistent and passes. (The guard
    // needs a valid dt; if dt is non-positive/non-finite the position delta is
    // still valid on its own, so we integrate it unguarded rather than drop it.)
    const bool finiteDt = std::isfinite(dt) && dt > 0.0;
    const bool jumped = finiteDt &&
      (std::abs(dPhiLeft - left->actualVelocity * dt) > jumpGuardTol_ ||
      std::abs(dPhiRight - right->actualVelocity * dt) > jumpGuardTol_);
    if (jumped) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Encoder position jump (homing/rollover?) -- re-baselining odometry this cycle");
    } else {
      const ddl::BodyVel d = model_.forwardDelta(dPhiLeft, dPhiRight);  // {ds, dtheta}
      integrator_.integrateDelta(d.linear, d.angular);
      deltas_.accumulateDelta(d.linear, d.angular);
    }
  }
  prevPosLeft_ = left->actualPos;
  prevPosRight_ = right->actualPos;
  prevLoopTime_ = now;
  havePrev_ = true;

  publishOmega(enc);
  publishOdometry(now, body);
  publishDeltas(now);
  publishAccel(now, body);
  if (publishTf_) {publishTf(now);}

  // ---- Command path ----
  if (left->ampAlarm || right->ampAlarm) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo alarm on. Please clear servo alarm");
    lastSentValid_ = false;  // force a resend once the alarm clears
    return;
  }
  if (!left->servoOn || !right->servoOn) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Servo off. Please set servo on");
    lastSentValid_ = false;  // force a resend once servo is back on
    return;
  }

  // Stale-command safety: zero the wheels if no fresh /cmd_vel within the timeout.
  // NOTE: this decelerates over dec_time (TimeAccTrapezoidal); a true emergency stop
  // must go through the WMX hardware-level stop path, not this software timeout.
  const bool stale = !haveCmd_ || (now - lastCmdTime_).seconds() > cmdVelTimeout_;
  const ddl::BodyVel cmd =
    stale ? ddl::BodyVel{0.0, 0.0} : ddl::BodyVel{cmdVelMsg_.linear.x, cmdVelMsg_.angular.z};

  const ddl::WheelOmega target = model_.inverse(cmd);
  commandWheels(target.left, target.right);
}

void DifferentialDriveController::commandWheels(double omegaLeft, double omegaRight)
{
  // Resend only on change so we don't restart the velocity trapezoid every cycle.
  // The status/alarm/timeout branches above invalidate the cache so a transition
  // (timeout->zero, alarm-recovery) is always re-sent.
  if (lastSentValid_ && omegaLeft == lastSentLeft_ && omegaRight == lastSentRight_) {
    return;
  }
  // Commit the resend cache only if BOTH axes accepted the command; otherwise leave
  // it invalid so the next cycle re-attempts this same target. A StartVel that failed
  // (e.g. a transient motion-state conflict) must NOT be remembered as "already sent"
  // — that would suppress retries, including a critical timeout->zero stop.
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
    return false;
  }
  return true;
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

  // Covariance. The localization EKF fuses ONLY twist vx, vy, vyaw from this source
  // (robot_localization odom0_config indices 6, 7, 11 of its 15-state vector). In this
  // nav_msgs 6x6 twist covariance those same components are the diagonal entries
  // [0]=vx, [7]=vy, [35]=vyaw — set below. The EKF ignores pose, but x/y/yaw are kept
  // authoritative (small variance) for the no-EKF fallback where this odom feeds Nav2
  // directly; unused axes (z/roll/pitch) are non-authoritative. Mirrors the proven upstream
  // diff_drive_node values.
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
  // accumulated |ds|, |dtheta| from per-wheel position deltas; resets on take()
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
  // Rate-limit accel publishing relative to the (fast) control loop. The estimator
  // differentiates body velocity over the actual elapsed accel interval, so we feed
  // it dt measured between accel publishes — not the control-loop dt.
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
  // Closed-form yaw-only quaternion (equivalent to tf2::Quaternion::setRPY(0,0,yaw)),
  // built directly to avoid tf2 LinearMath/geometry_msgs header churn across distros.
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void DifferentialDriveController::setWmxParam(char * path)
{
  err_ = wmx3LibCm_->config->ImportAndSetAll(path);
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to set WMX params. Error=%d (%s)", err_, errString_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Success to set WMX params");
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
  // Guard the kinematics preconditions in release builds too: DiffDriveModel only
  // assert()s wheel_radius/separation > 0, and assert is elided under NDEBUG (the
  // default for optimized ROS 2 builds), so a 0 here would yield inf/nan odometry.
  if (wheelRadius_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "wheel_radius must be > 0; falling back to 0.095");
    wheelRadius_ = 0.095;
  }
  if (wheelToWheel_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "wheel_to_wheel must be > 0; falling back to 0.55");
    wheelToWheel_ = 0.55;
  }
  // accel_publish_rate: 0 means "publish /odom_accel every control cycle"; a negative
  // value is a misconfiguration, so guard it (otherwise it falls through to every-cycle).
  if (accelPublishRate_ < 0.0) {
    RCLCPP_WARN(this->get_logger(), "accel_publish_rate must be >= 0; falling back to 10.0");
    accelPublishRate_ = 10.0;
  }
  // pos_unit_scale converts actualPos user-units -> wheel rad; 0 or non-finite would
  // zero/poison all position-based odometry, so guard it to the rad-native default.
  if (!std::isfinite(posUnitScale_) || posUnitScale_ == 0.0) {
    RCLCPP_WARN(this->get_logger(),
      "pos_unit_scale must be finite and non-zero; falling back to 1.0");
    posUnitScale_ = 1.0;
  }
  // jump_guard_tol must be > 0 (it bounds |dPhi - actualVelocity*dt| before re-baselining).
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
  RCLCPP_INFO(this->get_logger(), "accel_publish_rate: %f, accel_alpha: %f",
    accelPublishRate_, accelAlpha_);
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publishTf_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "odom_frame: %s, base_frame: %s",
    odomFrame_.c_str(), baseFrame_.c_str());
  RCLCPP_INFO(this->get_logger(), "pos_unit_scale: %f, jump_guard_tol: %f",
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
  rclcpp::spin(std::make_shared<DifferentialDriveController>());
  rclcpp::shutdown();
  return 0;
}
