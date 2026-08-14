// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.
//
// servo_stream_controller
//
// Streams MoveIt Servo joint commands into the WMX3 API buffer, which replays
// them under the real-time OS at communication-cycle boundaries.
//
// This is the buffered alternative to joint_position_controller, which calls
// StartLinearIntplPos directly from the ROS subscription callback. That path
// works, but every command crosses from a non-real-time Linux thread straight
// into the engine, so ROS scheduling jitter lands directly on motion timing and
// a late thread leaves a gap in the stream. Here the API buffer absorbs that
// jitter, and its watch function adds a hardware-side safety net.
//
// The two nodes subscribe to the same topic and MUST NOT run together.
//
//   Servo topic -> ROS ring (bounded, drop-oldest) -> pump -> API buffer -> engine
//
// Each setpoint becomes two blocks:
//   StartLinearIntplPos(6 axes)   coordinated, lands as a FastBlending override
//   USleep(T)                     paces the queue, T trimmed to hold depth
//
// Notes on the motion model:
//   - LinearIntplOverrideType is FastBlending, so consecutive interpolations
//     blend rather than stop-and-go.
//   - profile.startingVelocity = 0 lets the override inherit the current
//     velocity; continuity comes from the blend. Setting endVelocity per
//     segment would fight it, so we do not.
//   - LinearIntplProfileCalcMode is read from axis[0], so the axis array is
//     always filled in joint_axes order. That ordering is semantics, not style.
//   - Segment distance is measured from the PREVIOUS SETPOINT, not from the
//     current commanded position: with a queue, posCmd lags the setpoints in
//     flight and would overestimate every step.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "ApiBufferApi.h"

using std::placeholders::_1;
using wmx3Api::ApiBuffer;
using wmx3Api::ApiBufferOptions;
using wmx3Api::ApiBufferState;
using wmx3Api::ApiBufferStatus;
using wmx3Api::ApiBufferWatch;
using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::Motion;
using wmx3Api::ProfileType;
using wmx3Api::SizeUnit;
using wmx3Api::WMX3Api;

namespace
{
constexpr int kBlocksPerSetpoint = 2;      // StartLinearIntplPos + USleep
constexpr auto kPumpIdleWait = std::chrono::milliseconds(5);
constexpr auto kStatusPollPeriod = std::chrono::milliseconds(100);   // ~10 Hz
}  // namespace

class ServoStreamController : public rclcpp::Node
{
public:
  ServoStreamController();
  ~ServoStreamController();

private:
  // --- WMX3 handles -------------------------------------------------------
  // Two devices. StartRecordBufferChannel is DEVICE-wide: while recording,
  // every WMX3 call through that device is recorded instead of executed. The
  // pump needs CoreMotion::GetStatus and Motion::Stop (both of which WOULD be
  // recorded), so the recording device is kept strictly separate.
  WMX3Api wmxRec_;
  WMX3Api wmxCtl_;
  std::unique_ptr<CoreMotion> cmRec_;
  std::unique_ptr<CoreMotion> cmCtl_;
  std::unique_ptr<ApiBuffer> abRec_;
  std::unique_ptr<ApiBuffer> abCtl_;

  bool recAttached_ = false;
  bool ctlAttached_ = false;
  bool bufferCreated_ = false;
  bool initialized_ = false;

  int err_ = 0;
  char errString_[256];

  // --- parameters ---------------------------------------------------------
  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  int channelMotion_ = 0;
  int channelEstop_ = 1;
  int bufferSizeMb_ = 5;
  int targetQueueDepth_ = 2;
  int nominalPeriodCycles_ = 25;
  int clampLoCycles_ = 20;
  int clampHiCycles_ = 30;
  double pacingKp_ = 0.15;
  int rosQueueDepth_ = 8;
  int starvationTimeoutMs_ = 75;
  double accelRatio_ = 0.3;
  bool stopOnError_ = true;

  std::map<std::string, int> axisByName_;
  AxisSelection axisSel_;
  Motion::LinearIntplCommand intpl_;

  // --- streaming state ----------------------------------------------------
  std::mutex ringMtx_;
  std::condition_variable ringCv_;
  std::deque<std::vector<double>> ring_;      // targets, in joint_axes order

  std::vector<double> lastTarget_;            // origin of the next segment
  bool haveLastTarget_ = false;
  bool streaming_ = false;                    // buffer Active and being fed
  long long blocksAdded_ = 0;                 // stands in for the SDK's missing
                                              // GetCumulativeBlockCount
  int lastErrorCount_ = 0;

  std::atomic<bool> inExecution_{false};      // move_group owns the arm
  std::atomic<bool> running_{true};
  std::atomic<int> observedDepth_{0};
  std::atomic<bool> initializing_{false};

  std::thread pumpThread_;
  std::thread initThread_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr depthPub_;
  rclcpp::TimerBase::SharedPtr statusTimer_;

  // --- helpers ------------------------------------------------------------
  void setRosParameter();
  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onExecActive(const std_msgs::msg::Bool::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void runInitSequence();
  bool attachDevice(WMX3Api * lib, const char * name);
  bool check(int err, const char * what);
  bool setupBuffers();
  void recordEstopRoutine();

  void pumpLoop();
  bool beginStream();
  void endStream(const char * reason, bool controlled_stop);
  bool recordSetpoint(const std::vector<double> & target, int period_cycles);
  int pacePeriodCycles(int depth) const;
  void pollStatus();
};

ServoStreamController::ServoStreamController()
: Node("servo_stream_controller")
{
  setRosParameter();

  depthPub_ = this->create_publisher<std_msgs::msg::Int32>("~/queue_depth", 10);

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&ServoStreamController::onEngineReady, this, _1));

  execActiveSub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local(),
    std::bind(&ServoStreamController::onExecActive, this, _1));

  RCLCPP_INFO(this->get_logger(), "servo_stream_controller waiting for engine...");
}

ServoStreamController::~ServoStreamController()
{
  running_ = false;
  ringCv_.notify_all();

  if (pumpThread_.joinable()) {
    pumpThread_.join();
  }
  if (initThread_.joinable()) {
    initThread_.join();
  }

  if (initialized_) {
    // Stop issuing blocks, drop what is queued, then bring the axes to rest
    // with the configured stop profile.
    if (abCtl_) {
      abCtl_->Halt(channelMotion_);
      abCtl_->Clear(channelMotion_);
    }
    if (cmCtl_) {
      cmCtl_->motion->Stop(&axisSel_);
      cmCtl_->motion->Wait(&axisSel_);
    }
    if (abCtl_ && bufferCreated_) {
      abCtl_->FreeApiBuffer(channelMotion_);
      abCtl_->FreeApiBuffer(channelEstop_);
    }
  }

  if (recAttached_) {
    wmxRec_.CloseDevice();
  }
  if (ctlAttached_) {
    wmxCtl_.CloseDevice();
  }

  RCLCPP_INFO(this->get_logger(), "servo_stream_controller is stopped");
}

void ServoStreamController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>(
    "joint_name", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  this->declare_parameter<std::string>(
    "joint_trajectory_topic", "/joint_trajectory_topic/no_param");
  this->declare_parameter<int>("api_buffer_channel", 0);
  this->declare_parameter<int>("estop_buffer_channel", 1);
  this->declare_parameter<int>("api_buffer_size_mb", 5);
  this->declare_parameter<int>("target_queue_depth", 2);
  this->declare_parameter<int>("nominal_period_cycles", 25);
  this->declare_parameter<int>("period_clamp_lo_cycles", 20);
  this->declare_parameter<int>("period_clamp_hi_cycles", 30);
  this->declare_parameter<double>("pacing_kp", 0.15);
  this->declare_parameter<int>("ros_queue_depth", 8);
  this->declare_parameter<int>("starvation_timeout_ms", 75);
  this->declare_parameter<double>("accel_ratio", 0.3);
  this->declare_parameter<bool>("stop_on_error", true);

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_name", jointNames_);
  this->get_parameter("joint_trajectory_topic", jointTrajectoryTopic_);
  this->get_parameter("api_buffer_channel", channelMotion_);
  this->get_parameter("estop_buffer_channel", channelEstop_);
  this->get_parameter("api_buffer_size_mb", bufferSizeMb_);
  this->get_parameter("target_queue_depth", targetQueueDepth_);
  this->get_parameter("nominal_period_cycles", nominalPeriodCycles_);
  this->get_parameter("period_clamp_lo_cycles", clampLoCycles_);
  this->get_parameter("period_clamp_hi_cycles", clampHiCycles_);
  this->get_parameter("pacing_kp", pacingKp_);
  this->get_parameter("ros_queue_depth", rosQueueDepth_);
  this->get_parameter("starvation_timeout_ms", starvationTimeoutMs_);
  this->get_parameter("accel_ratio", accelRatio_);
  this->get_parameter("stop_on_error", stopOnError_);

  for (size_t i = 0; i < jointNames_.size() && i < jointAxes_.size(); ++i) {
    axisByName_[jointNames_[i]] = static_cast<int>(jointAxes_[i]);
  }

  // Fixed order: axis[0] determines LinearIntplProfileCalcMode.
  axisSel_.axisCount = jointAxes_.size();
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    axisSel_.axis[i] = static_cast<int>(jointAxes_[i]);
  }

  if (channelMotion_ == channelEstop_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "estop_buffer_channel must differ from api_buffer_channel; watch trigger disabled");
  }

  std::string axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {axes_str += ", ";}
    axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_topic: %s", jointTrajectoryTopic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "api_buffer_channel: %d, estop_buffer_channel: %d",
    channelMotion_, channelEstop_);
  RCLCPP_INFO(this->get_logger(), "api_buffer_size_mb: %d", bufferSizeMb_);
  RCLCPP_INFO(
    this->get_logger(), "target_queue_depth: %d setpoints (~%d ms of latency)",
    targetQueueDepth_, targetQueueDepth_ * nominalPeriodCycles_);
  RCLCPP_INFO(
    this->get_logger(), "period: %d cycles, clamp [%d, %d], Kp %.2f",
    nominalPeriodCycles_, clampLoCycles_, clampHiCycles_, pacingKp_);
  RCLCPP_INFO(this->get_logger(), "ros_queue_depth: %d", rosQueueDepth_);
  RCLCPP_INFO(this->get_logger(), "starvation_timeout_ms: %d", starvationTimeoutMs_);
  RCLCPP_INFO(this->get_logger(), "accel_ratio: %.2f", accelRatio_);
  RCLCPP_INFO(this->get_logger(), "stop_on_error: %s", stopOnError_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "===========================");
}

bool ServoStreamController::check(int err, const char * what)
{
  if (err == ErrorCode::None) {
    return true;
  }
  err_ = err;
  ApiBuffer::ErrorToString(err, errString_, sizeof(errString_));
  RCLCPP_ERROR(this->get_logger(), "%s failed. Error=%d (%s)", what, err, errString_);
  return false;
}

void ServoStreamController::onEngineReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || initialized_ || initializing_.exchange(true)) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Engine ready — starting init on dedicated thread...");
  if (initThread_.joinable()) {
    initThread_.join();
  }
  initThread_ = std::thread(&ServoStreamController::runInitSequence, this);
}

bool ServoStreamController::attachDevice(WMX3Api * lib, const char * name)
{
  static constexpr int kMaxDeviceRetries = 30;
  unsigned int timeout = 10000;
  int err = ErrorCode::None;

  for (int attempt = 1; attempt <= kMaxDeviceRetries; ++attempt) {
    err = lib->CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);
    if (err == ErrorCode::None) {
      break;
    }
    lib->ErrorToString(err, errString_, sizeof(errString_));
    if (err == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        this->get_logger(), "Device lock busy for '%s', retrying in 1s... (%d/%d)",
        name, attempt, kMaxDeviceRetries);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to attach device '%s'. Error=%d (%s)",
        name, err, errString_);
      return false;
    }
  }

  if (err != ErrorCode::None) {
    RCLCPP_FATAL(
      this->get_logger(), "Device '%s' lock busy after %d retries", name, kMaxDeviceRetries);
    return false;
  }

  lib->SetDeviceName(name);
  RCLCPP_INFO(this->get_logger(), "Attached WMX3 device '%s'", name);
  return true;
}

// The watch trigger routine: a pre-recorded Stop on every joint axis, living in
// its own channel. If any watched axis goes servo-off, offline, amp-alarm or
// hits a limit, the engine halts the motion channel and runs this — entirely
// inside the RT side, with no dependency on ROS or Linux still being alive.
void ServoStreamController::recordEstopRoutine()
{
  if (channelMotion_ == channelEstop_) {
    return;
  }
  abRec_->StartRecordBufferChannel(channelEstop_);
  cmRec_->motion->Stop(&axisSel_);
  abRec_->EndRecordBufferChannel();
  RCLCPP_INFO(this->get_logger(), "Recorded e-stop routine into channel %d", channelEstop_);
}

bool ServoStreamController::setupBuffers()
{
  if (!check(
      abCtl_->CreateApiBuffer(channelMotion_, bufferSizeMb_, SizeUnit::Megabyte),
      "CreateApiBuffer(motion)"))
  {
    return false;
  }
  if (channelMotion_ != channelEstop_) {
    if (!check(
        abCtl_->CreateApiBuffer(channelEstop_, 1, SizeUnit::Megabyte),
        "CreateApiBuffer(estop)"))
    {
      return false;
    }
  }
  bufferCreated_ = true;

  recordEstopRoutine();

  ApiBufferOptions opt;
  opt.stopOnError = stopOnError_;
  opt.autoRewind = false;      // the ring wraps within seconds; rewind is unusable here
  opt.stopOnLastBlock = false; // MUST be false: an Active channel picks up new blocks
  if (!check(abCtl_->SetOptions(channelMotion_, &opt), "SetOptions")) {
    return false;
  }

  if (channelMotion_ != channelEstop_) {
    ApiBufferWatch watch;
    watch.enableWatch = true;
    watch.watchAxes = axisSel_;
    watch.enableWatchTriggerRoutine = true;
    watch.watchTriggerRoutineChannel = channelEstop_;
    if (!check(abCtl_->SetWatch(channelMotion_, &watch), "SetWatch")) {
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Watch enabled on %u axes, trigger routine on channel %d",
      axisSel_.axisCount, channelEstop_);
  }

  return true;
}

void ServoStreamController::runInitSequence()
{
  if (!attachDevice(&wmxRec_, "servo_stream_rec") ||
    !attachDevice(&wmxCtl_, "servo_stream_ctl"))
  {
    initializing_ = false;
    return;
  }
  recAttached_ = true;
  ctlAttached_ = true;

  cmRec_ = std::make_unique<CoreMotion>(&wmxRec_);
  cmCtl_ = std::make_unique<CoreMotion>(&wmxCtl_);
  abRec_ = std::make_unique<ApiBuffer>(&wmxRec_);
  abCtl_ = std::make_unique<ApiBuffer>(&wmxCtl_);

  if (!setupBuffers()) {
    initializing_ = false;
    return;
  }

  pumpThread_ = std::thread(&ServoStreamController::pumpLoop, this);

  statusTimer_ = this->create_wall_timer(
    kStatusPollPeriod, std::bind(&ServoStreamController::pollStatus, this));

  jointTrajectorySub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    jointTrajectoryTopic_, 1,
    std::bind(&ServoStreamController::jointTrajectoryCallback, this, _1));

  initialized_ = true;
  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "servo_stream_controller is ready");
}

void ServoStreamController::onExecActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (inExecution_.exchange(msg->data) == msg->data) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Servo stream %s (move_group execution %s)",
    msg->data ? "blocked" : "allowed", msg->data ? "started" : "finished");

  if (msg->data) {
    // move_group is about to move the arm somewhere else: everything queued is
    // now stale. Drop it and stop the channel. No Motion::Stop here — the
    // trajectory controller owns the axes from this point.
    {
      std::lock_guard<std::mutex> lock(ringMtx_);
      ring_.clear();
    }
    endStream("move_group execution started", false);
  }
}

void ServoStreamController::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty() || inExecution_.load() || !initialized_) {
    return;
  }

  const auto & pt = msg->points.back();
  const size_t count = jointAxes_.size();

  if (pt.positions.size() < count) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dropped trajectory: %zu positions for %zu axes", pt.positions.size(), count);
    return;
  }

  // Reorder incoming positions into joint_axes order.
  std::vector<double> target(count, 0.0);
  for (size_t i = 0; i < count; ++i) {
    if (msg->joint_names.empty()) {
      target[i] = pt.positions[i];
      continue;
    }
    const auto it = axisByName_.find(msg->joint_names[i]);
    if (it == axisByName_.end()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: joint '%s' is not in joint_name",
        msg->joint_names[i].c_str());
      return;
    }
    // Find where this axis sits in our fixed ordering.
    const auto pos = std::find(jointAxes_.begin(), jointAxes_.end(), it->second);
    if (pos == jointAxes_.end()) {
      return;
    }
    target[static_cast<size_t>(std::distance(jointAxes_.begin(), pos))] = pt.positions[i];
  }

  // Drop-oldest. A stale teleop setpoint is worthless, and the WMX3 ring cannot
  // be edited once written — so the drop policy has to live on this side.
  {
    std::lock_guard<std::mutex> lock(ringMtx_);
    while (ring_.size() >= static_cast<size_t>(rosQueueDepth_)) {
      ring_.pop_front();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ROS queue full (%d); dropping oldest setpoint — pump is not keeping up",
        rosQueueDepth_);
    }
    ring_.push_back(std::move(target));
  }
  ringCv_.notify_one();
}

// Adaptive playout: trim the sleep to hold the queue at target depth. Worked in
// whole cycles because the engine ticks at 1 kHz and USleep cannot resolve finer.
int ServoStreamController::pacePeriodCycles(int depth) const
{
  const double adjust = pacingKp_ * static_cast<double>(depth - targetQueueDepth_);
  const int cycles = static_cast<int>(std::lround(nominalPeriodCycles_ + adjust));
  return std::max(clampLoCycles_, std::min(clampHiCycles_, cycles));
}

bool ServoStreamController::beginStream()
{
  // Seed the segment origin from the arm's current commanded position. Every
  // later segment measures from the previous setpoint instead.
  CoreMotionStatus cmStatus;
  cmCtl_->GetStatus(&cmStatus);

  lastTarget_.assign(jointAxes_.size(), 0.0);
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    lastTarget_[i] = cmStatus.axesStatus[jointAxes_[i]].posCmd;
  }
  haveLastTarget_ = true;

  if (!check(abCtl_->Clear(channelMotion_), "ApiBuffer::Clear")) {
    return false;
  }
  if (!check(abCtl_->Execute(channelMotion_), "ApiBuffer::Execute")) {
    return false;
  }

  blocksAdded_ = 0;
  lastErrorCount_ = 0;
  streaming_ = true;
  RCLCPP_INFO(this->get_logger(), "Stream started (channel %d Active)", channelMotion_);
  return true;
}

// controlled_stop: bring the axes to rest with the configured stop profile.
// Halt alone does not stop motion already commanded — the last interpolation
// would run to its target with that segment's (deliberately brisk) deceleration.
void ServoStreamController::endStream(const char * reason, bool controlled_stop)
{
  if (!streaming_) {
    return;
  }
  streaming_ = false;
  haveLastTarget_ = false;

  abCtl_->Halt(channelMotion_);
  abCtl_->Clear(channelMotion_);

  if (controlled_stop) {
    cmCtl_->motion->Stop(&axisSel_);
  }

  RCLCPP_INFO(
    this->get_logger(), "Stream ended: %s%s", reason,
    controlled_stop ? " (axes stopped)" : "");
}

bool ServoStreamController::recordSetpoint(const std::vector<double> & target, int period_cycles)
{
  const size_t count = jointAxes_.size();
  const double period_s = period_cycles / 1000.0;   // 1 kHz cycle

  intpl_.axisCount = count;
  for (size_t i = 0; i < count; ++i) {
    const double step = std::fabs(target[i] - lastTarget_[i]);
    const double velocity = step / period_s;
    const double accel = velocity / (accelRatio_ * period_s);

    intpl_.axis[i] = static_cast<int>(jointAxes_[i]);
    intpl_.target[i] = target[i];
    intpl_.maxVelocity[i] = velocity;
    intpl_.maxAcc[i] = accel;
    intpl_.maxDec[i] = accel;
  }

  intpl_.profile.type = ProfileType::Trapezoidal;
  intpl_.profile.velocity = 0.0;      // per-axis limits govern under AxisLimit calc mode
  intpl_.profile.acc = 0.0;
  intpl_.profile.dec = 0.0;
  intpl_.profile.startingVelocity = 0.0;   // override inherits the current velocity
  intpl_.profile.endVelocity = 0.0;

  if (!check(abRec_->StartRecordBufferChannel(channelMotion_), "StartRecordBufferChannel")) {
    return false;
  }
  const int motion_err = cmRec_->motion->StartLinearIntplPos(&intpl_);
  const int sleep_err =
    (motion_err == ErrorCode::None) ? abRec_->USleep(period_cycles * 1000) : ErrorCode::None;
  abRec_->EndRecordBufferChannel();

  if (motion_err != ErrorCode::None || sleep_err != ErrorCode::None) {
    const int e = (motion_err != ErrorCode::None) ? motion_err : sleep_err;
    wmxRec_.ErrorToString(e, errString_, sizeof(errString_));
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Failed to record setpoint. Error=%d (%s)", e, errString_);
    return false;
  }

  blocksAdded_ += kBlocksPerSetpoint;
  lastTarget_ = target;
  return true;
}

void ServoStreamController::pumpLoop()
{
  const auto starvation = std::chrono::milliseconds(starvationTimeoutMs_);
  auto last_sent = std::chrono::steady_clock::now();

  while (running_.load() && rclcpp::ok()) {
    std::vector<double> target;
    bool have_target = false;

    {
      std::unique_lock<std::mutex> lock(ringMtx_);
      if (ring_.empty()) {
        ringCv_.wait_for(lock, kPumpIdleWait, [this] {
          return !ring_.empty() || !running_.load();
        });
      }
      if (!ring_.empty()) {
        target = std::move(ring_.front());
        ring_.pop_front();
        have_target = true;
      }
    }

    if (!running_.load()) {
      break;
    }

    if (!have_target) {
      // Nothing to send. If we were streaming and the gap has grown past the
      // starvation window, bring the arm to a controlled rest rather than
      // letting the buffer simply run dry.
      if (streaming_ && std::chrono::steady_clock::now() - last_sent > starvation) {
        RCLCPP_WARN(
          this->get_logger(), "Servo stream starved (>%d ms); stopping axes",
          starvationTimeoutMs_);
        endStream("stream starved", true);
      }
      continue;
    }

    if (inExecution_.load()) {
      continue;   // move_group owns the arm; discard
    }

    if (!streaming_ && !beginStream()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // GetStatus is cyclic (one cycle stale), which is fine for a slow pacing
    // loop but means freeSize must be read with margin, not exactly.
    ApiBufferStatus st;
    if (abCtl_->GetStatus(channelMotion_, &st) != ErrorCode::None) {
      continue;
    }

    if (st.state == ApiBufferState::Stop) {
      // stopOnError tripped, or the watch fired.
      RCLCPP_ERROR(
        this->get_logger(),
        "Motion channel stopped unexpectedly (errors=%d watchError=%d axis=%d code=%d)",
        st.errorCount, st.watchError ? 1 : 0, st.watchErrorAxis, st.watchErrorCode);
      endStream("channel stopped", true);
      continue;
    }

    const int depth = st.remainingBlockCount / kBlocksPerSetpoint;
    observedDepth_.store(depth);

    if (depth > 4 * targetQueueDepth_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "API buffer backlog: %d setpoints in flight (target %d)", depth, targetQueueDepth_);
    }

    if (recordSetpoint(target, pacePeriodCycles(depth))) {
      last_sent = std::chrono::steady_clock::now();
    }
  }

  if (streaming_) {
    endStream("shutting down", true);
  }
}

// Surfaces the error log. The log holds only 10 entries with index 0 newest, so
// errorCount (not the array length) is what reveals how many were missed.
void ServoStreamController::pollStatus()
{
  if (!initialized_) {
    return;
  }

  std_msgs::msg::Int32 depth_msg;
  depth_msg.data = observedDepth_.load();
  depthPub_->publish(depth_msg);

  ApiBufferStatus st;
  if (abCtl_->GetStatus(channelMotion_, &st) != ErrorCode::None) {
    return;
  }

  if (st.errorCount == lastErrorCount_) {
    return;
  }

  const int missed = st.errorCount - lastErrorCount_;
  lastErrorCount_ = st.errorCount;

  const int reportable = std::min(missed, static_cast<int>(wmx3Api::constants::maxApiBufferErrorLog));
  for (int i = 0; i < reportable; ++i) {
    ApiBuffer::ErrorToString(st.errorLog[i].errorCode, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "API buffer error at block %d (setpoint ~%lld): %d (%s)",
      st.errorLog[i].execBlockNumber,
      static_cast<long long>(st.errorLog[i].execBlockNumber) / kBlocksPerSetpoint,
      st.errorLog[i].errorCode, errString_);
  }
  if (missed > reportable) {
    RCLCPP_ERROR(
      this->get_logger(), "%d further errors overflowed the log between polls",
      missed - reportable);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoStreamController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
