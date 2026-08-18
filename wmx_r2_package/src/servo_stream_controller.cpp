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
// T is in microseconds and is NOT quantised to the engine cycle: api_buffer_probe
// measured USleep tracking sub-millisecond requests to within ~0.3%. Measured
// cost per setpoint is ~396 bytes (320 motion + 76 sleep), so a 1 MB channel
// holds ~2650 setpoints, over a minute of stream at Servo's 40 Hz.
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
#include <cstdio>
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

// Backoff after the motion channel stops on a fault. Without this the pump
// re-Executes on the very next setpoint, so a PERSISTENT fault (servo off, for
// one) becomes a 40 Hz Clear/Execute/Stop storm against the engine that never
// gives up. Doubles per consecutive fault, capped, and resets the moment a
// setpoint records successfully — so a transient trip still recovers promptly.
constexpr auto kFaultBackoffBase = std::chrono::milliseconds(500);
constexpr auto kFaultBackoffMax = std::chrono::milliseconds(5000);
constexpr int kFaultBackoffMaxShift = 4;

// How long a stream must survive before the fault history is forgiven. A
// recorded block is NOT evidence of health: recordSetpoint only writes into the
// buffer and succeeds even when every block is failing at execution time. Only
// elapsed time with the channel still Active proves the engine is consuming.
constexpr auto kHealthyStreamTime = std::chrono::seconds(1);

// A Stop state read within this window of Execute is not trusted. GetStatus is
// cyclic, so the first poll after Execute can still be reporting the Stop that
// the preceding Halt/Clear left behind. Long enough to cover several engine
// cycles and the pump's own 25 ms cadence, short enough that a genuine
// error-free Stop (an external Halt, say) is still caught promptly.
constexpr auto kStatusSettleTime = std::chrono::milliseconds(50);
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
  int nominalPeriodUs_ = 25000;
  int clampLoUs_ = 20000;
  int clampHiUs_ = 30000;
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
  long long lastErrorCount_ = 0;             // ApiBufferStatus::errorCount is long long

  int consecutiveFaults_ = 0;                       // channel stops without progress
  std::chrono::steady_clock::time_point faultUntil_{};   // suppress restarts until
  std::chrono::steady_clock::time_point streamStartedAt_{};

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
  bool createBuffer(int channel, int size_mb, const char * what);
  void teardownPartialInit();
  bool check(int err, const char * what);
  void blockErrorToString(int code, char * buf, size_t size);
  bool setupBuffers();
  void recordEstopRoutine();

  void pumpLoop();
  bool beginStream();
  void endStream(const char * reason, bool controlled_stop);
  bool recordSetpoint(const std::vector<double> & target, int period_us);
  int pacePeriodUs(int depth) const;
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
      // Bounded. The no-timeout Wait can block indefinitely if an axis never
      // reaches rest (servo off, alarm), and a shutdown that hangs here never
      // reaches FreeApiBuffer below — leaving the channels allocated in the
      // engine so the NEXT run fails with "Queue ID is already used".
      cmCtl_->motion->Wait(&axisSel_, 2000);
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
  this->declare_parameter<int>("nominal_period_us", 25000);
  this->declare_parameter<int>("period_clamp_lo_us", 20000);
  this->declare_parameter<int>("period_clamp_hi_us", 30000);
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
  this->get_parameter("nominal_period_us", nominalPeriodUs_);
  this->get_parameter("period_clamp_lo_us", clampLoUs_);
  this->get_parameter("period_clamp_hi_us", clampHiUs_);
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
    targetQueueDepth_, targetQueueDepth_ * nominalPeriodUs_ / 1000);
  RCLCPP_INFO(
    this->get_logger(), "period: %d us, clamp [%d, %d] us, Kp %.2f (%.0f us per setpoint of error)",
    nominalPeriodUs_, clampLoUs_, clampHiUs_, pacingKp_, pacingKp_ * 1000.0);
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

// A block error comes from the recorded command itself (StartLinearIntplPos),
// so it carries a CoreMotion error code, NOT an ApiBuffer one. Decoding it with
// ApiBuffer::ErrorToString yields an empty string — on the bench, code 65556
// printed as "()". Try CoreMotion first, fall back to ApiBuffer, then admit
// defeat rather than printing an empty pair of brackets.
void ServoStreamController::blockErrorToString(int code, char * buf, size_t size)
{
  const auto n = static_cast<unsigned int>(size);
  buf[0] = '\0';
  CoreMotion::ErrorToString(code, buf, n);
  if (buf[0] == '\0') {
    ApiBuffer::ErrorToString(code, buf, n);
  }
  if (buf[0] == '\0') {
    snprintf(buf, size, "unrecognised code");
  }
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

// API buffers are engine-side and outlive the client process, so a controller
// that died without freeing them leaves the channel allocated and the next run
// gets "Queue ID is already used". Reclaim the channel instead of making the
// operator restart the whole engine.
bool ServoStreamController::createBuffer(int channel, int size_mb, const char * what)
{
  int err = abCtl_->CreateApiBuffer(channel, size_mb, SizeUnit::Megabyte);
  if (err == ErrorCode::None) {
    return true;
  }

  ApiBuffer::ErrorToString(err, errString_, sizeof(errString_));
  RCLCPP_WARN(
    this->get_logger(),
    "%s failed (Error=%d %s); channel %d appears stale, reclaiming",
    what, err, errString_, channel);

  abCtl_->Halt(channel);
  abCtl_->Clear(channel);
  abCtl_->FreeApiBuffer(channel);

  return check(abCtl_->CreateApiBuffer(channel, size_mb, SizeUnit::Megabyte), what);
}

// Undo whatever runInitSequence managed before failing. Without this, a failure
// part-way through leaks engine-side buffers AND device handles, and the 1 Hz
// engine-ready timer then retries forever — leaking two more devices a second.
void ServoStreamController::teardownPartialInit()
{
  if (abCtl_) {
    // Unconditional: bufferCreated_ is only set once BOTH channels exist, so it
    // cannot tell us whether the first one was already allocated. Freeing an
    // unallocated channel is harmless.
    abCtl_->Halt(channelMotion_);
    abCtl_->Clear(channelMotion_);
    abCtl_->FreeApiBuffer(channelMotion_);
    if (channelMotion_ != channelEstop_) {
      abCtl_->FreeApiBuffer(channelEstop_);
    }
  }
  bufferCreated_ = false;

  abRec_.reset();
  abCtl_.reset();
  cmRec_.reset();
  cmCtl_.reset();

  if (recAttached_) {
    wmxRec_.CloseDevice();
    recAttached_ = false;
  }
  if (ctlAttached_) {
    wmxCtl_.CloseDevice();
    ctlAttached_ = false;
  }
}

bool ServoStreamController::setupBuffers()
{
  if (!createBuffer(channelMotion_, bufferSizeMb_, "CreateApiBuffer(motion)")) {
    return false;
  }
  if (channelMotion_ != channelEstop_) {
    if (!createBuffer(channelEstop_, 1, "CreateApiBuffer(estop)")) {
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
  // Flags are set per-device, not after both: if the second attach fails the
  // first device still needs closing, or the retry leaks it.
  if (!attachDevice(&wmxRec_, "servo_stream_rec")) {
    teardownPartialInit();
    initializing_ = false;
    return;
  }
  recAttached_ = true;

  if (!attachDevice(&wmxCtl_, "servo_stream_ctl")) {
    teardownPartialInit();
    initializing_ = false;
    return;
  }
  ctlAttached_ = true;

  cmRec_ = std::make_unique<CoreMotion>(&wmxRec_);
  cmCtl_ = std::make_unique<CoreMotion>(&wmxCtl_);
  abRec_ = std::make_unique<ApiBuffer>(&wmxRec_);
  abCtl_ = std::make_unique<ApiBuffer>(&wmxCtl_);

  if (!setupBuffers()) {
    teardownPartialInit();
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

  std::vector<double> target(count, 0.0);
  std::vector<bool> filled(count, false);

  if (msg->joint_names.empty()) {
    // Unnamed: positional, assumed already in joint_axes order.
    if (pt.positions.size() < count) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: %zu positions for %zu axes", pt.positions.size(), count);
      return;
    }
    for (size_t i = 0; i < count; ++i) {
      target[i] = pt.positions[i];
      filled[i] = true;
    }
  } else {
    // Iterate the MESSAGE's joints and pick out the ones this node drives.
    // Doing it this way round lets a subset configuration (e.g. joint_axes: [0]
    // for single-axis bring-up) consume Servo's full six-joint output, and is
    // robust to a publisher that orders joints differently than we do.
    const size_t n = std::min(msg->joint_names.size(), pt.positions.size());
    for (size_t i = 0; i < n; ++i) {
      const auto it = axisByName_.find(msg->joint_names[i]);
      if (it == axisByName_.end()) {
        continue;                       // a joint we are not responsible for
      }
      const auto pos = std::find(jointAxes_.begin(), jointAxes_.end(), it->second);
      if (pos == jointAxes_.end()) {
        continue;
      }
      const size_t slot = static_cast<size_t>(std::distance(jointAxes_.begin(), pos));
      target[slot] = pt.positions[i];
      filled[slot] = true;
    }
  }

  // Every axis we drive must be commanded: a partial setpoint would hold some
  // joints and move others, bending the path Servo computed.
  for (size_t i = 0; i < count; ++i) {
    if (!filled[i]) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped trajectory: no command for axis %d (check joint_name vs publisher)",
        static_cast<int>(jointAxes_[i]));
      return;
    }
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

// Adaptive playout: trim the sleep to hold the queue at target depth.
//
// Worked in MICROSECONDS, not engine cycles. api_buffer_probe measured USleep
// honouring sub-millisecond requests to within ~0.3% (1333 us -> 1341 us), so
// there is no 1 kHz quantisation to round to. Rounding to whole cycles used to
// swallow the correction entirely: at Kp 0.15 the depth error had to exceed
// 3.34 setpoints before lround moved the period at all, which left the loop
// unable to ever speed up — depth cannot go far enough below target to trip it.
//
// pacing_kp keeps its original meaning: cycles of adjustment per setpoint of
// depth error, hence the x1000 to microseconds.
//
// SIGN: the period is what the ENGINE spends per setpoint, while the pump feeds
// at whatever rate ROS delivers. So depth grows when the period exceeds the
// arrival interval and shrinks when it is shorter — meaning a depth BELOW target
// calls for a LONGER period, not a shorter one. The error term is therefore
// (target - depth). Getting this backwards makes the loop positive-feedback:
// it pins depth at 0 (nothing ever buffered, so no jitter absorption at all)
// and lets a backlog run away instead of draining it.
int ServoStreamController::pacePeriodUs(int depth) const
{
  const double adjust = pacingKp_ * static_cast<double>(targetQueueDepth_ - depth) * 1000.0;
  const int us = static_cast<int>(std::lround(nominalPeriodUs_ + adjust));
  return std::max(clampLoUs_, std::min(clampHiUs_, us));
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
  streamStartedAt_ = std::chrono::steady_clock::now();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Stream started (channel %d Active)", channelMotion_);
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

  // The pump only writes observedDepth_ on its success path, so without this
  // the depth topic keeps republishing the last in-flight count for as long as
  // the stream stays down -- reporting a healthy backlog at exactly the moment
  // the buffer is empty and the axes have stopped. Nothing is in flight past
  // the Clear below, so say so.
  observedDepth_.store(0);

  abCtl_->Halt(channelMotion_);
  abCtl_->Clear(channelMotion_);

  if (controlled_stop) {
    cmCtl_->motion->Stop(&axisSel_);
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Stream ended: %s%s", reason, controlled_stop ? " (axes stopped)" : "");
}

bool ServoStreamController::recordSetpoint(const std::vector<double> & target, int period_us)
{
  const size_t count = jointAxes_.size();
  const double period_s = period_us / 1000000.0;

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
    (motion_err == ErrorCode::None) ? abRec_->USleep(period_us) : ErrorCode::None;
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

    // Holding off after a fault. Discard setpoints rather than restarting the
    // channel into a condition that is still present.
    if (std::chrono::steady_clock::now() < faultUntil_) {
      continue;
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
      // A Stop with nothing recorded against it is not a fault: since GetStatus
      // is one cycle stale, the first poll after Execute reads back the Stop
      // that endStream's Halt left behind. Faulting on that tears down the
      // stream 30 us after starting it, Halt/Clear/Stops the axes on nothing,
      // and escalates the backoff -- so the next attempt is delayed on evidence
      // that was never there. Let the status catch up instead.
      const bool recorded_error = st.watchError || st.errorCount > 0;
      if (!recorded_error &&
        std::chrono::steady_clock::now() - streamStartedAt_ < kStatusSettleTime)
      {
        continue;
      }

      ++consecutiveFaults_;
      const auto backoff = std::min(
        kFaultBackoffMax,
        kFaultBackoffBase * (1 << std::min(consecutiveFaults_ - 1, kFaultBackoffMaxShift)));
      faultUntil_ = std::chrono::steady_clock::now() + backoff;

      // watchErrorCode only means anything when watchError is set, and
      // "stopOnError" was a guess at the cause rather than a reading of it.
      // Name what the device recorded: the watch, the block that failed (error
      // log index 0 is newest), or nothing at all.
      char cause[320];
      if (st.watchError) {
        wmxCtl_.ErrorToString(st.watchErrorCode, errString_, sizeof(errString_));
        snprintf(
          cause, sizeof(cause), "watch fired on axis %d, code %d (%s)",
          st.watchErrorAxis, st.watchErrorCode, errString_);
      } else if (st.errorCount > 0) {
        wmxCtl_.ErrorToString(st.errorLog[0].errorCode, errString_, sizeof(errString_));
        snprintf(
          cause, sizeof(cause), "block %lld failed, code %d (%s)",
          static_cast<long long>(st.errorLog[0].execBlockNumber),
          st.errorLog[0].errorCode, errString_);
      } else {
        snprintf(
          cause, sizeof(cause), "no error recorded, channel never left Stop after Execute");
      }
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Motion channel stopped (%s; errors=%lld); fault %d, holding off %ld ms",
        cause, st.errorCount, consecutiveFaults_, static_cast<long>(backoff.count()));

      endStream("channel stopped", true);
      continue;
    }

    const int depth = static_cast<int>(st.remainingBlockCount / kBlocksPerSetpoint);
    observedDepth_.store(depth);

    if (depth > 4 * targetQueueDepth_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "API buffer backlog: %d setpoints in flight (target %d)", depth, targetQueueDepth_);
    }

    if (recordSetpoint(target, pacePeriodUs(depth))) {
      last_sent = std::chrono::steady_clock::now();
      // Forgive the fault history only once the stream has actually survived a
      // while. Resetting on a successful record instead would pin the backoff
      // at its first step forever, since recording succeeds regardless of
      // whether the engine can execute what was recorded.
      if (last_sent - streamStartedAt_ > kHealthyStreamTime) {
        consecutiveFaults_ = 0;
      }
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

  const long long missed = st.errorCount - lastErrorCount_;
  lastErrorCount_ = st.errorCount;

  const int reportable = static_cast<int>(
    std::min<long long>(missed, wmx3Api::constants::maxApiBufferErrorLog));
  for (int i = 0; i < reportable; ++i) {
    blockErrorToString(st.errorLog[i].errorCode, errString_, sizeof(errString_));
    RCLCPP_ERROR(
      this->get_logger(),
      "API buffer error at block %lld (setpoint ~%lld): %d (%s)",
      st.errorLog[i].execBlockNumber,
      st.errorLog[i].execBlockNumber / kBlocksPerSetpoint,
      st.errorLog[i].errorCode, errString_);
  }
  if (missed > reportable) {
    RCLCPP_ERROR(
      this->get_logger(), "%lld further errors overflowed the log between polls",
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
