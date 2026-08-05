// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_r2_control/wmx_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#ifndef WMX3_SDK_PATH
#define WMX3_SDK_PATH "/opt/wmx3/"
#endif

namespace wmx_r2_control
{

using wmx3Api::CoreMotion;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;

const char * modeName(JointMode mode)
{
  switch (mode) {
    case JointMode::Position:
      return "position";
    case JointMode::Velocity:
      return "velocity";
    default:
      return "state-only";
  }
}
}  // namespace

std::string WmxSystemHardware::getHwParam(
  const std::string & key, const std::string & def) const
{
  auto it = info_.hardware_parameters.find(key);
  return (it != info_.hardware_parameters.end()) ? it->second : def;
}

#if WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS
hardware_interface::CallbackReturn WmxSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return initImpl();
}
#else
hardware_interface::CallbackReturn WmxSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return initImpl();
}
#endif

hardware_interface::CallbackReturn WmxSystemHardware::initImpl()
{
  sdk_path_ = getHwParam("wmx_sdk_path", WMX3_SDK_PATH);
  device_name_ = getHwParam("device_name", "wmx_r2_control");
  wmx_param_file_ = getHwParam("wmx_param_file", "");
  acc_time_ms_ = std::stod(getHwParam("acc_time_ms", "1.0"));
  dec_time_ms_ = std::stod(getHwParam("dec_time_ms", "1.0"));
  max_device_retries_ = std::stoi(getHwParam("max_device_retries", "30"));
  auto_servo_on_ = (getHwParam("auto_servo_on", "true") == "true");
  cyclic_buffer_horizon_s_ = std::stod(getHwParam("cyclic_buffer_horizon_s", "5.0"));
  cyclic_target_periods_ = std::stoi(getHwParam("cyclic_target_periods", "2"));
  cyclic_max_acc_ = std::stod(getHwParam("cyclic_max_acc", "0.0"));
  cyclic_push_period_ms_ = std::stod(getHwParam("cyclic_push_period_ms", "10.0"));

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & j : info_.joints) {
    WmxJoint joint;
    joint.name = j.name;

    auto axis_it = j.parameters.find("axis");
    if (axis_it == j.parameters.end()) {
      RCLCPP_FATAL(
        logger_, "Joint '%s' is missing required <param name=\"axis\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint.axis = std::stoi(axis_it->second);

    const auto & cmd_ifs = j.command_interfaces;
    if (cmd_ifs.empty()) {
      joint.mode = JointMode::StateOnly;
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_POSITION) {
      joint.mode = JointMode::Position;
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_VELOCITY) {
      joint.mode = JointMode::Velocity;
    } else {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must declare either no command_interface (state-only) or a "
        "single 'position' or 'velocity' command_interface", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Joint '%s' -> WMX axis %d, mode=%s",
      joint.name.c_str(), joint.axis, modeName(joint.mode));

    joints_.push_back(joint);
  }

  bool has_velocity_joint = false;
  position_joint_idx_.clear();
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].mode == JointMode::Position) {
      position_joint_idx_.push_back(i);
    } else if (joints_[i].mode == JointMode::Velocity) {
      has_velocity_joint = true;
    }
  }

  // Position joints are driven by the cyclic buffer, which puts their axes into
  // DirectControl; velocity joints are driven by StartVel, which needs the axis
  // idle to accept a new profile. One ros2_control block cannot do both.
  if (!position_joint_idx_.empty() && has_velocity_joint) {
    RCLCPP_FATAL(
      logger_,
      "Mixing 'position' and 'velocity' command interfaces in a single "
      "<ros2_control> block is not supported: position joints stream through the "
      "cyclic buffer (axes enter DirectControl) while velocity joints use "
      "StartVel. Declare them in separate <ros2_control> blocks.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // write() runs in the control loop and must not allocate.
  q_cmd_scratch_.assign(position_joint_idx_.size(), 0.0);
  stream_err_.reserve(256);

  RCLCPP_INFO(
    logger_,
    "WmxSystemHardware initialised: %zu joints (%zu position-streamed), "
    "sdk_path=%s, param_file=%s",
    joints_.size(), position_joint_idx_.size(), sdk_path_.c_str(),
    wmx_param_file_.empty() ? "(none)" : wmx_param_file_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool WmxSystemHardware::attachDevice()
{
  const unsigned int timeout = 10000;
  int err = ErrorCode::None;

  for (int attempt = 1; attempt <= max_device_retries_; ++attempt) {
    err = wmx_.CreateDevice(sdk_path_.c_str(), DeviceType::DeviceTypeNormal, timeout);
    if (err == ErrorCode::None) {
      break;
    }
    wmx_.ErrorToString(err, err_str_, sizeof(err_str_));
    if (err == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(
        logger_, "WMX device lock busy, retrying in 1s... (%d/%d)", attempt, max_device_retries_);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_FATAL(logger_, "Failed to attach to WMX device. Error=%d (%s)", err, err_str_);
      return false;
    }
  }

  if (err != ErrorCode::None) {
    RCLCPP_FATAL(
      logger_, "WMX device lock busy after %d retries, giving up", max_device_retries_);
    return false;
  }

  wmx_.SetDeviceName(device_name_.c_str());
  device_open_ = true;
  RCLCPP_INFO(logger_, "Attached to WMX3 device as '%s'", device_name_.c_str());
  return true;
}

void WmxSystemHardware::closeDevice()
{
  if (!device_open_) {
    return;
  }
  int err = wmx_.CloseDevice();
  if (err != ErrorCode::None) {
    wmx_.ErrorToString(err, err_str_, sizeof(err_str_));
    RCLCPP_ERROR(logger_, "Failed to close WMX device. Error=%d (%s)", err, err_str_);
  } else {
    RCLCPP_INFO(logger_, "WMX device closed");
  }
  device_open_ = false;
}

bool WmxSystemHardware::engineCommunicating()
{
  cm_->GetStatus(&cm_status_);
  return cm_status_.engineState == EngineState::T::Communicating;
}

bool WmxSystemHardware::seedPositionCommands()
{
  for (const size_t i : position_joint_idx_) {
    auto & joint = joints_[i];
    // The *command* position, not the feedback position: it is what the cyclic
    // buffer's primed hold is built from (see WmxCyclicStream::holdCommands), so
    // seeding from feedback here would make the first streamed command differ
    // from the hold by the servo following error -- a step on every axis at once.
    double commanded = 0.0;
    const int err = cm_->axisControl->GetPosCommand(joint.axis, &commanded);
    if (err != ErrorCode::None) {
      cm_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR(
        logger_, "GetPosCommand failed on axis %d. Error=%d (%s)",
        joint.axis, err, err_str_);
      return false;
    }
    joint.cmd = commanded;
  }
  return true;
}

bool WmxSystemHardware::openAndStartStream()
{
  WmxCyclicStream::Config cfg;
  cfg.axes.reserve(position_joint_idx_.size());
  for (const size_t i : position_joint_idx_) {
    cfg.axes.push_back(joints_[i].axis);
  }
  cfg.buffer_horizon_s = cyclic_buffer_horizon_s_;
  cfg.target_periods = static_cast<unsigned int>(std::max(1, cyclic_target_periods_));
  cfg.max_acc = cyclic_max_acc_;

  if (!stream_.open(&wmx_, cm_.get(), cfg, cyclic_push_period_ms_, &stream_err_)) {
    RCLCPP_ERROR(logger_, "Cyclic buffer open failed: %s", stream_err_.c_str());
    return false;
  }
  if (!stream_.start(&stream_err_)) {
    RCLCPP_ERROR(logger_, "Cyclic buffer start failed: %s", stream_err_.c_str());
    stream_.close();
    return false;
  }

  const auto & timing = stream_.timing();
  RCLCPP_INFO(
    logger_,
    "Cyclic buffer streaming %zu axes: engine cycle %.3f ms, seed push period "
    "%.3f ms -> %u cycles per command, depth setpoint %u commands, maxAcc %.3f",
    cfg.axes.size(), timing.cycle_ms, timing.push_period_ms,
    timing.nominalInterval(), stream_.targetDepth(), cfg.max_acc);
  return true;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!attachDevice()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cm_ = std::make_unique<CoreMotion>(&wmx_);

  if (!wmx_param_file_.empty()) {
    int err = cm_->config->ImportAndSetAll(const_cast<char *>(wmx_param_file_.c_str()));
    if (err != ErrorCode::None) {
      cm_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR(logger_, "Failed to import WMX params. Error=%d (%s)", err, err_str_);
    } else {
      RCLCPP_INFO(logger_, "Applied WMX params from %s", wmx_param_file_.c_str());
    }
  }

  cm_->GetStatus(&cm_status_);
  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
    // 0 means "stop" for a velocity joint but "go to 0 rad" for a position one,
    // so a position joint is seeded to where it already is. Nothing reads the
    // command interfaces while INACTIVE; on_activate re-seeds from the WMX
    // command position, which is the value the stream's hold is built from.
    joint.cmd = (joint.mode == JointMode::Position) ? joint.pos_state : 0.0;
    joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // wmx_engine_node is launched alongside the controller_manager and takes ~1s
  // to bring EtherCAT to Communicating. Wait for it (bounded) rather than
  // failing activation outright, which aborts ros2_control_node.
  bool communicating = false;
  for (int attempt = 1; attempt <= max_device_retries_; ++attempt) {
    if (engineCommunicating()) {
      communicating = true;
      break;
    }
    RCLCPP_WARN(
      logger_, "WMX engine not Communicating yet, waiting... (%d/%d)",
      attempt, max_device_retries_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (!communicating) {
    RCLCPP_ERROR(
      logger_,
      "WMX engine is not Communicating after %d s. Start wmx_engine_node and "
      "EtherCAT communication first.", max_device_retries_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Clear amp alarms and enable the servos (StartVel/CyclicBuffer require
  // servo-on). Disable via the 'auto_servo_on' hardware param if servos are
  // managed elsewhere.
  if (auto_servo_on_) {
    for (auto & joint : joints_) {
      cm_->axisControl->ClearAmpAlarm(joint.axis);
    }
    for (auto & joint : joints_) {
      int err = cm_->axisControl->SetServoOn(joint.axis, 1, 2000);
      if (err != ErrorCode::None) {
        cm_->ErrorToString(err, err_str_, sizeof(err_str_));
        RCLCPP_ERROR(
          logger_, "Failed to servo-on axis %d. Error=%d (%s)", joint.axis, err, err_str_);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(logger_, "Servos enabled on all axes");
  }

  cm_->GetStatus(&cm_status_);
  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
    joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
    if (joint.mode != JointMode::Position) {
      joint.cmd = 0.0;   // velocity joints: 0 = stop
    }
  }

  // Position joints: seed the command from where the axis is, then open and
  // start the stream. Both must happen after servo-on above -- the cyclic buffer
  // reports ServoOff otherwise -- and seeding must precede start() so the first
  // write() to land, before any controller has claimed the interfaces, is a hold
  // rather than a six-axis move to 0 rad.
  if (!position_joint_idx_.empty()) {
    if (!seedPositionCommands() || !openAndStartStream()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    stream_started_ = true;
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Abort before servoing off: the axes are in DirectControl until the buffer is
  // aborted, and dropping the servo out from under a running stream leaves it in
  // the latching ServoOff state instead of cleanly Stopped.
  if (stream_started_) {
    stream_.abort();
    stream_started_ = false;
  }
  for (auto & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      startVelocity(joint, 0.0);
    }
  }
  if (auto_servo_on_) {
    for (auto & joint : joints_) {
      cm_->axisControl->SetServoOn(joint.axis, 0, 2000);
    }
  }
  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Deallocates on the engine; must happen while the device is still open.
  stream_.close();
  stream_started_ = false;
  closeDevice();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WmxSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & joint : joints_) {
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.pos_state);
    state_interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel_state);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WmxSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & joint : joints_) {
    if (joint.mode == JointMode::Position) {
      command_interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.cmd);
    } else if (joint.mode == JointMode::Velocity) {
      command_interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd);
    }
  }
  return command_interfaces;
}

hardware_interface::return_type WmxSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int err = cm_->GetStatus(&cm_status_);
  if (err != ErrorCode::None) {
    cm_->ErrorToString(err, err_str_, sizeof(err_str_));
    RCLCPP_ERROR_THROTTLE(
      logger_, clock_, 1000, "GetStatus failed. Error=%d (%s)", err, err_str_);
    return hardware_interface::return_type::ERROR;
  }

  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
  }
  return hardware_interface::return_type::OK;
}

void WmxSystemHardware::startVelocity(const WmxJoint & joint, double omega)
{
  Velocity::VelCommand cmd;
  cmd.axis = joint.axis;
  cmd.profile.velocity = omega;
  cmd.profile.type = ProfileType::T::TimeAccTrapezoidal;
  cmd.profile.accTimeMilliseconds = acc_time_ms_;
  cmd.profile.decTimeMilliseconds = dec_time_ms_;

  int err = cm_->velocity->StartVel(&cmd);
  if (err != ErrorCode::None) {
    cm_->ErrorToString(err, err_str_, sizeof(err_str_));
    RCLCPP_ERROR_THROTTLE(
      logger_, clock_, 1000,
      "StartVel failed on axis %d. Error=%d (%s)", joint.axis, err, err_str_);
  }
}

hardware_interface::return_type WmxSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (cm_status_.engineState != EngineState::T::Communicating) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "WMX engine not Communicating; skipping write()");
    return hardware_interface::return_type::OK;
  }

  if (stream_started_) {
    // The commanded interval is a ratio of the loop period, so track the real
    // one rather than trusting the configured update_rate.
    stream_.observePushPeriod(period.seconds() * 1000.0);

    for (size_t k = 0; k < position_joint_idx_.size(); ++k) {
      q_cmd_scratch_[k] = joints_[position_joint_idx_[k]].cmd;
    }

    // push() polls the buffer itself, so there is deliberately no refresh() in
    // read(): that would double the engine round-trips per control cycle.
    if (!stream_.push(q_cmd_scratch_, &stream_err_)) {
      if (stream_.faulted()) {
        // Latching: another push cannot recover it. Surfacing ERROR lets the
        // controller manager deactivate rather than spin on a dead stream.
        RCLCPP_ERROR_THROTTLE(
          logger_, clock_, 1000,
          "Cyclic buffer faulted (%s): %s", stream_.stateName(), stream_err_.c_str());
        return hardware_interface::return_type::ERROR;
      }
      // Transient: a full buffer or a non-finite command. The axis holds at its
      // last commanded position until a push lands again.
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "Cyclic push rejected (state=%s depth=%u/%u rejected=%u): %s",
        stream_.stateName(), stream_.depth(), stream_.targetDepth(),
        stream_.rejected(), stream_err_.c_str());
    }
  }

  for (auto & joint : joints_) {
    if (joint.mode != JointMode::Velocity) {
      continue;
    }
    // Skip (don't spam StartVel errors) while the servo is off or alarmed.
    const auto & axis_status = cm_status_.axesStatus[joint.axis];
    if (!axis_status.servoOn || axis_status.ampAlarm) {
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 2000,
        "Axis %d not ready (servoOn=%d, ampAlarm=%d); skipping velocity command",
        joint.axis, axis_status.servoOn, axis_status.ampAlarm);
      joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    if (std::isnan(joint.last_cmd) || std::fabs(joint.cmd - joint.last_cmd) > kCmdEpsilon) {
      startVelocity(joint, joint.cmd);
      joint.last_cmd = joint.cmd;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace wmx_r2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_r2_control::WmxSystemHardware, hardware_interface::SystemInterface)
