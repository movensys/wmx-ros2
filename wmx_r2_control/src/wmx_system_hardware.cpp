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
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;
}  // namespace

WmxSystemHardwareApi::WmxSystemHardwareApi(
  const rclcpp::Logger & logger, const Config & config)
: logger_(logger), config_(config)
{
}

WmxSystemHardwareApi::~WmxSystemHardwareApi()
{
  if (cm_) {
    releaseDevice();
  }
}

std::string WmxSystemHardwareApi::errorText(int err)
{
  char errString[256] = {};
  wmx3Lib_.ErrorToString(err, errString, sizeof(errString));
  return errString;
}

int WmxSystemHardwareApi::attachDevice(std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (cm_) {
    message = "Already attached to the WMX3 device";
    return ErrorCode::None;
  }

  int err = ErrorCode::None;

  for (int attempt = 1; attempt <= config_.maxDeviceRetries; ++attempt) {
    err = wmx3Lib_.CreateDevice(
      config_.sdkPath.c_str(), DeviceType::DeviceTypeNormal, timeout_);
    if (err == ErrorCode::None) {
      break;
    }

    if (err != ErrorCode::StartProcessLockError) {
      message = "Failed to attach to WMX device. Error=" + std::to_string(err) +
        " (" + errorText(err) + ")";
      RCLCPP_FATAL(logger_, "%s", message.c_str());
      return err;
    }

    RCLCPP_WARN(
      logger_, "WMX device lock busy, retrying in 1s... (%d/%d)",
      attempt, config_.maxDeviceRetries);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (err != ErrorCode::None) {
    message = "WMX device lock busy after " + std::to_string(config_.maxDeviceRetries) +
      " retries, giving up";
    RCLCPP_FATAL(logger_, "%s", message.c_str());
    return err;
  }

  wmx3Lib_.SetDeviceName(config_.deviceName.c_str());
  cm_ = std::make_unique<CoreMotion>(&wmx3Lib_);

  message = "Attached to WMX3 device as '" + config_.deviceName + "'";
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

void WmxSystemHardwareApi::releaseDevice()
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  cm_.reset();

  const int err = wmx3Lib_.CloseDevice();
  if (err != ErrorCode::None) {
    RCLCPP_ERROR(
      logger_, "Failed to close WMX device. Error=%d (%s)", err, errorText(err).c_str());
  } else {
    RCLCPP_INFO(logger_, "WMX device closed");
  }
}

int WmxSystemHardwareApi::importAndSetAll(const std::string & path, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot import WMX params. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  std::vector<char> pathBuffer(path.begin(), path.end());
  pathBuffer.push_back('\0');

  const int err = cm_->config->ImportAndSetAll(pathBuffer.data());
  if (err != ErrorCode::None) {
    message = "Failed to import WMX params. Error=" + std::to_string(err) +
      " (" + errorText(err) + ")";
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return err;
  }

  message = "Applied WMX params from " + path;
  RCLCPP_INFO(logger_, "%s", message.c_str());
  return ErrorCode::None;
}

int WmxSystemHardwareApi::getStatus(
  const std::vector<int> & axes, std::vector<AxisFeedback> & feedback,
  bool & communicating, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot read status. Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  CoreMotionStatus status;
  const int err = cm_->GetStatus(&status);
  if (err != ErrorCode::None) {
    message = "GetStatus failed. Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    return err;
  }

  communicating = (status.engineState == EngineState::T::Communicating);

  feedback.clear();
  feedback.reserve(axes.size());
  for (const int axis : axes) {
    const auto & raw = status.axesStatus[axis];
    AxisFeedback entry;
    entry.actualPos = raw.actualPos;
    entry.actualVelocity = raw.actualVelocity;
    entry.servoOn = raw.servoOn;
    entry.ampAlarm = raw.ampAlarm;
    feedback.push_back(entry);
  }

  return ErrorCode::None;
}

int WmxSystemHardwareApi::startVel(int axis, double omega, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot move axis " + std::to_string(axis) + ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  Velocity::VelCommand cmd;
  cmd.axis = axis;
  cmd.profile.velocity = omega;
  cmd.profile.type = ProfileType::T::TimeAccTrapezoidal;
  cmd.profile.accTimeMilliseconds = config_.accTimeMilliseconds;
  cmd.profile.decTimeMilliseconds = config_.decTimeMilliseconds;

  const int err = cm_->velocity->StartVel(&cmd);
  if (err != ErrorCode::None) {
    message = "StartVel failed on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    return err;
  }

  message = "Axis " + std::to_string(axis) + " running at " + std::to_string(omega);
  return ErrorCode::None;
}

int WmxSystemHardwareApi::setServoOn(int axis, int on, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot set servo on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->axisControl->SetServoOn(axis, on, servoOnTimeout_);
  if (err != ErrorCode::None) {
    message = "Failed to servo-" + std::string(on ? "on" : "off") + " axis " +
      std::to_string(axis) + ". Error=" + std::to_string(err) + " (" + errorText(err) + ")";
    return err;
  }

  message = "Axis " + std::to_string(axis) + " servo " + (on ? "on" : "off");
  return ErrorCode::None;
}

int WmxSystemHardwareApi::clearAmpAlarm(int axis, std::string & message)
{
  std::lock_guard<std::mutex> lock(deviceMutex_);

  if (!cm_) {
    message = "Cannot clear alarm on axis " + std::to_string(axis) +
      ". Core motion is not attached.";
    return ErrorCode::DeviceIsNull;
  }

  const int err = cm_->axisControl->ClearAmpAlarm(axis);
  if (err != ErrorCode::None) {
    message = "Failed to clear alarm on axis " + std::to_string(axis) + ". Error=" +
      std::to_string(err) + " (" + errorText(err) + ")";
    return err;
  }

  message = "Cleared alarm on axis " + std::to_string(axis);
  return ErrorCode::None;
}

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
  WmxSystemHardwareApi::Config config;
  config.sdkPath = getHwParam("wmx_sdk_path", WMX3_SDK_PATH);
  config.deviceName = getHwParam("device_name", "wmx_r2_control");
  config.accTimeMilliseconds = std::stod(getHwParam("acc_time_ms", "1.0"));
  config.decTimeMilliseconds = std::stod(getHwParam("dec_time_ms", "1.0"));
  config.maxDeviceRetries = std::stoi(getHwParam("max_device_retries", "30"));

  wmx_param_file_ = getHwParam("wmx_param_file", "");
  max_device_retries_ = config.maxDeviceRetries;
  auto_servo_on_ = (getHwParam("auto_servo_on", "true") == "true");

  api_ = std::make_unique<WmxSystemHardwareApi>(logger_, config);

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
    } else if (cmd_ifs.size() == 1 && cmd_ifs[0].name == hardware_interface::HW_IF_VELOCITY) {
      joint.mode = JointMode::Velocity;
    } else {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must declare either no command_interface (state-only) or a "
        "single 'velocity' command_interface", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Joint '%s' -> WMX axis %d, mode=%s",
      joint.name.c_str(), joint.axis,
      joint.mode == JointMode::Velocity ? "velocity" : "state-only");

    joints_.push_back(joint);
  }

  axes_.clear();
  axes_.reserve(joints_.size());
  for (const auto & joint : joints_) {
    axes_.push_back(joint.axis);
  }

  RCLCPP_INFO(
    logger_,
    "WmxSystemHardware initialised: %zu joints, sdk_path=%s, param_file=%s",
    joints_.size(), config.sdkPath.c_str(),
    wmx_param_file_.empty() ? "(none)" : wmx_param_file_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Refresh the cached feedback and re-baseline the exported state interfaces.
void WmxSystemHardware::seedJointStates()
{
  std::string message;
  api_->getStatus(axes_, feedback_, communicating_, message);

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    joints_[i].pos_state = feedback_[i].actualPos;
    joints_[i].vel_state = feedback_[i].actualVelocity;
    joints_[i].cmd = 0.0;
    joints_[i].last_cmd = std::numeric_limits<double>::quiet_NaN();
  }
}

hardware_interface::CallbackReturn WmxSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string message;
  if (api_->attachDevice(message) != ErrorCode::None) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!wmx_param_file_.empty()) {
    // A bad param file is logged but not fatal: the engine keeps its current params.
    api_->importAndSetAll(wmx_param_file_, message);
  }

  seedJointStates();

  RCLCPP_INFO(logger_, "WmxSystemHardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// wmx_engine_node is launched alongside the controller_manager and takes ~1s
// to bring EtherCAT to Communicating. Wait for it (bounded) rather than
// failing activation outright, which aborts ros2_control_node.
bool WmxSystemHardware::waitForCommunicating()
{
  for (int attempt = 1; attempt <= max_device_retries_; ++attempt) {
    std::string message;
    api_->getStatus(axes_, feedback_, communicating_, message);
    if (communicating_) {
      return true;
    }

    RCLCPP_WARN(
      logger_, "WMX engine not Communicating yet, waiting... (%d/%d)",
      attempt, max_device_retries_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  RCLCPP_ERROR(
    logger_,
    "WMX engine is not Communicating after %d s. Start wmx_engine_node and "
    "EtherCAT communication first.", max_device_retries_);
  return false;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!waitForCommunicating()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Clear amp alarms and enable the servos (StartVel/CyclicBuffer require
  // servo-on). Disable via the 'auto_servo_on' hardware param if servos are
  // managed elsewhere.
  if (auto_servo_on_) {
    std::string message;
    for (const auto & joint : joints_) {
      api_->clearAmpAlarm(joint.axis, message);
    }
    for (const auto & joint : joints_) {
      if (api_->setServoOn(joint.axis, 1, message) != ErrorCode::None) {
        RCLCPP_ERROR(logger_, "%s", message.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(logger_, "Servos enabled on all axes");
  }

  seedJointStates();

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      startVelocity(joint, 0.0);
    }
  }
  if (auto_servo_on_) {
    std::string message;
    for (const auto & joint : joints_) {
      api_->setServoOn(joint.axis, 0, message);
    }
  }
  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (api_->isDeviceOpen()) {
    api_->releaseDevice();
  }
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
    if (joint.mode == JointMode::Velocity) {
      command_interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd);
    }
  }
  return command_interfaces;
}

hardware_interface::return_type WmxSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string message;
  if (api_->getStatus(axes_, feedback_, communicating_, message) != ErrorCode::None) {
    RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000, "%s", message.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    joints_[i].pos_state = feedback_[i].actualPos;
    joints_[i].vel_state = feedback_[i].actualVelocity;
  }
  return hardware_interface::return_type::OK;
}

void WmxSystemHardware::startVelocity(const WmxJoint & joint, double omega)
{
  std::string message;
  if (api_->startVel(joint.axis, omega, message) != ErrorCode::None) {
    RCLCPP_ERROR_THROTTLE(logger_, clock_, 1000, "%s", message.c_str());
  }
}

hardware_interface::return_type WmxSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!communicating_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "WMX engine not Communicating; skipping write()");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < joints_.size() && i < feedback_.size(); ++i) {
    WmxJoint & joint = joints_[i];
    if (joint.mode != JointMode::Velocity) {
      continue;
    }
    // Skip (don't spam StartVel errors) while the servo is off or alarmed.
    const auto & axis_status = feedback_[i];
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
