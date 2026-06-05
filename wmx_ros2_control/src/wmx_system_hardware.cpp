// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include "wmx_ros2_control/wmx_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#ifndef WMX3_SDK_PATH
#define WMX3_SDK_PATH "/opt/wmx3/"
#endif

namespace wmx_ros2_control
{

using wmx3Api::AxisSelection;
using wmx3Api::CoreMotion;
using wmx3Api::CyclicBuffer;
using wmx3Api::CyclicBufferCommandType;
using wmx3Api::CyclicBufferMultiAxisCommands;
using wmx3Api::DeviceType;
using wmx3Api::EngineState;
using wmx3Api::ErrorCode;
using wmx3Api::ProfileType;
using wmx3Api::Velocity;

namespace
{
constexpr double kCmdEpsilon = 1e-9;
}  // namespace

std::string WmxSystemHardware::getHwParam(
  const std::string & key, const std::string & def) const
{
  auto it = info_.hardware_parameters.find(key);
  return (it != info_.hardware_parameters.end()) ? it->second : def;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  sdk_path_ = getHwParam("wmx_sdk_path", WMX3_SDK_PATH);
  device_name_ = getHwParam("device_name", "wmx_ros2_control");
  wmx_param_file_ = getHwParam("wmx_param_file", "");
  interval_cycles_ = static_cast<unsigned int>(std::stoul(getHwParam("interval_cycles", "1")));
  acc_time_ms_ = std::stod(getHwParam("acc_time_ms", "1.0"));
  dec_time_ms_ = std::stod(getHwParam("dec_time_ms", "1.0"));
  max_device_retries_ = std::stoi(getHwParam("max_device_retries", "30"));

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & j : info_.joints) {
    WmxJoint joint;
    joint.name = j.name;

    // WMX axis index is mandatory per joint.
    auto axis_it = j.parameters.find("axis");
    if (axis_it == j.parameters.end()) {
      RCLCPP_FATAL(
        logger_, "Joint '%s' is missing required <param name=\"axis\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint.axis = std::stoi(axis_it->second);

    // Command mode is derived from the single declared command interface.
    if (j.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        logger_, "Joint '%s' must declare exactly one command_interface (got %zu)",
        j.name.c_str(), j.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string & cmd_if = j.command_interfaces[0].name;
    if (cmd_if == hardware_interface::HW_IF_VELOCITY) {
      joint.mode = JointMode::Velocity;
    } else if (cmd_if == hardware_interface::HW_IF_POSITION) {
      joint.mode = JointMode::Position;
      has_position_joints_ = true;
    } else {
      RCLCPP_FATAL(
        logger_, "Joint '%s' has unsupported command_interface '%s' (expected position|velocity)",
        j.name.c_str(), cmd_if.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_, "Joint '%s' -> WMX axis %d, mode=%s",
      joint.name.c_str(), joint.axis,
      joint.mode == JointMode::Velocity ? "velocity" : "position");

    joints_.push_back(joint);
  }

  // Build the axis selection for the position joints. They are streamed together
  // in one multi-axis CyclicBuffer call, indexed by selection position j (with
  // axis[j] = absolute WMX axis) — the same convention used by
  // wmx_ros2_package's joint_trajectory_controller.
  pos_joint_idx_.clear();
  pos_axis_sel_.axisCount = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].mode == JointMode::Position) {
      pos_axis_sel_.axis[pos_axis_sel_.axisCount] = joints_[i].axis;
      pos_axis_sel_.axisCount += 1;
      pos_joint_idx_.push_back(i);
    }
  }

  RCLCPP_INFO(
    logger_,
    "WmxSystemHardware initialised: %zu joints (%d position), sdk_path=%s, "
    "param_file=%s, interval_cycles=%u",
    joints_.size(), pos_axis_sel_.axisCount, sdk_path_.c_str(),
    wmx_param_file_.empty() ? "(none)" : wmx_param_file_.c_str(), interval_cycles_);

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

hardware_interface::CallbackReturn WmxSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!attachDevice()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cm_ = std::make_unique<CoreMotion>(&wmx_);
  cyclic_ = std::make_unique<CyclicBuffer>(&wmx_);

  // Apply the WMX axis parameter set (gear ratios, units, limits, ...).
  // The engine itself is started/owned by wmx_engine_node; here we only attach.
  if (!wmx_param_file_.empty()) {
    int err = cm_->config->ImportAndSetAll(const_cast<char *>(wmx_param_file_.c_str()));
    if (err != ErrorCode::None) {
      cm_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR(logger_, "Failed to import WMX params. Error=%d (%s)", err, err_str_);
    } else {
      RCLCPP_INFO(logger_, "Applied WMX params from %s", wmx_param_file_.c_str());
    }
  }

  // Initialise commands to the current feedback so activation does not jump.
  cm_->GetStatus(&cm_status_);
  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
    joint.cmd = (joint.mode == JointMode::Position) ? joint.pos_state : 0.0;
    joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!engineCommunicating()) {
    RCLCPP_ERROR(
      logger_,
      "WMX engine is not Communicating. Start wmx_engine_node and EtherCAT communication first.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed commands from current state so activation does not jump.
  cm_->GetStatus(&cm_status_);
  for (auto & joint : joints_) {
    joint.pos_state = cm_status_.axesStatus[joint.axis].actualPos;
    joint.vel_state = cm_status_.axesStatus[joint.axis].actualVelocity;
    joint.cmd = (joint.mode == JointMode::Position) ? joint.pos_state : 0.0;
    joint.last_cmd = std::numeric_limits<double>::quiet_NaN();
  }

  // Open + execute a single multi-axis cyclic buffer covering all position axes.
  if (has_position_joints_) {
    // A few cycles of buffer headroom relative to the per-command interval.
    const unsigned int buffer_cycles = interval_cycles_ * 4u;
    int err = cyclic_->OpenCyclicBuffer(&pos_axis_sel_, buffer_cycles);
    if (err != ErrorCode::None) {
      cyclic_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR(logger_, "OpenCyclicBuffer failed. Error=%d (%s)", err, err_str_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    err = cyclic_->Execute(&pos_axis_sel_);
    if (err != ErrorCode::None) {
      cyclic_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR(logger_, "CyclicBuffer Execute failed. Error=%d (%s)", err, err_str_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "WmxSystemHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & joint : joints_) {
    if (joint.mode == JointMode::Velocity) {
      startVelocity(joint, 0.0);
    }
  }
  if (has_position_joints_) {
    cyclic_->Abort(&pos_axis_sel_);
    cyclic_->CloseCyclicBuffer(&pos_axis_sel_);
  }
  RCLCPP_INFO(logger_, "WmxSystemHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WmxSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
    const char * iface = (joint.mode == JointMode::Position)
      ? hardware_interface::HW_IF_POSITION
      : hardware_interface::HW_IF_VELOCITY;
    command_interfaces.emplace_back(joint.name, iface, &joint.cmd);
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
      logger_, clock_, 1000,
      "GetStatus failed. Error=%d (%s)", err, err_str_);
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Engine must be communicating to accept motion commands.
  if (cm_status_.engineState != EngineState::T::Communicating) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000,
      "WMX engine not Communicating; skipping write()");
    return hardware_interface::return_type::OK;
  }

  // Velocity joints: (re)issue a profile only when the command actually changes,
  // to avoid restarting the trapezoidal profile every control cycle.
  for (auto & joint : joints_) {
    if (joint.mode != JointMode::Velocity) {
      continue;
    }
    if (std::isnan(joint.last_cmd) || std::fabs(joint.cmd - joint.last_cmd) > kCmdEpsilon) {
      startVelocity(joint, joint.cmd);
      joint.last_cmd = joint.cmd;
    }
  }

  // Position joints: stream one absolute-position segment per cycle for ALL
  // position axes in a single multi-axis CyclicBuffer command. cmd[j] aligns
  // with pos_axis_sel_.axis[j] (selection-position indexing).
  if (has_position_joints_) {
    CyclicBufferMultiAxisCommands cmds;
    for (size_t j = 0; j < pos_joint_idx_.size(); ++j) {
      WmxJoint & joint = joints_[pos_joint_idx_[j]];
      cmds.cmd[j].type = CyclicBufferCommandType::AbsolutePos;
      cmds.cmd[j].command = joint.cmd;
      cmds.cmd[j].intervalCycles = interval_cycles_;
      joint.last_cmd = joint.cmd;
    }
    int err = cyclic_->AddCommand(&pos_axis_sel_, &cmds);
    if (err != ErrorCode::None) {
      cyclic_->ErrorToString(err, err_str_, sizeof(err_str_));
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 1000,
        "CyclicBuffer multi-axis AddCommand failed. Error=%d (%s)", err, err_str_);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace wmx_ros2_control

PLUGINLIB_EXPORT_CLASS(
  wmx_ros2_control::WmxSystemHardware, hardware_interface::SystemInterface)
