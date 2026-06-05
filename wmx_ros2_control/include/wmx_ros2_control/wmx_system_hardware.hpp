// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ROS2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
#define WMX_ROS2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "CyclicBufferApi.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace wmx_ros2_control
{

/// Per-joint command mode, derived from the command interface declared in the URDF.
enum class JointMode
{
  Velocity,   ///< command_interface "velocity"  -> WMX CoreMotion StartVel
  Position    ///< command_interface "position"  -> WMX CyclicBuffer AbsolutePos
};

/// A single ros2_control joint backed by one WMX3 axis.
struct WmxJoint
{
  std::string name;
  int axis = -1;
  JointMode mode = JointMode::Velocity;

  // State (1:1 with WMX user units; the WMX parameter file is expected to be
  // configured so that user units are radians, matching the existing nodes).
  double pos_state = 0.0;
  double vel_state = 0.0;

  // Command (interface depends on mode)
  double cmd = 0.0;
  double last_cmd = 0.0;
};

/// ros2_control SystemInterface that drives WMX3-controlled EtherCAT axes.
///
/// This hardware plugin attaches to the WMX3 engine started by wmx_engine_node
/// (it does NOT start/stop the engine or communication itself). It supports two
/// per-joint command modes within the same system:
///   * velocity joints (e.g. a differential-drive base) are driven with
///     CoreMotion velocity commands (StartVel);
///   * position joints (e.g. a manipulator under joint_trajectory_controller)
///     are streamed with the CyclicBuffer module as AbsolutePos commands at the
///     controller_manager update rate.
class WmxSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WmxSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("WmxSystemHardware");
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  // WMX3 handles
  wmx3Api::WMX3Api wmx_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;
  std::unique_ptr<wmx3Api::CyclicBuffer> cyclic_;
  wmx3Api::CoreMotionStatus cm_status_;
  bool device_open_ = false;

  // Joints
  std::vector<WmxJoint> joints_;
  bool has_position_joints_ = false;

  // Position joints are streamed together through a single multi-axis
  // CyclicBuffer (same convention as wmx_ros2_package's joint_trajectory_controller:
  // multi-axis arrays are indexed by selection position j, with axis[j] giving
  // the absolute WMX axis). pos_joint_idx_[j] indexes into joints_.
  wmx3Api::AxisSelection pos_axis_sel_{};
  std::vector<size_t> pos_joint_idx_;

  // Hardware parameters (from <hardware><param> in the ros2_control xacro)
  std::string sdk_path_;
  std::string device_name_;
  std::string wmx_param_file_;
  unsigned int interval_cycles_ = 1;
  double acc_time_ms_ = 1.0;
  double dec_time_ms_ = 1.0;
  int max_device_retries_ = 30;

  char err_str_[256] = {0};

  // Helpers
  bool attachDevice();
  void closeDevice();
  bool engineCommunicating();
  void startVelocity(const WmxJoint & joint, double omega);
  std::string getHwParam(const std::string & key, const std::string & def) const;
};

}  // namespace wmx_ros2_control

#endif  // WMX_ROS2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
