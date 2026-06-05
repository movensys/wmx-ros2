// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ROS2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
#define WMX_ROS2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// hardware_interface >= 4.x (Jazzy) deprecates on_init(HardwareInfo) in favour
// of on_init(HardwareComponentInterfaceParams). Detect the new signature so the
// plugin builds warning-free on Jazzy while staying compatible with Humble.
#if defined(__has_include)
#  if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#    include "hardware_interface/types/hardware_component_interface_params.hpp"
#    define WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS 1
#  endif
#endif

#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace wmx_ros2_control
{

/// Per-joint role, derived from the command interface declared in the URDF.
enum class JointMode
{
  Velocity,    ///< command_interface "velocity" -> WMX CoreMotion StartVel
  StateOnly    ///< no command_interface -> feedback only (motion driven elsewhere,
               ///< e.g. the custom joint_trajectory_controller node for the arm)
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

  // Velocity command (only used when mode == Velocity)
  double cmd = 0.0;
  double last_cmd = 0.0;
};

/// ros2_control SystemInterface that drives WMX3-controlled EtherCAT axes.
///
/// This hardware plugin attaches to the WMX3 engine started by wmx_engine_node
/// (it does NOT start/stop the engine or communication itself). It supports:
///   * velocity joints (e.g. a differential-drive base) driven with CoreMotion
///     velocity commands (StartVel);
///   * state-only joints (e.g. a manipulator whose trajectory is executed by the
///     standalone joint_trajectory_controller node via AdvancedMotion C-Spline) —
///     these expose encoder position/velocity for joint_state_broadcaster but
///     take no command from ros2_control.
class WmxSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WmxSystemHardware)

#if WMX_HAS_HW_COMPONENT_INTERFACE_PARAMS
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#endif

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
  wmx3Api::CoreMotionStatus cm_status_;
  bool device_open_ = false;

  // Joints
  std::vector<WmxJoint> joints_;

  // Hardware parameters (from <hardware><param> in the ros2_control xacro)
  std::string sdk_path_;
  std::string device_name_;
  std::string wmx_param_file_;
  double acc_time_ms_ = 1.0;
  double dec_time_ms_ = 1.0;
  int max_device_retries_ = 30;

  char err_str_[256] = {0};

  // Helpers
  hardware_interface::CallbackReturn initImpl();
  bool attachDevice();
  void closeDevice();
  bool engineCommunicating();
  void startVelocity(const WmxJoint & joint, double omega);
  std::string getHwParam(const std::string & key, const std::string & def) const;
};

}  // namespace wmx_ros2_control

#endif  // WMX_ROS2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
