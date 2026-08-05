// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
#define WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

#include "wmx_r2_control/wmx_cyclic_stream.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

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

namespace wmx_r2_control
{

enum class JointMode
{
  Position,    // command_interface "position" -> WMX CyclicBuffer (AbsolutePos)
  Velocity,    // command_interface "velocity" -> WMX CoreMotion StartVel
  StateOnly    // no command_interface -> feedback only (motion driven elsewhere)
};

struct WmxJoint
{
  std::string name;
  int axis = -1;
  JointMode mode = JointMode::Velocity;

  double pos_state = 0.0;
  double vel_state = 0.0;

  double cmd = 0.0;
  double last_cmd = 0.0;
};

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

  // Position-mode streaming (all position joints share one cyclic buffer, since
  // they are commanded by a single multi-axis AddCommand).
  WmxCyclicStream stream_;
  std::vector<size_t> position_joint_idx_;   ///< indices into joints_, in axis order
  std::vector<double> q_cmd_scratch_;        ///< preallocated: write() must not allocate
  std::string stream_err_;                   ///< reused so the happy path stays allocation-free
  bool stream_started_ = false;

  // Hardware parameters (from <hardware><param> in the ros2_control xacro)
  std::string sdk_path_;
  std::string device_name_;
  std::string wmx_param_file_;
  double acc_time_ms_ = 1.0;
  double dec_time_ms_ = 1.0;
  int max_device_retries_ = 30;
  bool auto_servo_on_ = true;   ///< clear alarms + servo-on in on_activate, servo-off on deactivate
  double cyclic_buffer_horizon_s_ = 5.0;
  int cyclic_target_periods_ = 2;
  double cyclic_max_acc_ = 0.0;
  /// Expected write() period [ms]; only a seed, the real one is measured from
  /// write()'s `period`. Should match 1000 / controller_manager update_rate.
  double cyclic_push_period_ms_ = 10.0;

  char err_str_[256] = {0};

  // Helpers
  hardware_interface::CallbackReturn initImpl();
  bool attachDevice();
  void closeDevice();
  bool engineCommunicating();
  void startVelocity(const WmxJoint & joint, double omega);
  std::string getHwParam(const std::string & key, const std::string & def) const;
  /// Seed each position joint's command from its current WMX *command* position,
  /// so a write() that lands before any controller has written is a hold.
  bool seedPositionCommands();
  bool openAndStartStream();
};

}  // namespace wmx_r2_control

#endif  // WMX_R2_CONTROL__WMX_SYSTEM_HARDWARE_HPP_
