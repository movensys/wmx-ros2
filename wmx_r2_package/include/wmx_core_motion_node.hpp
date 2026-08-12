// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_CORE_MOTION_NODE_HPP_
#define WMX_CORE_MOTION_NODE_HPP_

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "wmx_r2_message/srv/set_axis.hpp"
#include "wmx_r2_message/srv/set_axis_gear_ratio.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"
#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/msg/axis_velocity.hpp"
#include "wmx_r2_message/msg/axis_state.hpp"
#include "wmx_r2_message/msg/axis_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using std::placeholders::_2;

// Managed node. wmx_engine_node drives the transitions:
//   configure  attach to the WMX3 device, read the axis count and advertise the
//              axis services, topics and (stopped) timers
//   activate   run the state/jog timers and accept motion commands
//   deactivate stop the timers, stop every jogging axis, reject commands
//   cleanup    drop the interfaces and detach from the device
class WmxCoreMotionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WmxCoreMotionNode();
  ~WmxCoreMotionNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::atomic<bool> active_{false};
  bool deviceAttached_ = false;
  int axisCount_ = 0;
  int err_;
  char errString_[256];
  char buffer_[512];
  const int rate_ = 100;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> wmx3LibCm_;
  wmx3Api::CoreMotionStatus cmStatus_;
  // For getting axisCount_, we use this api.
  wmx3Api::EngineStatus engineStatus_;

  wmx3Api::Velocity::VelCommand velocity_;
  wmx3Api::Motion::PosCommand position_;
  wmx3Api::Config::HomeParam homeParam_;

  // Jog is a dead-man command: the publisher must keep refreshing wmx/axis/jog,
  // and the axis is stopped once refreshes stop arriving (cmd_vel pattern).
  struct JogState
  {
    double velocity;
    rclcpp::Time deadline;
  };

  double jogTimeoutMs_ = 200.0;
  double jogRunTimeMs_ = 2000.0;
  double jogJerkRatio_ = 0.75;
  std::mutex jogMutex_;
  std::unordered_map<int, JogState> jogState_;

  rclcpp::TimerBase::SharedPtr axisStateTimer_;
  rclcpp::TimerBase::SharedPtr jogWatchdogTimer_;
  wmx_r2_message::msg::AxisState axisStateMsg_;

  rclcpp::CallbackGroup::SharedPtr homing_cb_group_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr coreMotionReadyPub_;
  rclcpp_lifecycle::LifecyclePublisher<wmx_r2_message::msg::AxisState>::SharedPtr axisStatePub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxisVelocity>::SharedPtr axisVelSub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxisVelocity>::SharedPtr axisJogSub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxisPose>::SharedPtr axisPoseSub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxisPose>::SharedPtr axisPoseRelativeSub_;

  rclcpp::Service<wmx_r2_message::srv::SetAxis>::SharedPtr setAxisOnService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxis>::SharedPtr clearAlarmService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxis>::SharedPtr setAxisModeService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxis>::SharedPtr setAxisPolarityService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxisGearRatio>::SharedPtr setAxisGearRatioService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxis>::SharedPtr setHomingService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxis>::SharedPtr stopAxisService_;
  rclcpp::Service<wmx_r2_message::srv::LoadWmxParams>::SharedPtr loadParamsService_;
  rclcpp::Service<wmx_r2_message::srv::GetWmxParams>::SharedPtr getParamsService_;

  bool attachDevice();
  void releaseDevice();
  void stopAllJogs();
  std::string notActiveMessage() const;

  void axisStateStep();

  void axisPoseCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg);
  void axisPoseRelativeCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg);
  void axisVelCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg);
  void axisJogCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg);

  void jogWatchdogStep();
  int stopAxis(int axis);

  void setAxisOn(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void setAxisMode(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void clearAlarm(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void setAxisPolarity(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void setAxisGearRatio(
    const std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Response> response);
  void setHoming(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void stopAxes(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void loadWmxParams(
    const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response);
  void getWmxParams(
    const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response);
};

#endif  // WMX_CORE_MOTION_NODE_HPP_
