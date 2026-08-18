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
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "wmx_r2_message/srv/set_axes.hpp"
#include "wmx_r2_message/srv/set_axes_gear_ratio.hpp"
#include "wmx_r2_message/srv/load_wmx_params.hpp"
#include "wmx_r2_message/srv/get_wmx_params.hpp"
#include "wmx_r2_message/msg/axes_velocity.hpp"
#include "wmx_r2_message/msg/axes_status.hpp"
#include "wmx_r2_message/msg/axes_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using std::placeholders::_1;
using std::placeholders::_2;

class WmxCoreMotionNode : public rclcpp::Node
{
public:
  WmxCoreMotionNode();
  ~WmxCoreMotionNode();

private:
  std::atomic<bool> initialized_{false};
  int axisCount_;
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

  // Jog is a dead-man command: the publisher must keep refreshing wmx/axes/start_jog,
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

  rclcpp::TimerBase::SharedPtr axesStatusTimer_;
  rclcpp::TimerBase::SharedPtr jogWatchdogTimer_;
  wmx_r2_message::msg::AxesStatus axesStatusMsg_;

  rclcpp::CallbackGroup::SharedPtr init_cb_group_;
  rclcpp::CallbackGroup::SharedPtr homing_cb_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr coreMotionReadyPub_;
  rclcpp::Publisher<wmx_r2_message::msg::AxesStatus>::SharedPtr axesStatusPub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxesVelocity>::SharedPtr startVelSub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxesVelocity>::SharedPtr startJogSub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxesPose>::SharedPtr startPosSub_;
  rclcpp::Subscription<wmx_r2_message::msg::AxesPose>::SharedPtr startMovSub_;

  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr setServoOnService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr clearAmpAlarmService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr setAxisCommandModeService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr setAxisPolarityService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxesGearRatio>::SharedPtr setGearRatioService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr startHomeService_;
  rclcpp::Service<wmx_r2_message::srv::SetAxes>::SharedPtr stopAxisService_;
  rclcpp::Service<wmx_r2_message::srv::LoadWmxParams>::SharedPtr loadWmxParamsService_;
  rclcpp::Service<wmx_r2_message::srv::GetWmxParams>::SharedPtr getWmxParamsService_;

  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void axesStatusStep();

  void startPosCallback(const wmx_r2_message::msg::AxesPose::SharedPtr msg);
  void startMovCallback(const wmx_r2_message::msg::AxesPose::SharedPtr msg);
  void startVelCallback(const wmx_r2_message::msg::AxesVelocity::SharedPtr msg);
  void startJogCallback(const wmx_r2_message::msg::AxesVelocity::SharedPtr msg);

  void jogWatchdogStep();
  int stopAxis(int axis);

  void setServoOnCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void setAxisCommandModeCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void clearAmpAlarmCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void setAxisPolarityCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void setGearRatioCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxesGearRatio::Response> response);
  void startHomeCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void stopAxisCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxes::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxes::Response> response);
  void loadWmxParamsCallback(
    const std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::LoadWmxParams::Response> response);
  void getWmxParamsCallback(
    const std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Request> request,
    std::shared_ptr<wmx_r2_message::srv::GetWmxParams::Response> response);
};

#endif  // WMX_CORE_MOTION_NODE_HPP_
