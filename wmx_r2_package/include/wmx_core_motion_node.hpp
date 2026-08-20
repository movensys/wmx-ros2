// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_CORE_MOTION_NODE_HPP_
#define WMX_CORE_MOTION_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

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

class WmxCoreMotionNodeApi
{
public:
  struct Config
  {
    double jogTimeoutMs = 200.0;
    double jogRunTimeMs = 2000.0;
    double jogJerkRatio = 0.75;
  };

  WmxCoreMotionNodeApi(const rclcpp::Logger & logger, const Config & config);
  ~WmxCoreMotionNodeApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  int readAxisCount();
  int getStatus(wmx3Api::CoreMotionStatus & status);

  int startPos(
    int axis, double target, double velocity, double acc, double dec, std::string & message);
  int startMov(
    int axis, double target, double velocity, double acc, double dec, std::string & message);
  int startVel(int axis, double velocity, double acc, double dec, std::string & message);
  int stopAxis(int axis);

  int startJog(
    int axis, double velocity, double acc, double dec, const rclcpp::Time & now,
    std::string & message);
  void stopExpiredJogs(const rclcpp::Time & now);
  void stopAllJogs();
  void clearJog(int axis);

  int setServoOn(int axis, int on, std::string & message);
  int setAxisCommandMode(int axis, int mode, std::string & message);
  int clearAmpAlarm(int axis, std::string & message);
  int setAxisPolarity(int axis, int polarity, std::string & message);
  int setGearRatio(int axis, double numerator, double denominator, std::string & message);
  int startHome(int axis, std::string & message);

  int loadWmxParams(const std::string & path, std::string & message);
  int getWmxParams(
    const std::vector<int32_t> & axes, std::vector<std::string> & dump, std::string & message);

  bool isDeviceOpen() const {return cm_ != nullptr;}

private:
  struct JogState
  {
    double velocity;
    rclcpp::Time deadline;
  };

  std::string errorText(int err);

  rclcpp::Logger logger_;
  Config config_;

  const char * deviceName_ = "wmx_core_motion_node";
  unsigned int timeout_ = 10000;
  unsigned int servoOnTimeout_ = 1000;

  std::mutex jogMutex_;
  std::unordered_map<int, JogState> jogState_;

  std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;
};

class WmxCoreMotionNode : public rclcpp::Node
{
public:
  WmxCoreMotionNode();
  ~WmxCoreMotionNode();

private:
  std::unique_ptr<WmxCoreMotionNodeApi> api_;

  std::atomic<bool> initialized_{false};
  int axisCount_ = 0;
  const int rate_ = 100;

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

  bool isReady();
  std::string notReadyMessage();

  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void axesStatusStep();

  void startPosCallback(const wmx_r2_message::msg::AxesPose::SharedPtr msg);
  void startMovCallback(const wmx_r2_message::msg::AxesPose::SharedPtr msg);
  void startVelCallback(const wmx_r2_message::msg::AxesVelocity::SharedPtr msg);
  void startJogCallback(const wmx_r2_message::msg::AxesVelocity::SharedPtr msg);

  void jogWatchdogStep();

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
