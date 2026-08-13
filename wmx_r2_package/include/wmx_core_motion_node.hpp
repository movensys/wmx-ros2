// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_CORE_MOTION_NODE_HPP_
#define WMX_CORE_MOTION_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "wmx_r2_message/srv/set_axis.hpp"
#include "wmx_r2_message/srv/set_axis_gear_ratio.hpp"
#include "wmx_r2_message/msg/axis_velocity.hpp"
#include "wmx_r2_message/msg/axis_state.hpp"
#include "wmx_r2_message/msg/axis_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

// Everything that talks to the WMX3 SDK. No ROS entities, only a logger.
//
// deviceMutex_ guards only the cm_ handle itself, never the SDK calls made
// through it: setHoming() blocks in motion->Wait() for the whole homing move,
// and holding a lock across that would stall the axis-state timer and the jog
// dead-man watchdog. Callers take a shared_ptr copy under the lock and then
// work outside it, so a concurrent releaseDevice() cannot pull the object out
// from under an in-flight call.
class WmxCoreMotionNodeApi
{
public:
  explicit WmxCoreMotionNodeApi(const rclcpp::Logger & logger);
  ~WmxCoreMotionNodeApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  // Number of axes the engine reports across its cyclic handlers.
  int readAxisCount();

  int getStatus(wmx3Api::CoreMotionStatus & status);

  // Motion. Each returns a WMX3 error code; message carries the outcome.
  int startPos(
    int axis, double target, double velocity, double acc, double dec, std::string & message);
  int startMov(
    int axis, double target, double velocity, double acc, double dec, std::string & message);
  int startVel(int axis, double velocity, double acc, double dec, std::string & message);
  int startJog(
    int axis, double velocity, double accTimeMs, double decTimeMs, double jerkRatio,
    double runTimeMs, std::string & message);

  // Stopping an already idle axis is expected (the jog run time may have
  // elapsed before the operator released), so this one only logs at DEBUG.
  int stopAxis(int axis);

  // Axis configuration and state.
  int setServoOn(int axis, int on, std::string & message);
  int setAxisCommandMode(int axis, int mode, std::string & message);
  int clearAmpAlarm(int axis, std::string & message);
  int setAxisPolarity(int axis, int polarity, std::string & message);
  int setGearRatio(int axis, int numerator, int denominator, std::string & message);
  int homeAxis(int axis, std::string & message);

  bool isDeviceOpen() const {return deviceOpen_;}

private:
  // A handle that stays valid for the duration of the call, or null.
  std::shared_ptr<wmx3Api::CoreMotion> device();

  std::string errorText(int err);

  rclcpp::Logger logger_;

  const char * deviceName_ = "wmx_core_motion_node";
  unsigned int timeout_ = 10000;
  unsigned int servoOnTimeout_ = 1000;

  mutable std::mutex deviceMutex_;

  wmx3Api::WMX3Api wmx3Lib_;
  std::shared_ptr<wmx3Api::CoreMotion> cm_;

  std::atomic<bool> deviceOpen_{false};
};

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
  std::unique_ptr<WmxCoreMotionNodeApi> api_;

  int axisCount_ = 0;
  const int rate_ = 100;

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

  void stopAllJogs();
  // Read straight off the lifecycle state machine: a mirrored flag can drift
  // from it when a transition fails.
  bool isNodeActive() const;
  std::string notActiveMessage() const;

  void axisStateStep();
  void jogWatchdogStep();

  void axisPoseCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg);
  void axisPoseRelativeCallback(const wmx_r2_message::msg::AxisPose::SharedPtr msg);
  void axisVelCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg);
  void axisJogCallback(const wmx_r2_message::msg::AxisVelocity::SharedPtr msg);

  void setAxisOnCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void setAxisModeCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void clearAlarmCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void setAxisPolarityCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void setAxisGearRatioCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxisGearRatio::Response> response);
  void setHomingCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
  void stopAxesCallback(
    const std::shared_ptr<wmx_r2_message::srv::SetAxis::Request> request,
    std::shared_ptr<wmx_r2_message::srv::SetAxis::Response> response);
};

#endif  // WMX_CORE_MOTION_NODE_HPP_
