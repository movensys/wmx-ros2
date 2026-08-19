// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_POSITION_CONTROLLER_HPP_
#define JOINT_POSITION_CONTROLLER_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class JointPositionControllerApi
{
public:
  explicit JointPositionControllerApi(const rclcpp::Logger & logger);
  ~JointPositionControllerApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  void setAxes(const std::vector<int64_t> & axes);

  int getPosCmd(
    const std::vector<int> & axes, std::vector<double> & posCmd, std::string & message);

  int startLinearIntplPos(
    const std::vector<int> & axes,
    const std::vector<double> & targets,
    const std::vector<double> & maxVelocity,
    const std::vector<double> & maxAcc,
    std::string & message);

  int stopAxes(std::string & message);

  bool isDeviceOpen() const {return cm_ != nullptr;}

private:
  std::string errorText(int err);

  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_position_controller";
  unsigned int timeout_ = 10000;

  std::mutex deviceMutex_;

  wmx3Api::AxisSelection axisSel_;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;
};

class JointPositionController : public rclcpp::Node
{
public:
  JointPositionController();
  ~JointPositionController();

private:
  std::unique_ptr<JointPositionControllerApi> api_;

  bool initialized_ = false;
  std::atomic<bool> initializing_{false};
  std::atomic<bool> in_execution_{false};

  std::vector<int64_t> jointAxes_;
  std::vector<std::string> jointNames_;
  std::string jointTrajectoryTopic_;
  double accelRatio_ = 0.0;
  double defaultVelocity_ = 0.0;
  double minStep_ = 0.0;

  static constexpr int kMaxDeviceRetries = 30;

  std::map<std::string, int> axisByName_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execActiveSub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointTrajectorySub_;

  std::thread init_thread_;

  struct IntplCommand
  {
    std::vector<int> axes;
    std::vector<double> targets;
    std::vector<double> maxVelocity;
    std::vector<double> maxAcc;
  };

  void setRosParameter();
  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onExecActive(const std_msgs::msg::Bool::SharedPtr msg);
  void runInitSequence();
  bool attachDeviceWithRetries();
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  bool buildCommand(
    const trajectory_msgs::msg::JointTrajectory & traj, IntplCommand & command);
};

#endif  // JOINT_POSITION_CONTROLLER_HPP_
