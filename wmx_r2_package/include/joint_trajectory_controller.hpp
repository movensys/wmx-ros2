// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef JOINT_TRAJECTORY_CONTROLLER_HPP_
#define JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/bool.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "AdvancedMotionApi.h"

class JointTrajectoryControllerApi
{
public:
  static constexpr int kSplineChannel = 0;
  static constexpr int kMaxTrajectoryPoints = 1000;

  explicit JointTrajectoryControllerApi(const rclcpp::Logger & logger);
  ~JointTrajectoryControllerApi();

  int attachDevice(std::string & message);
  void releaseDevice();

  void setAxes(const std::vector<int64_t> & axes);

  int importAndSetAll(
    const std::string & path, wmx3Api::Config::AxisParam & axisParamError,
    std::string & message);
  int getAxisParam(wmx3Api::Config::AxisParam & axisParam, std::string & message);

  int startCSplinePos(
    const std::vector<std::vector<double>> & positions,
    const std::vector<double> & timesMs,
    std::string & message);

  int getInPos(bool & inPos, std::string & message);
  int stopAxes(std::string & message);

  bool isDeviceOpen() const {return cm_ != nullptr;}

private:
  std::string errorText(int err);

  rclcpp::Logger logger_;

  const char * deviceName_ = "joint_trajectory_controller";
  unsigned int timeout_ = 10000;

  std::mutex deviceMutex_;

  std::vector<int64_t> axes_;
  wmx3Api::AxisSelection axisSel_;

  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::CoreMotion> cm_;
  std::unique_ptr<wmx3Api::AdvancedMotion> am_;
};

class JointTrajectoryController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  JointTrajectoryController();
  ~JointTrajectoryController() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::unique_ptr<JointTrajectoryControllerApi> api_;

  std::vector<int64_t> jointAxes_;
  std::string jointTrajectoryAction_;
  std::string wmxParamFilePath_;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr execActivePub_;
  rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::JointJog>::SharedPtr servoResetPub_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<GoalHandleFJT> goal_handle);

  void handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle);

  void execute(std::shared_ptr<GoalHandleFJT> goal_handle);

  bool isNodeActive();
  std::string notActiveMessage();

  void setRosParameter();
  void setWmxParam(const std::string & path);
  void getWmxParam();
  void logTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);
  void publishExecActive(bool active);
  void resetServo(const std::vector<std::string> & joint_names);
};

#endif  // JOINT_TRAJECTORY_CONTROLLER_HPP_
