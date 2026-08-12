// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <sstream>
#include <chrono>
#include <string>
#include <vector>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "AdvancedMotionApi.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/bool.hpp"

#define MAX_TRAJ_POINTS 1000

struct ScopeExit
{
  std::function<void()> fn;
  ~ScopeExit() {fn();}
};

using wmx3Api::AdvancedMotion;
using wmx3Api::AdvMotion;
using wmx3Api::AxisSelection;
using wmx3Api::Config;
using wmx3Api::CoreMotion;
using wmx3Api::CoreMotionStatus;
using wmx3Api::DeviceType;
using wmx3Api::ErrorCode;
using wmx3Api::WMX3Api;

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

  std::vector<int64_t> jointAxes_;
  std::string jointTrajectoryAction_;

  int err_;
  char errString_[256];

private:
  std::atomic<bool> isActive_{false};
  bool isDeviceAttached_ = false;

  WMX3Api wmx3Lib_;
  CoreMotion wmx3LibCm_;
  AdvancedMotion wmx3LibAm_;
  AdvMotion::PointTimeSplineCommand spl;
  AdvMotion::SplinePoint pt_spl[MAX_TRAJ_POINTS];
  double time_spl[MAX_TRAJ_POINTS];
  AxisSelection axisSel;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr execActivePub_;
  rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::JointJog>::SharedPtr servoResetPub_;

  // Action server callback declarations
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<GoalHandleFJT> goal_handle);

  void handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle);

  void execute(std::shared_ptr<GoalHandleFJT> goal_handle);

  void setRosParameter();
  bool attachDevice();
  void releaseDevice();
  void logTrajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);
  void publishExecActive(bool active);
  void resetServo(const std::vector<std::string> & joint_names);
};

JointTrajectoryController::JointTrajectoryController()
: LifecycleNode("joint_trajectory_controller")
{
  setRosParameter();

  RCLCPP_INFO(
    this->get_logger(), "joint_trajectory_controller is unconfigured, waiting for configure...");
}

JointTrajectoryController::~JointTrajectoryController()
{
  RCLCPP_INFO(this->get_logger(), "Stop joint_trajectory_controller");

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is stopped");
}

bool JointTrajectoryController::attachDevice()
{
  if (isDeviceAttached_) {
    return true;
  }

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device (lock busy). Is the engine communicating?");
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return false;
  }

  wmx3Lib_.SetDeviceName("joint_trajectory_controller");
  isDeviceAttached_ = true;
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");
  return true;
}

void JointTrajectoryController::releaseDevice()
{
  if (!isDeviceAttached_) {
    return;
  }

  wmx3LibAm_.advMotion->FreeSplineBuffer(0);

  err_ = wmx3Lib_.CloseDevice();
  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    RCLCPP_ERROR(this->get_logger(), "Failed to close device");
  } else {
    RCLCPP_INFO(this->get_logger(), "Device closed");
  }
  isDeviceAttached_ = false;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring joint_trajectory_controller...");

  if (!attachDevice()) {
    return CallbackReturn::FAILURE;
  }

  wmx3LibCm_ = CoreMotion(&wmx3Lib_);
  wmx3LibAm_ = AdvancedMotion(&wmx3Lib_);
  wmx3LibAm_.advMotion->CreateSplineBuffer(0, MAX_TRAJ_POINTS);

  servoResetPub_ = this->create_publisher<control_msgs::msg::JointJog>(
    "/servo_node/delta_joint_cmds", 10);

  execActivePub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/moveit2_trajectory/execution_active", rclcpp::QoS(1).transient_local());

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    jointTrajectoryAction_,
    std::bind(
      &JointTrajectoryController::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&JointTrajectoryController::handle_cancel, this, std::placeholders::_1),
    std::bind(&JointTrajectoryController::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is configured");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  isActive_ = true;

  publishExecActive(false);

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is active");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  isActive_ = false;

  publishExecActive(false);

  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is inactive");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  isActive_ = false;

  action_server_.reset();
  execActivePub_.reset();
  servoResetPub_.reset();

  releaseDevice();

  RCLCPP_INFO(this->get_logger(), "joint_trajectory_controller is cleaned up");
  return CallbackReturn::SUCCESS;
}

JointTrajectoryController::CallbackReturn JointTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

void JointTrajectoryController::setRosParameter()
{
  this->declare_parameter<std::vector<int64_t>>("joint_axes", std::vector<int64_t>{});
  this->declare_parameter<std::string>(
    "joint_trajectory_action", "/joint_trajectory_action/no_param");

  this->get_parameter("joint_axes", jointAxes_);
  this->get_parameter("joint_trajectory_action", jointTrajectoryAction_);

  std::string joint_axes_str;
  for (size_t i = 0; i < jointAxes_.size(); ++i) {
    if (i > 0) {joint_axes_str += ", ";}
    joint_axes_str += std::to_string(jointAxes_[i]);
  }

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_axes: [%s]", joint_axes_str.c_str());
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_action: %s", jointTrajectoryAction_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

rclcpp_action::GoalResponse JointTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;
  (void)goal;

  if (!isActive_) {
    RCLCPP_WARN(
      this->get_logger(), "Goal rejected: joint_trajectory_controller is not active (state: %s).",
      this->get_current_state().label().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::handle_cancel(
  std::shared_ptr<GoalHandleFJT> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::thread{std::bind(&JointTrajectoryController::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void JointTrajectoryController::publishExecActive(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  execActivePub_->publish(msg);
}

void JointTrajectoryController::resetServo(const std::vector<std::string> & joint_names)
{
  control_msgs::msg::JointJog jog;
  jog.header.stamp = this->get_clock()->now();
  jog.joint_names = joint_names;
  jog.velocities.assign(joint_names.size(), 0.0);
  servoResetPub_->publish(jog);
}

void JointTrajectoryController::execute(std::shared_ptr<GoalHandleFJT> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto & trajectory = goal->trajectory;

  publishExecActive(true);
  const auto servo_joint_names = trajectory.joint_names;
  ScopeExit on_exit{[this, servo_joint_names]() {
      publishExecActive(false);
      resetServo(servo_joint_names);
    }};

  int num_points = trajectory.points.size();

  RCLCPP_INFO(this->get_logger(), "Received a new trajectory goal! Point number: [%d]", num_points);

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  double timeMilliseconds;

  if (num_points > MAX_TRAJ_POINTS) {
    RCLCPP_WARN(
      this->get_logger(),
      "Too many trajectory point size! "
      "current points:%d / max traj points:%d \nAborting current goal.",
      num_points, MAX_TRAJ_POINTS);
    goal_handle->abort(result);
    return;
  }

  logTrajectory(trajectory);

  // Generate spline commands from trajectory.points
  axisSel.axisCount = jointAxes_.size();
  spl.dimensionCount = jointAxes_.size();
  for (size_t j = 0; j < jointAxes_.size(); ++j) {
    axisSel.axis[j] = jointAxes_[j];
    spl.axis[j] = jointAxes_[j];
  }

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    timeMilliseconds = rclcpp::Duration(pt.time_from_start).seconds() * 1000;
    time_spl[i] = timeMilliseconds;

    for (size_t j = 0; j < jointAxes_.size(); ++j) {
      pt_spl[i].pos[j] = pt.positions.at(j);
    }
  }

  // If first time interval is not zero, make it zero
  if (time_spl[0] != 0.0) {
    time_spl[0] = 0.0;
  }

  // if last time interval is less than 1ms, ignore the last point.
  double last = trajectory.points.size() - 1;
  if (rclcpp::Duration(trajectory.points[last].time_from_start).seconds() -
    rclcpp::Duration(trajectory.points[last - 1].time_from_start).seconds() < 1e-3)
  {
    num_points -= 1;
  }

  if (num_points == 0) {
    RCLCPP_INFO(this->get_logger(), "Point count is zero. It is already in the targeted position");
  } else {
    RCLCPP_INFO(this->get_logger(), "Command Start!!!");
    err_ = wmx3LibAm_.advMotion->StartCSplinePos(0, &spl, num_points, pt_spl, time_spl);
    if (err_ != 0) {
      wmx3LibAm_.ErrorToString(err_, errString_, 256);
      RCLCPP_ERROR(this->get_logger(), "StartCSplinePos Error: %s", errString_);
      result->error_code = err_;
      goal_handle->abort(result);
      return;
    }

    while (true) {
      if (goal_handle->is_canceling()) {
        wmx3LibCm_.motion->Stop(&axisSel);
        wmx3LibCm_.motion->Wait(&axisSel);
        result->error_code = 0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled, axes stopped");
        return;
      }

      CoreMotionStatus cmStatus;
      wmx3LibCm_.GetStatus(&cmStatus);
      bool all_done = true;
      for (int axis : jointAxes_) {
        if (!cmStatus.axesStatus[axis].inPos) {all_done = false; break;}
      }
      if (all_done) {break;}

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  result->error_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
}

void JointTrajectoryController::logTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory){
  std::ostringstream jn;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    if (i) {jn << ", ";}
    jn << trajectory.joint_names[i];
  }
  RCLCPP_INFO(this->get_logger(), "Joint Names: [%s]", jn.str().c_str());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    std::ostringstream pos, vel, acc;
    for (size_t k = 0; k < pt.positions.size(); ++k) {
      if (k) {pos << ", ";}
      pos << pt.positions[k];
    }
    for (size_t k = 0; k < pt.velocities.size(); ++k) {
      if (k) {vel << ", ";}
      vel << pt.velocities[k];
    }
    for (size_t k = 0; k < pt.accelerations.size(); ++k) {
      if (k) {acc << ", ";}
      acc << pt.accelerations[k];
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Point %zu: Positions: [%s], Velocities: [%s], "
      "Accelerations: [%s], TimeFromStart: %d s %u ns",
      i, pos.str().c_str(), vel.str().c_str(), acc.str().c_str(),
      pt.time_from_start.sec, pt.time_from_start.nanosec);

    if (i != 0) {
      rclcpp::Duration duration_cur(trajectory.points[i].time_from_start);
      rclcpp::Duration duration_pre(trajectory.points[i - 1].time_from_start);
      RCLCPP_INFO(
        this->get_logger(), "Time interval: %f",
        (duration_cur - duration_pre).seconds());
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
