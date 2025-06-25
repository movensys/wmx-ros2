#ifndef WMX_ROS2_AXIS_POSE_HPP
#define WMX_ROS2_AXIS_POSE_HPP

#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "wmx_ros2_message/action/axis_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

class WmxRos2AxisPoseServer : public rclcpp::Node
{
public:
  using AxisPose = wmx_ros2_message::action::AxisPose;
  using GoalHandleAxisPose = rclcpp_action::ServerGoalHandle<AxisPose>;

  explicit WmxRos2AxisPoseServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  int err_;
  char errString_[256];
  char buffer_[512];
  int rate_ = 10;

  wmx3Api::WMX3Api wmx3Lib_;
  wmx3Api::CoreMotion wmx3LibCm_;
  wmx3Api::CoreMotionStatus cmStatus_;

  wmx3Api::Motion::PosCommand position_;

  rclcpp_action::Server<AxisPose>::SharedPtr wmx_ros2_axis_pose_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AxisPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAxisPose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleAxisPose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleAxisPose> goal_handle);
};

#endif  // WMX_ROS2_AXIS_POSE_HPP