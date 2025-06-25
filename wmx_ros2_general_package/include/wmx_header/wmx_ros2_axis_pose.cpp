#include "wmx_header/wmx_ros2_axis_pose.hpp"

WmxRos2AxisPoseServer::WmxRos2AxisPoseServer(const rclcpp::NodeOptions & options) : Node("wmx_ros2_axis_pose_server", options){
  using namespace std::placeholders;

  this->wmx_ros2_axis_pose_ = rclcpp_action::create_server<AxisPose>(
    this, "/wmx/axis/position",
    std::bind(&WmxRos2AxisPoseServer::handle_goal, this, _1, _2),
    std::bind(&WmxRos2AxisPoseServer::handle_cancel, this, _1),
    std::bind(&WmxRos2AxisPoseServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "wmx_ros2_axis_pose_server is ready");
}

rclcpp_action::GoalResponse WmxRos2AxisPoseServer::handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const AxisPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received axis target pose");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WmxRos2AxisPoseServer::handle_cancel(
                const std::shared_ptr<GoalHandleAxisPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancel moving axis pose");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WmxRos2AxisPoseServer::handle_accepted(const std::shared_ptr<GoalHandleAxisPose> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&WmxRos2AxisPoseServer::execute, this, _1), goal_handle}.detach();
}

void WmxRos2AxisPoseServer::execute(const std::shared_ptr<GoalHandleAxisPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing moving axis pose");
  rclcpp::Rate loop_rate(rate_);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<AxisPose::Feedback>();
  auto result = std::make_shared<AxisPose::Result>();

  wmx3LibCm_.GetStatus(&cmStatus_);
  feedback->current = cmStatus_.axesStatus[goal->index].actualVelocity;

  while (std::abs(goal->target - feedback->current) <= goal->threshold) {
    wmx3LibCm_.GetStatus(&cmStatus_);
    feedback->current = cmStatus_.axesStatus[goal->index].actualVelocity;

    if (goal_handle->is_canceling()) {
      snprintf(buffer_, sizeof(buffer_), "Moving axis pose is canceled");
      RCLCPP_INFO(this->get_logger(), "%s", buffer_);
      result->success = false;
      result->final = feedback->current;
      result->message = std::string(buffer_);
      goal_handle->canceled(result);
      return;
    }

    position_.axis = goal->index;
    position_.target = goal->target;
    position_.profile.velocity = goal->velocity;
    position_.profile.type = wmx3Api::ProfileType::T::Trapezoidal;
    position_.profile.acc = goal->acc;
    position_.profile.dec = goal->dec;

    err_ = wmx3LibCm_.motion->StartMov(&position_);

    if (err_ != wmx3Api::ErrorCode::None) {
      snprintf(buffer_, sizeof(buffer_), "Failed to move pose axis %d", goal->index);
      RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
      result->success = false;
      result->final = feedback->current;
      result->message = std::string(buffer_);
      goal_handle->abort(result);
      return;
    }

    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    snprintf(buffer_, sizeof(buffer_), "Moving axis pose succeeded");
    RCLCPP_INFO(this->get_logger(), "%s", buffer_);
    result->success = true;
    result->final = feedback->current;
    result->message = std::string(buffer_);
    goal_handle->succeed(result);
  }
}
