#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "wmx_ros2_message/action/axis_pose.hpp"

class WmxRos2AxisPoseServer : public rclcpp::Node
{
  public:
  using AxisPose = wmx_ros2_message::action::AxisPose;  
  using GoalHandleAxisPose = rclcpp_action::ServerGoalHandle<wmx_ros2_message::action::AxisPose>;

  explicit AxisPoseServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("wmx_ros2_axis_pose_server", options)
  {
    using namespace std::placeholders;

    this->wmx_ros2_axis_pose_ = rclcpp_action::create_server<AxisPose>(
      this, "/wmx/axis/position",
      std::bind(&AxisPoseServer::handle_goal, this, _1, _2),
      std::bind(&AxisPoseServer::handle_cancel, this, _1),
      std::bind(&AxisPoseServer::handle_accepted, this, _1));
      
      RCLCPP_INFO(this->get_logger(), "wmx_ros2_axis_pose_server is ready");
    }

  private:
    rclcpp_action::Server<AxisPose>::SharedPtr wmx_ros2_axis_pose_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AxisPose::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received axis target pose");
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAxisPose> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Cancel moving axis pose");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAxisPose> goal_handle)
    {
      using namespace std::placeholders;
      std::thread{std::bind(&AxisPoseServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleAxisPose> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing moving axis");
      rclcpp::Rate loop_rate(10);

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<AxisPose::Feedback>();
      auto result = std::make_shared<AxisPose::Result>();

      
    }
}; 