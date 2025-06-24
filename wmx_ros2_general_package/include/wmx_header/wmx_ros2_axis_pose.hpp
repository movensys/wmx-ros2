#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "wmx_ros2_message/action/axis_pose.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"

using namespace wmx3Api;
using namespace std;

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
    int err_;
    char errString_[256];  
    char buffer_[512];
    int rate_ = 10;

    WMX3Api wmx3Lib_;    
    CoreMotion wmx3LibCm_;    
    CoreMotionStatus cmStatus_;

    wmx3Api::Position::PoseCommand position_ = wmx3Api::Position::PoseCommand();

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
      RCLCPP_INFO(this->get_logger(), "Executing moving axis pose");
      rclcpp::Rate loop_rate(rate_);

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<AxisPose::Feedback>();
      auto result = std::make_shared<AxisPose::Result>();

      wmx3LibCm_.GetStatus(&cmStatus_);
      feedback->current = cmStatus_.axesStatus[goal->index].actualVelocity;

      while(std::abs(goal->target - feedback->current) <= goal->threshold){
        wmx3LibCm_.GetStatus(&cmStatus_);
        feedback->current = cmStatus_.axesStatus[goal->index].actualVelocity;

        if (goal_handle->is_canceling()) {
          snprintf(buffer_, sizeof(buffer_), "Moving axis pose is canceled");
          RCLCPP_INFO(this->get_logger(), "%s", buffer_);
          result->succes = false;
          result->final = feedback->current;
          result->message = std::string(buffer_);
          goal_handle->canceled(result);
          return;
        }

        position_.axis = goal->index;
        position_.profile.pose = goal->pose;
        position_.profile.velocity = goal->velocity;
        position_.profile.type = ProfileType::T::Trapezoidal; //goal->profile
        position_.profile.acc = goal->acc;
        position_.profile.dec = goal->dec;

        err_ = wmx3LibCm_.position->move(&position_);

        if (err_ != ErrorCode::None) {
          snprintf(buffer_, sizeof(buffer_), "Failed to moving pose axis %d", goal->index);
          RCLCPP_ERROR(this->get_logger(), "%s", buffer_);
          response->success = false;
          result->final = feedback->current;
          response->message = std::string(buffer_);
          goal_handle->abort(result)
        }

        
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
      }

      if (rclcpp::ok()) {
        snprintf(buffer_, sizeof(buffer_), "Moving axis pose is succesed");
        RCLCPP_INFO(this->get_logger(), "%s", buffer_);
        result->succes = true;
        result->final = feedback->current;
        result->message = std::string(buffer_);
        goal_handle->succeed(result);
      }
    }
}; 