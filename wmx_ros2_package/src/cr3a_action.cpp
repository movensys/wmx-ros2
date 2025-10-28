#include <memory>
#include <thread>
#include <sstream>
#include <chrono>

#include "WMX3Api.h"
#include "CoreMotionApi.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace wmx3Api;

class FollowJointTrajectoryServer : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  int err_;
  char errString_[256];

  FollowJointTrajectoryServer() : Node("cr3a_group_controller"), wmx3LibCm_(&wmx3Lib_)
  {
    err_ = wmx3Lib_.CreateDevice("/opt/lmx/", DeviceType::DeviceTypeNormal, INFINITE);

    wmx3Lib_.SetDeviceName("cr3aLMX");
    if (err_ != ErrorCode::None) {
        wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
        RCLCPP_ERROR(this->get_logger(), "Failed to create device. Error=%d (%s)", err_, errString_);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Created a device");
    }

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/cr3_group_controller/follow_joint_trajectory",
      std::bind(&FollowJointTrajectoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FollowJointTrajectoryServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FollowJointTrajectoryServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "FollowJointTrajectory Action Server is ready...");
  }

private:
  WMX3Api wmx3Lib_;
  CoreMotion wmx3LibCm_;
  wmx3Api::Motion::PosCommand posCommands_[6];

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleFJT>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    std::thread{std::bind(&FollowJointTrajectoryServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    double vel_, acc_, velPre_;
    RCLCPP_INFO(this->get_logger(), "Received a new trajectory goal!");
    const auto goal = goal_handle->get_goal();
    const auto &trajectory = goal->trajectory;

    // Log joint names
    std::ostringstream jn;
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      if (i) jn << ", ";
      jn << trajectory.joint_names[i];
    }
    RCLCPP_INFO(this->get_logger(), "Joint Names: [%s]", jn.str().c_str());

    // Log points
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];
      std::ostringstream pos, vel, acc;
      for (size_t k = 0; k < pt.positions.size(); ++k) { if (k) pos << ", "; pos << pt.positions[k]; }
      for (size_t k = 0; k < pt.velocities.size(); ++k) { if (k) vel << ", "; vel << pt.velocities[k]; }
      for (size_t k = 0; k < pt.accelerations.size(); ++k) { if (k) acc << ", "; acc << pt.accelerations[k]; }
      RCLCPP_INFO(
        this->get_logger(),
        "Point %zu: Positions: [%s], Velocities: [%s], Accelerations: [%s], TimeFromStart: %d s %u ns",
        i, pos.str().c_str(), vel.str().c_str(), acc.str().c_str(),
        pt.time_from_start.sec, pt.time_from_start.nanosec);
    }

    RCLCPP_INFO(this->get_logger(), "Command Start!!!");

    rclcpp::Rate rate(10.0); // 0.1 s
    const size_t N = 6;      // expected DOF

    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];

      // positions
      for (size_t j = 0; j < N; ++j) {
        posCommands_[j].axis = j;
        posCommands_[j].target = pt.positions.at(j);
        posCommands_[j].profile.type = ProfileType::Trapezoidal;

        vel_ = std::abs(pt.velocities.at(j));
        acc_ = std::abs(pt.accelerations.at(j));
        velPre_ = (i == 0.0) ? 0.0 : std::abs(trajectory.points[i - 1].velocities.at(j));

        vel_ = vel_ < 1e-6 ? 1e-6 : vel_;
        acc_ = acc_ < 1e-6 ? 1e-6 : acc_;
        velPre_ = velPre_ < 1e-6 ? 1e-6 : velPre_;
        
        posCommands_[j].profile.endVelocity = vel_ == 1e-6? 0.0 : vel_;
        posCommands_[j].profile.velocity = vel_ < velPre_ ? (velPre_+vel_)/2.0 : vel_;
        posCommands_[j].profile.acc = acc_;
        posCommands_[j].profile.dec = acc_;
      }
      
      err_ = wmx3LibCm_.motion->StartPos(6, posCommands_);
      if(err_ != 0) {
          wmx3LibCm_.ErrorToString(err_, errString_, 256);
          RCLCPP_INFO(this->get_logger(), "%s", errString_);
          for (int i = 0; i < 6; ++i) {
              RCLCPP_INFO(this->get_logger(), "pos[%lf] vel[%lf] acc[%lf]", posCommands_[i].target, posCommands_[i].profile.velocity, posCommands_[i].profile.acc);
          }
      }

      rate.sleep();
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = 0;
    goal_handle->succeed(result);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowJointTrajectoryServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}