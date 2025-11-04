#include <memory>
#include <thread>
#include <sstream>
#include <chrono>

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "AdvancedMotionApi.h"
#include "IOApi.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_srvs/srv/set_bool.hpp"

#define MAX_TRAJ_POINTS 1000

using namespace wmx3Api;

class FollowJointTrajectoryServer : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setGripperService_;

  int err_;
  char errString_[256];

  FollowJointTrajectoryServer() : Node("cr3a_group_controller")
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

    wmx3LibCm_ = CoreMotion(&wmx3Lib_);
    wmx3LibAm_ = AdvancedMotion(&wmx3Lib_);
    Wmx3Lib_Io_ = Io(&wmx3Lib_); 

    wmx3LibAm_.advMotion->CreateSplineBuffer(0, MAX_TRAJ_POINTS);

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/iifes_arm_controller/follow_joint_trajectory",
      std::bind(&FollowJointTrajectoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&FollowJointTrajectoryServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&FollowJointTrajectoryServer::handle_accepted, this, std::placeholders::_1));

    setGripperService_ = this->create_service<std_srvs::srv::SetBool>("/wmx/set_gripper",
                                std::bind(&FollowJointTrajectoryServer::setGripper, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "FollowJointTrajectory Action Server is ready...");
  }

  ~FollowJointTrajectoryServer()
  {
    wmx3LibAm_.advMotion->FreeSplineBuffer(0);
  }

private:
  WMX3Api wmx3Lib_;
  CoreMotion wmx3LibCm_;
  AdvancedMotion wmx3LibAm_;
  AdvMotion::PointTimeSplineCommand spl;
  AdvMotion::SplinePoint pt_spl[MAX_TRAJ_POINTS];
  unsigned int time_spl[MAX_TRAJ_POINTS];
  AxisSelection axisSel;
  Io Wmx3Lib_Io_;

  void setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      if (request->data){
        err_ = Wmx3Lib_Io_.SetOutBit(0, 0, 1);
        if (err_ != ErrorCode::None) {
          wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
          RCLCPP_ERROR(this->get_logger(), "Gripper fails to Close");
          response->success = false;
        }
        else{
          RCLCPP_INFO(this->get_logger(), "Gripper success to Close");
          response->success = true;
        }
      } 
      else{
        err_ = Wmx3Lib_Io_.SetOutBit(0, 0, 0);
        if (err_ != ErrorCode::None) {
          wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
          RCLCPP_ERROR(this->get_logger(), "Gripper fails to Open");
          response->success = false;
        }
        else{
          RCLCPP_INFO(this->get_logger(), "Gripper success to Open");
          response->success = true;
        }
      }
  }

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
    RCLCPP_INFO(this->get_logger(), "Received a new trajectory goal!");
    const auto goal = goal_handle->get_goal();
    const auto &trajectory = goal->trajectory;
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    int num_points = trajectory.points.size();
    double timeMilliseconds;

    if(num_points > MAX_TRAJ_POINTS) {
      RCLCPP_WARN(this->get_logger(), "Too many trajectory point size! current points:%d / max traj points:%d \nAborting current goal.", num_points, MAX_TRAJ_POINTS);
      goal_handle->abort(result);
      return;
    }

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
      if(i!=0) {
        rclcpp::Duration duration_cur(trajectory.points[i].time_from_start);
        rclcpp::Duration duration_pre(trajectory.points[i-1].time_from_start);

        RCLCPP_INFO(
          this->get_logger(),
          "Time interval: %f",
          (duration_cur-duration_pre).seconds());        
      }
    }
    
    const size_t N = 6;      // expected DOF;

    // generate spline commands from trajectory.points
    axisSel.axisCount = N;
    spl.dimensionCount = N;
    for (size_t j = 0; j < N; ++j) {
        axisSel.axis[j] = j;
        spl.axis[j] = j;
    }

    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &pt = trajectory.points[i];
      timeMilliseconds = rclcpp::Duration(pt.time_from_start).seconds() * 1000;
      time_spl[i] = timeMilliseconds;

      for (size_t j = 0; j < N; ++j) {
        pt_spl[i].pos[j] = pt.positions.at(j);
      }
    }

    // if last time interval is less than 1ms, ignore the last point.
    double last = trajectory.points.size() - 1;
    if(rclcpp::Duration(trajectory.points[last].time_from_start).seconds() - rclcpp::Duration(trajectory.points[last-1].time_from_start).seconds() < 1e-3) {
      num_points -= 1;
    }

    RCLCPP_INFO(this->get_logger(), "Command Start!!!");
    err_ = wmx3LibAm_.advMotion->StartCSplinePos(0, &spl, num_points, pt_spl, time_spl);
    if(err_ != 0) {
        wmx3LibAm_.ErrorToString(err_, errString_, 256);
        RCLCPP_INFO(this->get_logger(), "StartCSplinePos Error: %s", errString_);
    }

    // TODO: instead of using blociking Wait function, monitor flags so that "goal_handle->is_canceling()" can be checked.
    err_ = wmx3LibCm_.motion->Wait(&axisSel);
    if(err_ != 0) {
        wmx3LibCm_.ErrorToString(err_, errString_, 256);
        RCLCPP_INFO(this->get_logger(), "Wait Error: %s", errString_);
    }

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