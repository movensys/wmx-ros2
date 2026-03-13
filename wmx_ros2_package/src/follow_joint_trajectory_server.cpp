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
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#define MAX_TRAJ_POINTS 1000

using namespace wmx3Api;

class FollowJointTrajectoryServer : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  FollowJointTrajectoryServer();
  ~FollowJointTrajectoryServer();

  int jointNumber_;
  std::string jointTrajectoryAction_;
  std::string wmxGripperTopic_;

  int err_;
  char errString_[256];

private:
  bool initialized_ = false;

  WMX3Api wmx3Lib_;
  CoreMotion wmx3LibCm_;
  AdvancedMotion wmx3LibAm_;
  AdvMotion::PointTimeSplineCommand spl;
  AdvMotion::SplinePoint pt_spl[MAX_TRAJ_POINTS];
  double time_spl[MAX_TRAJ_POINTS];
  AxisSelection axisSel;
  Io Wmx3Lib_Io_;

  rclcpp::CallbackGroup::SharedPtr sdk_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setGripperService_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  // Action server callback declarations
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<GoalHandleFJT> goal_handle);

  void handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle);

  void execute(std::shared_ptr<GoalHandleFJT> goal_handle);

  // Service callback declaration
  void setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void setRosParameter();
  void onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg);
};

FollowJointTrajectoryServer::FollowJointTrajectoryServer()
  : Node("follow_joint_trajectory_server"){

  setRosParameter();

  auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
  engineReadySub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wmx/engine/ready", ready_qos,
    std::bind(&FollowJointTrajectoryServer::onEngineReady, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "follow_joint_trajectory_server waiting for engine...");
}

FollowJointTrajectoryServer::~FollowJointTrajectoryServer(){
  RCLCPP_INFO(this->get_logger(), "Stop follow_joint_trajectory_server");

  if (initialized_) {
    wmx3LibAm_.advMotion->FreeSplineBuffer(0);

    err_ = wmx3Lib_.CloseDevice();
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Failed to close device");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device closed");
    }
  }

  RCLCPP_INFO(this->get_logger(), "follow_joint_trajectory_server is stopped");
}

void FollowJointTrajectoryServer::onEngineReady(std_msgs::msg::Bool::ConstSharedPtr msg) {
  if (!msg->data || initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Engine ready — initializing AdvancedMotion...");

  unsigned int timeout = 10000;
  err_ = wmx3Lib_.CreateDevice(WMX3_SDK_PATH, DeviceType::DeviceTypeNormal, timeout);

  if (err_ != ErrorCode::None) {
    wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
    if (err_ == ErrorCode::StartProcessLockError) {
      RCLCPP_WARN(this->get_logger(), "Failed to attach to device (lock busy, retrying).");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to attach to device. Error=%d (%s)", err_, errString_);
    }
    return;
  }

  wmx3Lib_.SetDeviceName("follow_joint_trajectory_server");
  RCLCPP_INFO(this->get_logger(), "Attached to WMX3 device");

  wmx3LibCm_ = CoreMotion(&wmx3Lib_);
  wmx3LibAm_ = AdvancedMotion(&wmx3Lib_);
  Wmx3Lib_Io_ = Io(&wmx3Lib_);
  wmx3LibAm_.advMotion->CreateSplineBuffer(0, MAX_TRAJ_POINTS);

  // MutuallyExclusive callback group serializes SDK access between
  // trajectory execution and gripper service callbacks.
  sdk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rcl_action_server_options_t action_options = rcl_action_server_get_default_options();
  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(this,
                    jointTrajectoryAction_,
                    std::bind(&FollowJointTrajectoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&FollowJointTrajectoryServer::handle_cancel, this, std::placeholders::_1),
                    std::bind(&FollowJointTrajectoryServer::handle_accepted, this, std::placeholders::_1),
                    action_options,
                    sdk_group_);

  setGripperService_ = this->create_service<std_srvs::srv::SetBool>(wmxGripperTopic_,
                    std::bind(&FollowJointTrajectoryServer::setGripper, this,
                    std::placeholders::_1, std::placeholders::_2),
                    rclcpp::ServicesQoS().get_rmw_qos_profile(),
                    sdk_group_);

  initialized_ = true;
  engineReadySub_.reset();

  RCLCPP_INFO(this->get_logger(), "follow_joint_trajectory_server is ready");
}

void FollowJointTrajectoryServer::setRosParameter(){
  this->declare_parameter<int>("joint_number", 0);
  this->declare_parameter<std::string>("joint_trajectory_action", "/joint_trajectory_action/no_param");
  this->declare_parameter<std::string>("wmx_gripper_topic", "/wmx_gripper_topic/no_param");

  this->get_parameter("joint_number", jointNumber_);
  this->get_parameter("joint_trajectory_action", jointTrajectoryAction_);
  this->get_parameter("wmx_gripper_topic", wmxGripperTopic_);

  RCLCPP_INFO(this->get_logger(), "===== ROS2 Parameters =====");
  RCLCPP_INFO(this->get_logger(), "joint_number: %d", jointNumber_);
  RCLCPP_INFO(this->get_logger(), "joint_trajectory_action: %s", jointTrajectoryAction_.c_str());
  RCLCPP_INFO(this->get_logger(), "wmx_gripper_topic: %s", wmxGripperTopic_.c_str());
  RCLCPP_INFO(this->get_logger(), "===========================");
}

rclcpp_action::GoalResponse FollowJointTrajectoryServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                                      std::shared_ptr<const FollowJointTrajectory::Goal> goal){
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowJointTrajectoryServer::handle_cancel(std::shared_ptr<GoalHandleFJT> goal_handle){
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowJointTrajectoryServer::handle_accepted(std::shared_ptr<GoalHandleFJT> goal_handle){
  std::thread{std::bind(&FollowJointTrajectoryServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void FollowJointTrajectoryServer::execute(std::shared_ptr<GoalHandleFJT> goal_handle){
  const auto goal = goal_handle->get_goal();
  const auto &trajectory = goal->trajectory;

  RCLCPP_INFO(this->get_logger(), "Received a new trajectory goal! Point number: [%zu]", trajectory.points.size());

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  int num_points = trajectory.points.size();
  double timeMilliseconds;

  if(num_points > MAX_TRAJ_POINTS) {
    RCLCPP_WARN(this->get_logger(),
                "Too many trajectory point size! current points:%d / max traj points:%d \nAborting current goal.",
                num_points, MAX_TRAJ_POINTS);
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
  RCLCPP_INFO(this->get_logger(), "Point number: [%zu]", trajectory.points.size());

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

    if(i != 0) {
      rclcpp::Duration duration_cur(trajectory.points[i].time_from_start);
      rclcpp::Duration duration_pre(trajectory.points[i-1].time_from_start);
      RCLCPP_INFO(this->get_logger(), "Time interval: %f", (duration_cur - duration_pre).seconds());
    }
  }

  // Generate spline commands from trajectory.points
  axisSel.axisCount = jointNumber_;
  spl.dimensionCount = jointNumber_;
  for (int j = 0; j < jointNumber_; ++j) {
    axisSel.axis[j] = j;
    spl.axis[j] = j;
  }

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto &pt = trajectory.points[i];
    timeMilliseconds = rclcpp::Duration(pt.time_from_start).seconds() * 1000;
    time_spl[i] = timeMilliseconds;

    for (int j = 0; j < jointNumber_; ++j) {
      pt_spl[i].pos[j] = pt.positions.at(j);
    }
  }

  // If first time interval is not zero, make it zero
  if (time_spl[0] != 0.0) {
    time_spl[0] = 0.0;
  }

  // if last time interval is less than 1ms, ignore the last point.
  double last = trajectory.points.size() - 1;
  if(rclcpp::Duration(trajectory.points[last].time_from_start).seconds() - rclcpp::Duration(trajectory.points[last-1].time_from_start).seconds() < 1e-3) {
    num_points -= 1;
  }

  if(num_points==0){
    RCLCPP_INFO(this->get_logger(), "Point count is zero. It is already in the targeted position");
  }

  else{
    RCLCPP_INFO(this->get_logger(), "Command Start!!!");
    err_ = wmx3LibAm_.advMotion->StartCSplinePos(0, &spl, num_points, pt_spl, time_spl);
    if(err_ != 0) {
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
      for (int j = 0; j < jointNumber_; ++j) {
        if (!cmStatus.axesStatus[j].inPos) { all_done = false; break; }
      }
      if (all_done) break;

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  result->error_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
}

void FollowJointTrajectoryServer::setGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                              std::shared_ptr<std_srvs::srv::SetBool::Response> response){
  if (request->data) {
    err_ = Wmx3Lib_Io_.SetOutBit(0, 0, 1);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Gripper fails to Close: %s", errString_);
      response->success = false;
      response->message = "Failed to close gripper";
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Gripper success to Close");
      response->success = true;
      response->message = "Gripper closed successfully";
    }
  }
  else {
    err_ = Wmx3Lib_Io_.SetOutBit(0, 0, 0);
    if (err_ != ErrorCode::None) {
      wmx3Lib_.ErrorToString(err_, errString_, sizeof(errString_));
      RCLCPP_ERROR(this->get_logger(), "Gripper fails to Open: %s", errString_);
      response->success = false;
      response->message = "Failed to open gripper";
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Gripper success to Open");
      response->success = true;
      response->message = "Gripper opened successfully";
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowJointTrajectoryServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
