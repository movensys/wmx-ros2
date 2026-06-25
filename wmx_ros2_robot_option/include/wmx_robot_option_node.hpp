// Copyright 2026 Movensys Corporation.
// Licensed under the MIT License. See LICENSE.txt for details.

#ifndef WMX_ROBOT_OPTION_NODE_HPP_
#define WMX_ROBOT_OPTION_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wmx_ros2_message/msg/robot_arm_status.hpp"
#include "wmx_ros2_message/srv/robot_jog_pose.hpp"
#include "wmx_ros2_message/srv/robot_move_pose.hpp"
#include "wmx_ros2_message/srv/robot_move_angle.hpp"
#include "wmx_ros2_message/srv/robot_check_pose.hpp"
#include "wmx_ros2_message/srv/robot_check_angle.hpp"
#include "wmx_ros2_message/srv/set_robot_scalar.hpp"

#include "WMX3Api.h"
#include "CoreMotionApi.h"
#include "RobotMotionApi.h"
#include "IOApi.h"

#include "robot_pose_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// Cartesian motion types accepted by set_pose service (mirrors MOTION_TYPE in
// wmx_arm_server/constants.h).
enum MotionType
{
  MOTION_LINEAR = 0,
  MOTION_CIRCULAR = 1,
  MOTION_JOINT = 2,
};

// wmx_robot_option_node reflects wmx_arm_server (wmx-server-development): it owns
// the WMX3 RobotMotion option and exposes the robot arm control surface over ROS 2
// services and a periodic status topic, in the style of wmx_ros2_package.
class WmxRobotOptionNode : public rclcpp::Node
{
public:
  WmxRobotOptionNode();
  ~WmxRobotOptionNode();

private:
  // --- lifecycle / engine attach ---
  void declareParameters();
  void onEngineReady(const std_msgs::msg::Bool::SharedPtr msg);
  int uploadParams();       // load robot.xml and push it to the engine
  int configureDevice();    // apply WMX3 system/axis parameters
  void statusStep();        // periodic status publication + safety check

  // --- option helpers (port of WMX_Arm) ---
  bool isReady();
  bool getServo();
  bool areArmAxesBusy();    // any arm axis still moving (motion from ANY engine client)
  bool isAxesNormalState();
  bool isRevoluteJoint(int joint_index);
  void stopMotionIfAlarm();
  std::string motionStateString(wmx3Api::kinematics::constants::MotionState state);
  std::tuple<std::string, int> motionErrorInfo(const wmx3Api::kinematics::MotionErrorInfo & info);
  void logErr(int err_code, const std::string & msg);

  int setMotion(
    const wmx3Api::coordinate::CartesianPose & pose, bool is_tool_frame,
    int motion_type, bool start_motion = true);
  int cartesianMove(
    const wmx3Api::coordinate::CartesianPose & pose, bool is_tool_frame, int motion_type);
  int jointMove(const std::vector<double> & angles);
  int jointRelMove(const std::vector<double> & angles);
  bool checkTargetPose(const wmx3Api::coordinate::CartesianPose & pose, double tolerance = 0.0001);
  bool checkTargetAngle(const std::vector<double> & angles, double tolerance = 0.0001);
  void enableCollisionForMotion();  // enable collision detection before a move (IO-gated)

  // --- service callbacks (mirror the wmx_arm_server RPC surface) ---
  void setServo(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setSpeed(
    const std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Response> response);
  void clearErrors(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stopMotion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void exportParams(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void jogPose(
    const std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Response> response);
  void jogPoseAbsolute(
    const std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotJogPose::Response> response);
  void setPosePtp(
    const std::shared_ptr<wmx_ros2_message::srv::RobotMovePose::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotMovePose::Response> response);

  void jogAngle(
    const std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Response> response);
  void jogAngleAbsolute(
    const std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Response> response);
  void setAnglePtp(
    const std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotMoveAngle::Response> response);

  void checkPose(
    const std::shared_ptr<wmx_ros2_message::srv::RobotCheckPose::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotCheckPose::Response> response);
  void checkAngle(
    const std::shared_ptr<wmx_ros2_message::srv::RobotCheckAngle::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::RobotCheckAngle::Response> response);

  void setCollisionEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setFittingParam(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void setCollisionSensitivity(
    const std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Request> request,
    std::shared_ptr<wmx_ros2_message::srv::SetRobotScalar::Response> response);

  // --- state ---
  std::atomic<bool> initialized_{false};
  bool params_uploaded_ = false;
  bool device_configured_ = false;
  int err_;
  char errString_[256];
  char buffer_[512];

  // ROS parameters
  std::string robot_xml_path_;
  std::string wmx_param_file_path_;
  std::string status_frame_;
  double collision_sensitivity_ = 6.0;
  int status_rate_ = 50;

  // WMX3 robot option handles
  wmx3Api::WMX3Api wmx3Lib_;
  std::unique_ptr<wmx3Api::RobotMotion> robot_;
  std::unique_ptr<wmx3Api::Io> io_;
  wmx3Api::CoreMotionStatus cmStatus_;
  wmx3Api::RobotMotionParam robotParam_;
  wmx3Api::RobotStatus robotStatus_;

  wmx_ros2_message::msg::RobotArmStatus statusMsg_;

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr init_cb_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engineReadySub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robotReadyPub_;
  rclcpp::Publisher<wmx_ros2_message::msg::RobotArmStatus>::SharedPtr statusPub_;
  rclcpp::TimerBase::SharedPtr statusTimer_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setServoService_;
  rclcpp::Service<wmx_ros2_message::srv::SetRobotScalar>::SharedPtr setSpeedService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clearErrorsService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopMotionService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exportParamsService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotJogPose>::SharedPtr jogPoseService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotJogPose>::SharedPtr jogPoseAbsoluteService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotMovePose>::SharedPtr setPosePtpService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotMoveAngle>::SharedPtr jogAngleService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotMoveAngle>::SharedPtr jogAngleAbsoluteService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotMoveAngle>::SharedPtr setAnglePtpService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotCheckPose>::SharedPtr checkPoseService_;
  rclcpp::Service<wmx_ros2_message::srv::RobotCheckAngle>::SharedPtr checkAngleService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setCollisionEnableService_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setFittingParamService_;
  rclcpp::Service<wmx_ros2_message::srv::SetRobotScalar>::SharedPtr setCollisionSensitivityService_;
};

#endif  // WMX_ROBOT_OPTION_NODE_HPP_
